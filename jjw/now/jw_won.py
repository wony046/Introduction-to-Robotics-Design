import serial
import time
import math
import json
import queue
import threading
import camera_tracker

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 포트 & 라이다 설정
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

LIDAR_OFFSET    = 10     # mm: 라이다 측정값 보정
LIDAR_MIN_VALID = 100   # mm: 이 미만 무시 (노이즈)
DETECTION_RANGE = 1500  # mm: 라이다 최대 신뢰 거리

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 로봇 & 속도 파라미터
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

ROBOT_HALF_WIDTH = 110   # mm: 라이다 중심 ~ 좌우 끝

FORWARD_SPEED    = 0.3
MIN_SPEED        = 0.12
MAX_W            = 1.8
W_MIN_DANGER     = 0.5   # rad/s: 위험 시 최소 회전
W_SMOOTH         = 0.7

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 계층형 바운딩 박스 정의 (6개 레이어)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 각 레이어: 거리 범위, horiz 임계, w_gain, 기본 가중치, 동적 가중치 여부, v 영향 여부

LAYERS = [
    # L1: 가장 가까움, 동적 가중치, weight_cap=7.5, v_max=0.22
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':140,
     'w_gain':2.8, 'weight_base':0.8, 'weight_cap':7.5, 'weight_dynamic':True,
     'v_max':0.2, 'affects_v':True},
    # L2: 가까움, 동적 가중치, weight_cap=4.5, v_max=0.38
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':120,
     'w_gain':2.5, 'weight_base':0.6, 'weight_cap':5.0, 'weight_dynamic':True,
     'v_max':0.25, 'affects_v':True},
    # L3: 중간, 동적 가중치, weight_cap=2.5, v_max=FORWARD_SPEED
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':120,
     'w_gain':2.0, 'weight_base':0.4, 'weight_cap':4.5, 'weight_dynamic':True, 'affects_v':True},
    # L4: 중간-원거리 (weight: 진입 0.2 → 끝 0.1)
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':110,
     'w_gain':1.0, 'weight_base':0.1, 'weight_start':0.2, 'weight_dynamic':False, 'affects_v':True},
    # L5: 원거리 (weight: 진입 0.1 → 끝 0.05)
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':110,
     'w_gain':0.4, 'weight_base':0.05,'weight_start':0.1, 'weight_dynamic':False, 'affects_v':False},
    # L6: 최원거리 (weight: 진입 0.05 → 끝 0.02)
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':110,
     'w_gain':0.3, 'weight_base':0.02,'weight_start':0.05,'weight_dynamic':False, 'affects_v':False},
]

LAYER_PERCENTILE = 5    # %: 하위 N% dist 평균으로 레이어 대표점 계산

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP zone (계층형과 완전 별도)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP rectangle: 전방 100~175mm 사이, horiz < 105mm (210mm 폭)

STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 175
STOP_HORIZ_TH = 105

PIVOT_CLEAR_RADIUS = STOP_FWD_MAX   # mm: 피버턴 클리어런스 검사 반경 (= STOP_FWD_MAX)
REAR_BLIND_HALF    = 23.0           # deg: 후방 카메라 구조물 차폐 반각 (총 46° 사각)

# STOP 탈출: 360° 전체 스캔, ROBOT_HALF_WIDTH*2 + 양쪽 20mm 마진
STOP_ESCAPE_MIN_GAP   = ROBOT_HALF_WIDTH * 2 + 40   # 260mm
STOP_MAX_CYCLES       = 30                          # 연속 STOP 사이클 상한 (초과 시 강제 탈출)
STOP_PIVOT_MAX_W      = 0.9   # rad/s: 피봇 최대 회전 속도 (목표에서 멀 때)
STOP_PIVOT_MIN_W      = 0.7   # rad/s: 피봇 최소 회전 속도 (목표 근처)
STOP_PIVOT_SLOW_DEG   = 15    # deg: 이 이내부터 선형 감속 시작

# FGM (Follow the Gap Method) — STOP escape 전용
FGM_MIN_ANG_DEG      = 3     # deg: 이 이상 각도 공백이면 갭으로 인식
FGM_MIN_DEPTH_MM     = 250   # mm: 갭 너머 최소 깊이 (얕은 함몰부 제외)
FGM_MAX_RANGE_MM     = 500   # mm: FGM 갭 탐색 최대 거리 (이 이상 포인트 무시)
FGM_RATIO_THRES      = 1.2   # 인접 포인트 거리 비율 이상이면 갭 경계로 인식 (벽 끝 완만 전환)

# 전방 갭 탐색 (기본 주행 방향 결정용)
FRONT_GAP_MIN_DEPTH  = 300   # mm: 전방 갭 최소 깊이 (이 미만 탈락)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 방향 점수제 (gap + layer 통합)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCORE_ALPHA       = 5.0    # gap_width 계수
SCORE_BETA        = 8     # 정면 레이어 push 계수 //수정
SCORE_SIDE        = 2500.0  # 측방 레이어 방향 가중치
HEADING_WEIGHT_MM = 5.0    # 헤딩 1° = 여유 5mm
DEPTH_JUMP_THRES  = 120    # mm: 이상이면 다른 물체로 인식

# 방향 히스테리시스: 이 점수 차 미만이면 직전 방향 유지 (정면 장애물 시 oscillation 방지)
DIRECTION_HYSTERESIS = 300.0

# ── 목표 방향 추종 (카메라 색지) ──────────────────────────────
GAP_TARGET_WEIGHT = 1.0           # 갭 선택: 목표 방향 추종 강도 (주 항)
GAP_SMOOTH_WEIGHT = 0.3           # 갭 선택: 직전 방향 유지 강도 (떨림 억제)
KP_GOAL            = MAX_W / 45.0  # 비례 조향 게인 (45° → MAX_W)
TARGET_ALIGN_ANGLE = 60.0         # deg: 이 각도 이상이면 v=MIN_SPEED (거의 제자리 회전)
TARGET_CLEAR_CONE  = 18           # deg: 목표 방향 ± 이 각도 범위를 막힘 검사 대상으로
TARGET_BLOCK_DIST = 600           # mm: 이 거리 이내 장애물이 있으면 "목표 방향 막힘"

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 스캔 범위 & 통신
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCAN_WIDE_HALF = 135   # 측면 반발력 감지 범위 (is_in_wide_scan 사용) (각도)
SEND_INTERVAL  = 0.1

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 측면 반발력 파라미터 (horiz 190mm × fwd 160mm 감지 구간)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SIDE_SAFE_MARGIN  = 300   # mm: 로봇 측면 안전 마진 (side_th = 110+190 = 300mm)
SIDE_FWD_LEAD     = 90    # mm: 라이다 기준 전방 여유 (진입 예측)
SIDE_FWD_REAR     = 90    # mm: 라이다 기준 후방 깊이 (로봇 몸체)
SIDE_REPULSE_GAIN = 1.25   # rad/s: 반발력 최대 w 기여
SIDE_EXP_K        = 2.0   # 지수 계수: 클수록 근접 시 반발력이 급격히 증가

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 측방 방향 레이어 (±15°~±75°, 600mm)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SIDE_LAYER_ANG_START = 15   # deg: 정면 레이어와 경계
SIDE_LAYER_ANG_END   = 75   # deg: 측방 레이어 바깥 경계
SIDE_LAYER_DIST_MAX  = 700  # mm: 측방 감지 최대 거리
SIDE_W_BOOST_GAIN    = 1.5  # rad/s: 측방 레이어 w 크기 기여 계수 (우측 push → +w, 좌측 push → -w)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [추가] 가상 장애물 (통과 불가 갭)  ─ 코드 1에서 이식
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [Option A] MIN_PASSAGE_WIDTH 를 STOP_ESCAPE_MIN_GAP 과 동일하게 정렬.
#   → "통과 가능" 정의가 front gap bonus 와 가상 장애물에서 일치
#   → 모든 갭은 항상 척력(< 기준) 또는 보너스(>= 기준), 중립 구간 없음
#   → 기준 한 곳(STOP_ESCAPE_MIN_GAP)만 바꾸면 두 시스템이 함께 따라감

MIN_PASSAGE_WIDTH       = STOP_ESCAPE_MIN_GAP  # 260mm: 이 미만 갭 → 통과 불가 → 가상 장애물
VIRTUAL_OBS_GAIN        = 1.5   # 가상 장애물 척력 배율 (레이어별 horiz_th 기준)
VIRTUAL_CENTER_DEADBAND = 10    # deg: 갭 중심이 ±이내면 정면 → 양쪽 동등 척력
                                 # 0° 근처 노이즈로 인한 방향 편향 방지
VIRTUAL_EXP_K           = 2.5   # 지수 계수: 클수록 좁은 갭에서 척력이 급격히 증가
                                 # 1.0=거의 선형 / 2.5=권장 / 3.5=급격 / 5.0↑=거의 이진
                                 # SIDE_EXP_K(2.0)보다 약간 크게 (갭은 더 민감하게)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 가상 경계 & 탐색 모드  ─ 코드 2에서 이식 (Mode1 / Mode2)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Mode 1 (도착후탐색): 중심 = 이전 색지 도착 위치
#   ① 초기 360° 피버턴 스캔
#   ② 피버턴 완료 후 장애물 회피 + 주기적 재피버턴
# Mode 2 (추적중소실): 중심 = 마지막으로 감지된 목표 색지 추정 위치
#   장애물 회피 + 경계 복귀만 수행 (피버턴 없음)
# 경계 이탈→복귀가 BOUNDARY_EXPAND_TRIGGER 회 누적되면 반경 확장.

BOUNDARY_RADIUS         = 1000.0   # mm: 기본 경계 반경
BOUNDARY_RADIUS_MAX     = 2000.0   # mm: 최대 경계 반경
BOUNDARY_RADIUS_EXPAND  = 500.0    # mm: 경계 1회 확장 폭
BOUNDARY_EXPAND_TRIGGER = 2        # 회: 이 횟수 이상 이탈→복귀 시 경계 확장
BOUNDARY_HYSTERESIS_MM  = 150.0    # mm: 경계 진입/이탈 히스테리시스 (진동 방지)
BOUNDARY_BLEND_DIST     = 300.0    # mm: 경계 초과 후 인력 100%까지 도달하는 거리
BOUNDARY_V_MIN          = 0.5      # 경계 완전 초과 시 v 감속 최소 비율
BOUNDARY_ALIGN_ANGLE    = 120.0    # deg: 중심 방향이 이 각도 이상 벗어나면 v→0 (등진 채 전진=발산 방지)
BOUNDARY_REAR_LATCH_DEG = 150.0    # deg: 중심이 이 각도 이상 후방이면 회전 방향 고정 (±180° 떨림 방지)

# Mode 1 피버턴 파라미터
PIVOT_W_SPEED      = 0.6    # rad/s: 탐색 피버턴 회전 속도
PIVOT_INTERVAL_DIST = 500.0   # mm: 마지막 피버턴 위치에서 이만큼 이동 시 재피버턴 (주 트리거)
PIVOT_TIMEOUT_SEC   = 15.0    # sec: 경계/장애물에 갇혀 못 움직여도 이 시간 경과 시 재피버턴 (안전망)
# Mode2 → Mode1 복귀: 시간이 아닌 '루프(같은 구역 반복 재진입)' 감지로 전환.
#   루프 판정 파라미터는 아래 LOOP_CELL_MM / LOOP_REVISIT_TRIG / LOOP_TRACK_SEC 재사용.

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 루프(리미트 사이클) 탈출
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Mode 1 회피 주행 중, 경계 인력 + 회피가 만드는 닫힌 궤도에 갇히는 문제 해결.
# 같은 셀에 반복 재진입하면 루프로 판정 → 제자리 회전으로 평형을 깸.
#   · 클리어런스 OK (대부분: 트인 곳) → 제자리 회전
#   · 클리어런스 부족 (드묾: 좁은 곳)  → 편향 회피로 자리 이탈 (STOP은 find_vw가 처리)
#   · 같은 자리에서 LOOP_REFLIP_TRIG회 이상 재탈출 → 탈출 방향 좌우 교번
#   · 다른 위치로 이동 성공 시 자동 리셋
LOOP_ESCAPE_ENABLED = False  # ★ 루프 탈출 메커니즘 on/off (False=비활성, 재피버턴·경계만 사용)
LOOP_CELL_MM      = 400.0   # mm: 위치 양자화 셀 크기 (2m 공간 → 약 5×5칸)
LOOP_REVISIT_TRIG = 4       # 회: 같은 셀 재진입 누적 이 횟수 → 루프로 판정
LOOP_ESCAPE_SEC   = 1.5     # sec: 탈출 강제 회전 지속 시간 (트인 곳이라 짧게)
LOOP_TRACK_SEC    = 12.0    # sec: 셀 재진입 카운트 유효 시간 (지나면 리셋)
LOOP_REFLIP_TRIG  = 3       # 회: 같은 자리 재탈출 이 횟수 이상 → 방향 반전
LOOP_SAME_SPOT_MM = 500.0   # mm: 직전 탈출과 이 거리 이내면 '같은 자리'로 간주

# ── 디버그 토글 ──────────────────────────────────────────────────────────────
DEBUG_LAYERS      = 0   # [L1~L6] 레이어 분석 결과
DEBUG_STOP        = 0   # [STOP] 발동 & 탈출 이벤트
DEBUG_STOP_PIVOT  = 0   # [STOP] 피봇 중 매 사이클 (noisy)
DEBUG_BRANCH      = 1   # [BRANCH] 분기 결정
DEBUG_TARGET      = 0   # [TARGET] 목표 방향 직진
DEBUG_GAP         = 1   # [GAP_FOLLOW] 갭 우회
DEBUG_FALLBACK    = 0   # [FALLBACK] 점수 기반 회피 요약
DEBUG_SCORE       = 0   # [SCORE] 점수 상세 (FALLBACK 하위)
DEBUG_DIR         = 0   # [DIR] 방향 결정 결과
DEBUG_CLEAR       = 0   # [CLEAR] 장애물 없음
DEBUG_FINAL       = 1   # [FINAL] 최종 v, w
DEBUG_SIDE        = 0   # [SIDE] 측면 반발력
DEBUG_SIDE_LAYER  = 0   # [SIDE_LAYER] 측방 레이어
DEBUG_VIRTUAL     = 0   # [VIRTUAL] 가상 장애물
DEBUG_CLOSE_INIT   = 0   # [CLOSE] 목표 좌표 계산 (진입 1회)
DEBUG_CLOSE_POS    = 0   # [CLOSE] 접근 중 위치/거리
DEBUG_CLOSE_HDG    = 0   # [CLOSE] 헤딩 오차 계산 (arduino_hdg / target_hdg / hdg_err / w)
DEBUG_CLOSE_DONE   = 0   # [CLOSE] 도달 판정
DEBUG_CLOSE_REMAIN = 0   # [CLOSE] 남은 거리 / 진행률 (매 사이클)
DEBUG_BOUNDARY    = 0   # [BOUNDARY] 가상 경계 초과 시
DEBUG_SEND        = 0   # [SEND] 모터 명령 전송

# ── 전역 상태 ────────────────────────────────────────────────────────────────
arduino_heading_deg   = 0.0
arduino_x_mm          = 0.0   # 오도메트리 x 위치 (mm, 우측 +)
arduino_y_mm          = 0.0   # 오도메트리 y 위치 (mm, 전방 +)
prev_w                = 0.0

# ── CLOSE 접근 제어 ───────────────────────────────────────────────────────────
_close_target_x    = None   # 색지 추정 x 좌표 (mm)
_close_target_y    = None   # 색지 추정 y 좌표 (mm)
_close_initial_dist = None  # CLOSE 진입 시 초기 거리 (진행률 계산용)
_close_observe_start = None # CLOSE 정지 관측 시작 시각
KP_CLOSE_HDG      = 0.1  # 헤딩 오차(deg) → w 게인  (포화: ±° → MAX_W)
CLOSE_SPEED_MAX   = 0.2   # CLOSE 모드 최대 전진 속도 (m/s)
CLOSE_SPEED_MIN   = 0.08  # CLOSE 모드 최소 전진 속도 (목표 근접 시 감속 하한, 부드러운 정차)
CLOSE_BRAKE_DIST_MM = 150.0  # 목표 이 거리(mm) 이내부터 SPEED_MIN까지 선형 감속
CLOSE_ARRIVE_MM   = 10    # 추정 좌표까지 이 거리 이내 → 색지 위 도달로 판정
CLOSE_OBSERVE_SEC = 1.0   # CLOSE 진입 후 정지 관측 시간 (sec)
STOP_EARLY_MM     = 50.0  # 관성 밀림 보상: 목표 좌표를 추정 거리보다 이만큼 앞당겨 찍음
prev_desired_heading  = 0.0   # 직전 사이클 조향 목표 각도 (갭 선택 평활화용)
_last_direction       = 1.0   # 마지막으로 결정된 방향 (+1=왼쪽, -1=오른쪽)
stop_cycle_count           = 0     # 현재 phase 내 사이클 카운터
stop_pivot_w               = 0.0   # 피봇 방향 (부호만 사용)
stop_locked_target         = 0.0
stop_locked_gap            = 0.0
stop_locked_global_heading = 0.0
stop_phase                 = 0     # 0=idle, 2=피봇

# ── 탐색 모드 & 경계 전역 상태 ───────────────────────────────────────────────
_search_mode             = 0      # 0=정상, 1=도착후탐색(Mode1), 2=추적중소실(Mode2)
_last_arrival_x          = None   # mm: 이전 색지 도착 위치 x (Mode1 경계 중심)
_last_arrival_y          = None   # mm: 이전 색지 도착 위치 y
_last_target_est_x       = None   # mm: 마지막 감지 목표 추정 위치 x (Mode2 경계 중심)
_last_target_est_y       = None   # mm: 마지막 감지 목표 추정 위치 y
_last_known_mission_idx  = 0      # 미션 인덱스 변화 감지용

# ── Mode 1 피버턴 상태 ────────────────────────────────────────────────────────
_pivot_active        = False   # True: 피버턴 진행 중
_pivot_prev_hdg      = 0.0    # 이전 사이클 헤딩 (누적 회전 계산용)
_pivot_total_rotated = 0.0    # 누적 회전량 (deg)
_pivot_direction     = 1.0    # +1=CCW, -1=CW
_last_pivot_time     = 0.0    # 마지막 피버턴 완료 시각 (시간 안전망용)
_last_pivot_x        = None   # mm: 마지막 피버턴 완료 위치 x (거리 주기 기준점)
_last_pivot_y        = None   # mm: 마지막 피버턴 완료 위치 y

# ── 가변 경계 상태 ────────────────────────────────────────────────────────────
_current_boundary_radius = BOUNDARY_RADIUS   # mm: 현재 적용 경계 반경
_boundary_exit_count     = 0                  # 경계 이탈→복귀 누적 횟수 (경계 확장용)
_boundary_was_outside    = False              # 히스테리시스: 경계 확장 카운트용 외부 여부
_boundary_pulling        = False              # 히스테리시스: 인력 활성 여부 (경계선 깜빡임 방지)
_boundary_turn_sign      = 0.0                # 정후방 회전 방향 래치 (0=미설정, ±1=고정)

# ── 루프 탈출 상태 ────────────────────────────────────────────────────────────
_loop_cell          = None   # (cx, cy): 현재 추적 중인 셀
_loop_count         = 0      # 그 셀 재진입 횟수
_loop_cell_time     = 0.0    # 마지막 카운트 갱신 시각
_loop_escape_until  = 0.0    # 이 시각까지 탈출 회전 강제
_loop_escape_w      = 0.0    # 탈출 회전 방향 (부호)
_loop_escape_count  = 0      # 같은 자리 연속 탈출 횟수 (방향 교번 트리거)
_loop_last_escape_x = None   # mm: 직전 탈출 발동 위치 x
_loop_last_escape_y = None   # mm: 직전 탈출 발동 위치 y
_loop_flip          = 1.0    # 방향 교번 부호 (재탈출 시 ×-1)

# ── Mode2 루프 감지 상태 (Mode1 복귀 트리거용) ─────────────────────────────────
_m2_loop_cell      = None   # (cx, cy): Mode2에서 추적 중인 셀
_m2_loop_count     = 0      # 그 셀 재진입 횟수
_m2_loop_cell_time = 0.0    # 마지막 카운트 갱신 시각

# ── 스레드 공유 상태 ─────────────────────────────────────────────────────────
_scan_lock   = threading.Lock()
_latest_scan = []            # 라이다 스레드가 완성된 스캔을 여기에 기록
_shutdown    = threading.Event()  # 종료 신호

# ── STOP 이벤트 비차단 로깅 ──────────────────────────────
STOP_LOG_ENABLED  = False              # 토글: 대회 런=False / 디버그=True
_stop_log_queue   = queue.Queue(maxsize=20)
_stop_log_counter = 0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return ((angle + 180) % 360) - 180

def is_in_front_90(a):
    return -90 <= a <= 90

def is_in_wide_scan(a):
    return -SCAN_WIDE_HALF <= a <= SCAN_WIDE_HALF

def is_rear_blind(angle):
    """후방 카메라 구조물 차폐 각도(|angle| > 180-REAR_BLIND_HALF)면 True."""
    return abs(angle) > 180.0 - REAR_BLIND_HALF

def is_pivot_clearance_ok(scan_points, radius):
    """후방 사각 제외, 반경 radius 내 유효 장애물이 없으면 True.
    LIDAR_MIN_VALID 미만 노이즈와 후방 카메라 구조물은 무시."""
    for angle, dist in scan_points:
        if dist < LIDAR_MIN_VALID:
            continue
        if is_rear_blind(angle):
            continue
        if dist <= radius:
            return False
    return True

def decompose(angle_deg, dist):
    rad = math.radians(angle_deg)
    horiz = abs(dist * math.sin(rad))
    fwd   = dist * math.cos(rad)
    return horiz, fwd

def cosine_dist(d1, d2, angle_diff_deg):
    theta = math.radians(abs(angle_diff_deg))
    return math.sqrt(d1**2 + d2**2 - 2 * d1 * d2 * math.cos(theta))

def point_to_segment_dist(px, py, ax, ay, bx, by):
    """점 P에서 선분 AB까지의 최단 거리.
    수직 교점이 선분 안에 있으면 수직거리, 바깥이면 가까운 끝점 거리."""
    dx, dy = bx - ax, by - ay
    seg_sq = dx*dx + dy*dy
    if seg_sq == 0:
        return math.sqrt((px - ax)**2 + (py - ay)**2)
    t = max(0.0, min(1.0, ((px - ax)*dx + (py - ay)*dy) / seg_sq))
    return math.sqrt((px - ax - t*dx)**2 + (py - ay - t*dy)**2)

def nearest_to_segments(px, py, cluster_xy):
    """점 P에서 클러스터 선분들(인접 포인트 쌍) 중 최단 거리.
    클러스터가 1점이면 점-점 거리로 fallback."""
    if len(cluster_xy) == 1:
        ox, oy = cluster_xy[0]
        return math.sqrt((px - ox)**2 + (py - oy)**2)
    return min(
        point_to_segment_dist(px, py,
                              cluster_xy[j][0], cluster_xy[j][1],
                              cluster_xy[j+1][0], cluster_xy[j+1][1])
        for j in range(len(cluster_xy) - 1)
    )

def parse_packet(data):
    if len(data) != 5: return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    distance_q2 = data[3] | (data[4] << 8)
    return (angle_q6 / 64.0), (distance_q2 / 4.0)

def read_arduino(arduino):
    global arduino_heading_deg, arduino_x_mm, arduino_y_mm
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('O:'):
                parts = line[2:].split(',')
                if len(parts) == 3:
                    arduino_x_mm        = float(parts[0])
                    arduino_y_mm        = float(parts[1])
                    arduino_heading_deg = float(parts[2])
            elif line.startswith('H:'):   # 구버전 아두이노 호환
                arduino_heading_deg = float(line[2:])
        except Exception: pass


def _compute_close_target():
    """CLOSE 진입 시 색지 추정 좌표 계산. (x_mm, y_mm) 반환."""
    bearing_global_deg = arduino_heading_deg + camera_tracker.get_last_close_bearing()
    # 관성 밀림 보상: 추정 거리보다 STOP_EARLY_MM 앞당겨 목표를 찍음
    dist_mm            = max(camera_tracker.get_estimated_distance_mm() - STOP_EARLY_MM, 50.0)
    hdg_rad            = math.radians(bearing_global_deg)
    x_t = arduino_x_mm + dist_mm * math.sin(hdg_rad)
    y_t = arduino_y_mm + dist_mm * math.cos(hdg_rad)
    if DEBUG_CLOSE_INIT:
        print(f"[CLOSE] 목표 좌표: ({x_t:.0f}, {y_t:.0f})mm  "
              f"dist={dist_mm:.0f}mm  global_bearing={bearing_global_deg:.1f}°")
    return x_t, y_t


# ── camera_tracker.get_state() → 미션 인덱스 변환 ────────────────────
# camera_tracker에 get_mission_idx()가 없으므로 get_state()로 대체.
#   get_state(): 'SEEK_RED' / 'SEEK_YELLOW' / 'SEEK_BLUE' / 'DONE'
#   → RED=0, YELLOW=1, BLUE=2, DONE=3
_MISSION_ORDER = ['RED', 'YELLOW', 'BLUE']

def _get_mission_idx():
    """현재 미션 인덱스 반환 (도착 시 색지 전환 감지용)."""
    state = camera_tracker.get_state()
    if state == 'DONE':
        return len(_MISSION_ORDER)
    color = state.replace('SEEK_', '')
    try:
        return _MISSION_ORDER.index(color)
    except ValueError:
        return 0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 가상 경계 & 탐색 모드 함수  ─ 코드 2에서 이식
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _get_boundary_correction(center_x, center_y, radius):
    """
    경계 초과 시 중심 방향 상대 베어링과 v 감속 비율을 반환.
    중심 방향을 target_bearing으로 find_vw_command()에 전달 →
    갭 추종·레이어 회피가 장애물을 우회하면서 중심으로 복귀.

    경계 내부:  rel_bearing=0.0, v_scale=1.0
    경계 초과:  blend=[0,1], v_scale=[BOUNDARY_V_MIN, 1.0]
    반환: (rel_bearing_deg, v_scale)
    """
    global _boundary_pulling, _boundary_turn_sign

    dx   = center_x - arduino_x_mm
    dy   = center_y - arduino_y_mm
    dist = math.sqrt(dx**2 + dy**2)

    # 인력 히스테리시스: radius 넘으면 ON, radius-HYST 안으로 들어와야 OFF
    #   → 경계선 근처에서 인력이 켜졌다 꺼졌다 하며 생기는 진동 방지
    if _boundary_pulling:
        if dist < radius - BOUNDARY_HYSTERESIS_MM:
            _boundary_pulling = False
    elif dist > radius:
        _boundary_pulling = True

    if not _boundary_pulling:
        _boundary_turn_sign = 0.0
        return 0.0, 1.0

    excess = max(dist - radius, 0.0)
    blend  = min(excess / BOUNDARY_BLEND_DIST, 1.0)

    # ★ 부호 체계: 카메라 bearing(오른쪽=음수)과 일치시켜야 분기①(+KP_GOAL)이
    #    중심 '쪽'으로 조향한다. atan2(dx,dy)는 오른쪽=양수(LiDAR 체계)이므로 반전.
    bearing_to_center = math.degrees(math.atan2(dx, dy))
    rel_bearing       = normalize_angle(arduino_heading_deg - bearing_to_center)

    # 정후방 떨림 방지: 중심이 후방(±180° 부근)이면 회전 방향을 한쪽으로 고정.
    #   normalize_angle이 +179/-179를 오가며 w 부호가 반전하는 좌우 떨림 차단.
    if abs(rel_bearing) > BOUNDARY_REAR_LATCH_DEG:
        if _boundary_turn_sign == 0.0:
            _boundary_turn_sign = 1.0 if rel_bearing >= 0 else -1.0
        rel_bearing = _boundary_turn_sign * abs(rel_bearing)
    else:
        _boundary_turn_sign = 0.0

    # 거리 기반 감속 × 각도 기반 감속:
    #   중심을 등지면(각도>ALIGN) v→0 제자리 회전 → 등진 채 전진(발산) 방지
    v_dist  = BOUNDARY_V_MIN + (1.0 - BOUNDARY_V_MIN) * (1.0 - blend)
    v_align = max(0.0, 1.0 - abs(rel_bearing) / BOUNDARY_ALIGN_ANGLE)
    v_scale = v_dist * v_align

    if DEBUG_BOUNDARY:
        print(f"  [BOUNDARY] dist={dist:.0f}mm radius={radius:.0f}mm "
              f"excess={excess:.0f}mm blend={blend:.2f} "
              f"target_b={rel_bearing:+.1f}° v_scale={v_scale:.2f} "
              f"pull={_boundary_pulling} latch={_boundary_turn_sign:+.0f}")

    return rel_bearing, v_scale


def _update_target_estimate():
    """색 감지 중 목표 색지 추정 위치 갱신 (Mode2 경계 중심용).
    거리 추정이 신뢰 범위(4000mm) 밖이면 갱신 생략."""
    global _last_target_est_x, _last_target_est_y
    dist_mm = camera_tracker.get_estimated_distance_mm()
    if dist_mm >= 4000.0:
        return
    bearing_rel = camera_tracker.get_last_stable_bearing()
    global_hdg  = arduino_heading_deg + bearing_rel
    hdg_rad     = math.radians(global_hdg)
    _last_target_est_x = arduino_x_mm + dist_mm * math.sin(hdg_rad)
    _last_target_est_y = arduino_y_mm + dist_mm * math.cos(hdg_rad)


def _update_boundary_exit_tracking(center_x, center_y):
    """경계 이탈→복귀 횟수 추적. 히스테리시스 적용.
    BOUNDARY_EXPAND_TRIGGER 회 누적 시 경계 반경 확장."""
    global _boundary_was_outside, _boundary_exit_count, _current_boundary_radius

    dx   = center_x - arduino_x_mm
    dy   = center_y - arduino_y_mm
    dist = math.sqrt(dx**2 + dy**2)

    out_threshold = _current_boundary_radius + BOUNDARY_HYSTERESIS_MM
    in_threshold  = _current_boundary_radius - BOUNDARY_HYSTERESIS_MM

    if not _boundary_was_outside and dist > out_threshold:
        _boundary_was_outside = True
        if DEBUG_BOUNDARY:
            print(f"  [BOUNDARY] 경계 이탈 (dist={dist:.0f}mm "
                  f"radius={_current_boundary_radius:.0f}mm)")
    elif _boundary_was_outside and dist < in_threshold:
        _boundary_was_outside = False
        _boundary_exit_count += 1
        if DEBUG_BOUNDARY:
            print(f"  [BOUNDARY] 경계 복귀 "
                  f"(카운트={_boundary_exit_count}/{BOUNDARY_EXPAND_TRIGGER})")
        if _boundary_exit_count >= BOUNDARY_EXPAND_TRIGGER:
            if _current_boundary_radius < BOUNDARY_RADIUS_MAX:
                _current_boundary_radius = min(
                    _current_boundary_radius + BOUNDARY_RADIUS_EXPAND,
                    BOUNDARY_RADIUS_MAX
                )
                _boundary_exit_count = 0
                print(f"  [BOUNDARY] 경계 확장 → {_current_boundary_radius:.0f}mm")


def _loop_check_and_escape(pts):
    """루프 감지 시 강제 (v, w)를 반환, 아니면 None(=평소 로직 진행).

    탈출 진행 중:
      클리어런스 OK → 제자리 회전 (주력)
      클리어런스 X  → 편향 회피로 자리 이탈 (폴백, STOP은 find_vw_command가 우선 처리)
    탈출 미진행:
      같은 셀 재진입 카운트 누적 → LOOP_REVISIT_TRIG 도달 시 탈출 개시 결정.
      같은 자리 반복 탈출이면 방향을 좌우로 뒤집어 다른 탈출로 시도.
    """
    global _loop_cell, _loop_count, _loop_cell_time, \
           _loop_escape_until, _loop_escape_w, \
           _loop_escape_count, _loop_last_escape_x, _loop_last_escape_y, _loop_flip

    if not LOOP_ESCAPE_ENABLED:
        return None   # 비활성: 평소 로직(재피버턴·경계 복귀)만 진행

    now = time.time()

    # ── 탈출 진행 중 ──────────────────────────────────────────────────────────
    if now < _loop_escape_until:
        if is_pivot_clearance_ok(pts, PIVOT_CLEAR_RADIUS):
            return 0.0, _loop_escape_w               # 주력: 제자리 회전으로 궤도 깸
        # 드문 케이스: 회전 불가 → 보류 말고 편향 회피로 일단 자리 이탈
        v, w = find_vw_command(pts, arduino_heading_deg, target_bearing=0.0)
        w = max(min(w + _loop_escape_w * 0.5, MAX_W), -MAX_W)
        return v, w

    # ── 셀 재진입 추적 ────────────────────────────────────────────────────────
    cell = (int(arduino_x_mm // LOOP_CELL_MM),
            int(arduino_y_mm // LOOP_CELL_MM))

    # 추적 카운트 만료 → 리셋 (정상 이동 중)
    if now - _loop_cell_time > LOOP_TRACK_SEC:
        _loop_cell, _loop_count, _loop_cell_time = cell, 1, now
        return None

    if cell == _loop_cell:
        _loop_count += 1
        _loop_cell_time = now

        if _loop_count >= LOOP_REVISIT_TRIG:
            # ── 같은 자리 재탈출 여부 판정 ────────────────────────────────────
            if _loop_last_escape_x is not None:
                spot_dist = math.hypot(arduino_x_mm - _loop_last_escape_x,
                                       arduino_y_mm - _loop_last_escape_y)
            else:
                spot_dist = 1e9

            if spot_dist < LOOP_SAME_SPOT_MM:
                _loop_escape_count += 1      # 같은 자리 → 누적
            else:
                _loop_escape_count = 1        # 새 위치 → 리셋
                _loop_flip = 1.0              # 방향 교번도 초기화

            # ── 탈출 방향 결정 ───────────────────────────────────────────────
            if _loop_escape_count >= LOOP_REFLIP_TRIG:
                # 같은 자리 반복 → 직전과 반대로 강제 (좌우 교번)
                _loop_flip *= -1.0
                _loop_escape_w = _loop_flip * MAX_W
                print(f"[LOOP] 재루프 {_loop_escape_count}회 → 방향 반전 "
                      f"w={_loop_escape_w:+.1f}")
            else:
                # 평소: 트인 쪽으로 (둘 다/둘 다 아니면 좌회전 기본)
                left_open  = not is_target_blocked(pts, -50)
                right_open = not is_target_blocked(pts,  50)
                _loop_escape_w = +MAX_W if (right_open and not left_open) else -MAX_W
                print(f"[LOOP] 루프 감지 → {LOOP_ESCAPE_SEC:.1f}s 제자리 회전 "
                      f"w={_loop_escape_w:+.1f} cell={cell}")

            # 탈출 개시 (다음 사이클부터 위 '탈출 진행 중' 블록이 받음)
            _loop_last_escape_x = arduino_x_mm
            _loop_last_escape_y = arduino_y_mm
            _loop_escape_until  = now + LOOP_ESCAPE_SEC
            _loop_count, _loop_cell_time = 0, now
            return None
    else:
        # 다른 셀로 이동 → 추적 대상 갱신 (정상 이동 중)
        _loop_cell, _loop_count, _loop_cell_time = cell, 1, now

    return None


def _loop_reset():
    """탐색 모드 전환/색지 재감지 시 루프 추적 상태 초기화."""
    global _loop_cell, _loop_count, _loop_cell_time, \
           _loop_escape_until, _loop_escape_w, \
           _loop_escape_count, _loop_last_escape_x, _loop_last_escape_y, _loop_flip
    _loop_cell          = None
    _loop_count         = 0
    _loop_cell_time     = 0.0
    _loop_escape_until  = 0.0
    _loop_escape_w      = 0.0
    _loop_escape_count  = 0
    _loop_last_escape_x = None
    _loop_last_escape_y = None
    _loop_flip          = 1.0


def _detect_mode2_loop():
    """Mode2 주행 중 같은 셀 반복 재진입(루프) 감지 → True면 Mode1 복귀 신호.
    LOOP_TRACK_SEC 윈도우 내 같은 셀에 LOOP_REVISIT_TRIG회 이상 재진입하면 루프로 판정.
    (루프 탈출 메커니즘과 동일한 셀 추적 방식, 단 전역 상태는 분리.)"""
    global _m2_loop_cell, _m2_loop_count, _m2_loop_cell_time
    now  = time.time()
    cell = (int(arduino_x_mm // LOOP_CELL_MM),
            int(arduino_y_mm // LOOP_CELL_MM))

    # 추적 윈도우 만료 → 리셋 (정상 이동 중)
    if now - _m2_loop_cell_time > LOOP_TRACK_SEC:
        _m2_loop_cell, _m2_loop_count, _m2_loop_cell_time = cell, 1, now
        return False

    if cell == _m2_loop_cell:
        _m2_loop_count += 1
        _m2_loop_cell_time = now
        if _m2_loop_count >= LOOP_REVISIT_TRIG:
            return True
    else:
        # 다른 셀로 이동 → 추적 대상 갱신 (정상 이동 중)
        _m2_loop_cell, _m2_loop_count, _m2_loop_cell_time = cell, 1, now

    return False


def _reset_mode2_loop():
    """Mode2 루프 추적 상태 초기화."""
    global _m2_loop_cell, _m2_loop_count, _m2_loop_cell_time
    _m2_loop_cell      = None
    _m2_loop_count     = 0
    _m2_loop_cell_time = 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP zone 감지 & 탈출
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def detect_stop_zone(scan_points):
    """STOP rectangle (fwd 100~175mm, horiz<105mm) 안에 장애물이 있는가?"""
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_front_90(angle_norm): continue
        horiz, fwd = decompose(angle_norm, dist)
        if STOP_FWD_MIN <= fwd <= STOP_FWD_MAX and horiz < STOP_HORIZ_TH:
            return True
    return False


def find_all_gaps(scan_points):
    """
    FGM: 360° 전체 스캔에서 depth jump / 각도 공백을 기준으로
    모든 갭(장애물 경계 쌍)을 추출.

    너비 계산: 에지 간 코사인 거리 대신,
    각 에지에서 반대편 클러스터의 최근접 점까지 Euclidean 거리로 측정.
    → 실제 통과 가능 폭을 물리적으로 정확히 반영.
    반환: list of dict {width, center_angle, edge_a, edge_b, depth}
    """
    pts = sorted(
        [(a, d) for a, d in scan_points
         if LIDAR_MIN_VALID < d < FGM_MAX_RANGE_MM],
        key=lambda p: p[0]
    )
    if len(pts) < 2:
        return []

    def to_xy(a, d):
        r = math.radians(a)
        return d * math.sin(r), d * math.cos(r)

    # 갭 경계 인덱스 탐색
    gap_indices = []
    for i in range(len(pts) - 1):
        a1, d1 = pts[i]
        a2, d2 = pts[i + 1]
        ang_diff = a2 - a1
        is_depth_jump   = abs(d2 - d1) > DEPTH_JUMP_THRES
        is_angular_hole = ang_diff >= FGM_MIN_ANG_DEG
        is_ratio_jump   = (d2 / d1 > FGM_RATIO_THRES) or (d1 / d2 > FGM_RATIO_THRES)
        if is_depth_jump or is_angular_hole or is_ratio_jump:
            gap_indices.append(i)

    if not gap_indices:
        return []

    # 클러스터 레이블링: 갭 경계마다 새 클러스터 번호 부여
    gap_set = set(gap_indices)
    cluster_ids = []
    cid = 0
    for i in range(len(pts)):
        cluster_ids.append(cid)
        if i in gap_set:
            cid += 1

    # 클러스터별 Cartesian 점 목록
    n_clusters = cluster_ids[-1] + 1
    clusters_xy = [[] for _ in range(n_clusters)]
    for i, (a, d) in enumerate(pts):
        clusters_xy[cluster_ids[i]].append(to_xy(a, d))

    gaps = []
    for i in gap_indices:
        a1, d1 = pts[i]
        a2, d2 = pts[i + 1]
        x1, y1 = to_xy(a1, d1)
        x2, y2 = to_xy(a2, d2)

        cid_L = cluster_ids[i]
        cid_R = cluster_ids[i + 1]

        # 왼쪽 에지 → 오른쪽 클러스터 선분 최근접 거리
        d_LR = nearest_to_segments(x1, y1, clusters_xy[cid_R])
        # 오른쪽 에지 → 왼쪽 클러스터 선분 최근접 거리
        d_RL = nearest_to_segments(x2, y2, clusters_xy[cid_L])

        # 실제 통과 가능 폭: 두 방향 최단 거리 중 더 좁은 쪽
        width = min(d_LR, d_RL)

        # 갭 중심: 두 에지의 Cartesian 중점 → atan2 (각도 평균은 wrap 위험)
        center_angle = math.degrees(math.atan2((x1 + x2) / 2, (y1 + y2) / 2))

        gaps.append({
            'width':        width,
            'center_angle': center_angle,
            'edge_a':       (a1, d1),
            'edge_b':       (a2, d2),
            'depth':        max(d1, d2),
        })
    return gaps


def choose_escape_gap(gaps, prefer_angle=0.0):
    """
    통과 가능한 갭(폭 >= STOP_ESCAPE_MIN_GAP, 깊이 >= FGM_MIN_DEPTH_MM) 중
    prefer_angle에 가장 가까운 갭 선택.
    통과 가능한 갭이 없으면 None 반환 (fallback 제거 — 불통과 갭 진입 방지).
    """
    passable = [g for g in gaps
                if g['width'] >= STOP_ESCAPE_MIN_GAP
                and g['depth'] >= FGM_MIN_DEPTH_MM]
    if passable:
        return min(passable,
                   key=lambda g: abs(((g['center_angle'] - prefer_angle) + 180) % 360 - 180))
    return None


def find_stop_escape_direction(scan_points, heading_deg=0.0):
    """FGM 기반 STOP 탈출 방향 결정. 반환: (target_angle, gap_width, gap_info_list)
    heading_deg 기준 글로벌 0°에 가장 가까운(최소 회전) 통과 가능 갭 선택.
    """
    gaps   = find_all_gaps(scan_points)
    chosen = choose_escape_gap(gaps, prefer_angle=heading_deg)

    if chosen is None:
        return 0.0, 0.0, []

    gap_info = [
        {
            'width':        g['width'],
            'center_angle': g['center_angle'],
            'edge_a':       list(g['edge_a']),
            'edge_b':       list(g['edge_b']),
            'depth':        g['depth'],
            'passable':     g['width'] >= STOP_ESCAPE_MIN_GAP and g['depth'] >= FGM_MIN_DEPTH_MM,
            'chosen':       g is chosen,
        }
        for g in gaps
    ]
    return float(chosen['center_angle']), float(chosen['width']), gap_info


def find_passable_gap_for_pivot(scan_points):
    """Mode 1 재피버턴 전용: 통과 가능하고 후방 사각이 아닌 갭 중 가장 넓은 것 반환.
    find_all_gaps에 전체 스캔을 넘긴 뒤 결과만 필터링 →
    사전 각도 필터링으로 인한 경계 인공 갭(가짜 갭) 생성 방지.
    없으면 None."""
    passable = [g for g in find_all_gaps(scan_points)
                if g['width'] >= STOP_ESCAPE_MIN_GAP
                and g['depth'] >= FGM_MIN_DEPTH_MM
                and not is_rear_blind(g['center_angle'])]
    if not passable:
        return None
    return max(passable, key=lambda g: g['width'])


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 레이어 처리
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def process_layer(scan_points, layer):
    """
    레이어 내 포인트들을 모아 분석.

    반환:
      None: 레이어 활성화 안 됨 (포인트 없음)
      dict: 분석 결과
        - weight: 이번 사이클 가중치 (동적 또는 고정)
        - urgency: w_gain × horiz_error / horiz_th
        - v_proposal: v 제안 (affects_v=True일 때만, 그 외 None)
        - rep_angle/horiz/fwd: 하위 5% 평균 대표점
        - push_left/push_right: 방향 점수용 (포인트 angle 부호로 분리)
    """
    pts = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_front_90(angle_norm): continue
        horiz, fwd = decompose(angle_norm, dist)
        if layer['fwd_min'] <= fwd < layer['fwd_max'] and horiz < layer['horiz_th']:
            pts.append({
                'angle': angle_norm, 'dist': dist,
                'horiz': horiz, 'fwd': fwd,
                'horiz_error': layer['horiz_th'] - horiz,
            })

    if not pts:
        return None

    # 하위 5% dist 포인트 → 대표점
    n_take = max(1, int(len(pts) * LAYER_PERCENTILE / 100))
    rep = sorted(pts, key=lambda p: p['dist'])[:n_take]

    rep_angle = sum(p['angle'] for p in rep) / len(rep)
    rep_horiz = sum(p['horiz'] for p in rep) / len(rep)
    rep_fwd   = sum(p['fwd']   for p in rep) / len(rep)
    rep_h_err = layer['horiz_th'] - rep_horiz

    # 가중치
    if layer['weight_dynamic']:
        cap = layer.get('weight_cap', 1.0)
        raw = rep_h_err / layer['horiz_th'] * cap
        weight = max(layer['weight_base'], min(cap, raw))
    else:
        # L4~L6: fwd 위치에 따라 weight_start → weight_base 선형 보간
        progress = (rep_fwd - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min'])
        progress = max(0.0, min(1.0, progress))
        weight = layer['weight_start'] + (layer['weight_base'] - layer['weight_start']) * progress

    # urgency (w 크기 기여)
    urgency = layer['w_gain'] * rep_h_err / layer['horiz_th']

    # v_proposal: 선형 보간 (near edge = MIN_SPEED, far edge = v_max or FORWARD_SPEED)
    if layer['affects_v']:
        progress = (rep_fwd - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min'])
        progress = max(0.0, min(1.0, progress))
        v_max = layer.get('v_max', FORWARD_SPEED)
        v_proposal = MIN_SPEED + (v_max - MIN_SPEED) * progress
    else:
        v_proposal = None

    # push split: 하위 5% 포인트들을 좌/우로 나누어 horiz_error 합산
    push_left  = sum(p['horiz_error'] for p in rep if p['angle'] < 0)
    push_right = sum(p['horiz_error'] for p in rep if p['angle'] > 0)

    return {
        'name': layer['name'],
        'weight': weight, 'urgency': urgency, 'v_proposal': v_proposal,
        'rep_angle': rep_angle, 'rep_horiz': rep_horiz, 'rep_fwd': rep_fwd,
        'push_left': push_left, 'push_right': push_right,
        'n_points': len(pts),
    }


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Gap 너비 계산 (코사인 법칙)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def get_gap_width(scan_points, ref_angle, ref_dist, is_left):
    """ref 기준 좌/우 첫 depth jump까지의 통과 가능 너비."""
    front = [(a, d) for a, d in scan_points if is_in_front_90(a)]

    if is_left:
        search = sorted([p for p in front if p[0] < ref_angle],
                        key=lambda x: x[0], reverse=True)
    else:
        search = sorted([p for p in front if p[0] > ref_angle],
                        key=lambda x: x[0])

    if not search:
        return 0.0

    edge_p = (ref_angle, ref_dist)
    for i, p in enumerate(search):
        if abs(p[1] - edge_p[1]) > DEPTH_JUMP_THRES:
            wall = search[i:]
            if wall:
                return min(cosine_dist(edge_p[1], wp[1], abs(edge_p[0] - wp[0]))
                           for wp in wall)
        edge_p = p

    rem_angle = abs((-90 - edge_p[0]) if is_left else (90 - edge_p[0]))
    if rem_angle > 15:
        return cosine_dist(edge_p[1], edge_p[1], rem_angle)
    return 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 측면 반발력 (50mm × 240mm 레이어)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def get_side_repulsion(scan_points):
    """
    로봇 좌우 옆면 감지 레이어 기반 반발력.

    감지 구간:
      horiz: ROBOT_HALF_WIDTH(110mm) ~ ROBOT_HALF_WIDTH + SIDE_SAFE_MARGIN(300mm)
      fwd:   -SIDE_FWD_REAR(-80mm) ~ +SIDE_FWD_LEAD(+80mm)

    반환: (delta_w, left_str, right_str)
      delta_w > 0 → 오른쪽 장애물 → 왼쪽 보정
      delta_w < 0 → 왼쪽 장애물  → 오른쪽 보정
    """
    side_inner = ROBOT_HALF_WIDTH               # 감지 시작: 로봇 끝 (110mm)
    side_outer = ROBOT_HALF_WIDTH + SIDE_SAFE_MARGIN  # 감지 끝: 110 + 190 = 300mm

    left_str  = 0.0
    right_str = 0.0

    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_wide_scan(angle_norm): continue

        horiz, fwd = decompose(angle_norm, dist)

        if fwd > SIDE_FWD_LEAD or fwd < -SIDE_FWD_REAR: continue
        # 로봇 끝(110mm) ~ 감지 경계(300mm) 구간만
        if horiz < side_inner or horiz >= side_outer: continue

        # 지수함수 반발력: 로봇 끝에 가까울수록 급격히 증가 (0~1)
        t = (horiz - side_inner) / SIDE_SAFE_MARGIN  # 0(로봇 끝) ~ 1(감지 경계)
        strength = (math.exp(SIDE_EXP_K * (1.0 - t)) - 1.0) / (math.exp(SIDE_EXP_K) - 1.0)

        if angle_norm < 0:
            left_str = max(left_str, strength)
        else:
            right_str = max(right_str, strength)

    delta_w = (right_str - left_str) * SIDE_REPULSE_GAIN

    if DEBUG_SIDE and (left_str > 0 or right_str > 0):
        print(f"  [SIDE] L={left_str:.2f} R={right_str:.2f} dw={delta_w:+.3f} "
              f"(zone {side_inner}~{side_outer}mm)")

    return delta_w, left_str, right_str


def get_side_layer_push(scan_points):
    """측방 방향 레이어: ±15°~±75°, 최대 600mm.
    장애물이 가까울수록 반대 방향으로 밀어내는 강도(0~1) 반환.
    반환: (left_push, right_push)
      left_push  > 0 → 좌측 장애물 감지 → 우측으로 유도
      right_push > 0 → 우측 장애물 감지 → 좌측으로 유도
    """
    left_push  = 0.0
    right_push = 0.0

    for angle, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > SIDE_LAYER_DIST_MAX:
            continue
        strength = (SIDE_LAYER_DIST_MAX - dist) / SIDE_LAYER_DIST_MAX  # 1=근접, 0=최대거리

        if -SIDE_LAYER_ANG_END <= angle <= -SIDE_LAYER_ANG_START:
            left_push = max(left_push, strength)
        elif SIDE_LAYER_ANG_START <= angle <= SIDE_LAYER_ANG_END:
            right_push = max(right_push, strength)

    if DEBUG_SIDE_LAYER and (left_push > 0 or right_push > 0):
        print(f"  [SIDE_LAYER] L={left_push:.2f} R={right_push:.2f} "
              f"-> score R+={SCORE_SIDE*left_push:.0f} L+={SCORE_SIDE*right_push:.0f}")

    return left_push, right_push


def get_front_passable_gaps(scan_points):
    """
    전방 ±90° 스캔에서 에지를 탐색, 통과 가능한 갭 후보 목록 반환.

    에지 기준: depth jump / angular hole / ratio jump (find_all_gaps와 동일)
    갭 통과 조건:
      - width  >= STOP_ESCAPE_MIN_GAP  (에지→반대 클러스터 선분 최단거리)
      - depth  >= FRONT_GAP_MIN_DEPTH  (갭 중심 방향 실제 스캔 거리, 없으면 800mm)
    점수: width × depth
    반환: score 내림차순 list of {center_angle, width, depth, score}
    """
    front = sorted(
        [(a, d) for a, d in scan_points
         if is_in_front_90(a) and LIDAR_MIN_VALID < d < DETECTION_RANGE],
        key=lambda p: p[0]
    )
    if len(front) < 2:
        return []

    def to_xy(a, d):
        r = math.radians(a)
        return d * math.sin(r), d * math.cos(r)

    # 에지 탐색 (연속성 단절 지점)
    edge_indices = []
    for i in range(len(front) - 1):
        a1, d1 = front[i]
        a2, d2 = front[i + 1]
        ang_diff = a2 - a1
        is_depth_jump   = abs(d2 - d1) > DEPTH_JUMP_THRES
        is_angular_hole = ang_diff >= FGM_MIN_ANG_DEG
        is_ratio_jump   = (d2 / d1 > FGM_RATIO_THRES) or (d1 / d2 > FGM_RATIO_THRES)
        if is_depth_jump or is_angular_hole or is_ratio_jump:
            edge_indices.append(i)

    if not edge_indices:
        return []

    # 클러스터 레이블링
    gap_set = set(edge_indices)
    cluster_ids = []
    cid = 0
    for i in range(len(front)):
        cluster_ids.append(cid)
        if i in gap_set:
            cid += 1

    n_clusters = cluster_ids[-1] + 1
    clusters_xy = [[] for _ in range(n_clusters)]
    for i, (a, d) in enumerate(front):
        clusters_xy[cluster_ids[i]].append(to_xy(a, d))

    def depth_at_center(center_ang):
        """갭 중심 각도에 가장 가까운 스캔 포인트 거리 (10° 이내 없으면 800mm)."""
        if not front:
            return FGM_MAX_RANGE_MM
        best = min(front, key=lambda p: abs(p[0] - center_ang))
        return best[1] if abs(best[0] - center_ang) < 10.0 else FGM_MAX_RANGE_MM

    passable = []
    for i in edge_indices:
        a1, d1 = front[i]
        a2, d2 = front[i + 1]
        x1, y1 = to_xy(a1, d1)
        x2, y2 = to_xy(a2, d2)

        cid_L = cluster_ids[i]
        cid_R = cluster_ids[i + 1]

        width = min(nearest_to_segments(x1, y1, clusters_xy[cid_R]),
                    nearest_to_segments(x2, y2, clusters_xy[cid_L]))

        if width < STOP_ESCAPE_MIN_GAP:
            continue

        center_angle = math.degrees(math.atan2((x1 + x2) / 2, (y1 + y2) / 2))
        depth = depth_at_center(center_angle)

        if depth < FRONT_GAP_MIN_DEPTH:
            continue

        passable.append({
            'center_angle': center_angle,
            'width':        width,
            'depth':        depth,
            'score':        width * depth,
        })

    return sorted(passable, key=lambda g: g['score'], reverse=True)


def choose_target_gap(passable_gaps, target_bearing, prev_heading):
    """통과 가능 갭 중 목표 방향에 가장 가깝되, 직전 방향에서 급변하지 않는 갭 선택.
    cost = 목표편차 + (직전방향편차 가중) → 비슷한 두 갭 사이 깜빡임(jitter) 억제."""
    if not passable_gaps:
        return None

    def cost(g):
        d_target = abs(((g['center_angle'] - target_bearing) + 180) % 360 - 180)
        d_prev   = abs(((g['center_angle'] - prev_heading)   + 180) % 360 - 180)
        return GAP_TARGET_WEIGHT * d_target + GAP_SMOOTH_WEIGHT * d_prev

    return min(passable_gaps, key=cost)


def is_target_blocked(scan_points, target_bearing):
    """목표 방향 ±TARGET_CLEAR_CONE° 안에 TARGET_BLOCK_DIST mm 이내 장애물이 있으면 True."""
    for a, d in scan_points:
        if LIDAR_MIN_VALID < d < TARGET_BLOCK_DIST and abs(a - target_bearing) < TARGET_CLEAR_CONE:
            return True
    return False


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [추가] 통과 불가 갭 → 가상 장애물 척력  ─ 코드 1에서 이식
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def get_narrow_gap_pushes(scan_points, layer, in_stop=False):
    """
    레이어 fwd 범위 내에서 통과 불가 갭을 탐지 → 가상 장애물 척력 반환.

    [Option A] MIN_PASSAGE_WIDTH == STOP_ESCAPE_MIN_GAP 으로 정렬되어,
    이 함수가 척력을 거는 갭(< 기준)과 get_front_passable_gaps 가 보너스를
    주는 갭(>= 기준)이 동일 기준으로 깔끔히 분리됨 (중립 구간 없음).

    [보완 사항]

    ① depth jump 방향 구분 (벽 끝 오인 방지)
       opening edge (d 증가): 장애물 오른쪽 끝
       closing edge (d 감소): 장애물 왼쪽 끝
       → opening-closing 쌍으로 매칭해야 실제 갭
       → opening만 있고 closing 없음 = 벽 끝 → 자동 스킵

    ② 수평 너비 사용 (코사인 거리 대신)
       x = d × sin(angle) → |x_closing - x_opening| = 실제 통과 가능 수평 폭

    ③ 정면 데드밴드 (±VIRTUAL_CENTER_DEADBAND)
       갭 중심이 0° 근처 → 양쪽 동등한 0.5×strength

    ④ STOP 피봇 중 비활성화
       in_stop=True → 즉시 0.0, 0.0 반환

    ⑤ 이중 반응 억제 (overlap_scale)
       갭 에지 horiz vs 레이어 horiz_th 비교
       - 양쪽 에지 모두 horiz_th 이내: scale=0.0 (완전 억제)
       - 한쪽만 horiz_th 이내: scale=0.4 (부분 억제)
       - 양쪽 모두 horiz_th 밖: scale=1.0 (억제 없음)
    """
    # ④ STOP 피봇 중 비활성화
    if in_stop:
        return 0.0, 0.0

    # 레이어 fwd 범위 내 포인트 수집 (horiz 제한 없음)
    pts = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE:
            continue
        if not is_in_front_90(angle_norm):
            continue
        _, fwd = decompose(angle_norm, dist)
        if layer['fwd_min'] <= fwd < layer['fwd_max']:
            pts.append((angle_norm, dist))

    if len(pts) < 2:
        return 0.0, 0.0

    pts_sorted = sorted(pts, key=lambda p: p[0])

    # ── ① opening / closing 에지 분리 ──────────────────────────────────────
    opening_edges = []
    closing_edges = []

    for i in range(len(pts_sorted) - 1):
        a1, d1 = pts_sorted[i]
        a2, d2 = pts_sorted[i + 1]
        if abs(d2 - d1) <= DEPTH_JUMP_THRES:
            continue
        if d2 > d1:
            # d 증가: 왼쪽 장애물의 오른쪽 끝
            opening_edges.append((a1, d1))
        else:
            # d 감소: 오른쪽 장애물의 왼쪽 끝
            closing_edges.append((a2, d2))

    if not opening_edges or not closing_edges:
        return 0.0, 0.0

    virtual_push_left  = 0.0
    virtual_push_right = 0.0

    for ao, do in opening_edges:
        # ao보다 오른쪽(큰 각도)의 closing edge 중 가장 가까운 것 = 갭 반대편
        candidates = [(ac, dc) for ac, dc in closing_edges if ac > ao]
        if not candidates:
            # 이후 closing edge 없음 = 벽 끝 → 열린 공간 → 스킵
            continue

        ac, dc = min(candidates, key=lambda x: x[0])

        # ── ② 수평 너비 계산 (x좌표 차이) ────────────────────────────────
        xo = do * math.sin(math.radians(ao))
        xc = dc * math.sin(math.radians(ac))
        gap_width = abs(xc - xo)

        if gap_width >= MIN_PASSAGE_WIDTH:
            continue

        # ── ⑤ 이중 반응 억제: overlap_scale ──────────────────────────────
        horiz_o   = abs(xo)
        horiz_c   = abs(xc)
        inside_o  = horiz_o < layer['horiz_th']
        inside_c  = horiz_c < layer['horiz_th']

        if inside_o and inside_c:
            overlap_scale = 0.0
        elif inside_o or inside_c:
            overlap_scale = 0.4
        else:
            overlap_scale = 1.0

        if overlap_scale == 0.0:
            if DEBUG_VIRTUAL:
                print(f"  [VIRTUAL/{layer['name']}] skip: 양쪽 에지 모두 horiz_th 이내 "
                      f"(ho={horiz_o:.0f} hc={horiz_c:.0f} th={layer['horiz_th']})")
            continue

        # ── 지수함수 척력 계산 ────────────────────────────────────────────
        # t: 갭 여유 비율 (0=완전 막힘, 1=간당간당 통과 경계)
        t         = gap_width / MIN_PASSAGE_WIDTH
        t         = max(0.0, min(1.0, t))   # 갭이 음수/초과 노이즈 클리핑
        exp_ratio = (math.exp(VIRTUAL_EXP_K * (1.0 - t)) - 1.0) \
                  / (math.exp(VIRTUAL_EXP_K) - 1.0)
        strength  = exp_ratio * layer['horiz_th'] * VIRTUAL_OBS_GAIN * overlap_scale

        center_angle = (ao + ac) / 2.0

        # ── ③ 정면 데드밴드: 양쪽 동등 척력 ──────────────────────────────
        if abs(center_angle) < VIRTUAL_CENTER_DEADBAND:
            half = strength * 0.5
            virtual_push_left  = max(virtual_push_left,  half)
            virtual_push_right = max(virtual_push_right, half)
            if DEBUG_VIRTUAL:
                print(f"  [VIRTUAL/{layer['name']}] CENTER "
                      f"gap={gap_width:.0f}mm t={t:.2f} exp={exp_ratio:.2f} "
                      f"center={center_angle:+.1f}° scale={overlap_scale:.1f} "
                      f"→ both={half:.0f}mm (deadband)")
        elif center_angle < 0:
            virtual_push_left  = max(virtual_push_left,  strength)
            if DEBUG_VIRTUAL:
                print(f"  [VIRTUAL/{layer['name']}] L "
                      f"gap={gap_width:.0f}mm t={t:.2f} exp={exp_ratio:.2f} "
                      f"center={center_angle:+.1f}° scale={overlap_scale:.1f} "
                      f"→ vL={strength:.0f}mm")
        else:
            virtual_push_right = max(virtual_push_right, strength)
            if DEBUG_VIRTUAL:
                print(f"  [VIRTUAL/{layer['name']}] R "
                      f"gap={gap_width:.0f}mm t={t:.2f} exp={exp_ratio:.2f} "
                      f"center={center_angle:+.1f}° scale={overlap_scale:.1f} "
                      f"→ vR={strength:.0f}mm")

    return virtual_push_left, virtual_push_right


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 계층형 v/w 산출 (메인 로직)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_layered(scan_points, heading_deg, target_bearing=0.0):
    """
    목표 방향 막힘 여부 우선 판정 후 4-way 분기:
      ① 목표 방향 열림           → 목표로 직진 (갭 무시)
      ② 막힘 + 통과 갭 존재      → 갭 중 목표 최근접으로 우회 (Gap Following)
      ③ 막힘 + 갭 없음 + 장애물  → score 기반 회피 (fallback)
      ④ 막힘 + 갭 없음 + 장애물X → 목표 유지
    안전 보정(측면 반발력)은 모든 분기에서 항상 가산.
    """
    global _last_direction, prev_desired_heading

    # ── 1. 레이어 처리 ──────────────────────────────────────────────────────
    layer_results = []
    for layer in LAYERS:
        r = process_layer(scan_points, layer)
        if r is not None:
            layer_results.append(r)

    if DEBUG_LAYERS:
        for r in layer_results:
            v_str = f" v={r['v_proposal']:.2f}" if r['v_proposal'] is not None else ""
            print(f"  [{r['name']}] n={r['n_points']:3d} "
                  f"rep:h={r['rep_horiz']:.0f} a={r['rep_angle']:+.1f}° "
                  f"f={r['rep_fwd']:.0f}  w={r['weight']:.2f} u={r['urgency']:.2f}{v_str}  "
                  f"pL={r['push_left']:.0f} pR={r['push_right']:.0f}")

    # ── 2. v 계산 (direction 무관 → 분기 앞) ───────────────────────────────
    v_layers = [r for r in layer_results if r['v_proposal'] is not None]
    if v_layers:
        total_w_v = sum(r['weight'] for r in v_layers)
        v = sum(r['weight'] * r['v_proposal'] for r in v_layers) / total_w_v
    else:
        v = FORWARD_SPEED

    # ── 3. 전방 통과 갭 탐색 + 목표 방향 막힘 판정 ─────────────────────────
    front_gaps = get_front_passable_gaps(scan_points)
    chosen_gap = choose_target_gap(front_gaps, target_bearing, prev_desired_heading)
    blocked    = is_target_blocked(scan_points, target_bearing)
    if DEBUG_BRANCH:
        print(f"[BRANCH] tb={target_bearing:+.0f} gaps={len(front_gaps)} "
              f"chosen_ca={chosen_gap['center_angle'] if chosen_gap else None} "
              f"layers={len(layer_results)} blocked={blocked}")

    # 측방 레이어 push — fallback 점수 계산 & 안전 보정 공용
    side_left_push, side_right_push = get_side_layer_push(scan_points)

    # ── 4. 4-way 분기: w 결정 ───────────────────────────────────────────────
    if not blocked:
        # ① 목표 방향이 비어있음 → 목표로 직진 (갭 무시)
        desired_heading      = target_bearing
        prev_desired_heading = desired_heading
        w = KP_GOAL * desired_heading

        # 목표 정렬도에 따라 v 조정
        # 정렬됨(0°) → FORWARD_SPEED / 많이 벗어남(≥TARGET_ALIGN_ANGLE) → MIN_SPEED
        align_factor = max(0.0, 1.0 - abs(desired_heading) / TARGET_ALIGN_ANGLE)
        v = MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * align_factor

        if DEBUG_TARGET:
            print(f"  [TARGET] clear -> head to target {target_bearing:+.1f}deg "
                  f"w={w:+.3f} v={v:.2f} align={align_factor:.2f}")

    elif chosen_gap is not None:
        # ② 목표 막힘 → 통과 갭 중 목표 최근접으로 우회 (gap-following)
        desired_heading      = chosen_gap['center_angle']
        prev_desired_heading = desired_heading
        w = -KP_GOAL * desired_heading  # ★ LiDAR center_angle ↔ heading 반대 부호 → -KP (분기① 카메라 bearing과 다름)
        if DEBUG_GAP:
            print(f"  [GAP_FOLLOW] {len(front_gaps)} gap(s) → chosen={desired_heading:+.1f}° "
                  f"target={target_bearing:+.1f}° w={w:+.3f}")

    elif layer_results:
        # ③ 막힘 + 통과 갭 없음 + 장애물 → 기존 score 기반 회피 (fallback)
        closest   = min(layer_results, key=lambda r: r['rep_horiz'])
        ref_angle = closest['rep_angle']
        ref_dist  = math.sqrt(closest['rep_horiz']**2 + closest['rep_fwd']**2)

        gap_L = get_gap_width(scan_points, ref_angle, ref_dist, is_left=True)
        gap_R = get_gap_width(scan_points, ref_angle, ref_dist, is_left=False)

        sum_pR = sum(r['weight'] * r['push_right'] for r in layer_results)
        sum_pL = sum(r['weight'] * r['push_left']  for r in layer_results)

        virt_push_L_total = 0.0
        virt_push_R_total = 0.0
        for layer in LAYERS:
            vpl, vpr = get_narrow_gap_pushes(
                scan_points, layer, in_stop=(stop_phase == 2))
            virt_push_L_total = max(virt_push_L_total, vpl)
            virt_push_R_total = max(virt_push_R_total, vpr)

        effective_push_R = max(sum_pR, virt_push_R_total)
        effective_push_L = max(sum_pL, virt_push_L_total)

        term_gap_L  = SCORE_ALPHA * gap_L
        term_gap_R  = SCORE_ALPHA * gap_R
        term_push_L = SCORE_BETA  * effective_push_R
        term_push_R = SCORE_BETA  * effective_push_L
        term_side_L = SCORE_SIDE  * side_right_push
        term_side_R = SCORE_SIDE  * side_left_push
        term_head_L = max(0.0, -heading_deg) * HEADING_WEIGHT_MM
        term_head_R = max(0.0,  heading_deg) * HEADING_WEIGHT_MM

        score_L = term_gap_L + term_push_L + term_side_L + term_head_L
        score_R = term_gap_R + term_push_R + term_side_R + term_head_R

        if DEBUG_FALLBACK:
            print(f"  [FALLBACK] no passable gap → score-based avoidance")
            print(f"  [GAP_W] L={gap_L:.0f}mm R={gap_R:.0f}mm  "
                  f"(ref={ref_angle:+.1f}°/{ref_dist:.0f}mm from {closest['name']})")
        if DEBUG_SCORE:
            print(f"  [SCORE] L={score_L:.0f}  R={score_R:.0f}")
            print(f"    gap   αL={term_gap_L:.0f} / αR={term_gap_R:.0f}")
            print(f"    push  βL={term_push_L:.0f} / βR={term_push_R:.0f}  "
                  f"[real {SCORE_BETA*sum_pR:.0f}/{SCORE_BETA*sum_pL:.0f}  "
                  f"virt {SCORE_BETA*virt_push_R_total:.0f}/{SCORE_BETA*virt_push_L_total:.0f}]")
            print(f"    side  γL={term_side_L:.0f} / γR={term_side_R:.0f}")
            print(f"    head  hL={term_head_L:.0f} / hR={term_head_R:.0f}")

        score_diff = score_L - score_R
        if _last_direction > 0:
            direction = 1.0 if score_diff > -DIRECTION_HYSTERESIS else -1.0
        else:
            direction = -1.0 if score_diff < DIRECTION_HYSTERESIS else 1.0
        if DEBUG_DIR:
            switched = "SWITCH" if direction != _last_direction else "HOLD"
            print(f"  [DIR] {'LEFT' if direction > 0 else 'RIGHT'} "
                  f"(diff={score_diff:+.0f} hyst=±{DIRECTION_HYSTERESIS:.0f} {switched})")  # noqa
        _last_direction = direction

        total_w_all = sum(r['weight'] for r in layer_results)
        w_mag = sum(r['weight'] * r['urgency'] for r in layer_results) / total_w_all
        w_mag = max(min(w_mag, MAX_W), W_MIN_DANGER)
        w = direction * w_mag

    else:
        # ④ 막힘인데 갭도 장애물도 없음(드묾) → 목표 유지
        desired_heading      = target_bearing
        prev_desired_heading = desired_heading
        w = KP_GOAL * target_bearing
        if DEBUG_CLEAR:
            print(f"  [CLEAR] no obstacles → target={target_bearing:+.1f}° w={w:+.3f}")

    # ── 5. 안전 보정 (항상 가산) ────────────────────────────────────────────
    w += (side_right_push - side_left_push) * SIDE_W_BOOST_GAIN
    side_dw, _, _ = get_side_repulsion(scan_points)
    w = max(min(w + side_dw, MAX_W), -MAX_W)

    if DEBUG_FINAL:
        print(f"  [FINAL] v={v:.2f} w={w:+.2f} target={target_bearing:+.1f}°")

    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 진입점 (STOP 우선)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _stop_reset():
    """STOP 상태 전역 변수 초기화."""
    global stop_cycle_count, stop_pivot_w, stop_phase, _last_direction
    stop_cycle_count = 0
    stop_pivot_w     = 0.0
    stop_phase       = 0
    _last_direction  = 1.0  # STOP 탈출 후 방향 히스테리시스 초기화


def _stop_set_pivot(heading_deg, target, gap_width):
    """피봇 목표 헤딩·방향 계산 및 전역 변수 세팅."""
    global stop_locked_target, stop_locked_gap, stop_locked_global_heading, stop_pivot_w
    stop_locked_target = target
    stop_locked_gap    = gap_width
    if gap_width == 0:
        stop_locked_global_heading = 0.0
        stop_pivot_w = (-math.copysign(MAX_W, heading_deg)
                        if abs(heading_deg) > 1 else -MAX_W)
    else:
        stop_locked_global_heading = ((heading_deg - target) + 180) % 360 - 180
        stop_pivot_w = -MAX_W if abs(target) < 5 else -math.copysign(MAX_W, target)


def find_vw_command(scan_points, heading_deg, target_bearing=0.0):
    """STOP zone 우선 검사 → 활성 시 피봇 탈출, 아니면 계층형 처리.

    상태 (stop_phase):
      0 = idle (정상 주행)
      2 = 피봇  : 360° 갭 방향으로 피봇, STOP 존 해제되면 즉시 layered 복귀
    """
    global stop_cycle_count, stop_pivot_w, stop_locked_target, stop_locked_gap, \
           stop_locked_global_heading, stop_phase

    # ── Phase 2: 피봇 중 ──────────────────────────────────────────────────────
    if stop_phase == 2:
        # STOP 존 해제가 헤딩 수렴보다 우선 — 방향이 조금 어긋나도 일단 주행 재개
        if not detect_stop_zone(scan_points):
            if DEBUG_STOP:
                err = abs(((heading_deg - stop_locked_global_heading) + 180) % 360 - 180)
                print(f"  [STOP] zone cleared (heading err={err:.1f}°) -> layered")
            _stop_reset()
            return find_vw_layered(scan_points, heading_deg, target_bearing)

        stop_cycle_count += 1
        if stop_cycle_count >= STOP_MAX_CYCLES:
            if DEBUG_STOP:
                print(f"  [STOP] max pivot cycles ({STOP_MAX_CYCLES}) -> force layered")
            _stop_reset()
            return find_vw_layered(scan_points, heading_deg, target_bearing)

        err   = abs(((heading_deg - stop_locked_global_heading) + 180) % 360 - 180)
        scale = min(1.0, err / STOP_PIVOT_SLOW_DEG)
        speed = STOP_PIVOT_MIN_W + (STOP_PIVOT_MAX_W - STOP_PIVOT_MIN_W) * scale
        dyn_w = math.copysign(speed, stop_pivot_w)
        if DEBUG_STOP_PIVOT:
            print(f"  [STOP] pivoting (cycle {stop_cycle_count}/{STOP_MAX_CYCLES}) "
                  f"target={stop_locked_target:+.0f}° "
                  f"(width={stop_locked_gap:.0f}mm) err={err:.1f}° w={dyn_w:+.2f}")
        return 0.0, dyn_w

    # ── Phase 0: 정상 → STOP 감지 시 즉시 피봇 ──────────────────────────────
    if detect_stop_zone(scan_points):
        target, gap_width, gap_info = find_stop_escape_direction(scan_points, heading_deg)
        _stop_set_pivot(heading_deg, target, gap_width)
        stop_cycle_count = 0
        stop_phase       = 2
        _enqueue_stop_event(heading_deg, target, gap_width, gap_info, scan_points)
        if DEBUG_STOP:
            print(f"  [STOP] triggered -> pivot target={target:+.0f}° "
                  f"global={stop_locked_global_heading:.1f}°")
        return 0.0, stop_pivot_w

    return find_vw_layered(scan_points, heading_deg, target_bearing)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 스레드: 라이다 수신 / 모터 제어
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _dedup_scan(pts):
    """1° 단위 버킷화, 중복 각도는 가장 가까운 유효 거리만 유지."""
    angle_map = {}
    for angle, dist in pts:
        if dist == 0:          # 무효 패킷이 유효 거리를 덮어쓰는 것 방지
            continue
        bucket = round(angle)
        if bucket not in angle_map or dist < angle_map[bucket]:
            angle_map[bucket] = dist
    return list(angle_map.items())


def _enqueue_stop_event(heading_deg, target, gap_width, gap_info, scan_points):
    """제어 스레드에서 호출. 절대 블로킹하지 않음.
    큐가 가득 차면(라이터가 SD 지연으로 밀림) 드롭 — 제어 루프 보호 우선."""
    global _stop_log_counter
    if not STOP_LOG_ENABLED:
        return
    _stop_log_counter += 1
    fname = f'stop_event_{int(time.time())}_{_stop_log_counter:04d}.json'
    try:
        _stop_log_queue.put_nowait(
            (fname, heading_deg, target, gap_width, gap_info, scan_points))
    except queue.Full:
        pass


def _stop_logger():
    """별도 스레드: 큐에서 꺼내 디스크에 기록. SD 지연이 제어 루프에 전파되지 않음.
    _shutdown 후엔 남은 큐를 비우고 종료."""
    while not _shutdown.is_set() or not _stop_log_queue.empty():
        try:
            item = _stop_log_queue.get(timeout=0.2)
        except queue.Empty:
            continue
        fname, heading_deg, target, gap_width, gap_info, scan_points = item
        try:
            with open(fname, 'w') as f:
                json.dump({
                    'heading':  heading_deg,
                    'target':   target,
                    'gap_dist': gap_width,
                    'gap_info': gap_info,
                    'scan':     [[a, d] for a, d in scan_points if d > 0],
                }, f)
        except Exception as e:
            print(f"[STOP_LOG] write failed: {e}")
        finally:
            _stop_log_queue.task_done()


def _lidar_reader(lidar):
    """라이다 수신 전용 스레드.
    in_waiting 만큼 일괄로 읽어 바이트 버퍼에 쌓고, 5바이트 단위로 파싱.
    패킷당 read(5) syscall(초당 5000회)을 청크 읽기로 줄여 GIL 부담↓.
    체크비트(parse_packet) 실패 시 1바이트씩 밀어 재동기(resync) →
    버퍼 오버런 등으로 정렬이 깨져도 회복."""
    buf       = bytearray()
    local_pts = []
    while not _shutdown.is_set():
        try:
            n     = lidar.in_waiting
            chunk = lidar.read(n if n > 0 else 1)   # 없으면 1바이트 블로킹(busy-spin 방지)
        except Exception:
            continue
        if not chunk:
            continue
        buf.extend(chunk)

        i     = 0
        n_buf = len(buf)
        while n_buf - i >= 5:
            pkt    = buf[i:i + 5]
            result = parse_packet(pkt)
            if result is None:
                i += 1                  # 정렬 불일치 → 1바이트 밀고 재시도 (resync)
                continue
            angle_raw, distance = result
            s_flag = pkt[0] & 0x01
            if s_flag == 1 and local_pts:           # 한 바퀴 완성
                deduped = _dedup_scan(local_pts)    # 락 밖에서 연산
                with _scan_lock:
                    _latest_scan.clear()
                    _latest_scan.extend(deduped)
                local_pts = []
            local_pts.append((
                normalize_angle(angle_raw),
                distance + LIDAR_OFFSET if distance > 0 else 0
            ))
            i += 5
        del buf[:i]                     # 소비분 제거, 남은 1~4바이트는 다음 청크와 이어붙임


def _motor_controller(arduino):
    """모터 제어 전용 스레드.
    SEND_INTERVAL마다 독립적으로 명령 송신 — 라이다 지연과 무관."""
    global prev_w, _close_target_x, _close_target_y, _close_initial_dist, _close_observe_start, \
           _search_mode, _last_arrival_x, _last_arrival_y, _last_known_mission_idx, \
           _pivot_active, _pivot_prev_hdg, _pivot_total_rotated, _pivot_direction, _last_pivot_time, \
           _last_pivot_x, _last_pivot_y, \
           _current_boundary_radius, _boundary_exit_count, _boundary_was_outside, \
           _boundary_pulling, _boundary_turn_sign, \
           _loop_cell, _loop_count, _loop_cell_time, \
           _loop_escape_until, _loop_escape_w, \
           _loop_escape_count, _loop_last_escape_x, _loop_last_escape_y, _loop_flip
    last_cmd_str = ""
    while not _shutdown.is_set():
        read_arduino(arduino)
        with _scan_lock:
            pts = [(a, d) for a, d in _latest_scan if d > 0]
        if pts:
            # ── 상태 1: DWELL / DONE → 정지 ────────────────────────────────
            if camera_tracker.is_done() or camera_tracker.is_dwelling():
                v, w = 0.0, 0.0
                prev_w = 0.0
                _close_target_x = _close_target_y = _close_initial_dist = _close_observe_start = None

            # ── 상태 2: CLOSE → 정지 관측 후 오도메트리 위치 제어 ──────────
            # _close_target_x가 이미 세팅돼 있으면 카메라 미감지 시에도 CLOSE 유지
            elif camera_tracker.is_close() or _close_target_x is not None:
                # ── 2a: 관측 단계 (CLOSE_OBSERVE_SEC 동안 정지) ─────────────
                if _close_target_x is None:
                    if _close_observe_start is None:
                        _close_observe_start = time.time()
                        print(f"[CLOSE] 관측 시작 — {CLOSE_OBSERVE_SEC:.1f}s 정지")
                    elapsed = time.time() - _close_observe_start
                    remaining = CLOSE_OBSERVE_SEC - elapsed
                    if remaining > 0:
                        v, w = 0.0, 0.0
                        prev_w = 0.0
                        print(f"[CLOSE] 관측 중 ... {remaining:.1f}s 남음")
                        # 모터 명령 전송 후 다음 사이클로
                        w_smooth = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
                        cmd = f"{v:.2f} {w_smooth:.2f}\n"
                        arduino.write(cmd.encode())
                        time.sleep(SEND_INTERVAL)
                        continue
                    # 관측 완료 → 목표 좌표 확정
                    _close_target_x, _close_target_y = _compute_close_target()
                    _close_initial_dist = None  # 첫 dist_err 계산 후 세팅

                ex = _close_target_x - arduino_x_mm
                ey = _close_target_y - arduino_y_mm
                dist_err = math.sqrt(ex ** 2 + ey ** 2)
                if _close_initial_dist is None:
                    _close_initial_dist = max(dist_err, 1.0)  # 0 나눔 방지

                if dist_err < CLOSE_ARRIVE_MM:
                    # 추정 좌표 도달 → 색지 위에 바퀴가 들어온 것으로 판정, 정지
                    # _close_target_x 유지: 다음 사이클도 재계산 없이 dist_err < threshold → 정지 유지
                    v, w = 0.0, 0.0
                    prev_w = 0.0
                    camera_tracker.signal_arrival()   # 카메라 peaked→drop이 막혀도 미션 진행
                    if DEBUG_CLOSE_DONE:
                        print(f"[CLOSE] 도달 ({dist_err:.0f}mm < {CLOSE_ARRIVE_MM}mm) → 정지")
                else:
                    target_hdg = math.degrees(math.atan2(ex, ey))
                    hdg_err    = normalize_angle(target_hdg - arduino_heading_deg)

                    w = max(min(KP_CLOSE_HDG * hdg_err, MAX_W), -MAX_W)
                    # 감속 브레이크: 목표 근접 시 SPEED_MIN까지 선형 감속 (관성 정차)
                    v_scale = min(1.0, dist_err / CLOSE_BRAKE_DIST_MM)
                    v = CLOSE_SPEED_MIN + (CLOSE_SPEED_MAX - CLOSE_SPEED_MIN) * v_scale

                    prev_w = w   # CLOSE 모드 내 스무딩 관성 제거

                    if DEBUG_CLOSE_REMAIN:
                        done_pct = (1.0 - dist_err / _close_initial_dist) * 100.0
                        bar_len  = 20
                        filled   = int(bar_len * done_pct / 100.0)
                        bar      = '█' * filled + '░' * (bar_len - filled)
                        print(f"[CLOSE_REMAIN] [{bar}] {done_pct:5.1f}%  "
                              f"remain={dist_err:.0f}mm / {_close_initial_dist:.0f}mm  "
                              f"pos=({arduino_x_mm:.0f},{arduino_y_mm:.0f})  "
                              f"tgt=({_close_target_x:.0f},{_close_target_y:.0f})")
                    if DEBUG_CLOSE_POS:
                        print(f"[CLOSE_POS] pos=({arduino_x_mm:.0f},{arduino_y_mm:.0f}) "
                              f"tgt=({_close_target_x:.0f},{_close_target_y:.0f}) "
                              f"dist={dist_err:.0f}mm  v={v:.2f}")
                    if DEBUG_CLOSE_HDG:
                        print(f"[CLOSE_HDG] arduino={arduino_heading_deg:+.1f}° "
                              f"target_hdg={target_hdg:+.1f}° "
                              f"hdg_err={hdg_err:+.1f}°  w={w:+.2f}")

            # ── 상태 3: SEEK → 카메라 bearing + 라이다 회피 (+ Mode1/Mode2 탐색) ──
            else:
                if _close_target_x is not None:
                    print(f"[STATE] CLOSE → SEEK  "
                          f"is_close={camera_tracker.is_close()}  "
                          f"is_done={camera_tracker.is_done()}  "
                          f"is_dwelling={camera_tracker.is_dwelling()}")
                _close_target_x = _close_target_y = _close_initial_dist = _close_observe_start = None
                bearing = camera_tracker.get_bearing()

                # 미션 인덱스 변화 감지 → Mode 1 진입 (색지 도착 직후)
                current_mission_idx = _get_mission_idx()
                if current_mission_idx != _last_known_mission_idx:
                    _last_arrival_x          = arduino_x_mm
                    _last_arrival_y          = arduino_y_mm
                    _last_known_mission_idx  = current_mission_idx
                    _search_mode             = 1
                    _pivot_active            = True
                    _pivot_prev_hdg          = arduino_heading_deg
                    _pivot_total_rotated     = 0.0
                    _pivot_direction         = 1.0   # CCW (도착 시 last_stable_bearing=0이므로 고정)
                    _last_pivot_time         = time.time()
                    _last_pivot_x            = arduino_x_mm   # 거리 주기 기준점 = 도착 위치
                    _last_pivot_y            = arduino_y_mm
                    _boundary_exit_count     = 0
                    _boundary_was_outside    = False
                    _boundary_pulling        = False
                    _boundary_turn_sign      = 0.0
                    _current_boundary_radius = BOUNDARY_RADIUS
                    _loop_reset()                            # 루프 추적 초기화
                    print(f"[MODE1] 탐색 시작: 도착=({arduino_x_mm:.0f},{arduino_y_mm:.0f}) "
                          f"피버턴=CCW 경계={BOUNDARY_RADIUS:.0f}mm")

                if bearing is not None:
                    # 색 감지 중 → 목표 추정 위치 갱신, 탐색 모드 해제
                    _update_target_estimate()
                    if _search_mode != 0:
                        print(f"[MODE{_search_mode}→0] 색지 재감지")
                        _loop_reset()                        # 색지 재감지 시 루프 추적 초기화
                    _search_mode             = 0
                    _pivot_active            = False
                    _boundary_exit_count     = 0
                    _boundary_was_outside    = False
                    _boundary_pulling        = False
                    _boundary_turn_sign      = 0.0
                    _current_boundary_radius = BOUNDARY_RADIUS
                    v, w = find_vw_command(pts, arduino_heading_deg, target_bearing=bearing)

                elif _search_mode == 1:
                    # ── Mode 1: 도착 후 탐색 ──────────────────────────────────
                    cx, cy = _last_arrival_x, _last_arrival_y

                    if _pivot_active:
                        # 우선순위: STOP zone / 주변 장애물 > 장애물 회피 > 피버턴
                        _sz = detect_stop_zone(pts)
                        _blockers = [(round(a), round(d)) for a, d in pts
                                     if d >= LIDAR_MIN_VALID and not is_rear_blind(a)
                                     and d <= PIVOT_CLEAR_RADIUS]
                        if _sz or _blockers:
                            # ▼ 진단용: 어느 가드가/어느 포인트가 피버턴을 막는지 (확인 후 제거)
                            print(f"[MODE1-DIAG] 피버턴 차단 stop_zone={_sz} "
                                  f"blockers(≤{PIVOT_CLEAR_RADIUS:.0f}mm)={_blockers[:6]}")
                            # 주변 장애물 → 피버턴 중단, 장애물 회피 위임
                            # (누적 회전은 갱신하지 않아 재개 시 올바르게 이어짐)
                            v, w = find_vw_command(pts, arduino_heading_deg, target_bearing=0.0)
                        else:
                            delta = normalize_angle(arduino_heading_deg - _pivot_prev_hdg)
                            _pivot_total_rotated += abs(delta)
                            _pivot_prev_hdg = arduino_heading_deg
                            if _pivot_total_rotated >= 350.0:
                                _pivot_active    = False
                                _last_pivot_time = time.time()
                                _last_pivot_x    = arduino_x_mm   # 거리 주기 기준점 갱신
                                _last_pivot_y    = arduino_y_mm
                                print(f"[MODE1] 360° 피버턴 완료 → 장애물 회피 탐색")
                                v, w = 0.0, 0.0
                            else:
                                v = 0.0
                                w = _pivot_direction * PIVOT_W_SPEED
                    else:
                        # 장애물 회피 중 + 경계 적용
                        _update_boundary_exit_tracking(cx, cy)
                        boundary_tb, v_scale = _get_boundary_correction(
                            cx, cy, _current_boundary_radius)

                        # ★ 루프(리미트 사이클) 감지/탈출 — 회피 주행 앞단 게이트
                        loop_cmd = _loop_check_and_escape(pts)
                        if loop_cmd is not None:
                            v, w = loop_cmd          # 루프 탈출 강제 (이번 사이클 점유)
                        else:
                            # 재피버턴 트리거: 마지막 피버턴 위치에서 일정 거리 이동(주)
                            #   또는 갇혀서 못 움직일 때 시간 안전망(보조)
                            if _last_pivot_x is not None:
                                moved = math.hypot(arduino_x_mm - _last_pivot_x,
                                                   arduino_y_mm - _last_pivot_y)
                            else:
                                moved = PIVOT_INTERVAL_DIST   # 기준점 없으면 즉시 재피버턴 허용
                            elapsed = time.time() - _last_pivot_time

                            if (not detect_stop_zone(pts) and
                                    (moved >= PIVOT_INTERVAL_DIST or elapsed > PIVOT_TIMEOUT_SEC)):
                                pivot_gap = find_passable_gap_for_pivot(pts)
                                if (pivot_gap is not None and
                                        is_pivot_clearance_ok(pts, PIVOT_CLEAR_RADIUS)):
                                    _pivot_active        = True
                                    _pivot_prev_hdg      = arduino_heading_deg
                                    _pivot_total_rotated = 0.0
                                    _pivot_direction     = (-1.0 if pivot_gap['center_angle'] > 0
                                                            else 1.0)
                                    trig = ("거리" if moved >= PIVOT_INTERVAL_DIST else "시간")
                                    print(f"[MODE1] 재피버턴 시작 ({trig} "
                                          f"moved={moved:.0f}mm/{PIVOT_INTERVAL_DIST:.0f}mm "
                                          f"경계={_current_boundary_radius:.0f}mm "
                                          f"갭방향={pivot_gap['center_angle']:+.1f}°)")
                                    v, w = 0.0, 0.0
                                else:
                                    v, w = find_vw_command(
                                        pts, arduino_heading_deg, target_bearing=boundary_tb)
                                    v *= v_scale
                            else:
                                v, w = find_vw_command(
                                    pts, arduino_heading_deg, target_bearing=boundary_tb)
                                v *= v_scale

                else:
                    # ── Mode 2 또는 Mode 0→2 전환 ────────────────────────────
                    if _search_mode == 0:
                        _search_mode      = 2
                        _reset_mode2_loop()                  # Mode2 루프 감지 초기화
                        _loop_reset()                        # (탈출용) 루프 추적 초기화
                        if _last_target_est_x is not None:
                            print(f"[MODE2] 탐색 시작: 목표추정="
                                  f"({_last_target_est_x:.0f},{_last_target_est_y:.0f}) "
                                  f"경계={_current_boundary_radius:.0f}mm")
                        else:
                            print(f"[MODE2] 탐색 시작: 추정 위치 없음 → 장애물 회피")

                    # Mode2 루프 감지 → Mode 1 복귀 (중심·경계 반경 그대로 유지)
                    if (_last_target_est_x is not None and _detect_mode2_loop()):
                        _search_mode         = 1
                        _last_arrival_x      = _last_target_est_x
                        _last_arrival_y      = _last_target_est_y
                        _pivot_active        = True
                        _pivot_prev_hdg      = arduino_heading_deg
                        _pivot_total_rotated = 0.0
                        _pivot_direction     = 1.0
                        _last_pivot_time     = time.time()
                        _last_pivot_x        = arduino_x_mm   # 거리 주기 기준점 초기화
                        _last_pivot_y        = arduino_y_mm
                        _reset_mode2_loop()                  # Mode2 루프 추적 초기화
                        _loop_reset()                        # 모드 전환 시 루프 추적 초기화
                        print(f"[MODE2→1] 루프 감지(같은 구역 {LOOP_REVISIT_TRIG}회 재진입) → "
                              f"중심=({_last_arrival_x:.0f},{_last_arrival_y:.0f}) "
                              f"경계={_current_boundary_radius:.0f}mm 피버턴 재시작")
                        v, w = 0.0, 0.0
                    elif _last_target_est_x is not None:
                        cx, cy = _last_target_est_x, _last_target_est_y
                        _update_boundary_exit_tracking(cx, cy)
                        boundary_tb, v_scale = _get_boundary_correction(
                            cx, cy, _current_boundary_radius)

                        # ★ 루프 감지/탈출 — Mode2 경계 복귀 주행에도 동일 적용
                        loop_cmd = _loop_check_and_escape(pts)
                        if loop_cmd is not None:
                            v, w = loop_cmd
                        else:
                            v, w = find_vw_command(
                                pts, arduino_heading_deg, target_bearing=boundary_tb)
                            v *= v_scale
                    else:
                        # 첫 번째 색지 탐색 전 (추정 위치 없음) → 일반 장애물 회피
                        v, w = find_vw_command(pts, arduino_heading_deg, target_bearing=0.0)

            w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
            prev_w = w
            cmd = f"{v:.2f} {w:.2f}\n"
            arduino.write(cmd.encode())
            if cmd != last_cmd_str and DEBUG_SEND:
                print(f"[SEND] v={v:.2f}  w={w:+.2f}  "
                      f"pos=({arduino_x_mm:.0f},{arduino_y_mm:.0f})mm  "
                      f"hdg={arduino_heading_deg:.1f}°")
                last_cmd_str = cmd
        time.sleep(SEND_INTERVAL)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    print("=== RPLIDAR Obstacle Avoidance (Layered + Front Gap + Virtual Obstacle + Search Modes + Loop Escape) ===")
    print(f"  Layers      : 6 layers (60~780mm), bottom {LAYER_PERCENTILE}% per layer")
    print(f"  L1-L3       : dynamic weight max(base, h_err/h_th*cap), affects v")
    print(f"  L4          : interp weight (0.2→0.1), affects v")
    print(f"  L5-L6       : interp weight (L5: 0.1→0.05, L6: 0.05→0.02), no v effect")
    print(f"  STOP zone   : fwd {STOP_FWD_MIN}-{STOP_FWD_MAX}mm, horiz<{STOP_HORIZ_TH}mm")
    print(f"  STOP escape : 360deg scan, min_gap={STOP_ESCAPE_MIN_GAP}mm")
    print(f"  Front gap   : bonus for passable gap (min_w={STOP_ESCAPE_MIN_GAP}mm "
          f"min_d={FRONT_GAP_MIN_DEPTH}mm)")
    print(f"  Virtual obs : MIN_PASSAGE={MIN_PASSAGE_WIDTH}mm (= STOP_ESCAPE_MIN_GAP) "
          f"GAIN={VIRTUAL_OBS_GAIN} EXP_K={VIRTUAL_EXP_K} "
          f"DEADBAND=±{VIRTUAL_CENTER_DEADBAND}°")
    print(f"  Search mode : Mode1(도착후탐색) 피버턴+경계 / Mode2(추적중소실) 경계복귀")
    print(f"  Boundary    : base={BOUNDARY_RADIUS:.0f}mm max={BOUNDARY_RADIUS_MAX:.0f}mm "
          f"expand={BOUNDARY_RADIUS_EXPAND:.0f}mm @ {BOUNDARY_EXPAND_TRIGGER}회")
    print(f"  Loop escape : cell={LOOP_CELL_MM:.0f}mm trig={LOOP_REVISIT_TRIG}회 "
          f"escape={LOOP_ESCAPE_SEC:.1f}s reflip@{LOOP_REFLIP_TRIG}회 "
          f"(same_spot<{LOOP_SAME_SPOT_MM:.0f}mm)")
    print(f"  Scoring     : alpha={SCORE_ALPHA} beta={SCORE_BETA} "
          f"(real/virtual push: max 선택)")
    print(f"  Direction   : score-based per cycle (no locking anywhere)")
    print(f"  Debug flags : LAYERS={DEBUG_LAYERS} STOP={DEBUG_STOP} "
          f"DIR={DEBUG_DIR} FINAL={DEBUG_FINAL} "
          f"SIDE={DEBUG_SIDE} VIRTUAL={DEBUG_VIRTUAL} BOUNDARY={DEBUG_BOUNDARY}")
    print("=" * 70)

    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)

    # 아두이노 헤딩 0으로 초기화 (정지 상태에서 실행할 것)
    arduino.write(b"R\n")
    time.sleep(0.1)
    print("[INIT] Arduino heading reset sent")

    lidar.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))
    lidar.read(7)

    t_lidar   = threading.Thread(target=_lidar_reader,     args=(lidar,),   daemon=True, name="lidar")
    t_motor   = threading.Thread(target=_motor_controller, args=(arduino,), daemon=True, name="motor")
    t_stoplog = threading.Thread(target=_stop_logger,      daemon=True,     name="stoplog")

    try:
        camera_tracker.start()
        t_lidar.start()
        t_motor.start()
        t_stoplog.start()
        while not _shutdown.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        _shutdown.set()
        camera_tracker.stop()
        t_lidar.join(timeout=2.0)
        t_motor.join(timeout=2.0)
        t_stoplog.join(timeout=2.0)
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        lidar.close()
        arduino.write(b"0.00 0.00\n")
        arduino.close()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()

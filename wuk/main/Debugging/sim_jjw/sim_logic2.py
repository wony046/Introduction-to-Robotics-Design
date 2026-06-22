# -*- coding: utf-8 -*-
"""
sim_logic.py  ─  Project 3 주행 로직 (하드웨어 의존 제거판)

jw_won.py 의 LiDAR 회피 알고리즘(레이어 / STOP / FGM 갭 / 측면반발 /
가상장애물 / 점수기반 회피)을 그대로 포팅한다. serial·threading·camera·
오도메트리·boundary·CLOSE 부분은 시뮬레이터(project3_sim.py)에서 처리하므로
여기서는 제외하고, 순수하게 (scan, heading, target_bearing) → (v, w) 만 계산한다.

규약 (실제 하드웨어와 동일하게 공급):
  · LiDAR angle  : 0=정면, +=물리적 우측, -=좌측
  · heading / w  : +w → heading 증가 → 물리적 좌회전(CCW). heading 양수=좌측
  · target_bearing: 양수=좌측 (heading과 동일)
  ⇒ LiDAR(우+) 와 heading/bearing(좌+) 는 반대 부호. 코드 내부에서 연결됨.

★ 슬라이더로 조절되는 상수는 apply_params() 로 런타임에 덮어쓴다.
"""

import math

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 상수 (jw_won.py 와 동일값)  ─  ★ 표시는 슬라이더 대상
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
LIDAR_MIN_VALID = 100
DETECTION_RANGE = 1500            # ★

ROBOT_HALF_WIDTH = 110

FORWARD_SPEED = 0.3               # ★
MIN_SPEED     = 0.12
MAX_W         = 1.8               # ★
W_MIN_DANGER  = 0.5
W_SMOOTH      = 0.7

LAYERS = [
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':140,
     'w_gain':2.8, 'weight_base':0.8, 'weight_cap':7.5, 'weight_dynamic':True,
     'v_max':0.2, 'affects_v':True},
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':120,
     'w_gain':2.5, 'weight_base':0.6, 'weight_cap':5.0, 'weight_dynamic':True,
     'v_max':0.25, 'affects_v':True},
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':120,
     'w_gain':2.0, 'weight_base':0.4, 'weight_cap':4.5, 'weight_dynamic':True, 'affects_v':True},
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':110,
     'w_gain':1.0, 'weight_base':0.1, 'weight_start':0.2, 'weight_dynamic':False, 'affects_v':True},
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':110,
     'w_gain':0.4, 'weight_base':0.05,'weight_start':0.1, 'weight_dynamic':False, 'affects_v':False},
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':110,
     'w_gain':0.3, 'weight_base':0.02,'weight_start':0.05,'weight_dynamic':False, 'affects_v':False},
]
LAYER_PERCENTILE = 5

STOP_FWD_MIN  = 100               # ★
STOP_FWD_MAX  = 175               # ★
STOP_HORIZ_TH = 105               # ★
STOP_MIN_POINTS = 2               # 이 수 이상 포인트가 있어야 STOP 인정 (단일 노이즈 점 무시)

PIVOT_CLEAR_RADIUS     = STOP_FWD_MAX   # mm: 피버턴 클리어런스 검사 반경 (= STOP_FWD_MAX)
PIVOT_CLEAR_MIN_POINTS = 2              # 이 수 이상 포인트가 반경 내일 때만 '막힘' (단일 노이즈 점 무시)
REAR_BLIND_HALF        = 23.0           # deg: 후방 카메라 구조물 차폐 반각 (총 46° 사각)

STOP_ESCAPE_MIN_GAP = ROBOT_HALF_WIDTH * 2 + 40   # ★ (=260)
STOP_MAX_CYCLES     = 30
STOP_PIVOT_MAX_W    = 0.9
STOP_PIVOT_MIN_W    = 0.7
STOP_PIVOT_SLOW_DEG = 15

FGM_MIN_ANG_DEG  = 3
FGM_MIN_DEPTH_MM = 250
FGM_MAX_RANGE_MM = 500
FGM_RATIO_THRES  = 1.2

FRONT_GAP_MIN_DEPTH = 300

SCORE_ALPHA       = 5.0           # ★
SCORE_BETA        = 8             # ★
SCORE_SIDE        = 2500.0
HEADING_WEIGHT_MM = 5.0
DEPTH_JUMP_THRES  = 120

DIRECTION_HYSTERESIS = 300.0

GAP_TARGET_WEIGHT = 1.0
GAP_SMOOTH_WEIGHT = 0.3
KP_GOAL           = MAX_W / 45.0  # ★ (시뮬에서는 독립 슬라이더)
COLOR_CONFIRM_SEC  = 0.2          # sec: Mode1/2→Mode0 전환 디바운스 (색 연속 감지 시간)
COLOR_CONFIRM_JUMP_DEG = 12.0     # deg: 확정 중 bearing이 직전 대비 이 각도 초과로 튀면 확정 타이머 재시작
TARGET_ALIGN_ANGLE = 50.0         # deg: 이 각도 이상이면 v=MIN_SPEED (60→50, Mode2 정렬 우선)
TARGET_CLEAR_CONE  = 18
TARGET_BLOCK_DIST  = 600
TARGET_UNBLOCK_RATIO = 1.25       # 막힘 히스테리시스: 일단 막히면 이 배(750mm)까지 비워져야 해제
# ── 정면 진행 통로 가드 (Mode1/2 전용, Mode0는 비활성) ──
FRONT_CORRIDOR_HALF = ROBOT_HALF_WIDTH + 40   # mm: 정면 통로 반폭 (몸체 110 + 마진 40)
FRONT_CORRIDOR_DIST = 500                     # mm: 정면 통로 검사 깊이
GOAL_BIAS_WEIGHT  = 8.0           # mm/deg: 분기③(갭 없는 회피)에서 목표 방향 쪽 score 가산
GOAL_BIAS_MAX     = 250.0         # mm: goal bias 상한 (tie-breaker 전용, 회피 항을 못 이기게)

SCAN_WIDE_HALF = 135

SIDE_SAFE_MARGIN  = 300
SIDE_FWD_LEAD     = 90
SIDE_FWD_REAR     = 90
SIDE_REPULSE_GAIN = 1.25          # ★
SIDE_EXP_K        = 2.0

SIDE_LAYER_ANG_START = 15
SIDE_LAYER_ANG_END   = 75
SIDE_LAYER_DIST_MAX  = 700
SIDE_W_BOOST_GAIN    = 1.5

MIN_PASSAGE_WIDTH       = STOP_ESCAPE_MIN_GAP
VIRTUAL_OBS_GAIN        = 1.5
VIRTUAL_CENTER_DEADBAND = 10
VIRTUAL_EXP_K           = 2.5

# ── 가상 경계 & 탐색 모드 (새 jw_won.py 기준) ──────────────────────────────
# Mode 1 (도착후탐색): 중심=이전 색지 도착 위치, 360° 피버턴 후 회피+재피버턴
# Mode 2 (추적중소실): 중심=마지막 감지 목표 추정 위치, 경계 복귀+회피
# 경계 이탈→복귀 누적 시 반경 확장.
BOUNDARY_RADIUS         = 1000.0      # ★ 기본 경계 반경
BOUNDARY_RADIUS_MAX     = 2000.0      # 최대 경계 반경
BOUNDARY_RADIUS_EXPAND  = 500.0       # 1회 확장 폭
BOUNDARY_EXPAND_TRIGGER = 2           # 이탈→복귀 N회 누적 시 확장
BOUNDARY_HYSTERESIS_MM  = 150.0       # 진입/이탈 히스테리시스
BOUNDARY_BLEND_DIST     = 300.0
BOUNDARY_V_MIN          = 0.5
PIVOT_W_SPEED           = 0.9         # 탐색 피버턴 회전 속도 (rad/s). 0.6→0.9 (저전압·마찰 시 토크 부족 정지 방지)
PIVOT_INTERVAL_SEC      = 5.0         # 주기 기반 재피버턴 주기 (sec, 개활지에서도 주기적 360°)
MODE2_TIMEOUT_SEC       = 40.0        # sec: Mode2에서 이 시간 초과 시 Mode1으로 복귀
MODE2_ARRIVE_SEARCH_SEC = 2.0         # sec: Mode2 standoff 도달·정지 후 이 시간 내 색지 미발견 시
                                      #      전체 타임아웃 안 기다리고 즉시 Mode1 피버턴 전환
MODE2_NEAR_TARGET_MM    = 400         # mm: 목표 추정 위치 이 거리 이내면 정지(능동 인력 OFF).
                                      #      ★ 색지 위까지(과거 120) 파고들면 카메라 근거리 사각에
                                      #        색지가 빠져 재포착 실패 → 카메라가 잘 보는 standoff(~400)에서
                                      #        멈춰 재포착 기회를 준다. (sim CAM_NEAR_MM=250 < 400)

# Mode 1 재피버턴 — 갭 유무/시야 변화 기반 트리거
MAX_PIVOT_GAP_WIDTH = 650    # mm: 이보다 넓은 갭은 통로가 아닌 개활지로 간주 → 피버턴 안 함
PIVOT_EDGE_NEAR_MM  = 600    # mm: 갭 양쪽 에지가 모두 이 거리 이내여야 실제 통로(개활지 배제)
NEW_GAP_DIST_MM     = 400    # mm: 기존 검사 갭들과 이만큼 떨어지면 '새 갭'으로 판정
PIVOT_MOVE_TRIGGER  = 600    # mm: 마지막 피버턴 이후 이만큼 이동 → 같은 갭이라도 시야 변화로 재피버턴
GAP_MEMORY_MERGE_MM = 300    # mm: 갭 글로벌 위치가 이 안이면 같은 갭으로 보고 중복 등록 안 함
PIVOT_COOLDOWN_SEC      = 4.0  # sec: 피버턴 완료 후 이 시간 동안은 재피버턴 금지
NEW_GAP_PERSIST_FRAMES  = 3    # 트리거 조건이 이 프레임 수 연속 유지될 때만 재피버턴 (디바운스)

KP_CLOSE_HDG        = 0.1         # ★
CLOSE_SPEED_MAX     = 0.2         # ★
CLOSE_ARRIVE_MM     = 30          # ★
CLOSE_OBSERVE_SEC   = 1.0
CLOSE_ENTER_MM      = 400.0       # ★
CLOSE_STANDOFF_MM   = 20          # 색지 추정 위치보다 이만큼 '덜' 접근해 정지 (0=색지 위까지)

# 디버그 (시뮬에서는 전부 OFF; print 호출만 무력화)
DEBUG_LAYERS = DEBUG_STOP = DEBUG_STOP_PIVOT = DEBUG_BRANCH = 0
DEBUG_TARGET = DEBUG_GAP = DEBUG_FALLBACK = DEBUG_SCORE = 0
DEBUG_DIR = DEBUG_CLEAR = DEBUG_FINAL = DEBUG_SIDE = 0
DEBUG_SIDE_LAYER = DEBUG_VIRTUAL = 0

# ── 회피 상태 (jw_won.py 전역과 동일 역할) ──────────────────────────────
prev_desired_heading = 0.0
_last_direction      = 1.0
_target_block_latch  = False       # is_target_blocked 히스테리시스 상태 (막힘 래치)
stop_cycle_count           = 0
stop_pivot_w               = 0.0
stop_locked_target         = 0.0
stop_locked_gap            = 0.0
stop_locked_global_heading = 0.0
stop_phase                 = 0     # 0=idle, 2=pivot

# ── 오도메트리 / 경계 상태 (jw_won.py 전역과 동일 역할) ──────────────────────
#   주의: 이 프레임은 (좌+X, 전방+Y, 좌회전+) 비표준 좌표계.
#   시뮬은 표준 월드(우+X)를 쓰므로, project3_sim 에서 매 스텝
#     arduino_x_mm = -robot_x_world,  arduino_y_mm = robot_y_world,
#     arduino_heading_deg = robot_h
#   로 미러링해서 세팅해야 boundary/CLOSE 함수가 올바르게 동작한다.
arduino_x_mm        = 0.0
arduino_y_mm        = 0.0
arduino_heading_deg = 0.0

# ── 탐색 모드 & 가변 경계 상태 (새 jw_won.py 전역과 동일 역할) ──────────────
_search_mode             = 0      # 0=정상, 1=도착후탐색, 2=추적중소실
_last_arrival_x          = None   # Mode1 경계 중심 (이전 색지 도착 위치)
_last_arrival_y          = None
_last_target_est_x       = None   # Mode2 경계 중심 (마지막 감지 목표 추정)
_last_target_est_y       = None
_last_known_mission_idx  = 0      # 미션 인덱스 변화 감지용
_color_confirm_start     = None   # 색 연속 감지 시작 시각 (Mode1/2→Mode0 디바운스; None=미감지)
_color_confirm_ref       = 0.0    # 확정 중 직전 bearing 기준값 (튀는 값 점프 감지용)
_pivot_active            = False  # 피버턴 진행 중
_pivot_prev_hdg          = 0.0    # 이전 사이클 헤딩 (누적 회전 계산)
_pivot_total_rotated     = 0.0    # 누적 회전량 (deg)
_pivot_direction         = 1.0    # +1=CCW, -1=CW
_last_pivot_time         = 0.0    # 마지막 피버턴 완료 시각 (sim time)
_mode2_start_time        = None   # Mode2 시작 시각 (타임아웃 계산용, sim time)
_mode2_arrived_time      = None   # Mode2 standoff 정지 시작 시각 (도착 후 색지 미발견 조기 전환용)
_inspected_gaps          = []     # 이미 피버턴으로 들여다본 갭들의 글로벌 (x_mm, y_mm)
_last_pivot_robot_x      = None   # mm: 마지막 피버턴 시점 로봇 위치 x (시야 변화 판정용)
_last_pivot_robot_y      = None   # mm: 마지막 피버턴 시점 로봇 위치 y
_new_gap_streak          = 0      # 재피버턴 트리거 조건 연속 충족 프레임 수 (디바운스)
_current_boundary_radius = BOUNDARY_RADIUS  # 현재 적용 경계 반경
_boundary_exit_count     = 0      # 경계 이탈→복귀 누적
_boundary_was_outside    = False  # 히스테리시스: 현재 경계 외부 여부

# ── 초기 탐색(첫 빨강 색지 발견 전) 상태 (jw_won.py 전역과 동일 역할) ──────────
#   시작하자마자 Mode1(피버턴 포함) 탐색을 돌리되 경계를 '정면 반원'으로 잡는다.
_initial_x               = 0.0     # mm: 프로그램 시작 위치 x (정면 반원 중심, arduino 프레임)
_initial_y               = 0.0     # mm: 프로그램 시작 위치 y
_initial_heading         = 0.0     # deg: 시작 헤딩 (정면 half-plane 정의)
_initial_pose_set        = False   # 시작 포즈 캡처 완료 여부
_use_semicircle_boundary = False   # True: 정면 반원 경계 사용 (초기 탐색 전용)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 파라미터 적용 / 상태 리셋
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 슬라이더 → 모듈 전역 매핑 (이름이 곧 전역변수명)
SLIDER_PARAMS = [
    'FORWARD_SPEED', 'CLOSE_SPEED_MAX', 'MAX_W', 'KP_GOAL', 'KP_CLOSE_HDG',
    'STOP_FWD_MIN', 'STOP_FWD_MAX', 'STOP_HORIZ_TH', 'STOP_ESCAPE_MIN_GAP',
    'DETECTION_RANGE', 'BOUNDARY_RADIUS', 'CLOSE_ENTER_MM', 'CLOSE_ARRIVE_MM',
]

def apply_params(values: dict):
    """슬라이더 값 dict 를 모듈 전역에 반영. 파생값(MIN_PASSAGE_WIDTH)도 동기화."""
    g = globals()
    for k, v in values.items():
        if k in g:
            g[k] = v
    # STOP_ESCAPE_MIN_GAP 이 바뀌면 MIN_PASSAGE_WIDTH 도 함께 (Option A 정합성)
    g['MIN_PASSAGE_WIDTH'] = g['STOP_ESCAPE_MIN_GAP']


def reset_state():
    """주행(STOP/방향 히스테리시스) 상태 초기화. 재시작 시 호출."""
    global prev_desired_heading, _last_direction, _target_block_latch
    global stop_cycle_count, stop_pivot_w, stop_locked_target
    global stop_locked_gap, stop_locked_global_heading, stop_phase
    prev_desired_heading = 0.0
    _last_direction      = 1.0
    _target_block_latch  = False
    stop_cycle_count = 0
    stop_pivot_w = 0.0
    stop_locked_target = 0.0
    stop_locked_gap = 0.0
    stop_locked_global_heading = 0.0
    stop_phase = 0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티 (jw_won.py 복사)
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
    단일 노이즈 점으로 피버턴이 중단되지 않도록 PIVOT_CLEAR_MIN_POINTS 이상일 때만 막힘."""
    count = 0
    for angle, dist in scan_points:
        if dist < LIDAR_MIN_VALID:
            continue
        if is_rear_blind(angle):
            continue
        if dist <= radius:
            count += 1
            if count >= PIVOT_CLEAR_MIN_POINTS:
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
    dx, dy = bx - ax, by - ay
    seg_sq = dx*dx + dy*dy
    if seg_sq == 0:
        return math.sqrt((px - ax)**2 + (py - ay)**2)
    t = max(0.0, min(1.0, ((px - ax)*dx + (py - ay)*dy) / seg_sq))
    return math.sqrt((px - ax - t*dx)**2 + (py - ay - t*dy)**2)

def nearest_to_segments(px, py, cluster_xy):
    if len(cluster_xy) == 1:
        ox, oy = cluster_xy[0]
        return math.sqrt((px - ox)**2 + (py - oy)**2)
    return min(
        point_to_segment_dist(px, py,
                              cluster_xy[j][0], cluster_xy[j][1],
                              cluster_xy[j+1][0], cluster_xy[j+1][1])
        for j in range(len(cluster_xy) - 1)
    )


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP zone 감지 & FGM 탈출
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def detect_stop_zone(scan_points):
    """STOP rectangle (fwd 100~175mm, horiz<105mm) 안에 장애물이 있는가?
    단일 노이즈 점으로 STOP이 오발동하지 않도록 STOP_MIN_POINTS 이상일 때만 True."""
    count = 0
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_front_90(angle_norm): continue
        horiz, fwd = decompose(angle_norm, dist)
        if STOP_FWD_MIN <= fwd <= STOP_FWD_MAX and horiz < STOP_HORIZ_TH:
            count += 1
            if count >= STOP_MIN_POINTS:
                return True
    return False


def find_all_gaps(scan_points):
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

    gap_set = set(gap_indices)
    cluster_ids = []
    cid = 0
    for i in range(len(pts)):
        cluster_ids.append(cid)
        if i in gap_set:
            cid += 1

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
        d_LR = nearest_to_segments(x1, y1, clusters_xy[cid_R])
        d_RL = nearest_to_segments(x2, y2, clusters_xy[cid_L])
        width = min(d_LR, d_RL)
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
    passable = [g for g in gaps
                if g['width'] >= STOP_ESCAPE_MIN_GAP
                and g['depth'] >= FGM_MIN_DEPTH_MM]
    if passable:
        return min(passable,
                   key=lambda g: abs(((g['center_angle'] - prefer_angle) + 180) % 360 - 180))
    return None


def find_free_sectors(scan_points, block_dist):
    """1° 빈 360개 점유 배열 → 원형 순회로 연속 free 구간 추출.
    free = block_dist 이내에 유효 포인트가 없는 각도 구간(=깊이 자동 충족).
    반환: list of {center_angle, width, ang_width, R}."""
    blocked  = [False] * 360
    shoulder = [block_dist] * 360
    for a, d in scan_points:
        if LIDAR_MIN_VALID < d < block_dist:
            i = int(round(a)) % 360
            blocked[i]  = True
            shoulder[i] = min(shoulder[i], d)

    if not any(blocked):
        return [{'center_angle': 0.0, 'width': 9999.0, 'ang_width': 360.0, 'R': block_dist}]

    start   = blocked.index(True)
    sectors = []
    k = 0
    while k < 360:
        if blocked[(start + k) % 360]:
            k += 1
            continue
        run_begin = k
        while k < 360 and not blocked[(start + k) % 360]:
            k += 1
        run_end   = k
        ang_width = run_end - run_begin
        left_sh   = shoulder[(start + run_begin - 1) % 360]
        right_sh  = shoulder[(start + run_end) % 360]
        R         = min(left_sh, right_sh, block_dist)
        width_mm  = 2.0 * R * math.sin(math.radians(min(ang_width, 180)) / 2.0)
        center    = normalize_angle(start + run_begin + ang_width / 2.0)
        sectors.append({'center_angle': center, 'width': width_mm,
                        'ang_width': ang_width, 'R': R})
    return sectors


def choose_escape_sector(sectors, prefer_angle=0.0):
    """폭 >= STOP_ESCAPE_MIN_GAP 섹터 중 prefer_angle 최근접 선택. 없으면 None."""
    passable = [s for s in sectors if s['width'] >= STOP_ESCAPE_MIN_GAP]
    if passable:
        return min(passable,
                   key=lambda s: abs(((s['center_angle'] - prefer_angle) + 180) % 360 - 180))
    return None


def find_stop_escape_direction(scan_points, heading_deg=0.0):
    """STOP 탈출 방향 결정. 반환: (target_angle, gap_width, info_list, method)
    1차 FGM(에지 기반). 통과 갭 0개면 2차 빈 섹터(각도 기반)로 폴백."""
    # ── 1차: FGM (에지 기반) ──
    gaps   = find_all_gaps(scan_points)
    chosen = choose_escape_gap(gaps, prefer_angle=heading_deg)
    method = 'FGM'

    # ── 2차: 통과 갭 0개 → 빈 섹터 탐색 ──
    if chosen is None:
        sectors = find_free_sectors(scan_points, FGM_MAX_RANGE_MM)
        chosen  = choose_escape_sector(sectors, prefer_angle=heading_deg)
        method  = 'SECTOR'

    if chosen is None:
        return 0.0, 0.0, [], 'NONE'

    if method == 'FGM':
        info = [
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
    else:
        info = [
            {
                'width':        s['width'],
                'center_angle': s['center_angle'],
                'ang_width':    s['ang_width'],
                'R':            s['R'],
                'depth':        s['R'],   # 렌더러 호환 (gap_info 그리기에서 depth 사용)
                'passable':     s['width'] >= STOP_ESCAPE_MIN_GAP,
                'chosen':       s is chosen,
            }
            for s in sectors
        ]

    return float(chosen['center_angle']), float(chosen['width']), info, method


def find_passable_gap_for_pivot(scan_points):
    """Mode 1 재피버턴 전용: 통과 가능하고 후방 사각이 아닌 갭 중 가장 넓은 것 반환.
    [개활지 배제] width 상한(MAX_PIVOT_GAP_WIDTH) + 양쪽 에지 근거리(PIVOT_EDGE_NEAR_MM)로
    실제 장애물에 둘러싸인 통로만 남긴다. 없으면 None."""
    passable = [g for g in find_all_gaps(scan_points)
                if STOP_ESCAPE_MIN_GAP <= g['width'] <= MAX_PIVOT_GAP_WIDTH
                and g['depth'] >= FGM_MIN_DEPTH_MM
                and g['edge_a'][1] <= PIVOT_EDGE_NEAR_MM
                and g['edge_b'][1] <= PIVOT_EDGE_NEAR_MM
                and not is_rear_blind(g['center_angle'])]
    if not passable:
        return None
    return max(passable, key=lambda g: g['width'])


def _gap_global_pos(gap):
    """갭 입구의 글로벌 좌표 (x_mm, y_mm) 추정 (오도메트리 기반)."""
    r   = (gap['edge_a'][1] + gap['edge_b'][1]) / 2.0
    ang = math.radians(arduino_heading_deg + gap['center_angle'])
    return arduino_x_mm + r * math.sin(ang), arduino_y_mm + r * math.cos(ang)


def _should_pivot_for_gap(scan_points, now):
    """Mode 1 재피버턴 트리거. 실제 통로 갭(개활지 제외)이 존재하고,
      ① 기존에 들여다보지 않은 '새 갭'이거나(NEW_GAP_DIST_MM),
      ② 마지막 피버턴 이후 충분히 이동(PIVOT_MOVE_TRIGGER)해 시야가 바뀐 경우
    에만 (gap, gx, gy)를 반환. 아니면 None.
    쿨다운(PIVOT_COOLDOWN_SEC) + 디바운스(NEW_GAP_PERSIST_FRAMES)로 오발동 억제.
    now = 시뮬 시각 (jw_won 의 time.time() 대체)."""
    global _new_gap_streak

    # 쿨다운: 피버턴 완료 직후 재트리거 금지
    if now - _last_pivot_time < PIVOT_COOLDOWN_SEC:
        _new_gap_streak = 0
        return None

    gap = find_passable_gap_for_pivot(scan_points)
    if gap is None:
        _new_gap_streak = 0
        return None

    gx, gy = _gap_global_pos(gap)
    is_new = all(math.hypot(gx - ix, gy - iy) > NEW_GAP_DIST_MM
                 for ix, iy in _inspected_gaps)
    moved  = (_last_pivot_robot_x is None or
              math.hypot(arduino_x_mm - _last_pivot_robot_x,
                         arduino_y_mm - _last_pivot_robot_y) > PIVOT_MOVE_TRIGGER)

    if not (is_new or moved):
        _new_gap_streak = 0
        return None

    # 디바운스: 연속 프레임 유지 확인
    _new_gap_streak += 1
    if _new_gap_streak < NEW_GAP_PERSIST_FRAMES:
        return None
    _new_gap_streak = 0
    return (gap, gx, gy)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 레이어 처리
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def process_layer(scan_points, layer):
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

    n_take = max(1, int(len(pts) * LAYER_PERCENTILE / 100))
    rep = sorted(pts, key=lambda p: p['dist'])[:n_take]
    rep_angle = sum(p['angle'] for p in rep) / len(rep)
    rep_horiz = sum(p['horiz'] for p in rep) / len(rep)
    rep_fwd   = sum(p['fwd']   for p in rep) / len(rep)
    rep_h_err = layer['horiz_th'] - rep_horiz

    if layer['weight_dynamic']:
        cap = layer.get('weight_cap', 1.0)
        raw = rep_h_err / layer['horiz_th'] * cap
        weight = max(layer['weight_base'], min(cap, raw))
    else:
        progress = (rep_fwd - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min'])
        progress = max(0.0, min(1.0, progress))
        weight = layer['weight_start'] + (layer['weight_base'] - layer['weight_start']) * progress

    urgency = layer['w_gain'] * rep_h_err / layer['horiz_th']

    if layer['affects_v']:
        progress = (rep_fwd - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min'])
        progress = max(0.0, min(1.0, progress))
        v_max = layer.get('v_max', FORWARD_SPEED)
        v_proposal = MIN_SPEED + (v_max - MIN_SPEED) * progress
    else:
        v_proposal = None

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
# Gap 너비 (코사인 법칙) — fallback 회피용
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def get_gap_width(scan_points, ref_angle, ref_dist, is_left):
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
# 측면 반발력 / 측방 레이어
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def get_side_repulsion(scan_points):
    side_inner = ROBOT_HALF_WIDTH
    side_outer = ROBOT_HALF_WIDTH + SIDE_SAFE_MARGIN
    left_str  = 0.0
    right_str = 0.0
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_wide_scan(angle_norm): continue
        horiz, fwd = decompose(angle_norm, dist)
        if fwd > SIDE_FWD_LEAD or fwd < -SIDE_FWD_REAR: continue
        if horiz < side_inner or horiz >= side_outer: continue
        t = (horiz - side_inner) / SIDE_SAFE_MARGIN
        strength = (math.exp(SIDE_EXP_K * (1.0 - t)) - 1.0) / (math.exp(SIDE_EXP_K) - 1.0)
        if angle_norm < 0:
            left_str = max(left_str, strength)
        else:
            right_str = max(right_str, strength)
    delta_w = (right_str - left_str) * SIDE_REPULSE_GAIN
    return delta_w, left_str, right_str


def get_side_layer_push(scan_points):
    left_push  = 0.0
    right_push = 0.0
    for angle, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > SIDE_LAYER_DIST_MAX:
            continue
        strength = (SIDE_LAYER_DIST_MAX - dist) / SIDE_LAYER_DIST_MAX
        if -SIDE_LAYER_ANG_END <= angle <= -SIDE_LAYER_ANG_START:
            left_push = max(left_push, strength)
        elif SIDE_LAYER_ANG_START <= angle <= SIDE_LAYER_ANG_END:
            right_push = max(right_push, strength)
    return left_push, right_push


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 전방 통과 갭 (gap-following)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def get_front_passable_gaps(scan_points):
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
    if not passable_gaps:
        return None
    # g['center_angle']는 라이다 규약(우측=+), target_bearing은 카메라 규약(우측=−).
    # 라이다 각도와 비교하려면 부호를 뒤집어 같은 방향을 가리키게 한다 (tb=0이면 영향 없음).
    lidar_tb = -target_bearing
    def cost(g):
        d_target = abs(((g['center_angle'] - lidar_tb)    + 180) % 360 - 180)
        d_prev   = abs(((g['center_angle'] - prev_heading) + 180) % 360 - 180)
        return GAP_TARGET_WEIGHT * d_target + GAP_SMOOTH_WEIGHT * d_prev
    return min(passable_gaps, key=cost)


def is_target_blocked(scan_points, target_bearing):
    """목표 방향 ±TARGET_CLEAR_CONE° 안에 장애물이 있으면 True (히스테리시스 적용).
    진입: TARGET_BLOCK_DIST 이내. 해제: 그 TARGET_UNBLOCK_RATIO배까지 비워져야.
    추가: Mode1/2 전용 '정면 진행 통로 가드' — 목표 방향과 무관하게 정면 통로가 막히면 True."""
    global _target_block_latch
    # target_bearing은 카메라 규약(우측=−), 라이다 각도 a는 우측=+로 반대.
    # 라이다와 같은 방향을 가리키도록 부호를 뒤집어 비교 (tb=0이면 영향 없음).
    lidar_tb = -target_bearing
    thresh = TARGET_BLOCK_DIST * (TARGET_UNBLOCK_RATIO if _target_block_latch else 1.0)
    cone_blocked = any(
        LIDAR_MIN_VALID < d < thresh
        and abs(((a - lidar_tb) + 180) % 360 - 180) < TARGET_CLEAR_CONE
        for a, d in scan_points)
    _target_block_latch = cone_blocked

    # ── 정면 진행 통로 가드 (Mode 1/2 전용, Mode 0 비활성) ──
    corridor_blocked = False
    if _search_mode != 0:
        for a, d in scan_points:
            if not (LIDAR_MIN_VALID < d < FRONT_CORRIDOR_DIST):
                continue
            if not is_in_front_90(a):
                continue
            horiz, fwd = decompose(a, d)
            if fwd > 0 and horiz < FRONT_CORRIDOR_HALF:
                corridor_blocked = True
                break

    return cone_blocked or corridor_blocked


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 통과 불가 갭 → 가상 장애물 척력
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def get_narrow_gap_pushes(scan_points, layer, in_stop=False):
    if in_stop:
        return 0.0, 0.0
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
    opening_edges = []
    closing_edges = []
    for i in range(len(pts_sorted) - 1):
        a1, d1 = pts_sorted[i]
        a2, d2 = pts_sorted[i + 1]
        if abs(d2 - d1) <= DEPTH_JUMP_THRES:
            continue
        if d2 > d1:
            opening_edges.append((a1, d1))
        else:
            closing_edges.append((a2, d2))
    if not opening_edges or not closing_edges:
        return 0.0, 0.0

    virtual_push_left  = 0.0
    virtual_push_right = 0.0
    for ao, do in opening_edges:
        candidates = [(ac, dc) for ac, dc in closing_edges if ac > ao]
        if not candidates:
            continue
        ac, dc = min(candidates, key=lambda x: x[0])
        xo = do * math.sin(math.radians(ao))
        xc = dc * math.sin(math.radians(ac))
        gap_width = abs(xc - xo)
        if gap_width >= MIN_PASSAGE_WIDTH:
            continue
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
            continue
        t         = gap_width / MIN_PASSAGE_WIDTH
        t         = max(0.0, min(1.0, t))
        exp_ratio = (math.exp(VIRTUAL_EXP_K * (1.0 - t)) - 1.0) \
                  / (math.exp(VIRTUAL_EXP_K) - 1.0)
        strength  = exp_ratio * layer['horiz_th'] * VIRTUAL_OBS_GAIN * overlap_scale
        center_angle = (ao + ac) / 2.0
        if abs(center_angle) < VIRTUAL_CENTER_DEADBAND:
            half = strength * 0.5
            virtual_push_left  = max(virtual_push_left,  half)
            virtual_push_right = max(virtual_push_right, half)
        elif center_angle < 0:
            virtual_push_left  = max(virtual_push_left,  strength)
        else:
            virtual_push_right = max(virtual_push_right, strength)
    return virtual_push_left, virtual_push_right


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 계층형 v/w 산출
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def find_vw_layered(scan_points, heading_deg, target_bearing=0.0):
    global _last_direction, prev_desired_heading

    layer_results = []
    for layer in LAYERS:
        r = process_layer(scan_points, layer)
        if r is not None:
            layer_results.append(r)

    v_layers = [r for r in layer_results if r['v_proposal'] is not None]
    if v_layers:
        total_w_v = sum(r['weight'] for r in v_layers)
        v = sum(r['weight'] * r['v_proposal'] for r in v_layers) / total_w_v
    else:
        v = FORWARD_SPEED

    front_gaps = get_front_passable_gaps(scan_points)
    chosen_gap = choose_target_gap(front_gaps, target_bearing, prev_desired_heading)
    blocked    = is_target_blocked(scan_points, target_bearing)

    side_left_push, side_right_push = get_side_layer_push(scan_points)

    if not blocked:
        desired_heading      = target_bearing
        prev_desired_heading = desired_heading
        w = KP_GOAL * desired_heading
        align_factor = max(0.0, 1.0 - abs(desired_heading) / TARGET_ALIGN_ANGLE)
        v = MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * align_factor

    elif chosen_gap is not None:
        desired_heading      = chosen_gap['center_angle']
        prev_desired_heading = desired_heading
        # ★ LiDAR center_angle(우+) ↔ heading(좌+) 반대 부호 → -KP (분기① bearing과 다름)
        w = -KP_GOAL * desired_heading

    elif layer_results:
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
        # 목표 방향 bias: 갭이 없어 회피만 할 때 '동점 깨기'로만 목표 쪽 편향.
        # GOAL_BIAS_MAX로 상한 → side/push 회피 항을 절대 override 못 함(돌진 방지).
        # ★ 분기①(w=+KP*target_bearing)과 같은 쪽: target_bearing>0 → score_L(+측).
        goal_mag    = min(abs(target_bearing) * GOAL_BIAS_WEIGHT, GOAL_BIAS_MAX)
        term_goal_L = goal_mag if target_bearing > 0 else 0.0
        term_goal_R = goal_mag if target_bearing < 0 else 0.0

        score_L = term_gap_L + term_push_L + term_side_L + term_head_L + term_goal_L
        score_R = term_gap_R + term_push_R + term_side_R + term_head_R + term_goal_R

        score_diff = score_L - score_R
        if _last_direction > 0:
            direction = 1.0 if score_diff > -DIRECTION_HYSTERESIS else -1.0
        else:
            direction = -1.0 if score_diff < DIRECTION_HYSTERESIS else 1.0
        _last_direction = direction

        total_w_all = sum(r['weight'] for r in layer_results)
        w_mag = sum(r['weight'] * r['urgency'] for r in layer_results) / total_w_all
        w_mag = max(min(w_mag, MAX_W), W_MIN_DANGER)
        w = direction * w_mag

    else:
        desired_heading      = target_bearing
        prev_desired_heading = desired_heading
        w = KP_GOAL * target_bearing

    # 안전 보정 (항상 가산)
    w += (side_right_push - side_left_push) * SIDE_W_BOOST_GAIN
    side_dw, _, _ = get_side_repulsion(scan_points)
    w = max(min(w + side_dw, MAX_W), -MAX_W)

    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP 우선 메인 진입점
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def _stop_reset():
    global stop_cycle_count, stop_pivot_w, stop_phase, _last_direction
    stop_cycle_count = 0
    stop_pivot_w     = 0.0
    stop_phase       = 0
    _last_direction  = 1.0


def _stop_set_pivot(heading_deg, target, gap_width):
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
    global stop_cycle_count, stop_pivot_w, stop_locked_target, stop_locked_gap, \
           stop_locked_global_heading, stop_phase

    # Phase 2: 피봇 중
    if stop_phase == 2:
        if not detect_stop_zone(scan_points):
            _stop_reset()
            return find_vw_layered(scan_points, heading_deg, target_bearing)
        stop_cycle_count += 1
        if stop_cycle_count >= STOP_MAX_CYCLES:
            _stop_reset()
            return find_vw_layered(scan_points, heading_deg, target_bearing)
        err   = abs(((heading_deg - stop_locked_global_heading) + 180) % 360 - 180)
        scale = min(1.0, err / STOP_PIVOT_SLOW_DEG)
        speed = STOP_PIVOT_MIN_W + (STOP_PIVOT_MAX_W - STOP_PIVOT_MIN_W) * scale
        dyn_w = math.copysign(speed, stop_pivot_w)
        return 0.0, dyn_w

    # Phase 0: 정상 → STOP 감지 시 피봇
    if detect_stop_zone(scan_points):
        target, gap_width, gap_info, escape_method = find_stop_escape_direction(scan_points, heading_deg)
        _stop_set_pivot(heading_deg, target, gap_width)
        stop_cycle_count = 0
        stop_phase       = 2
        return 0.0, stop_pivot_w

    return find_vw_layered(scan_points, heading_deg, target_bearing)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 시각화용 분석 (렌더러가 그릴 중간 데이터)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def analyze_scan(scan_points, heading_deg, target_bearing=0.0):
    """렌더러가 레이어/STOP/갭을 그리기 위한 중간 데이터 묶음."""
    layer_results = []
    active_names  = set()
    for layer in LAYERS:
        r = process_layer(scan_points, layer)
        if r is not None:
            layer_results.append(r)
            active_names.add(r['name'])

    stop_triggered = detect_stop_zone(scan_points)
    esc_target, esc_width, gap_info, esc_method = find_stop_escape_direction(scan_points, heading_deg)

    # STOP 발동 시 피봇 방향 미리보기(부호만): +면 좌회전, -면 우회전
    pivot_w_preview = 0.0
    if gap_info:
        if esc_width == 0:
            pivot_w_preview = (-math.copysign(1.0, heading_deg)
                               if abs(heading_deg) > 1 else -1.0)
        else:
            pivot_w_preview = -1.0 if abs(esc_target) < 5 else -math.copysign(1.0, esc_target)

    front_gaps = get_front_passable_gaps(scan_points)
    chosen_front = choose_target_gap(front_gaps, target_bearing, prev_desired_heading)

    side_left, side_right       = get_side_layer_push(scan_points)
    side_dw, side_lstr, side_rstr = get_side_repulsion(scan_points)

    return {
        'layer_results': layer_results,
        'active_names':  active_names,
        'stop_triggered': stop_triggered,
        'esc_target':    esc_target,
        'esc_width':     esc_width,
        'gap_info':      gap_info,
        'pivot_w_preview': pivot_w_preview,
        'front_gaps':    front_gaps,
        'chosen_front':  chosen_front,
        'side_layer':    (side_left, side_right),
        'side_repulse':  (side_dw, side_lstr, side_rstr),
    }


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Phase 2: 오도메트리 / 경계 / CLOSE 목표 (jw_won.py 이식)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def compute_close_target(close_bearing_deg, dist_mm):
    """CLOSE 진입 시 색지 추정 좌표 (arduino 프레임 mm). jw_won._compute_close_target 와
    수식 동일 — 단 camera 의존(get_last_close_bearing/get_estimated_distance_mm)을
    인자로 분리. (x_t, y_t) 반환.
        bearing_global = heading + close_bearing
        target_dist    = max(dist - CLOSE_STANDOFF_MM, 0)   # 색지보다 덜 접근해 정지
        x_t = arduino_x + target_dist*sin(bearing_global)
        y_t = arduino_y + target_dist*cos(bearing_global)
    """
    bearing_global_deg = arduino_heading_deg + close_bearing_deg
    target_dist        = max(dist_mm - CLOSE_STANDOFF_MM, 0.0)
    hdg_rad            = math.radians(bearing_global_deg)
    x_t = arduino_x_mm + target_dist * math.sin(hdg_rad)
    y_t = arduino_y_mm + target_dist * math.cos(hdg_rad)
    return x_t, y_t


def set_boundary_center():
    """[미사용 - 호환용 스텁] 새 로직은 Mode1/2 가 중심을 관리한다."""
    pass


def get_boundary_correction(center_x, center_y, radius):
    """경계 초과 시 (중심 방향 상대 베어링[좌+], v 감속비율) 반환. 경계 내부 → (0.0, 1.0).
    ★ jw_won 새 버전은 여기서 rel_bearing 부호를 뒤집는다(우+ 오도메트리→카메라 규약).
      시뮬은 harness 가 arduino_x = -robot_x 로 '좌+ 미러 프레임'을 쓰므로(_set_arduino_odom)
      atan2(dx,dy) 가 이미 좌+(카메라) 규약을 내놓는다 → 부호 뒤집으면 이중반전으로 발산.
      따라서 여기서는 의도적으로 flip 하지 않는다 (결과값은 jw_won 과 동일)."""
    dx = center_x - arduino_x_mm
    dy = center_y - arduino_y_mm
    dist = math.sqrt(dx ** 2 + dy ** 2)
    if dist <= radius:
        return 0.0, 1.0
    excess = dist - radius
    blend  = min(excess / BOUNDARY_BLEND_DIST, 1.0)
    bearing_to_center = math.degrees(math.atan2(dx, dy))
    rel_bearing = normalize_angle(bearing_to_center - arduino_heading_deg)
    v_scale = BOUNDARY_V_MIN + (1.0 - BOUNDARY_V_MIN) * (1.0 - blend)
    return rel_bearing, v_scale


def get_semicircle_boundary_correction(center_x, center_y, fwd_heading_deg, radius):
    """정면 반원 경계 (초기 탐색 전용). jw_won._get_semicircle_boundary_correction 와 동일.
    중심에서 반경 이내 AND 시작 헤딩 기준 '정면 half-plane' 안이면 내부.
      - 반경 초과       → 중심 방향으로 복귀
      - 정면선 뒤로 넘어감 → 정면(시작 헤딩) 방향으로 복귀
      - 둘 다 위반       → 위반 깊이가 큰 쪽으로 복귀
    반환: (rel_bearing_deg[좌+], v_scale). 경계 내부 → (0.0, 1.0).
    ★ get_boundary_correction 과 동일 이유로 jw_won 의 부호 flip 은 이식하지 않는다(미러 프레임)."""
    dx   = center_x - arduino_x_mm     # 로봇 → 중심
    dy   = center_y - arduino_y_mm
    dist = math.sqrt(dx ** 2 + dy ** 2)

    # 시작 헤딩 정면 단위벡터 (월드, x=sin·y=cos 규약)
    fr     = math.radians(fwd_heading_deg)
    fx, fy = math.sin(fr), math.cos(fr)
    # (로봇−중심)·정면 = 정면 진행 성분. >0: 정면 half / <0: 정면선 뒤로 넘어감
    proj   = (-dx) * fx + (-dy) * fy

    radial_excess = max(0.0, dist - radius)
    behind_excess = max(0.0, -proj)

    if radial_excess <= 0.0 and behind_excess <= 0.0:
        return 0.0, 1.0

    if radial_excess >= behind_excess:
        bearing_to = math.degrees(math.atan2(dx, dy))   # 중심 방향
        excess     = radial_excess
    else:
        bearing_to = fwd_heading_deg                    # 정면(시작 헤딩) 방향
        excess     = behind_excess

    rel_bearing = normalize_angle(bearing_to - arduino_heading_deg)
    blend       = min(excess / BOUNDARY_BLEND_DIST, 1.0)
    v_scale     = BOUNDARY_V_MIN + (1.0 - BOUNDARY_V_MIN) * (1.0 - blend)
    return rel_bearing, v_scale


def get_target_bearing(target_x, target_y):
    """목표 좌표로의 상대 베어링(deg, 좌+). 경계 안/밖 무관하게 항상 목표를 향함. Mode2 능동 접근용.
    ★ jw_won 새 버전은 반환값 부호를 뒤집지만(우+ 오도메트리), 시뮬 미러 프레임에서는
      atan2 가 이미 좌+(카메라) 규약 → flip 미이식 (get_boundary_correction 과 동일 이유)."""
    dx = target_x - arduino_x_mm
    dy = target_y - arduino_y_mm
    bearing_to_target = math.degrees(math.atan2(dx, dy))
    return normalize_angle(bearing_to_target - arduino_heading_deg)


def update_target_estimate(bearing_rel_deg, dist_mm):
    """색 감지 중 목표 색지 추정 위치 갱신 (Mode2 경계 중심용).
    거리 추정이 신뢰 범위(4000mm) 밖이면 갱신 생략. camera 의존은 인자로 분리.
    ★ jw_won 새 버전은 global_hdg = heading - bearing_rel (우+ 프레임)이지만,
      시뮬 미러 프레임(좌+)에서는 +bearing_rel 이 올바른 배치 → 부호 유지."""
    global _last_target_est_x, _last_target_est_y
    if dist_mm >= 4000.0:
        return
    global_hdg = arduino_heading_deg + bearing_rel_deg
    hdg_rad    = math.radians(global_hdg)
    _last_target_est_x = arduino_x_mm + dist_mm * math.sin(hdg_rad)
    _last_target_est_y = arduino_y_mm + dist_mm * math.cos(hdg_rad)


def update_boundary_exit_tracking(center_x, center_y):
    """경계 이탈→복귀 횟수 추적(히스테리시스). BOUNDARY_EXPAND_TRIGGER 회 누적 시 반경 확장.
    jw_won._update_boundary_exit_tracking 와 동일."""
    global _boundary_was_outside, _boundary_exit_count, _current_boundary_radius
    dx = center_x - arduino_x_mm
    dy = center_y - arduino_y_mm
    dist = math.sqrt(dx ** 2 + dy ** 2)
    out_threshold = _current_boundary_radius + BOUNDARY_HYSTERESIS_MM
    in_threshold  = _current_boundary_radius - BOUNDARY_HYSTERESIS_MM
    if not _boundary_was_outside and dist > out_threshold:
        _boundary_was_outside = True
    elif _boundary_was_outside and dist < in_threshold:
        _boundary_was_outside = False
        _boundary_exit_count += 1
        if _boundary_exit_count >= BOUNDARY_EXPAND_TRIGGER:
            if _current_boundary_radius < BOUNDARY_RADIUS_MAX:
                _current_boundary_radius = min(
                    _current_boundary_radius + BOUNDARY_RADIUS_EXPAND,
                    BOUNDARY_RADIUS_MAX)
                _boundary_exit_count = 0


def switch_mode2_to_mode1(reason):
    """Mode2 → Mode1 전환. 마지막 목표 추정 위치를 새 탐색 중심으로 삼아 피버턴 재시작.
    타임아웃·도착후 색지 미발견 등 여러 전환 사유에서 공통 호출.
    jw_won._switch_mode2_to_mode1 와 동일."""
    global _search_mode, _last_arrival_x, _last_arrival_y
    global _pivot_active, _pivot_prev_hdg, _pivot_total_rotated, _pivot_direction
    global _mode2_start_time, _mode2_arrived_time
    global _last_pivot_robot_x, _last_pivot_robot_y
    _search_mode         = 1
    _last_arrival_x      = _last_target_est_x
    _last_arrival_y      = _last_target_est_y
    _pivot_active        = True
    _pivot_prev_hdg      = arduino_heading_deg
    _pivot_total_rotated = 0.0
    _pivot_direction     = 1.0
    _mode2_start_time    = None
    _mode2_arrived_time  = None
    _inspected_gaps[:]   = []    # 새 탐색 중심 → 갭 메모리 초기화
    _last_pivot_robot_x  = arduino_x_mm
    _last_pivot_robot_y  = arduino_y_mm
    print(f"[MODE2→1] {reason}: "
          f"중심=({_last_arrival_x:.0f},{_last_arrival_y:.0f}) "
          f"경계={_current_boundary_radius:.0f}mm 피버턴 재시작")


def reset_search_state():
    """오도메트리 + 탐색 모드 + 가변 경계 전역 초기화 (재시작 시)."""
    global arduino_x_mm, arduino_y_mm, arduino_heading_deg
    global _search_mode, _last_arrival_x, _last_arrival_y
    global _last_target_est_x, _last_target_est_y, _last_known_mission_idx
    global _color_confirm_start, _color_confirm_ref
    global _pivot_active, _pivot_prev_hdg, _pivot_total_rotated, _pivot_direction, _last_pivot_time
    global _mode2_start_time, _mode2_arrived_time, _inspected_gaps, _last_pivot_robot_x, _last_pivot_robot_y, _new_gap_streak
    global _current_boundary_radius, _boundary_exit_count, _boundary_was_outside
    global _initial_x, _initial_y, _initial_heading, _initial_pose_set, _use_semicircle_boundary
    arduino_x_mm = arduino_y_mm = arduino_heading_deg = 0.0
    _search_mode = 0
    _last_arrival_x = _last_arrival_y = None
    _last_target_est_x = _last_target_est_y = None
    _last_known_mission_idx = 0
    _color_confirm_start = None
    _color_confirm_ref = 0.0
    _pivot_active = False
    _pivot_prev_hdg = 0.0
    _pivot_total_rotated = 0.0
    _pivot_direction = 1.0
    _last_pivot_time = 0.0
    _mode2_start_time = None
    _mode2_arrived_time = None
    _inspected_gaps = []
    _last_pivot_robot_x = _last_pivot_robot_y = None
    _new_gap_streak = 0
    _current_boundary_radius = BOUNDARY_RADIUS
    _boundary_exit_count = 0
    _boundary_was_outside = False
    _initial_x = _initial_y = _initial_heading = 0.0
    _initial_pose_set = False
    _use_semicircle_boundary = False


# 호환 alias (project3_sim_v2 가 호출)
reset_odom_state = reset_search_state
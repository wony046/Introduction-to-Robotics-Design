import serial
import time
import math
import json
import threading

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

FORWARD_SPEED    = 0.45
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
     'v_max':0.22, 'affects_v':True},
    # L2: 가까움, 동적 가중치, weight_cap=4.5, v_max=0.38
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':120,
     'w_gain':2.5, 'weight_base':0.6, 'weight_cap':5.0, 'weight_dynamic':True,
     'v_max':0.38, 'affects_v':True},
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
SCORE_GAP_FRONT      = 900.0 # 전방 통과 가능 갭 방향 보너스 계수

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
SIDE_LAYER_DIST_MAX  = 600  # mm: 측방 감지 최대 거리
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

# ── 디버그 토글 ──────────────────────────────────────────────────────────────
DEBUG_LAYERS  = True    # 각 레이어 처리 결과
DEBUG_STOP    = True    # STOP zone 감지 & 탈출
DEBUG_DIR     = True    # 점수 계산 & 방향 결정
DEBUG_FINAL   = True    # 최종 v, w
DEBUG_SIDE    = True    # 측면 반발력
DEBUG_VIRTUAL = True    # [추가] 가상 장애물 디버그

# ── 전역 상태 ────────────────────────────────────────────────────────────────
arduino_heading_deg   = 0.0
prev_w                = 0.0
_last_direction       = 1.0   # 마지막으로 결정된 방향 (+1=왼쪽, -1=오른쪽)
stop_cycle_count           = 0     # 현재 phase 내 사이클 카운터
stop_pivot_w               = 0.0   # 피봇 방향 (부호만 사용)
stop_locked_target         = 0.0
stop_locked_gap            = 0.0
stop_locked_global_heading = 0.0
stop_phase                 = 0     # 0=idle, 2=피봇

# ── 스레드 공유 상태 ─────────────────────────────────────────────────────────
_scan_lock   = threading.Lock()
_latest_scan = []            # 라이다 스레드가 완성된 스캔을 여기에 기록
_shutdown    = threading.Event()  # 종료 신호


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return angle - 360 if angle > 180 else angle

def is_in_front_90(a):
    return -90 <= a <= 90

def is_in_wide_scan(a):
    return -SCAN_WIDE_HALF <= a <= SCAN_WIDE_HALF

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
    global arduino_heading_deg
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'): arduino_heading_deg = float(line[2:])
        except Exception: pass


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

    if DEBUG_DIR and (left_push > 0 or right_push > 0):
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

def find_vw_layered(scan_points, heading_deg):
    """
    1. 6개 레이어 병렬 처리
    2. gap 너비 계산 (가장 가까운 레이어의 대표점 기준)
    3. 좌우 점수 통합 → 방향 결정 (매 사이클 score로 재결정)
       - [추가] 통과 불가 갭 가상 장애물 척력 반영
         이중 반응 방지를 위해 effective_push = max(real, virtual)
    4. v: affects_v 레이어 v_proposal의 가중 평균
    5. w: 모든 활성 레이어 urgency의 가중 평균 × direction
    6. w 보정: 측방 레이어 net delta + 측면 반발력 합산
    """
    # 1. 레이어 처리
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

    # 활성 레이어 없으면 직진
    if not layer_results:
        if DEBUG_FINAL:
            print(f"  [FINAL] no active layers -> v={FORWARD_SPEED:.2f} w=0.00")
        return FORWARD_SPEED, 0.0

    # 2. gap 너비 계산 (가장 가까운 레이어의 대표점 기준)
    closest = min(layer_results, key=lambda r: r['rep_horiz'])
    ref_angle = closest['rep_angle']
    ref_dist  = math.sqrt(closest['rep_horiz']**2 + closest['rep_fwd']**2)

    gap_L = get_gap_width(scan_points, ref_angle, ref_dist, is_left=True)
    gap_R = get_gap_width(scan_points, ref_angle, ref_dist, is_left=False)

    # 2-2. 전방 통과 가능 갭 탐색 (에지→선분 기반)
    front_gaps = get_front_passable_gaps(scan_points)
    best_fg = front_gaps[0] if front_gaps else None

    gap_bonus_L = 0.0
    gap_bonus_R = 0.0
    if best_fg is not None:
        norm = ((best_fg['width'] / STOP_ESCAPE_MIN_GAP) *
                (best_fg['depth'] / FRONT_GAP_MIN_DEPTH))
        bonus = SCORE_GAP_FRONT * norm
        if best_fg['center_angle'] < 0:
            gap_bonus_L = bonus
        else:
            gap_bonus_R = bonus

    if DEBUG_DIR:
        print(f"  [FRONT_GAP] {len(front_gaps)} passable gap(s) found  "
              f"(min_w={STOP_ESCAPE_MIN_GAP}mm min_d={FRONT_GAP_MIN_DEPTH}mm)")
        for g in front_gaps:
            chosen = " ← best" if g is best_fg else ""
            print(f"    ca={g['center_angle']:+.1f}° w={g['width']:.0f}mm "
                  f"d={g['depth']:.0f}mm score={g['score']:.0f}{chosen}")
        if best_fg is not None:
            side = "LEFT" if best_fg['center_angle'] < 0 else "RIGHT"
            print(f"  [FRONT_GAP] bonus → {side}  +{gap_bonus_L or gap_bonus_R:.0f}")
        else:
            print(f"  [FRONT_GAP] no passable gap → no bonus")

    # 3. 좌우 점수 통합
    sum_pR = sum(r['weight'] * r['push_right'] for r in layer_results)
    sum_pL = sum(r['weight'] * r['push_left']  for r in layer_results)

    # 3-2. [추가] 가상 장애물 척력 (통과 불가 갭) — 레이어별 max 취합
    #   레이어 경계 중복 가산 없음. STOP 피봇 중에는 비활성화.
    virt_push_L_total = 0.0
    virt_push_R_total = 0.0
    for layer in LAYERS:
        vpl, vpr = get_narrow_gap_pushes(
            scan_points, layer,
            in_stop=(stop_phase == 2)
        )
        virt_push_L_total = max(virt_push_L_total, vpl)
        virt_push_R_total = max(virt_push_R_total, vpr)

    # ── 이중 반응 안전망: max(실제, 가상) → 더 강한 신호 하나만 적용 ─────────
    #   sum_pR / virt_push_R_total : 오른쪽 장애물·불통과 갭 → 둘 다 score_L 기여
    #   같은 방향 신호이므로 합산 대신 max() 사용
    effective_push_R = max(sum_pR, virt_push_R_total)
    effective_push_L = max(sum_pL, virt_push_L_total)

    # 측방 레이어 방향 기여 (방향 결정 주도)
    side_left_push, side_right_push = get_side_layer_push(scan_points)

    term_gap_L    = SCORE_ALPHA * gap_L
    term_gap_R    = SCORE_ALPHA * gap_R
    term_push_L   = SCORE_BETA  * effective_push_R
    term_push_R   = SCORE_BETA  * effective_push_L
    term_side_L   = SCORE_SIDE  * side_right_push
    term_side_R   = SCORE_SIDE  * side_left_push
    term_head_L   = max(0.0, -heading_deg) * HEADING_WEIGHT_MM
    term_head_R   = max(0.0,  heading_deg) * HEADING_WEIGHT_MM

    score_L = term_gap_L + term_push_L + term_side_L + term_head_L + gap_bonus_L
    score_R = term_gap_R + term_push_R + term_side_R + term_head_R + gap_bonus_R

    if DEBUG_DIR:
        print(f"  [GAP_W] L={gap_L:.0f}mm R={gap_R:.0f}mm  "
              f"(ref={ref_angle:+.1f}°/{ref_dist:.0f}mm from {closest['name']})")
        print(f"  [SCORE] L={score_L:.0f}  R={score_R:.0f}")
        print(f"    gap   αL={term_gap_L:.0f} / αR={term_gap_R:.0f}")
        print(f"    push  βL={term_push_L:.0f} / βR={term_push_R:.0f}  "
              f"[real {SCORE_BETA*sum_pR:.0f}/{SCORE_BETA*sum_pL:.0f}  "
              f"virt {SCORE_BETA*virt_push_R_total:.0f}/{SCORE_BETA*virt_push_L_total:.0f}]")
        print(f"    side  γL={term_side_L:.0f} / γR={term_side_R:.0f}")
        print(f"    head  hL={term_head_L:.0f} / hR={term_head_R:.0f}")
        print(f"    fgap  fL={gap_bonus_L:.0f} / fR={gap_bonus_R:.0f}")

    # 4. 방향 결정 (히스테리시스: 점수 차가 DIRECTION_HYSTERESIS 미만이면 직전 방향 유지)
    global _last_direction
    score_diff = score_L - score_R  # 양수 = 왼쪽 유리, 음수 = 오른쪽 유리
    if _last_direction > 0:
        direction = 1.0 if score_diff > -DIRECTION_HYSTERESIS else -1.0
    else:
        direction = -1.0 if score_diff < DIRECTION_HYSTERESIS else 1.0
    if DEBUG_DIR:
        switched = "SWITCH" if direction != _last_direction else "HOLD"
        print(f"  [DIR] {'LEFT' if direction > 0 else 'RIGHT'} (diff={score_diff:+.0f} hyst=±{DIRECTION_HYSTERESIS:.0f} {switched})")
    _last_direction = direction

    # 5. v 계산: affects_v 레이어 v_proposal의 가중 평균
    v_layers = [r for r in layer_results if r['v_proposal'] is not None]
    if v_layers:
        total_w = sum(r['weight'] for r in v_layers)
        v = sum(r['weight'] * r['v_proposal'] for r in v_layers) / total_w
    else:
        v = FORWARD_SPEED   # L5/L6만 활성 → 최대 속도

    # 6. w 크기: 모든 활성 레이어 urgency 가중 평균
    total_w_all = sum(r['weight'] for r in layer_results)
    w_mag = sum(r['weight'] * r['urgency'] for r in layer_results) / total_w_all
    w_mag = max(min(w_mag, MAX_W), W_MIN_DANGER)

    w = direction * w_mag

    # 측방 레이어 거리 기반 w 크기 보정 (부호 있는 net delta)
    side_w_delta = (side_right_push - side_left_push) * SIDE_W_BOOST_GAIN
    w = w + side_w_delta

    # 측면 반발력 합산
    side_dw, _, _ = get_side_repulsion(scan_points)
    w = max(min(w + side_dw, MAX_W), -MAX_W)

    if DEBUG_FINAL:
        print(f"  [FINAL] v={v:.2f} w={w:+.2f} dir={'L' if direction > 0 else 'R'} "
              f"side_boost={side_w_delta:+.3f}")

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


def find_vw_command(scan_points, heading_deg):
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
            return find_vw_layered(scan_points, heading_deg)

        stop_cycle_count += 1
        if stop_cycle_count >= STOP_MAX_CYCLES:
            if DEBUG_STOP:
                print(f"  [STOP] max pivot cycles ({STOP_MAX_CYCLES}) -> force layered")
            _stop_reset()
            return find_vw_layered(scan_points, heading_deg)

        err   = abs(((heading_deg - stop_locked_global_heading) + 180) % 360 - 180)
        scale = min(1.0, err / STOP_PIVOT_SLOW_DEG)
        speed = STOP_PIVOT_MIN_W + (STOP_PIVOT_MAX_W - STOP_PIVOT_MIN_W) * scale
        dyn_w = math.copysign(speed, stop_pivot_w)
        if DEBUG_STOP:
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
        _fname = f'stop_event_{int(time.time())}.json'
        with open(_fname, 'w') as _f:
            json.dump({
                'heading':  heading_deg,
                'target':   target,
                'gap_dist': gap_width,
                'gap_info': gap_info,
                'scan':     [[a, d] for a, d in scan_points if d > 0],
            }, _f)
        if DEBUG_STOP:
            print(f"  [STOP] triggered -> pivot target={target:+.0f}° "
                  f"global={stop_locked_global_heading:.1f}° event saved {_fname}")
        return 0.0, stop_pivot_w

    return find_vw_layered(scan_points, heading_deg)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 스레드: 라이다 수신 / 모터 제어
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _dedup_scan(pts):
    """1° 단위 버킷화, 중복 각도는 가장 가까운 거리만 유지."""
    angle_map = {}
    for angle, dist in pts:
        bucket = round(angle)
        if bucket not in angle_map or dist < angle_map[bucket]:
            angle_map[bucket] = dist
    return list(angle_map.items())


def _lidar_reader(lidar):
    """라이다 수신 전용 스레드.
    lidar.read(5) 블로킹이 모터 루프에 영향을 주지 않도록 분리.
    한 바퀴 완성 시 중복 각도 제거 후 _latest_scan에 덮어쓰기 (누적 없음)."""
    local_pts = []
    while not _shutdown.is_set():
        try:
            raw = lidar.read(5)
        except Exception:
            continue
        result = parse_packet(raw)
        if result is None:
            continue
        angle_raw, distance = result
        s_flag = raw[0] & 0x01
        if s_flag == 1 and local_pts:
            deduped = _dedup_scan(local_pts)   # 락 밖에서 연산
            with _scan_lock:
                _latest_scan.clear()
                _latest_scan.extend(deduped)
            local_pts = []
        local_pts.append((
            normalize_angle(angle_raw),
            distance + LIDAR_OFFSET if distance > 0 else 0
        ))


def _motor_controller(arduino):
    """모터 제어 전용 스레드.
    SEND_INTERVAL마다 독립적으로 명령 송신 — 라이다 지연과 무관."""
    global prev_w
    last_cmd_str = ""
    while not _shutdown.is_set():
        read_arduino(arduino)
        with _scan_lock:
            pts = [(a, d) for a, d in _latest_scan if d > 0]
        if pts:
            v, w = find_vw_command(pts, arduino_heading_deg)
            w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
            prev_w = w
            cmd = f"{v:.2f} {w:.2f}\n"
            arduino.write(cmd.encode())
            if cmd != last_cmd_str:
                print(f"[SEND] v={v:.2f}  w={w:+.2f}  "
                      f"heading={arduino_heading_deg:.1f}deg")
                last_cmd_str = cmd
        time.sleep(SEND_INTERVAL)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    print("=== RPLIDAR Obstacle Avoidance (Layered + Front Gap + Virtual Obstacle) ===")
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
    print(f"  Scoring     : alpha={SCORE_ALPHA} beta={SCORE_BETA} "
          f"(real/virtual push: max 선택)")
    print(f"  Direction   : score-based per cycle (no locking anywhere)")
    print(f"  Debug flags : LAYERS={DEBUG_LAYERS} STOP={DEBUG_STOP} "
          f"DIR={DEBUG_DIR} FINAL={DEBUG_FINAL} "
          f"SIDE={DEBUG_SIDE} VIRTUAL={DEBUG_VIRTUAL}")
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

    t_lidar = threading.Thread(target=_lidar_reader,      args=(lidar,),   daemon=True, name="lidar")
    t_motor = threading.Thread(target=_motor_controller,  args=(arduino,), daemon=True, name="motor")

    try:
        t_lidar.start()
        t_motor.start()
        while not _shutdown.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        _shutdown.set()
        t_lidar.join(timeout=2.0)
        t_motor.join(timeout=2.0)
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        lidar.close()
        arduino.write(b"0.00 0.00\n")
        arduino.close()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
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
MAX_W            = 2.0
W_MIN_DANGER     = 0.5   # rad/s: 위험 시 최소 회전
W_SMOOTH         = 0.6

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 계층형 바운딩 박스 정의 (6개 레이어)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 각 레이어: 거리 범위, horiz 임계, w_gain, 기본 가중치, 동적 가중치 여부, v 영향 여부

LAYERS = [
    # L1: 가장 가까움, 동적 가중치, weight_cap=5.0, v_max=0.30
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':120,
     'w_gain':2.8, 'weight_base':0.8, 'weight_cap':7.5, 'weight_dynamic':True,
     'v_max':0.30, 'affects_v':True},
    # L2: 가까움, 동적 가중치, weight_cap=3.0, v_max=0.38
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':120,
     'w_gain':2.5, 'weight_base':0.6, 'weight_cap':4.5, 'weight_dynamic':True,
     'v_max':0.38, 'affects_v':True},
    # L3: 중간 (weight: 진입 0.4 → 끝 0.2 선형 보간)
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':120,
     'w_gain':1.8, 'weight_base':0.2, 'weight_start':0.4, 'weight_dynamic':False, 'affects_v':True},
    # L4: 중간-원거리 (weight: 진입 0.2 → 끝 0.1)
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':100,
     'w_gain':1.0, 'weight_base':0.1, 'weight_start':0.2, 'weight_dynamic':False, 'affects_v':True},
    # L5: 원거리 (weight: 진입 0.1 → 끝 0.05)
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':100,
     'w_gain':0.4, 'weight_base':0.05,'weight_start':0.1, 'weight_dynamic':False, 'affects_v':False},
    # L6: 최원거리 (weight: 진입 0.05 → 끝 0.02)
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':100,
     'w_gain':0.3, 'weight_base':0.02,'weight_start':0.05,'weight_dynamic':False, 'affects_v':False},
]

LAYER_PERCENTILE = 5    # %: 하위 N% dist 평균으로 레이어 대표점 계산

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP zone (계층형과 완전 별도)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP rectangle: 전방 100~180mm 사이, horiz < 105mm (210mm 폭)

STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 175
STOP_HORIZ_TH = 105

# STOP 탈출: 360° 전체 스캔, ROBOT_HALF_WIDTH*2 + 양쪽 20mm 마진
STOP_ESCAPE_MIN_GAP   = ROBOT_HALF_WIDTH * 2 + 40   # 260mm
STOP_MAX_CYCLES       = 30                          # 연속 STOP 사이클 상한 (초과 시 강제 탈출)
STOP_PIVOT_MAX_W      = 1.0   # rad/s: STOP 피봇 일정 회전 속도

# FGM (Follow the Gap Method) — STOP escape 전용
FGM_MIN_ANG_DEG      = 5     # deg: 이 이상 각도 공백이면 갭으로 인식
FGM_MIN_DEPTH_MM     = 200   # mm: 갭 너머 최소 깊이 (얕은 함몰부 제외)
FGM_MAX_RANGE_MM     = 800   # mm: FGM 갭 탐색 최대 거리 (이 이상 포인트 무시)
FGM_RATIO_THRES      = 1.5   # 인접 포인트 거리 비율 이상이면 갭 경계로 인식 (벽 끝 완만 전환)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 방향 점수제 (gap + layer 통합)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCORE_ALPHA       = 5.0    # gap_width 계수
SCORE_BETA        = 8     # 정면 레이어 push 계수 (방향 영향 약화)
SCORE_SIDE        = 10000.0  # 측방 레이어 방향 가중치 (방향 결정 주도)
HEADING_WEIGHT_MM = 5.0    # 헤딩 1° = 여유 5mm
DEPTH_JUMP_THRES  = 120    # mm: 이상이면 다른 물체로 인식

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 스캔 범위 & 통신
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCAN_WIDE_HALF = 135   # 측면 반발력 감지 범위 (is_in_wide_scan 사용) (각도)
SEND_INTERVAL  = 0.1

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 측면 반발력 파라미터 (50mm × 240mm 레이어)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SIDE_SAFE_MARGIN  = 190   # mm: 로봇 측면 안전 마진 (side_th = 110+190 = 300mm)
SIDE_FWD_LEAD     = 80    # mm: 라이다 기준 전방 여유 (진입 예측)
SIDE_FWD_REAR     = 80    # mm: 라이다 기준 후방 깊이 (로봇 몸체)
SIDE_REPULSE_GAIN = 0.8   # rad/s: 반발력 최대 w 기여
SIDE_EXP_K        = 3.0   # 지수 계수: 클수록 근접 시 반발력이 급격히 증가

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 측방 방향 레이어 (±15°~±75°, 600mm)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SIDE_LAYER_ANG_START = 15   # deg: 정면 레이어와 경계
SIDE_LAYER_ANG_END   = 75   # deg: 측방 레이어 바깥 경계
SIDE_LAYER_DIST_MAX  = 600  # mm: 측방 감지 최대 거리

# ── 디버그 토글 ──────────────────────────────────────────────────────────────
DEBUG_LAYERS = True    # 각 레이어 처리 결과
DEBUG_STOP   = True    # STOP zone 감지 & 탈출
DEBUG_DIR    = True    # 점수 계산 & 방향 결정
DEBUG_FINAL  = True    # 최종 v, w
DEBUG_SIDE   = True    # 측면 반발력

# ── 전역 상태 ────────────────────────────────────────────────────────────────
arduino_heading_deg   = 0.0
prev_w                = 0.0
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

def polar_to_xy(angle_deg, dist):
    """극좌표 → 직교좌표 (전방=+y, 우측=+x)"""
    rad = math.radians(angle_deg)
    return dist * math.sin(rad), dist * math.cos(rad)

def point_to_segment_dist(px, py, ax, ay, bx, by):
    """점 P → 선분 AB 최소 거리"""
    dx, dy = bx - ax, by - ay
    seg_sq = dx * dx + dy * dy
    if seg_sq < 1e-10:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

def segment_min_dist(a1, a2, b1, b2):
    """
    두 선분 (a1→a2), (b1→b2) 사이의 최소 거리 (2D Cartesian).

    4가지 경우(각 끝점 → 상대 선분) 중 최솟값을 반환한다.
    두 선분이 교차하면 0에 가까운 값이 나오므로 통과 불가로 판정된다.

      a1 ──── a2          b1 ──── b2
      (좌측 장애물 선분)   (우측 장애물 선분)

    min(
        dist(a1 → seg_b),   # 좌측 뒤쪽 끝 → 우측 선분
        dist(a2 → seg_b),   # 좌측 갭 엣지  → 우측 선분  ← 보통 최솟값
        dist(b1 → seg_a),   # 우측 갭 엣지  → 좌측 선분  ← 또는 이게
        dist(b2 → seg_a),   # 우측 뒤쪽 끝  → 좌측 선분
    )
    """
    return min(
        point_to_segment_dist(a1[0], a1[1], b1[0], b1[1], b2[0], b2[1]),
        point_to_segment_dist(a2[0], a2[1], b1[0], b1[1], b2[0], b2[1]),
        point_to_segment_dist(b1[0], b1[1], a1[0], a1[1], a2[0], a2[1]),
        point_to_segment_dist(b2[0], b2[1], a1[0], a1[1], a2[0], a2[1]),
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
    """STOP rectangle (fwd 100~150mm, horiz<110mm) 안에 장애물이 있는가?"""
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

    갭 너비 계산 — 선분-선분 최소 거리:
      각 갭 경계 양쪽의 장애물을 선분으로 모델링하고
      두 선분 사이의 최소 거리를 통과 가능 너비로 사용한다.

      pts[i-1] ──── pts[i]   |갭|   pts[i+1] ──── pts[i+2]
      (좌측 장애물 선분 L)          (우측 장애물 선분 R)

      width = segment_min_dist(L, R)

      인접 포인트가 없는 스캔 양 끝(i==0 또는 i+2>=len)은
      기존 cosine_dist fallback 사용.

    반환: list of dict {width, center_angle, edge_a, edge_b, depth}
    """
    pts = sorted(
        [(a, d) for a, d in scan_points
         if LIDAR_MIN_VALID < d < FGM_MAX_RANGE_MM],
        key=lambda p: p[0]
    )
    if len(pts) < 2:
        return []

    gaps = []
    for i in range(len(pts) - 1):
        a1, d1 = pts[i]
        a2, d2 = pts[i + 1]
        ang_diff = a2 - a1  # 항상 양수 (오름차순 정렬)

        is_depth_jump   = abs(d2 - d1) > DEPTH_JUMP_THRES
        is_angular_hole = ang_diff >= FGM_MIN_ANG_DEG
        is_ratio_jump   = (d2 / d1 > FGM_RATIO_THRES) or (d1 / d2 > FGM_RATIO_THRES)

        if not (is_depth_jump or is_angular_hole or is_ratio_jump):
            continue

        # ── 갭 너비: 선분-선분 최소 거리 ──────────────────────────────
        # 좌측 장애물 선분: pts[i-1] → pts[i]   (갭의 왼쪽 벽)
        # 우측 장애물 선분: pts[i+1] → pts[i+2] (갭의 오른쪽 벽)
        # 인접 포인트가 존재할 때만 선분-선분 거리를 사용하고,
        # 스캔 양 끝(인접 포인트 없음)은 기존 엣지-엣지 거리로 fallback.
        if i > 0 and i + 2 < len(pts):
            la = polar_to_xy(*pts[i - 1])   # 좌측 장애물 뒤쪽 끝
            lb = polar_to_xy(*pts[i])        # 좌측 장애물 갭 쪽 엣지
            ra = polar_to_xy(*pts[i + 1])    # 우측 장애물 갭 쪽 엣지
            rb = polar_to_xy(*pts[i + 2])    # 우측 장애물 뒤쪽 끝
            width = segment_min_dist(la, lb, ra, rb)
        else:
            # 스캔 양 끝 — 인접 포인트 없음 → 기존 방식 fallback
            width = cosine_dist(d1, d2, ang_diff)
        # ─────────────────────────────────────────────────────────────

        # 갭 중심: 두 엣지점의 Cartesian 중점 → 각도 변환 (각도 평균은 wrap 위험)
        x1 = d1 * math.sin(math.radians(a1))
        y1 = d1 * math.cos(math.radians(a1))
        x2 = d2 * math.sin(math.radians(a2))
        y2 = d2 * math.cos(math.radians(a2))
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
        # L3~L6: fwd 위치에 따라 weight_start → weight_base 선형 보간
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
      fwd:   -SIDE_FWD_REAR(-240mm) ~ +SIDE_FWD_LEAD(+50mm)
             라이다 뒤쪽(로봇 몸체)이 주 감지 영역

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


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 계층형 v/w 산출 (메인 로직)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_layered(scan_points, heading_deg):
    """
    1. 6개 레이어 병렬 처리
    2. gap 너비 계산 (가장 가까운 레이어의 대표점 기준)
    3. 좌우 점수 통합 → 방향 결정 (매 사이클 score로 재결정)
    4. v: affects_v 레이어 v_proposal의 가중 평균
    5. w: 모든 활성 레이어 urgency의 가중 합 × direction
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

    # 3. 좌우 점수 통합
    sum_pR = sum(r['weight'] * r['push_right'] for r in layer_results)
    sum_pL = sum(r['weight'] * r['push_left']  for r in layer_results)

    # 측방 레이어 방향 기여 (방향 결정 주도)
    side_left_push, side_right_push = get_side_layer_push(scan_points)

    score_L = (SCORE_ALPHA * gap_L
               + SCORE_BETA  * sum_pR
               + SCORE_SIDE  * side_right_push   # 우측 장애물 → 좌로 회피
               + max(0.0, -heading_deg) * HEADING_WEIGHT_MM)
    score_R = (SCORE_ALPHA * gap_R
               + SCORE_BETA  * sum_pL
               + SCORE_SIDE  * side_left_push    # 좌측 장애물 → 우로 회피
               + max(0.0,  heading_deg) * HEADING_WEIGHT_MM)

    if DEBUG_DIR:
        print(f"  [GAP] L={gap_L:.0f}mm R={gap_R:.0f}mm  "
              f"(ref={ref_angle:+.1f}°/{ref_dist:.0f}mm from {closest['name']})")
        print(f"  [SCORE] L={score_L:.0f}  R={score_R:.0f}  "
              f"(gap αL={SCORE_ALPHA*gap_L:.0f}/αR={SCORE_ALPHA*gap_R:.0f}  "
              f"push βL={SCORE_BETA*sum_pL:.0f}/βR={SCORE_BETA*sum_pR:.0f}  "
              f"side γL={SCORE_SIDE*side_right_push:.0f}/γR={SCORE_SIDE*side_left_push:.0f})")

    # 4. 방향 결정 (매 사이클 score로 재결정, 잠금 없음)
    direction = 1.0 if score_L >= score_R else -1.0
    if DEBUG_DIR:
        print(f"  [DIR] {'LEFT' if direction > 0 else 'RIGHT'}")

    # 5. v 계산: affects_v 레이어 v_proposal의 가중 평균
    v_layers = [r for r in layer_results if r['v_proposal'] is not None]
    if v_layers:
        total_w = sum(r['weight'] for r in v_layers)
        v = sum(r['weight'] * r['v_proposal'] for r in v_layers) / total_w
    else:
        v = FORWARD_SPEED   # L5/L6만 활성 → 최대 속도

    # 6. w 크기: 모든 활성 레이어 urgency 가중 합
    total_w_all = sum(r['weight'] for r in layer_results)
    w_mag = sum(r['weight'] * r['urgency'] for r in layer_results) / total_w_all
    w_mag = max(min(w_mag, MAX_W), W_MIN_DANGER)

    w = direction * w_mag

    # 측면 반발력 합산
    side_dw, _, _ = get_side_repulsion(scan_points)
    w = max(min(w + side_dw, MAX_W), -MAX_W)

    if DEBUG_FINAL:
        print(f"  [FINAL] v={v:.2f} w={w:+.2f} dir={'L' if direction > 0 else 'R'}")

    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 진입점 (STOP 우선)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _stop_reset():
    """STOP 상태 전역 변수 초기화."""
    global stop_cycle_count, stop_pivot_w, stop_phase
    stop_cycle_count = 0
    stop_pivot_w     = 0.0
    stop_phase       = 0


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

        dyn_w = math.copysign(STOP_PIVOT_MAX_W, stop_pivot_w)
        if DEBUG_STOP:
            err = abs(((heading_deg - stop_locked_global_heading) + 180) % 360 - 180)
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
    print("=== RPLIDAR Obstacle Avoidance (Layered Bounding Box) ===")
    print(f"  Layers      : 6 layers (60~780mm), bottom {LAYER_PERCENTILE}% per layer")
    print(f"  L1-L2       : dynamic weight max(base, h_err/h_th), affects v")
    print(f"  L3-L4       : interp weight (L3: 0.4→0.2, L4: 0.2→0.1), affects v")
    print(f"  L5-L6       : interp weight (L5: 0.1→0.05, L6: 0.05→0.02), no v effect")
    print(f"  STOP zone   : fwd {STOP_FWD_MIN}-{STOP_FWD_MAX}mm, horiz<{STOP_HORIZ_TH}mm")
    print(f"  STOP escape : 360deg scan, min_gap={STOP_ESCAPE_MIN_GAP}mm")
    print(f"  Gap width   : segment-to-segment min dist (fallback: cosine dist)")
    print(f"  Scoring     : alpha={SCORE_ALPHA} beta={SCORE_BETA}")
    print(f"  Direction   : score-based per cycle (no locking anywhere)")
    print(f"  Debug flags : LAYERS={DEBUG_LAYERS} STOP={DEBUG_STOP} "
          f"DIR={DEBUG_DIR} FINAL={DEBUG_FINAL}")
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

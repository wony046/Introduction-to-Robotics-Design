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
TARGET_ALIGN_ANGLE = 60.0
TARGET_CLEAR_CONE  = 18
TARGET_BLOCK_DIST  = 600

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

# 가상 경계 (색 미감지 시 활성화) / CLOSE (Phase 2~3 에서 시뮬이 사용)
BOUNDARY_RADIUS     = 1500.0      # ★ mm: 가상 경계 반경
BOUNDARY_BLEND_DIST = 300.0       # mm: 경계 초과 후 인력 100%까지 도달하는 거리
BOUNDARY_V_MIN      = 0.5         # 경계 완전 초과 시 v 감속 최소 비율 (원래 v의 50%)
KP_CLOSE_HDG        = 0.1         # ★
CLOSE_SPEED_MAX     = 0.2         # ★
CLOSE_ARRIVE_MM     = 30          # ★
CLOSE_OBSERVE_SEC   = 1.0
CLOSE_ENTER_MM      = 400.0       # ★

# 디버그 (시뮬에서는 전부 OFF; print 호출만 무력화)
DEBUG_LAYERS = DEBUG_STOP = DEBUG_STOP_PIVOT = DEBUG_BRANCH = 0
DEBUG_TARGET = DEBUG_GAP = DEBUG_FALLBACK = DEBUG_SCORE = 0
DEBUG_DIR = DEBUG_CLEAR = DEBUG_FINAL = DEBUG_SIDE = 0
DEBUG_SIDE_LAYER = DEBUG_VIRTUAL = 0

# ── 회피 상태 (jw_won.py 전역과 동일 역할) ──────────────────────────────
prev_desired_heading = 0.0
_last_direction      = 1.0
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
#   로 미러링해서 세팅해야 경계(boundary)/CLOSE 함수가 올바르게 동작한다.
arduino_x_mm        = 0.0
arduino_y_mm        = 0.0
arduino_heading_deg = 0.0
_boundary_center_x  = None
_boundary_center_y  = None


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
    global prev_desired_heading, _last_direction
    global stop_cycle_count, stop_pivot_w, stop_locked_target
    global stop_locked_gap, stop_locked_global_heading, stop_phase
    prev_desired_heading = 0.0
    _last_direction      = 1.0
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
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_front_90(angle_norm): continue
        horiz, fwd = decompose(angle_norm, dist)
        if STOP_FWD_MIN <= fwd <= STOP_FWD_MAX and horiz < STOP_HORIZ_TH:
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


def find_stop_escape_direction(scan_points, heading_deg=0.0):
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
    def cost(g):
        d_target = abs(((g['center_angle'] - target_bearing) + 180) % 360 - 180)
        d_prev   = abs(((g['center_angle'] - prev_heading)   + 180) % 360 - 180)
        return GAP_TARGET_WEIGHT * d_target + GAP_SMOOTH_WEIGHT * d_prev
    return min(passable_gaps, key=cost)


def is_target_blocked(scan_points, target_bearing):
    for a, d in scan_points:
        if LIDAR_MIN_VALID < d < TARGET_BLOCK_DIST and abs(a - target_bearing) < TARGET_CLEAR_CONE:
            return True
    return False


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
        w = KP_GOAL * desired_heading

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

        score_L = term_gap_L + term_push_L + term_side_L + term_head_L
        score_R = term_gap_R + term_push_R + term_side_R + term_head_R

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
        target, gap_width, gap_info = find_stop_escape_direction(scan_points, heading_deg)
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
    esc_target, esc_width, gap_info = find_stop_escape_direction(scan_points, heading_deg)

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
        x_t = arduino_x + dist*sin(bearing_global)
        y_t = arduino_y + dist*cos(bearing_global)
    """
    bearing_global_deg = arduino_heading_deg + close_bearing_deg
    hdg_rad = math.radians(bearing_global_deg)
    x_t = arduino_x_mm + dist_mm * math.sin(hdg_rad)
    y_t = arduino_y_mm + dist_mm * math.cos(hdg_rad)
    return x_t, y_t


def set_boundary_center():
    """색 미감지 전환 시 최초 1회 현재 위치를 경계 원 중심으로 설정 (이후 고정)."""
    global _boundary_center_x, _boundary_center_y
    if _boundary_center_x is None:
        _boundary_center_x = arduino_x_mm
        _boundary_center_y = arduino_y_mm


def clear_boundary_center():
    """색 감지 중 호출: 다음 소실 때 현재 위치가 새 기준이 되도록 리셋."""
    global _boundary_center_x, _boundary_center_y
    _boundary_center_x = None
    _boundary_center_y = None


def get_boundary_correction():
    """경계 초과 시 (중심 방향 상대 베어링[좌+], v 감속비율) 반환.
    jw_won._get_boundary_correction 와 동일. 경계 내부 → (0.0, 1.0)."""
    if _boundary_center_x is None:
        return 0.0, 1.0
    dx = _boundary_center_x - arduino_x_mm
    dy = _boundary_center_y - arduino_y_mm
    dist = math.sqrt(dx ** 2 + dy ** 2)
    if dist <= BOUNDARY_RADIUS:
        return 0.0, 1.0
    excess = dist - BOUNDARY_RADIUS
    blend  = min(excess / BOUNDARY_BLEND_DIST, 1.0)
    bearing_to_center = math.degrees(math.atan2(dx, dy))
    rel_bearing = normalize_angle(bearing_to_center - arduino_heading_deg)
    v_scale = BOUNDARY_V_MIN + (1.0 - BOUNDARY_V_MIN) * (1.0 - blend)
    return rel_bearing, v_scale


def reset_odom_state():
    """오도메트리/경계 전역 초기화 (재시작 시)."""
    global arduino_x_mm, arduino_y_mm, arduino_heading_deg
    global _boundary_center_x, _boundary_center_y
    arduino_x_mm = 0.0
    arduino_y_mm = 0.0
    arduino_heading_deg = 0.0
    _boundary_center_x = None
    _boundary_center_y = None
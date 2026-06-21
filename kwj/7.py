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

ROBOT_HALF_WIDTH = 115   # mm: 라이다 중심 ~ 좌우 끝

FORWARD_SPEED    = 0.3
MIN_SPEED        = 0.12
MAX_W            = 1.8
W_MIN_DANGER     = 0.5   # rad/s: 위험 시 최소 회전
W_SMOOTH         = 0.7

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 계층형 바운딩 박스 정의 (6개 레이어)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

LAYERS = [
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':140, 'w_gain':2.8, 'weight_base':0.8, 'weight_cap':7.5, 'weight_dynamic':True, 'v_max':0.2, 'affects_v':True},
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':120, 'w_gain':2.5, 'weight_base':0.6, 'weight_cap':5.0, 'weight_dynamic':True, 'v_max':0.25, 'affects_v':True},
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':120, 'w_gain':2.0, 'weight_base':0.4, 'weight_cap':4.5, 'weight_dynamic':True, 'affects_v':True},
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':110, 'w_gain':1.0, 'weight_base':0.1, 'weight_start':0.2, 'weight_dynamic':False, 'affects_v':True},
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':110, 'w_gain':0.4, 'weight_base':0.05,'weight_start':0.1, 'weight_dynamic':False, 'affects_v':False},
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':110, 'w_gain':0.3, 'weight_base':0.02,'weight_start':0.05,'weight_dynamic':False, 'affects_v':False},
]

LAYER_PERCENTILE = 5    

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP zone & FGM
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 200
STOP_HORIZ_TH = 115

STOP_ESCAPE_MIN_GAP   = ROBOT_HALF_WIDTH * 2 + 40   
STOP_MAX_CYCLES       = 30                          
STOP_PIVOT_MAX_W      = 0.9   
STOP_PIVOT_MIN_W      = 0.7   
STOP_PIVOT_SLOW_DEG   = 15    

FGM_MIN_ANG_DEG      = 3     
FGM_MIN_DEPTH_MM     = 250   
FGM_MAX_RANGE_MM     = 500   
FGM_RATIO_THRES      = 1.2   
FRONT_GAP_MIN_DEPTH  = 300   

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 방향 점수제 (gap + layer 통합)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCORE_ALPHA       = 5.0    
SCORE_BETA        = 8     
SCORE_SIDE        = 2500.0  
HEADING_WEIGHT_MM = 5.0    
DEPTH_JUMP_THRES  = 120    

DIRECTION_HYSTERESIS = 300.0

GAP_TARGET_WEIGHT = 1.0           
GAP_SMOOTH_WEIGHT = 0.3           
KP_GOAL            = MAX_W / 45.0  
TARGET_ALIGN_ANGLE = 60.0         
TARGET_CLEAR_CONE  = 18           
TARGET_BLOCK_DIST = 600           

SCAN_WIDE_HALF = 135   
SEND_INTERVAL  = 0.1

SIDE_SAFE_MARGIN  = 300   
SIDE_FWD_LEAD     = 90    
SIDE_FWD_REAR     = 90    
SIDE_REPULSE_GAIN = 1.25   
SIDE_EXP_K        = 2.0   

SIDE_LAYER_ANG_START = 15   
SIDE_LAYER_ANG_END   = 75   
SIDE_LAYER_DIST_MAX  = 700  
SIDE_W_BOOST_GAIN    = 1.5  

MIN_PASSAGE_WIDTH       = STOP_ESCAPE_MIN_GAP  
VIRTUAL_OBS_GAIN        = 1.5   
VIRTUAL_CENTER_DEADBAND = 10    
VIRTUAL_EXP_K           = 2.5   

# ★ 교체됨: 가상 경계 대신 나선형 배회(Spiral Search) 파라미터 적용
SPIRAL_MAX_RADIUS   = 1000.0   # mm: 나선 탐색 최대 반경 (1m)
SPIRAL_EXPAND_ANGLE = 30.0     # deg: 나선이 바깥으로 퍼지는 각도

# ── 디버그 토글 ──────────────────────────────────────────────────────────────
DEBUG_LAYERS = 0; DEBUG_STOP = 0; DEBUG_STOP_PIVOT = 0; DEBUG_BRANCH = 1
DEBUG_TARGET = 0; DEBUG_GAP = 1; DEBUG_FALLBACK = 0; DEBUG_SCORE = 0
DEBUG_DIR = 0; DEBUG_CLEAR = 0; DEBUG_FINAL = 1; DEBUG_SIDE = 0
DEBUG_SIDE_LAYER = 0; DEBUG_VIRTUAL = 0; DEBUG_CLOSE_INIT = 0; DEBUG_CLOSE_POS = 0
DEBUG_CLOSE_HDG = 0; DEBUG_CLOSE_DONE = 0; DEBUG_CLOSE_REMAIN = 0
DEBUG_BOUNDARY = 0; DEBUG_SEND = 0

# ── 전역 상태 ────────────────────────────────────────────────────────────────
arduino_heading_deg   = 0.0
arduino_x_mm          = 0.0   
arduino_y_mm          = 0.0   
prev_w                = 0.0

# ── CLOSE 접근 제어 ───────────────────────────────────────────────────────────
_close_target_x    = None   
_close_target_y    = None   
_close_initial_dist = None  
_close_observe_start = None 
KP_CLOSE_HDG      = 0.1  
CLOSE_SPEED_MAX   = 0.2   
CLOSE_SPEED_MIN   = 0.08  
CLOSE_ARRIVE_MM   = 10    
CLOSE_OBSERVE_SEC = 1.0   

prev_desired_heading  = 0.0   
_last_direction       = 1.0   
stop_cycle_count           = 0     
stop_pivot_w               = 0.0   
stop_locked_target         = 0.0
stop_locked_gap            = 0.0
stop_locked_global_heading = 0.0
stop_phase                 = 0     

_scan_lock   = threading.Lock()
_latest_scan = []            
_shutdown    = threading.Event()  

STOP_LOG_ENABLED  = False              
_stop_log_queue   = queue.Queue(maxsize=20)
_stop_log_counter = 0

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def normalize_angle(angle): return ((angle + 180) % 360) - 180
def is_in_front_90(a): return -90 <= a <= 90
def is_in_wide_scan(a): return -SCAN_WIDE_HALF <= a <= SCAN_WIDE_HALF
def decompose(angle_deg, dist):
    rad = math.radians(angle_deg)
    return abs(dist * math.sin(rad)), dist * math.cos(rad)
def cosine_dist(d1, d2, angle_diff_deg):
    theta = math.radians(abs(angle_diff_deg))
    return math.sqrt(d1**2 + d2**2 - 2 * d1 * d2 * math.cos(theta))
def point_to_segment_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    seg_sq = dx*dx + dy*dy
    if seg_sq == 0: return math.sqrt((px - ax)**2 + (py - ay)**2)
    t = max(0.0, min(1.0, ((px - ax)*dx + (py - ay)*dy) / seg_sq))
    return math.sqrt((px - ax - t*dx)**2 + (py - ay - t*dy)**2)
def nearest_to_segments(px, py, cluster_xy):
    if len(cluster_xy) == 1:
        ox, oy = cluster_xy[0]
        return math.sqrt((px - ox)**2 + (py - oy)**2)
    return min(point_to_segment_dist(px, py, cluster_xy[j][0], cluster_xy[j][1], cluster_xy[j+1][0], cluster_xy[j+1][1]) for j in range(len(cluster_xy) - 1))
def parse_packet(data):
    if len(data) != 5: return None
    s_flag = data[0] & 0x01
    if ((data[0] & 0x02) >> 1) != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None
    angle_q6 = (data[1] >> 1) | (data[2] << 7)
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
                    arduino_x_mm, arduino_y_mm, arduino_heading_deg = float(parts[0]), float(parts[1]), float(parts[2])
            elif line.startswith('H:'):
                arduino_heading_deg = float(line[2:])
        except Exception: pass

def _compute_close_target():
    bearing_global_deg = arduino_heading_deg + camera_tracker.get_last_close_bearing()
    dist_mm            = camera_tracker.get_estimated_distance_mm()
    hdg_rad            = math.radians(bearing_global_deg)
    return arduino_x_mm + dist_mm * math.sin(hdg_rad), arduino_y_mm + dist_mm * math.cos(hdg_rad)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# ★ 추가됨: 나선형 배회(Spiral Search) 함수
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def _get_spiral_search_bearing():
    """
    오도메트리 원점(최근 색지 또는 출발지) 기준으로 나선형 탐색.
    """
    r = math.sqrt(arduino_x_mm**2 + arduino_y_mm**2)
    
    if r < 150:
        # ★ 수정됨: 시작 직후 15cm 이내에서는 제자리 맴돎을 방지하고 
        # 살짝만(5도) 틀면서 시원하게 앞으로 뻗어나가도록 유도
        target_global_heading = arduino_heading_deg + 5.0 
    else:
        # 원점으로부터 현재 로봇이 위치한 각도
        angle_from_origin = math.degrees(math.atan2(arduino_x_mm, arduino_y_mm))
        
        if r < SPIRAL_MAX_RADIUS:
            # 15cm 밖으로 나오면 본격적인 나선 궤도(90도 + 팽창각) 시작
            expansion = SPIRAL_EXPAND_ANGLE * (1.0 - r / SPIRAL_MAX_RADIUS)
            target_global_heading = angle_from_origin + 90.0 + expansion
        else:
            # 1m 초과 시 원 안으로 다시 복귀
            correction = min(60.0, (r - SPIRAL_MAX_RADIUS) * 0.2)
            target_global_heading = angle_from_origin + 90.0 - correction
            
    rel_bearing = normalize_angle(target_global_heading - arduino_heading_deg)
    return rel_bearing, 1.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 라이다 장애물 인식 및 회피 연산 (기존 동일)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def detect_stop_zone(scan_points):
    for a, d in scan_points:
        if LIDAR_MIN_VALID <= d <= DETECTION_RANGE and is_in_front_90(a):
            h, f = decompose(a, d)
            if STOP_FWD_MIN <= f <= STOP_FWD_MAX and h < STOP_HORIZ_TH: return True
    return False

def find_all_gaps(scan_points):
    pts = sorted([(a, d) for a, d in scan_points if LIDAR_MIN_VALID < d < FGM_MAX_RANGE_MM], key=lambda p: p[0])
    if len(pts) < 2: return []
    def to_xy(a, d): r = math.radians(a); return d * math.sin(r), d * math.cos(r)
    gap_indices = []
    for i in range(len(pts) - 1):
        a1, d1 = pts[i]; a2, d2 = pts[i + 1]
        if abs(d2 - d1) > DEPTH_JUMP_THRES or (a2 - a1) >= FGM_MIN_ANG_DEG or (d2/d1 > FGM_RATIO_THRES) or (d1/d2 > FGM_RATIO_THRES):
            gap_indices.append(i)
    if not gap_indices: return []
    
    cluster_ids = []; cid = 0; gap_set = set(gap_indices)
    for i in range(len(pts)):
        cluster_ids.append(cid)
        if i in gap_set: cid += 1
    clusters_xy = [[] for _ in range(cid + 1)]
    for i, (a, d) in enumerate(pts): clusters_xy[cluster_ids[i]].append(to_xy(a, d))

    gaps = []
    for i in gap_indices:
        a1, d1 = pts[i]; a2, d2 = pts[i + 1]
        x1, y1 = to_xy(a1, d1); x2, y2 = to_xy(a2, d2)
        w = min(nearest_to_segments(x1, y1, clusters_xy[cluster_ids[i+1]]), nearest_to_segments(x2, y2, clusters_xy[cluster_ids[i]]))
        ca = math.degrees(math.atan2((x1 + x2)/2, (y1 + y2)/2))
        gaps.append({'width': w, 'center_angle': ca, 'edge_a': (a1, d1), 'edge_b': (a2, d2), 'depth': max(d1, d2)})
    return gaps

def choose_escape_gap(gaps, prefer_angle=0.0):
    passable = [g for g in gaps if g['width'] >= STOP_ESCAPE_MIN_GAP and g['depth'] >= FGM_MIN_DEPTH_MM]
    return min(passable, key=lambda g: abs(((g['center_angle'] - prefer_angle) + 180) % 360 - 180)) if passable else None

def find_stop_escape_direction(scan_points, heading_deg=0.0):
    gaps = find_all_gaps(scan_points)
    chosen = choose_escape_gap(gaps, prefer_angle=heading_deg)
    if not chosen: return 0.0, 0.0, []
    return float(chosen['center_angle']), float(chosen['width']), []

def process_layer(scan_points, layer):
    pts = []
    for a, d in scan_points:
        if LIDAR_MIN_VALID <= d <= DETECTION_RANGE and is_in_front_90(a):
            h, f = decompose(a, d)
            if layer['fwd_min'] <= f < layer['fwd_max'] and h < layer['horiz_th']:
                pts.append({'angle': a, 'dist': d, 'horiz': h, 'fwd': f, 'horiz_error': layer['horiz_th'] - h})
    if not pts: return None

    rep = sorted(pts, key=lambda p: p['dist'])[:max(1, int(len(pts) * LAYER_PERCENTILE / 100))]
    ra = sum(p['angle'] for p in rep) / len(rep)
    rh = sum(p['horiz'] for p in rep) / len(rep)
    rf = sum(p['fwd'] for p in rep) / len(rep)

    if layer['weight_dynamic']:
        cap = layer.get('weight_cap', 1.0)
        weight = max(layer['weight_base'], min(cap, (layer['horiz_th'] - rh) / layer['horiz_th'] * cap))
    else:
        progress = max(0.0, min(1.0, (rf - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min'])))
        weight = layer['weight_start'] + (layer['weight_base'] - layer['weight_start']) * progress

    v_prop = MIN_SPEED + (layer.get('v_max', FORWARD_SPEED) - MIN_SPEED) * max(0.0, min(1.0, (rf - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min']))) if layer['affects_v'] else None
    return {'name': layer['name'], 'weight': weight, 'urgency': layer['w_gain'] * (layer['horiz_th'] - rh) / layer['horiz_th'], 'v_proposal': v_prop, 'rep_angle': ra, 'rep_horiz': rh, 'rep_fwd': rf, 'push_left': sum(p['horiz_error'] for p in rep if p['angle'] < 0), 'push_right': sum(p['horiz_error'] for p in rep if p['angle'] > 0), 'n_points': len(pts)}

def get_gap_width(scan_points, ref_angle, ref_dist, is_left):
    front = [(a, d) for a, d in scan_points if is_in_front_90(a)]
    search = sorted([p for p in front if (p[0] < ref_angle if is_left else p[0] > ref_angle)], key=lambda x: x[0], reverse=is_left)
    if not search: return 0.0
    ep = (ref_angle, ref_dist)
    for i, p in enumerate(search):
        if abs(p[1] - ep[1]) > DEPTH_JUMP_THRES:
            return min(cosine_dist(ep[1], wp[1], abs(ep[0] - wp[0])) for wp in search[i:]) if search[i:] else 0.0
        ep = p
    rem = abs((-90 - ep[0]) if is_left else (90 - ep[0]))
    return cosine_dist(ep[1], ep[1], rem) if rem > 15 else 0.0

def get_side_repulsion(scan_points):
    ls, rs = 0.0, 0.0
    for a, d in scan_points:
        if LIDAR_MIN_VALID <= d <= DETECTION_RANGE and is_in_wide_scan(a):
            h, f = decompose(a, d)
            if -SIDE_FWD_REAR <= f <= SIDE_FWD_LEAD and ROBOT_HALF_WIDTH <= h < ROBOT_HALF_WIDTH + SIDE_SAFE_MARGIN:
                s = (math.exp(SIDE_EXP_K * (1.0 - (h - ROBOT_HALF_WIDTH) / SIDE_SAFE_MARGIN)) - 1.0) / (math.exp(SIDE_EXP_K) - 1.0)
                if a < 0: ls = max(ls, s)
                else: rs = max(rs, s)
    return (rs - ls) * SIDE_REPULSE_GAIN, ls, rs

def get_side_layer_push(scan_points):
    lp, rp = 0.0, 0.0
    for a, d in scan_points:
        if LIDAR_MIN_VALID <= d <= SIDE_LAYER_DIST_MAX:
            s = (SIDE_LAYER_DIST_MAX - d) / SIDE_LAYER_DIST_MAX
            if -SIDE_LAYER_ANG_END <= a <= -SIDE_LAYER_ANG_START: lp = max(lp, s)
            elif SIDE_LAYER_ANG_START <= a <= SIDE_LAYER_ANG_END: rp = max(rp, s)
    return lp, rp

def get_front_passable_gaps(scan_points):
    front = sorted([(a, d) for a, d in scan_points if is_in_front_90(a) and LIDAR_MIN_VALID < d < DETECTION_RANGE], key=lambda p: p[0])
    if len(front) < 2: return []
    def to_xy(a, d): r = math.radians(a); return d * math.sin(r), d * math.cos(r)
    edge_idx = [i for i in range(len(front)-1) if abs(front[i+1][1] - front[i][1]) > DEPTH_JUMP_THRES or (front[i+1][0] - front[i][0]) >= FGM_MIN_ANG_DEG or (front[i+1][1]/front[i][1] > FGM_RATIO_THRES) or (front[i][1]/front[i+1][1] > FGM_RATIO_THRES)]
    if not edge_idx: return []
    
    cids = []; cid = 0; gset = set(edge_idx)
    for i in range(len(front)):
        cids.append(cid)
        if i in gset: cid += 1
    c_xy = [[] for _ in range(cid + 1)]
    for i, (a, d) in enumerate(front): c_xy[cids[i]].append(to_xy(a, d))
    
    def depth_c(ca):
        b = min(front, key=lambda p: abs(p[0] - ca))
        return b[1] if abs(b[0] - ca) < 10.0 else FGM_MAX_RANGE_MM

    passable = []
    for i in edge_idx:
        x1, y1 = to_xy(*front[i]); x2, y2 = to_xy(*front[i+1])
        w = min(nearest_to_segments(x1, y1, c_xy[cids[i+1]]), nearest_to_segments(x2, y2, c_xy[cids[i]]))
        if w >= STOP_ESCAPE_MIN_GAP:
            ca = math.degrees(math.atan2((x1 + x2)/2, (y1 + y2)/2)); d = depth_c(ca)
            if d >= FRONT_GAP_MIN_DEPTH: passable.append({'center_angle': ca, 'width': w, 'depth': d, 'score': w * d})
    return sorted(passable, key=lambda g: g['score'], reverse=True)

def choose_target_gap(passable_gaps, target_bearing, prev_heading):
    if not passable_gaps: return None
    return min(passable_gaps, key=lambda g: GAP_TARGET_WEIGHT * abs(((g['center_angle'] - target_bearing) + 180) % 360 - 180) + GAP_SMOOTH_WEIGHT * abs(((g['center_angle'] - prev_heading) + 180) % 360 - 180))

def is_target_blocked(scan_points, target_bearing):
    return any(LIDAR_MIN_VALID < d < TARGET_BLOCK_DIST and abs(a - target_bearing) < TARGET_CLEAR_CONE for a, d in scan_points)

def get_narrow_gap_pushes(scan_points, layer, in_stop=False):
    if in_stop: return 0.0, 0.0
    pts = sorted([(a, d) for a, d in scan_points if LIDAR_MIN_VALID <= d <= DETECTION_RANGE and is_in_front_90(a) and layer['fwd_min'] <= decompose(a, d)[1] < layer['fwd_max']], key=lambda p: p[0])
    if len(pts) < 2: return 0.0, 0.0
    
    oe = [(pts[i][0], pts[i][1]) for i in range(len(pts)-1) if abs(pts[i+1][1] - pts[i][1]) > DEPTH_JUMP_THRES and pts[i+1][1] > pts[i][1]]
    ce = [(pts[i+1][0], pts[i+1][1]) for i in range(len(pts)-1) if abs(pts[i+1][1] - pts[i][1]) > DEPTH_JUMP_THRES and pts[i+1][1] <= pts[i][1]]
    if not oe or not ce: return 0.0, 0.0
    
    vpl, vpr = 0.0, 0.0
    for ao, do in oe:
        cands = [(ac, dc) for ac, dc in ce if ac > ao]
        if not cands: continue
        ac, dc = min(cands, key=lambda x: x[0])
        xo = do * math.sin(math.radians(ao)); xc = dc * math.sin(math.radians(ac))
        w = abs(xc - xo)
        if w >= MIN_PASSAGE_WIDTH: continue
        
        o_s = 0.0 if (abs(xo) < layer['horiz_th'] and abs(xc) < layer['horiz_th']) else (0.4 if (abs(xo) < layer['horiz_th'] or abs(xc) < layer['horiz_th']) else 1.0)
        if o_s == 0.0: continue
        
        t = max(0.0, min(1.0, w / MIN_PASSAGE_WIDTH))
        s = ((math.exp(VIRTUAL_EXP_K * (1.0 - t)) - 1.0) / (math.exp(VIRTUAL_EXP_K) - 1.0)) * layer['horiz_th'] * VIRTUAL_OBS_GAIN * o_s
        ca = (ao + ac) / 2.0
        
        if abs(ca) < VIRTUAL_CENTER_DEADBAND: vpl = max(vpl, s * 0.5); vpr = max(vpr, s * 0.5)
        elif ca < 0: vpl = max(vpl, s)
        else: vpr = max(vpr, s)
    return vpl, vpr

def find_vw_layered(scan_points, heading_deg, target_bearing=0.0):
    global _last_direction, prev_desired_heading
    l_res = [r for r in (process_layer(scan_points, l) for l in LAYERS) if r is not None]
    
    v_layers = [r for r in l_res if r['v_proposal'] is not None]
    v = sum(r['weight'] * r['v_proposal'] for r in v_layers) / sum(r['weight'] for r in v_layers) if v_layers else FORWARD_SPEED

    fg = get_front_passable_gaps(scan_points)
    cg = choose_target_gap(fg, target_bearing, prev_desired_heading)
    blocked = is_target_blocked(scan_points, target_bearing)
    slp, srp = get_side_layer_push(scan_points)

    if not blocked:
        prev_desired_heading = target_bearing
        w = KP_GOAL * target_bearing
        v = MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * max(0.0, 1.0 - abs(target_bearing) / TARGET_ALIGN_ANGLE)
    elif cg:
        prev_desired_heading = cg['center_angle']
        w = -KP_GOAL * cg['center_angle']
    elif l_res:
        closest = min(l_res, key=lambda r: r['rep_horiz'])
        ra, rd = closest['rep_angle'], math.sqrt(closest['rep_horiz']**2 + closest['rep_fwd']**2)
        gl, gr = get_gap_width(scan_points, ra, rd, True), get_gap_width(scan_points, ra, rd, False)
        
        spr = sum(r['weight'] * r['push_right'] for r in l_res)
        spl = sum(r['weight'] * r['push_left']  for r in l_res)
        
        vpl_t, vpr_t = 0.0, 0.0
        for l in LAYERS:
            vl, vr = get_narrow_gap_pushes(scan_points, l, in_stop=(stop_phase == 2))
            vpl_t = max(vpl_t, vl); vpr_t = max(vpr_t, vr)
            
        epr, epl = max(spr, vpr_t), max(spl, vpl_t)
        
        score_L = SCORE_ALPHA * gl + SCORE_BETA * epr + SCORE_SIDE * srp + max(0.0, -heading_deg) * HEADING_WEIGHT_MM
        score_R = SCORE_ALPHA * gr + SCORE_BETA * epl + SCORE_SIDE * slp + max(0.0, heading_deg) * HEADING_WEIGHT_MM
        
        sd = score_L - score_R
        d = 1.0 if sd > -DIRECTION_HYSTERESIS else -1.0 if _last_direction > 0 else -1.0 if sd < DIRECTION_HYSTERESIS else 1.0
        _last_direction = d
        
        wm = max(min(sum(r['weight'] * r['urgency'] for r in l_res) / sum(r['weight'] for r in l_res), MAX_W), W_MIN_DANGER)
        w = d * wm
    else:
        prev_desired_heading = target_bearing
        w = KP_GOAL * target_bearing

    w += (srp - slp) * SIDE_W_BOOST_GAIN
    sdw, _, _ = get_side_repulsion(scan_points)
    return v, max(min(w + sdw, MAX_W), -MAX_W)

def _stop_reset():
    global stop_cycle_count, stop_pivot_w, stop_phase, _last_direction
    stop_cycle_count = 0; stop_pivot_w = 0.0; stop_phase = 0; _last_direction = 1.0

def _stop_set_pivot(hdg, tgt, gw):
    global stop_locked_target, stop_locked_gap, stop_locked_global_heading, stop_pivot_w
    stop_locked_target = tgt; stop_locked_gap = gw
    if gw == 0:
        stop_locked_global_heading = 0.0
        stop_pivot_w = -math.copysign(MAX_W, hdg) if abs(hdg) > 1 else -MAX_W
    else:
        stop_locked_global_heading = ((hdg - tgt) + 180) % 360 - 180
        stop_pivot_w = -MAX_W if abs(tgt) < 5 else -math.copysign(MAX_W, tgt)

def find_vw_command(scan_points, heading_deg, target_bearing=0.0):
    global stop_cycle_count, stop_phase
    if stop_phase == 2:
        if not detect_stop_zone(scan_points):
            _stop_reset()
            return find_vw_layered(scan_points, heading_deg, target_bearing)
        stop_cycle_count += 1
        if stop_cycle_count >= STOP_MAX_CYCLES:
            _stop_reset()
            return find_vw_layered(scan_points, heading_deg, target_bearing)
        err = abs(((heading_deg - stop_locked_global_heading) + 180) % 360 - 180)
        speed = STOP_PIVOT_MIN_W + (STOP_PIVOT_MAX_W - STOP_PIVOT_MIN_W) * min(1.0, err / STOP_PIVOT_SLOW_DEG)
        return 0.0, math.copysign(speed, stop_pivot_w)

    if detect_stop_zone(scan_points):
        tgt, gw, gi = find_stop_escape_direction(scan_points, heading_deg)
        _stop_set_pivot(heading_deg, tgt, gw)
        stop_cycle_count = 0; stop_phase = 2
        _enqueue_stop_event(heading_deg, tgt, gw, gi, scan_points)
        return 0.0, stop_pivot_w

    return find_vw_layered(scan_points, heading_deg, target_bearing)

def _dedup_scan(pts):
    m = {}
    for a, d in pts:
        if d == 0: continue
        b = round(a)
        if b not in m or d < m[b]: m[b] = d
    return list(m.items())

def _enqueue_stop_event(hdg, tgt, gw, gi, sp):
    global _stop_log_counter
    if not STOP_LOG_ENABLED: return
    _stop_log_counter += 1
    try: _stop_log_queue.put_nowait((f'stop_event_{int(time.time())}_{_stop_log_counter:04d}.json', hdg, tgt, gw, gi, sp))
    except queue.Full: pass

def _stop_logger():
    while not _shutdown.is_set() or not _stop_log_queue.empty():
        try: item = _stop_log_queue.get(timeout=0.2)
        except queue.Empty: continue
        try:
            with open(item[0], 'w') as f: json.dump({'heading': item[1], 'target': item[2], 'gap_dist': item[3], 'gap_info': item[4], 'scan': [[a, d] for a, d in item[5] if d > 0]}, f)
        except Exception: pass
        finally: _stop_log_queue.task_done()

def _lidar_reader(lidar):
    buf = bytearray(); local_pts = []
    while not _shutdown.is_set():
        try:
            n = lidar.in_waiting
            chunk = lidar.read(n if n > 0 else 1)
        except Exception: continue
        if not chunk: continue
        buf.extend(chunk)

        i = 0; n_buf = len(buf)
        while n_buf - i >= 5:
            pkt = buf[i:i + 5]
            res = parse_packet(pkt)
            if res is None: i += 1; continue
            ar, d = res
            if (pkt[0] & 0x01) == 1 and local_pts:
                deduped = _dedup_scan(local_pts)
                with _scan_lock: _latest_scan.clear(); _latest_scan.extend(deduped)
                local_pts = []
            local_pts.append((normalize_angle(ar), d + LIDAR_OFFSET if d > 0 else 0))
            i += 5
        del buf[:i]

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 모터 제어 스레드 (감속 브레이크 + 나선 탐색 반영)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def _motor_controller(arduino):
    global prev_w, _close_target_x, _close_target_y, _close_initial_dist, _close_observe_start
    # ★ 추가됨: 나선 탐색을 위한 오도메트리 전역 변수
    global arduino_x_mm, arduino_y_mm, arduino_heading_deg 
    
    last_cmd_str = ""
    
    while not _shutdown.is_set():
        read_arduino(arduino)
        
        # ★ 추가됨: 카메라에서 미션 변경(색지 완료) 신호가 오면 오도메트리 강제 리셋
        if camera_tracker.check_mission_changed():
            arduino.write(b"R\n")
            arduino_x_mm = 0.0
            arduino_y_mm = 0.0
            arduino_heading_deg = 0.0
            print("[MAIN] 오도메트리 리셋! (새로운 나선 탐색 기준점 생성)")

        with _scan_lock:
            pts = [(a, d) for a, d in _latest_scan if d > 0]
            
        if pts:
            if camera_tracker.is_done() or camera_tracker.is_dwelling():
                v, w = 0.0, 0.0
                prev_w = 0.0
                _close_target_x = _close_target_y = _close_initial_dist = _close_observe_start = None

            elif camera_tracker.is_close() or _close_target_x is not None:
                if _close_target_x is None:
                    if _close_observe_start is None:
                        _close_observe_start = time.time()
                        print(f"[CLOSE] 관측 시작 — {CLOSE_OBSERVE_SEC:.1f}s 정지")
                        
                    elapsed = time.time() - _close_observe_start
                    remaining = CLOSE_OBSERVE_SEC - elapsed
                    
                    if remaining > 0:
                        v, w = 0.0, 0.0
                        prev_w = 0.0
                        w_smooth = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
                        cmd = f"{v:.2f} {w_smooth:.2f}\n"
                        arduino.write(cmd.encode())
                        time.sleep(SEND_INTERVAL)
                        continue
                        
                    _close_target_x, _close_target_y = _compute_close_target()
                    _close_initial_dist = None  

                ex = _close_target_x - arduino_x_mm
                ey = _close_target_y - arduino_y_mm
                dist_err = math.sqrt(ex ** 2 + ey ** 2)
                
                if _close_initial_dist is None:
                    _close_initial_dist = max(dist_err, 1.0)  

                if dist_err < CLOSE_ARRIVE_MM:
                    v, w = 0.0, 0.0
                    prev_w = 0.0
                    camera_tracker.signal_arrival()   
                else:
                    target_hdg = math.degrees(math.atan2(ex, ey))
                    hdg_err    = normalize_angle(target_hdg - arduino_heading_deg)

                    v_scale = min(1.0, dist_err / 150.0)
                    v = CLOSE_SPEED_MIN + (CLOSE_SPEED_MAX - CLOSE_SPEED_MIN) * v_scale
                    
                    w = max(min(KP_CLOSE_HDG * hdg_err, MAX_W), -MAX_W)
                    prev_w = w   

            else:
                if _close_target_x is not None:
                    _close_target_x = _close_target_y = _close_initial_dist = _close_observe_start = None
                    
                bearing = camera_tracker.get_bearing()
                if bearing is not None:
                    v, w = find_vw_command(pts, arduino_heading_deg, target_bearing=bearing)
                else:
                    # ★ 교체됨: 색 미감지 시 가상 경계 대신 나선형 배회(Spiral) 적용
                    spiral_tb, v_scale = _get_spiral_search_bearing()
                    v, w = find_vw_command(pts, arduino_heading_deg, target_bearing=spiral_tb)
                    v *= v_scale

            w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
            prev_w = w
            cmd = f"{v:.2f} {w:.2f}\n"
            arduino.write(cmd.encode())
            if cmd != last_cmd_str and DEBUG_SEND:
                last_cmd_str = cmd
                
        time.sleep(SEND_INTERVAL)

def main():
    print("=== System Starting (Spiral Search Enabled) ===")
    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)

    arduino.write(b"R\n")
    time.sleep(0.1)

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
        pass
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

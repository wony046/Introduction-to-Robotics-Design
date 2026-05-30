import serial
import time
import math
import json
import threading
import cv2
import numpy as np

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [1] 카메라 & 비전 파라미터 (회전 대응)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CAMERA_INDEX = 0
FRAME_WIDTH = 320   # 연산 속도를 위해 해상도 축소
FRAME_HEIGHT = 240
SHOW_CV_WINDOW = True # 테스트 시 화면 출력 (실전에서는 False 권장)

MIN_CONTOUR_AREA = 500  # 노이즈를 무시할 최소 픽셀 덩어리 크기
ARRIVE_Y_RATIO = 0.85   # 화면 세로의 85% 지점 아래로 색지가 내려오면 도착으로 간주

# 타겟 색상 (빨 -> 노 -> 파)
MISSION_COLORS = ['RED', 'YELLOW', 'BLUE']
# 조명에 맞게 반드시 튜닝해야 하는 HSV 임계값
COLOR_HSV_RANGES = {
    'RED':    [(0, 100, 100), (10, 255, 255), (160, 100, 100), (180, 255, 255)],
    'YELLOW': [(20, 100, 100), (35, 255, 255)],
    'BLUE':   [(100, 100, 50), (130, 255, 255)]
}

SCORE_COLOR_TARGET = 5000.0  # 색지 발견 시 목표 방향으로 끌어당기는 압도적 점수
SCORE_EXPLORE_BIAS = 600.0   # 색지가 없을 때 완만하게 왼쪽으로 회전하며 탐색(벽타기)하게 만드는 점수

# ── 전역 비전 상태 (스레드 공유) ─────────────────────────
_cam_lock = threading.Lock()
is_color_visible = False
camera_target_error_x = 0.0  
color_bottom_y = 0           
current_color_idx = 0        
mission_phase = 0            # 0: 탐색/접근, 1: 색지 위 도착(대기 중)
arrive_time = 0.0

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [2] 포트 & 라이다/로봇 파라미터 (과제 2 원본)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

LIDAR_OFFSET    = 10     
LIDAR_MIN_VALID = 100   
DETECTION_RANGE = 1500  

ROBOT_HALF_WIDTH = 110   
FORWARD_SPEED    = 0.45
MIN_SPEED        = 0.12
MAX_W            = 1.8
W_MIN_DANGER     = 0.5   
W_SMOOTH         = 0.7

LAYERS = [
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':140, 'w_gain':2.8, 'weight_base':0.8, 'weight_cap':7.5, 'weight_dynamic':True, 'v_max':0.22, 'affects_v':True},
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':120, 'w_gain':2.5, 'weight_base':0.6, 'weight_cap':5.0, 'weight_dynamic':True, 'v_max':0.38, 'affects_v':True},
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':120, 'w_gain':2.0, 'weight_base':0.4, 'weight_cap':4.5, 'weight_dynamic':True, 'affects_v':True},
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':110, 'w_gain':1.0, 'weight_base':0.1, 'weight_start':0.2, 'weight_dynamic':False, 'affects_v':True},
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':110, 'w_gain':0.4, 'weight_base':0.05,'weight_start':0.1, 'weight_dynamic':False, 'affects_v':False},
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':110, 'w_gain':0.3, 'weight_base':0.02,'weight_start':0.05,'weight_dynamic':False, 'affects_v':False},
]

LAYER_PERCENTILE = 5    
STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 175
STOP_HORIZ_TH = 105
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
SCORE_GAP_FRONT      = 900.0 

SCORE_ALPHA       = 5.0    
SCORE_BETA        = 8      
SCORE_SIDE        = 2500.0  
HEADING_WEIGHT_MM = 5.0    
DEPTH_JUMP_THRES  = 120    
DIRECTION_HYSTERESIS = 300.0

SCAN_WIDE_HALF = 135   
SEND_INTERVAL  = 0.1
SIDE_SAFE_MARGIN  = 300   
SIDE_FWD_LEAD     = 90    
SIDE_FWD_REAR     = 90    
SIDE_REPULSE_GAIN = 1.25   
SIDE_EXP_K        = 2.0   
SIDE_LAYER_ANG_START = 15   
SIDE_LAYER_ANG_END   = 75   
SIDE_LAYER_DIST_MAX  = 600  
SIDE_W_BOOST_GAIN    = 1.5  

MIN_PASSAGE_WIDTH       = STOP_ESCAPE_MIN_GAP  
VIRTUAL_OBS_GAIN        = 1.5   
VIRTUAL_CENTER_DEADBAND = 10    
VIRTUAL_EXP_K           = 2.5   

DEBUG_LAYERS  = False
DEBUG_STOP    = True
DEBUG_DIR     = False
DEBUG_FINAL   = True
DEBUG_SIDE    = False
DEBUG_VIRTUAL = False

arduino_heading_deg   = 0.0
prev_w                = 0.0
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

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [3] 유틸리티 및 라이다 연산 (과제 2 원본 유지)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def normalize_angle(angle): return angle - 360 if angle > 180 else angle
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
        return math.sqrt((px - cluster_xy[0][0])**2 + (py - cluster_xy[0][1])**2)
    return min(point_to_segment_dist(px, py, cluster_xy[j][0], cluster_xy[j][1], cluster_xy[j+1][0], cluster_xy[j+1][1]) for j in range(len(cluster_xy) - 1))
def parse_packet(data):
    if len(data) != 5: return None
    s_flag, s_inv_flag = data[0] & 0x01, (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None
    return (((data[1] >> 1) | (data[2] << 7)) / 64.0), ((data[3] | (data[4] << 8)) / 4.0)
def read_arduino(arduino):
    global arduino_heading_deg
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'): arduino_heading_deg = float(line[2:])
        except: pass

def detect_stop_zone(scan_points):
    for a, d in scan_points:
        if d < LIDAR_MIN_VALID or d > DETECTION_RANGE: continue
        if not is_in_front_90(a): continue
        horiz, fwd = decompose(a, d)
        if STOP_FWD_MIN <= fwd <= STOP_FWD_MAX and horiz < STOP_HORIZ_TH: return True
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
    gap_set, cluster_ids, cid = set(gap_indices), [], 0
    for i in range(len(pts)):
        cluster_ids.append(cid)
        if i in gap_set: cid += 1
    clusters_xy = [[] for _ in range(cid + 1)]
    for i, (a, d) in enumerate(pts): clusters_xy[cluster_ids[i]].append(to_xy(a, d))
    gaps = []
    for i in gap_indices:
        a1, d1 = pts[i]; a2, d2 = pts[i + 1]
        x1, y1 = to_xy(a1, d1); x2, y2 = to_xy(a2, d2)
        w = min(nearest_to_segments(x1, y1, clusters_xy[cluster_ids[i + 1]]), nearest_to_segments(x2, y2, clusters_xy[cluster_ids[i]]))
        ca = math.degrees(math.atan2((x1 + x2) / 2, (y1 + y2) / 2))
        gaps.append({'width': w, 'center_angle': ca, 'edge_a': (a1, d1), 'edge_b': (a2, d2), 'depth': max(d1, d2)})
    return gaps

def choose_escape_gap(gaps, prefer_angle=0.0):
    passable = [g for g in gaps if g['width'] >= STOP_ESCAPE_MIN_GAP and g['depth'] >= FGM_MIN_DEPTH_MM]
    if passable: return min(passable, key=lambda g: abs(((g['center_angle'] - prefer_angle) + 180) % 360 - 180))
    return None

def find_stop_escape_direction(scan_points, heading_deg=0.0):
    gaps = find_all_gaps(scan_points)
    chosen = choose_escape_gap(gaps, prefer_angle=heading_deg)
    if chosen is None: return 0.0, 0.0, []
    return float(chosen['center_angle']), float(chosen['width']), []

def process_layer(scan_points, layer):
    pts = []
    for a, d in scan_points:
        if d < LIDAR_MIN_VALID or d > DETECTION_RANGE: continue
        if not is_in_front_90(a): continue
        horiz, fwd = decompose(a, d)
        if layer['fwd_min'] <= fwd < layer['fwd_max'] and horiz < layer['horiz_th']:
            pts.append({'angle': a, 'dist': d, 'horiz': horiz, 'fwd': fwd, 'horiz_error': layer['horiz_th'] - horiz})
    if not pts: return None
    rep = sorted(pts, key=lambda p: p['dist'])[:max(1, int(len(pts) * LAYER_PERCENTILE / 100))]
    ra, rh, rf = sum(p['angle'] for p in rep)/len(rep), sum(p['horiz'] for p in rep)/len(rep), sum(p['fwd'] for p in rep)/len(rep)
    if layer['weight_dynamic']:
        cap = layer.get('weight_cap', 1.0)
        weight = max(layer['weight_base'], min(cap, (layer['horiz_th']-rh) / layer['horiz_th'] * cap))
    else:
        prog = max(0.0, min(1.0, (rf - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min'])))
        weight = layer['weight_start'] + (layer['weight_base'] - layer['weight_start']) * prog
    urgency = layer['w_gain'] * (layer['horiz_th']-rh) / layer['horiz_th']
    v_prop = MIN_SPEED + (layer.get('v_max', FORWARD_SPEED) - MIN_SPEED) * max(0.0, min(1.0, (rf - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min']))) if layer['affects_v'] else None
    return {'name': layer['name'], 'weight': weight, 'urgency': urgency, 'v_proposal': v_prop,
            'rep_angle': ra, 'rep_horiz': rh, 'rep_fwd': rf,
            'push_left': sum(p['horiz_error'] for p in rep if p['angle'] < 0),
            'push_right': sum(p['horiz_error'] for p in rep if p['angle'] > 0), 'n_points': len(pts)}

def get_gap_width(scan_points, ref_angle, ref_dist, is_left):
    front = [(a, d) for a, d in scan_points if is_in_front_90(a)]
    search = sorted([p for p in front if (p[0] < ref_angle if is_left else p[0] > ref_angle)], key=lambda x: x[0], reverse=is_left)
    if not search: return 0.0
    edge_p = (ref_angle, ref_dist)
    for p in search:
        if abs(p[1] - edge_p[1]) > DEPTH_JUMP_THRES: return cosine_dist(edge_p[1], p[1], abs(edge_p[0] - p[0]))
        edge_p = p
    rem_angle = abs((-90 - edge_p[0]) if is_left else (90 - edge_p[0]))
    return cosine_dist(edge_p[1], edge_p[1], rem_angle) if rem_angle > 15 else 0.0

def get_side_repulsion(scan_points):
    ls, rs = 0.0, 0.0
    for a, d in scan_points:
        if d < LIDAR_MIN_VALID or d > DETECTION_RANGE or not is_in_wide_scan(a): continue
        h, f = decompose(a, d)
        if f > SIDE_FWD_LEAD or f < -SIDE_FWD_REAR or h < ROBOT_HALF_WIDTH or h >= ROBOT_HALF_WIDTH + SIDE_SAFE_MARGIN: continue
        t = (h - ROBOT_HALF_WIDTH) / SIDE_SAFE_MARGIN
        st = (math.exp(SIDE_EXP_K * (1.0 - t)) - 1.0) / (math.exp(SIDE_EXP_K) - 1.0)
        if a < 0: ls = max(ls, st)
        else: rs = max(rs, st)
    return (rs - ls) * SIDE_REPULSE_GAIN, ls, rs

def get_side_layer_push(scan_points):
    lp, rp = 0.0, 0.0
    for a, d in scan_points:
        if d < LIDAR_MIN_VALID or d > SIDE_LAYER_DIST_MAX: continue
        st = (SIDE_LAYER_DIST_MAX - d) / SIDE_LAYER_DIST_MAX
        if -SIDE_LAYER_ANG_END <= a <= -SIDE_LAYER_ANG_START: lp = max(lp, st)
        elif SIDE_LAYER_ANG_START <= a <= SIDE_LAYER_ANG_END: rp = max(rp, st)
    return lp, rp

def get_narrow_gap_pushes(scan_points, layer, in_stop=False):
    if in_stop: return 0.0, 0.0
    pts = sorted([(a, d) for a, d in scan_points if LIDAR_MIN_VALID < d < DETECTION_RANGE and is_in_front_90(a) and layer['fwd_min'] <= decompose(a, d)[1] < layer['fwd_max']], key=lambda p: p[0])
    if len(pts) < 2: return 0.0, 0.0
    oe, ce = [], []
    for i in range(len(pts) - 1):
        if abs(pts[i+1][1] - pts[i][1]) > DEPTH_JUMP_THRES:
            if pts[i+1][1] > pts[i][1]: oe.append(pts[i])
            else: ce.append(pts[i+1])
    vl, vr = 0.0, 0.0
    for ao, do in oe:
        cands = [(ac, dc) for ac, dc in ce if ac > ao]
        if not cands: continue
        ac, dc = min(cands, key=lambda x: x[0])
        xo, xc = do * math.sin(math.radians(ao)), dc * math.sin(math.radians(ac))
        gw = abs(xc - xo)
        if gw >= MIN_PASSAGE_WIDTH: continue
        os = 0.0 if (abs(xo) < layer['horiz_th'] and abs(xc) < layer['horiz_th']) else (0.4 if (abs(xo) < layer['horiz_th'] or abs(xc) < layer['horiz_th']) else 1.0)
        if os == 0.0: continue
        t = max(0.0, min(1.0, gw / MIN_PASSAGE_WIDTH))
        st = ((math.exp(VIRTUAL_EXP_K * (1.0 - t)) - 1.0) / (math.exp(VIRTUAL_EXP_K) - 1.0)) * layer['horiz_th'] * VIRTUAL_OBS_GAIN * os
        ca = (ao + ac) / 2.0
        if abs(ca) < VIRTUAL_CENTER_DEADBAND: vl, vr = max(vl, st*0.5), max(vr, st*0.5)
        elif ca < 0: vl = max(vl, st)
        else: vr = max(vr, st)
    return vl, vr

def get_front_passable_gaps(scan_points):
    return []  # 이 부분은 단순화를 위해 생략해도 주행에 큰 지장이 없어 안전하게 빈 리스트 처리

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [4] 비전 통합 주행 제어 (V/W 산출)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def find_vw_layered(scan_points, heading_deg):
    global is_color_visible, camera_target_error_x, _last_direction
    
    layer_results = [r for l in LAYERS if (r := process_layer(scan_points, l)) is not None]
    if not layer_results: return FORWARD_SPEED, 0.0

    closest = min(layer_results, key=lambda r: r['rep_horiz'])
    ref_a = closest['rep_angle']
    ref_d = math.sqrt(closest['rep_horiz']**2 + closest['rep_fwd']**2)

    gap_L = get_gap_width(scan_points, ref_a, ref_d, True)
    gap_R = get_gap_width(scan_points, ref_a, ref_d, False)
    
    sum_pR = sum(r['weight'] * r['push_right'] for r in layer_results)
    sum_pL = sum(r['weight'] * r['push_left']  for r in layer_results)

    vR_tot, vL_tot = 0.0, 0.0
    for l in LAYERS:
        vl, vr = get_narrow_gap_pushes(scan_points, l, in_stop=(stop_phase == 2))
        vL_tot, vR_tot = max(vL_tot, vl), max(vR_tot, vr)

    eff_pR, eff_pL = max(sum_pR, vR_tot), max(sum_pL, vL_tot)
    side_l_push, side_r_push = get_side_layer_push(scan_points)

    t_gL = SCORE_ALPHA * gap_L
    t_gR = SCORE_ALPHA * gap_R
    t_pL = SCORE_BETA  * eff_pR
    t_pR = SCORE_BETA  * eff_pL
    t_sL = SCORE_SIDE  * side_r_push
    t_sR = SCORE_SIDE  * side_l_push
    t_hL = max(0.0, -heading_deg) * HEADING_WEIGHT_MM
    t_hR = max(0.0,  heading_deg) * HEADING_WEIGHT_MM

    # [핵심] 비전 기반 점수 개입
    term_color_L, term_color_R = 0.0, 0.0
    term_explore_L, term_explore_R = 0.0, 0.0

    with _cam_lock:
        visible = is_color_visible
        cam_err_x = camera_target_error_x

    if visible:
        if cam_err_x < 0: term_color_L = SCORE_COLOR_TARGET * abs(cam_err_x)
        else:             term_color_R = SCORE_COLOR_TARGET * abs(cam_err_x)
    else:
        term_explore_L = SCORE_EXPLORE_BIAS

    score_L = t_gL + t_pL + t_sL + t_hL + term_color_L + term_explore_L
    score_R = t_gR + t_pR + t_sR + t_hR + term_color_R + term_explore_R

    s_diff = score_L - score_R
    if _last_direction > 0: direction = 1.0 if s_diff > -DIRECTION_HYSTERESIS else -1.0
    else: direction = -1.0 if s_diff < DIRECTION_HYSTERESIS else 1.0
    _last_direction = direction

    v_layers = [r for r in layer_results if r['v_proposal'] is not None]
    if v_layers: v = sum(r['weight'] * r['v_proposal'] for r in v_layers) / sum(r['weight'] for r in v_layers)
    else: v = FORWARD_SPEED

    w_mag = sum(r['weight'] * r['urgency'] for r in layer_results) / sum(r['weight'] for r in layer_results)
    w_mag = max(min(w_mag, MAX_W), W_MIN_DANGER)
    
    w = direction * w_mag + (side_r_push - side_l_push) * SIDE_W_BOOST_GAIN
    side_dw, _, _ = get_side_repulsion(scan_points)
    w = max(min(w + side_dw, MAX_W), -MAX_W)

    return v, w

def _stop_reset():
    global stop_cycle_count, stop_pivot_w, stop_phase, _last_direction
    stop_cycle_count, stop_pivot_w, stop_phase, _last_direction = 0, 0.0, 0, 1.0

def find_vw_command(scan_points, heading_deg):
    global stop_cycle_count, stop_pivot_w, stop_phase, mission_phase, arrive_time, current_color_idx

    if mission_phase == 1:
        if time.time() - arrive_time > 2.0:
            current_color_idx += 1          
            mission_phase = 0               
            print(f"[MISSION] Next Target: {MISSION_COLORS[current_color_idx % len(MISSION_COLORS)]}")
        return 0.0, 0.0

    with _cam_lock:
        bottom_y = color_bottom_y
        visible = is_color_visible
    
    if visible and bottom_y > (FRAME_HEIGHT * ARRIVE_Y_RATIO):
        print(f"[MISSION] ARRIVED at {MISSION_COLORS[current_color_idx % len(MISSION_COLORS)]}!")
        mission_phase = 1
        arrive_time = time.time()
        return 0.0, 0.0

    if stop_phase == 2:
        if not detect_stop_zone(scan_points):
            _stop_reset()
            return find_vw_layered(scan_points, heading_deg)
        stop_cycle_count += 1
        if stop_cycle_count >= STOP_MAX_CYCLES:
            _stop_reset()
            return find_vw_layered(scan_points, heading_deg)
        return 0.0, stop_pivot_w

    if detect_stop_zone(scan_points):
        target, gap_width, _ = find_stop_escape_direction(scan_points, heading_deg)
        stop_pivot_w = -MAX_W if abs(target) < 5 else -math.copysign(MAX_W, target)
        stop_cycle_count, stop_phase = 0, 2
        return 0.0, stop_pivot_w

    return find_vw_layered(scan_points, heading_deg)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [5] 스레드: 비전, 라이다, 아두이노 모터
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def _camera_processor():
    global is_color_visible, camera_target_error_x, color_bottom_y, current_color_idx
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    kernel = np.ones((5,5), np.uint8)

    while not _shutdown.is_set():
        ret, raw_frame = cap.read()
        if not ret: continue

        frame = cv2.rotate(raw_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        h, w, _ = frame.shape 

        target_name = MISSION_COLORS[current_color_idx % len(MISSION_COLORS)]
        hsv_ranges = COLOR_HSV_RANGES[target_name]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        mask = None
        if target_name == 'RED':
            mask1 = cv2.inRange(hsv, hsv_ranges[0], hsv_ranges[1])
            mask2 = cv2.inRange(hsv, hsv_ranges[2], hsv_ranges[3])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, hsv_ranges[0], hsv_ranges[1])

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        found = False
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                x, y, box_w, box_h = cv2.boundingRect(c)
                cx = x + box_w // 2
                err_x = (cx - (w / 2)) / (w / 2)
                bottom_y = y + box_h
                
                with _cam_lock:
                    is_color_visible = True
                    camera_target_error_x = err_x
                    color_bottom_y = bottom_y
                found = True

                if SHOW_CV_WINDOW:
                    cv2.rectangle(frame, (x, y), (x+box_w, y+box_h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, y+box_h), 5, (0, 0, 255), -1)
                    arrive_line_y = int(h * ARRIVE_Y_RATIO)
                    cv2.line(frame, (0, arrive_line_y), (w, arrive_line_y), (255, 0, 0), 2)

        if not found:
            with _cam_lock: is_color_visible = False

        if SHOW_CV_WINDOW:
            cv2.putText(frame, f"TARGET: {target_name}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.imshow("Robot Vision (Rotated)", frame)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)
            
    cap.release()
    cv2.destroyAllWindows()

def _dedup_scan(pts):
    angle_map = {}
    for angle, dist in pts:
        bucket = round(angle)
        if bucket not in angle_map or dist < angle_map[bucket]: angle_map[bucket] = dist
    return list(angle_map.items())

def _lidar_reader(lidar):
    local_pts = []
    while not _shutdown.is_set():
        try: raw = lidar.read(5)
        except: continue
        result = parse_packet(raw)
        if result is None: continue
        angle_raw, distance = result
        if (raw[0] & 0x01) == 1 and local_pts:
            deduped = _dedup_scan(local_pts)
            with _scan_lock:
                _latest_scan.clear()
                _latest_scan.extend(deduped)
            local_pts = []
        local_pts.append((normalize_angle(angle_raw), distance + LIDAR_OFFSET if distance > 0 else 0))

def _motor_controller(arduino):
    global prev_w
    while not _shutdown.is_set():
        read_arduino(arduino)
        with _scan_lock:
            pts = [(a, d) for a, d in _latest_scan if d > 0]
        if pts:
            v, w = find_vw_command(pts, arduino_heading_deg)
            w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
            prev_w = w
            arduino.write(f"{v:.2f} {w:.2f}\n".encode())
        time.sleep(SEND_INTERVAL)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [6] 메인 실행 (이 부분이 가장 중요합니다!)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def main():
    print("=== Robot Navigation + Vision Target ===")
    
    try:
        lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    except Exception as e:
        print(f"포트 연결 에러: {e}")
        return

    time.sleep(2)
    arduino.write(b"R\n")
    time.sleep(0.1)
    
    lidar.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))
    lidar.read(7)

    t_lidar  = threading.Thread(target=_lidar_reader, args=(lidar,), daemon=True)
    t_motor  = threading.Thread(target=_motor_controller, args=(arduino,), daemon=True)
    t_camera = threading.Thread(target=_camera_processor, daemon=True) 

    try:
        t_lidar.start()
        t_motor.start()
        t_camera.start()
        print("모든 시스템이 정상 작동 중입니다. Ctrl+C를 눌러 종료하세요.")
        while not _shutdown.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n종료 신호를 받았습니다...")
    finally:
        _shutdown.set()
        t_lidar.join(timeout=2.0)
        t_motor.join(timeout=2.0)
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        lidar.close()
        arduino.write(b"0.00 0.00\n")
        arduino.close()
        print("정상적으로 시스템을 종료했습니다.")

# ⬇️ 파이썬 스크립트 실행의 심장! ⬇️
if __name__ == "__main__":
    main()

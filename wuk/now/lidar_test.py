#!/usr/bin/env python3
"""
LIDAR 실시간 테스트 — 거리/각도 그래프 + 계산된 v/w 표시
Usage: python3 lidar_test.py
"""
import serial, time, math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── 설정 (jw_won.py 와 동일하게 유지) ────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
BAUDRATE_LIDAR   = 460800
LIDAR_MIN_VALID  = 100
DETECTION_RANGE  = 1500
ROBOT_HALF_WIDTH = 110

FORWARD_SPEED = 0.35
MIN_SPEED     = 0.07
MAX_W         = 2.0
W_MIN_DANGER  = 0.5
W_SMOOTH      = 0.45

STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 180
STOP_HORIZ_TH = 105

SCORE_ALPHA       = 5.0
SCORE_BETA        = 20
HEADING_WEIGHT_MM = 5.0
DEPTH_JUMP_THRES  = 120
LAYER_PERCENTILE  = 5
UPDATE_INTERVAL   = 150   # ms

LAYERS = [
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':200,
     'w_gain':2.8, 'weight_base':0.8, 'weight_dynamic':True,  'affects_v':True},
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':190,
     'w_gain':2.5, 'weight_base':0.6, 'weight_dynamic':True,  'affects_v':True},
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':140,
     'w_gain':1.8, 'weight_base':0.2, 'weight_start':0.4, 'weight_dynamic':False, 'affects_v':True},
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':140,
     'w_gain':1.0, 'weight_base':0.1, 'weight_start':0.2, 'weight_dynamic':False, 'affects_v':True},
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':120,
     'w_gain':0.4, 'weight_base':0.05,'weight_start':0.1, 'weight_dynamic':False, 'affects_v':False},
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':100,
     'w_gain':0.3, 'weight_base':0.02,'weight_start':0.05,'weight_dynamic':False, 'affects_v':False},
]

# ── 유틸 (jw_won.py 와 동일) ──────────────────────────────────────────────────
def normalize_angle(a):
    return a - 360 if a > 180 else a

def is_in_front_90(a):
    return -90 <= a <= 90

def decompose(angle_deg, dist):
    rad = math.radians(angle_deg)
    return abs(dist * math.sin(rad)), dist * math.cos(rad)

def cosine_dist(d1, d2, ang):
    t = math.radians(abs(ang))
    return math.sqrt(d1**2 + d2**2 - 2*d1*d2*math.cos(t))

def parse_packet(data):
    if len(data) != 5: return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    distance_q2 = data[3] | (data[4] << 8)
    return angle_q6 / 64.0, distance_q2 / 4.0

# ── v/w 계산 (jw_won.py 핵심 로직 복사) ──────────────────────────────────────
def process_layer(scan_points, layer):
    pts = []
    for a, d in scan_points:
        if d < LIDAR_MIN_VALID or d > DETECTION_RANGE: continue
        if not is_in_front_90(a): continue
        horiz, fwd = decompose(a, d)
        if layer['fwd_min'] <= fwd < layer['fwd_max'] and horiz < layer['horiz_th']:
            pts.append({'angle':a,'dist':d,'horiz':horiz,'fwd':fwd,
                        'horiz_error': layer['horiz_th'] - horiz})
    if not pts: return None
    n_take = max(1, int(len(pts) * LAYER_PERCENTILE / 100))
    rep = sorted(pts, key=lambda p: p['dist'])[:n_take]
    rep_angle = sum(p['angle'] for p in rep) / len(rep)
    rep_horiz = sum(p['horiz'] for p in rep) / len(rep)
    rep_fwd   = sum(p['fwd']   for p in rep) / len(rep)
    rep_h_err = layer['horiz_th'] - rep_horiz
    if layer['weight_dynamic']:
        weight = max(layer['weight_base'], min(1.0, rep_h_err / layer['horiz_th']))
    else:
        progress = max(0.0, min(1.0, (rep_fwd - layer['fwd_min']) /
                                     (layer['fwd_max'] - layer['fwd_min'])))
        weight = layer['weight_start'] + (layer['weight_base'] - layer['weight_start']) * progress
    urgency = layer['w_gain'] * rep_h_err / layer['horiz_th']
    if layer['affects_v']:
        progress = max(0.0, min(1.0, (rep_fwd - layer['fwd_min']) /
                                     (layer['fwd_max'] - layer['fwd_min'])))
        v_proposal = MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * progress
    else:
        v_proposal = None
    push_left  = sum(p['horiz_error'] for p in rep if p['angle'] < 0)
    push_right = sum(p['horiz_error'] for p in rep if p['angle'] > 0)
    return {'weight':weight,'urgency':urgency,'v_proposal':v_proposal,
            'rep_angle':rep_angle,'rep_horiz':rep_horiz,'rep_fwd':rep_fwd,
            'push_left':push_left,'push_right':push_right,'name':layer['name']}

def get_gap_width(scan_points, ref_angle, ref_dist, is_left):
    front = [(a, d) for a, d in scan_points if is_in_front_90(a)]
    if is_left:
        search = sorted([p for p in front if p[0] < ref_angle], key=lambda x: x[0], reverse=True)
    else:
        search = sorted([p for p in front if p[0] > ref_angle], key=lambda x: x[0])
    if not search: return 0.0
    edge_p = (ref_angle, ref_dist)
    for i, p in enumerate(search):
        if abs(p[1] - edge_p[1]) > DEPTH_JUMP_THRES:
            wall = search[i:]
            if wall:
                return min(cosine_dist(edge_p[1], wp[1], abs(edge_p[0] - wp[0])) for wp in wall)
        edge_p = p
    rem_angle = abs((-90 - edge_p[0]) if is_left else (90 - edge_p[0]))
    if rem_angle > 15:
        return cosine_dist(edge_p[1], edge_p[1], rem_angle)
    return 0.0

def compute_vw(scan_points, heading_deg=0.0):
    """jw_won.py find_vw_layered 간소화 버전 (STOP 제외)"""
    layer_results = [r for layer in LAYERS
                     for r in [process_layer(scan_points, layer)] if r]
    if not layer_results:
        return FORWARD_SPEED, 0.0, 'no layers', 0.0, 0.0

    closest  = min(layer_results, key=lambda r: r['rep_horiz'])
    ref_angle = closest['rep_angle']
    ref_dist  = math.sqrt(closest['rep_horiz']**2 + closest['rep_fwd']**2)
    gap_L = get_gap_width(scan_points, ref_angle, ref_dist, is_left=True)
    gap_R = get_gap_width(scan_points, ref_angle, ref_dist, is_left=False)

    sum_pR = sum(r['weight'] * r['push_right'] for r in layer_results)
    sum_pL = sum(r['weight'] * r['push_left']  for r in layer_results)
    score_L = SCORE_ALPHA*gap_L + SCORE_BETA*sum_pR + max(0,-heading_deg)*HEADING_WEIGHT_MM
    score_R = SCORE_ALPHA*gap_R + SCORE_BETA*sum_pL + max(0, heading_deg)*HEADING_WEIGHT_MM
    direction = 1.0 if score_L >= score_R else -1.0

    v_layers = [r for r in layer_results if r['v_proposal'] is not None]
    v = (sum(r['weight']*r['v_proposal'] for r in v_layers) /
         sum(r['weight'] for r in v_layers)) if v_layers else FORWARD_SPEED

    total_w = sum(r['weight'] for r in layer_results)
    w_mag = sum(r['weight']*r['urgency'] for r in layer_results) / total_w
    w_mag = max(min(w_mag, MAX_W), W_MIN_DANGER)
    w = direction * w_mag
    dir_str = 'L' if direction > 0 else 'R'
    return v, w, dir_str, gap_L, gap_R

def detect_stop(scan_points):
    for a, d in scan_points:
        if d < LIDAR_MIN_VALID or d > DETECTION_RANGE: continue
        if not is_in_front_90(a): continue
        h, f = decompose(a, d)
        if STOP_FWD_MIN <= f <= STOP_FWD_MAX and h < STOP_HORIZ_TH:
            return True
    return False

# ── 라이다 연결 ───────────────────────────────────────────────────────────────
lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
lidar.write(bytes([0xA5, 0x40]))
time.sleep(1)
lidar.write(bytes([0xA5, 0x20]))
lidar.read(7)
print("LIDAR connected. Ctrl+C to stop.")

# ── 화면 구성 (단일 플롯) ─────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 6))

ax.set_xlim(-180, 180)
ax.set_ylim(0, DETECTION_RANGE)
ax.set_xlabel('Angle (deg)   <- Left (-) | (+) Right ->')
ax.set_ylabel('Distance (mm)')
ax.set_title('Distance vs Angle  (360 deg)')
ax.grid(True, alpha=0.3)
ax.axvline(0, color='gray', lw=0.8, linestyle=':')
ax.axhline(STOP_FWD_MIN, color='red', lw=1.2, linestyle='--',
           alpha=0.6, label=f'STOP min {STOP_FWD_MIN}mm')
ax.axhline(STOP_FWD_MAX, color='red', lw=1.2, linestyle='-.',
           alpha=0.6, label=f'STOP max {STOP_FWD_MAX}mm')
ax.legend(fontsize=8, loc='upper right')

# 동적 요소
dist_line,  = ax.plot([], [], '.', color='steelblue', markersize=3, alpha=0.7)
stop_dots,  = ax.plot([], [], 'r.', markersize=8, zorder=6)
near_vline  = ax.axvline(0, color='orange', lw=1.5, linestyle='--', alpha=0.0)
dir_arrow   = ax.axvline(0, color='limegreen', lw=2.5, alpha=0.0)  # 방향 표시

# 텍스트 박스 (v/w 정보)
info_box = ax.text(
    0.01, 0.97, '', transform=ax.transAxes,
    fontsize=10, verticalalignment='top', fontfamily='monospace',
    bbox=dict(boxstyle='round', facecolor='white', alpha=0.85)
)
title_txt = ax.title

DYNAMIC_ARTISTS = (dist_line, stop_dots, near_vline, dir_arrow, info_box, title_txt)

# ── 버퍼 ─────────────────────────────────────────────────────────────────────
current_scan = []
display_scan = []
prev_w       = 0.0

# ── 업데이트 ─────────────────────────────────────────────────────────────────
def update(_frame):
    global current_scan, display_scan, prev_w

    while lidar.in_waiting >= 5:
        raw = lidar.read(5)
        result = parse_packet(raw)
        if result is None: continue
        angle_raw, distance = result
        if (raw[0] & 0x01) == 1 and current_scan:
            display_scan = list(current_scan)
            current_scan = []
        current_scan.append((normalize_angle(angle_raw), distance))

    if not display_scan:
        return DYNAMIC_ARTISTS

    arr  = np.array(display_scan)
    mask = (arr[:, 1] > LIDAR_MIN_VALID) & (arr[:, 1] < DETECTION_RANGE)
    arr  = arr[mask]
    if len(arr) == 0:
        return DYNAMIC_ARTISTS

    angles_np = arr[:, 0]
    dists_np  = arr[:, 1]
    rads      = np.radians(angles_np)
    ys        = dists_np * np.cos(rads)
    horizs    = np.abs(dists_np * np.sin(rads))

    # 거리 vs 각도
    dist_line.set_data(angles_np, dists_np)

    # 최근접
    idx = int(np.argmin(dists_np))
    na, nd = float(angles_np[idx]), float(dists_np[idx])
    near_vline.set_xdata([na]); near_vline.set_alpha(0.7)

    # STOP 포인트
    stop_mask = (ys >= STOP_FWD_MIN) & (ys <= STOP_FWD_MAX) & (horizs < STOP_HORIZ_TH)
    if np.any(stop_mask):
        stop_dots.set_data(angles_np[stop_mask], dists_np[stop_mask])
    else:
        stop_dots.set_data([], [])

    # v/w 계산
    pts = list(map(tuple, arr))
    is_stop = bool(np.any(stop_mask))
    v, w, dir_str, gap_L, gap_R = compute_vw(pts)
    w_smooth = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
    prev_w   = w_smooth

    # 방향 화살표 (w 부호로 각도 표시)
    arrow_angle = -30 if w_smooth > 0 else 30   # 왼쪽=음수각, 오른쪽=양수각
    dir_arrow.set_xdata([arrow_angle]); dir_arrow.set_alpha(0.6)

    # 정보 박스
    stop_str = " *** STOP ***" if is_stop else ""
    info_box.set_text(
        f" v  = {v:+.3f} m/s\n"
        f" w  = {w_smooth:+.3f} rad/s\n"
        f" dir= {dir_str}\n"
        f" gapL= {gap_L:.0f}mm\n"
        f" gapR= {gap_R:.0f}mm\n"
        f" near= {nd:.0f}mm @ {na:+.1f}deg"
        f"{stop_str}"
    )
    info_box.get_bbox_patch().set_facecolor('salmon' if is_stop else 'white')

    title_txt.set_text(
        f"Distance vs Angle  |  {len(arr)}pts  "
        f"v={v:.2f} w={w_smooth:+.2f}  dir={dir_str}"
    )

    return DYNAMIC_ARTISTS


ani = animation.FuncAnimation(fig, update, interval=UPDATE_INTERVAL,
                               blit=True, cache_frame_data=False)

try:
    plt.tight_layout()
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    lidar.write(bytes([0xA5, 0x25]))
    time.sleep(0.1)
    lidar.close()
    print("Lidar stopped.")

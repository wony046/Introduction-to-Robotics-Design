"""
RPLIDAR C1 bounding box visualizer
====================================
Usage:
    python3 lidar_visualizer.py

Controls:
    Ctrl+C or close window -> exit
"""
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from rplidar_c1 import RPLidar

# ── settings ──────────────────────────────────────────────────────────────────
LIDAR_PORT      = '/dev/ttyUSB0'
LIDAR_BAUDRATE  = 460800
MAX_RANGE_MM    = 800
MIN_VALID_MM    = 100
UPDATE_INTERVAL = 100   # ms

# ── jw_won.py parameters (keep in sync) ───────────────────────────────────────
ROBOT_HALF_WIDTH = 110
STOP_FWD_MIN     = 100
STOP_FWD_MAX     = 175
STOP_HORIZ_TH    = 105

DETECTION_RANGE  = 1500
FORWARD_SPEED    = 0.45
MIN_SPEED        = 0.12
MAX_W            = 2.0
W_MIN_DANGER     = 0.5
LAYER_PERCENTILE = 5
SCORE_ALPHA      = 5.0
SCORE_BETA       = 8        # 정면 방향 영향 (약화)
SCORE_SIDE       = 2000.0   # 측방 방향 가중치 (주도)
DEPTH_JUMP_THRES = 120

LAYERS = [
    # L1: 가장 가까움, 동적 가중치, weight_cap=7.5, v_max=0.30
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':140, 'color':'#FF4444',
     'w_gain':2.8, 'weight_base':0.8, 'weight_cap':7.5, 'weight_dynamic':True,
     'v_max':0.22, 'affects_v':True},
    # L2: 가까움, 동적 가중치, weight_cap=4.5, v_max=0.38
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':120, 'color':'#FF8800',
     'w_gain':2.5, 'weight_base':0.6, 'weight_cap':4.5, 'weight_dynamic':True,
     'v_max':0.38, 'affects_v':True},
    # L3: 중간, 동적 가중치, weight_cap=2.5, v_max=FORWARD_SPEED
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':120, 'color':'#DDCC00',
     'w_gain':1.8, 'weight_base':0.2, 'weight_cap':2.5, 'weight_dynamic':True, 'affects_v':True},
    # L4: 중간-원거리 (weight: 진입 0.2 → 끝 0.1)
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':100, 'color':'#88CC00',
     'w_gain':1.0, 'weight_base':0.1, 'weight_start':0.2, 'weight_dynamic':False, 'affects_v':True},
    # L5: 원거리 (weight: 진입 0.1 → 끝 0.05)
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':100, 'color':'#00BB44',
     'w_gain':0.4, 'weight_base':0.05, 'weight_start':0.1,  'weight_dynamic':False, 'affects_v':False},
    # L6: 최원거리 (weight: 진입 0.05 → 끝 0.02)
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':100, 'color':'#0088CC',
     'w_gain':0.3, 'weight_base':0.02, 'weight_start':0.05, 'weight_dynamic':False, 'affects_v':False},
]

# ── side repulsion parameters (jw_won.py: get_side_repulsion) ─────────────────
# Detection zone: horiz 110~300 mm,  fwd -80~+80 mm
SIDE_INNER        = ROBOT_HALF_WIDTH
SIDE_SAFE_MARGIN  = 190
SIDE_OUTER        = SIDE_INNER + SIDE_SAFE_MARGIN  # 300 mm
SIDE_FWD_LEAD     = 80
SIDE_FWD_REAR     = 80
SIDE_REPULSE_GAIN = 0.8
SIDE_EXP_K        = 2.0
SCAN_WIDE_HALF    = 135

# ── side layer parameters (jw_won.py: get_side_layer_push) ───────────────────
# Sector: ±15°~±75° (robot-local), up to 600 mm
SIDE_LAYER_ANG_START = 15   # deg
SIDE_LAYER_ANG_END   = 75   # deg
SIDE_LAYER_DIST_MAX  = 600  # mm
SIDE_W_BOOST_GAIN    = 3.0  # 측방 레이어 net delta 계수 (부호 있는 합산)

# ── path prediction / strength bar display ────────────────────────────────────
PREDICT_SEC = 1.5    # s: how far ahead to draw the predicted path
PREDICT_N   = 50     # number of integration steps
SBAR_Y      = -170   # mm: y-position of side strength bars
SBAR_SCALE  = 140    # mm per unit strength (max bar length)

# ── figure ────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 10))

# ── static: side layer fan zones (±15°~±75°, 600mm) ──────────────────────────
# Coordinate: +x=right, +y=forward → matplotlib Wedge angle from +x (CCW)
# Robot +15°~+75° → mpl 15°~75°  (right side)
# Robot -75°~-15° → mpl 105°~165° (left side)
ax.add_patch(patches.Wedge(
    (0, 0), SIDE_LAYER_DIST_MAX, 15, 75,
    facecolor='cyan', alpha=0.08, edgecolor='darkcyan', linewidth=1.2,
    linestyle='-.', label='side layer R', zorder=2
))
ax.add_patch(patches.Wedge(
    (0, 0), SIDE_LAYER_DIST_MAX, 105, 165,
    facecolor='cyan', alpha=0.08, edgecolor='darkcyan', linewidth=1.2,
    linestyle='-.', label='side layer L', zorder=2
))

# ── static: layer bounding boxes (front layers) ───────────────────────────────
for layer in LAYERS:
    ax.add_patch(patches.Rectangle(
        (-layer['horiz_th'], layer['fwd_min']),
        layer['horiz_th'] * 2, layer['fwd_max'] - layer['fwd_min'],
        linewidth=1.5, edgecolor=layer['color'], facecolor=layer['color'],
        alpha=0.13, label=layer['name'], zorder=2
    ))

# ── static: STOP zone ─────────────────────────────────────────────────────────
ax.add_patch(patches.Rectangle(
    (-STOP_HORIZ_TH, STOP_FWD_MIN), STOP_HORIZ_TH * 2, STOP_FWD_MAX - STOP_FWD_MIN,
    linewidth=2, edgecolor='red', facecolor='red',
    alpha=0.25, label='STOP zone', zorder=3
))

# ── static: side repulsion detection zones (purple dashed) ────────────────────
# Left:  x [-300, -110],  y [-80, +80]
# Right: x [+110, +300],  y [-80, +80]
_sw = SIDE_OUTER - SIDE_INNER        # 190 mm
_sh = SIDE_FWD_REAR + SIDE_FWD_LEAD  # 160 mm
ax.add_patch(patches.Rectangle(
    (-SIDE_OUTER, -SIDE_FWD_REAR), _sw, _sh,
    linewidth=1.8, edgecolor='purple', facecolor='purple',
    alpha=0.10, linestyle='--', label='side repulsion', zorder=2
))
ax.add_patch(patches.Rectangle(
    (SIDE_INNER, -SIDE_FWD_REAR), _sw, _sh,
    linewidth=1.8, edgecolor='purple', facecolor='purple',
    alpha=0.10, linestyle='--', zorder=2
))

# ── static: robot body ────────────────────────────────────────────────────────
ax.add_patch(patches.FancyBboxPatch(
    (-ROBOT_HALF_WIDTH, -80), ROBOT_HALF_WIDTH * 2, 240,
    boxstyle='round,pad=5', linewidth=2,
    edgecolor='#333', facecolor='#888', alpha=0.6, zorder=4
))
ax.annotate('', xy=(0, 200), xytext=(0, 90),
            arrowprops=dict(arrowstyle='->', color='black', lw=2.5), zorder=5)
ax.text(0, 215, 'fwd', ha='center', fontsize=8)

# ── static: sector boundary lines (±15°, ±75°) ────────────────────────────────
for ang_deg in [15, 75]:
    for sign in [1, -1]:
        r = math.radians(sign * ang_deg)
        ex = SIDE_LAYER_DIST_MAX * math.sin(r)
        ey = SIDE_LAYER_DIST_MAX * math.cos(r)
        ax.plot([0, ex], [0, ey], color='darkcyan', lw=0.8,
                linestyle=':', alpha=0.5, zorder=2)

# ── static: strength bar backgrounds ──────────────────────────────────────────
ax.plot([-ROBOT_HALF_WIDTH - SBAR_SCALE, -ROBOT_HALF_WIDTH], [SBAR_Y, SBAR_Y],
        '--', color='purple', lw=1.5, alpha=0.35, zorder=2)
ax.plot([ROBOT_HALF_WIDTH, ROBOT_HALF_WIDTH + SBAR_SCALE], [SBAR_Y, SBAR_Y],
        '--', color='orchid', lw=1.5, alpha=0.35, zorder=2)
ax.text(-ROBOT_HALF_WIDTH - SBAR_SCALE / 2, SBAR_Y - 22,
        'L str', ha='center', fontsize=7, color='purple', alpha=0.75)
ax.text( ROBOT_HALF_WIDTH + SBAR_SCALE / 2, SBAR_Y - 22,
        'R str', ha='center', fontsize=7, color='orchid', alpha=0.75)

# ── axis config ───────────────────────────────────────────────────────────────
ax.set_xlim(-MAX_RANGE_MM, MAX_RANGE_MM)
ax.set_ylim(-MAX_RANGE_MM * 0.4, MAX_RANGE_MM)
ax.set_aspect('equal')
ax.set_xticks(range(-MAX_RANGE_MM, MAX_RANGE_MM + 1, 100))
ax.set_yticks(range(-300, MAX_RANGE_MM + 1, 100))
ax.tick_params(labelsize=7)
ax.axhline(0, color='gray', lw=0.5, zorder=1)
ax.axvline(0, color='gray', lw=0.5, zorder=1)
ax.set_xlabel('<- Left (mm)  |  Right (mm) ->', fontsize=9)
ax.set_ylabel('Forward (mm)', fontsize=9)
ax.grid(True, alpha=0.25, zorder=0)
ax.legend(fontsize=7, loc='upper right', ncol=2)

# ── dynamic artists ───────────────────────────────────────────────────────────
scan_line,       = ax.plot([], [], '.', color='steelblue', markersize=3,
                           alpha=0.85, zorder=6)
stop_line,       = ax.plot([], [], 'rx', markersize=9, markeredgewidth=2, zorder=8)
side_left_line,  = ax.plot([], [], 'o', color='purple', markersize=5,
                           alpha=0.85, zorder=7)
side_right_line, = ax.plot([], [], 'o', color='orchid',  markersize=5,
                           alpha=0.85, zorder=7)
slayer_left_line,  = ax.plot([], [], 'o', color='darkcyan', markersize=4,
                              alpha=0.80, zorder=7)
slayer_right_line, = ax.plot([], [], 'o', color='teal', markersize=4,
                              alpha=0.80, zorder=7)

# Predicted path (gray dashed = w_layer only,  green solid = w_total with side correction)
path_base_line,  = ax.plot([], [], '--', color='gray',      lw=1.8, alpha=0.7, zorder=8)
path_line,       = ax.plot([], [], '-',  color='limegreen', lw=3.0, zorder=9)
path_tip,        = ax.plot([], [], 'o',  color='limegreen', markersize=8, zorder=10)

# Side strength bars
side_left_bar,   = ax.plot([], [], '-', color='purple', lw=5,
                           solid_capstyle='round', zorder=8)
side_right_bar,  = ax.plot([], [], '-', color='orchid',  lw=5,
                           solid_capstyle='round', zorder=8)

info_text = ax.text(
    -MAX_RANGE_MM + 15, -MAX_RANGE_MM * 0.38, '',
    fontsize=8, color='black', va='top',
    bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.75)
)
title_obj = ax.title
ax.set_title('RPLIDAR C1 - Bounding Box View')

DYNAMIC_ARTISTS = (scan_line, stop_line,
                   side_left_line, side_right_line,
                   slayer_left_line, slayer_right_line,
                   path_base_line, path_line, path_tip,
                   side_left_bar, side_right_bar,
                   info_text, title_obj)

# ── LIDAR connection ──────────────────────────────────────────────────────────
print(f"[Visualizer] Connecting: {LIDAR_PORT}")
lidar     = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
scan_iter = lidar.iter_scans(max_buf_meas=2000, min_len=5)
print("[Visualizer] Ready. Close window or Ctrl+C to stop.")


# ── computation: ported from jw_won.py ────────────────────────────────────────

def _is_in_front_90(a):
    return -90 <= a <= 90

def _decompose(a_deg, dist):
    rad = math.radians(a_deg)
    return abs(dist * math.sin(rad)), dist * math.cos(rad)  # horiz, fwd

def _cosine_dist(d1, d2, ang_diff_deg):
    t = math.radians(abs(ang_diff_deg))
    return math.sqrt(d1**2 + d2**2 - 2 * d1 * d2 * math.cos(t))

def _process_layer(scan_norm, layer):
    pts = []
    for a, d in scan_norm:
        if d < MIN_VALID_MM or d > DETECTION_RANGE:
            continue
        if not _is_in_front_90(a):
            continue
        horiz, fwd = _decompose(a, d)
        if layer['fwd_min'] <= fwd < layer['fwd_max'] and horiz < layer['horiz_th']:
            pts.append({'angle': a, 'dist': d, 'horiz': horiz, 'fwd': fwd,
                        'horiz_error': layer['horiz_th'] - horiz})
    if not pts:
        return None

    n_take = max(1, int(len(pts) * LAYER_PERCENTILE / 100))
    rep = sorted(pts, key=lambda p: p['dist'])[:n_take]
    rep_angle = sum(p['angle'] for p in rep) / len(rep)
    rep_horiz = sum(p['horiz'] for p in rep) / len(rep)
    rep_fwd   = sum(p['fwd']   for p in rep) / len(rep)
    rep_h_err = layer['horiz_th'] - rep_horiz

    if layer['weight_dynamic']:
        cap    = layer.get('weight_cap', 1.0)
        raw    = rep_h_err / layer['horiz_th'] * cap
        weight = max(layer['weight_base'], min(cap, raw))
    else:
        # L4~L6: fwd 위치에 따라 weight_start → weight_base 선형 보간
        prog = max(0.0, min(1.0,
               (rep_fwd - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min'])))
        weight = layer['weight_start'] + (layer['weight_base'] - layer['weight_start']) * prog

    urgency = layer['w_gain'] * rep_h_err / layer['horiz_th']

    if layer['affects_v']:
        prog  = max(0.0, min(1.0,
                (rep_fwd - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min'])))
        v_max = layer.get('v_max', FORWARD_SPEED)
        v_proposal = MIN_SPEED + (v_max - MIN_SPEED) * prog
    else:
        v_proposal = None

    push_left  = sum(p['horiz_error'] for p in rep if p['angle'] < 0)
    push_right = sum(p['horiz_error'] for p in rep if p['angle'] > 0)

    return {'weight': weight, 'urgency': urgency, 'v_proposal': v_proposal,
            'rep_angle': rep_angle, 'rep_horiz': rep_horiz, 'rep_fwd': rep_fwd,
            'push_left': push_left, 'push_right': push_right, 'name': layer['name']}

def _get_gap_width(scan_norm, ref_angle, ref_dist, is_left):
    front = [(a, d) for a, d in scan_norm if _is_in_front_90(a)]
    if is_left:
        search = sorted([(a, d) for a, d in front if a < ref_angle],
                        key=lambda x: x[0], reverse=True)
    else:
        search = sorted([(a, d) for a, d in front if a > ref_angle],
                        key=lambda x: x[0])
    if not search:
        return 0.0
    edge_p = (ref_angle, ref_dist)
    for i, p in enumerate(search):
        if abs(p[1] - edge_p[1]) > DEPTH_JUMP_THRES:
            wall = search[i:]
            if wall:
                return min(_cosine_dist(edge_p[1], wp[1], abs(edge_p[0] - wp[0]))
                           for wp in wall)
        edge_p = p
    rem = abs((-90 - edge_p[0]) if is_left else (90 - edge_p[0]))
    if rem > 15:
        return _cosine_dist(edge_p[1], edge_p[1], rem)
    return 0.0

def _get_side_layer_push(scan_norm):
    """측방 레이어: ±15°~±75°, 최대 600mm. 반환: (left_push, right_push) [0~1]"""
    left_push  = 0.0
    right_push = 0.0
    for a, d in scan_norm:
        if d < MIN_VALID_MM or d > SIDE_LAYER_DIST_MAX:
            continue
        strength = (SIDE_LAYER_DIST_MAX - d) / SIDE_LAYER_DIST_MAX
        if -SIDE_LAYER_ANG_END <= a <= -SIDE_LAYER_ANG_START:
            left_push  = max(left_push,  strength)
        elif SIDE_LAYER_ANG_START <= a <= SIDE_LAYER_ANG_END:
            right_push = max(right_push, strength)
    return left_push, right_push

def _compute_vw(scan_norm):
    """
    Port of find_vw_layered.
    Returns (v, w_base, w_with_side):
      w_base      = direction * forward_urgency  (정면 레이어만, 회색 경로)
      w_with_side = w_base + side_w_delta        (측방 net delta 합산 후, 초록 경로 기준)
    heading_deg assumed 0 (no IMU in visualizer).
    """
    layer_results = [r for r in (_process_layer(scan_norm, L) for L in LAYERS)
                     if r is not None]
    if not layer_results:
        return FORWARD_SPEED, 0.0, 0.0

    closest   = min(layer_results, key=lambda r: r['rep_horiz'])
    ref_angle = closest['rep_angle']
    ref_dist  = math.sqrt(closest['rep_horiz']**2 + closest['rep_fwd']**2)

    gap_L = _get_gap_width(scan_norm, ref_angle, ref_dist, is_left=True)
    gap_R = _get_gap_width(scan_norm, ref_angle, ref_dist, is_left=False)

    sum_pR = sum(r['weight'] * r['push_right'] for r in layer_results)
    sum_pL = sum(r['weight'] * r['push_left']  for r in layer_results)

    side_left_push, side_right_push = _get_side_layer_push(scan_norm)

    score_L = (SCORE_ALPHA * gap_L
               + SCORE_BETA  * sum_pR
               + SCORE_SIDE  * side_right_push)
    score_R = (SCORE_ALPHA * gap_R
               + SCORE_BETA  * sum_pL
               + SCORE_SIDE  * side_left_push)

    direction       = 1.0 if score_L >= score_R else -1.0
    total_w_all     = sum(r['weight'] for r in layer_results)
    forward_urgency = sum(r['weight'] * r['urgency'] for r in layer_results) / total_w_all
    forward_urgency = max(min(forward_urgency, MAX_W), W_MIN_DANGER)
    w_base          = direction * forward_urgency

    # 측방 레이어 net delta: 부호 있는 합산 (방향과 같으면 크기 증가, 반대면 감소)
    side_w_delta = (side_right_push - side_left_push) * SIDE_W_BOOST_GAIN
    w_with_side  = w_base + side_w_delta

    v_layers = [r for r in layer_results if r['v_proposal'] is not None]
    if v_layers:
        tw = sum(r['weight'] for r in v_layers)
        v  = sum(r['weight'] * r['v_proposal'] for r in v_layers) / tw
    else:
        v = FORWARD_SPEED

    return v, w_base, w_with_side

def _exp_strength(horizs_in_zone):
    """Exponential repulsion: 1.0 at robot edge (110 mm), 0.0 at outer boundary (300 mm)."""
    if len(horizs_in_zone) == 0:
        return 0.0
    t = (horizs_in_zone - SIDE_INNER) / SIDE_SAFE_MARGIN
    s = (np.exp(SIDE_EXP_K * (1.0 - t)) - 1.0) / (math.exp(SIDE_EXP_K) - 1.0)
    return float(np.max(s))

def _predict_path(v, w):
    """
    Differential drive path prediction over PREDICT_SEC seconds.
    Coordinate: x=lateral (right=+), y=forward (+y), robot starts at (0,0) facing +y.
    """
    dt = PREDICT_SEC / PREDICT_N
    theta, px, py = 0.0, 0.0, 0.0
    xs, ys = [0.0], [0.0]
    for _ in range(PREDICT_N):
        theta += w * dt
        px    += -v * math.sin(theta) * dt * 1000
        py    +=  v * math.cos(theta) * dt * 1000
        xs.append(px)
        ys.append(py)
    return xs, ys


# ── update loop ───────────────────────────────────────────────────────────────
def update(_frame):
    try:
        scan = next(scan_iter)
    except StopIteration:
        return DYNAMIC_ARTISTS

    raw = [(q, a, d) for q, a, d in scan
           if q > 0 and MIN_VALID_MM < d < MAX_RANGE_MM]
    if not raw:
        return DYNAMIC_ARTISTS

    raw_angles = np.array([a for _, a, _ in raw])
    dists      = np.array([d for _, _, d in raw])

    xs     = dists * np.sin(np.radians(raw_angles))   # lateral (+x = right)
    ys     = dists * np.cos(np.radians(raw_angles))   # forward (+y = fwd)
    horizs = np.abs(xs)

    # 0~360 -> -180~+180  (required for layer / wide-scan logic)
    norm_angles = np.where(raw_angles > 180, raw_angles - 360, raw_angles)
    scan_norm   = list(zip(norm_angles.tolist(), dists.tolist()))

    # ── scan points ───────────────────────────────────────────────────────────
    scan_line.set_data(xs, ys)

    # ── STOP trigger ──────────────────────────────────────────────────────────
    stop_mask = (ys >= STOP_FWD_MIN) & (ys <= STOP_FWD_MAX) & (horizs < STOP_HORIZ_TH)
    stop_on   = bool(np.any(stop_mask))
    if stop_on:
        stop_line.set_data(xs[stop_mask], ys[stop_mask])
    else:
        stop_line.set_data([], [])

    # ── side repulsion (mirrors jw_won.py get_side_repulsion) ─────────────────
    wide_mask = np.abs(norm_angles) <= SCAN_WIDE_HALF
    fwd_mask  = (ys >= -SIDE_FWD_REAR) & (ys <= SIDE_FWD_LEAD)
    hrz_mask  = (horizs >= SIDE_INNER)  & (horizs < SIDE_OUTER)
    side_mask = wide_mask & fwd_mask & hrz_mask

    left_mask  = side_mask & (xs < 0)
    right_mask = side_mask & (xs > 0)

    side_left_line.set_data(xs[left_mask],   ys[left_mask])
    side_right_line.set_data(xs[right_mask], ys[right_mask])

    left_str  = _exp_strength(horizs[left_mask])
    right_str = _exp_strength(horizs[right_mask])
    side_dw   = (right_str - left_str) * SIDE_REPULSE_GAIN

    # ── side layer (±15°~±75°, 600mm) ─────────────────────────────────────────
    slayer_left_mask  = ((norm_angles >= -SIDE_LAYER_ANG_END) &
                         (norm_angles <= -SIDE_LAYER_ANG_START) &
                         (dists <= SIDE_LAYER_DIST_MAX))
    slayer_right_mask = ((norm_angles >= SIDE_LAYER_ANG_START) &
                         (norm_angles <= SIDE_LAYER_ANG_END) &
                         (dists <= SIDE_LAYER_DIST_MAX))

    slayer_left_line.set_data(xs[slayer_left_mask],   ys[slayer_left_mask])
    slayer_right_line.set_data(xs[slayer_right_mask], ys[slayer_right_mask])

    side_left_push, side_right_push = _get_side_layer_push(scan_norm)

    # ── v / w from layers ─────────────────────────────────────────────────────
    v, w_base, w_with_side = _compute_vw(scan_norm)
    w_total = float(np.clip(w_with_side + side_dw, -MAX_W, MAX_W))

    # ── predicted paths ───────────────────────────────────────────────────────
    # 회색 점선: 정면 레이어 urgency만 (w_base)
    bx, by = _predict_path(v, w_base)
    path_base_line.set_data(bx, by)

    # 초록 실선: 측방 net delta + 측면 반발력 합산 후
    gx, gy = _predict_path(v, w_total)
    path_line.set_data(gx, gy)
    path_tip.set_data([gx[-1]], [gy[-1]])

    # ── side strength bars ────────────────────────────────────────────────────
    if left_str > 0.02:
        side_left_bar.set_data(
            [-ROBOT_HALF_WIDTH, -(ROBOT_HALF_WIDTH + left_str * SBAR_SCALE)],
            [SBAR_Y, SBAR_Y]
        )
    else:
        side_left_bar.set_data([], [])

    if right_str > 0.02:
        side_right_bar.set_data(
            [ROBOT_HALF_WIDTH, ROBOT_HALF_WIDTH + right_str * SBAR_SCALE],
            [SBAR_Y, SBAR_Y]
        )
    else:
        side_right_bar.set_data([], [])

    # ── nearest point ─────────────────────────────────────────────────────────
    idx = int(np.argmin(dists))
    nd  = float(dists[idx])
    na  = float(math.degrees(math.atan2(xs[idx], ys[idx])))

    # ── info text ─────────────────────────────────────────────────────────────
    dir_str  = 'L' if w_total > 0.05 else ('R' if w_total < -0.05 else 'straight')
    stop_str = '  *** STOP ***' if stop_on else ''
    info_text.set_text(
        f'Nearest    : {nd:.0f}mm @ {na:+.1f}deg\n'
        f'v          : {v:.3f} m/s\n'
        f'w_base     : {w_base:+.3f} rad/s  (gray, fwd urgency only)\n'
        f'side delta : {w_with_side - w_base:+.3f} rad/s  (net: L={side_left_push:.2f} R={side_right_push:.2f})\n'
        f'side rep dw: {side_dw:+.3f} rad/s  L={left_str:.2f} R={right_str:.2f}\n'
        f'w_total    : {w_total:+.3f} rad/s  [{dir_str}]  (green path)'
    )
    title_obj.set_text(
        f'RPLIDAR C1 - Bounding Box View  |  '
        f'v={v:.2f}m/s  w={w_total:+.2f}rad/s  nearest {nd:.0f}mm{stop_str}'
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
    print("[Visualizer] Stopping...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print("[Visualizer] Done.")

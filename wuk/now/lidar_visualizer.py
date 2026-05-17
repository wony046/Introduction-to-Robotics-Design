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
STOP_FWD_MAX     = 180
STOP_HORIZ_TH    = 105

LAYERS = [
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':200, 'color':'#FF4444'},
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':200, 'color':'#FF8800'},
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':140, 'color':'#DDCC00'},
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':140, 'color':'#88CC00'},
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':120, 'color':'#00BB44'},
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':100, 'color':'#0088CC'},
]

# ── side repulsion parameters (jw_won.py: get_side_repulsion) ─────────────────
# Detection zone: horiz 110~300 mm,  fwd -240~+50 mm
SIDE_INNER        = ROBOT_HALF_WIDTH               # 110 mm: robot edge
SIDE_SAFE_MARGIN  = 190                            # mm  →  outer = 300 mm
SIDE_OUTER        = SIDE_INNER + SIDE_SAFE_MARGIN  # 300 mm
SIDE_FWD_LEAD     = 50                             # mm: forward margin
SIDE_FWD_REAR     = 240                            # mm: rear depth
SIDE_REPULSE_GAIN = 0.8                            # rad/s max contribution
SIDE_EXP_K        = 3.0
SCAN_WIDE_HALF    = 135                            # deg: ±135 lateral range

# ── strength bar display ───────────────────────────────────────────────────────
SBAR_Y     = -170   # mm: y-position of strength bars (below robot body)
SBAR_SCALE = 140    # mm per unit strength (max 140 mm at str=1.0)

# ── arc radius for delta_w indicator ──────────────────────────────────────────
ARC_R      = 75     # mm: radius of rotation arc around robot center

# ── figure ────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 10))

# ── static: layer bounding boxes ──────────────────────────────────────────────
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
# Left:  x [-300, -110],  y [-240, +50]
# Right: x [+110, +300],  y [-240, +50]
_sw = SIDE_OUTER - SIDE_INNER        # 190 mm wide
_sh = SIDE_FWD_REAR + SIDE_FWD_LEAD  # 290 mm tall
ax.add_patch(patches.Rectangle(
    (-SIDE_OUTER, -SIDE_FWD_REAR), _sw, _sh,
    linewidth=1.8, edgecolor='purple', facecolor='purple',
    alpha=0.10, linestyle='--', label='side zone', zorder=2
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

# ── static: rotation arc reference circle (gray dotted) ───────────────────────
# Parametrization used: x=r*sin(t), y=r*cos(t)
#   t=0   → top   (0, r)  = forward
#   t>0   → right = CW    (right turn, negative delta_w)
#   t<0   → left  = CCW   (left turn,  positive delta_w)
_t_bg = np.linspace(0, 2 * math.pi, 80)
ax.plot(ARC_R * np.sin(_t_bg), ARC_R * np.cos(_t_bg),
        ':', color='lightgray', lw=1, alpha=0.6, zorder=3)
ax.text(0, ARC_R + 5, 'w', ha='center', va='bottom', fontsize=7, color='gray')

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

# Rotation arc: shows delta_w direction (CCW=left, CW=right) and magnitude
rot_arc_line,    = ax.plot([], [], '-',  color='limegreen', lw=3.5, zorder=9)
rot_arc_tip,     = ax.plot([], [], 'o',  color='limegreen', markersize=7,  zorder=10)

# Side strength bars: horizontal bars below robot showing L/R repulsion strength
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
                   rot_arc_line, rot_arc_tip,
                   side_left_bar, side_right_bar,
                   info_text, title_obj)

# ── LIDAR connection ──────────────────────────────────────────────────────────
print(f"[Visualizer] Connecting: {LIDAR_PORT}")
lidar     = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
scan_iter = lidar.iter_scans(max_buf_meas=2000, min_len=5)
print("[Visualizer] Ready. Close window or Ctrl+C to stop.")


def _exp_strength(horizs_in_zone):
    """Exponential repulsion: 1.0 at robot edge (110 mm), 0.0 at outer boundary (300 mm)."""
    if len(horizs_in_zone) == 0:
        return 0.0
    t = (horizs_in_zone - SIDE_INNER) / SIDE_SAFE_MARGIN   # 0=edge, 1=boundary
    s = (np.exp(SIDE_EXP_K * (1.0 - t)) - 1.0) / (math.exp(SIDE_EXP_K) - 1.0)
    return float(np.max(s))


def _make_rotation_arc(dw):
    """
    Rotation arc around robot center (radius=ARC_R).

    Parametrization: x = r*sin(t),  y = r*cos(t)
      t = 0         -> top  (0, r)  = forward direction
      t increasing  -> rightward = CW  rotation (right turn)
      t decreasing  -> leftward  = CCW rotation (left turn)

    delta_w > 0  ->  right obstacle  ->  left turn  (CCW)  ->  sweep < 0
    delta_w < 0  ->  left  obstacle  ->  right turn (CW)   ->  sweep > 0
    """
    if abs(dw) < 0.02:
        return np.array([]), np.array([])
    max_sweep = 0.75 * math.pi                          # max arc = 135 deg at full gain
    sweep = -(dw / SIDE_REPULSE_GAIN) * max_sweep       # negative for CCW (left turn)
    t = np.linspace(0.0, sweep, 40)
    return ARC_R * np.sin(t), ARC_R * np.cos(t)


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

    xs     = dists * np.sin(np.radians(raw_angles))   # lateral  (+x = right)
    ys     = dists * np.cos(np.radians(raw_angles))   # forward  (+y = fwd)
    horizs = np.abs(xs)

    # 0~360 -> -180~+180  (for wide-scan range check)
    norm_angles = np.where(raw_angles > 180, raw_angles - 360, raw_angles)

    # ── scan points ───────────────────────────────────────────────────────────
    scan_line.set_data(xs, ys)

    # ── STOP trigger ──────────────────────────────────────────────────────────
    stop_mask = (ys >= STOP_FWD_MIN) & (ys <= STOP_FWD_MAX) & (horizs < STOP_HORIZ_TH)
    stop_on   = bool(np.any(stop_mask))
    if stop_on:
        stop_line.set_data(xs[stop_mask], ys[stop_mask])
    else:
        stop_line.set_data([], [])

    # ── side repulsion (mirrors jw_won.py get_side_repulsion exactly) ─────────
    wide_mask = np.abs(norm_angles) <= SCAN_WIDE_HALF           # |angle| <= 135 deg
    fwd_mask  = (ys >= -SIDE_FWD_REAR) & (ys <= SIDE_FWD_LEAD) # -240 ~ +50 mm
    hrz_mask  = (horizs >= SIDE_INNER)  & (horizs < SIDE_OUTER) # 110 ~ 300 mm
    side_mask = wide_mask & fwd_mask & hrz_mask

    left_mask  = side_mask & (xs < 0)
    right_mask = side_mask & (xs > 0)

    side_left_line.set_data(xs[left_mask],   ys[left_mask])
    side_right_line.set_data(xs[right_mask], ys[right_mask])

    left_str  = _exp_strength(horizs[left_mask])
    right_str = _exp_strength(horizs[right_mask])
    delta_w   = (right_str - left_str) * SIDE_REPULSE_GAIN

    # ── rotation arc (delta_w) ────────────────────────────────────────────────
    arc_xs, arc_ys = _make_rotation_arc(delta_w)
    rot_arc_line.set_data(arc_xs, arc_ys)
    if len(arc_xs) > 0:
        rot_arc_tip.set_data([arc_xs[-1]], [arc_ys[-1]])
    else:
        rot_arc_tip.set_data([], [])

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
    dir_str  = 'CCW(L)' if delta_w > 0.02 else ('CW(R)' if delta_w < -0.02 else 'none')
    stop_str = '  *** STOP ***' if stop_on else ''
    info_text.set_text(
        f'Nearest : {nd:.0f}mm @ {na:+.1f}deg\n'
        f'Side  L : str={left_str:.2f}  ({int(np.sum(left_mask))}pts)\n'
        f'Side  R : str={right_str:.2f}  ({int(np.sum(right_mask))}pts)\n'
        f'Side dw : {delta_w:+.3f} rad/s  [{dir_str}]'
    )
    title_obj.set_text(
        f'RPLIDAR C1 - Bounding Box View  |  '
        f'{len(raw)}pts  nearest {nd:.0f}mm{stop_str}'
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

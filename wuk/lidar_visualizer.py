"""
RPLIDAR C1 바운딩 박스 + 방향 예측 시각화
==========================================
- jw_won.py 레이어 + 측면 감지 구역 상시 표시
- 계산된 v/w 를 예측 경로 벡터로 표시
- 각 레이어 urgency 를 벡터 크기로 오버레이

실행: python3 lidar_visualizer.py
종료: Ctrl+C 또는 창 닫기
"""
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from rplidar_c1 import RPLidar

# ── 설정 (jw_won.py 와 동기화) ────────────────────────────────────────────────
LIDAR_PORT      = '/dev/ttyUSB0'
LIDAR_BAUDRATE  = 460800
MAX_RANGE_MM    = 800
MIN_VALID_MM    = 100
UPDATE_INTERVAL = 120   # ms

ROBOT_HALF_WIDTH = 110
FORWARD_SPEED    = 0.35
MIN_SPEED        = 0.07
MAX_W            = 2.0
W_MIN_DANGER     = 0.5
W_SMOOTH         = 0.45

STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 180
STOP_HORIZ_TH = 105

SIDE_SAFE_MARGIN = 190   # side_outer = 300mm
SIDE_FWD_LEAD    = 50
SIDE_FWD_REAR    = 240
SIDE_REPULSE_GAIN = 0.8
SIDE_EXP_K        = 3.0

SCORE_ALPHA      = 5.0
SCORE_BETA       = 20
DEPTH_JUMP_THRES = 120
LAYER_PERCENTILE = 5
PREDICT_SEC      = 1.5   # 예측 경로 시간 (초)

LAYERS = [
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':200,
     'w_gain':2.8, 'weight_base':0.8, 'weight_start':0.8, 'weight_dynamic':True,  'affects_v':True,  'color':'#FF4444'},
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':200,
     'w_gain':2.5, 'weight_base':0.6, 'weight_start':0.6, 'weight_dynamic':True,  'affects_v':True,  'color':'#FF8800'},
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':140,
     'w_gain':1.8, 'weight_base':0.2, 'weight_start':0.4, 'weight_dynamic':False, 'affects_v':True,  'color':'#DDCC00'},
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':140,
     'w_gain':1.0, 'weight_base':0.1, 'weight_start':0.2, 'weight_dynamic':False, 'affects_v':True,  'color':'#88CC00'},
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':120,
     'w_gain':0.4, 'weight_base':0.05,'weight_start':0.1, 'weight_dynamic':False, 'affects_v':False, 'color':'#00BB44'},
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':100,
     'w_gain':0.3, 'weight_base':0.02,'weight_start':0.05,'weight_dynamic':False, 'affects_v':False, 'color':'#0088CC'},
]

# ── v/w 계산 함수 (jw_won.py 핵심 복사) ──────────────────────────────────────
def is_in_front_90(a):  return -90 <= a <= 90

def decompose(a, d):
    r = math.radians(a)
    return abs(d * math.sin(r)), d * math.cos(r)

def cosine_dist(d1, d2, ang):
    t = math.radians(abs(ang))
    return math.sqrt(d1**2 + d2**2 - 2*d1*d2*math.cos(t))

def process_layer(pts, layer):
    hits = []
    for a, d in pts:
        if d < MIN_VALID_MM or d > MAX_RANGE_MM: continue
        if not is_in_front_90(a): continue
        h, f = decompose(a, d)
        if layer['fwd_min'] <= f < layer['fwd_max'] and h < layer['horiz_th']:
            hits.append({'a':a,'d':d,'h':h,'f':f,'herr':layer['horiz_th']-h})
    if not hits: return None
    n = max(1, int(len(hits) * LAYER_PERCENTILE / 100))
    rep = sorted(hits, key=lambda p: p['d'])[:n]
    rh  = sum(p['h'] for p in rep) / n
    rf  = sum(p['f'] for p in rep) / n
    herr = layer['horiz_th'] - rh
    if layer['weight_dynamic']:
        weight = max(layer['weight_base'], min(1.0, herr / layer['horiz_th']))
    else:
        prog   = max(0.0, min(1.0, (rf - layer['fwd_min']) /
                                    (layer['fwd_max'] - layer['fwd_min'])))
        weight = layer['weight_start'] + (layer['weight_base'] - layer['weight_start']) * prog
    urgency    = layer['w_gain'] * herr / layer['horiz_th']
    prog_v     = max(0.0, min(1.0, (rf - layer['fwd_min']) /
                                    (layer['fwd_max'] - layer['fwd_min'])))
    v_proposal = (MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * prog_v
                  if layer['affects_v'] else None)
    ra = sum(p['a'] for p in rep) / n
    return {'weight':weight,'urgency':urgency,'v_proposal':v_proposal,
            'rep_angle':ra,'rep_horiz':rh,'rep_fwd':rf,
            'push_L':sum(p['herr'] for p in rep if p['a']<0),
            'push_R':sum(p['herr'] for p in rep if p['a']>0),
            'name':layer['name'],'color':layer['color']}

def get_gap_width(pts, ref_a, ref_d, is_left):
    front = [(a,d) for a,d in pts if is_in_front_90(a)]
    search = sorted([p for p in front if (p[0]<ref_a if is_left else p[0]>ref_a)],
                    key=lambda x: x[0], reverse=is_left)
    if not search: return 0.0
    edge = (ref_a, ref_d)
    for i, p in enumerate(search):
        if abs(p[1] - edge[1]) > DEPTH_JUMP_THRES:
            wall = search[i:]
            if wall:
                return min(cosine_dist(edge[1], wp[1], abs(edge[0]-wp[0])) for wp in wall)
        edge = p
    rem = abs((-90-edge[0]) if is_left else (90-edge[0]))
    return cosine_dist(edge[1], edge[1], rem) if rem > 15 else 0.0

def get_side_repulsion(pts):
    inner  = ROBOT_HALF_WIDTH
    outer  = ROBOT_HALF_WIDTH + SIDE_SAFE_MARGIN
    lstr, rstr = 0.0, 0.0
    for a, d in pts:
        if d < MIN_VALID_MM or d > MAX_RANGE_MM: continue
        if not (-135 <= a <= 135): continue
        h, f = decompose(a, d)
        if f > SIDE_FWD_LEAD or f < -SIDE_FWD_REAR: continue
        if h < inner or h >= outer: continue
        t = (h - inner) / SIDE_SAFE_MARGIN
        strength = (math.exp(SIDE_EXP_K*(1.0-t)) - 1.0) / (math.exp(SIDE_EXP_K) - 1.0)
        if a < 0: lstr = max(lstr, strength)
        else:     rstr = max(rstr, strength)
    return (rstr - lstr) * SIDE_REPULSE_GAIN, lstr, rstr

def compute_vw(pts):
    results = [r for L in LAYERS for r in [process_layer(pts, L)] if r]
    if not results:
        return FORWARD_SPEED, 0.0, [], 0.0, 0.0

    closest  = min(results, key=lambda r: r['rep_horiz'])
    ref_a    = closest['rep_angle']
    ref_d    = math.sqrt(closest['rep_horiz']**2 + closest['rep_fwd']**2)
    gL = get_gap_width(pts, ref_a, ref_d, True)
    gR = get_gap_width(pts, ref_a, ref_d, False)

    spR = sum(r['weight']*r['push_R'] for r in results)
    spL = sum(r['weight']*r['push_L'] for r in results)
    sL  = SCORE_ALPHA*gL + SCORE_BETA*spR
    sR  = SCORE_ALPHA*gR + SCORE_BETA*spL
    direction = 1.0 if sL >= sR else -1.0

    vl = [r for r in results if r['v_proposal'] is not None]
    v  = (sum(r['weight']*r['v_proposal'] for r in vl) /
          sum(r['weight'] for r in vl)) if vl else FORWARD_SPEED

    tw  = sum(r['weight'] for r in results)
    wm  = sum(r['weight']*r['urgency'] for r in results) / tw
    wm  = max(min(wm, MAX_W), W_MIN_DANGER)

    side_dw, _, _ = get_side_repulsion(pts)
    w = max(min(direction*wm + side_dw, MAX_W), -MAX_W)

    return v, w, results, gL, gR

def predict_path(v, w, n=25):
    """v/w 로 예측 경로 계산 (로봇 프레임 x=lateral, y=fwd)"""
    dt = PREDICT_SEC / n
    xs, ys = [0.0], [0.0]
    theta, px, py = 0.0, 0.0, 0.0
    for _ in range(n):
        theta += w * dt
        px += -v * math.sin(theta) * dt * 1000  # m→mm
        py +=  v * math.cos(theta) * dt * 1000
        xs.append(px); ys.append(py)
    return xs, ys

# ── 화면 구성 ─────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(9, 11))

SIDE_INNER = ROBOT_HALF_WIDTH
SIDE_OUTER = ROBOT_HALF_WIDTH + SIDE_SAFE_MARGIN

# ── 정적: 측면 감지 구역 ──────────────────────────────────────────────────────
for sign, label in [(1, 'Side R'), (-1, 'Side L')]:
    x0 = sign * SIDE_INNER if sign > 0 else -SIDE_OUTER
    ax.add_patch(patches.Rectangle(
        (x0, -SIDE_FWD_REAR), SIDE_OUTER - SIDE_INNER, SIDE_FWD_REAR + SIDE_FWD_LEAD,
        linewidth=1.2, edgecolor='#AA44AA', facecolor='#CC88CC',
        alpha=0.13, zorder=2, linestyle='--'
    ))
ax.text( SIDE_OUTER - 10, -SIDE_FWD_REAR + 10, 'Side R',
         ha='right', fontsize=7, color='#AA44AA')
ax.text(-SIDE_OUTER + 10, -SIDE_FWD_REAR + 10, 'Side L',
         ha='left',  fontsize=7, color='#AA44AA')

# ── 정적: 레이어 바운딩 박스 ──────────────────────────────────────────────────
for layer in LAYERS:
    ax.add_patch(patches.Rectangle(
        (-layer['horiz_th'], layer['fwd_min']),
        layer['horiz_th'] * 2, layer['fwd_max'] - layer['fwd_min'],
        linewidth=1.5, edgecolor=layer['color'], facecolor=layer['color'],
        alpha=0.13, label=layer['name'], zorder=3
    ))

# ── 정적: STOP 구역 ───────────────────────────────────────────────────────────
ax.add_patch(patches.Rectangle(
    (-STOP_HORIZ_TH, STOP_FWD_MIN), STOP_HORIZ_TH*2, STOP_FWD_MAX-STOP_FWD_MIN,
    linewidth=2, edgecolor='red', facecolor='red', alpha=0.25,
    label='STOP', zorder=4
))

# ── 정적: 로봇 본체 ───────────────────────────────────────────────────────────
ax.add_patch(patches.FancyBboxPatch(
    (-ROBOT_HALF_WIDTH, -80), ROBOT_HALF_WIDTH*2, 240,
    boxstyle='round,pad=5', linewidth=2,
    edgecolor='#333', facecolor='#888', alpha=0.6, zorder=5
))
ax.annotate('', xy=(0,200), xytext=(0,90),
            arrowprops=dict(arrowstyle='->', color='black', lw=2.5), zorder=6)
ax.text(0, 215, 'fwd', ha='center', fontsize=8)

# ── 정적: 축 설정 ─────────────────────────────────────────────────────────────
ax.set_xlim(-MAX_RANGE_MM, MAX_RANGE_MM)
ax.set_ylim(-MAX_RANGE_MM*0.4, MAX_RANGE_MM)
ax.set_aspect('equal')
ax.set_xticks(range(-MAX_RANGE_MM, MAX_RANGE_MM+1, 100))
ax.set_yticks(range(-300, MAX_RANGE_MM+1, 100))
ax.tick_params(labelsize=7)
ax.axhline(0, color='gray', lw=0.5, zorder=1)
ax.axvline(0, color='gray', lw=0.5, zorder=1)
ax.set_xlabel('<- Left (mm)  |  Right (mm) ->', fontsize=9)
ax.set_ylabel('Forward (mm)', fontsize=9)
ax.grid(True, alpha=0.2, zorder=0)
ax.legend(fontsize=7, loc='upper right', ncol=2)

# ── 동적 요소 ─────────────────────────────────────────────────────────────────
scan_line,  = ax.plot([], [], '.', color='steelblue', markersize=3, alpha=0.8, zorder=7)
stop_line,  = ax.plot([], [], 'rx', markersize=9, markeredgewidth=2, zorder=9)
path_line,  = ax.plot([], [], '-', color='limegreen', lw=2.5, alpha=0.85, zorder=10)
path_tip,   = ax.plot([], [], 'o', color='limegreen', markersize=7, zorder=11)

# 레이어별 urgency 벡터 (최대 6개)
layer_arrows = [ax.annotate('', xy=(0,0), xytext=(0,0),
                             arrowprops=dict(arrowstyle='->', lw=1.8,
                                             color=L['color'], alpha=0.0),
                             zorder=8)
                for L in LAYERS]

near_text = ax.text(0, -MAX_RANGE_MM*0.33, '', ha='center',
                    fontsize=8, color='orangered', fontweight='bold')
info_box  = ax.text(0.01, 0.01, '', transform=ax.transAxes,
                    fontsize=8, verticalalignment='bottom', fontfamily='monospace',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.85))
title_obj = ax.title
ax.set_title('RPLIDAR C1 - Bounding Box + Path Prediction')

DYNAMIC_ARTISTS = (scan_line, stop_line, path_line, path_tip,
                   near_text, info_box, title_obj, *layer_arrows)

# ── 라이다 연결 ───────────────────────────────────────────────────────────────
print(f"[Visualizer] Connecting: {LIDAR_PORT}")
lidar     = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
scan_iter = lidar.iter_scans(max_buf_meas=2000, min_len=5)
prev_w    = 0.0
print("[Visualizer] Ready. Close window or Ctrl+C to stop.")

# ── 업데이트 ─────────────────────────────────────────────────────────────────
def update(_frame):
    global prev_w

    try:
        scan = next(scan_iter)
    except StopIteration:
        return DYNAMIC_ARTISTS

    # 데이터 변환 (rplidar_c1 → numpy)
    raw = [(a, d) for q, a, d in scan
           if q > 0 and MIN_VALID_MM < d < MAX_RANGE_MM]
    if not raw:
        return DYNAMIC_ARTISTS

    arr    = np.array(raw)
    angles = arr[:, 0]
    dists  = arr[:, 1]

    # angles: 0~360 → normalize to -180~+180
    angles_n = np.where(angles > 180, angles - 360, angles)
    rads   = np.radians(angles_n)
    xs     = dists * np.sin(rads)
    ys     = dists * np.cos(rads)
    horizs = np.abs(xs)

    # 스캔 포인트
    scan_line.set_data(xs, ys)

    # STOP 트리거
    sm = (ys >= STOP_FWD_MIN) & (ys <= STOP_FWD_MAX) & (horizs < STOP_HORIZ_TH)
    stop_line.set_data(xs[sm], ys[sm]) if np.any(sm) else stop_line.set_data([],[])

    # 최근접
    idx = int(np.argmin(dists))
    nd, na = float(dists[idx]), float(angles_n[idx])
    near_text.set_text(f'nearest: {nd:.0f}mm @ {na:+.1f}deg')

    # v/w 계산
    pts  = list(zip(angles_n.tolist(), dists.tolist()))
    v, w, layer_results, gL, gR = compute_vw(pts)
    w_s  = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
    prev_w = w_s

    # 예측 경로
    px, py = predict_path(v, w_s)
    path_line.set_data(px, py)
    path_tip.set_data([px[-1]], [py[-1]])

    # 레이어 urgency 벡터 (레이어 대표점에서 horiz 방향 반발력 표시)
    ARROW_SCALE = 80   # urgency 1.0 = 80mm 길이
    for i, arrow in enumerate(layer_arrows):
        if i < len(layer_results):
            r   = layer_results[i]
            rx  = r['rep_horiz'] * (1 if r['rep_angle'] > 0 else -1)
            ry  = r['rep_fwd']
            # 반발 방향: 장애물 반대쪽 (horiz 방향 반전)
            dx  = -rx / (abs(rx) + 1e-9) * r['urgency'] * ARROW_SCALE
            dy  = 0.0
            arrow.set_position((rx, ry))
            arrow.xy = (rx + dx, ry + dy)
            arrow.xytext = (rx, ry)
            arrow.arrowprops['alpha'] = min(0.9, r['weight'])
        else:
            arrow.arrowprops['alpha'] = 0.0

    # 정보 박스
    stop_str = '  *** STOP ***' if np.any(sm) else ''
    layer_str = '  '.join(f"{r['name']}:u={r['urgency']:.1f}" for r in layer_results)
    info_box.set_text(
        f" v  = {v:+.3f} m/s\n"
        f" w  = {w_s:+.3f} rad/s\n"
        f" gapL={gL:.0f}mm  gapR={gR:.0f}mm\n"
        f" {layer_str}"
    )
    info_box.get_bbox_patch().set_facecolor('salmon' if np.any(sm) else 'white')

    title_obj.set_text(
        f'Bounding Box + Path  |  {len(raw)}pts  '
        f'v={v:.2f} w={w_s:+.2f}{stop_str}'
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

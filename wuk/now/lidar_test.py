#!/usr/bin/env python3
"""
LIDAR 실시간 테스트 — 모터 없이 라이다 데이터만 시각화
Usage: python3 lidar_test.py
"""
import serial, time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

# ── 설정 ──────────────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
BAUDRATE_LIDAR   = 460800
LIDAR_MIN_VALID  = 100
DETECTION_RANGE  = 1500
ROBOT_HALF_WIDTH = 110

STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 180
STOP_HORIZ_TH = 105

UPDATE_INTERVAL = 150   # ms: 갱신 주기 (낮출수록 빠르나 CPU 사용 증가)

LAYERS = [
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':200, 'color':'#FF4444'},
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':190, 'color':'#FF8800'},
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':140, 'color':'#DDCC00'},
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':140, 'color':'#88CC00'},
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':120, 'color':'#00BB44'},
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':100, 'color':'#0088CC'},
]

# ── 유틸 ──────────────────────────────────────────────────────────────────────
def normalize_angle(a):
    return a - 360 if a > 180 else a

def parse_packet(data):
    if len(data) != 5: return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    distance_q2 = data[3] | (data[4] << 8)
    return angle_q6 / 64.0, distance_q2 / 4.0

# ── 라이다 연결 ───────────────────────────────────────────────────────────────
lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
lidar.write(bytes([0xA5, 0x40]))
time.sleep(1)
lidar.write(bytes([0xA5, 0x20]))
lidar.read(7)
print("LIDAR connected. Ctrl+C to stop.")

# ── 화면 구성 ─────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 8))
ax1 = fig.add_subplot(1, 2, 1)
ax2 = fig.add_subplot(1, 2, 2)

# ── ax1: 정적 요소 (한 번만 그림) ─────────────────────────────────────────────
for layer in LAYERS:
    ax1.add_patch(patches.Rectangle(
        (-layer['horiz_th'], layer['fwd_min']),
        layer['horiz_th'] * 2, layer['fwd_max'] - layer['fwd_min'],
        linewidth=1.5, edgecolor=layer['color'], facecolor=layer['color'],
        alpha=0.12, label=layer['name'], zorder=2
    ))

ax1.add_patch(patches.Rectangle(
    (-STOP_HORIZ_TH, STOP_FWD_MIN), STOP_HORIZ_TH * 2, STOP_FWD_MAX - STOP_FWD_MIN,
    linewidth=2, edgecolor='red', facecolor='red', alpha=0.2, label='STOP', zorder=3
))
ax1.add_patch(patches.FancyBboxPatch(
    (-ROBOT_HALF_WIDTH, -80), ROBOT_HALF_WIDTH * 2, 240,
    boxstyle='round,pad=5', linewidth=2,
    edgecolor='#333', facecolor='#888', alpha=0.6, zorder=4
))
ax1.annotate('', xy=(0, 200), xytext=(0, 90),
             arrowprops=dict(arrowstyle='->', color='black', lw=2.5), zorder=5)
ax1.text(0, 220, 'fwd', ha='center', fontsize=8)

ax1.set_xlim(-1600, 1600)
ax1.set_ylim(-600, 1800)
ax1.set_aspect('equal')
ax1.axhline(0, color='gray', lw=0.5, zorder=1)
ax1.axvline(0, color='gray', lw=0.5, zorder=1)
ax1.set_xlabel('<- Left  |  Lateral (mm)  |  Right ->')
ax1.set_ylabel('Forward (mm)')
ax1.set_title('Top-down View  (Layer boxes + STOP zone)')
ax1.grid(True, alpha=0.2, zorder=0)
ax1.legend(fontsize=7, loc='upper right')

# ── ax1: 동적 요소 ────────────────────────────────────────────────────────────
scan_line, = ax1.plot([], [], '.', color='steelblue', markersize=3, alpha=0.8, zorder=6)
stop_line, = ax1.plot([], [], 'rx', markersize=8, markeredgewidth=2, zorder=8)
near_ann   = ax1.text(0, -460, '', ha='center', fontsize=9,
                      color='orangered', fontweight='bold')

# ── ax2: 정적 요소 ────────────────────────────────────────────────────────────
ax2.set_xlim(-180, 180)
ax2.set_ylim(0, DETECTION_RANGE)
ax2.set_xlabel('Angle (deg)   <- Left (-) | (+) Right ->')
ax2.set_ylabel('Distance (mm)')
ax2.set_title('Distance vs Angle  (360 deg)')
ax2.grid(True, alpha=0.3)
ax2.axhline(STOP_FWD_MIN, color='red', lw=1.2, linestyle='--',
            alpha=0.6, label=f'STOP min {STOP_FWD_MIN}mm')
ax2.axhline(STOP_FWD_MAX, color='red', lw=1.2, linestyle='-.',
            alpha=0.6, label=f'STOP max {STOP_FWD_MAX}mm')
ax2.axvline(0, color='gray', lw=0.8, linestyle=':')
ax2.legend(fontsize=8, loc='upper right')

# ── ax2: 동적 요소 ────────────────────────────────────────────────────────────
dist_line,  = ax2.plot([], [], '.', color='steelblue', markersize=3, alpha=0.7)
near_vline  = ax2.axvline(0, color='orange', lw=1.8, linestyle='--', alpha=0.0)
near_hline  = ax2.axhline(0, color='orange', lw=1.2, linestyle=':', alpha=0.0)
near_label  = ax2.text(0, 0, '', fontsize=8, color='darkorange',
                       fontweight='bold', va='bottom')

fig.suptitle('LIDAR Real-time Monitor', fontsize=11, fontweight='bold')
dyn_title = ax1.title   # ax1 title을 동적 정보 표시에 사용

DYNAMIC_ARTISTS = (scan_line, stop_line, near_ann,
                   dist_line, near_vline, near_hline, near_label, dyn_title)

# ── 스캔 버퍼 ─────────────────────────────────────────────────────────────────
current_scan = []
display_scan = []

# ── 업데이트 (numpy 벡터 연산으로 최적화) ────────────────────────────────────
def update(_frame):
    global current_scan, display_scan

    # 시리얼 버퍼 비우기
    while lidar.in_waiting >= 5:
        raw = lidar.read(5)
        result = parse_packet(raw)
        if result is None:
            continue
        angle_raw, distance = result
        if (raw[0] & 0x01) == 1 and current_scan:
            display_scan = list(current_scan)
            current_scan = []
        current_scan.append((normalize_angle(angle_raw), distance))

    if not display_scan:
        return DYNAMIC_ARTISTS

    # numpy 변환 (리스트 컴프리헨션 대신 벡터 연산)
    arr = np.array(display_scan)
    mask = (arr[:, 1] > LIDAR_MIN_VALID) & (arr[:, 1] < DETECTION_RANGE)
    arr  = arr[mask]
    if len(arr) == 0:
        return DYNAMIC_ARTISTS

    angles_np = arr[:, 0]
    dists_np  = arr[:, 1]
    rads      = np.radians(angles_np)
    xs        = dists_np * np.sin(rads)
    ys        = dists_np * np.cos(rads)   # fwd
    horizs    = np.abs(xs)

    # 탑뷰 스캔 포인트
    scan_line.set_data(xs, ys)

    # STOP 트리거 (numpy 마스크)
    stop_mask = ((ys >= STOP_FWD_MIN) & (ys <= STOP_FWD_MAX) & (horizs < STOP_HORIZ_TH))
    if np.any(stop_mask):
        stop_line.set_data(xs[stop_mask], ys[stop_mask])
    else:
        stop_line.set_data([], [])

    # 최근접 포인트
    idx = int(np.argmin(dists_np))
    na, nd = float(angles_np[idx]), float(dists_np[idx])
    near_ann.set_text(f'nearest: {nd:.0f}mm @ {na:+.1f}°')

    # 거리 vs 각도
    dist_line.set_data(angles_np, dists_np)
    near_vline.set_xdata([na]); near_vline.set_alpha(0.8)
    near_hline.set_ydata([nd]); near_hline.set_alpha(0.6)
    near_label.set_position((na + 5, nd + 20))
    near_label.set_text(f'{nd:.0f}mm\n{na:+.1f}°')

    # 레이어별 포인트 수 (numpy 마스크로 일괄 계산)
    counts = []
    for layer in LAYERS:
        m = ((ys >= layer['fwd_min']) & (ys < layer['fwd_max']) &
             (horizs < layer['horiz_th']))
        counts.append(f"{layer['name']}:{np.sum(m)}")

    stop_n = int(np.sum(stop_mask))
    stop_str = f"  STOP! ({stop_n}pts)" if stop_n else ""
    dyn_title.set_text(
        f"Top-down View  |  {len(arr)}pts  nearest {nd:.0f}mm @ {na:+.1f}deg{stop_str}"
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

"""
RPLIDAR C1 바운딩 박스 시각화 도구
====================================
용도:
    jw_won.py 레이어 바운딩 박스 + STOP zone 상시 표시
    라이다 마운트 방향 및 장애물 인식 범위 확인

실행:
    python3 lidar_visualizer.py

제어:
    Ctrl+C 또는 창 닫기 -> 종료
"""
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from rplidar_c1 import RPLidar

# ── 설정 ──────────────────────────────────────────────────────────────────────
LIDAR_PORT      = '/dev/ttyUSB0'
LIDAR_BAUDRATE  = 460800
MAX_RANGE_MM    = 800    # 표시 최대 거리 (mm)
MIN_VALID_MM    = 100    # 노이즈 컷 (mm)
UPDATE_INTERVAL = 100    # ms

# ── jw_won.py 바운딩 박스 파라미터 ───────────────────────────────────────────
ROBOT_HALF_WIDTH = 110
STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 180
STOP_HORIZ_TH = 105

LAYERS = [
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':200, 'color':'#FF4444'},
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':190, 'color':'#FF8800'},
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':140, 'color':'#DDCC00'},
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':140, 'color':'#88CC00'},
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':120, 'color':'#00BB44'},
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':100, 'color':'#0088CC'},
]

# ── 화면 구성 ─────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 10))

# ── 정적 요소: 바운딩 박스 (한 번만 그림) ────────────────────────────────────
for layer in LAYERS:
    ax.add_patch(patches.Rectangle(
        (-layer['horiz_th'], layer['fwd_min']),
        layer['horiz_th'] * 2, layer['fwd_max'] - layer['fwd_min'],
        linewidth=1.5, edgecolor=layer['color'], facecolor=layer['color'],
        alpha=0.13, label=layer['name'], zorder=2
    ))

ax.add_patch(patches.Rectangle(
    (-STOP_HORIZ_TH, STOP_FWD_MIN), STOP_HORIZ_TH * 2, STOP_FWD_MAX - STOP_FWD_MIN,
    linewidth=2, edgecolor='red', facecolor='red',
    alpha=0.25, label='STOP', zorder=3
))

ax.add_patch(patches.FancyBboxPatch(
    (-ROBOT_HALF_WIDTH, -80), ROBOT_HALF_WIDTH * 2, 240,
    boxstyle='round,pad=5', linewidth=2,
    edgecolor='#333', facecolor='#888', alpha=0.6, zorder=4
))
ax.annotate('', xy=(0, 200), xytext=(0, 90),
            arrowprops=dict(arrowstyle='->', color='black', lw=2.5), zorder=5)
ax.text(0, 215, 'fwd', ha='center', fontsize=8)

# 100mm 격자
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

# ── 동적 요소 ─────────────────────────────────────────────────────────────────
scan_line, = ax.plot([], [], '.', color='steelblue', markersize=3,
                     alpha=0.85, zorder=6)
stop_line, = ax.plot([], [], 'rx', markersize=9, markeredgewidth=2, zorder=8)
near_text  = ax.text(0, -MAX_RANGE_MM * 0.3, '', ha='center', fontsize=8,
                     color='orangered', fontweight='bold')
title_obj  = ax.title
ax.set_title('RPLIDAR C1 - Bounding Box View')

DYNAMIC_ARTISTS = (scan_line, stop_line, near_text, title_obj)

# ── 라이다 연결 ───────────────────────────────────────────────────────────────
print(f"[Visualizer] Connecting: {LIDAR_PORT}")
lidar     = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
scan_iter = lidar.iter_scans(max_buf_meas=2000, min_len=5)
print("[Visualizer] Ready. Close window or Ctrl+C to stop.")

# ── 업데이트 ─────────────────────────────────────────────────────────────────
def update(_frame):
    try:
        scan = next(scan_iter)
    except StopIteration:
        return DYNAMIC_ARTISTS

    # rplidar_c1 -> numpy 변환
    raw = [(q, a, d) for q, a, d in scan
           if q > 0 and MIN_VALID_MM < d < MAX_RANGE_MM]
    if not raw:
        return DYNAMIC_ARTISTS

    arr    = np.array([(d * math.sin(math.radians(a)),   # x (lateral)
                        d * math.cos(math.radians(a)))    # y (forward)
                       for _, a, d in raw])
    dists  = np.array([d for _, _, d in raw])
    xs, ys = arr[:, 0], arr[:, 1]
    horizs = np.abs(xs)

    # 스캔 포인트
    scan_line.set_data(xs, ys)

    # STOP 트리거
    stop_mask = (ys >= STOP_FWD_MIN) & (ys <= STOP_FWD_MAX) & (horizs < STOP_HORIZ_TH)
    if np.any(stop_mask):
        stop_line.set_data(xs[stop_mask], ys[stop_mask])
    else:
        stop_line.set_data([], [])

    # 최근접
    idx = int(np.argmin(dists))
    nd  = float(dists[idx])
    na  = float(math.degrees(math.atan2(xs[idx], ys[idx])))
    near_text.set_text(f'nearest: {nd:.0f}mm @ {na:+.1f}deg')

    stop_str = '  *** STOP ***' if np.any(stop_mask) else ''
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

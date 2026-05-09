"""
RPLIDAR C1 독립 실행형 시각화 도구
====================================
용도:
    1) LiDAR 마운트 방향 검증 (0°가 진짜 로봇 전방인지)
    2) 측정 노이즈/사각지대 확인
    3) 장애물 인식 거리 확인

실행:
    python3 lidar_visualizer.py

제어:
    Ctrl+C 또는 창 닫기 → 종료
"""

import math
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from rplidar c1 import RPLidar


# ============================================================
# 설정
# ============================================================
LIDAR_PORT      = '/dev/ttyUSB0'
MAX_RANGE       = 3.5             # m, 표시 최대 거리
MIN_VALID_RANGE = 0.10            # m, 노이즈 컷
UPDATE_INTERVAL = 100             # ms, 화면 갱신 주기
LIDAR_BAUDRATE  = 460800            # ← 이 줄 추가 (C1 전용)
MAX_RANGE       = 3.5

# ============================================================
# 폴라 플롯 설정
# ============================================================
fig = plt.figure(figsize=(8, 8))
ax  = fig.add_subplot(111, projection='polar')

# LiDAR 좌표 ↔ matplotlib 폴라 좌표 정합
ax.set_theta_zero_location('N')   # 0° = 위쪽 (= 로봇 전방)
ax.set_theta_direction(-1)         # 시계방향 양수 (= LiDAR 규약)

ax.set_rmax(MAX_RANGE)
ax.set_rticks([0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
ax.set_rlabel_position(135)        # 거리 라벨 위치
ax.grid(True, alpha=0.4)
ax.set_title('RPLIDAR C1 Live Scan  (Front = Top, CW = +)\n'
             'Verify: object directly in front should appear at TOP',
             fontsize=11, pad=20)

# 거리별 색상 그라디언트로 표시 (가까울수록 빨강)
scatter = ax.scatter([], [], s=8, c=[], cmap='RdYlBu_r',
                      vmin=0, vmax=MAX_RANGE, alpha=0.8)

# 정면 참고선
ax.plot([0, 0], [0, MAX_RANGE], 'r--', alpha=0.4, linewidth=1.5,
        label='Front (0°)')
# 좌우 90° 참고선
ax.plot([math.radians(90), math.radians(90)], [0, MAX_RANGE],
        'b--', alpha=0.3, linewidth=1, label='Right (+90°)')
ax.plot([math.radians(-90), math.radians(-90)], [0, MAX_RANGE],
        'g--', alpha=0.3, linewidth=1, label='Left (-90°)')
ax.legend(loc='upper right', fontsize=9)

# 정면 거리 텍스트 표시용
front_text = ax.text(0.5, -0.08, '', transform=ax.transAxes,
                     ha='center', fontsize=10,
                     bbox=dict(boxstyle='round', facecolor='lightyellow'))


# ============================================================
# LiDAR 연결
# ============================================================
print(f"[Visualizer] LiDAR 연결 중: {LIDAR_PORT}")
lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
try:
    info = lidar.get_info()
    print(f"[Visualizer] Info: {info}")
    health = lidar.get_health()
    print(f"[Visualizer] Health: {health}")
except Exception as e:
    print(f"[Visualizer] Info/Health 조회 실패 (무시): {e}")

scan_iter = lidar.iter_scans(max_buf_meas=2000, min_len=5)


# ============================================================
# 갱신 함수
# ============================================================
def update(frame):
    try:
        scan = next(scan_iter)
    except StopIteration:
        return scatter, front_text

    angles, distances = [], []
    front_min = MAX_RANGE  # 정면 ±5° 최소 거리

    for (quality, angle_deg, distance_mm) in scan:
        if quality == 0 or distance_mm == 0:
            continue
        d_m = distance_mm / 1000.0
        if d_m < MIN_VALID_RANGE or d_m > MAX_RANGE:
            continue

        angles.append(math.radians(angle_deg))
        distances.append(d_m)

        # 정면 ±5° 범위 최소 거리 추적
        if angle_deg <= 5 or angle_deg >= 355:
            if d_m < front_min:
                front_min = d_m

    if angles:
        offsets = np.column_stack([angles, distances])
        scatter.set_offsets(offsets)
        scatter.set_array(np.array(distances))

    # 정면 거리 표시
    if front_min < MAX_RANGE:
        front_text.set_text(f'Front (0°±5°) min: {front_min:.2f} m')
    else:
        front_text.set_text('Front (0°±5°): clear')

    return scatter, front_text


# ============================================================
# 메인
# ============================================================
ani = animation.FuncAnimation(fig, update, interval=UPDATE_INTERVAL,
                               blit=True, cache_frame_data=False)

print("[Visualizer] 시각화 시작 — 창을 닫거나 Ctrl+C로 종료")
print("            로봇 정면에 물체를 두면 위쪽에 표시되는지 확인하세요.")

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    print("[Visualizer] 종료 중...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print("[Visualizer] 종료 완료")

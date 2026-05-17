#!/usr/bin/env python3
"""
LIDAR 실시간 텍스트 출력 — 그래프 없이 수치만 표시
Usage: python3 lidar_test.py
"""
import serial, time
import numpy as np

# ── 설정 ──────────────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
BAUDRATE_LIDAR   = 460800
LIDAR_MIN_VALID  = 100
DETECTION_RANGE  = 1500
ROBOT_HALF_WIDTH = 110

STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 180
STOP_HORIZ_TH = 105

# 출력할 각도 구간 (None 이면 전체 360도 출력)
PRINT_ANGLE_MIN = -90
PRINT_ANGLE_MAX =  90

LAYERS = [
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':200},
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':190},
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':140},
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':140},
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':120},
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':100},
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
print(f"Showing angles: {PRINT_ANGLE_MIN} ~ {PRINT_ANGLE_MAX} deg")
print("=" * 60)

# ── 메인 루프 ─────────────────────────────────────────────────────────────────
current_scan = []
scan_count   = 0

try:
    while True:
        raw = lidar.read(5)
        result = parse_packet(raw)
        if result is None:
            continue

        angle_raw, distance = result

        if (raw[0] & 0x01) == 1 and current_scan:
            scan_count += 1
            pts = [(a, d) for a, d in current_scan
                   if LIDAR_MIN_VALID < d < DETECTION_RANGE]

            if pts:
                arr      = np.array(pts)
                angles   = arr[:, 0]
                dists    = arr[:, 1]
                rads     = np.radians(angles)
                xs       = dists * np.sin(rads)
                ys       = dists * np.cos(rads)
                horizs   = np.abs(xs)

                # 최근접
                idx     = int(np.argmin(dists))
                nearest = (float(angles[idx]), float(dists[idx]))

                # STOP 여부
                stop_mask = (ys >= STOP_FWD_MIN) & (ys <= STOP_FWD_MAX) & (horizs < STOP_HORIZ_TH)
                stop_n    = int(np.sum(stop_mask))

                # 레이어 카운트
                layer_info = []
                for layer in LAYERS:
                    m = ((ys >= layer['fwd_min']) & (ys < layer['fwd_max']) &
                         (horizs < layer['horiz_th']))
                    layer_info.append(f"{layer['name']}:{int(np.sum(m))}")

                # 출력 범위 필터
                view_mask = (angles >= PRINT_ANGLE_MIN) & (angles <= PRINT_ANGLE_MAX)
                view_pts  = arr[view_mask]
                view_pts  = view_pts[np.argsort(view_pts[:, 0])]  # 각도 순 정렬

                print(f"\n[Scan #{scan_count}]  total={len(pts)}pts  "
                      f"nearest={nearest[1]:.0f}mm @ {nearest[0]:+.1f}deg  "
                      f"STOP={'YES (' + str(stop_n) + 'pts)' if stop_n else 'no'}")
                print(f"  Layers: {' | '.join(layer_info)}")
                print(f"  --- angle/dist ({PRINT_ANGLE_MIN}~{PRINT_ANGLE_MAX} deg) ---")
                for a, d in view_pts:
                    bar_len = int(d / 50)           # 50mm = 1칸
                    bar     = '#' * min(bar_len, 30)
                    print(f"  {a:+7.1f}deg  {d:6.0f}mm  {bar}")

            current_scan = []

        current_scan.append((normalize_angle(angle_raw), distance))

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    lidar.write(bytes([0xA5, 0x25]))
    time.sleep(0.1)
    lidar.close()

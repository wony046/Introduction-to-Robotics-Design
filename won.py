#!/usr/bin/env python3
"""
Follow-the-Gap (FTG) Navigation for Differential Drive Robot
============================================================

시나리오: 좌우 벽 + 중앙 장애물 통로를 멈추지 않고 부드럽게 통과

흐름:이이잉이
  RPLidar C1  →  전처리 (FOV 클립 + 평활)  →  Safety Bubble
              →  Largest Gap 탐색  →  목표각 산출 + LPF
              →  (v, w) 계산  →  UART → Arduino ("v,w\n")

요구 라이브러리:
    pip install rplidar-roboticia numpy pyserial
"""

import math
import time
import signal
import sys
import serial
import numpy as np
from rplidar import RPLidar


# ============================================================
# 설정
# ============================================================
# ----- 시리얼 -----
LIDAR_PORT     = '/dev/ttyUSB0'
LIDAR_BAUD     = 460800       # RPLidar C1
ARDUINO_PORT   = '/dev/ttyUSB1'
ARDUINO_BAUD   = 9600

# ----- 라이다 → 로봇 프레임 보정 -----
# RPLidar C1: 0°가 로봇 정면이 아닐 경우 오프셋 (deg)
LIDAR_ANGLE_OFFSET_DEG = 0.0
# 각도 증가 방향이 반대일 경우 True (양수 각이 좌측이 되도록)
LIDAR_FLIP_DIRECTION   = False

# ----- 시야 / 처리 -----
FOV_HALF_DEG   = 90.0         # 정면 ±90°만 사용 (후방 무시)
RANGE_MAX      = 4.0          # m, 이상은 RANGE_MAX로 클립
RANGE_MIN      = 0.10         # m, 이내는 노이즈로 간주 (0 처리)
SMOOTH_KSIZE   = 5            # 거리 이동평균 윈도우 (홀수)

# ----- Safety Bubble -----
BUBBLE_RADIUS  = 0.30         # m, 가장 가까운 점 주변 무효화 반경

# ----- 속도 정책 -----
V_MAX          = 0.30         # m/s, 최대 직진 속도
V_MIN          = 0.10         # m/s, ★ 멈추지 않기 위한 최저속
W_MAX          = 1.5          # rad/s, 최대 각속도
KP_W           = 1.6          # 헤딩 P 게인

# 부드러운 추종을 위한 1차 LPF (alpha 클수록 즉응, 작을수록 부드러움)
ALPHA_TARGET   = 0.4          # 목표각 LPF
ALPHA_VW       = 0.5          # (v, w) 명령 LPF

# ----- 비상 거리 -----
EMERG_DIST     = 0.20         # m, 정면 콘 내 이 거리보다 가까우면 V_MIN으로

# ----- 디버그 -----
PRINT_HZ       = 5


# ============================================================
# 신호 처리 유틸
# ============================================================
def moving_average(arr, k):
    """1D 이동평균 (윈도우 k, 양 끝은 same 모드)."""
    if k <= 1:
        return arr
    kernel = np.ones(k) / k
    return np.convolve(arr, kernel, mode='same')


def scan_to_arrays(scan):
    """
    rplidar 스캔 [(quality, angle_deg_0~360, distance_mm), ...]
        → 로봇 프레임의 (angles_rad ∈ [-π, π], ranges_m)
    각도 오름차순으로 정렬되어 반환됨.
    """
    if not scan:
        return None, None

    a_deg = np.array([s[1] for s in scan], dtype=float)
    r_mm  = np.array([s[2] for s in scan], dtype=float)

    # 라이다 → 로봇 프레임
    a_deg = a_deg + LIDAR_ANGLE_OFFSET_DEG
    if LIDAR_FLIP_DIRECTION:
        a_deg = -a_deg
    # 0~360 → -180~180 으로 wrap
    a_deg = ((a_deg + 180.0) % 360.0) - 180.0

    a_rad = np.deg2rad(a_deg)
    r_m   = r_mm / 1000.0
    # 거리 0(=무효 측정)는 RANGE_MAX로 처리
    r_m = np.where(r_m <= 0, RANGE_MAX, r_m)

    order = np.argsort(a_rad)
    return a_rad[order], r_m[order]


# ============================================================
# FTG 핵심 알고리즘
# ============================================================
def preprocess(angles, ranges):
    """FOV 마스킹 + 클립 + 평활."""
    fov = math.radians(FOV_HALF_DEG)
    mask = np.abs(angles) <= fov
    a = angles[mask]
    r = ranges[mask].copy()

    # NaN/inf 안전 처리
    r = np.where(np.isfinite(r), r, RANGE_MAX)
    r = np.clip(r, 0.0, RANGE_MAX)

    # 너무 가까운 값은 센서 노이즈/거품 → 0(차단)으로
    r = np.where(r < RANGE_MIN, 0.0, r)

    # 평활
    r = moving_average(r, SMOOTH_KSIZE)
    return a, r


def safety_bubble(angles, ranges):
    """
    가장 가까운 유효점 주변을 BUBBLE_RADIUS만큼 0으로 마스킹.
    거리 d에서 반경 R이 만드는 각도 폭은 atan2(R, d).
    """
    valid = ranges > 0
    if not np.any(valid):
        return ranges

    masked = np.where(valid, ranges, np.inf)
    idx = int(np.argmin(masked))
    d   = ranges[idx]
    if d <= 0:
        return ranges

    half_ang = math.atan2(BUBBLE_RADIUS, max(d, 0.05))
    out = ranges.copy()
    out[np.abs(angles - angles[idx]) <= half_ang] = 0.0
    return out


def find_largest_gap(ranges):
    """
    가장 긴 연속 nonzero 구간 [start, end] (둘 다 포함).
    빈 갭이면 (0, -1) 반환.
    """
    n = len(ranges)
    best_s, best_e, best_len = 0, -1, 0
    i = 0
    while i < n:
        if ranges[i] > 0:
            j = i
            while j < n and ranges[j] > 0:
                j += 1
            length = j - i
            if length > best_len:
                best_s, best_e, best_len = i, j - 1, length
            i = j
        else:
            i += 1
    return best_s, best_e


def best_point_in_gap(start, end, ranges, mode='midpoint'):
    """
    갭 안의 목표 인덱스 선택.
        midpoint : 갭 중점 (가장 부드럽고 안정적) ★ 권장
        deepest  : 갭 안 최대 거리 (공격적, 끝에서 떨림 가능)
    """
    if mode == 'deepest':
        return start + int(np.argmax(ranges[start:end + 1]))
    return (start + end) // 2


def front_clearance(angles, ranges, half_cone_deg=15):
    """정면 좁은 콘에서의 최소 거리 (속도 결정용)."""
    cone = math.radians(half_cone_deg)
    mask = (np.abs(angles) <= cone) & (ranges > 0)
    if not np.any(mask):
        return RANGE_MAX
    return float(np.min(ranges[mask]))


# ============================================================
# 속도 명령 산출
# ============================================================
def compute_velocity(target_angle, clearance):
    """
    목표각, 정면거리 → (v, w)
        - 회전이 클수록 v 감소
        - 정면이 가까울수록 v 감소
        - V_MIN으로 하한 보장 (멈추지 않음)
    """
    # 회전 의존: |angle|=60° 이상이면 0.3까지 감소
    turn_factor = max(0.3, 1.0 - abs(target_angle) / math.radians(60))
    # 거리 의존: 1.5m 이상은 1.0, 가까울수록 0.4까지 감소
    dist_factor = float(np.clip(clearance / 1.5, 0.4, 1.0))

    v = V_MAX * min(turn_factor, dist_factor)
    if clearance < EMERG_DIST:
        v = V_MIN                            # 비상 시도 멈추지 않고 천천히
    v = max(V_MIN, v)

    # 헤딩 P 제어
    w = float(np.clip(KP_W * target_angle, -W_MAX, W_MAX))
    return v, w


# ============================================================
# 시리얼 송신
# ============================================================
def send_cmd(ser, v, w):
    msg = f"{v:.3f},{w:.3f}\n"
    ser.write(msg.encode())


# ============================================================
# Main
# ============================================================
def main():
    # ----- Arduino 연결 -----
    print(f"[Arduino] {ARDUINO_PORT} @ {ARDUINO_BAUD}")
    ardu = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
    time.sleep(2.0)              # 부팅/리셋 대기
    send_cmd(ardu, 0.0, 0.0)

    # ----- LiDAR 연결 -----
    print(f"[LiDAR]   {LIDAR_PORT} @ {LIDAR_BAUD}")
    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD)
    lidar.connect()
    lidar.start_motor()
    time.sleep(1.0)

    # ----- 종료 핸들러 -----
    shutdown_done = {'v': False}
    def shutdown(*_):
        if shutdown_done['v']:
            return
        shutdown_done['v'] = True
        print("\n[종료] 정지 명령 전송, 정리 중...")
        try: send_cmd(ardu, 0.0, 0.0)
        except Exception: pass
        try: time.sleep(0.1); lidar.stop()
        except Exception: pass
        try: lidar.stop_motor()
        except Exception: pass
        try: lidar.disconnect()
        except Exception: pass
        try: ardu.close()
        except Exception: pass
        sys.exit(0)
    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # ----- 상태 변수 -----
    target_lpf = 0.0
    v_lpf      = 0.0
    w_lpf      = 0.0
    last_print = 0.0

    print("[READY] 주행 시작 (Ctrl+C 종료)\n")

    try:
        for scan in lidar.iter_scans(min_len=80):
            t = time.time()

            angles, ranges = scan_to_arrays(scan)
            if angles is None or len(angles) < 20:
                continue

            # 1) 전처리
            a, r = preprocess(angles, ranges)

            # 2) Safety bubble (가장 가까운 위협 제거)
            r = safety_bubble(a, r)

            # 3) Largest gap 탐색
            s, e = find_largest_gap(r)

            if e >= s:
                idx = best_point_in_gap(s, e, r, mode='midpoint')
                target = float(a[idx])
            else:
                # 모든 방향 막힘 → 직전 방향으로 살살 회전 (멈추지 않음)
                target = math.radians(60) if target_lpf >= 0 else math.radians(-60)

            # 4) 정면 거리 (속도 결정용)
            clr = front_clearance(a, r, half_cone_deg=15)

            # 5) LPF (부드러운 추종 핵심)
            target_lpf = ALPHA_TARGET * target + (1 - ALPHA_TARGET) * target_lpf

            # 6) 속도 산출
            v, w = compute_velocity(target_lpf, clr)
            v_lpf = ALPHA_VW * v + (1 - ALPHA_VW) * v_lpf
            w_lpf = ALPHA_VW * w + (1 - ALPHA_VW) * w_lpf

            # 7) Arduino로 전송
            send_cmd(ardu, v_lpf, w_lpf)

            # 8) 디버그 출력
            if t - last_print > 1.0 / PRINT_HZ:
                gap_w_deg = math.degrees(a[e] - a[s]) if e >= s else 0.0
                print(f"target={math.degrees(target_lpf):+6.1f}°  "
                      f"clr={clr:4.2f}m  "
                      f"gap={gap_w_deg:5.1f}°  "
                      f"v={v_lpf:.2f}  w={w_lpf:+.2f}")
                last_print = t

    except Exception as ex:
        print(f"[ERROR] {ex}")
    finally:
        shutdown()


if __name__ == "__main__":
    main()

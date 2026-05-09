"""
RPLIDAR C1 장애물 회피 코드 (v2 - 선속도/각속도 제어)
- 라이다(ttyUSB0)로 정면 180도 스캔
- 장애물 수평거리 계산 → 충돌 판단
- 선속도(v) / 각속도(w) 를 아두이노로 UART 전송

[하드웨어 연결]
  라이다  : /dev/ttyUSB0  (USB)
  아두이노: /dev/ttyS0    (UART GPIO — GPIO14 TX, GPIO15 RX)
  ※ 라즈베리파이 3.3V ↔ 아두이노 5V 사이에 레벨 컨버터 필수
  ※ 라즈베리파이 TX(GPIO14) → 레벨컨버터 LV → HV → 아두이노 RX
     라즈베리파이 RX(GPIO15) ← 레벨컨버터 LV ← HV ← 아두이노 TX

[UART 활성화 방법 (최초 1회)]
  sudo raspi-config
  → 3 Interface Options → I6 Serial Port
  → login shell over serial: No
  → serial port hardware: Yes
  → 재부팅

[아두이노 전송 포맷]  "v w\n"
  예) "0.20  0.00\n"  → 직진
      "0.20  0.52\n"  → 전진+좌회전
      "0.20 -0.52\n"  → 전진+우회전
      "0.00  0.00\n"  → 정지

[v, w 부호 약속 — 슬라이드 기준]
  v (m/s) : v > 0 → 전진
  w (rad/s): w > 0 → 좌회전(반시계),  w < 0 → 우회전(시계)
"""

import serial
import time
import math

# ── 포트 / 통신 속도 ──────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"   # 라이다 USB 포트
ARDUINO_PORT     = "/dev/ttyS0"     # 아두이노 UART (GPIO14 TX / GPIO15 RX)
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 9600             # 아두이노 Serial1.begin(9600) 과 동일

# ── 로봇 파라미터 ─────────────────────────────────────────────────────────────
ROBOT_HALF_WIDTH = 110      # 라이다 중심 ~ 로봇 좌우 끝 (mm)
SAFETY_MARGIN    = 50       # 안전 여유 (mm)  →  충돌 기준 = 160 mm
DETECTION_RANGE  = 1500     # 탐지 최대 거리 (mm)
EMERGENCY_DIST   = 200      # 비상 정지 거리 (mm): 이보다 가까우면 정지

# ── 속도 파라미터 (튜닝 가능) ─────────────────────────────────────────────────
FORWARD_SPEED = 0.20        # 기본 전진 선속도 (m/s)
W_GAIN        = 1.5         # 회피각도 → 각속도 게인 (rad/s per rad)
MAX_W         = 1.5         # 최대 각속도 상한 (rad/s)

# ── 스캔 파라미터 ─────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE = 90        # 정면 기준 ±탐지 각도 (도)
ANGLE_STEP      = 5         # 각도 버킷 해상도 (도)
SEND_INTERVAL   = 0.1       # 명령 전송 주기 (초)
# ─────────────────────────────────────────────────────────────────────────────


def normalize_angle(angle):
    """라이다 각도(0~360) → 정면 기준 -180~+180"""
    if angle > 180:
        return angle - 360
    return angle


def is_in_front(angle_norm):
    return -SCAN_HALF_ANGLE <= angle_norm <= SCAN_HALF_ANGLE


def parse_packet(data):
    """
    5바이트 라이다 패킷 파싱 + S/S̄/C 비트 검증
    반환: (angle_deg, distance_mm, quality) 또는 None
    """
    if len(data) != 5:
        return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):     # S̄ = not S 검증
        return None
    if (data[1] & 0x01) != 1:          # C 비트 = 1 검증
        return None
    quality     = data[0] >> 2
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    angle       = angle_q6 / 64.0
    distance_q2 = data[3] | (data[4] << 8)
    distance    = distance_q2 / 4.0
    return angle, distance, quality


def compute_horizontal_dist(angle_norm_deg, distance_mm):
    """
    극좌표 → 직교 분해
      horizontal = distance × |sin(angle)|  (좌우 방향 성분)
      forward    = distance × cos(angle)    (전진 방향 성분)
    """
    rad        = math.radians(angle_norm_deg)
    horizontal = abs(distance_mm * math.sin(rad))
    forward    = distance_mm * math.cos(rad)
    return horizontal, forward


def find_vw_command(scan_points):
    """
    정면 스캔 데이터 → (v m/s, w rad/s) 명령 계산

    [알고리즘]
    1. 수평거리 < 충돌기준인 위험 포인트 수집
    2. 가장 가까운 장애물 → reference_dist 결정
    3. reference_dist 기준으로 좌/우 빈 공간 각도 폭 비교
    4. 더 넓은 쪽으로 회피,  회피 각도 → 각속도 w 변환

    반환: (v, w)
    """
    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN

    # 1. 위험 포인트 수집
    danger_points = []
    for angle_norm, dist in scan_points:
        if dist <= 0 or dist > DETECTION_RANGE:
            continue
        horiz, fwd = compute_horizontal_dist(angle_norm, dist)
        if fwd > 0 and horiz < threshold:
            danger_points.append((angle_norm, dist, horiz, fwd))

    if not danger_points:
        return FORWARD_SPEED, 0.0      # 장애물 없음 → 직진

    # 2. 가장 가까운 장애물 (기준 포인트)
    nearest        = min(danger_points, key=lambda p: p[1])
    reference_dist = nearest[1]

    print(f"  [기준장애물] 각도:{nearest[0]:.1f}°  "
          f"거리:{reference_dist:.0f}mm  수평:{nearest[2]:.0f}mm")

    # 3. 비상 정지
    if reference_dist < EMERGENCY_DIST:
        print("  [비상정지] 장애물 너무 가까움!")
        return 0.0, 0.0

    # 4. 각도별 최소 거리 딕셔너리
    scan_dict = {}
    for angle_norm, dist in scan_points:
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    # 5. reference_dist 기준 좌/우 빈 공간 폭 계산
    #   측정거리 >= reference_dist  → 그 방향은 통과 가능 ✅
    #   측정거리 <  reference_dist  → 더 가까운 장애물 존재 ❌
    left_clear  = 0
    right_clear = 0

    for a in range(-SCAN_HALF_ANGLE, 0, ANGLE_STEP):
        d = scan_dict.get(a, DETECTION_RANGE + 1)
        if d >= reference_dist:
            left_clear += ANGLE_STEP

    for a in range(ANGLE_STEP, SCAN_HALF_ANGLE + 1, ANGLE_STEP):
        d = scan_dict.get(a, DETECTION_RANGE + 1)
        if d >= reference_dist:
            right_clear += ANGLE_STEP

    print(f"  [여유공간] 왼쪽:{left_clear}°  오른쪽:{right_clear}°")

    # 6. 최소 필요 회피 각도 (asin 공식)
    ratio       = min(threshold / max(reference_dist, 1.0), 1.0)
    avoid_angle = math.asin(ratio)      # 라디안

    # 7. 회피 방향 + 각속도 w 결정
    #   w > 0 → 좌회전 (슬라이드: v>0, w>0 → 전진하며 좌회전)
    #   w < 0 → 우회전
    if left_clear >= right_clear:
        w = min(W_GAIN * avoid_angle, MAX_W)    # 좌회전 (양수)
        dir_str = "좌회전"
    else:
        w = -min(W_GAIN * avoid_angle, MAX_W)   # 우회전 (음수)
        dir_str = "우회전"

    print(f"  [회피] {dir_str}  "
          f"각도:{math.degrees(avoid_angle):.1f}°  "
          f"v:{FORWARD_SPEED:.2f}m/s  w:{w:.2f}rad/s")

    return FORWARD_SPEED, w


def main():
    print("=== RPLIDAR 장애물 회피 (v/w 제어) ===")
    print(f"  라이다 포트   : {LIDAR_PORT}")
    print(f"  아두이노 포트 : {ARDUINO_PORT}  (UART GPIO14/15)")
    print(f"  전진 속도     : {FORWARD_SPEED} m/s")
    print(f"  로봇 반폭     : {ROBOT_HALF_WIDTH} mm")
    print(f"  충돌 판단 기준: {ROBOT_HALF_WIDTH + SAFETY_MARGIN} mm")
    print(f"  비상 정지 거리: {EMERGENCY_DIST} mm")
    print("=" * 45)

    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)

    lidar.write(bytes([0xA5, 0x40]))    # RESET
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))    # START SCAN
    print("스캔 시작...")
    lidar.read(7)   # 응답 디스크립터 버림

    scan_points  = []
    last_send    = time.time()
    last_cmd_str = ""

    try:
        while True:
            raw    = lidar.read(5)
            result = parse_packet(raw)
            if result is None:
                continue

            angle_raw, distance, quality = result
            s_flag = raw[0] & 0x01

            if s_flag == 1 and scan_points:
                front_points = [
                    (a, d) for a, d in scan_points
                    if is_in_front(a) and d > 0
                ]

                now = time.time()
                if now - last_send >= SEND_INTERVAL:
                    v, w = find_vw_command(front_points)
                    cmd  = f"{v:.2f} {w:.2f}\n"
                    arduino.write(cmd.encode())

                    if cmd != last_cmd_str:
                        print(f"[전송] v={v:.2f}m/s  w={w:.2f}rad/s  "
                              f"(포인트수:{len(front_points)})")
                        last_cmd_str = cmd

                    last_send = now

                scan_points = []

            angle_norm = normalize_angle(angle_raw)
            scan_points.append((angle_norm, distance))

    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        lidar.write(bytes([0xA5, 0x25]))    # STOP
        time.sleep(0.1)
        lidar.close()
        arduino.write(b"0.00 0.00\n")
        arduino.close()
        print("종료 완료.")


if __name__ == "__main__":
    main()

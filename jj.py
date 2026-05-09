"""
RPLIDAR C1 장애물 회피 코드 (v3)
포트: 라이다 /dev/ttyUSB0 / 아두이노 /dev/ttyS0 (UART GPIO14/15)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[회전각도 계산 방식 비교]

 ▶ 구버전: asin(threshold / reference_dist)
   - reference_dist = 장애물까지 사선(직선)거리
   - 문제: 정면/측면 장애물을 동일하게 처리
           각도가 과소평가됨

 ▶ 신버전: atan2(threshold, fwd_dist)
   - fwd_dist = 장애물까지 전방(수직)거리 성분
   - 효과: 전방거리가 짧을수록(가까울수록) 각도가 커짐
           실제 충돌까지 남은 "시간"에 비례한 정확한 회피각

   예시 (threshold=160mm)
   장애물 45° / 400mm → fwd=283mm
     구버전: asin(160/400) = 23.6°
     신버전: atan2(160,283) = 29.5°  ← 더 큰 회피각
   장애물 정면 / 300mm → fwd=300mm
     구버전: asin(160/300) = 32.2°
     신버전: atan2(160,300) = 28.1°
   장애물 10° / 200mm → fwd=197mm, horiz=35mm
     구버전: asin(160/200) = 53.1°  ← 과도함(측면에 가까운데)
     신버전: atan2(160,197) = 39.1°  ← 전방거리 반영

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[위험구역 형태 변경]

 ▶ 구버전: 반구형 (직선거리 기준)
   조건: dist <= DETECTION_RANGE AND horiz < threshold
   → 멀리 있는 측면 장애물도 트리거 가능

 ▶ 신버전: 직사각형 (전방/수평 거리 독립 판단)

   ┌──────────────────────────┐
   │   일반 위험구역           │  ← 회피 기동
   │  fwd ≤ FORWARD_RANGE     │
   │  horiz ≤ threshold       │
   ├──────────────────────────┤
   │   긴급 회피구역           │  ← 속도 감소 + 강화 회피
   │  fwd ≤ EMERGENCY_FWD     │
   │  horiz ≤ threshold       │
   └──────────────────────────┘

   로봇 진행방향 ↑  (위쪽이 정면)
   좌우 = threshold (로봇 폭 + 안전여유)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[긴급 회피 동작]
 - 비상 정지(v=0) 제거
 - 긴급구역 진입 시 속도를 거리에 비례해 감속
   v = EMERGENCY_MIN_SPEED + (FORWARD_SPEED - EMERGENCY_MIN_SPEED)
       × (fwd / EMERGENCY_FWD_RANGE)
 - 동시에 회피 각속도 w를 EMERGENCY_W_MULT 배 강화

[아두이노 전송 포맷]  "v w\n"
  w > 0 → 좌회전,  w < 0 → 우회전
"""

import serial
import time
import math

# ── 포트 ─────────────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyS0"     # UART GPIO14(TX) / GPIO15(RX)
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 9600

# ── 로봇 파라미터 ─────────────────────────────────────────────────────────────
ROBOT_HALF_WIDTH = 110      # 라이다 중심 ~ 로봇 좌우 끝 (mm)
SAFETY_MARGIN    = 50       # 안전 여유 (mm)
# → threshold = 160mm : 수평거리가 이보다 작으면 위험

# ── 직사각형 위험구역 ─────────────────────────────────────────────────────────
DETECTION_RANGE     = 1500  # LiDAR 최대 신뢰 거리 (mm) — 필터용
FORWARD_RANGE       = 800   # 일반 위험구역 전방 깊이 (mm)
EMERGENCY_FWD_RANGE   = 125   # 긴급 회피구역 전방 깊이 (mm)
EMERGENCY_HORIZ_RANGE = 110   # 긴급 회피구역 수평 깊이 (mm) ← 로봇 반폭과 동일

# ── 속도 파라미터 ─────────────────────────────────────────────────────────────
FORWARD_SPEED       = 0.20  # 기본 전진 선속도 (m/s)
EMERGENCY_MIN_SPEED = 0.05  # 긴급 상황 최소 속도 (m/s) — 0으로 세우지 않음
W_GAIN              = 2.0   # 회피각 → 각속도 게인  (구버전 1.5 → 2.0 상향)
MAX_W               = 2.0   # 최대 각속도 (rad/s)    (구버전 1.5 → 2.0 상향)
EMERGENCY_W_MULT    = 1.5   # 긴급 시 각속도 배율 (기본 w × 1.5)

# ── 스캔 파라미터 ─────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE = 90
ANGLE_STEP      = 5
SEND_INTERVAL   = 0.1
# ─────────────────────────────────────────────────────────────────────────────


def normalize_angle(angle):
    """라이다 각도(0~360) → 정면 기준 -180~+180"""
    return angle - 360 if angle > 180 else angle


def is_in_front(angle_norm):
    return -SCAN_HALF_ANGLE <= angle_norm <= SCAN_HALF_ANGLE


def parse_packet(data):
    """5바이트 라이다 패킷 파싱 + S/S̄/C 비트 검증"""
    if len(data) != 5:
        return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        return None
    if (data[1] & 0x01) != 1:
        return None
    quality     = data[0] >> 2
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    angle       = angle_q6 / 64.0
    distance_q2 = data[3] | (data[4] << 8)
    distance    = distance_q2 / 4.0
    return angle, distance, quality


def decompose(angle_norm_deg, distance_mm):
    """
    극좌표 → 직교 분해
      horiz = |distance × sin(angle)|  ← 수평(좌우) 성분
      fwd   =  distance × cos(angle)   ← 전방 성분
    """
    rad   = math.radians(angle_norm_deg)
    horiz = abs(distance_mm * math.sin(rad))
    fwd   = distance_mm * math.cos(rad)
    return horiz, fwd


def find_vw_command(scan_points):
    """
    정면 스캔 → (v m/s, w rad/s) 계산

    [직사각형 위험구역]
      일반구역 : fwd ≤ FORWARD_RANGE       AND horiz < threshold
      긴급구역 : fwd ≤ EMERGENCY_FWD_RANGE AND horiz < threshold  (일반구역의 부분집합)

    [회피각 계산]
      avoid_angle = atan2(threshold, nearest_fwd)
      → 전방거리 기준이므로 가까울수록 회피각이 커짐

    [긴급 회피]
      긴급구역 진입 시:
        v 감속: EMERGENCY_MIN_SPEED ~ FORWARD_SPEED (전방거리에 비례)
        w 강화: × EMERGENCY_W_MULT
    """
    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN   # 160 mm

    # ── 1. 직사각형 위험구역으로 포인트 필터링 ──────────────────────────────
    danger_points = []
    for angle_norm, dist in scan_points:
        if dist <= 0 or dist > DETECTION_RANGE:
            continue
        horiz, fwd = decompose(angle_norm, dist)
        # 직사각형 조건: 전방거리 AND 수평거리 각각 독립 판단
        if fwd > 0 and fwd <= FORWARD_RANGE and horiz < threshold:
            danger_points.append((angle_norm, dist, horiz, fwd))

    if not danger_points:
        return FORWARD_SPEED, 0.0      # 장애물 없음 → 직진

    # ── 2. 가장 가까운 장애물 (직선거리 기준) ──────────────────────────────
    nearest                              = min(danger_points, key=lambda p: p[1])
    nearest_angle, ref_dist, n_horiz, n_fwd = nearest

    print(f"  [기준장애물] 각도:{nearest_angle:.1f}°  "
          f"직선:{ref_dist:.0f}mm  전방:{n_fwd:.0f}mm  수평:{n_horiz:.0f}mm")

    # ── 3. 긴급 회피구역 판정 ────────────────────────────────────────────────
    # 일반구역보다 좁은 직사각형: 전방 125mm × 수평 110mm
    in_emergency = (n_fwd <= EMERGENCY_FWD_RANGE and n_horiz <= EMERGENCY_HORIZ_RANGE)

    if in_emergency:
        # 전방거리 0 → EMERGENCY_MIN_SPEED, EMERGENCY_FWD_RANGE → FORWARD_SPEED
        ratio = n_fwd / EMERGENCY_FWD_RANGE         # 0.0 ~ 1.0
        v     = EMERGENCY_MIN_SPEED + (FORWARD_SPEED - EMERGENCY_MIN_SPEED) * ratio
        print(f"  [긴급회피] 전방:{n_fwd:.0f}mm  속도 감소 → v={v:.2f}m/s")
    else:
        v = FORWARD_SPEED

    # ── 4. 각도별 최소 거리 딕셔너리 (좌우 여유공간 판단용) ─────────────────
    scan_dict = {}
    for angle_norm, dist in scan_points:
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    # ── 5. 기준 직선거리 기준 좌/우 빈 공간 각도 폭 계산 ────────────────────
    left_clear = right_clear = 0
    for a in range(-SCAN_HALF_ANGLE, 0, ANGLE_STEP):
        if scan_dict.get(a, DETECTION_RANGE + 1) >= ref_dist:
            left_clear += ANGLE_STEP
    for a in range(ANGLE_STEP, SCAN_HALF_ANGLE + 1, ANGLE_STEP):
        if scan_dict.get(a, DETECTION_RANGE + 1) >= ref_dist:
            right_clear += ANGLE_STEP

    print(f"  [여유공간] 왼쪽:{left_clear}°  오른쪽:{right_clear}°")

    # ── 6. 회피각 계산 (atan2 방식) ─────────────────────────────────────────
    #
    #   avoid_angle = atan2(threshold, n_fwd)
    #
    #   n_fwd = 장애물까지 전방(수직)거리
    #   threshold = 필요한 수평 여유거리 (160mm)
    #
    #   의미: 로봇이 n_fwd 만큼 앞으로 가면서
    #         threshold 만큼 옆으로 벗어나려면 얼마나 꺾어야 하는가
    #
    #   n_fwd 작을수록(가까울수록) 각도가 커짐 → 더 급격하게 회피
    #   n_fwd 클수록(멀수록)  각도가 작아짐 → 완만하게 회피
    #
    avoid_angle = math.atan2(threshold, max(n_fwd, 1.0))   # 라디안

    # 긴급구역에서는 회피각 추가 강화
    if in_emergency:
        avoid_angle = min(avoid_angle * EMERGENCY_W_MULT, math.pi / 2)

    # ── 7. 회피 방향 결정 + 각속도 w 계산 ──────────────────────────────────
    if left_clear >= right_clear:
        w_sign  = 1.0    # 왼쪽 회피 (w > 0 → 반시계 → 좌회전)
        dir_str = "좌회전"
    else:
        w_sign  = -1.0   # 오른쪽 회피
        dir_str = "우회전"

    w = w_sign * min(W_GAIN * avoid_angle, MAX_W)

    print(f"  [회피명령] {dir_str}  "
          f"회피각:{math.degrees(avoid_angle):.1f}°  "
          f"v:{v:.2f}m/s  w:{w:.2f}rad/s"
          + ("  [긴급]" if in_emergency else ""))

    return v, w


def main():
    print("=== RPLIDAR 장애물 회피 v3 ===")
    print(f"  라이다 포트    : {LIDAR_PORT}")
    print(f"  아두이노 포트  : {ARDUINO_PORT}  (UART GPIO14/15)")
    print(f"  전진 속도      : {FORWARD_SPEED} m/s")
    print(f"  로봇 반폭      : {ROBOT_HALF_WIDTH} mm")
    print(f"  충돌 판단 기준 : {ROBOT_HALF_WIDTH + SAFETY_MARGIN} mm (수평)")
    print(f"  일반 위험구역  : 전방 {FORWARD_RANGE}mm × 수평 {ROBOT_HALF_WIDTH+SAFETY_MARGIN}mm (직사각형)")
    print(f"  긴급 회피구역  : 전방 {EMERGENCY_FWD_RANGE}mm × 수평 {EMERGENCY_HORIZ_RANGE}mm (직사각형)")
    print(f"  최소 속도      : {EMERGENCY_MIN_SPEED} m/s (긴급 시)")
    print("=" * 48)

    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)

    lidar.write(bytes([0xA5, 0x40]))    # RESET
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))    # START SCAN
    print("스캔 시작...")
    lidar.read(7)                       # 응답 디스크립터 버림

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
                    v, w  = find_vw_command(front_points)
                    cmd   = f"{v:.2f} {w:.2f}\n"
                    arduino.write(cmd.encode())

                    if cmd != last_cmd_str:
                        print(f"[전송] v={v:.2f}m/s  w={w:.2f}rad/s  "
                              f"(포인트:{len(front_points)})")
                        last_cmd_str = cmd

                    last_send = now

                scan_points = []

            scan_points.append((normalize_angle(angle_raw), distance))

    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        lidar.close()
        arduino.write(b"0.00 0.00\n")
        arduino.close()
        print("종료 완료.")


if __name__ == "__main__":
    main()

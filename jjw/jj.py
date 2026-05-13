"""
RPLIDAR C1 장애물 회피 - find_vw_command 핵심 로직만 유지

포트: 라이다 /dev/ttyUSB0 / 아두이노 /dev/ttyAMA3 (UART)
"""

import serial
import time
import math

# ── 포트 ─────────────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 9600

# ── 라이다 보정 ───────────────────────────────────────────────────────────────
LIDAR_OFFSET = 20      # mm

# ── 로봇 파라미터 ─────────────────────────────────────────────────────────────
ROBOT_HALF_WIDTH = 110   # mm: 라이다 중심 ~ 좌우 끝
SAFETY_MARGIN    = 10    # mm: 수평 안전 여유 → threshold = 120mm

# ── 위험구역 ──────────────────────────────────────────────────────────────────
DETECTION_RANGE  = 1500  # mm: LiDAR 최대 신뢰 거리
FORWARD_RANGE    = 800   # mm: 위험구역 전방 깊이

# ── 속도 파라미터 ─────────────────────────────────────────────────────────────
FORWARD_SPEED    = 0.35  # m/s: 최고 선속도
MIN_SPEED        = 0.07  # m/s: 최소 선속도
SLOW_START_DIST  = 250   # mm: 이 전방거리부터 감속 시작
STOP_FWD_RANGE   = 130   # mm: v=0 구역 전방 깊이
STOP_HORIZ_RANGE = 110   # mm: v=0 구역 수평 폭
W_GAIN           = 1.2
MAX_W            = 1.5
W_MIN_DANGER     = 0.5   # rad/s: 위험구역 최소 회전
W_SMOOTH         = 0.6
SIDE_ROTATE_SAFE = 130   # mm: 측면 장애물 수평거리 임계값
SIDE_CHECK_ANGLE = 90    # deg: 측면 확인 각도 범위

# ── 헤딩 방향 점수제 ──────────────────────────────────────────────────────────
HEADING_WEIGHT   = 1.0   # 헤딩 1° = 여유공간 1.0° 가중치
MIN_VIABLE_CLEAR = 25    # deg: 이 미만이면 해당 방향 진입 불가

# ── 스캔 파라미터 ─────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE  = 90
ANGLE_STEP       = 5
SEND_INTERVAL    = 0.1

# ── 전역 상태 ─────────────────────────────────────────────────────────────────
arduino_heading_deg = 0.0
avoidance_w_sign    = 0.0
no_danger_count     = 0
prev_w              = 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return angle - 360 if angle > 180 else angle


def is_in_front(angle_norm):
    return -SCAN_HALF_ANGLE <= angle_norm <= SCAN_HALF_ANGLE


def parse_packet(data):
    if len(data) != 5:
        return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        return None
    if (data[1] & 0x01) != 1:
        return None
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    angle       = angle_q6 / 64.0
    distance_q2 = data[3] | (data[4] << 8)
    distance    = distance_q2 / 4.0
    return angle, distance


def decompose(angle_norm_deg, distance_mm):
    rad   = math.radians(angle_norm_deg)
    horiz = abs(distance_mm * math.sin(rad))
    fwd   = distance_mm * math.cos(rad)
    return horiz, fwd


def read_arduino(arduino):
    global arduino_heading_deg
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'):
                arduino_heading_deg = float(line[2:])
        except Exception:
            pass


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 회피 방향 결정 (점수제)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def select_direction(left_clear, right_clear, heading_deg):
    """
    여유공간 + 헤딩 보정 점수로 회피 방향 결정

    [갇힘 방지 — MIN_VIABLE_CLEAR]
      한쪽 여유공간이 MIN_VIABLE_CLEAR(25°) 미만이면 진입 불가로 판단
      헤딩 보너스 무시하고 열린 쪽 강제 선택

    [점수제 — 양쪽 모두 통과 가능할 때]
      left_score  = left_clear  + max(0, -heading_deg) × HEADING_WEIGHT
      right_score = right_clear + max(0,  heading_deg) × HEADING_WEIGHT
    """
    left_ok  = left_clear  >= MIN_VIABLE_CLEAR
    right_ok = right_clear >= MIN_VIABLE_CLEAR

    if left_ok and not right_ok:
        print(f"  [방향] 오른쪽 막힘({right_clear}°) → 왼쪽 강제")
        return 1.0
    if right_ok and not left_ok:
        print(f"  [방향] 왼쪽 막힘({left_clear}°) → 오른쪽 강제")
        return -1.0
    if not left_ok and not right_ok:
        print(f"  [방향] 양쪽 협소 → {'왼쪽' if left_clear >= right_clear else '오른쪽'} 선택")
        return 1.0 if left_clear >= right_clear else -1.0

    left_score  = left_clear  + max(0.0, -heading_deg) * HEADING_WEIGHT
    right_score = right_clear + max(0.0,  heading_deg) * HEADING_WEIGHT

    bonus_side = "R" if heading_deg > 0 else "L"
    bonus_val  = abs(heading_deg) * HEADING_WEIGHT
    print(f"  [방향점수] L={left_score:.0f}  R={right_score:.0f}"
          f"  (여유 L={left_clear}° R={right_clear}°"
          f"  헤딩보너스 {bonus_side}+{bonus_val:.0f})")

    return 1.0 if left_score >= right_score else -1.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# v/w 명령 계산 (핵심)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_command(scan_points, heading_deg):
    """
    정면 스캔 + 헤딩 → (v m/s, w rad/s) 반환

    [Stop zone] 장애물 각도로 방향 직접 결정 (hysteresis 무시, 충돌 후 오판 방지)
    [Danger zone] 여유공간 점수제 + avoidance_w_sign hysteresis
    [공통] 측면 안전 검사(수평거리) + W_MIN_DANGER 최소 회전 보장
    """
    global avoidance_w_sign, no_danger_count
    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN

    # 위험 포인트 수집
    danger_points = []
    for angle_norm, dist in scan_points:
        if dist <= 0 or dist > DETECTION_RANGE:
            continue
        horiz, fwd = decompose(angle_norm, dist)
        if fwd > 0 and fwd <= FORWARD_RANGE and horiz < threshold:
            danger_points.append((angle_norm, dist, horiz, fwd))

    NO_DANGER_RESET = 3
    if not danger_points:
        no_danger_count += 1
        if no_danger_count >= NO_DANGER_RESET:
            avoidance_w_sign = 0.0
        return FORWARD_SPEED, 0.0
    no_danger_count = 0

    # 기준값 계산
    stop_points = [p for p in danger_points
                   if p[3] <= STOP_FWD_RANGE and p[2] <= STOP_HORIZ_RANGE]
    frontal   = [p for p in danger_points if p[3] >= p[2]]
    n_fwd_ref = min((p[3] for p in frontal), default=SLOW_START_DIST + 1)
    horiz_ref = min(danger_points, key=lambda p: p[2])
    nearest_angle, ref_dist, n_horiz, _ = horiz_ref

    print(f"  [기준] 전방:{n_fwd_ref:.0f}mm  정지:{len(stop_points)}개  "
          f"각도:{nearest_angle:.1f}°  수평:{n_horiz:.0f}mm")

    # 선속도
    if stop_points:
        v = 0.0
    elif n_fwd_ref >= SLOW_START_DIST:
        v = FORWARD_SPEED
    else:
        ratio = (n_fwd_ref - STOP_FWD_RANGE) / (SLOW_START_DIST - STOP_FWD_RANGE)
        v = max(FORWARD_SPEED * ratio, MIN_SPEED)

    horiz_error = threshold - n_horiz
    if horiz_error <= 0:
        avoidance_w_sign = 0.0
        return v, 0.0

    # scan_dict 구성
    scan_dict = {}
    for angle_norm, dist in scan_points:
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    # 방향 결정
    if stop_points:
        # Stop zone: 장애물 각도로 직접 결정
        stop_angle = min(stop_points, key=lambda p: p[2])[0]
        avoidance_w_sign = 1.0 if stop_angle >= 0 else -1.0
        print(f"  [정지구역] 각도:{stop_angle:.1f}° → "
              f"{'왼쪽' if avoidance_w_sign>0 else '오른쪽'} 직접 결정")
    else:
        # Danger zone: 여유공간 점수제 + hysteresis
        left_clear = right_clear = 0
        for a in range(-SCAN_HALF_ANGLE, 0, ANGLE_STEP):
            if scan_dict.get(a, DETECTION_RANGE + 1) >= ref_dist:
                left_clear += ANGLE_STEP
        for a in range(ANGLE_STEP, SCAN_HALF_ANGLE + 1, ANGLE_STEP):
            if scan_dict.get(a, DETECTION_RANGE + 1) >= ref_dist:
                right_clear += ANGLE_STEP

        print(f"  [여유] 왼:{left_clear}°  오:{right_clear}°  헤딩:{heading_deg:.1f}°")

        if avoidance_w_sign == 0.0:
            avoidance_w_sign = select_direction(left_clear, right_clear, heading_deg)
            print(f"  [방향결정] {'왼쪽' if avoidance_w_sign>0 else '오른쪽'} 고착")
        else:
            committed_clear = left_clear if avoidance_w_sign > 0 else right_clear
            if committed_clear < MIN_VIABLE_CLEAR:
                old = avoidance_w_sign
                avoidance_w_sign = select_direction(left_clear, right_clear, heading_deg)
                if avoidance_w_sign != old:
                    print(f"  [방향전환] 막힘({committed_clear}°) → "
                          f"{'왼쪽' if avoidance_w_sign>0 else '오른쪽'}")

    # 측면 안전 검사 (수평거리 기준)
    def side_horiz_blocked(is_left):
        angles = (range(-ANGLE_STEP, -(SIDE_CHECK_ANGLE + ANGLE_STEP), -ANGLE_STEP)
                  if is_left else
                  range(ANGLE_STEP, SIDE_CHECK_ANGLE + ANGLE_STEP, ANGLE_STEP))
        for a in angles:
            d = scan_dict.get(a, 0)
            if d <= 0:
                continue
            if d * abs(math.sin(math.radians(a))) < SIDE_ROTATE_SAFE:
                return True
        return False

    left_close  = side_horiz_blocked(is_left=True)
    right_close = side_horiz_blocked(is_left=False)
    if avoidance_w_sign > 0 and left_close and not right_close:
        print("  [측면차단] 왼쪽 → 오른쪽 강제")
        avoidance_w_sign = -1.0
    elif avoidance_w_sign < 0 and right_close and not left_close:
        print("  [측면차단] 오른쪽 → 왼쪽 강제")
        avoidance_w_sign = 1.0

    # 각속도: W_MIN_DANGER로 최소 회전 보장
    w_mag = max(min(W_GAIN * horiz_error / threshold, MAX_W), W_MIN_DANGER)
    w     = avoidance_w_sign * w_mag

    print(f"  [명령] v:{v:.2f}  w:{w:.2f}  (수평오차:{horiz_error:.0f}mm)")
    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    global prev_w

    print("=== RPLIDAR 장애물 회피 (find_vw_command 단독) ===")
    print(f"  라이다 포트    : {LIDAR_PORT}")
    print(f"  아두이노 포트  : {ARDUINO_PORT}")
    print(f"  위험구역       : 전방 {FORWARD_RANGE}mm × 수평 {ROBOT_HALF_WIDTH + SAFETY_MARGIN}mm")
    print(f"  속도           : 최고 {FORWARD_SPEED}m/s  최저 {MIN_SPEED}m/s")
    print("=" * 50)

    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)

    lidar.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))
    print("스캔 시작...")
    lidar.read(7)

    scan_points  = []
    last_send    = time.time()
    last_cmd_str = ""

    try:
        while True:
            read_arduino(arduino)

            raw    = lidar.read(5)
            result = parse_packet(raw)
            if result is None:
                continue

            angle_raw, distance = result
            s_flag = raw[0] & 0x01

            if s_flag == 1 and scan_points:
                front_points = [
                    (a, d) for a, d in scan_points
                    if is_in_front(a) and d > 0
                ]

                now = time.time()
                if now - last_send >= SEND_INTERVAL:
                    v, w = find_vw_command(front_points, arduino_heading_deg)

                    # w 저역통과 필터
                    w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
                    prev_w = w

                    cmd = f"{v:.2f} {w:.2f}\n"
                    arduino.write(cmd.encode())

                    if cmd != last_cmd_str:
                        print(f"[전송] v={v:.2f}  w={w:.2f}  "
                              f"헤딩={arduino_heading_deg:.1f}°")
                        last_cmd_str = cmd

                    last_send = now

                scan_points = []

            scan_points.append((
                normalize_angle(angle_raw),
                distance + LIDAR_OFFSET if distance > 0 else 0
            ))

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

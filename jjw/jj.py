"""
RPLIDAR C1 장애물 회피 - 기구학적 파라미터 적용 (안전 수치 반영)

포트: 라이다 /dev/ttyUSB0 / 아두이노 /dev/ttyAMA3 (UART)

[변경 이력]
  - 로봇 치수: 좌우폭 110mm, 라이다-전면 125mm, 차축 후퇴 60mm (차축-전면 185mm)
  - 스윙아웃 회전 반경(약 215.2mm)을 고려하여 파라미터 안전 수치로 재계산
  - STOP_FWD_RANGE 상향 (125mm -> 180mm): 앞범퍼 스윙아웃 충돌 방지
  - SLOW_START_DIST 상향 (250mm -> 400mm): 늘어난 정지거리에 맞춘 감속 시작점
  - SIDE_FWD_DEADZONE 상향 (140mm -> 180mm): 모서리 사각지대 방지
  - SIDE_HORIZ_LIMIT 상향 (130mm -> 140mm): 수평 안전 여유 추가
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
LIDAR_OFFSET    = 20    # mm: 라이다 측정값 보정
LIDAR_MIN_VALID = 100   # mm: 이 미만은 라이다 오류로 간주 → 무시

# ── 로봇 파라미터 ─────────────────────────────────────────────────────────────
ROBOT_HALF_WIDTH = 110   # mm: 라이다 중심 ~ 좌우 끝
SAFETY_MARGIN    = 30    # mm: 수평 안전 여유 (10 -> 30으로 상향) → threshold = 140mm

# ── 위험구역 ──────────────────────────────────────────────────────────────────
DETECTION_RANGE  = 1500  # mm: LiDAR 최대 신뢰 거리
FORWARD_RANGE    = 800   # mm: 위험구역 전방 깊이

# ── 속도 파라미터 ─────────────────────────────────────────────────────────────
FORWARD_SPEED    = 0.35  # m/s: 최고 선속도
MIN_SPEED        = 0.07  # m/s: 최소 선속도
SLOW_START_DIST  = 400   # mm: 이 전방거리부터 감속 시작 (기존 250 -> 400)
STOP_FWD_RANGE   = 180   # mm: v=0 구역 전방 깊이 (기존 125 -> 180, 스윙아웃 방지)
W_GAIN           = 1.2
MAX_W            = 1.5
W_MIN_DANGER     = 0.5   # rad/s: 위험구역 최소 회전
W_SMOOTH         = 0.6

# ── 측면 감지 (수직/수평 거리 기반) ──────────────────────────────────────────
SIDE_HORIZ_LIMIT  = 140  # mm: 측면 수평거리 임계값 (기존 130 -> 140)
SIDE_FWD_DEADZONE = 180  # mm: 이 수직거리 이내는 정면으로 간주 (기존 140 -> 180)

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
# 측면 감지 (수직/수평 거리 기반)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def side_horiz_blocked(scan_points, is_left):
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID:
            continue
        if is_left     and angle_norm >= 0:
            continue
        if not is_left and angle_norm <= 0:
            continue
        rad   = math.radians(angle_norm)
        horiz = dist * abs(math.sin(rad))
        fwd   = dist * math.cos(rad)
        if fwd < SIDE_FWD_DEADZONE:
            continue
        if horiz < SIDE_HORIZ_LIMIT:
            return True
    return False


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 회피 방향 결정 (점수제 + 측면 감지 통합)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def select_direction(left_clear, right_clear, heading_deg,
                     left_side_blocked, right_side_blocked):
    left_ok  = (left_clear  >= MIN_VIABLE_CLEAR) and not left_side_blocked
    right_ok = (right_clear >= MIN_VIABLE_CLEAR) and not right_side_blocked

    side_log = []
    if left_side_blocked:
        side_log.append("왼쪽측면차단")
    if right_side_blocked:
        side_log.append("오른쪽측면차단")
    if side_log:
        print(f"  [측면감지] {' / '.join(side_log)}")

    if left_ok and not right_ok:
        reason = f"오른쪽 불가(여유:{right_clear}°" + \
                 (" 측면차단" if right_side_blocked else "") + ")"
        print(f"  [방향] {reason} → 왼쪽 강제")
        return 1.0
    if right_ok and not left_ok:
        reason = f"왼쪽 불가(여유:{left_clear}°" + \
                 (" 측면차단" if left_side_blocked else "") + ")"
        print(f"  [방향] {reason} → 오른쪽 강제")
        return -1.0
    if not left_ok and not right_ok:
        print(f"  [방향] 양쪽 불가 → {'왼쪽' if left_clear >= right_clear else '오른쪽'} 선택 (탈출)")
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
    global avoidance_w_sign, no_danger_count
    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN

    danger_points = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE:
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

    stop_points = [p for p in danger_points if p[3] <= STOP_FWD_RANGE and p[2] < ROBOT_HALF_WIDTH]
    frontal     = [p for p in danger_points if p[3] >= p[2]]
    n_fwd_ref   = min((p[3] for p in frontal), default=SLOW_START_DIST + 1)
    horiz_ref   = min(danger_points, key=lambda p: p[2])
    nearest_angle, ref_dist, n_horiz, _ = horiz_ref

    print(f"  [기준] 전방:{n_fwd_ref:.0f}mm  정지:{len(stop_points)}개  "
          f"각도:{nearest_angle:.1f}°  수평:{n_horiz:.0f}mm")

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

    scan_dict = {}
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    left_side_blocked  = side_horiz_blocked(scan_points, is_left=True)
    right_side_blocked = side_horiz_blocked(scan_points, is_left=False)

    if stop_points:
        stop_angle = min(stop_points, key=lambda p: p[2])[0]
        raw_sign   = 1.0 if stop_angle >= 0 else -1.0

        is_raw_blocked = (raw_sign > 0 and left_side_blocked) or \
                         (raw_sign < 0 and right_side_blocked)
        if is_raw_blocked:
            alt_sign       = -raw_sign
            is_alt_blocked = (alt_sign > 0 and left_side_blocked) or \
                             (alt_sign < 0 and right_side_blocked)
            if not is_alt_blocked:
                print(f"  [정지구역] 각도:{stop_angle:.1f}° → "
                      f"측면차단으로 {'왼쪽' if alt_sign > 0 else '오른쪽'} 전환")
                avoidance_w_sign = alt_sign
            else:
                print(f"  [정지구역] 각도:{stop_angle:.1f}° → "
                      f"양쪽 측면차단, 각도 우선({'왼쪽' if raw_sign > 0 else '오른쪽'})")
                avoidance_w_sign = raw_sign
        else:
            print(f"  [정지구역] 각도:{stop_angle:.1f}° → "
                  f"{'왼쪽' if raw_sign > 0 else '오른쪽'} 직접 결정")
            avoidance_w_sign = raw_sign

    else:
        left_clear = right_clear = 0
        for a in range(-SCAN_HALF_ANGLE, 0, ANGLE_STEP):
            if scan_dict.get(a, DETECTION_RANGE + 1) >= ref_dist:
                left_clear += ANGLE_STEP
        for a in range(ANGLE_STEP, SCAN_HALF_ANGLE + 1, ANGLE_STEP):
            if scan_dict.get(a, DETECTION_RANGE + 1) >= ref_dist:
                right_clear += ANGLE_STEP

        print(f"  [여유] 왼:{left_clear}°  오:{right_clear}°  헤딩:{heading_deg:.1f}°")

        if avoidance_w_sign == 0.0:
            avoidance_w_sign = select_direction(
                left_clear, right_clear, heading_deg,
                left_side_blocked, right_side_blocked
            )
            print(f"  [방향결정] {'왼쪽' if avoidance_w_sign > 0 else '오른쪽'} 고착")
        else:
            committed_clear = left_clear if avoidance_w_sign > 0 else right_clear
            committed_blocked = (avoidance_w_sign > 0 and left_side_blocked) or \
                                (avoidance_w_sign < 0 and right_side_blocked)
            if committed_clear < MIN_VIABLE_CLEAR or committed_blocked:
                old = avoidance_w_sign
                avoidance_w_sign = select_direction(
                    left_clear, right_clear, heading_deg,
                    left_side_blocked, right_side_blocked
                )
                if avoidance_w_sign != old:
                    reason = "여유부족" if committed_clear < MIN_VIABLE_CLEAR else "측면차단"
                    print(f"  [방향전환] {reason} → "
                          f"{'왼쪽' if avoidance_w_sign > 0 else '오른쪽'}")

    w_mag = max(min(W_GAIN * horiz_error / threshold, MAX_W), W_MIN_DANGER)
    w     = avoidance_w_sign * w_mag

    print(f"  [명령] v:{v:.2f}  w:{w:.2f}  (수평오차:{horiz_error:.0f}mm)")
    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    global prev_w

    print("=== RPLIDAR 장애물 회피 (기구학 적용) ===")
    print(f"  라이다 포트    : {LIDAR_PORT}")
    print(f"  아두이노 포트  : {ARDUINO_PORT}")
    print(f"  위험구역       : 전방 {FORWARD_RANGE}mm × 수평 {ROBOT_HALF_WIDTH + SAFETY_MARGIN}mm")
    print(f"  속도           : 최고 {FORWARD_SPEED}m/s  최저 {MIN_SPEED}m/s")
    print(f"  측면감지       : 수평 {SIDE_HORIZ_LIMIT}mm 이내 / 수직 {SIDE_FWD_DEADZONE}mm 이상")
    print(f"  오류제거       : {LIDAR_MIN_VALID}mm 미만 무시")
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

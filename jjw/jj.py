"""
RPLIDAR C1 장애물 회피 - 좁은 통로 특화 (진동 방지 & 코사인 법칙)

포트: 라이다 /dev/ttyUSB0 / 아두이노 /dev/ttyAMA3 (UART)

[개선 사항]
  - get_gap_width: depth jump를 절댓값으로 처리 (음수 점프 = 더 가까운 벽 감지)
  - get_gap_width: fallback 시 DETECTION_RANGE 대신 edge_p 거리로 보수적 추정
  - get_gap_width: ref_angle 자체를 탐색에서 제외 (장애물 자체부터 시작하는 오류 방지)
  - stop_zone_w_sign 분리: stop zone / danger zone 방향 메모리 독립
  - stop zone에서 측면 감지 비활성화
  - [방법 A] proximity_points 도입: 기준점(ref) 안정화로 진동 방지
    → danger zone 밖이어도 PROXIMITY_HORIZ 이내 장애물을 ref 후보로 포함
    → get_gap_width 기준점이 임계값 경계에서 급변하는 문제 해결
    → horiz_error(w 크기)는 여전히 danger zone 기준 유지
"""

import serial
import time
import math

# ── 포트 ─────────────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

# ── 라이다 보정 ───────────────────────────────────────────────────────────────
LIDAR_OFFSET    = 20    # mm: 라이다 측정값 보정
LIDAR_MIN_VALID = 100   # mm: 이 미만은 라이다 오류로 간주 → 무시

# ── 로봇 파라미터 & 기구학 ────────────────────────────────────────────────────
ROBOT_HALF_WIDTH  = 110  # mm: 라이다 중심 ~ 좌우 끝
SAFETY_MARGIN     = 30   # mm: 수평 안전 여유 → threshold = 140mm
MIN_PASSAGE_WIDTH = 240  # mm: 이 물리적 너비 이상이어야 통과 가능하다고 판단

# ── 위험구역 ──────────────────────────────────────────────────────────────────
DETECTION_RANGE  = 1500  # mm: LiDAR 최대 신뢰 거리
FORWARD_RANGE    = 800   # mm: 위험구역 전방 깊이

# ── 속도 파라미터 ─────────────────────────────────────────────────────────────
FORWARD_SPEED    = 0.35  # m/s: 최고 선속도
MIN_SPEED        = 0.07  # m/s: 최소 선속도
SLOW_START_DIST  = 400   # mm: 이 전방거리부터 감속 시작
STOP_FWD_RANGE   = 180   # mm: 앞범퍼 스윙아웃 방지를 위한 정지 거리
W_GAIN           = 1.2
MAX_W            = 1.5
W_MIN_DANGER     = 0.5   # rad/s: 위험구역 최소 회전
W_SMOOTH         = 0.6

# ── 측면 감지 (수직/수평 거리 기반) ──────────────────────────────────────────
SIDE_HORIZ_LIMIT  = 140  # mm: 측면 수평거리 임계값
SIDE_FWD_DEADZONE = 130  # mm: 이 수직거리 이내는 정면으로 간주 → 측면 판정 제외

# ── 근접 후보 (방법 A) ────────────────────────────────────────────────────────
PROXIMITY_HORIZ = 170    # mm: danger zone(140mm) 밖이어도 이 이내면 ref 후보 포함
                          #     threshold(140mm)보다 크고 실제 충돌 위험 범위 이내로 설정
                          #     너무 크면 무관한 장애물이 ref가 되어 역효과

# ── 헤딩 방향 점수제 ──────────────────────────────────────────────────────────
HEADING_WEIGHT_MM = 5.0  # 헤딩 1°당 여유공간 5mm의 가중치 보너스

# ── 스캔 파라미터 ─────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE  = 90
ANGLE_STEP       = 5
SEND_INTERVAL    = 0.1
DEPTH_JUMP_THRES = 120   # mm: 이 이상 거리 변화 시 다른 물체로 간주 (양방향)

# ── Stop zone 정면 노이즈 억제 ────────────────────────────────────────────────
STOP_FRONT_DEADBAND = 15  # deg: 이 각도 이내 정면 장애물은 기존 방향 유지

# ── 전역 상태 ─────────────────────────────────────────────────────────────────
arduino_heading_deg = 0.0
avoidance_w_sign    = 0.0   # danger zone 방향 메모리
stop_zone_w_sign    = 0.0   # stop zone 전용 방향 메모리 (danger zone과 독립)
no_danger_count     = 0
prev_w              = 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티 & 수학 연산
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return angle - 360 if angle > 180 else angle

def is_in_front(angle_norm):
    return -SCAN_HALF_ANGLE <= angle_norm <= SCAN_HALF_ANGLE

def decompose(angle_norm_deg, distance_mm):
    rad   = math.radians(angle_norm_deg)
    horiz = abs(distance_mm * math.sin(rad))
    fwd   = distance_mm * math.cos(rad)
    return horiz, fwd

def calc_law_of_cosines(d1, d2, angle_diff_deg):
    theta = math.radians(abs(angle_diff_deg))
    return math.sqrt(d1**2 + d2**2 - 2 * d1 * d2 * math.cos(theta))

def parse_packet(data):
    if len(data) != 5: return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    distance_q2 = data[3] | (data[4] << 8)
    return (angle_q6 / 64.0), (distance_q2 / 4.0)

def read_arduino(arduino):
    global arduino_heading_deg
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'): arduino_heading_deg = float(line[2:])
        except Exception: pass


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 측면 감지
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def side_horiz_blocked(scan_points, is_left):
    """
    danger zone 전용: stop zone에서는 호출하지 않음

    조건 (3단계):
      1. dist >= LIDAR_MIN_VALID       → 오류값 제거
      2. 수직거리 >= SIDE_FWD_DEADZONE → 정면 장애물 오판 방지
      3. 수평거리 <  SIDE_HORIZ_LIMIT  → 측면 근접 감지
    """
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID: continue
        if is_left     and angle_norm >= 0: continue
        if not is_left and angle_norm <= 0: continue
        horiz, fwd = decompose(angle_norm, dist)
        if fwd < SIDE_FWD_DEADZONE: continue
        if horiz < SIDE_HORIZ_LIMIT: return True
    return False


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 코사인 법칙 기반 빈 공간 너비 계산
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def get_gap_width(scan_points, ref_angle, ref_dist, is_left):
    """
    가장 가까운 장애물(ref) 옆으로 얼마나 넓은 공간이 있는지 계산

    ref_angle/ref_dist: proximity_points 포함 후보 중 선택된 기준점
    → danger zone 경계에서 ref가 급변하지 않아 방향 결정이 안정적
    """
    if is_left:
        search_points = sorted(
            [p for p in scan_points if p[0] < ref_angle],
            key=lambda x: x[0], reverse=True
        )
    else:
        search_points = sorted(
            [p for p in scan_points if p[0] > ref_angle],
            key=lambda x: x[0]
        )

    if not search_points:
        return 0.0

    edge_p = (ref_angle, ref_dist)

    for i, p in enumerate(search_points):
        if abs(p[1] - edge_p[1]) > DEPTH_JUMP_THRES:
            wall_points = search_points[i:]
            if wall_points:
                min_width = min(
                    calc_law_of_cosines(edge_p[1], wp[1], abs(edge_p[0] - wp[0]))
                    for wp in wall_points
                )
                print(f"  [gap {'L' if is_left else 'R'}] "
                      f"edge={edge_p[0]:.0f}°/{edge_p[1]:.0f}mm "
                      f"jump={p[1]-edge_p[1]:+.0f}mm → 너비={min_width:.0f}mm")
                return min_width
        edge_p = p

    rem_angle = abs((-SCAN_HALF_ANGLE - edge_p[0]) if is_left
                    else (SCAN_HALF_ANGLE - edge_p[0]))

    if rem_angle > 15:
        width = calc_law_of_cosines(edge_p[1], edge_p[1], rem_angle)
        print(f"  [gap {'L' if is_left else 'R'}] "
              f"연속벽 끝까지 → 보수적 너비={width:.0f}mm "
              f"(edge {edge_p[1]:.0f}mm × {rem_angle:.0f}°)")
        return width

    print(f"  [gap {'L' if is_left else 'R'}] 스캔 각도 부족 → 0mm")
    return 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 회피 방향 결정 (danger zone 전용)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def select_direction_by_width(left_width, right_width, heading_deg,
                              left_side_blocked, right_side_blocked):
    """
    실제 통과 너비(mm) + 측면 감지 + 헤딩 보너스로 회피 방향 1회 결정

    [우선순위]
      1. 측면 장애물 차단 → 해당 방향 불가
      2. MIN_PASSAGE_WIDTH 미만 → 진입 불가
      3. 양쪽 모두 가능 → 점수제 (너비 + 헤딩 보너스)
      4. 양쪽 모두 불가 → 너비가 더 넓은 쪽으로 탈출 시도
    """
    left_ok  = (left_width  >= MIN_PASSAGE_WIDTH) and not left_side_blocked
    right_ok = (right_width >= MIN_PASSAGE_WIDTH) and not right_side_blocked

    side_log = []
    if left_side_blocked:  side_log.append("왼쪽측면차단")
    if right_side_blocked: side_log.append("오른쪽측면차단")
    if side_log:
        print(f"  [측면감지] {' / '.join(side_log)}")

    if left_ok and not right_ok:
        reason = f"오른쪽 불가(너비:{right_width:.0f}mm" + \
                 (" 측면차단" if right_side_blocked else "") + ")"
        print(f"  [방향] {reason} → 왼쪽 강제")
        return 1.0

    if right_ok and not left_ok:
        reason = f"왼쪽 불가(너비:{left_width:.0f}mm" + \
                 (" 측면차단" if left_side_blocked else "") + ")"
        print(f"  [방향] {reason} → 오른쪽 강제")
        return -1.0

    if not left_ok and not right_ok:
        chosen = "왼쪽" if left_width >= right_width else "오른쪽"
        print(f"  [방향] 양쪽 좁음(L:{left_width:.0f} R:{right_width:.0f}mm)"
              f" → {chosen} 뚝심 돌파")
        return 1.0 if left_width >= right_width else -1.0

    left_score  = left_width  + max(0.0, -heading_deg) * HEADING_WEIGHT_MM
    right_score = right_width + max(0.0,  heading_deg) * HEADING_WEIGHT_MM

    bonus_side = "R" if heading_deg > 0 else "L"
    bonus_val  = abs(heading_deg) * HEADING_WEIGHT_MM
    print(f"  [방향점수] L={left_score:.0f}  R={right_score:.0f}"
          f"  (너비 L={left_width:.0f}mm R={right_width:.0f}mm"
          f"  헤딩보너스 {bonus_side}+{bonus_val:.0f})")

    return 1.0 if left_score >= right_score else -1.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# v/w 명령 계산
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_command(scan_points, heading_deg):
    """
    정면 스캔 + 헤딩 → (v m/s, w rad/s) 반환

    [방법 A: proximity_points 도입]

      기존 문제:
        danger zone(horiz < 140mm) 밖에 있던 장애물이 회전 중 경계를 넘는 순간
        horiz_ref(기준점)가 갑자기 바뀌어 get_gap_width 결과가 급변
        → 방향 전환 → 반대쪽 장애물이 danger 진입 → 반복 → 진동

      해결:
        proximity_points: danger zone 밖(threshold~PROXIMITY_HORIZ)이어도
                          PROXIMITY_HORIZ 이내 장애물을 ref 후보로 포함
        ref_angle/ref_dist: danger + proximity 합산 후 horiz 최소값으로 선택
        → danger zone 경계 전후로 ref가 서서히 바뀌어 방향 결정 안정화

      분리:
        ref_angle/ref_dist → get_gap_width 기준점 (proximity 포함)
        n_horiz / horiz_error → w 크기 계산 (danger zone만, 엄격 유지)
    """
    global avoidance_w_sign, stop_zone_w_sign, no_danger_count

    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN

    # ── 1. 위험 포인트 수집 ───────────────────────────────────────────────────
    danger_points = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        horiz, fwd = decompose(angle_norm, dist)
        if fwd > 0 and fwd <= FORWARD_RANGE and horiz < threshold:
            danger_points.append((angle_norm, dist, horiz, fwd))

    NO_DANGER_RESET = 3
    if not danger_points:
        no_danger_count += 1
        if no_danger_count >= NO_DANGER_RESET:
            avoidance_w_sign = 0.0
            stop_zone_w_sign = 0.0
        return FORWARD_SPEED, 0.0
    no_danger_count = 0

    # ── 2. 근접 후보 수집 (방법 A) ───────────────────────────────────────────
    # danger zone 밖이지만 PROXIMITY_HORIZ 이내: ref 안정화용
    # horiz_error 계산에는 사용하지 않음
    proximity_points = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        horiz, fwd = decompose(angle_norm, dist)
        if fwd > 0 and fwd <= FORWARD_RANGE and threshold <= horiz < PROXIMITY_HORIZ:
            proximity_points.append((angle_norm, dist, horiz, fwd))

    # ── 3. 선속도 결정 ────────────────────────────────────────────────────────
    stop_points = [p for p in danger_points
                   if p[3] <= STOP_FWD_RANGE and p[2] < ROBOT_HALF_WIDTH]
    frontal     = [p for p in danger_points if p[3] >= p[2]]
    n_fwd_ref   = min((p[3] for p in frontal), default=SLOW_START_DIST + 1)

    # horiz_error: danger zone 기준 (w 크기 계산에 사용)
    horiz_ref_danger            = min(danger_points, key=lambda p: p[2])
    _, _, n_horiz, _            = horiz_ref_danger
    horiz_error                 = threshold - n_horiz

    # ref: danger + proximity 합산 후 horiz 최소 (get_gap_width 기준점)
    # proximity 장애물이 더 가까우면 그쪽이 ref가 되어 gap 계산 안정화
    ref_candidates              = danger_points + proximity_points
    horiz_ref_stable            = min(ref_candidates, key=lambda p: p[2])
    ref_angle, ref_dist, _, _   = horiz_ref_stable

    is_proximity_ref = horiz_ref_stable not in danger_points
    print(f"  [기준] 전방:{n_fwd_ref:.0f}mm  정지:{len(stop_points)}개  "
          f"ref각도:{ref_angle:.1f}°  수평(danger):{n_horiz:.0f}mm"
          + (" [근접후보]" if is_proximity_ref else ""))

    if stop_points:
        v = 0.0
    elif n_fwd_ref >= SLOW_START_DIST:
        v = FORWARD_SPEED
    else:
        ratio = (n_fwd_ref - STOP_FWD_RANGE) / (SLOW_START_DIST - STOP_FWD_RANGE)
        v = max(FORWARD_SPEED * ratio, MIN_SPEED)

    if horiz_error <= 0:
        avoidance_w_sign = 0.0
        stop_zone_w_sign = 0.0
        return v, 0.0

    # ── 4. 회전 방향 결정 ────────────────────────────────────────────────────
    if stop_points:
        # ── Stop zone ──────────────────────────────────────────────────────
        # 측면 감지 비활성 + stop_zone_w_sign 노이즈 억제
        stop_angle = min(stop_points, key=lambda p: p[2])[0]

        if abs(stop_angle) < STOP_FRONT_DEADBAND and stop_zone_w_sign != 0.0:
            avoidance_w_sign = stop_zone_w_sign
            print(f"  [정지구역] 정면 노이즈(각도:{stop_angle:.1f}°) → "
                  f"기존 방향 유지({'왼쪽' if avoidance_w_sign > 0 else '오른쪽'})")
        else:
            avoidance_w_sign = 1.0 if stop_angle >= 0 else -1.0
            stop_zone_w_sign = avoidance_w_sign
            print(f"  [정지구역] 각도:{stop_angle:.1f}° → "
                  f"{'왼쪽' if avoidance_w_sign > 0 else '오른쪽'} 즉결")

    else:
        # ── Danger zone ────────────────────────────────────────────────────
        stop_zone_w_sign = 0.0

        # 측면 감지 활성 (danger zone에서만 사용)
        left_side_blocked  = side_horiz_blocked(scan_points, is_left=True)
        right_side_blocked = side_horiz_blocked(scan_points, is_left=False)

        # get_gap_width: 안정화된 ref_angle/ref_dist 사용
        left_width  = get_gap_width(scan_points, ref_angle, ref_dist, is_left=True)
        right_width = get_gap_width(scan_points, ref_angle, ref_dist, is_left=False)

        if avoidance_w_sign == 0.0:
            avoidance_w_sign = select_direction_by_width(
                left_width, right_width, heading_deg,
                left_side_blocked, right_side_blocked
            )
            print(f"  [방향결정] {'왼쪽' if avoidance_w_sign > 0 else '오른쪽'} 고착")
        else:
            committed_width   = left_width  if avoidance_w_sign > 0 else right_width
            opposite_width    = right_width if avoidance_w_sign > 0 else left_width
            committed_blocked = (avoidance_w_sign > 0 and left_side_blocked) or \
                                (avoidance_w_sign < 0 and right_side_blocked)

            need_reselect = committed_blocked or (
                committed_width < MIN_PASSAGE_WIDTH
                and opposite_width >= MIN_PASSAGE_WIDTH
            )
            if need_reselect:
                old = avoidance_w_sign
                avoidance_w_sign = select_direction_by_width(
                    left_width, right_width, heading_deg,
                    left_side_blocked, right_side_blocked
                )
                if avoidance_w_sign != old:
                    reason = "측면물리차단" if committed_blocked \
                             else "반대쪽 통과 가능"
                    print(f"  [방향전환] {reason} → "
                          f"{'왼쪽' if avoidance_w_sign > 0 else '오른쪽'}")

    # ── 5. 각속도 계산 ────────────────────────────────────────────────────────
    # horiz_error는 danger zone 기준으로 계산 (proximity 장애물 영향 없음)
    w_mag = max(min(W_GAIN * horiz_error / threshold, MAX_W), W_MIN_DANGER)
    w     = avoidance_w_sign * w_mag

    print(f"  [명령] v:{v:.2f}  w:{w:.2f}  (수평오차:{horiz_error:.0f}mm)")
    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    global prev_w
    print("=== RPLIDAR 장애물 회피 (진동 방지 & 좁은 통로 돌파 특화) ===")
    print(f"  측면감지    : 수평 {SIDE_HORIZ_LIMIT}mm 이내 / 수직 {SIDE_FWD_DEADZONE}mm 이상")
    print(f"  통과너비    : {MIN_PASSAGE_WIDTH}mm 이상")
    print(f"  근접후보    : 수평 {PROXIMITY_HORIZ}mm 이내 (ref 안정화)")
    print(f"  오류제거    : {LIDAR_MIN_VALID}mm 미만 무시")
    print(f"  depth jump  : ±{DEPTH_JUMP_THRES}mm (양방향)")
    print(f"  정면데드밴드: ±{STOP_FRONT_DEADBAND}° (stop zone 노이즈 억제)")
    print("=" * 55)

    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)

    lidar.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))
    lidar.read(7)

    scan_points  = []
    last_send    = time.time()
    last_cmd_str = ""

    try:
        while True:
            read_arduino(arduino)
            raw = lidar.read(5)
            result = parse_packet(raw)
            if result is None: continue

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

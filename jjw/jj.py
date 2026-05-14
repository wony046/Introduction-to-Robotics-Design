"""
RPLIDAR C1 장애물 회피 - 코사인 법칙 기반 물리적 빈 공간(Gap) 탐색 적용

포트: 라이다 /dev/ttyUSB0 / 아두이노 /dev/ttyAMA3 (UART)

[핵심 변경 이력]
  - calc_law_of_cosines 함수 추가: 두 점 사이의 실제 물리적 거리(mm) 산출
  - get_gap_width 로직 추가: 장애물 모서리(Edge) 검출 및 병목 구간 너비 계산
  - 점수제 가중치 변경: 여유 각도(°) 기반에서 물리적 너비(mm) 기반 점수제로 전환
  - MIN_PASSAGE_WIDTH (280mm) 설정: 로봇 폭(220) + 여유(60) 확보 시에만 통과 시도
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

# ── 로봇 파라미터 & 기구학 ────────────────────────────────────────────────────
ROBOT_HALF_WIDTH = 110   # mm: 라이다 중심 ~ 좌우 끝
SAFETY_MARGIN    = 30    # mm: 수평 안전 여유
MIN_PASSAGE_WIDTH = 280  # mm: 이 물리적 너비 이상이어야 통과 가능하다고 판단

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
SIDE_FWD_DEADZONE = 180  # mm: 모서리 사각지대 방지를 위한 전방 데드존

# ── 헤딩 방향 점수제 (mm 기반으로 스케일 조정) ─────────────────────────────────
HEADING_WEIGHT_MM = 5.0  # 헤딩 1°당 여유공간 5mm의 가중치 보너스

# ── 스캔 파라미터 ─────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE  = 90
ANGLE_STEP       = 5
SEND_INTERVAL    = 0.1
DEPTH_JUMP_THRES = 200   # mm: 이 이상 거리가 뛰면 다른 물체(장애물과 벽의 분리)로 간주

# ── 전역 상태 ─────────────────────────────────────────────────────────────────
arduino_heading_deg = 0.0
avoidance_w_sign    = 0.0
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
    """코사인 제2법칙: 라이다 상의 두 점 사이의 실제 물리적 거리를 계산합니다."""
    theta = math.radians(abs(angle_diff_deg))
    # W = sqrt(d1^2 + d2^2 - 2*d1*d2*cos(theta))
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
# 측면 감지 & 코사인 법칙 기반 빈 공간 너비 계산
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def side_horiz_blocked(scan_points, is_left):
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID: continue
        if is_left and angle_norm >= 0: continue
        if not is_left and angle_norm <= 0: continue
        horiz, fwd = decompose(angle_norm, dist)
        if fwd < SIDE_FWD_DEADZONE: continue
        if horiz < SIDE_HORIZ_LIMIT: return True
    return False

def get_gap_width(scan_points, ref_angle, ref_dist, is_left):
    """
    가장 위험한 장애물을 기준으로 왼쪽/오른쪽의 빈 공간 너비(mm)를 계산합니다.
    """
    edge_p = (ref_angle, ref_dist)
    wall_points = []
    
    # 중앙을 기준으로 왼쪽/오른쪽 점들을 분리하여 바깥쪽으로 스캔
    if is_left:
        search_points = [p for p in scan_points if p[0] <= ref_angle]
        search_points.sort(key=lambda x: x[0], reverse=True) # 각도 0 -> -90 방향
    else:
        search_points = [p for p in scan_points if p[0] >= ref_angle]
        search_points.sort(key=lambda x: x[0]) # 각도 0 -> 90 방향

    # 1. 거리 단절(Depth Jump)을 찾아 장애물의 끝 모서리(Edge) 도출
    for i, p in enumerate(search_points):
        if p[1] - edge_p[1] > DEPTH_JUMP_THRES:
            wall_points = search_points[i:] # 여기서부터는 뒷쪽의 벽/빈 공간
            break
        edge_p = p # 단절이 없으면 모서리 확장
        
    # 2. 장애물 모서리와 반대편 벽 사이의 최소 간격(병목 너비) 계산
    if wall_points:
        min_width = min(calc_law_of_cosines(edge_p[1], wp[1], edge_p[0] - wp[0]) for wp in wall_points)
        return min_width
    else:
        # 뚜렷한 벽이 감지되지 않은 경우: 남은 각도만큼 뚫려있다고 가정하고 최대 거리 대비 너비 계산
        rem_angle = (-SCAN_HALF_ANGLE - edge_p[0]) if is_left else (SCAN_HALF_ANGLE - edge_p[0])
        if abs(rem_angle) > 15:
            return calc_law_of_cosines(edge_p[1], DETECTION_RANGE, rem_angle)
        return 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 회피 방향 결정 (코사인 법칙 너비 기반)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def select_direction_by_width(left_width, right_width, heading_deg,
                              left_side_blocked, right_side_blocked):
    left_ok  = (left_width  >= MIN_PASSAGE_WIDTH) and not left_side_blocked
    right_ok = (right_width >= MIN_PASSAGE_WIDTH) and not right_side_blocked

    side_log = []
    if left_side_blocked:  side_log.append("왼쪽측면차단")
    if right_side_blocked: side_log.append("오른쪽측면차단")
    if side_log:
        print(f"  [측면감지] {' / '.join(side_log)}")

    # 강제 회전 조건
    if left_ok and not right_ok:
        reason = f"오른쪽 불가(너비:{right_width:.0f}mm" + (" 측면차단" if right_side_blocked else "") + ")"
        print(f"  [방향] {reason} → 왼쪽 강제")
        return 1.0
    if right_ok and not left_ok:
        reason = f"왼쪽 불가(너비:{left_width:.0f}mm" + (" 측면차단" if left_side_blocked else "") + ")"
        print(f"  [방향] {reason} → 오른쪽 강제")
        return -1.0
    if not left_ok and not right_ok:
        print(f"  [방향] 양쪽 불가 → {'왼쪽' if left_width >= right_width else '오른쪽'} 선택 (비상탈출)")
        return 1.0 if left_width >= right_width else -1.0

    # 점수 계산 (너비 mm + 헤딩 보정값)
    left_score  = left_width  + max(0.0, -heading_deg) * HEADING_WEIGHT_MM
    right_score = right_width + max(0.0,  heading_deg) * HEADING_WEIGHT_MM

    bonus_side = "R" if heading_deg > 0 else "L"
    bonus_val  = abs(heading_deg) * HEADING_WEIGHT_MM
    print(f"  [방향점수] L={left_score:.0f}점  R={right_score:.0f}점"
          f"  (너비 L={left_width:.0f}mm R={right_width:.0f}mm"
          f"  헤딩보너스 {bonus_side}+{bonus_val:.0f})")

    return 1.0 if left_score >= right_score else -1.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# v/w 명령 계산
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_command(scan_points, heading_deg):
    global avoidance_w_sign, no_danger_count
    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN

    danger_points = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        horiz, fwd = decompose(angle_norm, dist)
        if fwd > 0 and fwd <= FORWARD_RANGE and horiz < threshold:
            danger_points.append((angle_norm, dist, horiz, fwd))

    # 위험구역 초기화
    NO_DANGER_RESET = 3
    if not danger_points:
        no_danger_count += 1
        if no_danger_count >= NO_DANGER_RESET: avoidance_w_sign = 0.0
        return FORWARD_SPEED, 0.0
    no_danger_count = 0

    stop_points = [p for p in danger_points if p[3] <= STOP_FWD_RANGE and p[2] < ROBOT_HALF_WIDTH]
    frontal     = [p for p in danger_points if p[3] >= p[2]]
    n_fwd_ref   = min((p[3] for p in frontal), default=SLOW_START_DIST + 1)
    
    # 기준 장애물 도출
    horiz_ref = min(danger_points, key=lambda p: p[2])
    ref_angle, ref_dist, n_horiz, _ = horiz_ref

    print(f"  [기준] 전방:{n_fwd_ref:.0f}mm  정지:{len(stop_points)}개  "
          f"각도:{ref_angle:.1f}°  수평:{n_horiz:.0f}mm")

    # 선속도 결정
    if stop_points: v = 0.0
    elif n_fwd_ref >= SLOW_START_DIST: v = FORWARD_SPEED
    else:
        ratio = (n_fwd_ref - STOP_FWD_RANGE) / (SLOW_START_DIST - STOP_FWD_RANGE)
        v = max(FORWARD_SPEED * ratio, MIN_SPEED)

    horiz_error = threshold - n_horiz
    if horiz_error <= 0:
        avoidance_w_sign = 0.0
        return v, 0.0

    left_side_blocked  = side_horiz_blocked(scan_points, is_left=True)
    right_side_blocked = side_horiz_blocked(scan_points, is_left=False)

    if stop_points:
        # 비상 정지 시에는 가장 가까운 장애물을 피해 즉각 회전 시도
        stop_angle = min(stop_points, key=lambda p: p[2])[0]
        raw_sign   = 1.0 if stop_angle >= 0 else -1.0
        is_raw_blocked = (raw_sign > 0 and left_side_blocked) or (raw_sign < 0 and right_side_blocked)
        if is_raw_blocked:
            alt_sign = -raw_sign
            is_alt_blocked = (alt_sign > 0 and left_side_blocked) or (alt_sign < 0 and right_side_blocked)
            avoidance_w_sign = raw_sign if is_alt_blocked else alt_sign
            print(f"  [정지구역] 측면차단 우회 → {'왼쪽' if avoidance_w_sign > 0 else '오른쪽'}")
        else:
            avoidance_w_sign = raw_sign
            print(f"  [정지구역] 즉각 결정 → {'왼쪽' if avoidance_w_sign > 0 else '오른쪽'}")
    else:
        # 코사인 법칙을 활용한 물리적 Gap 너비 계산
        left_gap_width  = get_gap_width(scan_points, ref_angle, ref_dist, is_left=True)
        right_gap_width = get_gap_width(scan_points, ref_angle, ref_dist, is_left=False)

        if avoidance_w_sign == 0.0:
            avoidance_w_sign = select_direction_by_width(
                left_gap_width, right_gap_width, heading_deg, left_side_blocked, right_side_blocked
            )
            print(f"  [방향결정] {'왼쪽' if avoidance_w_sign > 0 else '오른쪽'} 고착")
        else:
            committed_width = left_gap_width if avoidance_w_sign > 0 else right_gap_width
            committed_blocked = (avoidance_w_sign > 0 and left_side_blocked) or \
                                (avoidance_w_sign < 0 and right_side_blocked)
            
            # 기존 방향의 여유 너비가 통과 가능 최소폭(280mm) 미만이거나 측면이 막히면 재탐색
            if committed_width < MIN_PASSAGE_WIDTH or committed_blocked:
                old = avoidance_w_sign
                avoidance_w_sign = select_direction_by_width(
                    left_gap_width, right_gap_width, heading_deg, left_side_blocked, right_side_blocked
                )
                if avoidance_w_sign != old:
                    reason = "너비부족" if committed_width < MIN_PASSAGE_WIDTH else "측면차단"
                    print(f"  [방향전환] {reason} → {'왼쪽' if avoidance_w_sign > 0 else '오른쪽'}")

    # 각속도 산출
    w_mag = max(min(W_GAIN * horiz_error / threshold, MAX_W), W_MIN_DANGER)
    w     = avoidance_w_sign * w_mag

    print(f"  [명령] v:{v:.2f}  w:{w:.2f}  (수평오차:{horiz_error:.0f}mm)")
    return v, w


def main():
    global prev_w
    print("=== RPLIDAR 장애물 회피 (코사인 법칙 기반 Gap 탐색 적용) ===")
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
                front_points = [(a, d) for a, d in scan_points if is_in_front(a) and d > 0]
                now = time.time()
                if now - last_send >= SEND_INTERVAL:
                    v, w = find_vw_command(front_points, arduino_heading_deg)
                    w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
                    prev_w = w
                    cmd = f"{v:.2f} {w:.2f}\n"
                    arduino.write(cmd.encode())
                    if cmd != last_cmd_str:
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

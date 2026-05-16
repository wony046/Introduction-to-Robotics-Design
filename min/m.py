"""
RPLIDAR C1 장애물 회피 - 좁은 통로 특화 (진동 억제 최소 수정 버전)

[기존 알고리즘 유지] 코사인 법칙 너비 계산 + 방향 메모리 + 뚝심 돌파
  → 회피 동작은 검증됨, 건드리지 않음

[수정 - 진동만 잡기]
  1) 조향 각도 콘 제한 (STEERING_ANGLE_LIMIT)
     단일 장애물이 옆으로 빠진 후에도 조향 대상이 되어 진동을 유발하던 문제
     → 라이다 각도 |±45°| 초과 시 danger_points에서 제외 (감속 대상에는 남음)

  2) 방향 전환 시간 잠금 (DIRECTION_LOCK_TIME)
     너비 노이즈로 매 프레임 방향이 뒤집히던 문제
     → 한 번 결정된 방향은 최소 0.8초 유지

  3) 스무딩 개선
     부호 반전 시 prev_w=0 리셋 → 새 명령 그대로 통과 (진폭 안 줄어듦)
     → 항상 EMA 적용으로 자연 감쇠

포트: 라이다 /dev/ttyUSB0 / 아두이노 /dev/ttyAMA3
"""

import serial
import time
import math
import traceback

# ── 포트 ─────────────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

# ── 라이다 보정 ───────────────────────────────────────────────────────────────
LIDAR_OFFSET    = 20
LIDAR_MIN_VALID = 100

# ── 로봇 파라미터 ─────────────────────────────────────────────────────────────
ROBOT_HALF_WIDTH  = 110
SAFETY_MARGIN     = 30
MIN_PASSAGE_WIDTH = 240

# ── 위험구역 ──────────────────────────────────────────────────────────────────
DETECTION_RANGE  = 1500
FORWARD_RANGE    = 550

# ── 속도 ──────────────────────────────────────────────────────────────────────
FORWARD_SPEED    = 0.35
MIN_SPEED        = 0.07
SLOW_START_DIST  = 400
STOP_FWD_RANGE   = 180
W_GAIN           = 0.7
MAX_W            = 0.65
W_MIN_DANGER     = 0.45
W_MIN_MOVING     = 0.10
W_SMOOTH         = 0.75

# ── 측면 감지 ─────────────────────────────────────────────────────────────────
SIDE_HORIZ_LIMIT  = 90
SIDE_FWD_DEADZONE = 130

# ── 근접 후보 ─────────────────────────────────────────────────────────────────
PROXIMITY_HORIZ = 170

# ── 헤딩 보너스 ───────────────────────────────────────────────────────────────
HEADING_WEIGHT_MM = 5.0

# ── 스캔 ──────────────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE  = 70
ANGLE_STEP       = 5
SEND_INTERVAL    = 0.1
DEPTH_JUMP_THRES = 120
LIDAR_WATCHDOG_TIMEOUT = 0.5
NO_DANGER_RESET        = 20

STOP_FRONT_DEADBAND = 15

# ── [신규] 진동 억제 파라미터 ────────────────────────────────────────────────
STEERING_ANGLE_LIMIT = 45.0   # deg: 이 각도 넘어가면 조향 대상 제외 (감속만)
DIRECTION_LOCK_TIME  = 0.8    # s: 방향 전환 후 이 시간 동안 재전환 금지

# ── 안전 기능 파라미터 ───────────────────────────────────────────────────────
KEEPALIVE_INTERVAL  = 0.30
PRINT_INTERVAL      = 0.5
RESYNC_THRESHOLD    = 100
DIAG_INTERVAL       = 2.0
LIDAR_SYNC_TIMEOUT  = 3.0

# ── 전역 상태 ─────────────────────────────────────────────────────────────────
arduino_heading_deg = 0.0
arduino_buf         = ""
avoidance_w_sign    = 0.0
stop_zone_w_sign    = 0.0
no_danger_count     = 0
prev_w              = 0.0
prev_v              = 0.0
last_direction_change_time = 0.0   # [신규] 방향 전환 잠금용


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티 & 수학
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return angle - 360 if angle > 180 else angle

def is_in_front(angle_norm):
    return -SCAN_HALF_ANGLE <= angle_norm <= SCAN_HALF_ANGLE


def decompose_signed(angle_norm_deg, distance_mm):
    """반환: (x = 정면 mm, y = 좌측 양수 mm)"""
    rad = math.radians(angle_norm_deg)
    x =  distance_mm * math.cos(rad)
    y = -distance_mm * math.sin(rad)
    return x, y


def decompose(angle_norm_deg, distance_mm):
    x, y = decompose_signed(angle_norm_deg, distance_mm)
    return abs(y), x


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
    global arduino_heading_deg, arduino_buf
    if arduino.in_waiting <= 0:
        return
    try:
        arduino_buf += arduino.read(arduino.in_waiting).decode('utf-8', errors='ignore')
        while '\n' in arduino_buf:
            line, arduino_buf = arduino_buf.split('\n', 1)
            line = line.strip()
            if line.startswith('H:'):
                arduino_heading_deg = float(line[2:])
    except Exception:
        pass


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 라이다 재동기화 & 안전한 시작
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_lidar_sync(lidar, verbose=True):
    if verbose:
        print("[라이다] 재동기화 시도...", flush=True)
    deadline = time.time() + LIDAR_SYNC_TIMEOUT
    while time.time() < deadline:
        b = lidar.read(1)
        if len(b) == 0:
            continue
        if (b[0] & 0x01) == ((b[0] >> 1) & 0x01):
            continue
        b2 = lidar.read(1)
        if len(b2) == 0:
            continue
        if (b2[0] & 0x01) != 1:
            continue
        rest = lidar.read(3)
        if len(rest) != 3:
            continue
        result = parse_packet(b + b2 + rest)
        if result is None:
            continue
        angle, distance = result
        if 0 <= angle <= 360 and 0 <= distance <= 10000:
            if verbose:
                print(f"[라이다] 동기화 OK (각도={angle:.1f}°, 거리={distance:.0f}mm)",
                      flush=True)
            return True
    if verbose:
        print("[라이다] ✗ 재동기화 실패", flush=True)
    return False


def start_lidar(lidar):
    print("[라이다] 시작 중...", flush=True)
    try:
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        try:
            lidar.dtr = False
        except AttributeError:
            pass
        time.sleep(0.5)
        lidar.reset_input_buffer()
        lidar.write(bytes([0xA5, 0x20]))
        time.sleep(0.5)
        descriptor = lidar.read(7)
        print(f"[라이다] descriptor: {descriptor.hex()}", flush=True)
        return True
    except serial.SerialException as e:
        print(f"[라이다] ✗ 시작 실패: {e}", flush=True)
        return False
    except Exception as e:
        print(f"[라이다] ✗ 예외: {e}", flush=True)
        return False


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 측면 감지
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def side_horiz_blocked(scan_points, is_left):
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID:
            continue
        x, y = decompose_signed(angle_norm, dist)
        if is_left     and y <= 0: continue
        if not is_left and y >= 0: continue
        if x < SIDE_FWD_DEADZONE: continue
        if abs(y) < SIDE_HORIZ_LIMIT:
            return True
    return False


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 코사인 법칙 기반 빈 공간 너비
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def get_gap_width(scan_points, ref_angle, ref_dist, is_left):
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
              f"연속벽 끝까지 → 보수적 너비={width:.0f}mm")
        return width

    horiz_edge = abs(edge_p[1] * math.sin(math.radians(edge_p[0])))
    available  = max(horiz_edge - ROBOT_HALF_WIDTH, 0.0)
    print(f"  [gap {'L' if is_left else 'R'}] 스캔 각도 부족 → 수평여유 {available:.0f}mm")
    return available


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 회피 방향 결정 (danger zone)
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

    if left_ok and not right_ok:
        print(f"  [방향] 오른쪽 불가(너비:{right_width:.0f}mm) → 왼쪽 강제")
        return 1.0

    if right_ok and not left_ok:
        print(f"  [방향] 왼쪽 불가(너비:{left_width:.0f}mm) → 오른쪽 강제")
        return -1.0

    if not left_ok and not right_ok:
        chosen = "왼쪽" if left_width >= right_width else "오른쪽"
        print(f"  [방향] 양쪽 좁음(L:{left_width:.0f} R:{right_width:.0f}mm)"
              f" → {chosen} 뚝심 돌파")
        return 1.0 if left_width >= right_width else -1.0

    left_score  = left_width  + max(0.0, -heading_deg) * HEADING_WEIGHT_MM
    right_score = right_width + max(0.0,  heading_deg) * HEADING_WEIGHT_MM

    print(f"  [방향점수] L={left_score:.0f}  R={right_score:.0f}")
    return 1.0 if left_score >= right_score else -1.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# v/w 명령 계산
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_command(scan_points, heading_deg):
    global avoidance_w_sign, stop_zone_w_sign, no_danger_count
    global last_direction_change_time

    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN
    now = time.time()

    # ── 1. 위험 포인트 수집 ───────────────────────────────────────────────────
    # [수정 1] 조향용과 감속용을 분리
    #   - danger_points_steer: 조향 계산에 사용 (각도 제한 적용)
    #   - danger_points_slow : 감속 판단에 사용 (각도 제한 없음, 옆 장애물도 포함)
    danger_points_steer = []
    danger_points_slow  = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE:
            continue
        x, y = decompose_signed(angle_norm, dist)
        horiz = abs(y)
        if horiz > 380:
            continue
        if x > 0 and x <= FORWARD_RANGE and horiz < threshold:
            point = (angle_norm, dist, horiz, x, y)
            danger_points_slow.append(point)
            # [수정 1] 옆으로 빠진 장애물은 조향에서 제외
            if abs(angle_norm) <= STEERING_ANGLE_LIMIT:
                danger_points_steer.append(point)

    if not danger_points_slow:
        no_danger_count += 1
        if no_danger_count >= NO_DANGER_RESET:
            avoidance_w_sign = 0.0
            stop_zone_w_sign = 0.0
        return FORWARD_SPEED, 0.0
    no_danger_count = 0

    # ── 2. 근접 후보 ─────────────────────────────────────────────────────────
    proximity_points = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE:
            continue
        if abs(angle_norm) > STEERING_ANGLE_LIMIT:
            continue   # 조향용 ref 안정화이므로 동일 각도 필터
        x, y = decompose_signed(angle_norm, dist)
        horiz = abs(y)
        if x > 0 and x <= FORWARD_RANGE and threshold <= horiz < PROXIMITY_HORIZ:
            proximity_points.append((angle_norm, dist, horiz, x, y))

    # ── 3. 선속도 결정 (감속용 사용) ─────────────────────────────────────────
    stop_points = [p for p in danger_points_slow
                   if p[3] <= STOP_FWD_RANGE and p[2] < ROBOT_HALF_WIDTH]
    
    frontal     = [p for p in danger_points_slow if p[2] <= ROBOT_HALF_WIDTH + 10]
    n_fwd_ref   = min((p[3] for p in frontal), default=SLOW_START_DIST + 1)

    if stop_points:
        v = 0.0
    elif n_fwd_ref >= SLOW_START_DIST:
        v = FORWARD_SPEED
    else:
        ratio = (n_fwd_ref - STOP_FWD_RANGE) / (SLOW_START_DIST - STOP_FWD_RANGE)
        v = max(FORWARD_SPEED * ratio, MIN_SPEED)

    # ── 4. 조향 대상이 없으면 직진 (감속만 적용) ─────────────────────────────
    # [수정 1] 옆으로 빠진 장애물만 남았을 때: 더 이상 꺾지 않음
    if not danger_points_steer:
        avoidance_w_sign = 0.0
        stop_zone_w_sign = 0.0
        print(f"  [조향대상 없음] 옆 장애물만 → 직진 (v={v:.2f})")
        return v, 0.0

    horiz_ref_danger = min(danger_points_steer, key=lambda p: p[2])
    n_horiz          = horiz_ref_danger[2]
    horiz_error      = threshold - n_horiz

    ref_candidates   = danger_points_steer + proximity_points
    horiz_ref_stable = min(ref_candidates, key=lambda p: p[2])
    ref_angle        = horiz_ref_stable[0]
    ref_dist         = horiz_ref_stable[1]

    is_proximity_ref = horiz_ref_stable not in danger_points_steer
    print(f"  [기준] 전방:{n_fwd_ref:.0f}mm  정지:{len(stop_points)}개  "
          f"ref각도:{ref_angle:.1f}°  수평:{n_horiz:.0f}mm"
          + (" [근접후보]" if is_proximity_ref else ""))

    if horiz_error <= 0:
        avoidance_w_sign = 0.0
        stop_zone_w_sign = 0.0
        return v, 0.0

    # ── 5. 회전 방향 결정 ────────────────────────────────────────────────────
    if stop_points:
        # Stop zone
        nearest_stop = min(stop_points, key=lambda p: p[2])
        stop_y       = nearest_stop[4]
        stop_angle   = nearest_stop[0]

        if abs(stop_angle) < STOP_FRONT_DEADBAND and stop_zone_w_sign != 0.0:
            avoidance_w_sign = stop_zone_w_sign
            print(f"  [정지구역] 정면 노이즈(각도:{stop_angle:.1f}°) → "
                  f"기존 방향 유지({'왼쪽' if avoidance_w_sign > 0 else '오른쪽'})")
        else:
            if stop_y > 0:
                new_sign = -1.0
            elif stop_y < 0:
                new_sign = 1.0
            else:
                new_sign = 1.0 if heading_deg <= 0 else -1.0

            if new_sign != avoidance_w_sign:
                last_direction_change_time = now
            avoidance_w_sign = new_sign
            stop_zone_w_sign = avoidance_w_sign
            print(f"  [정지구역] y={stop_y:+.0f}mm 각도:{stop_angle:.1f}° → "
                  f"{'좌회전' if avoidance_w_sign > 0 else '우회전'} 즉결")

    else:
        # Danger zone
        stop_zone_w_sign = 0.0

        left_side_blocked  = side_horiz_blocked(scan_points, is_left=True)
        right_side_blocked = side_horiz_blocked(scan_points, is_left=False)

        left_width  = get_gap_width(scan_points, ref_angle, ref_dist, is_left=True)
        right_width = get_gap_width(scan_points, ref_angle, ref_dist, is_left=False)

        if avoidance_w_sign == 0.0:
            avoidance_w_sign = select_direction_by_width(
                left_width, right_width, heading_deg,
                left_side_blocked, right_side_blocked
            )
            last_direction_change_time = now
            print(f"  [방향결정] {'좌회전' if avoidance_w_sign > 0 else '우회전'} 고착")
        else:
            committed_width   = left_width  if avoidance_w_sign > 0 else right_width
            opposite_width    = right_width if avoidance_w_sign > 0 else left_width
            committed_blocked = (avoidance_w_sign > 0 and left_side_blocked) or \
                                (avoidance_w_sign < 0 and right_side_blocked)

            need_reselect = committed_blocked or (
                committed_width < MIN_PASSAGE_WIDTH
                and opposite_width >= MIN_PASSAGE_WIDTH * 1.5
            )

            # [수정 2] 시간 잠금: 최근 방향 전환 후 일정 시간은 재전환 금지
            #         단, 측면 물리 차단(committed_blocked)은 안전상 즉시 허용
            time_locked = (now - last_direction_change_time) < DIRECTION_LOCK_TIME
            if need_reselect and time_locked and not committed_blocked:
                print(f"  [방향잠금] 전환 후 {now - last_direction_change_time:.2f}s "
                      f"(< {DIRECTION_LOCK_TIME}s) → 재전환 무시")
                need_reselect = False

            if need_reselect:
                old = avoidance_w_sign
                avoidance_w_sign = select_direction_by_width(
                    left_width, right_width, heading_deg,
                    left_side_blocked, right_side_blocked
                )
                if avoidance_w_sign != old:
                    last_direction_change_time = now
                    reason = "측면물리차단" if committed_blocked else "반대쪽 통과 가능"
                    print(f"  [방향전환] {reason} → "
                          f"{'좌회전' if avoidance_w_sign > 0 else '우회전'}")

    # ── 6. 각속도 계산 ────────────────────────────────────────────────────────
    w_min = W_MIN_DANGER if v == 0.0 else W_MIN_MOVING
    w_mag = max(min(W_GAIN * horiz_error / threshold, MAX_W), w_min)
    w     = avoidance_w_sign * w_mag

    # 관통 모드
    min_strict_fwd = min((p[3] for p in danger_points_steer if p[2] < ROBOT_HALF_WIDTH),
                        default=FORWARD_RANGE)
    if min_strict_fwd > STOP_FWD_RANGE and ROBOT_HALF_WIDTH <= n_horiz < threshold:
        w = 0.0
        print(f"  [관통모드] 대각선/측면 스치기 → 조향 잠금 (w=0)")

    print(f"  [명령] v:{v:.2f}  w:{w:.2f}  (수평오차:{horiz_error:.0f}mm)")
    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    global prev_w, prev_v

    print("=" * 60)
    print("RPLIDAR 장애물 회피 (진동 억제 최소 수정 버전)")
    print("=" * 60)
    print(f"  알고리즘    : 기존 코사인+방향메모리+뚝심돌파 유지")
    print(f"  진동 억제 1 : 조향 각도 콘 ±{STEERING_ANGLE_LIMIT}° (옆 장애물 제외)")
    print(f"  진동 억제 2 : 방향 전환 잠금 {DIRECTION_LOCK_TIME}s")
    print(f"  진동 억제 3 : 스무딩 부호반전 리셋 제거")
    print(f"  통과너비    : {MIN_PASSAGE_WIDTH}mm 이상")
    print("=" * 60)

    lidar = None
    arduino = None
    try:
        lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
        time.sleep(0.5)
    except Exception as e:
        print(f"[치명] 라이다 포트 열기 실패: {e}")
        return

    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=0.1)
        time.sleep(0.5)
    except Exception as e:
        print(f"[치명] 아두이노 포트 열기 실패: {e}")
        if lidar: lidar.close()
        return

    arduino.write(b"0.00 0.00\n")
    time.sleep(0.3)

    if not start_lidar(lidar):
        arduino.write(b"0.00 0.00\n")
        lidar.close()
        arduino.close()
        return

    if not find_lidar_sync(lidar):
        arduino.write(b"0.00 0.00\n")
        try: lidar.write(bytes([0xA5, 0x25]))
        except: pass
        lidar.close()
        arduino.close()
        return

    print("\n주행 시작!\n", flush=True)

    scan_points     = []
    last_send       = time.time()
    last_scan_time  = time.time()
    last_cmd_time   = time.time()
    last_print_time = 0.0
    last_diag_time  = time.time()
    last_cmd_str    = ""
    invalid_count   = 0
    packet_count    = 0

    try:
        while True:
            read_arduino(arduino)
            raw = lidar.read(5)

            now = time.time()

            if now - last_scan_time > LIDAR_WATCHDOG_TIMEOUT:
                arduino.write(b"0.00 0.00\n")
                last_cmd_time = now
                print(f"[경고] 라이다 스캔 없음 ({LIDAR_WATCHDOG_TIMEOUT}s) → 비상 정지")
                last_scan_time = now

            if len(raw) < 5:
                invalid_count += 1
            else:
                result = parse_packet(raw)
                if result is None:
                    invalid_count += 1
                    if invalid_count >= RESYNC_THRESHOLD:
                        print(f"[라이다] 파싱실패 {invalid_count}회 → 재동기화", flush=True)
                        lidar.reset_input_buffer()
                        if find_lidar_sync(lidar, verbose=False):
                            print("[라이다] 재동기화 성공", flush=True)
                        invalid_count = 0
                        scan_points = []
                else:
                    invalid_count = 0
                    packet_count += 1
                    angle_raw, distance = result
                    s_flag = raw[0] & 0x01

                    if s_flag == 1 and scan_points:
                        last_scan_time = time.time()
                        front_points = [
                            (a, d) for a, d in scan_points
                            if is_in_front(a) and d > 0
                        ]
                        now = time.time()
                        if now - last_send >= SEND_INTERVAL:
                            v, w = find_vw_command(front_points, arduino_heading_deg)

                            # [수정 3] 부호 반전 시 prev_w=0 리셋 제거
                            # 기존: 부호 바뀌면 0으로 리셋 → 새 명령 그대로 통과 (진폭 안 줄어듦)
                            # 개선: 항상 EMA → 부호 반전 시 더 완만하게 변함
                            w = W_SMOOTH * prev_w + (1.0 - W_SMOOTH) * w
                            prev_w = w
                            prev_v = v

                            cmd = f"{v:.2f} {w:.2f}\n"
                            arduino.write(cmd.encode())
                            last_cmd_time = now

                            if cmd != last_cmd_str:
                                if now - last_print_time >= PRINT_INTERVAL:
                                    print(f"[전송] v={v:.2f}  w={w:.2f}  "
                                          f"헤딩={arduino_heading_deg:.1f}°", flush=True)
                                    last_print_time = now
                                last_cmd_str = cmd
                            last_send = now
                        scan_points = []

                    scan_points.append((
                        normalize_angle(angle_raw),
                        distance + LIDAR_OFFSET if distance > 0 else 0
                    ))

            now = time.time()
            if now - last_cmd_time > KEEPALIVE_INTERVAL:
                cmd = f"{prev_v:.2f} {prev_w:.2f}\n"
                arduino.write(cmd.encode())
                last_cmd_time = now

            if now - last_diag_time >= DIAG_INTERVAL:
                if packet_count == 0:
                    print("[진단] 패킷 0 — 라이다 연결 의심", flush=True)
                packet_count = 0
                last_diag_time = now

    except KeyboardInterrupt:
        print("\n[종료] Ctrl-C 수신", flush=True)
    except serial.SerialException as e:
        print(f"\n[치명] 시리얼 예외: {e}", flush=True)
        traceback.print_exc()
    except Exception as e:
        print(f"\n[치명] 예상치 못한 예외: {e}", flush=True)
        traceback.print_exc()
    finally:
        print("[정리] 정지 신호 전송 및 포트 닫기", flush=True)
        if arduino is not None:
            try:
                arduino.write(b"0.00 0.00\n")
                time.sleep(0.1)
                arduino.write(b"0.00 0.00\n")
            except Exception:
                pass
            try: arduino.close()
            except Exception: pass
        if lidar is not None:
            try:
                lidar.write(bytes([0xA5, 0x25]))
                time.sleep(0.1)
            except Exception:
                pass
            try: lidar.close()
            except Exception: pass
        print("[종료] 완료", flush=True)


if __name__ == "__main__":
    main()

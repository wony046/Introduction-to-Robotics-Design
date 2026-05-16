"""
RPLIDAR C1 장애물 회피 - 좁은 통로 특화 (병합 버전)

[알고리즘] 첫 번째 코드 기반:
  - 코사인 법칙 기반 좌/우 통과 너비 계산
  - proximity_points로 ref 안정화 (방법 A)
  - Stop zone / Danger zone 방향 메모리 분리
  - "양쪽 좁아도 뚝심 돌파" — 절대 멈추지 않음

[안전 기능] 두 번째 코드에서 도입 (우선순위 적용 순):
  1. 메인 루프 try/except + finally 정지 ─ 어떤 예외든 Arduino에 정지 신호
  2. Keepalive ─ 0.3초마다 마지막 명령 재전송 (회전 중 패킷 손실 방지)
  3. 라이다 재동기화 ─ 패킷 100개 연속 깨지면 sync 재시도
  4. 좌표 변환 통합 ─ decompose_signed로 표준 좌표계(x=정면, y=좌측+) 일원화
                      → Stop zone 부호 버그 구조적 해결
  5. Print 스로틀링 ─ 상태 로그 2Hz, 이벤트 로그는 즉시
  6. start_lidar 안전 처리 ─ try/except + descriptor 출력

[중요 - 좌표/부호 컨벤션 통일]
  라이다 원시: 양수 각도 = 우측 (CW)
  표준 좌표계: x = 정면(mm),  y = 좌측 양수(mm)
  로봇 회전:   +w = 좌회전, -w = 우회전
  → "y > 0 (왼쪽 장애물)이면 우회전(-w)으로 피한다"

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
LIDAR_OFFSET    = 20    # mm
LIDAR_MIN_VALID = 100   # mm: 이 미만은 라이다 오류로 간주 → 무시

# ── 로봇 파라미터 ─────────────────────────────────────────────────────────────
ROBOT_HALF_WIDTH  = 110  # mm
SAFETY_MARGIN     = 30   # mm: threshold = 140mm
MIN_PASSAGE_WIDTH = 240  # mm

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

# ── 근접 후보 (방법 A) ────────────────────────────────────────────────────────
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

# ── [신규] 안전 기능 파라미터 ────────────────────────────────────────────────
KEEPALIVE_INTERVAL  = 0.30   # s: 마지막 명령 재전송 주기
PRINT_INTERVAL      = 0.5    # s: 상태 로그 스로틀링 (2Hz)
RESYNC_THRESHOLD    = 100    # 연속 파싱 실패 개수 → 재동기화 트리거
DIAG_INTERVAL       = 2.0    # s: 패킷 카운터 진단 주기
LIDAR_SYNC_TIMEOUT  = 3.0    # s: 동기화 최대 대기 시간

# ── 전역 상태 ─────────────────────────────────────────────────────────────────
arduino_heading_deg = 0.0
arduino_buf         = ""
avoidance_w_sign    = 0.0
stop_zone_w_sign    = 0.0
no_danger_count     = 0
prev_w              = 0.0
prev_v              = 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티 & 수학
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return angle - 360 if angle > 180 else angle

def is_in_front(angle_norm):
    return -SCAN_HALF_ANGLE <= angle_norm <= SCAN_HALF_ANGLE


# [개선 4] 좌표 변환 통합 ─────────────────────────────────────────────────────
# 라이다 양수 각도 = 우측, 우리는 y가 좌측 양수가 되도록 통일
def decompose_signed(angle_norm_deg, distance_mm):
    """반환: (x = 정면 mm, y = 좌측 양수 mm)
    
    y > 0 → 장애물이 왼쪽   → 우회전(-w)으로 피해야 함
    y < 0 → 장애물이 오른쪽 → 좌회전(+w)으로 피해야 함
    """
    rad = math.radians(angle_norm_deg)
    x =  distance_mm * math.cos(rad)
    y = -distance_mm * math.sin(rad)   # CW → 표준 y축 반전
    return x, y


def decompose(angle_norm_deg, distance_mm):
    """기존 호환용: (horiz, fwd) — horiz는 부호 없는 절댓값"""
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
# [개선 3] 라이다 재동기화 & [개선 6] 안전한 시작
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_lidar_sync(lidar, verbose=True):
    """패킷 경계를 다시 찾는다. 패킷 파싱이 연속 실패할 때 호출."""
    if verbose:
        print("[라이다] 재동기화 시도...", flush=True)
    deadline = time.time() + LIDAR_SYNC_TIMEOUT
    while time.time() < deadline:
        b = lidar.read(1)
        if len(b) == 0:
            continue
        # 첫 바이트 유효성 검사
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
    """라이다 시작 — 예외 처리 + descriptor 출력"""
    print("[라이다] 시작 중...", flush=True)
    try:
        # 이전 상태가 어떻든 일단 정지
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        try:
            lidar.dtr = False
        except AttributeError:
            pass
        time.sleep(0.5)
        lidar.reset_input_buffer()
        # SCAN 시작
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
# 측면 감지 — 좌표 통일 후 부호 일관
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def side_horiz_blocked(scan_points, is_left):
    """y > 0 = 왼쪽, y < 0 = 오른쪽 (decompose_signed 기준)
    
    danger zone 전용: stop zone에서는 호출하지 않음
    """
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID:
            continue
        x, y = decompose_signed(angle_norm, dist)
        # 왼쪽 검사 시 y <= 0 (오른쪽) 제외, 그 반대도 마찬가지
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
    """가장 가까운 장애물(ref) 옆으로 얼마나 넓은 공간이 있는지 계산
    
    is_left=True  → 라이다 각도 음수 방향(좌측)을 탐색
    is_left=False → 라이다 각도 양수 방향(우측)을 탐색
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
    """반환: +1.0 = 좌회전(왼쪽으로 회피), -1.0 = 우회전(오른쪽으로 회피)"""
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
    """정면 스캔 + 헤딩 → (v, w) 반환
    
    좌표계 통일: 모든 측면 판정은 decompose_signed의 y로 통일
    → 라이다 양수 각도 = 우측 = y < 0
    """
    global avoidance_w_sign, stop_zone_w_sign, no_danger_count

    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN

    # ── 1. 위험 포인트 수집 ───────────────────────────────────────────────────
    danger_points = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE:
            continue
        x, y = decompose_signed(angle_norm, dist)
        horiz = abs(y)
        if horiz > 380:
            continue
        if x > 0 and x <= FORWARD_RANGE and horiz < threshold:
            # 튜플: (라이다각도, 거리, 수평절댓값, 정면거리, y부호있음)
            danger_points.append((angle_norm, dist, horiz, x, y))

    if not danger_points:
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
        x, y = decompose_signed(angle_norm, dist)
        horiz = abs(y)
        if x > 0 and x <= FORWARD_RANGE and threshold <= horiz < PROXIMITY_HORIZ:
            proximity_points.append((angle_norm, dist, horiz, x, y))

    # ── 3. 선속도 결정 ────────────────────────────────────────────────────────
    stop_points = [p for p in danger_points
                   if p[3] <= STOP_FWD_RANGE and p[2] < ROBOT_HALF_WIDTH]
    
    frontal     = [p for p in danger_points if p[2] <= ROBOT_HALF_WIDTH + 10]
    n_fwd_ref   = min((p[3] for p in frontal), default=SLOW_START_DIST + 1)

    horiz_ref_danger = min(danger_points, key=lambda p: p[2])
    n_horiz          = horiz_ref_danger[2]
    horiz_error      = threshold - n_horiz

    ref_candidates   = danger_points + proximity_points
    horiz_ref_stable = min(ref_candidates, key=lambda p: p[2])
    ref_angle        = horiz_ref_stable[0]
    ref_dist         = horiz_ref_stable[1]

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
        # 좌표 통일: 가장 가까운 stop point의 y(좌측 양수) 부호로 결정
        # y > 0 (왼쪽 장애물) → 우회전(-1.0)으로 피한다
        # y < 0 (오른쪽 장애물) → 좌회전(+1.0)으로 피한다
        nearest_stop = min(stop_points, key=lambda p: p[2])
        stop_y       = nearest_stop[4]
        stop_angle   = nearest_stop[0]

        if abs(stop_angle) < STOP_FRONT_DEADBAND and stop_zone_w_sign != 0.0:
            avoidance_w_sign = stop_zone_w_sign
            print(f"  [정지구역] 정면 노이즈(각도:{stop_angle:.1f}°) → "
                  f"기존 방향 유지({'왼쪽' if avoidance_w_sign > 0 else '오른쪽'})")
        else:
            # 좌표 통일 후 부호: y > 0 (왼쪽 장애물) → -1.0 (우회전)
            if stop_y > 0:
                avoidance_w_sign = -1.0   # 왼쪽 장애물 → 우회전
            elif stop_y < 0:
                avoidance_w_sign = 1.0    # 오른쪽 장애물 → 좌회전
            else:
                # 정확히 정면(y=0) — 헤딩 보조 사용
                avoidance_w_sign = 1.0 if heading_deg <= 0 else -1.0

            stop_zone_w_sign = avoidance_w_sign
            print(f"  [정지구역] y={stop_y:+.0f}mm 각도:{stop_angle:.1f}° → "
                  f"{'좌회전' if avoidance_w_sign > 0 else '우회전'} 즉결")

    else:
        # ── Danger zone ───────────────────────────────────────────────────
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
            if need_reselect:
                old = avoidance_w_sign
                avoidance_w_sign = select_direction_by_width(
                    left_width, right_width, heading_deg,
                    left_side_blocked, right_side_blocked
                )
                if avoidance_w_sign != old:
                    reason = "측면물리차단" if committed_blocked else "반대쪽 통과 가능"
                    print(f"  [방향전환] {reason} → "
                          f"{'좌회전' if avoidance_w_sign > 0 else '우회전'}")

    # ── 5. 각속도 계산 ────────────────────────────────────────────────────────
    w_min = W_MIN_DANGER if v == 0.0 else W_MIN_MOVING
    w_mag = max(min(W_GAIN * horiz_error / threshold, MAX_W), w_min)
    w     = avoidance_w_sign * w_mag

    # 관통 모드
    min_strict_fwd = min((p[3] for p in danger_points if p[2] < ROBOT_HALF_WIDTH),
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
    print("RPLIDAR 장애물 회피 (병합 버전 - 좁은 통로 돌파 특화)")
    print("=" * 60)
    print(f"  알고리즘    : 코사인 법칙 너비 계산 + 방향 메모리 + 뚝심 돌파")
    print(f"  통과너비    : {MIN_PASSAGE_WIDTH}mm 이상")
    print(f"  근접후보    : 수평 {PROXIMITY_HORIZ}mm 이내 (ref 안정화)")
    print(f"  좌표통일    : decompose_signed (y > 0 = 좌측)")
    print(f"  Keepalive   : {KEEPALIVE_INTERVAL}s")
    print(f"  재동기화    : 파싱실패 {RESYNC_THRESHOLD}회 연속 시")
    print(f"  로그스로틀  : 상태로그 {1/PRINT_INTERVAL:.0f}Hz / 이벤트로그 즉시")
    print("=" * 60)

    # [개선 1] 시리얼 포트 열기도 try로 감싸기
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

    # 시작 시 일단 정지 신호
    arduino.write(b"0.00 0.00\n")
    time.sleep(0.3)

    # [개선 6] 안전한 라이다 시작
    if not start_lidar(lidar):
        arduino.write(b"0.00 0.00\n")
        lidar.close()
        arduino.close()
        return

    # [개선 3] 초기 동기화
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
    last_cmd_time   = time.time()      # [개선 2] keepalive 추적
    last_print_time = 0.0              # [개선 5] print 스로틀링
    last_diag_time  = time.time()      # [개선 5] diagnostic
    last_cmd_str    = ""
    invalid_count   = 0                # [개선 3] 재동기화 트리거
    packet_count    = 0                # [개선 5] diagnostic 카운터

    # [개선 1] 메인 루프 전체를 try/except/finally로 감싸기
    try:
        while True:
            read_arduino(arduino)
            raw = lidar.read(5)

            now = time.time()

            # ── Watchdog ─────────────────────────────────────────────────────
            if now - last_scan_time > LIDAR_WATCHDOG_TIMEOUT:
                arduino.write(b"0.00 0.00\n")
                last_cmd_time = now
                print(f"[경고] 라이다 스캔 없음 ({LIDAR_WATCHDOG_TIMEOUT}s) → 비상 정지")
                last_scan_time = now

            # ── 패킷 수신 처리 ───────────────────────────────────────────────
            if len(raw) < 5:
                invalid_count += 1
            else:
                result = parse_packet(raw)
                if result is None:
                    invalid_count += 1
                    # [개선 3] 누적되면 재동기화
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
                            if w == 0.0:
                                prev_w = 0.0
                            else:
                                if w * prev_w < 0:
                                    prev_w = 0.0
                                w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
                                prev_w = w
                            prev_v = v
                            cmd = f"{v:.2f} {w:.2f}\n"
                            arduino.write(cmd.encode())
                            last_cmd_time = now    # [개선 2] keepalive 갱신
                            
                            # [개선 5] 명령 전송은 변경 시에만 출력
                            if cmd != last_cmd_str:
                                # 상태 로그는 스로틀링
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

            # ── [개선 2] Keepalive: 회전 중 패킷 손실 보완 ────────────────────
            now = time.time()
            if now - last_cmd_time > KEEPALIVE_INTERVAL:
                cmd = f"{prev_v:.2f} {prev_w:.2f}\n"
                arduino.write(cmd.encode())
                last_cmd_time = now

            # ── [개선 5] Diagnostic: 2초마다 패킷 카운터 점검 ──────────────────
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
        # [개선 1] 어떤 경로로 빠져나오든 반드시 정지 신호 전송
        print("[정리] 정지 신호 전송 및 포트 닫기", flush=True)
        if arduino is not None:
            try:
                arduino.write(b"0.00 0.00\n")
                time.sleep(0.1)
                arduino.write(b"0.00 0.00\n")   # 한 번 더, 확실히
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

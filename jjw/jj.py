"""
RPLIDAR C1 장애물 회피 코드 (v6b)
포트: 라이다 /dev/ttyUSB0 / 아두이노 /dev/ttyAMA3 (UART)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[v5 → v6 변경사항]

  [버그 수정] normalize_angle 부호 반전
    RPLIDAR C1은 위에서 봤을 때 반시계(CCW) 방향으로 스캔
    → 원시 90° = 실제 왼쪽, 원시 270° = 실제 오른쪽
    → 기존 코드는 좌/우가 뒤바뀐 채로 동작 → 장애물 방향으로 회전하는 버그
    → normalize 후 부호를 반전하여 양수=오른쪽, 음수=왼쪽으로 통일

  [개선] MISSION_HEADING_LIMIT  90° → 70°
    Arduino 하드리밋(±90°)에 도달하기 전에 RPi가 먼저 소프트 보정 개입
    완충구간 20° 확보

  [정리] 미사용 코드 제거
    - get_heading_recovery_cmd() 함수 (호출 없음)
    - HEADING_OVER_90, RECOVERY_SAFE_DIST, STOP_BACKUP_TIME
    - ESCAPE_TOLERANCE, ESCAPE_EXTRA_ANGLE
    - stop_zone_entry_time 변수 (항상 None, 실질 로직 없음)
    - execute_escape_rotation() 내부 중복 stuck_count 리셋

[막힘 감지 - LiDAR 공간 기반]
  정면 180° 스캔에서 통과 가능한 경로가 없을 때 막힘으로 판단
  각 방향에서 STUCK_CLEAR_DIST(400mm) 이상 → 열림
  열린 구간 너비를 코사인 법칙으로 계산 → 로봇 폭(220mm+여유) 이상이면 통과 가능

[탈출 회전 - 최적 방향으로 동적 회전]
  360도 스캔에서 가장 넓은 열린 섹터 방향으로 전진 가능할 때까지 회전
  Arduino에 "ESC\n" 전송 → 헤딩 리셋 + 헤딩가드 일시 비활성

[통신]
  RPi → Arduino: "v w\n" / "ESC\n"
  Arduino → RPi: "H:XX.X\n"

[방향 규칙 (LiDAR 기준)]
  양수 각도 = 오른쪽,  음수 각도 = 왼쪽
  양수 w    = 왼쪽 회전,  음수 w = 오른쪽 회전
  양수 heading = 왼쪽으로 틀어진 상태
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
LIDAR_OFFSET = 20      # mm: 라이다 측정값이 실제보다 20mm 짧으므로 보정

# ── 로봇 파라미터 ─────────────────────────────────────────────────────────────
ROBOT_HALF_WIDTH = 110   # 라이다 중심 ~ 좌우 끝 (mm)
ROBOT_FRONT_DIST = 120   # 라이다 중심 ~ 정면 끝 (mm)
SAFETY_MARGIN    = 10    # 수평 안전 여유 → threshold = 120mm

# ── 위험구역 ──────────────────────────────────────────────────────────────────
DETECTION_RANGE  = 1500  # mm: LiDAR 최대 신뢰 거리
FORWARD_RANGE    = 800   # mm: 위험구역 전방 깊이

# ── 속도 파라미터 ─────────────────────────────────────────────────────────────
FORWARD_SPEED    = 0.35  # m/s: 최고 선속도
MIN_SPEED        = 0.07  # m/s: 최소 선속도 (완전 정지 방지)
SLOW_START_DIST  = 250   # mm: 이 전방거리부터 감속 시작
STOP_FWD_RANGE   = 125   # mm: v=0 구역 전방 깊이
STOP_HORIZ_RANGE = 110   # mm: v=0 구역 수평 폭
W_GAIN           = 1.2
MAX_W            = 1.5
W_MIN_DANGER     = 0.5   # rad/s: 위험구역 최소 회전
W_SMOOTH         = 0.6
SIDE_ROTATE_SAFE = 150   # mm: 측면 장애물 수평거리가 이 미만이면 해당 방향 회전 금지
SIDE_CHECK_ANGLE = 60    # deg: 측면 확인 각도 범위 (±60°)

# ── 헤딩 방향 점수제 ──────────────────────────────────────────────────────────
HEADING_WEIGHT   = 1.0   # 헤딩 1° = 여유공간 1.0° 가중치
MIN_VIABLE_CLEAR = 25    # deg: 이 미만이면 해당 방향 진입 불가로 판단

# ── 헤딩 > 90° 능동 복귀 ─────────────────────────────────────────────────────
RECOVERY_W = 0.8

# ── 반대방향 감지 및 방향 보정 ────────────────────────────────────────────────
# Arduino 하드리밋(±90°)보다 낮게 설정 → 소프트 보정이 먼저 개입
MISSION_HEADING_LIMIT = 70.0   # deg: 이 범위(±) 초과 시 제자리 회전으로 복귀

# ── 막힘 감지 ─────────────────────────────────────────────────────────────────
STUCK_CLEAR_DIST    = 400   # mm
STUCK_MAX_SAFETY    = 30    # mm
STUCK_TRIGGER_COUNT = 3     # 회: 이 횟수 연속 막힘 판정 시 탈출 실행

# ── 탈출 회전 ─────────────────────────────────────────────────────────────────
ESCAPE_CLEAR_DIST    = 500
ESCAPE_W             = 1.0
ESCAPE_TIMEOUT       = 15.0
ESCAPE_ROTATION_SAFE = 310
MAX_ESCAPE_ANGLE     = 120   # deg: 최대 탈출 회전 각도 (180° 반전 직진 방지)
BACKUP_SPEED         = 0.10
BACKUP_DURATION      = 0.6

# ── 스캔 파라미터 ─────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE  = 90
ANGLE_STEP       = 5
SEND_INTERVAL    = 0.1
# ─────────────────────────────────────────────────────────────────────────────

arduino_heading_deg = 0.0
stuck_count         = 0
prev_w              = 0.0
avoidance_w_sign    = 0.0
no_danger_count     = 0     # 연속 장애물 없음 횟수 (avoidance_w_sign 리셋 hysteresis)


def normalize_angle(angle):
    """
    RPLIDAR 원시 각도(0~360°, CW 기준) → -180~+180°
    RPLIDAR C1은 CW(시계방향) 스캔 → 양수=오른쪽, 음수=왼쪽
    """
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
    quality     = data[0] >> 2
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    angle       = angle_q6 / 64.0
    distance_q2 = data[3] | (data[4] << 8)
    distance    = distance_q2 / 4.0
    return angle, distance, quality


def decompose(angle_norm_deg, distance_mm):
    rad   = math.radians(angle_norm_deg)
    horiz = abs(distance_mm * math.sin(rad))
    fwd   = distance_mm * math.cos(rad)
    return horiz, fwd


def read_arduino(arduino):
    """아두이노 헤딩 데이터 수신 (non-blocking)

    Arduino 헤딩 부호 규칙: 왼쪽 회전 → 양수
    RPi 내부 규칙: 왼쪽 회전 → 음수 (부호 반전하여 저장)
      → heading_deg > 0 = 오른쪽으로 틀어짐
      → heading_deg < 0 = 왼쪽으로 틀어짐
    """
    global arduino_heading_deg
    msg = None
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'):
                arduino_heading_deg = -float(line[2:])   # 부호 반전
            elif line:
                msg = line
        except Exception:
            pass
    return msg


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 막힘 감지: LiDAR 공간 기반
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def is_path_blocked(front_scan_points):
    """
    정면 180도 스캔에서 로봇이 물리적으로 통과 가능한 열린 구간이 있는지 확인

    [계산 방법 — 코사인 법칙]
      두 경계 장애물 사이의 실제 너비:
        w = √(d_L² + d_R² − 2·d_L·d_R·cos(θ))   θ = 경계 사이 각도

      min_gap = ROBOT_HALF_WIDTH × 2 + safety  (거리 비례 안전 여유)

    Returns: True(막힘) / False(통과 가능)
    """
    scan_dict = {}
    for angle_norm, dist in front_scan_points:
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    angles  = list(range(-SCAN_HALF_ANGLE, SCAN_HALF_ANGLE + ANGLE_STEP, ANGLE_STEP))
    in_open = False
    l_angle = None
    l_dist  = None

    for idx, a in enumerate(angles):
        d       = scan_dict.get(a, 0)
        is_open = (d >= STUCK_CLEAR_DIST)

        if not in_open:
            if is_open:
                in_open = True
                if idx > 0:
                    prev_a  = angles[idx - 1]
                    l_angle = prev_a
                    l_dist  = scan_dict.get(prev_a, 1) or 1
                else:
                    l_angle = a - ANGLE_STEP
                    l_dist  = STUCK_CLEAR_DIST
        else:
            if not is_open:
                r_angle = a
                r_dist  = d or 1
                theta   = math.radians(r_angle - l_angle)
                if theta > 0 and l_dist > 0:
                    w = math.sqrt(l_dist**2 + r_dist**2
                                  - 2 * l_dist * r_dist * math.cos(theta))
                    d_ref   = min(l_dist, r_dist)
                    safety  = STUCK_MAX_SAFETY * min(d_ref / STUCK_CLEAR_DIST, 1.0)
                    min_gap = ROBOT_HALF_WIDTH * 2 + safety
                    print(f"  [열린구간] {l_angle}°~{r_angle-ANGLE_STEP}°  "
                          f"d_L={l_dist:.0f} d_R={r_dist:.0f} "
                          f"너비={w:.0f}mm 기준={min_gap:.0f}mm "
                          + ("✓통과가능" if w >= min_gap else "✗협소"))
                    if w >= min_gap:
                        return False
                in_open = False

    if in_open and l_dist:
        r_angle = SCAN_HALF_ANGLE + ANGLE_STEP
        r_dist  = STUCK_CLEAR_DIST
        theta   = math.radians(r_angle - l_angle)
        if theta > 0:
            w = math.sqrt(l_dist**2 + r_dist**2
                          - 2 * l_dist * r_dist * math.cos(theta))
            d_ref   = min(l_dist, r_dist)
            safety  = STUCK_MAX_SAFETY * min(d_ref / STUCK_CLEAR_DIST, 1.0)
            min_gap = ROBOT_HALF_WIDTH * 2 + safety
            print(f"  [열린구간끝] {l_angle}°~{SCAN_HALF_ANGLE}°  "
                  f"d_L={l_dist:.0f} 너비≈{w:.0f}mm 기준={min_gap:.0f}mm "
                  + ("✓통과가능" if w >= min_gap else "✗협소"))
            if w >= min_gap:
                return False

    return True


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 탈출 방향 계산: 가장 넓은 열린 섹터 중심
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def find_escape_angle(all_scan_points):
    """
    360도 스캔에서 가장 넓은 열린 섹터를 찾아 그 중심 방향 반환

    Returns: target_angle_deg (양수=오른쪽, 음수=왼쪽)
    """
    scan_dict = {}
    for angle_norm, dist in all_scan_points:
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    all_angles = list(range(-180, 180, ANGLE_STEP))
    n = len(all_angles)

    open_flags = [
        scan_dict.get(a, 0) >= ESCAPE_CLEAR_DIST
        for a in all_angles
    ]

    best_len   = 0
    best_start = 0
    for start in range(n):
        length = 0
        for i in range(n):
            if open_flags[(start + i) % n]:
                length += 1
            else:
                break
        if length > best_len:
            best_len   = length
            best_start = start

    if best_len == 0:
        print("  [탈출] 열린 공간 없음 → 90° 회전")
        return 90.0

    center_idx   = (best_start + best_len // 2) % n
    target_angle = all_angles[center_idx]

    print(f"  [탈출방향] 최대 열린 섹터 {best_len * ANGLE_STEP}°  "
          f"→ 목표각도 {target_angle}°")

    # MAX_ESCAPE_ANGLE 범위 초과 시 범위 내 최적 각도로 대체
    if abs(target_angle) > MAX_ESCAPE_ANGLE:
        print(f"  [탈출방향 제한] {target_angle}° > {MAX_ESCAPE_ANGLE}° "
              f"→ ±{MAX_ESCAPE_ANGLE}° 이내 최적 방향 탐색")

        limited_angles = [a for a in all_angles if abs(a) <= MAX_ESCAPE_ANGLE]
        limited_open   = [open_flags[all_angles.index(a)] for a in limited_angles]

        best_l_len   = 0
        best_l_start = 0
        for start in range(len(limited_angles)):
            length = 0
            for i in range(len(limited_angles)):
                if limited_open[(start + i) % len(limited_angles)]:
                    length += 1
                else:
                    break
            if length > best_l_len:
                best_l_len   = length
                best_l_start = start

        if best_l_len > 0:
            c_idx        = (best_l_start + best_l_len // 2) % len(limited_angles)
            target_angle = limited_angles[c_idx]
            print(f"  [탈출방향 제한] 범위 내 최적: {target_angle}°")
        else:
            target_angle = MAX_ESCAPE_ANGLE * (1 if target_angle > 0 else -1)
            print(f"  [탈출방향 제한] 열린 공간 없음 → {target_angle}° 클램프")

    return float(target_angle)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 탈출 회전 실행
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def collect_scan_during_rotation(arduino, lidar, duration=0.12):
    """회전 중 LiDAR 스캔 수집 + 헤딩 업데이트 (비블로킹)"""
    scan_buf = []
    t = time.time()
    while time.time() - t < duration:
        read_arduino(arduino)
        while lidar.in_waiting >= 5:
            raw = lidar.read(5)
            result = parse_packet(raw)
            if result:
                a, d, _ = result
                if d > 0:
                    scan_buf.append((normalize_angle(a), d + LIDAR_OFFSET))
    return scan_buf


def check_rotation_blocked(w_sign, scan_points):
    """
    회전 방향 측면에 ESCAPE_ROTATION_SAFE 이내 장애물 있는지 확인

    양수=오른쪽, 음수=왼쪽 (RPLIDAR C1 CW 기준)
    음수 w = 오른쪽 회전 → 오른쪽 반구(양수 각도) 확인
    양수 w = 왼쪽 회전  → 왼쪽 반구(음수 각도) 확인
    """
    scan_dict = {}
    for angle_norm, dist in scan_points:
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    if w_sign < 0:   # 오른쪽 회전 → 오른쪽 근거리(양수 각도) 확인
        check = range(ANGLE_STEP, 61, ANGLE_STEP)
    else:            # 왼쪽 회전 → 왼쪽 근거리(음수 각도) 확인
        check = range(-ANGLE_STEP, -61, -ANGLE_STEP)

    return any(
        0 < scan_dict.get(a, DETECTION_RANGE + 1) < ESCAPE_ROTATION_SAFE
        for a in check
    )


def execute_escape_rotation(arduino, lidar, all_scan_points):
    """
    막힘 탈출: 동적 후진 + 전진 가능 방향까지 회전

    [초기 방향 결정 — 3단계 우선순위]
      1차: 헤딩 반대 방향 (헤딩>0 → 오른쪽(w<0), 헤딩<0 → 왼쪽(w>0))
      2차: 1차 막힘 → 후진하면서 그 방향이 열릴 때까지 대기
      3차: 최대 후진 후에도 안 열리면 반대 방향 시도 → 스캔 기반
    """
    global arduino_heading_deg, stuck_count, prev_w

    print("\n" + "="*52)
    heading_deg    = arduino_heading_deg
    BACKUP_MAX_TIME = 3.0
    HEADING_HINT_MIN = 5.0
    print(f"[ESCAPE] 전진 불가  헤딩:{heading_deg:.1f}°")

    # 초기 방향 결정
    # heading > 0 (왼쪽으로 틀어짐) → 오른쪽(w<0)으로 복귀
    if abs(heading_deg) >= HEADING_HINT_MIN:
        w_sign = -1.0 if heading_deg > 0 else 1.0
        hint   = "헤딩 반대"
    else:
        t      = find_escape_angle(all_scan_points)
        w_sign = -1.0 if t >= 0 else 1.0   # 양수 각도(오른쪽) → 오른쪽 회전(w<0)
        hint   = "스캔 기반(헤딩≈0°)"

    print(f"  [{hint}] 목표 방향: {'오른쪽' if w_sign<0 else '왼쪽'}")

    def backup_until_clear(target_sign):
        t_start = time.time()
        while time.time() - t_start < BACKUP_MAX_TIME:
            arduino.write(f"{-BACKUP_SPEED:.2f} 0.00\n".encode())
            scan_buf = collect_scan_during_rotation(arduino, lidar, duration=0.1)
            if scan_buf and not check_rotation_blocked(target_sign, scan_buf):
                arduino.write(b"0.00 0.00\n")
                time.sleep(0.05)
                return True
        arduino.write(b"0.00 0.00\n")
        return False

    if check_rotation_blocked(w_sign, all_scan_points):
        dir_str = "오른쪽" if w_sign < 0 else "왼쪽"
        print(f"  [{dir_str}] 막힘 → 열릴 때까지 후진 중...")
        cleared = backup_until_clear(w_sign)

        if not cleared:
            w_sign  = -w_sign
            dir_str = "오른쪽" if w_sign < 0 else "왼쪽"
            print(f"  최대 후진 후에도 막힘 → [{dir_str}] 시도")

            if check_rotation_blocked(w_sign, all_scan_points):
                print(f"  [{dir_str}]도 막힘 → 후진 시도")
                cleared = backup_until_clear(w_sign)
                if not cleared:
                    t      = find_escape_angle(all_scan_points)
                    w_sign = -1.0 if t >= 0 else 1.0
                    print("  스캔 기반(3차)으로 결정")
    else:
        print(f"  방향 열림 → 최소 후진 ({BACKUP_DURATION}초)")
        t_backup = time.time()
        while time.time() - t_backup < BACKUP_DURATION:
            arduino.write(f"{-BACKUP_SPEED:.2f} 0.00\n".encode())
            time.sleep(0.05)
        arduino.write(b"0.00 0.00\n")
        time.sleep(0.05)

    print(f"  최종 방향: {'왼쪽' if w_sign>0 else '오른쪽'}")

    arduino.write(b"ESC\n")
    time.sleep(0.15)

    MAX_ROT = 350
    t_start = time.time()

    while time.time() - t_start < ESCAPE_TIMEOUT:
        scan_buf  = collect_scan_during_rotation(arduino, lidar, duration=0.12)
        front_pts = [(a, d) for a, d in scan_buf if is_in_front(a) and d > 0]

        if front_pts and not is_path_blocked(front_pts):
            print(f"  [탈출완료] 전진 가능 발견 "
                  f"(회전량:{abs(arduino_heading_deg):.1f}°)")
            break

        if abs(arduino_heading_deg) > MAX_ROT:
            print("  [탈출] 350° 탐색 후 경로 없음 → 종료")
            break

        if scan_buf and check_rotation_blocked(w_sign, scan_buf):
            alt = -w_sign
            if not check_rotation_blocked(alt, scan_buf):
                print(f"  [탈출] 장애물 → {'왼쪽' if alt>0 else '오른쪽'} 전환")
                w_sign = alt
            else:
                arduino.write(b"0.00 0.00\n")
                time.sleep(0.1)
                continue

        arduino.write(f"0.00 {w_sign * ESCAPE_W:.2f}\n".encode())

    else:
        print("  [탈출] 타임아웃")

    arduino.write(b"0.00 0.00\n")
    time.sleep(0.3)
    stuck_count = 0   # 탈출 완료 후 카운터 리셋
    prev_w      = 0.0
    print("="*52 + "\n")


def execute_direction_correction(arduino, lidar, all_scan_points):
    """
    반대방향 감지 시 제자리 회전으로 헤딩 복귀

    |헤딩| > MISSION_HEADING_LIMIT(70°) 일 때 호출
    Arduino 하드리밋(90°) 전에 소프트하게 개입
    """
    global arduino_heading_deg, prev_w

    heading_deg = arduino_heading_deg
    print("\n" + "="*52)
    print(f"[방향보정] 헤딩:{heading_deg:.1f}° → "
          f"±{MISSION_HEADING_LIMIT}° 이내로 복귀")

    # RPi 내부 규칙: heading_deg > 0 = 오른쪽으로 틀어짐 → 왼쪽(w>0)으로 복귀
    #               heading_deg < 0 = 왼쪽으로 틀어짐  → 오른쪽(w<0)으로 복귀
    w_sign = 1.0 if heading_deg > 0 else -1.0

    if check_rotation_blocked(w_sign, all_scan_points):
        w_sign = -w_sign
        if check_rotation_blocked(w_sign, all_scan_points):
            print("  양쪽 막힘 → 방향보정 불가, 정상 회피로 복귀")
            print("="*52 + "\n")
            return

    print(f"  회전 방향: {'왼쪽' if w_sign>0 else '오른쪽'}")

    t_start = time.time()
    while time.time() - t_start < ESCAPE_TIMEOUT:
        scan_buf = collect_scan_during_rotation(arduino, lidar, duration=0.12)

        if abs(arduino_heading_deg) <= MISSION_HEADING_LIMIT:
            print(f"  [방향보정 완료] 헤딩:{arduino_heading_deg:.1f}°")
            break

        if scan_buf and check_rotation_blocked(w_sign, scan_buf):
            alt = -w_sign
            if not check_rotation_blocked(alt, scan_buf):
                print(f"  [방향보정] 장애물 → {'왼쪽' if alt>0 else '오른쪽'} 전환")
                w_sign = alt
            else:
                arduino.write(b"0.00 0.00\n")
                time.sleep(0.1)
                continue

        arduino.write(f"0.00 {w_sign * RECOVERY_W:.2f}\n".encode())

    else:
        print("  [방향보정] 타임아웃")

    arduino.write(b"0.00 0.00\n")
    time.sleep(0.3)
    prev_w = 0.0
    print("="*52 + "\n")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 회피 방향 결정 (점수제)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def select_direction(left_clear, right_clear, heading_deg):
    """
    여유공간 + 헤딩 보정 점수로 회피 방향 결정

    RPi 내부 규칙 (read_arduino에서 부호 반전):
      heading_deg > 0 = 오른쪽으로 틀어짐 → 왼쪽 보너스 → 중심 복귀
      heading_deg < 0 = 왼쪽으로 틀어짐  → 오른쪽 보너스 → 중심 복귀

    [갇힘 방지]
    한쪽이 MIN_VIABLE_CLEAR(25°) 미만이면 열린 쪽 강제 선택
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

    # 양쪽 모두 통과 가능 → 점수제
    # RPi 내부 규칙: heading_deg > 0 = 오른쪽으로 틀어짐 → 왼쪽에 보너스 → 중심 복귀
    #               heading_deg < 0 = 왼쪽으로 틀어짐  → 오른쪽에 보너스 → 중심 복귀
    left_score  = left_clear  + max(0.0,  heading_deg) * HEADING_WEIGHT
    right_score = right_clear + max(0.0, -heading_deg) * HEADING_WEIGHT

    bonus_side = "L" if heading_deg > 0 else "R"
    bonus_val  = abs(heading_deg) * HEADING_WEIGHT
    print(f"  [방향점수] L={left_score:.0f}  R={right_score:.0f}"
          f"  (여유 L={left_clear}° R={right_clear}°"
          f"  헤딩보너스 {bonus_side}+{bonus_val:.0f})")

    return 1.0 if left_score >= right_score else -1.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# v/w 명령 계산
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def find_vw_command(scan_points, heading_deg):
    """
    정면 스캔 + 헤딩 → (v m/s, w rad/s) 반환

    [방향 규칙]
      양수 각도 = 오른쪽 장애물 → w 양수(왼쪽)로 회피
      음수 각도 = 왼쪽 장애물  → w 음수(오른쪽)로 회피
    """
    global avoidance_w_sign, no_danger_count
    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN

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

    stop_points = [p for p in danger_points
                   if p[3] <= STOP_FWD_RANGE and p[2] <= STOP_HORIZ_RANGE]
    frontal   = [p for p in danger_points if p[3] >= p[2]]
    n_fwd_ref = min((p[3] for p in frontal), default=SLOW_START_DIST + 1)
    horiz_ref = min(danger_points, key=lambda p: p[2])
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
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    if stop_points:
        # 정지구역: 장애물 각도로 직접 결정
        # 양수 각도(오른쪽 장애물) → 왼쪽(+1) 회피
        # 음수 각도(왼쪽 장애물)  → 오른쪽(-1) 회피
        stop_angle = min(stop_points, key=lambda p: p[2])[0]
        avoidance_w_sign = 1.0 if stop_angle >= 0 else -1.0
        print(f"  [정지구역] 각도:{stop_angle:.1f}° → "
              f"{'왼쪽' if avoidance_w_sign>0 else '오른쪽'} 직접 결정")
    else:
        # 위험구역: 여유공간 점수제 + hysteresis
        # 음수 각도 구간 = 왼쪽 공간, 양수 각도 구간 = 오른쪽 공간
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
        print("  [측면차단] 왼쪽 근접 → 오른쪽 강제")
        avoidance_w_sign = -1.0
    elif avoidance_w_sign < 0 and right_close and not left_close:
        print("  [측면차단] 오른쪽 근접 → 왼쪽 강제")
        avoidance_w_sign = 1.0

    w_mag = max(min(W_GAIN * horiz_error / threshold, MAX_W), W_MIN_DANGER)
    w     = avoidance_w_sign * w_mag

    print(f"  [명령] v:{v:.2f}  w:{w:.2f}  (수평오차:{horiz_error:.0f}mm)")
    return v, w


def main():
    global arduino_heading_deg, stuck_count, prev_w, avoidance_w_sign, no_danger_count

    print("=== RPLIDAR 장애물 회피 v6 ===")
    print(f"  라이다 포트    : {LIDAR_PORT}")
    print(f"  아두이노 포트  : {ARDUINO_PORT}")
    print(f"  라이다 보정    : +{LIDAR_OFFSET}mm  (CCW→CW 방향 반전 적용)")
    print(f"  위험구역       : 전방 {FORWARD_RANGE}mm × 수평 {ROBOT_HALF_WIDTH+SAFETY_MARGIN}mm")
    print(f"  속도           : 최고 {FORWARD_SPEED}m/s  최저 {MIN_SPEED}m/s")
    print(f"  감속 시작      : {SLOW_START_DIST}mm → {STOP_FWD_RANGE}×{STOP_HORIZ_RANGE}mm 구역 최저속")
    print(f"  방향보정 임계  : ±{MISSION_HEADING_LIMIT}° (Arduino 하드리밋 ±90° 전 개입)")
    print(f"  막힘감지       : 전방 통과불가 즉시 탈출 회전")
    print(f"  탈출 각속도    : {ESCAPE_W} rad/s")
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

            angle_raw, distance, quality = result
            s_flag = raw[0] & 0x01

            if s_flag == 1 and scan_points:
                all_scan_points = list(scan_points)
                front_points    = [
                    (a, d) for a, d in scan_points
                    if is_in_front(a) and d > 0
                ]

                now = time.time()
                if now - last_send >= SEND_INTERVAL:

                    # ── ① 막힘 감지 → 탈출 ──────────────────────────────────
                    if is_path_blocked(front_points):
                        stuck_count += 1
                        print(f"  [막힘감지] {stuck_count}/{STUCK_TRIGGER_COUNT}회")
                        if stuck_count >= STUCK_TRIGGER_COUNT:
                            execute_escape_rotation(arduino, lidar, all_scan_points)
                            # stuck_count는 execute_escape_rotation 내부에서 리셋
                            avoidance_w_sign = 0.0
                            last_cmd_str     = ""
                            last_send        = time.time()
                            scan_points      = []
                            continue
                    else:
                        if stuck_count > 0:
                            print(f"  [막힘해제] 카운터 리셋 ({stuck_count}회)")
                        stuck_count = 0

                    # ── ② 반대방향 감지 → 헤딩 보정 ────────────────────────
                    if abs(arduino_heading_deg) > MISSION_HEADING_LIMIT:
                        execute_direction_correction(
                            arduino, lidar, all_scan_points
                        )
                        avoidance_w_sign = 0.0
                        last_cmd_str     = ""
                        last_send        = time.time()
                        scan_points      = []
                        continue

                    # ── ③ 정상 v/w 명령 ─────────────────────────────────────
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

            scan_points.append((normalize_angle(angle_raw),
                                distance + LIDAR_OFFSET if distance > 0 else 0))

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

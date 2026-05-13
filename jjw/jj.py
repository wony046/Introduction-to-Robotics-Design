"""
RPLIDAR C1 장애물 회피 코드 (v5)
포트: 라이다 /dev/ttyUSB0 / 아두이노 /dev/ttyS0 (UART GPIO14/15)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[막힘 감지 - LiDAR 공간 기반]

  이전: |헤딩| > 60° + 4초 → 오판 가능성 높음
  변경: 정면 180° 스캔에서 통과 가능한 경로가 없을 때

  판단 방법:
    각 방향에서 STUCK_CLEAR_DIST(400mm) 이상 → "열림"
    연속으로 STUCK_CLEAR_ANGLE(30°) 이상 열린 구간 → 통과 가능
    그런 구간이 하나도 없으면 → 전진 불가 → 막힘

  단점 없이 장점:
    - 헤딩과 무관하게 실제 공간으로 판단
    - 코너 회전 중 오발동 없음
    - 반응 시간 단축 (STUCK_TIMEOUT 2초)

[탈출 회전 - 최적 방향으로 최소 각도 회전]

  이전: 항상 180도 고정 회전 (T180L/R)
  변경: 360도 스캔에서 가장 넓은 열린 섹터를 찾아 그 방향으로 회전

  탈출 각도 계산:
    전체 360도를 ANGLE_STEP 간격으로 순환 탐색
    ESCAPE_CLEAR_DIST(500mm) 이상인 연속 구간 중 가장 넓은 섹터 선택
    그 섹터 중심 방향이 목표 회전 각도

  실행:
    Arduino에 "ESC\n" 전송 → 헤딩 리셋 + 헤딩가드 일시 비활성
    v=0, w=±ESCAPE_W 전송
    Arduino 헤딩 피드백으로 목표 도달 확인 → 정지

[통신]
  RPi → Arduino: "v w\n" / "ESC\n"
  Arduino → RPi: "H:XX.X\n"
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
FORWARD_SPEED    = 0.35  # m/s: 최고 선속도 (기존 0.20에서 상향)
MIN_SPEED        = 0.07  # m/s: 최소 선속도 (완전 정지 방지)
SLOW_START_DIST  = 250   # mm: 이 전방거리부터 감속 시작 (기존 400에서 단축)
STOP_FWD_RANGE   = 125   # mm: v=0 구역 전방 깊이
STOP_HORIZ_RANGE = 110   # mm: v=0 구역 수평 폭
STOP_BACKUP_TIME = 0.3   # sec: 위험구역 진입 시 후진 시간 (약 15mm)
W_GAIN           = 1.2
MAX_W            = 1.5
W_MIN_DANGER     = 0.5   # rad/s: 위험구역 최소 회전 (horiz_error 작아도 충분히 회전)
W_SMOOTH         = 0.6
SIDE_ROTATE_SAFE = 150   # mm: 측면 장애물 수평거리가 이 미만이면 해당 방향 회전 금지
                          #     ROBOT_HALF_WIDTH(110mm) + 여유(40mm)
SIDE_CHECK_ANGLE = 60    # deg: 측면 확인 각도 범위 (±60°)

# ── 헤딩 방향 점수제 ──────────────────────────────────────────────────────────
HEADING_WEIGHT   = 1.0   # 헤딩 1° = 여유공간 1.0° 가중치 (기존 1.5에서 낮춤)
MIN_VIABLE_CLEAR = 25    # deg: 이 미만이면 해당 방향 진입 불가로 판단
                          #      헤딩 보너스 무시하고 반대 방향 강제 → oscillation 방지

# ── 헤딩 > 90° 능동 복귀 ─────────────────────────────────────────────────────
HEADING_OVER_90      = 90.0
RECOVERY_W           = 0.8
RECOVERY_SAFE_DIST   = 350

# ── 반대방향 감지 및 방향 보정 ────────────────────────────────────────────────
MISSION_HEADING_LIMIT = 90.0  # deg: 이 범위(±) 초과 시 제자리 회전으로 복귀
                               #      조정 가능: 좁은 공간이면 낮게, 여유있으면 높게

# ── 막힘 감지 ─────────────────────────────────────────────────────────────────
STUCK_CLEAR_DIST    = 400   # mm
STUCK_MAX_SAFETY    = 30    # mm
STUCK_TRIGGER_COUNT = 3     # 회: 이 횟수 연속 막힘 판정 시 탈출 실행
                             #     정상 회피 중 단발 막힘 오판 방지

# ── 탈출 회전 ─────────────────────────────────────────────────────────────────
ESCAPE_CLEAR_DIST    = 500
ESCAPE_W             = 1.0
ESCAPE_TIMEOUT       = 15.0
ESCAPE_TOLERANCE     = 8.0
ESCAPE_ROTATION_SAFE = 310
ESCAPE_EXTRA_ANGLE   = 5     # deg: 여유 회전 (기존 15° → 5°로 축소)
MAX_ESCAPE_ANGLE     = 120   # deg: 최대 탈출 회전 각도 (180° 반전 직진 방지)
BACKUP_SPEED         = 0.10
BACKUP_DURATION      = 0.6

# ── 스캔 파라미터 ─────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE  = 90
ANGLE_STEP       = 5
SEND_INTERVAL    = 0.1
# ─────────────────────────────────────────────────────────────────────────────

arduino_heading_deg  = 0.0
stuck_count          = 0
prev_w               = 0.0
avoidance_w_sign     = 0.0
no_danger_count      = 0     # 연속 장애물 없음 횟수 (avoidance_w_sign 리셋 hysteresis)
stop_zone_entry_time = None


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
    quality     = data[0] >> 2
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    angle       = angle_q6 / 64.0
    distance_q2 = data[3] | (data[4] << 8)
    distance    = distance_q2 / 4.0   # 보정은 유효값 확인 후 적용
    return angle, distance, quality


def decompose(angle_norm_deg, distance_mm):
    rad   = math.radians(angle_norm_deg)
    horiz = abs(distance_mm * math.sin(rad))
    fwd   = distance_mm * math.cos(rad)
    return horiz, fwd


def read_arduino(arduino):
    """아두이노 헤딩 데이터 수신 (non-blocking)"""
    global arduino_heading_deg
    msg = None
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'):
                arduino_heading_deg = float(line[2:])
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

      라이다(원점), 열린 구간 왼쪽 경계 장애물(d_L, α), 오른쪽 경계 장애물(d_R, β)
      두 장애물 사이의 실제 거리(열린 구간 너비):

        w = √(d_L² + d_R² − 2·d_L·d_R·cos(θ))   θ = β − α

      [너비 판정 — 거리 비례 안전 여유]
      d_ref     = min(d_L, d_R)           (두 경계 중 더 가까운 쪽)
      safety    = STUCK_MAX_SAFETY × min(d_ref / STUCK_CLEAR_DIST, 1.0)
      min_gap   = ROBOT_HALF_WIDTH × 2 + safety

      d_ref=0   → safety=0mm  → min_gap=220mm (로봇 폭 그대로, 코앞이면 통과 시도)
      d_ref=400 → safety=30mm → min_gap=250mm (멀리 있으면 여유 있게 판단)

    [경계 처리]
      - 스캔 시작(-90°)이 열린 구간이면: 왼쪽 경계를 STUCK_CLEAR_DIST로 추정
      - 스캔 끝(+90°)까지 열린 구간이면: 오른쪽 경계를 STUCK_CLEAR_DIST로 추정
      → 보수적으로 처리 (실제보다 좁게 추정)

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
    l_angle = None   # 열린 구간 왼쪽 경계 장애물 각도
    l_dist  = None   # 열린 구간 왼쪽 경계 장애물 거리

    for idx, a in enumerate(angles):
        d       = scan_dict.get(a, 0)
        is_open = (d >= STUCK_CLEAR_DIST)

        if not in_open:
            if is_open:
                # 열린 구간 시작 → 직전 버킷이 왼쪽 경계 장애물
                in_open = True
                if idx > 0:
                    prev_a  = angles[idx - 1]
                    l_angle = prev_a
                    l_dist  = scan_dict.get(prev_a, 1) or 1
                else:
                    # 스캔 첫 번째부터 열림 → 왼쪽 경계를 보수적으로 추정
                    l_angle = a - ANGLE_STEP
                    l_dist  = STUCK_CLEAR_DIST

        else:  # in_open
            if not is_open:
                # 열린 구간 종료 → 현재 버킷이 오른쪽 경계 장애물
                r_angle = a
                r_dist  = d or 1

                theta = math.radians(r_angle - l_angle)
                if theta > 0 and l_dist > 0:
                    w = math.sqrt(l_dist**2 + r_dist**2
                                  - 2 * l_dist * r_dist * math.cos(theta))
                    # 거리 비례 안전 여유: 경계가 가까울수록 여유 줄어듦
                    d_ref   = min(l_dist, r_dist)
                    safety  = STUCK_MAX_SAFETY * min(d_ref / STUCK_CLEAR_DIST, 1.0)
                    min_gap = ROBOT_HALF_WIDTH * 2 + safety
                    print(f"  [열린구간] {l_angle}°~{r_angle-ANGLE_STEP}°  "
                          f"d_L={l_dist:.0f} d_R={r_dist:.0f} "
                          f"너비={w:.0f}mm 기준={min_gap:.0f}mm "
                          + ("✓통과가능" if w >= min_gap else "✗협소"))
                    if w >= min_gap:
                        return False   # 통과 가능

                in_open = False

    # 스캔 끝까지 열린 구간인 경우
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

    return True   # 통과 가능한 구간 없음 → 막힘


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 탈출 방향 계산: 가장 넓은 열린 섹터 중심
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def find_escape_angle(all_scan_points):
    """
    360도 스캔에서 가장 넓은 열린 섹터를 찾아 그 중심 방향 반환

    [알고리즘]
    1. 전체 -180°~+180°를 ANGLE_STEP 간격으로 분할
    2. 각 버킷이 ESCAPE_CLEAR_DIST 이상이면 "열림"
    3. 순환 배열에서 가장 긴 연속 열린 구간 탐색
    4. 그 구간의 중심 각도를 목표 방향으로 반환

    Returns: target_angle_deg (현재 헤딩 0° 기준 상대 각도)
    """
    scan_dict = {}
    for angle_norm, dist in all_scan_points:
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    # 전체 360도 각도 목록 (-180 ~ +175)
    all_angles = list(range(-180, 180, ANGLE_STEP))
    n = len(all_angles)

    # 각 버킷의 열림 여부
    open_flags = [
        scan_dict.get(a, 0) >= ESCAPE_CLEAR_DIST
        for a in all_angles
    ]

    # 순환 배열에서 가장 긴 연속 열린 구간 탐색
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

    # ── MAX_ESCAPE_ANGLE 범위 초과 시 범위 내 최적 각도로 대체 ────────────────
    if abs(target_angle) > MAX_ESCAPE_ANGLE:
        print(f"  [탈출방향 제한] {target_angle}° > {MAX_ESCAPE_ANGLE}° "
              f"→ ±{MAX_ESCAPE_ANGLE}° 이내 최적 방향 탐색")

        # ±MAX_ESCAPE_ANGLE 범위 내에서 가장 넓은 열린 섹터 탐색
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
            # 범위 내 열린 공간 없음 → 그나마 가까운 각도로 클램프
            target_angle = MAX_ESCAPE_ANGLE * (1 if target_angle > 0 else -1)
            print(f"  [탈출방향 제한] 열린 공간 없음 → {target_angle}° 클램프")

    return float(target_angle)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 탈출 회전 실행
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def collect_scan_during_rotation(arduino, lidar, duration=0.12):
    """
    회전 중 LiDAR 스캔 수집 + 헤딩 업데이트 (비블로킹)
    duration: 수집 시간(초) ≈ LiDAR 한 바퀴(100ms)
    """
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

    LiDAR 각도 규칙: 양수(+) = 오른쪽,  음수(-) = 왼쪽
    오른쪽 회전(w<0) → 오른쪽 반구(양수 각도) 확인
    왼쪽  회전(w>0) → 왼쪽  반구(음수 각도) 확인
    """
    scan_dict = {}
    for angle_norm, dist in scan_points:
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    if w_sign < 0:   # 오른쪽 회전 → 오른쪽 근거리(+5°~+60°) 확인
        check = range(ANGLE_STEP, 61, ANGLE_STEP)
    else:            # 왼쪽 회전 → 왼쪽 근거리(-5°~-60°) 확인
        check = range(-ANGLE_STEP, -61, -ANGLE_STEP)

    return any(
        0 < scan_dict.get(a, DETECTION_RANGE + 1) < ESCAPE_ROTATION_SAFE
        for a in check
    )


def execute_escape_rotation(arduino, lidar, all_scan_points):
    """
    막힘 탈출: 동적 후진 + 전진 가능 방향까지 회전

    [초기 방향 결정 — 3단계 우선순위]
      1차: 헤딩 반대 방향 (헤딩>0 → 오른쪽, 헤딩<0 → 왼쪽)
      2차: 1차 막힘 → 후진하면서 그 방향이 열릴 때까지 대기
      3차: 최대 후진 후에도 안 열리면 반대 방향 → 그것도 막히면 스캔 기반

    [후진 방식]
      고정 시간이 아닌 동적 후진:
      회전 방향에 장애물 없어질 때까지 계속 후진
      (최대 BACKUP_MAX_TIME 초 제한)
    """
    global arduino_heading_deg, stuck_count, prev_w

    print("\n" + "="*52)
    heading_deg    = arduino_heading_deg
    BACKUP_MAX_TIME = 3.0    # sec: 최대 후진 시간
    HEADING_HINT_MIN = 5.0   # deg: 헤딩 힌트 사용 최소 기준
    print(f"[ESCAPE] 전진 불가  헤딩:{heading_deg:.1f}°")

    # ── 초기 방향 결정 ────────────────────────────────────────────────────────
    if abs(heading_deg) >= HEADING_HINT_MIN:
        w_sign  = -1.0 if heading_deg > 0 else 1.0    # 헤딩 반대 방향
        hint    = "헤딩 반대"
    else:
        t       = find_escape_angle(all_scan_points)
        w_sign  = -1.0 if t >= 0 else 1.0
        hint    = "스캔 기반(헤딩≈0°)"

    print(f"  [{hint}] 목표 방향: {'오른쪽' if w_sign<0 else '왼쪽'}")

    # ── 동적 후진: 회전 방향이 열릴 때까지 ───────────────────────────────────
    def backup_until_clear(target_sign):
        """
        target_sign 방향이 열릴 때까지 후진
        Returns: True(열림) / False(최대 시간 초과)
        """
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
            # 반대 방향 시도
            w_sign  = -w_sign
            dir_str = "오른쪽" if w_sign < 0 else "왼쪽"
            print(f"  최대 후진 후에도 막힘 → [{dir_str}] 시도")

            if check_rotation_blocked(w_sign, all_scan_points):
                print(f"  [{dir_str}]도 막힘 → 후진 시도")
                cleared = backup_until_clear(w_sign)
                if not cleared:
                    # 스캔 기반으로 최종 결정
                    t      = find_escape_angle(all_scan_points)
                    w_sign = -1.0 if t >= 0 else 1.0
                    print(f"  스캔 기반(3차)으로 결정")
    else:
        # 처음부터 열려있어도 최소 후진은 실행 (회전 공간 확보)
        print(f"  방향 열림 → 최소 후진 ({BACKUP_DURATION}초)")
        t_backup = time.time()
        while time.time() - t_backup < BACKUP_DURATION:
            arduino.write(f"{-BACKUP_SPEED:.2f} 0.00\n".encode())
            time.sleep(0.05)
        arduino.write(b"0.00 0.00\n")
        time.sleep(0.05)

    print(f"  최종 방향: {'왼쪽' if w_sign>0 else '오른쪽'}")

    # ── ESC 전송 ─────────────────────────────────────────────────────────────
    arduino.write(b"ESC\n")
    time.sleep(0.15)

    # ── 전진 가능 방향 나올 때까지 회전 ─────────────────────────────────────
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
    stuck_count = 0
    prev_w      = 0.0
    print("="*52 + "\n")


def execute_direction_correction(arduino, lidar, all_scan_points):
    """
    반대방향 감지 시 제자리 회전으로 헤딩 복귀

    |헤딩| > MISSION_HEADING_LIMIT 일 때 호출
    → LiDAR로 회전 가능 여부 확인 후 헤딩이 한계 이내가 될 때까지 회전
    → 회전 중 충돌 위험 감지 시 반대 방향 전환
    """
    global arduino_heading_deg, prev_w

    heading_deg = arduino_heading_deg
    print("\n" + "="*52)
    print(f"[방향보정] 헤딩:{heading_deg:.1f}° → "
          f"±{MISSION_HEADING_LIMIT}° 이내로 복귀")

    # 헤딩 감소 방향 (자연 방향)
    w_sign = -1.0 if heading_deg > 0 else 1.0

    # 회전 가능 여부 확인
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

        # 헤딩 복귀 완료 확인
        if abs(arduino_heading_deg) <= MISSION_HEADING_LIMIT:
            print(f"  [방향보정 완료] 헤딩:{arduino_heading_deg:.1f}°")
            break

        # 회전 중 장애물 확인
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
# 헤딩 > 90° 복귀 명령 결정 (v, w 동시 반환)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def get_heading_recovery_cmd(heading_deg, front_scan_points):
    """
    헤딩이 90°를 넘었을 때 0°로 복귀하는 v, w 결정

    [A안: 직진 + 보정 회전 동시 적용]
      직진(v=FORWARD_SPEED)하면서 헤딩 감소 방향으로 약한 w 추가
      → 부드러운 곡선으로 전진하며 자연스럽게 헤딩 복귀

    [판단 순서]
      1. 정면 ±45° 열려있는지 확인
      2. 헤딩 감소 방향(보정 방향)에 장애물 있는지 확인

      정면 열림 + 보정 안전 → v=전진,  w=보정회전  (A안 정상)
      정면 열림 + 보정 막힘 → v=전진,  w=0          (직진만, 기회 기다림)
      정면 막힘 + 보정 안전 → v=0,     w=보정회전   (제자리 회전 복귀)
      정면 막힘 + 보정 막힘 → v=0,     w=반대방향   (우회 회전)
    """
    natural_sign = -1.0 if heading_deg > 0 else 1.0  # 헤딩 감소 방향 부호

    scan_dict = {}
    for angle_norm, dist in front_scan_points:
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    # 정면 ±45° 열림 확인
    frontal_open = any(
        scan_dict.get(a, 0) >= RECOVERY_SAFE_DIST
        for a in range(-45, 50, ANGLE_STEP)
    )

    # 보정 방향 장애물 확인
    # 오른쪽 보정(natural_sign<0, w<0) → 오른쪽(양수 각도) 확인
    # 왼쪽  보정(natural_sign>0, w>0) → 왼쪽 (음수 각도) 확인
    if natural_sign < 0:  # 오른쪽 보정 → 오른쪽(양수) 반구
        correction_blocked = any(
            0 < scan_dict.get(a, DETECTION_RANGE + 1) < RECOVERY_SAFE_DIST
            for a in range(ANGLE_STEP, SCAN_HALF_ANGLE + ANGLE_STEP, ANGLE_STEP)
        )
    else:                 # 왼쪽 보정 → 왼쪽(음수) 반구
        correction_blocked = any(
            0 < scan_dict.get(a, DETECTION_RANGE + 1) < RECOVERY_SAFE_DIST
            for a in range(-ANGLE_STEP, -SCAN_HALF_ANGLE - ANGLE_STEP, -ANGLE_STEP)
        )

    corr_dir = "오른쪽" if natural_sign < 0 else "왼쪽"

    if frontal_open and not correction_blocked:
        print(f"  [헤딩복귀-A] 직진+보정회전({corr_dir})  "
              f"v={FORWARD_SPEED:.2f} w={natural_sign*RECOVERY_W:.2f}")
        return FORWARD_SPEED, natural_sign * RECOVERY_W

    elif frontal_open and correction_blocked:
        print(f"  [헤딩복귀] 직진만 (보정방향 {corr_dir} 막힘)")
        return FORWARD_SPEED, 0.0

    elif not frontal_open and not correction_blocked:
        print(f"  [헤딩복귀] 제자리 보정회전({corr_dir})  w={natural_sign*RECOVERY_W:.2f}")
        return 0.0, natural_sign * RECOVERY_W

    else:  # 정면 막힘 + 보정 막힘
        alt_dir = "왼쪽" if natural_sign < 0 else "오른쪽"
        print(f"  [헤딩복귀] 양쪽 막힘 → 우회({alt_dir}) 회전")
        return 0.0, -natural_sign * RECOVERY_W


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 회피 방향 결정 (점수제)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def select_direction(left_clear, right_clear, heading_deg):
    """
    여유공간 + 헤딩 보정 점수로 회피 방향 결정

    [갇힘 방지 — MIN_VIABLE_CLEAR]
      한쪽 여유공간이 MIN_VIABLE_CLEAR(25°) 미만이면 진입 불가로 판단
      헤딩 보너스 무시하고 열린 쪽 강제 선택
      → 헤딩 때문에 막힌 쪽으로 계속 꺾으려는 oscillation 방지

    [점수제 — 양쪽 모두 통과 가능할 때]
      left_score  = left_clear  + max(0, -heading_deg) × HEADING_WEIGHT
      right_score = right_clear + max(0,  heading_deg) × HEADING_WEIGHT
    """
    # 한쪽이 막혀있으면 열린 쪽 강제 (헤딩 무시)
    left_ok  = left_clear  >= MIN_VIABLE_CLEAR
    right_ok = right_clear >= MIN_VIABLE_CLEAR

    if left_ok and not right_ok:
        print(f"  [방향] 오른쪽 막힘({right_clear}°) → 왼쪽 강제")
        return 1.0
    if right_ok and not left_ok:
        print(f"  [방향] 왼쪽 막힘({left_clear}°) → 오른쪽 강제")
        return -1.0
    if not left_ok and not right_ok:
        # 양쪽 모두 좁음 → 그나마 넓은 쪽
        print(f"  [방향] 양쪽 협소 → {'왼쪽' if left_clear >= right_clear else '오른쪽'} 선택")
        return 1.0 if left_clear >= right_clear else -1.0

    # 양쪽 모두 통과 가능 → 점수제
    left_score  = left_clear  + max(0.0, -heading_deg) * HEADING_WEIGHT
    right_score = right_clear + max(0.0,  heading_deg) * HEADING_WEIGHT

    bonus_side = "R" if heading_deg > 0 else "L"
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

    # scan_dict 공통 계산
    scan_dict = {}
    for angle_norm, dist in scan_points:
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    # 방향 결정
    if stop_points:
        # Stop zone: 장애물 각도로 직접 결정 (충돌 후 오판 방지)
        # 양수각(오른쪽 장애물) → 왼쪽(+1), 음수각(왼쪽 장애물) → 오른쪽(-1)
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
        angles = (range(-ANGLE_STEP, -(SIDE_CHECK_ANGLE+ANGLE_STEP), -ANGLE_STEP)
                  if is_left else
                  range(ANGLE_STEP, SIDE_CHECK_ANGLE+ANGLE_STEP, ANGLE_STEP))
        for a in angles:
            d = scan_dict.get(a, 0)
            if d <= 0: continue
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


def main():
    global arduino_heading_deg, stuck_count, prev_w, avoidance_w_sign, stop_zone_entry_time, no_danger_count

    print("=== RPLIDAR 장애물 회피 v5 ===")
    print(f"  라이다 포트    : {LIDAR_PORT}")
    print(f"  아두이노 포트  : {ARDUINO_PORT}")
    print(f"  라이다 보정    : +{LIDAR_OFFSET}mm")
    print(f"  위험구역       : 전방 {FORWARD_RANGE}mm × 수평 {ROBOT_HALF_WIDTH+SAFETY_MARGIN}mm")
    print(f"  속도           : 최고 {FORWARD_SPEED}m/s  최저 {MIN_SPEED}m/s (완전정지 없음)")
    print(f"  선속도 감속    : {SLOW_START_DIST}mm부터 감속 → {STOP_FWD_RANGE}×{STOP_HORIZ_RANGE}mm 구역에서 최저속")
    print(f"  각속도 방식    : 수평오차 P제어 (horiz < {ROBOT_HALF_WIDTH+SAFETY_MARGIN}mm 동안 유지)")
    print(f"  막힘감지       : 전방 통과불가 즉시 탈출 회전")
    print(f"  탈출 각속도    : {ESCAPE_W} rad/s (최적 방향)")
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
                all_scan_points = list(scan_points)   # 전체 360도 스냅샷
                front_points    = [
                    (a, d) for a, d in scan_points
                    if is_in_front(a) and d > 0
                ]

                now = time.time()
                if now - last_send >= SEND_INTERVAL:

                    # ── ③ 정상 v/w 명령 ────────────────────────────────────
                    v, w = find_vw_command(front_points, arduino_heading_deg)

                    # 위험구역(stop zone) → v=0 제자리 회전
                    # 후진 제거: 45° 측면 장애물에 후진은 효과 없고
                    #            후진 중 danger_points 소멸로 avoidance_w_sign 리셋 야기
                    in_stop_zone = (v == 0.0 and abs(w) > 0.01)
                    if not in_stop_zone:
                        stop_zone_entry_time = None

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

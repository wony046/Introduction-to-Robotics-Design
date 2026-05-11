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
ARDUINO_PORT     = "/dev/ttyS0"
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
FORWARD_SPEED    = 0.20  # m/s: 최고 선속도
SLOW_START_DIST  = 400   # mm: 이 전방거리부터 감속 시작
STOP_DIST        = 120   # mm: 이 전방거리 이하에서 선속도=0 (회전만)
                          #     = ROBOT_FRONT_DIST (로봇 정면이 장애물에 닿는 거리)
W_GAIN           = 2.0   # 수평오차 P 게인: w = W_GAIN × (threshold-horiz)/threshold
MAX_W            = 2.0   # rad/s: 최대 각속도

# ── 헤딩 방향 점수제 ──────────────────────────────────────────────────────────
HEADING_WEIGHT   = 1.5   # 헤딩 1° = 여유공간 1.5° 가중치

# ── 헤딩 > 90° 능동 복귀 ─────────────────────────────────────────────────────
HEADING_OVER_90    = 90.0  # deg: 이 이상이면 LiDAR 확인 후 복귀 회전
RECOVERY_W         = 0.8   # rad/s: 복귀 회전 각속도
RECOVERY_SAFE_DIST = 350   # mm: 복귀 방향 장애물 판단 거리

# ── 막힘 감지 ─────────────────────────────────────────────────────────────────
STUCK_CLEAR_DIST = 400   # mm: 이 거리 이상이면 열린 공간으로 간주
STUCK_MAX_SAFETY = 30    # mm: 최대 안전 여유 (거리 비례 감소, d=0이면 0mm)
STUCK_TIMEOUT    = 2.0   # sec

# ── 탈출 회전 ─────────────────────────────────────────────────────────────────
ESCAPE_CLEAR_DIST = 500  # mm: 탈출 방향 판단 최소 거리
ESCAPE_W          = 1.0  # rad/s: 탈출 회전 각속도
ESCAPE_TIMEOUT    = 15.0 # sec: 탈출 회전 최대 시간
ESCAPE_TOLERANCE  = 8.0  # deg: 목표 각도 허용 오차

# ── 스캔 파라미터 ─────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE  = 90
ANGLE_STEP       = 5
SEND_INTERVAL    = 0.1
# ─────────────────────────────────────────────────────────────────────────────

arduino_heading_deg = 0.0
stuck_since         = None


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
    distance    = distance_q2 / 4.0 + LIDAR_OFFSET   # +20mm 보정
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
        r_dist  = STUCK_CLEAR_DIST   # 보수적 추정
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
        # 전 방향이 막힘 → 정반대(180°)로
        print("  [탈출] 열린 공간 없음 → 180° 회전")
        return 180.0

    # 가장 넓은 섹터의 중심 인덱스
    center_idx   = (best_start + best_len // 2) % n
    target_angle = all_angles[center_idx]

    print(f"  [탈출방향] 최대 열린 섹터 {best_len * ANGLE_STEP}°  "
          f"→ 목표각도 {target_angle}°")
    return float(target_angle)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 탈출 회전 실행
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def execute_escape_rotation(arduino, all_scan_points):
    """
    막힘 탈출 회전 실행

    1. find_escape_angle()로 최적 탈출 각도 계산
    2. 아두이노에 "ESC\\n" 전송 (헤딩 리셋 + 헤딩가드 비활성)
    3. v=0, w=±ESCAPE_W 전송
    4. 아두이노 헤딩 피드백으로 목표 각도 도달 확인
    5. 정지 후 정상 모드 복귀
    """
    global arduino_heading_deg, stuck_since

    print("\n" + "="*52)
    print("[ESCAPE] 전진 불가 감지 → 최적 방향 탈출 회전")

    # 1. 탈출 각도 계산
    target_deg = find_escape_angle(all_scan_points)
    target_rad = math.radians(target_deg)
    w_sign     = 1.0 if target_deg >= 0 else -1.0
    dir_str    = "왼쪽" if w_sign > 0 else "오른쪽"
    print(f"  회전 방향: {dir_str}  목표: {target_deg:.1f}°")

    # 2. 아두이노 탈출 모드 진입 (헤딩 리셋 + 가드 비활성)
    arduino.write(b"ESC\n")
    time.sleep(0.15)

    # 3. 회전 실행 (헤딩 모니터링)
    t_start = time.time()
    while time.time() - t_start < ESCAPE_TIMEOUT:
        read_arduino(arduino)

        current_hdg = abs(arduino_heading_deg)
        target_abs  = abs(target_deg)

        # 목표 각도 도달 여부 (허용 오차 ±ESCAPE_TOLERANCE°)
        if current_hdg >= target_abs - ESCAPE_TOLERANCE:
            print(f"  [완료] 헤딩 {arduino_heading_deg:.1f}° → 목표 {target_deg:.1f}°")
            break

        cmd = f"0.00 {w_sign * ESCAPE_W:.2f}\n"
        arduino.write(cmd.encode())
        time.sleep(SEND_INTERVAL)

    else:
        print(f"  [타임아웃] 최대 시간 초과, 강제 종료")

    # 4. 정지
    arduino.write(b"0.00 0.00\n")
    time.sleep(0.3)
    stuck_since = None
    print("="*52 + "\n")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 헤딩 > 90° 능동 복귀: 안전한 회전 방향 결정
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def get_recovery_direction(heading_deg, front_scan_points):
    """
    헤딩이 90°를 넘었을 때 0°로 복귀하는 안전한 방향 결정

    [자연 방향]
      heading > 0 (왼쪽으로 돌아있음) → 오른쪽(w<0)으로 회전해 0°로 복귀
      heading < 0 (오른쪽으로 돌아있음) → 왼쪽(w>0)

    [LiDAR 안전 확인]
      회전할 방향 쪽 (오른쪽 회전 → 음수 각도 구간) 에
      RECOVERY_SAFE_DIST 이내 장애물 있으면 → 반대 방향 시도

    Returns: w 값 (+: 왼쪽, -: 오른쪽)
    """
    natural_sign = -1.0 if heading_deg > 0 else 1.0   # 자연 방향 부호

    scan_dict = {}
    for angle_norm, dist in front_scan_points:
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    def side_blocked(sign):
        """sign < 0: 오른쪽(음수각도) 확인, sign > 0: 왼쪽(양수각도) 확인"""
        if sign < 0:
            check = range(-ANGLE_STEP, -SCAN_HALF_ANGLE - ANGLE_STEP, -ANGLE_STEP)
        else:
            check = range(ANGLE_STEP, SCAN_HALF_ANGLE + ANGLE_STEP, ANGLE_STEP)
        return any(
            0 < scan_dict.get(a, DETECTION_RANGE + 1) < RECOVERY_SAFE_DIST
            for a in check
        )

    if not side_blocked(natural_sign):
        dir_str = "오른쪽" if natural_sign < 0 else "왼쪽"
        print(f"  [복귀] 헤딩:{heading_deg:.1f}° → {dir_str} 회전 (자연방향 안전)")
        return natural_sign * RECOVERY_W
    else:
        alt_sign = -natural_sign
        dir_str  = "오른쪽" if alt_sign < 0 else "왼쪽"
        print(f"  [복귀] 자연방향 막힘 → {dir_str} 우회 회전")
        return alt_sign * RECOVERY_W


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 회피 방향 결정 (점수제)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def select_direction(left_clear, right_clear, heading_deg):
    """
    여유공간 + 헤딩 보정 점수로 회피 방향 결정

    left_score  = left_clear  + max(0, -heading_deg) × HEADING_WEIGHT
    right_score = right_clear + max(0,  heading_deg) × HEADING_WEIGHT

    heading > 0 (왼쪽으로 돌아있음) → 오른쪽에 헤딩 보너스
    heading < 0 (오른쪽으로 돌아있음) → 왼쪽에 헤딩 보너스

    효과:
      여유공간 차이가 크면 여유공간 방향이 이김
      여유공간이 비슷하면 헤딩 감소 방향이 이김
      헤딩이 클수록 보너스가 커져 자연스럽게 헤딩 복귀 유도
    """
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

    [선속도 — 전방거리 기반 연속 감속]
      n_fwd >= SLOW_START_DIST(400mm) → v = FORWARD_SPEED (최고속도)
      n_fwd <= STOP_DIST(120mm)       → v = 0 (회전만)
      그 사이                          → 선형 감소

    [각속도 — 수평오차 P제어]
      n_horiz < threshold(120mm) 인 동안 w 지속
      w = sign × W_GAIN × (threshold - n_horiz) / threshold
      n_horiz >= threshold → w = 0 (수평 여유 확보됨, 직진)
    """
    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN   # 120mm

    # 위험 포인트 수집
    danger_points = []
    for angle_norm, dist in scan_points:
        if dist <= 0 or dist > DETECTION_RANGE:
            continue
        horiz, fwd = decompose(angle_norm, dist)
        if fwd > 0 and fwd <= FORWARD_RANGE and horiz < threshold:
            danger_points.append((angle_norm, dist, horiz, fwd))

    if not danger_points:
        return FORWARD_SPEED, 0.0

    # 가장 가까운 장애물
    nearest = min(danger_points, key=lambda p: p[1])
    nearest_angle, ref_dist, n_horiz, n_fwd = nearest

    print(f"  [기준장애물] 각도:{nearest_angle:.1f}°  "
          f"직선:{ref_dist:.0f}mm  전방:{n_fwd:.0f}mm  수평:{n_horiz:.0f}mm")

    # ── 선속도: 전방거리 기반 연속 감속 ─────────────────────────────────────
    if n_fwd >= SLOW_START_DIST:
        v = FORWARD_SPEED
    elif n_fwd <= STOP_DIST:
        v = 0.0
    else:
        ratio = (n_fwd - STOP_DIST) / (SLOW_START_DIST - STOP_DIST)
        v = FORWARD_SPEED * ratio

    # ── 각속도: 수평오차 P제어 ───────────────────────────────────────────────
    horiz_error = threshold - n_horiz   # > 0: 아직 위험구역, ≤ 0: 안전
    if horiz_error > 0:
        # 좌우 여유공간
        scan_dict = {}
        for angle_norm, dist in scan_points:
            if dist <= 0:
                continue
            bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
            if bucket not in scan_dict or dist < scan_dict[bucket]:
                scan_dict[bucket] = dist

        left_clear = right_clear = 0
        for a in range(-SCAN_HALF_ANGLE, 0, ANGLE_STEP):
            if scan_dict.get(a, DETECTION_RANGE + 1) >= ref_dist:
                left_clear += ANGLE_STEP
        for a in range(ANGLE_STEP, SCAN_HALF_ANGLE + 1, ANGLE_STEP):
            if scan_dict.get(a, DETECTION_RANGE + 1) >= ref_dist:
                right_clear += ANGLE_STEP

        print(f"  [여유공간] 왼쪽:{left_clear}°  오른쪽:{right_clear}°  헤딩:{heading_deg:.1f}°")

        w_sign = select_direction(left_clear, right_clear, heading_deg)
        w      = w_sign * min(W_GAIN * horiz_error / threshold, MAX_W)
    else:
        w = 0.0   # 수평거리 충분 → 직진

    print(f"  [속도명령] v:{v:.2f}m/s  w:{w:.2f}rad/s  "
          f"(fwd:{n_fwd:.0f}mm  horiz_err:{horiz_error:.0f}mm)")

    return v, w


def main():
    global stuck_since, arduino_heading_deg

    print("=== RPLIDAR 장애물 회피 v5 ===")
    print(f"  라이다 포트    : {LIDAR_PORT}")
    print(f"  아두이노 포트  : {ARDUINO_PORT}")
    print(f"  라이다 보정    : +{LIDAR_OFFSET}mm")
    print(f"  위험구역       : 전방 {FORWARD_RANGE}mm × 수평 {ROBOT_HALF_WIDTH+SAFETY_MARGIN}mm")
    print(f"  선속도 감속    : {SLOW_START_DIST}mm부터 감속 → {STOP_DIST}mm에서 v=0")
    print(f"  각속도 방식    : 수평오차 P제어 (horiz < {ROBOT_HALF_WIDTH+SAFETY_MARGIN}mm 동안 유지)")
    print(f"  막힘감지       : 열린구간 너비 < {ROBOT_HALF_WIDTH*2}~{ROBOT_HALF_WIDTH*2+STUCK_MAX_SAFETY}mm → {STUCK_TIMEOUT}초 지속 시 탈출")
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

                    # ── ① 헤딩 > 90°: 능동 복귀 우선 ──────────────────────
                    if abs(arduino_heading_deg) > HEADING_OVER_90:
                        recovery_w = get_recovery_direction(
                            arduino_heading_deg, front_points
                        )
                        cmd = f"0.00 {recovery_w:.2f}\n"
                        arduino.write(cmd.encode())
                        print(f"[복귀모드] 헤딩:{arduino_heading_deg:.1f}°  "
                              f"v=0  w={recovery_w:.2f}")
                        last_cmd_str = cmd
                        last_send    = now
                        scan_points  = []
                        continue

                    # ── ② 막힘 감지 (LiDAR 공간 기반) ─────────────────────
                    blocked = is_path_blocked(front_points)

                    if blocked:
                        if stuck_since is None:
                            stuck_since = now
                            print("  [막힘시작] 통과 가능한 열린 구간 없음")
                        elif now - stuck_since >= STUCK_TIMEOUT:
                            # 탈출 회전 실행
                            execute_escape_rotation(arduino, all_scan_points)
                            last_cmd_str = ""
                            last_send    = time.time()
                            scan_points  = []
                            continue
                        else:
                            remaining = STUCK_TIMEOUT - (now - stuck_since)
                            print(f"  [막힘대기] {remaining:.1f}초 후 탈출 회전")
                    else:
                        if stuck_since is not None:
                            print("  [막힘해제] 경로 열림")
                        stuck_since = None

                    # ── ③ 정상 v/w 명령 ────────────────────────────────────
                    v, w = find_vw_command(front_points, arduino_heading_deg)
                    cmd  = f"{v:.2f} {w:.2f}\n"
                    arduino.write(cmd.encode())

                    if cmd != last_cmd_str:
                        print(f"[전송] v={v:.2f}  w={w:.2f}  "
                              f"헤딩={arduino_heading_deg:.1f}°")
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

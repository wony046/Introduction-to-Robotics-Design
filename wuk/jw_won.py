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

# ── 측면 보정 (방법 A: 차분 방식) ─────────────────────────────────────────────
# 측면 밴드 = "바운딩 박스 밖 + ±90° 이내 모든 포인트"
# 각 측면 하위 5% 거리 → 선형 weight → 차분으로 w_side 생성
SIDE_D_MIN       = 150   # mm: 이 미만 → weight = 1.0 (최대 보정)
SIDE_D_MAX       = 400   # mm: 이 이상 → weight = 0.0 (보정 없음)
SIDE_W_GAIN      = 1.5   # rad/s: |weight_R - weight_L| = 1일 때 회전 게인
SIDE_V_REDUCE    = 0.5   # max_weight=1.0일 때 선속도 50% 감속
SIDE_PERCENTILE  = 5     # %: 하위 N% 거리 평균을 측면 대표값으로 사용

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

def percentile_low(values, pct):
    """하위 pct% 거리의 평균 (노이즈 견고한 '가까운 거리' 추정)."""
    if not values: return None
    n_take = max(1, int(len(values) * pct / 100))
    return sum(sorted(values)[:n_take]) / n_take


def calc_side_correction(scan_points, danger_threshold):
    """
    측면 거리 기반 v/w 보정 신호 계산

    [현재 알고리즘: A안 = 차분 방식]
      w_side  = SIDE_W_GAIN × (weight_R - weight_L)
      v_scale = 1 - SIDE_V_REDUCE × max(weight_L, weight_R)

    [측면 밴드 정의]
      "바운딩 박스(horiz < threshold AND 0 < fwd ≤ FORWARD_RANGE) 밖"인
      ±90° 이내 모든 포인트

    [확장 hook]
      B안 (좁은통로 모드): max_weight 임계값으로 분기 → v 감속↑, w_side↓
      C안 (차분+magnitude): w_side = sign(R-L) × max_weight × K
      위치는 ⚙ 표시된 블록만 교체하면 됨
    """
    left_dists, right_dists = [], []

    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        horiz, fwd = decompose(angle_norm, dist)

        # 바운딩 박스 내부 제외 (전방 로직이 이미 처리)
        in_bbox = (fwd > 0 and fwd <= FORWARD_RANGE and horiz < danger_threshold)
        if in_bbox: continue

        # ±90° 측면 밴드 분류 (angle_norm은 normalize_angle 통과한 -180~180 값)
        if   angle_norm < 0: left_dists.append(dist)
        elif angle_norm > 0: right_dists.append(dist)

    # 하위 5% 평균 거리 (없으면 SIDE_D_MAX = weight 0)
    d_left  = percentile_low(left_dists,  SIDE_PERCENTILE) or SIDE_D_MAX
    d_right = percentile_low(right_dists, SIDE_PERCENTILE) or SIDE_D_MAX

    # 선형 weight: 가까울수록 1.0
    rng = SIDE_D_MAX - SIDE_D_MIN
    weight_L = max(0.0, min(1.0, (SIDE_D_MAX - d_left)  / rng))
    weight_R = max(0.0, min(1.0, (SIDE_D_MAX - d_right) / rng))

    # ⚙ ── 알고리즘 본체: A안 차분 ─────────────────────────────────────────
    # weight_R > weight_L → 오른쪽이 더 가까움 → w_side > 0 → 좌회전
    w_side = SIDE_W_GAIN * (weight_R - weight_L)
    max_weight = max(weight_L, weight_R)
    v_scale = 1.0 - SIDE_V_REDUCE * max_weight
    # ──────────────────────────────────────────────────────────────────────

    return {
        'w_side': w_side, 'v_scale': v_scale, 'max_weight': max_weight,
        'weight_L': weight_L, 'weight_R': weight_R,
        'd_left': d_left, 'd_right': d_right,
    }


def apply_side_correction(v_front, w_front, side_info):
    """전방 명령(v_front, w_front)에 측면 보정 블렌딩."""
    mw = side_info['max_weight']
    w_blended = (1.0 - mw) * w_front + mw * side_info['w_side']
    v_blended = v_front * side_info['v_scale']
    w_blended = max(-MAX_W, min(MAX_W, w_blended))
    return v_blended, w_blended


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

def select_direction_by_width(left_width, right_width, heading_deg):
    """실제 통과 너비(mm) + 헤딩 보너스로 회피 방향 1회 결정.
    (측면 차단 판단은 calc_side_correction의 연속적 w_side로 위임됨)
    """
    left_ok  = (left_width  >= MIN_PASSAGE_WIDTH)
    right_ok = (right_width >= MIN_PASSAGE_WIDTH)

    if left_ok and not right_ok:
        print(f"  [방향] 오른쪽 불가(너비:{right_width:.0f}mm) → 왼쪽 강제")
        return 1.0
    if right_ok and not left_ok:
        print(f"  [방향] 왼쪽 불가(너비:{left_width:.0f}mm) → 오른쪽 강제")
        return -1.0
    if not left_ok and not right_ok:
        chosen = "왼쪽" if left_width >= right_width else "오른쪽"
        print(f"  [방향] 양쪽 좁음(L:{left_width:.0f} R:{right_width:.0f}mm) → {chosen} 뚝심 돌파")
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
    global avoidance_w_sign, stop_zone_w_sign, no_danger_count
    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN

    # ── 0. 측면 보정 미리 계산 (모든 분기에 동일 적용) ────────────────────
    side_info = calc_side_correction(scan_points, threshold)
    if side_info['max_weight'] > 0.01:
        print(f"  [측면] d_L:{side_info['d_left']:.0f}mm d_R:{side_info['d_right']:.0f}mm  "
              f"w_L:{side_info['weight_L']:.2f} w_R:{side_info['weight_R']:.2f}  "
              f"w_side:{side_info['w_side']:+.2f}  v_scale:{side_info['v_scale']:.2f}")

    # ── 1. 위험 포인트 수집 ───────────────────────────────────────────────
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
        return apply_side_correction(FORWARD_SPEED, 0.0, side_info)
    no_danger_count = 0

    # ── 2. 근접 후보 수집 (기존) ──────────────────────────────────────────
    proximity_points = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        horiz, fwd = decompose(angle_norm, dist)
        if fwd > 0 and fwd <= FORWARD_RANGE and threshold <= horiz < PROXIMITY_HORIZ:
            proximity_points.append((angle_norm, dist, horiz, fwd))

    # ── 3. 선속도 결정 (기존) ─────────────────────────────────────────────
    stop_points = [p for p in danger_points
                   if p[3] <= STOP_FWD_RANGE and p[2] < ROBOT_HALF_WIDTH]
    frontal     = [p for p in danger_points if p[3] >= p[2]]
    n_fwd_ref   = min((p[3] for p in frontal), default=SLOW_START_DIST + 1)

    horiz_ref_danger          = min(danger_points, key=lambda p: p[2])
    _, _, n_horiz, _          = horiz_ref_danger
    horiz_error               = threshold - n_horiz

    ref_candidates            = danger_points + proximity_points
    horiz_ref_stable          = min(ref_candidates, key=lambda p: p[2])
    ref_angle, ref_dist, _, _ = horiz_ref_stable

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
        return apply_side_correction(v, 0.0, side_info)

    # ── 4. 회전 방향 결정 (측면 차단 로직 제거됨) ─────────────────────────
    if stop_points:
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
        stop_zone_w_sign = 0.0
        left_width  = get_gap_width(scan_points, ref_angle, ref_dist, is_left=True)
        right_width = get_gap_width(scan_points, ref_angle, ref_dist, is_left=False)

        if avoidance_w_sign == 0.0:
            avoidance_w_sign = select_direction_by_width(left_width, right_width, heading_deg)
            print(f"  [방향결정] {'왼쪽' if avoidance_w_sign > 0 else '오른쪽'} 고착")
        else:
            committed_width = left_width  if avoidance_w_sign > 0 else right_width
            opposite_width  = right_width if avoidance_w_sign > 0 else left_width
            if committed_width < MIN_PASSAGE_WIDTH and opposite_width >= MIN_PASSAGE_WIDTH:
                old = avoidance_w_sign
                avoidance_w_sign = select_direction_by_width(left_width, right_width, heading_deg)
                if avoidance_w_sign != old:
                    print(f"  [방향전환] 반대쪽 통과 가능 → "
                          f"{'왼쪽' if avoidance_w_sign > 0 else '오른쪽'}")

    # ── 5. 각속도 계산 ────────────────────────────────────────────────────
    w_mag = max(min(W_GAIN * horiz_error / threshold, MAX_W), W_MIN_DANGER)
    w     = avoidance_w_sign * w_mag

    print(f"  [명령(전방)] v:{v:.2f}  w:{w:.2f}  (수평오차:{horiz_error:.0f}mm)")
    return apply_side_correction(v, w, side_info)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    global prev_w
    print("=== RPLIDAR Obstacle Avoidance (Side Correction A: Differential) ===")
    print(f"  BoundingBox : horiz {ROBOT_HALF_WIDTH + SAFETY_MARGIN}mm x fwd {FORWARD_RANGE}mm")
    print(f"  SideCorr    : D=[{SIDE_D_MIN}~{SIDE_D_MAX}]mm  bottom {SIDE_PERCENTILE}% percentile")
    print(f"  SideGain    : w_gain={SIDE_W_GAIN}  v_reduce={SIDE_V_REDUCE}")
    print(f"  MinPassage  : {MIN_PASSAGE_WIDTH}mm")
    print(f"  ProximityRef: horiz < {PROXIMITY_HORIZ}mm (ref stabilization)")
    print(f"  NoiseFilter : ignore dist < {LIDAR_MIN_VALID}mm")
    print(f"  DepthJump   : +/-{DEPTH_JUMP_THRES}mm (bidirectional)")
    print(f"  FrontDead   : +/-{STOP_FRONT_DEADBAND}deg (stop zone noise suppression)")
    print("=" *60)

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

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
SIDE_W_GAIN      = 1.0   # rad/s: |weight_R - weight_L| = 1일 때 회전 게인
SIDE_V_REDUCE    = 0.5   # max_weight=1.0일 때 선속도 50% 감속
SIDE_PERCENTILE  = 10     # %: 하위 N% 거리 평균을 측면 대표값으로 사용

# ── 근접 후보 (방법 A) ────────────────────────────────────────────────────────
PROXIMITY_HORIZ = 170    # mm: danger zone(140mm) 밖이어도 이 이내면 ref 후보 포함

# ── 헤딩 방향 점수제 ──────────────────────────────────────────────────────────
HEADING_WEIGHT_MM = 5.0  # 헤딩 1°당 여유공간 5mm의 가중치 보너스

# ── 스캔 파라미터 ─────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE  = 90
ANGLE_STEP       = 5
SEND_INTERVAL    = 0.1
DEPTH_JUMP_THRES = 120

# ── Stop zone 정면 노이즈 억제 ────────────────────────────────────────────────
STOP_FRONT_DEADBAND = 15

# ── 디버그 출력 토글 ──────────────────────────────────────────────────────────
# 노이즈 줄이고 싶을 때 False로 끄세요. SIDE/BLEND는 사용자 관심사라 기본 ON.
DEBUG_SIDE  = True    # [SIDE] 좌우 거리 / 가중치 / w_side / v_scale
DEBUG_BLEND = True    # [BLEND] 전방 명령 → 측면 블렌딩 결과
DEBUG_FRONT = True    # [FRONT], [FRONT_CMD] 전방 기준점 및 명령
DEBUG_GAP   = True    # [GAP L/R] 빈공간 너비 계산
DEBUG_DIR   = True    # [DIR], [STOP] 방향 결정/전환

# ── 전역 상태 ─────────────────────────────────────────────────────────────────
arduino_heading_deg = 0.0
avoidance_w_sign    = 0.0
stop_zone_w_sign    = 0.0
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
    측면 거리 기반 v/w 보정 신호 계산 (A안 = 차분 방식)

    측면 밴드 = "바운딩 박스 밖 + ±90° 이내 모든 포인트"
    좌/우 각각 하위 SIDE_PERCENTILE% 거리 → 선형 weight → 차분
    """
    left_dists, right_dists = [], []

    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        horiz, fwd = decompose(angle_norm, dist)

        # 바운딩 박스 내부 제외 (전방 로직이 이미 처리)
        in_bbox = (fwd > 0 and fwd <= FORWARD_RANGE and horiz < danger_threshold)
        if in_bbox: continue

        if   angle_norm < 0: left_dists.append(dist)
        elif angle_norm > 0: right_dists.append(dist)

    d_left  = percentile_low(left_dists,  SIDE_PERCENTILE) if left_dists  else SIDE_D_MAX
    d_right = percentile_low(right_dists, SIDE_PERCENTILE) if right_dists else SIDE_D_MAX

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
        'n_left': len(left_dists), 'n_right': len(right_dists),
        'raw_min_L': min(left_dists)  if left_dists  else None,
        'raw_min_R': min(right_dists) if right_dists else None,
    }


def apply_side_correction(v_front, w_front, side_info):
    """전방 명령(v_front, w_front)에 측면 보정 블렌딩."""
    mw = side_info['max_weight']
    w_blended = (1.0 - mw) * w_front + mw * side_info['w_side']
    v_blended = v_front * side_info['v_scale']
    w_blended = max(-MAX_W, min(MAX_W, w_blended))
    if DEBUG_BLEND and mw > 0.01:
        print(f"  [BLEND] mw={mw:.2f}: "
              f"v({v_front:.2f}->{v_blended:.2f})  "
              f"w({w_front:+.2f}->{w_blended:+.2f})")
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
    tag = 'L' if is_left else 'R'

    for i, p in enumerate(search_points):
        if abs(p[1] - edge_p[1]) > DEPTH_JUMP_THRES:
            wall_points = search_points[i:]
            if wall_points:
                min_width = min(
                    calc_law_of_cosines(edge_p[1], wp[1], abs(edge_p[0] - wp[0]))
                    for wp in wall_points
                )
                if DEBUG_GAP:
                    print(f"  [GAP {tag}] edge={edge_p[0]:.0f}deg/{edge_p[1]:.0f}mm  "
                          f"jump={p[1]-edge_p[1]:+.0f}mm  ->  width={min_width:.0f}mm")
                return min_width
        edge_p = p

    rem_angle = abs((-SCAN_HALF_ANGLE - edge_p[0]) if is_left
                    else (SCAN_HALF_ANGLE - edge_p[0]))

    if rem_angle > 15:
        width = calc_law_of_cosines(edge_p[1], edge_p[1], rem_angle)
        if DEBUG_GAP:
            print(f"  [GAP {tag}] wall extends to scan end  ->  "
                  f"conservative width={width:.0f}mm "
                  f"(edge {edge_p[1]:.0f}mm x {rem_angle:.0f}deg)")
        return width

    if DEBUG_GAP:
        print(f"  [GAP {tag}] insufficient scan angle  ->  0mm")
    return 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 회피 방향 결정
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def select_direction_by_width(left_width, right_width, heading_deg):
    """실제 통과 너비(mm) + 헤딩 보너스로 회피 방향 1회 결정."""
    left_ok  = (left_width  >= MIN_PASSAGE_WIDTH)
    right_ok = (right_width >= MIN_PASSAGE_WIDTH)

    if left_ok and not right_ok:
        if DEBUG_DIR:
            print(f"  [DIR] right blocked (width:{right_width:.0f}mm)  ->  force LEFT")
        return 1.0
    if right_ok and not left_ok:
        if DEBUG_DIR:
            print(f"  [DIR] left blocked (width:{left_width:.0f}mm)  ->  force RIGHT")
        return -1.0
    if not left_ok and not right_ok:
        chosen = "LEFT" if left_width >= right_width else "RIGHT"
        if DEBUG_DIR:
            print(f"  [DIR] both narrow (L:{left_width:.0f} R:{right_width:.0f}mm)"
                  f"  ->  push through {chosen}")
        return 1.0 if left_width >= right_width else -1.0

    left_score  = left_width  + max(0.0, -heading_deg) * HEADING_WEIGHT_MM
    right_score = right_width + max(0.0,  heading_deg) * HEADING_WEIGHT_MM
    bonus_side = "R" if heading_deg > 0 else "L"
    bonus_val  = abs(heading_deg) * HEADING_WEIGHT_MM
    if DEBUG_DIR:
        print(f"  [DIR_SCORE] L={left_score:.0f}  R={right_score:.0f}  "
              f"(width L={left_width:.0f}mm R={right_width:.0f}mm  "
              f"heading_bonus {bonus_side}+{bonus_val:.0f})")
    return 1.0 if left_score >= right_score else -1.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# v/w 명령 계산
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_command(scan_points, heading_deg):
    global avoidance_w_sign, stop_zone_w_sign, no_danger_count
    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN

    # ── 0. 측면 보정 미리 계산 (모든 분기에 동일 적용) ────────────────────
    side_info = calc_side_correction(scan_points, threshold)
    if DEBUG_SIDE:
        si = side_info
        rmin_L = si['raw_min_L'] if si['raw_min_L'] is not None else 0
        rmin_R = si['raw_min_R'] if si['raw_min_R'] is not None else 0
        print(f"  [SIDE]  L: d={si['d_left']:.0f}mm wL={si['weight_L']:.2f} "
              f"n={si['n_left']} min={rmin_L:.0f}  |  "
              f"R: d={si['d_right']:.0f}mm wR={si['weight_R']:.2f} "
              f"n={si['n_right']} min={rmin_R:.0f}")
        print(f"          ->  w_side={si['w_side']:+.2f}  "
              f"v_scale={si['v_scale']:.2f}  mw={si['max_weight']:.2f}")

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
        if DEBUG_FRONT:
            print(f"  [FRONT] no danger points  ->  v={FORWARD_SPEED:.2f} w=0.00")
        return apply_side_correction(FORWARD_SPEED, 0.0, side_info)
    no_danger_count = 0

    # ── 2. 근접 후보 수집 ─────────────────────────────────────────────────
    proximity_points = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        horiz, fwd = decompose(angle_norm, dist)
        if fwd > 0 and fwd <= FORWARD_RANGE and threshold <= horiz < PROXIMITY_HORIZ:
            proximity_points.append((angle_norm, dist, horiz, fwd))

    # ── 3. 선속도 결정 ────────────────────────────────────────────────────
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
    if DEBUG_FRONT:
        print(f"  [FRONT] fwd={n_fwd_ref:.0f}mm  stop={len(stop_points)}  "
              f"ref_angle={ref_angle:.1f}deg  horiz(danger)={n_horiz:.0f}mm"
              + ("  [proximity_ref]" if is_proximity_ref else ""))

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
        if DEBUG_FRONT:
            print(f"  [FRONT_CMD] v={v:.2f}  w=0.00  (no horiz error, passing through)")
        return apply_side_correction(v, 0.0, side_info)

    # ── 4. 회전 방향 결정 ─────────────────────────────────────────────────
    if stop_points:
        stop_angle = min(stop_points, key=lambda p: p[2])[0]
        if abs(stop_angle) < STOP_FRONT_DEADBAND and stop_zone_w_sign != 0.0:
            avoidance_w_sign = stop_zone_w_sign
            if DEBUG_DIR:
                print(f"  [STOP] front noise (angle:{stop_angle:.1f}deg)  ->  "
                      f"keep prev dir ({'LEFT' if avoidance_w_sign > 0 else 'RIGHT'})")
        else:
            avoidance_w_sign = 1.0 if stop_angle >= 0 else -1.0
            stop_zone_w_sign = avoidance_w_sign
            if DEBUG_DIR:
                print(f"  [STOP] angle:{stop_angle:.1f}deg  ->  "
                      f"{'LEFT' if avoidance_w_sign > 0 else 'RIGHT'} (immediate)")
    else:
        stop_zone_w_sign = 0.0
        left_width  = get_gap_width(scan_points, ref_angle, ref_dist, is_left=True)
        right_width = get_gap_width(scan_points, ref_angle, ref_dist, is_left=False)

        if avoidance_w_sign == 0.0:
            avoidance_w_sign = select_direction_by_width(left_width, right_width, heading_deg)
            if DEBUG_DIR:
                print(f"  [DIR_LOCK] {'LEFT' if avoidance_w_sign > 0 else 'RIGHT'}")
        else:
            committed_width = left_width  if avoidance_w_sign > 0 else right_width
            opposite_width  = right_width if avoidance_w_sign > 0 else left_width
            if committed_width < MIN_PASSAGE_WIDTH and opposite_width >= MIN_PASSAGE_WIDTH:
                old = avoidance_w_sign
                avoidance_w_sign = select_direction_by_width(left_width, right_width, heading_deg)
                if avoidance_w_sign != old and DEBUG_DIR:
                    print(f"  [DIR_FLIP] opposite passable  ->  "
                          f"{'LEFT' if avoidance_w_sign > 0 else 'RIGHT'}")

    # ── 5. 각속도 계산 ────────────────────────────────────────────────────
    w_mag = max(min(W_GAIN * horiz_error / threshold, MAX_W), W_MIN_DANGER)
    w     = avoidance_w_sign * w_mag

    if DEBUG_FRONT:
        print(f"  [FRONT_CMD] v={v:.2f}  w={w:+.2f}  (horiz_err={horiz_error:.0f}mm)")
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
    print(f"  Debug flags : SIDE={DEBUG_SIDE} BLEND={DEBUG_BLEND} FRONT={DEBUG_FRONT} "
          f"GAP={DEBUG_GAP} DIR={DEBUG_DIR}")
    print("=" * 70)

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
                        print(f"[SEND] v={v:.2f}  w={w:+.2f}  "
                              f"heading={arduino_heading_deg:.1f}deg")
                        last_cmd_str = cmd
                    last_send = now
                scan_points = []

            scan_points.append((
                normalize_angle(angle_raw),
                distance + LIDAR_OFFSET if distance > 0 else 0
            ))

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        lidar.close()
        arduino.write(b"0.00 0.00\n")
        arduino.close()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()

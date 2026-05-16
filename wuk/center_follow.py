import serial
import time
import math

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 포트 & 라이다 설정
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

LIDAR_OFFSET    = 20    # mm: 라이다 측정값 보정
LIDAR_MIN_VALID = 100   # mm: 이 미만 무시 (노이즈)
DETECTION_RANGE = 1500  # mm: 라이다 최대 신뢰 거리

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 로봇 & 속도 파라미터
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

ROBOT_HALF_WIDTH = 110   # mm: 라이다 중심 ~ 좌우 끝
ROBOT_RADIUS     = 100   # mm: 이 원 안의 포인트는 무시 (로봇 자신 영역)

FORWARD_SPEED    = 0.35  # m/s: 최대 전진 속도
MIN_SPEED        = 0.07  # m/s: 최소 전진 속도
MAX_W            = 1.5   # rad/s: 최대 각속도
W_SMOOTH         = 0.6   # w 지수 평활 계수 (클수록 새 값 비중 ↑)

CLUSTER_PERCENTILE = 5   # %: 하위 N% 최근접 포인트 → 대표점 군집

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 조향(w) 박스 : 가로 1000 x 세로 800
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#   좌(angle<0)/우(angle>0) 각각 최근접 5% 군집 → 대표점 1개씩
#   두 대표점의 직교좌표 중점 방향으로 부드럽게 조향

STEER_BOX_HALF_W = 500   # mm: 좌우 ±500 (총 1000)
STEER_BOX_DEPTH  = 800   # mm: 전방 0~800
STEER_GAIN       = 0.035 # w = -STEER_GAIN * target_deg  (약 ±43°에서 MAX_W 포화)

MIN_PASSAGE_WIDTH = 240  # mm: 좌/우 대표점 직선거리(d_LR)가 이보다 좁으면
                         #     통과 불가로 판단 → STOP. (= 로봇폭 220 + 마진 20)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 속도(v) 박스 : 가로 220 x 세로 800
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#   로봇 정면 차선폭 안의 최근접 5% 군집 → 전방거리 기준 비선형 감속

SPEED_BOX_HALF_W = 110   # mm: 좌우 ±110 (총 220)
SPEED_BOX_DEPTH  = 800   # mm: 전방 0~800

V_FULL_DIST   = 800      # mm: 이 이상이면 FORWARD_SPEED
V_STOP_DIST   = 200      # mm: 이 이하이면 MIN_SPEED (STOP zone 직전)
V_DECEL_GAMMA = 0.45     # 감속 곡선 지수 (<1 : knee가 가까운 쪽에 형성)
                         #   ↓ 낮추면 → 더 멀리까지 고속 유지 후 가까이서 급제동
                         #   ↑ 높이면 → 멀리서부터 완만하게 감속 (1.0 = 선형)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP zone (기존 유지)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 180
STOP_HORIZ_TH = 110

STOP_ESCAPE_SCAN_HALF = 135
STOP_ESCAPE_MIN_GAP   = ROBOT_HALF_WIDTH * 2 + 40   # 260mm
STOP_SECTOR_SIZE      = 10                          # deg
STOP_MAX_CYCLES       = 8
DEPTH_JUMP_THRES      = 120   # mm: STOP escape 갭 계산용

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 스캔 범위 & 통신
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCAN_WIDE_HALF = 135   # 메인에서 받는 스캔 범위 (STOP escape용)
SEND_INTERVAL  = 0.1

# ── 디버그 토글 ──────────────────────────────────────────────────────────────
DEBUG_STEER = True
DEBUG_SPEED = True
DEBUG_STOP  = True
DEBUG_FINAL = True

# ── 전역 상태 ────────────────────────────────────────────────────────────────
arduino_heading_deg = 0.0
prev_w              = 0.0
stop_cycle_count    = 0     # 연속 STOP-zone 사이클 카운터


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return angle - 360 if angle > 180 else angle

def is_in_front_90(a):
    return -90 <= a <= 90

def is_in_wide_scan(a):
    return -SCAN_WIDE_HALF <= a <= SCAN_WIDE_HALF

def decompose(angle_deg, dist):
    """극좌표 → (horiz, fwd). horiz는 항상 양수, fwd는 전방 양수."""
    rad = math.radians(angle_deg)
    horiz = abs(dist * math.sin(rad))
    fwd   = dist * math.cos(rad)
    return horiz, fwd

def cosine_dist(d1, d2, angle_diff_deg):
    """코사인 제2법칙: 두 극좌표 점 사이의 직선거리."""
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
# STOP zone 감지 & 탈출 (기존 로직 유지)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def detect_stop_zone(scan_points):
    """STOP rectangle(fwd 100~180mm, horiz<110mm) 안에 장애물이 있는가?"""
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_front_90(angle_norm): continue
        horiz, fwd = decompose(angle_norm, dist)
        if STOP_FWD_MIN <= fwd <= STOP_FWD_MAX and horiz < STOP_HORIZ_TH:
            return True
    return False


def get_gap_width(scan_points, ref_angle, ref_dist, is_left):
    """ref 기준 좌/우 첫 depth jump까지의 통과 가능 너비."""
    front = [(a, d) for a, d in scan_points if is_in_front_90(a)]
    if is_left:
        search = sorted([p for p in front if p[0] < ref_angle],
                        key=lambda x: x[0], reverse=True)
    else:
        search = sorted([p for p in front if p[0] > ref_angle],
                        key=lambda x: x[0])
    if not search:
        return 0.0
    edge_p = (ref_angle, ref_dist)
    for i, p in enumerate(search):
        if abs(p[1] - edge_p[1]) > DEPTH_JUMP_THRES:
            wall = search[i:]
            if wall:
                return min(cosine_dist(edge_p[1], wp[1], abs(edge_p[0] - wp[0]))
                           for wp in wall)
        edge_p = p
    rem_angle = abs((-90 - edge_p[0]) if is_left else (90 - edge_p[0]))
    if rem_angle > 15:
        return cosine_dist(edge_p[1], edge_p[1], rem_angle)
    return 0.0


def find_stop_escape_direction(scan_points):
    """±135° 범위를 sector로 나누어 가장 빈 방향(전방 선호) 반환."""
    sectors = {}
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_wide_scan(angle_norm): continue
        center = round(angle_norm / STOP_SECTOR_SIZE) * STOP_SECTOR_SIZE
        sectors.setdefault(center, []).append(dist)
    if not sectors:
        return 0.0, 0.0
    sector_avg = {c: sum(d) / len(d) for c, d in sectors.items()}

    valid = {}
    for c, avg_dist in sector_avg.items():
        gap_l = get_gap_width(scan_points, c, avg_dist, is_left=True)
        gap_r = get_gap_width(scan_points, c, avg_dist, is_left=False)
        if gap_l + gap_r >= STOP_ESCAPE_MIN_GAP:
            valid[c] = avg_dist
    candidates = valid if valid else sector_avg

    def forward_score(c, dist):
        factor = (1.0 + math.cos(math.radians(c))) / 2.0  # 0°=1.0, 90°=0.5
        return dist * factor

    best = max(candidates.keys(), key=lambda c: forward_score(c, candidates[c]))
    return float(best), candidates[best]


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 조향(w) : 1000x800 박스 → 좌/우 대표점 → 중점 방향
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def get_side_representative(scan_points, is_left):
    """
    조향 박스(1000x800) 안에서 좌(angle<0) 또는 우(angle>0)의
    최근접 하위 5% 군집 평균 → 대표점 dict {'angle','dist','n'}.
    포인트 없으면 None.
    """
    pts = []
    for angle_norm, dist in scan_points:
        # 로봇 반경 100mm 원 안의 포인트 + 노이즈 제거
        if dist < max(LIDAR_MIN_VALID, ROBOT_RADIUS) or dist > DETECTION_RANGE:
            continue
        if is_left and angle_norm >= 0:        continue
        if (not is_left) and angle_norm <= 0:  continue
        horiz, fwd = decompose(angle_norm, dist)
        if horiz < STEER_BOX_HALF_W and 0 < fwd < STEER_BOX_DEPTH:
            pts.append((angle_norm, dist))

    if not pts:
        return None

    pts.sort(key=lambda p: p[1])                       # 거리 오름차순
    n_take = max(1, int(len(pts) * CLUSTER_PERCENTILE / 100))
    rep = pts[:n_take]
    return {
        'angle': sum(p[0] for p in rep) / len(rep),
        'dist':  sum(p[1] for p in rep) / len(rep),
        'n':     len(pts),
    }


def compute_steering(scan_points):
    """
    반환: (status, target_angle_deg, dbg)
      status = 'OK'       → target 방향으로 조향
               'STRAIGHT' → 양쪽 모두 비어있음 → 직진
               'STOP'     → 한쪽만 있음 / 통로 너무 좁음 → 정지
    """
    L = get_side_representative(scan_points, is_left=True)
    R = get_side_representative(scan_points, is_left=False)

    # 양쪽 모두 없음 → 직진
    if L is None and R is None:
        return 'STRAIGHT', 0.0, {}

    # 한쪽만 있음 → 정지 (사용자 결정)
    if L is None or R is None:
        return 'STOP', 0.0, {'reason': 'one_side_only',
                             'missing': 'L' if L is None else 'R'}

    # 코사인 제2법칙으로 두 대표점 직선거리
    d_LR = cosine_dist(L['dist'], R['dist'], R['angle'] - L['angle'])

    # 통로가 로봇폭+마진보다 좁음 → 사실상 같은 물체 → 정지
    if d_LR < MIN_PASSAGE_WIDTH:
        return 'STOP', 0.0, {'reason': 'narrow_gap', 'd_LR': d_LR,
                             'L': L, 'R': R}

    # 두 대표점의 직교좌표 중점 → 목표 조향각
    xL = L['dist'] * math.sin(math.radians(L['angle']))
    yL = L['dist'] * math.cos(math.radians(L['angle']))
    xR = R['dist'] * math.sin(math.radians(R['angle']))
    yR = R['dist'] * math.cos(math.radians(R['angle']))
    mx, my = (xL + xR) / 2.0, (yL + yR) / 2.0
    target = math.degrees(math.atan2(mx, my))   # +면 우측, -면 좌측

    return 'OK', target, {'L': L, 'R': R, 'd_LR': d_LR, 'mid': (mx, my)}


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 속도(v) : 220x800 박스 → 최근접 대표점 → 비선형 감속
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def compute_speed(scan_points):
    """
    속도 박스(220x800) 안 최근접 하위 5% 군집의 전방거리 기준으로
    비선형 감속. 박스 안에 포인트 없으면 FORWARD_SPEED.
    반환: (v, rep_fwd or None)
    """
    fwds = []
    for angle_norm, dist in scan_points:
        if dist < max(LIDAR_MIN_VALID, ROBOT_RADIUS) or dist > DETECTION_RANGE:
            continue
        horiz, fwd = decompose(angle_norm, dist)
        if horiz < SPEED_BOX_HALF_W and 0 < fwd < SPEED_BOX_DEPTH:
            fwds.append(fwd)

    if not fwds:
        return FORWARD_SPEED, None

    fwds.sort()                                        # 전방거리 오름차순
    n_take = max(1, int(len(fwds) * CLUSTER_PERCENTILE / 100))
    rep_fwd = sum(fwds[:n_take]) / n_take

    # 비선형 감속: progress^gamma  (gamma<1 → 300mm 부근부터 급감속)
    if rep_fwd >= V_FULL_DIST:
        v = FORWARD_SPEED
    elif rep_fwd <= V_STOP_DIST:
        v = MIN_SPEED
    else:
        p = (rep_fwd - V_STOP_DIST) / (V_FULL_DIST - V_STOP_DIST)   # 0~1
        v = MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * (p ** V_DECEL_GAMMA)

    return v, rep_fwd


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 진입점 (STOP 우선 → 조향 → 속도)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_command(scan_points, heading_deg):
    global stop_cycle_count

    # 1) STOP zone 우선 검사
    if detect_stop_zone(scan_points) and stop_cycle_count < STOP_MAX_CYCLES:
        target, gap_dist = find_stop_escape_direction(scan_points)
        if abs(target) < 5:
            pivot_w = -MAX_W                       # 정면이 가장 빈 경우 default 우회전
        else:
            pivot_w = -math.copysign(MAX_W, target)
        stop_cycle_count += 1
        if DEBUG_STOP:
            print(f"  [STOP] zone (cycle {stop_cycle_count}/{STOP_MAX_CYCLES}) "
                  f"escape={target:+.0f}° gap={gap_dist:.0f}mm pivot_w={pivot_w:+.2f}")
        return 0.0, pivot_w

    if stop_cycle_count >= STOP_MAX_CYCLES and DEBUG_STOP:
        print(f"  [STOP] max cycles reached -> force normal mode")
    stop_cycle_count = 0

    # 2) 조향 계산
    status, target, dbg = compute_steering(scan_points)

    if DEBUG_STEER:
        if status == 'OK':
            L, R = dbg['L'], dbg['R']
            print(f"  [STEER] L(a={L['angle']:+.1f}° d={L['dist']:.0f} n={L['n']}) "
                  f"R(a={R['angle']:+.1f}° d={R['dist']:.0f} n={R['n']})  "
                  f"d_LR={dbg['d_LR']:.0f}mm  target={target:+.1f}°")
        elif status == 'STRAIGHT':
            print(f"  [STEER] both sides empty -> STRAIGHT")
        else:
            reason = dbg.get('reason')
            extra = (f" missing={dbg.get('missing')}" if reason == 'one_side_only'
                     else f" d_LR={dbg.get('d_LR', 0):.0f}mm")
            print(f"  [STEER] STOP ({reason}){extra}")

    # 정지 조건 → v=0, w=0
    if status == 'STOP':
        if DEBUG_FINAL:
            print(f"  [FINAL] v=0.00 w=0.00 (steering STOP)")
        return 0.0, 0.0

    # 3) 속도 계산
    v, rep_fwd = compute_speed(scan_points)
    if DEBUG_SPEED:
        if rep_fwd is None:
            print(f"  [SPEED] lane clear -> v={v:.2f}")
        else:
            print(f"  [SPEED] nearest fwd={rep_fwd:.0f}mm -> v={v:.2f}")

    # 4) w 산출 (목표각 비례 → 부드러운 조향, 메인 루프에서 추가 평활)
    w = max(-MAX_W, min(MAX_W, -STEER_GAIN * target))

    if DEBUG_FINAL:
        print(f"  [FINAL] v={v:.2f} w={w:+.2f} (status={status})")
    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    global prev_w
    print("=== RPLIDAR Obstacle Avoidance (Dual-Box / v8) ===")
    print(f"  Steering box : {STEER_BOX_HALF_W*2}x{STEER_BOX_DEPTH}mm "
          f"(L/R 최근접 {CLUSTER_PERCENTILE}% 군집 → 중점 방향)")
    print(f"  Speed box    : {SPEED_BOX_HALF_W*2}x{SPEED_BOX_DEPTH}mm "
          f"(비선형 감속 gamma={V_DECEL_GAMMA}, {V_STOP_DIST}~{V_FULL_DIST}mm)")
    print(f"  Robot radius : {ROBOT_RADIUS}mm 원 안 포인트 무시")
    print(f"  STOP zone    : fwd {STOP_FWD_MIN}-{STOP_FWD_MAX}mm, "
          f"horiz<{STOP_HORIZ_TH}mm")
    print(f"  STOP cases   : 한쪽만 군집 / d_LR<{MIN_PASSAGE_WIDTH}mm -> v=0,w=0")
    print(f"  Debug        : STEER={DEBUG_STEER} SPEED={DEBUG_SPEED} "
          f"STOP={DEBUG_STOP} FINAL={DEBUG_FINAL}")
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
                # ±135°까지 통과 (STOP escape에서 사용)
                wide_points = [
                    (a, d) for a, d in scan_points
                    if is_in_wide_scan(a) and d > 0
                ]
                now = time.time()
                if now - last_send >= SEND_INTERVAL:
                    v, w = find_vw_command(wide_points, arduino_heading_deg)
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

"""
RPLIDAR C1 장애물 회피 코드 (v4)
포트: 라이다 /dev/ttyUSB0 / 아두이노 /dev/ttyS0 (UART GPIO14/15)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[통신 구조]
  라즈베리파이 → 아두이노:
    "v w\n"      : 정상 속도 명령  (예: "0.20 0.52\n")
    "T180L\n"    : 왼쪽 180도 회전 명령
    "T180R\n"    : 오른쪽 180도 회전 명령

  아두이노 → 라즈베리파이:
    "H:XX.X\n"   : 현재 헤딩 각도 (도), 200ms 주기 전송
    "DONE:T180\n": 180도 회전 완료 신호

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[추가된 알고리즘]

① 헤딩 우선 방향 선택
  - 아두이노에서 수신한 헤딩값 활용
  - |헤딩| > HEADING_CAUTION_DEG 이면 여유공간 무시하고
    헤딩을 0으로 줄이는 방향을 우선 선택
  - 예) 헤딩 +75° (왼쪽으로 많이 돌았음)
        → 오른쪽(w < 0) 우선 선택 (헤딩 감소 방향)

② 진행 불가 판단 → 180도 회전
  - 판단 조건:
      장애물이 위험구역에 STUCK_TIMEOUT 초 이상 지속
      AND |헤딩| > HEADING_CAUTION_DEG (헤딩 한계 근접)
  - 안전 방향 결정 (전체 360도 스캔 사용):
      왼쪽 반구(0°~180°)와 오른쪽 반구(-180°~0°)에서
      가장 가까운 장애물 거리 비교
      → 더 멀리 비어있는 쪽으로 회전
  - T180L 또는 T180R 명령 전송 후 DONE:T180 수신 대기

[회피각 계산]
  delta_horiz = threshold - n_horiz + HORIZ_EXTRA
  avoid_angle = atan2(delta_horiz, n_fwd)
  → 실제 추가 필요 수평거리 기반, 더 정확한 회피각

[직사각형 위험구역]
  일반: 전방 800mm × 수평 120mm
  긴급: 전방 125mm × 수평 110mm (속도 감소 + 전반 감속)
"""

import serial
import time
import math

# ── 포트 ─────────────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyS0"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 9600

# ── 로봇 파라미터 ─────────────────────────────────────────────────────────────
ROBOT_HALF_WIDTH = 110      # 라이다 중심 ~ 로봇 좌우 끝 (mm)
SAFETY_MARGIN    = 10       # 장애물 인식 안전 여유 (mm) → threshold = 120mm

# ── 직사각형 위험구역 ─────────────────────────────────────────────────────────
DETECTION_RANGE     = 1500  # LiDAR 최대 신뢰 거리 (mm)
FORWARD_RANGE       = 800   # 일반 위험구역 전방 깊이 (mm)
EMERGENCY_FWD_RANGE = 125   # 긴급 회피구역 전방 깊이 (mm)
EMERGENCY_HORIZ_RANGE = 110 # 긴급 회피구역 수평 깊이 (mm)

# ── 속도 파라미터 ─────────────────────────────────────────────────────────────
FORWARD_SPEED       = 0.20  # 기본 전진 선속도 (m/s)
EMERGENCY_MIN_SPEED = 0.05  # 긴급 상황 최소 속도 (m/s)
W_GAIN              = 2.0   # 회피각 → 각속도 게인
MAX_W               = 2.0   # 최대 각속도 (rad/s)
HORIZ_EXTRA         = 20    # 회피각 계산 수평 이동량 여유 (mm)

# ── 헤딩 우선 방향 파라미터 ──────────────────────────────────────────────────
HEADING_CAUTION_DEG = 60.0  # 이 이상이면 헤딩 감소 방향 우선 (도)

# ── 진행 불가 / 180도 회전 파라미터 ──────────────────────────────────────────
STUCK_TIMEOUT    = 4.0      # 위험구역 지속 + 헤딩 한계 근접 시 180도 회전 판단 (초)
TURN180_W        = 1.2      # 180도 회전 시 각속도 (rad/s)
TURN_SAFE_DIST   = 250      # 180도 회전 안전 확인 최소 장애물 거리 (mm)
TURN180_TIMEOUT  = 8.0      # 180도 회전 최대 대기 시간 (초)

# ── 스캔 파라미터 ─────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE = 90
ANGLE_STEP      = 5
SEND_INTERVAL   = 0.1       # 명령 전송 주기 (초)
# ─────────────────────────────────────────────────────────────────────────────

# ── 전역 상태 ─────────────────────────────────────────────────────────────────
arduino_heading_deg = 0.0   # 아두이노에서 수신한 헤딩 (도)
stuck_since         = None  # 막힘 시작 시각


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
    distance    = distance_q2 / 4.0
    return angle, distance, quality


def decompose(angle_norm_deg, distance_mm):
    rad   = math.radians(angle_norm_deg)
    horiz = abs(distance_mm * math.sin(rad))
    fwd   = distance_mm * math.cos(rad)
    return horiz, fwd


def read_arduino(arduino):
    """
    아두이노에서 데이터 수신 (non-blocking)
    "H:XX.X\n" 형식으로 헤딩 수신 → arduino_heading_deg 업데이트
    반환: 수신된 특수 메시지 문자열 또는 None
    """
    global arduino_heading_deg
    msg = None
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'):
                arduino_heading_deg = float(line[2:])
            elif line:
                msg = line   # DONE:T180 등 특수 메시지
        except Exception:
            pass
    return msg


def select_direction(left_clear, right_clear, heading_deg):
    """
    회피 방향 결정

    우선순위:
    1. 헤딩이 HEADING_CAUTION_DEG 초과 → 헤딩 감소 방향 강제
       예) 헤딩 = +75° (좌회전 많이 됨) → 오른쪽(w < 0) 우선
       예) 헤딩 = -75° (우회전 많이 됨) → 왼쪽(w > 0) 우선
    2. 그 외 → 여유공간 넓은 쪽 선택
    """
    if heading_deg > HEADING_CAUTION_DEG:
        print(f"  [헤딩우선] 헤딩:{heading_deg:.1f}° > {HEADING_CAUTION_DEG}° → 오른쪽 강제")
        return -1.0   # 오른쪽 (헤딩 감소 방향)
    elif heading_deg < -HEADING_CAUTION_DEG:
        print(f"  [헤딩우선] 헤딩:{heading_deg:.1f}° < -{HEADING_CAUTION_DEG}° → 왼쪽 강제")
        return 1.0    # 왼쪽 (헤딩 증가 방향 = 헤딩 절댓값 감소)
    else:
        return 1.0 if left_clear >= right_clear else -1.0


def find_safe_180_direction(all_scan_points):
    """
    전체 360도 스캔에서 180도 회전에 안전한 방향 결정

    [기하학적 의미]
      왼쪽 회전(반시계): 로봇 왼쪽·후방 영역(0°~180°)이 쓸려감
      오른쪽 회전(시계): 로봇 오른쪽·후방 영역(-180°~0°)이 쓸려감

      각 반구에서 TURN_SAFE_DIST 이내 장애물 수 비교
      → 장애물이 더 적고/멀리 있는 쪽으로 회전

    반환: 'L' (왼쪽 회전) 또는 'R' (오른쪽 회전)
    """
    left_obstacles  = []   # 왼쪽 반구 (0°~180°) 장애물 거리
    right_obstacles = []   # 오른쪽 반구 (-180°~0°) 장애물 거리

    for angle_norm, dist in all_scan_points:
        if dist <= 0 or dist > DETECTION_RANGE:
            continue
        if 0 < angle_norm <= 180:
            left_obstacles.append(dist)
        elif -180 <= angle_norm < 0:
            right_obstacles.append(dist)

    # 안전 거리 이내 장애물 수 비교
    left_danger  = sum(1 for d in left_obstacles  if d < TURN_SAFE_DIST)
    right_danger = sum(1 for d in right_obstacles if d < TURN_SAFE_DIST)

    # 가장 가까운 장애물 거리 비교 (장애물 없으면 무한대)
    left_min  = min(left_obstacles,  default=float('inf'))
    right_min = min(right_obstacles, default=float('inf'))

    print(f"  [T180판단] 왼쪽반구 위험{left_danger}개(최근:{left_min:.0f}mm) "
          f"오른쪽반구 위험{right_danger}개(최근:{right_min:.0f}mm)")

    # 위험 수가 같으면 최근접 장애물 거리로 판단
    if left_danger < right_danger:
        return 'L'
    elif right_danger < left_danger:
        return 'R'
    else:
        return 'L' if left_min >= right_min else 'R'


def find_vw_command(scan_points, heading_deg):
    """
    정면 스캔 + 현재 헤딩 → (v, w, danger_with_heading_limit) 반환

    반환: (v_m_s, w_rad_s, is_stuck_condition)
      is_stuck_condition: True면 헤딩 한계+장애물 동시 조건 (막힘 카운터 증가용)
    """
    global arduino_heading_deg

    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN   # 120mm

    # ── 1. 직사각형 위험구역 장애물 수집 ────────────────────────────────────
    danger_points = []
    for angle_norm, dist in scan_points:
        if dist <= 0 or dist > DETECTION_RANGE:
            continue
        horiz, fwd = decompose(angle_norm, dist)
        if fwd > 0 and fwd <= FORWARD_RANGE and horiz < threshold:
            danger_points.append((angle_norm, dist, horiz, fwd))

    if not danger_points:
        return FORWARD_SPEED, 0.0, False   # 장애물 없음 → 직진

    # ── 2. 가장 가까운 장애물 ────────────────────────────────────────────────
    nearest                              = min(danger_points, key=lambda p: p[1])
    nearest_angle, ref_dist, n_horiz, n_fwd = nearest

    print(f"  [기준장애물] 각도:{nearest_angle:.1f}°  "
          f"직선:{ref_dist:.0f}mm  전방:{n_fwd:.0f}mm  수평:{n_horiz:.0f}mm")

    # ── 3. 긴급 회피구역 판정 ────────────────────────────────────────────────
    in_emergency = (n_fwd <= EMERGENCY_FWD_RANGE and n_horiz <= EMERGENCY_HORIZ_RANGE)

    if in_emergency:
        ratio       = n_fwd / EMERGENCY_FWD_RANGE
        min_scale   = EMERGENCY_MIN_SPEED / FORWARD_SPEED
        speed_scale = min_scale + (1.0 - min_scale) * ratio
        v           = FORWARD_SPEED * speed_scale
        print(f"  [긴급회피] 전방:{n_fwd:.0f}mm  스케일:{speed_scale:.2f}  v={v:.2f}m/s")
    else:
        speed_scale = 1.0
        v           = FORWARD_SPEED

    # ── 4. 좌우 여유공간 계산 ────────────────────────────────────────────────
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

    # ── 5. 회피각 계산 (수평·수직 이동 기반 atan2) ──────────────────────────
    #   delta_horiz = threshold - n_horiz + HORIZ_EXTRA
    #     (실제 추가 필요 수평 이동량 + 여유 20mm)
    #   avoid_angle = atan2(delta_horiz, n_fwd)
    #     (이 각도만큼 꺾으면 obstacle을 여유 있게 통과)
    delta_horiz = threshold - n_horiz + HORIZ_EXTRA
    avoid_angle = math.atan2(max(delta_horiz, 1.0), max(n_fwd, 1.0))

    # ── 6. 회피 방향 결정 (헤딩 우선) ───────────────────────────────────────
    w_sign = select_direction(left_clear, right_clear, heading_deg)
    dir_str = "좌회전" if w_sign > 0 else "오른쪽"

    # ── 7. 각속도 계산 + 긴급 시 전반 감속 ──────────────────────────────────
    w = w_sign * min(W_GAIN * avoid_angle, MAX_W)
    w *= speed_scale   # 긴급 시 v와 동일 비율로 w도 감소

    # ── 8. 막힘 조건 판단 ────────────────────────────────────────────────────
    #   헤딩 한계에 근접 + 장애물 있음 → 막힘 가능성
    is_stuck_cond = abs(heading_deg) > HEADING_CAUTION_DEG

    print(f"  [회피명령] {dir_str}  "
          f"회피각:{math.degrees(avoid_angle):.1f}°  "
          f"v:{v:.2f}m/s  w:{w:.2f}rad/s"
          + ("  [긴급]" if in_emergency else "")
          + ("  [헤딩한계]" if is_stuck_cond else ""))

    return v, w, is_stuck_cond


def execute_180_turn(arduino, lidar, all_scan_points):
    """
    180도 회전 실행

    1. 전체 360도 스캔으로 안전 방향 결정
    2. T180L 또는 T180R 명령 전송
    3. DONE:T180 신호 수신 또는 타임아웃 대기
    4. 완료 후 정상 모드 복귀
    """
    print("\n" + "="*50)
    print("[STUCK] 진행 불가 감지 → 180도 회전 시작")

    # 안전 방향 결정
    turn_dir = find_safe_180_direction(all_scan_points)
    cmd      = f"T180{turn_dir}\n"
    arduino.write(cmd.encode())
    print(f"[T180] {turn_dir} 방향 180도 회전 명령 전송")

    # 완료 대기 (DONE:T180 수신 또는 타임아웃)
    t_start = time.time()
    while time.time() - t_start < TURN180_TIMEOUT:
        msg = read_arduino(arduino)
        if msg == "DONE:T180":
            print("[T180] 180도 회전 완료 신호 수신")
            break
        time.sleep(0.05)
    else:
        print("[T180] 타임아웃 → 강제 복귀")
        arduino.write(b"0.00 0.00\n")

    print("="*50 + "\n")


def main():
    global stuck_since, arduino_heading_deg

    print("=== RPLIDAR 장애물 회피 v4 ===")
    print(f"  라이다 포트    : {LIDAR_PORT}")
    print(f"  아두이노 포트  : {ARDUINO_PORT}  (UART GPIO14/15)")
    print(f"  전진 속도      : {FORWARD_SPEED} m/s")
    print(f"  충돌 기준      : {ROBOT_HALF_WIDTH + SAFETY_MARGIN} mm (수평)")
    print(f"  일반 위험구역  : 전방 {FORWARD_RANGE}mm × 수평 {ROBOT_HALF_WIDTH+SAFETY_MARGIN}mm")
    print(f"  긴급 회피구역  : 전방 {EMERGENCY_FWD_RANGE}mm × 수평 {EMERGENCY_HORIZ_RANGE}mm")
    print(f"  헤딩 우선 전환 : ±{HEADING_CAUTION_DEG}° 초과 시")
    print(f"  막힘 판단 시간 : {STUCK_TIMEOUT}초")
    print("=" * 48)

    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)

    lidar.write(bytes([0xA5, 0x40]))    # RESET
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))    # START SCAN
    print("스캔 시작...")
    lidar.read(7)

    scan_points     = []            # 현재 스캔 버퍼 (360도 전체)
    last_send       = time.time()
    last_cmd_str    = ""

    try:
        while True:
            # 아두이노 데이터 수신 (헤딩 업데이트)
            read_arduino(arduino)

            raw    = lidar.read(5)
            result = parse_packet(raw)
            if result is None:
                continue

            angle_raw, distance, quality = result
            s_flag = raw[0] & 0x01

            if s_flag == 1 and scan_points:
                # 전체 스캔 스냅샷 저장 (180도 회전 방향 결정에 사용)
                all_scan_points = list(scan_points)

                # 정면 180도 포인트만 추출
                front_points = [
                    (a, d) for a, d in scan_points
                    if is_in_front(a) and d > 0
                ]

                now = time.time()
                if now - last_send >= SEND_INTERVAL:

                    v, w, is_stuck_cond = find_vw_command(
                        front_points, arduino_heading_deg
                    )

                    # ── 막힘 감지 및 180도 회전 ───────────────────────────
                    if is_stuck_cond:
                        if stuck_since is None:
                            stuck_since = now
                            print(f"  [막힘시작] 헤딩:{arduino_heading_deg:.1f}° 막힘 카운터 시작")
                        elif now - stuck_since >= STUCK_TIMEOUT:
                            # 180도 회전 실행
                            execute_180_turn(arduino, lidar, all_scan_points)
                            stuck_since = None
                            last_cmd_str = ""
                            last_send = time.time()
                            scan_points = []
                            continue
                    else:
                        if stuck_since is not None:
                            print(f"  [막힘해제] 카운터 리셋")
                        stuck_since = None

                    # ── 정상 명령 전송 ────────────────────────────────────
                    cmd = f"{v:.2f} {w:.2f}\n"
                    arduino.write(cmd.encode())

                    if cmd != last_cmd_str:
                        print(f"[전송] v={v:.2f}m/s  w={w:.2f}rad/s  "
                              f"헤딩={arduino_heading_deg:.1f}°  "
                              f"(포인트:{len(front_points)})")
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

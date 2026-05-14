import serial
import time
import math

# ── 1. 설정 및 파라미터 ───────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 9600

# 로봇 하드웨어 (mm)
ROBOT_FRONT  = 110      # 앞
ROBOT_BACK   = 150      # 뒤 (엉덩이 충돌 방지)
ROBOT_HALF_W = 110      # 좌우
MARGIN       = 30       # 안전 여유폭 (35→30으로 줄임)

# 주행 성능
MAX_V = 0.35           # m/s
MAX_W = 1.5            # rad/s

# DWA 채점 가중치
W_HEADING   = 2.0
W_CLEARANCE = 1.8
W_VELOCITY  = 1.0
BIAS_BONUS  = 0.3

# ── 2. FSM 및 전역 상태 ───────────────────────────────────────────────────────
class RobotState:
    DRIVE    = 1
    RECOVERY = 2

current_state        = RobotState.DRIVE
stuck_timer          = 0.0
last_w_sign          = 0.0
arduino_heading_deg  = 0.0

# ── 3. 유틸리티 및 라이다 파싱 ────────────────────────────────────────────────
def normalize_angle(angle):
    while angle >  180: angle -= 360
    while angle < -180: angle += 360
    return angle


def parse_packet(data):
    if len(data) != 5:
        return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        return None
    if (data[1] & 0x01) != 1:
        return None
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    angle       = angle_q6 / 64.0
    distance_q2 = data[3] | (data[4] << 8)
    distance    = distance_q2 / 4.0
    return angle, distance


def read_arduino(arduino):
    global arduino_heading_deg
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'):
                arduino_heading_deg = float(line[2:])
        except Exception:
            pass

# ── 4. DWA 코어 ───────────────────────────────────────────────────────────────
def generate_vw_window(current_v, current_w):
    # ★ 수정: 후보 더 촘촘하게, 저속 전진 추가
    v_cands = [0.0, 0.10, 0.15, 0.20, 0.25, MAX_V]
    w_cands = [-MAX_W, -1.2, -0.8, -0.5, -0.3, 0.0,
                0.3,    0.5,  0.8,  1.2,  MAX_W]
    return v_cands, w_cands


def check_collision_and_clearance(v_m_s, w_rad_s, scan_points,
                                   predict_t=0.6,   # ★ 수정: 1.0 → 0.6초
                                   step=0.1):        # ★ 수정: 0.2 → 0.1초 (정밀도↑)
    v_mm_s   = v_m_s * 1000.0
    max_dist = 1200.0

    local_pts = [
        ( dist * math.cos(math.radians(ang)),
         -dist * math.sin(math.radians(ang)))
        for ang, dist in scan_points if 0 < dist <= max_dist
    ]
    if not local_pts:
        return 1000.0

    curr_x, curr_y, curr_th = 0.0, 0.0, 0.0
    t            = 0.0
    min_clear_sq = 1000000.0

    front_bound =  ROBOT_FRONT  + MARGIN
    back_bound  = -(ROBOT_BACK  + MARGIN)
    side_bound  =  ROBOT_HALF_W + MARGIN

    while t <= predict_t:
        curr_x  += v_mm_s * math.cos(curr_th) * step
        curr_y  += v_mm_s * math.sin(curr_th) * step
        curr_th += w_rad_s * step
        t       += step

        cos_t = math.cos(-curr_th)
        sin_t = math.sin(-curr_th)

        for px, py in local_pts:
            dx, dy = px - curr_x, py - curr_y
            lx =  dx * cos_t - dy * sin_t
            ly =  dx * sin_t + dy * cos_t

            if back_bound <= lx <= front_bound and -side_bound <= ly <= side_bound:
                return -1.0  # 충돌 궤적

            dist_sq = dx**2 + dy**2
            if dist_sq < min_clear_sq:
                min_clear_sq = dist_sq

    return math.sqrt(min_clear_sq)


def run_dwa(scan_points, curr_heading, current_v, current_w):
    global last_w_sign
    v_cands, w_cands = generate_vw_window(current_v, current_w)

    best_v, best_w = 0.0, 0.0
    max_score      = -1.0

    for v in v_cands:
        for w in w_cands:
            clearance = check_collision_and_clearance(v, w, scan_points)
            if clearance <= 0:
                continue

            pred_turn   = math.degrees(w * 1.0)
            fut_heading = normalize_angle(curr_heading - pred_turn)

            score_heading   = max(0.0, 1.0 - (abs(fut_heading) / 180.0))
            score_clearance = min(1.0, clearance / 1000.0)
            score_velocity  = max(0.0, v / MAX_V)

            bias = BIAS_BONUS if (w * last_w_sign > 0) else 0.0

            # ★ 수정: v=0일 때 페널티 → 웬만하면 정지 선택 안 함
            stop_penalty = -0.5 if (v == 0.0) else 0.0

            total_score = (W_HEADING   * score_heading   +
                           W_CLEARANCE * score_clearance +
                           W_VELOCITY  * score_velocity  +
                           bias + stop_penalty)

            if total_score > max_score:
                max_score      = total_score
                best_v, best_w = v, w

    # ★ 수정: 모든 전진이 막혔을 때 제자리 회전으로 fallback
    if best_v == 0.0 and best_w == 0.0:
        best_w = MAX_W if curr_heading <= 0 else -MAX_W

    if best_w != 0:
        last_w_sign = 1.0 if best_w > 0 else -1.0

    return best_v, best_w

# ── 5. 메인 루프 ──────────────────────────────────────────────────────────────
def main():
    global current_state, stuck_timer, arduino_heading_deg

    print("라이다 및 아두이노 초기화 중...")
    try:
        lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    except Exception as e:
        print(f"[에러] 포트 연결 실패: {e}")
        return

    # 라이다 스캔 시작 명령
    lidar.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))
    lidar.read(7)   # 응답 헤더 무시
    print("주행 시작!")

    scan_points   = []
    last_send     = time.time()
    SEND_INTERVAL = 0.1     # 0.1초마다 갱신
    current_v, current_w = 0.0, 0.0

    try:
        while True:
            # 1. 아두이노 헤딩 읽기
            read_arduino(arduino)

            # 2. 라이다 데이터 읽기
            raw    = lidar.read(5)
            result = parse_packet(raw)
            if result is None:
                continue

            angle_raw, distance = result
            s_flag = raw[0] & 0x01

            # 150mm 이하는 로봇 본체 노이즈 → 무시
            if distance > 150:
                scan_points.append((normalize_angle(angle_raw), distance))

            # 3. 한 프레임 완료 + 전송 주기 도래 시 연산
            now = time.time()
            if s_flag == 1 and scan_points and (now - last_send >= SEND_INTERVAL):

                # ── FSM ──────────────────────────────────────────────────────
                if current_state == RobotState.DRIVE:
                    v, w = run_dwa(scan_points, arduino_heading_deg,
                                   current_v, current_w)

                    # ★ 수정: v < 0.1 이하(저속 포함)를 stuck으로 판정
                    if v < 0.1:
                        stuck_timer += SEND_INTERVAL
                        if stuck_timer >= 2.0:
                            print("[상태 전환] 갇힘 감지! Recovery 모드 진입")
                            arduino.write(b"ESC\n")
                            current_state = RobotState.RECOVERY
                            stuck_timer   = 0.0
                    else:
                        stuck_timer = 0.0

                elif current_state == RobotState.RECOVERY:
                    # ★ 수정: 더 강한 후진+회전, 시간 2.5초로 늘림
                    v, w = -0.15, MAX_W
                    stuck_timer += SEND_INTERVAL

                    if stuck_timer >= 2.5:
                        print("[상태 전환] 탈출 완료. Drive 모드 복귀")
                        current_state = RobotState.DRIVE
                        stuck_timer   = 0.0

                # ── 모터 제어 명령 전송 ───────────────────────────────────
                current_v, current_w = v, w
                cmd = f"{v:.2f} {w:.2f}\n"
                arduino.write(cmd.encode('utf-8'))

                scan_points = []    # 스캔 데이터 초기화
                last_send   = now

    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        lidar.write(bytes([0xA5, 0x25]))    # 라이다 정지
        arduino.write(b"0.00 0.00\n")       # 모터 정지
        lidar.close()
        arduino.close()
        print("종료 완료.")


if __name__ == "__main__":
    main()

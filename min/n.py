import serial
import time
import math

# ── 1. 설정 및 파라미터 ───────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

# 로봇 하드웨어 (mm)
ROBOT_FRONT  = 110
ROBOT_BACK   = 150
ROBOT_HALF_W = 110
MARGIN       = 40       # 안전 여유폭 (조금 더 보수적으로)

# 주행 성능
MAX_V = 0.35           # m/s
MAX_W = 1.5            # rad/s

# DWA 채점 가중치
W_HEADING   = 1.5      # 헤딩 가중치 약화 (직진 편향 줄임)
W_CLEARANCE = 2.5      # 안전거리 가중치 강화
W_VELOCITY  = 0.8      # 속도 가중치 약화 (돌진 억제)
BIAS_BONUS  = 0.2

# 시뮬레이션 파라미터
PREDICT_T = 1.2        # ★ 핵심: 1.2초 예측 (v=MAX_V로 420mm + 박스 = 약 560mm 시야)
SIM_STEP  = 0.1

# ── 2. FSM 및 전역 상태 ───────────────────────────────────────────────────────
class RobotState:
    DRIVE    = 1
    RECOVERY = 2

current_state       = RobotState.DRIVE
stuck_timer         = 0.0
last_w_sign         = 0.0
arduino_heading_deg = 0.0

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
def generate_vw_window():
    # v 후보: 정지 + 다양한 전진속도
    v_cands = [0.0, 0.10, 0.18, 0.25, MAX_V]
    # w 후보: 회전 각도 다양화 (특히 큰 회전 강화)
    w_cands = [-MAX_W, -1.2, -0.9, -0.6, -0.3, 0.0,
                0.3,    0.6,  0.9,  1.2,  MAX_W]
    return v_cands, w_cands


def check_collision_and_clearance(v_m_s, w_rad_s, local_pts,
                                   predict_t=PREDICT_T,
                                   step=SIM_STEP):
    """
    좌표계: 앞=+X, 왼쪽=+Y, 반시계회전=+θ (표준 로봇 좌표계)
    
    local_pts: t=0 시점 로봇 기준 장애물 좌표 [(px, py), ...]
    
    수식 (t 시점):
        로봇 자세: (rx, ry, θ) — 적분으로 계산
        장애물의 로봇 로컬 좌표:
            lx =  cos(θ)·(px-rx) + sin(θ)·(py-ry)
            ly = -sin(θ)·(px-rx) + cos(θ)·(py-ry)
        충돌 판정:
            -(BACK+M) ≤ lx ≤ (FRONT+M)  AND  |ly| ≤ (HALF_W+M)
    """
    v_mm_s = v_m_s * 1000.0

    if not local_pts:
        return 1000.0

    curr_x, curr_y, curr_th = 0.0, 0.0, 0.0
    t            = 0.0
    min_clear_sq = 1e12

    front_bound =  ROBOT_FRONT  + MARGIN
    back_bound  = -(ROBOT_BACK  + MARGIN)
    side_bound  =  ROBOT_HALF_W + MARGIN

    # 시간 적분 (Euler)
    n_steps = int(predict_t / step) + 1
    for _ in range(n_steps):
        # 자세 업데이트
        curr_x  += v_mm_s * math.cos(curr_th) * step
        curr_y  += v_mm_s * math.sin(curr_th) * step
        curr_th += w_rad_s * step
        t       += step

        cos_t = math.cos(curr_th)
        sin_t = math.sin(curr_th)

        for px, py in local_pts:
            dx = px - curr_x
            dy = py - curr_y

            # 월드(t=0 로봇기준) → 현재 로봇 자세 로컬 변환: R(-θ) · (p - r)
            lx =  cos_t * dx + sin_t * dy
            ly = -sin_t * dx + cos_t * dy

            # 충돌 판정 (로봇 박스 내부)
            if back_bound <= lx <= front_bound and -side_bound <= ly <= side_bound:
                return -1.0

            # 최소 거리 추적 (clearance)
            dist_sq = dx * dx + dy * dy
            if dist_sq < min_clear_sq:
                min_clear_sq = dist_sq

    return math.sqrt(min_clear_sq)


def run_dwa(scan_points, curr_heading):
    global last_w_sign

    # 라이다 포인트를 로봇 로컬 좌표로 한 번만 변환
    # 라이다 각도 규약: 0°=정면, 시계방향 양수
    # 로봇 좌표(앞=+X, 왼쪽=+Y)로 변환: x=d·cos(a), y=-d·sin(a)
    local_pts = []
    for ang, dist in scan_points:
        if 0 < dist <= 1500:   # 1.5m 이내만 (시야 제한)
            rad = math.radians(ang)
            local_pts.append((
                 dist * math.cos(rad),
                -dist * math.sin(rad)
            ))

    v_cands, w_cands = generate_vw_window()

    best_v, best_w = 0.0, 0.0
    max_score      = -1e9

    for v in v_cands:
        for w in w_cands:
            clearance = check_collision_and_clearance(v, w, local_pts)
            if clearance <= 0:
                continue   # 충돌 궤적 폐기

            # 헤딩 점수: w가 헤딩을 줄이는 방향이면 가점
            # (아두이노 헤딩 정의에 맞춰 부호 유지: heading - pred_turn)
            pred_turn   = math.degrees(w * 1.0)
            fut_heading = normalize_angle(curr_heading - pred_turn)
            score_heading = max(0.0, 1.0 - abs(fut_heading) / 180.0)

            # Clearance 점수: 더 멀수록 좋음 (1m 기준 정규화)
            score_clearance = min(1.0, clearance / 1000.0)

            # 속도 점수
            score_velocity = max(0.0, v / MAX_V)

            # 일관된 회전 방향 보너스
            bias = BIAS_BONUS if (w != 0 and w * last_w_sign > 0) else 0.0

            # 정지 패널티
            stop_penalty = -0.8 if (v == 0.0) else 0.0

            total = (W_HEADING   * score_heading   +
                     W_CLEARANCE * score_clearance +
                     W_VELOCITY  * score_velocity  +
                     bias + stop_penalty)

            if total > max_score:
                max_score      = total
                best_v, best_w = v, w

    # Fallback: 모든 궤적이 충돌이면 제자리 회전
    if max_score < -1e8:
        best_v = 0.0
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

    lidar.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))
    lidar.read(7)
    print("주행 시작!")

    scan_points   = []
    last_send     = time.time()
    SEND_INTERVAL = 0.1
    current_v, current_w = 0.0, 0.0

    try:
        while True:
            read_arduino(arduino)

            raw    = lidar.read(5)
            result = parse_packet(raw)
            if result is None:
                continue

            angle_raw, distance = result
            s_flag = raw[0] & 0x01

            if distance > 150:
                scan_points.append((normalize_angle(angle_raw), distance))

            now = time.time()
            if s_flag == 1 and scan_points and (now - last_send >= SEND_INTERVAL):

                if current_state == RobotState.DRIVE:
                    v, w = run_dwa(scan_points, arduino_heading_deg)

                    if v < 0.05:
                        stuck_timer += SEND_INTERVAL
                        if stuck_timer >= 2.0:
                            print("[상태 전환] 갇힘 감지 → Recovery")
                            arduino.write(b"ESC\n")
                            current_state = RobotState.RECOVERY
                            stuck_timer   = 0.0
                    else:
                        stuck_timer = 0.0

                elif current_state == RobotState.RECOVERY:
                    v, w = -0.15, MAX_W
                    stuck_timer += SEND_INTERVAL
                    if stuck_timer >= 2.5:
                        print("[상태 전환] Recovery 종료 → Drive")
                        current_state = RobotState.DRIVE
                        stuck_timer   = 0.0

                current_v, current_w = v, w
                cmd = f"{v:.2f} {w:.2f}\n"
                arduino.write(cmd.encode('utf-8'))

                print(f"[CMD] v={v:.2f} w={w:.2f} hdg={arduino_heading_deg:.1f}")

                scan_points = []
                last_send   = now

    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        lidar.write(bytes([0xA5, 0x25]))
        arduino.write(b"0.00 0.00\n")
        lidar.close()
        arduino.close()
        print("종료 완료.")


if __name__ == "__main__":
    main()

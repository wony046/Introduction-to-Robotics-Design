"""
장애물 회피 DWA - 수식 엄밀 검증 버전

좌표계:
  세계(World): +X=정면, +Y=좌측, +θ=반시계 (오른손 법칙)
  
라이다(RPLIDAR):
  0°=정면, 시계방향(CW) 증가
  변환: p_x = d·cos(α), p_y = -d·sin(α)
  
아두이노 heading:
  heading += (dsR - dsL)/WHEEL_BASE
  → 우바퀴 빠름 = 좌회전 = +θ (반시계)
  → 우리 좌표계의 +θ와 일치
  
로봇 모델: Unicycle
  ẋ = v·cos(θ), ẏ = v·sin(θ), θ̇ = ω
"""

import serial
import time
import math

# ── 1. 설정 ───────────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

# 로봇 하드웨어 (mm)
ROBOT_FRONT  = 110
ROBOT_BACK   = 150
ROBOT_HALF_W = 110
MARGIN       = 70

# 주행 성능
MAX_V = 0.22           # m/s
MAX_W = 1.5            # rad/s

# DWA 가중치 (검증된 수식 기준)
W_HEADING   = 1.0
W_CLEARANCE = 3.5
W_VELOCITY  = 0.7

# 진동 방지
W_SMOOTH_ALPHA = 0.5
DEAD_BAND_W    = 0.15
BIAS_BONUS     = 0.8

# 비상 안전
EMERGENCY_FRONT_MM = 280
EMERGENCY_SIDE_MM  = 110
FRONT_ANGLE_RANGE  = 50

# 시뮬레이션
PREDICT_T = 1.2
SIM_STEP  = 0.08

# ── 2. FSM ────────────────────────────────────────────────────────────────────
class RobotState:
    DRIVE     = 1
    RECOVERY  = 2
    EMERGENCY = 3

current_state       = RobotState.DRIVE
stuck_timer         = 0.0
emergency_timer     = 0.0
arduino_heading_deg = 0.0

last_w_sign    = 0
last_w_command = 0.0

# ── 3. 라이다 ─────────────────────────────────────────────────────────────────
def normalize_angle(angle):
    """각도를 [-180, +180] 범위로"""
    while angle >  180: angle -= 360
    while angle < -180: angle += 360
    return angle


def parse_packet(data):
    if len(data) != 5: return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    angle       = angle_q6 / 64.0
    distance_q2 = data[3] | (data[4] << 8)
    distance    = distance_q2 / 4.0
    return angle, distance, s_flag


def find_lidar_sync(lidar):
    print("[라이다] 동기화...", flush=True)
    timeout_start = time.time()
    while time.time() - timeout_start < 3.0:
        b = lidar.read(1)
        if len(b) == 0: continue
        s_flag = b[0] & 0x01
        s_inv  = (b[0] & 0x02) >> 1
        if s_inv == (1 - s_flag):
            b2 = lidar.read(1)
            if len(b2) == 0: continue
            if (b2[0] & 0x01) == 1:
                b3 = lidar.read(3)
                if len(b3) == 3:
                    print("[라이다] OK", flush=True)
                    return True
    return False


def read_arduino(arduino):
    global arduino_heading_deg
    try:
        while arduino.in_waiting > 0:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'):
                try: arduino_heading_deg = float(line[2:])
                except: pass
    except: pass


# ── 4. 좌표 변환 (검증된 수식) ────────────────────────────────────────────────
def lidar_to_robot(angle_deg, distance):
    """
    RPLIDAR 좌표 → 로봇 좌표
    
    수식:
        p_x = d · cos(α)      [정면이 양수]
        p_y = -d · sin(α)     [좌측이 양수, 라이다 CW를 CCW로 뒤집음]
    """
    rad = math.radians(angle_deg)
    px = distance * math.cos(rad)
    py = -distance * math.sin(rad)
    return px, py


# ── 5. 근접 분석 ──────────────────────────────────────────────────────────────
def analyze_proximity(scan_points):
    """
    영역별 최소 거리 분석 (로봇 로컬 좌표 기준)
    
    영역 정의 (각도 a는 normalize_angle 통과 후, -180~+180):
        정면: |a| <= FRONT_ANGLE_RANGE
        좌측: -90 <= a < -FRONT_ANGLE_RANGE  (라이다 CW이므로 음수가 좌측)
        우측: +FRONT_ANGLE_RANGE < a <= +90
    
    ★ 라이다 0°에서 시계방향 증가 → 라이다 30°는 로봇의 우측
       따라서 음수 각도(=라이다 330°)가 로봇 좌측이 맞음
    """
    front_min = 99999
    left_min  = 99999
    right_min = 99999
    front_ang = 0

    for ang, dist in scan_points:
        if dist <= 150 or dist > 2000:
            continue
        a = normalize_angle(ang)
        
        if abs(a) <= FRONT_ANGLE_RANGE:
            if dist < front_min:
                front_min = dist
                front_ang = a
        elif -90 <= a < -FRONT_ANGLE_RANGE:
            # 라이다 각도 -60° = 라이다 300° (CW 측정) = 로봇 좌측
            if dist < left_min:
                left_min = dist
        elif FRONT_ANGLE_RANGE < a <= 90:
            # 라이다 각도 +60° (CW 측정) = 로봇 우측
            if dist < right_min:
                right_min = dist

    return {
        'front': front_min,
        'left':  left_min,
        'right': right_min,
        'front_ang': front_ang
    }


def emergency_check(prox):
    """비상 회피 명령 생성"""
    if prox['front'] < EMERGENCY_FRONT_MM:
        # 정면 막힘 → 여유 있는 쪽으로 후진+회전
        # ω > 0 = 좌회전 (반시계)
        if prox['left'] > prox['right']:
            return -0.10, +MAX_W, f"F={prox['front']:.0f}→L"
        else:
            return -0.10, -MAX_W, f"F={prox['front']:.0f}→R"

    if prox['left'] < EMERGENCY_SIDE_MM:
        # 좌측 벽 → 우회전 (ω < 0)
        return 0.08, -MAX_W * 0.8, f"L={prox['left']:.0f}→R회전"

    if prox['right'] < EMERGENCY_SIDE_MM:
        # 우측 벽 → 좌회전 (ω > 0)
        return 0.08, +MAX_W * 0.8, f"R={prox['right']:.0f}→L회전"

    return None, None, None


# ── 6. 충돌 체크 (검증된 수식) ────────────────────────────────────────────────
def check_collision_and_clearance(v_m_s, w_rad_s, local_pts,
                                   predict_t=PREDICT_T, step=SIM_STEP):
    """
    유니사이클 모델로 (v, ω) 궤적 시뮬레이션
    
    수식:
        x_{k+1} = x_k + v·cos(θ_k)·Δt   [v 단위는 mm/s]
        y_{k+1} = y_k + v·sin(θ_k)·Δt
        θ_{k+1} = θ_k + ω·Δt
    
    충돌 판정 (장애물을 현재 로봇 자세 기준 로컬 좌표로 역변환):
        l_x =  cos(θ)·(p_x - r_x) + sin(θ)·(p_y - r_y)
        l_y = -sin(θ)·(p_x - r_x) + cos(θ)·(p_y - r_y)
        
        충돌 if  -BACK_M ≤ l_x ≤ FRONT_M  AND  |l_y| ≤ SIDE_M
    """
    v_mm_s = v_m_s * 1000.0
    if not local_pts:
        return 1000.0

    rx, ry, theta = 0.0, 0.0, 0.0
    min_clear_sq = 1e12

    front_M = ROBOT_FRONT  + MARGIN
    back_M  = ROBOT_BACK   + MARGIN
    side_M  = ROBOT_HALF_W + MARGIN

    n_steps = max(1, int(predict_t / step))
    for _ in range(n_steps):
        # 자세 적분
        rx    += v_mm_s * math.cos(theta) * step
        ry    += v_mm_s * math.sin(theta) * step
        theta += w_rad_s * step

        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        for px, py in local_pts:
            dx = px - rx
            dy = py - ry
            
            # 역변환 R(-θ): 장애물을 로봇 로컬로
            lx =  cos_t * dx + sin_t * dy
            ly = -sin_t * dx + cos_t * dy

            # 충돌 박스 체크
            if -back_M <= lx <= front_M and -side_M <= ly <= side_M:
                return -1.0

            # 최소 거리 (점수용)
            dist_sq = dx*dx + dy*dy
            if dist_sq < min_clear_sq:
                min_clear_sq = dist_sq

    return math.sqrt(min_clear_sq)


# ── 7. DWA 메인 ───────────────────────────────────────────────────────────────
def generate_vw_window():
    v_cands = [0.0, 0.08, 0.15, MAX_V]
    w_cands = [-MAX_W, -1.0, -0.6, 0.0, +0.6, +1.0, +MAX_W]
    return v_cands, w_cands


def run_dwa(scan_points, curr_heading_deg):
    """
    점수 함수 (검증된 부호):
        헤딩 점수: 미래 헤딩이 0에 가까울수록 좋음
            fut_heading = curr_heading + pred_turn  ★ (+가 맞음)
            pred_turn = deg(ω · PREDICT_T)
            
        안전거리: clearance/1000 [0~1]
        속도: v/MAX_V [0~1]
        일관성 보너스: 같은 방향 회전 유지 시 +BIAS
    """
    global last_w_sign, last_w_command

    # 라이다 → 로봇 로컬 (검증된 변환)
    local_pts = []
    for ang, dist in scan_points:
        if 0 < dist <= 1500:
            px, py = lidar_to_robot(ang, dist)
            local_pts.append((px, py))

    if len(local_pts) < 5:
        return 0.0, 0.0, 0

    v_cands, w_cands = generate_vw_window()
    best_v, best_w = 0.0, 0.0
    max_score      = -1e9
    num_safe       = 0

    for v in v_cands:
        for w in w_cands:
            clearance = check_collision_and_clearance(v, w, local_pts)
            if clearance < 0:
                continue
            num_safe += 1

            # 1. 헤딩 점수
            # ω(rad/s) × PREDICT_T(s) → 회전 라디안
            # 헤딩 부호: +θ = 반시계 = 좌회전 = 헤딩 deg 증가
            pred_turn_deg = math.degrees(w * PREDICT_T)
            fut_heading = normalize_angle(curr_heading_deg + pred_turn_deg)
            # 헤딩 0에서 멀수록 감점
            score_heading = max(0.0, 1.0 - abs(fut_heading) / 180.0)

            # 2. 안전거리 점수
            score_clearance = min(1.0, clearance / 1000.0)

            # 3. 속도 점수
            score_velocity = v / MAX_V if v > 0 else 0.0

            # 4. 일관성 보너스
            bias = 0.0
            if w != 0 and last_w_sign != 0 and (w * last_w_sign > 0):
                bias = BIAS_BONUS

            # 5. 정지 패널티
            stop_pen = -0.8 if v < 0.01 else 0.0

            total = (W_HEADING   * score_heading +
                     W_CLEARANCE * score_clearance +
                     W_VELOCITY  * score_velocity +
                     bias + stop_pen)

            if total > max_score:
                max_score = total
                best_v, best_w = v, w

    # Fallback
    if max_score <= -1e8:
        best_v = 0.0
        # 헤딩이 +면 (좌측 기울어짐) → 우회전(ω<0)으로 복귀
        # 헤딩이 -면 (우측 기울어짐) → 좌회전(ω>0)으로 복귀
        best_w = -MAX_W if curr_heading_deg > 0 else +MAX_W

    # 데드밴드
    if abs(best_w) < DEAD_BAND_W:
        best_w = 0.0

    # 저역통과 필터
    smoothed_w = W_SMOOTH_ALPHA * last_w_command + (1 - W_SMOOTH_ALPHA) * best_w

    if best_w != 0:
        last_w_sign = 1 if best_w > 0 else -1

    last_w_command = smoothed_w
    return best_v, smoothed_w, num_safe


# ── 8. 메인 루프 ──────────────────────────────────────────────────────────────
def main():
    global current_state, stuck_timer, emergency_timer
    global arduino_heading_deg, last_w_command

    print("="*60, flush=True)
    print("DWA 장애물 회피 (수식 검증 버전)", flush=True)
    print("="*60, flush=True)

    try:
        lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
        time.sleep(0.5)
        print("✓ 라이다", flush=True)
    except Exception as e:
        print(f"✗ 라이다: {e}", flush=True)
        return

    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=0.1)
        time.sleep(0.5)
        print("✓ 아두이노", flush=True)
    except Exception as e:
        print(f"✗ 아두이노: {e}", flush=True)
        lidar.close()
        return

    arduino.write(b"0.00 0.00\n")
    time.sleep(0.2)

    try:
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        lidar.reset_input_buffer()
        lidar.write(bytes([0xA5, 0x20]))
        time.sleep(0.5)
        lidar.read(7)
    except Exception as e:
        print(f"⚠ 라이다: {e}", flush=True)

    if not find_lidar_sync(lidar):
        arduino.write(b"0.00 0.00\n")
        lidar.close()
        arduino.close()
        return

    print("\n" + "="*60, flush=True)
    print("주행 시작!", flush=True)
    print("="*60, flush=True)

    scan_points = []
    last_send = time.time()
    SEND_INTERVAL = 0.1
    last_cmd_time = time.time()
    dwa_count = 0

    try:
        while True:
            read_arduino(arduino)

            raw = lidar.read(5)
            if len(raw) == 5:
                result = parse_packet(raw)
                if result is not None:
                    angle_raw, distance, s_flag = result
                    if distance > 150:
                        scan_points.append((normalize_angle(angle_raw), distance))

                    now = time.time()
                    if s_flag == 1 and scan_points and (now - last_send >= SEND_INTERVAL):
                        dwa_count += 1
                        last_cmd_time = now

                        prox = analyze_proximity(scan_points)
                        em_v, em_w, em_reason = emergency_check(prox)

                        if em_v is not None:
                            v, w = em_v, em_w
                            num_safe = -1
                            current_state = RobotState.EMERGENCY
                            emergency_timer = 0.3
                            last_w_command = w
                            print(f"[!비상!] {em_reason}", flush=True)

                        elif current_state == RobotState.EMERGENCY:
                            emergency_timer -= SEND_INTERVAL
                            if emergency_timer <= 0:
                                current_state = RobotState.DRIVE
                            v, w, num_safe = -0.08, last_w_command, 0

                        elif current_state == RobotState.DRIVE:
                            v, w, num_safe = run_dwa(scan_points, arduino_heading_deg)
                            if v < 0.05:
                                stuck_timer += SEND_INTERVAL
                                if stuck_timer >= 2.0:
                                    arduino.write(b"ESC\n")
                                    current_state = RobotState.RECOVERY
                                    stuck_timer = 0.0
                            else:
                                stuck_timer = 0.0
                        else:
                            v, w, num_safe = -0.15, MAX_W, 0
                            stuck_timer += SEND_INTERVAL
                            if stuck_timer >= 2.5:
                                current_state = RobotState.DRIVE
                                stuck_timer = 0.0

                        cmd = f"{v:.2f} {w:.2f}\n"
                        arduino.write(cmd.encode('utf-8'))

                        st = {1:"D",2:"R",3:"E"}.get(current_state, "?")
                        print(f"[{dwa_count:4d}] {st} v={v:+.2f} w={w:+.2f} "
                              f"hdg={arduino_heading_deg:+5.0f}° "
                              f"F={prox['front']:.0f} L={prox['left']:.0f} R={prox['right']:.0f} "
                              f"safe={num_safe}", flush=True)

                        scan_points = []
                        last_send = now

            now = time.time()
            if now - last_cmd_time > 1.0:
                arduino.write(b"0.00 0.00\n")
                last_cmd_time = now

    except KeyboardInterrupt:
        print("\n종료...", flush=True)
    except Exception as e:
        print(f"[에러] {e}", flush=True)
        import traceback
        traceback.print_exc()
    finally:
        try:
            arduino.write(b"0.00 0.00\n")
            time.sleep(0.1)
            lidar.write(bytes([0xA5, 0x25]))
        except: pass
        lidar.close()
        arduino.close()
        print("✓ 종료", flush=True)


if __name__ == "__main__":
    main()

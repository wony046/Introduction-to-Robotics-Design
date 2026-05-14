"""
장애물 회피 DWA - Pro Version (동적 윈도우, 복도 중앙화, 교차 회피, 방향 가중치)
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
MARGIN       = 50  # 좁은 곳을 위해 약간 타이트하게

# [우선순위 3] Dynamic Window 물리적 한계치 설정
MAX_V   = 0.22      # 최대 선속도 (m/s)
MAX_W   = 1.5       # 최대 각속도 (rad/s)
MAX_A_V = 0.6       # 최대 선가속도 (m/s^2)
MAX_A_W = 4.0       # 최대 각가속도 (rad/s^2)

# DWA 가중치 (고급화)
W_HEADING   = 1.0
W_CLEARANCE = 2.5   # 기본 가중치 (방향에 따라 유동적으로 변함)
W_VELOCITY  = 0.7
W_CENTERING = 1.2   # [우선순위 4] 복도 중앙 유지 가중치 추가

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

last_v_command = 0.0
last_w_command = 0.0
last_w_sign    = 0
recovery_count = 0  # [우선순위 2] Alternating Turn 카운터

# ── 3. 유틸리티 및 센서 함수 ────────────────────────────────────────────────
def normalize_angle(angle):
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

def lidar_to_robot(angle_deg, distance):
    rad = math.radians(angle_deg)
    px = distance * math.cos(rad)
    py = -distance * math.sin(rad)
    return px, py

def analyze_proximity(scan_points):
    front_min, left_min, right_min = 99999, 99999, 99999
    front_ang = 0
    for ang, dist in scan_points:
        if dist <= 150 or dist > 2000: continue
        a = normalize_angle(ang)
        if abs(a) <= FRONT_ANGLE_RANGE:
            if dist < front_min: front_min, front_ang = dist, a
        elif -90 <= a < -FRONT_ANGLE_RANGE:
            if dist < left_min: left_min = dist
        elif FRONT_ANGLE_RANGE < a <= 90:
            if dist < right_min: right_min = dist
    return {'front': front_min, 'left': left_min, 'right': right_min, 'front_ang': front_ang}

def emergency_check(prox):
    if prox['front'] < EMERGENCY_FRONT_MM:
        if prox['left'] > prox['right']: return -0.10, +MAX_W, f"F={prox['front']:.0f}→L"
        else: return -0.10, -MAX_W, f"F={prox['front']:.0f}→R"
    if prox['left'] < EMERGENCY_SIDE_MM: return 0.08, -MAX_W * 0.8, f"L={prox['left']:.0f}→R회전"
    if prox['right'] < EMERGENCY_SIDE_MM: return 0.08, +MAX_W * 0.8, f"R={prox['right']:.0f}→L회전"
    return None, None, None

# ── 4. DWA 핵심 알고리즘 ────────────────────────────────────────────────────

# [우선순위 3] Dynamic Window 생성 함수
def generate_dynamic_window(curr_v, curr_w, dt=0.1):
    # 가속도 한계 내에서만 도달 가능한 속도 계산
    v_min = max(0.0, curr_v - MAX_A_V * dt)
    v_max = min(MAX_V, curr_v + MAX_A_V * dt)
    w_min = max(-MAX_W, curr_w - MAX_A_W * dt)
    w_max = min(MAX_W, curr_w + MAX_A_W * dt)

    v_cands = [v_min, v_min + (v_max-v_min)*0.4, v_min + (v_max-v_min)*0.7, v_max] if v_max > v_min else [0.0]
    w_step = (w_max - w_min) / 6.0 if w_max > w_min else 0.0
    w_cands = [w_min + w_step * i for i in range(7)] if w_step > 0 else [0.0]
    return v_cands, w_cands


def check_collision_and_clearance(v_m_s, w_rad_s, local_pts, predict_t=PREDICT_T, step=SIM_STEP):
    v_mm_s = v_m_s * 1000.0
    if not local_pts: return 1000.0, 1000.0, 1000.0, False

    rx, ry, theta = 0.0, 0.0, 0.0
    min_clear_sq = 1e12
    min_left_sq  = 1e12  # [우선순위 4] 좌측 여유 공간
    min_right_sq = 1e12  # [우선순위 4] 우측 여유 공간
    is_front_obs = False # [우선순위 5] 정면 장애물 플래그

    front_M = ROBOT_FRONT  + MARGIN
    back_M  = ROBOT_BACK   + MARGIN
    side_M  = ROBOT_HALF_W + MARGIN

    n_steps = max(1, int(predict_t / step))
    for _ in range(n_steps):
        rx += v_mm_s * math.cos(theta) * step
        ry += v_mm_s * math.sin(theta) * step
        theta += w_rad_s * step
        cos_t, sin_t = math.cos(theta), math.sin(theta)

        for px, py in local_pts:
            dx, dy = px - rx, py - ry
            lx =  cos_t * dx + sin_t * dy
            ly = -sin_t * dx + cos_t * dy

            if -back_M <= lx <= front_M and -side_M <= ly <= side_M:
                return -1.0, 0.0, 0.0, False

            dist_sq = lx*lx + ly*ly
            if dist_sq < min_clear_sq:
                min_clear_sq = dist_sq
                # [우선순위 5] 가장 가까운 장애물이 로봇 정면(±25도 이내)에 있는지 판별
                angle_to_obs = math.degrees(math.atan2(ly, lx))
                is_front_obs = (abs(angle_to_obs) < 25.0 and lx > 0)

            # [우선순위 4] 좌/우측 분리 저장
            if ly > 0 and dist_sq < min_left_sq: min_left_sq = dist_sq
            elif ly < 0 and dist_sq < min_right_sq: min_right_sq = dist_sq

    return math.sqrt(min_clear_sq), math.sqrt(min_left_sq), math.sqrt(min_right_sq), is_front_obs


def run_dwa(scan_points, curr_heading_deg, curr_v, curr_w):
    global last_w_sign, last_w_command

    local_pts = [(px, py) for px, py in [lidar_to_robot(a, d) for a, d in scan_points if 0 < d <= 1500]]
    if len(local_pts) < 5: return 0.0, 0.0, 0

    v_cands, w_cands = generate_dynamic_window(curr_v, curr_w, dt=0.1)
    best_v, best_w = 0.0, 0.0
    max_score = -1e9
    num_safe = 0

    for v in v_cands:
        for w in w_cands:
            clearance, left_c, right_c, is_front_obs = check_collision_and_clearance(v, w, local_pts)
            if clearance < 0: continue
            num_safe += 1

            pred_turn_deg = math.degrees(w * PREDICT_T)
            fut_heading = normalize_angle(curr_heading_deg + pred_turn_deg)
            score_heading = max(0.0, 1.0 - abs(fut_heading) / 180.0)

            # [우선순위 5] 정면 막힘시 회피(Clearance)에 초집중, 측면은 덜 민감하게 (틈새 주파)
            dyn_weight_c = W_CLEARANCE * (1.5 if is_front_obs else 0.8)
            score_clearance = min(1.0, clearance / 1000.0)

            # [우선순위 4] Corridor Centering 점수: 좌우 여유 공간 밸런스 맞추기
            if left_c < 1000 and right_c < 1000:
                diff = abs(left_c - right_c)
                score_centering = max(0.0, 1.0 - diff / (left_c + right_c))
            else:
                score_centering = 1.0 # 주변이 탁 트였으면 중앙 유지 불필요

            score_velocity = v / MAX_V if v > 0 else 0.0
            bias = BIAS_BONUS if (w != 0 and last_w_sign != 0 and w * last_w_sign > 0) else 0.0
            stop_pen = -0.8 if v < 0.01 else 0.0

            total = (W_HEADING * score_heading +
                     dyn_weight_c * score_clearance +
                     W_CENTERING * score_centering +
                     W_VELOCITY * score_velocity +
                     bias + stop_pen)

            if total > max_score:
                max_score = total
                best_v, best_w = v, w

    if max_score <= -1e8:
        best_v = 0.0
        best_w = -MAX_W if curr_heading_deg > 0 else +MAX_W

    if abs(best_w) < DEAD_BAND_W: best_w = 0.0
    smoothed_w = W_SMOOTH_ALPHA * last_w_command + (1 - W_SMOOTH_ALPHA) * best_w
    if best_w != 0: last_w_sign = 1 if best_w > 0 else -1

    last_w_command = smoothed_w
    return best_v, smoothed_w, num_safe

# ── 5. 메인 루프 ──────────────────────────────────────────────────────────────
def main():
    global current_state, stuck_timer, emergency_timer
    global arduino_heading_deg, last_v_command, last_w_command, recovery_count

    print("="*60, flush=True)
    print("DWA Pro 플래너 시작 (버그 픽스 완료)", flush=True)
    print("="*60, flush=True)

    try:
        lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
        time.sleep(0.5)
    except Exception as e:
        print(f"✗ 라이다 에러: {e}", flush=True); return
        
    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=0.1)
        time.sleep(0.5)
    except Exception as e:
        print(f"✗ 아두이노 에러: {e}", flush=True); lidar.close(); return

    arduino.write(b"0.00 0.00\n")
    time.sleep(0.2)

    try:
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        lidar.reset_input_buffer()
        lidar.write(bytes([0xA5, 0x20]))
        time.sleep(0.5)
        lidar.read(7)
    except Exception as e: pass

    if not find_lidar_sync(lidar):
        arduino.write(b"0.00 0.00\n"); lidar.close(); arduino.close(); return

    scan_points = []
    last_send = time.time()
    SEND_INTERVAL = 0.1
    last_cmd_time = time.time()
    dwa_count = 0
    
    current_v, current_w = 0.0, 0.0

    try:
        while True:
            read_arduino(arduino)
            raw = lidar.read(5)
            if len(raw) == 5:
                result = parse_packet(raw)
                if result is not None:
                    angle_raw, distance, s_flag = result
                    now = time.time()

                    if s_flag == 1 and scan_points and (now - last_send >= SEND_INTERVAL):
                        dwa_count += 1
                        last_cmd_time = now

                        prox = analyze_proximity(scan_points)
                        em_v, em_w, em_reason = emergency_check(prox)

                        # ★ [버그수정 2] RECOVERY 중에는 EMERGENCY로 납치되지 않도록 방어!
                        if em_v is not None and current_state == RobotState.DRIVE:
                            v, w = em_v, em_w
                            num_safe = -1
                            current_state = RobotState.EMERGENCY
                            emergency_timer = 0.3
                            last_v_command, last_w_command = v, w
                            print(f"[!비상!] {em_reason}", flush=True)

                        elif current_state == RobotState.EMERGENCY:
                            emergency_timer -= SEND_INTERVAL
                            if emergency_timer <= 0: current_state = RobotState.DRIVE
                            v, w, num_safe = last_v_command, last_w_command, 0

                        elif current_state == RobotState.DRIVE:
                            # 후진 등에서 복귀할 때 Dynamic Window 붕괴(Stutter) 방지
                            if current_v < 0: current_v = 0.0 
                            
                            v, w, num_safe = run_dwa(scan_points, arduino_heading_deg, current_v, current_w)
                            if v < 0.05:
                                stuck_timer += SEND_INTERVAL
                                if stuck_timer >= 2.0:
                                    # ★ [버그수정 1] arduino.write(b"ESC\n") 삭제 -> 파이썬이 탈출 지휘!
                                    current_state = RobotState.RECOVERY
                                    stuck_timer = 0.0
                                    recovery_count += 1 
                            else:
                                stuck_timer = 0.0

                        else: # RobotState.RECOVERY
                            stuck_timer += SEND_INTERVAL
                            if stuck_timer < 0.5:
                                v, w = -0.15, 0.0 # 0.5초간 똑바로 후진
                            else:
                                turn_sign = 1 if (recovery_count % 2 == 1) else -1 
                                if prox['left'] > prox['right'] + 300: turn_sign = 1
                                elif prox['right'] > prox['left'] + 300: turn_sign = -1
                                
                                v, w = -0.10, MAX_W * turn_sign
                            
                            num_safe = 0
                            last_v_command, last_w_command = v, w
                            if stuck_timer >= 2.5:
                                current_state = RobotState.DRIVE
                                stuck_timer = 0.0
                                current_v = 0.0 # 탈출 후 정지 상태에서 안정적 출발

                        current_v, current_w = v, w 
                        cmd = f"{v:.2f} {w:.2f}\n"
                        arduino.write(cmd.encode('utf-8'))

                        st = {1:"D",2:"R",3:"E"}.get(current_state, "?")
                        print(f"[{dwa_count:4d}] {st} v={v:+.2f} w={w:+.2f} "
                              f"hdg={arduino_heading_deg:+5.0f}° "
                              f"F={prox['front']:.0f} L={prox['left']:.0f} R={prox['right']:.0f} "
                              f"safe={num_safe}", flush=True)

                        scan_points = []
                        last_send = now

                    # ★ [버그수정 3] 코앞 데이터(150mm 이하) 증발 막기 위해 기준치 60으로 하향 조정
                    if distance > 60:
                        scan_points.append((normalize_angle(angle_raw), distance))

            now = time.time()
            if now - last_cmd_time > 1.0:
                arduino.write(b"0.00 0.00\n")
                last_cmd_time = now

    except KeyboardInterrupt: print("\n종료...", flush=True)
    except Exception as e: print(f"[에러] {e}", flush=True)
    finally:
        try: arduino.write(b"0.00 0.00\n"); time.sleep(0.1); lidar.write(bytes([0xA5, 0x25]))
        except: pass
        lidar.close(); arduino.close(); print("✓ 종료", flush=True)

if __name__ == "__main__":
    main()

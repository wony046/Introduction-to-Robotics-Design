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
MARGIN       = 35       # 안전 여유폭

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
    DRIVE = 1
    RECOVERY = 2

current_state = RobotState.DRIVE
stuck_timer = 0.0
last_w_sign = 0.0
arduino_heading_deg = 0.0

# ── 3. 유틸리티 및 라이다 파싱 ────────────────────────────────────────────────
def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

def parse_packet(data):
    if len(data) != 5: return None
    s_flag = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None
    angle_q6 = (data[1] >> 1) | (data[2] << 7)
    angle = angle_q6 / 64.0
    distance_q2 = data[3] | (data[4] << 8)
    distance = distance_q2 / 4.0
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

# ── 4. DWA 코어 (수학적 시뮬레이션) ───────────────────────────────────────────
def generate_vw_window(current_v, current_w):
    v_cands = [0.0, 0.15, 0.25, MAX_V]
    w_cands = [-MAX_W, -1.0, -0.5, 0.0, 0.5, 1.0, MAX_W]
    return v_cands, w_cands

def check_collision_and_clearance(v_m_s, w_rad_s, scan_points, predict_t=1.0, step=0.2):
    v_mm_s = v_m_s * 1000.0
    max_dist = abs(v_mm_s * predict_t) + max(ROBOT_FRONT, ROBOT_BACK) + MARGIN + 100
    
    # ★ 수정 1: 수학적 거울 반전(Mirroring) 해결
    # 라이다의 회전 방향(시계)과 수학의 회전 방향(반시계)을 맞추기 위해 Y축(sin)에 마이너스(-) 부호 적용
    local_pts = [(dist * math.cos(math.radians(ang)), -dist * math.sin(math.radians(ang))) 
                 for ang, dist in scan_points if 0 < dist <= max_dist]
                 
    if not local_pts: return 1000.0 
    
    curr_x, curr_y, curr_th = 0.0, 0.0, 0.0
    t = 0.0
    min_clear_sq = 1000000.0 

    front_bound = ROBOT_FRONT + MARGIN
    back_bound  = -ROBOT_BACK - MARGIN
    side_bound  = ROBOT_HALF_W + MARGIN

    while t <= predict_t:
        curr_x += v_mm_s * math.cos(curr_th) * step
        curr_y += v_mm_s * math.sin(curr_th) * step
        curr_th += w_rad_s * step
        t += step
        
        cos_t, sin_t = math.cos(curr_th), math.sin(curr_th)
        
        for px, py in local_pts:
            dx, dy = px - curr_x, py - curr_y
            lx = dx * cos_t + dy * sin_t
            ly = -dx * sin_t + dy * cos_t
            
            if back_bound <= lx <= front_bound and -side_bound <= ly <= side_bound:
                return -1.0 # 충돌 궤적
                
            dist_sq = dx**2 + dy**2
            if dist_sq < min_clear_sq:
                min_clear_sq = dist_sq

    return math.sqrt(min_clear_sq)

def run_dwa(scan_points, curr_heading, current_v, current_w):
    global last_w_sign
    v_cands, w_cands = generate_vw_window(current_v, current_w)
    
    best_v, best_w = 0.0, 0.0
    max_score = -1.0
    
    for v in v_cands:
        for w in w_cands:
            clearance = check_collision_and_clearance(v, w, scan_points)
            if clearance <= 0: continue 
            
            # ★ 수정 2: 로봇 하드웨어 배선에 맞춘 헤딩 뺄셈(-) 유지
            pred_turn = math.degrees(w * 1.0)
            fut_heading = normalize_angle(curr_heading - pred_turn) 
            
            score_heading = max(0.0, 1.0 - (abs(fut_heading) / 180.0))
            score_clearance = min(1.0, clearance / 1000.0)
            score_velocity = max(0.0, v / MAX_V)
            
            bias = BIAS_BONUS if (w * last_w_sign > 0) else 0.0
            total_score = (W_HEADING * score_heading) + (W_CLEARANCE * score_clearance) + (W_VELOCITY * score_velocity) + bias
                          
            if total_score > max_score:
                max_score = total_score
                best_v, best_w = v, w
                
    if best_w != 0: last_w_sign = 1.0 if best_w > 0 else -1.0
    return best_v, best_w

# ── 5. 메인 루프 (교통정리) ───────────────────────────────────────────────────
def main():
    global current_state, stuck_timer, arduino_heading_deg
    
    print("라이다 및 아두이노 초기화 중...")
    try:
        lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    except Exception as e:
        print(f"[에러] 포트 연결 실패: {e}")
        return

    # 라이다 스캔 시작 명령
    lidar.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))
    lidar.read(7) # 응답 헤더 무시
    print("주행 시작!")

    scan_points = []
    last_send = time.time()
    SEND_INTERVAL = 0.1 # 0.1초마다 갱신
    current_v, current_w = 0.0, 0.0

    try:
        while True:
            # 1. 아두이노 헤딩 읽기
            read_arduino(arduino)

            # 2. 라이다 데이터 읽기
            raw = lidar.read(5)
            result = parse_packet(raw)
            if result is None: continue

            angle_raw, distance = result
            s_flag = raw[0] & 0x01
            
            # ★ 수정 3: 로봇 내부 노이즈 필터링 (가장 중요한 부분)
            # 150mm 이하의 데이터는 라이다 본체나 선 등 '내 몸통'을 찍은 것이므로 무시합니다!
            if distance > 150:
                scan_points.append((normalize_angle(angle_raw), distance))

            # 3. 라이다가 1바퀴(한 프레임) 다 돌았고, 전송 주기가 지났을 때 연산 시작
            now = time.time()
            if s_flag == 1 and scan_points and (now - last_send >= SEND_INTERVAL):
                
                # [FSM 상태 머신]
                if current_state == RobotState.DRIVE:
                    v, w = run_dwa(scan_points, arduino_heading_deg, current_v, current_w)
                    
                    if v == 0.0:
                        stuck_timer += SEND_INTERVAL
                        if stuck_timer >= 2.0:
                            print("[상태 전환] 갇힘 감지! Recovery 모드 진입")
                            arduino.write(b"ESC\n") # 아두이노에 탈출 모드 알림
                            current_state = RobotState.RECOVERY
                            stuck_timer = 0.0
                    else:
                        stuck_timer = 0.0
                        
                elif current_state == RobotState.RECOVERY:
                    v, w = -0.1, 1.5 # 뒤로 빼면서 크게 회전
                    stuck_timer += SEND_INTERVAL
                    
                    if stuck_timer >= 1.5:
                        print("[상태 전환] 탈출 완료. Drive 모드 복귀")
                        current_state = RobotState.DRIVE
                        stuck_timer = 0.0
                
                # 모터 제어 명령 전송
                current_v, current_w = v, w
                cmd = f"{v:.2f} {w:.2f}\n"
                arduino.write(cmd.encode('utf-8'))
                
                scan_points = [] # 스캔 데이터 초기화
                last_send = now

    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        lidar.write(bytes([0xA5, 0x25])) # 라이다 정지
        arduino.write(b"0.00 0.00\n")    # 아두이노 모터 정지
        lidar.close()
        arduino.close()
        print("종료 완료.")

if __name__ == "__main__":
    main()

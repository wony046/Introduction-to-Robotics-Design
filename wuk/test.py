import serial
import time
import math

# ==========================================
# 1. 통신 포트 설정
# ==========================================
ARDUINO_PORT = '/dev/ttyAMA3'   
LIDAR_PORT = '/dev/ttyUSB0'   
print("[INIT] 통신 포트 연결 시도 중...")
arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT, 460800, timeout=1)
print("[INIT] 아두이노 부팅 대기 (2초)...")
time.sleep(2)  # ★ 이 두 줄이 USB 직결 시 반드시 필요합니다!

# ==========================================
# 2. 자율 주행 파라미터
# ==========================================
SAFE_DISTANCE = 600      # 60cm (이 거리가 뚫려있어야 풀악셀)
MAX_SPEED = 250          # 뻥 뚫렸을 때 속도
AVOID_SPEED = 120        # 정상 회피 속도
ESCAPE_SPEED = 90        # 좁은 틈 거북이 속도
STEER_GAIN = 1.5         # 조향 민감도

ROBOT_HALF_WIDTH = 115   # 230 / 2 (타이트하게 잡아 측면 오해 방지)

# ★ NEW: Yaw 보정 파라미터 -----------------------
YAW_DEADBAND_DEG = 15    # 이 범위 안에서는 보정 거의 안 함
YAW_NORMAL_DEG = 60      # 일반 보정 영역 상한
YAW_EMERGENCY_DEG = 70   # 이 이상이면 비상 회전 방향을 강제로 복귀로
# -------------------------------------------------

# 상태 변수
last_avoid_dir = 0       # 1(우회전), -1(좌회전), 0(직진)
yaw_acc_deg = 0.0        # ★ NEW: Arduino에서 받는 누적 yaw (좌+/우-)
_rx_buffer = ""          # ★ NEW: 시리얼 라인 조립용 버퍼

# ==========================================
# ★ NEW: Arduino에서 yaw 비차단 수신
# ==========================================
def update_yaw_from_arduino():
    """Arduino가 'Y:<deg>\\n' 형식으로 보낸 누적 yaw를 읽어 yaw_acc_deg 갱신.
       데이터가 없으면 즉시 리턴 (블로킹 없음)."""
    global yaw_acc_deg, _rx_buffer
    
    if arduino.in_waiting == 0:
        return
    
    try:
        chunk = arduino.read(arduino.in_waiting).decode('utf-8', errors='ignore')
        _rx_buffer += chunk
    except Exception:
        return
    
    # 줄 단위로 파싱 - 여러 줄이 들어와 있으면 가장 최근 값으로 덮어쓰기
    while '\n' in _rx_buffer:
        line, _rx_buffer = _rx_buffer.split('\n', 1)
        line = line.strip()
        if line.startswith('Y:'):
            try:
                yaw_acc_deg = float(line[2:])
            except ValueError:
                pass

def reset_yaw():
    """Arduino에 yaw 적분 초기화 명령. 출발 직전 1회 호출."""
    global yaw_acc_deg, _rx_buffer
    arduino.reset_input_buffer()
    _rx_buffer = ""
    arduino.write(b"YAW_RESET\n")
    yaw_acc_deg = 0.0
    time.sleep(0.1)

# ==========================================
# 3. 핵심 회피 알고리즘 (yaw 보정 적용)
# ==========================================
def calculate_steering(scan_data):
    global last_avoid_dir, yaw_acc_deg
    
    bins = {angle: 9999 for angle in range(-90, 91, 10)}
    
    front_emergency_dist = 9999
    front_clear_x = 9999
    left_wall_min = 9999
    right_wall_min = 9999
    
    for angle, distance in scan_data:
        if angle > 180: angle -= 360
            
        if -90 <= angle <= 90 and distance > 0:
            bin_angle = round(angle / 10) * 10
            if bin_angle in bins and distance < bins[bin_angle]:
                bins[bin_angle] = distance
                
            x_pos = distance * math.cos(math.radians(angle))
            y_pos = distance * math.sin(math.radians(angle))
            
            if -20 <= angle <= 20:
                if distance < front_emergency_dist:
                    front_emergency_dist = distance

            if x_pos > 50 and abs(y_pos) <= ROBOT_HALF_WIDTH:
                if x_pos < front_clear_x:
                    front_clear_x = x_pos
                    
            if -100 < x_pos < 400:
                if angle >= 30:
                    if y_pos < left_wall_min: left_wall_min = y_pos
                elif angle <= -30:
                    if abs(y_pos) < right_wall_min: right_wall_min = abs(y_pos)

    # ========================================================
    # [1] 비상 회피 모드 (제자리 회전) + ★ yaw 가드
    # ========================================================
    if front_emergency_dist < 200:
        # ★ NEW: 이미 너무 많이 돌아갔으면 무조건 복귀 방향으로
        # yaw_acc > 0 = 좌측으로 돌아간 상태 → 우회전(-) 으로 복귀
        if abs(yaw_acc_deg) > YAW_EMERGENCY_DEG:
            emergency_steer = -75 if yaw_acc_deg > 0 else 75
            return 0, emergency_steer
        
        # 기존 로직: 직전 회피 방향 유지 또는 측면 공간 비교
        if last_avoid_dir == -1:
            emergency_steer = 75   # 계속 좌회전
        elif last_avoid_dir == 1:
            emergency_steer = -75  # 계속 우회전
        else:
            if left_wall_min > right_wall_min:
                emergency_steer = 75
                last_avoid_dir = -1
            else:
                emergency_steer = -75
                last_avoid_dir = 1
                
        return 0, emergency_steer  

    # ========================================================
    # [2] 스코어링 모드 - ★ heading_penalty 추가
    # ========================================================
    
    # ★ NEW: 비선형 yaw 게인 (작은 오차는 거의 무시, 큰 오차는 강하게)
    # 수정 제안: yaw_err에 비례하여 부드럽게 증가하는 gain
    yaw_err = abs(yaw_acc_deg)

    if yaw_err < YAW_DEADBAND_DEG:
        heading_gain = 0.2
    else:
        # 오차가 커질수록 자연스럽게 gain이 증가 (최대 4.0으로 제한)
        heading_gain = min(0.5 + (yaw_err / 30.0), 4.0) 
    
    # ★ NEW: 로봇 좌표계에서 GOAL이 위치한 각도
    # yaw_acc 가 +30°(좌측으로 30° 돌아감)이면 GOAL은 로봇 기준 -30° 방향
    target_angle = -yaw_acc_deg
    
    best_angle = 0
    best_score = -99999
    
    for angle in range(-90, 91, 10):
        dist = bins[angle]
        
        # 1. 거리 점수
        dist_score = min(dist, 400) * 1.0
        
        # 2. ★ MODIFIED: 기존 center_penalty 가중치 감소 (3.5 → 1.5)
        #    yaw가 0에 가까울 때도 부드러운 조향이 유지되도록 약하게 남겨둠
        center_penalty = abs(angle) * 1.5
        
        # 3. ★ NEW: heading_penalty (GOAL 방향 선호)
        #    각도차를 90°로 clamp 해서 폭주 방지
        angle_diff = min(abs(angle - target_angle), 90)
        heading_penalty = angle_diff * heading_gain
        
        # 4. 복원력(Hysteresis) - 기존과 동일
        hysteresis_bonus = 0
        if front_clear_x > 300: 
            if last_avoid_dir == 1 and angle > 10:
                hysteresis_bonus = 150
            elif last_avoid_dir == -1 and angle < -10:
                hysteresis_bonus = 150

        # 5. 측면 벽 밀어내기 - 기존과 동일
        wall_repulsion_bonus = 0
        if right_wall_min < 200 and left_wall_min > right_wall_min + 40:
            if angle > 10: wall_repulsion_bonus = 150
        elif left_wall_min < 200 and right_wall_min > left_wall_min + 40:
            if angle < -10: wall_repulsion_bonus = 150
                
        # 총점 합산 - ★ heading_penalty 추가
        score = (dist_score 
                 - center_penalty 
                 - heading_penalty 
                 + hysteresis_bonus 
                 + wall_repulsion_bonus)
        
        if score > best_score:
            best_score = score
            best_angle = angle

    # 상태 업데이트
    if best_angle < -15:
        last_avoid_dir = 1
    elif best_angle > 15:
        last_avoid_dir = -1
    else:
        last_avoid_dir = 0

    steer_pwm = -int(best_angle * STEER_GAIN)
    
    # ========================================================
    # [3] 4단계 속도 결정 (기존과 동일)
    # ========================================================
    if front_clear_x <= 400:
        current_speed = ESCAPE_SPEED
    elif abs(best_angle) <= 15 and front_clear_x > SAFE_DISTANCE:
        current_speed = MAX_SPEED
    else:
        current_speed = AVOID_SPEED
    
    return current_speed, steer_pwm

# ==========================================
# 4. 메인 루프
# ==========================================
def main():
    print("[INIT] 라이다 초기화(RESET) 명령 전송...")
    lidar_ser.write(bytes([0xA5, 0x40])) 
    time.sleep(1)
    
    print("[INIT] 라이다 스캔(SCAN) 명령 전송...")
    lidar_ser.write(bytes([0xA5, 0x20])) 
    time.sleep(0.5)
    
    print("[INIT] 아두이노로 Yaw 초기화 명령 전송...")
    reset_yaw()
    
    print("[INFO] 자율 주행 시작! (라이다 데이터 수신 대기 중...)")
    scan_data = []
    debug_counter = 0

    try:
        while True:
            data = lidar_ser.read(5)
            if len(data) != 5: continue

            # 패킷 검증
            s_flag = data[0] & 0x01
            s_inv_flag = (data[0] & 0x02) >> 1
            if s_inv_flag != (1 - s_flag): continue
            if (data[1] & 0x01) != 1: continue

            angle_q6 = ((data[1] >> 1) | (data[2] << 7))
            angle = angle_q6 / 64.0
            distance_q2 = (data[3] | (data[4] << 8))
            distance = distance_q2 / 4.0

            # 한 바퀴 스캔 완료 시
            if s_flag == 1:
                if len(scan_data) > 50: 
                    # ★ NEW: 스캔 끝날 때마다 yaw 갱신
                    update_yaw_from_arduino()
                    
                    speed, steer = calculate_steering(scan_data)
                    command = f"{speed},{steer}\n"
                    arduino.write(command.encode('utf-8'))
                    
                    # ★ NEW: 10사이클마다 디버그 출력
                    debug_counter += 1
                    if debug_counter % 10 == 0:
                        print(f"[DBG] yaw={yaw_acc_deg:+6.1f}°  "
                              f"speed={speed:3d}  steer={steer:+4d}")
                scan_data = []

            if distance > 0:
                scan_data.append((angle, distance))

    except KeyboardInterrupt:
        print("\n[INFO] 정지 명령 수신. 모터를 끕니다.")
        arduino.write(b"0,0\n")
        lidar_ser.write(bytes([0xA5, 0x25]))
        arduino.close()
        lidar_ser.close()

if __name__ == '__main__':
    main()

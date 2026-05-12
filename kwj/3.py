import serial
import time
import math

# ==========================================
# 1. 통신 포트 설정
# ==========================================
ARDUINO_PORT = '/dev/ttyAMA3'   
LIDAR_PORT = '/dev/ttyUSB0'   

arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT, 460800, timeout=1)

# ==========================================
# 2. 자율 주행 파라미터 (★ 무정지 스무스 코너링 ★)
# ==========================================
SAFE_DISTANCE = 600      
MAX_SPEED = 250          
AVOID_SPEED = 150        
ESCAPE_SPEED = 100       

# ★ 수정 1: 조향 민감도 감소 (덜 예민하게)
STEER_GAIN = 1.3         

# ★ 추가: 조향 부드러움 계수 (0.0 ~ 1.0)
# 1.0에 가까울수록 즉시 꺾고, 낮을수록 서서히 꺾습니다.
SMOOTHING_FACTOR = 0.5   

ROBOT_HALF_WIDTH = 115   

last_avoid_dir = 0       
last_steer_pwm = 0       # ★ 추가: 이전 프레임의 핸들 각도 기억용

# ==========================================
# 3. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_avoid_dir, last_steer_pwm
    
    bins = {angle: 9999 for angle in range(-140, 141, 10)}
    
    front_emergency_dist = 9999  
    front_clear_x = 9999         
    left_wall_min = 9999         
    right_wall_min = 9999        
    
    for angle, distance in scan_data:
        if angle > 180: angle -= 360
            
        if -135 <= angle <= 135 and distance > 0:
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
                    
            if -200 < x_pos < 400:
                if angle >= 30:   
                    if y_pos < left_wall_min: left_wall_min = y_pos
                elif angle <= -30: 
                    if abs(y_pos) < right_wall_min: right_wall_min = abs(y_pos)

    # ========================================================
    # [1] 피벗 턴 (Pivot Turn) 모드 - 코앞 발동 거리 축소
    # ========================================================
    # ★ 수정 2: 발동 거리를 200으로 낮춰 스무스 코너링 공간 확보
    if front_emergency_dist < 200 or front_clear_x < 200:
        left_openness = sum(bins[a] for a in range(10, 91, 10) if a in bins)
        right_openness = sum(bins[a] for a in range(-90, 0, 10) if a in bins)

        # 피벗 턴 각도도 100 -> 80으로 살짝 부드럽게 조정
        if left_openness > right_openness + 500:
            last_avoid_dir = -1
            pivot_steer = 80
        elif right_openness > left_openness + 500:
            last_avoid_dir = 1
            pivot_steer = -80
        else:
            if last_avoid_dir == -1:
                pivot_steer = 80
            elif last_avoid_dir == 1:
                pivot_steer = -80
            else:
                if left_wall_min > right_wall_min:
                    pivot_steer = 80
                    last_avoid_dir = -1
                else:
                    pivot_steer = -80
                    last_avoid_dir = 1
        
        last_steer_pwm = pivot_steer
        return 100, pivot_steer

    # ========================================================
    # [2] 거리 비례 스코어링 모드
    # ========================================================
    best_angle = 0
    best_score = -99999
    
    for angle in range(-90, 91, 10):
        dist = bins[angle]
        
        dist_score = min(dist, 1000) * 1.0
        center_penalty = abs(angle) * 3.5
        
        hysteresis_bonus = 0
        if front_clear_x > 300: 
            if last_avoid_dir == 1 and angle > 10:
                hysteresis_bonus = 150
            elif last_avoid_dir == -1 and angle < -10:
                hysteresis_bonus = 150

        wall_repulsion_bonus = 0
        
        if left_wall_min <= 200 and right_wall_min <= 200:
            if right_wall_min < left_wall_min - 20:
                if angle > 0: wall_repulsion_bonus = 80  
            elif left_wall_min < right_wall_min - 20:
                if angle < 0: wall_repulsion_bonus = 80  
        else:
            # ★ 수정 3: 긴급 회피 가중치를 200 -> 150으로 완화하여 과조향 방지
            if right_wall_min <= 130 and left_wall_min > right_wall_min + 30:
                if angle >= 15: wall_repulsion_bonus = 150
            elif left_wall_min <= 130 and right_wall_min > left_wall_min + 30:
                if angle <= -15: wall_repulsion_bonus = 150
                
        score = dist_score - center_penalty + hysteresis_bonus + wall_repulsion_bonus
        
        if score > best_score:
            best_score = score
            best_angle = angle

    if best_angle < -15:
        last_avoid_dir = 1
    elif best_angle > 15:
        last_avoid_dir = -1
    else:
        last_avoid_dir = 0

    # 목표로 하는 원본 조향값 계산
    target_steer_pwm = int(best_angle * STEER_GAIN)
    
    # ========================================================
    # [3] 4단계 속도 결정 및 조향 한계/스무딩 필터 적용
    # ========================================================
    if front_clear_x <= 450:
        current_speed = ESCAPE_SPEED
    elif abs(best_angle) <= 15 and front_clear_x > SAFE_DISTANCE:
        current_speed = MAX_SPEED
    else:
        current_speed = AVOID_SPEED
    
    # ★ 핵심 1: 조향 한계 (Steering Limit)
    # 조향값이 현재 속도의 70%를 넘지 않도록 제한하여, 안쪽 바퀴가 멈추는 급회전을 막습니다.
    max_allowed_steer = int(current_speed * 0.7)
    if target_steer_pwm > max_allowed_steer:
        target_steer_pwm = max_allowed_steer
    elif target_steer_pwm < -max_allowed_steer:
        target_steer_pwm = -max_allowed_steer

    # ★ 핵심 2: 조향 부드러움 필터 (Low-Pass Filter)
    # 확 꺾지 않고, 이전 핸들 각도와 새로운 목표 각도를 섞어 스무스하게 돌립니다.
    steer_pwm = int((SMOOTHING_FACTOR * target_steer_pwm) + ((1.0 - SMOOTHING_FACTOR) * last_steer_pwm))
    last_steer_pwm = steer_pwm

    return current_speed, steer_pwm

# ==========================================
# 4. 메인 루프
# ==========================================
def main():
    print("[INFO] 라이다 초기화 중...")
    lidar_ser.write(bytes([0xA5, 0x40])) 
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20])) 
    time.sleep(0.5)
    print("[INFO] 스무스 코너링 자율 주행 시작! (정지하려면 Ctrl+C)")

    scan_data = []

    try:
        while True:
            data = lidar_ser.read(5)
            if len(data) != 5: continue

            s_flag = data[0] & 0x01
            s_inv_flag = (data[0] & 0x02) >> 1
            if s_inv_flag != (1 - s_flag): continue
            if (data[1] & 0x01) != 1: continue

            angle_q6 = ((data[1] >> 1) | (data[2] << 7))
            angle = angle_q6 / 64.0
            distance_q2 = (data[3] | (data[4] << 8))
            distance = distance_q2 / 4.0

            if s_flag == 1:
                if len(scan_data) > 50: 
                    speed, steer = calculate_steering(scan_data)
                    command = f"{speed},{steer}\n"
                    arduino.write(command.encode('utf-8'))
                scan_data = []

            if distance > 0:
                scan_data.append((angle, distance))

    except KeyboardInterrupt:
        print("\n[INFO] 정지 명령 수신. 모터를 끕니다.")
        arduino.write("0,0\n".encode('utf-8'))
        lidar_ser.write(bytes([0xA5, 0x25]))
        arduino.close()
        lidar_ser.close()

if __name__ == '__main__':
    main()

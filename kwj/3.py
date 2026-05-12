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
# 2. 자율 주행 파라미터 (★ 무정지 전진 전용 ★)
# ==========================================
SAFE_DISTANCE = 600      
MAX_SPEED = 250          
AVOID_SPEED = 150        
ESCAPE_SPEED = 100       
STEER_GAIN = 1.7         

ROBOT_HALF_WIDTH = 115   # 로봇 폭 절반

last_avoid_dir = 0       

# ==========================================
# 3. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_avoid_dir
    
    # ★ 270도 시야각을 담을 수 있도록 Bins 범위 확장 (-140 ~ +140)
    bins = {angle: 9999 for angle in range(-140, 141, 10)}
    
    front_emergency_dist = 9999  
    front_clear_x = 9999         
    left_wall_min = 9999         
    right_wall_min = 9999        
    
    for angle, distance in scan_data:
        if angle > 180: angle -= 360
            
        # ★ 270도 시야 확보 (-135도 ~ +135도)
        if -135 <= angle <= 135 and distance > 0:
            bin_angle = round(angle / 10) * 10
            if bin_angle in bins and distance < bins[bin_angle]:
                bins[bin_angle] = distance
                
            x_pos = distance * math.cos(math.radians(angle))
            y_pos = distance * math.sin(math.radians(angle))
            
            # [A] 코앞 비상 감지 (25cm 이내)
            if -20 <= angle <= 20:
                if distance < front_emergency_dist:
                    front_emergency_dist = distance

            # [B] 내 궤적 내 방해물 (전방 직진 시)
            if x_pos > 50 and abs(y_pos) <= ROBOT_HALF_WIDTH:
                if x_pos < front_clear_x:
                    front_clear_x = x_pos
                    
            # [C] ★ 270도 측면 긁힘 감시 (로봇 뒤쪽 20cm부터 전방 40cm까지) ★
            # 회전할 때 바깥쪽으로 밀리는 엉덩이나 뒷바퀴가 벽에 긁히는 것을 감지합니다.
            if -200 < x_pos < 400:
                if angle >= 30:   
                    if y_pos < left_wall_min: left_wall_min = y_pos
                elif angle <= -30: 
                    if abs(y_pos) < right_wall_min: right_wall_min = abs(y_pos)

    # ========================================================
    # [1] 피벗 턴 (Pivot Turn) 모드: 정지/제자리 회전 절대 불가
    # ========================================================
    if front_emergency_dist < 250 or front_clear_x < 250:
        # 피벗 턴 방향 결정 시에는 정면(90도 이내) 데이터만 참고합니다.
        left_openness = sum(bins[a] for a in range(10, 91, 10) if a in bins)
        right_openness = sum(bins[a] for a in range(-90, 0, 10) if a in bins)

        if left_openness > right_openness + 500:
            last_avoid_dir = -1
            pivot_steer = 100
        elif right_openness > left_openness + 500:
            last_avoid_dir = 1
            pivot_steer = -100
        else:
            if last_avoid_dir == -1:
                pivot_steer = 100
            elif last_avoid_dir == 1:
                pivot_steer = -100
            else:
                if left_wall_min > right_wall_min:
                    pivot_steer = 100
                    last_avoid_dir = -1
                else:
                    pivot_steer = -100
                    last_avoid_dir = 1
        
        return 100, pivot_steer

    # ========================================================
    # [2] 거리 비례 스코어링 모드 (길 찾기)
    # ========================================================
    best_angle = 0
    best_score = -99999
    
    # 조향(핸들) 점수는 정면(-90 ~ 90)만 계산합니다. (뒤로 조향할 순 없으므로)
    for angle in range(-90, 91, 10):
        dist = bins[angle]
        
        # ★ 1m 시야 확보: 코너 탈출구를 미리 보고 부드럽게 진입합니다!
        dist_score = min(dist, 1000) * 1.0
        center_penalty = abs(angle) * 3.5
        
        hysteresis_bonus = 0
        if front_clear_x > 300: 
            if last_avoid_dir == 1 and angle > 10:
                hysteresis_bonus = 150
            elif last_avoid_dir == -1 and angle < -10:
                hysteresis_bonus = 150

        wall_repulsion_bonus = 0
        # ★ 측면 긁힘 철벽 방어 (Wall Repulsion 강화) ★
        # 벽이 차체 중앙에서 18cm 이내(약 6.5cm 여유)로 가까워지면 즉시 강한 보너스(250점)를 주어 밀어냅니다!
        if right_wall_min < 180 and left_wall_min > right_wall_min + 40:
            if angle > 15: wall_repulsion_bonus = 250
        elif left_wall_min < 180 and right_wall_min > left_wall_min + 40:
            if angle < -15: wall_repulsion_bonus = 250
                
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

    steer_pwm = int(best_angle * STEER_GAIN)
    
    # ========================================================
    # [3] 4단계 속도 결정 및 역회전 방지 잠금장치
    # ========================================================
    if front_clear_x <= 450:
        current_speed = ESCAPE_SPEED
    elif abs(best_angle) <= 15 and front_clear_x > SAFE_DISTANCE:
        current_speed = MAX_SPEED
    else:
        current_speed = AVOID_SPEED
    
    if steer_pwm > current_speed:
        steer_pwm = current_speed
    elif steer_pwm < -current_speed:
        steer_pwm = -current_speed

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
    print("[INFO] 무정지 270도 시야 자율 주행 시작! (정지하려면 Ctrl+C)")

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

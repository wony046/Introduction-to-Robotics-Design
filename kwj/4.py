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
# 2. 자율 주행 파라미터 
# ==========================================
SAFE_DISTANCE = 600      
MAX_SPEED = 250          
AVOID_SPEED = 150        
CAUTION_SPEED = 110      
ESCAPE_SPEED = 100       

STEER_GAIN = 1.3         
SMOOTHING_FACTOR = 0.5   

ROBOT_HALF_WIDTH = 125   # 모서리 스윕 대비 1cm 추가 여유

last_avoid_dir = 0       
last_steer_pwm = 0       

# ==========================================
# 3. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_avoid_dir, last_steer_pwm
    
    bins = {angle: 9999 for angle in range(-140, 141, 5)}
    
    front_emergency_dist = 9999  
    front_clear_x = 9999         
    left_wall_min = 9999         
    right_wall_min = 9999        
    closest_dist = 9999          
    
    for angle, distance in scan_data:
        if angle > 180: angle -= 360
            
        if -135 <= angle <= 135 and distance > 0:
            if distance < closest_dist:
                closest_dist = distance

            bin_angle = round(angle / 5) * 5
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
    # [1] 피벗 턴 (Pivot Turn) 모드
    # ========================================================
    if front_emergency_dist < 150 or front_clear_x < 150:
        left_openness = sum(bins[a] for a in range(5, 91, 5) if a in bins)
        right_openness = sum(bins[a] for a in range(-90, 0, 5) if a in bins)

        if left_openness > right_openness + 500:
            last_avoid_dir = -1
            pivot_steer = 65
        elif right_openness > left_openness + 500:
            last_avoid_dir = 1
            pivot_steer = -65
        else:
            if last_avoid_dir == -1:
                pivot_steer = 65
            elif last_avoid_dir == 1:
                pivot_steer = -65
            else:
                if left_wall_min > right_wall_min:
                    pivot_steer = 65
                    last_avoid_dir = -1
                else:
                    pivot_steer = -65
                    last_avoid_dir = 1
        
        last_steer_pwm = pivot_steer
        return CAUTION_SPEED, pivot_steer

    # ========================================================
    # [2] 거리 비례 스코어링 모드 (5도 해상도)
    # ========================================================
    best_angle = 0
    best_score = -99999
    
    for angle in range(-90, 91, 5):
        dist = bins[angle]
        
        dist_score = min(dist, 1000) * 1.0
        center_penalty = abs(angle) * 3.5
        
        hysteresis_bonus = 0
        if front_clear_x > 300: 
            if last_avoid_dir == 1 and angle > 5:
                hysteresis_bonus = 150
            elif last_avoid_dir == -1 and angle < -5:
                hysteresis_bonus = 150

        wall_repulsion_bonus = 0
        
        # ====================================================
        # ★ 수정: 18cm 이내 폭발적 2단계 비례 제어 도입 ★
        # ====================================================
        # 오른쪽 벽 감시
        if right_wall_min < 240:
            repel_force = (240 - right_wall_min) * 1.5
            if right_wall_min < 180:
                repel_force += (180 - right_wall_min) * 4.0  # 18cm 이내 폭발적 점수 증가!
            
            if angle > 0:   # 좌회전(회피)
                wall_repulsion_bonus += repel_force
            elif angle < 0: # 우회전(모서리로 파고들기)
                wall_repulsion_bonus -= (repel_force * 1.5)  # 파고드는 조향엔 더 가혹한 페널티!

        # 왼쪽 벽 감시
        if left_wall_min < 240:
            repel_force = (240 - left_wall_min) * 1.5
            if left_wall_min < 180:
                repel_force += (180 - left_wall_min) * 4.0   # 18cm 이내 폭발적 점수 증가!
                
            if angle < 0:   # 우회전(회피)
                wall_repulsion_bonus += repel_force
            elif angle > 0: # 좌회전(모서리로 파고들기)
                wall_repulsion_bonus -= (repel_force * 1.5)  # 파고드는 조향엔 더 가혹한 페널티!
                
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

    target_steer_pwm = int(best_angle * STEER_GAIN)
    
    # ========================================================
    # [3] 속도 결정 및 조향 한계/스무딩 필터
    # ========================================================
    if closest_dist <= 170:
        current_speed = CAUTION_SPEED
    elif front_clear_x <= 450:
        current_speed = ESCAPE_SPEED
    elif abs(best_angle) <= 15 and front_clear_x > SAFE_DISTANCE:
        current_speed = MAX_SPEED
    else:
        current_speed = AVOID_SPEED
    
    max_allowed_steer = int(current_speed * 0.55)
    if target_steer_pwm > max_allowed_steer:
        target_steer_pwm = max_allowed_steer
    elif target_steer_pwm < -max_allowed_steer:
        target_steer_pwm = -max_allowed_steer

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
    print("[INFO] 폭발적 측면 방어 자율 주행 시작! (정지하려면 Ctrl+C)")

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

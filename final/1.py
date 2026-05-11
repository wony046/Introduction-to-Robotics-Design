import serial
import time
import math

# ==========================================
# 1. 통신 포트 설정
# ==========================================
ARDUINO_PORT = '/dev/ttyS0'   
LIDAR_PORT = '/dev/ttyUSB0'   

arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT, 460800, timeout=1)

# ==========================================
# 2. 자율 주행 파라미터
# ==========================================
SAFE_DISTANCE = 400      # 40cm (내 차폭 기준 앞길이 이만큼 뚫리면 풀악셀)
MAX_SPEED = 250          # 뻥 뚫렸을 때 속도
AVOID_SPEED = 120        # 정상 회피 속도
ESCAPE_SPEED = 90        # 좁은 틈 거북이 속도
STEER_GAIN = 1.5         # 조향 민감도

# 로봇 크기 설정 (가로세로 230mm)
ROBOT_HALF_WIDTH = 115   # 230 / 2 (타이트하게 잡아 측면 오해 방지)

last_avoid_dir = 0       # 1(우회전), -1(좌회전), 0(직진)

# ==========================================
# 3. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_avoid_dir
    
    bins = {angle: 9999 for angle in range(-90, 91, 10)}
    
    front_emergency_dist = 9999  # 오직 -20 ~ +20도 사이의 순수 최단 거리 (비상 정지용)
    front_clear_x = 9999         # 내 차폭 안으로 들어오는 X축 거리 (주행 판단용)
    left_wall_min = 9999         # 좌측 벽 최단 Y거리
    right_wall_min = 9999        # 우측 벽 최단 Y거리
    
    for angle, distance in scan_data:
        if angle > 180: angle -= 360
            
        if -90 <= angle <= 90 and distance > 0:
            bin_angle = round(angle / 10) * 10
            if bin_angle in bins and distance < bins[bin_angle]:
                bins[bin_angle] = distance
                
            x_pos = distance * math.cos(math.radians(angle))
            y_pos = distance * math.sin(math.radians(angle))
            
            # [A] 진짜 코앞 비상 정지용 (-20도 ~ +20도)
            if -20 <= angle <= 20:
                if distance < front_emergency_dist:
                    front_emergency_dist = distance

            # [B] 궤적 투영: 내 차폭 안에 들어오는 전방 물체
            if x_pos > 50 and abs(y_pos) <= ROBOT_HALF_WIDTH:
                if x_pos < front_clear_x:
                    front_clear_x = x_pos
                    
            # [C] 측면 벽 감시 (거북이 모드 시 긁힘 방지용)
            if -100 < x_pos < 400:
                if angle >= 30:   # 좌측 벽
                    if y_pos < left_wall_min: left_wall_min = y_pos
                elif angle <= -30: # 우측 벽
                    if abs(y_pos) < right_wall_min: right_wall_min = abs(y_pos)

    # ========================================================
    # [1] 비상 회피 모드 (제자리 회전)
    # ========================================================
    if front_emergency_dist < 200:
        if last_avoid_dir == -1:
            emergency_steer = 75   
        elif last_avoid_dir == 1:
            emergency_steer = -75  
        else:
            if left_wall_min > right_wall_min:
                emergency_steer = 75
                last_avoid_dir = -1
            else:
                emergency_steer = -75
                last_avoid_dir = 1
                
        return 0, emergency_steer  

    # ========================================================
    # [2] 거리 비례 스코어링 모드 (길 찾기 & 측면 벽 밀어내기)
    # ========================================================
    best_angle = 0
    best_score = -99999
    
    for angle in range(-90, 91, 10):
        dist = bins[angle]
        
        dist_score = min(dist, 400) * 1.0
        center_penalty = abs(angle) * 3.5
        
        # 앞길이 어느 정도 열렸을 때 원래 길로 복귀
        hysteresis_bonus = 0
        if front_clear_x > 300: 
            if last_avoid_dir == 1 and angle > 10:
                hysteresis_bonus = 150
            elif last_avoid_dir == -1 and angle < -10:
                hysteresis_bonus = 150

        # 측면 벽 밀어내기 (Wall Repulsion)
        wall_repulsion_bonus = 0
        if right_wall_min < 200 and left_wall_min > right_wall_min + 40:
            if angle > 10: wall_repulsion_bonus = 150
        elif left_wall_min < 200 and right_wall_min > left_wall_min + 40:
            if angle < -10: wall_repulsion_bonus = 150
                
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
    # [3] 4단계 속도 결정
    # ========================================================
    # 거북이 모드: 차폭 내 앞길이 40cm 이내일 때 (비비며 탈출)
    if front_clear_x <= 400:
        current_speed = ESCAPE_SPEED
        
    # 풀악셀: 거의 직진 중이고, 내 차폭 기준 앞길이 60cm 이상 뻥 뚫렸을 때
    elif abs(best_angle) <= 15 and front_clear_x > SAFE_DISTANCE:
        current_speed = MAX_SPEED
        
    # 일반 회피: 정면은 40cm 이상 뚫렸지만 코너링 중일 때
    else:
        current_speed = AVOID_SPEED
    
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
    print("[INFO] 자율 주행 시작! (정지하려면 Ctrl+C)")

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

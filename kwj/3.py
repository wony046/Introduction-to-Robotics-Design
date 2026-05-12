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
SAFE_DISTANCE = 600      # 60cm (이 거리가 뚫려있어야 풀악셀)
MAX_SPEED = 250          # 뻥 뚫렸을 때 속도
AVOID_SPEED = 150        # 정상 회피 속도
ESCAPE_SPEED = 100       # 좁은 틈 거북이 속도 (절대 0이 되지 않음!)
STEER_GAIN = 1.7         # 조향 민감도

ROBOT_HALF_WIDTH = 115   # 로봇의 좌우 반경

last_avoid_dir = 0       

# ==========================================
# 3. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_avoid_dir
    
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
            
            # [A] 코앞 비상 감지 (정지 불가능하므로 기준 거리를 200->250으로 상향!)
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
    # [1] 피벗 턴 (Pivot Turn) 모드: 정지/제자리 회전 절대 불가!
    # ========================================================
    # 코앞 25cm에 벽이 나타나면 멈추지 않고, 전진하며 날카롭게 꺾어버립니다.
    if front_emergency_dist < 250 or front_clear_x < 250:
        left_openness = sum(bins[a] for a in range(10, 91, 10))
        right_openness = sum(bins[a] for a in range(-90, 0, 10))

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
        
        # ★ Speed 100, Steer 100을 주면 -> 한 바퀴는 0, 다른 바퀴는 200으로 '전진 피벗 회전' 합니다.
        return 100, pivot_steer

    # ========================================================
    # [2] 거리 비례 스코어링 모드 (길 찾기)
    # ========================================================
    best_angle = 0
    best_score = -99999
    
    for angle in range(-90, 91, 10):
        dist = bins[angle]
        
        # 거리를 600까지만 봐서 헛것을 보지 않되, 함정에 빠지기 전에 피할 시야는 확보
        dist_score = min(dist, 600) * 1.0
        center_penalty = abs(angle) * 3.5
        
        hysteresis_bonus = 0
        if front_clear_x > 300: 
            if last_avoid_dir == 1 and angle > 10:
                hysteresis_bonus = 150
            elif last_avoid_dir == -1 and angle < -10:
                hysteresis_bonus = 150

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
    # [3] 4단계 속도 결정 및 ★ 역회전 방지 잠금장치 ★
    # ========================================================
    if front_clear_x <= 450:
        current_speed = ESCAPE_SPEED
    elif abs(best_angle) <= 15 and front_clear_x > SAFE_DISTANCE:
        current_speed = MAX_SPEED
    else:
        current_speed = AVOID_SPEED
    
    # ★ 핵심 규정 방어: 조향값이 현재 속도보다 커지면 바퀴 하나가 음수(후진)가 됩니다.
    # 이를 원천 차단하기 위해 steer_pwm을 current_speed 범위 안으로 가둡니다.
    if steer_pwm > current_speed:
        steer_pwm = current_speed
    elif steer_pwm < -current_speed:
        steer_pwm = -current_speed

    return current_speed, steer_pwm

# ==========================================
# 4. 메인 루프 (라이다 데이터 파싱 및 전송)
# ==========================================
def main():
    print("[INFO] 라이다 초기화 중...")
    lidar_ser.write(bytes([0xA5, 0x40])) 
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20])) 
    time.sleep(0.5)
    print("[INFO] 무정지 전진 자율 주행 시작! (정지하려면 Ctrl+C)")

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

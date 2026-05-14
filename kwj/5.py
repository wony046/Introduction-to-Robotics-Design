import serial
import time
import math

# ==========================================
# 1. 자율 주행 파라미터 
# ==========================================
SPEED_MAX = 250        
SPEED_DRIVE = 180      
SPEED_SAFETY = 140     
SPEED_REVERSE = -130   
ESCAPE_SPEED = 0       

STEER_GAIN = 1.3         
SMOOTHING_FACTOR = 0.5   

MARGIN = 20            
ROBOT_FRONT = 115 + MARGIN  
ROBOT_SIDE = 105 + MARGIN   
ROBOT_REAR = 130 + MARGIN   

SAFE_RADIUS = 300      
DANGER_RADIUS = 50     
GAP_THRESHOLD = 300    

ARDUINO_PORT = '/dev/ttyAMA3'
LIDAR_PORT = '/dev/ttyUSB0'

arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT, 460800, timeout=1)

last_chosen_angle = 0
last_steer_pwm = 0
last_ideal_angle = 0  # ★ 추가: 목표 각도의 관성을 위한 변수

# ==========================================
# 2. 핵심 회피 알고리즘 (두뇌)
# ==========================================
def calculate_steering(scan_data):
    global last_chosen_angle, last_steer_pwm, last_ideal_angle
    
    bins = {angle: 9999 for angle in range(-135, 136, 5)}
    closest_obj_dist = 9999
    front_clear_dist = 9999 
    
    left_wall_min = 9999
    right_wall_min = 9999

    # [1] 데이터 파싱 및 거리 분류
    for angle, distance in scan_data:
        if angle > 180: angle -= 360
        if -135 <= angle <= 135 and distance > 0:
            if distance < closest_obj_dist:
                closest_obj_dist = distance

            bin_angle = round(angle / 5) * 5
            if bin_angle in bins and distance < bins[bin_angle]:
                bins[bin_angle] = distance
                
            rad = math.radians(angle)
            x = distance * math.cos(rad)
            y = distance * math.sin(rad)
            
            if x > 0 and abs(y) <= ROBOT_SIDE:
                if x < front_clear_dist:
                    front_clear_dist = x
                    
            # ★ 처방 1: 측면 시야를 90도 -> 130도로 대폭 확대 (뒤통수까지 감시)
            if -100 < x < 400:
                if 10 <= angle <= 130:     
                    if y < left_wall_min: left_wall_min = y
                elif -130 <= angle <= -10: 
                    if abs(y) < right_wall_min: right_wall_min = abs(y)

    # [2] 틈새(Gap) 파악
    gaps = []
    angles = sorted(bins.keys())
    for i in range(1, len(angles)):
        if abs(bins[angles[i]] - bins[angles[i-1]]) > GAP_THRESHOLD:
            target = angles[i] if bins[angles[i]] > bins[angles[i-1]] else angles[i-1]
            gaps.append(target)

    left_vol = sum(min(bins[a], 1000) for a in range(10, 91, 5) if a in bins)
    right_vol = sum(min(bins[a], 1000) for a in range(-90, 0, 5) if a in bins)

    # ========================================================
    # [3] 방향 결정 (가상 중심선 필터링)
    # ========================================================
    target_ideal_angle = 0
    WALL_SAFE_DIST = 220 
    
    # 벽이 가까우면 밀어내는 힘 계산
    if right_wall_min < WALL_SAFE_DIST:
        target_ideal_angle += (WALL_SAFE_DIST - right_wall_min) * 0.7  
    if left_wall_min < WALL_SAFE_DIST:
        target_ideal_angle -= (WALL_SAFE_DIST - left_wall_min) * 0.7   
        
    target_ideal_angle = max(-60, min(60, target_ideal_angle))
    
    # ★ 처방 2: 생각의 관성(Low-Pass Filter). 타겟 각도가 미친 듯이 널뛰는 것을 방지
    # 끈적하게 각도를 유지해서 코너 진입 시 와리가리 원천 차단
    ideal_angle = (0.3 * target_ideal_angle) + (0.7 * last_ideal_angle)
    last_ideal_angle = ideal_angle

    best_angle = 0
    min_score = float('inf')
    
    for angle in range(-90, 91, 5): 
        dist = bins[angle]
        raw_dist = min(dist, 1000)
        
        # 1. 기본 거리 점수
        score = 1000 - raw_dist
            
        # 2. 직진 본능 (0도가 아니라 끈적한 ideal_angle을 따라감)
        score += abs(angle - ideal_angle) * 3.0
        
        # 3. 틈새(Gap) 보너스
        for gap_angle in gaps:
            if abs(angle - gap_angle) <= 10:
                score -= 400 
                
        # 4. 거시적 방향 보너스
        if left_vol > right_vol + 1500 and angle > 0: score -= 150
        elif right_vol > left_vol + 1500 and angle < 0: score -= 150
            
        # 5. 절대 충돌 방지(히트박스)
        rad = math.radians(angle)
        x = raw_dist * math.cos(rad)
        y = raw_dist * math.sin(rad)
        if (abs(y) < ROBOT_SIDE + 20) and (x < ROBOT_FRONT + 30):
            score += 5000 

        if score < min_score:
            min_score = score
            best_angle = angle

    # ========================================================
    # [4] 속도 결정
    # ========================================================
    if min_score >= 8000:
        speed = SPEED_REVERSE
        steer_pwm = 80 if last_chosen_angle < 0 else -80
        
    elif front_clear_dist < 180: 
        speed = ESCAPE_SPEED
        steer_pwm = 90 if best_angle > 0 else -90
        
    else:
        if front_clear_dist < 300 or closest_obj_dist < 200: speed = SPEED_SAFETY
        elif front_clear_dist < 500 or closest_obj_dist < 350: speed = SPEED_DRIVE
        else: speed = SPEED_MAX
            
        target_steer = int(best_angle * STEER_GAIN)
        max_steer = int(speed * 0.8)
        
        if target_steer > max_steer: target_steer = max_steer
        elif target_steer < -max_steer: target_steer = -max_steer

        steer_pwm = int((SMOOTHING_FACTOR * target_steer) + ((1.0 - SMOOTHING_FACTOR) * last_steer_pwm))
        
    last_chosen_angle = best_angle
    last_steer_pwm = steer_pwm

    return speed, steer_pwm

# ==========================================
# 3. 메인 루프 
# ==========================================
def main():
    print("[INFO] 라이다 초기화 중...")
    lidar_ser.write(bytes([0xA5, 0x40])) 
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20])) 
    time.sleep(0.5)
    print("[INFO] 뒤통수 시야 확보 & 생각의 관성 장착! 주행 시작!")

    scan_data = []

    try:
        while True:
            data = lidar_ser.read(5)
            if len(data) != 5: continue

            s_flag = data[0] & 0x01
            if (data[1] & 0x01) != 1: continue

            angle_q6 = ((data[1] >> 1) | (data[2] << 7))
            angle = angle_q6 / 64.0
            distance_q2 = (data[3] | (data[4] << 8))
            distance = distance_q2 / 4.0

            if s_flag == 1:
                if len(scan_data) > 30: 
                    speed, steer = calculate_steering(scan_data)
                    
                    command = f"{int(speed)},{int(steer)}\n"
                    arduino.write(command.encode('utf-8'))
                    
                scan_data = []

            if distance > 0:
                scan_data.append((angle, distance))

    except KeyboardInterrupt:
        print("\n[INFO] 안전 정지 수신. 모터를 끕니다.")
        arduino.write("0,0\n".encode('utf-8'))
        lidar_ser.write(bytes([0xA5, 0x25]))
        arduino.close()
        lidar_ser.close()

if __name__ == '__main__':
    main()

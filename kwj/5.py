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

MARGIN = 15            
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
last_ideal_angle = 0

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
                    
            if -100 < x < 400:
                if 10 <= angle <= 130:     
                    if y < left_wall_min: left_wall_min = y
                elif -130 <= angle <= -10: 
                    if abs(y) < right_wall_min: right_wall_min = abs(y)

    gaps = []
    angles = sorted(bins.keys())
    for i in range(1, len(angles)):
        if abs(bins[angles[i]] - bins[angles[i-1]]) > GAP_THRESHOLD:
            target = angles[i] if bins[angles[i]] > bins[angles[i-1]] else angles[i-1]
            gaps.append(target)

    left_vol = sum(min(bins.get(a, 1000), 1000) for a in range(10, 91, 5))
    right_vol = sum(min(bins.get(a, 1000), 1000) for a in range(-90, 0, 5))

    # ========================================================
    # [3] 방향 결정: ★ 골목길 데드존 + 둔감화 로직 ★
    # ========================================================
    eff_left = min(left_wall_min, 400)
    eff_right = min(right_wall_min, 400)
    diff = eff_left - eff_right
    
    # 1. 좁은 골목길(Tunnel) 감지: 양쪽 벽이 모두 200mm 이내일 때
    in_narrow_alley = (eff_left < 200 and eff_right < 200)

    # 2. 데드존(Deadzone): 양쪽 오차가 3cm(30mm) 이내면 "완벽한 중앙"으로 간주
    if abs(diff) < 30:
        diff = 0

    # 3. 동적 P-Gain: 골목길에서는 민감도를 확 낮춰서 와리가리 차단
    p_gain = 0.15 if in_narrow_alley else 0.35
    target_ideal_angle = diff * p_gain  
    
    # 4. 비상 회피 마진 억제: 골목길에서는 호들갑 떨지 않고 직진 유지!
    if not in_narrow_alley:
        if right_wall_min < ROBOT_SIDE + 5: 
            target_ideal_angle += (125 - right_wall_min) * 2.0
        if left_wall_min < ROBOT_SIDE + 5:
            target_ideal_angle -= (125 - left_wall_min) * 2.0
            
    # 5. 조향 한계 제한: 골목길에서는 핸들을 15도 이상 꺾지 못하게 강제 (엉덩이 충돌 방지)
    if in_narrow_alley:
        target_ideal_angle = max(-15, min(15, target_ideal_angle))
    else:
        target_ideal_angle = max(-65, min(65, target_ideal_angle))
    
    ideal_angle = (0.35 * target_ideal_angle) + (0.65 * last_ideal_angle)
    last_ideal_angle = ideal_angle

    best_angle = 0
    min_score = float('inf')
    
    for angle in range(-90, 91, 5): 
        dist = bins[angle]
        raw_dist = min(dist, 1000)
        
        score = 2000 - (raw_dist * 2.0)
        score += abs(angle - ideal_angle) * 3.5 
        
        if left_vol > right_vol + 2000 and angle > 0: score -= 300
        elif right_vol > left_vol + 2000 and angle < 0: score -= 300
        
        for gap_angle in gaps:
            if abs(angle - gap_angle) <= 10:
                score -= 600 
                
        rad = math.radians(angle)
        x = raw_dist * math.cos(rad)
        y = raw_dist * math.sin(rad)
        
        # 물리적 히트박스 페널티
        if (abs(y) <= ROBOT_SIDE) and (x <= ROBOT_FRONT):
            score += 8000 

        if score < min_score:
            min_score = score
            best_angle = angle

    # ========================================================
    # [4] 속도 결정
    # ========================================================
    if min_score >= 8000:
        speed = SPEED_REVERSE
        steer_pwm = 80 if left_vol < right_vol else -80
        
    elif front_clear_dist < 150: 
        speed = ESCAPE_SPEED
        steer_pwm = 90 if left_vol > right_vol else -90
        
    else:
        if front_clear_dist < 300 or closest_obj_dist < 180: speed = SPEED_SAFETY
        elif front_clear_dist < 500 or closest_obj_dist < 300: speed = SPEED_DRIVE
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
    print("[INFO] 골목길 둔감화(Deadzone) 적용! 핑퐁 없는 직진 시작!")

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

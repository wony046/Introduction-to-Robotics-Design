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

# ★ 극도로 타이트해진 측면 최소 마진 1.5cm (15mm)
MARGIN = 15            
ROBOT_FRONT = 115 + MARGIN  # 130mm
ROBOT_SIDE = 105 + MARGIN   # 120mm (이 선을 넘으면 충돌로 간주)

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
            
            # 전방 궤적 확인 (차폭 120mm 기준)
            if x > 0 and abs(y) <= ROBOT_SIDE:
                if x < front_clear_dist:
                    front_clear_dist = x
                    
            # 측면 거리 확인 (전방 40cm, 후방 10cm 구역)
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
    # [3] 방향 결정: ★ 순수 차등 P-제어 (Differential P-Control) ★
    # ========================================================
    
    # 1. 계산의 일관성을 위해 너무 먼 거리는 400mm로 캡핑(Capping)
    eff_left = min(left_wall_min, 400)
    eff_right = min(right_wall_min, 400)
    
    # 2. 오차 계산: 왼쪽 공간과 오른쪽 공간의 차이
    # diff가 양수면 왼쪽이 더 넓음 -> 좌회전(+) 필요
    # diff가 음수면 오른쪽이 더 넓음 -> 우회전(-) 필요
    diff = eff_left - eff_right
    
    # 3. 비례 제어 (P-Gain = 0.3)
    target_ideal_angle = diff * 0.3  
    
    # 4. [초비상 마진 1.5cm 방어] 물리적으로 긁히기 직전(125mm 이하)일 때만 강제 회피!
    if right_wall_min < ROBOT_SIDE + 5: # 120 + 5 = 125mm
        target_ideal_angle += (125 - right_wall_min) * 2.0
    if left_wall_min < ROBOT_SIDE + 5:
        target_ideal_angle -= (125 - left_wall_min) * 2.0
        
    target_ideal_angle = max(-65, min(65, target_ideal_angle))
    
    # 소프트 관성(Low-Pass Filter)
    ideal_angle = (0.35 * target_ideal_angle) + (0.65 * last_ideal_angle)
    last_ideal_angle = ideal_angle

    # [스코어링 - 이제 로봇은 맹목적으로 ideal_angle을 추종합니다]
    best_angle = 0
    min_score = float('inf')
    
    for angle in range(-90, 91, 5): 
        dist = bins[angle]
        raw_dist = min(dist, 1000)
        
        # 기본 거리 확보 점수
        score = 2000 - (raw_dist * 2.0)
        
        # ★ P-제어로 계산된 '가장 완벽한 중앙선(ideal_angle)'을 강하게 추종
        score += abs(angle - ideal_angle) * 3.5 
        
        # 거시적 공간 중력 보너스
        if left_vol > right_vol + 2000 and angle > 0: score -= 300
        elif right_vol > left_vol + 2000 and angle < 0: score -= 300
        
        # 틈새 보너스
        for gap_angle in gaps:
            if abs(angle - gap_angle) <= 10:
                score -= 600 
                
        # 절대 히트박스 페널티 (차폭 120mm 이내의 충돌 궤적 차단)
        rad = math.radians(angle)
        x = raw_dist * math.cos(rad)
        y = raw_dist * math.sin(rad)
        if (abs(y) <= ROBOT_SIDE) and (x <= ROBOT_FRONT):
            score += 8000 

        if score < min_score:
            min_score = score
            best_angle = angle

    # ========================================================
    # [4] 속도 결정 (순수 거리 기반)
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
    print("[INFO] 차등 오차 P-제어(Differential P-Control) 장착 완료!")

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

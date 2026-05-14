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

# 측면 여유 마진 1.5cm (15mm)
MARGIN = 15            
ROBOT_FRONT = 115 + MARGIN  # 130mm
ROBOT_SIDE = 105 + MARGIN   # 120mm
ROBOT_REAR = 130 + MARGIN   # 145mm

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
            
            # 전방 궤적 확인 (차폭 120mm 적용)
            if x > 0 and abs(y) <= ROBOT_SIDE:
                if x < front_clear_dist:
                    front_clear_dist = x
                    
            # 측면 130도까지 감시 (뒤통수 보호)
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

    # ★ 좌우 공간 전체 넓이(체적) 측정 (와리가리 방지의 핵심 무기)
    left_vol = sum(min(bins.get(a, 1000), 1000) for a in range(10, 91, 5))
    right_vol = sum(min(bins.get(a, 1000), 1000) for a in range(-90, 0, 5))

    # ========================================================
    # [3] 방향 결정 (가상 중심선 필터링 + 순수 스코어링)
    # ========================================================
    target_ideal_angle = 0
    WALL_SAFE_DIST = 150 
    
    if right_wall_min < WALL_SAFE_DIST:
        target_ideal_angle += (WALL_SAFE_DIST - right_wall_min) * 1.5  
    if left_wall_min < WALL_SAFE_DIST:
        target_ideal_angle -= (WALL_SAFE_DIST - left_wall_min) * 1.5   
        
    target_ideal_angle = max(-60, min(60, target_ideal_angle))
    ideal_angle = (0.4 * target_ideal_angle) + (0.6 * last_ideal_angle)
    last_ideal_angle = ideal_angle

    best_angle = 0
    min_score = float('inf')
    
    for angle in range(-90, 91, 5): 
        dist = bins[angle]
        raw_dist = min(dist, 1000)
        
        score = 2000 - (raw_dist * 2.0)
        score += abs(angle - ideal_angle) * 2.0
        
        # 거시적 공간 중력
        if left_vol > right_vol + 2000 and angle > 0: score -= 500
        elif right_vol > left_vol + 2000 and angle < 0: score -= 500
            
        # 소프트 관성 보너스 (살짝 더 강화)
        if (last_chosen_angle > 10 and angle > 10) or (last_chosen_angle < -10 and angle < -10):
            score -= 400 
        
        for gap_angle in gaps:
            if abs(angle - gap_angle) <= 10:
                score -= 600 
                
        rad = math.radians(angle)
        x = raw_dist * math.cos(rad)
        y = raw_dist * math.sin(rad)
        if (abs(y) < ROBOT_SIDE + 5) and (x < ROBOT_FRONT + 10):
            score += 8000 

        if score < min_score:
            min_score = score
            best_angle = angle

    # ========================================================
    # [4] 속도 결정 (순수 거리 기반)
    # ========================================================
    
    # 1. 사면초가
    if min_score >= 8000:
        speed = SPEED_REVERSE
        # 후진 시에도 볼륨 비교로 넓은 쪽으로 엉덩이를 뺌
        steer_pwm = 80 if left_vol < right_vol else -80
        
    # 2. 제자리 회전 모드 (전방 15cm 이내 막힘)
    elif front_clear_dist < 150: 
        speed = ESCAPE_SPEED
        # ★ 와리가리 원천 차단: 단일 각도가 아닌 좌우 전체 볼륨 크기로 확신을 갖고 팽이 회전!
        steer_pwm = 90 if left_vol > right_vol else -90
        
    # 3. 일반 주행
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
    print("[INFO] 공간 체적(Volume) 기반 절대 회전 로직 장착 완료!")

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

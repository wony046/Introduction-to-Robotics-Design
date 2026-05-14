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

# ==========================================
# 2. 핵심 회피 알고리즘 (두뇌)
# ==========================================
def calculate_steering(scan_data):
    global last_chosen_angle, last_steer_pwm
    
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
            
            # 전방 궤적 (내 차폭 안의 장애물)
            if x > 0 and abs(y) <= ROBOT_SIDE:
                if x < front_clear_dist:
                    front_clear_dist = x
                    
            # 측면 벽 (전방 40cm, 후방 10cm 이내의 양옆 장애물)
            if -100 < x < 400:
                if 10 <= angle <= 90:     
                    if y < left_wall_min: left_wall_min = y
                elif -90 <= angle <= -10: 
                    if abs(y) < right_wall_min: right_wall_min = abs(y)

    # [2] 틈새(Gap) 파악
    gaps = []
    angles = sorted(bins.keys())
    for i in range(1, len(angles)):
        if abs(bins[angles[i]] - bins[angles[i-1]]) > GAP_THRESHOLD:
            target = angles[i] if bins[angles[i]] > bins[angles[i-1]] else angles[i-1]
            gaps.append(target)

    # 거시적 좌우 볼륨
    left_vol = sum(min(bins[a], 1000) for a in range(10, 91, 5) if a in bins)
    right_vol = sum(min(bins[a], 1000) for a in range(-90, 0, 5) if a in bins)

    # ========================================================
    # [3] 방향 결정 (Steering): 가상 중심선 이동 로직 적용!
    # ========================================================
    
    # ★ 핵심: 양쪽 벽과의 거리를 계산해 '가장 이상적인 목표 각도'를 먼저 구합니다.
    ideal_angle = 0
    WALL_SAFE_DIST = 220 # 125(차폭) + 95(여유공간)
    
    # 우측 벽이 위험거리 내에 있으면 좌회전(양수) 방향으로 가상 중심을 이동!
    if right_wall_min < WALL_SAFE_DIST:
        ideal_angle += (WALL_SAFE_DIST - right_wall_min) * 0.8  
        
    # 좌측 벽이 위험거리 내에 있으면 우회전(음수) 방향으로 가상 중심을 이동!
    if left_wall_min < WALL_SAFE_DIST:
        ideal_angle -= (WALL_SAFE_DIST - left_wall_min) * 0.8   
        
    # 가상 중심이 너무 꺾이지 않도록 ±75도로 제한
    ideal_angle = max(-75, min(75, ideal_angle))

    best_angle = 0
    min_score = float('inf')
    
    for angle in range(-90, 91, 5): 
        dist = bins[angle]
        raw_dist = min(dist, 1000)
        
        # 1. 기본 거리 점수 (가까울수록 폭발적인 점수 1000)
        score = 1000 - raw_dist
            
        # 2. ★ 새로운 본능: 0도가 아니라, '이상적인 각도(ideal_angle)'를 벗어날수록 페널티!
        # 이제 로봇은 0도(직진)를 고집하지 않고 이상적인 회전각을 편안하게 따릅니다.
        score += abs(angle - ideal_angle) * 3.0
        
        # 3. 틈새(Gap) 보너스 (-)
        for gap_angle in gaps:
            if abs(angle - gap_angle) <= 10:
                score -= 400 
                
        # 4. 거시적 방향 보너스 (-)
        if left_vol > right_vol + 1500 and angle > 0: score -= 150
        elif right_vol > left_vol + 1500 and angle < 0: score -= 150
            
        # 5. 절대 충돌 방지(히트박스) (+)
        rad = math.radians(angle)
        x = raw_dist * math.cos(rad)
        y = raw_dist * math.sin(rad)
        if (abs(y) < ROBOT_SIDE + 20) and (x < ROBOT_FRONT + 30):
            score += 5000 

        # 최저 점수 갱신
        if score < min_score:
            min_score = score
            best_angle = angle

    # ========================================================
    # [4] 속도 결정 (Speed): 거리로만 쿨하게 결정
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
    print("[INFO] 가상 중심선 이동(P-Control) 탑재! 부드러운 자율주행 시작!")

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

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

# 라이다 중심축을 기준으로 한 로봇의 크기 + 절대 마진(20mm)
MARGIN = 20            
ROBOT_FRONT = 115 + MARGIN  # 앞범퍼: 135mm
ROBOT_SIDE = 105 + MARGIN   # 측면 폭: 125mm
ROBOT_REAR = 130 + MARGIN   # 뒷범퍼: 150mm

SAFE_RADIUS = 300      
DANGER_RADIUS = 50     
GAP_THRESHOLD = 300    

# 통신 설정
ARDUINO_PORT = '/dev/ttyAMA3'
LIDAR_PORT = '/dev/ttyUSB0'

# ==========================================
# 2. 초기화 및 전역 변수
# ==========================================
arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT, 460800, timeout=1)

last_chosen_angle = 0
last_steer_pwm = 0

# ==========================================
# 3. 핵심 회피 알고리즘 (두뇌)
# ==========================================
def calculate_steering(scan_data):
    global last_chosen_angle, last_steer_pwm
    
    bins = {angle: 9999 for angle in range(-135, 136, 5)}
    closest_obj_dist = 9999
    front_clear_dist = 9999 # ★ 내 직진 궤적 상의 전방 최단 거리
    
    # [1] 라이다 데이터 파싱 및 필터링
    for angle, distance in scan_data:
        if angle > 180: angle -= 360
        if -135 <= angle <= 135 and distance > 0:
            if distance < closest_obj_dist:
                closest_obj_dist = distance

            bin_angle = round(angle / 5) * 5
            if bin_angle in bins and distance < bins[bin_angle]:
                bins[bin_angle] = distance
                
            # ★ 정면 차폭 이내의 장애물 거리 구하기
            rad = math.radians(angle)
            x = distance * math.cos(rad)
            y = distance * math.sin(rad)
            if x > 0 and abs(y) <= ROBOT_SIDE:
                if x < front_clear_dist:
                    front_clear_dist = x

    # [2] 틈새(Gap) 리스트 추출
    gaps = []
    angles = sorted(bins.keys())
    for i in range(1, len(angles)):
        diff = abs(bins[angles[i]] - bins[angles[i-1]])
        if diff > GAP_THRESHOLD:
            target = angles[i] if bins[angles[i]] > bins[angles[i-1]] else angles[i-1]
            gaps.append(target)

    left_vol = sum(min(bins[a], 1000) for a in range(10, 91, 5) if a in bins)
    right_vol = sum(min(bins[a], 1000) for a in range(-90, 0, 5) if a in bins)

    # [3] 각도별 스코어링 (점수가 낮을수록 좋은 길)
    best_angle = 0
    min_score = float('inf')
    
    # 측면 벽 최단 거리 계산 (전방 40cm 이내)
    right_wall_min = min((bins[a] * math.sin(math.radians(abs(a)))) for a in range(-90, 0, 5) if a in bins and bins[a] * math.cos(math.radians(a)) < 400) if any(bins[a] * math.cos(math.radians(a)) < 400 for a in range(-90, 0, 5) if a in bins) else 9999
    left_wall_min = min((bins[a] * math.sin(math.radians(a))) for a in range(5, 91, 5) if a in bins and bins[a] * math.cos(math.radians(a)) < 400) if any(bins[a] * math.cos(math.radians(a)) < 400 for a in range(5, 91, 5) if a in bins) else 9999

    for angle in range(-90, 91, 5): 
        dist = bins[angle]
        raw_dist = min(dist, 1000)
        
        # 1. 기본 거리 점수
        if raw_dist <= 600: score = (1000 / (raw_dist + 1))
        else:
            effective_dist = 600 + (raw_dist - 600) * 0.5
            score = (1000 / (effective_dist + 1))
            
        # 2. 직진 본능 (핸들을 불필요하게 꺾는 것을 방지)
        score += abs(angle) * 3.0
        
        # 3. 틈새(Gap) 보너스 (-)
        for gap_angle in gaps:
            if abs(angle - gap_angle) <= 10:
                score -= 300 
                
        # 4. 거시적 방향 보너스 (-)
        if left_vol > right_vol + 1500 and angle > 0: score -= 100
        elif right_vol > left_vol + 1500 and angle < 0: score -= 100
            
        # 5. 절대 마진 보호 (히트박스)
        rad = math.radians(angle)
        x = raw_dist * math.cos(rad)
        y = raw_dist * math.sin(rad)
        if (abs(y) < ROBOT_SIDE) and (x < ROBOT_FRONT):
            score += 5000 
            
        # ====================================================
        # 6. ★ 조건부 측면 방어 (와리가리 방지 핵심) ★
        # ====================================================
        # 전방이 40cm 이상 뚫려있고, 거의 직진(±10도)을 평가할 때는
        # 절대 마진(ROBOT_SIDE)만 안 긁히면 측면을 완전히 무시합니다.
        if front_clear_dist > 400 and abs(angle) <= 10:
            if left_wall_min < ROBOT_SIDE or right_wall_min < ROBOT_SIDE:
                score += 2000 # 긁히기 직전이면 패널티
            # 그 외에는 side repulsion 점수(밀어내기)를 아예 더하지 않습니다!
            
        # 전방이 막혔거나, 크게 회전(±15도 이상)해야 할 때는 
        # 어깨 스윕 볼륨을 감안하여 기존의 강력한 측면 방어를 가동합니다.
        else:
            if right_wall_min < 240:
                repel = (240 - right_wall_min) * 1.5
                if right_wall_min < 180: repel += (180 - right_wall_min) * 3.0
                if angle > 0: score -= repel            
                elif angle < 0: score += (repel * 2.0)  

            if left_wall_min < 240:
                repel = (240 - left_wall_min) * 1.5
                if left_wall_min < 180: repel += (180 - left_wall_min) * 3.0
                if angle < 0: score -= repel            
                elif angle > 0: score += (repel * 2.0)  
        # ====================================================

        # 최고 좋은 길 갱신
        if score < min_score:
            min_score = score
            best_angle = angle

    # ========================================================
    # [4] 주행 판단 및 속도 제어
    # ========================================================
    if min_score > 3000: 
        return SPEED_REVERSE, (80 if last_chosen_angle < 0 else -80)
    
    if front_clear_dist <= 180:
        steer_val = 90 if best_angle > 0 else -90
        return ESCAPE_SPEED, steer_val

    if closest_obj_dist < DANGER_RADIUS + ROBOT_SIDE: speed = SPEED_SAFETY
    elif closest_obj_dist < SAFE_RADIUS: speed = SPEED_DRIVE
    else: speed = SPEED_MAX
        
    last_chosen_angle = best_angle
    target_steer_pwm = int(best_angle * STEER_GAIN)
    
    max_allowed_steer = int(speed * 0.8)
    if target_steer_pwm > max_allowed_steer: target_steer_pwm = max_allowed_steer
    elif target_steer_pwm < -max_allowed_steer: target_steer_pwm = -max_allowed_steer

    steer_pwm = int((SMOOTHING_FACTOR * target_steer_pwm) + ((1.0 - SMOOTHING_FACTOR) * last_steer_pwm))
    last_steer_pwm = steer_pwm

    return speed, steer_pwm

# ==========================================
# 5. 메인 루프 
# ==========================================
def main():
    print("[INFO] 라이다 초기화 중...")
    lidar_ser.write(bytes([0xA5, 0x40])) 
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20])) 
    time.sleep(0.5)
    print("[INFO] 직진 우선 본능 장착 완료! 자율주행 시작! (정지: Ctrl+C)")

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

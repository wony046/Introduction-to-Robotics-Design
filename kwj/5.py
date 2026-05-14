import serial
import time
import math

# ==========================================
# 1. 자율 주행 파라미터 
# ==========================================
# [속도 설정]
SPEED_MAX = 250        
SPEED_DRIVE = 180      
SPEED_SAFETY = 140     
SPEED_REVERSE = -130   
ESCAPE_SPEED = 0       

STEER_GAIN = 1.3         
SMOOTHING_FACTOR = 0.5   

# [하드웨어 크기 및 마진 (mm)]
MARGIN = 20            
ROBOT_FRONT = 115 + MARGIN  # 135mm
ROBOT_SIDE = 105 + MARGIN   # 125mm
ROBOT_REAR = 130 + MARGIN   # 150mm

GAP_THRESHOLD = 300    

# [통신 설정]
ARDUINO_PORT = '/dev/ttyAMA3'
LIDAR_PORT = '/dev/ttyUSB0'

arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT, 460800, timeout=1)

last_chosen_angle = 0
last_steer_pwm = 0

# ==========================================
# 2. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_chosen_angle, last_steer_pwm
    
    bins = {angle: 9999 for angle in range(-135, 136, 5)}
    closest_obj_dist = 9999
    front_clear_dist = 9999 # 직진 궤적 상 최단 거리
    
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
                    
            # 측면 벽 (전방 40cm 이내의 양옆 장애물)
            if 0 < x < 400:
                if 10 <= angle <= 135:     # 좌측면
                    if y < left_wall_min: left_wall_min = y
                elif -135 <= angle <= -10: # 우측면
                    if abs(y) < right_wall_min: right_wall_min = abs(y)

    # [2] 틈새(Gap) 파악 및 거시적 볼륨 계산
    gaps = []
    angles = sorted(bins.keys())
    for i in range(1, len(angles)):
        if abs(bins[angles[i]] - bins[angles[i-1]]) > GAP_THRESHOLD:
            target = angles[i] if bins[angles[i]] > bins[angles[i-1]] else angles[i-1]
            gaps.append(target)

    left_vol = sum(min(bins[a], 1000) for a in range(10, 91, 5) if a in bins)
    right_vol = sum(min(bins[a], 1000) for a in range(-90, 0, 5) if a in bins)

    # ========================================================
    # [3] 방향 결정 (Steering): 오직 스코어 경쟁! (낮을수록 좋음)
    # ========================================================
    best_angle = 0
    min_score = float('inf')
    
    for angle in range(-90, 91, 5): 
        dist = bins[angle]
        raw_dist = min(dist, 1000)
        
        # 1. 기본 거리 점수: 1000 기준에서 거리를 뺌 (가까울수록 점수 폭등)
        score = 1000 - raw_dist
            
        # 2. 직진 본능: 불필요한 핸들링 방지
        score += abs(angle) * 1.5
        
        # 3. 틈새(Gap) 보너스 (-)
        for gap_angle in gaps:
            if abs(angle - gap_angle) <= 10:
                score -= 400 
                
        # 4. 거시적 방향 보너스 (-)
        if left_vol > right_vol + 1500 and angle > 0: score -= 150
        elif right_vol > left_vol + 1500 and angle < 0: score -= 150
            
        # 5. 절대 마진(히트박스) 방어 (+)
        rad = math.radians(angle)
        x = raw_dist * math.cos(rad)
        y = raw_dist * math.sin(rad)
        if (abs(y) < ROBOT_SIDE) and (x < ROBOT_FRONT):
            score += 10000 # 물리적으로 충돌하는 궤적은 완전 배제
            
        # 6. ★ 비대칭 측면 방어 (와리가리 원천 차단) ★
        if right_wall_min < 250:
            penalty = (250 - right_wall_min)
            if angle < 0: score += (penalty * 5.0)  # 우측 벽이 있는데 우회전? 지옥의 페널티
            elif angle > 0: score -= (penalty * 2.0) # 우측 벽을 피해 좌회전? 보너스 지급

        if left_wall_min < 250:
            penalty = (250 - left_wall_min)
            if angle > 0: score += (penalty * 5.0)  # 좌측 벽이 있는데 좌회전? 지옥의 페널티
            elif angle < 0: score -= (penalty * 2.0) # 좌측 벽을 피해 우회전? 보너스 지급

        # 최고 좋은 길(최저 점수) 갱신
        if score < min_score:
            min_score = score
            best_angle = angle

    # ========================================================
    # [4] 속도 결정 (Speed): 오직 장애물 거리로만 결정!
    # ========================================================
    
    # 사면초가 (가장 좋은 길조차 히트박스 충돌 시) -> 강제 후진
    if min_score >= 9000:
        speed = SPEED_REVERSE
        steer_pwm = 80 if last_chosen_angle < 0 else -80
        
    # 코앞이 물리적으로 막힘 -> 직진 0, 제자리 팽이 회전(Pivot)
    elif front_clear_dist < 160: 
        speed = ESCAPE_SPEED
        steer_pwm = 90 if best_angle > 0 else -90
        
    # 물리적 공간은 있으나, 주변 환경에 따라 3단계 변속
    else:
        # 안전 모드: 전방 30cm 또는 주변 20cm 이내 장애물
        if front_clear_dist < 300 or closest_obj_dist < 200:
            speed = SPEED_SAFETY
        # 주행 모드: 전방 50cm 또는 주변 35cm 이내 장애물
        elif front_clear_dist < 500 or closest_obj_dist < 350:
            speed = SPEED_DRIVE
        # 전속력 모드: 뻥 뚫림
        else:
            speed = SPEED_MAX
            
        # 조향값 계산 (최대 80% 제한으로 안쪽 바퀴 역회전 방지)
        target_steer = int(best_angle * STEER_GAIN)
        max_steer = int(speed * 0.8)
        
        if target_steer > max_steer: target_steer = max_steer
        elif target_steer < -max_steer: target_steer = -max_steer

        # 스무딩 필터 적용
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
    print("[INFO] 역할 분리(속도/방향) 완벽 적용! 자율주행 시작!")

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

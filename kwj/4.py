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
SAFE_DISTANCE = 600      # 60cm 이상 확보 시 최고속도
MAX_SPEED = 250          # 최고 속도
AVOID_SPEED = 150        # 일반 회피 속도
CAUTION_SPEED = 120      # 주변 17cm 이내 감지 시 안전 속도
ESCAPE_SPEED = 100       # 긴급 회전/18cm 위험 구역 진입 시 거북이 모드

STEER_GAIN = 1.3         # 조향 민감도
SMOOTHING_FACTOR = 0.5   # 조향 부드러움 필터 (0.5 = 이전 각도 50% + 목표 각도 50%)

# 차체 크기 + 회전 시 모서리 튀어나옴(Swept volume) 여유분 1cm 추가
ROBOT_HALF_WIDTH = 125   

last_avoid_dir = 0       
last_steer_pwm = 0       

# ==========================================
# 3. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_avoid_dir, last_steer_pwm
    
    # 해상도 5도 단위, 시야각 270도 (-135 ~ +135) 설정
    bins = {angle: 9999 for angle in range(-140, 141, 5)}
    
    front_emergency_dist = 9999  
    front_clear_x = 9999         
    left_wall_min = 9999         
    right_wall_min = 9999        
    closest_dist = 9999          
    
    # 라이다 데이터 파싱 및 삼각함수 좌표 변환
    for angle, distance in scan_data:
        if angle > 180: angle -= 360
            
        if -135 <= angle <= 135 and distance > 0:
            # 주변 전체 최단 거리 추적
            if distance < closest_dist:
                closest_dist = distance

            bin_angle = round(angle / 5) * 5
            if bin_angle in bins and distance < bins[bin_angle]:
                bins[bin_angle] = distance
                
            x_pos = distance * math.cos(math.radians(angle))
            y_pos = distance * math.sin(math.radians(angle))
            
            # [A] 정면 코앞 비상 감지용 (-20도 ~ +20도)
            if -20 <= angle <= 20:
                if distance < front_emergency_dist:
                    front_emergency_dist = distance

            # [B] 내 직진 궤적(차폭) 내 방해물 감지
            if x_pos > 50 and abs(y_pos) <= ROBOT_HALF_WIDTH:
                if x_pos < front_clear_x:
                    front_clear_x = x_pos
                    
            # [C] 측면 270도 방어망 (차체 뒤쪽 20cm ~ 전방 40cm)
            if -200 < x_pos < 400:
                if angle >= 30:   
                    if y_pos < left_wall_min: left_wall_min = y_pos
                elif angle <= -30: 
                    if abs(y_pos) < right_wall_min: right_wall_min = abs(y_pos)

    # ========================================================
    # [1] 피벗 턴 (Pivot Turn) 긴급 회피 모드
    # ========================================================
    # 무정지/전진 전용 규정에 맞춰 속도는 ESCAPE_SPEED 유지, 강하게 조향
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
            if last_avoid_dir == -1: pivot_steer = 65
            elif last_avoid_dir == 1: pivot_steer = -65
            else:
                if left_wall_min > right_wall_min:
                    pivot_steer = 65
                    last_avoid_dir = -1
                else:
                    pivot_steer = -65
                    last_avoid_dir = 1
        
        last_steer_pwm = pivot_steer
        return ESCAPE_SPEED, pivot_steer

    # ========================================================
    # [2] 거리 비례 & 틈새 추종 스코어링 모드 (5도 해상도)
    # ========================================================
    best_angle = 0
    best_score = -99999
    
    # 거시적 판단(Macro Bias): 좌우 전체 볼륨 비교 (최대 거리 1000 제한)
    left_vol = sum(min(bins[a], 1000) for a in range(10, 91, 5) if a in bins)
    right_vol = sum(min(bins[a], 1000) for a in range(-90, 0, 5) if a in bins)
    
    for angle in range(-90, 91, 5):
        dist = bins[angle]
        raw_dist = min(dist, 1000)
        
        # 1. 원거리 감가상각 (Piecewise Scoring): 60cm 이상은 0.5배율로 환산
        if raw_dist <= 600:
            dist_score = raw_dist * 1.0
        else:
            dist_score = 600 + (raw_dist - 600) * 0.5
            
        center_penalty = abs(angle) * 3.5
        
        hysteresis_bonus = 0
        if front_clear_x > 300: 
            if last_avoid_dir == 1 and angle > 5: hysteresis_bonus = 150
            elif last_avoid_dir == -1 and angle < -5: hysteresis_bonus = 150

        # 2. C자 코스용 거시적 보너스: 전체적으로 뚫린 방향 선호
        macro_bonus = 0
        if left_vol > right_vol + 1500:
            if angle > 0: macro_bonus = 80
        elif right_vol > left_vol + 1500:
            if angle < 0: macro_bonus = 80

        # ====================================================
        # 3. ★ 연속성 파괴 감지 (Follow The Gap) ★
        # ====================================================
        gap_bonus = 0
        # 좌우(5도 차이) 이웃 레이저의 거리 확인
        left_neighbor = min(bins.get(angle + 5, raw_dist), 1000)
        right_neighbor = min(bins.get(angle - 5, raw_dist), 1000)

        # 현재 거리가 이웃보다 30cm(300mm) 이상 급격하게 뚫려있다면 숨겨진 틈새로 판단!
        if raw_dist > left_neighbor + 300 or raw_dist > right_neighbor + 300:
            gap_bonus = 300  
            
        # ====================================================
        # 4. ★ 2단계 폭발적 측면 방어 (Two-Stage Repulsion) ★
        # ====================================================
        wall_repulsion_bonus = 0
        
        # 오른쪽 벽 감시
        if right_wall_min < 240:
            repel_force = (240 - right_wall_min) * 1.5
            if right_wall_min < 180:
                repel_force += (180 - right_wall_min) * 3.0  # 18cm 이내 폭발적 증가
            
            if angle > 0:   wall_repulsion_bonus += repel_force
            elif angle < 0: wall_repulsion_bonus -= (repel_force * 1.5) # 파고들면 강력 페널티

        # 왼쪽 벽 감시
        if left_wall_min < 240:
            repel_force = (240 - left_wall_min) * 1.5
            if left_wall_min < 180:
                repel_force += (180 - left_wall_min) * 3.0   
                
            if angle < 0:   wall_repulsion_bonus += repel_force
            elif angle > 0: wall_repulsion_bonus -= (repel_force * 1.5)
                
        # 총점 합산
        score = dist_score - center_penalty + hysteresis_bonus + macro_bonus + gap_bonus + wall_repulsion_bonus
        
        if score > best_score:
            best_score = score
            best_angle = angle

    if best_angle < -15: last_avoid_dir = 1
    elif best_angle > 15: last_avoid_dir = -1
    else: last_avoid_dir = 0

    target_steer_pwm = int(best_angle * STEER_GAIN)
    
    # ========================================================
    # [3] 속도 제어 및 조향 한계/스무딩 필터
    # ========================================================
    # 18cm 이내 위험 구역 감지 시 강제 거북이 모드
    if closest_dist <= 180:
        current_speed = ESCAPE_SPEED
    elif front_clear_x <= 450:
        current_speed = CAUTION_SPEED
    elif abs(best_angle) <= 15 and front_clear_x > SAFE_DISTANCE:
        current_speed = MAX_SPEED
    else:
        current_speed = AVOID_SPEED
    
    # 조향값이 속도의 80%를 넘지 못하게 하여 날카로운 급정거 방지 (시원한 코너링)
    max_allowed_steer = int(current_speed * 0.80)
    if target_steer_pwm > max_allowed_steer: target_steer_pwm = max_allowed_steer
    elif target_steer_pwm < -max_allowed_steer: target_steer_pwm = -max_allowed_steer

    # 조향 부드러움 필터 (로우패스 필터)
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
    print("[INFO] 틈새 추종 & 폭발 방어 자율 주행 시작! (정지하려면 Ctrl+C)")

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

import serial
import time
import math

# ==========================================
# 1. 자율 주행 상수 설정 (튜닝 변수 모음)
# ==========================================
# [속도 설정] (0~250 내부 스케일)
SPEED_MAX = 250        # 전방 뻥 뚫림
SPEED_DRIVE = 180      # 일반 주행
SPEED_SAFETY = 140     # 주변 5cm 이내 장애물 감지 시
SPEED_REVERSE = -130   # 모든 길이 막혀 후진할 때
ESCAPE_SPEED = 0       # 제자리 팽이 회전 시 전진 속도 0

STEER_GAIN = 1.3         # 조향 민감도
SMOOTHING_FACTOR = 0.5   # 조향 스무딩 필터

# [하드웨어 크기 및 마진 (mm)]
MARGIN = 20                 # 절대 사수 최소 마진 (2cm)
ROBOT_FRONT = 115 + MARGIN  # 앞범퍼: 135mm
ROBOT_SIDE = 105 + MARGIN   # 측면 폭: 125mm
ROBOT_REAR = 130 + MARGIN   # 뒷범퍼: 150mm

SAFE_RADIUS = 300      # 30cm 이내 장애물 진입 시 속도 180으로 감속
DANGER_RADIUS = 50     # 5cm 이내 진입 시 속도 140으로 대폭 감속
GAP_THRESHOLD = 300    # 인접 레이저 거리차가 30cm 이상이면 틈새(길)로 판단

# [통신 설정]
ARDUINO_PORT = '/dev/ttyAMA3'
LIDAR_PORT = '/dev/ttyUSB0'

# ==========================================
# 2. 초기화 및 전역 변수
# ==========================================
arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT, 460800, timeout=1)

last_chosen_angle = 0
last_steer_pwm = 0

def send_to_arduino(speed, steer):
    arduino.write("R\n".encode('utf-8')) # 아두이노 헤딩 강제 리셋 (노예 모드)
    
    v_mps = (speed / 250.0) * 0.35 
    w_radps = (steer / 100.0) * 1.5 
    
    command = f"{v_mps:.3f} {w_radps:.3f}\n"
    arduino.write(command.encode('utf-8'))
    
    while arduino.in_waiting > 0:
        arduino.readline()

# ==========================================
# 3. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_chosen_angle, last_steer_pwm
    
    bins = {angle: 9999 for angle in range(-135, 136, 5)}
    
    left_wall_min = 9999
    right_wall_min = 9999
    front_clear_x = 9999
    closest_dist = 9999
    
    # [데이터 파싱 및 삼각함수 매핑]
    for angle, distance in scan_data:
        if angle > 180: angle -= 360
        if -135 <= angle <= 135 and distance > 0:
            if distance < closest_dist:
                closest_dist = distance

            bin_angle = round(angle / 5) * 5
            if bin_angle in bins and distance < bins[bin_angle]:
                bins[bin_angle] = distance

            # 삼각함수를 통한 X(전후), Y(좌우) 분리 계산
            rad = math.radians(angle)
            x = distance * math.cos(rad)
            y = distance * math.sin(rad)

            # 전방 절대 히트박스 확인
            if abs(y) <= ROBOT_SIDE and x > 0:
                if x < front_clear_x: front_clear_x = x

            # 측면 벽 거리 확인 (전방 40cm 이내의 벽만 스캔)
            if -100 < x < 400:
                if 10 <= angle <= 135:     # 로봇 좌측
                    if y < left_wall_min: left_wall_min = y
                elif -135 <= angle <= -10: # 로봇 우측
                    if abs(y) < right_wall_min: right_wall_min = abs(y)

    # [틈새(Gap) 파악 - 길 찾기용]
    gaps = []
    angles = sorted(bins.keys())
    for i in range(1, len(angles)):
        if abs(bins[angles[i]] - bins[angles[i-1]]) > GAP_THRESHOLD:
            target = angles[i] if bins[angles[i]] > bins[angles[i-1]] else angles[i-1]
            gaps.append(target)

    # ========================================================
    # [경로 스코어링] (점수가 높을수록 좋은 길!)
    # ========================================================
    best_angle = 0
    max_score = -99999 
    
    for angle in range(-90, 91, 5): 
        dist = bins[angle]
        raw_dist = min(dist, 1000)
        
        # 1. 기본 거리 점수 (+ 부여)
        if raw_dist <= 600: score = raw_dist * 1.0
        else: score = 600 + (raw_dist - 600) * 0.5
            
        # 2. 직진 선호 페널티 (- 차감)
        score -= abs(angle) * 3.5
        
        # 3. 틈새(Gap) 보너스 (+ 부여)
        for gap_angle in gaps:
            if abs(angle - gap_angle) <= 10:
                score += 300 
                
        # 4. ★ 의도 기반 측면 방어 (가장 중요) ★
        side_penalty = 0
        
        if abs(angle) <= 5: 
            # [직진 의도] 측면이 절대 마진(차폭 125 + 여유 20 = 145mm)을 깰 때만 피함
            if left_wall_min < ROBOT_SIDE + 20 or right_wall_min < ROBOT_SIDE + 20:
                side_penalty = 800  # 침범 시 강력한 직진 포기 유도
                
        elif angle > 5: 
            # [좌회전 의도] 안쪽(왼쪽) 측면이 긁히지 않게 240mm까지 확인
            if left_wall_min < 240:
                side_penalty = (240 - left_wall_min) * 4.0
                
        elif angle < -5: 
            # [우회전 의도] 안쪽(오른쪽) 측면이 긁히지 않게 240mm까지 확인
            if right_wall_min < 240:
                side_penalty = (240 - right_wall_min) * 4.0

        score -= side_penalty
            
        # 5. 절대 전방 히트박스 충돌 방지
        rad = math.radians(angle)
        x_proj = raw_dist * math.cos(rad)
        y_proj = raw_dist * math.sin(rad)
        if abs(y_proj) < ROBOT_SIDE and x_proj < ROBOT_FRONT:
            score -= 2000 # 무조건 피해야 하는 길
            
        # 최고 점수 갱신
        if score > max_score:
            max_score = score
            best_angle = angle

    # ========================================================
    # [주행 판단 및 속도 제어]
    # ========================================================
    
    # 1. 사면초가 (최고 점수가 0 이하) -> 즉시 후진하여 새로운 각 탐색
    if max_score < 0: 
        return SPEED_REVERSE, (80 if last_chosen_angle < 0 else -80)
    
    # 2. 전방은 15cm 이내로 막혔으나 옆은 뚫림 -> 멈춰서 팽이 회전 (Pivot)
    if front_clear_x < 150:
        steer_val = 90 if best_angle > 0 else -90
        return ESCAPE_SPEED, steer_val

    # 3. 일반 주행 속도 변속 로직
    if closest_dist < DANGER_RADIUS + ROBOT_SIDE: 
        speed = SPEED_SAFETY
    elif closest_dist < SAFE_RADIUS: 
        speed = SPEED_DRIVE
    else: 
        speed = SPEED_MAX
        
    last_chosen_angle = best_angle
    target_steer_pwm = int(best_angle * STEER_GAIN)
    
    # 조향값이 속도의 80%를 넘지 않게 제한 (급정거 방지)
    max_allowed_steer = int(speed * 0.8)
    if target_steer_pwm > max_allowed_steer: target_steer_pwm = max_allowed_steer
    elif target_steer_pwm < -max_allowed_steer: target_steer_pwm = -max_allowed_steer

    # 조향 스무딩 필터 적용 (파르르 떠는 현상 제거)
    steer_pwm = int((SMOOTHING_FACTOR * target_steer_pwm) + ((1.0 - SMOOTHING_FACTOR) * last_steer_pwm))
    last_steer_pwm = steer_pwm

    return speed, steer_pwm

# ==========================================
# 4. 메인 루프
# ==========================================
def main():
    print("[INFO] 라이다 초기화 중...")
    lidar_ser.write(bytes([0xA5, 0x40])) 
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20])) 
    time.sleep(0.5)
    print("[INFO] 의도 기반(Intent) 스코어링 자율주행 시작!")

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
                    send_to_arduino(speed, steer)
                scan_data = []

            if distance > 0:
                scan_data.append((angle, distance))

    except KeyboardInterrupt:
        print("\n[INFO] 안전 정지")
        arduino.write("0.0 0.0\n".encode('utf-8'))
        lidar_ser.write(bytes([0xA5, 0x25]))
        arduino.close()
        lidar_ser.close()

if __name__ == '__main__':
    main()

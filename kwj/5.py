import serial
import time
import math

# ==========================================
# 1. 자율 주행 상수 설정
# ==========================================
# [속도 설정] (0~255 PWM 스케일)
SPEED_MAX = 250        # 뻥 뚫렸을 때 전속력
SPEED_DRIVE = 180      # 일반 주행
SPEED_SAFETY = 140     # 주변에 무언가 감지됐을 때 안전 속도
SPEED_REVERSE = -130   # V자 함정 탈출용 후진
ESCAPE_SPEED = 0       # 제자리 회전 시 직진 속도는 0

STEER_GAIN = 1.3         # 조향 민감도
SMOOTHING_FACTOR = 0.5   # 조향 부드러움 필터 (급커브 진동 방지)

# [거리 및 차체 설정 (mm)]
# 라이다 중심축을 기준으로 한 로봇의 크기 + 절대 마진(20mm)
MARGIN = 20            
ROBOT_FRONT = 115 + MARGIN  # 앞범퍼: 135mm
ROBOT_SIDE = 105 + MARGIN   # 옆면: 125mm
ROBOT_REAR = 130 + MARGIN   # 뒷범퍼: 150mm

SAFE_RADIUS = 300      # 30cm 이내면 속도를 180으로 줄임
DANGER_RADIUS = 50     # 5cm 이내면 속도를 140으로 대폭 줄임
GAP_THRESHOLD = 300    # 30cm 이상의 거리 단차가 있으면 틈새(길)로 판단

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

# ==========================================
# 3. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_chosen_angle, last_steer_pwm
    
    # 270도 사각지대 방어용 (-135도 ~ +135도)
    bins = {angle: 9999 for angle in range(-135, 136, 5)}
    closest_obj_dist = 9999
    
    # 1. 라이다 데이터 파싱 및 필터링
    for angle, distance in scan_data:
        if angle > 180: angle -= 360
        if -135 <= angle <= 135 and distance > 0:
            if distance < closest_obj_dist:
                closest_obj_dist = distance

            bin_angle = round(angle / 5) * 5
            if bin_angle in bins and distance < bins[bin_angle]:
                bins[bin_angle] = distance

    # 2. 틈새(Gap) 리스트 추출 (연속성이 끊기는 숨겨진 길 찾기)
    gaps = []
    angles = sorted(bins.keys())
    for i in range(1, len(angles)):
        diff = abs(bins[angles[i]] - bins[angles[i-1]])
        if diff > GAP_THRESHOLD:
            # 뚫려있는 더 먼 쪽의 각도를 틈새 입구로 기록
            target = angles[i] if bins[angles[i]] > bins[angles[i-1]] else angles[i-1]
            gaps.append(target)

    # 거시적 좌우 공간 크기 비교 (C자 코스용)
    left_vol = sum(min(bins[a], 1000) for a in range(10, 91, 5) if a in bins)
    right_vol = sum(min(bins[a], 1000) for a in range(-90, 0, 5) if a in bins)

    # 3. 각도별 스코어링 (점수가 낮을수록 좋은 길)
    best_angle = 0
    min_score = float('inf')
    
    for angle in range(-90, 91, 5): 
        dist = bins[angle]
        raw_dist = min(dist, 1000)
        
        # [기본 거리 점수] 멀수록 점수가 낮음 (Piecewise: 60cm 이상은 매력도 반감)
        if raw_dist <= 600:
            score = (1000 / (raw_dist + 1))
        else:
            effective_dist = 600 + (raw_dist - 600) * 0.5
            score = (1000 / (effective_dist + 1))
        
        # [틈새 보너스] 틈새 방향이면 점수 대폭 할인 (-)
        for gap_angle in gaps:
            if abs(angle - gap_angle) <= 10:
                score -= 300 
                
        # [거시적 방향 보너스] 한쪽이 압도적으로 넓으면 그 방향 할인 (-)
        if left_vol > right_vol + 1500 and angle > 0: score -= 100
        elif right_vol > left_vol + 1500 and angle < 0: score -= 100
            
        # [절대 마진 보호 (히트박스)] 차체 크기 + 2cm 이내면 절대 불가 (+)
        rad = math.radians(angle)
        x = raw_dist * math.cos(rad)
        y = raw_dist * math.sin(rad)
        if (abs(y) < ROBOT_SIDE) and (x < ROBOT_FRONT):
            score += 5000 
            
        # [중앙 정렬 유도]
        if closest_obj_dist < ROBOT_SIDE + 50:
            score += abs(angle) * 1.5 
            
        # [2단계 폭발적 측면 방어] 어깨가 긁히지 않도록 완벽 밀어내기
        right_wall_min = min((bins[a] * math.sin(math.radians(abs(a)))) for a in range(-90, 0, 5) if a in bins and bins[a] * math.cos(math.radians(a)) < 400) if any(bins[a] * math.cos(math.radians(a)) < 400 for a in range(-90, 0, 5) if a in bins) else 9999
        left_wall_min = min((bins[a] * math.sin(math.radians(a))) for a in range(5, 91, 5) if a in bins and bins[a] * math.cos(math.radians(a)) < 400) if any(bins[a] * math.cos(math.radians(a)) < 400 for a in range(5, 91, 5) if a in bins) else 9999

        if right_wall_min < 240:
            repel = (240 - right_wall_min) * 1.5
            if right_wall_min < 180: repel += (180 - right_wall_min) * 3.0
            if angle > 0: score -= repel            # 우측 위험 -> 좌회전(양수) 점수 할인
            elif angle < 0: score += (repel * 2.0)  # 우측 위험 -> 우회전(음수) 페널티 폭탄

        if left_wall_min < 240:
            repel = (240 - left_wall_min) * 1.5
            if left_wall_min < 180: repel += (180 - left_wall_min) * 3.0
            if angle < 0: score -= repel            # 좌측 위험 -> 우회전(음수) 점수 할인
            elif angle > 0: score += (repel * 2.0)  # 좌측 위험 -> 좌회전(양수) 페널티 폭탄

        # 최고 좋은 길(최저 점수) 갱신
        if score < min_score:
            min_score = score
            best_angle = angle

    # 4. 주행 판단 (정지/후진/속도 제어)
    if min_score > 3000: 
        # 모든 길이 막힘 -> 후진 탈출
        return SPEED_REVERSE, (80 if last_chosen_angle < 0 else -80)
    
    if closest_obj_dist <= 180:
        # 초근접 코앞 위험 -> 직진 0, 제자리 팽이 회전
        steer_val = 90 if best_angle > 0 else -90
        return ESCAPE_SPEED, steer_val

    # 일반 주행 모드 (장애물 거리에 따른 속도 변속)
    if closest_obj_dist < DANGER_RADIUS + ROBOT_SIDE: speed = SPEED_SAFETY
    elif closest_obj_dist < SAFE_RADIUS: speed = SPEED_DRIVE
    else: speed = SPEED_MAX
        
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
    print("[INFO] 심플 아두이노 호환 자율주행 시작! (정지: Ctrl+C)")

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
                    # 1. 최고 지능 알고리즘 연산
                    speed, steer = calculate_steering(scan_data)
                    
                    # 2. 가장 심플한 콤마(,) 포맷으로 아두이노 전송
                    command = f"{speed},{steer}\n"
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

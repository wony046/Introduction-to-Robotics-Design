import serial
import time
import math

# ==========================================
# 1. 자율 주행 상수 설정
# ==========================================
SPEED_MAX = 250        
SPEED_DRIVE = 180      
SPEED_SAFETY = 140     
SPEED_REVERSE = -130   
ESCAPE_SPEED = 0       

# [거리 설정 (mm)]
MARGIN = 20            
ROBOT_FRONT = 115 + MARGIN  
ROBOT_SIDE = 105 + MARGIN   
ROBOT_REAR = 130 + MARGIN   

SAFE_RADIUS = 300      
DANGER_RADIUS = 50     
GAP_THRESHOLD = 300    

# [통신 설정]
ARDUINO_PORT = '/dev/ttyAMA3'
LIDAR_PORT = '/dev/ttyUSB0'

# ==========================================
# 2. 초기화 및 전역 변수
# ==========================================
arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT, 460800, timeout=1)

last_chosen_angle = 0

# ==========================================
# 3. 아두이노 전송 함수 (헤딩 무력화 포함)
# ==========================================
def send_to_arduino(speed, steer):
    arduino.write("R\n".encode('utf-8')) # 아두이노 헤딩 리셋 (해킹)
    
    v_mps = (speed / 250.0) * 0.35 
    w_radps = (steer / 100.0) * 1.5 
    
    command = f"{v_mps:.3f} {w_radps:.3f}\n"
    arduino.write(command.encode('utf-8'))
    
    while arduino.in_waiting > 0:
        arduino.readline()

# ==========================================
# 4. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_chosen_angle
    
    bins = {angle: 9999 for angle in range(-135, 136, 5)}
    closest_obj_dist = 9999
    
    for angle, distance in scan_data:
        if angle > 180: angle -= 360
        if -135 <= angle <= 135 and distance > 0:
            bin_angle = round(angle / 5) * 5
            if bin_angle in bins and distance < bins[bin_angle]:
                bins[bin_angle] = distance
            if distance < closest_obj_dist:
                closest_obj_dist = distance

    # [단계 1] 틈새(Gap) 탐색
    gaps = []
    angles = sorted(bins.keys())
    for i in range(1, len(angles)):
        diff = abs(bins[angles[i]] - bins[angles[i-1]])
        if diff > GAP_THRESHOLD:
            target = angles[i] if bins[angles[i]] > bins[angles[i-1]] else angles[i-1]
            gaps.append(target)

    # 거시적 볼륨(C자 코스용)
    left_vol = sum(min(bins[a], 1000) for a in range(10, 91, 5) if a in bins)
    right_vol = sum(min(bins[a], 1000) for a in range(-90, 0, 5) if a in bins)

    # [단계 2] 각도별 스코어링 (점수가 낮을수록 좋은 길)
    best_angle = 0
    min_score = float('inf')
    
    for angle in range(-90, 91, 5): 
        dist = bins[angle]
        raw_dist = min(dist, 1000)
        
        # 1. 기본 점수 (멀수록 점수 낮음)
        if raw_dist <= 600: score = (1000 / (raw_dist + 1))
        else: score = (1000 / (600 + (raw_dist - 600)*0.5 + 1))
        
        # 2. 틈새 보너스 (-)
        for gap_angle in gaps:
            if abs(angle - gap_angle) <= 10:
                score -= 300  # ★ 부호 수정: 점수를 깎아서 유도함
        
        # 3. 관성 보너스 (갈지자 주행 방지)
        if (last_chosen_angle > 0 and angle > 0) or (last_chosen_angle < 0 and angle < 0):
            score -= 50       # ★ 부호 수정: 가던 방향 유지 시 점수 할인
            
        # 4. 거시적 방향 보너스 (-)
        if left_vol > right_vol + 1500 and angle > 0:
            score -= 100      # ★ 부호 수정: 왼쪽이 넓으면 좌회전 점수 할인
        elif right_vol > left_vol + 1500 and angle < 0:
            score -= 100
            
        # 5. 절대 마진 보호 (히트박스) (+)
        rad = math.radians(angle)
        x = raw_dist * math.cos(rad)
        y = raw_dist * math.sin(rad)
        
        if (abs(y) < ROBOT_SIDE) and (x < ROBOT_FRONT):
            score += 5000     # 이건 절대 못 가는 길 (강력한 페널티)
            
        # 6. 중앙 정렬 가중치 (+)
        if closest_obj_dist < ROBOT_SIDE + 50:
            score += abs(angle) * 1.5 
            
        # 7. ★ 폭발적 측면 방어 (부호 완벽 수정) ★
        # 점수를 낮춰야 그쪽으로 핸들을 꺾습니다!
        right_wall_min = min((bins[a] * math.sin(math.radians(abs(a)))) for a in range(-90, 0, 5) if a in bins and bins[a] * math.cos(math.radians(a)) < 400) if any(bins[a] * math.cos(math.radians(a)) < 400 for a in range(-90, 0, 5) if a in bins) else 9999
        left_wall_min = min((bins[a] * math.sin(math.radians(a))) for a in range(5, 91, 5) if a in bins and bins[a] * math.cos(math.radians(a)) < 400) if any(bins[a] * math.cos(math.radians(a)) < 400 for a in range(5, 91, 5) if a in bins) else 9999

        if right_wall_min < 240:
            repel = (240 - right_wall_min) * 1.5
            if right_wall_min < 180: repel += (180 - right_wall_min) * 3.0
            if angle > 0: score -= repel            # 우측 벽 근접 시 -> 좌회전(양수) 점수 대폭 할인! (살길)
            elif angle < 0: score += (repel * 2.0)  # 우측 벽 근접 시 -> 우회전(음수) 점수 폭탄! (죽을길)

        if left_wall_min < 240:
            repel = (240 - left_wall_min) * 1.5
            if left_wall_min < 180: repel += (180 - left_wall_min) * 3.0
            if angle < 0: score -= repel            # 좌측 벽 근접 시 -> 우회전(음수) 점수 대폭 할인! (살길)
            elif angle > 0: score += (repel * 2.0)  # 좌측 벽 근접 시 -> 좌회전(양수) 점수 폭탄! (죽을길)
            
        # 최종 최고 점수(최저치) 갱신
        if score < min_score:
            min_score = score
            best_angle = angle

    # [단계 3] 주행 판단
    # 모든 길이 막힘 -> 후진
    if min_score > 3000: 
        return SPEED_REVERSE, (80 if last_chosen_angle < 0 else -80)
    
    # 초근접 위험 -> 제자리 팽이 회전
    if closest_obj_dist <= 180:
        steer_val = 90 if best_angle > 0 else -90
        return ESCAPE_SPEED, steer_val

    # 일반 주행 속도 제어
    if closest_obj_dist < DANGER_RADIUS + ROBOT_SIDE: speed = SPEED_SAFETY
    elif closest_obj_dist < SAFE_RADIUS: speed = SPEED_DRIVE
    else: speed = SPEED_MAX
        
    last_chosen_angle = best_angle
    steer = int(best_angle * 1.5) 
    
    return speed, steer

# ==========================================
# 5. 메인 루프
# ==========================================
def main():
    print("[INFO] 라이다 초기화 중...")
    lidar_ser.write(bytes([0xA5, 0x40])) 
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20])) 
    time.sleep(0.5)
    print("[INFO] 부호 픽스 완료! 무적의 반응형 주행 시작!")

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

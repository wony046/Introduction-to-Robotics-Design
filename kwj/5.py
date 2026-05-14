import serial
import time
import math

# ==========================================
# 1. 자율 주행 상수 설정 (쉽게 변경 가능)
# ==========================================
# [속도 설정]
SPEED_MAX = 250        # 전속력 모드
SPEED_DRIVE = 180      # 일반 주행 모드
SPEED_SAFETY = 140     # 초근접 안전 모드
SPEED_REVERSE = -130   # 후진 모드

# [거리 설정 (mm)]
MARGIN = 20            # 절대 사수 마진 (2cm)
ROBOT_FRONT = 115 + MARGIN  # 135mm
ROBOT_SIDE = 105 + MARGIN   # 125mm
ROBOT_REAR = 130 + MARGIN   # 150mm

SAFE_RADIUS = 300      # 30cm (속도 줄이는 기준)
DANGER_RADIUS = 50     # 5cm (안전 모드 진입 기준)
GAP_THRESHOLD = 200    # 틈새 이론 발동 거리 차이 (200cm)

# [통신 설정]
ARDUINO_PORT = '/dev/ttyAMA3'
LIDAR_PORT = '/dev/ttyUSB0'

# ==========================================
# 2. 초기화 및 전역 변수
# ==========================================
arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT, 460800, timeout=1)

# 이전 주행 방향 기록 (역주행 방지용)
last_chosen_angle = 0

def calculate_steering(scan_data):
    global last_chosen_angle
    
    # 270도 감지 (-135 ~ +135), 5도 단위
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

    # [단계 1] 틈새(Gap) 리스트 추출
    gaps = []
    angles = sorted(bins.keys())
    for i in range(1, len(angles)):
        diff = abs(bins[angles[i]] - bins[angles[i-1]])
        if diff > GAP_THRESHOLD:
            # 더 먼 쪽의 각도를 틈새 입구로 기록
            target = angles[i] if bins[angles[i]] > bins[angles[i-1]] else angles[i-1]
            gaps.append(target)

    # [단계 2] 각도별 스코어링 (점수가 낮을수록 우수)
    best_angle = 0
    min_score = float('inf')
    
    for angle in range(-90, 91, 5): # 주행 결정은 정면 180도 내에서
        dist = bins[angle]
        
        # 기본 점수: 거리의 역수 (멀수록 점수 낮음)
        score = (1000 / (dist + 1)) 
        
        # 1. 틈새 보너스 (Gap Theory)
        for gap_angle in gaps:
            if abs(angle - gap_angle) <= 10:
                score -= 500 # 강력한 유도 점수
        
        # 2. 역주행 방지 페널티 (왔던 길은 피함)
        # 내가 방금 선택했던 방향의 정반대(180도) 부근 점수를 대폭 높임
        opposite_angle = last_chosen_angle + 180
        if opposite_angle > 180: opposite_angle -= 360
        if abs(angle - opposite_angle) < 60:
            score += 800
            
        # 3. 절대 마진 보호 (2cm 이내면 스코어 폭발)
        rad = math.radians(angle)
        x = dist * math.cos(rad)
        y = dist * math.sin(rad)
        
        if (abs(y) < ROBOT_SIDE) and (x < ROBOT_FRONT):
            score += 5000 # 절대 못가는 길
            
        # 4. 중앙 정렬 가중치 (5cm 이내일 때 활성화)
        if closest_obj_dist < ROBOT_SIDE + 50:
            score += abs(angle) * 2 # 최대한 정면을 보게 함
            
        if score < min_score:
            min_score = score
            best_angle = angle

    # [단계 3] 속도 및 주행 상태 결정
    if min_score > 4000: # 모든 길이 막혔다고 판단될 때
        return SPEED_REVERSE, 0 # 후진 탈출
    
    # 속도 모드 결정
    if closest_obj_dist < DANGER_RADIUS + ROBOT_SIDE: # 5cm 이내
        speed = SPEED_SAFETY
    elif closest_obj_dist < SAFE_RADIUS: # 30cm 이내
        speed = SPEED_DRIVE
    else:
        speed = SPEED_MAX
        
    last_chosen_angle = best_angle
    steer = int(best_angle * 1.5) # 조향 민감도 보정
    
    return speed, steer

def main():
    print("[INFO] 라이다 초기화 및 전방 기록 중...")
    lidar_ser.write(bytes([0xA5, 0x40])) 
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20])) 
    time.sleep(0.5)

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
                    command = f"{speed},{steer}\n"
                    arduino.write(command.encode('utf-8'))
                scan_data = []

            if distance > 0:
                scan_data.append((angle, distance))

    except KeyboardInterrupt:
        arduino.write("0,0\n".encode('utf-8'))
        lidar_ser.write(bytes([0xA5, 0x25]))
        arduino.close()
        lidar_ser.close()

if __name__ == '__main__':
    main()

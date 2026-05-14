import serial
import time
import math

# ==========================================
# 1. 자율 주행 상수 설정 (쉽게 변경 가능)
# ==========================================
# [속도 설정 (내부 연산용 0~250 스케일)]
SPEED_MAX = 250        # 전속력 모드
SPEED_DRIVE = 180      # 일반 주행 모드
SPEED_SAFETY = 140     # 초근접 안전 모드
SPEED_REVERSE = -130   # 후진 모드
ESCAPE_SPEED = 0       # 제자리 회전 시 직진 속도는 0

# [거리 설정 (mm)]
MARGIN = 20            # 절대 사수 마진 (2cm)
ROBOT_FRONT = 115 + MARGIN  # 135mm
ROBOT_SIDE = 105 + MARGIN   # 125mm
ROBOT_REAR = 130 + MARGIN   # 150mm

SAFE_RADIUS = 300      # 30cm (속도 줄이는 기준)
DANGER_RADIUS = 50     # 5cm (안전 모드 진입 기준)
GAP_THRESHOLD = 300    # 틈새 이론 발동 거리 차이 (30cm)

# [통신 설정]
ARDUINO_PORT = '/dev/ttyAMA3'
LIDAR_PORT = '/dev/ttyUSB0'

# ==========================================
# 2. 초기화 및 전역 변수
# ==========================================
arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT, 460800, timeout=1)

last_chosen_angle = 0
is_currently_escaping = False  # ESC 모드 상태 추적

# ==========================================
# 3. 데이터 변환기 (Python -> Arduino v5)
# ==========================================
def send_to_arduino(speed, steer, trigger_esc):
    global is_currently_escaping
    
    # 1. ESC 모드 진입 감지 및 명령 전송
    if trigger_esc and not is_currently_escaping:
        arduino.write("ESC\n".encode('utf-8'))
        print("[System] 탈출 모드(ESC) 발동! 헤딩가드 해제")
        time.sleep(0.05) # 아두이노 처리 시간 대기
        is_currently_escaping = True
    elif not trigger_esc and is_currently_escaping:
        is_currently_escaping = False # 안전 구역으로 나오면 상태 초기화

    # 2. 스케일 변환 (내부 수치 -> 물리 단위 m/s, rad/s)
    # 속도(v): 250일 때 약 0.35 m/s 로 매핑
    v_mps = (speed / 250.0) * 0.35 
    
    # 각속도(w): steer(조향값)를 rad/s로 변환. (양수=좌회전, 음수=우회전)
    # steer 100일 때 약 1.5 rad/s 로 매핑
    w_radps = (steer / 100.0) * 1.5 
    
    # 3. 아두이노 v5 포맷 전송 ("v w\n")
    command = f"{v_mps:.3f} {w_radps:.3f}\n"
    arduino.write(command.encode('utf-8'))
    
    # 아두이노가 보내는 "H:XX.X\n" 버퍼 비우기 (메모리 오버플로우 방지)
    while arduino.in_waiting > 0:
        arduino.readline()

# ==========================================
# 4. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_chosen_angle
    
    bins = {angle: 9999 for angle in range(-135, 136, 5)}
    closest_obj_dist = 9999
    trigger_esc = False  # 아두이노 ESC 발동 플래그
    
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
            target = angles[i] if bins[angles[i]] > bins[angles[i-1]] else angles[i-1]
            gaps.append(target)

    # [단계 2] 각도별 스코어링
    best_angle = 0
    min_score = float('inf')
    
    for angle in range(-90, 91, 5): 
        dist = bins[angle]
        score = (1000 / (dist + 1)) 
        
        # 1. 틈새 보너스
        for gap_angle in gaps:
            if abs(angle - gap_angle) <= 10:
                score -= 500 
        
        # 2. 역주행 방지 페널티
        opposite_angle = last_chosen_angle + 180
        if opposite_angle > 180: opposite_angle -= 360
        if abs(angle - opposite_angle) < 60:
            score += 800
            
        # 3. 절대 마진 보호 (히트박스)
        rad = math.radians(angle)
        x = dist * math.cos(rad)
        y = dist * math.sin(rad)
        
        if (abs(y) < ROBOT_SIDE) and (x < ROBOT_FRONT):
            score += 5000 
            
        # 4. 중앙 정렬 가중치
        if closest_obj_dist < ROBOT_SIDE + 50:
            score += abs(angle) * 2 
            
        if score < min_score:
            min_score = score
            best_angle = angle

    # [단계 3] 위기 탈출 및 속도 제어
    # 길이 완전히 막힘 -> 후진 턴 발동 및 ESC 트리거
    if min_score > 4000: 
        trigger_esc = True
        return SPEED_REVERSE, (80 if last_chosen_angle < 0 else -80), trigger_esc
    
    # 초근접 위험 -> 제자리 회전(피벗) 턴 발동 및 ESC 트리거
    if closest_obj_dist <= 180:
        trigger_esc = True
        steer_val = 90 if best_angle > 0 else -90
        return ESCAPE_SPEED, steer_val, trigger_esc

    # 일반 주행 모드
    if closest_obj_dist < DANGER_RADIUS + ROBOT_SIDE: 
        speed = SPEED_SAFETY
    elif closest_obj_dist < SAFE_RADIUS: 
        speed = SPEED_DRIVE
    else:
        speed = SPEED_MAX
        
    last_chosen_angle = best_angle
    steer = int(best_angle * 1.5) 
    
    return speed, steer, trigger_esc

# ==========================================
# 5. 메인 루프
# ==========================================
def main():
    print("[INFO] 라이다 초기화 및 아두이노 동기화 중...")
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
                    # speed, steer, esc 발동 여부 3가지를 리턴받음
                    speed, steer, trigger_esc = calculate_steering(scan_data)
                    # 아두이노 v5 포맷에 맞춰 변환 후 전송
                    send_to_arduino(speed, steer, trigger_esc)
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

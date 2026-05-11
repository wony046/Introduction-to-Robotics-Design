import serial
import time
import math
import cv2
import numpy as np

# ==========================================
# 1. 통신 및 시각화 설정
# ==========================================
ARDUINO_PORT = '/dev/ttyS0'   
LIDAR_PORT = '/dev/ttyUSB0'   

arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT, 460800, timeout=1)

# 시각화 상수 (PDF 기준)
MAP_SIZE = 800
MAP_CENTER = MAP_SIZE // 2
SCALE = 0.2  # 1mm당 0.2픽셀 (5m가 화면 끝)

# ==========================================
# 2. 자율 주행 파라미터 (동일)
# ==========================================
SAFE_DISTANCE = 600
MAX_SPEED = 250
AVOID_SPEED = 120
ESCAPE_SPEED = 90
STEER_GAIN = 1.5
ROBOT_HALF_WIDTH = 115
last_avoid_dir = 0

# ==========================================
# 3. 시각화 보조 함수 (PDF 내용 반영)
# ==========================================
def draw_guides(img):
    # 거리 가이드 원 (1m, 2m, 3m ...)
    for r in range(1000, 5001, 1000):
        cv2.circle(img, (MAP_CENTER, MAP_CENTER), int(r * SCALE), (50, 50, 50), 1)
    # 각도 가이드 선
    for a in range(0, 360, 45):
        rad = math.radians(a)
        x = int(MAP_CENTER + 400 * math.cos(rad))
        y = int(MAP_CENTER + 400 * math.sin(rad))
        cv2.line(img, (MAP_CENTER, MAP_CENTER), (x, y), (50, 50, 50), 1)

def visualize(scan_data, speed, steer):
    # 검은색 배경 생성
    map_img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
    draw_guides(map_img)

    for angle, distance in scan_data:
        # 각도 보정 (라이다 전방이 0도인 경우)
        angle_rad = math.radians(angle - 90) # 90도 회전하여 위쪽을 전방으로 표시
        
        # 좌표 변환 (mm -> pixel)
        x = int(MAP_CENTER + (distance * SCALE) * math.cos(angle_rad))
        y = int(MAP_CENTER + (distance * SCALE) * math.sin(angle_rad))

        if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
            # 장애물 점 찍기 (하얀색)
            cv2.circle(map_img, (x, y), 2, (255, 255, 255), -1)

    # 로봇 본체 표시 (초록색 원)
    cv2.circle(map_img, (MAP_CENTER, MAP_CENTER), int(ROBOT_HALF_WIDTH * 2 * SCALE), (0, 255, 0), 2)
    
    # 조향 방향 표시 (빨간색 선)
    steer_rad = math.radians(steer / STEER_GAIN - 90)
    sx = int(MAP_CENTER + 100 * math.cos(steer_rad))
    sy = int(MAP_CENTER + 100 * math.sin(steer_rad))
    cv2.line(map_img, (MAP_CENTER, MAP_CENTER), (sx, sy), (0, 0, 255), 3)

    # 정보 텍스트
    cv2.putText(map_img, f"Speed: {speed}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(map_img, f"Steer: {steer}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    cv2.imshow("LiDAR Autonomous Drive", map_img)
    cv2.waitKey(1)

# ==========================================
# 3. 핵심 회피 알고리즘 (기존과 동일)
# ==========================================
def calculate_steering(scan_data):
    global last_avoid_dir
    bins = {angle: 9999 for angle in range(-90, 91, 10)}
    front_emergency_dist = 9999
    front_clear_x = 9999
    left_wall_min = 9999
    right_wall_min = 9999
    
    for angle, distance in scan_data:
        temp_angle = angle
        if temp_angle > 180: temp_angle -= 360
            
        if -90 <= temp_angle <= 90 and distance > 0:
            bin_angle = round(temp_angle / 10) * 10
            if bin_angle in bins and distance < bins[bin_angle]:
                bins[bin_angle] = distance
            
            x_pos = distance * math.cos(math.radians(temp_angle))
            y_pos = distance * math.sin(math.radians(temp_angle))
            
            if -20 <= temp_angle <= 20:
                if distance < front_emergency_dist: front_emergency_dist = distance
            if x_pos > 50 and abs(y_pos) <= ROBOT_HALF_WIDTH:
                if x_pos < front_clear_x: front_clear_x = x_pos
            if -100 < x_pos < 400:
                if temp_angle >= 30:
                    if y_pos < left_wall_min: left_wall_min = y_pos
                elif temp_angle <= -30:
                    if abs(y_pos) < right_wall_min: right_wall_min = abs(y_pos)

    if front_emergency_dist < 200:
        emergency_steer = 75 if last_avoid_dir == -1 else -75
        return 0, emergency_steer  

    best_angle = 0
    best_score = -99999
    for angle in range(-90, 91, 10):
        dist_score = min(bins[angle], 400) * 1.0
        center_penalty = abs(angle) * 3.5
        score = dist_score - center_penalty
        if score > best_score:
            best_score = score
            best_angle = angle

    last_avoid_dir = 1 if best_angle < -15 else (-1 if best_angle > 15 else 0)
    steer_pwm = int(best_angle * STEER_GAIN)
    
    if front_clear_x <= 400: current_speed = ESCAPE_SPEED
    elif abs(best_angle) <= 15 and front_clear_x > SAFE_DISTANCE: current_speed = MAX_SPEED
    else: current_speed = AVOID_SPEED
    
    return current_speed, steer_pwm

# ==========================================
# 4. 메인 루프
# ==========================================
def main():
    print("[INFO] 라이다 초기화 및 시각화 준비...")
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
            angle_q6 = ((data[1] >> 1) | (data[2] << 7))
            angle = angle_q6 / 64.0
            distance_q2 = (data[3] | (data[4] << 8))
            distance = distance_q2 / 4.0

            if s_flag == 1:
                if len(scan_data) > 50: 
                    # 1. 판단
                    speed, steer = calculate_steering(scan_data)
                    # 2. 시각화 (추가됨)
                    visualize(scan_data, speed, steer)
                    # 3. 전송
                    command = f"{speed},{steer}\n"
                    arduino.write(command.encode('utf-8'))
                scan_data = []

            if distance > 0:
                scan_data.append((angle, distance))

    except KeyboardInterrupt:
        print("\n[INFO] 종료 중...")
        arduino.write("0,0\n".encode('utf-8'))
        cv2.destroyAllWindows()
        lidar_ser.write(bytes([0xA5, 0x25]))
        arduino.close()
        lidar_ser.close()

if __name__ == '__main__':
    main()

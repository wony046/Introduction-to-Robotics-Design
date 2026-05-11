import serial
import time
import math
from collections import deque
import numpy as np

# ==========================================
# 1. 통신 포트 설정
# ==========================================
ARDUINO_PORT = '/dev/ttyS0'
LIDAR_PORT = '/dev/ttyUSB0'

arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT, 460800, timeout=1)

# ==========================================
# 2. 파라미터
# ==========================================
SAFE_DISTANCE = 400
MAX_SPEED = 250
AVOID_SPEED = 120
ESCAPE_SPEED = 90
STEER_GAIN = 1.5
ROBOT_HALF_WIDTH = 115

last_avoid_dir = 0
frame_buffer = deque(maxlen=3)

# ==========================================
# 3. 전처리 함수 (calculate_steering 위에 정의!)
# ==========================================
def to_xy(scan_data):
    pts = []
    for angle, dist in scan_data:
        if dist <= 0: continue
        x = dist * math.cos(math.radians(angle))
        y = dist * math.sin(math.radians(angle))
        pts.append((x, y))
    return pts

def ransac_remove_walls(pts, iterations=40, thresh=35, min_inliers=15):
    if len(pts) < min_inliers:
        return pts
    pts_np = np.array(pts)
    remaining = list(range(len(pts_np)))

    for _ in range(2):
        if len(remaining) < min_inliers:
            break
        subset = pts_np[remaining]
        best_inliers = []

        for _ in range(iterations):
            idx = np.random.choice(len(subset), 2, replace=False)
            p1, p2 = subset[idx[0]], subset[idx[1]]
            denom = np.linalg.norm(p2 - p1)
            if denom < 1: continue
            dists = np.abs(np.cross(p2 - p1, p1 - subset) / denom)
            inliers = np.where(dists < thresh)[0].tolist()
            if len(inliers) > len(best_inliers):
                best_inliers = inliers

        inlier_set = set(best_inliers)
        remaining = [remaining[i] for i in range(len(remaining))
                     if i not in inlier_set]

    return [tuple(pts_np[i]) for i in remaining]

def get_robust_front_dist(scan_data):
    pts = to_xy(scan_data)
    obstacle_pts = ransac_remove_walls(pts)

    front_dists = []
    for (x, y) in obstacle_pts:
        angle = math.degrees(math.atan2(y, x))
        dist = math.sqrt(x**2 + y**2)
        if -20 <= angle <= 20 and dist > 0:
            front_dists.append(dist)

    if len(front_dists) < 3:
        return 9999

    front_dists.sort()
    idx = max(0, int(len(front_dists) * 0.2))
    current_dist = front_dists[idx]

    frame_buffer.append(current_dist)
    if len(frame_buffer) < 3:
        return 9999

    return max(frame_buffer)

# ==========================================
# 4. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_avoid_dir

    bins = {angle: 9999 for angle in range(-90, 91, 10)}

    # 전처리된 값 사용 (덮어쓰지 않음!)
    front_emergency_dist = get_robust_front_dist(scan_data)
    front_clear_x = 9999
    left_wall_min = 9999
    right_wall_min = 9999

    for angle, distance in scan_data:
        if angle > 180: angle -= 360
        if -90 <= angle <= 90 and distance > 0:
            bin_angle = round(angle / 10) * 10
            if bin_angle in bins and distance < bins[bin_angle]:
                bins[bin_angle] = distance

            x_pos = distance * math.cos(math.radians(angle))
            y_pos = distance * math.sin(math.radians(angle))

            # front_emergency_dist 덮어쓰는 코드 제거!

            if x_pos > 50 and abs(y_pos) <= ROBOT_HALF_WIDTH:
                if x_pos < front_clear_x:
                    front_clear_x = x_pos

            if -100 < x_pos < 400:
                if angle >= 30:
                    if y_pos < left_wall_min: left_wall_min = y_pos
                elif angle <= -30:
                    if abs(y_pos) < right_wall_min: right_wall_min = abs(y_pos)

    if front_emergency_dist < 200:
        if last_avoid_dir == -1:
            emergency_steer = 75
        elif last_avoid_dir == 1:
            emergency_steer = -75
        else:
            if left_wall_min > right_wall_min:
                emergency_steer = 75
                last_avoid_dir = -1
            else:
                emergency_steer = -75
                last_avoid_dir = 1
        return 0, emergency_steer

    best_angle = 0
    best_score = -99999

    for angle in range(-90, 91, 10):
        dist = bins[angle]
        dist_score = min(dist, 400) * 1.0
        center_penalty = abs(angle) * 3.5

        hysteresis_bonus = 0
        if front_clear_x > 300:
            if last_avoid_dir == 1 and angle > 10:
                hysteresis_bonus = 150
            elif last_avoid_dir == -1 and angle < -10:
                hysteresis_bonus = 150

        wall_repulsion_bonus = 0
        if right_wall_min < 200 and left_wall_min > right_wall_min + 40:
            if angle > 10: wall_repulsion_bonus = 150
        elif left_wall_min < 200 and right_wall_min > left_wall_min + 40:
            if angle < -10: wall_repulsion_bonus = 150

        score = dist_score - center_penalty + hysteresis_bonus + wall_repulsion_bonus
        if score > best_score:
            best_score = score
            best_angle = angle

    if best_angle < -15:
        last_avoid_dir = 1
    elif best_angle > 15:
        last_avoid_dir = -1
    else:
        last_avoid_dir = 0

    steer_pwm = int(best_angle * STEER_GAIN)

    if front_clear_x <= 400:
        current_speed = ESCAPE_SPEED
    elif abs(best_angle) <= 15 and front_clear_x > SAFE_DISTANCE:
        current_speed = MAX_SPEED
    else:
        current_speed = AVOID_SPEED

    return current_speed, steer_pwm

# ==========================================
# 5. 메인 루프
# ==========================================
def main():
    print("[INFO] 라이다 초기화 중...")
    lidar_ser.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20]))
    time.sleep(0.5)
    print("[INFO] 자율 주행 시작!")

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
                    arduino.write(f"{speed},{steer}\n".encode('utf-8'))
                scan_data = []

            if distance > 0:
                scan_data.append((angle, distance))

    except KeyboardInterrupt:
        print("\n[INFO] 정지.")
        arduino.write("0,0\n".encode('utf-8'))
        lidar_ser.write(bytes([0xA5, 0x25]))
        arduino.close()
        lidar_ser.close()

if __name__ == '__main__':
    main()

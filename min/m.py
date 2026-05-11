"""
lidar_drive_v3.py — 원본 코드 + 3가지 안정성 개선

추가된 기능:
  1. 라이다 노이즈 필터링 — bin당 최소 N개 점 누적되어야 신뢰
  2. 시간 평활화 — 최근 3 스캔의 best_angle 평균 사용 (떨림 제거)
  3. 비상 정지 타임아웃 — 2초 이상 갇히면 회피 방향 강제 반전
"""

import serial
import time
import math
from collections import deque

# ==========================================
# 1. 통신 포트 설정
# ==========================================
ARDUINO_PORT = '/dev/ttyS0'
LIDAR_PORT   = '/dev/ttyUSB0'

arduino   = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT,  460800, timeout=1)

# ==========================================
# 2. 자율 주행 파라미터
# ==========================================
SAFE_DISTANCE    = 600
MAX_SPEED        = 250
AVOID_SPEED      = 120
ESCAPE_SPEED     = 90
STEER_GAIN       = 1.5
ROBOT_HALF_WIDTH = 115

# ★ 신규 안정성 파라미터
MIN_POINTS_PER_BIN   = 2     # bin 신뢰성 — 점이 이 미만이면 무시 (노이즈 필터)
MIN_POINTS_EMERGENCY = 2     # 비상 정지 트리거 — 단일 튀는 점으로는 안 멈춤
SMOOTHING_WINDOW     = 3     # 최근 N개 best_angle 평균 (조향 떨림 억제)
EMERGENCY_TIMEOUT    = 2.0   # 초 — 이 시간 이상 비상 모드면 회피 방향 반전

# ==========================================
# 3. 전역 상태 변수
# ==========================================
last_avoid_dir       = 0
recent_angles        = deque(maxlen=SMOOTHING_WINDOW)
emergency_start_time = None  # 비상 모드 진입 시각 (None = 비상 아님)


# ==========================================
# 4. 헬퍼: k-번째 최솟값 (노이즈 필터링 핵심)
# ==========================================
def kth_smallest(values, k, default=9999):
    """
    리스트에서 k번째로 작은 값을 반환.
    - 점이 k개 미만이면 default(=무한대 취급) 반환 → 노이즈 자동 제거
    - 단일 튀는 점은 1번째이지만 2번째 점이 없으면 default → 무시됨
    - 진짜 장애물은 보통 점이 여러 개 잡히므로 2번째 점도 가까움 → 검출됨
    """
    if len(values) < k:
        return default
    return sorted(values)[k - 1]


# ==========================================
# 5. 핵심 회피 알고리즘
# ==========================================
def calculate_steering(scan_data):
    global last_avoid_dir, emergency_start_time

    # bin별 거리 리스트 (단일 최솟값 → 다중 점 누적으로 변경)
    bin_dists = {angle: [] for angle in range(-90, 91, 10)}

    # 영역별 측정값 리스트
    front_emergency_dists = []
    front_clear_xs        = []
    left_wall_ys          = []
    right_wall_ys         = []

    for angle, distance in scan_data:
        if angle > 180:
            angle -= 360
        if not (-90 <= angle <= 90 and distance > 0):
            continue

        bin_angle = round(angle / 10) * 10
        if bin_angle in bin_dists:
            bin_dists[bin_angle].append(distance)

        x_pos = distance * math.cos(math.radians(angle))
        y_pos = distance * math.sin(math.radians(angle))

        if -20 <= angle <= 20:
            front_emergency_dists.append(distance)

        if x_pos > 50 and abs(y_pos) <= ROBOT_HALF_WIDTH:
            front_clear_xs.append(x_pos)

        if -100 < x_pos < 400:
            if angle >= 30:
                left_wall_ys.append(y_pos)
            elif angle <= -30:
                right_wall_ys.append(abs(y_pos))

    # ★ [개선 1] 노이즈 필터링 — k-번째 최솟값 적용
    bins = {a: kth_smallest(bin_dists[a], MIN_POINTS_PER_BIN) for a in bin_dists}
    front_emergency_dist = kth_smallest(front_emergency_dists, MIN_POINTS_EMERGENCY)
    front_clear_x        = kth_smallest(front_clear_xs,        MIN_POINTS_PER_BIN)
    left_wall_min        = kth_smallest(left_wall_ys,          MIN_POINTS_PER_BIN)
    right_wall_min       = kth_smallest(right_wall_ys,         MIN_POINTS_PER_BIN)

    # ========================================================
    # [1] 비상 회피 모드 + 타임아웃
    # ========================================================
    if front_emergency_dist < 200:
        now = time.time()

        # ★ [개선 3] 비상 진입 시점 기록 (최초 1회만)
        if emergency_start_time is None:
            emergency_start_time = now
            recent_angles.clear()   # 평활화 버퍼도 초기화

        # ★ 타임아웃: 너무 오래 못 빠져나오면 회피 방향 반전
        elif now - emergency_start_time > EMERGENCY_TIMEOUT:
            if last_avoid_dir != 0:
                last_avoid_dir = -last_avoid_dir
            emergency_start_time = now  # 타이머 재시작
            print(f"[WARN] 비상 타임아웃 — 회피 방향 반전 ({last_avoid_dir})")

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

    # 비상 모드 종료 시 타이머 리셋
    emergency_start_time = None

    # ========================================================
    # [2] 스코어링 모드
    # ========================================================
    best_angle = 0
    best_score = -99999

    for angle in range(-90, 91, 10):
        dist = bins[angle]
        dist_score     = min(dist, 400) * 1.0
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

    # ★ [개선 2] 시간 평활화 — 최근 N개 평균 사용
    recent_angles.append(best_angle)
    smoothed_angle = sum(recent_angles) / len(recent_angles)

    # 상태 업데이트 (평활화된 각도 기준)
    if smoothed_angle < -15:
        last_avoid_dir = 1
    elif smoothed_angle > 15:
        last_avoid_dir = -1
    else:
        last_avoid_dir = 0

    steer_pwm = int(smoothed_angle * STEER_GAIN)

    # ========================================================
    # [3] 속도 결정 (평활화된 각도 기준)
    # ========================================================
    if front_clear_x <= 400:
        current_speed = ESCAPE_SPEED
    elif abs(smoothed_angle) <= 15 and front_clear_x > SAFE_DISTANCE:
        current_speed = MAX_SPEED
    else:
        current_speed = AVOID_SPEED

    return current_speed, steer_pwm


# ==========================================
# 6. 메인 루프
# ==========================================
def main():
    print("[INFO] 라이다 초기화 중...")
    lidar_ser.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20]))
    time.sleep(0.5)
    print("[INFO] 자율 주행 시작! (정지: Ctrl+C)")
    print(f"[INFO] 노이즈 필터링: bin당 최소 {MIN_POINTS_PER_BIN}점, 비상 {MIN_POINTS_EMERGENCY}점")
    print(f"[INFO] 평활화: 최근 {SMOOTHING_WINDOW}회 평균")
    print(f"[INFO] 비상 타임아웃: {EMERGENCY_TIMEOUT}s")

    scan_data = []

    try:
        while True:
            data = lidar_ser.read(5)
            if len(data) != 5:
                continue

            s_flag     = data[0] & 0x01
            s_inv_flag = (data[0] & 0x02) >> 1
            if s_inv_flag != (1 - s_flag):
                continue
            if (data[1] & 0x01) != 1:
                continue

            angle_q6    = ((data[1] >> 1) | (data[2] << 7))
            angle       = angle_q6 / 64.0
            distance_q2 = (data[3] | (data[4] << 8))
            distance    = distance_q2 / 4.0

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

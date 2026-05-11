"""
lidar_drive_vfh.py  (수정본)
==============================
RPLiDAR + Arduino 자율주행 — VFH (Vector Field Histogram) 구현

수정 사항:
  1. RPLiDAR 좌표계 통일 (CW 양의 각도 = 오른쪽)
     - 내부 VFH 좌표: 양(+) = 왼쪽, 음(-) = 오른쪽으로 변환
  2. 직진 가능 상황에서 불필요한 회전 방지 (전방 클리어 판단 강화)
  3. 좌우 교대 회피 알고리즘 추가
  4. 로봇 크기 반영 (라이다 기준 앞뒤좌우 11cm)
  5. steer_pwm 부호 방향 수정

좌표계 정의 (VFH 내부):
  - 0도   = 전방 (직진)
  - +각도 = 왼쪽
  - -각도 = 오른쪽

흐름:
  LiDAR 원시 패킷
    └─► RPLiDAR 각도 → VFH 내부 각도 변환
          └─► 밀도 기반 극좌표 히스토그램 생성
                └─► 가우시안 평활화
                      └─► Valley(통과 가능 섹터) 탐색
                            └─► 최적 Valley 선택 (좌우 교대 우선)
                                  └─► 벽 반발 보정 → 속도 결정 → Arduino 전송
"""

import serial
import time
import math
from collections import deque

# ============================================================
# 1. 통신 포트 설정
# ============================================================
ARDUINO_PORT = '/dev/ttyS0'
LIDAR_PORT   = '/dev/ttyUSB0'

arduino   = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT,  460800, timeout=1)

# ============================================================
# 2. 주행 파라미터
# ============================================================
SAFE_DISTANCE    = 600    # mm — 이 이상 뚫려있어야 풀악셀
MAX_SPEED        = 250    # 풀악셀 속도
AVOID_SPEED      = 120    # 일반 회피 속도
ESCAPE_SPEED     = 90     # 거북이 속도
MIN_WHEEL_SPEED  = 30     # 회전 중에도 느린 쪽 바퀴가 유지할 최소 속도
STEER_GAIN       = 1.5    # 조향각(도) → PWM 변환 계수
ROBOT_HALF_WIDTH = 110    # mm — 로봇 반폭 (라이다 중심에서 좌우 각 11cm)
ROBOT_FRONT      = 110    # mm — 라이다 중심에서 전방 11cm
ROBOT_REAR       = 110    # mm — 라이다 중심에서 후방 11cm

# ============================================================
# 3. VFH 파라미터
# ============================================================
ANGLE_STEP        = 10     # 히스토그램 해상도 (도)
MAX_OBSTACLE_DIST = 1500   # mm — 이 거리 이상 장애물은 무시
VALLEY_THRESHOLD  = 0.16   # certainty² 임계값
VALLEY_MIN_WIDTH  = 20     # 도 — 로봇이 통과할 수 있는 최소 valley 폭
SMOOTH_KERNEL     = [1, 2, 3, 2, 1]  # 가우시안 근사 평활화 커널

# ============================================================
# 4. 스캔 완료율 필터 파라미터
# ============================================================
SCAN_HISTORY_SIZE   = 10
MIN_COMPLETION_RATE = 0.70
SCAN_WARMUP_COUNT   = 3
MIN_POINTS_ABS      = 50

scan_history = deque(maxlen=SCAN_HISTORY_SIZE)

# ============================================================
# 5. 좌우 교대 회피 상태
# ============================================================
# last_avoid_dir: 마지막으로 회피한 방향
#   +1 = 왼쪽으로 회피했음 → 다음에는 오른쪽 우선
#   -1 = 오른쪽으로 회피했음 → 다음에는 왼쪽 우선
#    0 = 직진 중 (교대 없음)
last_avoid_dir  = 0
avoid_count     = 0    # 연속 회피 횟수 (교대 판단용)


# ============================================================
# 6. RPLiDAR 각도 → VFH 내부 각도 변환
# ============================================================
def lidar_to_vfh_angle(raw_angle: float) -> float:
    """
    RPLiDAR 좌표계 → VFH 내부 좌표계 변환.

    RPLiDAR:  0도=전방, CW 증가 (90도=오른쪽, 270도=왼쪽)
    VFH 내부: 0도=전방, 양(+)=왼쪽, 음(-)=오른쪽

    변환: RPLiDAR 0~180 → VFH 0~-180 (오른쪽)
          RPLiDAR 180~360 → VFH +180~0 (왼쪽)
    """
    if raw_angle <= 180:
        return -raw_angle      # 0~180 → 0~-180 (오른쪽)
    else:
        return 360 - raw_angle  # 181~359 → +179~+1 (왼쪽)


# ============================================================
# 7. 스캔 완료율 필터
# ============================================================
def is_scan_valid(scan_size: int) -> bool:
    if len(scan_history) < SCAN_WARMUP_COUNT:
        return scan_size >= MIN_POINTS_ABS
    avg = sum(scan_history) / len(scan_history)
    return (scan_size / avg >= MIN_COMPLETION_RATE) and (scan_size >= MIN_POINTS_ABS)


# ============================================================
# 8. 좌/우 바퀴 PWM 혼합 함수
# ============================================================
def mix_drive(speed: int, steer_pwm: int) -> tuple:
    """
    speed와 steer_pwm을 좌/우 바퀴 PWM으로 변환.

    steer_pwm > 0 → 왼쪽으로 회전 (left 느리게, right 빠르게)
    steer_pwm < 0 → 오른쪽으로 회전 (left 빠르게, right 느리게)

    [speed = 0] 제자리 회전
    [speed > 0] 일반 주행
    """
    if speed == 0:
        # 제자리 회전: steer_pwm > 0이면 왼쪽 회전 (left 후진, right 전진)
        return -steer_pwm, steer_pwm

    # 왼쪽 회전: left 느리게, right 빠르게
    left  = speed - steer_pwm
    right = speed + steer_pwm

    # 느린 쪽 바퀴 최솟값 보장
    slowest = min(left, right)
    if slowest < MIN_WHEEL_SPEED:
        delta  = MIN_WHEEL_SPEED - slowest
        left  += delta
        right += delta

    # 빠른 쪽 바퀴 최댓값 보장 (비율 유지)
    fastest = max(left, right)
    if fastest > MAX_SPEED:
        scale = MAX_SPEED / fastest
        left  = int(left  * scale)
        right = int(right * scale)

    return int(left), int(right)


# ============================================================
# 9. VFH 핵심 함수들
# ============================================================

def build_polar_histogram(scan_data: list) -> dict:
    """
    [VFH Step 1] 밀도 기반 극좌표 히스토그램 생성.

    각 빈(bin)에 해당 방향의 최대 certainty² 값을 기록.
    VFH 내부 좌표계 사용: 양(+)=왼쪽, 음(-)=오른쪽
    """
    hist = {a: 0.0 for a in range(-90, 91, ANGLE_STEP)}

    for raw_angle, distance in scan_data:
        vfh_angle = lidar_to_vfh_angle(raw_angle)

        if not (-90 <= vfh_angle <= 90) or distance <= 0:
            continue

        bin_angle = round(vfh_angle / ANGLE_STEP) * ANGLE_STEP
        bin_angle = max(-90, min(90, bin_angle))  # 범위 클램프

        certainty = max(0.0, 1.0 - distance / MAX_OBSTACLE_DIST)
        hist[bin_angle] = max(hist[bin_angle], certainty ** 2)

    return hist


def smooth_histogram(hist: dict) -> dict:
    """
    [VFH Step 2] 가우시안 근사 평활화.
    """
    half     = len(SMOOTH_KERNEL) // 2
    smoothed = {}

    for a in range(-90, 91, ANGLE_STEP):
        total      = 0.0
        weight_sum = 0.0
        for i, w in enumerate(SMOOTH_KERNEL):
            neighbor = a + (i - half) * ANGLE_STEP
            if neighbor in hist:
                total      += hist[neighbor] * w
                weight_sum += w
        smoothed[a] = total / weight_sum if weight_sum > 0 else 0.0

    return smoothed


def find_valleys(smoothed: dict) -> list:
    """
    [VFH Step 3] 통과 가능한 연속 섹터(Valley) 탐색.

    VALLEY_THRESHOLD 이하인 빈이 VALLEY_MIN_WIDTH 이상 연속되면 valley.
    반환: [(시작각, 끝각), ...] 리스트
    """
    valleys      = []
    in_valley    = False
    valley_start = None

    for a in range(-90, 91, ANGLE_STEP):
        passable = smoothed[a] <= VALLEY_THRESHOLD

        if passable and not in_valley:
            valley_start = a
            in_valley    = True
        elif not passable and in_valley:
            width = a - valley_start
            if width >= VALLEY_MIN_WIDTH:
                valleys.append((valley_start, a - ANGLE_STEP))
            in_valley = False

    if in_valley:
        width = 90 - valley_start + ANGLE_STEP
        if width >= VALLEY_MIN_WIDTH:
            valleys.append((valley_start, 90))

    return valleys


def select_best_valley(valleys: list, prefer_dir: int) -> tuple:
    """
    [VFH Step 4] 최적 Valley 선택 — 좌우 교대 회피 적용.

    우선순위:
      1. 0도를 포함하는 valley (직진 가능) → 즉시 반환
      2. prefer_dir 방향에 있는 valley 중 0도에 가장 가까운 것
      3. 어느 방향이든 0도에 가장 가까운 valley

    prefer_dir:
      +1 = 왼쪽(양의 각도) 우선
      -1 = 오른쪽(음의 각도) 우선
       0 = 방향 무관 (0도에 가장 가까운 것)
    """
    if not valleys:
        return None

    # 1순위: 직진 가능 valley (0도 포함)
    for v in valleys:
        if v[0] <= 0 <= v[1]:
            return v

    # 2순위: prefer_dir 방향의 valley 중 0도에 가장 가까운 것
    if prefer_dir != 0:
        preferred = []
        for v in valleys:
            center = (v[0] + v[1]) / 2
            if prefer_dir > 0 and center > 0:      # 왼쪽 valley
                preferred.append(v)
            elif prefer_dir < 0 and center < 0:     # 오른쪽 valley
                preferred.append(v)

        if preferred:
            return min(preferred, key=lambda v: abs((v[0] + v[1]) / 2))

    # 3순위: 방향 무관, 0도에 가장 가까운 valley
    return min(valleys, key=lambda v: abs((v[0] + v[1]) / 2))


def valley_to_angle(valley: tuple, target: int = 0) -> int:
    """
    Valley 내에서 목표 방향에 가장 가까운 조향각 결정.
    """
    start, end = valley
    if start <= target <= end:
        return target
    center = round((start + end) / 2 / ANGLE_STEP) * ANGLE_STEP
    return center


# ============================================================
# 10. 보조 센서값 계산
# ============================================================

def extract_sensor_values(scan_data: list) -> tuple:
    """
    스캔 데이터에서 4가지 센서값 추출 (VFH 내부 좌표계 사용).

    VFH 좌표계: 양(+)=왼쪽, 음(-)=오른쪽

    반환:
        front_emergency_dist : 전방 ±20도 최단 거리 (비상 정지용)
        front_clear_x        : 차폭(±ROBOT_HALF_WIDTH) 내 전방 장애물 X거리
        left_wall_min        : 좌측 벽 최단 거리
        right_wall_min       : 우측 벽 최단 거리
    """
    front_emergency_dist = 9999.0
    front_clear_x        = 9999.0
    left_wall_min        = 9999.0
    right_wall_min       = 9999.0

    for raw_angle, distance in scan_data:
        vfh_angle = lidar_to_vfh_angle(raw_angle)

        if not (-90 <= vfh_angle <= 90) or distance <= 0:
            continue

        rad   = math.radians(vfh_angle)
        x_pos = distance * math.cos(rad)   # 전방 거리 (항상 양수)
        y_pos = distance * math.sin(rad)   # 양수=왼쪽, 음수=오른쪽

        # [A] 비상 정지용 — 좁은 전방 섹터
        if -20 <= vfh_angle <= 20:
            front_emergency_dist = min(front_emergency_dist, distance)

        # [B] 차폭 내 전방 클리어 거리
        #     로봇 전방 끝(ROBOT_FRONT)보다 먼 장애물만 고려
        if x_pos > ROBOT_FRONT and abs(y_pos) <= ROBOT_HALF_WIDTH + 30:
            front_clear_x = min(front_clear_x, x_pos)

        # [C] 측면 벽 (차체 범위 내)
        if -ROBOT_REAR < x_pos < 400:
            if vfh_angle >= 30:    # 왼쪽 (양의 각도 = 왼쪽)
                left_wall_min = min(left_wall_min, abs(y_pos))
            elif vfh_angle <= -30:  # 오른쪽 (음의 각도 = 오른쪽)
                right_wall_min = min(right_wall_min, abs(y_pos))

    return front_emergency_dist, front_clear_x, left_wall_min, right_wall_min


# ============================================================
# 11. 메인 조향 함수 (VFH 통합 + 좌우 교대 회피)
# ============================================================

def calculate_steering(scan_data: list) -> tuple:
    """
    스캔 데이터 → (left_pwm, right_pwm) 반환.

    단계:
      1. 센서값 추출
      2. [긴급] 비상 정지 & 제자리 회전
      3. [VFH] 히스토그램 → 평활화 → Valley 탐색 → 최적 Valley 선택
      4. [보정] Valley 없음 처리 / 벽 반발 보정
      5. 속도 결정 → mix_drive()로 좌/우 PWM 변환

    좌우 교대 회피:
      - 왼쪽으로 회피했으면 다음 회피 시 오른쪽 우선
      - 오른쪽으로 회피했으면 다음 회피 시 왼쪽 우선
      - 직진 가능하면 교대 상태 유지 (리셋하지 않음)
    """
    global last_avoid_dir, avoid_count

    # ── 1. 센서값 추출 ──────────────────────────────────────
    front_emergency_dist, front_clear_x, left_wall_min, right_wall_min = \
        extract_sensor_values(scan_data)

    # ── 교대 회피 방향 결정 ─────────────────────────────────
    # last_avoid_dir = +1이면 이전에 왼쪽으로 피함 → 이번에는 오른쪽(-1) 우선
    # last_avoid_dir = -1이면 이전에 오른쪽으로 피함 → 이번에는 왼쪽(+1) 우선
    if last_avoid_dir == 1:
        prefer_dir = -1   # 다음엔 오른쪽 우선
    elif last_avoid_dir == -1:
        prefer_dir = +1   # 다음엔 왼쪽 우선
    else:
        # 첫 회피: 더 넓은 쪽으로
        if left_wall_min > right_wall_min:
            prefer_dir = +1   # 왼쪽이 넓으면 왼쪽
        else:
            prefer_dir = -1   # 오른쪽이 넓으면 오른쪽

    # ── 2. 비상 회피 (20cm 이내 코앞 장애물) ────────────────
    if front_emergency_dist < 200:
        if prefer_dir > 0:
            steer = 75       # 왼쪽으로 회전
        else:
            steer = -75      # 오른쪽으로 회전

        # 교대 상태 업데이트: 이번에 회피한 방향 기록
        if steer > 0:
            last_avoid_dir = +1   # 왼쪽으로 회피함
        else:
            last_avoid_dir = -1   # 오른쪽으로 회피함
        avoid_count += 1

        return mix_drive(0, steer)

    # ── 3. VFH: 히스토그램 → 평활화 → Valley 탐색 ──────────
    hist     = build_polar_histogram(scan_data)
    smoothed = smooth_histogram(hist)
    valleys  = find_valleys(smoothed)
    best_v   = select_best_valley(valleys, prefer_dir)

    # ── 4-A. Valley 없음 → 교대 방향으로 탈출 회전 ──────────
    if best_v is None:
        if prefer_dir > 0:
            steer = 60       # 왼쪽으로 탈출
            last_avoid_dir = +1
        else:
            steer = -60      # 오른쪽으로 탈출
            last_avoid_dir = -1
        avoid_count += 1
        return mix_drive(ESCAPE_SPEED, steer)

    # ── 4-B. 조향각 결정 ─────────────────────────────────────
    target_angle = valley_to_angle(best_v, target=0)

    # ── 4-C. 벽 반발 보정 (Wall Repulsion) ──────────────────
    wall_offset = 0
    if right_wall_min < 200 and left_wall_min > right_wall_min + 40:
        wall_offset = +10   # 오른쪽 벽 가까움 → 왼쪽(+)으로 보정
    elif left_wall_min < 200 and right_wall_min > left_wall_min + 40:
        wall_offset = -10   # 왼쪽 벽 가까움 → 오른쪽(-)으로 보정

    final_angle = max(-90, min(90, target_angle + wall_offset))

    # ── 교대 상태 업데이트 ──────────────────────────────────
    if final_angle > 15:
        # 왼쪽으로 회피 중
        last_avoid_dir = +1
        avoid_count += 1
    elif final_angle < -15:
        # 오른쪽으로 회피 중
        last_avoid_dir = -1
        avoid_count += 1
    else:
        # 직진 중 — 교대 상태는 유지 (리셋하지 않음)
        # 연속 회피 카운터만 리셋
        avoid_count = 0

    steer_pwm = int(final_angle * STEER_GAIN)

    # ── 5. 속도 결정 (front_clear_x 기반 3단계) ─────────────
    if front_clear_x <= 400:
        speed = ESCAPE_SPEED
    elif abs(final_angle) <= 15 and front_clear_x > SAFE_DISTANCE:
        speed = MAX_SPEED
    else:
        speed = AVOID_SPEED

    # ── 6. 좌/우 PWM 변환 ────────────────────────────────────
    return mix_drive(speed, steer_pwm)


# ============================================================
# 12. 메인 루프 (LiDAR 패킷 파싱 → Arduino 전송)
# ============================================================

def main():
    print("[INFO] LiDAR 초기화 중...")
    lidar_ser.write(bytes([0xA5, 0x40]))   # RESET
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20]))   # SCAN 시작
    time.sleep(0.5)
    print("[INFO] 자율 주행 시작! (정지: Ctrl+C)")
    print(f"[INFO] VFH 파라미터: step={ANGLE_STEP}°, "
          f"threshold={VALLEY_THRESHOLD}, min_valley={VALLEY_MIN_WIDTH}°")
    print(f"[INFO] 로봇 크기: 전후좌우 {ROBOT_FRONT}mm, 반폭 {ROBOT_HALF_WIDTH}mm")
    print(f"[INFO] 좌우 교대 회피 알고리즘 활성화")

    scan_data     = []
    skipped_scans = 0

    try:
        while True:
            raw = lidar_ser.read(5)
            if len(raw) != 5:
                continue

            # ── 패킷 유효성 검증 ──
            s_flag     = raw[0] & 0x01
            s_inv_flag = (raw[0] & 0x02) >> 1
            if s_inv_flag != (1 - s_flag):
                continue
            if (raw[1] & 0x01) != 1:
                continue

            # ── 각도 / 거리 파싱 ──
            angle    = ((raw[1] >> 1) | (raw[2] << 7)) / 64.0
            distance = (raw[3] | (raw[4] << 8)) / 4.0

            # ── 한 바퀴 완료 시 판단 ──
            if s_flag == 1:
                scan_size = len(scan_data)

                if is_scan_valid(scan_size):
                    left, right = calculate_steering(scan_data)
                    arduino.write(f"{left},{right}\n".encode('utf-8'))
                else:
                    skipped_scans += 1
                    if skipped_scans % 10 == 0 and scan_history:
                        avg = sum(scan_history) / len(scan_history)
                        print(f"[WARN] 불완전 스캔 누적 {skipped_scans}회 "
                              f"(현재={scan_size}pts, 평균={avg:.0f}pts)")

                scan_history.append(scan_size)
                scan_data = []

            if distance > 0:
                scan_data.append((angle, distance))

    except KeyboardInterrupt:
        print(f"\n[INFO] 정지. 총 스킵 스캔={skipped_scans}회")
        arduino.write("0,0\n".encode('utf-8'))
        lidar_ser.write(bytes([0xA5, 0x25]))   # STOP
        arduino.close()
        lidar_ser.close()


if __name__ == '__main__':
    main()

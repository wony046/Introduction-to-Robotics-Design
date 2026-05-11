"""
lidar_drive_vfh.py
==================
RPLiDAR + Arduino 자율주행 — 완전한 VFH (Vector Field Histogram) 구현

흐름:
  LiDAR 원시 패킷
    └─► 밀도 기반 극좌표 히스토그램 생성
          └─► 가우시안 평활화
                └─► Valley(통과 가능 섹터) 탐색
                      └─► 최적 Valley 선택 → 조향각 결정
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
STEER_GAIN       = 1.5    # 조향각(도) → PWM 변환 계수
ROBOT_HALF_WIDTH = 115    # mm — 로봇 반폭 (230mm / 2)

# ============================================================
# 3. VFH 파라미터
# ============================================================
ANGLE_STEP        = 5      # 히스토그램 해상도 (도)
MAX_OBSTACLE_DIST = 2000   # mm — 이 거리 이상은 히스토그램 기여 없음
VALLEY_THRESHOLD  = 0.5    # 이 밀도 이하 빈을 "통과 가능"으로 판정
VALLEY_MIN_WIDTH  = 20     # 도 — 로봇이 통과할 수 있는 최소 valley 폭
SMOOTH_KERNEL     = [1, 2, 3, 2, 1]  # 가우시안 근사 평활화 커널 (±2빈 = ±10도)

# ============================================================
# 4. 스캔 완료율 필터 파라미터
# ============================================================
SCAN_HISTORY_SIZE   = 10   # 최근 몇 회 스캔을 평균 산정에 사용할지
MIN_COMPLETION_RATE = 0.70 # 평균 대비 최소 완료율
SCAN_WARMUP_COUNT   = 3    # 워밍업 — 이 회수는 절대값만 확인
MIN_POINTS_ABS      = 50   # 절대 최솟값 (포인트 수)

scan_history = deque(maxlen=SCAN_HISTORY_SIZE)

# 이전 회피 방향 기억 (비상 정지 연속성 / Hysteresis)
last_avoid_dir = 0   # 1: 우회전, -1: 좌회전, 0: 직진


# ============================================================
# 5. 스캔 완료율 필터
# ============================================================
def is_scan_valid(scan_size: int) -> bool:
    """
    워밍업 구간: 절대 최솟값(MIN_POINTS_ABS) 이상이면 유효.
    이후       : 최근 평균의 MIN_COMPLETION_RATE 이상이어야 유효.
    scan_history 갱신은 호출 측에서 처리.
    """
    if len(scan_history) < SCAN_WARMUP_COUNT:
        return scan_size >= MIN_POINTS_ABS

    avg = sum(scan_history) / len(scan_history)
    return (scan_size / avg >= MIN_COMPLETION_RATE) and (scan_size >= MIN_POINTS_ABS)


# ============================================================
# 6. VFH 핵심 함수들
# ============================================================

def build_polar_histogram(scan_data: list) -> dict:
    """
    [VFH Step 1] 밀도 기반 극좌표 히스토그램 생성.

    각 포인트의 기여도:
        certainty = max(0, 1 - distance / MAX_OBSTACLE_DIST)
        hist[bin] += certainty^2   (가까울수록 기여 급증)

    반환: {각도: 밀도} — 전방 ±90도, ANGLE_STEP 단위
    """
    hist = {a: 0.0 for a in range(-90, 91, ANGLE_STEP)}

    for raw_angle, distance in scan_data:
        angle = raw_angle if raw_angle <= 180 else raw_angle - 360
        if not (-90 <= angle <= 90) or distance <= 0:
            continue

        bin_angle = round(angle / ANGLE_STEP) * ANGLE_STEP
        if bin_angle not in hist:
            continue

        certainty = max(0.0, 1.0 - distance / MAX_OBSTACLE_DIST)
        hist[bin_angle] += certainty ** 2

    return hist


def smooth_histogram(hist: dict) -> dict:
    """
    [VFH Step 2] 가우시안 근사 평활화.

    SMOOTH_KERNEL로 이웃 빈을 가중 평균.
    경계 처리: 존재하는 빈만 합산 (패딩 없음).
    """
    half   = len(SMOOTH_KERNEL) // 2
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


def find_valleys(smoothed: dict) -> list[tuple[int, int]]:
    """
    [VFH Step 3] 통과 가능한 연속 섹터(Valley) 탐색.

    VALLEY_THRESHOLD 이하인 빈이 VALLEY_MIN_WIDTH 이상 연속되면 valley.
    반환: [(시작각, 끝각), ...] 리스트 (없으면 빈 리스트)
    """
    valleys    = []
    in_valley  = False
    valley_start = None

    for a in range(-90, 91, ANGLE_STEP):
        passable = smoothed[a] <= VALLEY_THRESHOLD

        if passable and not in_valley:
            valley_start = a
            in_valley    = True
        elif not passable and in_valley:
            width = (a - ANGLE_STEP) - valley_start
            if width >= VALLEY_MIN_WIDTH:
                valleys.append((valley_start, a - ANGLE_STEP))
            in_valley = False

    # 마지막 valley가 +90도까지 이어진 경우
    if in_valley:
        width = 90 - valley_start
        if width >= VALLEY_MIN_WIDTH:
            valleys.append((valley_start, 90))

    return valleys


def select_best_valley(valleys: list[tuple[int, int]]) -> tuple[int, int] | None:
    """
    [VFH Step 4] 목표 방향(직진 = 0도) 기준 최적 Valley 선택.

    우선순위:
      1. 0도를 포함하는 valley (직진 가능) → 즉시 반환
      2. valley 중심각 기준 0도에 가장 가까운 것
    """
    if not valleys:
        return None

    # 직진 가능 valley 우선
    for v in valleys:
        if v[0] <= 0 <= v[1]:
            return v

    # 중심각 기준 최근접
    return min(valleys, key=lambda v: abs((v[0] + v[1]) / 2))


def valley_to_angle(valley: tuple[int, int], target: int = 0) -> int:
    """
    Valley 내에서 목표 방향에 가장 가까운 조향각 결정.

    - target이 valley 안에 있으면 → target 그대로 (직진)
    - target이 valley 밖이면 → valley 중심각
    """
    start, end = valley
    if start <= target <= end:
        return target
    center = (start + end) // 2
    return center


# ============================================================
# 7. 보조 센서값 계산 (비상 정지 / 측면 벽 / 전방 클리어)
# ============================================================

def extract_sensor_values(scan_data: list) -> tuple[float, float, float, float]:
    """
    스캔 데이터에서 4가지 센서값 추출.

    반환:
        front_emergency_dist : -20 ~ +20도 최단 거리 (비상 정지용)
        front_clear_x        : 차폭(±ROBOT_HALF_WIDTH) 내 전방 장애물 X거리
        left_wall_min        : 좌측 벽 최단 Y거리
        right_wall_min       : 우측 벽 최단 Y거리
    """
    front_emergency_dist = 9999.0
    front_clear_x        = 9999.0
    left_wall_min        = 9999.0
    right_wall_min       = 9999.0

    for raw_angle, distance in scan_data:
        angle = raw_angle if raw_angle <= 180 else raw_angle - 360
        if not (-90 <= angle <= 90) or distance <= 0:
            continue

        x_pos = distance * math.cos(math.radians(angle))
        y_pos = distance * math.sin(math.radians(angle))

        # [A] 비상 정지용 — 좁은 전방 섹터
        if -20 <= angle <= 20:
            front_emergency_dist = min(front_emergency_dist, distance)

        # [B] 차폭 내 전방 클리어 거리
        if x_pos > 50 and abs(y_pos) <= ROBOT_HALF_WIDTH:
            front_clear_x = min(front_clear_x, x_pos)

        # [C] 측면 벽 (차체 후방 -10cm ~ 전방 40cm 범위)
        if -100 < x_pos < 400:
            if angle >= 30:
                left_wall_min  = min(left_wall_min, y_pos)
            elif angle <= -30:
                right_wall_min = min(right_wall_min, abs(y_pos))

    return front_emergency_dist, front_clear_x, left_wall_min, right_wall_min


# ============================================================
# 8. 메인 조향 함수 (VFH 통합)
# ============================================================

def calculate_steering(scan_data: list) -> tuple[int, int]:
    """
    스캔 데이터 → (속도, 조향 PWM) 반환.

    단계:
      1. 센서값 추출
      2. [긴급] 비상 정지 & 제자리 회전
      3. [VFH] 히스토그램 → 평활화 → Valley 탐색 → 최적 Valley 선택
      4. [보정] Valley 없음 처리 / 벽 반발 보정
      5. 속도 결정
    """
    global last_avoid_dir

    # ── 1. 센서값 추출 ──────────────────────────────────────
    front_emergency_dist, front_clear_x, left_wall_min, right_wall_min = \
        extract_sensor_values(scan_data)

    # ── 2. 비상 회피 (20cm 이내 코앞 장애물) ────────────────
    if front_emergency_dist < 200:
        if last_avoid_dir == -1:
            steer = 75          # 계속 좌회전
        elif last_avoid_dir == 1:
            steer = -75         # 계속 우회전
        else:
            # 측면 공간이 넓은 쪽으로 회전
            if left_wall_min > right_wall_min:
                steer          = 75
                last_avoid_dir = -1
            else:
                steer          = -75
                last_avoid_dir = 1
        return 0, steer

    # ── 3. VFH: 히스토그램 → 평활화 → Valley 탐색 ──────────
    hist     = build_polar_histogram(scan_data)
    smoothed = smooth_histogram(hist)
    valleys  = find_valleys(smoothed)
    best_v   = select_best_valley(valleys)

    # ── 4-A. Valley 없음 → 측면 공간 기반 탈출 회전 ─────────
    if best_v is None:
        # 전방이 완전히 막혔을 때 측면 여유 방향으로 회전
        if left_wall_min > right_wall_min:
            steer          = 60
            last_avoid_dir = -1
        else:
            steer          = -60
            last_avoid_dir = 1
        return ESCAPE_SPEED, steer

    # ── 4-B. 조향각 결정 ─────────────────────────────────────
    target_angle = valley_to_angle(best_v, target=0)

    # ── 4-C. 벽 반발 보정 (Wall Repulsion) ──────────────────
    # Valley 방향이 결정됐어도 한쪽 벽이 바짝 붙으면 반대 방향 가중
    wall_offset = 0
    if right_wall_min < 200 and left_wall_min > right_wall_min + 40:
        wall_offset = +5    # 우측 벽 → 좌측(양수)으로 5도 보정
    elif left_wall_min < 200 and right_wall_min > left_wall_min + 40:
        wall_offset = -5    # 좌측 벽 → 우측(음수)으로 5도 보정

    final_angle = max(-90, min(90, target_angle + wall_offset))

    # 상태 업데이트
    if final_angle < -15:
        last_avoid_dir = 1
    elif final_angle > 15:
        last_avoid_dir = -1
    else:
        last_avoid_dir = 0

    steer_pwm = int(final_angle * STEER_GAIN)

    # ── 5. 속도 결정 (front_clear_x 기반 3단계) ─────────────
    if front_clear_x <= 400:
        speed = ESCAPE_SPEED
    elif abs(final_angle) <= 15 and front_clear_x > SAFE_DISTANCE:
        speed = MAX_SPEED
    else:
        speed = AVOID_SPEED

    return speed, steer_pwm


# ============================================================
# 9. 메인 루프 (LiDAR 패킷 파싱 → Arduino 전송)
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
                    speed, steer = calculate_steering(scan_data)
                    arduino.write(f"{speed},{steer}\n".encode('utf-8'))
                else:
                    skipped_scans += 1
                    if skipped_scans % 10 == 0 and scan_history:
                        avg = sum(scan_history) / len(scan_history)
                        print(f"[WARN] 불완전 스캔 누적 {skipped_scans}회 "
                              f"(현재={scan_size}pts, 평균={avg:.0f}pts)")

                scan_history.append(scan_size)   # 유효·무효 무관 기록
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

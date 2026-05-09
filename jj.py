"""
RPLIDAR C1 장애물 회피 코드
- 라이다(USB0)로 정면 180도 스캔
- 장애물의 수평거리 계산 → 로봇 폭과 비교
- 회피 방향 + 각도를 아두이노(USB1)로 시리얼 전송

[좌표 기준]
  라이다 0° = 정면
  왼쪽 : 270~360° (또는 -90~0°)
  오른쪽: 0~90°
  정면 180도 범위: 270~360° + 0~90° → -90° ~ +90°로 통일해서 처리

[수평거리 계산]
  horizontal_dist = distance * cos(angle)   ← 로봇 폭 방향 성분
  forward_dist    = distance * sin(|angle|) ← 전진 방향 거리

[아두이노 전송 포맷]
  장애물 없을 때 : "F\n"          (직진)
  왼쪽 회피      : "L<각도>\n"    예) "L35\n"
  오른쪽 회피    : "R<각도>\n"    예) "R40\n"
"""

import serial
import time
import math

# ── 설정값 (필요에 맞게 수정) ──────────────────────────────────────────────
LIDAR_PORT   = "/dev/ttyUSB0"   # 라이다 시리얼 포트
ARDUINO_PORT = "/dev/ttyUSB1"   # 아두이노 시리얼 포트
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 9600

ROBOT_HALF_WIDTH = 110          # 로봇 폭의 절반 (mm) — 라이다 중심~좌우 끝 110mm
SAFETY_MARGIN    = 20           # 추가 안전 여유 (mm)
DETECTION_RANGE  = 1500         # 장애물 탐지 최대 거리 (mm) — 1.5m
SEND_INTERVAL    = 0.1          # 아두이노로 명령 전송 주기 (초)

# 정면 스캔 범위: -90° ~ +90° (라이다 각도 기준 270~360 + 0~90)
SCAN_HALF_ANGLE = 90            # 정면 기준 좌우 탐지 각도 범위 (도)

# 회피 각도 계산 시 탐색할 방향 해상도 (도 단위)
ANGLE_STEP = 5
# ────────────────────────────────────────────────────────────────────────────


def normalize_angle(angle):
    """라이다 각도(0~360)를 정면 기준 -180~180으로 변환"""
    if angle > 180:
        return angle - 360
    return angle


def is_in_front(angle_norm):
    """정면 ±SCAN_HALF_ANGLE 범위인지 확인"""
    return -SCAN_HALF_ANGLE <= angle_norm <= SCAN_HALF_ANGLE


def parse_packet(data):
    """
    5바이트 라이다 패킷 파싱
    반환: (angle_deg, distance_mm, quality) 또는 None (유효하지 않은 패킷)
    """
    if len(data) != 5:
        return None

    # S, S̄ 검증
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        return None

    # C 비트 검증
    check_bit = data[1] & 0x01
    if check_bit != 1:
        return None

    quality     = data[0] >> 2
    angle_q6    = ((data[1] >> 1) | (data[2] << 7))
    angle       = angle_q6 / 64.0
    distance_q2 = (data[3] | (data[4] << 8))
    distance    = distance_q2 / 4.0

    return angle, distance, quality


def compute_horizontal_dist(angle_norm_deg, distance_mm):
    """
    수평거리(로봇 폭 방향 성분) 계산
      horizontal = distance * sin(angle)  ← 좌우 방향 성분
      forward    = distance * cos(angle)  ← 전진 방향 성분
    """
    angle_rad = math.radians(angle_norm_deg)
    horizontal = abs(distance_mm * math.sin(angle_rad))
    forward    = distance_mm * math.cos(angle_rad)
    return horizontal, forward


def find_avoidance_angle(scan_points):
    """
    정면 180도 스캔 데이터에서 회피 방향과 각도를 결정

    [알고리즘]
    1. 위험거리 내 장애물 중 가장 가까운 포인트를 기준(reference_dist)으로 설정
    2. 그 reference_dist와 동일한 직선거리 기준으로 좌/우 빈 공간 각도 폭 계산
       → 각 각도 버킷의 측정 거리 >= reference_dist 이면 해당 방향은 비어있음
    3. 빈 공간이 더 넓은 쪽으로 회피 방향 결정
    4. 회피 각도 = asin(threshold / reference_dist) — 최소 필요 꺾임 각도

    scan_points: [(angle_norm, distance_mm), ...]  — 정면 범위 내 유효 포인트

    반환: (direction, angle_deg)
      direction : 'F'(직진), 'L'(왼쪽), 'R'(오른쪽)
      angle_deg : 회피 조향 각도 (0이면 직진)
    """
    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN  # 충돌 판단 수평거리 한계 (mm)

    # ── 1. 위험 포인트 수집 ──────────────────────────────────────────────────
    # 조건: 전진 방향 성분 > 0, 수평거리 < threshold, 탐지 범위 내
    danger_points = []
    for angle_norm, dist in scan_points:
        if dist <= 0 or dist > DETECTION_RANGE:
            continue
        horiz, fwd = compute_horizontal_dist(angle_norm, dist)
        if fwd > 0 and horiz < threshold:
            danger_points.append((angle_norm, dist, horiz, fwd))

    # 위험 없으면 직진
    if not danger_points:
        return 'F', 0

    # ── 2. 가장 가까운 장애물(기준 포인트) 선정 ─────────────────────────────
    nearest        = min(danger_points, key=lambda p: p[1])  # 직선거리 최소
    nearest_angle  = nearest[0]   # 기준 각도 (도)
    reference_dist = nearest[1]   # 기준 직선거리 (mm)

    print(f"  [기준 장애물] 각도: {nearest_angle:.1f}°  "
          f"거리: {reference_dist:.1f}mm  "
          f"수평: {nearest[2]:.1f}mm")

    # ── 3. 각도별 최소 거리 딕셔너리 구성 ──────────────────────────────────
    # 같은 각도 버킷 안에 여러 포인트가 있으면 가장 가까운 값(최소)만 사용
    scan_dict = {}
    for angle_norm, dist in scan_points:
        if dist <= 0:
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    # ── 4. reference_dist 기준으로 좌/우 빈 공간 각도 폭 계산 ───────────────
    # 판단 기준: 해당 각도에서 측정된 거리 >= reference_dist
    #           → 그 방향의 reference_dist 지점은 비어있음 (통과 가능)
    #           측정 거리 < reference_dist
    #           → 그 방향에 reference_dist보다 가까운 장애물이 있음 (막힘)
    left_clear  = 0   # 왼쪽 빈 공간 각도 폭 합계  (음수 각도: -SCAN_HALF_ANGLE ~ 0)
    right_clear = 0   # 오른쪽 빈 공간 각도 폭 합계 (양수 각도: 0 ~ +SCAN_HALF_ANGLE)

    for a in range(-SCAN_HALF_ANGLE, 0, ANGLE_STEP):
        d = scan_dict.get(a, DETECTION_RANGE + 1)  # 측정값 없으면 충분히 멀다고 가정
        if d >= reference_dist:
            left_clear += ANGLE_STEP

    for a in range(ANGLE_STEP, SCAN_HALF_ANGLE + 1, ANGLE_STEP):
        d = scan_dict.get(a, DETECTION_RANGE + 1)
        if d >= reference_dist:
            right_clear += ANGLE_STEP

    print(f"  [여유 공간] 왼쪽: {left_clear}°  오른쪽: {right_clear}°")

    # ── 5. 회피 방향 결정 (더 넓은 쪽) ─────────────────────────────────────
    direction = 'L' if left_clear >= right_clear else 'R'

    # ── 6. 회피 각도 계산 ────────────────────────────────────────────────────
    # 최소 꺾임 각도 = asin(threshold / reference_dist)
    # → 이 각도만큼 꺾으면 로봇 끝이 장애물과 딱 맞닿는 최소 각도
    # 안전 여유가 이미 threshold에 포함돼 있으므로 추가 여유 없이 그대로 사용
    ratio       = min(threshold / max(reference_dist, 1.0), 1.0)
    avoid_angle = math.degrees(math.asin(ratio))
    avoid_angle = min(int(math.ceil(avoid_angle)), SCAN_HALF_ANGLE)

    return direction, avoid_angle


def main():
    print("=== RPLIDAR 장애물 회피 시스템 시작 ===")
    print(f"  라이다 포트   : {LIDAR_PORT}")
    print(f"  아두이노 포트 : {ARDUINO_PORT}")
    print(f"  로봇 반폭     : {ROBOT_HALF_WIDTH} mm")
    print(f"  안전 여유     : {SAFETY_MARGIN} mm")
    print(f"  탐지 거리     : {DETECTION_RANGE} mm")
    print("=" * 40)

    # 시리얼 포트 열기
    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)  # 아두이노 리셋 대기

    # 라이다 RESET → SCAN 시작
    lidar.write(bytes([0xA5, 0x40]))  # RESET
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))  # START SCAN
    print("스캔 시작...")

    # 응답 디스크립터 7바이트 읽어서 버림 (0xA5 5A 05 00 00 40 81)
    lidar.read(7)

    scan_points  = []   # 현재 360도 스캔 누적 버퍼
    last_send    = time.time()
    last_cmd     = ""

    try:
        while True:
            raw = lidar.read(5)
            result = parse_packet(raw)
            if result is None:
                continue

            angle_raw, distance, quality = result

            # 새 스캔 시작(S=1) → 이전 스캔 데이터로 판단 후 초기화
            s_flag = raw[0] & 0x01
            if s_flag == 1 and scan_points:
                # 정면 180도 포인트만 필터링
                front_points = [
                    (a, d) for a, d in scan_points
                    if is_in_front(a) and d > 0
                ]

                now = time.time()
                if now - last_send >= SEND_INTERVAL:
                    direction, avoid_angle = find_avoidance_angle(front_points)

                    if direction == 'F':
                        cmd = "F"
                    else:
                        cmd = f"{direction}{avoid_angle}"

                    # 이전 명령과 다를 때만 전송 (불필요한 통신 최소화)
                    if cmd != last_cmd:
                        msg = cmd + "\n"
                        arduino.write(msg.encode())
                        print(f"[전송] {msg.strip()}  "
                              f"(정면 포인트 수: {len(front_points)})")
                        last_cmd = cmd

                    last_send = now

                scan_points = []  # 버퍼 초기화

            # 각도 정규화 후 버퍼에 추가
            angle_norm = normalize_angle(angle_raw)
            scan_points.append((angle_norm, distance))

    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        # 라이다 정지
        lidar.write(bytes([0xA5, 0x25]))  # STOP
        time.sleep(0.1)
        lidar.close()
        arduino.write(b"F\n")  # 아두이노에 직진(정지) 명령
        arduino.close()
        print("포트 닫힘. 종료 완료.")


if __name__ == "__main__":
    main()

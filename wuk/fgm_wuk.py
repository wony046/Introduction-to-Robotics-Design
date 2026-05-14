"""
RPLIDAR C1 장애물 회피 - Follow the Gap Method (FGM) PRO v2
* 경기장: 폭 1.1m x 길이 3.1m
* 적용: 240도 광각 시야, 동적 전방 감시, 3단계 탈출 머신, 넓은 길 우대 알고리즘
* v2 수정 사항:
    [Fix 1] raw_ranges ZeroDivisionError 방어 코드 추가
    [Fix 2] Deadlock Phase 3 무한 재진입 버그 수정 (쿨다운 변수 도입)
    [Fix 3] SAFE_DIST 140 → 250 (ROBOT_RADIUS + 120)으로 현실화
"""

import serial
import time
import math

# ── 포트 및 통신 ──────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

# ── FGM 알고리즘 파라미터 ──────────────────────────────────────────────────────
ROBOT_RADIUS     = 130   # mm: 물리적 로봇 반경 (충돌 한계선)
MAX_RANGE        = 1000  # mm: 이 거리보다 멀면 완전히 뚫린 길로 간주
# [Fix 3] SAFE_DIST: 140 → 250 (ROBOT_RADIUS + 120)
# 이유: 140은 Inflation 후 장애물 바로 옆 10mm만 남겨 1.1m 복도에서 갭이 거의 안 잡힘.
#       로봇 한쪽 여유 12cm(=ROBOT_RADIUS + 120)이 현실적인 안전 기준.
SAFE_DIST        = ROBOT_RADIUS + 120  # = 250mm
MIN_GAP_DEPTH    = 300   # mm: ㄱ자 함정 방지 (갭의 최대 깊이가 이보다 깊어야 함)
GAP_MARGIN_DEG   = 5     # deg: 장애물 가장자리에서 띄울 여유 각도
BRAKE_START_DIST = 500   # mm: 동적 감속이 시작되는 전방 거리

# ── 속도 및 제어 파라미터 ─────────────────────────────────────────────────────
FORWARD_SPEED    = 0.40  # m/s: 직진 최고 속도
MIN_SPEED        = 0.15  # m/s: 주행 시 유지할 최저 속도
Kp_W             = 0.015 # 조향 P제어 게인
MAX_W            = 2.0   # rad/s: 최대 각속도
SEND_INTERVAL    = 0.1   # 초: 명령 전송 주기

# ── Deadlock 탈출 타이밍 파라미터 ─────────────────────────────────────────────
DEADLOCK_ROTATE_DURATION  = 1.5  # 초: Phase 1 탐색 회전 지속 시간
DEADLOCK_REVERSE_DURATION = 1.0  # 초: Phase 2 후진 지속 시간 (0.5s → 1.0s로 여유 증가)
# [Fix 2] 쿨다운: Phase 3 리셋 직후 재진입 방지 대기 시간
DEADLOCK_COOLDOWN_SEC     = 1.0  # 초

# ── 전역 상태 관리 ─────────────────────────────────────────────────────────────
arduino_heading_deg   = 0.0
prev_w                = 0.0    # 조향 스무딩 용도

# Deadlock State Machine 변수
deadlock_start        = None   # Phase 시작 타임스탬프
deadlock_phase        = 0      # 0: 정상, 1: 탐색 회전, 2: 후진
# [Fix 2] 쿨다운 만료 시각 (이 시각 이전에는 Deadlock 재진입 차단)
deadlock_cooldown_until = 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티 & 패킷 파서
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return angle - 360 if angle > 180 else angle

def parse_packet(data):
    if len(data) != 5: return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None

    angle_q6   = (data[1] >> 1) | (data[2] << 7)
    angle      = angle_q6 / 64.0
    distance_q2 = data[3] | (data[4] << 8)
    distance   = distance_q2 / 4.0
    return angle, distance

def read_arduino(arduino):
    global arduino_heading_deg
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'):
                arduino_heading_deg = float(line[2:])
        except Exception:
            pass


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# FGM 핵심 알고리즘
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def apply_fgm(scan_dict, heading_deg):
    global prev_w, deadlock_start, deadlock_phase, deadlock_cooldown_until

    # ── 1. 1D 배열화 (-120° ~ +120°, 240° 광각 시야) ─────────────────────────
    angles = list(range(-120, 121))
    raw_ranges = []
    for a in angles:
        dist = scan_dict.get(a, MAX_RANGE)
        # [Fix 1] dist가 0이면 MAX_RANGE로 대체 → Inflation에서 ZeroDivisionError 방지
        if dist <= 0:
            dist = MAX_RANGE
        raw_ranges.append(dist)

    # ── 2. 장애물 팽창 (Inflation) ────────────────────────────────────────────
    inflated_ranges = list(raw_ranges)
    for i, dist in enumerate(raw_ranges):
        if dist < MAX_RANGE:
            if dist > ROBOT_RADIUS:
                spread_deg = math.degrees(math.asin(ROBOT_RADIUS / dist))
            else:
                spread_deg = 120  # 충돌 반경 내부 → 좌우 완전 차단

            spread_idx = math.ceil(spread_deg)  # 올림으로 보수적 팽창
            start_idx  = max(0, i - spread_idx)
            end_idx    = min(len(raw_ranges) - 1, i + spread_idx)

            for j in range(start_idx, end_idx + 1):
                inflated_ranges[j] = min(inflated_ranges[j], dist)

    # ── 3. Gap(빈 공간) 탐색 ─────────────────────────────────────────────────
    gaps = []
    current_gap = []
    for i, dist in enumerate(inflated_ranges):
        if dist > SAFE_DIST:
            current_gap.append(i)
        else:
            if current_gap:
                gaps.append(current_gap)
                current_gap = []
    if current_gap:
        gaps.append(current_gap)

    # ── 4. ㄱ자 함정 갭 필터링 (Depth 조건) ──────────────────────────────────
    valid_gaps = []
    for gap in gaps:
        max_depth = max(raw_ranges[idx] for idx in gap)
        if max_depth >= MIN_GAP_DEPTH:
            valid_gaps.append(gap)

    # ── Deadlock 3단계 탈출 정책 ──────────────────────────────────────────────
    if not valid_gaps:
        now = time.time()

        # [Fix 2] 쿨다운 중이면 회전만 유지하고 상태 재진입 차단
        if now < deadlock_cooldown_until:
            print(f"  [Deadlock] 쿨다운 중... ({deadlock_cooldown_until - now:.1f}s 남음)")
            return 0.0, MAX_W

        if deadlock_start is None:
            deadlock_start = now
            deadlock_phase = 1

        elapsed = now - deadlock_start
        print(f"  [경고] Deadlock! Phase {deadlock_phase} ({elapsed:.1f}s)")

        if elapsed < DEADLOCK_ROTATE_DURATION:
            # Phase 1: 더 트인 쪽으로 탐색 회전
            left_max  = max([raw_ranges[i] for i, a in enumerate(angles) if  10 <= a <= 120], default=0)
            right_max = max([raw_ranges[i] for i, a in enumerate(angles) if -120 <= a <= -10], default=0)
            w = MAX_W if left_max > right_max else -MAX_W
            prev_w = w
            return 0.0, w

        elif elapsed < DEADLOCK_ROTATE_DURATION + DEADLOCK_REVERSE_DURATION:
            # Phase 2: 후진으로 공간 확보
            deadlock_phase = 2
            prev_w = 0.0
            return -0.20, 0.0

        else:
            # Phase 3: 상태 리셋 + 쿨다운 설정 → 즉시 재진입 방지
            # [Fix 2] cooldown_until을 설정해야 다음 프레임에 Phase 1로 재진입하지 않음
            deadlock_start          = None
            deadlock_phase          = 0
            deadlock_cooldown_until = time.time() + DEADLOCK_COOLDOWN_SEC
            prev_w                  = MAX_W
            print("  [Deadlock] Phase 3: 리셋 완료, 쿨다운 시작")
            return 0.0, MAX_W

    else:
        # 정상 주행 가능 → Deadlock 상태 전체 초기화
        deadlock_start          = None
        deadlock_phase          = 0
        deadlock_cooldown_until = 0.0

    # ── 5. 최적 조향각 계산 (클램프 기반 너비 보너스) ──────────────────────────
    target_angle_local = -heading_deg
    best_target_angle  = 0
    min_cost           = float('inf')

    for gap in valid_gaps:
        gap_start_angle = angles[gap[0]]
        gap_end_angle   = angles[gap[-1]]

        # 목표 방향이 갭 내부에 있으면 즉시 채택
        if gap_start_angle <= target_angle_local <= gap_end_angle:
            best_target_angle = target_angle_local
            min_cost = 0
            break

        dist_to_start = abs(target_angle_local - gap_start_angle)
        dist_to_end   = abs(target_angle_local - gap_end_angle)

        candidate_angle = (gap_start_angle + GAP_MARGIN_DEG
                           if dist_to_start < dist_to_end
                           else gap_end_angle - GAP_MARGIN_DEG)

        gap_width      = gap_end_angle - gap_start_angle
        raw_cost       = abs(target_angle_local - candidate_angle)
        # 너비 보너스: 오차의 90% 이상은 깎지 못하도록 상한 적용 → cost 음수 역전 방지
        width_discount = min(raw_cost * 0.9, gap_width * 0.5)
        cost           = raw_cost - width_discount

        if cost < min_cost:
            min_cost          = cost
            best_target_angle = candidate_angle

    # ── 6. v, w 제어량 계산 ───────────────────────────────────────────────────

    # 조향 제어 (P 게인 + 로우패스 필터)
    w_target = best_target_angle * Kp_W
    w_target = max(min(w_target, MAX_W), -MAX_W)
    w        = 0.6 * w_target + 0.4 * prev_w
    prev_w   = w

    # 동적 전방 감시 (로봇 폭 기반 삼각함수 판별)
    min_front_dist = MAX_RANGE
    for a, d in scan_dict.items():
        if d <= 0: continue
        lateral_dist = d * abs(math.sin(math.radians(a)))
        if lateral_dist <= ROBOT_RADIUS:
            frontal_dist = d * math.cos(math.radians(a))
            if frontal_dist > 0:
                min_front_dist = min(min_front_dist, frontal_dist)

    # 선속도 제어 (파라미터 연동)
    angle_speed_factor = max(0.0, 1.0 - abs(best_target_angle) / 45.0)
    dist_speed_factor  = max(0.0, min(1.0,
        (min_front_dist - ROBOT_RADIUS) / (BRAKE_START_DIST - ROBOT_RADIUS)
    ))
    final_speed_factor = min(angle_speed_factor, dist_speed_factor)
    v = MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * final_speed_factor

    # 최후의 물리적 충돌 방지 브레이크
    if min_front_dist <= ROBOT_RADIUS:
        v = 0.0

    print(f"  [FGM] 조향: {best_target_angle:5.1f}° | 전방거리: {min_front_dist:4.0f}mm | v: {v:.2f}, w: {w:.2f}")
    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    print("=== RPi FGM PRO v2 Navigation Started ===")

    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)

    lidar.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))
    lidar.read(7)

    scan_dict = {}
    last_send = time.time()

    try:
        while True:
            read_arduino(arduino)

            raw    = lidar.read(5)
            result = parse_packet(raw)
            if result is None: continue

            angle_raw, distance = result
            s_flag = raw[0] & 0x01

            norm_angle = normalize_angle(angle_raw)

            # 240° 범위 내, 유효한 값만 최솟값으로 저장
            if -120 <= norm_angle <= 120 and distance > 0:
                angle_key = int(round(norm_angle))
                if angle_key not in scan_dict or distance < scan_dict[angle_key]:
                    scan_dict[angle_key] = distance

            if s_flag == 1:
                now = time.time()
                if now - last_send >= SEND_INTERVAL:
                    if scan_dict:
                        v, w = apply_fgm(scan_dict, arduino_heading_deg)
                        cmd  = f"{v:.2f} {w:.2f}\n"
                        arduino.write(cmd.encode())
                        last_send = now
                    scan_dict.clear()

    except KeyboardInterrupt:
        print("\n종료 중...")
    finally:
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        lidar.close()
        arduino.write(b"0.00 0.00\n")
        arduino.close()
        print("종료 완료.")

if __name__ == "__main__":
    main()

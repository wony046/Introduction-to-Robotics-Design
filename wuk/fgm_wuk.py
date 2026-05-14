"""
RPLIDAR C1 장애물 회피 - Follow the Gap Method (FGM)
* 경기장: 폭 1.1m x 길이 3.1m
* 특징: Inflation 기반, 헤딩 오차 보정, 부드러운 연속 주행(Smooth Driving)
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
ROBOT_RADIUS     = 130   # mm: 로봇 절반 너비(110) + 안전 마진(20)
MAX_RANGE        = 1000  # mm: 이 거리보다 멀면 빈 공간(Gap)으로 취급
SAFE_DIST        = 250   # mm: 최소 통과 보장 거리 임계값
MIN_GAP_DEPTH    = 300   # mm: 갇힘(ㄱ자) 방지. Gap 내부 최대 거리가 이보다 길어야 함
GAP_MARGIN_DEG   = 5     # deg: 갭의 가장자리를 탈 때 추가로 띄우는 안전 각도

# ── 속도 및 제어 파라미터 ─────────────────────────────────────────────────────
FORWARD_SPEED    = 0.40  # m/s: 기본 직진 속도
MIN_SPEED        = 0.15  # m/s: 회전 시 최소 속도 (코너링을 위해 조금 낮춤)
Kp_W             = 0.015 # 조향각(deg)을 각속도(rad/s)로 변환하는 P제어 게인
MAX_W            = 2.0   # rad/s: 최대 각속도
SEND_INTERVAL    = 0.1   # 초: 아두이노 명령 전송 주기

# ── 전역 상태 ─────────────────────────────────────────────────────────────────
arduino_heading_deg = 0.0
prev_w = 0.0  # 부드러운 조향(도리도리 방지)을 위한 이전 각속도 저장 변수


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티 & 파서
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return angle - 360 if angle > 180 else angle

def parse_packet(data):
    if len(data) != 5: return None
    s_flag = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None
    
    angle_q6 = (data[1] >> 1) | (data[2] << 7)
    angle = angle_q6 / 64.0
    distance_q2 = data[3] | (data[4] << 8)
    distance = distance_q2 / 4.0
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
    global prev_w

    # 1. 1D 배열화 (-90도 ~ +90도, 1도 간격)
    angles = list(range(-90, 91))
    raw_ranges = []
    for a in angles:
        dist = scan_dict.get(a, MAX_RANGE)
        if dist <= 0: dist = MAX_RANGE
        raw_ranges.append(dist)

    # 2. 장애물 팽창 (Inflation)
    inflated_ranges = list(raw_ranges)
    for i, dist in enumerate(raw_ranges):
        if dist < MAX_RANGE:
            if dist > ROBOT_RADIUS:
                spread_deg = math.degrees(math.asin(ROBOT_RADIUS / dist))
            else:
                spread_deg = 90
            
            spread_idx = int(spread_deg)
            start_idx = max(0, i - spread_idx)
            end_idx = min(len(raw_ranges) - 1, i + spread_idx)
            
            for j in range(start_idx, end_idx + 1):
                inflated_ranges[j] = min(inflated_ranges[j], dist)

    # 3. Gap(빈 공간) 찾기
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

    # 4. 막힌 길 필터링
    valid_gaps = []
    for gap in gaps:
        max_depth = max(raw_ranges[idx] for idx in gap)
        if max_depth >= MIN_GAP_DEPTH:
            valid_gaps.append(gap)

    # 길이 모두 막혔을 경우 (안전 최우선 제자리 회전)
    if not valid_gaps:
        print("  [경고] 진행 가능한 Gap이 없습니다! 탐색 회전.")
        v = 0.0
        w = MAX_W if heading_deg < 0 else -MAX_W
        prev_w = w
        return v, w

    target_angle_local = -heading_deg
    
    # 5. 최적의 조향각 계산
    best_target_angle = 0
    min_cost = float('inf')

    for gap in valid_gaps:
        gap_start_angle = angles[gap[0]]
        gap_end_angle = angles[gap[-1]]
        
        if gap_start_angle <= target_angle_local <= gap_end_angle:
            best_target_angle = target_angle_local
            min_cost = 0 
            break

        dist_to_start = abs(target_angle_local - gap_start_angle)
        dist_to_end = abs(target_angle_local - gap_end_angle)
        
        if dist_to_start < dist_to_end:
            candidate_angle = gap_start_angle + GAP_MARGIN_DEG
        else:
            candidate_angle = gap_end_angle - GAP_MARGIN_DEG
            
        cost = abs(target_angle_local - candidate_angle)
        if cost < min_cost:
            min_cost = cost
            best_target_angle = candidate_angle

    # ─────────────────────────────────────────────────────────────────
    # 6. 부드러운 주행을 위한 v, w 제어량 계산 (Smooth Control)
    # ─────────────────────────────────────────────────────────────────
    
    # [조향 제어] P제어 및 로우패스 필터 (진동 방지)
    heading_error = best_target_angle
    w_target = heading_error * Kp_W
    w_target = max(min(w_target, MAX_W), -MAX_W)
    
    w = 0.6 * w_target + 0.4 * prev_w
    prev_w = w

    # [전방 거리 확인] 로봇 정면(-15도 ~ +15도)의 최단 거리 측정
    front_dists = [scan_dict.get(a, MAX_RANGE) for a in range(-15, 16) if a in scan_dict]
    min_front_dist = min(front_dists) if front_dists else MAX_RANGE

    # [선속도 제어]
    # 1. 조향각에 의한 감속 비율 (크게 꺾어야 하면 감속)
    angle_speed_factor = max(0.0, 1.0 - abs(heading_error) / 45.0)

    # 2. 전방 거리에 의한 감속 비율 
    # 500mm부터 부드럽게 감속을 시작하되, 130mm에 도달할 때까지 속도가 서서히 0에 수렴하도록 변경
    dist_speed_factor = max(0.0, min(1.0, (min_front_dist - 130) / 370.0))

    # 두 가지 감속 요인 중 더 강한(값이 작은) 요인을 채택
    final_speed_factor = min(angle_speed_factor, dist_speed_factor)
    
    # 최종 속도 계산
    v = MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * final_speed_factor

    # [최후의 보루] 로봇의 최소 반경인 130mm 이내로 들어오면 물리적 충돌 직전이므로 즉시 정지
    if min_front_dist <= 130:
        v = 0.0

    print(f"  [FGM] 조향결정: {best_target_angle:5.1f}° | 전방거리: {min_front_dist:4.0f}mm | v: {v:.2f}, w: {w:.2f}")
    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    print("=== RPi FGM Navigation Started ===")
    
    lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
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

            raw = lidar.read(5)
            result = parse_packet(raw)
            if result is None: continue

            angle_raw, distance = result
            s_flag = raw[0] & 0x01
            
            norm_angle = normalize_angle(angle_raw)
            
            if -90 <= norm_angle <= 90:
                scan_dict[int(round(norm_angle))] = distance

            if s_flag == 1:
                now = time.time()
                if now - last_send >= SEND_INTERVAL:
                    if scan_dict:
                        v, w = apply_fgm(scan_dict, arduino_heading_deg)
                        cmd = f"{v:.2f} {w:.2f}\n"
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

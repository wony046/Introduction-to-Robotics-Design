"""
RPLIDAR C1 장애물 회피 - Follow the Gap Method (FGM)
* 경기장: 폭 1.1m x 길이 3.1m
* 특징: Inflation 기반, 헤딩 오차 보정, 막힌 길(Trap) 필터링
"""

import serial
import time
import math

# ── 포트 및 통신 ──────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

# ── FGM 알고리즘 파라미터 (★환경에 맞게 튜닝 필수★) ──────────────────────────
ROBOT_RADIUS     = 130   # mm: 로봇 절반 너비(110) + 안전 마진(20)
MAX_RANGE        = 1000  # mm: 이 거리보다 멀면 빈 공간(Gap)으로 취급
SAFE_DIST        = 250   # mm: 최소 통과 보장 거리 임계값
MIN_GAP_DEPTH    = 300  # mm: 갇힘(ㄱ자) 방지. Gap 내부 최대 거리가 이보다 길어야 함
GAP_MARGIN_DEG   = 5    # deg: 갭의 가장자리를 탈 때 추가로 띄우는 안전 각도

# ── 속도 및 제어 파라미터 ─────────────────────────────────────────────────────
FORWARD_SPEED    = 0.40  # m/s: 기본 직진 속도
MIN_SPEED        = 0.20  # m/s: 회전 시 최소 속도
Kp_W             = 0.015 # 조향각(deg)을 각속도(rad/s)로 변환하는 P제어 게인
MAX_W            = 2   # rad/s: 최대 각속도
SEND_INTERVAL    = 0.1   # 초: 아두이노 명령 전송 주기

# ── 전역 상태 ─────────────────────────────────────────────────────────────────
arduino_heading_deg = 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티 & 파서 (기존 로직 유지)
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
    """
    1. 배열화 -> 2. Inflation -> 3. Gap 추출 -> 4. 타겟 각도 선정
    """
    # 1. 1D 배열화 (-90도 ~ +90도, 1도 간격)
    angles = list(range(-90, 91))
    raw_ranges = []
    for a in angles:
        # 스캔 데이터가 없으면 안전한 빈 공간(MAX_RANGE)으로 간주
        dist = scan_dict.get(a, MAX_RANGE)
        # 센서 노이즈(0) 처리
        if dist <= 0: dist = MAX_RANGE
        raw_ranges.append(dist)

    # 2. 장애물 팽창 (Inflation)
    inflated_ranges = list(raw_ranges)
    for i, dist in enumerate(raw_ranges):
        if dist < MAX_RANGE:
            # 해당 거리에서 로봇 반경이 차지하는 각도 계산 (가까울수록 크게 부풀어오름)
            if dist > ROBOT_RADIUS:
                spread_deg = math.degrees(math.asin(ROBOT_RADIUS / dist))
            else:
                spread_deg = 90 # 이미 충돌 반경 안이면 최대치 팽창
            
            spread_idx = int(spread_deg)
            start_idx = max(0, i - spread_idx)
            end_idx = min(len(raw_ranges) - 1, i + spread_idx)
            
            # 팽창 범위 내의 값들을 최솟값으로 덮어씌움
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

    # 4. 막힌 길 필터링 (Depth Check) 및 가장 넓은 길 찾기
    valid_gaps = []
    for gap in gaps:
        # 이 gap 범위 내에서 실제 도달한 최대 거리 확인
        max_depth = max(raw_ranges[idx] for idx in gap)
        if max_depth >= MIN_GAP_DEPTH:
            valid_gaps.append(gap)

    # 길이 모두 막혔을 경우 제자리 회전 또는 정지
    if not valid_gaps:
        print("  [경고] 진행 가능한 Gap이 없습니다! 제자리 대기.")
        return 0.0, MAX_W if heading_deg < 0 else -MAX_W

    # 글로벌 목표는 0도 (직진). 현재 로봇 기준 목적지 방향(로컬)
    target_angle_local = -heading_deg
    
    # 5. 최적의 조향각(Target Angle) 계산
    best_target_angle = 0
    min_cost = float('inf')

    for gap in valid_gaps:
        # 배열 인덱스를 실제 각도로 변환
        gap_start_angle = angles[gap[0]]
        gap_end_angle = angles[gap[-1]]
        
        # 기하학적 투영: 목적지 방향이 Gap '안'에 있다면 그대로 직진!
        if gap_start_angle <= target_angle_local <= gap_end_angle:
            best_target_angle = target_angle_local
            min_cost = 0 
            break # 완벽한 길이므로 더 이상 찾을 필요 없음

        # 목적지가 Gap 밖에 있다면, Gap의 양 끝단 중 목적지와 가까운 곳 선택
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

    # 6. v, w 제어량 계산
    # 목표 방향과 현재 방향의 차이가 크면 감속, 직선에 가까우면 가속
    heading_error = best_target_angle
    w = heading_error * Kp_W
    w = max(min(w, MAX_W), -MAX_W) # 제한
    
    speed_factor = max(0.0, 1.0 - abs(heading_error) / 45.0) # 45도 이상 꺾이면 최저속도
    v = MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * speed_factor

    print(f"  [FGM] 목적지방향: {target_angle_local:.1f}° | 조향결정: {best_target_angle:.1f}° | v: {v:.2f}, w: {w:.2f}")
    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    print("=== RPi FGM Navigation Started ===")
    
    lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)

    # 라이다 스캔 시작
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
            
            # 각도를 -180 ~ +180으로 정규화
            norm_angle = normalize_angle(angle_raw)
            
            # 전방 180도 데이터만 수집 (반올림하여 딕셔너리에 저장)
            if -90 <= norm_angle <= 90:
                scan_dict[int(round(norm_angle))] = distance

            # 새로운 스캔 사이클이 시작될 때마다 연산 및 전송
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

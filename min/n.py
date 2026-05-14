"""
DWA Pro v4 - 충돌 체크 + 헤딩 방향 버그 수정
─────────────────────────────────────────────────────────────
수정 사항:
1) 충돌 체크 버그: 궤적의 모든 점에 대해 충돌 검사 (이전엔 끝점만 검사)
2) 헤딩 점수: 단순 회전량이 아닌, "현재 헤딩 + 예측 회전 → 0에 가까울수록 좋음"
3) 후진 방지: v < 0 후보는 DRIVE 모드에서 제외 (RECOVERY 전용)
4) 큰 회전 페널티: |w| 클수록 점수 감점 → 180° 턴 방지
"""

import time
import math
import serial

# ── 1. 하드웨어 ─────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

# ── 2. 안전 ────────────────────────────────────────────────────────────────
SAFETY_DIST_MM         = 130.0
ROBOT_RADIUS_MM        = 150.0   # ★ 130 → 150 (안전 마진 추가)
EMERGENCY_THRESHOLD_MM = 130.0
RECOVERY_CLEAR_MM      = 250.0
LIDAR_NOISE_MM         = 80.0

# ── 3. DWA ─────────────────────────────────────────────────────────────────
MAX_V         = 0.30     # ★ 0.45 → 0.30 (안전 위해 낮춤)
MAX_V_NARROW  = 0.15
MAX_W         = 1.50
DT            = 0.10
PREDICT_TIME  = 1.20     # ★ 1.0 → 1.2 (멀리 보기)
SEND_INTERVAL = 0.10

W_HEADING     = 1.2      # ★ 0.8 → 1.2
W_CLEARANCE   = 3.0      # ★ 2.5 → 3.0
W_VELOCITY    = 0.6      # ★ 1.0 → 0.6 (속도 욕심 줄임)
W_SMOOTHNESS  = 2.5
W_TURN_PENALTY = 1.5     # ★ 신규: 큰 회전 페널티

W_DEADZONE = 0.10

# ── 4. RECOVERY ─────────────────────────────────────────────────────────────
REC_BACK_DUR    = 0.8
REC_TURN_DUR    = 1.4
REC_SPIN_DUR    = 0.8
REC_CYCLE       = REC_BACK_DUR + REC_TURN_DUR + REC_SPIN_DUR
REC_MAX_ATTEMPT = 5

KEEPALIVE_INTERVAL = 0.30


# ════════════════════════════════════════════════════════════════════════════
# 상태
# ════════════════════════════════════════════════════════════════════════════
class RobotState:
    DRIVE    = 1
    RECOVERY = 2

current_state          = RobotState.DRIVE
arduino_heading_deg    = 0.0
prev_v_cmd = 0.0
prev_w_cmd = 0.0
rec_start_time    = 0.0
rec_attempt       = 0
rec_initial_sign  = 1


# ════════════════════════════════════════════════════════════════════════════
# 유틸
# ════════════════════════════════════════════════════════════════════════════
def normalize_angle_deg(angle):
    while angle > 180:   angle -= 360
    while angle <= -180: angle += 360
    return angle

def normalize_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ════════════════════════════════════════════════════════════════════════════
# 라이다 (검증된 동기화)
# ════════════════════════════════════════════════════════════════════════════
def is_valid_first_byte(b):
    return (b & 0x01) != ((b >> 1) & 0x01)

def is_valid_second_byte(b):
    return (b & 0x01) == 1


def parse_packet(raw):
    if len(raw) < 5: return None
    if not is_valid_first_byte(raw[0]): return None
    if not is_valid_second_byte(raw[1]): return None
    
    s_flag      = raw[0] & 0x01
    angle_q6    = ((raw[2] << 7) | (raw[1] >> 1)) & 0x7FFF
    angle_deg   = angle_q6 / 64.0
    distance_q2 = (raw[4] << 8) | raw[3]
    distance_mm = distance_q2 / 4.0
    return angle_deg, distance_mm, s_flag


def find_lidar_sync(lidar, verbose=True):
    if verbose: print("[라이다] 동기화 중...", flush=True)
    deadline = time.time() + 3.0
    bytes_read = 0
    
    while time.time() < deadline:
        b = lidar.read(1)
        if len(b) == 0: continue
        bytes_read += 1
        
        if not is_valid_first_byte(b[0]): continue
        b2 = lidar.read(1)
        if len(b2) == 0: continue
        if not is_valid_second_byte(b2[0]): continue
        
        rest = lidar.read(3)
        if len(rest) != 3: continue
        
        result = parse_packet(b + b2 + rest)
        if result is None: continue
        
        angle, distance, s_flag = result
        if 0 <= angle <= 360 and 0 <= distance <= 10000:
            if verbose:
                print(f"[라이다] OK (각도={angle:.1f}°, 거리={distance:.0f}mm)",
                      flush=True)
            return True
    
    if verbose:
        print(f"[라이다] ✗ 실패 (읽음={bytes_read}B)", flush=True)
    return False


def start_lidar(lidar):
    print("[라이다] 시작...", flush=True)
    lidar.write(bytes([0xA5, 0x25]))
    time.sleep(0.1)
    try: lidar.dtr = False
    except: pass
    time.sleep(0.5)
    lidar.reset_input_buffer()
    lidar.write(bytes([0xA5, 0x20]))
    time.sleep(0.5)
    descriptor = lidar.read(7)
    print(f"[라이다] descriptor: {descriptor.hex()}", flush=True)
    return True


# ════════════════════════════════════════════════════════════════════════════
# 아두이노
# ════════════════════════════════════════════════════════════════════════════
def read_arduino(arduino):
    global arduino_heading_deg
    if arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("H:"):
                arduino_heading_deg = float(line.split(":")[1])
        except Exception:
            pass


# ════════════════════════════════════════════════════════════════════════════
# 스캔 분석
# ════════════════════════════════════════════════════════════════════════════
def analyze_proximity(scan_points):
    p = {'front': 99999, 'front_left': 99999, 'front_right': 99999,
         'left':  99999, 'right':       99999}
    for a, d in scan_points:
        na = normalize_angle_deg(a)
        if -25 <= na <= 25:
            p['front'] = min(p['front'], d)
        if  10 <= na <= 70:
            p['front_left'] = min(p['front_left'], d)
        if -70 <= na <= -10:
            p['front_right'] = min(p['front_right'], d)
        if  70 <  na <= 130:
            p['left'] = min(p['left'], d)
        if -130 <= na < -70:
            p['right'] = min(p['right'], d)
    return p


# ════════════════════════════════════════════════════════════════════════════
# ★ 충돌 체크 (수정됨)
# ════════════════════════════════════════════════════════════════════════════
def calculate_clearance(v, w, scan_points):
    """
    ★ 수정: 궤적 위 모든 점에서 충돌 검사
    + 정지 궤적: 가장 가까운 장애물만 확인 (그대로)
    + 이동 궤적: 매 step마다 모든 장애물과의 거리 검사
                  + 시작 위치(0,0) 근처 장애물도 검사
    """
    min_dist = float('inf')
    x = y = theta = 0.0
    steps = int(PREDICT_TIME / DT)

    # 정지 궤적: 현재 위치 기준 가장 가까운 장애물
    if abs(v) < 1e-3 and abs(w) < 1e-3:
        nearest = min((d for _, d in scan_points), default=99999)
        return nearest if nearest >= ROBOT_RADIUS_MM else -1.0

    # ★ 핵심 수정: t=0 시점부터 검사 (시작 위치에 이미 장애물 있으면 즉시 충돌)
    for a_deg, d in scan_points:
        if d < ROBOT_RADIUS_MM:
            return -1.0   # 이미 장애물 안에 있음

    # 시간 진행하며 매 step마다 충돌 검사
    for step in range(steps):
        x     += v * math.cos(theta) * DT * 1000.0
        y     += v * math.sin(theta) * DT * 1000.0
        theta += w * DT
        
        for a_deg, d in scan_points:
            ar = math.radians(a_deg)
            ox = d * math.cos(ar)
            oy = d * math.sin(ar)
            dist = math.hypot(ox - x, oy - y)
            if dist < ROBOT_RADIUS_MM:
                return -1.0
            if dist < min_dist:
                min_dist = dist
    
    return min_dist


# ════════════════════════════════════════════════════════════════════════════
# ★ DWA 점수 (수정됨)
# ════════════════════════════════════════════════════════════════════════════
def run_dwa(scan_points, prev_v, prev_w, narrow_mode, curr_heading_deg):
    """
    ★ 헤딩 점수 수정:
    - 이전: |회전량|이 작을수록 좋음 → 180° 턴이 0° 턴과 같은 절댓값 점수
    - 수정: 현재 헤딩 + 예측 회전 → 0에 가까울수록 좋음
    
    ★ 후진 방지:
    - DRIVE 모드에서는 v < 0 후보 제외 (RECOVERY 전용)
    
    ★ 큰 회전 페널티:
    - |w|가 클수록 페널티 → 가능한 한 작은 회전 선호
    """
    v_max = MAX_V_NARROW if narrow_mode else MAX_V

    # ★ 양수 v만 (후진 금지)
    v_candidates = [0.0, v_max * 0.33, v_max * 0.66, v_max]
    w_candidates = [-1.2, -0.8, -0.5, -0.25, -0.1, 0.0,
                     0.1, 0.25, 0.5, 0.8, 1.2]

    best_v, best_w = 0.0, 0.0
    best_score = -float('inf')
    safe_count = 0

    for v in v_candidates:
        for w in w_candidates:
            if abs(w) > MAX_W: continue

            clearance = calculate_clearance(v, w, scan_points)
            if clearance < 0: continue
            safe_count += 1

            # ──────────────────────────────────────────────────────────
            # 1) ★ 헤딩 점수: 미래 헤딩이 0(원래 진행 방향)에 가까울수록 좋음
            #    아두이노 heading +가 좌회전(반시계), 라이다도 동일 가정
            pred_turn_deg = math.degrees(w * PREDICT_TIME)
            future_heading_deg = normalize_angle_deg(curr_heading_deg + pred_turn_deg)
            # 0°에서 멀수록 감점 (180°가 가장 나쁨)
            heading_score = 1.0 - abs(future_heading_deg) / 180.0
            
            # 2) 안전거리 점수
            clearance_score = min(clearance / 1000.0, 1.0)
            
            # 3) 속도 점수 (양수만)
            velocity_score = v / max(v_max, 1e-3)
            
            # 4) 부드러움 (이전 명령과의 차이 페널티)
            smoothness_score = -abs(w - prev_w) / (2.0 * MAX_W)
            
            # 5) ★ 큰 회전 페널티: |w|가 클수록 감점
            turn_penalty = -abs(w) / MAX_W
            
            # 6) 정지 페널티
            stop_penalty = -0.5 if v < 0.01 else 0.0

            score = (W_HEADING      * heading_score
                   + W_CLEARANCE    * clearance_score
                   + W_VELOCITY     * velocity_score
                   + W_SMOOTHNESS   * smoothness_score
                   + W_TURN_PENALTY * turn_penalty
                   + stop_penalty)

            if score > best_score:
                best_score = score
                best_v, best_w = v, w

    if abs(best_w) < W_DEADZONE:
        best_w = 0.0

    return best_v, best_w, safe_count


# ════════════════════════════════════════════════════════════════════════════
# RECOVERY
# ════════════════════════════════════════════════════════════════════════════
def pick_recovery_direction(prox):
    left_room  = prox['front_left']  + prox['left']
    right_room = prox['front_right'] + prox['right']
    return 1 if left_room >= right_room else -1


def recovery_step(elapsed, attempt, initial_sign, prox):
    sign = initial_sign if (attempt % 2 == 0) else -initial_sign
    if elapsed < REC_BACK_DUR:
        return -0.15, 0.0
    elif elapsed < REC_BACK_DUR + REC_TURN_DUR:
        return -0.08, MAX_W * sign
    else:
        return 0.0, MAX_W * sign


# ════════════════════════════════════════════════════════════════════════════
# 메인
# ════════════════════════════════════════════════════════════════════════════
def main():
    global current_state, arduino_heading_deg
    global prev_v_cmd, prev_w_cmd
    global rec_start_time, rec_attempt, rec_initial_sign

    print("="*60, flush=True)
    print("DWA Pro v4 (충돌체크 + 헤딩 버그 수정)", flush=True)
    print("="*60, flush=True)

    try:
        lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
        time.sleep(0.5)
        print(f"✓ 라이다 연결", flush=True)
    except Exception as e:
        print(f"✗ 라이다: {e}", flush=True); return

    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=0.1)
        time.sleep(0.5)
        print(f"✓ 아두이노 연결", flush=True)
    except Exception as e:
        print(f"✗ 아두이노: {e}", flush=True); lidar.close(); return

    arduino.write(b"0.00 0.00\n")
    time.sleep(0.3)
    start_lidar(lidar)

    if not find_lidar_sync(lidar):
        arduino.write(b"0.00 0.00\n")
        lidar.close(); arduino.close(); return

    print("\n" + "="*60, flush=True)
    print("주행 시작!", flush=True)
    print("="*60, flush=True)

    scan_points    = []
    last_send_time = time.time()
    last_cmd_time  = time.time()
    dwa_count      = 0
    invalid_count  = 0

    try:
        while True:
            read_arduino(arduino)

            raw = lidar.read(5)
            if len(raw) == 5:
                pkt = parse_packet(raw)
                if pkt is None:
                    invalid_count += 1
                    if invalid_count > 100:
                        lidar.reset_input_buffer()
                        find_lidar_sync(lidar, verbose=False)
                        invalid_count = 0
                else:
                    angle_raw, distance, s_flag = pkt
                    now = time.time()

                    if distance > LIDAR_NOISE_MM:
                        scan_points.append((angle_raw, distance))

                    if s_flag == 1 and len(scan_points) > 30 \
                       and (now - last_send_time >= SEND_INTERVAL):

                        dwa_count += 1
                        prox = analyze_proximity(scan_points)
                        front_min = min(prox['front'],
                                        prox['front_left'],
                                        prox['front_right'])

                        if current_state == RobotState.DRIVE:
                            narrow = front_min < 400.0
                            v, w, safe_count = run_dwa(
                                scan_points, prev_v_cmd, prev_w_cmd,
                                narrow, arduino_heading_deg
                            )

                            if front_min < EMERGENCY_THRESHOLD_MM or safe_count == 0:
                                print(f"  [RECOVERY] front={front_min:.0f} safe={safe_count}",
                                      flush=True)
                                current_state    = RobotState.RECOVERY
                                rec_start_time   = now
                                rec_attempt      = 0
                                rec_initial_sign = pick_recovery_direction(prox)
                                v, w = -0.15, 0.0
                                safe_count = -1

                        else:
                            elapsed = now - rec_start_time
                            v, w = recovery_step(elapsed, rec_attempt,
                                                  rec_initial_sign, prox)
                            safe_count = -1

                            if elapsed >= REC_CYCLE:
                                if front_min > RECOVERY_CLEAR_MM:
                                    print(f"  [탈출] front={front_min:.0f}", flush=True)
                                    current_state = RobotState.DRIVE
                                    prev_v_cmd = 0.0
                                    prev_w_cmd = 0.0
                                else:
                                    rec_attempt += 1
                                    rec_start_time = now
                                    if rec_attempt >= REC_MAX_ATTEMPT:
                                        rec_attempt = 0
                                        rec_initial_sign = pick_recovery_direction(prox)

                        cmd = f"{v:.2f} {w:.2f}\n"
                        arduino.write(cmd.encode('utf-8'))
                        last_cmd_time = now

                        prev_v_cmd = v
                        prev_w_cmd = w

                        st_tag = "D" if current_state == RobotState.DRIVE else "R"
                        print(f"[{dwa_count:4d}] {st_tag} "
                              f"v={v:+.2f} w={w:+.2f} "
                              f"hdg={arduino_heading_deg:+5.0f}° "
                              f"F={prox['front']:.0f} "
                              f"FL={prox['front_left']:.0f} "
                              f"FR={prox['front_right']:.0f} "
                              f"safe={safe_count}",
                              flush=True)

                        scan_points    = []
                        last_send_time = now

            now = time.time()
            if now - last_cmd_time > KEEPALIVE_INTERVAL:
                cmd = f"{prev_v_cmd:.2f} {prev_w_cmd:.2f}\n"
                arduino.write(cmd.encode('utf-8'))
                last_cmd_time = now

    except KeyboardInterrupt:
        print("\n중단", flush=True)
    except Exception as e:
        print(f"[에러] {e}", flush=True)
        import traceback
        traceback.print_exc()
    finally:
        try:
            arduino.write(b"0.00 0.00\n")
            time.sleep(0.1)
            lidar.write(bytes([0xA5, 0x25]))
        except: pass
        lidar.close()
        arduino.close()
        print("✓ 종료", flush=True)


if __name__ == "__main__":
    main()

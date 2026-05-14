"""
DWA Pro v3 - 라이다 동기화 버그 수정
─────────────────────────────────────────────────────────────
수정사항:
1) find_lidar_sync 완전 재작성: RPLIDAR 표준 5바이트 패킷 헤더 검증
2) parse_packet 검증 강화: S/!S 플래그 + C=1 비트 체크
3) 동기화 실패 시 진단 정보 출력
"""

import time
import math
import serial

# ── 1. 하드웨어 포트 ─────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

# ── 2. 안전 파라미터 ─────────────────────────────────────────────────────────
SAFETY_DIST_MM         = 130.0
ROBOT_RADIUS_MM        = 130.0
EMERGENCY_THRESHOLD_MM = 130.0
RECOVERY_CLEAR_MM      = 220.0
LIDAR_NOISE_MM         = 80.0

# ── 3. DWA 파라미터 ──────────────────────────────────────────────────────────
MAX_V         = 0.45
MAX_V_NARROW  = 0.20
MAX_W         = 1.50
DT            = 0.10
PREDICT_TIME  = 1.00
SEND_INTERVAL = 0.10

W_HEADING     = 0.8
W_CLEARANCE   = 2.5
W_VELOCITY    = 1.0
W_SMOOTHNESS  = 3.5

W_DEADZONE = 0.10

# ── 4. RECOVERY 파라미터 ─────────────────────────────────────────────────────
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
# ★ 라이다 통신 (수정)
# ════════════════════════════════════════════════════════════════════════════
def is_valid_first_byte(b):
    """
    RPLIDAR 표준 스캔 패킷의 첫 바이트 검증:
    - bit 0: S (start flag)
    - bit 1: !S (inverse start flag)
    - S와 !S는 항상 반대여야 함
    """
    s     = b & 0x01
    s_inv = (b >> 1) & 0x01
    return s != s_inv

def is_valid_second_byte(b):
    """두 번째 바이트의 bit 0(C)은 항상 1이어야 함"""
    return (b & 0x01) == 1


def parse_packet(raw):
    """
    RPLIDAR 표준 5바이트 패킷 파싱 (검증 포함)
    """
    if len(raw) < 5:
        return None
    
    # Byte 0 검증: S != !S
    if not is_valid_first_byte(raw[0]):
        return None
    
    # Byte 1 검증: C = 1
    if not is_valid_second_byte(raw[1]):
        return None
    
    s_flag      = raw[0] & 0x01
    quality     = (raw[0] >> 2) & 0x3F
    angle_q6    = ((raw[2] << 7) | (raw[1] >> 1)) & 0x7FFF
    angle_deg   = angle_q6 / 64.0
    distance_q2 = (raw[4] << 8) | raw[3]
    distance_mm = distance_q2 / 4.0
    
    return angle_deg, distance_mm, s_flag


def find_lidar_sync(lidar, verbose=True):
    """
    ★ 수정된 라이다 동기화
    
    1바이트씩 읽으면서 유효한 5바이트 패킷 헤더 찾기
    """
    if verbose:
        print("[라이다] 동기화 중 (표준 5바이트 패킷)...", flush=True)
    
    deadline = time.time() + 3.0
    bytes_read = 0
    rejects = 0
    
    while time.time() < deadline:
        # 1바이트 읽기
        b = lidar.read(1)
        if len(b) == 0:
            continue
        bytes_read += 1
        
        # Byte 0 후보 검증
        if not is_valid_first_byte(b[0]):
            rejects += 1
            continue
        
        # Byte 1 읽기 + 검증
        b2 = lidar.read(1)
        if len(b2) == 0:
            continue
        
        if not is_valid_second_byte(b2[0]):
            rejects += 1
            continue
        
        # 나머지 3바이트 읽기
        rest = lidar.read(3)
        if len(rest) != 3:
            rejects += 1
            continue
        
        # 한 번 더 파싱 검증
        full_packet = b + b2 + rest
        result = parse_packet(full_packet)
        if result is None:
            rejects += 1
            continue
        
        angle, distance, s_flag = result
        
        # 각도가 [0, 360] 범위인지 확인 (정상이면 거의 항상 그렇음)
        if 0 <= angle <= 360 and 0 <= distance <= 10000:
            if verbose:
                print(f"[라이다] 동기화 성공! (읽음={bytes_read}B, 거부={rejects})",
                      flush=True)
                print(f"[라이다] 첫 패킷: 각도={angle:.1f}°, 거리={distance:.0f}mm",
                      flush=True)
            return True
        else:
            rejects += 1
            continue
    
    if verbose:
        print(f"[라이다] ✗ 동기화 실패 (읽음={bytes_read}B, 거부={rejects})", flush=True)
        if bytes_read == 0:
            print("[진단] 데이터가 전혀 안 들어옴 → 라이다 모터 미작동 또는 연결 문제",
                  flush=True)
        elif bytes_read > 100 and rejects == bytes_read:
            print("[진단] 데이터는 오는데 패킷 형식이 다름 → 보드레이트 또는 모델 확인",
                  flush=True)
    return False


def start_lidar(lidar):
    """라이다 정지 → 모터 시작 → 스캔 시작 절차"""
    print("[라이다] 시작 명령 전송 중...", flush=True)
    
    # 1. 정지
    lidar.write(bytes([0xA5, 0x25]))
    time.sleep(0.1)
    
    # 2. 모터 컨트롤 (일부 모델)
    try:
        lidar.dtr = False   # DTR Low → 모터 ON (RPLIDAR A1)
    except Exception:
        pass
    time.sleep(0.5)
    
    # 3. 입력 버퍼 클리어
    lidar.reset_input_buffer()
    
    # 4. 스캔 시작 명령
    lidar.write(bytes([0xA5, 0x20]))
    time.sleep(0.5)
    
    # 5. 응답 디스크립터 7바이트 읽기
    descriptor = lidar.read(7)
    print(f"[라이다] 응답 디스크립터: {descriptor.hex()} (길이={len(descriptor)})",
          flush=True)
    
    # 디스크립터 검증: 0xA5 0x5A로 시작해야 함
    if len(descriptor) >= 2 and descriptor[0] == 0xA5 and descriptor[1] == 0x5A:
        print("[라이다] ✓ 유효한 디스크립터 확인", flush=True)
        return True
    else:
        print("[라이다] ⚠ 디스크립터 비정상 - 계속 진행", flush=True)
        return True   # 일부 모델에서 디스크립터가 다를 수 있어 계속 진행


# ════════════════════════════════════════════════════════════════════════════
# 아두이노 통신
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
    """방향별 최소 거리. 라이다 0°=정면, +가 좌측(반시계), -가 우측(시계)."""
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
# DWA 핵심
# ════════════════════════════════════════════════════════════════════════════
def calculate_clearance(v, w, scan_points):
    min_dist = float('inf')
    x = y = theta = 0.0
    steps = int(PREDICT_TIME / DT)

    if abs(v) < 1e-3 and abs(w) < 1e-3:
        nearest = min((d for _, d in scan_points), default=99999)
        return nearest if nearest >= ROBOT_RADIUS_MM else -1.0

    for _ in range(steps):
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


def run_dwa(scan_points, prev_v, prev_w, narrow_mode):
    v_max = MAX_V_NARROW if narrow_mode else MAX_V

    v_candidates = [0.0, v_max * 0.33, v_max * 0.66, v_max]
    w_candidates = [-1.2, -0.8, -0.5, -0.25, -0.1, 0.0, 0.1, 0.25, 0.5, 0.8, 1.2]

    best_v, best_w = 0.0, 0.0
    best_score = -float('inf')
    safe_count = 0

    for v in v_candidates:
        for w in w_candidates:
            if abs(w) > MAX_W: continue

            clearance = calculate_clearance(v, w, scan_points)
            if clearance < 0: continue
            safe_count += 1

            heading_score = (math.pi - abs(normalize_angle_rad(w * PREDICT_TIME))) / math.pi
            clearance_score = min(clearance / 1000.0, 1.0)
            velocity_score = v / max(v_max, 1e-3)
            smoothness_score = -abs(w - prev_w) / (2.0 * MAX_W)

            score = (W_HEADING    * heading_score
                   + W_CLEARANCE  * clearance_score
                   + W_VELOCITY   * velocity_score
                   + W_SMOOTHNESS * smoothness_score)

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

    print("=" * 60, flush=True)
    print("DWA Pro v3 (라이다 동기화 수정)", flush=True)
    print("=" * 60, flush=True)

    # 라이다 연결
    try:
        lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
        time.sleep(0.5)
        print(f"✓ 라이다 시리얼 연결: {LIDAR_PORT} @ {BAUDRATE_LIDAR}", flush=True)
    except Exception as e:
        print(f"✗ 라이다 연결 실패: {e}", flush=True)
        return

    # 아두이노 연결
    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=0.1)
        time.sleep(0.5)
        print(f"✓ 아두이노 연결: {ARDUINO_PORT} @ {BAUDRATE_ARDUINO}", flush=True)
    except Exception as e:
        print(f"✗ 아두이노 연결 실패: {e}", flush=True)
        lidar.close()
        return

    # 초기 정지
    arduino.write(b"0.00 0.00\n")
    time.sleep(0.3)

    # ★ 라이다 시작 (개선된 절차)
    if not start_lidar(lidar):
        print("✗ 라이다 시작 실패", flush=True)
        arduino.write(b"0.00 0.00\n")
        lidar.close()
        arduino.close()
        return

    # ★ 동기화 (수정된 함수)
    if not find_lidar_sync(lidar):
        print("✗ 동기화 실패 → 종료", flush=True)
        print("\n[디버그 명령]", flush=True)
        print("  ls /dev/ttyUSB*           # 라이다 포트 확인", flush=True)
        print("  cat /dev/ttyUSB0 | xxd | head  # raw 데이터 확인", flush=True)
        arduino.write(b"0.00 0.00\n")
        try:
            lidar.write(bytes([0xA5, 0x25]))
        except: pass
        lidar.close()
        arduino.close()
        return

    print("\n" + "=" * 60, flush=True)
    print("주행 시작!", flush=True)
    print("=" * 60, flush=True)

    scan_points    = []
    last_send_time = time.time()
    last_cmd_time  = time.time()
    dwa_count      = 0
    packet_count   = 0
    invalid_count  = 0
    last_diag_time = time.time()

    try:
        while True:
            read_arduino(arduino)

            raw = lidar.read(5)
            if len(raw) == 5:
                pkt = parse_packet(raw)
                if pkt is None:
                    invalid_count += 1
                    # 패킷 동기화 깨졌을 가능성 → 재동기화
                    if invalid_count > 100:
                        print(f"[경고] 무효 패킷 {invalid_count}회 → 재동기화 시도", flush=True)
                        lidar.reset_input_buffer()
                        find_lidar_sync(lidar, verbose=False)
                        invalid_count = 0
                else:
                    packet_count += 1
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
                            narrow = front_min < 350.0
                            v, w, safe_count = run_dwa(
                                scan_points, prev_v_cmd, prev_w_cmd, narrow
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
                                    print(f"  [탈출 성공] front={front_min:.0f}",
                                          flush=True)
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
            else:
                invalid_count += 1

            # 1초마다 진단
            now = time.time()
            if now - last_diag_time >= 2.0:
                if packet_count == 0:
                    print(f"[진단] 패킷 0 수신! 버퍼={lidar.in_waiting}B", flush=True)
                packet_count = 0
                invalid_count = 0
                last_diag_time = now

            # Keep-alive
            if now - last_cmd_time > KEEPALIVE_INTERVAL:
                cmd = f"{prev_v_cmd:.2f} {prev_w_cmd:.2f}\n"
                arduino.write(cmd.encode('utf-8'))
                last_cmd_time = now

    except KeyboardInterrupt:
        print("\n사용자 중단", flush=True)
    except Exception as e:
        print(f"[에러] {e}", flush=True)
        import traceback
        traceback.print_exc()
    finally:
        try:
            arduino.write(b"0.00 0.00\n")
            time.sleep(0.1)
            lidar.write(bytes([0xA5, 0x25]))
        except Exception:
            pass
        lidar.close()
        arduino.close()
        print("✓ 종료", flush=True)


if __name__ == "__main__":
    main()

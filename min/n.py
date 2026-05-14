"""
DWA Pro v2 - 진동 방지 / 13cm 안전거리 / 강제 탈출
─────────────────────────────────────────────────────────────
요구사항 반영:
1) 좌우 진동 억제 → 이전 명령과의 연속성 페널티 + 작은 w 데드존
2) 13cm까지는 멈추지 않음 → SAFETY_DIST_MM = 130
3) 멈추면 어떻게든 탈출 → 다단계 RECOVERY (후진 → 회전 → 재시도)
4) 작은 맵 가정 → 라이다 노이즈 필터를 130mm 미만이 아니라 80mm 미만으로
"""

import time
import math
import serial

# ── 1. 하드웨어 포트 (본인 환경에 맞게 수정) ─────────────────────────────────
LIDAR_PORT   = '/dev/ttyUSB0'
ARDUINO_PORT = '/dev/ttyACM3'
BAUDRATE     = 115200

# ── 2. 안전 파라미터 (사용자 요구사항) ──────────────────────────────────────
SAFETY_DIST_MM         = 130.0   # 13cm: 이 안쪽으로 들어오면 비상
ROBOT_RADIUS_MM        = 130.0   # DWA 충돌 판정 임계점 = 안전거리와 동일
EMERGENCY_THRESHOLD_MM = 130.0   # 비상 전환 임계점
RECOVERY_CLEAR_MM      = 220.0   # 22cm 이상 확보되면 RECOVERY 종료
LIDAR_NOISE_MM         = 80.0    # 8cm 미만은 노이즈로 간주 (스캐너 자체 허위값)

# ── 3. DWA 파라미터 ─────────────────────────────────────────────────────────
MAX_V         = 0.45
MAX_V_NARROW  = 0.20    # 좁은 곳에서는 천천히
MAX_W         = 1.50
DT            = 0.10
PREDICT_TIME  = 1.00
SEND_INTERVAL = 0.10

# DWA 점수 가중치 (진동 방지의 핵심)
W_HEADING     = 0.8
W_CLEARANCE   = 2.5     # 장애물 회피 최우선
W_VELOCITY    = 1.0
W_SMOOTHNESS  = 3.5     # 이전 명령과의 차이 페널티 (이게 진동을 막음)

W_DEADZONE = 0.10       # 절댓값 이하의 w는 0으로 (떨림 방지)

# ── 4. RECOVERY 파라미터 ────────────────────────────────────────────────────
REC_BACK_DUR    = 0.8   # 후진 단계
REC_TURN_DUR    = 1.4   # 회전 단계
REC_SPIN_DUR    = 0.8   # 제자리 회전 단계
REC_CYCLE       = REC_BACK_DUR + REC_TURN_DUR + REC_SPIN_DUR  # 한 사이클 = 3.0초
REC_MAX_ATTEMPT = 5     # 시도 횟수 (방향 번갈아가며)

# ── 5. 통신 워치독 ──────────────────────────────────────────────────────────
KEEPALIVE_INTERVAL = 0.30   # 아두이노 CMD_TIMEOUT(0.5s)보다 짧게


# ════════════════════════════════════════════════════════════════════════════
# 상태 및 글로벌 변수
# ════════════════════════════════════════════════════════════════════════════
class RobotState:
    DRIVE    = 1
    RECOVERY = 2

current_state          = RobotState.DRIVE
arduino_heading_deg    = 0.0

# 진동 방지를 위한 이전 명령 기억
prev_v_cmd = 0.0
prev_w_cmd = 0.0

# RECOVERY 상태 변수
rec_start_time    = 0.0
rec_attempt       = 0
rec_initial_sign  = 1     # 처음 회전 방향 (좌/우 거리로 결정)


# ════════════════════════════════════════════════════════════════════════════
# 유틸리티
# ════════════════════════════════════════════════════════════════════════════
def normalize_angle_deg(angle):
    while angle > 180:   angle -= 360
    while angle <= -180: angle += 360
    return angle

def normalize_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ════════════════════════════════════════════════════════════════════════════
# 라이다 통신
# ════════════════════════════════════════════════════════════════════════════
def parse_packet(raw):
    if len(raw) < 5: return None
    s_flag      = raw[0] & 0x01
    angle_q6    = (raw[2] << 7) | (raw[1] >> 1)
    angle_deg   = angle_q6 / 64.0
    distance_q2 = (raw[4] << 8) | raw[3]
    distance_mm = distance_q2 / 4.0
    return angle_deg, distance_mm, s_flag

def find_lidar_sync(lidar):
    print("라이다 동기화 중...", flush=True)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        b = lidar.read(1)
        if b == b'\xaa' and lidar.read(1) == b'\x55':
            return True
    return False


# ════════════════════════════════════════════════════════════════════════════
# 아두이노 통신 (헤딩 수신)
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
# 라이다 스캔 분석
# ════════════════════════════════════════════════════════════════════════════
def analyze_proximity(scan_points):
    """방향별 최소 거리 (mm). 0도가 정면, +가 좌, -가 우."""
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
# DWA 핵심: 충돌 판정 + 점수 평가
# ════════════════════════════════════════════════════════════════════════════
def calculate_clearance(v, w, scan_points):
    """예측 궤적이 장애물 ROBOT_RADIUS_MM 안에 들어가면 -1 반환."""
    min_dist = float('inf')
    x = y = theta = 0.0
    steps = int(PREDICT_TIME / DT)

    # 정지 궤적은 현재 가장 가까운 장애물 거리만 평가
    if abs(v) < 1e-3 and abs(w) < 1e-3:
        nearest = min((d for _, d in scan_points), default=99999)
        return nearest if nearest >= ROBOT_RADIUS_MM else -1.0

    for _ in range(steps):
        x     += v * math.cos(theta) * DT * 1000.0   # m → mm
        y     += v * math.sin(theta) * DT * 1000.0
        theta += w * DT
        for a_deg, d in scan_points:
            ar = math.radians(a_deg)
            ox = d * math.cos(ar)
            oy = d * math.sin(ar)
            dist = math.hypot(ox - x, oy - y)
            if dist < ROBOT_RADIUS_MM:
                return -1.0   # 충돌
            if dist < min_dist:
                min_dist = dist
    return min_dist


def run_dwa(scan_points, prev_v, prev_w, narrow_mode):
    """다음 (v, w) 결정. narrow_mode=True면 속도 후보를 줄임."""
    v_max = MAX_V_NARROW if narrow_mode else MAX_V

    # 후보 (속도는 양수만 — 후진은 RECOVERY 전용)
    v_candidates = [0.0, v_max * 0.33, v_max * 0.66, v_max]
    w_candidates = [-1.2, -0.8, -0.5, -0.25, -0.1, 0.0, 0.1, 0.25, 0.5, 0.8, 1.2]

    best_v, best_w = 0.0, 0.0
    best_score = -float('inf')
    safe_count = 0

    for v in v_candidates:
        for w in w_candidates:
            if abs(w) > MAX_W: continue

            clearance = calculate_clearance(v, w, scan_points)
            if clearance < 0: continue       # 충돌 → 폐기

            safe_count += 1

            # ── 점수 계산 ────────────────────────────────────────────────
            # 1) 헤딩: 정면을 향해 직진하는 궤적을 선호 (예측 헤딩이 0에 가까울수록 좋음)
            heading_score = (math.pi - abs(normalize_angle_rad(w * PREDICT_TIME))) / math.pi

            # 2) 여유공간: 정규화 (1m 이상이면 만점)
            clearance_score = min(clearance / 1000.0, 1.0)

            # 3) 속도: 빠를수록 좋음
            velocity_score = v / max(v_max, 1e-3)

            # 4) 부드러움(진동 방지): 이전 w와의 차이 페널티
            smoothness_score = -abs(w - prev_w) / (2.0 * MAX_W)

            score = (W_HEADING    * heading_score
                   + W_CLEARANCE  * clearance_score
                   + W_VELOCITY   * velocity_score
                   + W_SMOOTHNESS * smoothness_score)

            if score > best_score:
                best_score = score
                best_v, best_w = v, w

    # 데드존: 작은 w는 0으로 → 직진 시 잔떨림 제거
    if abs(best_w) < W_DEADZONE:
        best_w = 0.0

    return best_v, best_w, safe_count


# ════════════════════════════════════════════════════════════════════════════
# RECOVERY: 단계별 강제 탈출
# ════════════════════════════════════════════════════════════════════════════
def pick_recovery_direction(prox):
    """좌/우 어느 쪽이 더 trav(통과 가능)한지 보고 회전 방향 결정."""
    left_room  = prox['front_left']  + prox['left']
    right_room = prox['front_right'] + prox['right']
    return 1 if left_room >= right_room else -1   # +1=좌회전, -1=우회전


def recovery_step(elapsed, attempt, initial_sign, prox):
    """
    한 사이클(REC_CYCLE = 3.0초):
      Phase 1 (0 ~ 0.8s)      : 순수 후진
      Phase 2 (0.8 ~ 2.2s)    : 후진 + 회전
      Phase 3 (2.2 ~ 3.0s)    : 제자리 회전 (강하게)
    홀수번째 시도는 반대 방향으로.
    """
    # 시도 횟수에 따라 방향 번갈아 가며
    sign = initial_sign if (attempt % 2 == 0) else -initial_sign

    if elapsed < REC_BACK_DUR:
        return -0.15, 0.0

    elif elapsed < REC_BACK_DUR + REC_TURN_DUR:
        return -0.08, MAX_W * sign

    else:
        # 제자리에서 강하게 회전
        return 0.0, MAX_W * sign


# ════════════════════════════════════════════════════════════════════════════
# 메인 루프
# ════════════════════════════════════════════════════════════════════════════
def main():
    global current_state, arduino_heading_deg
    global prev_v_cmd, prev_w_cmd
    global rec_start_time, rec_attempt, rec_initial_sign

    print("=" * 60, flush=True)
    print("DWA Pro v2 시작", flush=True)
    print(f"  안전거리      : {SAFETY_DIST_MM:.0f} mm", flush=True)
    print(f"  로봇반경      : {ROBOT_RADIUS_MM:.0f} mm", flush=True)
    print(f"  라이다 필터   : {LIDAR_NOISE_MM:.0f} mm 미만 무시", flush=True)
    print(f"  진동방지 가중 : {W_SMOOTHNESS}", flush=True)
    print("=" * 60, flush=True)

    # 시리얼 초기화
    try:
        lidar = serial.Serial(LIDAR_PORT, BAUDRATE, timeout=1)
        time.sleep(0.5)
    except Exception as e:
        print(f"✗ 라이다 연결 실패: {e}", flush=True); return

    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE, timeout=0.1)
        time.sleep(0.5)
    except Exception as e:
        print(f"✗ 아두이노 연결 실패: {e}", flush=True); lidar.close(); return

    # 초기 정지 명령 (아두이노 모드 락은 공백 구분자 → 고급 모드)
    arduino.write(b"0.00 0.00\n")
    time.sleep(0.3)

    # 라이다 스캔 시작
    try:
        lidar.write(bytes([0xA5, 0x25]))   # STOP
        time.sleep(0.1)
        lidar.reset_input_buffer()
        lidar.write(bytes([0xA5, 0x20]))   # SCAN
        time.sleep(0.5)
        lidar.read(7)                       # response descriptor 소비
    except Exception:
        pass

    if not find_lidar_sync(lidar):
        print("✗ 라이다 동기화 실패", flush=True)
        arduino.write(b"0.00 0.00\n")
        lidar.close(); arduino.close(); return

    scan_points    = []
    last_send_time = time.time()
    last_cmd_time  = time.time()
    dwa_count      = 0

    try:
        while True:
            # 1) 아두이노로부터 헤딩 수신
            read_arduino(arduino)

            # 2) 라이다 1패킷 수신
            raw = lidar.read(5)
            if len(raw) == 5:
                pkt = parse_packet(raw)
                if pkt is not None:
                    angle_raw, distance, s_flag = pkt
                    now = time.time()

                    # 노이즈 필터: 8cm 미만만 버림 (13cm 안전거리 영역은 살림!)
                    if distance > LIDAR_NOISE_MM:
                        scan_points.append((angle_raw, distance))

                    # 한 바퀴 완료 (s_flag=1) + 전송 주기 도달
                    if s_flag == 1 and len(scan_points) > 30 \
                       and (now - last_send_time >= SEND_INTERVAL):

                        dwa_count += 1
                        prox = analyze_proximity(scan_points)
                        front_min = min(prox['front'],
                                        prox['front_left'],
                                        prox['front_right'])

                        # ── 상태 머신 ─────────────────────────────────────
                        if current_state == RobotState.DRIVE:
                            # 좁은 영역이면 속도 제한
                            narrow = front_min < 350.0

                            v, w, safe_count = run_dwa(
                                scan_points, prev_v_cmd, prev_w_cmd, narrow
                            )

                            # 13cm 안으로 들어왔거나 모든 궤적이 막혔으면 RECOVERY로
                            if front_min < EMERGENCY_THRESHOLD_MM or safe_count == 0:
                                print(f"  [RECOVERY 진입] front_min={front_min:.0f}mm, "
                                      f"safe={safe_count}", flush=True)
                                current_state    = RobotState.RECOVERY
                                rec_start_time   = now
                                rec_attempt      = 0
                                rec_initial_sign = pick_recovery_direction(prox)
                                v, w = -0.15, 0.0
                                safe_count = -1

                        else:  # RECOVERY
                            elapsed = now - rec_start_time
                            v, w = recovery_step(elapsed, rec_attempt,
                                                  rec_initial_sign, prox)
                            safe_count = -1

                            # 사이클 종료 시점: 탈출 여부 점검
                            if elapsed >= REC_CYCLE:
                                if front_min > RECOVERY_CLEAR_MM:
                                    # 탈출 성공
                                    print(f"  [탈출 성공] front_min={front_min:.0f}mm "
                                          f"(시도 {rec_attempt+1}회차)", flush=True)
                                    current_state = RobotState.DRIVE
                                    prev_v_cmd = 0.0
                                    prev_w_cmd = 0.0
                                else:
                                    # 사이클 재시작 (방향 바꿔서)
                                    rec_attempt += 1
                                    rec_start_time = now
                                    print(f"  [재시도] {rec_attempt}회차, "
                                          f"방향 {'L' if (rec_attempt % 2 == 0) == (rec_initial_sign > 0) else 'R'}",
                                          flush=True)

                                    # 일정 횟수 이상 실패 시 좌/우 거리 다시 계산
                                    if rec_attempt >= REC_MAX_ATTEMPT:
                                        rec_attempt      = 0
                                        rec_initial_sign = pick_recovery_direction(prox)
                                        print(f"  [재계획] 방향 다시 결정", flush=True)

                        # 명령 송신
                        cmd = f"{v:.2f} {w:.2f}\n"
                        arduino.write(cmd.encode('utf-8'))
                        last_cmd_time = now

                        # 이전 명령 갱신 (진동 방지용)
                        prev_v_cmd = v
                        prev_w_cmd = w

                        # 로그
                        st_tag = "DRIVE" if current_state == RobotState.DRIVE else "RECOV"
                        print(f"[{dwa_count:4d}] {st_tag} "
                              f"v={v:+.2f} w={w:+.2f}  "
                              f"hdg={arduino_heading_deg:+6.1f}°  "
                              f"F={prox['front']:5.0f} FL={prox['front_left']:5.0f} "
                              f"FR={prox['front_right']:5.0f}  "
                              f"safe={safe_count}",
                              flush=True)

                        scan_points    = []
                        last_send_time = now

            # 3) Keep-alive (아두이노 0.5초 워치독보다 짧게)
            now = time.time()
            if now - last_cmd_time > KEEPALIVE_INTERVAL:
                # 마지막 명령을 그대로 재전송 (정지 강제하지 않음)
                cmd = f"{prev_v_cmd:.2f} {prev_w_cmd:.2f}\n"
                arduino.write(cmd.encode('utf-8'))
                last_cmd_time = now

    except KeyboardInterrupt:
        print("\n사용자 중단", flush=True)
    except Exception as e:
        print(f"[에러] {e}", flush=True)
    finally:
        try:
            arduino.write(b"0.00 0.00\n")
            time.sleep(0.1)
            lidar.write(bytes([0xA5, 0x25]))   # STOP
        except Exception:
            pass
        lidar.close()
        arduino.close()
        print("✓ 종료", flush=True)


if __name__ == "__main__":
    main()

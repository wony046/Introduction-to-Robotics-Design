import serial
import time
import math
import sys

# ── 1. 설정 및 파라미터 ───────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

# 로봇 하드웨어 (mm)
ROBOT_FRONT  = 110
ROBOT_BACK   = 150
ROBOT_HALF_W = 110
MARGIN       = 40

# 주행 성능
MAX_V = 0.35
MAX_W = 1.5

# DWA 채점 가중치
W_HEADING   = 1.5
W_CLEARANCE = 2.5
W_VELOCITY  = 0.8
BIAS_BONUS  = 0.2

# 시뮬레이션 파라미터
PREDICT_T = 1.2
SIM_STEP  = 0.1

# ── 2. FSM 및 전역 상태 ───────────────────────────────────────────────────────
class RobotState:
    DRIVE    = 1
    RECOVERY = 2

current_state       = RobotState.DRIVE
stuck_timer         = 0.0
last_w_sign         = 0.0
arduino_heading_deg = 0.0

# ── 3. 유틸리티 및 라이다 파싱 ────────────────────────────────────────────────
def normalize_angle(angle):
    while angle >  180: angle -= 360
    while angle < -180: angle += 360
    return angle


def parse_packet(data):
    if len(data) != 5:
        return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        return None
    if (data[1] & 0x01) != 1:
        return None
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    angle       = angle_q6 / 64.0
    distance_q2 = data[3] | (data[4] << 8)
    distance    = distance_q2 / 4.0
    return angle, distance


def read_arduino(arduino):
    """아두이노에서 헤딩 정보 읽기 (논블로킹)"""
    global arduino_heading_deg
    try:
        while arduino.in_waiting > 0:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'):
                arduino_heading_deg = float(line[2:])
            elif line:
                print(f"[Arduino] {line}")
    except Exception as e:
        print(f"[경고] Arduino 읽기 오류: {e}")

# ── 4. DWA 코어 ───────────────────────────────────────────────────────────────
def generate_vw_window():
    v_cands = [0.0, 0.10, 0.18, 0.25, MAX_V]
    w_cands = [-MAX_W, -1.2, -0.9, -0.6, -0.3, 0.0,
                0.3,    0.6,  0.9,  1.2,  MAX_W]
    return v_cands, w_cands


def check_collision_and_clearance(v_m_s, w_rad_s, local_pts,
                                   predict_t=PREDICT_T,
                                   step=SIM_STEP):
    """
    궤적 시뮬레이션 및 충돌 체크
    
    좌표계: 앞=+X, 왼쪽=+Y, 반시계회전=+θ
    """
    v_mm_s = v_m_s * 1000.0

    if not local_pts:
        return 1000.0

    curr_x, curr_y, curr_th = 0.0, 0.0, 0.0
    min_clear_sq = 1e12

    front_bound =  ROBOT_FRONT  + MARGIN
    back_bound  = -(ROBOT_BACK  + MARGIN)
    side_bound  =  ROBOT_HALF_W + MARGIN

    # 시간 적분 (Euler)
    n_steps = max(1, int(predict_t / step) + 1)
    for _ in range(n_steps):
        curr_x  += v_mm_s * math.cos(curr_th) * step
        curr_y  += v_mm_s * math.sin(curr_th) * step
        curr_th += w_rad_s * step

        cos_t = math.cos(curr_th)
        sin_t = math.sin(curr_th)

        for px, py in local_pts:
            dx = px - curr_x
            dy = py - curr_y

            # 현재 로봇 자세 기준 로컬 좌표
            lx =  cos_t * dx + sin_t * dy
            ly = -sin_t * dx + cos_t * dy

            # 충돌 판정
            if back_bound <= lx <= front_bound and -side_bound <= ly <= side_bound:
                return -1.0

            dist_sq = dx * dx + dy * dy
            if dist_sq < min_clear_sq:
                min_clear_sq = dist_sq

    return math.sqrt(min_clear_sq)


def run_dwa(scan_points, curr_heading):
    global last_w_sign

    # 라이다 포인트 → 로봇 로컬 좌표 변환
    local_pts = []
    for ang, dist in scan_points:
        if 0 < dist <= 1500:
            rad = math.radians(ang)
            local_pts.append((
                 dist * math.cos(rad),
                -dist * math.sin(rad)
            ))

    if not local_pts:
        # 스캔 데이터 없음 → 천천히 전진
        return 0.15, 0.0

    v_cands, w_cands = generate_vw_window()

    best_v, best_w = 0.0, 0.0
    max_score      = -1e9

    for v in v_cands:
        for w in w_cands:
            clearance = check_collision_and_clearance(v, w, local_pts)
            if clearance < 0:  # 충돌
                continue

            # 헤딩 점수
            pred_turn   = math.degrees(w * 1.0)
            fut_heading = normalize_angle(curr_heading - pred_turn)
            score_heading = max(0.0, 1.0 - abs(fut_heading) / 180.0)

            # 안전거리 점수
            score_clearance = min(1.0, max(0, clearance) / 1000.0)

            # 속도 점수
            score_velocity = v / MAX_V if v > 0 else 0.0

            # 일관된 회전 방향 보너스
            bias = BIAS_BONUS if (w != 0 and w * last_w_sign > 0) else 0.0

            # 정지 패널티
            stop_penalty = -0.8 if (v < 0.01) else 0.0

            total = (W_HEADING   * score_heading   +
                     W_CLEARANCE * score_clearance +
                     W_VELOCITY  * score_velocity  +
                     bias + stop_penalty)

            if total > max_score:
                max_score      = total
                best_v, best_w = v, w

    # Fallback: 모든 궤적 충돌 시
    if max_score <= -1e8:
        best_v = 0.0
        best_w = MAX_W if curr_heading <= 0 else -MAX_W
        print("[경고] 모든 궤적 충돌 → 제자리 회전")

    if best_w != 0:
        last_w_sign = 1.0 if best_w > 0 else -1.0

    return best_v, best_w

# ── 5. 메인 루프 ──────────────────────────────────────────────────────────────
def main():
    global current_state, stuck_timer, arduino_heading_deg

    print("=" * 60)
    print("라이다 및 아두이노 초기화 중...")
    print("=" * 60)

    # 시리얼 포트 연결
    try:
        print(f"라이다 포트: {LIDAR_PORT} (보드레이트: {BAUDRATE_LIDAR})")
        lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
        time.sleep(0.5)
        print("✓ 라이다 연결 완료")
    except Exception as e:
        print(f"✗ 라이다 연결 실패: {e}")
        return

    try:
        print(f"아두이노 포트: {ARDUINO_PORT} (보드레이트: {BAUDRATE_ARDUINO})")
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
        time.sleep(0.5)
        print("✓ 아두이노 연결 완료")
    except Exception as e:
        print(f"✗ 아두이노 연결 실패: {e}")
        lidar.close()
        return

    # 라이다 초기화
    print("\n라이다 스캔 시작...")
    try:
        lidar.write(bytes([0xA5, 0x40]))
        time.sleep(1)
        lidar.write(bytes([0xA5, 0x20]))
        response = lidar.read(7)
        if len(response) > 0:
            print(f"✓ 라이다 응답: {response.hex()}")
        else:
            print("⚠ 라이다 응답 없음 (계속 진행)")
    except Exception as e:
        print(f"⚠ 라이다 초기화 오류: {e}")

    print("\n" + "=" * 60)
    print("주행 시작! (Ctrl+C로 종료)")
    print("=" * 60)

    scan_points   = []
    last_send     = time.time()
    SEND_INTERVAL = 0.1
    current_v, current_w = 0.0, 0.0
    frame_count = 0
    dwa_count = 0

    try:
        while True:
            # 아두이노 헤딩 읽기
            read_arduino(arduino)

            # 라이다 데이터 읽기 (5바이트 패킷)
            raw = lidar.read(5)
            if len(raw) != 5:
                continue

            result = parse_packet(raw)
            if result is None:
                continue

            angle_raw, distance = result
            s_flag = raw[0] & 0x01
            frame_count += 1

            # 자기 몸 필터 (150mm 이하는 무시)
            if distance > 150:
                scan_points.append((normalize_angle(angle_raw), distance))

            # 한 프레임 완료 + 주기 도래 시 DWA 실행
            now = time.time()
            if s_flag == 1 and scan_points and (now - last_send >= SEND_INTERVAL):
                dwa_count += 1

                if current_state == RobotState.DRIVE:
                    v, w = run_dwa(scan_points, arduino_heading_deg)

                    if v < 0.05:
                        stuck_timer += SEND_INTERVAL
                        if stuck_timer >= 2.0:
                            print("[FSM] 갇힘 감지 → Recovery 모드")
                            try:
                                arduino.write(b"ESC\n")
                            except:
                                pass
                            current_state = RobotState.RECOVERY
                            stuck_timer = 0.0
                    else:
                        stuck_timer = 0.0

                elif current_state == RobotState.RECOVERY:
                    v, w = -0.15, MAX_W
                    stuck_timer += SEND_INTERVAL
                    if stuck_timer >= 2.5:
                        print("[FSM] Recovery 종료 → Drive 모드")
                        current_state = RobotState.DRIVE
                        stuck_timer = 0.0

                # 명령 전송
                current_v, current_w = v, w
                cmd = f"{v:.2f} {w:.2f}\n"
                try:
                    arduino.write(cmd.encode('utf-8'))
                except Exception as e:
                    print(f"[에러] 명령 전송 실패: {e}")

                # 디버그 출력 (5번째 DWA마다)
                if dwa_count % 5 == 0:
                    state_name = "DRIVE" if current_state == RobotState.DRIVE else "RECOVERY"
                    print(f"[{dwa_count:3d}] v={v:6.2f} w={w:6.2f} hdg={arduino_heading_deg:7.1f}° "
                          f"pts={len(scan_points):2d} state={state_name}")

                scan_points = []
                last_send = now

    except KeyboardInterrupt:
        print("\n\n종료 신호 감지...")
    except Exception as e:
        print(f"\n[심각 에러] {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n정리 중...")
        try:
            lidar.write(bytes([0xA5, 0x25]))  # 라이다 정지
            time.sleep(0.1)
        except:
            pass
        try:
            arduino.write(b"0.00 0.00\n")     # 모터 정지
            time.sleep(0.1)
        except:
            pass
        lidar.close()
        arduino.close()
        print("✓ 종료 완료")
        print(f"총 프레임: {frame_count}, DWA 실행: {dwa_count}")


if __name__ == "__main__":
    main()

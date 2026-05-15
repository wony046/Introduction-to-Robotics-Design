"""
DWA Pro v5 - 개선 버전
변경 사항:
  1. 전역 상태 → DriveState dataclass 캡슐화
  2. calculate_clearance numpy 벡터화 (10~50배 성능 향상)
  3. analyze_proximity 각도 겹침 주석 명확화
  4. stuck_count 상한(REC_MAX_STUCK) 추가
  5. start_lidar 예외 처리 추가
  6. print 출력 스로틀링 (10Hz → 2Hz)
"""

import time
import math
import numpy as np
import serial
from dataclasses import dataclass
from typing import Optional

# ── 1. 하드웨어 ─────────────────────────────────────────────────────────────
LIDAR_PORT = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

# ── 2. 안전 ─────────────────────────────────────────────────────────────────
SAFETY_DIST_MM         = 130.0
ROBOT_RADIUS_MM        = 150.0
EMERGENCY_THRESHOLD_MM = 150.0
RECOVERY_CLEAR_MM      = 220.0
LIDAR_NOISE_MM         = 80.0

# ── 3. DWA ──────────────────────────────────────────────────────────────────
MAX_V         = 0.45
MAX_V_NARROW  = 0.20
MAX_W         = 1.50
DT            = 0.10
PREDICT_TIME  = 1.00

W_HEADING     = 1.5
W_CLEARANCE   = 2.5
W_VELOCITY    = 0.7
W_SMOOTHNESS  = 1.5
W_DEADZONE    = 0.10

# ── 4. RECOVERY ─────────────────────────────────────────────────────────────
REC_BACK_DUR     = 0.8
REC_TURN_DUR     = 0.6
REC_TURN_W_RATIO = 0.7
REC_SPIN_DUR     = 0.3
REC_CYCLE        = REC_BACK_DUR + REC_TURN_DUR + REC_SPIN_DUR
REC_STUCK_CYCLE  = REC_BACK_DUR * 1.5 + 1.2
REC_MAX_ATTEMPT  = 2
REC_MAX_STUCK    = 5   # stuck_count 상한 — 이 이상 증가해도 행동은 같고 로그만 오염

KEEPALIVE_INTERVAL = 0.30
SPIN_LOCK_COUNT    = 12     # ~125ms × 12 = 1.5s
PRINT_INTERVAL     = 0.5    # 출력 스로틀링 (초)


class RobotState:
    DRIVE    = 1
    RECOVERY = 2


@dataclass
class DriveState:
    """주행 관련 가변 상태를 한 곳에 모아 전역 변수 제거."""
    current_state:    int            = RobotState.DRIVE
    arduino_heading:  float          = 0.0
    goal_heading:     Optional[float] = None
    prev_v:           float          = 0.0
    prev_w:           float          = 0.0
    rec_start_time:   float          = 0.0
    rec_attempt:      int            = 0
    rec_initial_sign: int            = 1
    stuck_count:      int            = 0
    spin_count:       int            = 0


def normalize_angle_deg(angle):
    while angle > 180:   angle -= 360
    while angle <= -180: angle += 360
    return angle

def normalize_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def is_valid_first_byte(b):
    return (b & 0x01) != ((b >> 1) & 0x01)

def is_valid_second_byte(b):
    return (b & 0x01) == 1


def parse_packet(raw):
    if len(raw) < 5: return None
    if not is_valid_first_byte(raw[0]): return None
    if not is_valid_second_byte(raw[1]): return None
    s_flag      = raw[0] & 0x01
    quality     = (raw[0] >> 2) & 0x3F
    angle_q6    = ((raw[2] << 7) | (raw[1] >> 1)) & 0x7FFF
    angle_deg   = angle_q6 / 64.0
    distance_q2 = (raw[4] << 8) | raw[3]
    distance_mm = distance_q2 / 4.0
    return angle_deg, distance_mm, s_flag, quality


def find_lidar_sync(lidar, verbose=True):
    if verbose: print("[라이다] 동기화...", flush=True)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        b = lidar.read(1)
        if len(b) == 0: continue
        if not is_valid_first_byte(b[0]): continue
        b2 = lidar.read(1)
        if len(b2) == 0: continue
        if not is_valid_second_byte(b2[0]): continue
        rest = lidar.read(3)
        if len(rest) != 3: continue
        result = parse_packet(b + b2 + rest)
        if result is None: continue
        angle, distance, _, _ = result
        if 0 <= angle <= 360 and 0 <= distance <= 10000:
            if verbose:
                print(f"[라이다] OK (각도={angle:.1f}°, 거리={distance:.0f}mm)",
                      flush=True)
            return True
    if verbose:
        print("[라이다] ✗ 동기화 실패", flush=True)
    return False


def start_lidar(lidar):
    print("[라이다] 시작...", flush=True)
    try:
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        try:
            lidar.dtr = False       # 일부 어댑터에서 지원 안 할 수 있음
        except AttributeError:
            pass
        time.sleep(0.5)
        lidar.reset_input_buffer()
        lidar.write(bytes([0xA5, 0x20]))
        time.sleep(0.5)
        descriptor = lidar.read(7)
        print(f"[라이다] descriptor: {descriptor.hex()}", flush=True)
    except serial.SerialException as e:
        print(f"[라이다] ✗ 시작 실패: {e}", flush=True)
        return False
    return True


def read_arduino(arduino, state: DriveState):
    if arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("H:"):
                state.arduino_heading = float(line.split(":")[1])
        except Exception:
            pass


def analyze_proximity(scan_points):
    """라이다 0°=정면, +가 우측(CW), -가 좌측(CCW).

    front와 front_left/front_right는 ±10°~25° 구간이 의도적으로 겹침.
    경계 장애물을 인접 두 섹터 모두에서 포착해 보수적 안전 거리를 유지한다.
    """
    p = {'front': 99999, 'front_left': 99999, 'front_right': 99999,
         'left':  99999, 'right':       99999, 'back': 99999}
    for a, d in scan_points:
        na = normalize_angle_deg(a)
        if -25 <= na <= 25:
            p['front'] = min(p['front'], d)
        if  10 <= na <= 70:
            p['front_right'] = min(p['front_right'], d)
        if -70 <= na <= -10:
            p['front_left']  = min(p['front_left'],  d)
        if  70 <  na <= 130:
            p['right'] = min(p['right'], d)
        if -130 <= na < -70:
            p['left']  = min(p['left'],  d)
        if na > 150 or na < -150:
            p['back'] = min(p['back'], d)
    return p


def _scan_to_cartesian(scan_points):
    """scan_points → numpy 장애물 좌표 (ox, oy).

    라이다 CW 각도 규약(+가 우측)을 로봇 좌표계(x=정면, y=좌측)로 변환.
    run_dwa 안에서 후보별로 반복 호출되지 않도록 DWA 진입 전 1회만 실행.
    """
    if not scan_points:
        return np.empty(0), np.empty(0)
    angles_rad = np.radians([a for a, _ in scan_points])
    dists      = np.array([d for _, d in scan_points])
    ox =  dists * np.cos(angles_rad)
    oy = -dists * np.sin(angles_rad)   # CW → 표준 y축 반전
    return ox, oy


def calculate_clearance(v, w, ox, oy):
    """numpy 벡터화: 궤적 전 스텝 × 전 장애물 거리를 한 번에 계산.

    반환값: 최소 이격 거리(mm), 충돌 예측 시 -1.0
    """
    if len(ox) == 0:
        return 99999.0

    if abs(v) < 1e-3 and abs(w) < 1e-3:
        nearest = float(np.min(np.hypot(ox, oy)))
        return nearest if nearest >= ROBOT_RADIUS_MM else -1.0

    steps = int(PREDICT_TIME / DT)
    j = np.arange(steps)
    thetas = j * w * DT                                     # 각 스텝의 헤딩

    robot_x = np.cumsum(v * np.cos(thetas) * DT * 1000.0)  # (steps,) mm
    robot_y = np.cumsum(v * np.sin(thetas) * DT * 1000.0)

    # (steps, N) 거리² 행렬 — sqrt는 비교 후 최솟값에 한 번만
    dx = ox[np.newaxis, :] - robot_x[:, np.newaxis]
    dy = oy[np.newaxis, :] - robot_y[:, np.newaxis]
    dists_sq = dx * dx + dy * dy

    if np.any(dists_sq < ROBOT_RADIUS_MM ** 2):
        return -1.0
    return float(math.sqrt(float(np.min(dists_sq))))


def run_dwa(scan_points, prev_v, prev_w, narrow_mode, goal_angle_rad=0.0):
    # t=0 즉시 충돌 검사 (정면 ±30° 한정)
    for a_deg, d in scan_points:
        na = normalize_angle_deg(a_deg)
        if abs(na) < 30 and d < ROBOT_RADIUS_MM:
            return 0.0, 0.0, 0

    ox, oy = _scan_to_cartesian(scan_points)   # 좌표 변환 1회
    v_max = MAX_V_NARROW if narrow_mode else MAX_V

    if narrow_mode:
        v_candidates = [0.0, v_max * 0.5, v_max]
        w_candidates = [-1.2, -0.6, -0.3, -0.15, -0.05, 0.0, 0.05, 0.15, 0.3, 0.6, 1.2]
    else:
        v_candidates = [0.0, v_max * 0.33, v_max * 0.66, v_max]
        w_candidates = [-1.2, -0.8, -0.5, -0.25, -0.1, 0.0, 0.1, 0.25, 0.5, 0.8, 1.2]

    best_v, best_w = 0.0, 0.0
    best_score = -float('inf')
    safe_count = 0

    for v in v_candidates:
        for w in w_candidates:
            if abs(w) > MAX_W: continue
            clearance = calculate_clearance(v, w, ox, oy)
            if clearance < 0: continue
            safe_count += 1

            remaining = normalize_angle_rad(goal_angle_rad - w * PREDICT_TIME)
            heading_score = (math.pi - abs(remaining)) / math.pi
            heading_score -= 0.04 * abs(w) * (1.0 - abs(remaining) / math.pi)
            clearance_score  = min(clearance / 1000.0, 1.0)
            velocity_score   = v / max(v_max, 1e-3)
            smoothness_score = -abs(w - prev_w) / (2.0 * MAX_W)

            score = (W_HEADING    * heading_score
                   + W_CLEARANCE  * clearance_score
                   + W_VELOCITY   * velocity_score
                   + W_SMOOTHNESS * smoothness_score)

            if score > best_score:
                best_score = score
                best_v, best_w = v, w

    deadzone = 0.04 if narrow_mode else W_DEADZONE
    if abs(best_w) < deadzone:
        best_w = 0.0

    return best_v, best_w, safe_count


def pick_recovery_direction(prox, goal_angle_rad=0.0):
    left_room  = prox['front_left']  + prox['left']
    right_room = prox['front_right'] + prox['right']
    if goal_angle_rad > 0.2:
        left_room  *= 1.4
    elif goal_angle_rad < -0.2:
        right_room *= 1.4
    return 1 if left_room >= right_room else -1


def recovery_step(elapsed, attempt, initial_sign, prox, stuck=0):
    sign = initial_sign if (attempt % 2 == 0) else -initial_sign
    if stuck >= 1:
        if elapsed < REC_BACK_DUR * 1.5:
            return -0.20, 0.0
        else:
            return 0.0, MAX_W * sign
    turn_w = MAX_W * REC_TURN_W_RATIO * sign
    if elapsed < REC_BACK_DUR:
        return -0.15, 0.0
    elif elapsed < REC_BACK_DUR + REC_TURN_DUR:
        return -0.08, turn_w
    else:
        return 0.0, turn_w


def main():
    state = DriveState()

    print("="*60, flush=True)
    print("DWA Pro v5 (개선 버전)", flush=True)
    print("="*60, flush=True)

    try:
        lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
        time.sleep(0.5)
    except Exception as e:
        print(f"✗ 라이다: {e}", flush=True); return

    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=0.1)
        time.sleep(0.5)
    except Exception as e:
        print(f"✗ 아두이노: {e}", flush=True); lidar.close(); return

    arduino.write(b"0.00 0.00\n")
    time.sleep(0.3)

    if not start_lidar(lidar):
        arduino.write(b"0.00 0.00\n")
        lidar.close(); arduino.close(); return

    if not find_lidar_sync(lidar):
        arduino.write(b"0.00 0.00\n")
        try: lidar.write(bytes([0xA5, 0x25]))
        except: pass
        lidar.close(); arduino.close(); return

    print("\n주행 시작!\n", flush=True)

    scan_points     = []
    last_cmd_time   = time.time()
    last_print_time = 0.0
    dwa_count       = 0
    packet_count    = 0
    invalid_count   = 0
    last_diag_time  = time.time()

    try:
        while True:
            read_arduino(arduino, state)

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
                    packet_count += 1
                    angle_raw, distance, s_flag, quality = pkt
                    now = time.time()

                    if distance > LIDAR_NOISE_MM and quality > 0:
                        scan_points.append((angle_raw, distance))

                    if s_flag == 1:
                        if len(scan_points) > 30:
                            dwa_count += 1
                            prox = analyze_proximity(scan_points)
                            front_min = min(prox['front'],
                                            prox['front_left'],
                                            prox['front_right'])

                            if state.goal_heading is None:
                                state.goal_heading = state.arduino_heading
                                print(f"  [목표] {state.goal_heading:.1f}°", flush=True)

                            goal_angle_rad = math.radians(
                                normalize_angle_deg(state.goal_heading - state.arduino_heading)
                            )

                            if state.current_state == RobotState.DRIVE:
                                side_min = min(prox['left'], prox['right'])
                                narrow = front_min < 350.0 or side_min < 140.0
                                v, w, safe_count = run_dwa(
                                    scan_points, state.prev_v, state.prev_w,
                                    narrow, goal_angle_rad
                                )

                                if v <= 0.01:
                                    state.spin_count += 1
                                else:
                                    state.spin_count = 0

                                needs_recovery = (
                                    state.spin_count >= SPIN_LOCK_COUNT
                                    or prox['front'] < EMERGENCY_THRESHOLD_MM
                                    or safe_count == 0
                                )
                                if needs_recovery:
                                    reason = ("SPIN" if state.spin_count >= SPIN_LOCK_COUNT
                                              else f"F={prox['front']:.0f}")
                                    print(f"  [RECOVERY] {reason} safe={safe_count}", flush=True)
                                    state.current_state    = RobotState.RECOVERY
                                    state.rec_start_time   = now
                                    state.rec_attempt      = 0
                                    state.stuck_count      = 0
                                    state.spin_count       = 0
                                    state.rec_initial_sign = pick_recovery_direction(
                                        prox, goal_angle_rad)
                                    v, w = -0.15, 0.0
                                    safe_count = -1

                            else:
                                elapsed = now - state.rec_start_time
                                v, w = recovery_step(elapsed, state.rec_attempt,
                                                     state.rec_initial_sign, prox,
                                                     state.stuck_count)
                                safe_count = -1

                                if v < 0 and prox['back'] < 200:
                                    v = 0.0
                                    w = MAX_W * REC_TURN_W_RATIO * state.rec_initial_sign

                                cycle = REC_STUCK_CYCLE if state.stuck_count >= 1 else REC_CYCLE
                                if elapsed >= cycle:
                                    if prox['front'] > RECOVERY_CLEAR_MM:
                                        print(f"  [탈출] front={prox['front']:.0f}", flush=True)
                                        state.current_state = RobotState.DRIVE
                                        state.stuck_count   = 0
                                        state.spin_count    = 0
                                        state.prev_v = 0.0
                                        state.prev_w = 0.0
                                    else:
                                        state.rec_attempt += 1
                                        state.rec_start_time = now
                                        if state.rec_attempt >= REC_MAX_ATTEMPT:
                                            state.stuck_count = min(
                                                state.stuck_count + 1, REC_MAX_STUCK
                                            )
                                            state.rec_attempt      = 0
                                            state.rec_initial_sign = pick_recovery_direction(
                                                prox, goal_angle_rad)
                                            print(f"  [STUCK] {state.stuck_count}", flush=True)

                            cmd = f"{v:.2f} {w:.2f}\n"
                            arduino.write(cmd.encode('utf-8'))
                            last_cmd_time = now
                            state.prev_v  = v
                            state.prev_w  = w

                            if now - last_print_time >= PRINT_INTERVAL:
                                st_tag = "D" if state.current_state == RobotState.DRIVE else "R"
                                print(f"[{dwa_count:4d}] {st_tag} "
                                      f"v={v:+.2f} w={w:+.2f} "
                                      f"hdg={state.arduino_heading:+5.0f}° "
                                      f"goal={math.degrees(goal_angle_rad):+5.0f}° "
                                      f"F={prox['front']:.0f} "
                                      f"FL={prox['front_left']:.0f} "
                                      f"FR={prox['front_right']:.0f} "
                                      f"safe={safe_count}",
                                      flush=True)
                                last_print_time = now

                        scan_points = []
            else:
                invalid_count += 1

            now = time.time()
            if now - last_diag_time >= 2.0:
                if packet_count == 0:
                    print("[진단] 패킷 0!", flush=True)
                packet_count  = 0
                invalid_count = 0
                last_diag_time = now

            if now - last_cmd_time > KEEPALIVE_INTERVAL:
                cmd = f"{state.prev_v:.2f} {state.prev_w:.2f}\n"
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
이 코드 분석해줘 

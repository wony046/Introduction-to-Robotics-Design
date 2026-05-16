"""
RPLIDAR C1 장애물 회피 - Clearance Balancing 버전 (진동 제거)

[변경 핵심 - 진동의 근본 원인 해결]
  기존 문제:
    1) "가장 가까운 한 점"에서만 멀어지려 함 → 통로 양쪽 벽 사이 갈지(之)자
    2) avoidance_w_sign 메모리가 다음 장애물에도 끌려감 → 과조향
    3) 옆으로 빠진 장애물(이미 회피 완료)도 계속 조향 명령 생성

  새 로직:
    A) 정면 콘(좁은 영역) 안의 점만 조향에 사용
    B) 좌/우 클리어런스 차이로 P 제어 (Clearance Balancing)
       → 통로 중앙으로 자연 정렬, 진동 구조적으로 억제
    C) 방향 메모리는 stop zone에서만 유지 (danger zone은 매 프레임 재계산)
    D) 데드밴드로 미세 진동 차단

[안전 기능 - 기존 유지]
  - 메인 루프 try/except + finally 정지
  - Keepalive 0.3s
  - 라이다 재동기화 (파싱 실패 100회 누적)
  - decompose_signed 좌표 통일 (x=정면, y=좌측 양수)
  - Print 스로틀링 2Hz
  - start_lidar 예외 처리

포트: 라이다 /dev/ttyUSB0 / 아두이노 /dev/ttyAMA3
"""

import serial
import time
import math
import traceback

# ── 포트 ─────────────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

# ── 라이다 보정 ───────────────────────────────────────────────────────────────
LIDAR_OFFSET    = 20    # mm
LIDAR_MIN_VALID = 100   # mm: 이 미만은 라이다 오류로 간주 → 무시

# ── 로봇 파라미터 ─────────────────────────────────────────────────────────────
ROBOT_HALF_WIDTH  = 110  # mm
SAFETY_MARGIN     = 30   # mm: threshold = 140mm

# ── 위험구역 ──────────────────────────────────────────────────────────────────
DETECTION_RANGE  = 1500
FORWARD_RANGE    = 550

# ── 속도 ──────────────────────────────────────────────────────────────────────
FORWARD_SPEED    = 0.35
MIN_SPEED        = 0.07
SLOW_START_DIST  = 400
STOP_FWD_RANGE   = 180
W_GAIN           = 0.7
MAX_W            = 0.65
W_MIN_DANGER     = 0.45    # stop zone 전용 (정지 상태 비상 회전)
W_MIN_MOVING     = 0.10
W_SMOOTH         = 0.75

# ── [신규] Clearance Balancing 파라미터 ──────────────────────────────────────
STEERING_CONE_HALF = 100   # mm: 이 폭 안의 정면 장애물만 조향 대상
STEERING_CONE_PAD  = 60    # mm: 콘 + 약간 여유 (가장자리 점도 보기 위해)
CLEARANCE_DEADBAND = 30    # mm: 좌우 차이가 이 안이면 직진 (미세 진동 제거)
CENTERING_GAIN     = 0.004 # 1/mm: clearance 차이 → w 변환 (P 게인)
EMPTY_CLEARANCE    = STEERING_CONE_HALF + STEERING_CONE_PAD  # 점 없을 때 가정값

# ── 스캔 ──────────────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE  = 70
SEND_INTERVAL    = 0.1
LIDAR_WATCHDOG_TIMEOUT = 0.5
NO_DANGER_RESET        = 20

STOP_FRONT_DEADBAND = 15

# ── 안전 기능 파라미터 ───────────────────────────────────────────────────────
KEEPALIVE_INTERVAL  = 0.30
PRINT_INTERVAL      = 0.5
RESYNC_THRESHOLD    = 100
DIAG_INTERVAL       = 2.0
LIDAR_SYNC_TIMEOUT  = 3.0

# ── 전역 상태 ─────────────────────────────────────────────────────────────────
arduino_heading_deg = 0.0
arduino_buf         = ""
avoidance_w_sign    = 0.0   # 로그/디버그용
stop_zone_w_sign    = 0.0   # stop zone 방향 메모리 (정면 노이즈 보호)
no_danger_count     = 0
prev_w              = 0.0
prev_v              = 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티 & 수학
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return angle - 360 if angle > 180 else angle

def is_in_front(angle_norm):
    return -SCAN_HALF_ANGLE <= angle_norm <= SCAN_HALF_ANGLE


def decompose_signed(angle_norm_deg, distance_mm):
    """반환: (x = 정면 mm, y = 좌측 양수 mm)

    y > 0 → 장애물이 왼쪽   → 우회전(-w)으로 피해야 함
    y < 0 → 장애물이 오른쪽 → 좌회전(+w)으로 피해야 함
    """
    rad = math.radians(angle_norm_deg)
    x =  distance_mm * math.cos(rad)
    y = -distance_mm * math.sin(rad)
    return x, y


def parse_packet(data):
    if len(data) != 5: return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    distance_q2 = data[3] | (data[4] << 8)
    return (angle_q6 / 64.0), (distance_q2 / 4.0)


def read_arduino(arduino):
    global arduino_heading_deg, arduino_buf
    if arduino.in_waiting <= 0:
        return
    try:
        arduino_buf += arduino.read(arduino.in_waiting).decode('utf-8', errors='ignore')
        while '\n' in arduino_buf:
            line, arduino_buf = arduino_buf.split('\n', 1)
            line = line.strip()
            if line.startswith('H:'):
                arduino_heading_deg = float(line[2:])
    except Exception:
        pass


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 라이다 재동기화 & 안전한 시작
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_lidar_sync(lidar, verbose=True):
    if verbose:
        print("[라이다] 재동기화 시도...", flush=True)
    deadline = time.time() + LIDAR_SYNC_TIMEOUT
    while time.time() < deadline:
        b = lidar.read(1)
        if len(b) == 0:
            continue
        if (b[0] & 0x01) == ((b[0] >> 1) & 0x01):
            continue
        b2 = lidar.read(1)
        if len(b2) == 0:
            continue
        if (b2[0] & 0x01) != 1:
            continue
        rest = lidar.read(3)
        if len(rest) != 3:
            continue
        result = parse_packet(b + b2 + rest)
        if result is None:
            continue
        angle, distance = result
        if 0 <= angle <= 360 and 0 <= distance <= 10000:
            if verbose:
                print(f"[라이다] 동기화 OK (각도={angle:.1f}°, 거리={distance:.0f}mm)",
                      flush=True)
            return True
    if verbose:
        print("[라이다] ✗ 재동기화 실패", flush=True)
    return False


def start_lidar(lidar):
    print("[라이다] 시작 중...", flush=True)
    try:
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        try:
            lidar.dtr = False
        except AttributeError:
            pass
        time.sleep(0.5)
        lidar.reset_input_buffer()
        lidar.write(bytes([0xA5, 0x20]))
        time.sleep(0.5)
        descriptor = lidar.read(7)
        print(f"[라이다] descriptor: {descriptor.hex()}", flush=True)
        return True
    except serial.SerialException as e:
        print(f"[라이다] ✗ 시작 실패: {e}", flush=True)
        return False
    except Exception as e:
        print(f"[라이다] ✗ 예외: {e}", flush=True)
        return False


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# v/w 명령 계산 — Clearance Balancing
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_command(scan_points, heading_deg):
    """정면 스캔 + 헤딩 → (v, w) 반환

    핵심: 좌/우 클리어런스 차이로 P 제어 (중앙 정렬)
    """
    global avoidance_w_sign, stop_zone_w_sign, no_danger_count

    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN
    cone_outer = STEERING_CONE_HALF + STEERING_CONE_PAD

    # ── 1. 포인트 분류 ───────────────────────────────────────────────────────
    danger_points = []   # 감속 판단용 (정면 threshold 안 모든 점)
    cone_left     = []   # 정면 콘 안 — 왼쪽 (y > 0)
    cone_right    = []   # 정면 콘 안 — 오른쪽 (y < 0)

    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE:
            continue
        x, y = decompose_signed(angle_norm, dist)
        if x <= 0 or x > FORWARD_RANGE:
            continue
        horiz = abs(y)

        # 감속 판단: threshold 이내인 모든 정면 점
        if horiz < threshold:
            danger_points.append((angle_norm, dist, horiz, x, y))

        # 조향 판단: 좁은 콘 안의 점 (좌/우 분리)
        if horiz < cone_outer:
            if y > 0:
                cone_left.append((x, y, horiz))
            elif y < 0:
                cone_right.append((x, y, horiz))

    # ── 2. 장애물 없음 → 직진 ────────────────────────────────────────────────
    if not danger_points and not cone_left and not cone_right:
        no_danger_count += 1
        if no_danger_count >= NO_DANGER_RESET:
            avoidance_w_sign = 0.0
            stop_zone_w_sign = 0.0
        return FORWARD_SPEED, 0.0
    no_danger_count = 0

    # ── 3. 선속도 결정 ────────────────────────────────────────────────────────
    stop_points = [p for p in danger_points
                   if p[3] <= STOP_FWD_RANGE and p[2] < ROBOT_HALF_WIDTH]
    frontal     = [p for p in danger_points if p[2] <= ROBOT_HALF_WIDTH + 10]
    n_fwd_ref   = min((p[3] for p in frontal), default=SLOW_START_DIST + 1)

    if stop_points:
        v = 0.0
    elif n_fwd_ref >= SLOW_START_DIST:
        v = FORWARD_SPEED
    else:
        ratio = (n_fwd_ref - STOP_FWD_RANGE) / (SLOW_START_DIST - STOP_FWD_RANGE)
        v = max(FORWARD_SPEED * ratio, MIN_SPEED)

    # ── 4. Stop zone: 비상 회전 (방향 메모리 사용) ───────────────────────────
    if stop_points:
        nearest = min(stop_points, key=lambda p: p[2])
        stop_y, stop_angle = nearest[4], nearest[0]

        if abs(stop_angle) < STOP_FRONT_DEADBAND and stop_zone_w_sign != 0.0:
            sign = stop_zone_w_sign
            print(f"  [정지] 정면노이즈 → 기존방향 유지({'좌' if sign>0 else '우'})")
        else:
            if   stop_y > 0: sign = -1.0   # 왼쪽 장애물 → 우회전
            elif stop_y < 0: sign =  1.0   # 오른쪽 장애물 → 좌회전
            else:            sign =  1.0 if heading_deg <= 0 else -1.0
            stop_zone_w_sign = sign
            print(f"  [정지] y={stop_y:+.0f}mm 각도={stop_angle:+.1f}° → "
                  f"{'좌' if sign>0 else '우'}회전 즉결")

        avoidance_w_sign = sign
        w_mag = max(W_GAIN * (threshold - nearest[2]) / threshold, W_MIN_DANGER)
        w_mag = min(w_mag, MAX_W)
        w = sign * w_mag
        print(f"  [명령] v={v:.2f} w={w:+.2f}")
        return v, w

    # ── 5. Danger zone: Clearance Balancing (방향 메모리 X) ──────────────────
    stop_zone_w_sign = 0.0

    # 좌/우 콘 안 가장 가까운 수평거리 (점 없으면 충분히 멀다고 가정)
    left_clear  = min((c[2] for c in cone_left),  default=EMPTY_CLEARANCE)
    right_clear = min((c[2] for c in cone_right), default=EMPTY_CLEARANCE)

    # diff > 0 → 오른쪽이 더 비어있음 → 오른쪽으로(-w)
    # diff < 0 → 왼쪽이 더 비어있음 → 왼쪽으로(+w)
    diff = right_clear - left_clear

    print(f"  [클리어런스] L={left_clear:.0f}  R={right_clear:.0f}  "
          f"diff={diff:+.0f}mm  콘:L{len(cone_left)}/R{len(cone_right)}점")

    if abs(diff) < CLEARANCE_DEADBAND:
        # 중앙 정렬 OK → 직진
        avoidance_w_sign = 0.0
        w = 0.0
        print(f"  [중앙정렬] OK → 직진")
    else:
        sign = -1.0 if diff > 0 else 1.0   # diff > 0 → 오른쪽(-w)
        w_mag = min(CENTERING_GAIN * abs(diff), MAX_W)
        w_mag = max(w_mag, W_MIN_MOVING)
        w = sign * w_mag
        avoidance_w_sign = sign
        print(f"  [중앙정렬] {'좌' if sign>0 else '우'}회전 w={w:+.2f}")

    print(f"  [명령] v={v:.2f} w={w:+.2f}")
    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    global prev_w, prev_v

    print("=" * 60)
    print("RPLIDAR 장애물 회피 (Clearance Balancing - 진동 제거 버전)")
    print("=" * 60)
    print(f"  알고리즘    : 좌/우 클리어런스 차이로 P 제어 (중앙 정렬)")
    print(f"  조향 콘     : ±{STEERING_CONE_HALF}mm (+ pad {STEERING_CONE_PAD}mm)")
    print(f"  데드밴드    : ±{CLEARANCE_DEADBAND}mm")
    print(f"  P 게인      : {CENTERING_GAIN} (1/mm)")
    print(f"  좌표통일    : decompose_signed (y > 0 = 좌측)")
    print(f"  Keepalive   : {KEEPALIVE_INTERVAL}s")
    print(f"  재동기화    : 파싱실패 {RESYNC_THRESHOLD}회 연속 시")
    print("=" * 60)

    lidar = None
    arduino = None
    try:
        lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
        time.sleep(0.5)
    except Exception as e:
        print(f"[치명] 라이다 포트 열기 실패: {e}")
        return

    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=0.1)
        time.sleep(0.5)
    except Exception as e:
        print(f"[치명] 아두이노 포트 열기 실패: {e}")
        if lidar: lidar.close()
        return

    arduino.write(b"0.00 0.00\n")
    time.sleep(0.3)

    if not start_lidar(lidar):
        arduino.write(b"0.00 0.00\n")
        lidar.close()
        arduino.close()
        return

    if not find_lidar_sync(lidar):
        arduino.write(b"0.00 0.00\n")
        try: lidar.write(bytes([0xA5, 0x25]))
        except: pass
        lidar.close()
        arduino.close()
        return

    print("\n주행 시작!\n", flush=True)

    scan_points     = []
    last_send       = time.time()
    last_scan_time  = time.time()
    last_cmd_time   = time.time()
    last_print_time = 0.0
    last_diag_time  = time.time()
    last_cmd_str    = ""
    invalid_count   = 0
    packet_count    = 0

    try:
        while True:
            read_arduino(arduino)
            raw = lidar.read(5)

            now = time.time()

            # ── Watchdog ─────────────────────────────────────────────────────
            if now - last_scan_time > LIDAR_WATCHDOG_TIMEOUT:
                arduino.write(b"0.00 0.00\n")
                last_cmd_time = now
                print(f"[경고] 라이다 스캔 없음 ({LIDAR_WATCHDOG_TIMEOUT}s) → 비상 정지")
                last_scan_time = now

            # ── 패킷 수신 처리 ───────────────────────────────────────────────
            if len(raw) < 5:
                invalid_count += 1
            else:
                result = parse_packet(raw)
                if result is None:
                    invalid_count += 1
                    if invalid_count >= RESYNC_THRESHOLD:
                        print(f"[라이다] 파싱실패 {invalid_count}회 → 재동기화", flush=True)
                        lidar.reset_input_buffer()
                        if find_lidar_sync(lidar, verbose=False):
                            print("[라이다] 재동기화 성공", flush=True)
                        invalid_count = 0
                        scan_points = []
                else:
                    invalid_count = 0
                    packet_count += 1
                    angle_raw, distance = result
                    s_flag = raw[0] & 0x01

                    if s_flag == 1 and scan_points:
                        last_scan_time = time.time()
                        front_points = [
                            (a, d) for a, d in scan_points
                            if is_in_front(a) and d > 0
                        ]
                        now = time.time()
                        if now - last_send >= SEND_INTERVAL:
                            v, w = find_vw_command(front_points, arduino_heading_deg)

                            # ── 스무딩: 부호 반전 시에도 부드럽게 ───────────────
                            # (기존: 부호 바뀌면 prev_w=0 리셋 → 새 명령 그대로 통과)
                            # (개선: 항상 EMA 적용 → 부호 반전 시 더 완만)
                            w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
                            prev_w = w
                            prev_v = v

                            cmd = f"{v:.2f} {w:.2f}\n"
                            arduino.write(cmd.encode())
                            last_cmd_time = now

                            if cmd != last_cmd_str:
                                if now - last_print_time >= PRINT_INTERVAL:
                                    print(f"[전송] v={v:.2f}  w={w:+.2f}  "
                                          f"헤딩={arduino_heading_deg:+.1f}°", flush=True)
                                    last_print_time = now
                                last_cmd_str = cmd
                            last_send = now
                        scan_points = []

                    scan_points.append((
                        normalize_angle(angle_raw),
                        distance + LIDAR_OFFSET if distance > 0 else 0
                    ))

            # ── Keepalive ────────────────────────────────────────────────────
            now = time.time()
            if now - last_cmd_time > KEEPALIVE_INTERVAL:
                cmd = f"{prev_v:.2f} {prev_w:.2f}\n"
                arduino.write(cmd.encode())
                last_cmd_time = now

            # ── Diagnostic ───────────────────────────────────────────────────
            if now - last_diag_time >= DIAG_INTERVAL:
                if packet_count == 0:
                    print("[진단] 패킷 0 — 라이다 연결 의심", flush=True)
                packet_count = 0
                last_diag_time = now

    except KeyboardInterrupt:
        print("\n[종료] Ctrl-C 수신", flush=True)
    except serial.SerialException as e:
        print(f"\n[치명] 시리얼 예외: {e}", flush=True)
        traceback.print_exc()
    except Exception as e:
        print(f"\n[치명] 예상치 못한 예외: {e}", flush=True)
        traceback.print_exc()
    finally:
        print("[정리] 정지 신호 전송 및 포트 닫기", flush=True)
        if arduino is not None:
            try:
                arduino.write(b"0.00 0.00\n")
                time.sleep(0.1)
                arduino.write(b"0.00 0.00\n")
            except Exception:
                pass
            try: arduino.close()
            except Exception: pass
        if lidar is not None:
            try:
                lidar.write(bytes([0xA5, 0x25]))
                time.sleep(0.1)
            except Exception:
                pass
            try: lidar.close()
            except Exception: pass
        print("[종료] 완료", flush=True)


if __name__ == "__main__":
    main()

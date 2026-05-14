"""
dwa_drive.py  (버그 수정판 + 맵 메모리)
=======================================
[버그 수정]
  BUG-1  : bias 항이 빈 공간에서도 발동 → 진동 유발
           → 장애물 근접 시에만, 거리에 비례하여 작용
  BUG-2  : EMERGENCY_DIST 계산 오류 (DWA보다 비상정지가 먼저 발동)
           → EMERGENCY_DIST 독립 설정
  BUG-3  : last_avoid_sign 갱신 임계 5deg/s → 노이즈에 부호 뒤집힘
           → 20deg/s + N회 연속 같은 방향 확인 후 갱신
  BUG-4  : DWA 샘플에 v=0 없음 → 제자리 회전 평가 불가
           → 0 ~ V_MAX 균등 샘플
  BUG-5  : 후진 중 후방 장애물 미확인 → rear_emergency_dist() 추가
  BUG-6  : heading 수식 epsilon 위치 오류 → 분모 전체에 적용
  BUG-7  : 충돌 안전 마진 없음 (실제 표면에서 충돌 판정)
           → COLLISION_MARGIN(30mm) 추가
  BUG-8  : alternating bias 반응성 부족 (CONFIRM=3)
           → CONFIRM=2로 완화
  BUG-9  : predict_trajectory가 Euler 적분 → transform_memory와 모델 불일치
           → arc 모션 닫힘공식으로 통일 (~11mm 오차 → 0)
  BUG-10 : arduino.write 예외 미처리 → 통신 끊김 시 크래시
           → try/except 보호

[신규 기능: 맵 메모리]
  - 최근 MEMORY_DURATION 초 동안 장애물 기억
  - 로봇 이동에 따라 메모리 좌표를 arc 모션으로 정확하게 변환
  - 40mm 이내 중복 제거 (최신 우선)
  - DWA 평가에 메모리 사용 (비상정지는 현재 스캔만)
"""

import serial
import time
import math
from collections import deque


# ============================================================
# 1. 통신 포트
# ============================================================
LIDAR_PORT   = "/dev/ttyUSB0"
ARDUINO_PORT = "/dev/ttyAMA3"
arduino   = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
lidar_ser = serial.Serial(LIDAR_PORT,  460800, timeout=1)


# ============================================================
# 2. 로봇 기구 / 센서 사양
# ============================================================
LIDAR_TO_EDGE    = 110    # mm — 라이다 센서 ~ 차체 외곽
ROBOT_HALF_WIDTH = 110    # mm — 차체 외곽 반폭
COLLISION_MARGIN = 30     # mm — 충돌 판정 안전 여유 (BUG-7 수정)
COLLISION_RADIUS = ROBOT_HALF_WIDTH + COLLISION_MARGIN  # = 140mm


# ============================================================
# 3. DWA 파라미터
# ============================================================
V_MAX        = 25.0        # cm/s
V_REVERSE    = -4.8        # cm/s
V_SAMPLES    = 7
W_MAX_DPS    = 90.0        # deg/s
W_SAMPLES    = 15
DT_PREDICT   = 0.6
DT_STEP      = 0.1

EMERGENCY_DIST   = 120    # mm
SAFE_DIST        = 250    # mm
EVAL_FORWARD     = 350    # mm

WG_HEADING       = 1.2
WG_CLEARANCE     = 2.0
WG_VELOCITY      = 0.8
WG_BIAS          = 1.5
WG_INFLATION     = 1.5    # ★ 신규: 장애물 근접 페널티 (Tier-1 #2)


# ============================================================
# 3-B. Inflation / Wall-follow / Gap-follow 파라미터 (★ 신규)
# ============================================================
INFLATION_RADIUS = 250    # mm — 이 거리 이내는 inflation 페널티
                           # COLLISION_RADIUS(140) ~ INFLATION_RADIUS(250) 사이에서 선형 감쇠

WALL_LEFT_MIN    = 60     # 좌측 벽 감지 섹터 (deg)
WALL_LEFT_MAX    = 120
WALL_RIGHT_MIN   = -120
WALL_RIGHT_MAX   = -60
WALL_GAIN        = 0.05   # mm 차이 → deg 보정 (100mm 차 → 5° 보정)
WALL_MAX_DEG     = 15.0   # 보정각 한계

GAP_MIN_CLEAR    = 200    # mm — 이 거리 이상이면 "열린" 방향으로 판정
GAP_MIN_WIDTH    = 15     # deg — 최소 gap 폭 (이 미만은 무시)
WALL_BLEND_RATIO = 0.3    # gap 발견 시 wall 보정 가중치


# ============================================================
# 4. 상태 머신 파라미터
# ============================================================
STUCK_WINDOW     = 25
STUCK_THRESHOLD  = 18
REVERSE_DURATION = 1.5

SIGN_UPDATE_W_MIN   = 20.0
SIGN_UPDATE_CONFIRM = 2     # BUG-8 수정: 3→2 (alternation 반응성 향상)


# ============================================================
# 5. 맵 메모리 파라미터  (★ 신규)
# ============================================================
MEMORY_DURATION    = 1.5    # s — 이 시간 이상 지난 항목 제거
MEMORY_DEDUP_DIST  = 40     # mm — 이 거리 이내는 중복으로 간주
MEMORY_X_MIN       = -200   # mm — 로봇 후방 한계
MEMORY_X_MAX       = EVAL_FORWARD + 200
MEMORY_Y_MAX       = ROBOT_HALF_WIDTH * 5


# ============================================================
# 6. 전역 상태
# ============================================================
last_avoid_sign   = 0
mode_history      = deque(maxlen=STUCK_WINDOW)
reverse_until     = 0.0
post_reverse_sign = 0
_sign_buf         = deque(maxlen=SIGN_UPDATE_CONFIRM)

# 맵 메모리 상태
memory_obstacles = []    # [(x_mm, y_mm, dist_mm, timestamp), ...]
last_cmd_v       = 0.0   # 마지막으로 보낸 v (cm/s)  — 메모리 변환에 사용
last_cmd_w       = 0.0   # 마지막으로 보낸 w (deg/s)
last_scan_time   = None  # 마지막 스캔 처리 시각


# ============================================================
# 7. LiDAR 데이터 처리
# ============================================================
def scan_to_obstacles(scan_data):
    """스캔 → 평가 영역 장애물 리스트 [(x, y, dist), ...]"""
    bin_pts = {}
    for raw_angle, distance in scan_data:
        if distance <= 0:
            continue
        angle = raw_angle if raw_angle <= 180 else raw_angle - 360
        rad = math.radians(angle)
        x = distance * math.cos(rad)
        y = distance * math.sin(rad)
        if -50 < x <= EVAL_FORWARD and abs(y) <= ROBOT_HALF_WIDTH * 4:
            ba = round(angle / 10) * 10
            bin_pts.setdefault(ba, []).append((x, y, distance))

    obstacles = []
    for pts in bin_pts.values():
        if len(pts) >= 2:
            pts.sort(key=lambda p: p[2])
            obstacles.append(pts[0])
    return obstacles


def _sector_min_dist(scan_data, angle_min, angle_max):
    pts = []
    for raw_angle, distance in scan_data:
        if distance <= 0:
            continue
        angle = raw_angle if raw_angle <= 180 else raw_angle - 360
        if angle_min <= angle <= angle_max:
            pts.append(distance)
    pts.sort()
    return pts[1] if len(pts) >= 2 else 9999


def front_emergency_dist(scan_data):
    return _sector_min_dist(scan_data, -15, 15)


def rear_emergency_dist(scan_data):
    pts = []
    for raw_angle, distance in scan_data:
        if distance <= 0:
            continue
        angle = raw_angle if raw_angle <= 180 else raw_angle - 360
        if abs(angle) >= 165:
            pts.append(distance)
    pts.sort()
    return pts[1] if len(pts) >= 2 else 9999


# ============================================================
# 8. 맵 메모리 — 변환 / 갱신 / 조회   (★ 신규)
# ============================================================
def transform_memory(dt):
    """
    직전 (v, w) 명령으로 dt 초 동안 이동한 로봇 변위를 계산하고,
    메모리 속 모든 장애물 좌표를 새 로봇 기준 좌표계로 변환.

    Arc 모션 공식 (로봇 시작점=원점, 헤딩=+x):
        w == 0:  dx = v*dt,            dy = 0
        else  :  R  = v/w,  dθ = w*dt
                 dx = R*sin(dθ),       dy = R*(1-cos(dθ))

    이후 각 장애물 (ox, oy)에 대해:
        translate : tx = ox - dx,  ty = oy - dy
        rotate(-dθ): nx = cos(dθ)*tx + sin(dθ)*ty
                     ny = -sin(dθ)*tx + cos(dθ)*ty
    """
    global memory_obstacles
    if dt <= 0 or not memory_obstacles:
        return

    v_mm  = last_cmd_v * 10.0
    w_rad = math.radians(last_cmd_w)
    dtheta = w_rad * dt

    if abs(w_rad) < 1e-6:
        dx, dy = v_mm * dt, 0.0
    else:
        R = v_mm / w_rad
        dx = R * math.sin(dtheta)
        dy = R * (1.0 - math.cos(dtheta))

    c, s = math.cos(dtheta), math.sin(dtheta)

    new_mem = []
    for ox, oy, _od, ts in memory_obstacles:
        tx = ox - dx
        ty = oy - dy
        nx = c * tx + s * ty
        ny = -s * tx + c * ty
        new_mem.append((nx, ny, math.hypot(nx, ny), ts))
    memory_obstacles = new_mem


def update_memory(current_obstacles, now):
    """
    현재 스캔 추가 → 중복 제거 (최신 우선) → 만료/범위 외 제거.
    """
    global memory_obstacles

    # 1) 현재 스캔 추가 (모두 timestamp = now)
    for ox, oy, od in current_obstacles:
        memory_obstacles.append((ox, oy, od, now))

    # 2) 최신 우선 정렬 → 중복 제거
    memory_obstacles.sort(key=lambda o: -o[3])
    deduped = []
    for obs in memory_obstacles:
        is_dup = False
        for kept in deduped:
            if math.hypot(obs[0] - kept[0], obs[1] - kept[1]) < MEMORY_DEDUP_DIST:
                is_dup = True
                break
        if not is_dup:
            deduped.append(obs)

    # 3) 만료 / 범위 외 항목 제거
    memory_obstacles = [
        o for o in deduped
        if (now - o[3] <= MEMORY_DURATION
            and MEMORY_X_MIN <= o[0] <= MEMORY_X_MAX
            and abs(o[1]) <= MEMORY_Y_MAX)
    ]


def memory_for_dwa():
    """메모리 → (x, y, dist) 튜플 리스트 (DWA 입력 형식)."""
    return [(o[0], o[1], o[2]) for o in memory_obstacles]


# ============================================================
# 9. 궤적 예측 & 충돌
# ============================================================
def predict_trajectory(v_cms, w_dps):
    """
    BUG-9 수정: Euler 적분 → arc 모션 닫힘공식.
    transform_memory()와 동일 모델 사용 → 메모리·궤적 정합성 향상.

    위치 (로봇 원점, 헤딩 +x 출발) at time t:
        w == 0:  x(t) = v*t,           y(t) = 0
        else  :  x(t) = (v/w)*sin(w*t),  y(t) = (v/w)*(1 - cos(w*t))
    """
    pts = []
    v_mm  = v_cms * 10.0
    w_rad = math.radians(w_dps)

    t = DT_STEP
    if abs(w_rad) < 1e-6:
        while t <= DT_PREDICT + 1e-9:
            pts.append((v_mm * t, 0.0))
            t += DT_STEP
    else:
        R = v_mm / w_rad
        while t <= DT_PREDICT + 1e-9:
            th = w_rad * t
            pts.append((R * math.sin(th), R * (1.0 - math.cos(th))))
            t += DT_STEP
    return pts


def trajectory_clearance(traj, obstacles):
    if not obstacles:
        return EVAL_FORWARD
    min_dist = 9999.0
    for (px, py) in traj:
        for (ox, oy, _) in obstacles:
            d = math.hypot(px - ox, py - oy)
            if d < min_dist:
                min_dist = d
            if d < COLLISION_RADIUS:        # BUG-7: 안전 마진 포함
                return -1
    return min_dist


# ============================================================
# 10. 비용 함수
# ============================================================
def evaluate(v, w, obstacles, bias_sign):
    traj = predict_trajectory(v, w)
    clr  = trajectory_clearance(traj, obstacles)
    if clr < 0:
        return None

    last_x, last_y = traj[-1]
    if last_x <= 0:
        heading = 0.0
    else:
        heading = last_x / (math.hypot(last_x, last_y) + 1e-9)   # BUG-6 수정

    clearance = min(clr, EVAL_FORWARD) / EVAL_FORWARD
    velocity  = max(0.0, v) / V_MAX

    # BUG-1 수정: 장애물 있을 때만, 근접할수록 강하게
    nearest = min((o[2] for o in obstacles), default=9999)
    if bias_sign != 0 and w * bias_sign > 0 and nearest < EVAL_FORWARD:
        proximity = 1.0 - (nearest / EVAL_FORWARD)
        bias = (abs(w) / W_MAX_DPS) * proximity
    else:
        bias = 0.0

    return (WG_HEADING   * heading +
            WG_CLEARANCE * clearance +
            WG_VELOCITY  * velocity +
            WG_BIAS      * bias)


# ============================================================
# 11. last_avoid_sign 안정 갱신 (BUG-3)
# ============================================================
def try_update_sign(best_w):
    global last_avoid_sign, _sign_buf
    if abs(best_w) < SIGN_UPDATE_W_MIN:
        _sign_buf.clear()
        return
    candidate = 1 if best_w > 0 else -1
    _sign_buf.append(candidate)
    if (len(_sign_buf) == SIGN_UPDATE_CONFIRM
            and all(s == candidate for s in _sign_buf)):
        last_avoid_sign = candidate


# ============================================================
# 12. 메인 의사결정
# ============================================================
def decide(scan_data, obstacles):
    """
    scan_data  : 비상정지 판정용 (raw)
    obstacles  : DWA 입력 (메모리 병합된 리스트)
    """
    global reverse_until, post_reverse_sign

    now = time.time()

    # A. 비상 정지 (raw 스캔)
    if front_emergency_dist(scan_data) < EMERGENCY_DIST:
        return 0.0, 0.0, 'EMERGENCY'

    # B. 후진 중
    if now < reverse_until:
        if rear_emergency_dist(scan_data) < EMERGENCY_DIST:
            reverse_until = 0.0
            return 0.0, 0.0, 'EMERGENCY'
        return V_REVERSE, 0.0, 'REVERSE'

    # C. 갇힘 → 후진
    if mode_history.count('AVOID') >= STUCK_THRESHOLD:
        reverse_until     = now + REVERSE_DURATION
        post_reverse_sign = -last_avoid_sign if last_avoid_sign != 0 else 1
        mode_history.clear()
        _sign_buf.clear()
        print(f"[ESCAPE] 갇힘 감지 → 후진 {REVERSE_DURATION}s  "
              f"다음 부호={post_reverse_sign:+d}")
        return V_REVERSE, 0.0, 'REVERSE'

    # D. bias 부호
    if post_reverse_sign != 0:
        bias_sign = post_reverse_sign
    elif last_avoid_sign != 0:
        bias_sign = -last_avoid_sign
    else:
        bias_sign = 0

    # E. DWA (BUG-4: v=0 포함, 0~V_MAX 균등)
    best_v, best_w, best_score = 0.0, 0.0, -1e9
    w_step = (2.0 * W_MAX_DPS) / (W_SAMPLES - 1)
    for i in range(V_SAMPLES):
        v = V_MAX * i / (V_SAMPLES - 1)
        for j in range(W_SAMPLES):
            w = -W_MAX_DPS + j * w_step
            s = evaluate(v, w, obstacles, bias_sign)
            if s is not None and s > best_score:
                best_score, best_v, best_w = s, v, w

    # F. 전부 충돌 → 제자리 회전
    if best_score < -1e8:
        spin_w = float(bias_sign or 1) * 60.0
        return 0.0, spin_w, 'AVOID'

    # G. 모드 분류
    nearest = min((o[2] for o in obstacles), default=9999)
    mode = 'MAX' if nearest > SAFE_DIST and abs(best_w) < 15.0 else 'AVOID'

    # H. 상태 갱신
    if mode == 'AVOID':
        try_update_sign(best_w)
        if post_reverse_sign != 0 and abs(best_w) > SIGN_UPDATE_W_MIN:
            post_reverse_sign = 0

    mode_history.append(mode)
    return best_v, best_w, mode


# ============================================================
# 13. 메인 루프
# ============================================================
def main():
    global last_cmd_v, last_cmd_w, last_scan_time

    print("[INFO] LiDAR 초기화 중...")
    lidar_ser.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20]))
    time.sleep(0.5)
    print("[INFO] DWA + 맵 메모리 자율주행 시작! (정지: Ctrl+C)")
    print(f"[INFO] 비상={EMERGENCY_DIST}mm  안전={SAFE_DIST}mm  "
          f"평가={EVAL_FORWARD}mm  메모리={MEMORY_DURATION}s")

    scan_data     = []
    last_log_time = time.time()

    try:
        while True:
            raw = lidar_ser.read(5)
            if len(raw) != 5:
                continue

            s_flag     = raw[0] & 0x01
            s_inv_flag = (raw[0] & 0x02) >> 1
            if s_inv_flag != (1 - s_flag):
                continue
            if (raw[1] & 0x01) != 1:
                continue

            angle    = ((raw[1] >> 1) | (raw[2] << 7)) / 64.0
            distance = (raw[3] | (raw[4] << 8)) / 4.0

            if s_flag == 1:                            # 한 바퀴 완료
                if len(scan_data) > 50:
                    now = time.time()

                    # ── 1) 직전 명령으로 메모리 좌표 변환 ──
                    if last_scan_time is not None:
                        transform_memory(now - last_scan_time)
                    last_scan_time = now

                    # ── 2) 현재 스캔 → 장애물 → 메모리 병합 ──
                    current_obs = scan_to_obstacles(scan_data)
                    update_memory(current_obs, now)
                    merged_obs = memory_for_dwa()

                    # ── 3) DWA (메모리 사용) ──
                    v, w, mode = decide(scan_data, merged_obs)

                    # BUG-10: Arduino write 예외 보호
                    try:
                        arduino.write(f"{v:.1f},{w:.1f}\n".encode('utf-8'))
                    except (serial.SerialException, OSError) as e:
                        print(f"[WARN] Arduino 통신 오류: {e}")

                    # ── 4) 다음 변환을 위해 명령 저장 ──
                    last_cmd_v, last_cmd_w = v, w

                    # ── 5) 1초마다 종합 로그 ──
                    if now - last_log_time > 1.0:
                        last_log_time = now
                        fed = front_emergency_dist(scan_data)
                        red = rear_emergency_dist(scan_data)
                        if merged_obs:
                            ox, oy, od = min(merged_obs, key=lambda o: o[2])
                            near_str = (f"{od:5.0f}mm@"
                                        f"{math.degrees(math.atan2(oy,ox)):+4.0f}°")
                        else:
                            near_str = "     none     "
                        print(f"[{mode:9s}] "
                              f"→ Arduino  v={v:+6.1f}cm/s  w={w:+6.1f}°/s   "
                              f"| pts={len(scan_data):4d}  "
                              f"front={fed:5.0f}mm  rear={red:5.0f}mm  "
                              f"nearest={near_str}  "
                              f"now={len(current_obs):2d}  mem={len(merged_obs):2d}  "
                              f"bias={last_avoid_sign:+d}")
                scan_data = []

            if distance > 0:
                scan_data.append((angle, distance))

    except KeyboardInterrupt:
        print("\n[INFO] 정지 명령 수신.")
        try:
            arduino.write("STOP\n".encode('utf-8'))
        except Exception:
            pass
        try:
            lidar_ser.write(bytes([0xA5, 0x25]))
        except Exception:
            pass
        time.sleep(0.1)
        arduino.close()
        lidar_ser.close()


if __name__ == '__main__':
    main()

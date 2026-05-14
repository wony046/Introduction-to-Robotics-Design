"""
dwa_drive.py (DWA + 맵 메모리 + RANSAC 벽 추종 + 틈새 인식 통합판)
===================================================================
[기능 요약]
1. Map Memory: 이동량을 계산해 사각지대 장애물을 기억
2. Inflation: 장애물 근접 시 안전거리 확보 (페널티)
3. Gap Detection: 장애물 사이의 뚫린 공간 탐색
4. RANSAC Wall-Follow: 노이즈를 무시하고 뚜렷한 선(벽)을 찾아 평행 주행
"""

import serial
import time
import math
from collections import deque
import numpy as np
from sklearn import linear_model

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
LIDAR_TO_EDGE    = 110    # mm 
ROBOT_HALF_WIDTH = 110    # mm 
COLLISION_MARGIN = 40     # mm (안전성 강화를 위해 40mm로 상향)
COLLISION_RADIUS = ROBOT_HALF_WIDTH + COLLISION_MARGIN  # 150mm

# ============================================================
# 3. DWA & 자율주행 파라미터
# ============================================================
V_MAX        = 25.0       # cm/s
V_REVERSE    = -4.8       # cm/s
V_SAMPLES    = 7
W_MAX_DPS    = 90.0       # deg/s
W_SAMPLES    = 15
DT_PREDICT   = 0.6
DT_STEP      = 0.1

EMERGENCY_DIST   = 120    # mm
SAFE_DIST        = 250    # mm
EVAL_FORWARD     = 350    # mm

# DWA 가중치
WG_HEADING   = 1.5       # 목표 방향(Gap) 정렬 가중치
WG_CLEARANCE = 2.0       # 충돌 회피
WG_VELOCITY  = 1.0       # 속도 유지
WG_BIAS      = 1.2       # 회피 부호 유지
WG_INFLATION = 1.8       # 장애물 근접 회피 (팽창 페널티)

# ============================================================
# 4. 고급 주행 (Inflation / Gap / RANSAC) 파라미터
# ============================================================
INFLATION_RADIUS = 300   # mm (이 거리 이내는 inflation 페널티)
GAP_MIN_WIDTH    = 300   # mm (최소 통과 가능 틈새 폭)

WALL_TARGET_DIST = 200   # mm (벽과의 목표 거리)
WALL_GAIN        = 0.08  # 벽 오차 → 조향 보정 게인
RANSAC_MAX_TRIALS = 9   # RANSAC 연산량 제한 (지연 방지용)

# ============================================================
# 5. 상태 머신 및 메모리 파라미터
# ============================================================
STUCK_WINDOW     = 25
STUCK_THRESHOLD  = 18
REVERSE_DURATION = 1.5

SIGN_UPDATE_W_MIN   = 20.0
SIGN_UPDATE_CONFIRM = 2     

MEMORY_DURATION    = 1.5    # s
MEMORY_DEDUP_DIST  = 40     # mm
MEMORY_X_MIN       = -200   # mm
MEMORY_X_MAX       = EVAL_FORWARD + 200
MEMORY_Y_MAX       = ROBOT_HALF_WIDTH * 5

# 전역 상태 변수
last_avoid_sign   = 0
mode_history      = deque(maxlen=STUCK_WINDOW)
reverse_until     = 0.0
post_reverse_sign = 0
_sign_buf         = deque(maxlen=SIGN_UPDATE_CONFIRM)

memory_obstacles = []    
last_cmd_v       = 0.0   
last_cmd_w       = 0.0   
last_scan_time   = None  

# ============================================================
# 6. LiDAR & 맵 메모리 유틸
# ============================================================
def scan_to_obstacles(scan_data):
    bin_pts = {}
    for raw_angle, distance in scan_data:
        if distance <= 0: continue
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

def front_emergency_dist(scan_data):
    pts = [d for a, d in scan_data if d > 0 and (a <= 15 or a >= 345)]
    pts.sort()
    return pts[1] if len(pts) >= 2 else 9999

def rear_emergency_dist(scan_data):
    pts = [d for a, d in scan_data if d > 0 and 165 <= a <= 195]
    pts.sort()
    return pts[1] if len(pts) >= 2 else 9999

def transform_memory(dt):
    global memory_obstacles
    if dt <= 0 or not memory_obstacles: return
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
        tx, ty = ox - dx, oy - dy
        nx, ny = c * tx + s * ty, -s * tx + c * ty
        new_mem.append((nx, ny, math.hypot(nx, ny), ts))
    memory_obstacles = new_mem

def update_memory(current_obstacles, now):
    global memory_obstacles
    for ox, oy, od in current_obstacles:
        memory_obstacles.append((ox, oy, od, now))
    
    memory_obstacles.sort(key=lambda o: -o[3])
    deduped = []
    for obs in memory_obstacles:
        if not any(math.hypot(obs[0]-k[0], obs[1]-k[1]) < MEMORY_DEDUP_DIST for k in deduped):
            deduped.append(obs)
            
    memory_obstacles = [o for o in deduped if (now - o[3] <= MEMORY_DURATION 
                        and MEMORY_X_MIN <= o[0] <= MEMORY_X_MAX 
                        and abs(o[1]) <= MEMORY_Y_MAX)]

def memory_for_dwa():
    return [(o[0], o[1], o[2]) for o in memory_obstacles]

# ============================================================
# 7. 공간 분석 (Gap & RANSAC)
# ============================================================
def find_best_gap(obstacles):
    if not obstacles: return 0.0
    polar_obs = sorted([(math.degrees(math.atan2(oy, ox)), od) for ox, oy, od in obstacles])
    
    max_gap_width, best_gap_angle = 0, 0.0
    for i in range(len(polar_obs) - 1):
        a1, d1 = polar_obs[i]
        a2, d2 = polar_obs[i+1]
        gap_dist = math.sqrt(d1**2 + d2**2 - 2*d1*d2*math.cos(math.radians(a2-a1)))
        
        if gap_dist > GAP_MIN_WIDTH and gap_dist > max_gap_width:
            max_gap_width = gap_dist
            best_gap_angle = (a1 + a2) / 2.0
            
    return best_gap_angle if max_gap_width > 0 else 0.0

def extract_line_ransac(points):
    if len(points) < 10: return None, None
    X = np.array([p[0] for p in points]).reshape(-1, 1)
    y = np.array([p[1] for p in points])

    ransac = linear_model.RANSACRegressor(
        min_samples=max(3, int(len(points)*0.2)),
        residual_threshold=30.0, 
        max_trials=RANSAC_MAX_TRIALS # 연산 속도 보호
    )
    try:
        ransac.fit(X, y)
        return ransac.estimator_.coef_[0], ransac.estimator_.intercept_
    except ValueError:
        return None, None

def get_wall_correction(obstacles):
    left_pts, right_pts = [], []
    for ox, oy, _ in obstacles:
        if 0 < ox < 600:
            if 50 < oy < 600: left_pts.append((ox, oy))
            elif -600 < oy < -50: right_pts.append((ox, oy))

    correction_w = 0.0
    slope, intercept = extract_line_ransac(left_pts)
    is_left = True
    
    if slope is None:
        slope, intercept = extract_line_ransac(right_pts)
        is_left = False

    if slope is not None:
        dist_to_wall = abs(intercept) / math.sqrt(slope**2 + 1)
        wall_angle = math.degrees(math.atan(slope))
        dist_err = dist_to_wall - WALL_TARGET_DIST

        if is_left: correction_w = (dist_err * WALL_GAIN) + (wall_angle * 0.6)
        else:       correction_w = -(dist_err * WALL_GAIN) + (wall_angle * 0.6)

    return max(min(correction_w, 20.0), -20.0)

# ============================================================
# 8. 궤적 예측 및 평가
# ============================================================
def predict_trajectory(v_cms, w_dps):
    pts = []
    v_mm, w_rad = v_cms * 10.0, math.radians(w_dps)
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

def evaluate(v, w, obstacles, bias_sign, target_gap_angle):
    traj = predict_trajectory(v, w)
    min_dist_to_obs = 9999.0
    inflation_penalty = 0.0
    
    for (px, py) in traj:
        for (ox, oy, _) in obstacles:
            d = math.hypot(px - ox, py - oy)
            if d < COLLISION_RADIUS:
                return None 
            
            if d < INFLATION_RADIUS:
                inf_val = (INFLATION_RADIUS - d) / (INFLATION_RADIUS - COLLISION_RADIUS)
                inflation_penalty = max(inflation_penalty, inf_val)
                
            if d < min_dist_to_obs:
                min_dist_to_obs = d

    # ==========================================================
    # ★ [버그 수정됨] 제자리 회전(v=0) 시 각도 계산 오류 방지
    # ==========================================================
    last_x, last_y = traj[-1]
    
    if abs(last_x) < 1e-3 and abs(last_y) < 1e-3:
        # v=0 이라서 (0,0)에 머무는 경우: 로봇이 회전한 각도를 그대로 사용
        traj_angle = w * DT_PREDICT
    else:
        # 이동하는 경우: 도착한 위치의 기하학적 각도를 사용
        traj_angle = math.degrees(math.atan2(last_y, last_x))
    # ==========================================================

    angle_diff = abs(traj_angle - target_gap_angle)
    if angle_diff > 180.0:
        angle_diff = 360.0 - angle_diff
    heading_score = 1.0 - (min(angle_diff, 90.0) / 90.0)

    clearance_score = min(min_dist_to_obs, EVAL_FORWARD) / EVAL_FORWARD
    velocity_score  = v / V_MAX
    inflation_score = 1.0 - inflation_penalty
    bias = (abs(w) / W_MAX_DPS) if (bias_sign != 0 and w * bias_sign > 0) else 0.0

    return (WG_HEADING   * heading_score +
            WG_CLEARANCE * clearance_score +
            WG_VELOCITY  * velocity_score +
            WG_BIAS      * bias +
            WG_INFLATION * inflation_score)

def try_update_sign(best_w):
    global last_avoid_sign, _sign_buf
    if abs(best_w) < SIGN_UPDATE_W_MIN:
        _sign_buf.clear()
        return
    candidate = 1 if best_w > 0 else -1
    _sign_buf.append(candidate)
    if len(_sign_buf) == SIGN_UPDATE_CONFIRM and all(s == candidate for s in _sign_buf):
        last_avoid_sign = candidate

# ============================================================
# 9. 의사 결정
# ============================================================
def decide(scan_data, obstacles):
    global reverse_until, post_reverse_sign, mode_history
    now = time.time()

    if front_emergency_dist(scan_data) < EMERGENCY_DIST:
        return 0.0, 0.0, 'EMERGENCY'
    if now < reverse_until:
        if rear_emergency_dist(scan_data) < EMERGENCY_DIST:
            reverse_until = 0.0
            return 0.0, 0.0, 'EMERGENCY'
        return V_REVERSE, 0.0, 'REVERSE'

    if mode_history.count('AVOID') >= STUCK_THRESHOLD:
        reverse_until     = now + REVERSE_DURATION
        post_reverse_sign = -last_avoid_sign if last_avoid_sign != 0 else 1
        mode_history.clear()
        _sign_buf.clear()
        return V_REVERSE, 0.0, 'REVERSE'

    # 공간 분석 정보
    target_gap = find_best_gap(obstacles)
    wall_corr  = get_wall_correction(obstacles)

    best_v, best_w, best_score = 0.0, 0.0, -1e9
    bias_sign = post_reverse_sign if post_reverse_sign != 0 else -last_avoid_sign
    w_step = (2.0 * W_MAX_DPS) / (W_SAMPLES - 1)
    
    for i in range(V_SAMPLES):
        v = V_MAX * i / (V_SAMPLES - 1)
        if 0 < v < 2.0: continue # 미세 전진 방지
        
        for j in range(W_SAMPLES):
            w_sample = -W_MAX_DPS + j * w_step
            # 물리적 조향 한계 초과 방지(Clamp)
            w = max(-W_MAX_DPS, min(W_MAX_DPS, w_sample + (wall_corr * (v / V_MAX))))
            
            s = evaluate(v, w, obstacles, bias_sign, target_gap)
            if s is not None and s > best_score:
                best_score, best_v, best_w = s, v, w

    if best_score < -1e8:
        return 0.0, float(bias_sign or 1) * 60.0, 'SPIN'

    nearest = min((o[2] for o in obstacles), default=9999)
    mode = 'MAX' if nearest > SAFE_DIST and abs(best_w) < 15.0 else 'AVOID'

    if mode == 'AVOID':
        try_update_sign(best_w)
        if post_reverse_sign != 0 and abs(best_w) > SIGN_UPDATE_W_MIN:
            post_reverse_sign = 0

    mode_history.append(mode)
    return best_v, best_w, mode

# ============================================================
# 10. 메인 루프
# ============================================================
def main():
    global last_cmd_v, last_cmd_w, last_scan_time
    print("[INFO] LiDAR 초기화 중...")
    lidar_ser.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20]))
    time.sleep(0.5)
    print("[INFO] 주행 시작 (정지: Ctrl+C)")

    scan_data = []
    last_log_time = time.time()

    try:
        while True:
            raw = lidar_ser.read(5)
            if len(raw) != 5: continue

            s_flag     = raw[0] & 0x01
            s_inv_flag = (raw[0] & 0x02) >> 1
            if s_inv_flag != (1 - s_flag) or (raw[1] & 0x01) != 1: continue

            angle    = ((raw[1] >> 1) | (raw[2] << 7)) / 64.0
            distance = (raw[3] | (raw[4] << 8)) / 4.0

            if s_flag == 1:
                if len(scan_data) > 50:
                    now = time.time()
                    if last_scan_time is not None:
                        transform_memory(now - last_scan_time)
                    last_scan_time = now

                    current_obs = scan_to_obstacles(scan_data)
                    update_memory(current_obs, now)
                    merged_obs = memory_for_dwa()

                    v, w, mode = decide(scan_data, merged_obs)

                    try:
                        arduino.write(f"{v:.1f},{w:.1f}\n".encode('utf-8'))
                    except Exception as e:
                        print(f"[WARN] Arduino 통신 오류: {e}")

                    last_cmd_v, last_cmd_w = v, w

                    if now - last_log_time > 1.0:
                        last_log_time = now
                        fed = front_emergency_dist(scan_data)
                        print(f"[{mode:9s}] v={v:+6.1f} w={w:+6.1f} | mem={len(merged_obs):2d} front={fed:5.0f}mm")
                scan_data = []

            if distance > 0:
                scan_data.append((angle, distance))

    except KeyboardInterrupt:
        print("\n[INFO] 정지 명령 수신.")
        try: arduino.write("STOP\n".encode('utf-8'))
        except: pass
        try: lidar_ser.write(bytes([0xA5, 0x25]))
        except: pass
        time.sleep(0.1)
        arduino.close()
        lidar_ser.close()

if __name__ == '__main__':
    main()

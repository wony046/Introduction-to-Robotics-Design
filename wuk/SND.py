import math
import time
import serial
import numpy as np
import matplotlib.pyplot as plt

# 사용자 환경의 RPLIDAR C1 드라이버 (기존)
from SND_rplidar_serial import RPLidarSerial


# ============================================================
# 설정 (튜닝 포인트는 # TUNE 표기)
# ============================================================

# --- 하드웨어 / 통신 ---
LIDAR_PORT          = "/dev/ttyUSB0"
LIDAR_BAUD          = 460800
ARDUINO_PORT        = '/dev/ttyAMA3'
ARDUINO_BAUD        = 115200

# --- 로봇 기하 ---
ROBOT_RADIUS_M      = 0.15          # TUNE: 실측 로봇 반경 (외곽 포함)

# --- SND 코어 파라미터 ---
USE_ADAPTIVE_Ds     = True          # 기하학적 폭 계산 활성화
Ds_TIGHT_M          = 0.45          # 좁은 통로에서 양쪽 벽을 밀어낼 큰 Ds
Ds_OPEN_M           = 0.18          # 넓은 공간에서 유연하게 빠져나갈 작은 Ds

# --- 속도 ---
V_MAX_MPS           = 0.20          # TUNE: 직진 최대 속도
V_MIN_MPS           = 0.06          # 근접 시 최소 속도
SLOWDOWN_DIST_M     = 0.35          # 이 거리 이하 근접 시 감속

# --- LiDAR 처리 ---
FRONT_FOV_DEG       = 180           # 전방 ±90° 사용
RANGE_MIN_M         = 0.05
RANGE_MAX_M         = 2.50
ANGLE_BIN_DEG       = 2             # 각도 빈 크기 (다운샘플링)

# --- 제어 루프 ---
LOOP_HZ             = 10
DS_UPDATE_HZ        = 5             # ★ Ds 갱신 주기 (5Hz)
MAX_HEADING_CMD_RAD = math.radians(60)

# --- 디버그 / 안전 ---
VERBOSE             = True
DRY_RUN             = True        # 실제 모터 구동
USE_VIS             = True        # 시각화 켜기/끄기


# ============================================================
# 유틸: LiDAR 스캔 다운샘플링
# ============================================================
def downsample_scan(scan_points, bin_deg=ANGLE_BIN_DEG, fov_deg=FRONT_FOV_DEG):
    half_fov = fov_deg / 2.0
    n_bins   = int(fov_deg / bin_deg)
    bins     = np.full(n_bins, np.inf)

    for pt in scan_points:
        _, angle_deg, dist_mm = pt
        a = ((angle_deg + 180.0) % 360.0) - 180.0
        if abs(a) > half_fov: continue
        d_m = dist_mm / 1000.0
        if d_m < RANGE_MIN_M or d_m > RANGE_MAX_M: continue
        idx = int((a + half_fov) // bin_deg)
        idx = max(0, min(n_bins - 1, idx))
        if d_m < bins[idx]: bins[idx] = d_m

    centers_deg = (np.arange(n_bins) + 0.5) * bin_deg - half_fov
    angles_rad  = np.radians(centers_deg)
    valid = bins < RANGE_MAX_M
    return angles_rad[valid], bins[valid]


# ============================================================
# SND 코어 함수
# ============================================================
def find_largest_gap(angles, dists, gap_threshold_m):
    is_gap = dists > gap_threshold_m
    if not np.any(is_gap): return None, 0.0
    diff = np.diff(is_gap.astype(int))
    starts = np.where(diff == 1)[0] + 1
    ends   = np.where(diff == -1)[0] + 1
    if is_gap[0]: starts = np.concatenate(([0], starts))
    if is_gap[-1]: ends = np.concatenate((ends, [len(is_gap)]))
    if len(starts) == 0: return None, 0.0
    
    best_width = -1.0
    best_seg   = None
    for s, e in zip(starts, ends):
        if e - s < 1: continue
        width = angles[e - 1] - angles[s]
        if width > best_width:
            best_width = width
            best_seg   = (s, e)
            
    if best_seg is None: return None, 0.0
    s, e = best_seg
    center = (angles[s] + angles[e - 1]) / 2.0
    return float(center), float(best_width)

def compute_avoidance_deflection(angles, dists, Ds, R):
    s_i = np.clip((Ds + R - dists) / Ds, 0.0, 1.0)
    w_i = s_i ** 2
    sum_w = np.sum(w_i)
    if sum_w < 1e-6: return 0.0, 0
    deflection = -np.sum(w_i * angles) / sum_w
    near_count = int(np.sum(s_i > 0))
    return float(deflection), near_count

# ★ 밀도 계산을 대체하는 "기하학적 폭 기반 Ds 계산" 함수
def calculate_geometric_Ds(angles, dists):
    if not USE_ADAPTIVE_Ds: 
        return Ds_TIGHT_M

    # 1. 전방 좌우(±10도 ~ ±90도)에서 가장 가까운 장애물 찾기
    # 정면(0도 근처)은 피하기 위해 0.17rad(약 10도) 띄움
    left_mask = (angles > 0.17) & (angles < math.pi/2)
    right_mask = (angles < -0.17) & (angles > -math.pi/2)

    left_dists = dists[left_mask]
    right_dists = dists[right_mask]

    # 한쪽이 완전히 뚫려있다면 넓은 공간으로 판단
    if len(left_dists) == 0 or len(right_dists) == 0:
        return Ds_OPEN_M

    # 2. 좌우 최단거리 점 추출
    min_idx_L = np.argmin(left_dists)
    d_L = left_dists[min_idx_L]
    theta_L = angles[left_mask][min_idx_L]

    min_idx_R = np.argmin(right_dists)
    d_R = right_dists[min_idx_R]
    theta_R = angles[right_mask][min_idx_R]

    # 3. 제2코사인 법칙을 이용해 양쪽 장애물 간의 실제 폭(Width) 연산
    angle_diff = abs(theta_L - theta_R)
    W = math.sqrt(d_L**2 + d_R**2 - 2 * d_L * d_R * math.cos(angle_diff))

    # 4. 폭에 따른 반비례 선형 보간 (좁은폭 -> 큰 Ds, 넓은폭 -> 작은 Ds)
    W_min = 0.5  # 0.5m 이하의 좁은 틈
    W_max = 1.2  # 1.2m 이상의 넓은 통로

    if W <= W_min:
        return Ds_TIGHT_M
    elif W >= W_max:
        return Ds_OPEN_M
    else:
        # W_min과 W_max 사이를 지날 때 부드럽게 Ds 조절
        ratio = (W - W_min) / (W_max - W_min)
        calc_ds = Ds_TIGHT_M - ratio * (Ds_TIGHT_M - Ds_OPEN_M)
        return float(calc_ds)

def select_speed(dists):
    if len(dists) == 0: return V_MIN_MPS
    min_d = float(np.min(dists))
    if min_d > SLOWDOWN_DIST_M: return V_MAX_MPS
    span = SLOWDOWN_DIST_M - ROBOT_RADIUS_M
    if span < 1e-3: return V_MIN_MPS
    ratio = max(0.0, (min_d - ROBOT_RADIUS_M) / span)
    v = V_MIN_MPS + (V_MAX_MPS - V_MIN_MPS) * ratio
    return float(max(V_MIN_MPS, min(V_MAX_MPS, v)))


# ============================================================
# 메인 클래스
# ============================================================
class SNDAvoider:
    def __init__(self):
        print("[INIT] RPLIDAR C1 열기...")
        self.lidar = RPLidarSerial(port=LIDAR_PORT, baudrate=LIDAR_BAUD)
        
        print("[INIT] 라이다 초기화 및 잔여 버퍼 비우기...")
        try:
            self.lidar.stop()
            time.sleep(0.5)
            self.lidar.reset()
        except Exception as e:
            print(f"[WARN] 라이다 초기화 예외: {e}")

        print("[INIT] 스캔 시작...")
        self.lidar.start_scan()
        self.scan_generator = self.lidar.iter_scans()

        self.arduino = None
        if not DRY_RUN:
            print("[INIT] Arduino 시리얼 열기...")
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
            time.sleep(2.0)

        self.start_time = time.time()

        self.smoothed_theta_target = 0.0  
        self.max_theta_step_rad = math.radians(10)  # 1루프당 최대 10도까지만 변화 허용 (튜닝 가능)
        self.lpf_alpha = 0.3                        # 로우패스 필터 계수 (0.1~0.5 사이. 작을수록 묵직함)
        
        # ★ 5Hz 갱신을 위한 타이머 및 상태 변수 추가
        self.last_ds_update_time = 0
        self.current_ds = Ds_TIGHT_M  # 초기값
        self.ds_update_interval = 1.0 / DS_UPDATE_HZ
        
        if USE_VIS:
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(6, 6))
            self.ax.set_title("SND LiDAR Vision")
            self.ax.set_xlim(-1.5, 1.5)
            self.ax.set_ylim(-0.5, 2.5)
            self.ax.grid(True)
            self.ax.set_aspect('equal')
            
            self.plot_pts, = self.ax.plot([], [], 'ro', markersize=3, label="Obstacles")
            self.plot_ds, = self.ax.plot([], [], 'b--', label="Ds (Safety Radius)")
            self.plot_target, = self.ax.plot([], [], 'g-', linewidth=4, label="Target Heading")
            self.ax.plot(0, 0, 'ks', markersize=8, label="Robot")
            self.ax.legend(loc="upper right")

    def send_command(self, v_mps, heading_rel_rad):
        v_mps           = max(-V_MAX_MPS, min(V_MAX_MPS, v_mps))
        heading_rel_rad = max(-MAX_HEADING_CMD_RAD,
                              min(MAX_HEADING_CMD_RAD, heading_rel_rad))
        cmd = f"C,{v_mps:+.3f},{heading_rel_rad:+.4f}\n"
        if self.arduino is not None:
            self.arduino.write(cmd.encode("ascii"))
        if VERBOSE:
            print(f"[CMD] v={v_mps:+.3f}  θ={math.degrees(heading_rel_rad):+5.1f}°")

    def stop(self):
        self.send_command(0.0, 0.0)

    def step(self):
        try:
            scan = next(self.scan_generator)
        except StopIteration:
            return
        except Exception as e:
            print(f"[WARN] LiDAR 스캔 실패: {e}")
            return

        if not scan:
            return

        angles, dists = downsample_scan(scan)
        if len(angles) == 0:
            self.send_command(V_MIN_MPS, 0.0)
            return

        # ★ 5Hz 주기로만 Ds 갱신 연산 수행
        now = time.time()
        if now - self.last_ds_update_time >= self.ds_update_interval:
            self.current_ds = calculate_geometric_Ds(angles, dists)
            self.last_ds_update_time = now
            
        Ds = self.current_ds
        gap_th = Ds + ROBOT_RADIUS_M
        theta_d, gap_w = find_largest_gap(angles, dists, gap_th)
        
        if theta_d is None:
            self.send_command(0.0, 0.0)
            theta_target = 0.0
            delta_avoid = 0.0
            near_count = 0
            if VERBOSE:
                print("[WARN] 통과 가능한 gap 없음 → 정지")
        else:
            delta_avoid, near_count = compute_avoidance_deflection(
                angles, dists, Ds, ROBOT_RADIUS_M
            )
            theta_target = theta_d + delta_avoid
            v = select_speed(dists)
            self.send_command(v, theta_target)

            if VERBOSE:
                print(f"      Ds={Ds:.2f}  Th_d={math.degrees(theta_d):+5.1f}deg  "
                      f"Del={math.degrees(delta_avoid):+5.1f}deg  gap_w={math.degrees(gap_w):.0f}deg")

        if USE_VIS:
            x_pts = dists * np.sin(angles)
            y_pts = dists * np.cos(angles)
            self.plot_pts.set_data(x_pts, y_pts)
            
            arc_angles = np.linspace(-math.pi/2, math.pi/2, 50)
            x_ds = Ds * np.sin(arc_angles)
            y_ds = Ds * np.cos(arc_angles)
            self.plot_ds.set_data(x_ds, y_ds)
            
            line_len = 0.8
            # ★ 초록 화살표 좌우 반전 오타 완전 제거 완료
            self.plot_target.set_data([0, line_len * math.sin(theta_target)],
                                      [0, line_len * math.cos(theta_target)])
            plt.pause(0.001)

    def run(self, duration_sec=60.0):
        period = 1.0 / LOOP_HZ
        print(f"[RUN] {duration_sec:.0f}s 동안 기하학적 적응형 SND 실행")
        try:
            while time.time() - self.start_time < duration_sec:
                t0 = time.time()
                self.step()
                dt = time.time() - t0
                if dt < period:
                    time.sleep(period - dt)
        except KeyboardInterrupt:
            print("\n[INFO] 사용자 인터럽트")
        finally:
            self.stop()
            time.sleep(0.3)
            try:
                self.lidar.stop()
                self.lidar.close()
            except Exception:
                pass
            if self.arduino:
                self.arduino.close()
            print("[DONE]")

if __name__ == "__main__":
    avoider = SNDAvoider()
    avoider.run(duration_sec=600.0)

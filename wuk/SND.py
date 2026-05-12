import math
import time
import serial
import numpy as np
import matplotlib.pyplot as plt  # 시각화를 위한 라이브러리 추가

# 사용자 환경의 RPLIDAR C1 드라이버 (기존)
from SND_rplidar_serial import RPLidarSerial


# ============================================================
# 설정 (튜닝 포인트는 # TUNE 표기)
# ============================================================

# --- 하드웨어 / 통신 ---
LIDAR_PORT          = "/dev/ttyUSB0"
LIDAR_BAUD          = 460800        # RPLIDAR C1 고정값
ARDUINO_PORT        = '/dev/ttyAMA3'
ARDUINO_BAUD        = 115200

# --- 로봇 기하 ---
ROBOT_RADIUS_M      = 0.15          # TUNE: 실측 로봇 반경 (외곽 포함)

# --- SND 코어 파라미터 ---
Ds_DEFAULT_M        = 0.30          # TUNE: 기본 안전거리
USE_ADAPTIVE_Ds     = True         # 처음엔 False, 충분히 튜닝 후 True
Ds_TIGHT_M          = 0.45          # TUNE: 고밀도(통로) Ds
Ds_OPEN_M           = 0.18          # TUNE: 저밀도(개활) Ds
DENSITY_NEAR_DIST_M = 0.40
DENSITY_THRESHOLD   = 25            # TUNE: 전방 0.4m 이내 점 개수 임계

# --- 속도 ---
V_MAX_MPS           = 0.20          # TUNE: 직진 최대 속도
V_MIN_MPS           = 0.06          # 근접 시 최소 속도
SLOWDOWN_DIST_M     = 0.35          # 이 거리 이하 근접 시 감속

# --- LiDAR 처리 ---
FRONT_FOV_DEG       = 180           # 전방 ±90° 사용
RANGE_MIN_M         = 0.05          # 노이즈/자기 자신 제거
RANGE_MAX_M         = 2.50
ANGLE_BIN_DEG       = 2             # 각도 빈 크기 (다운샘플링)

# --- 제어 루프 ---
LOOP_HZ             = 10
MAX_HEADING_CMD_RAD = math.radians(60)   # 명령 헤딩 클램프

# --- 디버그 / 안전 ---
VERBOSE             = True
DRY_RUN             = False          # 시각화 테스트 시 True(모터정지) 권장
USE_VIS             = False          # ★ 실시간 시각화 켜기/끄기


# ============================================================
# 유틸: LiDAR 스캔 다운샘플링 및 SND 코어 함수는 기존과 완벽히 동일
# ============================================================
# (공간 절약을 위해 함수 내부 생략. 기존 코드의 함수들을 그대로 유지하세요.)
def downsample_scan(scan_points, bin_deg=ANGLE_BIN_DEG, fov_deg=FRONT_FOV_DEG):
    # ... (기존과 동일) ...
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

def find_largest_gap(angles, dists, gap_threshold_m):
    # ... (기존과 동일) ...
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
    # ... (기존과 동일) ...
    s_i = np.clip((Ds + R - dists) / Ds, 0.0, 1.0)
    w_i = s_i ** 2
    sum_w = np.sum(w_i)
    if sum_w < 1e-6: return 0.0, 0
    deflection = -np.sum(w_i * angles) / sum_w
    near_count = int(np.sum(s_i > 0))
    return float(deflection), near_count

def select_Ds(angles, dists):
    # ... (기존과 동일) ...
    if not USE_ADAPTIVE_Ds: return Ds_DEFAULT_M
    near = int(np.sum(dists < DENSITY_NEAR_DIST_M))
    return Ds_TIGHT_M if near >= DENSITY_THRESHOLD else Ds_OPEN_M

def select_speed(dists):
    # ... (기존과 동일) ...
    if len(dists) == 0: return V_MIN_MPS
    min_d = float(np.min(dists))
    if min_d > SLOWDOWN_DIST_M: return V_MAX_MPS
    span = SLOWDOWN_DIST_M - ROBOT_RADIUS_M
    if span < 1e-3: return V_MIN_MPS
    ratio = max(0.0, (min_d - ROBOT_RADIUS_M) / span)
    v = V_MIN_MPS + (V_MAX_MPS - V_MIN_MPS) * ratio
    return float(max(V_MIN_MPS, min(V_MAX_MPS, v)))


# ============================================================
# 메인 클래스 (시각화 추가)
# ============================================================
class SNDAvoider:
    def __init__(self):
        print("[INIT] RPLIDAR C1 열기...")
        self.lidar = RPLidarSerial(port=LIDAR_PORT, baudrate=LIDAR_BAUD)
        self.lidar.start_scan()
        self.scan_generator = self.lidar.iter_scans()

        self.arduino = None
        if not DRY_RUN:
            print("[INIT] Arduino 시리얼 열기...")
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
            time.sleep(2.0)   # Arduino 리셋 대기

        self.start_time = time.time()
        
        # ★ 시각화 그래프 초기 셋업
        if USE_VIS:
            plt.ion() # 대화형 모드 켜기 (코드가 멈추지 않고 계속 실행됨)
            self.fig, self.ax = plt.subplots(figsize=(6, 6))
            self.ax.set_title("SND LiDAR Vision")
            
            # 그래프 범위 (로봇 위치 0,0 기준. Y축이 앞, X축이 오른쪽)
            self.ax.set_xlim(-1.5, 1.5)
            self.ax.set_ylim(-0.5, 2.5)
            self.ax.grid(True)
            self.ax.set_aspect('equal')
            
            # 그릴 요소들 껍데기 생성
            self.plot_pts, = self.ax.plot([], [], 'ro', markersize=3, label="Obstacles")
            self.plot_ds, = self.ax.plot([], [], 'b--', label="Ds (Safety Radius)")
            self.plot_target, = self.ax.plot([], [], 'g-', linewidth=4, label="Target Heading")
            self.ax.plot(0, 0, 'ks', markersize=8, label="Robot") # 로봇 본체
            
            self.ax.legend(loc="upper right")

    # --------------------------------------------------------
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

    # --------------------------------------------------------
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

        Ds = select_Ds(angles, dists)
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
                print(f"      Ds={Ds:.2f}  θ_d={math.degrees(theta_d):+5.1f}°  "
                      f"Δ={math.degrees(delta_avoid):+5.1f}°  "
                      f"near={near_count}  gap_w={math.degrees(gap_w):.0f}°")

        # ★ 시각화 업데이트 로직
        if USE_VIS:
            # 1. 라이다 점들을 2D 좌표(X, Y)로 변환
            # (수학 공식: 앞이 Y축, 왼쪽이 -X축이 되도록 변환)
            x_pts = dists * np.sin(angles)
            y_pts = dists * np.cos(angles)
            self.plot_pts.set_data(x_pts, y_pts)
            
            # 2. 안전거리(Ds) 반원 그리기
            arc_angles = np.linspace(-math.pi/2, math.pi/2, 50)
            x_ds = Ds * np.sin(arc_angles)
            y_ds = Ds * np.cos(arc_angles)
            self.plot_ds.set_data(x_ds, y_ds)
            
            # 3. 최종 목표 방향 화살표(초록선) 그리기
            line_len = 0.8
            self.plot_target.set_data([0, -line_len * math.sin(theta_target)],
                                      [0, line_len * math.cos(theta_target)])
            
            # 4. 화면 새로고침 (아주 짧은 시간 대기)
            plt.pause(0.001)

    # --------------------------------------------------------
    def run(self, duration_sec=60.0):
        period = 1.0 / LOOP_HZ
        print(f"[RUN] {duration_sec:.0f}s 동안 SND 실행 (period={period*1000:.0f}ms)")
        try:
            while time.time() - self.start_time < duration_sec:
                t0 = time.time()
                self.step()
                dt = time.time() - t0
                if dt < period:
                    time.sleep(period - dt)
                elif VERBOSE:
                    pass # 시각화 때문에 사이클이 초과될 수 있으므로 로그 생략
        except KeyboardInterrupt:
            print("\n[INFO] 사용자 인터럽트")
        finally:
            self.stop()
            time.sleep(0.3)
            try:
                self.lidar.stop()
                self.lidar.close()
            except Exception as e:
                pass
            if self.arduino:
                self.arduino.close()
            print("[DONE]")


if __name__ == "__main__":
    avoider = SNDAvoider()
    avoider.run(duration_sec=600.0) # 10분 동안 넉넉하게 실행되게 늘림

"""
snd_obstacle_avoider.py — v7
============================
주요 변경점 (v6 대비):
  ① Gap 통과 임계를 Ds와 분리 (GAP_PASS_TH_M)
       - Ds는 deflection 계산용으로만 사용 (SND 원래 의미)
       - Gap 찾기는 robot radius + 여유로 별도 판정
  ② 충돌 안전망 (COLLISION_GUARD_M) — 즉시 회피 + 정지
  ③ 위급도(urgency) 기반 동적 필터
       - 평상시: 작은 step + 작은 α  (부드러움)
       - 위급시: 큰 step + 큰 α       (즉응)
  ④ 데드락 정책 — 후진/제자리회전 최소화
       단계 1 (1.0s): 전진 유지 + 큰 회전 시도
       단계 2 (0.5s): 짧은 후진 + 회전 (last resort)
       단계 3      : 리셋 후 재시도
  ⑤ Gap 점수 = 각도폭 × 평균거리 (호 길이) — 멀리 있는 작은 gap 우회
  ⑥ SLOWDOWN_DIST 0.35 → 0.50 (감속 일찍 시작, 회피각 형성 시간 확보)
"""

import math
import time
import serial
import numpy as np
import matplotlib.pyplot as plt

from SND_rplidar_serial import RPLidarSerial


# ============================================================
# 설정
# ============================================================

# --- 하드웨어 / 통신 ---
LIDAR_PORT          = "/dev/ttyUSB0"
LIDAR_BAUD          = 460800
ARDUINO_PORT        = '/dev/ttyAMA3'
ARDUINO_BAUD        = 115200

# --- 로봇 기하 ---
ROBOT_RADIUS_M      = 0.15

# --- SND Ds (deflection 계산용) ---
USE_ADAPTIVE_Ds     = True
Ds_TIGHT_M          = 0.45
Ds_OPEN_M           = 0.18

# --- ★ Gap 통과 판정 (Ds와 분리) ---
GAP_PASS_TH_M       = ROBOT_RADIUS_M + 0.05   # = 0.20 m

# --- ★ 위급도 임계 ---
WARNING_DIST_M      = 0.40        # 이 이하부터 응급 모드 점진 진입
EMERGENCY_DIST_M    = 0.22        # 이 이하면 완전 응급 (urgency=1.0)
COLLISION_GUARD_M   = 0.18        # 즉시 정지 + 큰 회전 (안전망)

# --- 속도 ---
V_MAX_MPS           = 0.20        # 최대 직진 속도 (필요시 0.25까지 증가 가능)
V_MIN_MPS           = 0.06        # 위급 시 최소 속도
SLOWDOWN_DIST_M     = 0.50        # ★ 감속 시작 거리 (0.35 → 0.50)
V_REVERSE_MPS       = -0.08       # ★ 데드락 last-resort 후진 속도

# --- LiDAR 처리 ---
FRONT_FOV_DEG       = 180
RANGE_MIN_M         = 0.05
RANGE_MAX_M         = 2.50
ANGLE_BIN_DEG       = 2

# --- 제어 루프 ---
LOOP_HZ             = 10
DS_UPDATE_HZ        = 5
MAX_HEADING_CMD_RAD = math.radians(70)    # ★ 60 → 70 (위급 회피용)

# --- ★ 동적 필터 (urgency 0~1 사이에서 보간) ---
STEP_DEG_CALM       = 10          # 평상시 1루프당 회전 한계
STEP_DEG_PANIC      = 60          # 위급 시
LPF_ALPHA_CALM      = 0.30
LPF_ALPHA_PANIC     = 0.90

# --- ★ 데드락 (gap 없을 때) ---
NO_GAP_TRY_FORWARD_S = 1.0        # 전진 유지 + 큰 회전 시도
NO_GAP_TRY_REVERSE_S = 0.5        # 그 후 짧은 후진 허용
NO_GAP_ESCAPE_ANGLE  = math.radians(55)

# --- 디버그 ---
VERBOSE             = True
DRY_RUN             = False
USE_VIS             = False


# ============================================================
# LiDAR 스캔 다운샘플링
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
    """
    ★ 변경: 각도 폭이 아니라 (각도 폭 × 평균 거리) = 호 길이로 점수화.
    멀리 있는 작은 gap이 가까이 있는 큰 gap을 이기는 문제 방지.
    """
    is_gap = dists > gap_threshold_m
    if not np.any(is_gap):
        return None, 0.0

    diff   = np.diff(is_gap.astype(int))
    starts = np.where(diff == 1)[0] + 1
    ends   = np.where(diff == -1)[0] + 1
    if is_gap[0]:  starts = np.concatenate(([0], starts))
    if is_gap[-1]: ends   = np.concatenate((ends, [len(is_gap)]))
    if len(starts) == 0:
        return None, 0.0

    best_score = -1.0
    best_seg   = None
    for s, e in zip(starts, ends):
        if e - s < 1: continue
        ang_width = angles[e - 1] - angles[s]
        avg_dist  = float(np.mean(dists[s:e]))
        score     = ang_width * avg_dist        # ★ 호 길이 근사
        if score > best_score:
            best_score = score
            best_seg   = (s, e, ang_width)

    if best_seg is None:
        return None, 0.0
    s, e, ang_width = best_seg
    center = (angles[s] + angles[e - 1]) / 2.0
    return float(center), float(ang_width)


def compute_avoidance_deflection(angles, dists, Ds, R):
    s_i = np.clip((Ds + R - dists) / Ds, 0.0, 1.0)
    w_i = s_i ** 2
    sum_w = np.sum(w_i)
    if sum_w < 1e-6:
        return 0.0, 0
    deflection = -np.sum(w_i * angles) / sum_w
    near_count = int(np.sum(s_i > 0))
    return float(deflection), near_count


def calculate_geometric_Ds(angles, dists):
    """좌우 최단점 사이의 실제 폭을 코사인 법칙으로 계산 → Ds 보간."""
    if not USE_ADAPTIVE_Ds:
        return Ds_TIGHT_M

    left_mask  = (angles >  0.17) & (angles <  math.pi/2)
    right_mask = (angles < -0.17) & (angles > -math.pi/2)

    left_dists  = dists[left_mask]
    right_dists = dists[right_mask]

    if len(left_dists) == 0 or len(right_dists) == 0:
        return Ds_OPEN_M

    min_idx_L = np.argmin(left_dists)
    d_L     = left_dists[min_idx_L]
    theta_L = angles[left_mask][min_idx_L]

    min_idx_R = np.argmin(right_dists)
    d_R     = right_dists[min_idx_R]
    theta_R = angles[right_mask][min_idx_R]

    angle_diff = abs(theta_L - theta_R)
    W = math.sqrt(d_L**2 + d_R**2 - 2 * d_L * d_R * math.cos(angle_diff))

    W_min = 0.5
    W_max = 1.2
    if W <= W_min:
        return Ds_TIGHT_M
    elif W >= W_max:
        return Ds_OPEN_M
    else:
        ratio = (W - W_min) / (W_max - W_min)
        return float(Ds_TIGHT_M - ratio * (Ds_TIGHT_M - Ds_OPEN_M))


def select_speed(min_d):
    """가장 가까운 장애물 거리 기반 선속도."""
    if min_d > SLOWDOWN_DIST_M:
        return V_MAX_MPS
    span = SLOWDOWN_DIST_M - ROBOT_RADIUS_M
    if span < 1e-3:
        return V_MIN_MPS
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

        # --- 헤딩 필터 상태 ---
        self.smoothed_theta_target = 0.0

        # --- Ds 갱신 ---
        self.last_ds_update_time = 0.0
        self.current_ds          = Ds_TIGHT_M
        self.ds_update_interval  = 1.0 / DS_UPDATE_HZ

        # --- ★ 데드락 추적 ---
        self.no_gap_count = 0

        # --- 시각화 (꺼져 있음) ---
        if USE_VIS:
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(6, 6))
            self.ax.set_xlim(-1.5, 1.5); self.ax.set_ylim(-0.5, 2.5)
            self.ax.grid(True); self.ax.set_aspect('equal')
            self.plot_pts,    = self.ax.plot([], [], 'ro', markersize=3)
            self.plot_ds,     = self.ax.plot([], [], 'b--')
            self.plot_target, = self.ax.plot([], [], 'g-',  linewidth=4)
            self.ax.plot(0, 0, 'ks', markersize=8)

    # ============================================================
    # 송신
    # ============================================================
    def send_command(self, v_mps, heading_rel_rad):
        v_mps           = max(-V_MAX_MPS, min(V_MAX_MPS, v_mps))
        heading_rel_rad = max(-MAX_HEADING_CMD_RAD,
                              min(MAX_HEADING_CMD_RAD, heading_rel_rad))
        cmd = f"C,{v_mps:+.3f},{heading_rel_rad:+.4f}\n"
        if self.arduino is not None:
            self.arduino.write(cmd.encode("ascii"))

    def stop(self):
        self.send_command(0.0, 0.0)

    # ============================================================
    # ★ 위급도 및 동적 필터
    # ============================================================
    @staticmethod
    def _compute_urgency(min_d):
        """0=안전, 1=완전응급. WARNING~EMERGENCY 사이 선형 보간."""
        if min_d >= WARNING_DIST_M:   return 0.0
        if min_d <= EMERGENCY_DIST_M: return 1.0
        return (WARNING_DIST_M - min_d) / (WARNING_DIST_M - EMERGENCY_DIST_M)

    @staticmethod
    def _dynamic_filter_params(urgency):
        step_deg = STEP_DEG_CALM   + (STEP_DEG_PANIC   - STEP_DEG_CALM)   * urgency
        alpha    = LPF_ALPHA_CALM  + (LPF_ALPHA_PANIC  - LPF_ALPHA_CALM)  * urgency
        return math.radians(step_deg), alpha

    def _apply_dynamic_filter(self, raw_theta, urgency):
        max_step, alpha = self._dynamic_filter_params(urgency)
        diff = raw_theta - self.smoothed_theta_target
        diff = max(-max_step, min(max_step, diff))
        limited = self.smoothed_theta_target + diff
        self.smoothed_theta_target = (1.0 - alpha) * self.smoothed_theta_target + alpha * limited
        return self.smoothed_theta_target

    # ============================================================
    # ★ 데드락 처리 (gap 없을 때)
    # ============================================================
    def _handle_no_gap(self, angles, dists):
        """
        후진/제자리회전을 최대한 미루는 단계별 정책.
          Phase 1 (0~1.0s):  V_MIN 전진 + 큰 회전
          Phase 2 (1.0~1.5s): 짧게 후진 + 회전
          Phase 3 (이후):     카운터 리셋, 재시도
        """
        self.no_gap_count += 1
        elapsed = self.no_gap_count / LOOP_HZ

        # 가장 가까운 장애물의 반대 방향으로 회전
        if len(angles) > 0:
            closest_idx = int(np.argmin(dists))
            escape_dir  = -np.sign(angles[closest_idx]) or 1.0
        else:
            escape_dir = 1.0

        target = escape_dir * NO_GAP_ESCAPE_ANGLE
        self.smoothed_theta_target = target   # 필터 우회

        if elapsed < NO_GAP_TRY_FORWARD_S:
            # Phase 1: 가능한 한 전진하면서 회피
            self.send_command(V_MIN_MPS, target)
            if VERBOSE:
                print(f"[NO_GAP/FWD] {elapsed:.1f}s  θ={math.degrees(target):+.0f}°")
            return

        if elapsed < NO_GAP_TRY_FORWARD_S + NO_GAP_TRY_REVERSE_S:
            # Phase 2: last resort — 짧게 후진
            self.send_command(V_REVERSE_MPS, target)
            if VERBOSE:
                print(f"[NO_GAP/REV] {elapsed:.1f}s  v={V_REVERSE_MPS:+.2f}  θ={math.degrees(target):+.0f}°")
            return

        # Phase 3: 리셋, 다시 처음부터
        self.no_gap_count = 0
        self.send_command(V_MIN_MPS, target)

    # ============================================================
    # 메인 step
    # ============================================================
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

        # ============================================
        # ① 최소 거리 및 위급도
        # ============================================
        min_d   = float(np.min(dists))
        min_idx = int(np.argmin(dists))
        urgency = self._compute_urgency(min_d)

        # ============================================
        # ② ★ 충돌 직전 안전망 (즉시 정지 + 큰 회전)
        #    후진은 안 함 (제자리에서 회전만)
        # ============================================
        if min_d < COLLISION_GUARD_M:
            escape_dir = -np.sign(angles[min_idx]) or 1.0
            target = escape_dir * math.radians(70)
            self.smoothed_theta_target = target   # 필터 우회
            self.send_command(0.0, target)        # 제자리 회전
            if VERBOSE:
                print(f"[GUARD] min_d={min_d:.2f}  θ={math.degrees(target):+.0f}° (제자리 회전)")
            return

        # ============================================
        # ③ Ds 갱신 (5Hz)
        # ============================================
        now = time.time()
        if now - self.last_ds_update_time >= self.ds_update_interval:
            self.current_ds = calculate_geometric_Ds(angles, dists)
            self.last_ds_update_time = now
        Ds = self.current_ds

        # ============================================
        # ④ Gap 찾기 (★ Ds와 분리된 임계)
        # ============================================
        theta_d, gap_w = find_largest_gap(angles, dists, GAP_PASS_TH_M)

        # ============================================
        # ⑤ Gap 없음 → 데드락 정책
        # ============================================
        if theta_d is None:
            self._handle_no_gap(angles, dists)
            return

        # gap 발견 → no-gap 카운터 리셋
        self.no_gap_count = 0

        # ============================================
        # ⑥ Deflection + 동적 필터
        # ============================================
        delta_avoid, near_count = compute_avoidance_deflection(
            angles, dists, Ds, ROBOT_RADIUS_M
        )
        raw_theta = theta_d + delta_avoid
        out_theta = self._apply_dynamic_filter(raw_theta, urgency)

        # ============================================
        # ⑦ 속도 결정 및 송신
        # ============================================
        v = select_speed(min_d)
        self.send_command(v, out_theta)

        if VERBOSE:
            print(f"  Ds={Ds:.2f}  min_d={min_d:.2f}  urg={urgency:.2f}  "
                  f"raw={math.degrees(raw_theta):+5.1f}°  "
                  f"out={math.degrees(out_theta):+5.1f}°  v={v:.2f}")

        # ============================================
        # ⑧ 시각화 (선택)
        # ============================================
        if USE_VIS:
            x_pts = dists * np.sin(angles); y_pts = dists * np.cos(angles)
            self.plot_pts.set_data(x_pts, y_pts)
            arc = np.linspace(-math.pi/2, math.pi/2, 50)
            self.plot_ds.set_data(Ds*np.sin(arc), Ds*np.cos(arc))
            L = 0.8
            self.plot_target.set_data([0, L*math.sin(out_theta)],
                                      [0, L*math.cos(out_theta)])
            plt.pause(0.001)

    # ============================================================
    # 실행
    # ============================================================
    def run(self, duration_sec=60.0):
        period = 1.0 / LOOP_HZ
        print(f"[RUN] {duration_sec:.0f}s — SND v7")
        try:
            while time.time() - self.start_time < duration_sec:
                t0 = time.time()
                self.step()
                dt = time.time() - t0
                if dt < period:
                    time.sleep(period - dt)
                elif VERBOSE:
                    print(f"[WARN] 사이클 초과: {dt*1000:.0f}ms")
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
    SNDAvoider().run(duration_sec=600.0)

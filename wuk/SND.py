"""
snd_obstacle_avoider.py
========================
SND (Smooth Nearness Diagram) 기반 장애물 회피 — Raspberry Pi 측.

References
----------
- Durham & Bullo, "Smooth Nearness-Diagram Navigation", IROS 2008
- 이나라, 권순환, 유혜정, "2차원 라이다 센서 데이터 분류를 이용한 적응형
  장애물 회피 알고리즘", J. Sens. Sci. Technol. 29(5), 2020
  → 본 코드에선 적응형 Ds를 휴리스틱(고밀도/저밀도)으로 단순화하여 도입.

Hardware
--------
- Raspberry Pi + RPLIDAR C1 (사용자 정의 rplidar_c1.py 드라이버)
- Serial → Arduino (UNO R4 Minima)

Protocol to Arduino
-------------------
"C,<v_mps>,<heading_rel_rad>\\n"
- v_mps        : 선속도 (m/s, +=전진, -=후진)
- heading_rel  : 현재 헤딩 기준 상대 회전 목표 (rad, +=좌, -=우)
"""

import math
import time
import serial
import numpy as np

# 사용자 환경의 RPLIDAR C1 드라이버 (기존)
import SND_rplidar_serial


# ============================================================
# 설정 (튜닝 포인트는 # TUNE 표기)
# ============================================================

# --- 하드웨어 / 통신 ---
LIDAR_PORT          = "/dev/ttyUSB0"
LIDAR_BAUD          = 460800        # RPLIDAR C1 고정값
ARDUINO_PORT        = "/dev/ttyACM0"
ARDUINO_BAUD        = 115200

# --- 로봇 기하 ---
ROBOT_RADIUS_M      = 0.09          # TUNE: 실측 로봇 반경 (외곽 포함)

# --- SND 코어 파라미터 ---
Ds_DEFAULT_M        = 0.30          # TUNE: 기본 안전거리
USE_ADAPTIVE_Ds     = False         # 처음엔 False, 충분히 튜닝 후 True
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
DRY_RUN             = False         # True면 시리얼 송신 안함 (테스트용)


# ============================================================
# 유틸: LiDAR 스캔 다운샘플링
# ============================================================

def downsample_scan(scan_points,
                    bin_deg=ANGLE_BIN_DEG,
                    fov_deg=FRONT_FOV_DEG):
    """
    rplidar 스캔 → (angles_rad, dists_m) 배열.

    좌표 약속
    --------
    - LiDAR 0° 방향이 로봇 정면이라고 가정 (RPLIDAR 장착 방향 확인 필요).
    - 반시계방향이 +각도. 즉 angle=0 정면, +π/2 좌측, -π/2 우측.
    - 각 angle bin 내에서 *최소* 거리만 보존 (가장 가까운 위험).

    Returns
    -------
    angles_rad : np.array (rad), 빈의 중심각
    dists_m    : np.array (m), 그 빈의 최소 거리
    """
    half_fov = fov_deg / 2.0
    n_bins   = int(fov_deg / bin_deg)
    bins     = np.full(n_bins, np.inf)

    for pt in scan_points:
        # 일반적 rplidar 출력 형식: (quality, angle_deg, dist_mm)
        # 사용자 드라이버 형식에 맞춰 unpack 수정 가능
        _, angle_deg, dist_mm = pt

        # angle을 [-180, 180]로 정규화 (RPLIDAR는 0~360 출력)
        a = ((angle_deg + 180.0) % 360.0) - 180.0
        if abs(a) > half_fov:
            continue

        d_m = dist_mm / 1000.0
        if d_m < RANGE_MIN_M or d_m > RANGE_MAX_M:
            continue

        idx = int((a + half_fov) // bin_deg)
        idx = max(0, min(n_bins - 1, idx))
        if d_m < bins[idx]:
            bins[idx] = d_m

    centers_deg = (np.arange(n_bins) + 0.5) * bin_deg - half_fov
    angles_rad  = np.radians(centers_deg)

    valid = bins < RANGE_MAX_M
    return angles_rad[valid], bins[valid]


# ============================================================
# SND 코어 함수
# ============================================================

def find_largest_gap(angles, dists, gap_threshold_m):
    """
    'gap' = dists > gap_threshold_m 인 연속 빔 구간.
    가장 넓은 (각도 폭이 가장 큰) gap의 중심각·폭 반환.

    Returns
    -------
    center_angle_rad : 가장 넓은 gap의 bisector. 없으면 None.
    width_rad        : 그 gap의 각도 폭 (rad).
    """
    is_gap = dists > gap_threshold_m
    if not np.any(is_gap):
        return None, 0.0

    diff = np.diff(is_gap.astype(int))
    starts = np.where(diff == 1)[0] + 1
    ends   = np.where(diff == -1)[0] + 1

    if is_gap[0]:
        starts = np.concatenate(([0], starts))
    if is_gap[-1]:
        ends = np.concatenate((ends, [len(is_gap)]))

    if len(starts) == 0:
        return None, 0.0

    best_width = -1.0
    best_seg   = None
    for s, e in zip(starts, ends):
        if e - s < 1:
            continue
        width = angles[e - 1] - angles[s]
        if width > best_width:
            best_width = width
            best_seg   = (s, e)

    if best_seg is None:
        return None, 0.0

    s, e = best_seg
    center = (angles[s] + angles[e - 1]) / 2.0
    return float(center), float(best_width)


def compute_avoidance_deflection(angles, dists, Ds, R):
    """
    SND 회피 편향 Δ_avoid.

    각 점 i에 대해:
      s_i = sat[0,1]( (Ds + R - D_i) / Ds )   ← 가까울수록 1에 근접
      w_i = s_i^2
    Δ_avoid = Σ w_i * (-θ_i) / Σ w_i
       (장애물이 -θ쪽에 있으면 헤딩을 +θ로 밀어내는 방향)
    """
    s_i = np.clip((Ds + R - dists) / Ds, 0.0, 1.0)
    w_i = s_i ** 2
    sum_w = np.sum(w_i)

    if sum_w < 1e-6:
        return 0.0, 0

    deflection = -np.sum(w_i * angles) / sum_w
    near_count = int(np.sum(s_i > 0))
    return float(deflection), near_count


def select_Ds(angles, dists):
    """
    적응형 Ds. 전방 DENSITY_NEAR_DIST_M 이내 점 개수로
    고밀도/저밀도 이진 판단.
    """
    if not USE_ADAPTIVE_Ds:
        return Ds_DEFAULT_M
    near = int(np.sum(dists < DENSITY_NEAR_DIST_M))
    return Ds_TIGHT_M if near >= DENSITY_THRESHOLD else Ds_OPEN_M


def select_speed(dists):
    """ 근접 장애물 있으면 선형 감속. """
    if len(dists) == 0:
        return V_MIN_MPS
    min_d = float(np.min(dists))
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
        self.lidar = RPLidarC1(port=LIDAR_PORT, baudrate=LIDAR_BAUD)

        self.arduino = None
        if not DRY_RUN:
            print("[INIT] Arduino 시리얼 열기...")
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
            time.sleep(2.0)   # Arduino 리셋 대기

        self.start_time = time.time()

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
        """ 한 사이클: 스캔 → SND → 명령 송신. """
        # 1) 스캔 획득
        #    rplidar_c1 모듈 API에 따라 호출 변경:
        #    - generator 패턴: next(self.lidar.iter_scans())
        #    - 단발 패턴:       self.lidar.get_scan()
        try:
            scan = next(self.lidar.iter_scans())
        except Exception as e:
            print(f"[WARN] LiDAR 스캔 실패: {e}")
            return

        if not scan:
            return

        # 2) 다운샘플
        angles, dists = downsample_scan(scan)
        if len(angles) == 0:
            self.send_command(V_MIN_MPS, 0.0)
            return

        # 3) Ds 결정
        Ds = select_Ds(angles, dists)

        # 4) θ_d : 가장 넓은 gap의 bisector
        gap_th = Ds + ROBOT_RADIUS_M
        theta_d, gap_w = find_largest_gap(angles, dists, gap_th)
        if theta_d is None:
            # 통과 가능한 gap 없음 → 일단 정지
            # (향후: 회전 탐색 / 후진 로직 추가 가능)
            self.send_command(0.0, 0.0)
            if VERBOSE:
                print("[WARN] 통과 가능한 gap 없음 → 정지")
            return

        # 5) Δ_avoid
        delta_avoid, near_count = compute_avoidance_deflection(
            angles, dists, Ds, ROBOT_RADIUS_M
        )

        # 6) 최종 헤딩
        theta_target = theta_d + delta_avoid

        # 7) 속도
        v = select_speed(dists)

        # 8) 송신
        self.send_command(v, theta_target)

        if VERBOSE:
            print(f"      Ds={Ds:.2f}  θ_d={math.degrees(theta_d):+5.1f}°  "
                  f"Δ={math.degrees(delta_avoid):+5.1f}°  "
                  f"near={near_count}  gap_w={math.degrees(gap_w):.0f}°")

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
                    print(f"[WARN] 사이클 초과: {dt*1000:.1f}ms > {period*1000:.0f}ms")
        except KeyboardInterrupt:
            print("\n[INFO] 사용자 인터럽트")
        finally:
            self.stop()
            time.sleep(0.3)
            try:
                self.lidar.stop()
                self.lidar.disconnect()
            except Exception:
                pass
            if self.arduino:
                self.arduino.close()
            print("[DONE]")


# ============================================================
# 진입점
# ============================================================
if __name__ == "__main__":
    avoider = SNDAvoider()
    avoider.run(duration_sec=60.0)

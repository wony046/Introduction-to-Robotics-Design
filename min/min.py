# -*- coding: utf-8 -*-
"""
═══════════════════════════════════════════════════════════════════════════════
 backup.py  —  제어 타워 (메인)
═══════════════════════════════════════════════════════════════════════════════
 camera_tracker1(센서) + 과제2 회피엔진(LiDAR) 을 받아 상태를 결정하고 모터를 굴린다.

 [문제1] 도달 인지 + 1초 정지
   센서의 arrived 플래그(면적 팽창→하단 이탈) 수신 → 즉시 정지 → ARRIVE_STOP_S
   이상 대기 → 다음 색으로 전환(RED→YELLOW→BLUE) → 주행 재개. 마지막이면 DONE.

 [문제2] 목표 미검출 시 3단계 상태머신
   A TRACK : bearing 정상 → 목표로 조향 + 장애물 회피(융합)
   B SPIN  : bearing None → 제자리 360° 회전 스캔
   C WANDER: 360° 돌아도 못 찾음 → 회피엔진(FGM)으로 빈 공간 전진(시야 확보)
   (공통) B/C 중 언제든 bearing 들어오면 즉시 A 복귀.

 [문제3] 경기장 이탈 감지/복귀 (LiDAR '허허벌판' 인식)
   "장애물이 모여있는 곳 = 경기장". 주변에 가까운 물체가 없고 전방 평균거리가
   멀면(허허벌판) → RETURN: 제자리 회전하며 가까운 물체가 많은 방향을 찾아
   그쪽(=경기장)으로 복귀. (목표가 보이면 목표가 곧 경기장 안이므로 TRACK 우선)

 실행
   1) 과제2 최종코드를 같은 폴더에 avoidance_lidar.py 로 둔다.
   2) python3 backup.py   (출발지 정지 상태에서 실행, 이후 입력 불가)
═══════════════════════════════════════════════════════════════════════════════
"""

import time
import math
import threading
import importlib

import serial

from camera_tracker1 import CameraTracker

# ── 회피엔진(과제2) 모듈 자동 탐색 ────────────────────────────────────────────
av = None
for _name in ["avoidance_lidar", "로입설_과제2_최종코드", "avoidance", "obstacle_avoidance"]:
    try:
        av = importlib.import_module(_name); print(f"[INIT] 회피엔진 로드: {_name}"); break
    except ImportError:
        continue
if av is None:
    raise ImportError("과제2 회피엔진을 같은 폴더에 avoidance_lidar.py 로 두세요.")
for _f in ("DEBUG_LAYERS","DEBUG_STOP","DEBUG_DIR","DEBUG_FINAL","DEBUG_SIDE","DEBUG_VIRTUAL"):
    if hasattr(av, _f):
        setattr(av, _f, False)


# ═══════════════════════════════════════════════════════════════════════════════
# 파라미터  (★ = 현장 캘리브레이션)
# ═══════════════════════════════════════════════════════════════════════════════
COLOR_SEQUENCE = ["RED", "YELLOW", "BLUE"]

# 통신/속도 (회피엔진과 공유)
LIDAR_PORT       = getattr(av, "LIDAR_PORT",       "/dev/ttyUSB0")
ARDUINO_PORT     = getattr(av, "ARDUINO_PORT",     "/dev/ttyAMA3")
BAUDRATE_LIDAR   = getattr(av, "BAUDRATE_LIDAR",   460800)
BAUDRATE_ARDUINO = getattr(av, "BAUDRATE_ARDUINO", 115200)
FORWARD_SPEED    = getattr(av, "FORWARD_SPEED",    0.45)
MAX_W            = getattr(av, "MAX_W",            1.8)
W_SMOOTH         = getattr(av, "W_SMOOTH",         0.7)
LIDAR_MIN_VALID  = getattr(av, "LIDAR_MIN_VALID",  100)   # mm

# [문제1] 도달 정지
ARRIVE_STOP_S    = 1.2       # 규칙 1초 + 마진

# [문제2] A TRACK 융합/접근
GOAL_W_GAIN      = 1.6
BEARING_TO_W_SIGN = -1.0     # +bearing(목표 오른쪽)→우회전(w<0). 카메라 좌우 반전 시 +1.0
BEARING_DEADBAND = 6.0       # deg
APPROACH_FAR     = 0.40
APPROACH_NEAR    = 0.12
AREA_FRAC_SLOW   = 0.06      # 면적비가 이 값이면 NEAR 까지 감속

# [문제2] B SPIN / C WANDER
SPIN_W           = 0.8       # rad/s 제자리 스캔
SPIN_FULL_TURN   = 2*math.pi*1.05   # 한 바퀴(+5% 마진) 누적 회전 → WANDER
WANDER_SPEED     = 0.32      # 배회 전진 상한

# [문제3] 허허벌판(이탈) 감지/복귀
OPEN_NEAR_DIST   = 1200      # mm: 전 방향 최근접이 이보다 멀면 '주변에 물체 없음'
OPEN_FIELD_DIST  = 1500      # mm: 전방 180° 평균이 이보다 멀면 '허허벌판'  (≈DETECTION_RANGE)
RETURN_SPIN_W    = 0.9       # rad/s: 복귀용 제자리 회전
RETURN_FACE_TOL  = 25        # deg: 경기장 방향을 이 이내로 향하면 전진 복귀
RETURN_SPEED     = 0.30

# 시간/루프
MATCH_TIME_LIMIT_S = 178.0
MAIN_LOOP_HZ       = 20

# 아두이노 인터페이스 (sketch_jjw.ino 대응)
#  · 아두이노는 정상 모드에서 heading ±90° 초과 시 같은 방향 w 를 0으로 차단(헤딩가드).
#    → 제자리 360° 스캔(SPIN)/복귀 회전(RETURN)이 막힘.
#  · 'ESC' 명령 = heading 리셋 + 18초간 헤딩가드 비활성(자유 회전).
#    탐색 과제는 자유 회전이 필수이므로, ESC 를 주기적으로 재전송해 가드를 상시 해제한다.
#  · 'R'(헤딩 리셋)은 USB Serial 전용이라 파이 링크(Serial1)에선 무시됨 → 시작도 ESC 사용.
ESC_KEEPALIVE_S = 12.0     # ESC 재전송 주기 (아두이노 ESCAPE_TIMEOUT 18초보다 짧게)


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# ═══════════════════════════════════════════════════════════════════════════════
# LiDAR 분석 (문제3)
# ═══════════════════════════════════════════════════════════════════════════════
def is_open_field(scan):
    """허허벌판(경기장 이탈) 판정: 주변에 가까운 물체가 없고 전방 평균이 멀다."""
    valid = [d for _, d in scan if d > LIDAR_MIN_VALID]
    if not valid:
        return True                      # 아무것도 안 잡힘 → 완전 개활지
    nearest = min(valid)
    front   = [d for a, d in scan if abs(_wrap180(a)) <= 90 and d > LIDAR_MIN_VALID]
    front_avg = (sum(front) / len(front)) if front else 1e9
    return nearest >= OPEN_NEAR_DIST and front_avg >= OPEN_FIELD_DIST


def arena_direction(scan):
    """가장 가까운 물체 방향(deg, +오른쪽)과 거리(mm). = 경기장 쪽. 없으면 (None,None)."""
    pts = [(a, d) for a, d in scan if d > LIDAR_MIN_VALID]
    if not pts:
        return None, None
    a, d = min(pts, key=lambda p: p[1])
    return _wrap180(a), d


# ═══════════════════════════════════════════════════════════════════════════════
# 제어 타워 상태머신
# ═══════════════════════════════════════════════════════════════════════════════
class ControlTower:
    TRACK, SPIN, WANDER, RETURN, STOP_HOLD, DONE = \
        "TRACK", "SPIN", "WANDER", "RETURN", "STOP_HOLD", "DONE"

    def __init__(self, cam: CameraTracker):
        self.cam   = cam
        self.idx   = 0
        self.mode  = self.SPIN
        self.cam.set_target(COLOR_SEQUENCE[0])

        self._spin_dir   = 1.0
        self._spin_accum = 0.0
        self._hold_t0    = 0.0
        self._announced  = None

    @property
    def target(self):
        return COLOR_SEQUENCE[self.idx]

    def _say(self, msg):
        if self._announced != msg:
            print(f"[STATE] {msg}")
            self._announced = msg

    def _enter_spin(self):
        self.mode = self.SPIN
        self._spin_accum = 0.0

    def _freeze_avoidance(self):
        """도달 정지 진입 시 회피엔진 내부 상태를 초기화.
        STOP 피봇(stop_phase) 잔류와 방향 히스테리시스를 비워 정지/재개 시
        엉뚱한 피봇·잔여 회전이 끼어드는 것을 막는다."""
        try:
            av._stop_reset()                 # stop_phase=0 등 STOP 상태 리셋
        except Exception:
            setattr(av, "stop_phase", 0)
        setattr(av, "prev_w", 0.0)           # 엔진 측 스무딩 잔류 제거

    def _advance(self):
        """도달 1초 정지 완료 → 다음 색으로. 마지막이면 DONE."""
        if self.idx >= len(COLOR_SEQUENCE) - 1:
            self.mode = self.DONE
            self._say(f"최종 {self.target} 도달 → DONE (완전 정지)")
            return
        self.idx += 1
        self.cam.set_target(self.target)
        self.cam.clear_arrival()
        self._enter_spin()
        self._say(f"다음 목표 → {self.target} (재주행)")

    # ── 매 사이클 결정 ────────────────────────────────────────────────────────
    def step(self, scan, dt):
        now = time.time()
        cam = self.cam.read()

        if self.mode == self.DONE:
            return 0.0, 0.0

        # [문제1] 도달 1초 정지
        if self.mode == self.STOP_HOLD:
            if now - self._hold_t0 >= ARRIVE_STOP_S:
                self._advance()
            return 0.0, 0.0
        if cam["arrived"]:
            self.mode = self.STOP_HOLD
            self._hold_t0 = now
            self._freeze_avoidance()      # 회피엔진 STOP 피봇/방향 상태 초기화(재개 간섭 차단)
            self._say(f"{self.target} 도달 플래그 → 정지 {ARRIVE_STOP_S}s 유지")
            return 0.0, 0.0

        # [문제2-A] TRACK : bearing 정상이면 무조건 추적 우선
        if cam["bearing"] is not None:
            self._say(f"{self.target} 추적(TRACK) bearing={cam['bearing']:+.1f}°")
            self.mode = self.TRACK
            return self._track(cam, scan)

        # 목표 상실 → B/C/RETURN
        if self.mode == self.TRACK:
            self._enter_spin()

        # [문제3] 허허벌판(이탈) → RETURN (목표 안 보일 때만)
        if is_open_field(scan):
            self.mode = self.RETURN
            self._say("허허벌판 감지(이탈) → RETURN(경기장 복귀)")
            return self._return_to_arena(scan)
        elif self.mode == self.RETURN:
            self._say("경기장 복귀 완료 → SPIN 재개")
            self._enter_spin()

        # [문제2-B] SPIN : 제자리 360° 스캔
        if self.mode == self.SPIN:
            self._spin_accum += SPIN_W * dt
            if self._spin_accum >= SPIN_FULL_TURN:
                self.mode = self.WANDER
                self._say(f"{self.target} 360° 스캔 실패 → WANDER(배회 탐색)")
                return self._wander(scan)
            self._say(f"{self.target} 제자리 스캔(SPIN)")
            return 0.0, self._spin_dir * SPIN_W

        # [문제2-C] WANDER : FGM 회피로 빈 공간 전진
        if self.mode == self.WANDER:
            self._say(f"{self.target} 배회 탐색(WANDER)")
            return self._wander(scan)

        # fallback
        self._enter_spin()
        return 0.0, self._spin_dir * SPIN_W

    # ── 상태별 동작 ───────────────────────────────────────────────────────────
    def _track(self, cam, scan):
        v_a, w_a  = av.find_vw_command(scan, 0.0)
        clearness = max(0.0, min(1.0, v_a / FORWARD_SPEED))
        b = cam["bearing"]
        # 엔진/아두이노 규약: +w=좌회전, 카메라 +bearing=목표 오른쪽 → 우회전(w<0) 필요.
        #   따라서 BEARING_TO_W_SIGN = -1. (카메라가 좌우 반전 장착이면 +1 로)
        w_goal = 0.0 if abs(b) < BEARING_DEADBAND \
                 else BEARING_TO_W_SIGN * GOAL_W_GAIN * clearness * math.sin(math.radians(b))
        w = w_a + w_goal
        p = min(1.0, cam["area_frac"] / AREA_FRAC_SLOW)     # 0(멀다)~1(가깝다)
        v = min(v_a, APPROACH_FAR + (APPROACH_NEAR - APPROACH_FAR) * p)
        return v, max(-MAX_W, min(MAX_W, w))

    def _wander(self, scan):
        v_a, w_a = av.find_vw_command(scan, 0.0)            # FGM 식 회피 전진
        return min(WANDER_SPEED, v_a), max(-MAX_W, min(MAX_W, w_a))

    def _return_to_arena(self, scan):
        ang, _ = arena_direction(scan)
        if ang is None:                                     # 아무것도 안 보임 → 회전 탐색
            return 0.0, self._spin_dir * RETURN_SPIN_W
        # 가장 가까운 물체(=경기장)가 +ang(오른쪽)이면 우회전(w<0)으로 향한다.
        #   LiDAR 규약 +각도=오른쪽, +w=좌회전 → copysign 부호를 반전.
        if abs(ang) > RETURN_FACE_TOL:                      # 경기장 쪽으로 정렬
            return 0.0, -math.copysign(RETURN_SPIN_W, ang)
        v_a, w_a = av.find_vw_command(scan, 0.0)            # 경기장 향해 전진(회피 병행)
        return min(RETURN_SPEED, v_a), max(-MAX_W, min(MAX_W, w_a))


# ═══════════════════════════════════════════════════════════════════════════════
# 제어 스레드
# ═══════════════════════════════════════════════════════════════════════════════
def control_worker(arduino, tower: ControlTower, shutdown, t_start):
    period = 1.0 / MAIN_LOOP_HZ
    prev_w = 0.0
    t_prev = time.time()
    last_log = ""
    last_esc = 0.0          # ESC keepalive 타이머(0=즉시 1회 전송)

    while not shutdown.is_set():
        if time.time() - t_start >= MATCH_TIME_LIMIT_S:
            print("[SAFETY] 시간 제한 → 정지")
            arduino.write(b"0.00 0.00\n")
            tower.mode = ControlTower.DONE

        av.read_arduino(arduino)
        with av._scan_lock:
            scan = [(a, d) for a, d in av._latest_scan if d > 0]

        now = time.time(); dt = now - t_prev; t_prev = now

        # 헤딩가드 상시 해제(자유 360° 회전) — ESC 주기적 재전송
        if now - last_esc >= ESC_KEEPALIVE_S:
            arduino.write(b"ESC\n")
            last_esc = now

        v, w = tower.step(scan, dt)

        # 도달 정지/완주 정지: 스무딩 우회 → 즉시·완전 0 (잔여 회전 차단)
        if tower.mode in (ControlTower.STOP_HOLD, ControlTower.DONE):
            prev_w = 0.0                                  # 재개 시 잔여 회전 carry 방지
            arduino.write(b"0.00 0.00\n")
        else:
            w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w  # 회피엔진과 동일 스무딩
            prev_w = w
            arduino.write(f"{v:.2f} {w:.2f}\n".encode())

        tag = f"{tower.mode}/{tower.target}"
        if tag + f"{v:.1f}{w:.1f}" != last_log:
            print(f"[SEND] {tag:18s} v={v:.2f} w={w:+.2f}")
            last_log = tag + f"{v:.1f}{w:.1f}"

        time.sleep(period)


# ═══════════════════════════════════════════════════════════════════════════════
# 메인
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    print("=" * 70)
    print(" backup.py — 색상영역 추적 제어 타워  RED → YELLOW → BLUE")
    print(f"  TRACK/SPIN/WANDER/RETURN | 도달정지 {ARRIVE_STOP_S}s | 시간제한 {MATCH_TIME_LIMIT_S}s")
    print(f"  허허벌판: nearest≥{OPEN_NEAR_DIST}mm & front_avg≥{OPEN_FIELD_DIST}mm")
    print("=" * 70)

    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(3)        # 아두이노 부팅/EEPROM 로드 대기(미캘리브레이션 시 더 길 수 있음)
    arduino.write(b"ESC\n"); time.sleep(0.1)   # heading 리셋 + 헤딩가드 해제(Serial1에서 R은 무시됨)
    lidar.write(bytes([0xA5, 0x40])); time.sleep(1)
    lidar.write(bytes([0xA5, 0x20])); lidar.read(7)

    cam = CameraTracker(); cam.start()
    tower    = ControlTower(cam)
    shutdown = av._shutdown; shutdown.clear()
    t_start  = time.time()

    t_lidar = threading.Thread(target=av._lidar_reader, args=(lidar,), daemon=True, name="lidar")
    t_ctrl  = threading.Thread(target=control_worker,
                               args=(arduino, tower, shutdown, t_start), daemon=True, name="control")
    try:
        t_lidar.start(); t_ctrl.start()
        while not shutdown.is_set():
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n[EXIT] 사용자 중단")
    finally:
        shutdown.set()
        cam.stop()
        t_lidar.join(timeout=2.0); t_ctrl.join(timeout=2.0)
        try:
            lidar.write(bytes([0xA5, 0x25])); time.sleep(0.1); lidar.close()
        except Exception:
            pass
        try:
            arduino.write(b"0.00 0.00\n"); arduino.close()
        except Exception:
            pass
        print("[EXIT] 종료 완료.")


if __name__ == "__main__":
    main()

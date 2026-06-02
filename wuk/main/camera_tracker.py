import cv2
import math
import time
import threading
import numpy as np

# ── 카메라 설정 ─────────────────────────────────────────────────────
CAMERA_INDEX      = 0         # 인식 안 되면 1로 변경 시도
FRAME_W           = 640
FRAME_H           = 480
HFOV_DEG          = 38.6      # ★ 보정 후 화면 가로(_EFF_W=480) 기준 실측값
                               #   f_px=685 실측 → 2×atan(240/685)=38.6°
# 카메라가 90° 회전 마운트된 경우 설정. None=정방향
# CW 회전 마운트 → ROTATE_90_COUNTERCLOCKWISE, CCW 회전 마운트 → ROTATE_90_CLOCKWISE
FRAME_ROTATE      = cv2.ROTATE_90_COUNTERCLOCKWISE

# 회전 후 실효 해상도 (bearing·도착 판정에 사용)
if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    _EFF_W, _EFF_H = FRAME_H, FRAME_W
else:
    _EFF_W, _EFF_H = FRAME_W, FRAME_H

# ── 도착 판정 ────────────────────────────────────────────────────────
ARRIVE_HOLD_SEC   = 1.2       # 연속 감지 유지 시간 (sec), 1초 인정 기준보다 0.2s 여유
ARRIVE_ROI_BOTTOM = 0.1       # 하단 ROI 비율 (회전 후 화면 세로의 하단 10%)
ARRIVE_ROI_PEAK   = 0.7       # ROI 점유율이 이 값 이상 → "꽉 참" 표시
ARRIVE_ROI_DROP   = 0.5       # peaked 후 이 값 미만으로 떨어지면 도착 판정
USE_ROI_ARRIVE    = 0         # 1=ROI peaked→drop 도착 판정 활성 / 0=비활성 (오도메트리만 사용)

# ── 근접 접근 제어 ────────────────────────────────────────────────────────
CLOSE_ENTER_MM     = 400.0    # 이 거리(mm) 이내로 들어오면 CLOSE 모드 전환
CAM_HEIGHT_MM      = 430.0    # ★ 카메라 ~ 바닥(색지) 수직 높이 (mm) 실측 필요
                               #   = 바퀴 반지름 + 바퀴축~카메라 높이(500mm)
CAM_TILT_DEG       = 34.5    # 역산값: actual=500mm, est=610mm, delta_v=0 → atan(420/610)
                               #   수평=0°, 아래로 내려다볼수록 +
CAM_POLAR_EPSILON  = 0.05     # 원근 보정 분모 하한 (0=하단끝 ±90° 폭발 방지)
USE_CLIPPING_GUARD = False    # True: 클리핑 시 bearing 갱신 중단 / False: 항상 갱신
CLOSE_BEARING_SCALE = 0.8212    # ★ calibrate_bearing.py 로 구한 보정 배율 (1.0=보정 없음)

# ── HSV 색상 범위 (OpenCV: H[0-179], S[0-255], V[0-255]) ─────────────
# 실내 조명 조건에서 반드시 튜닝 필요
COLOR_RANGES = {
    'RED': [
        ((146, 100, 80), (179, 255, 255)),   # 실측값
    ],
    'YELLOW': [
        ((0, 78, 114), (44, 162, 255)),   # 실측값
    ],
    'BLUE': [
        ((79, 116, 114), (119, 162, 255)),   # 실측값
    ],
}

# ── 미션 순서 ────────────────────────────────────────────────────────
MISSION_ORDER = ['RED', 'YELLOW', 'BLUE']

# ── 디버그 ───────────────────────────────────────────────────────────
DEBUG_CAMERA  = 0     # 카메라 감지 로그 (0=끔, 1=켬)
SHOW_FRAME    = 1     # imshow 디버그 창 표시 (0=끔, 1=켬, VNC/모니터 필요)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 공유 상태 (모두 _lock 안에서 접근)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
_lock                = threading.Lock()
_target_bearing      = None    # float(deg) 또는 None (미감지)
_color_detected      = False
_mission_idx         = 0       # 0=RED, 1=YELLOW, 2=BLUE, 3=DONE
_dwell_start         = None    # 도착 판정 시작 시각 (time.time())
_dwelling            = False   # True 동안 모터 정지
_done                = False   # BLUE 완료 → 영구 정지
_shutdown            = threading.Event()
_arrival_signal      = threading.Event()   # 외부(오도메트리 등)에서 도착 신호
_roi_peaked          = False   # 카메라 스레드 전용: ROI 점유율이 peak를 찍었는지
_close               = False   # CLOSE 모드 (blob 크기 > 임계)
_last_stable_bearing = 0.0     # 클리핑 전 마지막 유효 bearing (deg)
_last_close_bearing  = 0.0     # CLOSE 진입 시 원근 보정 bearing (deg)
_last_cy             = None    # 마지막 centroid y (기하 거리 추정용)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 내부 함수
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _detect_color(frame, color_name):
    """
    frame에서 color_name 색을 검출.
    반환: (centroid, area, clipped_l, clipped_r)
      centroid=(cx,cy) 또는 None, area=float
      clipped_l/r: blob이 좌/우 프레임 경계에 닿으면 True
    """
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (lo, hi) in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0, False, False

    largest = max(contours, key=cv2.contourArea)
    area    = cv2.contourArea(largest)
    if area < 500:
        return None, 0.0, False, False

    M = cv2.moments(largest)
    if M['m00'] == 0:
        return None, 0.0, False, False

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    bx, _, bw, _ = cv2.boundingRect(largest)
    clipped_l = (bx <= 1)
    clipped_r = (bx + bw >= _EFF_W - 1)

    return (cx, cy), area, clipped_l, clipped_r


def _to_bearing_seek(cx):
    """SEEK 모드: atan2 정확 모델 — cx만 사용, 거리 무관."""
    f_px = (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
    return -math.degrees(math.atan2(cx - _EFF_W / 2.0, f_px))


def _to_bearing_close(cx, cy):
    """
    CLOSE 모드: 원근 보정 bearing.
    화면 하단(가까울수록) cy가 커지면 같은 cx 오프셋에서도 더 큰 각도 반환.
    bearing = atan2(lateral, forward)
      lateral = (cx-cx0) / f_px   (정규화 수평)
      forward = (_EFF_H-cy)/_EFF_H + epsilon  (0=하단/가까움 ~ 1=상단/멀리)
    """
    f_px    = (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
    lateral = (cx - _EFF_W / 2.0) / f_px
    forward = (_EFF_H - cy) / _EFF_H + CAM_POLAR_EPSILON
    return -math.degrees(math.atan2(lateral, forward)) * CLOSE_BEARING_SCALE


def _get_roi_fill(frame, color_name, bottom_ratio=None):
    """하단 ROI 내 목표 색 점유율(0.0~1.0) 반환.
    bottom_ratio 미지정 시 ARRIVE_ROI_BOTTOM 사용."""
    ratio     = bottom_ratio if bottom_ratio is not None else ARRIVE_ROI_BOTTOM
    roi_start = int(_EFF_H * (1.0 - ratio))
    roi       = frame[roi_start:, :]
    roi_total = roi.shape[0] * roi.shape[1]
    if roi_total == 0:
        return 0.0
    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    return cv2.countNonZero(mask) / roi_total


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 카메라 스레드 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _camera_loop():
    global _target_bearing, _color_detected
    global _mission_idx, _dwell_start, _dwelling, _done
    global _close, _last_stable_bearing, _last_close_bearing, _last_cy

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        print("[CAMERA] ERROR: 카메라를 열 수 없습니다. CAMERA_INDEX를 확인하세요.")
        return

    print(f"[CAMERA] 시작 — {FRAME_W}x{FRAME_H}, HFOV={HFOV_DEG}°")

    while not _shutdown.is_set():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.03)
            continue
        if FRAME_ROTATE is not None:
            frame = cv2.rotate(frame, FRAME_ROTATE)

        with _lock:
            idx  = _mission_idx
            done = _done

        if done or idx >= len(MISSION_ORDER):
            with _lock:
                _target_bearing = None
                _color_detected = False
                _dwelling       = True
            time.sleep(0.03)
            continue

        target_color                       = MISSION_ORDER[idx]
        centroid, area, clip_l, clip_r     = _detect_color(frame, target_color)
        roi_fill                        = _get_roi_fill(frame, target_color)

        # ── 근접 / 클리핑 판정 (lock 밖, 카메라 스레드 전용) ──────────────
        global _roi_peaked

        # bearing 계산: SEEK=atan2 / CLOSE=원근보정
        if centroid is not None:
            cx, cy   = centroid
            clipped  = clip_l or clip_r

            # ── 거리 기반 CLOSE 판정 ────────────────────────────────────────
            f_px_v     = (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
            delta_v    = math.degrees(math.atan2(cy - _EFF_H / 2.0, f_px_v))
            depression = CAM_TILT_DEG + delta_v
            cam_dist   = (CAM_HEIGHT_MM / math.tan(math.radians(depression))
                          if depression > 1.0 else 5000.0)
            is_close_now = (cam_dist < CLOSE_ENTER_MM)

            bearing_seek = _to_bearing_seek(cx)
            if not (USE_CLIPPING_GUARD and clipped):
                _last_stable_bearing = bearing_seek

            if is_close_now:
                bearing = _last_stable_bearing if (USE_CLIPPING_GUARD and clipped) \
                          else _to_bearing_close(cx, cy)
                _last_close_bearing = bearing
            else:
                bearing = _last_stable_bearing
            _last_cy = cy
        else:
            bearing      = None
            is_close_now = False
            cam_dist     = 5000.0

        # 도착 판정 (3가지 중 하나라도 충족 시 arrived=True)
        if USE_ROI_ARRIVE:
            if roi_fill >= ARRIVE_ROI_PEAK:
                if not _roi_peaked and DEBUG_CAMERA:
                    print(f"[CAMERA] {target_color} ROI peak! fill={roi_fill:.2f}")
                _roi_peaked = True
            # ① fill 높음 유지 (색지 위 정지)
            # ② peaked 후 drop (색지 통과)
            # ③ 오도메트리 외부 신호
            arrived = (roi_fill >= ARRIVE_ROI_PEAK
                       or (_roi_peaked and roi_fill < ARRIVE_ROI_DROP)
                       or _arrival_signal.is_set())
        else:
            arrived = _arrival_signal.is_set()   # USE_ROI_ARRIVE=0이면 오도메트리만

        with _lock:
            _target_bearing = bearing
            _color_detected = bearing is not None
            _close          = is_close_now

            if DEBUG_CAMERA:
                clip_str = ('L' if clip_l else '') + ('R' if clip_r else '') or '-'
                print(f"[CAMERA] {target_color} area={area:.0f} "
                      f"clip={clip_str} close={is_close_now} "
                      f"bearing={bearing:+.1f}° dist={cam_dist:.0f}mm fill={roi_fill:.2f} "
                      f"peaked={_roi_peaked}" if bearing is not None else
                      f"[CAMERA] {target_color} 미감지 fill={roi_fill:.2f}")

            if arrived:
                if _dwell_start is None:
                    _dwell_start = time.time()
                    if DEBUG_CAMERA:
                        print(f"[CAMERA] {target_color} 도착 감지 시작")
                _dwelling = True

                elapsed = time.time() - _dwell_start
                if elapsed >= ARRIVE_HOLD_SEC:
                    print(f"[CAMERA] {target_color} 완료! → ", end='')
                    _mission_idx         += 1
                    _dwell_start          = None
                    _dwelling             = False
                    _roi_peaked           = False
                    _close                = False
                    _last_stable_bearing  = 0.0
                    _arrival_signal.clear()        # 외부 도착 신호 초기화
                    if _mission_idx >= len(MISSION_ORDER):
                        _done = True
                        print("DONE (완주)")
                    else:
                        print(f"{MISSION_ORDER[_mission_idx]} 탐색 시작")
            else:
                _dwell_start = None
                _dwelling    = False

            _disp_bearing = _target_bearing
            _disp_idx     = _mission_idx
            _disp_done    = _done

        if SHOW_FRAME:
            _disp_color = (MISSION_ORDER[_disp_idx]
                           if not _disp_done and _disp_idx < len(MISSION_ORDER) else 'DONE')
            display = frame.copy()
            cv2.line(display, (_EFF_W // 2, 0), (_EFF_W // 2, _EFF_H), (180, 180, 180), 1)
            if centroid is not None:
                cv2.circle(display, centroid, 14, (0, 255, 0),  3)
                cv2.circle(display, centroid,  3, (0, 255, 0), -1)
            bearing_str = f"{_disp_bearing:+.1f}" if _disp_bearing is not None else "None"
            cv2.putText(display, f"Target:  {_disp_color}",      (10,  35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (  0, 255, 255), 2)
            cv2.putText(display, f"Bearing: {bearing_str} deg",  (10,  75), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (  0, 255,   0), 2)
            cv2.putText(display, f"Area:    {area:.0f} px2",     (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255,   0), 2)
            cv2.imshow('camera_tracker', display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                _shutdown.set()

        time.sleep(0.03)   # ~30fps

    cap.release()
    if SHOW_FRAME:
        cv2.destroyAllWindows()
    print("[CAMERA] 스레드 종료")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Public API
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def start():
    """카메라 스레드 시작. main()에서 호출."""
    t = threading.Thread(target=_camera_loop, daemon=True, name='camera')
    t.start()


def stop():
    """종료 신호. _shutdown.set()으로 루프 탈출."""
    _shutdown.set()


def get_bearing():
    """
    현재 목표 색지의 bearing (deg) 반환.
    감지 안 됨 → None.
    """
    with _lock:
        return _target_bearing


def is_dwelling():
    """
    True: 색지 위 정지 중 또는 완주 완료 → 모터 v=0, w=0.
    False: 주행 가능.
    """
    with _lock:
        return _dwelling


def is_done():
    """True: BLUE 완료 → 영구 정지."""
    with _lock:
        return _done


def is_close():
    """True: blob이 충분히 커서 CLOSE 모드 (원근 보정 bearing + 위치 제어 전환)."""
    with _lock:
        return _close


def get_last_stable_bearing():
    """클리핑 전 마지막 유효 bearing (deg). CLOSE 목표 좌표 계산에 사용."""
    with _lock:
        return _last_stable_bearing


def get_last_close_bearing():
    """CLOSE 모드 원근 보정 bearing (deg). _compute_close_target 용."""
    with _lock:
        return _last_close_bearing


def get_estimated_distance_mm():
    """
    카메라 기하학 기반 거리 추정 (mm).
    공식: d = CAM_HEIGHT_MM / tan(CAM_TILT_DEG + delta_v)
      delta_v: centroid cy → 카메라 광축 기준 수직 편차 각도
      카메라 90° 회전 마운트이므로 수직 방향 f_px = HFOV_DEG 기준으로 계산
    """
    with _lock:
        cy = _last_cy
    if cy is None:
        return 500.0

    # f_px는 렌즈 고유값 — HFOV_DEG가 _EFF_W 기준으로 측정됐으므로 _EFF_W로 계산
    f_px       = (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
    delta_v    = math.degrees(math.atan2(cy - _EFF_H / 2.0, f_px))
    depression = CAM_TILT_DEG + delta_v   # cy 클수록(하단) → depression 커짐 → 가까움

    if depression <= 1.0:
        return 5000.0   # 수평 이상 → 유효 범위 밖 (매우 먼 거리)

    d = CAM_HEIGHT_MM / math.tan(math.radians(depression))
    return max(d, 50.0)   # 최소 50mm 클램프


def signal_arrival():
    """오도메트리 등 외부 시스템이 도착을 알릴 때 호출.
    카메라의 peaked→drop 판정이 막혀 있어도 미션이 진행된다."""
    _arrival_signal.set()


def get_state():
    """현재 미션 상태 문자열 반환. 디버그용."""
    with _lock:
        if _done:
            return 'DONE'
        if _mission_idx < len(MISSION_ORDER):
            return f'SEEK_{MISSION_ORDER[_mission_idx]}'
        return 'DONE'

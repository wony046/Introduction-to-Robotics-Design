import cv2
import math
import time
import threading
import numpy as np

# ── 카메라 설정 ─────────────────────────────────────────────────────
CAMERA_INDEX      = 0
FRAME_W           = 640
FRAME_H           = 480
HFOV_DEG          = 38.6
FRAME_ROTATE      = cv2.ROTATE_90_COUNTERCLOCKWISE

# 회전 후 실효 해상도 (bearing·도착 판정에 사용)
if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    _EFF_W, _EFF_H = FRAME_H, FRAME_W
else:
    _EFF_W, _EFF_H = FRAME_W, FRAME_H

# ── 도착 판정 ────────────────────────────────────────────────────────
ARRIVE_HOLD_SEC   = 1.2
ARRIVE_ROI_BOTTOM = 0.1
ARRIVE_ROI_PEAK   = 0.7
ARRIVE_ROI_DROP   = 0.5
USE_ROI_ARRIVE    = 1

# ── 근접 접근 제어 ────────────────────────────────────────────────────────
CLOSE_ROI_BOTTOM   = 0.5
CLOSE_ROI_FILL     = 0.25
CAM_HEIGHT_MM      = 420.0
CAM_TILT_DEG       = 34.5
CAM_POLAR_EPSILON  = 0.05
SMOOTH_ALPHA       = 1.0

# ── HSV 색상 범위 ─────────────────────────────────────────────────────
COLOR_RANGES = {
    'RED': [
        ((134, 70, 75), (179, 188, 255)),
    ],
    'YELLOW': [
        ((16, 95, 155), (59, 183, 255)),
    ],
    'BLUE': [
        ((64, 46, 138), (125, 160, 247)),
    ],
}

# ── 미션 순서 ────────────────────────────────────────────────────────
MISSION_ORDER = ['RED', 'YELLOW', 'BLUE']

# ── 디버그 ───────────────────────────────────────────────────────────
DEBUG_CAMERA  = 1
SHOW_FRAME    = 0

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 공유 상태 (모두 _lock 안에서 접근)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
_lock                = threading.Lock()
_target_bearing      = None
_color_detected      = False
_mission_idx         = 0
_dwell_start         = None
_dwelling            = False
_done                = False
_shutdown            = threading.Event()
_arrival_signal      = threading.Event()
_roi_peaked          = False
_close               = False
_last_stable_bearing = 0.0
_last_cy             = None
_smooth_cx           = None
_smooth_cy           = None
# ★ 추가: 드웰 타이머 잠금 플래그
# True이면 ROI 일시 하락으로 _dwell_start가 리셋되지 않음
_arrive_locked       = False


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 내부 함수
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _detect_color(frame, color_name):
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
    f_px = (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
    return math.degrees(math.atan2(cx - _EFF_W / 2.0, f_px))


def _to_bearing_close(cx, cy):
    f_px    = (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
    lateral = (cx - _EFF_W / 2.0) / f_px
    forward = (_EFF_H - cy) / _EFF_H + CAM_POLAR_EPSILON
    return math.degrees(math.atan2(lateral, forward))


def _get_roi_fill(frame, color_name, bottom_ratio=None):
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
    global _close, _last_stable_bearing, _last_cy
    global _smooth_cx, _smooth_cy
    global _arrive_locked   # ★ 추가

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

        target_color                    = MISSION_ORDER[idx]
        centroid, area, clip_l, clip_r  = _detect_color(frame, target_color)
        roi_fill                        = _get_roi_fill(frame, target_color)
        close_roi_fill                  = _get_roi_fill(frame, target_color,
                                                        bottom_ratio=CLOSE_ROI_BOTTOM)

        global _roi_peaked
        is_close_now = (close_roi_fill >= CLOSE_ROI_FILL)

        if centroid is not None:
            cx_raw, cy_raw = centroid
            if _smooth_cx is None:
                _smooth_cx, _smooth_cy = float(cx_raw), float(cy_raw)
            else:
                _smooth_cx = SMOOTH_ALPHA * cx_raw + (1.0 - SMOOTH_ALPHA) * _smooth_cx
                _smooth_cy = SMOOTH_ALPHA * cy_raw + (1.0 - SMOOTH_ALPHA) * _smooth_cy
            cx, cy = int(_smooth_cx), int(_smooth_cy)

            clipped = clip_l or clip_r
            if is_close_now:
                if clipped:
                    bearing = _last_stable_bearing
                else:
                    bearing = _to_bearing_close(cx, cy)
            else:
                bearing = _to_bearing_seek(cx)
                if not clipped:
                    _last_stable_bearing = bearing
            _last_cy = cy
        else:
            bearing      = None
            is_close_now = False

        # 도착 판정 (3가지 중 하나라도 충족 시 arrived=True)
        if USE_ROI_ARRIVE:
            if roi_fill >= ARRIVE_ROI_PEAK:
                if not _roi_peaked and DEBUG_CAMERA:
                    print(f"[CAMERA] {target_color} ROI peak! fill={roi_fill:.2f}")
                _roi_peaked = True
            arrived = (roi_fill >= ARRIVE_ROI_PEAK
                       or (_roi_peaked and roi_fill < ARRIVE_ROI_DROP)
                       or _arrival_signal.is_set())
        else:
            arrived = _arrival_signal.is_set()

        with _lock:
            _target_bearing = bearing
            _color_detected = bearing is not None
            _close          = is_close_now

            if DEBUG_CAMERA:
                clip_str = ('L' if clip_l else '') + ('R' if clip_r else '') or '-'
                if bearing is not None:
                    print(f"[CAMERA] {target_color} area={area:.0f} "
                          f"clip={clip_str} close={is_close_now} "
                          f"bearing={bearing:+.1f}° fill={roi_fill:.2f} "
                          f"peaked={_roi_peaked}")
                else:
                    print(f"[CAMERA] {target_color} 미감지 fill={roi_fill:.2f}")

            # ── 도착/드웰 처리 ──────────────────────────────────────────────
            # ★ 수정: _arrive_locked가 True이면 ROI 임시 하락으로 타이머가
            #         리셋되지 않도록 보호. arrived가 한 번이라도 True가 된
            #         이후로는 1.2초가 완료될 때까지 dwelling 상태를 유지.
            if arrived or _arrive_locked:
                if _dwell_start is None:
                    _dwell_start   = time.time()
                    _arrive_locked = True   # 타이머 시작과 동시에 잠금
                    if DEBUG_CAMERA:
                        print(f"[CAMERA] {target_color} 도착 감지 시작 (locked)")
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
                    _smooth_cx            = None
                    _smooth_cy            = None
                    _arrival_signal.clear()
                    _arrive_locked        = False   # ★ 다음 미션을 위해 잠금 해제
                    if _mission_idx >= len(MISSION_ORDER):
                        _done = True
                        print("DONE (완주)")
                    else:
                        print(f"{MISSION_ORDER[_mission_idx]} 탐색 시작")
            else:
                # _arrive_locked=False이고 arrived=False일 때만 타이머 리셋
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

        time.sleep(0.03)

    cap.release()
    if SHOW_FRAME:
        cv2.destroyAllWindows()
    print("[CAMERA] 스레드 종료")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Public API
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def start():
    t = threading.Thread(target=_camera_loop, daemon=True, name='camera')
    t.start()


def stop():
    _shutdown.set()


def get_bearing():
    with _lock:
        return _target_bearing


def is_dwelling():
    with _lock:
        return _dwelling


def is_done():
    with _lock:
        return _done


def is_close():
    with _lock:
        return _close


def get_last_stable_bearing():
    with _lock:
        return _last_stable_bearing


def get_estimated_distance_mm():
    with _lock:
        cy = _last_cy
    if cy is None:
        return 500.0

    f_px       = (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
    delta_v    = math.degrees(math.atan2(cy - _EFF_H / 2.0, f_px))
    depression = CAM_TILT_DEG + delta_v

    if depression <= 1.0:
        return 5000.0

    d = CAM_HEIGHT_MM / math.tan(math.radians(depression))
    return max(d, 50.0)


def signal_arrival():
    _arrival_signal.set()


def get_state():
    with _lock:
        if _done:
            return 'DONE'
        if _mission_idx < len(MISSION_ORDER):
            return f'SEEK_{MISSION_ORDER[_mission_idx]}'
        return 'DONE'

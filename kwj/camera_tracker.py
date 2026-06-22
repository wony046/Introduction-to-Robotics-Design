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

if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    _EFF_W, _EFF_H = FRAME_H, FRAME_W
else:
    _EFF_W, _EFF_H = FRAME_W, FRAME_H

# ── 도착 판정 및 제어 파라미터 ───────────────────────────────────────
ARRIVE_HOLD_SEC   = 1.2       
ARRIVE_ROI_BOTTOM = 0.1       
ARRIVE_ROI_PEAK   = 0.7       
ARRIVE_ROI_DROP   = 0.5       
USE_ROI_ARRIVE    = 0         

CLOSE_ENTER_MM     = 450.0    
CAM_HEIGHT_MM      = 590.0    
CAM_TILT_DEG       = 40.4     
CAM_POLAR_EPSILON  = 0.05     
USE_CLIPPING_GUARD = False    
CLOSE_BEARING_SCALE = 0.8212  
STOP_EARLY_MM      = 50.0     

# ── ★ 전달받은 최종 다중 영역 LAB 색상 범위 ─────────────────────────
COLOR_PARAMS = {
    'RED': [
        {'a': 180, 'b': 160, 'tol': 35},
    ],
    'YELLOW': [
        {'a': 127, 'b': 177, 'tol': 20},
    ],
    'BLUE': [
        {'a': 111, 'b': 80, 'tol': 26},
        {'a': 123, 'b': 79, 'tol': 25},
    ],
}
L_MIN = 30

COLOR_RANGES = {}
for color in ['RED', 'YELLOW', 'BLUE']:
    COLOR_RANGES[color] = []
    for p in COLOR_PARAMS[color]:
        lower = (L_MIN, max(0, p['a'] - p['tol']), max(0, p['b'] - p['tol']))
        upper = (255,   min(255, p['a'] + p['tol']), min(255, p['b'] + p['tol']))
        COLOR_RANGES[color].append((lower, upper))

_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
MISSION_ORDER = ['RED', 'YELLOW', 'BLUE']

SHOW_FRAME    = 0     

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 전역 상태 변수
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
_last_close_bearing  = 0.0     
_last_cy             = None    
_mission_changed     = False   

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 내부 함수
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def _get_f_px():
    return (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))

def _to_lab(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = _clahe.apply(l)
    return cv2.merge([l, a, b])

def _detect_color(frame, color_name):
    lab  = _to_lab(frame)
    mask = np.zeros(lab.shape[:2], dtype=np.uint8)
    
    # 여러 개의 영역(범위)을 하나의 마스크로 합침
    for (lo, hi) in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(lab, np.array(lo), np.array(hi))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
    return -math.degrees(math.atan2(cx - _EFF_W / 2.0, _get_f_px()))

def _to_bearing_close(cx, cy):
    lateral = (cx - _EFF_W / 2.0) / _get_f_px()
    forward = (_EFF_H - cy) / _EFF_H + CAM_POLAR_EPSILON
    return -math.degrees(math.atan2(lateral, forward)) * CLOSE_BEARING_SCALE

def _get_roi_fill(frame, color_name, bottom_ratio=None):
    ratio     = bottom_ratio if bottom_ratio is not None else ARRIVE_ROI_BOTTOM
    roi_start = int(_EFF_H * (1.0 - ratio))
    roi       = frame[roi_start:, :]
    roi_total = roi.shape[0] * roi.shape[1]
    if roi_total == 0: return 0.0
    lab  = _to_lab(roi)
    mask = np.zeros(lab.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(lab, np.array(lo), np.array(hi))
    return cv2.countNonZero(mask) / roi_total

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 카메라 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def _camera_loop():
    global _target_bearing, _color_detected
    global _mission_idx, _dwell_start, _dwelling, _done
    global _close, _last_stable_bearing, _last_close_bearing, _last_cy, _mission_changed

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        print("[CAMERA] ERROR: 카메라를 열 수 없습니다.")
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

        global _roi_peaked

        if centroid is not None:
            cx, cy   = centroid
            clipped  = clip_l or clip_r

            delta_v    = math.degrees(math.atan2(cy - _EFF_H / 2.0, _get_f_px()))
            depression = CAM_TILT_DEG + delta_v
            
            if depression > 1.0:
                raw_dist = CAM_HEIGHT_MM / math.tan(math.radians(depression))
                cam_dist = max(raw_dist - STOP_EARLY_MM, 50.0) 
            else:
                cam_dist = 5000.0
                
            is_close_now = (cam_dist < CLOSE_ENTER_MM)

            bearing_seek = _to_bearing_seek(cx)
            if not (USE_CLIPPING_GUARD and clipped):
                _last_stable_bearing = bearing_seek

            if is_close_now:
                bearing = _last_stable_bearing if (USE_CLIPPING_GUARD and clipped) else _to_bearing_close(cx, cy)
                _last_close_bearing = bearing
            else:
                bearing = _last_stable_bearing
            _last_cy = cy
        else:
            bearing      = None
            is_close_now = False
            cam_dist     = 5000.0

        if USE_ROI_ARRIVE:
            if roi_fill >= ARRIVE_ROI_PEAK:
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

            if arrived:
                if _dwell_start is None:
                    _dwell_start = time.time()
                _dwelling = True

                elapsed = time.time() - _dwell_start
                if elapsed >= ARRIVE_HOLD_SEC:
                    print(f"\n[CAMERA] ★★★ {target_color} 미션 도착 완료! ★★★")
                    _mission_idx         += 1
                    _mission_changed      = True
                    _dwell_start          = None
                    _dwelling             = False
                    _roi_peaked           = False
                    _close                = False
                    _last_stable_bearing  = 0.0
                    _arrival_signal.clear()
                    if _mission_idx >= len(MISSION_ORDER):
                        _done = True
                        print("[CAMERA] DONE (모든 미션 완주)\n")
                    else:
                        print(f"[CAMERA] 다음 타겟: {MISSION_ORDER[_mission_idx]} 탐색 시작\n")
            else:
                _dwell_start = None
                _dwelling    = False

            _disp_bearing = _target_bearing
            _disp_idx     = _mission_idx
            _disp_done    = _done

        if SHOW_FRAME:
            _disp_color = (MISSION_ORDER[_disp_idx] if not _disp_done and _disp_idx < len(MISSION_ORDER) else 'DONE')
            display = frame.copy()
            cv2.line(display, (_EFF_W // 2, 0), (_EFF_W // 2, _EFF_H), (180, 180, 180), 1)
            if centroid is not None:
                cv2.circle(display, centroid, 14, (0, 255, 0),  3)
                cv2.circle(display, centroid,  3, (0, 255, 0), -1)
            cv2.imshow('camera_tracker', display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                _shutdown.set()

        time.sleep(0.03)

    cap.release()
    if SHOW_FRAME: cv2.destroyAllWindows()
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
    with _lock: return _target_bearing

def is_dwelling():
    with _lock: return _dwelling

def is_done():
    with _lock: return _done

def is_close():
    with _lock: return _close

def get_last_stable_bearing():
    with _lock: return _last_stable_bearing

def get_last_close_bearing():
    with _lock: return _last_close_bearing

def get_estimated_distance_mm():
    with _lock:
        cy = _last_cy
    if cy is None: return 500.0
    delta_v    = math.degrees(math.atan2(cy - _EFF_H / 2.0, _get_f_px()))
    depression = CAM_TILT_DEG + delta_v   
    if depression <= 1.0: return 5000.0   
    raw_dist = CAM_HEIGHT_MM / math.tan(math.radians(depression))
    return max(raw_dist - STOP_EARLY_MM, 50.0)

def signal_arrival():
    _arrival_signal.set()

def get_state():
    with _lock:
        if _done: return 'DONE'
        if _mission_idx < len(MISSION_ORDER): return f'SEEK_{MISSION_ORDER[_mission_idx]}'
        return 'DONE'

def check_mission_changed():
    global _mission_changed
    with _lock:
        if _mission_changed:
            _mission_changed = False
            return True
        return False

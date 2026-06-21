import cv2
import math
import time
import threading
import numpy as np

# ── 카메라 설정 ─────────────────────────────────────────────────────
CAMERA_INDEX      = 0         
FRAME_W           = 640
FRAME_H           = 400       # ★ 해상도 변경 반영 (기존 480 -> 400)
HFOV_DEG          = 38.6      

# 카메라가 90° 회전 마운트된 경우 설정
FRAME_ROTATE      = cv2.ROTATE_90_COUNTERCLOCKWISE

# 회전 후 실효 해상도
if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    _EFF_W, _EFF_H = FRAME_H, FRAME_W  # 회전 후: 400 x 640
else:
    _EFF_W, _EFF_H = FRAME_W, FRAME_H

# ── 도착 판정 ────────────────────────────────────────────────────────
ARRIVE_HOLD_SEC   = 1.2       
ARRIVE_ROI_BOTTOM = 0.1       
ARRIVE_ROI_PEAK   = 0.7       
ARRIVE_ROI_DROP   = 0.5       
USE_ROI_ARRIVE    = 0         

# ── 근접 접근 제어 & 캘리브레이션 ──────────────────────────────────────
CLOSE_ENTER_MM     = 450.0    # ★ 테스트 반영: 400 -> 450으로 진입 시점 앞당김
CAM_HEIGHT_MM      = 590.0    # ★ 실측 높이 반영
CAM_TILT_DEG       = 40.4     # ★ 캘리브레이션 각도 반영
CAM_POLAR_EPSILON  = 0.05     
USE_CLIPPING_GUARD = False    
CLOSE_BEARING_SCALE = 0.8212  

# ★ 새로 추가된 고정 캘리브레이션 값
F_PX_FIXED         = 685.0    # 해상도가 크롭되어도 렌즈 고유의 초점거리를 유지
STOP_EARLY_MM      = 50.0     # 제동 관성 밀림 보상 (절대 좌표 5cm 앞당김)

# ── LAB 색상 범위 ───────────────────────────────────────────────────
COLOR_RANGES = {
    'RED':    [((28,  152, 110), (255, 212, 170))],
    'YELLOW': [((155, 112, 147), (255, 148, 183))],
    'BLUE':   [((18,  129,  87), (255, 143, 101))],
}

_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
MISSION_ORDER = ['RED', 'YELLOW', 'BLUE']

# ── 디버그 ───────────────────────────────────────────────────────────
DEBUG_CAMERA  = 0     
SHOW_FRAME    = 0     

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 공유 상태
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

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 내부 함수
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _to_lab(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = _clahe.apply(l)
    return cv2.merge([l, a, b])

def _detect_color(frame, color_name):
    lab  = _to_lab(frame)
    mask = np.zeros(lab.shape[:2], dtype=np.uint8)
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
    # ★ f_px 수식 대신 F_PX_FIXED 사용 (화면 잘림으로 인한 사시 현상 방지)
    return -math.degrees(math.atan2(cx - _EFF_W / 2.0, F_PX_FIXED))

def _to_bearing_close(cx, cy):
    # ★ F_PX_FIXED 고정 적용
    lateral = (cx - _EFF_W / 2.0) / F_PX_FIXED
    forward = (_EFF_H - cy) / _EFF_H + CAM_POLAR_EPSILON
    return -math.degrees(math.atan2(lateral, forward)) * CLOSE_BEARING_SCALE

def _get_roi_fill(frame, color_name, bottom_ratio=None):
    ratio     = bottom_ratio if bottom_ratio is not None else ARRIVE_ROI_BOTTOM
    roi_start = int(_EFF_H * (1.0 - ratio))
    roi       = frame[roi_start:, :]
    roi_total = roi.shape[0] * roi.shape[1]
    if roi_total == 0:
        return 0.0
    lab  = _to_lab(roi)
    mask = np.zeros(lab.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(lab, np.array(lo), np.array(hi))
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

            # ★ 거리 계산 시 F_PX_FIXED 고정 적용
            delta_v    = math.degrees(math.atan2(cy - _EFF_H / 2.0, F_PX_FIXED))
            depression = CAM_TILT_DEG + delta_v
            
            # 카메라가 CLOSE 모드로 진입할지 여부 판단
            if depression > 1.0:
                raw_dist = CAM_HEIGHT_MM / math.tan(math.radians(depression))
                cam_dist = max(raw_dist - STOP_EARLY_MM, 50.0) # 밀림 보상 미리 적용
            else:
                cam_dist = 5000.0
                
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

            if arrived:
                if _dwell_start is None:
                    _dwell_start = time.time()
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
                    _arrival_signal.clear()
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

def get_last_close_bearing():
    with _lock:
        return _last_close_bearing

def get_estimated_distance_mm():
    """
    카메라 기하학 기반 절대 거리 추정 (mm). 관성 보정치 적용됨.
    """
    with _lock:
        cy = _last_cy
    if cy is None:
        return 500.0

    # ★ 거리 계산 시 F_PX_FIXED 고정 적용
    delta_v    = math.degrees(math.atan2(cy - _EFF_H / 2.0, F_PX_FIXED))
    depression = CAM_TILT_DEG + delta_v   

    if depression <= 1.0:
        return 5000.0   

    raw_dist = CAM_HEIGHT_MM / math.tan(math.radians(depression))
    # ★ 밀림 보상을 위해 미리 목표 좌표를 앞으로 당김
    return max(raw_dist - STOP_EARLY_MM, 50.0)

def signal_arrival():
    _arrival_signal.set()

def get_state():
    with _lock:
        if _done:
            return 'DONE'
        if _mission_idx < len(MISSION_ORDER):
            return f'SEEK_{MISSION_ORDER[_mission_idx]}'
        return 'DONE'

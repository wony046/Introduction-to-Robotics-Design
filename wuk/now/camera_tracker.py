import cv2
import math
import time
import threading
import numpy as np

# ── 카메라 설정 ─────────────────────────────────────────────────────
CAMERA_INDEX      = 0         # 인식 안 되면 1로 변경 시도
FRAME_W           = 1280
FRAME_H           = 720
HFOV_DEG          = 60.0      # ★ 수평 FOV (deg) — 실측 필요, 튜닝값
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
ARRIVE_ROI_FILL   = 0.2       # ROI 내 목표 색 점유율 >= 이 값이면 도착 판정

# ── HSV 색상 범위 (OpenCV: H[0-179], S[0-255], V[0-255]) ─────────────
# 실내 조명 조건에서 반드시 튜닝 필요
COLOR_RANGES = {
    'RED': [
        ((163, 84, 161), (179, 255, 255)),   # 실측값
    ],
    'YELLOW': [
        ((16, 95, 155), (59, 183, 255)),   # 실측값
    ],
    'BLUE': [
        ((64, 46, 138), (125, 160, 247)),   # 실측값
    ],
}

# ── 미션 순서 ────────────────────────────────────────────────────────
MISSION_ORDER = ['RED', 'YELLOW', 'BLUE']

# ── 디버그 ───────────────────────────────────────────────────────────
DEBUG_CAMERA  = True
SHOW_FRAME    = True     # True → imshow 디버그 창 표시 (VNC/모니터 필요)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 공유 상태 (모두 _lock 안에서 접근)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
_lock            = threading.Lock()
_target_bearing  = None    # float(deg) 또는 None (미감지)
_color_detected  = False
_mission_idx     = 0       # 0=RED, 1=YELLOW, 2=BLUE, 3=DONE
_dwell_start     = None    # 도착 판정 시작 시각 (time.time())
_dwelling        = False   # True 동안 모터 정지
_done            = False   # BLUE 완료 → 영구 정지
_shutdown        = threading.Event()


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 내부 함수
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _detect_color(frame, color_name):
    """
    frame에서 color_name 색을 검출.
    반환: (centroid, area) — centroid=(cx, cy) 또는 None, area=float
    가장 큰 blob의 centroid와 면적만 반환.
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
        return None, 0.0

    largest = max(contours, key=cv2.contourArea)
    area    = cv2.contourArea(largest)
    if area < 500:
        return None, 0.0

    M = cv2.moments(largest)
    if M['m00'] == 0:
        return None, 0.0

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return (cx, cy), area


def _to_bearing(cx):
    """
    픽셀 x 좌표 → 로봇 좌표계 각도 (deg).
    화면 중앙=0, 오른쪽=양수, 왼쪽=음수.
    jw_won.py의 스캔 각도 규약과 동일.
    """
    offset = cx - _EFF_W / 2.0
    return offset * (HFOV_DEG / _EFF_W)


def _check_arrival(frame, color_name):
    """
    색지 위에 올라섰는지 판정.
    조건: 회전 후 화면 하단 10% ROI에서 목표 색 점유율 >= ARRIVE_ROI_FILL(90%)
    """
    roi_start = int(_EFF_H * (1.0 - ARRIVE_ROI_BOTTOM))
    roi = frame[roi_start:, :]
    roi_total = roi.shape[0] * roi.shape[1]
    if roi_total == 0:
        return False
    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    return cv2.countNonZero(mask) / roi_total >= ARRIVE_ROI_FILL


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 카메라 스레드 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _camera_loop():
    global _target_bearing, _color_detected
    global _mission_idx, _dwell_start, _dwelling, _done

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

        target_color   = MISSION_ORDER[idx]
        centroid, area = _detect_color(frame, target_color)   # CV 연산은 lock 밖
        arrived        = _check_arrival(frame, target_color)

        with _lock:
            if centroid is not None:
                _target_bearing = _to_bearing(centroid[0])
                _color_detected = True
            else:
                _target_bearing = None
                _color_detected = False

            if arrived:
                if _dwell_start is None:
                    _dwell_start = time.time()
                    if DEBUG_CAMERA:
                        print(f"[CAMERA] {target_color} 도착 감지 시작")
                _dwelling = True

                elapsed = time.time() - _dwell_start
                if DEBUG_CAMERA:
                    print(f"[CAMERA] {target_color} dwell {elapsed:.1f}s "
                          f"area={area:.0f} bearing={_target_bearing}")

                if elapsed >= ARRIVE_HOLD_SEC:
                    print(f"[CAMERA] {target_color} 완료! → ", end='')
                    _mission_idx += 1
                    _dwell_start  = None
                    _dwelling     = False
                    if _mission_idx >= len(MISSION_ORDER):
                        _done = True
                        print("DONE (완주)")
                    else:
                        print(f"{MISSION_ORDER[_mission_idx]} 탐색 시작")
            else:
                if _dwell_start is not None and DEBUG_CAMERA:
                    print(f"[CAMERA] {target_color} 도착 조건 해제 (타이머 리셋)")
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


def get_state():
    """현재 미션 상태 문자열 반환. 디버그용."""
    with _lock:
        if _done:
            return 'DONE'
        if _mission_idx < len(MISSION_ORDER):
            return f'SEEK_{MISSION_ORDER[_mission_idx]}'
        return 'DONE'

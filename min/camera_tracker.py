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
ARRIVE_AREA_MIN   = 12000     # px²: blob 최소 면적 (색지 위 판정 기준) — 튜닝값
ARRIVE_Y_RATIO    = 0.5       # centroid y > H × 이 값 → 화면 하단 = 가까움 — 튜닝값
ARRIVE_HOLD_SEC   = 1.2       # 연속 감지 유지 시간 (sec), 1초 인정 기준보다 0.2s 여유

# ── LAB 색상 범위 (OpenCV LAB: L[0-255], A[0-255 / 128=중립], B[0-255 / 128=중립]) ──
# CLAHE 전처리 후 적용. 실내 조명 조건에서 반드시 튜닝 필요
# A 채널: 128 이상=적색 방향, 128 이하=녹색 방향
# B 채널: 128 이상=황색 방향, 128 이하=청색 방향
COLOR_RANGES = {
    'RED': [
        ((40, 155, 115), (220, 255, 185)),   # A 축 양수(적색)
    ],
    'YELLOW': [
        ((150, 120, 155), (255, 155, 255)),  # L 밝음, B 축 양수(황색)
    ],
    'BLUE': [
        ((30, 100, 0), (190, 140, 105)),     # B 축 음수(청색)
    ],
}

# ── 미션 순서 ────────────────────────────────────────────────────────
MISSION_ORDER = ['RED', 'YELLOW', 'BLUE']

# ── 색 소실 판정 ─────────────────────────────────────────────────────
# 현재 목표 색이 연속 COLOR_LOST_FRAMES 프레임 동안 감지 안 되면 "소실"로 판정
COLOR_LOST_FRAMES = 60   # ~2초 (30fps 기준). 필요 시 튜닝

# ── 디버그 ───────────────────────────────────────────────────────────
DEBUG_CAMERA  = True
SHOW_FRAME    = True     # True → imshow 디버그 창 표시 (VNC/모니터 필요)

# ── CLAHE 전처리 객체 (L 채널 조명 정규화) ───────────────────────────
_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 공유 상태 (모두 _lock 안에서 접근)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
_lock             = threading.Lock()
_target_bearing   = None    # float(deg) 또는 None (미감지)
_color_detected   = False
_mission_idx      = 0       # 0=RED, 1=YELLOW, 2=BLUE, 3=DONE
_dwell_start      = None    # 도착 판정 시작 시각 (time.time())
_dwelling         = False   # True 동안 모터 정지
_done             = False   # BLUE 완료 → 영구 정지
_shutdown         = threading.Event()
_lost_frame_count = 0       # 현재 목표 색 연속 미감지 프레임 수
_color_lost_flag  = False   # True → 목표 색 완전 소실 → 모터 정지 권장


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 내부 함수
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _to_lab(frame):
    """BGR → LAB 변환. L 채널에 CLAHE 적용 후 반환."""
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = _clahe.apply(l)
    return cv2.merge([l, a, b])


def _detect_color(frame, color_name):
    """
    frame에서 color_name 색을 검출.
    반환: (centroid, area) — centroid=(cx, cy) 또는 None, area=float
    가장 큰 blob의 centroid와 면적만 반환.
    """
    lab  = _to_lab(frame)
    mask = np.zeros(lab.shape[:2], dtype=np.uint8)
    for (lo, hi) in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(lab, np.array(lo), np.array(hi))

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


def _check_arrival(centroid, area):
    """
    색지 위에 올라섰는지 판정.
    조건: blob 면적 >= ARRIVE_AREA_MIN AND centroid가 화면 하단
    """
    if centroid is None or area < ARRIVE_AREA_MIN:
        return False
    _, cy = centroid
    return cy > _EFF_H * ARRIVE_Y_RATIO


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 카메라 스레드 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _camera_loop():
    global _target_bearing, _color_detected
    global _mission_idx, _dwell_start, _dwelling, _done
    global _lost_frame_count, _color_lost_flag

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
        arrived        = _check_arrival(centroid, area)

        with _lock:
            if centroid is not None:
                _target_bearing   = _to_bearing(centroid[0])
                _color_detected   = True
                _lost_frame_count = 0
                _color_lost_flag  = False
            else:
                _target_bearing    = None
                _color_detected    = False
                _lost_frame_count += 1
                if _lost_frame_count >= COLOR_LOST_FRAMES:
                    if not _color_lost_flag and DEBUG_CAMERA:
                        print(f"[CAMERA] {target_color} 완전 소실 "
                              f"({COLOR_LOST_FRAMES}프레임) → 정지 신호")
                    _color_lost_flag = True

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
                    _mission_idx      += 1
                    _dwell_start       = None
                    _dwelling          = False
                    _lost_frame_count  = 0     # 다음 색 탐색 시작 시 초기화
                    _color_lost_flag   = False
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


def is_color_lost():
    """
    True: 현재 목표 색이 COLOR_LOST_FRAMES 프레임 이상 연속으로 미감지.
    → 모터 정지 권장. 색이 다시 보이면 자동으로 False로 복귀.
    """
    with _lock:
        return _color_lost_flag


def get_state():
    """현재 미션 상태 문자열 반환. 디버그용."""
    with _lock:
        if _done:
            return 'DONE'
        if _mission_idx < len(MISSION_ORDER):
            return f'SEEK_{MISSION_ORDER[_mission_idx]}'
        return 'DONE'
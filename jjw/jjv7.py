#!/usr/bin/env python3
"""
color_detection.py  —  색상 감지 모듈
======================================
강의 자료(Camera 1·2) 기반으로 작성.
미션 순서: RED → YELLOW → BLUE  (30cm × 30cm 색종이)

실행 모드
---------
  python3 color_detection.py          → 색상 감지 실시간 테스트
  python3 color_detection.py calib    → HSV 캘리브레이션 (트랙바)
  python3 color_detection.py calib red/yellow/blue   → 특정 색 캘리브레이션

라즈베리파이에서 VNC로 접속해 실행하면 화면을 볼 수 있습니다.
"""

import cv2
import numpy as np
import threading
import time
import sys


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 미션 색상 순서
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

COLOR_SEQUENCE = ['red', 'yellow', 'blue']   # ← 시험 당일 확인 후 필요시 변경


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# HSV 색상 범위  (실제 조명 환경에서 캘리브레이션 필수!)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#
# OpenCV HSV 스케일:
#   H (Hue)        :   0 ~ 180   (실제 각도의 절반)
#   S (Saturation) :   0 ~ 255
#   V (Value)      :   0 ~ 255
#
# 강의 자료 핵심 포인트:
#   - RGB보다 HSV가 조명 변화에 강함 (H값은 밝기 변해도 유지)
#   - 빨간색은 Hue 0°/360° 경계에 걸쳐 두 구간 필요
#   - 채도(S)·명도(V) 하한을 적절히 설정해 노이즈 제거
#
# 형식: [(lo_array, hi_array), ...]  — 여러 구간이면 OR 합산

HSV_RANGES = {

    # ──────────────────────────────────────────────
    # 빨간색: Hue 0° 근방 + 180° 근방 두 구간 OR 합산
    # ──────────────────────────────────────────────
    'red': [
        (np.array([  0, 100,  80]), np.array([ 10, 255, 255])),   # 저Hue 빨강
        (np.array([165, 100,  80]), np.array([180, 255, 255])),   # 고Hue 빨강
    ],

    # ──────────────────────────────────────────────
    # 노란색: Hue 15~35 구간  (강의 practice6.py 참고)
    # ──────────────────────────────────────────────
    'yellow': [
        (np.array([ 15, 100, 100]), np.array([ 35, 255, 255])),
    ],

    # ──────────────────────────────────────────────
    # 파란색: Hue 100~130 구간
    # ──────────────────────────────────────────────
    'blue': [
        (np.array([100, 120,  80]), np.array([130, 255, 255])),
    ],
}

# 색상별 BGR 디버그 표시 색상
DEBUG_COLOR_BGR = {
    'red':    (0,   0,   255),
    'yellow': (0,   220, 220),
    'blue':   (255, 100,   0),
}


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 감지 파라미터
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

DETECT_MIN_RATIO  = 0.02   # 이 비율 미만 → "감지 안 됨"
ON_PAPER_RATIO    = 0.60   # 이 비율 이상 → "색종이 위에 있음" 판단
BODY_CUT_RATIO    = 0.75   # 하단 25%는 로봇 본체 → ROI 제외
WRONG_MIN_RATIO   = 0.05   # 잘못된 색 척력 발생 최소 비율
WRONG_MAX_RATIO   = 0.15   # 잘못된 색 척력 포화 비율
REPULSE_GAIN      = 1.5    # 척력 각속도 게인


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 스레드 공유 상태
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

_cam_lock   = threading.Lock()
_latest_cam = {
    'target_detected': False,   # 목표 색 감지 여부
    'target_ratio':    0.0,     # 목표 색 픽셀 비율 (0.0 ~ 1.0)
    'target_offset':   0.0,     # centroid x 오프셋  (-1=좌  0=중앙  +1=우)
    'repulse_w':       0.0,     # 잘못된 색 척력 합산 delta_w
    'blockage_dir':    None,    # 가로막힘 방향: 'left' / 'right' / None
    'debug_frame':     None,    # 디버그 시각화 프레임
}


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 핵심 헬퍼 함수
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def make_mask(hsv_img, color_name):
    """
    HSV 이미지 → 지정 색상의 이진 마스크.

    강의 자료 원리:
      cv2.inRange()로 범위 내 픽셀 → 흰색(255), 나머지 → 검은색(0)
      빨간색처럼 두 구간이 필요하면 cv2.bitwise_or()로 합산

    반환: uint8 마스크
    """
    ranges = HSV_RANGES[color_name]
    mask   = cv2.inRange(hsv_img, ranges[0][0], ranges[0][1])
    for lo, hi in ranges[1:]:                             # 추가 구간 OR 합산
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv_img, lo, hi))
    return mask


def clean_mask(mask, ksize=5):
    """
    형태학적 연산으로 마스크 노이즈 제거 (강의 자료 Morphological Operations 참고).

    OPEN  (침식→팽창): 작은 노이즈 점 제거
    CLOSE (팽창→침식): 색 영역 내부 작은 구멍 메우기
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def get_centroid_offset(mask, frame_w):
    """
    마스크 무게중심(centroid) x좌표 → 프레임 중앙 기준 정규화 오프셋 반환.
      -1.0 = 완전 좌측,  0.0 = 중앙,  +1.0 = 완전 우측
    유효 blob 없으면 0.0 반환.

    강의 자료: cv2.moments()로 무게중심 계산
    """
    M = cv2.moments(mask)
    if M['m00'] == 0:
        return 0.0
    cx = M['m10'] / M['m00']
    return (cx - frame_w / 2.0) / (frame_w / 2.0)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 감지 함수
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def detect_colors(frame, target_color, wrong_colors, max_w=1.8):
    """
    한 프레임에서 목표 색·잘못된 색 감지 후 결과 딕셔너리 반환.

    처리 흐름 (강의 자료 기반):
      ① 가우시안 블러로 노이즈 제거
      ② BGR → HSV 변환 (조명 변화에 강함)
      ③ cv2.inRange()로 색상 마스크 생성
      ④ 형태학적 연산으로 마스크 정제
      ⑤ 픽셀 비율·centroid 계산

    Parameters
    ----------
    frame        : BGR 프레임 (640×480)
    target_color : 현재 목표 색 ('red'/'yellow'/'blue')
    wrong_colors : 피해야 할 색 리스트
    max_w        : 척력 클리핑 최대값

    Returns
    -------
    dict
      target_detected : bool    감지 여부
      target_ratio    : float   목표 색 픽셀 비율 (0~1)
      target_offset   : float   centroid x 오프셋 (-1~+1)
      repulse_w       : float   척력 delta_w
      blockage_dir    : str|None 가로막힘 방향
      annotated_frame : ndarray 디버그 시각화 프레임
    """
    h, w = frame.shape[:2]
    roi_h = int(h * BODY_CUT_RATIO)    # ROI 높이 (하단 로봇 본체 제외)
    roi   = frame[:roi_h, :]           # 상단 75% 영역만 사용
    n_px  = roi_h * w                  # ROI 전체 픽셀 수

    # ① 가우시안 블러 (노이즈 → 엣지 오인식 방지, 강의 자료 Canny 전처리와 동일 개념)
    blurred = cv2.GaussianBlur(roi, (5, 5), 0)

    # ② BGR → HSV (강의 자료: 조명 변화에 강한 HSV 사용 권장)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # ──────────────────────────────────────
    # ③④⑤  목표 색 감지
    # ──────────────────────────────────────
    target_detected = False
    target_ratio    = 0.0
    target_offset   = 0.0
    t_mask          = None

    if target_color in HSV_RANGES:
        t_mask          = make_mask(hsv, target_color)   # ③ 마스크 생성
        t_mask          = clean_mask(t_mask)              # ④ 노이즈 제거
        target_ratio    = cv2.countNonZero(t_mask) / n_px  # ⑤ 비율
        target_detected = target_ratio > DETECT_MIN_RATIO
        if target_detected:
            target_offset = get_centroid_offset(t_mask, w)

    # ──────────────────────────────────────
    # 잘못된 색 척력 계산
    # ──────────────────────────────────────
    repulse_w     = 0.0
    wrong_cx_list = []              # 가로막힘 감지용 centroid x 좌표 모음

    for color in wrong_colors:
        if color not in HSV_RANGES:
            continue

        w_mask = make_mask(hsv, color)
        ratio  = cv2.countNonZero(w_mask) / n_px

        if ratio < WRONG_MIN_RATIO:
            continue                # 너무 작으면 무시

        # 비율 → 강도 선형 보간 (WRONG_MIN ~ WRONG_MAX 구간에서 0→1)
        strength = min(1.0, (ratio - WRONG_MIN_RATIO) /
                            (WRONG_MAX_RATIO - WRONG_MIN_RATIO))

        offset     = get_centroid_offset(w_mask, w)
        repulse_w -= offset * strength * REPULSE_GAIN  # 잘못된 색 반대 방향으로 밀기

        M = cv2.moments(w_mask)
        if M['m00'] > 0:
            wrong_cx_list.append(M['m10'] / M['m00'])

    repulse_w = max(-max_w, min(max_w, repulse_w))   # 클리핑

    # ──────────────────────────────────────
    # 가로막힘(Blockage) 감지
    # 잘못된 색 centroid가 목표 색 centroid 근처에 있으면 가로막힘
    # ──────────────────────────────────────
    blockage_dir = None
    if target_detected and wrong_cx_list:
        target_cx_px = (target_offset + 1.0) / 2.0 * w
        wrong_cx_px  = sum(wrong_cx_list) / len(wrong_cx_list)
        if abs(wrong_cx_px - target_cx_px) < w * 0.20:   # 프레임 폭 20% 이내
            blockage_dir = 'left' if wrong_cx_px < w / 2 else 'right'

    # ──────────────────────────────────────
    # 디버그 시각화
    # ──────────────────────────────────────
    annotated = _draw_debug(frame.copy(), roi_h, t_mask,
                            target_color, target_ratio,
                            target_offset, target_detected, blockage_dir)

    return {
        'target_detected': target_detected,
        'target_ratio':    target_ratio,
        'target_offset':   target_offset,
        'repulse_w':       repulse_w,
        'blockage_dir':    blockage_dir,
        'annotated_frame': annotated,
    }


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 디버그 시각화
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _draw_debug(frame, roi_h, t_mask, target_color, ratio,
                offset, detected, blockage_dir):
    """디버그 정보를 프레임에 그려 반환 (VNC 확인용)."""
    h, w = frame.shape[:2]

    # ROI 경계선 (회색 점선 효과)
    cv2.line(frame, (0, roi_h), (w, roi_h), (100, 100, 100), 1)
    cv2.putText(frame, "ROI", (5, roi_h - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

    # 목표 색 마스크 반투명 오버레이
    if t_mask is not None and detected:
        color_bgr = DEBUG_COLOR_BGR.get(target_color, (255, 255, 255))
        overlay   = np.zeros_like(frame[:roi_h])
        overlay[t_mask > 0] = color_bgr
        frame[:roi_h] = cv2.addWeighted(frame[:roi_h], 0.65, overlay, 0.35, 0)

        # centroid 마커
        M = cv2.moments(t_mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy),  3, (0, 255, 0), -1)

        # 중심선 (조향 기준)
        cv2.line(frame, (w // 2, 0), (w // 2, roi_h), (0, 255, 0), 1)

    # 상태 텍스트
    status_txt  = "DETECTED" if detected else "NOT FOUND"
    status_bgr  = (0, 220, 0) if detected else (0, 0, 220)
    cv2.putText(frame, f"Target: {target_color.upper()}  [{status_txt}]",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_bgr, 2)
    cv2.putText(frame, f"Ratio: {ratio:.3f}  Offset: {offset:+.3f}",
                (10, 53), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1)

    # ON_PAPER 상태
    if ratio >= ON_PAPER_RATIO:
        cv2.putText(frame, "▶ ON PAPER!",
                    (10, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    # 가로막힘 경고
    if blockage_dir:
        cv2.putText(frame, f"BLOCKED: {blockage_dir.upper()}",
                    (10, 103), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 140, 255), 2)

    # 비율 게이지 바
    bar_full = w - 20
    bar_fill = int(min(ratio / ON_PAPER_RATIO, 1.0) * bar_full)
    cv2.rectangle(frame, (10, h - 22), (w - 10, h - 10), (40,  40,  40), -1)
    cv2.rectangle(frame, (10, h - 22), (10 + bar_fill, h - 10),
                  (0, 200, 0) if ratio < ON_PAPER_RATIO else (0, 255, 255), -1)
    cv2.putText(frame, f"{ratio * 100:.1f}% / {ON_PAPER_RATIO*100:.0f}%",
                (12, h - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)

    return frame


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 카메라 스레드 (로봇 메인 코드와 연동)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def camera_reader(fsm, shutdown_event, show_window=False):
    """
    카메라 전용 스레드 (~30 fps).

    Parameters
    ----------
    fsm            : MissionFSM 인스턴스  (target_color, wrong_colors 참조)
    shutdown_event : threading.Event()  종료 신호
    show_window    : True → VNC에서 실시간 화면 표시 (디버깅용)
    """
    # ── 카메라 초기화 ────────────────────────────────────
    cap = cv2.VideoCapture(0)           # 인덱스 0이 기본값 (ls /dev/video* 로 확인)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS,           30)

    if not cap.isOpened():
        print("[CAM] ❌ 카메라를 열 수 없습니다.")
        return

    print("[CAM] ✅ 카메라 스레드 시작")

    while not shutdown_event.is_set():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        # FSM 상태에서 현재 목표·척력 색상 읽기 (GIL이 단순 속성 읽기 보호)
        target = fsm.target_color   # None이면 미션 완료
        wrongs = fsm.wrong_colors

        if target is None:
            time.sleep(0.1)
            continue

        # 색상 감지 실행
        result = detect_colors(frame, target, wrongs)

        # 공유 변수 갱신 (lock 사용)
        with _cam_lock:
            _latest_cam['target_detected'] = result['target_detected']
            _latest_cam['target_ratio']    = result['target_ratio']
            _latest_cam['target_offset']   = result['target_offset']
            _latest_cam['repulse_w']       = result['repulse_w']
            _latest_cam['blockage_dir']    = result['blockage_dir']
            _latest_cam['debug_frame']     = result['annotated_frame']

        # VNC 디버그 창
        if show_window:
            cv2.imshow("Color Detection", result['annotated_frame'])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                shutdown_event.set()
                break

        time.sleep(1 / 30)

    cap.release()
    if show_window:
        cv2.destroyAllWindows()
    print("[CAM] 카메라 스레드 종료")


def get_cam_data():
    """
    최신 카메라 결과를 복사본으로 반환 (스레드 안전).
    모터 제어 스레드에서 호출해 사용.
    """
    with _cam_lock:
        return {k: v for k, v in _latest_cam.items() if k != 'debug_frame'}


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# HSV 캘리브레이션 도구  (단독 실행)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def run_calibration(color_hint='yellow'):
    """
    트랙바로 HSV 범위를 실시간 조정하는 캘리브레이션 도구.
    (강의 자료 색상 공간 실습 확장판)

    사용법:
      python3 color_detection.py calib          → 노란색 기본값
      python3 color_detection.py calib red      → 빨간색 초기값
      python3 color_detection.py calib yellow   → 노란색 초기값
      python3 color_detection.py calib blue     → 파란색 초기값

    'q'키 누르면 현재 HSV 범위를 터미널에 출력 후 종료.
    """
    # 색상별 초기값 (현재 HSV_RANGES 기준)
    init = {
        'red':    {'H_lo':   0, 'H_hi':  10, 'S_lo': 100, 'S_hi': 255, 'V_lo':  80, 'V_hi': 255},
        'yellow': {'H_lo':  15, 'H_hi':  35, 'S_lo': 100, 'S_hi': 255, 'V_lo': 100, 'V_hi': 255},
        'blue':   {'H_lo': 100, 'H_hi': 130, 'S_lo': 120, 'S_hi': 255, 'V_lo':  80, 'V_hi': 255},
    }.get(color_hint, {'H_lo': 15, 'H_hi': 35, 'S_lo': 100, 'S_hi': 255, 'V_lo': 100, 'V_hi': 255})

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    WIN = "HSV Calibration"
    cv2.namedWindow(WIN)

    # 트랙바 생성
    for name, val, max_val in [
        ("H_low",  init['H_lo'],  180),
        ("H_high", init['H_hi'],  180),
        ("S_low",  init['S_lo'],  255),
        ("S_high", init['S_hi'],  255),
        ("V_low",  init['V_lo'],  255),
        ("V_high", init['V_hi'],  255),
    ]:
        cv2.createTrackbar(name, WIN, val, max_val, lambda x: None)

    print("=" * 55)
    print(f" HSV 캘리브레이션 모드  [{color_hint.upper()}]")
    print("  트랙바로 H/S/V 범위를 조정하세요.")
    print("  목표 색 종이가 마스크 창에서 흰색으로 나타나면 완료.")
    print("  'q' → 현재 값 출력 후 종료")
    print("=" * 55)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h_lo = cv2.getTrackbarPos("H_low",  WIN)
        h_hi = cv2.getTrackbarPos("H_high", WIN)
        s_lo = cv2.getTrackbarPos("S_low",  WIN)
        s_hi = cv2.getTrackbarPos("S_high", WIN)
        v_lo = cv2.getTrackbarPos("V_low",  WIN)
        v_hi = cv2.getTrackbarPos("V_high", WIN)

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        lo   = np.array([h_lo, s_lo, v_lo])
        hi   = np.array([h_hi, s_hi, v_hi])
        mask = cv2.inRange(hsv, lo, hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 비율 계산
        ratio = cv2.countNonZero(mask) / (frame.shape[0] * frame.shape[1])

        # 마스크 적용 결과 (컬러)
        result_img = cv2.bitwise_and(frame, frame, mask=mask)

        # 정보 텍스트
        info = (f"H:[{h_lo},{h_hi}]  S:[{s_lo},{s_hi}]  "
                f"V:[{v_lo},{v_hi}]   ratio={ratio:.3f}")
        cv2.putText(frame, info, (5, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, "Press 'q' to save & quit",
                    (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

        cv2.imshow(WIN,        frame)       # 원본 + 정보
        cv2.imshow("Mask",     mask)        # 이진 마스크
        cv2.imshow("Result",   result_img)  # 마스크 적용 결과

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(f"\n── [{color_hint.upper()}] 캘리브레이션 결과 ──")
            print(f"  lower = np.array([{h_lo:3d}, {s_lo:3d}, {v_lo:3d}])")
            print(f"  upper = np.array([{h_hi:3d}, {s_hi:3d}, {v_hi:3d}])")
            print()
            print("  HSV_RANGES 적용 예시:")
            print(f"  '{color_hint}': [")
            print(f"      (np.array([{h_lo:3d}, {s_lo:3d}, {v_lo:3d}]),")
            print(f"       np.array([{h_hi:3d}, {s_hi:3d}, {v_hi:3d}])),")
            print(f"  ],")
            break

    cap.release()
    cv2.destroyAllWindows()


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 실시간 감지 테스트  (단독 실행)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def run_detection_test():
    """
    VNC에서 실시간 색상 감지 결과를 확인하는 테스트.
    'n'키로 다음 목표 색으로 전환, 'q'키로 종료.
    """
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 색상별 척력 대상 매핑
    wrong_map = {
        'red':    ['yellow', 'blue'],
        'yellow': ['red',    'blue'],
        'blue':   ['red',    'yellow'],
    }

    seq_idx = 0
    print("=" * 55)
    print(" 색상 감지 실시간 테스트")
    print(f" 순서: {' → '.join(c.upper() for c in COLOR_SEQUENCE)}")
    print(" 'n' : 다음 색상으로 전환   'q' : 종료")
    print("=" * 55)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        target = COLOR_SEQUENCE[seq_idx]
        wrongs = wrong_map[target]
        result = detect_colors(frame, target, wrongs)

        # 터미널 로그 (한 줄 갱신)
        print(
            f"\r[{target.upper():<6}] "
            f"det={str(result['target_detected']):<5} "
            f"ratio={result['target_ratio']:.3f}  "
            f"offset={result['target_offset']:+.3f}  "
            f"repulse={result['repulse_w']:+.3f}  "
            f"block={str(result['blockage_dir']):<6}  "
            f"{'◀ ON PAPER!' if result['target_ratio'] >= ON_PAPER_RATIO else ''}",
            end='', flush=True
        )

        cv2.imshow("Color Detection Test", result['annotated_frame'])
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print()
            break
        elif key == ord('n'):
            seq_idx = (seq_idx + 1) % len(COLOR_SEQUENCE)
            print(f"\n→ 목표 변경: {COLOR_SEQUENCE[seq_idx].upper()}")

    cap.release()
    cv2.destroyAllWindows()


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 진입점
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "calib":
        color_hint = sys.argv[2] if len(sys.argv) >= 3 else 'yellow'
        if color_hint not in HSV_RANGES:
            print(f"사용 가능한 색상: {list(HSV_RANGES.keys())}")
            sys.exit(1)
        run_calibration(color_hint)
    else:
        run_detection_test()

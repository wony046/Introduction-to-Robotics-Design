"""
Bearing 시각화 도구
─────────────────────────────────────────────────────
SEEK bearing vs CLOSE bearing 비교 + 각도 다이어그램

조작:
  1 / 2 / 3  : 색상 선택 (RED / YELLOW / BLUE)
  q          : 종료
"""

import cv2
import math
import numpy as np

# ── camera_tracker.py 와 동일한 상수 ──────────────────────────────────────────
CAMERA_INDEX     = 0
FRAME_W          = 640
FRAME_H          = 480
FRAME_ROTATE     = cv2.ROTATE_90_COUNTERCLOCKWISE

if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    _EFF_W, _EFF_H = FRAME_H, FRAME_W   # 480 × 640
else:
    _EFF_W, _EFF_H = FRAME_W, FRAME_H

HFOV_DEG          = 38.6
CAM_POLAR_EPSILON = 0.05
CLOSE_ROI_BOTTOM  = 0.3
CLOSE_ROI_FILL    = 0.5

COLOR_RANGES = {
    'RED':    [((170, 120, 90),  (179, 220, 255))],
    'YELLOW': [((16,  95, 155),  (59,  183, 255))],
    'BLUE':   [((64,  46, 138),  (125, 160, 247))],
}
COLOR_KEYS = ['RED', 'YELLOW', 'BLUE']

# ── 색상 상수 (BGR) ───────────────────────────────────────────────────────────
C_WHITE  = (255, 255, 255)
C_BLACK  = (0,   0,   0  )
C_GRAY   = (160, 160, 160)
C_GREEN  = (50,  230, 50 )
C_CYAN   = (255, 220, 0  )
C_ORANGE = (0,   165, 255)
C_RED    = (60,  60,  240)
C_YELLOW_B = (0, 220, 220)
C_BLUE_B   = (220, 100, 50)

DISPLAY_SCALE  = 0.65   # 카메라 패널 축소 비율
DIAG_W         = 420    # 다이어그램 패널 너비
DIAG_H         = int(_EFF_H * DISPLAY_SCALE)


# ── 유틸 ────────────────────────────────────────────────────────────────────

def _put(img, text, pos, scale=0.70, color=C_WHITE, thickness=1):
    x, y = pos
    cv2.putText(img, text, (x+1, y+1), cv2.FONT_HERSHEY_SIMPLEX,
                scale, C_BLACK, thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, (x,   y  ), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color,   thickness,     cv2.LINE_AA)


def _f_px():
    return (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))


def _bearing_seek(cx):
    f = _f_px()
    return math.degrees(math.atan2(cx - _EFF_W / 2.0, f))


def _bearing_close(cx, cy):
    f       = _f_px()
    lateral = (cx - _EFF_W / 2.0) / f
    forward = (_EFF_H - cy) / _EFF_H + CAM_POLAR_EPSILON
    return math.degrees(math.atan2(lateral, forward)), lateral, forward


def _detect(frame, color_name):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0
    largest = max(contours, key=cv2.contourArea)
    area    = cv2.contourArea(largest)
    if area < 300:
        return None, 0.0
    M = cv2.moments(largest)
    if M['m00'] == 0:
        return None, 0.0
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return (cx, cy), area


def _roi_fill(frame, color_name, ratio):
    start = int(_EFF_H * (1.0 - ratio))
    roi   = frame[start:, :]
    total = roi.shape[0] * roi.shape[1]
    if total == 0:
        return 0.0
    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    return cv2.countNonZero(mask) / total


# ── 카메라 패널 그리기 ────────────────────────────────────────────────────────

def _draw_cam_panel(frame, centroid, color_name, is_close, b_seek, b_close_val):
    disp = cv2.resize(frame,
                      (int(_EFF_W * DISPLAY_SCALE), int(_EFF_H * DISPLAY_SCALE)))
    sw, sh = disp.shape[1], disp.shape[0]

    cx_mid_disp = int(_EFF_W / 2 * DISPLAY_SCALE)

    # 중심 수직선
    cv2.line(disp, (cx_mid_disp, 0), (cx_mid_disp, sh), C_GRAY, 1)

    # CLOSE ROI 하단 영역 표시
    roi_y = int(sh * (1.0 - CLOSE_ROI_BOTTOM))
    overlay = disp.copy()
    cv2.rectangle(overlay, (0, roi_y), (sw, sh), (40, 80, 40), -1)
    cv2.addWeighted(overlay, 0.25, disp, 0.75, 0, disp)
    cv2.line(disp, (0, roi_y), (sw, roi_y), (60, 160, 60), 1)
    _put(disp, f"CLOSE ROI ({int(CLOSE_ROI_BOTTOM*100)}%)",
         (4, roi_y - 8), scale=0.50, color=(60, 200, 60))

    if centroid is not None:
        cx, cy = centroid
        dx = int(cx * DISPLAY_SCALE)
        dy = int(cy * DISPLAY_SCALE)

        # centroid 마커
        cv2.circle(disp, (dx, dy), 12, C_GREEN,  2)
        cv2.circle(disp, (dx, dy),  3, C_GREEN, -1)
        _put(disp, f"({cx},{cy})", (dx + 14, dy + 6), scale=0.55, color=C_GREEN)

        # cy 수평선
        cv2.line(disp, (0, dy), (sw, dy), (0, 180, 180), 1)

        # bearing 화살표 (화면 중심에서 centroid 방향으로)
        bearing = b_close_val if is_close else b_seek
        arrow_len = 80
        ax = int(cx_mid_disp + arrow_len * math.sin(math.radians(bearing)))
        ay = int(sh * 0.5   - arrow_len * math.cos(math.radians(bearing)))
        col = C_ORANGE if is_close else C_CYAN
        cv2.arrowedLine(disp, (cx_mid_disp, sh // 2), (ax, ay), col, 2, tipLength=0.25)

    # 모드 표시 (좌상단)
    mode_str = "[ CLOSE ]" if is_close else "[ SEEK  ]"
    mode_col = C_ORANGE if is_close else C_CYAN
    _put(disp, mode_str, (6, 26), scale=0.80, color=mode_col, thickness=2)

    # 색상 표시 (우상단)
    _put(disp, color_name, (sw - 100, 26), scale=0.75, color=C_WHITE)

    return disp


# ── 다이어그램 패널 ──────────────────────────────────────────────────────────

def _draw_diag_panel(centroid, is_close, b_seek, b_close_val, lateral, forward,
                     roi_fill_close, color_name):
    img = np.zeros((DIAG_H, DIAG_W, 3), dtype=np.uint8)
    img[:] = (25, 25, 25)

    # ── 상단: bearing 다이어그램 ───────────────────────────────────────────
    cx_d = DIAG_W // 2
    cy_d = int(DIAG_H * 0.40)
    r    = 110

    # 원
    cv2.circle(img, (cx_d, cy_d), r, (60, 60, 60), 1)
    cv2.circle(img, (cx_d, cy_d), 3, C_GRAY, -1)

    # 각도 눈금 (-90° ~ +90°)
    for deg in range(-90, 91, 30):
        ex = int(cx_d + r * math.sin(math.radians(deg)))
        ey = int(cy_d - r * math.cos(math.radians(deg)))
        cv2.line(img, (cx_d, cy_d), (ex, ey), (45, 45, 45), 1)
        if deg % 30 == 0 and deg != 0:
            lx = int(cx_d + (r + 18) * math.sin(math.radians(deg)))
            ly = int(cy_d - (r + 18) * math.cos(math.radians(deg)))
            _put(img, f"{deg:+d}", (lx - 20, ly + 5), scale=0.45, color=(100, 100, 100))

    # 전방 0° 선
    cv2.line(img, (cx_d, cy_d), (cx_d, cy_d - r), (80, 80, 80), 1)
    _put(img, "0°", (cx_d + 4, cy_d - r - 8), scale=0.45, color=(100, 100, 100))

    def _draw_arrow(bearing, color, label, inner=0.35):
        ex = int(cx_d + r * math.sin(math.radians(bearing)))
        ey = int(cy_d - r * math.cos(math.radians(bearing)))
        sx = int(cx_d + r * inner * math.sin(math.radians(bearing)))
        sy = int(cy_d - r * inner * math.cos(math.radians(bearing)))
        cv2.arrowedLine(img, (sx, sy), (ex, ey), color, 2, tipLength=0.20)
        lx = int(cx_d + (r + 26) * math.sin(math.radians(bearing)))
        ly = int(cy_d - (r + 26) * math.cos(math.radians(bearing)))
        _put(img, f"{label}:{bearing:+.1f}", (lx - 55, ly + 5), scale=0.50, color=color)

    if centroid is not None:
        _draw_arrow(b_seek,      C_CYAN,   "SK")
        _draw_arrow(b_close_val, C_ORANGE, "CL")

    # 현재 사용 중인 bearing 강조
    active_b = b_close_val if is_close else b_seek
    active_c = C_ORANGE    if is_close else C_CYAN
    ex = int(cx_d + r * math.sin(math.radians(active_b)))
    ey = int(cy_d - r * math.cos(math.radians(active_b)))
    cv2.line(img, (cx_d, cy_d), (ex, ey), active_c, 3)

    # ── 중간: 수치 정보 ──────────────────────────────────────────────────
    y = int(DIAG_H * 0.65)
    line_h = 28

    def row(label, value, color=C_WHITE):
        nonlocal y
        _put(img, label, (14,        y), scale=0.60, color=(140, 140, 140))
        _put(img, value, (DIAG_W//2, y), scale=0.60, color=color)
        y += line_h

    if centroid is not None:
        cx, cy_val = centroid
        row("cx / cy",      f"{cx} / {cy_val}")
        row("f_px",         f"{_f_px():.1f} px")
        row("lateral",      f"{lateral:+.4f}")
        row("forward",      f"{forward:.4f}",
            C_ORANGE if forward < 0.1 else C_WHITE)   # forward 작으면 경고색
        row("SEEK bearing",  f"{b_seek:+.2f} deg",  C_CYAN)
        row("CLOSE bearing", f"{b_close_val:+.2f} deg", C_ORANGE)
        diff = abs(b_close_val - b_seek)
        row("차이 |CL-SK|",  f"{diff:.2f} deg",
            (60, 60, 240) if diff > 10 else C_GREEN)
    else:
        _put(img, "미감지", (DIAG_W//2 - 30, y + 20), scale=0.80, color=(100, 100, 100))

    # ── 하단: ROI 바 ──────────────────────────────────────────────────────
    bar_y  = DIAG_H - 46
    bar_x0 = 14
    bar_w  = DIAG_W - 28
    bar_h  = 18

    cv2.rectangle(img, (bar_x0, bar_y), (bar_x0 + bar_w, bar_y + bar_h),
                  (50, 50, 50), -1)
    fill_w = int(bar_w * min(1.0, roi_fill_close))
    bar_col = C_ORANGE if roi_fill_close >= CLOSE_ROI_FILL else C_GREEN
    if fill_w > 0:
        cv2.rectangle(img, (bar_x0, bar_y),
                      (bar_x0 + fill_w, bar_y + bar_h), bar_col, -1)
    thresh_x = int(bar_x0 + bar_w * CLOSE_ROI_FILL)
    cv2.line(img, (thresh_x, bar_y - 4), (thresh_x, bar_y + bar_h + 4), C_WHITE, 2)
    _put(img, f"CLOSE ROI fill: {roi_fill_close:.2f}  "
              f"(thr={CLOSE_ROI_FILL})  {'← CLOSE' if roi_fill_close >= CLOSE_ROI_FILL else '← SEEK'}",
         (bar_x0, bar_y - 10), scale=0.50, color=bar_col)

    return img


# ── 메인 ─────────────────────────────────────────────────────────────────────

def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[카메라] 요청: {FRAME_W}x{FRAME_H}  실제: {actual_w}x{actual_h}")
    print(f"[유효 해상도] _EFF_W={_EFF_W}  _EFF_H={_EFF_H}")
    print(f"[f_px] {_f_px():.1f} px")
    print()
    print("1=RED  2=YELLOW  3=BLUE  q=종료")

    color_idx  = 0
    win        = 'Bearing Test'
    cv2.namedWindow(win)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        if FRAME_ROTATE is not None:
            frame = cv2.rotate(frame, FRAME_ROTATE)

        color_name = COLOR_KEYS[color_idx]
        centroid, area = _detect(frame, color_name)
        roi_fill_close = _roi_fill(frame, color_name, CLOSE_ROI_BOTTOM)
        is_close       = (roi_fill_close >= CLOSE_ROI_FILL)

        b_seek      = 0.0
        b_close_val = 0.0
        lateral     = 0.0
        forward_val = CAM_POLAR_EPSILON

        if centroid is not None:
            cx, cy = centroid
            b_seek              = _bearing_seek(cx)
            b_close_val, lateral, forward_val = _bearing_close(cx, cy)

        # 패널 합성
        cam_panel  = _draw_cam_panel(frame, centroid, color_name, is_close,
                                     b_seek, b_close_val)
        diag_panel = _draw_diag_panel(centroid, is_close, b_seek, b_close_val,
                                      lateral, forward_val, roi_fill_close, color_name)

        # 높이 맞추기
        ch = cam_panel.shape[0]
        dh = diag_panel.shape[0]
        if ch < dh:
            cam_panel  = cv2.copyMakeBorder(cam_panel,  0, dh - ch, 0, 0,
                                             cv2.BORDER_CONSTANT, value=(20, 20, 20))
        elif dh < ch:
            diag_panel = cv2.copyMakeBorder(diag_panel, 0, ch - dh, 0, 0,
                                             cv2.BORDER_CONSTANT, value=(20, 20, 20))

        combined = np.hstack([cam_panel, diag_panel])
        cv2.imshow(win, combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('1'):
            color_idx = 0
            print(f"[색상] RED 선택")
        elif key == ord('2'):
            color_idx = 1
            print(f"[색상] YELLOW 선택")
        elif key == ord('3'):
            color_idx = 2
            print(f"[색상] BLUE 선택")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

"""
Bearing 시각화 도구 v2
─────────────────────────────────────────────────────
거리 추정 · CLOSE 거리 기반 판정

조작:
  1 / 2 / 3  : 색상 선택 (RED / YELLOW / BLUE)
  q          : 종료
"""

import cv2
import math
import numpy as np

# ── camera_tracker.py 와 동일한 상수 ─────────────────────────────────────────
CAMERA_INDEX      = 0
FRAME_W           = 848   # 16:9 (848×480) 로 변경. FRAME_H=480 유지
FRAME_H           = 480
FRAME_ROTATE      = cv2.ROTATE_90_COUNTERCLOCKWISE

if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    _EFF_W, _EFF_H = FRAME_H, FRAME_W          # 480 × 640
else:
    _EFF_W, _EFF_H = FRAME_W, FRAME_H

HFOV_DEG          = 32.1   # 848×480(16:9) 재측정값 (기존 4:3: 38.6)
CAM_POLAR_EPSILON = 0.05
CAM_HEIGHT_MM     = 590.0   # 실측값 59cm
CAM_TILT_DEG      = 41.8    # 848×480 거리검증 보정값 (40.4→41.8, 4:3: 34.5)
CLOSE_ENTER_MM    = 400.0
COLOR_RANGES = {
    'RED':    [((146, 100,  80), (179, 255, 255))],
    'YELLOW': [((18,  35, 186), ( 72, 177, 255))],
    'BLUE':   [((79,  116, 114), (119, 162, 255))],
}
COLOR_KEYS = ['RED', 'YELLOW', 'BLUE']

# ── 표시 배율 150% ────────────────────────────────────────────────────────────
SC             = 1.5
DISPLAY_SCALE  = 0.65 * SC          # ≈ 0.975 (카메라 패널 배율)
DIAG_W         = int(420 * SC)      # 630px
DIAG_H         = int(_EFF_H * DISPLAY_SCALE)   # ≈ 624px

# ── BGR 색상 ─────────────────────────────────────────────────────────────────
C_WHITE  = (255, 255, 255)
C_BLACK  = (0,   0,   0  )
C_GRAY   = (160, 160, 160)
C_GREEN  = (50,  230, 50 )
C_CYAN   = (255, 220, 0  )
C_ORANGE = (0,   165, 255)
C_RED_B  = (60,  60,  240)


# ── 유틸 ─────────────────────────────────────────────────────────────────────

def _put(img, text, pos, scale=0.70 * SC, color=C_WHITE, thickness=1):
    x, y = pos
    cv2.putText(img, text, (x+1, y+1), cv2.FONT_HERSHEY_SIMPLEX,
                scale, C_BLACK, thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, (x,   y  ), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color,   thickness,     cv2.LINE_AA)


def _f_px():
    return (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))


def _bearing_seek(cx):
    """시각화 전용: 오른쪽=+, 왼쪽=- (camera_tracker와 부호 반대)."""
    return math.degrees(math.atan2(cx - _EFF_W / 2.0, _f_px()))


def _bearing_close(cx, cy):
    """시각화 전용: 오른쪽=+, 왼쪽=- (camera_tracker와 부호 반대)."""
    f       = _f_px()
    lateral = (cx - _EFF_W / 2.0) / f
    forward = (_EFF_H - cy) / _EFF_H + CAM_POLAR_EPSILON
    return math.degrees(math.atan2(lateral, forward)), lateral, forward


def _estimate_dist(cy):
    """camera_tracker.get_estimated_distance_mm 와 동일 로직."""
    delta_v    = math.degrees(math.atan2(cy - _EFF_H / 2.0, _f_px()))
    depression = CAM_TILT_DEG + delta_v
    if depression <= 1.0:
        return 5000.0
    return max(CAM_HEIGHT_MM / math.tan(math.radians(depression)), 50.0)


def _detect(frame, color_name):
    """
    camera_tracker._detect_color 와 동일.
    반환: (centroid, area, clipped_l, clipped_r)
    """
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
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


# ── 카메라 패널 ───────────────────────────────────────────────────────────────

def _draw_cam_panel(frame, centroid, color_name, is_close,
                    b_seek, b_close_val, clip_l, clip_r):
    disp = cv2.resize(frame, (int(_EFF_W * DISPLAY_SCALE),
                               int(_EFF_H * DISPLAY_SCALE)))
    sw, sh = disp.shape[1], disp.shape[0]
    cx_mid = int(_EFF_W / 2 * DISPLAY_SCALE)

    # 중심 수직선
    cv2.line(disp, (cx_mid, 0), (cx_mid, sh), C_GRAY, 1)

    # 클리핑 경계 빨간 테두리
    if clip_l:
        cv2.rectangle(disp, (0, 0), (int(6*SC), sh), (0, 0, 220), -1)
    if clip_r:
        cv2.rectangle(disp, (sw - int(6*SC), 0), (sw, sh), (0, 0, 220), -1)

    if centroid is not None:
        cx, cy = centroid
        dx = int(cx * DISPLAY_SCALE)
        dy = int(cy * DISPLAY_SCALE)
        arrow = int(80 * SC)
        cy_center = sh // 2

        # centroid 마커
        cv2.circle(disp, (dx, dy), int(12 * SC), C_GREEN, 2)
        cv2.circle(disp, (dx, dy), 3, C_GREEN, -1)
        _put(disp, f"({cx},{cy})",
             (dx + int(14*SC), dy + int(6*SC)), scale=0.50*SC, color=C_GREEN)

        # cy 수평선
        cv2.line(disp, (0, dy), (sw, dy), (0, 160, 160), 1)

        def _arrow(bearing, color, width=2):
            ax = int(cx_mid + arrow * math.sin(math.radians(bearing)))
            ay = int(cy_center - arrow * math.cos(math.radians(bearing)))
            cv2.arrowedLine(disp, (cx_mid, cy_center), (ax, ay),
                            color, width, tipLength=0.22)

        _arrow(b_seek,      C_CYAN,   2)
        _arrow(b_close_val, C_ORANGE, 2)

    # 모드 (좌상단)
    mode_str = "[ CLOSE ]" if is_close else "[ SEEK  ]"
    mode_col = C_ORANGE    if is_close else C_CYAN
    _put(disp, mode_str, (6, int(32*SC)), scale=0.80*SC, color=mode_col, thickness=2)

    # 클리핑 경고
    clip_tag = ("L" if clip_l else "") + ("R" if clip_r else "")
    if clip_tag:
        _put(disp, f"CLIPPED {clip_tag}",
             (6, int(62*SC)), scale=0.60*SC, color=C_RED_B)

    # 색상명 (우상단)
    _put(disp, color_name,
         (sw - int(110*SC), int(32*SC)), scale=0.75*SC, color=C_WHITE)

    # 범례 (우하단)
    legend_y = sh - int(55*SC)
    for text, col in [("SK=SEEK", C_CYAN), ("CL=CLOSE", C_ORANGE)]:
        _put(disp, text, (int(6*SC), legend_y), scale=0.45*SC, color=col)
        legend_y += int(22*SC)

    return disp


# ── 다이어그램 패널 ───────────────────────────────────────────────────────────

def _draw_diag_panel(centroid, is_close, b_seek, b_close_val, dist_mm):
    img = np.zeros((DIAG_H, DIAG_W, 3), dtype=np.uint8)
    img[:] = (25, 25, 25)

    # ── 상단: bearing 다이어그램 ──────────────────────────────────────────────
    cx_d = DIAG_W // 2
    cy_d = int(DIAG_H * 0.30)
    r    = int(80 * SC)

    cv2.circle(img, (cx_d, cy_d), r, (60, 60, 60), 1)
    cv2.circle(img, (cx_d, cy_d), 3, C_GRAY, -1)

    for deg in range(-90, 91, 30):
        ex = int(cx_d + r * math.sin(math.radians(deg)))
        ey = int(cy_d - r * math.cos(math.radians(deg)))
        cv2.line(img, (cx_d, cy_d), (ex, ey), (40, 40, 40), 1)
        if deg != 0:
            lx = int(cx_d + (r + int(20*SC)) * math.sin(math.radians(deg)))
            ly = int(cy_d - (r + int(20*SC)) * math.cos(math.radians(deg)))
            _put(img, f"{deg:+d}",
                 (lx - int(18*SC), ly + int(5*SC)), scale=0.42*SC,
                 color=(90, 90, 90))

    cv2.line(img, (cx_d, cy_d), (cx_d, cy_d - r), (70, 70, 70), 1)
    _put(img, "0°", (cx_d + 4, cy_d - r - int(10*SC)),
         scale=0.42*SC, color=(90, 90, 90))

    def _diag_arrow(bearing, color, inner=0.30, thickness=2):
        ex = int(cx_d + r * math.sin(math.radians(bearing)))
        ey = int(cy_d - r * math.cos(math.radians(bearing)))
        sx = int(cx_d + r * inner * math.sin(math.radians(bearing)))
        sy = int(cy_d - r * inner * math.cos(math.radians(bearing)))
        cv2.arrowedLine(img, (sx, sy), (ex, ey), color, thickness, tipLength=0.20)

    if centroid is not None:
        _diag_arrow(b_seek,      C_CYAN)
        _diag_arrow(b_close_val, C_ORANGE)
        _put(img, f"SK:{b_seek:+.1f}", (14, 22), scale=0.48*SC, color=C_CYAN)
        _put(img, f"CL:{b_close_val:+.1f}", (DIAG_W//2 + 10, 22), scale=0.48*SC, color=C_ORANGE)

    # 활성 bearing 강조선
    active_b = b_close_val if is_close else b_seek
    active_c = C_ORANGE    if is_close else C_CYAN
    cv2.line(img,
             (cx_d, cy_d),
             (int(cx_d + r * math.sin(math.radians(active_b))),
              int(cy_d - r * math.cos(math.radians(active_b)))),
             active_c, 3)

    # ── 수치 정보 ─────────────────────────────────────────────────────────────
    y      = int(DIAG_H * 0.52)
    line_h = int(22 * SC)

    def row(label, value, color=C_WHITE, bold=False):
        nonlocal y
        lscale = 0.58 * SC
        vscale = 0.62 * SC if bold else 0.58 * SC
        _put(img, label, (int(14),          y), scale=lscale, color=(130, 130, 130))
        _put(img, value, (int(DIAG_W * 0.48), y), scale=vscale, color=color,
             thickness=2 if bold else 1)
        y += line_h

    if centroid is not None:
        cx_v, cy_v = centroid
        row("cx / cy", f"{cx_v} / {cy_v}")

        row("SEEK bearing",  f"{b_seek:+.2f} deg",     C_CYAN)
        row("CLOSE bearing", f"{b_close_val:+.2f} deg", C_ORANGE)

        row("|CL - SK|",
            f"{abs(b_close_val - b_seek):.2f} deg",
            C_RED_B if abs(b_close_val - b_seek) > 10 else C_GREEN)

        row("dist estimate",
            f"{dist_mm:.0f} mm",
            C_ORANGE if dist_mm < CLOSE_ENTER_MM else C_WHITE)
        row("CLOSE mode",
            f"{'YES' if is_close else 'NO'}  (< {CLOSE_ENTER_MM:.0f} mm)",
            C_ORANGE if is_close else C_CYAN)

    else:
        _put(img, "NO TARGET",
             (DIAG_W//2 - int(40*SC), y + int(20*SC)),
             scale=0.60*SC, color=(100, 100, 100))

    # ── 하단: 거리 바 (CLOSE 진입 기준) ──────────────────────────────────────
    bar_y  = DIAG_H - int(50*SC)
    bar_x0 = int(14)
    bar_w  = DIAG_W - int(28)
    bar_h  = int(18 * SC)

    cv2.rectangle(img, (bar_x0, bar_y),
                  (bar_x0 + bar_w, bar_y + bar_h), (50, 50, 50), -1)

    DIST_MAX = 1500.0
    fill_ratio = max(0.0, min(1.0, 1.0 - (dist_mm - 0) / DIST_MAX))
    fill_w     = int(bar_w * fill_ratio)
    bar_col    = C_ORANGE if is_close else C_GREEN
    if fill_w > 0:
        cv2.rectangle(img, (bar_x0, bar_y),
                      (bar_x0 + fill_w, bar_y + bar_h), bar_col, -1)

    thresh_ratio = 1.0 - CLOSE_ENTER_MM / DIST_MAX
    thresh_x     = int(bar_x0 + bar_w * thresh_ratio)
    cv2.line(img, (thresh_x, bar_y - int(5*SC)),
             (thresh_x, bar_y + bar_h + int(5*SC)), C_WHITE, 2)

    dist_str = f"{dist_mm:.0f}mm" if dist_mm < 4999 else "---"
    _put(img,
         f"dist: {dist_str}  CLOSE <{CLOSE_ENTER_MM:.0f}mm  "
         f"{'← CLOSE' if is_close else '← SEEK'}",
         (bar_x0, bar_y - int(12*SC)), scale=0.50*SC, color=bar_col)

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
    print(f"[f_px] {_f_px():.1f} px  HFOV={HFOV_DEG}°")
    print(f"[거리 추정] CAM_H={CAM_HEIGHT_MM}mm  TILT={CAM_TILT_DEG}°  "
          f"CLOSE < {CLOSE_ENTER_MM}mm")
    print()
    print("1=RED  2=YELLOW  3=BLUE  q=종료")

    color_idx = 0

    cv2.namedWindow('Bearing Test')

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        if FRAME_ROTATE is not None:
            frame = cv2.rotate(frame, FRAME_ROTATE)

        color_name = COLOR_KEYS[color_idx]
        centroid, _, clip_l, clip_r = _detect(frame, color_name)

        b_seek      = 0.0
        b_close_val = 0.0
        dist_mm     = 5000.0
        is_close    = False

        if centroid is not None:
            cx, cy      = centroid
            b_seek      = _bearing_seek(cx)
            b_close_val, _, _ = _bearing_close(cx, cy)
            dist_mm     = _estimate_dist(cy)
            is_close    = (dist_mm < CLOSE_ENTER_MM)

        cam_panel  = _draw_cam_panel(
            frame, centroid, color_name, is_close,
            b_seek, b_close_val, clip_l, clip_r)
        diag_panel = _draw_diag_panel(
            centroid, is_close, b_seek, b_close_val, dist_mm)

        # 높이 맞추기
        ch, dh = cam_panel.shape[0], diag_panel.shape[0]
        if ch < dh:
            cam_panel  = cv2.copyMakeBorder(cam_panel,  0, dh-ch, 0, 0,
                                             cv2.BORDER_CONSTANT, value=(20, 20, 20))
        elif dh < ch:
            diag_panel = cv2.copyMakeBorder(diag_panel, 0, ch-dh, 0, 0,
                                             cv2.BORDER_CONSTANT, value=(20, 20, 20))

        cv2.imshow('Bearing Test', np.hstack([cam_panel, diag_panel]))

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('1'):
            color_idx = 0
            print("[색상] RED")
        elif key == ord('2'):
            color_idx = 1
            print("[색상] YELLOW")
        elif key == ord('3'):
            color_idx = 2
            print("[색상] BLUE")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

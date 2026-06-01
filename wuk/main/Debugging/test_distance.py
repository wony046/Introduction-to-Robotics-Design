"""
test_distance.py — 기하학 거리 추정 모델 검증
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
색지를 카메라 앞 여러 거리에 놓고
추정 거리 vs 실제 거리를 비교하여 모델 정확도 확인.

사용법:
  python3 test_distance.py

조작:
  숫자 + Enter : 실제 거리(mm) 입력 → 추정값과 오차 기록
  q + Enter    : 종료 및 오차 통계 출력
  OpenCV 창에서 q 키도 종료
"""

import cv2
import math
import time
import select
import sys
import numpy as np

# ── camera_tracker.py와 동일하게 맞출 것 ───────────────────────────────────
CAMERA_INDEX   = 0
FRAME_W        = 640
FRAME_H        = 480
FRAME_ROTATE   = cv2.ROTATE_90_COUNTERCLOCKWISE

HFOV_DEG       = 38.6     # 실측값
CAM_HEIGHT_MM  = 430.0    # ★ 실측 필요 (바닥~카메라 수직 높이 mm)
CAM_TILT_DEG   = 34.5     # 역산값: actual=500mm, est=610mm, delta_v=0 → atan(420/610)
CLOSE_ENTER_MM = 350.0    # 이 거리 이내 → CLOSE 모드 진입 (camera_tracker.py와 동일)

TARGET_COLOR   = 'RED'    # 테스트할 색상

COLOR_RANGES = {
    'RED':    [((118, 104, 136), (179, 255, 255))],
    'YELLOW': [((9,   90,  64), (41,  194, 255))],
    'BLUE':   [((107,  93, 109), (127, 182, 180))],
}

DISPLAY_SCALE  = 0.6
MIN_AREA       = 500

# ── 회전 후 실효 해상도 ────────────────────────────────────────────────────
if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    _EFF_W, _EFF_H = FRAME_H, FRAME_W
else:
    _EFF_W, _EFF_H = FRAME_W, FRAME_H

# ── 색상 상수 ──────────────────────────────────────────────────────────────
C_WHITE  = (255, 255, 255)
C_BLACK  = (0,   0,   0  )
C_GRAY   = (150, 150, 150)
C_GREEN  = (60,  230, 60 )
C_RED    = (60,  60,  230)
C_CYAN   = (230, 220, 0  )
C_YELLOW = (0,   220, 255)
C_ORANGE = (0,   165, 255)

OVERLAY_BGR = {
    'RED':    (60,  60,  200),
    'YELLOW': (0,   200, 220),
    'BLUE':   (200, 80,  0  ),
}


# ─────────────────────────────────────────────────────────────────────────────
def detect(frame):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_RANGES[TARGET_COLOR]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0, None
    largest = max(contours, key=cv2.contourArea)
    area    = cv2.contourArea(largest)
    if area < MIN_AREA:
        return None, 0.0, largest
    M  = cv2.moments(largest)
    if M['m00'] == 0:
        return None, 0.0, largest
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return (cx, cy), area, largest


def estimate_distance(cy):
    """cy 픽셀 위치 → 기하학적 거리 추정 (mm)."""
    f_px       = (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
    delta_v    = math.degrees(math.atan2(cy - _EFF_H / 2.0, f_px))
    depression = CAM_TILT_DEG + delta_v
    if depression <= 1.0:
        return None, delta_v, depression
    d = CAM_HEIGHT_MM / math.tan(math.radians(depression))
    return max(d, 50.0), delta_v, depression


def dist_to_cy(d_mm):
    """거리(mm) → 해당 cy 픽셀 역산. CLOSE 임계선 표시에 사용."""
    f_px       = (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
    depression = math.degrees(math.atan(CAM_HEIGHT_MM / d_mm))
    delta_v    = depression - CAM_TILT_DEG
    return int(_EFF_H / 2.0 + f_px * math.tan(math.radians(delta_v)))


# CLOSE 진입 임계선 cy (매 프레임 재계산 불필요)
_CLOSE_CY = dist_to_cy(CLOSE_ENTER_MM)


def _put(img, text, pos, scale=0.78, color=C_WHITE, thickness=2):
    x, y = pos
    cv2.putText(img, text, (x+2, y+2), cv2.FONT_HERSHEY_SIMPLEX,
                scale, C_BLACK, thickness+2, cv2.LINE_AA)
    cv2.putText(img, text, (x,   y  ), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color,   thickness,   cv2.LINE_AA)


def draw_geo_diagram(img, x0, y0, w, h, delta_v, depression, est_dist):
    """
    우측 패널에 기하 모델 다이어그램 그리기.
    카메라, 기울기 각도, 추정 거리를 시각화.
    """
    # 배경 패널
    overlay = img.copy()
    cv2.rectangle(overlay, (x0, y0), (x0+w, y0+h), (25, 25, 25), -1)
    cv2.addWeighted(overlay, 0.75, img, 0.25, 0, img)
    cv2.rectangle(img, (x0, y0), (x0+w, y0+h), C_GRAY, 1)

    # 다이어그램 영역 내부 좌표
    pad    = 20
    cam_x  = x0 + pad + 20          # 카메라 위치 x
    floor_y = y0 + h - pad - 20     # 바닥 y
    cam_y   = floor_y - int((CAM_HEIGHT_MM / 600.0) * (h - 2*pad - 60))

    # 바닥선
    cv2.line(img, (x0+pad, floor_y), (x0+w-pad, floor_y), C_GRAY, 2)
    _put(img, "floor", (x0+pad, floor_y+18), scale=0.52, color=C_GRAY)

    # 카메라 높이 수직선
    cv2.line(img, (cam_x, cam_y), (cam_x, floor_y), (100, 100, 100), 1)
    mid_y = (cam_y + floor_y) // 2
    _put(img, f"H={CAM_HEIGHT_MM:.0f}mm", (cam_x+6, mid_y), scale=0.55, color=C_GRAY)

    # 카메라 아이콘
    cv2.rectangle(img, (cam_x-10, cam_y-7), (cam_x+10, cam_y+7), C_CYAN, -1)
    cv2.rectangle(img, (cam_x-10, cam_y-7), (cam_x+10, cam_y+7), C_WHITE, 1)
    _put(img, "CAM", (cam_x-14, cam_y-12), scale=0.48, color=C_CYAN)

    # 광축 방향 (CAM_TILT_DEG)
    ray_len = int((h - 2*pad) * 0.7)
    tilt_rad = math.radians(depression if depression > 0 else CAM_TILT_DEG)
    ex = cam_x + int(ray_len * math.sin(0))      # 수평
    ey = cam_y + int(ray_len * math.cos(0))
    # 실제 광선: 수직 아래 성분
    rx = cam_x + int(ray_len * 0.6 * math.sin(math.radians(depression if depression > 0 else CAM_TILT_DEG)))
    ry = cam_y + int(ray_len * 0.6)

    # 광축선 (CAM_TILT)
    tilt_rx = cam_x + int(ray_len * 0.5 * math.sin(math.radians(CAM_TILT_DEG)))
    tilt_ry = cam_y + int(ray_len * 0.5)
    cv2.arrowedLine(img, (cam_x, cam_y), (tilt_rx, tilt_ry),
                    C_GRAY, 1, tipLength=0.15)

    # 시선 방향 (depression = CAM_TILT + delta_v)
    if depression > 0:
        dep_rx = cam_x + int(ray_len * 0.8 * math.sin(math.radians(depression)))
        dep_ry = cam_y + int(ray_len * 0.8)
        dep_rx = min(dep_rx, x0+w-pad)
        dep_ry = min(dep_ry, floor_y)
        cv2.arrowedLine(img, (cam_x, cam_y), (dep_rx, dep_ry),
                        C_ORANGE, 2, tipLength=0.12)

        # 바닥 도달 지점
        target_x = cam_x + int((CAM_HEIGHT_MM / math.tan(math.radians(depression)))
                                / 600.0 * (w - 2*pad - 40))
        target_x = min(target_x, x0+w-pad-5)
        cv2.circle(img, (target_x, floor_y), 7, C_ORANGE, -1)
        cv2.line(img, (cam_x, floor_y), (target_x, floor_y), C_ORANGE, 1)

        # 거리 표시
        mid_fx = (cam_x + target_x) // 2
        if est_dist is not None:
            _put(img, f"{est_dist:.0f}mm", (mid_fx-20, floor_y-16),
                 scale=0.62, color=C_ORANGE)

    # 각도 표시
    ang_label_y = cam_y + 30
    _put(img, f"tilt={CAM_TILT_DEG:.0f}°", (cam_x+14, ang_label_y),
         scale=0.52, color=C_GRAY)
    if depression > 0:
        _put(img, f"dep={depression:.1f}°", (cam_x+14, ang_label_y+22),
             scale=0.52, color=C_ORANGE)
        _put(img, f"dv={delta_v:+.1f}°",   (cam_x+14, ang_label_y+44),
             scale=0.52, color=C_YELLOW)


def build_display(frame, centroid, contour, est_dist, delta_v, depression,
                  last_actual, records):
    """
    좌: 카메라 + 색지 오버레이 + cy 라인
    우: 기하 다이어그램 + 수치 패널
    """
    cam_w = _EFF_W
    cam_h = _EFF_H
    panel_w = 260
    total_w = cam_w + panel_w

    canvas = np.zeros((cam_h, total_w, 3), dtype=np.uint8)

    # ── 좌측: 카메라 ──────────────────────────────────────────────────────
    vis = frame.copy()

    if contour is not None:
        ov = vis.copy()
        cv2.drawContours(ov, [contour], -1, OVERLAY_BGR[TARGET_COLOR], -1)
        cv2.addWeighted(ov, 0.35, vis, 0.65, 0, vis)
        cv2.drawContours(vis, [contour], -1, OVERLAY_BGR[TARGET_COLOR], 3)

    # 중심 수직선
    cv2.line(vis, (cam_w//2, 0), (cam_w//2, cam_h), C_GRAY, 1)

    # CLOSE 진입 임계선 (350mm)
    if 0 <= _CLOSE_CY < cam_h:
        cv2.line(vis, (0, _CLOSE_CY), (cam_w, _CLOSE_CY), C_GREEN, 2)
        _put(vis, f"CLOSE {CLOSE_ENTER_MM:.0f}mm", (6, _CLOSE_CY - 8),
             scale=0.58, color=C_GREEN)

    if centroid is not None:
        cx, cy = centroid
        # cy 수평선 (거리 계산에 사용되는 줄)
        cv2.line(vis, (0, cy), (cam_w, cy), C_ORANGE, 1)
        # cy 레이블
        _put(vis, f"cy={cy}", (cam_w - 90, cy - 8), scale=0.60, color=C_ORANGE)
        # centroid 마커
        cv2.circle(vis, (cx, cy), 12, C_WHITE,  2)
        cv2.circle(vis, (cx, cy),  5, C_GREEN, -1)
        # 화면 중앙 기준선
        cv2.line(vis, (0, cam_h//2), (cam_w, cam_h//2), (60, 60, 60), 1)
        # delta_v 방향 표시
        center_y = cam_h // 2
        arrow_color = C_YELLOW
        cv2.arrowedLine(vis, (30, center_y), (30, cy), arrow_color, 2, tipLength=0.15)
        dv_sign = "↓" if cy > center_y else "↑"
        _put(vis, f"dv{dv_sign}", (36, (center_y+cy)//2), scale=0.58, color=C_YELLOW)

    # 거리 크게 표시
    if est_dist is not None:
        dist_str = f"{est_dist:.0f} mm"
        color_dist = C_GREEN if est_dist < CLOSE_ENTER_MM else C_ORANGE
        _put(vis, dist_str, (cam_w//2 - 60, 50), scale=1.2,
             color=color_dist, thickness=2)
        _put(vis, "est. dist", (cam_w//2 - 48, 80), scale=0.60, color=C_GRAY)
    else:
        _put(vis, "NOT DETECTED", (20, 50), scale=0.9, color=C_RED)

    canvas[:, :cam_w] = vis

    # ── 우측: 패널 ────────────────────────────────────────────────────────
    panel = canvas[:, cam_w:]

    # 기하 다이어그램 (상단 60%)
    diag_h = int(cam_h * 0.58)
    draw_geo_diagram(canvas, cam_w, 0, panel_w, diag_h,
                     delta_v if delta_v is not None else 0.0,
                     depression if depression is not None else CAM_TILT_DEG,
                     est_dist)

    # 수치 정보 (하단 40%)
    info_y0 = diag_h + 8
    iy = info_y0

    def pi(text, color=C_WHITE, scale=0.68):
        nonlocal iy
        _put(canvas, text, (cam_w+8, iy), scale=scale, color=color)
        iy += 28

    pi("── 현재 값 ──", color=C_GRAY, scale=0.60)
    if centroid:
        pi(f"cy    = {centroid[1]} px")
        pi(f"dv    = {delta_v:+.1f} deg",  color=C_YELLOW)
        pi(f"dep   = {depression:.1f} deg", color=C_ORANGE)
        pi(f"est   = {est_dist:.0f} mm" if est_dist else "est   = ---",
           color=C_GREEN)
    else:
        pi("미감지", color=C_RED)

    iy += 6
    pi("── 비교 ──", color=C_GRAY, scale=0.60)
    if last_actual and est_dist:
        err = est_dist - last_actual['actual']
        err_pct = err / last_actual['actual'] * 100
        color_err = C_GREEN if abs(err_pct) < 10 else C_ORANGE if abs(err_pct) < 20 else C_RED
        pi(f"actual= {last_actual['actual']:.0f} mm", color=C_CYAN)
        pi(f"error = {err:+.0f} mm", color=color_err)
        pi(f"       ({err_pct:+.1f}%)",  color=color_err)
    else:
        pi("터미널에 실제", color=C_GRAY, scale=0.62)
        pi("거리(mm) 입력", color=C_GRAY, scale=0.62)

    if records:
        avg_err = sum(r['error'] for r in records) / len(records)
        pi(f"평균오차= {avg_err:+.0f}mm (n={len(records)})", color=C_CYAN, scale=0.64)

    # 하단 조작 안내
    _put(canvas, "Enter=기록  q=종료",
         (cam_w+4, cam_h-18), scale=0.54, color=C_GRAY, thickness=1)

    # 축소
    h, w = canvas.shape[:2]
    return cv2.resize(canvas, (int(w*DISPLAY_SCALE), int(h*DISPLAY_SCALE)))


# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("=" * 55)
    print("  거리 추정 모델 검증  (test_distance.py)")
    print("=" * 55)
    print(f"  대상 색상    : {TARGET_COLOR}")
    print(f"  CAM_HEIGHT   : {CAM_HEIGHT_MM} mm")
    print(f"  CAM_TILT     : {CAM_TILT_DEG} deg")
    print(f"  HFOV         : {HFOV_DEG} deg")
    print(f"  해상도       : {FRAME_W}x{FRAME_H} → _EFF {_EFF_W}x{_EFF_H}")
    print()
    print("색지를 카메라 앞에 놓고 실제 거리(mm)를 입력하세요.")
    print("q + Enter 또는 창에서 q 키 → 종료")
    print("=" * 55)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[카메라] 실제 해상도: {actual_w}x{actual_h}")

    win = 'Distance Test'
    cv2.namedWindow(win)

    records    = []
    last_actual = None
    last_frame  = None

    while True:
        ret, frame = cap.read()
        if ret:
            if FRAME_ROTATE is not None:
                frame = cv2.rotate(frame, FRAME_ROTATE)
            last_frame = frame

        if last_frame is None:
            continue

        centroid, area, contour = detect(last_frame)

        est_dist = delta_v = depression = None
        if centroid is not None:
            cx, cy = centroid
            est_dist, delta_v, depression = estimate_distance(cy)
            print(f"\r[LIVE] cy={cy:4d}  dv={delta_v:+5.1f}°  "
                  f"dep={depression:5.1f}°  est={est_dist:6.0f}mm    ",
                  end='', flush=True)
        else:
            print(f"\r[LIVE] 미감지                                    ",
                  end='', flush=True)

        disp = build_display(last_frame, centroid, contour,
                             est_dist, delta_v, depression,
                             last_actual, records)
        cv2.imshow(win, disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        # 터미널 입력
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            if line.lower() == 'q':
                break
            try:
                actual_mm = float(line)
            except ValueError:
                print(f"\n[WARN] 숫자를 입력하세요 (예: 500)")
                continue

            if est_dist is None:
                print(f"\n[SKIP] 색지가 감지되지 않아 기록하지 않았습니다.")
                continue

            error = est_dist - actual_mm
            err_pct = error / actual_mm * 100
            last_actual = {'actual': actual_mm, 'est': est_dist, 'error': error}
            records.append(last_actual)
            print(f"\n[기록 #{len(records)}] "
                  f"actual={actual_mm:.0f}mm  "
                  f"est={est_dist:.0f}mm  "
                  f"error={error:+.0f}mm ({err_pct:+.1f}%)")

    cap.release()
    cv2.destroyAllWindows()

    # ── 결과 통계 ─────────────────────────────────────────────────────────
    print("\n\n" + "=" * 50)
    print("거리 추정 오차 통계")
    print("=" * 50)
    if not records:
        print("기록 없음")
        return

    print(f"{'#':>3}  {'actual':>8}  {'est':>8}  {'error':>8}  {'%':>7}")
    print("-" * 45)
    for i, r in enumerate(records, 1):
        print(f"{i:>3}  {r['actual']:>8.0f}  {r['est']:>8.0f}  "
              f"{r['error']:>+8.0f}  {r['error']/r['actual']*100:>+6.1f}%")

    errors = [r['error'] for r in records]
    abs_errors = [abs(e) for e in errors]
    print("-" * 45)
    print(f"  평균 오차   : {sum(errors)/len(errors):+.0f} mm")
    print(f"  평균 절대오차: {sum(abs_errors)/len(abs_errors):.0f} mm")
    print(f"  최대 절대오차: {max(abs_errors):.0f} mm")
    print()
    if sum(abs_errors)/len(abs_errors) > 100:
        print("★ CAM_HEIGHT_MM 또는 CAM_TILT_DEG 재측정 권장")
    else:
        print("★ 오차 양호")


if __name__ == '__main__':
    main()

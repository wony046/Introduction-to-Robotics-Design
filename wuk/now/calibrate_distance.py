"""
DISTANCE_K_MM 캘리브레이션 도구 (시각화 포함)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
창 2개:
  [camera] 원본 프레임 + 감지 영역 색상 오버레이 + 윤곽선 + 텍스트
  [mask]   HSV 마스크 (흰색 = 인식 영역)

터미널:
  숫자 + Enter  : 해당 거리(mm)에서 K 샘플 기록
  q + Enter     : 종료 및 결과 출력
  OpenCV 창에서 q 키도 종료
"""

import cv2
import math
import time
import select
import sys
import numpy as np

# ── 카메라 설정 (camera_tracker.py와 동일) ─────────────────────────────────
CAMERA_INDEX  = 0
FRAME_W       = 1280
FRAME_H       = 720
FRAME_ROTATE  = cv2.ROTATE_90_COUNTERCLOCKWISE

if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    _EFF_W, _EFF_H = FRAME_H, FRAME_W
else:
    _EFF_W, _EFF_H = FRAME_W, FRAME_H

# ── 색상 범위 (camera_tracker.py와 동일) ───────────────────────────────────
COLOR_RANGES = {
    'RED':    [((170, 120, 90), (179, 220, 255))],
    'YELLOW': [((16,  95, 155), (59,  183, 255))],
    'BLUE':   [((64,  46, 138), (125, 160, 247))],
}

# 감지 색상 오버레이 색 (BGR)
OVERLAY_BGR = {
    'RED':    (0,   0,   220),
    'YELLOW': (0,   220, 220),
    'BLUE':   (220, 80,  0  ),
}

TARGET_COLOR  = 'RED'    # ← 캘리할 색상
MIN_AREA      = 500      # 최소 유효 blob 면적
DISPLAY_SCALE = 0.5      # 창 축소 비율 (화면이 작으면 0.4로 줄임)


# ─────────────────────────────────────────────────────────────────────────────

def detect_area(frame, color_name):
    """blob 검출. 반환: (centroid, area, mask, contour)"""
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0, mask, None

    largest = max(contours, key=cv2.contourArea)
    area    = cv2.contourArea(largest)
    if area < MIN_AREA:
        return None, 0.0, mask, largest

    M  = cv2.moments(largest)
    if M['m00'] == 0:
        return None, 0.0, mask, largest
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return (cx, cy), area, mask, largest


def build_display(frame, mask, centroid, area, contour, last_sample):
    """
    좌: 원본 + 감지 오버레이 (색상 반투명 채움 + 윤곽선 + 텍스트)
    우: 마스크 (흰색=인식)를 컬러로 표시
    두 이미지를 가로로 붙여 반환
    """
    vis = frame.copy()

    # ── 감지 영역 색상 오버레이 (반투명) ─────────────────────────────────
    if contour is not None:
        overlay = vis.copy()
        cv2.drawContours(overlay, [contour], -1, OVERLAY_BGR[TARGET_COLOR], -1)
        cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)
        cv2.drawContours(vis, [contour], -1, OVERLAY_BGR[TARGET_COLOR], 3)

    # ── 중심점 ────────────────────────────────────────────────────────────
    if centroid is not None:
        cx, cy = centroid
        cv2.circle(vis, (cx, cy), 14, (0, 255, 0), 3)
        cv2.circle(vis, (cx, cy),  4, (0, 255, 0), -1)
        cv2.line(vis, (_EFF_W // 2, 0), (_EFF_W // 2, _EFF_H), (180, 180, 180), 1)

    # ── 텍스트 오버레이 ───────────────────────────────────────────────────
    font      = cv2.FONT_HERSHEY_SIMPLEX
    txt_color = (255, 255, 255)
    shadow    = (0, 0, 0)

    def put(img, text, pos, scale=0.9, color=txt_color):
        x, y = pos
        cv2.putText(img, text, (x+2, y+2), font, scale, shadow,    3, cv2.LINE_AA)
        cv2.putText(img, text, (x,   y  ), font, scale, color,     2, cv2.LINE_AA)

    if centroid is not None:
        put(vis, f"Target : {TARGET_COLOR}",          (10,  45))
        put(vis, f"Area   : {area:.0f} px",           (10,  85))
        put(vis, f"sqrtA  : {math.sqrt(area):.1f}",   (10, 125))
        put(vis, f"cx,cy  : ({centroid[0]},{centroid[1]})", (10, 165))
        if last_sample:
            d, a, K = last_sample
            put(vis, f"Last K : {K:.0f}  @{d:.0f}mm", (10, 205), color=(0, 255, 180))
    else:
        put(vis, f"Target : {TARGET_COLOR}",          (10,  45))
        put(vis, "NOT DETECTED",                      (10,  85), color=(0, 80, 255))

    put(vis, "Terminal: dist(mm)+Enter / q+Enter", (10, _EFF_H - 20), scale=0.6,
        color=(200, 200, 200))

    # ── 마스크 → 컬러 ─────────────────────────────────────────────────────
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    # 흰 영역(=인식)을 대상 색으로 채색
    tint = np.zeros_like(mask_bgr)
    tint[mask > 0] = OVERLAY_BGR[TARGET_COLOR]
    mask_vis = cv2.addWeighted(mask_bgr, 0.4, tint, 0.6, 0)
    cv2.putText(mask_vis, "HSV mask", (10, 40), font, 0.9, (220, 220, 220), 2, cv2.LINE_AA)

    # ── 두 이미지 가로로 합치기 ───────────────────────────────────────────
    combined = np.hstack([vis, mask_vis])
    h, w = combined.shape[:2]
    display = cv2.resize(combined, (int(w * DISPLAY_SCALE), int(h * DISPLAY_SCALE)))
    return display


def collect_samples(cap, n_frames=15):
    """n_frames 동안 area 수집 → 평균 반환."""
    areas = []
    for _ in range(n_frames):
        ret, f = cap.read()
        if not ret:
            continue
        if FRAME_ROTATE is not None:
            f = cv2.rotate(f, FRAME_ROTATE)
        _, a, _, _ = detect_area(f, TARGET_COLOR)
        if a >= MIN_AREA:
            areas.append(a)
        time.sleep(0.03)
    return (sum(areas) / len(areas)) if areas else None


# ─────────────────────────────────────────────────────────────────────────────

def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        return

    print(f"[CAL] 대상: {TARGET_COLOR}  해상도: {_EFF_W}x{_EFF_H}")
    print("=" * 55)
    print("색지를 정면에 고정 → 터미널에 거리(mm) 입력 + Enter")
    print("q + Enter  또는  OpenCV 창에서 q 키 → 종료")
    print("=" * 55)

    samples    = []
    last_sample = None

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        if FRAME_ROTATE is not None:
            frame = cv2.rotate(frame, FRAME_ROTATE)

        centroid, area, mask, contour = detect_area(frame, TARGET_COLOR)

        # ── 터미널 출력 ───────────────────────────────────────────────────
        if centroid is not None:
            print(f"\r[LIVE] area={area:8.0f}  √area={math.sqrt(area):6.1f}  "
                  f"pos=({centroid[0]:4d},{centroid[1]:4d})  감지O    ",
                  end='', flush=True)
        else:
            print(f"\r[LIVE] 미감지                                          ",
                  end='', flush=True)

        # ── OpenCV 시각화 ─────────────────────────────────────────────────
        disp = build_display(frame, mask, centroid, area, contour, last_sample)
        cv2.imshow('CAL: camera + mask', disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        # ── 터미널 입력 (논블로킹) ────────────────────────────────────────
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            if line.lower() == 'q':
                break
            try:
                dist_mm = float(line)
            except ValueError:
                print(f"\n[WARN] 숫자 입력 (예: 300)")
                continue

            if centroid is None or area < MIN_AREA:
                print(f"\n[SKIP] 색지 미감지 — 샘플 기록 안 함")
                continue

            print(f"\n[수집중] 0.5초간 {TARGET_COLOR} blob 측정...", flush=True)
            avg_area = collect_samples(cap, n_frames=15)
            if avg_area is None:
                print("[SKIP] 수집 실패")
                continue

            K = dist_mm * math.sqrt(avg_area)
            last_sample = (dist_mm, avg_area, K)
            samples.append(last_sample)
            print(f"[SAMPLE #{len(samples)}] dist={dist_mm:.0f}mm  "
                  f"avg_area={avg_area:.0f}  √area={math.sqrt(avg_area):.1f}  K={K:.0f}")

    cap.release()
    cv2.destroyAllWindows()

    # ── 결과 출력 ─────────────────────────────────────────────────────────
    print("\n\n" + "=" * 50)
    print("캘리브레이션 결과")
    print("=" * 50)
    if not samples:
        print("샘플 없음")
        return

    print(f"{'#':>3}  {'dist(mm)':>10}  {'avg_area':>10}  {'K':>10}")
    print("-" * 40)
    for i, (dist, area, K) in enumerate(samples, 1):
        print(f"{i:>3}  {dist:>10.0f}  {area:>10.0f}  {K:>10.0f}")

    avg_K = sum(K for _, _, K in samples) / len(samples)
    print("-" * 40)
    print(f"{'평균 K':>26}  {avg_K:>10.0f}")
    print()
    print("▶ camera_tracker.py 에 적용:")
    print(f"  DISTANCE_K_MM = {avg_K:.0f}")


if __name__ == '__main__':
    main()

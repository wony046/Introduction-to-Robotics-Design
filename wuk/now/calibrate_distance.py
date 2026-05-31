"""
DISTANCE_K_MM 캘리브레이션 도구
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
사용법:
  1. 색지를 카메라 정면에 고정 (로봇 정지 상태)
  2. python3 calibrate_distance.py
  3. 실제 거리(mm)를 입력하고 Enter → 해당 거리에서 K값 기록
  4. 여러 거리에서 반복 → 평균 K 산출

조작키 (터미널):
  Enter  : 현재 area로 K 샘플 기록
  q      : 종료 및 결과 출력

SHOW_FRAME = True 로 바꾸면 OpenCV 창에서 마스크를 시각 확인 가능
(VNC 또는 모니터 연결 필요)
"""

import cv2
import math
import numpy as np

# ── 카메라 설정 (camera_tracker.py와 동일하게 맞출 것) ─────────────────────
CAMERA_INDEX  = 0
FRAME_W       = 1280
FRAME_H       = 720
FRAME_ROTATE  = cv2.ROTATE_90_COUNTERCLOCKWISE

# 회전 후 실효 해상도
if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    _EFF_W, _EFF_H = FRAME_H, FRAME_W
else:
    _EFF_W, _EFF_H = FRAME_W, FRAME_H

# ── 색상 범위 (camera_tracker.py와 동일하게 맞출 것) ───────────────────────
COLOR_RANGES = {
    'RED': [
        ((163, 84, 161), (179, 255, 255)),
    ],
    'YELLOW': [
        ((16, 95, 155), (59, 183, 255)),
    ],
    'BLUE': [
        ((64, 46, 138), (125, 160, 247)),
    ],
}

TARGET_COLOR = 'RED'   # ← 캘리할 색상 변경
SHOW_FRAME   = False   # True = OpenCV 마스크 창 표시 (모니터 필요)
MIN_AREA     = 500     # 최소 유효 blob 면적


def detect_area(frame, color_name):
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
        return None, 0.0, mask

    largest = max(contours, key=cv2.contourArea)
    area    = cv2.contourArea(largest)
    if area < MIN_AREA:
        return None, 0.0, mask

    M  = cv2.moments(largest)
    cx = int(M['m10'] / M['m00']) if M['m00'] else _EFF_W // 2
    cy = int(M['m01'] / M['m00']) if M['m00'] else _EFF_H // 2
    return (cx, cy), area, mask


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        return

    print(f"[CAL] 대상 색상: {TARGET_COLOR}")
    print(f"[CAL] 실효 해상도: {_EFF_W}x{_EFF_H}")
    print("=" * 50)
    print("색지를 카메라 정면에 고정하고")
    print("실제 거리(mm)를 입력한 뒤 Enter를 누르세요.")
    print("q + Enter 로 종료 및 결과 출력")
    print("=" * 50)

    samples = []   # [(dist_mm, area, K), ...]

    import select, sys

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        if FRAME_ROTATE is not None:
            frame = cv2.rotate(frame, FRAME_ROTATE)

        centroid, area, mask = detect_area(frame, TARGET_COLOR)

        if centroid is not None:
            k_est = math.sqrt(area)   # 거리를 모르므로 일단 √area만
            print(f"\r[LIVE] area={area:8.0f}  √area={math.sqrt(area):6.1f}  "
                  f"blob=({centroid[0]},{centroid[1]})  감지O          ", end='', flush=True)
        else:
            print(f"\r[LIVE] 색지 미감지 (area<{MIN_AREA} 또는 없음)              ",
                  end='', flush=True)

        if SHOW_FRAME:
            cv2.imshow('mask', mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 논블로킹 키 입력 (터미널)
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            if line.lower() == 'q':
                break
            try:
                dist_mm = float(line)
            except ValueError:
                print(f"\n[WARN] 숫자를 입력하세요 (예: 300)")
                continue

            if centroid is None or area < MIN_AREA:
                print(f"\n[SKIP] 색지가 감지되지 않아 샘플을 기록하지 않았습니다.")
                continue

            # 여러 프레임 평균 (0.5초 동안 수집)
            areas = []
            t0 = __import__('time').time()
            while __import__('time').time() - t0 < 0.5:
                ret2, f2 = cap.read()
                if not ret2: continue
                if FRAME_ROTATE is not None:
                    f2 = cv2.rotate(f2, FRAME_ROTATE)
                _, a2, _ = detect_area(f2, TARGET_COLOR)
                if a2 >= MIN_AREA:
                    areas.append(a2)

            if not areas:
                print(f"\n[SKIP] 샘플 수집 실패")
                continue

            avg_area = sum(areas) / len(areas)
            K = dist_mm * math.sqrt(avg_area)
            samples.append((dist_mm, avg_area, K))
            print(f"\n[SAMPLE] dist={dist_mm:.0f}mm  "
                  f"avg_area={avg_area:.0f}  √area={math.sqrt(avg_area):.1f}  "
                  f"K={K:.0f}  (n={len(areas)}프레임)")

    cap.release()
    if SHOW_FRAME:
        cv2.destroyAllWindows()

    print("\n\n" + "=" * 50)
    print("캘리브레이션 결과")
    print("=" * 50)
    if not samples:
        print("샘플 없음")
        return

    print(f"{'dist(mm)':>10}  {'avg_area':>10}  {'K':>10}")
    print("-" * 35)
    for dist, area, K in samples:
        print(f"{dist:>10.0f}  {area:>10.0f}  {K:>10.0f}")

    avg_K = sum(K for _, _, K in samples) / len(samples)
    print("-" * 35)
    print(f"{'평균 K':>22}  {avg_K:>10.0f}")
    print()
    print(f"▶ camera_tracker.py 에 적용:")
    print(f"  DISTANCE_K_MM = {avg_K:.0f}")


if __name__ == '__main__':
    main()

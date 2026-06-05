"""
CLOSE bearing 캘리브레이션 스크립트
====================================
CLOSE 진입 거리(~400mm) 근방에서 색지를 여러 각도에 놓고,
카메라 측정 bearing vs 실제 bearing 을 비교해 보정 계수를 구합니다.

사용법:
  1. 색지를 CLOSE 진입 거리 내 여러 위치(X, Y)에 놓는다
     (예: 정면 350mm, 좌 150mm 등 5개 이상)
  2. 카메라가 감지하면 화면에 bearing이 표시됨
  3. [c] 키 → 터미널에 실제 X(우측+), Y(전방+) 입력 (mm)
  4. 5개 이상 수집 후 [s] 키 → 보정 결과 출력
  5. [q] 종료

출력된 CLOSE_BEARING_SCALE 값을 camera_tracker.py 에 반영하면 됩니다.
"""

import cv2
import math
import numpy as np

# ── camera_tracker.py 와 동일하게 맞출 것 ───────────────────────────────────
CAMERA_INDEX      = 0
FRAME_W           = 640
FRAME_H           = 480
FRAME_ROTATE      = cv2.ROTATE_90_COUNTERCLOCKWISE
HFOV_DEG          = 38.6
CAM_POLAR_EPSILON = 0.05
TARGET_COLOR      = 'RED'   # 캘리브레이션에 사용할 색

COLOR_RANGES = {
    'RED':    [((146, 100,  80), (179, 255, 255))],
    'YELLOW': [((18,  35, 186), ( 72, 177, 255))],
    'BLUE':   [((79,  116, 114), (119, 162, 255))],
}
# ────────────────────────────────────────────────────────────────────────────

if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    _EFF_W, _EFF_H = FRAME_H, FRAME_W
else:
    _EFF_W, _EFF_H = FRAME_W, FRAME_H

_f_px = (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))


def _detect(frame):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in COLOR_RANGES[TARGET_COLOR]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 500:
        return None
    M = cv2.moments(largest)
    if M['m00'] == 0:
        return None
    return int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])


def _bearing_close(cx, cy):
    lateral = (cx - _EFF_W / 2.0) / _f_px
    forward = (_EFF_H - cy) / _EFF_H + CAM_POLAR_EPSILON
    return -math.degrees(math.atan2(lateral, forward))


def _bearing_seek(cx):
    return -math.degrees(math.atan2(cx - _EFF_W / 2.0, _f_px))


def _compute_scale(data):
    """원점 통과 최소자승: scale = Σ(measured*true) / Σ(measured²)"""
    sm = sum(d['meas'] * d['true'] for d in data)
    ss = sum(d['meas'] ** 2          for d in data)
    return sm / ss if ss != 0 else 1.0


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        return

    print(f"=== CLOSE Bearing 캘리브레이션 ===")
    print(f"  대상 색  : {TARGET_COLOR}")
    print(f"  EFF 해상도: {_EFF_W}×{_EFF_H}   f_px={_f_px:.1f}")
    print()
    print("  [c] 현재 감지 캡처 후 실제 좌표 입력")
    print("  [s] 결과 계산 및 출력 (샘플 2개 이상 필요)")
    print("  [q] 종료")
    print()

    data = []

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        if FRAME_ROTATE is not None:
            frame = cv2.rotate(frame, FRAME_ROTATE)

        result = _detect(frame)
        disp   = frame.copy()

        if result:
            cx, cy  = result
            b_close = _bearing_close(cx, cy)
            b_seek  = _bearing_seek(cx)

            cv2.circle(disp, (cx, cy), 8, (0, 255, 0), -1)
            cv2.line(disp, (_EFF_W // 2, 0), (_EFF_W // 2, _EFF_H), (80, 80, 80), 1)
            cv2.putText(disp, f"close: {b_close:+.1f} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(disp, f"seek : {b_seek:+.1f} deg",  (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
            cv2.putText(disp, f"cx={cx}  cy={cy}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)
        else:
            cv2.putText(disp, "미감지", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        cv2.putText(disp, f"samples: {len(data)}  [c]캡처 [s]결과 [q]종료",
                    (10, _EFF_H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.imshow("Bearing Calibration", disp)
        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('c'):
            if not result:
                print("[캡처 실패] 색지가 감지되지 않았습니다.")
                continue

            cx, cy  = result
            b_close = _bearing_close(cx, cy)
            b_seek  = _bearing_seek(cx)
            print(f"\n[캡처 #{len(data)+1}]  cx={cx}  cy={cy}")
            print(f"  bearing_close = {b_close:+.2f}°   bearing_seek = {b_seek:+.2f}°")
            try:
                x_mm   = float(input("  실제 X (mm, 우측+, 좌측-): "))
                y_mm   = float(input("  실제 Y (mm, 전방+)       : "))
            except ValueError:
                print("  숫자 입력 오류, 스킵")
                continue

            true_b = math.degrees(math.atan2(x_mm, y_mm))
            data.append({
                'cx': cx, 'cy': cy,
                'x_mm': x_mm, 'y_mm': y_mm,
                'meas': b_close,
                'true': true_b,
            })
            print(f"  true_bearing  = {true_b:+.2f}°  "
                  f"오차 = {b_close - true_b:+.2f}°  "
                  f"[총 {len(data)}개 누적]")

        elif key == ord('s'):
            if len(data) < 2:
                print("[결과 출력 불가] 샘플이 2개 이상 필요합니다.")
                continue

            scale = _compute_scale(data)

            print(f"\n{'='*55}")
            print(f"  샘플 수: {len(data)}")
            print()
            print(f"  {'#':>2}  {'cx':>4}{'cy':>5}  {'X':>6}{'Y':>6}  "
                  f"{'true':>7}  {'meas':>7}  {'보정후':>7}  {'오차':>7}")
            total_err_before = 0.0
            total_err_after  = 0.0
            for i, d in enumerate(data):
                corrected = d['meas'] * scale
                err_b = d['meas']      - d['true']
                err_a = corrected      - d['true']
                total_err_before += err_b ** 2
                total_err_after  += err_a ** 2
                print(f"  {i+1:>2}  {d['cx']:>4}{d['cy']:>5}  "
                      f"{d['x_mm']:>6.0f}{d['y_mm']:>6.0f}  "
                      f"{d['true']:>+7.2f}  {d['meas']:>+7.2f}  "
                      f"{corrected:>+7.2f}  {err_a:>+7.2f}")

            rmse_before = math.sqrt(total_err_before / len(data))
            rmse_after  = math.sqrt(total_err_after  / len(data))
            print()
            print(f"  RMSE 보정 전: {rmse_before:.2f}°   보정 후: {rmse_after:.2f}°")
            print()
            print(f"  ★ camera_tracker.py 에 적용할 값:")
            print(f"    CLOSE_BEARING_SCALE = {scale:.4f}")
            print(f"{'='*55}")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

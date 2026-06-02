"""
HSV Tuner — 카메라 색상 범위 실시간 캘리브레이션 도구
사용법:
  python3 hsv_tuner.py

화면 구성: [원본 + 컨투어] | [마스크]
조작:
  트랙바 — H/S/V 범위 조정
  'p'    — 현재 HSV 범위를 터미널에 출력 (COLOR_RANGES 복붙용)
  'q'    — 종료
"""

import cv2
import numpy as np

CAMERA_INDEX  = 0
FRAME_W       = 640
FRAME_H       = 480
FRAME_ROTATE  = cv2.ROTATE_90_COUNTERCLOCKWISE   # camera_tracker.py 와 동일
DISPLAY_SCALE = 2.0                               # 표시 배율 (2.0 = 200%)

if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    EFF_W, EFF_H = FRAME_H, FRAME_W
else:
    EFF_W, EFF_H = FRAME_W, FRAME_H

DISP_W = int(EFF_W * DISPLAY_SCALE)
DISP_H = int(EFF_H * DISPLAY_SCALE)


def nothing(_):
    pass


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        print("ERROR: 카메라를 열 수 없습니다. CAMERA_INDEX를 확인하세요.")
        return

    win = 'HSV Tuner'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, DISP_W * 2, DISP_H)

    cv2.createTrackbar('H_lo', win,   0, 179, nothing)
    cv2.createTrackbar('H_hi', win, 179, 179, nothing)
    cv2.createTrackbar('S_lo', win, 100, 255, nothing)
    cv2.createTrackbar('S_hi', win, 255, 255, nothing)
    cv2.createTrackbar('V_lo', win,  80, 255, nothing)
    cv2.createTrackbar('V_hi', win, 255, 255, nothing)

    print(f"HSV Tuner 시작.  EFF={EFF_W}x{EFF_H}  display={DISP_W}x{DISP_H}")
    print("'p' = 범위 출력,  'q' = 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        if FRAME_ROTATE is not None:
            frame = cv2.rotate(frame, FRAME_ROTATE)

        h_lo = cv2.getTrackbarPos('H_lo', win)
        h_hi = cv2.getTrackbarPos('H_hi', win)
        s_lo = cv2.getTrackbarPos('S_lo', win)
        s_hi = cv2.getTrackbarPos('S_hi', win)
        v_lo = cv2.getTrackbarPos('V_lo', win)
        v_hi = cv2.getTrackbarPos('V_hi', win)

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([h_lo, s_lo, v_lo]),
                           np.array([h_hi, s_hi, v_hi]))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        overlay = frame.copy()
        area    = 0.0
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area    = cv2.contourArea(largest)
            cv2.drawContours(overlay, [largest], -1, (0, 255, 0), 2)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(overlay, (cx, cy), 8, (0, 0, 255), -1)
                cv2.putText(overlay, f"area={area:.0f}  cx={cx}",
                            (cx + 12, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 중심선 (회전 후 기준)
        cv2.line(overlay, (EFF_W // 2, 0), (EFF_W // 2, EFF_H), (180, 180, 180), 1)

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        left     = cv2.resize(overlay,  (DISP_W, DISP_H))
        right    = cv2.resize(mask_bgr, (DISP_W, DISP_H))
        combined = np.hstack([left, right])

        label = (f"H:[{h_lo},{h_hi}]  S:[{s_lo},{s_hi}]  V:[{v_lo},{v_hi}]"
                 f"    area={area:.0f}    p=출력  q=종료")
        cv2.putText(combined, label, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow(win, combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('p'):
            print(f"\n--- 현재 HSV 범위 ---")
            print(f"  H:[{h_lo}, {h_hi}]  S:[{s_lo}, {s_hi}]  V:[{v_lo}, {v_hi}]")
            print(f"  → COLOR_RANGES 복붙용:")
            print(f"    (({h_lo}, {s_lo}, {v_lo}), ({h_hi}, {s_hi}, {v_hi})),")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

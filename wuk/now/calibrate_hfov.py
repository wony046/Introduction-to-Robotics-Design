"""
HFOV_DEG 캘리브레이션 도구
━━━━━━━━━━━━━━━━━━━━━━━━━━
원리:
  바닥/벽에 두 표시(P1, P2)를 W_mm 간격으로 놓고
  로봇 바퀴 축에서 D_mm 앞에 정지.
  화면에서 두 표시를 클릭 → 픽셀 간격으로 HFOV 계산.

  f_px  = pixel_span × D / W
  HFOV  = 2 × atan(EFF_W/2 / f_px)

물리 준비:
  1. 테이프 두 장을 정확히 W_mm 간격으로 바닥/벽에 붙임
     (예: 400mm — 자로 정확히 측정)
  2. 로봇을 바퀴 축 중심이 두 테이프 중앙에서 D_mm 앞에 오도록 정지
     (예: 1000mm)
  3. 스크립트 실행 → D, W 입력

조작:
  마우스 클릭 : P1 → P2 순서로 두 표시 클릭
  s           : 현재 샘플 저장 후 초기화
  r           : 클릭 초기화
  q           : 종료 및 평균 HFOV 출력
"""

import cv2
import math
import numpy as np

# ── 카메라 설정 (camera_tracker.py와 동일) ─────────────────────────────────
CAMERA_INDEX = 0
FRAME_W      = 1280
FRAME_H      = 720
FRAME_ROTATE = cv2.ROTATE_90_COUNTERCLOCKWISE

if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    _EFF_W, _EFF_H = FRAME_H, FRAME_W
else:
    _EFF_W, _EFF_H = FRAME_W, FRAME_H

DISPLAY_SCALE = 0.55   # 창 크기 조정 (화면이 작으면 0.4로)

# ── 전역 상태 ──────────────────────────────────────────────────────────────
_clicks  = []    # [(x1,y1), (x2,y2)]
_samples = []    # 저장된 HFOV 값 목록
_frozen  = None  # 저장 직후 잠시 표시용 스냅샷


def _mouse_cb(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(_clicks) < 2:
        _clicks.append((x, y))


def _compute(px1, px2, D_mm, W_mm):
    """두 픽셀 클릭으로 f_px, HFOV 계산."""
    span = abs(px2[0] - px1[0])
    if span == 0:
        return None, None
    f_px = span * D_mm / W_mm
    hfov = 2.0 * math.degrees(math.atan2(_EFF_W / 2.0, f_px))
    return f_px, hfov


def _draw(frame, D_mm, W_mm):
    """화면 렌더링. display 이미지 반환."""
    disp = frame.copy()
    font   = cv2.FONT_HERSHEY_SIMPLEX
    shadow = (0, 0, 0)

    # 중심선
    cx = _EFF_W // 2
    cv2.line(disp, (cx, 0), (cx, _EFF_H), (160, 160, 160), 1)

    # 클릭 포인트
    colors = [(60, 60, 255), (60, 230, 60)]
    labels = ['P1 (왼쪽)', 'P2 (오른쪽)']
    for i, (px, py) in enumerate(_clicks):
        c = colors[i]
        cv2.circle(disp, (px, py), 10, (255, 255, 255), 3)
        cv2.circle(disp, (px, py), 8,  c, -1)
        cv2.putText(disp, labels[i], (px + 14, py + 6), font, 0.7, shadow, 3, cv2.LINE_AA)
        cv2.putText(disp, labels[i], (px + 14, py + 6), font, 0.7, c,      2, cv2.LINE_AA)

    # P1-P2 연결선 + 픽셀 간격
    hfov_now = None
    f_px_now = None
    if len(_clicks) == 2:
        f_px_now, hfov_now = _compute(_clicks[0], _clicks[1], D_mm, W_mm)
        if hfov_now:
            span = abs(_clicks[1][0] - _clicks[0][0])
            mid  = ((_clicks[0][0] + _clicks[1][0]) // 2,
                    (min(_clicks[0][1], _clicks[1][1])) - 18)
            cv2.line(disp, _clicks[0], _clicks[1], (255, 210, 0), 2)
            cv2.putText(disp, f"{span}px", mid, font, 0.75, shadow,        3, cv2.LINE_AA)
            cv2.putText(disp, f"{span}px", mid, font, 0.75, (255, 210, 0), 2, cv2.LINE_AA)

    # ── 정보 패널 ─────────────────────────────────────────────────────────
    y = 42

    def put(text, color=(240, 240, 240), scale=0.82):
        nonlocal y
        cv2.putText(disp, text, (12, y + 2), font, scale, shadow, 3, cv2.LINE_AA)
        cv2.putText(disp, text, (10, y),     font, scale, color,  2, cv2.LINE_AA)
        y += 36

    put(f"D = {D_mm:.0f} mm   W = {W_mm:.0f} mm")

    if len(_clicks) == 0:
        put("→ P1 클릭 (왼쪽 표시)",  color=(255, 200, 80))
    elif len(_clicks) == 1:
        put("→ P2 클릭 (오른쪽 표시)", color=(255, 200, 80))
    else:
        put("→ s=저장   r=재클릭",     color=(255, 200, 80))

    if hfov_now:
        put(f"HFOV  = {hfov_now:.2f} deg", color=(80, 255, 160))
        put(f"f_px  = {f_px_now:.1f} px",  color=(80, 255, 160))

    if _samples:
        avg = sum(_samples) / len(_samples)
        put(f"평균 HFOV = {avg:.2f} deg  (n={len(_samples)})", color=(80, 220, 255))

    # 하단 안내
    guide = "s=저장  r=초기화  q=종료"
    cv2.putText(disp, guide, (10, _EFF_H - 16), font, 0.6, shadow, 3, cv2.LINE_AA)
    cv2.putText(disp, guide, (10, _EFF_H - 16), font, 0.6, (200, 200, 200), 1, cv2.LINE_AA)

    # 축소
    h, w = disp.shape[:2]
    disp = cv2.resize(disp, (int(w * DISPLAY_SCALE), int(h * DISPLAY_SCALE)))
    return disp


def main():
    global _clicks, _frozen

    print("=" * 50)
    print("  HFOV 캘리브레이션")
    print("=" * 50)
    print("두 테이프 표시를 바닥/벽에 붙이고")
    print("로봇 바퀴 축을 그 앞 D_mm에 정지하세요.")
    print()

    try:
        D_mm = float(input("D_mm (바퀴축 ~ 표시 거리,  예 1000): ").strip())
        W_mm = float(input("W_mm (두 표시 사이 실제 폭, 예  400): ").strip())
    except ValueError:
        print("[ERROR] 숫자를 입력하세요.")
        return

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        return

    win = 'HFOV Calibration'
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, _mouse_cb)

    print()
    print("창에서 P1(왼쪽 표시) → P2(오른쪽 표시) 순서로 클릭")
    print("s=저장  r=초기화  q=종료")

    freeze_until = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        if FRAME_ROTATE is not None:
            frame = cv2.rotate(frame, FRAME_ROTATE)

        now = __import__('time').time()

        if now < freeze_until and _frozen is not None:
            cv2.imshow(win, _frozen)
        else:
            _frozen = None
            disp = _draw(frame, D_mm, W_mm)
            cv2.imshow(win, disp)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('r'):
            _clicks.clear()

        elif key == ord('s') and len(_clicks) == 2:
            f_px, hfov = _compute(_clicks[0], _clicks[1], D_mm, W_mm)
            if hfov:
                _samples.append(hfov)
                span = abs(_clicks[1][0] - _clicks[0][0])
                print(f"[SAMPLE #{len(_samples)}] "
                      f"span={span}px  f_px={f_px:.1f}  HFOV={hfov:.2f}°")
                # 저장 확인 표시 (0.8초 프리즈)
                _frozen     = _draw(frame, D_mm, W_mm)
                freeze_until = now + 0.8
                _clicks.clear()

    cap.release()
    cv2.destroyAllWindows()

    # ── 결과 출력 ──────────────────────────────────────────────────────────
    print("\n" + "=" * 45)
    print("캘리브레이션 결과")
    print("=" * 45)
    if not _samples:
        print("샘플 없음")
        return

    for i, h in enumerate(_samples, 1):
        print(f"  #{i:2d} : {h:.2f}°")

    avg = sum(_samples) / len(_samples)
    print("-" * 30)
    print(f"  평균 : {avg:.2f}°")
    print()
    print("▶ camera_tracker.py 에 적용:")
    print(f"  HFOV_DEG = {avg:.1f}")


if __name__ == '__main__':
    main()

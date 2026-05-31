"""
HFOV_DEG 캘리브레이션 도구 (시각화)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
테이프 색상: 흰색 또는 주황색 추천
  → 빨강/노랑/파랑은 미션 색지와 혼동 가능

물리 준비:
  1. 흰색(또는 주황) 테이프 두 장을 바닥/벽에 정확히 W_mm 간격으로 붙임
  2. 로봇 바퀴 축 중심을 테이프 중앙에서 D_mm 앞에 정지

조작:
  마우스 클릭 : P1(왼쪽) → P2(오른쪽) 순서로 클릭
  s           : 샘플 저장
  r           : 클릭 초기화
  q           : 종료
"""

import cv2
import math
import time
import numpy as np

# ── 카메라 설정 (camera_tracker.py와 동일) ─────────────────────────────────
CAMERA_INDEX = 0
FRAME_W      = 640
FRAME_H      = 480
FRAME_ROTATE = cv2.ROTATE_90_COUNTERCLOCKWISE

if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
    _EFF_W, _EFF_H = FRAME_H, FRAME_W
else:
    _EFF_W, _EFF_H = FRAME_W, FRAME_H

DISPLAY_SCALE = 0.55

# ── 색상 상수 ──────────────────────────────────────────────────────────────
C_WHITE  = (255, 255, 255)
C_BLACK  = (0,   0,   0  )
C_GRAY   = (160, 160, 160)
C_YELLOW = (0,   220, 255)
C_GREEN  = (60,  230, 60 )
C_RED    = (60,  60,  255)
C_CYAN   = (255, 220, 0  )
C_ORANGE = (0,   160, 255)

# ── 전역 상태 ──────────────────────────────────────────────────────────────
_clicks      = []
_samples     = []
_mouse_pos   = (0, 0)
_freeze_disp = None
_freeze_until = 0.0


def _mouse_cb(event, x, y, flags, param):
    global _mouse_pos
    # 축소 화면 좌표 → 원본 이미지 좌표로 역변환
    ix = int(x / DISPLAY_SCALE)
    iy = int(y / DISPLAY_SCALE)
    _mouse_pos = (ix, iy)
    if event == cv2.EVENT_LBUTTONDOWN and len(_clicks) < 2:
        _clicks.append((ix, iy))


def _compute(p1, p2, D_mm, W_mm):
    span = abs(p2[0] - p1[0])
    if span == 0:
        return None, None
    f_px = span * D_mm / W_mm
    hfov = 2.0 * math.degrees(math.atan2(_EFF_W / 2.0, f_px))
    return f_px, hfov


def _put(img, text, pos, scale=0.80, color=C_WHITE, thickness=2):
    x, y = pos
    cv2.putText(img, text, (x+2, y+2), cv2.FONT_HERSHEY_SIMPLEX,
                scale, C_BLACK, thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, (x,   y  ), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color,   thickness,     cv2.LINE_AA)


def _draw_crosshair(img, x, y, color, size=18, thickness=1):
    cv2.line(img, (x - size, y), (x + size, y), color, thickness)
    cv2.line(img, (x, y - size), (x, y + size), color, thickness)
    cv2.circle(img, (x, y), 6, color, thickness)


def _draw_step_panel(img, step):
    """왼쪽 상단 단계 안내 패널."""
    steps = [
        (1, "P1 클릭  (왼쪽 테이프)"),
        (2, "P2 클릭  (오른쪽 테이프)"),
        (3, "s 키로 저장"),
    ]
    panel_x, panel_y = 10, 10
    panel_w, panel_h = 310, len(steps) * 38 + 16
    overlay = img.copy()
    cv2.rectangle(overlay, (panel_x, panel_y),
                  (panel_x + panel_w, panel_y + panel_h),
                  (30, 30, 30), -1)
    cv2.addWeighted(overlay, 0.65, img, 0.35, 0, img)
    cv2.rectangle(img, (panel_x, panel_y),
                  (panel_x + panel_w, panel_y + panel_h),
                  C_GRAY, 1)

    for i, (n, text) in enumerate(steps):
        y = panel_y + 16 + i * 38
        if n < step:
            color = (80, 180, 80)
            mark  = "✓"
        elif n == step:
            color = C_YELLOW
            mark  = "▶"
        else:
            color = (120, 120, 120)
            mark  = "  "
        _put(img, f"{mark} Step {n}: {text}", (panel_x + 10, y),
             scale=0.72, color=color)


def _draw_result_panel(img, f_px, hfov, samples):
    """오른쪽 하단 결과 패널."""
    lines = []
    if hfov is not None:
        lines.append(("현재 HFOV", f"{hfov:.2f} deg", C_GREEN))
        lines.append(("현재 f_px", f"{f_px:.1f} px",  C_GREEN))
    if samples:
        avg = sum(samples) / len(samples)
        lines.append(("평균 HFOV", f"{avg:.2f} deg  (n={len(samples)})", C_CYAN))

    if not lines:
        return

    pw = 300
    ph = len(lines) * 36 + 16
    px = _EFF_W - pw - 10
    py = _EFF_H - ph - 10

    overlay = img.copy()
    cv2.rectangle(overlay, (px, py), (px + pw, py + ph), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.70, img, 0.30, 0, img)
    cv2.rectangle(img, (px, py), (px + pw, py + ph), C_GRAY, 1)

    for i, (label, value, color) in enumerate(lines):
        y = py + 16 + i * 36
        _put(img, f"{label}: {value}", (px + 10, y), scale=0.72, color=color)


def _draw(frame, D_mm, W_mm, step):
    disp = frame.copy()

    # ── 중심 수직선 ───────────────────────────────────────────────────────
    cx = _EFF_W // 2
    cv2.line(disp, (cx, 0), (cx, _EFF_H), C_GRAY, 1)

    # ── 마우스 커서 십자선 ────────────────────────────────────────────────
    mx, my = _mouse_pos
    if 0 < mx < _EFF_W and 0 < my < _EFF_H:
        _draw_crosshair(disp, mx, my, (200, 200, 80), size=20, thickness=1)

    # ── 클릭된 포인트 ─────────────────────────────────────────────────────
    point_cfg = [(C_RED, "P1"), (C_GREEN, "P2")]
    for i, (px, py) in enumerate(_clicks):
        color, label = point_cfg[i]
        # 수직 가이드선
        cv2.line(disp, (px, 0), (px, _EFF_H), color, 1)
        # 포인트 마커
        cv2.circle(disp, (px, py), 14, C_WHITE, 2)
        cv2.circle(disp, (px, py), 12, color,   -1)
        _put(disp, label, (px + 16, py + 6), scale=0.85, color=color)

    # ── P1-P2 연결선 + 픽셀 간격 + 계산 결과 ─────────────────────────────
    f_px_now = None
    hfov_now = None
    if len(_clicks) == 2:
        f_px_now, hfov_now = _compute(_clicks[0], _clicks[1], D_mm, W_mm)
        if hfov_now:
            p1x, p1y = _clicks[0]
            p2x, p2y = _clicks[1]
            span = abs(p2x - p1x)
            mid_x = (p1x + p2x) // 2
            mid_y = min(p1y, p2y) - 24

            # 수평 연결선 (클릭 y 평균)
            avg_y = (p1y + p2y) // 2
            cv2.line(disp, (p1x, avg_y), (p2x, avg_y), C_CYAN, 2)
            # 양쪽 세로 눈금
            cv2.line(disp, (p1x, avg_y - 10), (p1x, avg_y + 10), C_CYAN, 2)
            cv2.line(disp, (p2x, avg_y - 10), (p2x, avg_y + 10), C_CYAN, 2)
            # 픽셀 간격 표시
            _put(disp, f"{span} px", (mid_x - 30, mid_y), scale=0.80, color=C_CYAN)

    # ── 파라미터 표시 (상단 오른쪽) ──────────────────────────────────────
    _put(disp, f"D = {D_mm:.0f} mm", (_EFF_W - 200, 36),  scale=0.78, color=C_WHITE)
    _put(disp, f"W = {W_mm:.0f} mm", (_EFF_W - 200, 72),  scale=0.78, color=C_WHITE)

    # ── 단계 패널 ─────────────────────────────────────────────────────────
    _draw_step_panel(disp, step)

    # ── 결과 패널 ─────────────────────────────────────────────────────────
    _draw_result_panel(disp, f_px_now, hfov_now, _samples)

    # ── 하단 조작 안내 ────────────────────────────────────────────────────
    guide = "s = 저장    r = 초기화    q = 종료"
    _put(disp, guide, (10, _EFF_H - 14), scale=0.62, color=C_GRAY, thickness=1)

    # 축소
    h, w = disp.shape[:2]
    return cv2.resize(disp, (int(w * DISPLAY_SCALE), int(h * DISPLAY_SCALE)))


def main():
    global _freeze_disp, _freeze_until

    print("=" * 55)
    print("  HFOV 캘리브레이션")
    print("=" * 55)
    print()
    print("테이프 색상:  흰색 또는 주황색 권장")
    print("  (빨강/노랑/파랑은 미션 색지와 혼동 주의)")
    print()
    print("준비:")
    print("  1. 흰색 테이프 2장을 바닥에 W_mm 간격으로 붙임")
    print("  2. 로봇 바퀴 축 중심을 테이프 중앙에서 D_mm 앞에 정지")
    print()

    try:
        D_mm = float(input("D_mm (바퀴 축 ~ 테이프 거리,  예 1000): ").strip())
        W_mm = float(input("W_mm (두 테이프 사이 실제 폭, 예  400): ").strip())
    except ValueError:
        print("[ERROR] 숫자를 입력하세요.")
        return

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        return

    # 실제 캡처 해상도 확인
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"\n[카메라] 요청: {FRAME_W}x{FRAME_H}  실제: {actual_w}x{actual_h}")
    if actual_w != FRAME_W or actual_h != FRAME_H:
        print(f"  ★ 해상도 불일치! camera_tracker.py의 FRAME_W/FRAME_H를")
        print(f"     FRAME_W={actual_w}, FRAME_H={actual_h} 로 수정하세요.")
        print(f"  ★ 캘리브레이션은 실제 해상도 기준으로 계속 진행합니다.")
    else:
        print(f"  해상도 일치 OK")

    win = 'HFOV Calibration'
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, _mouse_cb)

    print()
    print("창에서 P1(왼쪽 테이프) → P2(오른쪽 테이프) 순으로 클릭")
    print("s=저장  r=초기화  q=종료")

    last_frame = None

    while True:
        ret, frame = cap.read()
        if ret:
            if FRAME_ROTATE is not None:
                frame = cv2.rotate(frame, FRAME_ROTATE)
            last_frame = frame

        if last_frame is None:
            continue

        now = time.time()

        if now < _freeze_until and _freeze_disp is not None:
            cv2.imshow(win, _freeze_disp)
        else:
            _freeze_disp = None
            step = len(_clicks) + 1
            if len(_clicks) == 2:
                step = 3
            disp = _draw(last_frame, D_mm, W_mm, step)
            cv2.imshow(win, disp)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('r'):
            _clicks.clear()
            print("[RESET] 클릭 초기화")

        elif key == ord('s') and len(_clicks) == 2:
            f_px, hfov = _compute(_clicks[0], _clicks[1], D_mm, W_mm)
            if hfov:
                _samples.append(hfov)
                span = abs(_clicks[1][0] - _clicks[0][0])
                print(f"[SAMPLE #{len(_samples)}] "
                      f"span={span}px  f_px={f_px:.1f}  HFOV={hfov:.2f}°")
                # 저장 확인 화면 0.8초 유지
                step_disp = _draw(last_frame, D_mm, W_mm, 3)

                # "저장됨!" 오버레이
                h, w = step_disp.shape[:2]
                _put(step_disp, f"저장됨!  HFOV={hfov:.2f}°",
                     (w // 2 - 140, h // 2),
                     scale=1.1, color=(60, 255, 100), thickness=2)
                _freeze_disp  = step_disp
                _freeze_until = now + 0.8
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

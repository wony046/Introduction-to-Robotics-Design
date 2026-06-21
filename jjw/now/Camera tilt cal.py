#!/usr/bin/env python3
"""
camera_tilt_cal.py
카메라 높이 변경 후 CAM_TILT_DEG / 거리추정 재캘리브레이션 도구.

camera_tracker.py 의 검출 로직(_detect_color)과 상수를 그대로 import 하므로
본 코드와 동일한 기하로 계산된다.

[사용법]
 1) 먼저 camera_tracker.py 의 CAM_HEIGHT_MM 을 새로 실측한 값으로 수정.
 2) 색지를 바닥에 두고, "카메라 렌즈 바로 아래 바닥점 → 색지 중심" 수평거리를
    자로 측정 → 아래 KNOWN_DIST_MM 에 입력.
 3) 실행 후 색지를 화면 중앙 부근에 두면 매 프레임:
       cy / delta_v / (현재상수)추정거리 / 역산된 CAM_TILT_DEG / 최근30평균
    이 출력된다.
 4) tilt_avg 가 수렴하면 그 값을 camera_tracker.CAM_TILT_DEG 에 반영.
 5) KNOWN_DIST_MM 을 400 / 600 / 900 등으로 바꿔가며 반복.
    → tilt 가 거리에 무관하게 일정하면 CAM_HEIGHT_MM 이 정확한 것.
       거리에 따라 한 방향으로 흐르면 높이값을 다시 조정할 것.

[주의] 로봇 메인(rpi_avoid)과 동시에 실행하지 말 것 (카메라 장치 충돌).
"""
import cv2
import math
import time
import camera_tracker as ct

# ── 설정 ─────────────────────────────────────────────────────────────
KNOWN_DIST_MM = 600.0     # ★ 렌즈 바로 아래 바닥점 → 색지 중심 수평거리 (mm)
TARGET_COLOR  = 'RED'     # 캘리브레이션용 색 ('RED'/'YELLOW'/'BLUE')
AVG_WINDOW    = 30        # 이동평균 샘플 수

# ── 초기화 (camera_tracker 상수 재사용) ──────────────────────────────
f_px = (ct._EFF_W / 2.0) / math.tan(math.radians(ct.HFOV_DEG / 2.0))

cap = cv2.VideoCapture(ct.CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  ct.FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ct.FRAME_H)
if not cap.isOpened():
    raise SystemExit("[CAL] 카메라를 열 수 없습니다. ct.CAMERA_INDEX 확인.")

print(f"[CAL] H={ct.CAM_HEIGHT_MM:.0f}mm  f_px={f_px:.1f}  "
      f"_EFF_W={ct._EFF_W} _EFF_H={ct._EFF_H}  "
      f"known={KNOWN_DIST_MM:.0f}mm  color={TARGET_COLOR}")
print(f"[CAL] 현재 CAM_TILT_DEG={ct.CAM_TILT_DEG:.2f}  (역산값과 비교용)")
print("[CAL] Ctrl+C 로 종료. tilt_avg 가 수렴하면 그 값을 사용하세요.\n")

samples = []
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.03)
            continue
        if ct.FRAME_ROTATE is not None:
            frame = cv2.rotate(frame, ct.FRAME_ROTATE)

        centroid, area, _, _ = ct._detect_color(frame, TARGET_COLOR)
        if centroid is None:
            print("[CAL] 미감지 ...                                        ", end='\r')
            time.sleep(0.05)
            continue

        cx, cy  = centroid
        delta_v = math.degrees(math.atan2(cy - ct._EFF_H / 2.0, f_px))

        # 현재 상수로 추정한 거리 (검증용)
        dep_now = ct.CAM_TILT_DEG + delta_v
        est_now = (ct.CAM_HEIGHT_MM / math.tan(math.radians(dep_now))
                   if dep_now > 1.0 else 9999.0)

        # 기지거리 → 올바른 tilt 역산
        dep_true = math.degrees(math.atan(ct.CAM_HEIGHT_MM / KNOWN_DIST_MM))
        tilt_fit = dep_true - delta_v

        samples.append(tilt_fit)
        if len(samples) > AVG_WINDOW:
            samples.pop(0)
        tilt_avg = sum(samples) / len(samples)

        print(f"[CAL] cy={cy:3d}  delta_v={delta_v:+6.2f}  "
              f"est_now={est_now:6.0f}mm  "
              f"tilt_fit={tilt_fit:+6.2f}  avg{len(samples):02d}={tilt_avg:+6.2f}",
              end='\r')
        time.sleep(0.05)

except KeyboardInterrupt:
    print()
    if samples:
        avg = sum(samples) / len(samples)
        print(f"\n[CAL] ===> 권장 CAM_TILT_DEG = {avg:.2f}")
        print(f"[CAL]      camera_tracker.py 의 CAM_TILT_DEG 에 반영하세요.")
        print(f"[CAL]      이어서 KNOWN_DIST_MM 을 다른 값으로 바꿔 일관성 확인 권장.")
    else:
        print("[CAL] 샘플 없음 (색지 미감지).")
finally:
    cap.release()

import cv2
import math
import numpy as np

# ── 설정 값 (수정된 높이 반영) ──────────────────────────────────
CAMERA_INDEX  = 0
FRAME_W       = 640
FRAME_H       = 400
HFOV_DEG      = 38.6      # 기존 화각 유지
CAM_HEIGHT_MM = 590.0     # ★ 새로 측정한 카메라 수직 높이 (59cm)
TARGET_DIST   = 500.0     # 캘리브레이션을 위해 바닥에 놓은 색지까지의 실제 수평 거리 (mm)

# 기존 코드의 회전 설정
FRAME_ROTATE  = cv2.ROTATE_90_COUNTERCLOCKWISE
_EFF_W, _EFF_H = FRAME_H, FRAME_W  # 회전 후 해상도: 480 x 640

# 기존 RED 색상 범위 및 CLAHE
COLOR_RANGES_RED = [((28, 152, 110), (255, 212, 170))]
_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

def get_recommended_tilt(cy):
    """cy 픽셀을 바탕으로 추천 CAM_TILT_DEG를 계산"""
    # 1. 렌즈의 초점 거리(픽셀 단위) 계산
    f_px = (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
    
    # 2. 화면 중심(cy = 320) 기준 수직 편차 각도
    delta_v = math.degrees(math.atan2(cy - _EFF_H / 2.0, f_px))
    
    # 3. 이상적인 전체 하향 각도
    target_depression = math.degrees(math.atan2(CAM_HEIGHT_MM, TARGET_DIST))
    
    # 4. 카메라 자체의 기울기 역산
    tilt_deg = target_depression - delta_v
    return tilt_deg, delta_v, target_depression

def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    print(f"=== 카메라 기울기(Tilt) 캘리브레이션 시작 ===")
    print(f"1. 로봇 렌즈(바닥 수직점)로부터 정확히 {TARGET_DIST}mm 앞 바닥에 '빨간색' 색지를 놓아주세요.")
    print(f"2. 아래 출력되는 '추천 CAM_TILT_DEG' 값을 확인하세요.")
    print(f"3. 종료하려면 'q'를 누르세요.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
            
        frame = cv2.rotate(frame, FRAME_ROTATE)
        
        # LAB 변환 및 전처리
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = _clahe.apply(l)
        lab_frame = cv2.merge([l, a, b])
        
        # 색상 마스크 생성
        mask = np.zeros(lab_frame.shape[:2], dtype=np.uint8)
        for (lo, hi) in COLOR_RANGES_RED:
            mask |= cv2.inRange(lab_frame, np.array(lo), np.array(hi))
            
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                M = cv2.moments(largest)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    tilt_deg, delta_v, target_dep = get_recommended_tilt(cy)
                    
                    # 시각화 (선택 사항: GUI 환경이 아닐 경우 콘솔만 확인해도 됨)
                    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                    cv2.putText(frame, f"cy: {cy} px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    cv2.putText(frame, f"Tilt: {tilt_deg:.1f} deg", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    
                    print(f"\r색상 감지됨! cy={cy:3d}px | 편차={delta_v:+.1f}° | 추천 CAM_TILT_DEG = {tilt_deg:.1f}°", end="")
        else:
            print("\r색상을 찾는 중... (빨간색 색지를 500mm 앞에 놓아주세요)", end="")

        cv2.imshow('Calibration', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("\n종료되었습니다.")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

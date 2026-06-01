import cv2
import numpy as np
import time
import math

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [1] 테스트 파라미터 설정
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CAMERA_INDEX = 0

# 🎯 3단 분할 도착 판정 조건 (엄격도 조절 가능)
ARRIVE_FILL_RATIO = 0.40     # [하단 0~30%] 영역: 목표 색상으로 40% 이상 채워져야 함 (차체 바로 밑)
MID_EMPTY_LIMIT = 0.10       # [중단 30~50%] 영역: 목표 색상이 10% 미만이어야 함 (90% 이상 비어있어야 함)
TOP_EMPTY_LIMIT = 0.10       # [상단 50~100%] 영역: 목표 색상이 10% 미만이어야 함

MIN_CONTOUR_AREA = 500       # 무시할 노이즈 크기

MISSION_COLORS = ['RED', 'YELLOW', 'BLUE']

# 💡 [수정] 조명 및 장애물 구분을 위해 튜닝된 HSV 임계값
COLOR_HSV_RANGES = {
    'RED':    [(0, 100, 100), (10, 255, 255), (160, 100, 100), (180, 255, 255)],
    'YELLOW': [(20, 100, 100), (35, 255, 255)],
    # [수정] 파란색은 명도(V) 하한을 130으로 확 올려서 어두운 장애물(남색)을 완전히 컷오프!
    'BLUE':   [(95, 100, 130), (130, 255, 255)] 
}

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [2] 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def main():
    print("=== Vision Only Test (3-Tier ROI & Strict Blue) ===")
    
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    
    kernel = np.ones((5,5), np.uint8)
    
    current_color_idx = 0
    mission_phase = 0  
    arrive_time = 0.0

    while True:
        ret, raw_frame = cap.read()
        if not ret: break

        # 카메라 90도 좌측 원상복구
        frame = cv2.rotate(raw_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        h, w, _ = frame.shape 

        target_name = MISSION_COLORS[current_color_idx % len(MISSION_COLORS)]
        
        if mission_phase == 1:
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, h), (0, 255, 0), -1)
            frame = cv2.addWeighted(overlay, 0.3, frame, 0.7, 0)
            cv2.putText(frame, f"ARRIVED: {target_name}!", (20, h//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            
            if time.time() - arrive_time > 2.0:
                current_color_idx += 1          
                mission_phase = 0               
                print(f"\n[MISSION] Next Target: {MISSION_COLORS[current_color_idx % len(MISSION_COLORS)]}")
            
            cv2.imshow("Robot Vision Test", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            continue

        hsv_ranges = COLOR_HSV_RANGES[target_name]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        mask = None
        if target_name == 'RED':
            mask1 = cv2.inRange(hsv, hsv_ranges[0], hsv_ranges[1])
            mask2 = cv2.inRange(hsv, hsv_ranges[2], hsv_ranges[3])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, hsv_ranges[0], hsv_ranges[1])

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        found = False
        bot_fill, mid_fill, top_fill, align_angle, err_x = 0.0, 0.0, 0.0, 0.0, 0.0
        
        # ── 3단 화면 분할 y좌표 설정 (밑에서부터 계산) ──
        roi_mid_top_y = int(h * 0.5)  # 위에서 50% 지점 (중단 시작점)
        roi_bot_top_y = int(h * 0.7)  # 위에서 70% 지점 (하단 시작점)

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                found = True
                x, y, box_w, box_h = cv2.boundingRect(c)
                cx = x + box_w // 2
                err_x = (cx - (w / 2)) / (w / 2)
                
                rect = cv2.minAreaRect(c)
                box = np.int32(cv2.boxPoints(rect))
                pts_y_sorted = sorted(box, key=lambda p: p[1])
                top_2 = pts_y_sorted[:2]
                tl, tr = sorted(top_2, key=lambda p: p[0])
                dx = tr[0] - tl[0]
                dy = tr[1] - tl[1]
                align_angle = math.degrees(math.atan2(dy, dx)) if dx != 0 else 0.0

                # ── 1. [하단 0~30%] 채움 비율 ──
                bottom_roi = mask[roi_bot_top_y:h, 0:w]
                white_bot = cv2.countNonZero(bottom_roi)
                total_bot = (h - roi_bot_top_y) * w
                bot_fill = (white_bot / total_bot) if total_bot > 0 else 0.0

                # ── 2. [중단 30~50%] 채움 비율 (새로 추가됨!) ──
                mid_roi = mask[roi_mid_top_y:roi_bot_top_y, 0:w]
                white_mid = cv2.countNonZero(mid_roi)
                total_mid = (roi_bot_top_y - roi_mid_top_y) * w
                mid_fill = (white_mid / total_mid) if total_mid > 0 else 0.0

                # ── 3. [상단 50~100%] 채움 비율 ──
                top_roi = mask[0:roi_mid_top_y, 0:w]
                white_top = cv2.countNonZero(top_roi)
                total_top = roi_mid_top_y * w
                top_fill = (white_top / total_top) if total_top > 0 else 0.0

                cv2.rectangle(frame, (x, y), (x+box_w, y+box_h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, y+box_h), 5, (0, 0, 255), -1)
                cv2.line(frame, tuple(tl), tuple(tr), (255, 0, 255), 3) 
                
                # 🎯 3단 조건 도착 판정
                if (bot_fill >= ARRIVE_FILL_RATIO) and (mid_fill < MID_EMPTY_LIMIT) and (top_fill < TOP_EMPTY_LIMIT):
                    print(f"\n[!] ARRIVE TRIGGERED!")
                    print(f" -> Bot:{bot_fill*100:.1f}% | Mid:{mid_fill*100:.1f}% | Top:{top_fill*100:.1f}%")
                    mission_phase = 1
                    arrive_time = time.time()

        # 화면에 3단 가이드라인 그리기
        cv2.rectangle(frame, (0, roi_bot_top_y), (w, h), (0, 255, 0), 2)         # 하단 박스 (초록색)
        cv2.rectangle(frame, (0, roi_mid_top_y), (w, roi_bot_top_y), (0, 165, 255), 2) # 중단 박스 (주황색)
        cv2.rectangle(frame, (0, 0), (w, roi_mid_top_y), (0, 0, 255), 2)         # 상단 박스 (빨간색)

        # 상태 텍스트 출력
        status_color = (0, 255, 0) if found else (150, 150, 150)
        cv2.putText(frame, f"FIND: {target_name}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        cv2.putText(frame, f"Bot Fill: {bot_fill*100:.1f}% (Need > {ARRIVE_FILL_RATIO*100}%)", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        cv2.putText(frame, f"Mid Fill: {mid_fill*100:.1f}% (Need < {MID_EMPTY_LIMIT*100}%)", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,165,255), 2)
        cv2.putText(frame, f"Top Fill: {top_fill*100:.1f}% (Need < {TOP_EMPTY_LIMIT*100}%)", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

        cv2.imshow("Robot Vision Test", frame)
        cv2.imshow("HSV Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

import cv2
import numpy as np
import time
import math

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [1] 테스트 파라미터 설정
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CAMERA_INDEX = 0

# 널널한 도착 판정 조건 (현재 환경에 맞춰 튜닝하세요)
ARRIVE_FILL_RATIO = 0.40     # 하단 30% 영역이 목표 색상으로 40% 이상 채워져야 함
TOP_EMPTY_LIMIT = 0.20       # 상단 70% 영역은 목표 색상이 20% 미만이어야 함 (노이즈 허용)
MIN_CONTOUR_AREA = 500       # 무시할 노이즈 크기

MISSION_COLORS = ['RED', 'YELLOW', 'BLUE']
COLOR_HSV_RANGES = {
    'RED':    [(0, 100, 100), (10, 255, 255), (160, 100, 100), (180, 255, 255)],
    'YELLOW': [(20, 100, 100), (35, 255, 255)],
    'BLUE':   [(100, 100, 50), (130, 255, 255)]
}

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [2] 메인 루프 (단일 스레드)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def main():
    print("=== Vision Only Test Started ===")
    print("카메라 화면을 클릭하고 'q'를 누르면 종료됩니다.")
    
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    
    kernel = np.ones((5,5), np.uint8)
    
    current_color_idx = 0
    mission_phase = 0  # 0: 탐색 중, 1: 도착 후 대기 중
    arrive_time = 0.0

    while True:
        ret, raw_frame = cap.read()
        if not ret:
            print("카메라 프레임을 읽을 수 없습니다.")
            break

        # [핵심] 카메라 물리적 우측 90도 회전을 소프트웨어로 좌측 90도 원상복구
        frame = cv2.rotate(raw_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        h, w, _ = frame.shape 

        # 현재 미션 색상
        target_name = MISSION_COLORS[current_color_idx % len(MISSION_COLORS)]
        
        # 미션 달성 (정지 대기 중) 처리
        if mission_phase == 1:
            # 화면 전체를 초록색 톤으로 덮어 도착 상태임을 시각적으로 강하게 표시
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, h), (0, 255, 0), -1)
            frame = cv2.addWeighted(overlay, 0.3, frame, 0.7, 0)
            cv2.putText(frame, f"ARRIVED AT {target_name}!", (20, h//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            
            # 2초 후 다음 색상으로 전환
            if time.time() - arrive_time > 2.0:
                current_color_idx += 1          
                mission_phase = 0               
                print(f"\n[MISSION] Next Target: {MISSION_COLORS[current_color_idx % len(MISSION_COLORS)]}")
            
            cv2.imshow("Robot Vision Test", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            continue

        # 영상 처리 및 HSV 마스킹
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
        bot_fill, top_fill, align_angle, err_x = 0.0, 0.0, 0.0, 0.0
        
        # 화면 분할 기준선 (하단 30% 영역 설정)
        roi_split_y = int(h * 0.7)

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                found = True
                # 1. Bounding Box (탐색/돌격 방향 표시용)
                x, y, box_w, box_h = cv2.boundingRect(c)
                cx = x + box_w // 2
                err_x = (cx - (w / 2)) / (w / 2)
                
                # 2. 사다리꼴 기울기 각도 (수평 정렬용) - np.int32 적용
                rect = cv2.minAreaRect(c)
                box = np.int32(cv2.boxPoints(rect))
                pts_y_sorted = sorted(box, key=lambda p: p[1])
                top_2 = pts_y_sorted[:2]
                tl, tr = sorted(top_2, key=lambda p: p[0])
                dx = tr[0] - tl[0]
                dy = tr[1] - tl[1]
                align_angle = math.degrees(math.atan2(dy, dx)) if dx != 0 else 0.0

                # 3. 상단/하단 영역 채움 비율 계산
                bottom_roi = mask[roi_split_y:h, 0:w]
                white_bot = cv2.countNonZero(bottom_roi)
                total_bot = (h - roi_split_y) * w
                bot_fill = (white_bot / total_bot) if total_bot > 0 else 0.0

                top_roi = mask[0:roi_split_y, 0:w]
                white_top = cv2.countNonZero(top_roi)
                total_top = roi_split_y * w
                top_fill = (white_top / total_top) if total_top > 0 else 0.0

                # 화면에 시각적 가이드 그리기
                cv2.rectangle(frame, (x, y), (x+box_w, y+box_h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, y+box_h), 5, (0, 0, 255), -1)
                cv2.line(frame, tuple(tl), tuple(tr), (255, 0, 255), 3) # 수평 정렬선 (보라색)
                
                # 🎯 도착 판정 로직
                if bot_fill >= ARRIVE_FILL_RATIO and top_fill < TOP_EMPTY_LIMIT:
                    print(f"\n[!] ARRIVE TRIGGERED!")
                    print(f" -> Target: {target_name} | Bot Fill: {bot_fill*100:.1f}% | Top Fill: {top_fill*100:.1f}%")
                    mission_phase = 1
                    arrive_time = time.time()

        # 화면 분할 가이드라인 (빨간 점선 박스)
        cv2.rectangle(frame, (0, roi_split_y), (w, h), (0, 0, 255), 2)

        # ── 화면 텍스트 UI 상태창 ──
        status_color = (0, 255, 0) if found else (150, 150, 150)
        cv2.putText(frame, f"FIND: {target_name}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(frame, f"Bot Fill: {bot_fill*100:.1f}% (Need > {ARRIVE_FILL_RATIO*100}%)", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)
        cv2.putText(frame, f"Top Fill: {top_fill*100:.1f}% (Need < {TOP_EMPTY_LIMIT*100}%)", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)
        cv2.putText(frame, f"Align Ang: {align_angle:+.1f} deg", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 2)
        cv2.putText(frame, f"Steer Err: {err_x:+.2f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

        cv2.imshow("Robot Vision Test", frame)
        cv2.imshow("HSV Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

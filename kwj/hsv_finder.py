import cv2
import numpy as np

# --- 전역 변수 ---
current_frame = None
hsv_frame = None
display_mask = None

# 누적을 위한 변수 (초기값은 반대로 설정)
overall_min = np.array([180, 255, 255])
overall_max = np.array([0, 0, 0])
is_updated = False

# 💡 [설정값] 클릭한 색상과 '얼마나 비슷한 색'까지 한 덩어리로 볼 것인가?
# 그림자나 주름 때문에 인식이 잘 안 되면 S_TOL, V_TOL 값을 살짝 더 올려주세요.
H_TOL = 15  # 색상 오차
S_TOL = 60  # 채도 오차
V_TOL = 60  # 명도 오차

def on_mouse_click(event, x, y, flags, param):
    global current_frame, hsv_frame, display_mask
    global overall_min, overall_max, is_updated

    if event == cv2.EVENT_LBUTTONDOWN:
        if hsv_frame is None:
            return

        # 1. 클릭한 픽셀의 HSV 값 가져오기
        clicked_hsv = hsv_frame[y, x]
        h, s, v = int(clicked_hsv[0]), int(clicked_hsv[1]), int(clicked_hsv[2])

        # 2. 클릭한 색상 기준 허용 오차 범위 생성
        lower_bound = np.array([max(0, h - H_TOL), max(0, s - S_TOL), max(0, v - V_TOL)])
        upper_bound = np.array([min(179, h + H_TOL), min(255, s + S_TOL), min(255, v + V_TOL)])
        
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

        # (중요) 빨간색처럼 H값이 0 부근일 때 179로 넘어가는 'Wrap-around' 현상 방지 처리
        if h - H_TOL < 0:
            lower2 = np.array([180 + (h - H_TOL), max(0, s - S_TOL), max(0, v - V_TOL)])
            upper2 = np.array([179, min(255, s + S_TOL), min(255, v + V_TOL)])
            mask2 = cv2.inRange(hsv_frame, lower2, upper2)
            mask = cv2.bitwise_or(mask, mask2)
        elif h + H_TOL > 179:
            lower2 = np.array([0, max(0, s - S_TOL), max(0, v - V_TOL)])
            upper2 = np.array([(h + H_TOL) - 180, min(255, s + S_TOL), min(255, v + V_TOL)])
            mask2 = cv2.inRange(hsv_frame, lower2, upper2)
            mask = cv2.bitwise_or(mask, mask2)

        # 3. 마스크에서 독립된 덩어리(윤곽선)들 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        selected_contour = None
        for cnt in contours:
            # 여러 색상 덩어리 중, '내가 마우스로 클릭한 (x,y) 좌표'가 포함된 덩어리만 선택
            if cv2.pointPolygonTest(cnt, (x, y), False) >= 0:
                selected_contour = cnt
                break

        if selected_contour is not None:
            # 선택한 덩어리 모양대로 칠할 텅 빈 캔버스 생성
            roi_mask = np.zeros_like(mask)
            cv2.drawContours(roi_mask, [selected_contour], -1, 255, thickness=cv2.FILLED)

            # 선택된 덩어리 내부에 있는 픽셀들의 실제 HSV 값들만 모조리 뽑아오기
            selected_pixels = hsv_frame[roi_mask == 255]

            if len(selected_pixels) > 0:
                current_min = np.min(selected_pixels, axis=0)
                current_max = np.max(selected_pixels, axis=0)

                # 기존에 기록된 최소/최대값과 갱신 (점점 범위가 확장됨)
                overall_min = np.minimum(overall_min, current_min)
                overall_max = np.maximum(overall_max, current_max)
                is_updated = True

                print(f"[*] 영역 추출 완료 (클릭한 픽셀 H:{h} S:{s} V:{v})")
                print(f"    -> 방금 추출된 범위: Min {list(current_min)} ~ Max {list(current_max)}")
                
                # 피드백을 위해 추출된 덩어리만 흑백 화면으로 따로 보여줌
                display_mask = roi_mask
        else:
            print("[-] 유효한 덩어리를 찾지 못했습니다. 색상의 중앙 부분을 다시 클릭해보세요.")

def main():
    global current_frame, hsv_frame, display_mask
    global overall_min, overall_max, is_updated

    cap = cv2.VideoCapture(0)
    cv2.namedWindow("Camera")
    cv2.setMouseCallback("Camera", on_mouse_click)

    print("="*50)
    print(" 🎯 타겟 전용 연속 영역 HSV 추출기")
    print("="*50)
    print(" - 마우스 좌클릭 : 천/물체를 클릭하면 해당 덩어리만 인식하여 기록")
    print(" - 'p' 키 : [최종 HSV 값] 터미널에 출력")
    print(" - 'r' 키 : 누적된 값 초기화")
    print(" - 'q' 키 : 종료")
    print("="*50)

    while True:
        ret, frame = cap.read()
        if not ret: 
            break

        # 약간의 블러를 주면 그림자나 노이즈로 인해 덩어리가 끊어지는 것을 막아줍니다.
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        current_frame = frame.copy()
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # 단축키 안내 텍스트 (그리기)
        cv2.putText(frame, "Click target color to Extract", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "P: Print | R: Reset", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow("Camera", frame)

        # 클릭 시 추출된 마스크를 확인하는 창
        if display_mask is not None:
            cv2.imshow("Extracted Area (Mask)", display_mask)

        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
            
        elif key == ord('p'):
            if is_updated:
                print("\n" + "▼"*20)
                print(" [로봇 코드에 복사/붙여넣기 할 최종 HSV 값]")
                print(f" lower_bound = np.array({list(overall_min)})")
                print(f" upper_bound = np.array({list(overall_max)})")
                print("▲"*20 + "\n")
            else:
                print("\n[알림] 먼저 화면에서 색상을 클릭해주세요.\n")
                
        elif key == ord('r'):
            overall_min = np.array([180, 255, 255])
            overall_max = np.array([0, 0, 0])
            display_mask = None
            is_updated = False
            print("\n[알림] 기록된 HSV 값이 리셋되었습니다.\n")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

import cv2
import numpy as np

# --- 전역 변수 설정 ---
current_frame = None
# 전체 누적 범위를 추적하기 위한 변수 (초기값은 반대로 설정)
overall_min = np.array([180, 255, 255])
overall_max = np.array([0, 0, 0])
is_updated = False

def on_mouse_click(event, x, y, flags, param):
    global current_frame, overall_min, overall_max, is_updated

    # 마우스 왼쪽 버튼 클릭 시
    if event == cv2.EVENT_LBUTTONDOWN:
        if current_frame is None:
            return

        # 원본 프레임 크기 가져오기
        h, w = current_frame.shape[:2]
        # floodFill(홍수 채우기)를 위한 마스크는 원본보다 2픽셀씩 커야 함
        mask = np.zeros((h + 2, w + 2), np.uint8)

        # 노이즈로 인한 끊김을 방지하기 위해 약간의 블러 처리
        blurred = cv2.GaussianBlur(current_frame, (5, 5), 0)

        # 💡 [핵심] 클릭한 픽셀과 색상 차이가 얼마나 나는 곳까지 같은 영역으로 볼 것인지 설정 (오차 범위)
        # 이 숫자를 키우면 더 넓은/비슷한 범위까지 추출되고, 줄이면 아주 똑같은 색만 추출됩니다.
        lo_diff = (15, 15, 15)
        up_diff = (15, 15, 15)
        
        # 클릭한 (x,y) 좌표를 시작점으로 인접한 픽셀들을 탐색하여 mask 생성
        flags_ff = 4 | (255 << 8) | cv2.FLOODFILL_MASK_ONLY
        cv2.floodFill(blurred, mask, (x, y), (255, 255, 255), lo_diff, up_diff, flags_ff)

        # 실제 이미지 크기에 맞게 마스크 테두리 1픽셀씩 잘라내기
        roi_mask = mask[1:-1, 1:-1]

        # 이미지를 HSV로 변환
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # 마스크에서 흰색(255)으로 칠해진 영역(연속된 색상)의 HSV 픽셀들만 추출
        selected_pixels = hsv_frame[roi_mask == 255]

        if len(selected_pixels) > 0:
            # 방금 클릭해서 찾아낸 영역의 HSV 최소/최대값
            current_min = np.min(selected_pixels, axis=0)
            current_max = np.max(selected_pixels, axis=0)

            # 기존에 기록된 전체 최소/최대값과 비교하여 범위를 넓힘 (누적)
            overall_min = np.minimum(overall_min, current_min)
            overall_max = np.maximum(overall_max, current_max)
            is_updated = True

            print(f"[*] 클릭 영역 추출됨  -> Min {list(current_min)} ~ Max {list(current_max)}")
            
            # 피드백: 방금 어느 영역이 추출되었는지 흑백 화면으로 잠깐 보여줌
            cv2.imshow("Extracted Area (Mask)", roi_mask)

def main():
    global current_frame, overall_min, overall_max, is_updated

    cap = cv2.VideoCapture(0)
    
    cv2.namedWindow("Camera")
    # 마우스 콜백 함수 등록
    cv2.setMouseCallback("Camera", on_mouse_click)

    print("="*40)
    print(" 🎯 순수 연속 영역 HSV 추출기 실행")
    print("="*40)
    print(" [조작법]")
    print(" - 마우스 좌클릭 : 해당 지점과 이어지는 색상 영역의 HSV 누적 기록")
    print(" - 'p' 키 : 지금까지 기록된 [최종 HSV 값] 출력")
    print(" - 'r' 키 : 기록된 값 초기화 (리셋)")
    print(" - 'q' 키 : 프로그램 종료")
    print("="*40)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("카메라를 읽을 수 없습니다.")
            break

        current_frame = frame.copy()

        # 화면에 간단한 텍스트 띄우기
        cv2.putText(frame, "Click color to Extract!", (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "P: Print | R: Reset | Q: Quit", (15, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Camera", frame)

        key = cv2.waitKey(1) & 0xFF
        
        # 'q' 키: 종료
        if key == ord('q'):
            print("\n프로그램을 종료합니다.")
            break
            
        # 'p' 키: 최종 HSV 값 출력
        elif key == ord('p'):
            if is_updated:
                print("\n" + "="*40)
                print(" 🟢 [최종 산출된 HSV 범위] 🟢")
                print(f" LOWER BOUND (최소값): {list(overall_min)}")
                print(f" UPPER BOUND (최대값): {list(overall_max)}")
                print("="*40 + "\n")
            else:
                print("\n[알림] 먼저 화면을 클릭해서 색상을 추출해주세요!\n")
                
        # 'r' 키: 리셋
        elif key == ord('r'):
            overall_min = np.array([180, 255, 255])
            overall_max = np.array([0, 0, 0])
            is_updated = False
            print("\n[알림] 기록된 HSV 값이 모두 초기화 되었습니다.\n")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


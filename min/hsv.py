import cv2
import numpy as np

def empty(a):
    pass

def main():
    # 1. 카메라 연결 (camera_tracker1.py와 동일하게 세팅)
    CAMERA_INDEX = 0
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    # 해상도 설정 (테스트용이므로 약간 작게 해도 무방합니다)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 2. 튜닝을 위한 윈도우 및 트랙바(슬라이더) 생성
    cv2.namedWindow("HSV Tuner")
    cv2.resizeWindow("HSV Tuner", 400, 300)
    
    # H(색상): 0~179 / S(채도): 0~255 / V(명도): 0~255
    cv2.createTrackbar("H Min", "HSV Tuner", 0, 179, empty)
    cv2.createTrackbar("H Max", "HSV Tuner", 179, 179, empty)
    cv2.createTrackbar("S Min", "HSV Tuner", 0, 255, empty)
    cv2.createTrackbar("S Max", "HSV Tuner", 255, 255, empty)
    cv2.createTrackbar("V Min", "HSV Tuner", 0, 255, empty)
    cv2.createTrackbar("V Max", "HSV Tuner", 255, 255, empty)

    # 초기값 설정 (일단 모두 열어둠)
    cv2.setTrackbarPos("H Min", "HSV Tuner", 0)
    cv2.setTrackbarPos("S Min", "HSV Tuner", 50)
    cv2.setTrackbarPos("V Min", "HSV Tuner", 50)
    cv2.setTrackbarPos("H Max", "HSV Tuner", 179)
    cv2.setTrackbarPos("S Max", "HSV Tuner", 255)
    cv2.setTrackbarPos("V Max", "HSV Tuner", 255)

    print("=========================================")
    print(" 툴 사용법:")
    print(" 1. 카메라에 목표 색지(RED, YELLOW, BLUE)를 비춥니다.")
    print(" 2. 트랙바를 움직여 'Mask' 창에 목표물만 하얗게 나오도록 조절합니다.")
    print(" 3. 조절을 마친 후 'q' 키를 누르면 콘솔에 결과값이 출력됩니다.")
    print("=========================================\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("카메라를 읽을 수 없습니다.")
            break

        # (선택) 원래 코드에서 90도 회전했다면 여기서도 동일하게 회전시켜 확인
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # BGR 이미지를 HSV로 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 트랙바의 현재 값 읽어오기
        h_min = cv2.getTrackbarPos("H Min", "HSV Tuner")
        s_min = cv2.getTrackbarPos("S Min", "HSV Tuner")
        v_min = cv2.getTrackbarPos("V Min", "HSV Tuner")
        h_max = cv2.getTrackbarPos("H Max", "HSV Tuner")
        s_max = cv2.getTrackbarPos("S Max", "HSV Tuner")
        v_max = cv2.getTrackbarPos("V Max", "HSV Tuner")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        # 마스크 생성 (해당 색상 영역은 흰색(255), 나머지는 검은색(0))
        mask = cv2.inRange(hsv, lower, upper)
        
        # 원본 이미지에 마스크 씌우기 (확인용)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # 화면 출력
        cv2.imshow("Original", frame)
        cv2.imshow("Mask (White = Detected)", mask)
        cv2.imshow("Result", result)

        # 'q' 키를 누르면 종료하고 값 출력
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("=========================================")
            print(" 복사해서 camera_tracker1.py 에 붙여넣을 값:")
            print(f" np.array([{h_min}, {s_min}, {v_min}]), np.array([{h_max}, {s_max}, {v_max}])")
            print("=========================================")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

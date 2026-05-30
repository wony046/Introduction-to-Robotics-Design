import cv2 # OpenCV 라이브러리 가져오기 [cite: 1472]

# 0번 카메라 연결 (라즈베리파이 기본 카메라 또는 노트북 내장 카메라) [cite: 1475, 1476]
# 만약 USB 웹캠을 따로 꽂았다면 0 대신 1이나 2로 변경해 보세요. 
cap = cv2.VideoCapture(0) [cite: 1475]

# 카메라가 정상적으로 열렸는지 확인 [cite: 1481]
if not cap.isOpened(): [cite: 1481]
    print("카메라를 열 수 없습니다.") [cite: 1483]
    exit() [cite: 1485]

# 무한 루프를 돌며 카메라에서 계속해서 프레임을 읽어옵니다. [cite: 1487, 1509]
while True: [cite: 1487]
    ret, frame = cap.read() # 카메라에서 프레임 읽기 [cite: 1488]
    
    # ret은 프레임을 정상적으로 읽었는지 여부(True/False)를 반환합니다. [cite: 1511, 1512]
    if not ret: [cite: 1488]
        print("프레임을 읽을 수 없습니다.") [cite: 1489]
        break [cite: 1489]

    # 읽어온 영상 프레임(이미지 데이터)을 'Camera'라는 이름의 창에 표시합니다. [cite: 1490, 1513]
    cv2.imshow("Camera", frame) [cite: 1490]

    # 'q' 키를 누르면 무한 루프를 빠져나와 종료합니다. 
    # waitKey(1)은 1ms 동안 키 입력을 대기하며 영상을 갱신하는 역할도 합니다. [cite: 1514, 1515]
    if cv2.waitKey(1) & 0xFF == ord('q'): [cite: 1502]
        break [cite: 1502]

# 사용이 끝난 카메라와의 연결을 해제합니다. [cite: 1504, 1516, 1517]
cap.release() [cite: 1504]

# OpenCV에서 생성한 모든 창을 닫습니다. [cite: 1505, 1518, 1519]
cv2.destroyAllWindows() [cite: 1505]

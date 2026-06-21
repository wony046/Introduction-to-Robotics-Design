import cv2
import math
import time
import serial
import threading
import numpy as np

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 튜닝 포인트 (이 값들을 변경하며 최적의 정차를 찾으세요)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CLOSE_ENTER_MM      = 450.0   # [튜닝] 이 거리 이내로 접근하면 정밀 제어(CLOSE) 시작
CLOSE_OBSERVE_SEC   = 1.0     # [튜닝] CLOSE 진입 시 목표를 확실히 찍기 위해 정지하는 시간(초)
CLOSE_ARRIVE_MM     = 10.0    # [튜닝] 목표 좌표 반경 몇 mm 안에 들어오면 최종 도착(DONE)으로 판정할지
CLOSE_SPEED_MAX     = 0.2     # CLOSE 모드 초기 전진 속도
CLOSE_BEARING_SCALE = 0.8212  # 근접 원근 보정 배율 (필요시 1.0으로 변경)
KP_CLOSE_HDG        = 0.1     # 오도메트리 추종 시 회전 민감도

# ★ 새로 추가된 제동(미끄러짐) 방지 튜닝 변수 ★
STOP_EARLY_MM       = 50.0    # [튜닝] 관성 밀림 보상: 원래 계산된 거리보다 이(mm)만큼 일찍 목표를 찍음
CLOSE_SPEED_MIN     = 0.08    # [튜닝] 목표물 15cm 이내 접근 시 감속할 최저 속도 (부드러운 정차)

# ── 하드웨어 & 카메라 설정 ──────────────────────────────────────────
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_ARDUINO = 115200

CAMERA_INDEX  = 0
FRAME_W, FRAME_H = 640, 400
HFOV_DEG      = 38.6
CAM_HEIGHT_MM = 590.0    # 새로 측정한 59cm
CAM_TILT_DEG  = 40.4     # 캘리브레이션으로 찾은 각도
FRAME_ROTATE  = cv2.ROTATE_90_COUNTERCLOCKWISE
_EFF_W, _EFF_H = 480, 640

# RED 색상 범위 및 CLAHE
COLOR_RANGES_RED = [((28, 152, 110), (255, 212, 170))]
_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

# ── 전역 상태 변수 ──────────────────────────────────────────────
arduino_x_mm        = 0.0
arduino_y_mm        = 0.0
arduino_heading_deg = 0.0
shutdown_flag       = False

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 아두이노 오도메트리 수신 스레드
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def _arduino_reader(arduino):
    global arduino_x_mm, arduino_y_mm, arduino_heading_deg
    while not shutdown_flag:
        try:
            if arduino.in_waiting > 0:
                line = arduino.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('O:'):
                    parts = line[2:].split(',')
                    if len(parts) == 3:
                        arduino_x_mm        = float(parts[0])
                        arduino_y_mm        = float(parts[1])
                        arduino_heading_deg = float(parts[2])
        except Exception:
            pass
        time.sleep(0.01)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def main():
    global shutdown_flag
    
    # 아두이노 연결
    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
        arduino.write(b"R\n") # 오도메트리 리셋
        time.sleep(0.5)
        print("[INIT] 아두이노 연결 및 오도메트리 리셋 완료")
    except Exception as e:
        print(f"[ERROR] 아두이노 연결 실패: {e}")
        return

    t_arduino = threading.Thread(target=_arduino_reader, args=(arduino,), daemon=True)
    t_arduino.start()

    # 카메라 연결
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        shutdown_flag = True
        return

    # 상태 관리 변수
    state = "SEEK"  # SEEK -> OBSERVE -> BLIND_APPROACH -> DONE
    target_x = None
    target_y = None
    observe_start = 0.0
    initial_dist = None

    print("\n=== 단독 접근(CLOSE) 테스트 시작 (제동 밀림 방지 적용) ===")
    print("종료하려면 영상 창을 클릭하고 'q'를 누르세요.\n")

    while not shutdown_flag:
        ret, frame = cap.read()
        if not ret: continue
        
        display = cv2.rotate(frame, FRAME_ROTATE)
        lab = cv2.cvtColor(display, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = _clahe.apply(l)
        lab_merge = cv2.merge([l, a, b])
        
        mask = np.zeros(lab_merge.shape[:2], dtype=np.uint8)
        for (lo, hi) in COLOR_RANGES_RED:
            mask |= cv2.inRange(lab_merge, np.array(lo), np.array(hi))
            
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        cx, cy, area = None, None, 0
        dist_mm = 5000.0
        bearing = 0.0
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 500:
                M = cv2.moments(largest)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # 수직 거리 계산
                    f_px = (_EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))
                    delta_v = math.degrees(math.atan2(cy - _EFF_H / 2.0, f_px))
                    depression = CAM_TILT_DEG + delta_v
                    
                    if depression > 1.0:
                        raw_dist = CAM_HEIGHT_MM / math.tan(math.radians(depression))
                        # ★ 관성 보상: 실제 계산된 거리보다 STOP_EARLY_MM 만큼 강제로 앞당겨서 인식시킴
                        dist_mm = max(raw_dist - STOP_EARLY_MM, 50.0) 
                    
                    # 방위각 (단순 atan2)
                    bearing = -math.degrees(math.atan2(cx - _EFF_W / 2.0, f_px))
                    
                    cv2.circle(display, (cx, cy), 10, (0, 255, 0), -1)

        v, w = 0.0, 0.0

        # ── 상태 머신 제어 ──────────────────────────────────────────────
        if state == "SEEK":
            if cx is not None:
                if dist_mm < CLOSE_ENTER_MM:
                    print(f"\n[SEEK -> OBSERVE] 목표물 접근! (보정된 거리: {dist_mm:.0f}mm < {CLOSE_ENTER_MM}mm)")
                    state = "OBSERVE"
                    observe_start = time.time()
                else:
                    # 목표물 멀리 있음: 직진하며 방향 맞춤
                    v = 0.3
                    w = max(min((1.8 / 45.0) * bearing, 1.8), -1.8)
                    print(f"\r[SEEK] 직진 중... 보정 거리: {dist_mm:4.0f}mm | 조향각: {bearing:+5.1f}°", end="")
            else:
                # 못 찾음: 제자리 정지 (테스트용)
                print(f"\r[SEEK] 빨간색을 찾는 중... (area: {area})               ", end="")

        elif state == "OBSERVE":
            v, w = 0.0, 0.0
            elapsed = time.time() - observe_start
            
            if cx is not None:
                # 관측 중 데이터 최신화 (마지막으로 본 정보 저장)
                lateral = (cx - _EFF_W / 2.0) / f_px
                forward = (_EFF_H - cy) / _EFF_H + 0.05
                close_bearing = -math.degrees(math.atan2(lateral, forward)) * CLOSE_BEARING_SCALE
            
            print(f"\r[OBSERVE] 목표 안정화 대기 중... {elapsed:.1f}s / {CLOSE_OBSERVE_SEC}s", end="")
            
            if elapsed >= CLOSE_OBSERVE_SEC:
                # 목표 절대 좌표 계산 (이미 STOP_EARLY_MM이 반영된 dist_mm을 사용)
                bearing_global_deg = arduino_heading_deg + close_bearing
                hdg_rad = math.radians(bearing_global_deg)
                target_x = arduino_x_mm + dist_mm * math.sin(hdg_rad)
                target_y = arduino_y_mm + dist_mm * math.cos(hdg_rad)
                
                print(f"\n[OBSERVE -> BLIND] 절대 목표 좌표 확정 (밀림 보상 완료): ({target_x:.0f}, {target_y:.0f})")
                state = "BLIND_APPROACH"

        elif state == "BLIND_APPROACH":
            ex = target_x - arduino_x_mm
            ey = target_y - arduino_y_mm
            dist_err = math.sqrt(ex**2 + ey**2)
            
            if initial_dist is None:
                initial_dist = max(dist_err, 1.0)
                
            if dist_err < CLOSE_ARRIVE_MM:
                print(f"\n[BLIND -> DONE] 목표 도착! (남은 오차: {dist_err:.0f}mm < {CLOSE_ARRIVE_MM}mm)")
                state = "DONE"
            else:
                target_hdg = math.degrees(math.atan2(ex, ey))
                # -180 ~ 180 정규화
                hdg_err = ((target_hdg - arduino_heading_deg + 180) % 360) - 180 
                
                # ★ 감속 브레이크: 목표까지 150mm 남은 시점부터 CLOSE_SPEED_MIN까지 부드럽게 감속
                v_scale = min(1.0, dist_err / 150.0) 
                v = CLOSE_SPEED_MIN + (CLOSE_SPEED_MAX - CLOSE_SPEED_MIN) * v_scale
                
                w = max(min(KP_CLOSE_HDG * hdg_err, 1.8), -1.8)
                
                progress = (1.0 - dist_err / initial_dist) * 100
                print(f"\r[BLIND] 브레이크 제어 중... 진행률: {progress:5.1f}% | 남은 거리: {dist_err:3.0f}mm | 현재 v: {v:.2f}", end="")

        elif state == "DONE":
            v, w = 0.0, 0.0
            print(f"\r[DONE] 완벽히 정차했습니다. 오도메트리: ({arduino_x_mm:.0f}, {arduino_y_mm:.0f})", end="")

        # ── 모터 명령 전송 및 화면 출력 ─────────────────────────────────
        cmd = f"{v:.2f} {w:.2f}\n"
        arduino.write(cmd.encode())
        
        cv2.putText(display, f"State: {state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(display, f"Dist: {dist_mm:.0f} mm", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow('Approach Test', display)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 종료 처리
    shutdown_flag = True
    arduino.write(b"0.00 0.00\n")
    arduino.close()
    cap.release()
    cv2.destroyAllWindows()
    print("\n테스트 종료")

if __name__ == "__main__":
    main()

import serial
import time
import math
import json
import threading
import cv2          # [추가] OpenCV
import numpy as np  # [추가] 행렬 연산

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [추가] 카메라 & 비전 파라미터
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CAMERA_INDEX = 0
FRAME_WIDTH = 320   # 연산 속도를 위해 해상도 축소
FRAME_HEIGHT = 240
SHOW_CV_WINDOW = True # 테스트 시 화면 출력 (실전에서는 False 권장)

MIN_CONTOUR_AREA = 500  # 노이즈를 무시할 최소 픽셀 덩어리 크기
ARRIVE_Y_RATIO = 0.85   # 화면 세로의 85% 지점 아래로 색지가 내려오면 도착으로 간주

# 타겟 색상 (빨 -> 노 -> 파)
MISSION_COLORS = ['RED', 'YELLOW', 'BLUE']
# 조명에 맞게 반드시 튜닝해야 하는 HSV 임계값
COLOR_HSV_RANGES = {
    'RED':    [(0, 100, 100), (10, 255, 255), (160, 100, 100), (180, 255, 255)], # 빨강은 2개 영역
    'YELLOW': [(20, 100, 100), (35, 255, 255)],
    'BLUE':   [(100, 100, 50), (130, 255, 255)]
}

SCORE_COLOR_TARGET = 5000.0  # 색지 발견 시 목표 방향으로 끌어당기는 압도적 점수
SCORE_EXPLORE_BIAS = 600.0   # 색지가 없을 때 완만하게 왼쪽으로 회전하며 탐색(벽타기)하게 만드는 점수

# ── 전역 비전 상태 (스레드 공유) ─────────────────────────
_cam_lock = threading.Lock()
is_color_visible = False
camera_target_error_x = 0.0  # 화면 중심 기준 타겟의 x좌표 오차 (-1.0 ~ 1.0)
color_bottom_y = 0           # 타겟 박스의 가장 아래 y 좌표
current_color_idx = 0        # 0:RED, 1:YELLOW, 2:BLUE
mission_phase = 0            # 0: 탐색/접근, 1: 색지 위 도착(대기 중)
arrive_time = 0.0

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 포트 & 라이다 설정 (기존 동일)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"  # 환경에 맞게 수정 필요 시 수정
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

LIDAR_OFFSET    = 10     
LIDAR_MIN_VALID = 100   
DETECTION_RANGE = 1500  

ROBOT_HALF_WIDTH = 110   
FORWARD_SPEED    = 0.45
MIN_SPEED        = 0.12
MAX_W            = 1.8
W_MIN_DANGER     = 0.5   
W_SMOOTH         = 0.7

# (기존 LAYERS, STOP zone 파라미터 생략 없이 그대로 유지)
LAYERS = [
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':140, 'w_gain':2.8, 'weight_base':0.8, 'weight_cap':7.5, 'weight_dynamic':True, 'v_max':0.22, 'affects_v':True},
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':120, 'w_gain':2.5, 'weight_base':0.6, 'weight_cap':5.0, 'weight_dynamic':True, 'v_max':0.38, 'affects_v':True},
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':120, 'w_gain':2.0, 'weight_base':0.4, 'weight_cap':4.5, 'weight_dynamic':True, 'affects_v':True},
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':110, 'w_gain':1.0, 'weight_base':0.1, 'weight_start':0.2, 'weight_dynamic':False, 'affects_v':True},
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':110, 'w_gain':0.4, 'weight_base':0.05,'weight_start':0.1, 'weight_dynamic':False, 'affects_v':False},
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':110, 'w_gain':0.3, 'weight_base':0.02,'weight_start':0.05,'weight_dynamic':False, 'affects_v':False},
]

LAYER_PERCENTILE = 5    
STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 175
STOP_HORIZ_TH = 105
STOP_ESCAPE_MIN_GAP   = ROBOT_HALF_WIDTH * 2 + 40   
STOP_MAX_CYCLES       = 30                          
STOP_PIVOT_MAX_W      = 0.9   
STOP_PIVOT_MIN_W      = 0.7   
STOP_PIVOT_SLOW_DEG   = 15    
FGM_MIN_ANG_DEG      = 3      
FGM_MIN_DEPTH_MM     = 250   
FGM_MAX_RANGE_MM     = 500   
FGM_RATIO_THRES      = 1.2   
FRONT_GAP_MIN_DEPTH  = 300   
SCORE_GAP_FRONT      = 900.0 

SCORE_ALPHA       = 5.0    
SCORE_BETA        = 8      
SCORE_SIDE        = 2500.0  
HEADING_WEIGHT_MM = 5.0    
DEPTH_JUMP_THRES  = 120    
DIRECTION_HYSTERESIS = 300.0

SCAN_WIDE_HALF = 135   
SEND_INTERVAL  = 0.1
SIDE_SAFE_MARGIN  = 300   
SIDE_FWD_LEAD     = 90    
SIDE_FWD_REAR     = 90    
SIDE_REPULSE_GAIN = 1.25   
SIDE_EXP_K        = 2.0   
SIDE_LAYER_ANG_START = 15   
SIDE_LAYER_ANG_END   = 75   
SIDE_LAYER_DIST_MAX  = 600  
SIDE_W_BOOST_GAIN    = 1.5  
MIN_PASSAGE_WIDTH       = STOP_ESCAPE_MIN_GAP  
VIRTUAL_OBS_GAIN        = 1.5   
VIRTUAL_CENTER_DEADBAND = 10    
VIRTUAL_EXP_K           = 2.5   

DEBUG_LAYERS  = False
DEBUG_STOP    = True
DEBUG_DIR     = False
DEBUG_FINAL   = True
DEBUG_SIDE    = False
DEBUG_VIRTUAL = False

arduino_heading_deg   = 0.0
prev_w                = 0.0
_last_direction       = 1.0   
stop_cycle_count           = 0     
stop_pivot_w               = 0.0   
stop_locked_target         = 0.0
stop_locked_gap            = 0.0
stop_locked_global_heading = 0.0
stop_phase                 = 0     

_scan_lock   = threading.Lock()
_latest_scan = []            
_shutdown    = threading.Event() 

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# (기존 유틸리티 및 라이다 연산 함수들 유지 - decompose, parse_packet 등)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def normalize_angle(angle): return angle - 360 if angle > 180 else angle
def is_in_front_90(a): return -90 <= a <= 90
def is_in_wide_scan(a): return -SCAN_WIDE_HALF <= a <= SCAN_WIDE_HALF
def decompose(angle_deg, dist):
    rad = math.radians(angle_deg)
    return abs(dist * math.sin(rad)), dist * math.cos(rad)
def cosine_dist(d1, d2, angle_diff_deg):
    theta = math.radians(abs(angle_diff_deg))
    return math.sqrt(d1**2 + d2**2 - 2 * d1 * d2 * math.cos(theta))
def point_to_segment_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    seg_sq = dx*dx + dy*dy
    if seg_sq == 0: return math.sqrt((px - ax)**2 + (py - ay)**2)
    t = max(0.0, min(1.0, ((px - ax)*dx + (py - ay)*dy) / seg_sq))
    return math.sqrt((px - ax - t*dx)**2 + (py - ay - t*dy)**2)
def nearest_to_segments(px, py, cluster_xy):
    if len(cluster_xy) == 1:
        return math.sqrt((px - cluster_xy[0][0])**2 + (py - cluster_xy[0][1])**2)
    return min(point_to_segment_dist(px, py, cluster_xy[j][0], cluster_xy[j][1], cluster_xy[j+1][0], cluster_xy[j+1][1]) for j in range(len(cluster_xy) - 1))
def parse_packet(data):
    if len(data) != 5: return None
    s_flag, s_inv_flag = data[0] & 0x01, (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None
    return (((data[1] >> 1) | (data[2] << 7)) / 64.0), ((data[3] | (data[4] << 8)) / 4.0)
def read_arduino(arduino):
    global arduino_heading_deg
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'): arduino_heading_deg = float(line[2:])
        except: pass

def detect_stop_zone(scan_points):
    for a, d in scan_points:
        if d < LIDAR_MIN_VALID or d > DETECTION_RANGE: continue
        if not is_in_front_90(a): continue
        horiz, fwd = decompose(a, d)
        if STOP_FWD_MIN <= fwd <= STOP_FWD_MAX and horiz < STOP_HORIZ_TH: return True
    return False

# (find_all_gaps, choose_escape_gap, find_stop_escape_direction, process_layer, get_gap_width, get_side_repulsion, get_side_layer_push, get_front_passable_gaps, get_narrow_gap_pushes 모두 기존 코드와 완전히 동일하게 삽입했다고 가정합니다. 지면상 생략)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [수정] 계층형 v/w 산출 (점수제에 카메라/탐색 편향 추가)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def find_vw_layered(scan_points, heading_deg):
    global is_color_visible, camera_target_error_x
    
    layer_results = []
    for layer in LAYERS:
        # (기존 process_layer 호출 부분 생략 - 동일)
        pass # 실제 코드에서는 기존처럼 process_layer 결과를 layer_results에 담습니다.

    if not layer_results:
        return FORWARD_SPEED, 0.0

    # (기존 gap_L, gap_R, gap_bonus 계산 부분 생략 - 동일)
    gap_L, gap_R = 100, 100 # 예시
    gap_bonus_L, gap_bonus_R = 0, 0
    sum_pL, sum_pR = 0, 0
    virt_push_L_total, virt_push_R_total = 0, 0
    side_left_push, side_right_push = 0, 0
    effective_push_R, effective_push_L = max(sum_pR, virt_push_R_total), max(sum_pL, virt_push_L_total)

    term_gap_L    = SCORE_ALPHA * gap_L
    term_gap_R    = SCORE_ALPHA * gap_R
    term_push_L   = SCORE_BETA  * effective_push_R
    term_push_R   = SCORE_BETA  * effective_push_L
    term_side_L   = SCORE_SIDE  * side_right_push
    term_side_R   = SCORE_SIDE  * side_left_push
    term_head_L   = max(0.0, -heading_deg) * HEADING_WEIGHT_MM
    term_head_R   = max(0.0,  heading_deg) * HEADING_WEIGHT_MM

    # [핵심 추가] 비전 기반 점수 개입
    term_color_L = 0.0
    term_color_R = 0.0
    term_explore_L = 0.0
    term_explore_R = 0.0

    with _cam_lock:
        visible = is_color_visible
        cam_err_x = camera_target_error_x

    if visible:
        # 색지가 보이면 타겟 방향에 엄청난 보너스 점수 부여 (장애물 점수를 압도)
        if cam_err_x < 0: # 색지가 화면 왼쪽에 있음
            term_color_L = SCORE_COLOR_TARGET * abs(cam_err_x)
        else:             # 색지가 화면 오른쪽에 있음
            term_color_R = SCORE_COLOR_TARGET * abs(cam_err_x)
    else:
        # 색지가 안 보이면, 완만하게 원/S자를 그리며 탐색하도록 좌측(또는 우측) 편향 점수 부여
        term_explore_L = SCORE_EXPLORE_BIAS

    score_L = term_gap_L + term_push_L + term_side_L + term_head_L + gap_bonus_L + term_color_L + term_explore_L
    score_R = term_gap_R + term_push_R + term_side_R + term_head_R + gap_bonus_R + term_color_R + term_explore_R

    # 방향 결정 로직 (기존 동일)
    global _last_direction
    score_diff = score_L - score_R
    if _last_direction > 0:
        direction = 1.0 if score_diff > -DIRECTION_HYSTERESIS else -1.0
    else:
        direction = -1.0 if score_diff < DIRECTION_HYSTERESIS else 1.0
    _last_direction = direction

    # v, w 계산 (기존 동일)
    v = FORWARD_SPEED # 예시
    w = direction * 0.5 # 예시
    
    return v, w

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [수정] 메인 진입점 (미션 상태 머신 추가)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def find_vw_command(scan_points, heading_deg):
    global stop_cycle_count, stop_pivot_w, stop_phase
    global mission_phase, arrive_time, current_color_idx

    # ── [미션 1단계] 색지 안착 후 정지 대기 ──
    if mission_phase == 1:
        if time.time() - arrive_time > 2.0: # 2초 정지 후
            current_color_idx += 1          # 다음 색상 타겟으로 전환
            mission_phase = 0               # 다시 주행 시작
            print(f"[MISSION] Next Target: {MISSION_COLORS[current_color_idx % len(MISSION_COLORS)]}")
        return 0.0, 0.0

    # ── [미션 0단계] 일반 주행 (탐색 또는 돌격) ──
    # [1순위 판단] 타겟 위에 도착했는가?
    with _cam_lock:
        bottom_y = color_bottom_y
        visible = is_color_visible
    
    # 색지가 프레임의 85% 하단 선을 넘었으면 도착으로 판단
    if visible and bottom_y > (FRAME_HEIGHT * ARRIVE_Y_RATIO):
        print(f"[MISSION] ARRIVED at {MISSION_COLORS[current_color_idx % len(MISSION_COLORS)]}!")
        mission_phase = 1
        arrive_time = time.time()
        return 0.0, 0.0

    # [2순위 판단] 라이다 긴급 장애물 정지구역인가? (기존 로직 유지)
    if stop_phase == 2:
        if not detect_stop_zone(scan_points):
            _stop_reset()
            return find_vw_layered(scan_points, heading_deg)
        # 피봇 로직 (기존 생략)
        return 0.0, stop_pivot_w

    if detect_stop_zone(scan_points):
        # 타겟에 도착하기 직전(색지가 화면 하단 근처)일 때는 라이다 스톱을 무시할 수도 있음(선택사항)
        # 여기서는 안전을 위해 스톱 우선 적용
        stop_phase = 2
        return 0.0, stop_pivot_w

    # [3/4순위 판단] 장애물 회피 + 비전 유도 (find_vw_layered 안에서 점수로 통합 처리됨)
    return find_vw_layered(scan_points, heading_deg)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [신규] 스레드: 카메라 비전 처리
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [수정] 스레드: 카메라 비전 처리 (세로 장착 대응)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def _camera_processor():
    global is_color_visible, camera_target_error_x, color_bottom_y, current_color_idx
    
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    # 카메라가 물리적으로 우측으로 누워있으므로, 
    # 원래 카메라 센서 기준으로는 해상도 세팅을 가로로 길게 해야 함 (예: 320x240)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) # 자동 노출 끄기 (환경에 따라 조절)
    
    kernel = np.ones((5,5), np.uint8)

    while not _shutdown.is_set():
        ret, raw_frame = cap.read()
        if not ret: continue

        # ── [핵심 변경] 프레임 원상 복구 (반시계 90도 회전) ──
        # 카메라가 우측(시계방향)으로 90도 누워 있으므로, 소프트웨어로 좌측(반시계) 90도 회전
        frame = cv2.rotate(raw_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        # 회전 후 프레임 크기 재계산 (예: 240x320)
        h, w, _ = frame.shape 

        target_name = MISSION_COLORS[current_color_idx % len(MISSION_COLORS)]
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
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                x, y, box_w, box_h = cv2.boundingRect(c)
                cx = x + box_w // 2
                
                # ── 오차 비율 계산 (회전된 너비 w 기준) ──
                # 화면 중심(w/2) 기준 오차 비율 (-1.0: 맨 왼쪽, 1.0: 맨 오른쪽)
                err_x = (cx - (w / 2)) / (w / 2)
                bottom_y = y + box_h
                
                with _cam_lock:
                    is_color_visible = True
                    camera_target_error_x = err_x
                    color_bottom_y = bottom_y
                found = True

                if SHOW_CV_WINDOW:
                    cv2.rectangle(frame, (x, y), (x+box_w, y+box_h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, y+box_h), 5, (0, 0, 255), -1)
                    # 도착 판정선(가로줄) 표시
                    arrive_line_y = int(h * ARRIVE_Y_RATIO)
                    cv2.line(frame, (0, arrive_line_y), (w, arrive_line_y), (255, 0, 0), 2)

        if not found:
            with _cam_lock:
                is_color_visible = False

        if SHOW_CV_WINDOW:
            cv2.putText(frame, f"TARGET: {target_name}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.imshow("Robot Vision (Rotated)", frame)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()
    
# (기존 라이다, 모터 스레드 유지)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def main():
    print("=== Robot Navigation + Vision Target ===")
    # ... (초기 설정 프린트)
    
    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    
    t_lidar  = threading.Thread(target=_lidar_reader, args=(lidar,), daemon=True)
    t_motor  = threading.Thread(target=_motor_controller, args=(arduino,), daemon=True)
    t_camera = threading.Thread(target=_camera_processor, daemon=True) # [추가] 카메라 스레드

    try:
        t_lidar.start()
        t_motor.start()
        t_camera.start()
        while not _shutdown.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        _shutdown.set()
        # ... 자원 해제 코드

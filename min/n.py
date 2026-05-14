import serial
import time
import math

# ── 1. 설정 및 파라미터 ───────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"

# 로봇 하드웨어 (mm) - 직사각형 히트박스 기준
ROBOT_FRONT = 110      # 라이다 중심 ~ 앞범퍼
ROBOT_BACK  = 150      # 라이다 중심 ~ 뒷범퍼
ROBOT_HALF_W = 110     # 라이다 중심 ~ 좌우 끝
MARGIN      = 30       # 수평/수직 안전 여유폭

# 주행 성능
MAX_V = 0.35           # m/s
MIN_V = 0.05           # m/s
MAX_W = 1.5            # rad/s

# DWA 채점 가중치 (환경에 맞춰 튜닝)
W_HEADING   = 2.0      # 목표 방향 추종
W_CLEARANCE = 1.5      # 장애물 회피 (안전)
W_VELOCITY  = 1.0      # 직진 본능
BIAS_BONUS  = 0.3      # 방향 고착화 (지그재그 방지)

# ── 2. FSM 및 전역 상태 ───────────────────────────────────────────────────────
class RobotState:
    DRIVE = 1
    RECOVERY = 2

current_state = RobotState.DRIVE
stuck_timer = 0.0
last_w_sign = 0.0

# ── 3. 유틸리티 함수 ──────────────────────────────────────────────────────────
def normalize_angle(angle):
    """각도를 -180 ~ 180도로 정규화"""
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

# ── 4. DWA 코어 (수학적 시뮬레이션) ───────────────────────────────────────────
def generate_vw_window(current_v, current_w):
    """현재 속도에서 선택 가능한 v, w 윈도우 생성 (단순화 버전)"""
    v_cands = [0.0, 0.15, 0.25, MAX_V]
    w_cands = [-MAX_W, -1.0, -0.5, 0.0, 0.5, 1.0, MAX_W]
    return v_cands, w_cands

def check_collision_and_clearance(v_m_s, w_rad_s, scan_points, predict_t=1.0, step=0.2):
    """미래 궤적(1초)을 직사각형 히트박스로 시뮬레이션하여 충돌 검사"""
    v_mm_s = v_m_s * 1000.0
    
    # 연산 최적화: 시뮬레이션 중 로봇 반경 밖의 점들은 무시
    max_dist = abs(v_mm_s * predict_t) + max(ROBOT_FRONT, ROBOT_BACK) + MARGIN + 100
    local_pts = [(dist * math.cos(math.radians(ang)), dist * math.sin(math.radians(ang))) 
                 for ang, dist in scan_points if 0 < dist <= max_dist]
                 
    if not local_pts: return 1000.0 # 앞이 뻥 뚫림
    
    curr_x, curr_y, curr_th = 0.0, 0.0, 0.0
    t = 0.0
    min_clear_sq = 1000000.0 # 제곱 거리로 비교 (루트 연산 최소화)

    front_bound = ROBOT_FRONT + MARGIN
    back_bound  = -ROBOT_BACK - MARGIN
    side_bound  = ROBOT_HALF_W + MARGIN

    while t <= predict_t:
        # Kinematics 운동학적 위치 추정 (Euler Integration)
        curr_x += v_mm_s * math.cos(curr_th) * step
        curr_y += v_mm_s * math.sin(curr_th) * step
        curr_th += w_rad_s * step
        t += step
        
        cos_t, sin_t = math.cos(curr_th), math.sin(curr_th)
        
        for px, py in local_pts:
            dx, dy = px - curr_x, py - curr_y
            
            # 회전 변환 (라이다 점을 현재 로봇의 로컬 좌표계로 가져옴)
            lx = dx * cos_t + dy * sin_t
            ly = -dx * sin_t + dy * cos_t
            
            # [직사각형 충돌 검사]
            if back_bound <= lx <= front_bound and -side_bound <= ly <= side_bound:
                return -1.0 # 충돌 궤적 폐기!
                
            dist_sq = dx**2 + dy**2
            if dist_sq < min_clear_sq:
                min_clear_sq = dist_sq

    return math.sqrt(min_clear_sq)

def run_dwa(scan_points, target_heading, current_v, current_w):
    global last_w_sign
    v_cands, w_cands = generate_vw_window(current_v, current_w)
    
    best_v, best_w = 0.0, 0.0
    max_score = -1.0
    
    for v in v_cands:
        for w in w_cands:
            clearance = check_collision_and_clearance(v, w, scan_points)
            if clearance <= 0: continue # 박치기하는 길은 무시
            
            # 1. 헤딩 점수 (목표 각도와의 일치율)
            pred_turn = math.degrees(w * 1.0)
            fut_heading = normalize_angle(target_heading - pred_turn)
            score_heading = max(0.0, 1.0 - (abs(fut_heading) / 180.0))
            
            # 2. 여유공간 점수 (장애물과 멀수록 좋음)
            score_clearance = min(1.0, clearance / 1000.0)
            
            # 3. 속도 점수
            score_velocity = max(0.0, v / MAX_V)
            
            # 4. 방향 고착 보너스
            bias = BIAS_BONUS if (w * last_w_sign > 0) else 0.0
            
            total_score = (W_HEADING * score_heading) + \
                          (W_CLEARANCE * score_clearance) + \
                          (W_VELOCITY * score_velocity) + bias
                          
            if total_score > max_score:
                max_score = total_score
                best_v, best_w = v, w
                
    if best_w != 0:
        last_w_sign = 1.0 if best_w > 0 else -1.0
        
    return best_v, best_w

# ── 5. 메인 루프 (교통정리) ───────────────────────────────────────────────────
def main():
    global current_state, stuck_timer
    
    # (통신 포트 초기화 로직 생략 - 이전 코드와 동일하게 적용)
    # lidar = serial.Serial(...)
    
    current_v, current_w = 0.0, 0.0
    target_heading = 0.0 # 아두이노에서 받아올 값
    dt = 0.1
    
    print("DWA + FSM Navigation Started")
    
    try:
        while True:
            scan_points = [] # 라이다에서 파싱된 [(angle, dist), ...] 데이터
            
            # [상태 머신 로직]
            if current_state == RobotState.DRIVE:
                v, w = run_dwa(scan_points, target_heading, current_v, current_w)
                
                if v == 0.0:
                    stuck_timer += dt
                    if stuck_timer >= 2.0:
                        print("[경고] 2초간 정지 -> Recovery 모드 진입!")
                        current_state = RobotState.RECOVERY
                        stuck_timer = 0.0
                else:
                    stuck_timer = 0.0
                    
            elif current_state == RobotState.RECOVERY:
                v, w = -0.1, 1.0 # 뒤로 살짝 빼면서 회전하여 시야 확보
                
                # 정면이 500mm 이상 확보되면 탈출
                front_clear = True # 실제로는 라이다 스캔으로 검사 구현 필요
                if front_clear:
                    print("[회복] 탈출 공간 확보 -> Drive 모드 복귀")
                    current_state = RobotState.DRIVE
                    
            current_v, current_w = v, w
            # arduino.write(f"{v:.2f} {w:.2f}\n".encode())
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("종료")

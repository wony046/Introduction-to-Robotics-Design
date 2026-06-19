import serial
import time
import math
import json
import threading
import camera_tracker

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 포트 & 라이다 설정
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

LIDAR_OFFSET    = 10     # mm: 라이다 측정값 보정
LIDAR_MIN_VALID = 100   # mm: 이 미만 무시 (노이즈)
DETECTION_RANGE = 1500  # mm: 라이다 최대 신뢰 거리

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 로봇 & 속도 파라미터
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

ROBOT_HALF_WIDTH = 110   # mm: 라이다 중심 ~ 좌우 끝

FORWARD_SPEED    = 0.3
MIN_SPEED        = 0.12
MAX_W            = 1.8
W_MIN_DANGER     = 0.5   # rad/s: 위험 시 최소 회전
W_SMOOTH         = 0.7

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 계층형 바운딩 박스 정의 (6개 레이어)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 각 레이어: 거리 범위, horiz 임계, w_gain, 기본 가중치, 동적 가중치 여부, v 영향 여부

LAYERS = [
    # L1: 가장 가까움, 동적 가중치, weight_cap=7.5, v_max=0.22
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':140,
     'w_gain':2.8, 'weight_base':0.8, 'weight_cap':7.5, 'weight_dynamic':True,
     'v_max':0.2, 'affects_v':True},
    # L2: 가까움, 동적 가중치, weight_cap=4.5, v_max=0.38
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':120,
     'w_gain':2.5, 'weight_base':0.6, 'weight_cap':5.0, 'weight_dynamic':True,
     'v_max':0.25, 'affects_v':True},
    # L3: 중간, 동적 가중치, weight_cap=2.5, v_max=FORWARD_SPEED
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':120,
     'w_gain':2.0, 'weight_base':0.4, 'weight_cap':4.5, 'weight_dynamic':True, 'affects_v':True},
    # L4: 중간-원거리 (weight: 진입 0.2 → 끝 0.1)
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':110,
     'w_gain':1.0, 'weight_base':0.1, 'weight_start':0.2, 'weight_dynamic':False, 'affects_v':True},
    # L5: 원거리 (weight: 진입 0.1 → 끝 0.05)
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':110,
     'w_gain':0.4, 'weight_base':0.05,'weight_start':0.1, 'weight_dynamic':False, 'affects_v':False},
    # L6: 최원거리 (weight: 진입 0.05 → 끝 0.02)
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':110,
     'w_gain':0.3, 'weight_base':0.02,'weight_start':0.05,'weight_dynamic':False, 'affects_v':False},
]

LAYER_PERCENTILE = 5    # %: 하위 N% dist 평균으로 레이어 대표점 계산

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP zone (계층형과 완전 별도)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP rectangle: 전방 100~175mm 사이, horiz < 105mm (210mm 폭)

STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 175
STOP_HORIZ_TH = 105

# STOP 탈출: 360° 전체 스캔, ROBOT_HALF_WIDTH*2 + 양쪽 20mm 마진
STOP_ESCAPE_MIN_GAP   = ROBOT_HALF_WIDTH * 2 + 40   # 260mm
STOP_MAX_CYCLES       = 30                          # 연속 STOP 사이클 상한 (초과 시 강제 탈출)
STOP_PIVOT_MAX_W      = 0.9   # rad/s: 피봇 최대 회전 속도 (목표에서 멀 때)
STOP_PIVOT_MIN_W      = 0.7   # rad/s: 피봇 최소 회전 속도 (목표 근처)
STOP_PIVOT_SLOW_DEG   = 15    # deg: 이 이내부터 선형 감속 시작

# FGM (Follow the Gap Method) — STOP escape 전용
FGM_MIN_ANG_DEG      = 3     # deg: 이 이상 각도 공백이면 갭으로 인식
FGM_MIN_DEPTH_MM     = 250   # mm: 갭 너머 최소 깊이 (얕은 함몰부 제외)
FGM_MAX_RANGE_MM     = 500   # mm: FGM 갭 탐색 최대 거리 (이 이상 포인트 무시)
FGM_RATIO_THRES      = 1.2   # 인접 포인트 거리 비율 이상이면 갭 경계로 인식 (벽 끝 완만 전환)

# 전방 갭 탐색 (기본 주행 방향 결정용)
FRONT_GAP_MIN_DEPTH  = 300   # mm: 전방 갭 최소 깊이 (이 미만 탈락)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 방향 점수제 (gap + layer 통합)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCORE_ALPHA       = 5.0    # gap_width 계수
SCORE_BETA        = 8      # 정면 레이어 push 계수
SCORE_SIDE        = 2500.0  # 측방 레이어 방향 가중치
HEADING_WEIGHT_MM = 5.0    # 헤딩 1° = 여유 5mm
DEPTH_JUMP_THRES  = 120    # mm: 이상이면 다른 물체로 인식

# 방향 히스테리시스: 이 점수 차 미만이면 직전 방향 유지 (정면 장애물 시 oscillation 방지)
DIRECTION_HYSTERESIS = 300.0

# ── 목표 방향 추종 (카메라 색지) ──────────────────────────────
GAP_TARGET_WEIGHT = 1.0           # 갭 선택: 목표 방향 추종 강도 (주 항)
GAP_SMOOTH_WEIGHT = 0.3           # 갭 선택: 직전 방향 유지 강도 (떨림 억제)
KP_GOAL            = MAX_W / 30.0  # 비례 조향 게인 (30° → MAX_W, 시간 단축을 위한 인코스 최적화)
TARGET_ALIGN_ANGLE = 60.0         # deg: 이 각도 이상이면 v=MIN_SPEED (거의 제자리 회전)
TARGET_CLEAR_CONE  = 18           # deg: 목표 방향 ± 이 각도 범위를 막힘 검사 대상으로
TARGET_BLOCK_DIST = 600           # mm: 이 거리 이내 장애물이 있으면 "목표 방향 막힘"

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 스캔 범위 & 통신
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCAN_WIDE_HALF = 135   # 측면 반발력 감지 범위 (is_in_wide_scan 사용) (각도)
SEND_INTERVAL  = 0.1

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 측면 반발력 파라미터 (horiz 190mm × fwd 160mm 감지 구간)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SIDE_SAFE_MARGIN  = 300   # mm: 로봇 측면 안전 마진 (side_th = 110+190 = 300mm)
SIDE_FWD_LEAD     = 90    # mm: 라이다 기준 전방 여유 (진입 예측)
SIDE_FWD_REAR     = 90    # mm: 라이다 기준 후방 깊이 (로봇 몸체)
SIDE_REPULSE_GAIN = 1.25   # rad/s: 반발력 최대 w 기여
SIDE_EXP_K        = 2.0   # 지수 계수: 클수록 근접 시 반발력이 급격히 증가

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 측방 방향 레이어 (±15°~±75°, 600mm)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SIDE_LAYER_ANG_START = 15   # deg: 정면 레이어와 경계
SIDE_LAYER_ANG_END   = 75   # deg: 측방 레이어 바깥 경계
SIDE_LAYER_DIST_MAX  = 700  # mm: 측방 감지 최대 거리
SIDE_W_BOOST_GAIN    = 1.5  # rad/s: 측방 레이어 w 크기 기여 계수 (우측 push → +w, 좌측 push → -w)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 가상 장애물 (통과 불가 갭) 
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
MIN_PASSAGE_WIDTH       = STOP_ESCAPE_MIN_GAP  # 260mm: 이 미만 갭 → 통과 불가 → 가상 장애물
VIRTUAL_OBS_GAIN        = 1.5   # 가상 장애물 척력 배율 (레이어별 horiz_th 기준)
VIRTUAL_CENTER_DEADBAND = 10    # deg: 갭 중심이 ±이내면 정면 → 양쪽 동등 척력
VIRTUAL_EXP_K           = 2.5   # 지수 계수: 클수록 좁은 갭에서 척력이 급격히 증가

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 가상 경계 & Seeker 모드 (색 미감지 시 활성화)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
BOUNDARY_RADIUS     = 1500.0   # mm: 가상 경계 반경 (색지 기준 1.5m)
BOUNDARY_BLEND_DIST = 300.0    # mm: 경계 초과 후 인력 100%까지 도달하는 거리
BOUNDARY_V_MIN      = 0.5      # 경계 완전 초과 시 v 감속 최소 비율 (원래 v의 50%)

# --- Seeker Spin 상태 제어용 전역 변수 ---
_prev_dwelling     = True    # 최초 시작 시에도 제자리 스캔을 수행하도록 True로 초기화
_spin_mode         = False
_spin_start_time   = None
SPIN_DURATION_SEC  = 6.0     # 360도 스캔 지속 시간
SPIN_W_SPEED       = 1.05    # 약 6초에 360도 회전하는 각속도 (rad/s)

# ── 디버그 토글 ──────────────────────────────────────────────────────────────
DEBUG_LAYERS      = 0   
DEBUG_STOP        = 0   
DEBUG_STOP_PIVOT  = 0   
DEBUG_BRANCH      = 0   
DEBUG_TARGET      = 0   
DEBUG_GAP         = 0   
DEBUG_FALLBACK    = 0   
DEBUG_SCORE       = 0   
DEBUG_DIR         = 0   
DEBUG_CLEAR       = 0   
DEBUG_FINAL       = 0   
DEBUG_SIDE        = 0   
DEBUG_SIDE_LAYER  = 0   
DEBUG_VIRTUAL     = 0   
DEBUG_CLOSE_INIT   = 1   
DEBUG_CLOSE_POS    = 0   
DEBUG_CLOSE_HDG    = 1   
DEBUG_CLOSE_DONE   = 1   
DEBUG_CLOSE_REMAIN = 1   
DEBUG_BOUNDARY    = 1   
DEBUG_SEND        = 0   

# ── 전역 상태 ────────────────────────────────────────────────────────────────
arduino_heading_deg   = 0.0
arduino_x_mm          = 0.0   
arduino_y_mm          = 0.0   
prev_w                = 0.0

# ── CLOSE 접근 제어 ───────────────────────────────────────────────────────────
_close_target_x    = None   
_close_target_y    = None   
_close_initial_dist = None  
_close_observe_start = None 
KP_CLOSE_HDG      = 0.1   
CLOSE_SPEED_MAX   = 0.2   
CLOSE_ARRIVE_MM   = 80    
CLOSE_OBSERVE_SEC = 0.5   
prev_desired_heading  = 0.0   
_last_direction       = 1.0   
stop_cycle_count           = 0     
stop_pivot_w               = 0.0   
stop_locked_target         = 0.0
stop_locked_gap            = 0.0
stop_locked_global_heading = 0.0
stop_phase                 = 0     

# ★ 가상 경계 전역 상태 (타겟 중심 앵커)
_boundary_center_x  = None   
_boundary_center_y  = None   

# ── 스레드 공유 상태 ─────────────────────────────────────────────────────────
_scan_lock   = threading.Lock()
_latest_scan = []             
_shutdown    = threading.Event()  


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return ((angle + 180) % 360) - 180

def is_in_front_90(a):
    return -90 <= a <= 90

def is_in_wide_scan(a):
    return -SCAN_WIDE_HALF <= a <= SCAN_WIDE_HALF

def decompose(angle_deg, dist):
    rad = math.radians(angle_deg)
    horiz = abs(dist * math.sin(rad))
    fwd   = dist * math.cos(rad)
    return horiz, fwd

def cosine_dist(d1, d2, angle_diff_deg):
    theta = math.radians(abs(angle_diff_deg))
    return math.sqrt(d1**2 + d2**2 - 2 * d1 * d2 * math.cos(theta))

def point_to_segment_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    seg_sq = dx*dx + dy*dy
    if seg_sq == 0:
        return math.sqrt((px - ax)**2 + (py - ay)**2)
    t = max(0.0, min(1.0, ((px - ax)*dx + (py - ay)*dy) / seg_sq))
    return math.sqrt((px - ax - t*dx)**2 + (py - ay - t*dy)**2)

def nearest_to_segments(px, py, cluster_xy):
    if len(cluster_xy) == 1:
        ox, oy = cluster_xy[0]
        return math.sqrt((px - ox)**2 + (py - oy)**2)
    return min(
        point_to_segment_dist(px, py,
                              cluster_xy[j][0], cluster_xy[j][1],
                              cluster_xy[j+1][0], cluster_xy[j+1][1])
        for j in range(len(cluster_xy) - 1)
    )

def parse_packet(data):
    if len(data) != 5: return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1: return None
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    distance_q2 = data[3] | (data[4] << 8)
    return (angle_q6 / 64.0), (distance_q2 / 4.0)

def read_arduino(arduino):
    global arduino_heading_deg, arduino_x_mm, arduino_y_mm
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('O:'):
                parts = line[2:].split(',')
                if len(parts) == 3:
                    arduino_x_mm        = float(parts[0])
                    arduino_y_mm        = float(parts[1])
                    arduino_heading_deg = float(parts[2])
            elif line.startswith('H:'):   
                arduino_heading_deg = float(line[2:])
        except Exception: pass


def _compute_close_target():
    bearing_global_deg = arduino_heading_deg + camera_tracker.get_last_close_bearing()
    dist_mm            = camera_tracker.get_estimated_distance_mm()
    hdg_rad            = math.radians(bearing_global_deg)
    x_t = arduino_x_mm + dist_mm * math.sin(hdg_rad)
    y_t = arduino_y_mm + dist_mm * math.cos(hdg_rad)
    if DEBUG_CLOSE_INIT:
        print(f"[CLOSE] 목표 좌표: ({x_t:.0f}, {y_t:.0f})mm  "
              f"dist={dist_mm:.0f}mm  global_bearing={bearing_global_deg:.1f}°")
    return x_t, y_t


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 가상 경계 제어 함수 (바운더리 중심점 갱신 로직이 메인 루프로 이동됨)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _get_boundary_correction():
    if _boundary_center_x is None:
        return 0.0, 1.0

    dx   = _boundary_center_x - arduino_x_mm
    dy   = _boundary_center_y - arduino_y_mm
    dist = math.sqrt(dx**2 + dy**2)

    if dist <= BOUNDARY_RADIUS:
        return 0.0, 1.0

    excess = dist - BOUNDARY_RADIUS
    blend  = min(excess / BOUNDARY_BLEND_DIST, 1.0)

    bearing_to_center = math.degrees(math.atan2(dx, dy))
    rel_bearing       = normalize_angle(bearing_to_center - arduino_heading_deg)

    v_scale = BOUNDARY_V_MIN + (1.0 - BOUNDARY_V_MIN) * (1.0 - blend)

    if DEBUG_BOUNDARY:
        print(f"  [BOUNDARY] 배회 반경 이탈 (dist={dist:.0f}mm > {BOUNDARY_RADIUS}mm) "
              f"-> 타겟 중심 복귀 유도 target_b={rel_bearing:+.1f}° v_scale={v_scale:.2f}")

    return rel_bearing, v_scale


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP zone 감지 & 탈출
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def detect_stop_zone(scan_points):
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_front_90(angle_norm): continue
        horiz, fwd = decompose(angle_norm, dist)
        if STOP_FWD_MIN <= fwd <= STOP_FWD_MAX and horiz < STOP_HORIZ_TH:
            return True
    return False


def find_all_gaps(scan_points):
    pts = sorted(
        [(a, d) for a, d in scan_points
         if LIDAR_MIN_VALID < d < FGM_MAX_RANGE_MM],
        key=lambda p: p[0]
    )
    if len(pts) < 2:
        return []

    def to_xy(a, d):
        r = math.radians(a)
        return d * math.sin(r), d * math.cos(r)

    gap_indices = []
    for i in range(len(pts) - 1):
        a1, d1 = pts[i]
        a2, d2 = pts[i + 1]
        ang_diff = a2 - a1
        is_depth_jump   = abs(d2 - d1) > DEPTH_JUMP_THRES
        is_angular_hole = ang_diff >= FGM_MIN_ANG_DEG
        is_ratio_jump   = (d2 / d1 > FGM_RATIO_THRES) or (d1 / d2 > FGM_RATIO_THRES)
        if is_depth_jump or is_angular_hole or is_ratio_jump:
            gap_indices.append(i)

    if not gap_indices:
        return []

    gap_set = set(gap_indices)
    cluster_ids = []
    cid = 0
    for i in range(len(pts)):
        cluster_ids.append(cid)
        if i in gap_set:
            cid += 1

    n_clusters = cluster_ids[-1] + 1
    clusters_xy = [[] for _ in range(n_clusters)]
    for i, (a, d) in enumerate(pts):
        clusters_xy[cluster_ids[i]].append(to_xy(a, d))

    gaps = []
    for i in gap_indices:
        a1, d1 = pts[i]
        a2, d2 = pts[i + 1]
        x1, y1 = to_xy(a1, d1)
        x2, y2 = to_xy(a2, d2)

        cid_L = cluster_ids[i]
        cid_R = cluster_ids[i + 1]

        d_LR = nearest_to_segments(x1, y1, clusters_xy[cid_R])
        d_RL = nearest_to_segments(x2, y2, clusters_xy[cid_L])

        width = min(d_LR, d_RL)

        center_angle = math.degrees(math.atan2((x1 + x2) / 2, (y1 + y2) / 2))

        gaps.append({
            'width':        width,
            'center_angle': center_angle,
            'edge_a':       (a1, d1),
            'edge_b':       (a2, d2),
            'depth':        max(d1, d2),
        })
    return gaps


def choose_escape_gap(gaps, prefer_angle=0.0):
    passable = [g for g in gaps
                if g['width'] >= STOP_ESCAPE_MIN_GAP
                and g['depth'] >= FGM_MIN_DEPTH_MM]
    if passable:
        return min(passable,
                   key=lambda g: abs(((g['center_angle'] - prefer_angle) + 180) % 360 - 180))
    return None


def find_stop_escape_direction(scan_points, heading_deg=0.0):
    gaps   = find_all_gaps(scan_points)
    chosen = choose_escape_gap(gaps, prefer_angle=heading_deg)

    if chosen is None:
        return 0.0, 0.0, []

    gap_info = [
        {
            'width':        g['width'],
            'center_angle': g['center_angle'],
            'edge_a':       list(g['edge_a']),
            'edge_b':       list(g['edge_b']),
            'depth':        g['depth'],
            'passable':     g['width'] >= STOP_ESCAPE_MIN_GAP and g['depth'] >= FGM_MIN_DEPTH_MM,
            'chosen':       g is chosen,
        }
        for g in gaps
    ]
    return float(chosen['center_angle']), float(chosen['width']), gap_info


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 레이어 처리
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def process_layer(scan_points, layer):
    pts = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_front_90(angle_norm): continue
        horiz, fwd = decompose(angle_norm, dist)
        if layer['fwd_min'] <= fwd < layer['fwd_max'] and horiz < layer['horiz_th']:
            pts.append({
                'angle': angle_norm, 'dist': dist,
                'horiz': horiz, 'fwd': fwd,
                'horiz_error': layer['horiz_th'] - horiz,
            })

    if not pts:
        return None

    n_take = max(1, int(len(pts) * LAYER_PERCENTILE / 100))
    rep = sorted(pts, key=lambda p: p['dist'])[:n_take]

    rep_angle = sum(p['angle'] for p in rep) / len(rep)
    rep_horiz = sum(p['horiz'] for p in rep) / len(rep)
    rep_fwd   = sum(p['fwd']   for p in rep) / len(rep)
    rep_h_err = layer['horiz_th'] - rep_horiz

    if layer['weight_dynamic']:
        cap = layer.get('weight_cap', 1.0)
        raw = rep_h_err / layer['horiz_th'] * cap
        weight = max(layer['weight_base'], min(cap, raw))
    else:
        progress = (rep_fwd - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min'])
        progress = max(0.0, min(1.0, progress))
        weight = layer['weight_start'] + (layer['weight_base'] - layer['weight_start']) * progress

    urgency = layer['w_gain'] * rep_h_err / layer['horiz_th']

    if layer['affects_v']:
        progress = (rep_fwd - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min'])
        progress = max(0.0, min(1.0, progress))
        v_max = layer.get('v_max', FORWARD_SPEED)
        v_proposal = MIN_SPEED + (v_max - MIN_SPEED) * progress
    else:
        v_proposal = None

    push_left  = sum(p['horiz_error'] for p in rep if p['angle'] < 0)
    push_right = sum(p['horiz_error'] for p in rep if p['angle'] > 0)

    return {
        'name': layer['name'],
        'weight': weight, 'urgency': urgency, 'v_proposal': v_proposal,
        'rep_angle': rep_angle, 'rep_horiz': rep_horiz, 'rep_fwd': rep_fwd,
        'push_left': push_left, 'push_right': push_right,
        'n_points': len(pts),
    }


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Gap 너비 계산 (코사인 법칙)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def get_gap_width(scan_points, ref_angle, ref_dist, is_left):
    front = [(a, d) for a, d in scan_points if is_in_front_90(a)]

    if is_left:
        search = sorted([p for p in front if p[0] < ref_angle],
                        key=lambda x: x[0], reverse=True)
    else:
        search = sorted([p for p in front if p[0] > ref_angle],
                        key=lambda x: x[0])

    if not search:
        return 0.0

    edge_p = (ref_angle, ref_dist)
    for i, p in enumerate(search):
        if abs(p[1] - edge_p[1]) > DEPTH_JUMP_THRES:
            wall = search[i:]
            if wall:
                return min(cosine_dist(edge_p[1], wp[1], abs(edge_p[0] - wp[0]))
                           for wp in wall)
        edge_p = p

    rem_angle = abs((-90 - edge_p[0]) if is_left else (90 - edge_p[0]))
    if rem_angle > 15:
        return cosine_dist(edge_p[1], edge_p[1], rem_angle)
    return 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 측면 반발력 (50mm × 240mm 레이어)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def get_side_repulsion(scan_points):
    side_inner = ROBOT_HALF_WIDTH               
    side_outer = ROBOT_HALF_WIDTH + SIDE_SAFE_MARGIN  

    left_str  = 0.0
    right_str = 0.0

    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_wide_scan(angle_norm): continue

        horiz, fwd = decompose(angle_norm, dist)

        if fwd > SIDE_FWD_LEAD or fwd < -SIDE_FWD_REAR: continue
        if horiz < side_inner or horiz >= side_outer: continue

        t = (horiz - side_inner) / SIDE_SAFE_MARGIN
        strength = (math.exp(SIDE_EXP_K * (1.0 - t)) - 1.0) / (math.exp(SIDE_EXP_K) - 1.0)

        if angle_norm < 0:
            left_str = max(left_str, strength)
        else:
            right_str = max(right_str, strength)

    delta_w = (right_str - left_str) * SIDE_REPULSE_GAIN

    if DEBUG_SIDE and (left_str > 0 or right_str > 0):
        print(f"  [SIDE] L={left_str:.2f} R={right_str:.2f} dw={delta_w:+.3f} "
              f"(zone {side_inner}~{side_outer}mm)")

    return delta_w, left_str, right_str


def get_side_layer_push(scan_points):
    left_push  = 0.0
    right_push = 0.0

    for angle, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > SIDE_LAYER_DIST_MAX:
            continue
        strength = (SIDE_LAYER_DIST_MAX - dist) / SIDE_LAYER_DIST_MAX

        if -SIDE_LAYER_ANG_END <= angle <= -SIDE_LAYER_ANG_START:
            left_push = max(left_push, strength)
        elif SIDE_LAYER_ANG_START <= angle <= SIDE_LAYER_ANG_END:
            right_push = max(right_push, strength)

    if DEBUG_SIDE_LAYER and (left_push > 0 or right_push > 0):
        print(f"  [SIDE_LAYER] L={left_push:.2f} R={right_push:.2f} "
              f"-> score R+={SCORE_SIDE*left_push:.0f} L+={SCORE_SIDE*right_push:.0f}")

    return left_push, right_push


def get_front_passable_gaps(scan_points):
    front = sorted(
        [(a, d) for a, d in scan_points
         if is_in_front_90(a) and LIDAR_MIN_VALID < d < DETECTION_RANGE],
        key=lambda p: p[0]
    )
    if len(front) < 2:
        return []

    def to_xy(a, d):
        r = math.radians(a)
        return d * math.sin(r), d * math.cos(r)

    edge_indices = []
    for i in range(len(front) - 1):
        a1, d1 = front[i]
        a2, d2 = front[i + 1]
        ang_diff = a2 - a1
        is_depth_jump   = abs(d2 - d1) > DEPTH_JUMP_THRES
        is_angular_hole = ang_diff >= FGM_MIN_ANG_DEG
        is_ratio_jump   = (d2 / d1 > FGM_RATIO_THRES) or (d1 / d2 > FGM_RATIO_THRES)
        if is_depth_jump or is_angular_hole or is_ratio_jump:
            edge_indices.append(i)

    if not edge_indices:
        return []

    gap_set = set(edge_indices)
    cluster_ids = []
    cid = 0
    for i in range(len(front)):
        cluster_ids.append(cid)
        if i in gap_set:
            cid += 1

    n_clusters = cluster_ids[-1] + 1
    clusters_xy = [[] for _ in range(n_clusters)]
    for i, (a, d) in enumerate(front):
        clusters_xy[cluster_ids[i]].append(to_xy(a, d))

    def depth_at_center(center_ang):
        if not front:
            return FGM_MAX_RANGE_MM
        best = min(front, key=lambda p: abs(p[0] - center_ang))
        return best[1] if abs(best[0] - center_ang) < 10.0 else FGM_MAX_RANGE_MM

    passable = []
    for i in edge_indices:
        a1, d1 = front[i]
        a2, d2 = front[i + 1]
        x1, y1 = to_xy(a1, d1)
        x2, y2 = to_xy(a2, d2)

        cid_L = cluster_ids[i]
        cid_R = cluster_ids[i + 1]

        width = min(nearest_to_segments(x1, y1, clusters_xy[cid_R]),
                    nearest_to_segments(x2, y2, clusters_xy[cid_L]))

        if width < STOP_ESCAPE_MIN_GAP:
            continue

        center_angle = math.degrees(math.atan2((x1 + x2) / 2, (y1 + y2) / 2))
        depth = depth_at_center(center_angle)

        if depth < FRONT_GAP_MIN_DEPTH:
            continue

        passable.append({
            'center_angle': center_angle,
            'width':        width,
            'depth':        depth,
            'score':        width * depth,
        })

    return sorted(passable, key=lambda g: g['score'], reverse=True)


def choose_target_gap(passable_gaps, target_bearing, prev_heading):
    if not passable_gaps:
        return None

    def cost(g):
        d_target = abs(((g['center_angle'] - target_bearing) + 180) % 360 - 180)
        d_prev   = abs(((g['center_angle'] - prev_heading)   + 180) % 360 - 180)
        return GAP_TARGET_WEIGHT * d_target + GAP_SMOOTH_WEIGHT * d_prev

    return min(passable_gaps, key=cost)


def is_target_blocked(scan_points, target_bearing):
    for a, d in scan_points:
        if LIDAR_MIN_VALID < d < TARGET_BLOCK_DIST and abs(a - target_bearing) < TARGET_CLEAR_CONE:
            return True
    return False


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 가상 장애물 척력 
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def get_narrow_gap_pushes(scan_points, layer, in_stop=False):
    if in_stop:
        return 0.0, 0.0

    pts = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE:
            continue
        if not is_in_front_90(angle_norm):
            continue
        _, fwd = decompose(angle_norm, dist)
        if layer['fwd_min'] <= fwd < layer['fwd_max']:
            pts.append((angle_norm, dist))

    if len(pts) < 2:
        return 0.0, 0.0

    pts_sorted = sorted(pts, key=lambda p: p[0])

    opening_edges = []
    closing_edges = []

    for i in range(len(pts_sorted) - 1):
        a1, d1 = pts_sorted[i]
        a2, d2 = pts_sorted[i + 1]
        if abs(d2 - d1) <= DEPTH_JUMP_THRES:
            continue
        if d2 > d1:
            opening_edges.append((a1, d1))
        else:
            closing_edges.append((a2, d2))

    if not opening_edges or not closing_edges:
        return 0.0, 0.0

    virtual_push_left  = 0.0
    virtual_push_right = 0.0

    for ao, do in opening_edges:
        candidates = [(ac, dc) for ac, dc in closing_edges if ac > ao]
        if not candidates:
            continue

        ac, dc = min(candidates, key=lambda x: x[0])

        xo = do * math.sin(math.radians(ao))
        xc = dc * math.sin(math.radians(ac))
        gap_width = abs(xc - xo)

        if gap_width >= MIN_PASSAGE_WIDTH:
            continue

        horiz_o   = abs(xo)
        horiz_c   = abs(xc)
        inside_o  = horiz_o < layer['horiz_th']
        inside_c  = horiz_c < layer['horiz_th']

        if inside_o and inside_c:
            overlap_scale = 0.0
        elif inside_o or inside_c:
            overlap_scale = 0.4
        else:
            overlap_scale = 1.0

        if overlap_scale == 0.0:
            continue

        t         = gap_width / MIN_PASSAGE_WIDTH
        t         = max(0.0, min(1.0, t))
        exp_ratio = (math.exp(VIRTUAL_EXP_K * (1.0 - t)) - 1.0) \
                  / (math.exp(VIRTUAL_EXP_K) - 1.0)
        strength  = exp_ratio * layer['horiz_th'] * VIRTUAL_OBS_GAIN * overlap_scale

        center_angle = (ao + ac) / 2.0

        if abs(center_angle) < VIRTUAL_CENTER_DEADBAND:
            half = strength * 0.5
            virtual_push_left  = max(virtual_push_left,  half)
            virtual_push_right = max(virtual_push_right, half)
        elif center_angle < 0:
            virtual_push_left  = max(virtual_push_left,  strength)
        else:
            virtual_push_right = max(virtual_push_right, strength)

    return virtual_push_left, virtual_push_right


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 계층형 v/w 산출 (메인 로직)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_layered(scan_points, heading_deg, target_bearing=0.0):
    global _last_direction, prev_desired_heading

    layer_results = []
    for layer in LAYERS:
        r = process_layer(scan_points, layer)
        if r is not None:
            layer_results.append(r)

    v_layers = [r for r in layer_results if r['v_proposal'] is not None]
    if v_layers:
        total_w_v = sum(r['weight'] for r in v_layers)
        v = sum(r['weight'] * r['v_proposal'] for r in v_layers) / total_w_v
    else:
        v = FORWARD_SPEED

    front_gaps = get_front_passable_gaps(scan_points)
    chosen_gap = choose_target_gap(front_gaps, target_bearing, prev_desired_heading)
    blocked    = is_target_blocked(scan_points, target_bearing)
    
    side_left_push, side_right_push = get_side_layer_push(scan_points)

    if not blocked:
        desired_heading      = target_bearing
        prev_desired_heading = desired_heading
        w = KP_GOAL * desired_heading

        align_factor = max(0.0, 1.0 - abs(desired_heading) / TARGET_ALIGN_ANGLE)
        v = MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * align_factor

    elif chosen_gap is not None:
        desired_heading      = chosen_gap['center_angle']
        prev_desired_heading = desired_heading
        w = KP_GOAL * desired_heading

    elif layer_results:
        closest   = min(layer_results, key=lambda r: r['rep_horiz'])
        ref_angle = closest['rep_angle']
        ref_dist  = math.sqrt(closest['rep_horiz']**2 + closest['rep_fwd']**2)

        gap_L = get_gap_width(scan_points, ref_angle, ref_dist, is_left=True)
        gap_R = get_gap_width(scan_points, ref_angle, ref_dist, is_left=False)

        sum_pR = sum(r['weight'] * r['push_right'] for r in layer_results)
        sum_pL = sum(r['weight'] * r['push_left']  for r in layer_results)

        virt_push_L_total = 0.0
        virt_push_R_total = 0.0
        for layer in LAYERS:
            vpl, vpr = get_narrow_gap_pushes(
                scan_points, layer, in_stop=(stop_phase == 2))
            virt_push_L_total = max(virt_push_L_total, vpl)
            virt_push_R_total = max(virt_push_R_total, vpr)

        effective_push_R = max(sum_pR, virt_push_R_total)
        effective_push_L = max(sum_pL, virt_push_L_total)

        term_gap_L  = SCORE_ALPHA * gap_L
        term_gap_R  = SCORE_ALPHA * gap_R
        term_push_L = SCORE_BETA  * effective_push_R
        term_push_R = SCORE_BETA  * effective_push_L
        term_side_L = SCORE_SIDE  * side_right_push
        term_side_R = SCORE_SIDE  * side_left_push
        term_head_L = max(0.0, -heading_deg) * HEADING_WEIGHT_MM
        term_head_R = max(0.0,  heading_deg) * HEADING_WEIGHT_MM

        score_L = term_gap_L + term_push_L + term_side_L + term_head_L
        score_R = term_gap_R + term_push_R + term_side_R + term_head_R

        score_diff = score_L - score_R
        if _last_direction > 0:
            direction = 1.0 if score_diff > -DIRECTION_HYSTERESIS else -1.0
        else:
            direction = -1.0 if score_diff < DIRECTION_HYSTERESIS else 1.0
        
        _last_direction = direction

        total_w_all = sum(r['weight'] for r in layer_results)
        w_mag = sum(r['weight'] * r['urgency'] for r in layer_results) / total_w_all
        w_mag = max(min(w_mag, MAX_W), W_MIN_DANGER)
        w = direction * w_mag

    else:
        desired_heading      = target_bearing
        prev_desired_heading = desired_heading
        w = KP_GOAL * target_bearing

    w += (side_right_push - side_left_push) * SIDE_W_BOOST_GAIN
    side_dw, _, _ = get_side_repulsion(scan_points)
    w = max(min(w + side_dw, MAX_W), -MAX_W)

    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 진입점 (STOP 우선)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _stop_reset():
    global stop_cycle_count, stop_pivot_w, stop_phase, _last_direction
    stop_cycle_count = 0
    stop_pivot_w     = 0.0
    stop_phase       = 0
    _last_direction  = 1.0


def _stop_set_pivot(heading_deg, target, gap_width):
    global stop_locked_target, stop_locked_gap, stop_locked_global_heading, stop_pivot_w
    stop_locked_target = target
    stop_locked_gap    = gap_width
    if gap_width == 0:
        stop_locked_global_heading = 0.0
        stop_pivot_w = (-math.copysign(MAX_W, heading_deg)
                        if abs(heading_deg) > 1 else -MAX_W)
    else:
        stop_locked_global_heading = ((heading_deg - target) + 180) % 360 - 180
        stop_pivot_w = -MAX_W if abs(target) < 5 else -math.copysign(MAX_W, target)


def find_vw_command(scan_points, heading_deg, target_bearing=0.0):
    global stop_cycle_count, stop_pivot_w, stop_locked_target, stop_locked_gap, \
           stop_locked_global_heading, stop_phase

    if stop_phase == 2:
        if not detect_stop_zone(scan_points):
            _stop_reset()
            return find_vw_layered(scan_points, heading_deg, target_bearing)

        stop_cycle_count += 1
        if stop_cycle_count >= STOP_MAX_CYCLES:
            _stop_reset()
            return find_vw_layered(scan_points, heading_deg, target_bearing)

        err   = abs(((heading_deg - stop_locked_global_heading) + 180) % 360 - 180)
        scale = min(1.0, err / STOP_PIVOT_SLOW_DEG)
        speed = STOP_PIVOT_MIN_W + (STOP_PIVOT_MAX_W - STOP_PIVOT_MIN_W) * scale
        dyn_w = math.copysign(speed, stop_pivot_w)
        return 0.0, dyn_w

    if detect_stop_zone(scan_points):
        target, gap_width, gap_info = find_stop_escape_direction(scan_points, heading_deg)
        _stop_set_pivot(heading_deg, target, gap_width)
        stop_cycle_count = 0
        stop_phase       = 2
        return 0.0, stop_pivot_w

    return find_vw_layered(scan_points, heading_deg, target_bearing)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 스레드: 라이다 수신 / 모터 제어
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _dedup_scan(pts):
    angle_map = {}
    for angle, dist in pts:
        if dist == 0:
            continue
        bucket = round(angle)
        if bucket not in angle_map or dist < angle_map[bucket]:
            angle_map[bucket] = dist
    return list(angle_map.items())


def _lidar_reader(lidar):
    local_pts = []
    while not _shutdown.is_set():
        try:
            raw = lidar.read(5)
        except Exception:
            continue
        result = parse_packet(raw)
        if result is None:
            continue
        angle_raw, distance = result
        s_flag = raw[0] & 0x01
        if s_flag == 1 and local_pts:
            deduped = _dedup_scan(local_pts)
            with _scan_lock:
                _latest_scan.clear()
                _latest_scan.extend(deduped)
            local_pts = []
        local_pts.append((
            normalize_angle(angle_raw),
            distance + LIDAR_OFFSET if distance > 0 else 0
        ))


def _motor_controller(arduino):
    global prev_w, _close_target_x, _close_target_y, _close_initial_dist, _close_observe_start, \
           _boundary_center_x, _boundary_center_y, _prev_dwelling, _spin_mode, _spin_start_time

    last_cmd_str = ""
    while not _shutdown.is_set():
        read_arduino(arduino)
        with _scan_lock:
            pts = [(a, d) for a, d in _latest_scan if d > 0]
        
        if pts:
            is_dwelling_now = camera_tracker.is_dwelling()
            is_done_now = camera_tracker.is_done()

            # ── 상태 1: DWELL / DONE → 정지
            if is_done_now or is_dwelling_now:
                v, w = 0.0, 0.0
                prev_w = 0.0
                _close_target_x = _close_target_y = _close_initial_dist = _close_observe_start = None
                
                if is_dwelling_now:
                    _prev_dwelling = True
                    # 목표 색지 위에 도달하여 멈췄을 때, 해당 위치를 새로운 바운더리의 중심으로 확정(고정)
                    _boundary_center_x = arduino_x_mm
                    _boundary_center_y = arduino_y_mm

            # ── 상태 1.5: 제자리 회전 탐색 (Seeker Mode)
            elif _spin_mode:
                # 스핀 중 색상을 찾으면 즉시 중단하고 배회 모드로 돌입(배회 로직에서 다시 색상쪽으로 유도됨)
                if camera_tracker.get_bearing() is not None:
                    print("[SPIN] 타겟 감지! 회전 탐색 조기 종료")
                    _spin_mode = False
                    v, w = 0.0, 0.0
                else:
                    elapsed = time.time() - _spin_start_time
                    if elapsed >= SPIN_DURATION_SEC:
                        print(f"[SPIN] {SPIN_DURATION_SEC}초 회전 종료, 배회 모드로 전환")
                        _spin_mode = False
                        v, w = 0.0, 0.0
                    else:
                        v = 0.0
                        w = SPIN_W_SPEED

            # ── 상태 2: CLOSE → 오도메트리 위치 제어
            elif camera_tracker.is_close() or _close_target_x is not None:
                if _close_target_x is None:
                    if _close_observe_start is None:
                        _close_observe_start = time.time()
                        print(f"[CLOSE] 관측 시작 — {CLOSE_OBSERVE_SEC:.1f}s 정지")
                    elapsed = time.time() - _close_observe_start
                    remaining = CLOSE_OBSERVE_SEC - elapsed
                    if remaining > 0:
                        v, w = 0.0, 0.0
                        prev_w = 0.0
                        w_smooth = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
                        cmd = f"{v:.2f} {w_smooth:.2f}\n"
                        arduino.write(cmd.encode())
                        time.sleep(SEND_INTERVAL)
                        continue
                    _close_target_x, _close_target_y = _compute_close_target()
                    _close_initial_dist = None

                ex = _close_target_x - arduino_x_mm
                ey = _close_target_y - arduino_y_mm
                dist_err = math.sqrt(ex ** 2 + ey ** 2)
                if _close_initial_dist is None:
                    _close_initial_dist = max(dist_err, 1.0)

                if dist_err < CLOSE_ARRIVE_MM:
                    v, w = 0.0, 0.0
                    prev_w = 0.0
                    camera_tracker.signal_arrival()
                    if DEBUG_CLOSE_DONE:
                        print(f"[CLOSE] 도달 ({dist_err:.0f}mm < {CLOSE_ARRIVE_MM}mm) → 정지")
                else:
                    target_hdg = math.degrees(math.atan2(ex, ey))
                    hdg_err    = normalize_angle(target_hdg - arduino_heading_deg)

                    w = max(min(KP_CLOSE_HDG * hdg_err, MAX_W), -MAX_W)
                    v = CLOSE_SPEED_MAX
                    prev_w = w

            # ── 상태 3: SEEK → 카메라 bearing + 라이다 회피 / 바운더리 배회
            else:
                # DWELL에서 방금 빠져나왔다면, SEEK로 가기 전에 SPIN 모드로 인터셉트
                if _prev_dwelling:
                    _prev_dwelling = False
                    _spin_mode = True
                    _spin_start_time = time.time()
                    print(f"[STATE] DWELL 종료 → SPIN 모드 (다음 색상 탐색) 진입")
                    continue  # 현재 루프 스킵하고 위쪽 스핀 로직으로 전환

                _close_target_x = _close_target_y = _close_initial_dist = _close_observe_start = None
                bearing = camera_tracker.get_bearing()
                
                if bearing is not None:
                    # ** 타겟을 검출했을 때: 색지의 절대 위치(세계 좌표)를 바운더리 중심으로 계속 갱신 **
                    # (색지를 향해 가는 도중에도 색지의 예상 위치로 중심이 이동됨)
                    dist_mm = camera_tracker.get_estimated_distance_mm()
                    if dist_mm is not None:
                        global_bearing = math.radians(arduino_heading_deg + bearing)
                        _boundary_center_x = arduino_x_mm + dist_mm * math.sin(global_bearing)
                        _boundary_center_y = arduino_y_mm + dist_mm * math.cos(global_bearing)
                    
                    v, w = find_vw_command(pts, arduino_heading_deg, target_bearing=bearing)
                
                else:
                    # ** 타겟 미검출 시 (배회): 마지막으로 갱신된(또는 고정된) 바운더리 중심 기준 1.5m 배회 **
                    if _boundary_center_x is None:
                        # 최초 실행 시 (0,0)을 앵커로 잡기 위함
                        _boundary_center_x = arduino_x_mm
                        _boundary_center_y = arduino_y_mm
                        print(f"[BOUNDARY] 최초 중심 설정: ({arduino_x_mm:.0f}, {arduino_y_mm:.0f})mm")
                        
                    boundary_tb, v_scale = _get_boundary_correction()
                    v, w = find_vw_command(pts, arduino_heading_deg, target_bearing=boundary_tb)
                    v *= v_scale

            w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
            prev_w = w
            cmd = f"{v:.2f} {w:.2f}\n"
            arduino.write(cmd.encode())
            
            if cmd != last_cmd_str and DEBUG_SEND:
                print(f"[SEND] v={v:.2f}  w={w:+.2f}  "
                      f"pos=({arduino_x_mm:.0f},{arduino_y_mm:.0f})mm  "
                      f"hdg={arduino_heading_deg:.1f}°")
                last_cmd_str = cmd
                
        time.sleep(SEND_INTERVAL)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    print("=== RPLIDAR Obstacle Avoidance (Target Anchoring & Seeker Spin) ===")
    print(f"  Scoring     : alpha={SCORE_ALPHA} beta={SCORE_BETA}")
    print(f"  Boundary    : Anchor = Target Color Coord, Radius = {BOUNDARY_RADIUS}mm")
    print(f"  Seeker Spin : Spin {SPIN_DURATION_SEC}sec after target completion")
    print("=" * 70)

    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)

    arduino.write(b"R\n")
    time.sleep(0.1)
    print("[INIT] Arduino heading reset sent")

    lidar.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))
    lidar.read(7)

    t_lidar = threading.Thread(target=_lidar_reader,      args=(lidar,),   daemon=True, name="lidar")
    t_motor = threading.Thread(target=_motor_controller,  args=(arduino,), daemon=True, name="motor")

    try:
        camera_tracker.start()
        t_lidar.start()
        t_motor.start()
        while not _shutdown.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        _shutdown.set()
        camera_tracker.stop()
        t_lidar.join(timeout=2.0)
        t_motor.join(timeout=2.0)
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        lidar.close()
        arduino.write(b"0.00 0.00\n")
        arduino.close()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()

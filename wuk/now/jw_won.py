import serial
import time
import math

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 포트 & 라이다 설정
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

LIDAR_OFFSET    = 20    # mm: 라이다 측정값 보정
LIDAR_MIN_VALID = 100   # mm: 이 미만 무시 (노이즈)
DETECTION_RANGE = 1500  # mm: 라이다 최대 신뢰 거리

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 로봇 & 속도 파라미터
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

ROBOT_HALF_WIDTH = 110   # mm: 라이다 중심 ~ 좌우 끝

FORWARD_SPEED    = 0.35
MIN_SPEED        = 0.07
MAX_W            = 1.5
W_MIN_DANGER     = 0.5   # rad/s: 위험 시 최소 회전
W_SMOOTH         = 0.6

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 계층형 바운딩 박스 정의 (6개 레이어)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 각 레이어: 거리 범위, horiz 임계, w_gain, 기본 가중치, 동적 가중치 여부, v 영향 여부

LAYERS = [
    # L1: 가장 가까움, 동적 가중치, 측면까지 넓게 봄
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':200,
     'w_gain':1.2, 'weight_base':0.4, 'weight_dynamic':True,  'affects_v':True},
    # L2: 가까움, 동적 가중치
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':170,
     'w_gain':1.0, 'weight_base':0.4, 'weight_dynamic':True,  'affects_v':True},
    # L3: 중간
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':140,
     'w_gain':0.8, 'weight_base':0.2, 'weight_dynamic':False, 'affects_v':True},
    # L4: 중간-원거리
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':140,
     'w_gain':0.6, 'weight_base':0.1, 'weight_dynamic':False, 'affects_v':True},
    # L5: 원거리, 미세 보정만 (v 영향 없음)
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':120,
     'w_gain':0.4, 'weight_base':0.05,'weight_dynamic':False, 'affects_v':False},
    # L6: 최원거리, 미세 보정만
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':100,
     'w_gain':0.3, 'weight_base':0.02,'weight_dynamic':False, 'affects_v':False},
]

LAYER_PERCENTILE = 5    # %: 하위 N% dist 평균으로 레이어 대표점 계산

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP zone (계층형과 완전 별도)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP rectangle: 전방 100~150mm 사이, horiz < 110mm (220mm 폭)

STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 180   # 50mm → 80mm 구간으로 확장 (히스테리시스 효과)
STOP_HORIZ_TH = 110

# STOP 탈출: ±135° 스캔, ROBOT_HALF_WIDTH*2 + 양쪽 20mm 마진
STOP_ESCAPE_SCAN_HALF = 135
STOP_ESCAPE_MIN_GAP   = ROBOT_HALF_WIDTH * 2 + 40   # 260mm
STOP_SECTOR_SIZE      = 10                          # deg: 갭 검색 sector 크기
STOP_MAX_CYCLES       = 8                           # 연속 STOP 사이클 상한 (초과 시 강제 탈출)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 방향 점수제 (gap + layer 통합)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCORE_ALPHA       = 1.0    # gap_width 계수
SCORE_BETA        = 200    # layer push 계수
HEADING_WEIGHT_MM = 5.0    # 헤딩 1° = 여유 5mm

MIN_PASSAGE_WIDTH = 240    # 갭이 이보다 좁으면 차단으로 판단
DEPTH_JUMP_THRES  = 120    # mm: 이상이면 다른 물체로 인식

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 스캔 범위 & 통신
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCAN_WIDE_HALF = 135   # 메인에서 받는 스캔 범위 (STOP escape용)
SEND_INTERVAL  = 0.1

# ── 디버그 토글 ──────────────────────────────────────────────────────────────
DEBUG_LAYERS = True    # 각 레이어 처리 결과
DEBUG_STOP   = True    # STOP zone 감지 & 탈출
DEBUG_DIR    = True    # 점수 계산 & 방향 결정
DEBUG_FINAL  = True    # 최종 v, w

# ── 전역 상태 ────────────────────────────────────────────────────────────────
arduino_heading_deg   = 0.0
avoidance_w_sign      = 0.0   # 방향 메모리 (옵션 A: 한 번 정하면 유지)
no_active_count       = 0
prev_w                = 0.0
stop_cycle_count      = 0     # 연속 STOP 사이클 카운터


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return angle - 360 if angle > 180 else angle

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
    global arduino_heading_deg
    while arduino.in_waiting > 0:
        try:
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'): arduino_heading_deg = float(line[2:])
        except Exception: pass


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP zone 감지 & 탈출
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def detect_stop_zone(scan_points):
    """STOP rectangle (fwd 100~150mm, horiz<110mm) 안에 장애물이 있는가?"""
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_front_90(angle_norm): continue
        horiz, fwd = decompose(angle_norm, dist)
        if STOP_FWD_MIN <= fwd <= STOP_FWD_MAX and horiz < STOP_HORIZ_TH:
            return True
    return False


def find_stop_escape_direction(scan_points):
    """
    ±135° 범위를 STOP_SECTOR_SIZE(10°) sector로 나누어
    각 sector의 평균 거리를 계산, 가장 빈 방향(angle deg) 반환.

    Returns:
      (target_angle_deg, gap_distance_mm)
    """
    sectors = {}  # sector_center → list of distances

    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_wide_scan(angle_norm): continue
        center = round(angle_norm / STOP_SECTOR_SIZE) * STOP_SECTOR_SIZE
        sectors.setdefault(center, []).append(dist)

    if not sectors:
        return 0.0, 0.0

    sector_avg = {c: sum(d_list) / len(d_list) for c, d_list in sectors.items()}

    # STOP_ESCAPE_MIN_GAP 이상 통과 가능한 섹터만 후보로 사용
    valid = {}
    for c, avg_dist in sector_avg.items():
        gap_l = get_gap_width(scan_points, c, avg_dist, is_left=True)
        gap_r = get_gap_width(scan_points, c, avg_dist, is_left=False)
        if gap_l + gap_r >= STOP_ESCAPE_MIN_GAP:
            valid[c] = avg_dist

    candidates = valid if valid else sector_avg  # 유효 갭 없으면 fallback

    # 전방 선호 보정: 전방(0°)에 가까울수록 가산점 (90°에서 factor=0.5, 135°에서 0.15)
    # → 옆/뒤 방향이 거리는 멀어도 전방 방향이 우선되어 불필요한 U턴 방지
    def forward_score(c, dist):
        factor = (1.0 + math.cos(math.radians(c))) / 2.0  # 0°=1.0, 90°=0.5, 180°=0.0
        return dist * factor

    best = max(candidates.keys(), key=lambda c: forward_score(c, candidates[c]))
    return float(best), candidates[best]


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 레이어 처리
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def process_layer(scan_points, layer):
    """
    레이어 내 포인트들을 모아 분석.

    반환:
      None: 레이어 활성화 안 됨 (포인트 없음)
      dict: 분석 결과
        - weight: 이번 사이클 가중치 (동적 또는 고정)
        - urgency: w_gain × horiz_error / horiz_th
        - v_proposal: v 제안 (affects_v=True일 때만, 그 외 None)
        - rep_angle/horiz/fwd: 하위 5% 평균 대표점
        - push_left/push_right: 방향 점수용 (포인트 angle 부호로 분리)
    """
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

    # 하위 5% dist 포인트 → 대표점
    n_take = max(1, int(len(pts) * LAYER_PERCENTILE / 100))
    rep = sorted(pts, key=lambda p: p['dist'])[:n_take]

    rep_angle = sum(p['angle'] for p in rep) / len(rep)
    rep_horiz = sum(p['horiz'] for p in rep) / len(rep)
    rep_fwd   = sum(p['fwd']   for p in rep) / len(rep)
    rep_h_err = layer['horiz_th'] - rep_horiz

    # 가중치
    if layer['weight_dynamic']:
        weight = max(layer['weight_base'],
                     min(1.0, rep_h_err / layer['horiz_th']))
    else:
        weight = layer['weight_base']

    # urgency (w 크기 기여)
    urgency = layer['w_gain'] * rep_h_err / layer['horiz_th']

    # v_proposal: 선형 보간 (near edge = MIN_SPEED, far edge = FORWARD_SPEED)
    if layer['affects_v']:
        progress = (rep_fwd - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min'])
        progress = max(0.0, min(1.0, progress))
        v_proposal = MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * progress
    else:
        v_proposal = None

    # push split: 하위 5% 포인트들을 좌/우로 나누어 horiz_error 합산
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
    """ref 기준 좌/우 첫 depth jump까지의 통과 가능 너비."""
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
# 계층형 v/w 산출 (메인 로직)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_layered(scan_points, heading_deg):
    """
    1. 6개 레이어 병렬 처리
    2. gap 너비 계산 (가장 가까운 레이어의 대표점 기준)
    3. 좌우 점수 통합 → 방향 결정 (avoidance_w_sign 적용)
    4. v: affects_v 레이어 v_proposal의 가중 평균
    5. w: 모든 활성 레이어 urgency의 가중 합 × direction
    """
    global avoidance_w_sign, no_active_count

    # 1. 레이어 처리
    layer_results = []
    for layer in LAYERS:
        r = process_layer(scan_points, layer)
        if r is not None:
            layer_results.append(r)

    if DEBUG_LAYERS:
        for r in layer_results:
            v_str = f" v={r['v_proposal']:.2f}" if r['v_proposal'] is not None else ""
            print(f"  [{r['name']}] n={r['n_points']:3d} "
                  f"rep:h={r['rep_horiz']:.0f} a={r['rep_angle']:+.1f}° "
                  f"f={r['rep_fwd']:.0f}  w={r['weight']:.2f} u={r['urgency']:.2f}{v_str}  "
                  f"pL={r['push_left']:.0f} pR={r['push_right']:.0f}")

    # 활성 레이어 없으면 직진
    NO_ACTIVE_RESET = 3
    if not layer_results:
        no_active_count += 1
        if no_active_count >= NO_ACTIVE_RESET:
            avoidance_w_sign = 0.0
        if DEBUG_FINAL:
            print(f"  [FINAL] no active layers -> v={FORWARD_SPEED:.2f} w=0.00")
        return FORWARD_SPEED, 0.0
    no_active_count = 0

    # 2. gap 너비 계산 (가장 가까운 레이어의 대표점 기준)
    closest = min(layer_results, key=lambda r: r['rep_horiz'])
    ref_angle = closest['rep_angle']
    ref_dist  = math.sqrt(closest['rep_horiz']**2 + closest['rep_fwd']**2)

    gap_L = get_gap_width(scan_points, ref_angle, ref_dist, is_left=True)
    gap_R = get_gap_width(scan_points, ref_angle, ref_dist, is_left=False)

    # 3. 좌우 점수 통합
    sum_pR = sum(r['weight'] * r['push_right'] for r in layer_results)
    sum_pL = sum(r['weight'] * r['push_left']  for r in layer_results)

    score_L = (SCORE_ALPHA * gap_L
               + SCORE_BETA  * sum_pR
               + max(0.0, -heading_deg) * HEADING_WEIGHT_MM)
    score_R = (SCORE_ALPHA * gap_R
               + SCORE_BETA  * sum_pL
               + max(0.0,  heading_deg) * HEADING_WEIGHT_MM)

    if DEBUG_DIR:
        print(f"  [GAP] L={gap_L:.0f}mm R={gap_R:.0f}mm  "
              f"(ref={ref_angle:+.1f}°/{ref_dist:.0f}mm from {closest['name']})")
        print(f"  [SCORE] L={score_L:.0f}  R={score_R:.0f}  "
              f"(gap αL={SCORE_ALPHA*gap_L:.0f}/αR={SCORE_ALPHA*gap_R:.0f}  "
              f"push βL={SCORE_BETA*sum_pL:.0f}/βR={SCORE_BETA*sum_pR:.0f})")

    # 4. 방향 결정 (avoidance_w_sign 옵션 A: 한 번 결정 후 고수)
    if avoidance_w_sign == 0.0:
        avoidance_w_sign = 1.0 if score_L >= score_R else -1.0
        if DEBUG_DIR:
            print(f"  [DIR_LOCK] {'LEFT' if avoidance_w_sign > 0 else 'RIGHT'}")
    direction = avoidance_w_sign

    # 5. v 계산: affects_v 레이어 v_proposal의 가중 평균
    v_layers = [r for r in layer_results if r['v_proposal'] is not None]
    if v_layers:
        total_w = sum(r['weight'] for r in v_layers)
        v = sum(r['weight'] * r['v_proposal'] for r in v_layers) / total_w
    else:
        v = FORWARD_SPEED   # L5/L6만 활성 → 최대 속도

    # 6. w 크기: 모든 활성 레이어 urgency 가중 합
    total_w_all = sum(r['weight'] for r in layer_results)
    w_mag = sum(r['weight'] * r['urgency'] for r in layer_results) / total_w_all
    w_mag = max(min(w_mag, MAX_W), W_MIN_DANGER)

    w = direction * w_mag

    if DEBUG_FINAL:
        print(f"  [FINAL] v={v:.2f} w={w:+.2f} dir={'L' if direction > 0 else 'R'}")

    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 진입점 (STOP 우선)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_command(scan_points, heading_deg):
    """STOP zone 우선 검사 → 활성 시 STOP escape, 아니면 계층형 처리."""
    global avoidance_w_sign, stop_cycle_count

    if detect_stop_zone(scan_points) and stop_cycle_count < STOP_MAX_CYCLES:
        target, gap_dist = find_stop_escape_direction(scan_points)

        # 피봇턴 방향: target angle 부호의 반대 (우측 갭이면 우회전 = w<0)
        if abs(target) < 5:
            pivot_w = -MAX_W  # 정면이 가장 빈 경우 default 우회전
        else:
            pivot_w = -math.copysign(MAX_W, target)

        # 피봇 방향을 메모리에 유지 (리셋 X → 탈출 후에도 같은 방향 고수)
        avoidance_w_sign = math.copysign(1.0, pivot_w)
        stop_cycle_count += 1

        if DEBUG_STOP:
            print(f"  [STOP] zone detected (cycle {stop_cycle_count}/{STOP_MAX_CYCLES}) "
                  f"-> escape target={target:+.0f}° "
                  f"(gap_dist={gap_dist:.0f}mm)  pivot w={pivot_w:+.2f}")

        return 0.0, pivot_w

    if stop_cycle_count >= STOP_MAX_CYCLES and DEBUG_STOP:
        print(f"  [STOP] max cycles reached ({STOP_MAX_CYCLES}) -> force layered mode")
    stop_cycle_count = 0
    return find_vw_layered(scan_points, heading_deg)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    global prev_w
    print("=== RPLIDAR Obstacle Avoidance (Layered Bounding Box) ===")
    print(f"  Layers      : 6 layers (60~780mm), bottom {LAYER_PERCENTILE}% per layer")
    print(f"  L1-L2       : dynamic weight max(base, h_err/h_th), affects v")
    print(f"  L3-L4       : fixed weight 0.2/0.1, affects v")
    print(f"  L5-L6       : fixed weight 0.05/0.02, no v effect")
    print(f"  STOP zone   : fwd {STOP_FWD_MIN}-{STOP_FWD_MAX}mm, horiz<{STOP_HORIZ_TH}mm")
    print(f"  STOP escape : +/-{STOP_ESCAPE_SCAN_HALF}deg scan, "
          f"min_gap={STOP_ESCAPE_MIN_GAP}mm, sector={STOP_SECTOR_SIZE}deg")
    print(f"  Scoring     : alpha={SCORE_ALPHA} beta={SCORE_BETA}")
    print(f"  Direction   : avoidance_w_sign (option A: lock first, reset after 3 clear)")
    print(f"  Debug flags : LAYERS={DEBUG_LAYERS} STOP={DEBUG_STOP} "
          f"DIR={DEBUG_DIR} FINAL={DEBUG_FINAL}")
    print("=" * 70)

    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)

    lidar.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    lidar.write(bytes([0xA5, 0x20]))
    lidar.read(7)

    scan_points  = []
    last_send    = time.time()
    last_cmd_str = ""

    try:
        while True:
            read_arduino(arduino)
            raw = lidar.read(5)
            result = parse_packet(raw)
            if result is None: continue

            angle_raw, distance = result
            s_flag = raw[0] & 0x01

            if s_flag == 1 and scan_points:
                # ±135°까지 통과 (STOP escape에서 사용)
                wide_points = [
                    (a, d) for a, d in scan_points
                    if is_in_wide_scan(a) and d > 0
                ]
                now = time.time()
                if now - last_send >= SEND_INTERVAL:
                    v, w = find_vw_command(wide_points, arduino_heading_deg)
                    w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
                    prev_w = w
                    cmd = f"{v:.2f} {w:.2f}\n"
                    arduino.write(cmd.encode())
                    if cmd != last_cmd_str:
                        print(f"[SEND] v={v:.2f}  w={w:+.2f}  "
                              f"heading={arduino_heading_deg:.1f}deg")
                        last_cmd_str = cmd
                    last_send = now
                scan_points = []

            scan_points.append((
                normalize_angle(angle_raw),
                distance + LIDAR_OFFSET if distance > 0 else 0
            ))

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        lidar.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        lidar.close()
        arduino.write(b"0.00 0.00\n")
        arduino.close()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()

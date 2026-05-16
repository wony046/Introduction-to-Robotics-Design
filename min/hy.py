"""
RPLIDAR C1 장애물 회피 - 하이브리드 (코드2 알고리즘 + 코드1 안전 인프라)

[설계 결정]
  STOP zone   : 코드2식 - 정지 후 FGM 피봇 (정확한 갭 정렬)
  방향 결정    : 코드2식 - 매 사이클 점수로 재결정 (commit 없음)
  레이어 감지  : 코드2식 - 6단계 풀버전 (선제적 회피)

[코드1에서 흡수한 안전 인프라]
  1. try/except/finally 메인루프 - 어떤 예외든 Arduino에 정지 신호
  2. Keepalive 0.3s - 패킷 손실 / 회전 중 명령 유지
  3. 라이다 재동기화 - 파싱 실패 100회 시 자동 복구
  4. find_lidar_sync - 시작 시 안전한 동기화
  5. 라이다 watchdog - 0.5s 스캔 없으면 비상정지
  6. decompose_signed 좌표 통일 - y > 0 = 좌측
  7. Print 스로틀링 - 상태 2Hz, 이벤트 즉시
  8. start_lidar 안전 처리 - try/except + descriptor 출력
  9. 진단 로그 - 패킷 0개 감지

[PDF 환경 맞춤 조정]
  - 경기장 폭 1.1m → 벽 무시 영역(WALL_IGNORE_HORIZ=450) 추가
  - 60초 제한 → STOP_MAX_CYCLES 축소 (16→10)로 정지시간 단축
  - 충돌 1회=1점 → SAFETY_MARGIN 강화
  - 5개 랜덤 배치 → 매사이클 점수 재결정 유지
  - W_SMOOTH=0.5 (코드1 0.75 vs 코드2 0.35의 중간) - 반응성 + 진동억제 균형

[제거한 것]
  - 코드2의 JSON 파일 저장 (디스크 I/O 지연 위험)
  - 코드2의 readline() blocking 호출 → 코드1의 non-blocking 버퍼링

포트: 라이다 /dev/ttyUSB0 / 아두이노 /dev/ttyAMA3
"""

import serial
import time
import math
import traceback

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 포트 & 라이다 설정
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

LIDAR_OFFSET    = 20    # mm
LIDAR_MIN_VALID = 100   # mm: 노이즈 무시
DETECTION_RANGE = 1500  # mm: 최대 신뢰 거리

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 로봇 & 속도
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

ROBOT_HALF_WIDTH = 110   # mm

FORWARD_SPEED    = 0.45  # 반응 여유 확보를 위해 소폭 감속
MIN_SPEED        = 0.07
MAX_W            = 1.5
W_MIN_DANGER     = 0.5
W_SMOOTH         = 0.5   # 코드1(0.75) vs 코드2(0.35) 중간 - 반응성 + 진동 억제

# 벽 무시 (1.1m 경기장 특화) - 코드1에서 가져옴
WALL_IGNORE_HORIZ = 500  # mm: 벽(550mm)에 더 가깝게 설정해 근처 장애물 누락 방지

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 6 레이어 정의 (코드2 그대로)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

LAYERS = [
    {'name':'L1', 'fwd_min':60,  'fwd_max':180, 'horiz_th':200,
     'w_gain':2.5, 'weight_base':0.4, 'weight_dynamic':True,  'affects_v':True},
    {'name':'L2', 'fwd_min':180, 'fwd_max':300, 'horiz_th':170,
     'w_gain':2.0, 'weight_base':0.4, 'weight_dynamic':True,  'affects_v':True},
    {'name':'L3', 'fwd_min':300, 'fwd_max':420, 'horiz_th':140,
     'w_gain':1.5, 'weight_base':0.2, 'weight_dynamic':False, 'affects_v':True},
    {'name':'L4', 'fwd_min':420, 'fwd_max':540, 'horiz_th':140,
     'w_gain':1.0, 'weight_base':0.1, 'weight_dynamic':False, 'affects_v':True},
    {'name':'L5', 'fwd_min':540, 'fwd_max':660, 'horiz_th':120,
     'w_gain':0.4, 'weight_base':0.05,'weight_dynamic':False, 'affects_v':False},
    {'name':'L6', 'fwd_min':660, 'fwd_max':780, 'horiz_th':100,
     'w_gain':0.3, 'weight_base':0.02,'weight_dynamic':False, 'affects_v':False},
]

LAYER_PERCENTILE = 5

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP zone & FGM (코드2 + 미세조정)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

STOP_FWD_MIN  = 100
STOP_FWD_MAX  = 180
STOP_HORIZ_TH = 150  # 로봇 반폭(110mm)보다 40mm 여유 → 비상정지 구역 확대

STOP_ESCAPE_SCAN_HALF = 90
STOP_ESCAPE_MIN_GAP   = ROBOT_HALF_WIDTH * 2 + 40   # 260mm
STOP_MAX_CYCLES       = 10   # 코드2(16)→10: 60초 제한 고려하여 정지시간 단축

FGM_MIN_ANG_DEG      = 5
FGM_MIN_DEPTH_MM     = 200
FGM_MAX_RANGE_MM     = 500
HEADING_CONVERGE_DEG = 15

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 방향 점수제
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCORE_ALPHA       = 5.0
SCORE_BETA        = 20
HEADING_WEIGHT_MM = 5.0

MIN_PASSAGE_WIDTH = 240
DEPTH_JUMP_THRES  = 120

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 스캔 & 통신
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCAN_WIDE_HALF = 135
SEND_INTERVAL  = 0.1

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 안전 인프라 파라미터 (코드1)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

KEEPALIVE_INTERVAL     = 0.30
PRINT_INTERVAL         = 0.5
RESYNC_THRESHOLD       = 100
DIAG_INTERVAL          = 2.0
LIDAR_SYNC_TIMEOUT     = 3.0
LIDAR_WATCHDOG_TIMEOUT = 0.5

# ── 디버그 토글 ──────────────────────────────────────────────────────────────
DEBUG_LAYERS = False   # 평가 시 노이즈 줄이려면 False
DEBUG_STOP   = True
DEBUG_DIR    = False
DEBUG_FINAL  = True

# ── 전역 상태 ────────────────────────────────────────────────────────────────
arduino_heading_deg        = 0.0
arduino_buf                = ""   # 코드1 비차단 버퍼
prev_w                     = 0.0
prev_v                     = 0.0
stop_cycle_count           = 0
stop_pivot_w               = 0.0
stop_locked_target         = 0.0
stop_locked_gap            = 0.0
stop_locked_global_heading = 0.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸리티
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def normalize_angle(angle):
    return angle - 360 if angle > 180 else angle

def is_in_front_90(a):
    return -90 <= a <= 90

def is_in_wide_scan(a):
    return -SCAN_WIDE_HALF <= a <= SCAN_WIDE_HALF


def decompose_signed(angle_norm_deg, distance_mm):
    """코드1의 부호 있는 분해.
    y > 0 = 좌측 (장애물 왼쪽 → 우회전 -w 필요)
    y < 0 = 우측 (장애물 오른쪽 → 좌회전 +w 필요)
    """
    rad = math.radians(angle_norm_deg)
    x =  distance_mm * math.cos(rad)
    y = -distance_mm * math.sin(rad)
    return x, y


def decompose(angle_deg, dist):
    """코드2 호환: (horiz, fwd) 반환."""
    x, y = decompose_signed(angle_deg, dist)
    return abs(y), x


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
    """코드1의 비차단 버퍼링 - readline() 블로킹 회피."""
    global arduino_heading_deg, arduino_buf
    if arduino.in_waiting <= 0:
        return
    try:
        arduino_buf += arduino.read(arduino.in_waiting).decode('utf-8', errors='ignore')
        while '\n' in arduino_buf:
            line, arduino_buf = arduino_buf.split('\n', 1)
            line = line.strip()
            if line.startswith('H:'):
                arduino_heading_deg = float(line[2:])
    except Exception:
        pass


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 라이다 안전한 시작 & 재동기화 (코드1)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_lidar_sync(lidar, verbose=True):
    if verbose:
        print("[라이다] 동기화 시도...", flush=True)
    deadline = time.time() + LIDAR_SYNC_TIMEOUT
    while time.time() < deadline:
        b = lidar.read(1)
        if len(b) == 0: continue
        if (b[0] & 0x01) == ((b[0] >> 1) & 0x01): continue
        b2 = lidar.read(1)
        if len(b2) == 0: continue
        if (b2[0] & 0x01) != 1: continue
        rest = lidar.read(3)
        if len(rest) != 3: continue
        result = parse_packet(b + b2 + rest)
        if result is None: continue
        angle, distance = result
        if 0 <= angle <= 360 and 0 <= distance <= 10000:
            if verbose:
                print(f"[라이다] 동기화 OK (각도={angle:.1f}°, 거리={distance:.0f}mm)",
                      flush=True)
            return True
    if verbose:
        print("[라이다] ✗ 동기화 실패", flush=True)
    return False


def start_lidar(lidar):
    print("[라이다] 시작 중...", flush=True)
    try:
        lidar.write(bytes([0xA5, 0x25]))   # 먼저 STOP
        time.sleep(0.1)
        try:
            lidar.dtr = False
        except AttributeError:
            pass
        time.sleep(0.5)
        lidar.reset_input_buffer()
        lidar.write(bytes([0xA5, 0x20]))   # SCAN
        time.sleep(0.5)
        descriptor = lidar.read(7)
        print(f"[라이다] descriptor: {descriptor.hex()}", flush=True)
        return True
    except serial.SerialException as e:
        print(f"[라이다] ✗ 시작 실패: {e}", flush=True)
        return False
    except Exception as e:
        print(f"[라이다] ✗ 예외: {e}", flush=True)
        return False


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# STOP zone 감지 & FGM 탈출 (코드2)
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
         if LIDAR_MIN_VALID < d < FGM_MAX_RANGE_MM
         and abs(a) <= SCAN_WIDE_HALF],
        key=lambda p: p[0]
    )
    if len(pts) < 2:
        return []

    gaps = []
    for i in range(len(pts) - 1):
        a1, d1 = pts[i]
        a2, d2 = pts[i + 1]
        ang_diff = a2 - a1

        is_depth_jump   = abs(d2 - d1) > DEPTH_JUMP_THRES
        is_angular_hole = ang_diff >= FGM_MIN_ANG_DEG

        if not (is_depth_jump or is_angular_hole):
            continue

        width = cosine_dist(d1, d2, ang_diff)

        x1 = d1 * math.sin(math.radians(a1))
        y1 = d1 * math.cos(math.radians(a1))
        x2 = d2 * math.sin(math.radians(a2))
        y2 = d2 * math.cos(math.radians(a2))
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
        return min(passable, key=lambda g: abs(g['center_angle'] - prefer_angle))
    return max(gaps, key=lambda g: g['width']) if gaps else None


def find_stop_escape_direction(scan_points):
    gaps   = find_all_gaps(scan_points)
    chosen = choose_escape_gap(gaps, prefer_angle=0.0)
    if chosen is None:
        return 0.0, 0.0
    return float(chosen['center_angle']), float(chosen['width'])


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 레이어 처리 (코드2 + 벽 무시 추가)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def process_layer(scan_points, layer):
    pts = []
    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > DETECTION_RANGE: continue
        if not is_in_front_90(angle_norm): continue
        horiz, fwd = decompose(angle_norm, dist)

        # 벽 무시 (1.1m 경기장 특화)
        if horiz > WALL_IGNORE_HORIZ:
            continue

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
        weight = max(layer['weight_base'],
                     min(1.0, rep_h_err / layer['horiz_th']))
    else:
        weight = layer['weight_base']

    urgency = layer['w_gain'] * rep_h_err / layer['horiz_th']

    if layer['affects_v']:
        progress = (rep_fwd - layer['fwd_min']) / (layer['fwd_max'] - layer['fwd_min'])
        progress = max(0.0, min(1.0, progress))
        v_proposal = MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * progress
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
# Gap 너비 (코사인 법칙)
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
# 계층형 v/w 산출 (코드2 메인 로직)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_layered(scan_points, heading_deg):
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

    if not layer_results:
        return FORWARD_SPEED, 0.0

    closest = min(layer_results, key=lambda r: r['rep_horiz'])
    ref_angle = closest['rep_angle']
    ref_dist  = math.sqrt(closest['rep_horiz']**2 + closest['rep_fwd']**2)

    gap_L = get_gap_width(scan_points, ref_angle, ref_dist, is_left=True)
    gap_R = get_gap_width(scan_points, ref_angle, ref_dist, is_left=False)

    sum_pR = sum(r['weight'] * r['push_right'] for r in layer_results)
    sum_pL = sum(r['weight'] * r['push_left']  for r in layer_results)

    score_L = (SCORE_ALPHA * gap_L
               + SCORE_BETA  * sum_pR
               + max(0.0, -heading_deg) * HEADING_WEIGHT_MM)
    score_R = (SCORE_ALPHA * gap_R
               + SCORE_BETA  * sum_pL
               + max(0.0,  heading_deg) * HEADING_WEIGHT_MM)

    if DEBUG_DIR:
        print(f"  [GAP] L={gap_L:.0f} R={gap_R:.0f}  ref={ref_angle:+.1f}°/{ref_dist:.0f}")
        print(f"  [SCORE] L={score_L:.0f}  R={score_R:.0f}")

    direction = 1.0 if score_L >= score_R else -1.0

    v_layers = [r for r in layer_results if r['v_proposal'] is not None]
    if v_layers:
        total_w = sum(r['weight'] for r in v_layers)
        v = sum(r['weight'] * r['v_proposal'] for r in v_layers) / total_w
    else:
        v = FORWARD_SPEED

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
    global stop_cycle_count, stop_pivot_w, stop_locked_target, stop_locked_gap, \
           stop_locked_global_heading

    # 피봇 중 목표 헤딩 도달 시 layered로 복귀
    if stop_cycle_count > 0:
        heading_err = abs(((heading_deg - stop_locked_global_heading) + 180) % 360 - 180)
        if heading_err < HEADING_CONVERGE_DEG:
            if DEBUG_STOP:
                print(f"  [STOP] heading converged "
                      f"(target={stop_locked_global_heading:.1f}° err={heading_err:.1f}°) "
                      f"-> layered")
            stop_cycle_count = 0
            stop_pivot_w     = 0.0
            return find_vw_layered(scan_points, heading_deg)

    if detect_stop_zone(scan_points) and stop_cycle_count < STOP_MAX_CYCLES:
        if stop_cycle_count == 0:
            target, gap_width = find_stop_escape_direction(scan_points)
            stop_locked_target = target
            stop_locked_gap    = gap_width
            stop_locked_global_heading = ((heading_deg - target) + 180) % 360 - 180
            if abs(target) < 5:
                # 양쪽 공간을 직접 비교해 더 열린 쪽으로 피봇
                left_dists  = [d for a, d in scan_points
                               if -80 <= a < -10 and LIDAR_MIN_VALID < d < DETECTION_RANGE]
                right_dists = [d for a, d in scan_points
                               if  10 < a <= 80  and LIDAR_MIN_VALID < d < DETECTION_RANGE]
                left_mean  = sum(left_dists)  / len(left_dists)  if left_dists  else 0
                right_mean = sum(right_dists) / len(right_dists) if right_dists else 0
                stop_pivot_w = -MAX_W if right_mean >= left_mean else MAX_W
            else:
                stop_pivot_w = -math.copysign(MAX_W, target)
            if DEBUG_STOP:
                print(f"  [STOP] entry target={target:+.1f}° gap={gap_width:.0f}mm "
                      f"global_target_h={stop_locked_global_heading:.1f}°")
        else:
            target    = stop_locked_target
            gap_width = stop_locked_gap

        pivot_w = stop_pivot_w
        stop_cycle_count += 1

        if DEBUG_STOP:
            print(f"  [STOP] zone (cycle {stop_cycle_count}/{STOP_MAX_CYCLES}) "
                  f"target={target:+.0f}° pivot w={pivot_w:+.2f}")

        return 0.0, pivot_w

    if stop_cycle_count > 0:
        if stop_cycle_count >= STOP_MAX_CYCLES and DEBUG_STOP:
            print(f"  [STOP] max cycles ({STOP_MAX_CYCLES}) -> force layered")
        stop_cycle_count = 0
        stop_pivot_w     = 0.0

    return find_vw_layered(scan_points, heading_deg)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 루프 (코드1 안전 인프라)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    global prev_w, prev_v

    print("=" * 70)
    print("RPLIDAR 장애물 회피 - 하이브리드")
    print("=" * 70)
    print(f"  알고리즘    : 6-layer + FGM STOP escape + 매사이클 점수")
    print(f"  안전 인프라 : try/finally, Keepalive {KEEPALIVE_INTERVAL}s, "
          f"재동기화 {RESYNC_THRESHOLD}회")
    print(f"  레이어      : 6단계 (60~780mm), 하위 {LAYER_PERCENTILE}% 평균")
    print(f"  STOP zone   : fwd {STOP_FWD_MIN}-{STOP_FWD_MAX}mm, "
          f"horiz<{STOP_HORIZ_TH}mm, max_cycles={STOP_MAX_CYCLES}")
    print(f"  벽 무시     : 수평 {WALL_IGNORE_HORIZ}mm 초과")
    print(f"  전진속도    : {FORWARD_SPEED}, W_SMOOTH={W_SMOOTH}")
    print("=" * 70)

    lidar   = None
    arduino = None
    try:
        lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
        time.sleep(0.5)
    except Exception as e:
        print(f"[치명] 라이다 포트 열기 실패: {e}")
        return

    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=0.1)
        time.sleep(0.5)
    except Exception as e:
        print(f"[치명] 아두이노 포트 열기 실패: {e}")
        if lidar: lidar.close()
        return

    arduino.write(b"0.00 0.00\n")
    time.sleep(0.3)

    if not start_lidar(lidar):
        arduino.write(b"0.00 0.00\n")
        lidar.close()
        arduino.close()
        return

    if not find_lidar_sync(lidar):
        arduino.write(b"0.00 0.00\n")
        try: lidar.write(bytes([0xA5, 0x25]))
        except Exception: pass
        lidar.close()
        arduino.close()
        return

    print("\n주행 시작!\n", flush=True)

    scan_points     = []
    last_send       = time.time()
    last_scan_time  = time.time()
    last_cmd_time   = time.time()
    last_print_time = 0.0
    last_diag_time  = time.time()
    last_cmd_str    = ""
    invalid_count   = 0
    packet_count    = 0

    try:
        while True:
            read_arduino(arduino)
            raw = lidar.read(5)
            now = time.time()

            # 라이다 watchdog
            if now - last_scan_time > LIDAR_WATCHDOG_TIMEOUT:
                arduino.write(b"0.00 0.00\n")
                last_cmd_time = now
                print(f"[경고] 라이다 스캔 없음 ({LIDAR_WATCHDOG_TIMEOUT}s) → 비상정지",
                      flush=True)
                last_scan_time = now

            if len(raw) < 5:
                invalid_count += 1
            else:
                result = parse_packet(raw)
                if result is None:
                    invalid_count += 1
                    if invalid_count >= RESYNC_THRESHOLD:
                        print(f"[라이다] 파싱실패 {invalid_count}회 → 재동기화",
                              flush=True)
                        lidar.reset_input_buffer()
                        if find_lidar_sync(lidar, verbose=False):
                            print("[라이다] 재동기화 성공", flush=True)
                        invalid_count = 0
                        scan_points = []
                else:
                    invalid_count = 0
                    packet_count += 1
                    angle_raw, distance = result
                    s_flag = raw[0] & 0x01

                    if s_flag == 1 and scan_points:
                        last_scan_time = time.time()
                        wide_points = [
                            (a, d) for a, d in scan_points
                            if is_in_wide_scan(a) and d > 0
                        ]
                        now = time.time()
                        if now - last_send >= SEND_INTERVAL:
                            v, w = find_vw_command(wide_points, arduino_heading_deg)

                            # W_SMOOTH (방향 전환 시 prev_w 리셋으로 즉시 반응)
                            if w == 0.0:
                                prev_w = 0.0
                            else:
                                if w * prev_w < 0:
                                    prev_w = 0.0   # 방향 전환 즉시 반영
                                w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
                                prev_w = w
                            prev_v = v

                            cmd = f"{v:.2f} {w:.2f}\n"
                            arduino.write(cmd.encode())
                            last_cmd_time = now

                            if cmd != last_cmd_str:
                                if now - last_print_time >= PRINT_INTERVAL:
                                    print(f"[전송] v={v:.2f}  w={w:+.2f}  "
                                          f"헤딩={arduino_heading_deg:.1f}°",
                                          flush=True)
                                    last_print_time = now
                                last_cmd_str = cmd
                            last_send = now
                        scan_points = []

                    scan_points.append((
                        normalize_angle(angle_raw),
                        distance + LIDAR_OFFSET if distance > 0 else 0
                    ))

            # Keepalive - 패킷 손실 시 마지막 명령 재전송
            now = time.time()
            if now - last_cmd_time > KEEPALIVE_INTERVAL:
                cmd = f"{prev_v:.2f} {prev_w:.2f}\n"
                arduino.write(cmd.encode())
                last_cmd_time = now

            # 진단
            if now - last_diag_time >= DIAG_INTERVAL:
                if packet_count == 0:
                    print("[진단] 패킷 0 — 라이다 연결 의심", flush=True)
                packet_count = 0
                last_diag_time = now

    except KeyboardInterrupt:
        print("\n[종료] Ctrl-C 수신", flush=True)
    except serial.SerialException as e:
        print(f"\n[치명] 시리얼 예외: {e}", flush=True)
        traceback.print_exc()
    except Exception as e:
        print(f"\n[치명] 예상치 못한 예외: {e}", flush=True)
        traceback.print_exc()
    finally:
        print("[정리] 정지 신호 전송 및 포트 닫기", flush=True)
        if arduino is not None:
            try:
                arduino.write(b"0.00 0.00\n")
                time.sleep(0.1)
                arduino.write(b"0.00 0.00\n")
            except Exception: pass
            try: arduino.close()
            except Exception: pass
        if lidar is not None:
            try:
                lidar.write(bytes([0xA5, 0x25]))
                time.sleep(0.1)
            except Exception: pass
            try: lidar.close()
            except Exception: pass
        print("[종료] 완료", flush=True)


if __name__ == "__main__":
    main()

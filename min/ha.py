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
  - 경기장 폭 1.1m → 벽 무시 영역(WALL_IGNORE_HORIZ=420) 추가
  - 60초 제한 → STOP_MAX_CYCLES 축소 (16→10)로 정지시간 단축
  - 충돌 1회=1점 → SAFETY_MARGIN 강화
  - 5개 랜덤 배치 → 매사이클 점수 재결정 유지
  - W_SMOOTH=0.75 (코드1 값 유지) - 반응성 우선, 장애물 긁힘 방지

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

FORWARD_SPEED    = 0.45
MIN_SPEED        = 0.15
MAX_W            = 1.5
W_MIN_DANGER     = 0.7  # 장애물 감지 시 최소 회전력 강화
W_SMOOTH         = 0.75  # 반응성 우선 — 장애물 긁힘 방지

# 벽 무시 (1.1m 경기장 특화) - 코드1에서 가져옴
WALL_IGNORE_HORIZ = 420  # mm: 벽(550mm) 130mm 안쪽까지만 무시, 벽 근처 장애물 감지

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
STOP_FWD_MAX  = 350  # 비상정지 감지 거리 확대 (고속 접근 대응)
STOP_HORIZ_TH = 150  # 로봇 반폭(110mm)보다 40mm 여유 → 비상정지 구역 확대

STOP_ESCAPE_SCAN_HALF = 90
STOP_ESCAPE_MIN_GAP   = ROBOT_HALF_WIDTH * 2 + 100  # 320mm — 여유폭 확대
STOP_MAX_CYCLES       = 10   # 코드2(16)→10: 60초 제한 고려하여 정지시간 단축
RECOVERY_MIN_GAP      = ROBOT_HALF_WIDTH * 2 + 80   # 300mm: 360° 복구 시 통과 최소폭

FGM_MIN_ANG_DEG      = 5
FGM_MIN_DEPTH_MM     = 400  # 막다른 틈 방지 — 최소 40cm 열린 공간 필요
FGM_MAX_RANGE_MM     = 500
HEADING_CONVERGE_DEG = 15

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 방향 점수제
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

SCORE_ALPHA       = 5.0
SCORE_BETA        = 20
HEADING_WEIGHT_MM = 25.0  # 직진 바이어스 강화 (기존 5.0은 SCORE_BETA 대비 무의미)

MIN_PASSAGE_WIDTH = 320  # 로봇폭(220mm) + 양쪽 50mm 여유
DEPTH_JUMP_THRES  = 120

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 반발력(포텐셜 필드) 파라미터
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

REP_INFLUENCE_MM = 600   # mm: 반발력 영향 반경 (이 거리서 반발력 0)
REP_GAIN_LAT     = 0.30  # 측면 합력 → w 변환 게인
HEADING_ATTRACT  = 0.05  # IMU 헤딩 오차 보정 게인 (°당)
FWD_ANGLE_HALF   = 40    # deg: 전방 감속 스캔 범위 (±)
REP_BLEND        = 0.40  # 반발력 w를 6레이어 w에 더하는 비율

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
_last_direction            = 0.0  # 방향 히스테리시스용
zone_clear_count           = 0    # STOP 탈출 연속 clear 카운터
recovery_mode              = False
recovery_pivot_w           = 0.0
recovery_locked_heading    = 0.0
recovery_align_count       = 0


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


def find_best_gap_360(full_scan):
    """360° 전체 스캔에서 통과 가능한 최적 갭 방향 반환."""
    pts = sorted(
        [(a, d) for a, d in full_scan
         if LIDAR_MIN_VALID < d < FGM_MAX_RANGE_MM],
        key=lambda p: p[0]
    )
    if len(pts) < 2:
        return 0.0, 0.0

    gaps = []
    for i in range(len(pts) - 1):
        a1, d1 = pts[i]
        a2, d2 = pts[i + 1]
        ang_diff = a2 - a1
        if not (abs(d2 - d1) > DEPTH_JUMP_THRES or ang_diff >= FGM_MIN_ANG_DEG):
            continue
        width = cosine_dist(d1, d2, ang_diff)
        cx = (d1 * math.sin(math.radians(a1)) + d2 * math.sin(math.radians(a2))) / 2
        cy = (d1 * math.cos(math.radians(a1)) + d2 * math.cos(math.radians(a2))) / 2
        center_angle = math.degrees(math.atan2(cx, cy))
        gaps.append({'width': width, 'center_angle': center_angle, 'depth': max(d1, d2)})

    # ±180° 경계 랩어라운드 갭 (정후방) 검사
    if len(pts) >= 2:
        a_last, d_last = pts[-1]
        a_first, d_first = pts[0]
        ang_diff_wrap = (a_first + 360) - a_last  # -180°/+180° 경계를 넘는 각도 차
        if abs(d_first - d_last) > DEPTH_JUMP_THRES or ang_diff_wrap >= FGM_MIN_ANG_DEG:
            width = cosine_dist(d_last, d_first, ang_diff_wrap)
            cx = (d_last * math.sin(math.radians(a_last)) + d_first * math.sin(math.radians(a_first))) / 2
            cy = (d_last * math.cos(math.radians(a_last)) + d_first * math.cos(math.radians(a_first))) / 2
            center_angle = math.degrees(math.atan2(cx, cy))
            gaps.append({'width': width, 'center_angle': center_angle, 'depth': max(d_last, d_first)})

    if not gaps:
        return 0.0, 0.0

    passable = [g for g in gaps
                if g['width'] >= RECOVERY_MIN_GAP and g['depth'] >= FGM_MIN_DEPTH_MM]
    chosen = (min(passable, key=lambda g: abs(g['center_angle'])) if passable
              else max(gaps, key=lambda g: g['width']))
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

    # obs_left: 좌측(angle<0) 장애물 누적 오차, obs_right: 우측(angle>0) 장애물 누적 오차
    obs_left  = sum(p['horiz_error'] for p in rep if p['angle'] < 0)
    obs_right = sum(p['horiz_error'] for p in rep if p['angle'] > 0)

    return {
        'name': layer['name'],
        'weight': weight, 'urgency': urgency, 'v_proposal': v_proposal,
        'rep_angle': rep_angle, 'rep_horiz': rep_horiz, 'rep_fwd': rep_fwd,
        'obs_left': obs_left, 'obs_right': obs_right,
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
    global _last_direction
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
                  f"obsL={r['obs_left']:.0f} obsR={r['obs_right']:.0f}")

    if not layer_results:
        _last_direction = 0.0  # 장애물 없으면 히스테리시스 편향 초기화
        return FORWARD_SPEED, 0.0

    closest = min(layer_results, key=lambda r: r['rep_horiz'])
    ref_angle = closest['rep_angle']
    ref_dist  = math.sqrt(closest['rep_horiz']**2 + closest['rep_fwd']**2)

    gap_L = get_gap_width(scan_points, ref_angle, ref_dist, is_left=True)
    gap_R = get_gap_width(scan_points, ref_angle, ref_dist, is_left=False)

    # obs_right(우측 장애물) → 좌측 방향 점수 가산, obs_left(좌측 장애물) → 우측 방향 점수 가산
    sum_obs_R = sum(r['weight'] * r['obs_right'] for r in layer_results)
    sum_obs_L = sum(r['weight'] * r['obs_left']  for r in layer_results)

    score_L = (SCORE_ALPHA * gap_L
               + SCORE_BETA  * sum_obs_R
               + max(0.0, -heading_deg) * HEADING_WEIGHT_MM)
    score_R = (SCORE_ALPHA * gap_R
               + SCORE_BETA  * sum_obs_L
               + max(0.0,  heading_deg) * HEADING_WEIGHT_MM)

    # MIN_PASSAGE_WIDTH 실제 적용 — 통과 불가 방향 점수 제거
    if gap_L < MIN_PASSAGE_WIDTH and gap_R >= MIN_PASSAGE_WIDTH:
        score_L = 0.0
    elif gap_R < MIN_PASSAGE_WIDTH and gap_L >= MIN_PASSAGE_WIDTH:
        score_R = 0.0

    if DEBUG_DIR:
        print(f"  [GAP] L={gap_L:.0f} R={gap_R:.0f}  ref={ref_angle:+.1f}°/{ref_dist:.0f}")
        print(f"  [SCORE] L={score_L:.0f}  R={score_R:.0f}")

    # 방향 히스테리시스 — 40% 이상 우세해야 방향 전환
    raw_dir = 1.0 if score_L >= score_R else -1.0
    if _last_direction != 0.0 and raw_dir != _last_direction:
        lead  = max(score_L, score_R)
        trail = min(score_L, score_R)
        if lead < trail * 1.4:
            raw_dir = _last_direction  # 우세하지 않으면 현재 방향 유지
    _last_direction = raw_dir
    direction = raw_dir

    v_layers = [r for r in layer_results if r['v_proposal'] is not None]
    if v_layers:
        total_w = sum(r['weight'] for r in v_layers)
        v = sum(r['weight'] * r['v_proposal'] for r in v_layers) / total_w
    else:
        v = FORWARD_SPEED

    total_w_all = sum(r['weight'] for r in layer_results)
    w_mag = sum(r['weight'] * r['urgency'] for r in layer_results) / total_w_all
    w_mag = min(w_mag, MAX_W)
    # L5/L6 (540mm+) 만 감지됐을 때는 강제 최소 회전력 불필요 — 부드럽게 유도
    if any(r['name'] in ('L1', 'L2', 'L3', 'L4') for r in layer_results):
        w_mag = max(w_mag, W_MIN_DANGER)
    w = direction * w_mag

    if DEBUG_FINAL:
        print(f"  [FINAL] v={v:.2f} w={w:+.2f} dir={'L' if direction > 0 else 'R'}")

    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 포텐셜 필드 반발력 기반 조향
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_repulsion(scan_points, heading_deg):
    """
    좌우 측면 반발력 합산 → w,  전방 최근접 거리 → v 스케일.

    반발력 공식: rep = 1/dist - 1/REP_INFLUENCE_MM
      · 영향권 경계(REP_INFLUENCE_MM)에서 자연스럽게 0으로 수렴
      · 가까울수록 급격히 증가

    부호 규칙 (decompose_signed 기준):
      y > 0 = 좌측 장애물 → 우회전(-w),  y < 0 = 우측 → 좌회전(+w)
      lat_force -= y * rep  으로 자동 반전
    """
    lat_force = 0.0

    for angle_norm, dist in scan_points:
        if dist < LIDAR_MIN_VALID or dist > REP_INFLUENCE_MM:
            continue
        if not is_in_wide_scan(angle_norm):   # ±135° 이내
            continue

        _, y = decompose_signed(angle_norm, dist)
        if abs(y) > WALL_IGNORE_HORIZ:        # 경기장 벽 제외
            continue

        rep = 1.0 / dist - 1.0 / REP_INFLUENCE_MM
        lat_force -= y * rep

    # IMU 헤딩 보정: 기여 상한 ±0.3 — 대편향 시 장애물 회피 압도 방지
    heading_correction = max(-0.3, min(0.3, -heading_deg * HEADING_ATTRACT))
    w = lat_force * REP_GAIN_LAT + heading_correction
    w = max(-MAX_W, min(MAX_W, w))

    # 전방 감속: ±FWD_ANGLE_HALF 내 최근접 거리로 v 선형 스케일
    fwd_dists = [d for a, d in scan_points
                 if LIDAR_MIN_VALID < d < REP_INFLUENCE_MM
                 and abs(a) <= FWD_ANGLE_HALF]
    if fwd_dists:
        min_fwd   = min(fwd_dists)
        clearance = max(0.0, min_fwd - STOP_FWD_MAX)
        v_scale   = min(1.0, clearance / (REP_INFLUENCE_MM - STOP_FWD_MAX))
        v = MIN_SPEED + (FORWARD_SPEED - MIN_SPEED) * v_scale
    else:
        v = FORWARD_SPEED

    if DEBUG_FINAL:
        print(f"  [REP] lat={lat_force:.3f} w={w:+.2f} v={v:.2f}")

    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 6레이어 + 반발력 병합
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_combined(scan_points, heading_deg):
    """
    v  : 6레이어의 거리 기반 속도 스케일 사용 (L1~L6 정밀 감속)
    w  : 6레이어 w + 반발력 w × REP_BLEND 합산, MAX_W 클램프
    효과: 6레이어가 갭 방향을 결정하고, 반발력이 벽/장애물 회피를 연속 보정
    """
    v,  w_layer = find_vw_layered(scan_points, heading_deg)
    _, w_rep    = find_vw_repulsion(scan_points, heading_deg)

    w = w_layer + w_rep * REP_BLEND
    w = max(-MAX_W, min(MAX_W, w))

    if DEBUG_FINAL:
        print(f"  [COMB] layer_w={w_layer:+.2f} rep_w={w_rep:+.2f} → w={w:+.2f} v={v:.2f}")

    return v, w


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메인 진입점 (STOP 우선)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def find_vw_command(scan_points, heading_deg, full_scan=None):
    global stop_cycle_count, stop_pivot_w, stop_locked_target, stop_locked_gap, \
           stop_locked_global_heading, zone_clear_count, \
           recovery_mode, recovery_pivot_w, recovery_locked_heading, recovery_align_count

    # ── 복구 모드: 360° 갭 탐색 후 정렬 피봇 ──
    if recovery_mode:
        heading_err = abs(((heading_deg - recovery_locked_heading) + 180) % 360 - 180)
        if heading_err < HEADING_CONVERGE_DEG:
            recovery_align_count += 1
        else:
            recovery_align_count = 0
        if recovery_align_count >= 2:
            if DEBUG_STOP:
                print(f"  [RECOVERY] 정렬 완료 err={heading_err:.1f}° → 주행 복귀")
            recovery_mode        = False
            recovery_align_count = 0
            return find_vw_combined(scan_points, heading_deg)
        if DEBUG_STOP:
            print(f"  [RECOVERY] 피봇 w={recovery_pivot_w:+.2f} err={heading_err:.1f}°")
        return MIN_SPEED, recovery_pivot_w

    # detect_stop_zone 이중 호출 방지: 결과를 캐시하여 재사용
    zone_detected = detect_stop_zone(scan_points)

    # ── STOP 피봇 진행 중: 탈출 조건 확인 ──
    if stop_cycle_count > 0:
        heading_err = abs(((heading_deg - stop_locked_global_heading) + 180) % 360 - 180)
        if not zone_detected:
            zone_clear_count += 1
        else:
            zone_clear_count = 0
        zone_clear = zone_clear_count >= 2
        if heading_err < HEADING_CONVERGE_DEG or zone_clear:
            if DEBUG_STOP:
                reason = "heading" if heading_err < HEADING_CONVERGE_DEG else "zone_clear"
                print(f"  [STOP] exit ({reason}) err={heading_err:.1f}° -> combined")
            stop_cycle_count = 0
            stop_pivot_w     = 0.0
            zone_clear_count = 0
            return find_vw_combined(scan_points, heading_deg)

    # ── STOP zone 감지 ──
    if zone_detected:
        if stop_cycle_count < STOP_MAX_CYCLES:
            if stop_cycle_count == 0:
                target, gap_width = find_stop_escape_direction(scan_points)
                stop_locked_target = target
                stop_locked_gap    = gap_width
                stop_locked_global_heading = ((heading_deg - target) + 180) % 360 - 180
                if abs(target) < 5:
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
            return MIN_SPEED, pivot_w
        else:
            # STOP 최대 사이클 초과 → 360° 복구 모드 진입
            fscan = full_scan if full_scan is not None else scan_points
            target, gap_width = find_best_gap_360(fscan)
            recovery_locked_heading = ((heading_deg - target) + 180) % 360 - 180
            if abs(target) > 5:
                recovery_pivot_w = -math.copysign(MAX_W, target)
            else:
                r_left  = [d for a, d in fscan if -80 <= a < -10 and LIDAR_MIN_VALID < d < DETECTION_RANGE]
                r_right = [d for a, d in fscan if  10 < a <= 80  and LIDAR_MIN_VALID < d < DETECTION_RANGE]
                lm = sum(r_left)  / len(r_left)  if r_left  else 0
                rm = sum(r_right) / len(r_right) if r_right else 0
                recovery_pivot_w = -MAX_W if rm >= lm else MAX_W
            recovery_mode        = True
            recovery_align_count = 0
            stop_cycle_count     = 0
            stop_pivot_w         = 0.0
            zone_clear_count     = 0
            if DEBUG_STOP:
                print(f"  [RECOVERY] 진입 — 360° 갭={target:+.1f}° "
                      f"width={gap_width:.0f}mm target_h={recovery_locked_heading:.1f}°")
            return MIN_SPEED, recovery_pivot_w

    if stop_cycle_count > 0:
        # zone_clear_count가 2 미만 — 아직 연속 클리어 미확인, 피봇 유지
        return MIN_SPEED, stop_pivot_w

    return find_vw_combined(scan_points, heading_deg)


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
                prev_v = 0.0  # keepalive가 이전 속도로 재주행하는 것 방지
                prev_w = 0.0
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
                            v, w = find_vw_command(wide_points, arduino_heading_deg, scan_points)

                            # STOP/Recovery 피봇은 즉시 반영, 일반 주행만 W_SMOOTH 적용
                            if stop_cycle_count > 0 or recovery_mode:
                                prev_w = w
                            elif w == 0.0:
                                prev_w = 0.0
                            else:
                                if w * prev_w < 0:
                                    prev_w = 0.0   # 방향 전환 시 히스토리 초기화 후 75% 적용
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

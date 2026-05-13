"""
장지원 자작 알고리즘
RPLIDAR C1 장애물 회피 및 탈출 알고리즘 (v5)
통신: 라이다 /dev/ttyUSB0 (UART) ↔ 아두이노 /dev/ttyS0 (UART)
"""

import serial  # 시리얼 통신(UART)을 사용하기 위한 라이브러리를 불러옵니다.
import time    # 딜레이(sleep) 및 현재 시간(타임스탬프) 측정을 위한 라이브러리를 불러옵니다.
import math    # 삼각함수(sin, cos), 제곱근(sqrt) 등 수학적 계산을 위한 라이브러리를 불러옵니다.

# ── 포트 ─────────────────────────────────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"  # 라이다 센서가 연결된 USB 시리얼 포트 경로를 지정합니다.
ARDUINO_PORT     = "/dev/ttyAMA3"  # 아두이노(모터 제어기)가 연결된 라즈베리파이의 하드웨어 시리얼 포트 경로를 지정합니다.
BAUDRATE_LIDAR   = 460800          # 라이다 센서와의 통신 속도를 460800 bps로 설정합니다.
BAUDRATE_ARDUINO = 9600            # 아두이노와의 통신 속도를 9600 bps로 설정합니다.

# ── 라이다 보정 ───────────────────────────────────────────────────────────────
LIDAR_OFFSET = 20  # 라이다 하드웨어 특성상 측정값이 실제 물리적 거리보다 20mm 짧게 나오므로 이를 보정하기 위한 상수입니다.

# ── 로봇 파라미터 ─────────────────────────────────────────────────────────────
ROBOT_HALF_WIDTH = 110  # 로봇의 중심(라이다 위치)부터 좌/우측 끝부분까지의 거리(110mm)를 정의합니다.
ROBOT_FRONT_DIST = 120  # 로봇의 중심(라이다 위치)부터 정면 끝부분(범퍼)까지의 거리(120mm)를 정의합니다.
SAFETY_MARGIN    = 10   # 로봇 폭에 추가로 여유를 둘 수평 안전 마진(10mm)을 정의합니다. (총 통과 폭 계산에 사용)

# ── 위험구역 ──────────────────────────────────────────────────────────────────
DETECTION_RANGE  = 1500  # 라이다 데이터 중 신뢰할 수 있는 최대 거리(1.5m)를 지정합니다. 이보다 먼 데이터는 무시합니다.
FORWARD_RANGE    = 800   # 로봇이 주행 중 장애물을 신경 써야 하는 전방 감시 구역의 깊이(800mm)를 지정합니다.

# ── 속도 파라미터 ─────────────────────────────────────────────────────────────
FORWARD_SPEED    = 0.35  # 로봇이 전방에 장애물이 없을 때 낼 수 있는 최고 직진 속도(0.35 m/s)입니다.
MIN_SPEED        = 0.07  # 로봇이 장애물에 근접해 감속하더라도, 완전히 멈추지 않고 유지할 최소 직진 속도(0.07 m/s)입니다.
SLOW_START_DIST  = 250   # 전방 장애물까지의 거리가 250mm 이하가 되면 이때부터 최고 속도에서 선형적으로 감속을 시작합니다.
STOP_FWD_RANGE   = 120   # 전방 장애물까지의 거리가 120mm 이하가 되면 로봇의 전진(v)을 완전히 멈추는 구역입니다.
STOP_HORIZ_RANGE = 110   # 전진을 멈추는 구역의 좌우 폭(110mm)을 지정합니다. (로봇 반폭과 동일)
STOP_BACKUP_TIME = 0.3   # 위험구역에 깊게 진입했을 때 안전 확보를 위해 뒤로 물러나는(후진) 기본 지속 시간(0.3초)입니다.
W_GAIN           = 1.2   # 장애물과의 수평 오차(horiz_error)에 곱해져 회전 각속도(w)를 결정하는 비례 제어(P-제어) 게인입니다.
MAX_W            = 1.5   # 로봇이 회피를 위해 제자리 회전 또는 곡선 주행 시 낼 수 있는 최대 각속도(1.5 rad/s)입니다.
W_MIN_DANGER     = 0.5   # 위험 구역에서 장애물을 회피할 때, 수평 오차가 작더라도 최소한으로 보장하는 회전 속도(0.5 rad/s)입니다.
W_SMOOTH         = 0.6   # 이전 각속도와 현재 각속도를 부드럽게 이어주기 위한 저역통과 필터(LPF) 계수입니다. (튀는 움직임 방지)
SIDE_ROTATE_SAFE = 150   # 제자리 회전 시 로봇 측면(옆면)이 장애물에 긁히지 않도록 확인하는 측면 최소 안전 거리(150mm)입니다.
SIDE_CHECK_ANGLE = 60    # 측면 안전을 확인할 때 기준이 되는 라이다 스캔 각도 범위(±60도)입니다.

# ── 헤딩 방향 점수제 ──────────────────────────────────────────────────────────
HEADING_WEIGHT   = 1.0   # 원래 가야 할 방향(헤딩)으로 돌아가려는 성향의 가중치입니다. 1도당 여유공간 1도의 가치를 줍니다.
MIN_VIABLE_CLEAR = 25    # 장애물 회피 시, 한쪽 방향에 연속된 빈 공간이 최소 25도 이상이어야 통과 가능한 것으로 판단합니다.

# ── 헤딩 > 90° 능동 복귀 ─────────────────────────────────────────────────────
HEADING_OVER_90      = 90.0  # 로봇의 현재 헤딩이 원래 목표에서 90도 이상 틀어졌음을 판단하는 기준 각도입니다.
RECOVERY_W           = 0.8   # 헤딩을 원래 방향으로 복귀시킬 때 사용할 제자리 회전 또는 보정 회전 각속도(0.8 rad/s)입니다.
RECOVERY_SAFE_DIST   = 350   # 헤딩 복귀 회전을 수행하기 위해 전방에 확보되어야 하는 최소 안전 거리(350mm)입니다.

# ── 반대방향 감지 및 방향 보정 ────────────────────────────────────────────────
MISSION_HEADING_LIMIT = 90.0  # 주행 중 로봇이 목표 궤도에서 ±90도 이상 벗어나면 주행을 멈추고 헤딩 복귀 모드로 들어가는 한계치입니다.

# ── 막힘 감지 ─────────────────────────────────────────────────────────────────
STUCK_CLEAR_DIST    = 400  # 공간 너비 계산 시, 라이다 거리가 400mm 이상 찍히면 해당 방향은 '열려 있는(뚫린) 공간'으로 간주합니다.
STUCK_MAX_SAFETY    = 30   # 통과 가능 너비를 계산할 때 로봇 폭 외에 추가로 더하는 최대 동적 안전 여유치(30mm)입니다.
STUCK_TRIGGER_COUNT = 3    # 단발성 노이즈로 인한 오판을 막기 위해, 3회 연속으로 '막힘' 판정이 나와야 진짜 갇혔다고 인정합니다.

# ── 탈출 회전 ─────────────────────────────────────────────────────────────────
ESCAPE_CLEAR_DIST    = 500  # 탈출할 새로운 방향을 찾을 때, 최소 500mm 이상의 빈 공간이 확보된 방향을 찾도록 하는 기준 거리입니다.
ESCAPE_W             = 1.0  # 막힘 상태에서 새로운 방향으로 제자리 회전(탈출)할 때 사용하는 각속도(1.0 rad/s)입니다.
ESCAPE_TIMEOUT       = 15.0 # 탈출을 위해 회전하거나 후진하는 행위가 최대 15초를 넘기면 강제로 타임아웃 처리하고 탈출을 중단합니다.
ESCAPE_TOLERANCE     = 8.0  # (현재 미사용) 탈출 목표 각도에 도달했음을 판정하는 오차 허용 범위(8도)입니다.
ESCAPE_ROTATION_SAFE = 310  # 제자리 회전 탈출 시, 측면 장애물에 부딪히지 않도록 담보되어야 하는 최소 회전 반경(310mm)입니다.
ESCAPE_EXTRA_ANGLE   = 5    # 장애물을 간신히 피하는 각도가 아니라, 여유 있게 5도를 더 회전하여 안전을 확보하기 위한 여유 각도입니다.
MAX_ESCAPE_ANGLE     = 120  # 한 번에 너무 크게 회전하여 역주행하는 것을 막기 위해 탈출 회전 각도를 최대 120도로 제한합니다.
BACKUP_SPEED         = 0.10 # 막혀서 탈출할 때 공간을 확보하기 위해 뒤로 이동하는 후진 선속도(0.10 m/s)입니다.
BACKUP_DURATION      = 0.6  # 기본적으로 0.6초 동안 후진하여 제자리 회전을 위한 물리적 공간을 확보합니다.

# ── 스캔 파라미터 ─────────────────────────────────────────────────────────────
SCAN_HALF_ANGLE  = 90  # 로봇 정면(0도)을 기준으로 좌/우 각각 90도(총 180도)를 주요 전방 관심 구역으로 설정합니다.
ANGLE_STEP       = 5   # 라이다의 수많은 점 데이터를 5도 단위의 버킷(구간)으로 묶어서 처리하여 연산량을 줄입니다.
SEND_INTERVAL    = 0.1 # 모터 제어 명령(v, w)을 아두이노로 송신하는 주기(0.1초 = 10Hz)를 지정합니다.
# ─────────────────────────────────────────────────────────────────────────────

arduino_heading_deg  = 0.0   # 아두이노 센서(IMU)로부터 수신받은 현재 로봇의 헤딩 각도를 저장할 전역 변수입니다.
stuck_count          = 0     # 로봇이 전방 통과 불가(막힘) 상태로 판정된 횟수를 누적하는 카운터입니다.
prev_w               = 0.0   # LPF(저역통과필터) 적용을 위해 직전 스텝에서 계산된 회전 각속도(w)를 저장하는 변수입니다.
avoidance_w_sign     = 0.0   # 장애물을 회피할 때 왼쪽(+1)으로 갈지 오른쪽(-1)으로 갈지 방향성을 유지(히스테리시스)하는 변수입니다.
no_danger_count      = 0     # 전방에 위험물이 감지되지 않은 상태가 몇 번 연속되었는지 세는 카운터입니다. (회피 방향 리셋에 사용)
stop_zone_entry_time = None  # (현재 미사용) 정지 구역에 진입한 시간을 기록하여 특정 시간 이상 머물면 예외 처리하기 위한 변수입니다.

# 각도가 180도를 넘어가면 -360을 빼서 -180 ~ +180 범위로 정규화(변환)해주는 함수입니다.
def normalize_angle(angle):
    return angle - 360 if angle > 180 else angle

# 주어진 각도가 로봇의 정면 스캔 범위(현재 설정상 -90도 ~ +90도) 내에 포함되는지 True/False로 반환합니다.
def is_in_front(angle_norm):
    return -SCAN_HALF_ANGLE <= angle_norm <= SCAN_HALF_ANGLE

# 라이다에서 들어오는 5바이트짜리 원시(raw) 바이너리 데이터를 파싱하는 함수입니다.
def parse_packet(data):
    if len(data) != 5: # 데이터 길이가 5바이트가 아니면 깨진 패킷이므로 무시합니다.
        return None
    s_flag     = data[0] & 0x01          # 첫 번째 바이트의 0번 비트: 새로운 360도 스캔의 시작을 알리는 플래그입니다.
    s_inv_flag = (data[0] & 0x02) >> 1   # 첫 번째 바이트의 1번 비트: s_flag의 반전 값(검증용)입니다.
    if s_inv_flag != (1 - s_flag):       # s_flag와 s_inv_flag의 관계가 올바르지 않으면 데이터가 손상된 것이므로 무시합니다.
        return None
    if (data[1] & 0x01) != 1:            # 두 번째 바이트의 0번 비트(체크 비트)가 1이 아니면 잘못된 데이터이므로 무시합니다.
        return None
    quality     = data[0] >> 2           # 첫 번째 바이트의 나머지 비트를 통해 측정된 거리 데이터의 품질(신뢰도)을 추출합니다.
    angle_q6    = (data[1] >> 1) | (data[2] << 7)  # 두 번째와 세 번째 바이트를 조합하여 64배 스케일링된 각도 값을 추출합니다.
    angle       = angle_q6 / 64.0        # 추출한 각도 값을 64로 나누어 실제 도(degree) 단위 각도로 변환합니다.
    distance_q2 = data[3] | (data[4] << 8)         # 네 번째와 다섯 번째 바이트를 조합하여 4배 스케일링된 거리 값을 추출합니다.
    distance    = distance_q2 / 4.0      # 추출한 거리 값을 4로 나누어 실제 밀리미터(mm) 단위 거리로 변환합니다.
    return angle, distance, quality      # 정상적으로 추출된 각도, 거리, 품질 값을 튜플 형태로 반환합니다.

# 극좌표계(각도, 거리)로 측정된 라이다 데이터를 직교좌표계(x, y) 성분으로 분해하는 함수입니다.
def decompose(angle_norm_deg, distance_mm):
    rad   = math.radians(angle_norm_deg)           # 도(degree) 단위의 각도를 수학 계산을 위해 라디안(radian)으로 변환합니다.
    horiz = abs(distance_mm * math.sin(rad))       # 사인 함수를 이용해 로봇 중심선으로부터 좌/우로 떨어진 수평 거리를 계산합니다. (절댓값)
    fwd   = distance_mm * math.cos(rad)            # 코사인 함수를 이용해 로봇 중심으로부터 앞/뒤로 떨어진 전방 거리를 계산합니다.
    return horiz, fwd                              # 계산된 수평 거리와 전방 거리를 반환합니다.

# 아두이노(시리얼 포트)로부터 비동기적으로(블로킹 없이) 헤딩 데이터를 읽어오는 함수입니다.
def read_arduino(arduino):
    global arduino_heading_deg            # 전역 변수인 헤딩 각도를 업데이트하기 위해 global로 선언합니다.
    msg = None                            # 반환할 메시지 변수를 초기화합니다.
    while arduino.in_waiting > 0:         # 아두이노로부터 수신 버퍼에 쌓인 데이터가 있는 동안 계속 반복합니다.
        try:
            # 버퍼에서 한 줄(엔터 기준)을 읽어오고, utf-8로 디코딩한 뒤 양옆 공백과 줄바꿈을 제거합니다.
            line = arduino.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('H:'):     # 읽어온 문자열이 'H:'로 시작한다면 헤딩 데이터로 간주합니다.
                arduino_heading_deg = -float(line[2:])  # 'H:' 좌측 틀어짐 (+) 우측으로 틀어짐 (-) 뒷부분의 문자열을 실수(float)로 변환하여 헤딩 각도를 업데이트합니다.
            elif line:                    # 'H:'로 시작하지 않지만 빈 문자열이 아니면 기타 메시지로 취급합니다.
                msg = line                # 기타 메시지를 변수에 저장합니다 (디버깅 등에 활용 가능).
        except Exception:                 # 통신 노이즈 등으로 디코딩 에러가 발생하면 무시하고 넘어갑니다.
            pass
    return msg                            # 마지막으로 수신한 기타 메시지(없으면 None)를 반환합니다.


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [알고리즘 1] 막힘 감지: 제2코사인법칙을 이용한 공간 너비 계산
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def is_path_blocked(front_scan_points):
    # 정면 180도 스캔 데이터를 받아, 로봇이 물리적으로 빠져나갈 수 있는 폭(너비)이 존재하는지 계산하여 막힘 여부를 반환합니다.
    scan_dict = {}  # 각도를 5도(ANGLE_STEP) 단위 버킷으로 묶어서, 해당 방향의 가장 가까운 장애물 거리를 저장할 딕셔너리입니다.
    for angle_norm, dist in front_scan_points:  # 전면 스캔 데이터 리스트를 순회합니다.
        if dist <= 40 or dist > DETECTION_RANGE: 
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP  # 각도를 5도 배수에 가장 가까운 값으로 반올림하여 버킷을 구합니다.
        if bucket not in scan_dict or dist < scan_dict[bucket]: # 버킷이 처음이거나, 기존에 저장된 거리보다 현재 거리가 더 짧다면 (가장 가까운 장애물 갱신)
            scan_dict[bucket] = dist            # 해당 버킷의 거리를 더 짧은 값으로 업데이트합니다.

    # 탐색을 수행할 -90도부터 +90도까지의 5도 간격 각도 리스트를 생성합니다.
    angles  = list(range(-SCAN_HALF_ANGLE, SCAN_HALF_ANGLE + ANGLE_STEP, ANGLE_STEP))
    in_open = False  # 현재 순회 중인 각도가 통과 가능한 '열린 공간' 내부에 있는지 추적하는 상태 변수입니다.
    l_angle = None   # 하나의 열린 구간이 시작될 때, 그 구간의 왼쪽 경계를 이루는 장애물의 각도를 저장합니다.
    l_dist  = None   # 열린 구간의 왼쪽 경계를 이루는 장애물까지의 거리를 저장합니다.

    for idx, a in enumerate(angles):            # 생성한 각도 리스트를 왼쪽(-90도)부터 오른쪽(+90도) 순서로 순회합니다.
        d       = scan_dict.get(a, 0)           # 해당 버킷의 장애물 거리를 가져오고, 데이터가 없으면 0으로 처리합니다.
        is_open = (d >= STUCK_CLEAR_DIST)       # 장애물 거리가 400mm(STUCK_CLEAR_DIST) 이상 떨어져 있다면 통과 가능한 공간으로 봅니다.

        if not in_open:                         # 현재 닫힌 공간(장애물 앞)을 순회 중일 때:
            if is_open:                         # 갑자기 400mm 이상 빈 공간이 나타났다면 (열린 구간 시작점 감지)
                in_open = True                  # 상태를 열린 공간 내부로 변경합니다.
                if idx > 0:                     # 이 버킷이 스캔의 맨 첫 번째가 아니라면
                    prev_a  = angles[idx - 1]   # 빈 공간이 시작되기 바로 직전의 각도(장애물이 있던 마지막 각도)를 찾습니다.
                    l_angle = prev_a            # 그 각도를 왼쪽 경계 장애물의 각도로 설정합니다.
                    l_dist  = scan_dict.get(prev_a, 1) or 1  # 왼쪽 경계 장애물까지의 거리를 가져옵니다 (0 방지를 위해 최소 1 부여).
                else:                           # 첫 번째 버킷부터 뚫려 있다면 (스캔 시작점 자체가 허공인 경우)
                    l_angle = a - ANGLE_STEP    # 보수적인 계산을 위해 가상의 왼쪽 경계를 -95도로 설정합니다.
                    l_dist  = STUCK_CLEAR_DIST  # 가상의 왼쪽 경계 거리도 안전 기준인 400mm로 설정합니다.

        else:  # in_open (현재 열린 공간 내부를 순회 중일 때):
            if not is_open:                     # 거리가 400mm 미만인 장애물이 다시 나타났다면 (열린 구간 종료점 감지)
                r_angle = a                     # 현재 각도를 열린 구간을 막아선 오른쪽 경계 장애물의 각도로 설정합니다.
                r_dist  = d or 1                # 오른쪽 경계 장애물까지의 거리를 가져옵니다 (0 방지를 위해 최소 1 부여).

                theta = math.radians(r_angle - l_angle)  # 좌측 경계와 우측 경계 사이의 각도 차이(끼인각)를 라디안으로 계산합니다.
                if theta > 0 and l_dist > 0:             # 각도 차이가 정상이고 왼쪽 거리가 유효하다면
                    # 제2코사인법칙을 사용하여, 라이다 센서 관점이 아닌 실제 두 장애물 사이의 직선 물리적 너비(w)를 계산합니다.
                    w = math.sqrt(l_dist**2 + r_dist**2 - 2 * l_dist * r_dist * math.cos(theta))
                    
                    d_ref   = min(l_dist, r_dist)        # 좌측과 우측 장애물 중 로봇에 더 가까운 쪽의 거리를 기준 거리로 삼습니다.
                    # 거리가 가까울수록 추가 안전 여유폭(safety)을 줄이고, 멀수록 STUCK_MAX_SAFETY(30mm)에 가깝게 부여합니다.
                    safety  = STUCK_MAX_SAFETY * min(d_ref / STUCK_CLEAR_DIST, 1.0)
                    min_gap = ROBOT_HALF_WIDTH * 2 + safety # 로봇의 전체 폭(반폭*2)에 동적으로 계산된 안전 여유폭을 더해 통과 필요 최소 너비를 구합니다.
                    
                    # 디버깅을 위해 열린 구간의 각도, 거리, 계산된 너비, 통과 가능 여부를 터미널에 출력합니다.
                    print(f"  [OpenGap] {l_angle}°~{r_angle-ANGLE_STEP}°  d_L={l_dist:.0f} d_R={r_dist:.0f} width={w:.0f}mm min={min_gap:.0f}mm " + ("✓Passable" if w >= min_gap else "✗Narrow"))
                    if w >= min_gap:            # 계산된 실제 너비가 로봇이 통과하기 위한 최소 필요 너비보다 넓다면
                        return False            # 막히지 않았음(통과 가능)을 의미하는 False를 반환하고 함수를 종료합니다.

                in_open = False                 # 해당 열린 구간은 로봇이 지나가기엔 너무 좁으므로, 다시 닫힌 공간 상태로 되돌리고 다음 탐색을 이어갑니다.

    # 180도 스캔 루프가 끝났는데 끝까지 열려있는 상태로 끝난 경우 (오른쪽 벽이 없는 경우)
    if in_open and l_dist:                      
        r_angle = SCAN_HALF_ANGLE + ANGLE_STEP  # 가상의 오른쪽 경계를 +95도로 설정합니다.
        r_dist  = STUCK_CLEAR_DIST              # 가상의 오른쪽 거리를 400mm로 설정합니다.
        theta   = math.radians(r_angle - l_angle) # 가상의 오른쪽 경계와 기존 왼쪽 경계 사이의 끼인각을 계산합니다.
        if theta > 0:                           # 유효한 각도라면
            # 제2코사인법칙으로 마지막 뚫린 구간의 너비를 추정 계산합니다.
            w = math.sqrt(l_dist**2 + r_dist**2 - 2 * l_dist * r_dist * math.cos(theta))
            d_ref   = min(l_dist, r_dist)       # 기준 거리 산정 (앞선 로직과 동일)
            safety  = STUCK_MAX_SAFETY * min(d_ref / STUCK_CLEAR_DIST, 1.0) # 동적 안전 여유폭 산정
            min_gap = ROBOT_HALF_WIDTH * 2 + safety # 최소 통과 필요 너비 산정
            # 마지막 구간의 결과 출력
            print(f"  [OpenGapEnd] {l_angle}°~{SCAN_HALF_ANGLE}°  d_L={l_dist:.0f} width≈{w:.0f}mm min={min_gap:.0f}mm " + ("✓Passable" if w >= min_gap else "✗Narrow"))
            if w >= min_gap:                    # 마지막 구간의 너비가 충분히 넓다면
                return False                    # 막히지 않았음을 반환합니다.

    return True  # 모든 탐색을 마쳤음에도 로봇이 빠져나갈 만큼 충분히 넓은 구간이 단 하나도 없다면, 전방이 완전히 막힌 것(True)으로 반환합니다.


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [알고리즘 2] 탈출 방향 계산: 360도 스캔 기반 최대 열린 섹터 탐색
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 전방이 막혔을 때 360도 전체 스캔 데이터를 바탕으로 가장 넓게 트인 탈출 방향(각도)을 찾아 반환하는 함수입니다.
def find_escape_angle(all_scan_points):
    scan_dict = {}                              # 360도 전체를 5도 단위 버킷으로 나누어 최단 장애물 거리를 저장할 딕셔너리입니다.
    for angle_norm, dist in all_scan_points:    # 360도 모든 스캔 데이터를 순회합니다.
        if dist <= 40 or dist > DETECTION_RANGE: 
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP # 각도를 5도 버킷 단위로 반올림합니다.
        if bucket not in scan_dict or dist < scan_dict[bucket]: # 해당 버킷에 값이 없거나 현재 거리가 더 짧다면 갱신합니다.
            scan_dict[bucket] = dist

    # -180도부터 175도까지 5도 간격으로 모든 각도를 담은 리스트를 만듭니다 (총 72개 요소).
    all_angles = list(range(-180, 180, ANGLE_STEP))
    n = len(all_angles)                         # 각도 버킷의 총개수(72)를 저장합니다.

    # 각 버킷(5도 각도)별로 장애물 거리가 탈출 기준 거리(ESCAPE_CLEAR_DIST, 500mm) 이상 뚫려 있는지 여부(True/False)를 리스트로 만듭니다.
    open_flags = [
        scan_dict.get(a, 0) >= ESCAPE_CLEAR_DIST
        for a in all_angles
    ]

    best_len   = 0  # 가장 길게 연속으로 열려있는(True) 구간의 길이(버킷 개수)를 추적합니다.
    best_start = 0  # 가장 긴 구간이 시작되는 인덱스(시작점)를 추적합니다.

    for start in range(n):                      # 0부터 71번 버킷까지 시작점을 옮겨가며 순환 배열처럼 검사합니다.
        length = 0                              # 현재 시작점에서부터 몇 개의 버킷이 연속으로 열려 있는지 세는 변수입니다.
        for i in range(n):                      # 최대 360도(n개)만큼 앞으로 전진하며 검사합니다.
            if open_flags[(start + i) % n]:     # 배열의 인덱스가 n을 넘어가면 다시 0으로 순환(%)하도록 하여 연속된 열림 상태를 확인합니다.
                length += 1                     # 열려 있다면 연속 길이를 1 증가시킵니다.
            else:
                break                           # 중간에 막힌 곳(False)이 나오면 이 시작점에서의 탐색을 즉시 종료합니다.
        if length > best_len:                   # 이번에 찾은 연속 열림 길이가 지금까지 찾은 최고 기록보다 길다면
            best_len   = length                 # 최고 기록(최장 길이)을 갱신합니다.
            best_start = start                  # 가장 긴 구간의 시작 인덱스도 갱신합니다.

    if best_len == 0:                           # 360도를 다 뒤졌는데 500mm 이상 뚫린 곳이 단 한 군데도 없다면
        print("  [Escape] No open space → Rotate 90°") # 터미널에 메시지를 남기고
        return 90.0                             # 임의로 우측 90도로 제자리 회전하도록 지시합니다 (궁여지책).

    # 가장 넓게 열린 섹터를 찾았다면, 그 섹터의 한가운데 인덱스를 계산합니다.
    center_idx   = (best_start + best_len // 2) % n
    target_angle = all_angles[center_idx]       # 한가운데 인덱스에 해당하는 실제 물리적 각도(도)를 구하여 목표 각도로 설정합니다.

    # 탈출 방향 결정 결과를 터미널에 출력합니다 (총 몇 도짜리 너비이고, 중심은 몇 도인지).
    print(f"  [EscapeDir] Max open sector {best_len * ANGLE_STEP}°  → Target angle {target_angle}°")

    # 하지만 찾은 최적 각도가 로봇이 한 번에 회전하도록 허용된 최대 각도(MAX_ESCAPE_ANGLE, 120도)를 넘어선다면
    if abs(target_angle) > MAX_ESCAPE_ANGLE:
        print(f"  [EscapeDir Limit] {target_angle}° > {MAX_ESCAPE_ANGLE}° → Search optimal dir within ±{MAX_ESCAPE_ANGLE}°")

        # 각도를 -120도 ~ +120도 사이로만 제한하여 다시 리스트를 만듭니다.
        limited_angles = [a for a in all_angles if abs(a) <= MAX_ESCAPE_ANGLE]
        # 제한된 각도 범위 내에서만 열림/닫힘(True/False) 상태를 추출합니다.
        limited_open   = [open_flags[all_angles.index(a)] for a in limited_angles]

        best_l_len   = 0                        # 제한된 범위 안에서 가장 긴 연속 구간의 길이를 저장합니다.
        best_l_start = 0                        # 제한된 범위 안에서 가장 긴 연속 구간의 시작 인덱스입니다.
        for start in range(len(limited_angles)):# 제한된 각도 리스트 안에서 동일한 방식으로 연속 열림 구간을 재탐색합니다.
            length = 0
            for i in range(len(limited_angles)):
                if limited_open[(start + i) % len(limited_angles)]: # 여기서는 원형 버퍼(%)를 쓰지만, 의미상 잘린 구간의 순환입니다.
                    length += 1
                else:
                    break
            if length > best_l_len:             # 기록 갱신 여부 확인
                best_l_len   = length
                best_l_start = start

        if best_l_len > 0:                      # 제한된 범위 내에서도 열린 구간을 찾았다면
            c_idx        = (best_l_start + best_l_len // 2) % len(limited_angles) # 그 구간의 중앙 인덱스를 구하고
            target_angle = limited_angles[c_idx] # 최종 목표 각도로 덮어씁니다.
            print(f"  [EscapeDir Limit] Optimal in range: {target_angle}°")
        else:                                   # 제한된 범위 내(-120~120도)는 죄다 막혀있고, 120도 넘어가는 후방쪽만 뚫려 있는 최악의 경우라면
            target_angle = MAX_ESCAPE_ANGLE * (1 if target_angle > 0 else -1) # 부호만 유지한 채 최대 한계치(±120도)로 강제 클램프 시킵니다.
            print(f"  [EscapeDir Limit] No open space → Clamped to {target_angle}°")

    return float(target_angle)                  # 최종적으로 계산된 로봇 중심 기준 상대적인 탈출 목표 각도를 반환합니다.


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 탈출 회전 유틸리티 & 실행
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 로봇이 제자리 회전하는 도중에도 주변 상황(라이다)과 자세(아두이노 헤딩)를 비동기적으로 계속 갱신하며 수집하는 함수입니다.
def collect_scan_during_rotation(arduino, lidar, duration=0.12):
    scan_buf = []                               # 수집한 스캔 데이터를 잠시 담아둘 빈 리스트 버퍼를 만듭니다.
    t = time.time()                             # 함수가 시작된 현재 시각을 기록합니다.
    while time.time() - t < duration:           # 지정된 시간(duration, 기본 0.12초)이 경과할 때까지 계속 반복합니다.
        read_arduino(arduino)                   # 반복하는 동안 최신 헤딩 데이터를 계속 아두이노로부터 읽어옵니다.
        while lidar.in_waiting >= 5:            # 라이다 시리얼 버퍼에 1패킷(5바이트) 이상의 데이터가 들어왔다면
            raw = lidar.read(5)                 # 5바이트를 읽어옵니다.
            result = parse_packet(raw)          # 바이너리 패킷을 파싱하여 각도, 거리, 품질로 변환합니다.
            if result:                          # 정상적으로 파싱되었다면
                a, d, _ = result                # 각도(a)와 거리(d)를 분리합니다.
                if d > 0:                       # 거리 데이터가 정상(0 초과)이라면
                    # 각도를 -180~180으로 정규화하고, 센서 물리적 오프셋을 더한 최종 거리를 버퍼에 추가합니다.
                    scan_buf.append((normalize_angle(a), d + LIDAR_OFFSET))
    return scan_buf                             # 정해진 시간 동안 모은 스캔 데이터 버퍼를 반환합니다.


# 로봇이 제자리 회전을 하려고 할 때, 회전하는 쪽 측면에 튀어나온 장애물이 있어서 긁히거나 부딪히지 않을지 사전에 확인하는 함수입니다.
def check_rotation_blocked(w_sign, scan_points):
    scan_dict = {}                              # 주어진 스캔 데이터를 5도 버킷 단위 최단 거리로 묶습니다.
    for angle_norm, dist in scan_points:
        if dist <= 40 or dist > DETECTION_RANGE: 
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    if w_sign < 0:                              # w_sign이 음수면 로봇이 우회전(시계방향)하려는 상황입니다.
        check = range(ANGLE_STEP, 61, ANGLE_STEP)  # 우회전이므로 로봇의 우측 전방(+5도 ~ +60도) 구역을 검사 범위로 잡습니다.
    else:                                       # w_sign이 양수면 로봇이 좌회전(반시계방향)하려는 상황입니다.
        check = range(-ANGLE_STEP, -61, -ANGLE_STEP) # 좌회전이므로 로봇의 좌측 전방(-5도 ~ -60도) 구역을 검사 범위로 잡습니다.

    return any(                                 # 검사 범위 안의 어떤 버킷(a)이라도 다음 조건을 만족하면 True(회전 시 충돌 위험)를 반환합니다.
        # 해당 각도의 장애물이 감지되었고(0 초과), 그 거리가 안전 회전 반경(ESCAPE_ROTATION_SAFE, 310mm)보다 짧은지 확인합니다.
        0 < scan_dict.get(a, DETECTION_RANGE + 1) < ESCAPE_ROTATION_SAFE
        for a in check
    )


# 로봇이 '완전히 막힘'으로 판정되었을 때, 이 위기를 벗어나기 위해 동적 후진 및 제자리 회전을 수행하는 핵심 탈출 시퀀스입니다.
def execute_escape_rotation(arduino, lidar, all_scan_points):
    global arduino_heading_deg, stuck_count, prev_w # 아두이노 헤딩, 막힘 카운트, 이전 w 값을 제어하기 위해 전역 변수로 가져옵니다.

    print("\n" + "="*52)                        # 탈출 모드 진입을 알리는 시각적 구분선을 터미널에 출력합니다.
    heading_deg    = arduino_heading_deg        # 현재 헤딩(궤도 이탈 각도)을 지역 변수에 백업해둡니다.
    BACKUP_MAX_TIME = 3.0                       # 최대 허용 후진 시간을 3초로 넉넉하게 설정합니다.
    HEADING_HINT_MIN = 5.0                      # 헤딩이 5도 이상 틀어져 있다면, 헤딩 각도를 탈출 방향 결정의 '힌트'로 사용하기 위한 최소 기준입니다.
    print(f"[ESCAPE] Cannot move forward  Heading:{heading_deg:.1f}°") # 현재 전진 불가를 선언하고 헤딩 값을 터미널에 알립니다.

    # [Step 1] 어느 방향으로 회전해 탈출할지 초기 방향을 결정합니다.
    if abs(heading_deg) >= HEADING_HINT_MIN:    # 로봇이 목표 궤도(0도)에서 5도 이상 틀어진 상태로 막혔다면
        w_sign  = -1.0 if heading_deg > 0 else 1.0  # 틀어진 반대 방향(헤딩을 줄이는 쪽)으로 도는 것을 최우선 탈출 방향으로 삼습니다.
        hint    = "Opposite Heading"            # 터미널 출력을 위한 힌트 문구를 설정합니다.
    else:                                       # 헤딩이 거의 0도(정면으로 가다 꽉 막힌 상태)라면
        t       = find_escape_angle(all_scan_points) # 앞서 정의한 함수를 통해 360도 스캔에서 가장 넓은 쪽 각도를 계산해옵니다.
        w_sign  = -1.0 if t >= 0 else 1.0       # 가장 넓은 각도가 우측(+)이면 우회전(-1), 좌측(-)이면 좌회전(+1) 부호를 결정합니다. (각도계의 방향에 따름)
        hint    = "Scan Based(Heading≈0°)"      # 힌트 문구를 설정합니다.

    print(f"  [{hint}] Target Dir: {'Right' if w_sign<0 else 'Left'}") # 결정된 1차 목표 회전 방향을 출력합니다.

    # [Step 2] 회전 시 부딪히지 않도록, 측면에 여유 공간이 생길 때까지 뒤로 물러나는 동적 후진 함수를 내부에 정의합니다.
    def backup_until_clear(target_sign):
        t_start = time.time()                   # 후진 시작 시각을 기록합니다.
        while time.time() - t_start < BACKUP_MAX_TIME: # 최대 후진 시간(3초)이 지나기 전까지 루프를 돕니다.
            arduino.write(f"{-BACKUP_SPEED:.2f} 0.00\n".encode()) # 아두이노에 음수 선속도(-0.1)와 각속도 0을 주어 뒤로 직진 후진하게 합니다.
            scan_buf = collect_scan_during_rotation(arduino, lidar, duration=0.1) # 후진하는 동안 0.1초치 라이다 스캔을 수집합니다.
            # 만약 방금 찍은 라이다 기준으로, 목표 회전 방향(target_sign) 측면에 걸리는 장애물이 더 이상 없다면
            if scan_buf and not check_rotation_blocked(target_sign, scan_buf):
                arduino.write(b"0.00 0.00\n")   # 즉시 아두이노에 정지(v=0, w=0) 명령을 보냅니다.
                time.sleep(0.05)                # 모터가 완전히 설 때까지 0.05초 대기합니다.
                return True                     # 장애물이 지워져서(clear) 성공적으로 후진을 마쳤음을 True로 반환합니다.
        arduino.write(b"0.00 0.00\n")           # 3초 넘게 뒤로 뺐는데도 안 비워지면 일단 멈춥니다.
        return False                            # 타임아웃으로 인해 후진만으로는 회전 반경 확보에 실패했음을 False로 반환합니다.

    # 방금 결정한 1차 목표 방향(w_sign) 쪽 측면에 회전 시 부딪힐만한 장애물이 있는지 즉시 확인합니다.
    if check_rotation_blocked(w_sign, all_scan_points): 
        dir_str = "Right" if w_sign < 0 else "Left" 
        print(f"  [{dir_str}] Blocked → Backing up until clear...") # 막혀있음을 알리고 후진을 시작합니다.
        cleared = backup_until_clear(w_sign)    # 위에서 정의한 동적 후진 함수를 실행하여 공간이 확보되었는지(cleared) 받습니다.

        if not cleared:                         # 뒤로 최대한 빼봤는데도 여전히 회전하려는 쪽 측면에 장애물이 걸린다면
            w_sign  = -w_sign                   # 안 되겠다 싶어 반대 방향으로 회전 목표를 180도 바꿉니다.
            dir_str = "Right" if w_sign < 0 else "Left"
            print(f"  Still blocked after max backup → Try [{dir_str}]") # 방향을 틀었다고 출력합니다.

            if check_rotation_blocked(w_sign, all_scan_points): # 바꾼 반대 방향조차도 측면에 장애물이 걸려있다면 (양옆이 다 좁은 골목)
                print(f"  [{dir_str}] also blocked → Try backing up")
                cleared = backup_until_clear(w_sign) # 바꾼 방향 기준으로도 다시 뒤로 빼면서 공간 확보를 시도해봅니다.
                if not cleared:                 # 이마저도 실패했다면 (완전히 좁고 긴 터널에 끼인 상태)
                    t      = find_escape_angle(all_scan_points) # 가장 확실하게 넓게 뚫린 곳을 찾기 위해 다시 전체 스캔 분석 함수를 호출합니다.
                    w_sign = -1.0 if t >= 0 else 1.0 # 분석 결과를 토대로 최종 3차 목표 방향을 강제합니다.
                    print(f"  Decided based on scan (3rd attempt)") 
    else:                                       # 1차 목표 방향 측면에 아무 장애물이 없이 처음부터 여유가 있었다면
        print(f"  Direction open → Min backup ({BACKUP_DURATION}s)") # 그래도 제자리 회전 시의 축 틀어짐을 대비해 아주 살짝만 뒤로 물러납니다.
        t_backup = time.time()                  # 타이머 시작
        while time.time() - t_backup < BACKUP_DURATION: # 기본 세팅된 짧은 시간(0.6초) 동안만 루프를 돕니다.
            arduino.write(f"{-BACKUP_SPEED:.2f} 0.00\n".encode()) # 짧게 후진 명령
            time.sleep(0.05)                    # CPU 과부하 방지용 짧은 딜레이
        arduino.write(b"0.00 0.00\n")           # 정지
        time.sleep(0.05)                        # 정지 대기

    print(f"  Final Dir: {'Left' if w_sign>0 else 'Right'}") # 우여곡절 끝에 확정된 최종 탈출 회전 방향을 출력합니다.

    # [Step 3] 아두이노에 제어권 리셋 신호를 보냅니다.
    arduino.write(b"ESC\n")                     # 'ESC' 문자열을 보내면 아두이노 내부의 헤딩 누적값이나 오차를 리셋하여 깔끔하게 회전을 시작하도록 합니다.
    time.sleep(0.15)                            # 아두이노가 신호를 받고 내부 변수를 초기화할 시간을 0.15초 줍니다.

    # [Step 4] 정면이 뚫렸다고 판단될 때까지 계속 제자리 회전합니다.
    MAX_ROT = 350                               # 무한 팽이처럼 도는 것을 막기 위해, 한 번 탈출 시 최대 350도까지만 회전하도록 제한합니다.
    t_start = time.time()                       # 탈출 회전 시작 시간을 기록합니다 (타임아웃 감시용).

    while time.time() - t_start < ESCAPE_TIMEOUT: # 탈출 제한 시간(15초)을 넘기지 않는 동안 루프를 반복합니다.
        scan_buf  = collect_scan_during_rotation(arduino, lidar, duration=0.12) # 돌면서 0.12초마다 최신 스캔 데이터를 찍어옵니다.
        # 찍어온 스캔 데이터 중, '로봇의 정면 구간'에 해당하는 유효 장애물 데이터만 추려냅니다.
        front_pts = [(a, d) for a, d in scan_buf if is_in_front(a) and d > 0]

        # 방금 추려낸 정면 데이터를 is_path_blocked 함수에 넣어, 로봇이 빠져나갈 만큼 너비가 충분한지(False) 확인합니다.
        if front_pts and not is_path_blocked(front_pts): 
            # 정면에 충분한 공간이 발견되었다면, 목표 방향을 제대로 찾은 것입니다.
            print(f"  [Escape Done] Forward path found (Rotated:{abs(arduino_heading_deg):.1f}°)") 
            break                               # 탈출 회전 루프를 즉시 깨고 빠져나옵니다.

        if abs(arduino_heading_deg) > MAX_ROT:  # 회전을 계속 하다가 아두이노가 측정한 누적 회전 각도가 한계치(350도)를 넘었다면
            print("  [Escape] No path after 350° search → Exit") # 빙글빙글 한 바퀴 다 돌았는데도 나갈 구멍이 없다는 뜻이므로 포기하고 중단합니다.
            break

        # 돌고 있는 와중에 새로운 장애물이 나타나서 회전하는 측면이 차단(blocked)될 위기라면
        if scan_buf and check_rotation_blocked(w_sign, scan_buf):
            alt = -w_sign                       # 돌고 있던 방향의 반대 방향으로 임시 전환을 고려해봅니다.
            if not check_rotation_blocked(alt, scan_buf): # 반대 방향은 측면이 비어있다면
                print(f"  [Escape] Obstacle → Switch to {'Left' if alt>0 else 'Right'}") 
                w_sign = alt                    # 회전 방향을 반대로 역전시킵니다.
            else:                               # 반대 방향도 막혀있다면 (돌지도 못하게 꽉 낀 상황)
                arduino.write(b"0.00 0.00\n")   # 모터를 완전히 세웁니다.
                time.sleep(0.1)                 # 0.1초 쉬었다가
                continue                        # 다음 스캔을 찍고 다시 판단하도록 루프의 처음으로 돌아갑니다.

        # 위 예외 상황들이 아니라면, 정상적으로 탈출용 회전 각속도(w_sign * ESCAPE_W)를 아두이노에 보냅니다. 선속도(v)는 0입니다.
        arduino.write(f"0.00 {w_sign * ESCAPE_W:.2f}\n".encode())

    else:                                       # 15초(ESCAPE_TIMEOUT)가 다 지날 때까지 루프가 break로 끝나지 않았다면 타임아웃입니다.
        print("  [Escape] Timeout")

    arduino.write(b"0.00 0.00\n")               # 탈출 과정이 끝났으므로 모터를 안전하게 정지시킵니다.
    time.sleep(0.3)                             # 관성으로 도는 것을 잡기 위해 0.3초간 정지 상태를 유지합니다.
    stuck_count = 0                             # 다시 일반 주행으로 복귀하기 위해 누적된 막힘 카운터를 0으로 초기화합니다.
    prev_w      = 0.0                           # LPF용 이전 각속도 변수도 초기화하여 회전이 튀지 않게 합니다.
    print("="*52 + "\n")                        # 탈출 모드 종료를 알리는 구분선을 출력합니다.


def execute_direction_correction(arduino, lidar, all_scan_points):
    # 로봇이 장애물을 피하느라 원래 목표 궤도(헤딩)에서 너무 크게 이탈했을 때(±90도 이상), 다시 원래 방향(0도 근처)으로 기수를 돌려놓는 함수입니다.
    global arduino_heading_deg, prev_w          # 헤딩 값과 LPF 변수를 가져옵니다.

    heading_deg = arduino_heading_deg           # 현재 틀어진 헤딩 각도를 저장합니다.
    print("\n" + "="*52)
    print(f"[Correction] Heading:{heading_deg:.1f}° → Return within ±{MISSION_HEADING_LIMIT}°") # 보정을 시작함을 알립니다.

    w_sign = -1.0 if heading_deg > 0 else 1.0   # 헤딩이 양수(오른쪽 틀어짐)면 좌회전(-1.0), 음수면 우회전(1.0)으로 원래 방향으로 가는 최단 거리 회전 부호를 정합니다.

    # 보정하려고 도는 방향 측면에 장애물이 있는지 사전에 체크합니다.
    if check_rotation_blocked(w_sign, all_scan_points): 
        w_sign = -w_sign                        # 장애물이 걸린다면 반대쪽으로 크게 돌아서 복귀하려고 부호를 바꿉니다.
        if check_rotation_blocked(w_sign, all_scan_points): # 반대쪽으로 도는 것도 측면에 걸린다면
            print("  Both sides blocked → Cannot correct direction, return to normal avoidance") # 지금 제자리 회전할 여유가 없으므로 보정을 포기하고 함수를 빠져나갑니다.
            print("="*52 + "\n")
            return

    print(f"  Rot Dir: {'Left' if w_sign>0 else 'Right'}") # 돌기로 결정된 방향을 출력합니다.

    t_start = time.time()                       # 타임아웃 방지용 타이머를 시작합니다.
    while time.time() - t_start < ESCAPE_TIMEOUT: # 타임아웃(15초) 내에서 루프를 돕니다.
        scan_buf = collect_scan_during_rotation(arduino, lidar, duration=0.12) # 회전하며 라이다 스캔을 찍어옵니다.

        # 돌다가 아두이노에서 보낸 현재 헤딩 각도의 절댓값이 허용 한계(MISSION_HEADING_LIMIT, 90도) 안으로 들어왔다면
        if abs(arduino_heading_deg) <= MISSION_HEADING_LIMIT: 
            print(f"  [Correction Done] Heading:{arduino_heading_deg:.1f}°") # 성공을 알리고
            break                               # 루프를 즉시 종료합니다.

        # 돌고 있는데 장애물이 다가와서 측면이 막힐 위기에 처했다면
        if scan_buf and check_rotation_blocked(w_sign, scan_buf):
            alt = -w_sign                       # 반대 방향 전환을 고려해보고
            if not check_rotation_blocked(alt, scan_buf): # 비어있으면
                print(f"  [Correction] Obstacle → Switch to {'Left' if alt>0 else 'Right'}")
                w_sign = alt                    # 반대로 회전 방향을 틉니다.
            else:                               # 양쪽 다 막히게 되면
                arduino.write(b"0.00 0.00\n")   # 일단 서서
                time.sleep(0.1)                 # 0.1초 쉬면서 상황을 봅니다.
                continue

        # 보정 회전용 각속도(RECOVERY_W)를 모터에 명령하여 실제로 회전시킵니다.
        arduino.write(f"0.00 {w_sign * RECOVERY_W:.2f}\n".encode())

    else:                                       # 15초 넘게 90도 안으로 못 들어왔으면 타임아웃 출력
        print("  [Correction] Timeout")

    arduino.write(b"0.00 0.00\n")               # 보정이 끝났으니 정지 명령
    time.sleep(0.3)                             # 정지 대기
    prev_w = 0.0                                # 필터 변수 초기화
    print("="*52 + "\n")


def get_heading_recovery_cmd(heading_deg, front_scan_points):
    # 정지해서 도는 것이 아니라, 전진(v)하면서 부드럽게 곡선을 그리며 헤딩을 복귀시키기 위한 제어량(v, w)을 계산하는 함수입니다. (현재 메인루프 미사용)
    natural_sign = -1.0 if heading_deg > 0 else 1.0  # 헤딩 오차를 줄이는 최적의 회전 방향(부호)을 결정합니다.

    scan_dict = {}                              # 전방 라이다 데이터를 5도 버킷으로 담습니다.
    for angle_norm, dist in front_scan_points:
        if dist <= 40 or dist > DETECTION_RANGE: 
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    # 정면 기준 좌우 45도 범위 내의 장애물들이 모두 복귀 안전 거리(RECOVERY_SAFE_DIST) 이상 멀리 떨어져 있는지(정면이 열렸는지) 확인합니다.
    frontal_open = any(
        scan_dict.get(a, 0) >= RECOVERY_SAFE_DIST
        for a in range(-45, 50, ANGLE_STEP)
    )

    # 보정하려고 곡선으로 꺾으려는 방향(좌/우) 앞쪽에 방해물이 있는지 확인합니다.
    if natural_sign < 0:                        # 오른쪽으로 곡선을 틀 때
        correction_blocked = any(               # 오른쪽 전방 영역(5도~90도)에 가까운 장애물이 하나라도 있는지 검사합니다.
            0 < scan_dict.get(a, DETECTION_RANGE + 1) < RECOVERY_SAFE_DIST
            for a in range(ANGLE_STEP, SCAN_HALF_ANGLE + ANGLE_STEP, ANGLE_STEP)
        )
    else:                                       # 왼쪽으로 곡선을 틀 때
        correction_blocked = any(               # 왼쪽 전방 영역(-5도~-90도)에 가까운 장애물이 있는지 검사합니다.
            0 < scan_dict.get(a, DETECTION_RANGE + 1) < RECOVERY_SAFE_DIST
            for a in range(-ANGLE_STEP, -SCAN_HALF_ANGLE - ANGLE_STEP, -ANGLE_STEP)
        )

    corr_dir = "Right" if natural_sign < 0 else "Left" # 방향 문자열 매핑

    if frontal_open and not correction_blocked: # 정면도 넓고 꺾으려는 쪽도 넓으면
        print(f"  [HeadingRec-A] Fwd+CorrectRot({corr_dir})  v={FORWARD_SPEED:.2f} w={natural_sign*RECOVERY_W:.2f}")
        return FORWARD_SPEED, natural_sign * RECOVERY_W # 최고 속도로 직진하면서 보정용 각속도를 섞어 부드럽게 복귀 궤적을 만듭니다.
    elif frontal_open and correction_blocked:   # 정면은 뚫렸는데 꺾으려는 쪽에 장애물이 있으면
        print(f"  [HeadingRec] Fwd only (Correct dir {corr_dir} blocked)")
        return FORWARD_SPEED, 0.0               # 일단 회전하지 않고 직진만 하여 장애물을 피합니다.
    elif not frontal_open and not correction_blocked: # 정면은 막혔는데 옆구리는 뚫려있다면
        print(f"  [HeadingRec] In-place CorrectRot({corr_dir})  w={natural_sign*RECOVERY_W:.2f}")
        return 0.0, natural_sign * RECOVERY_W   # 직진을 멈추고(v=0) 그 자리에서 보정 방향으로 제자리 회전합니다.
    else:                                       # 정면도 막히고 꺾으려는 쪽도 막혔다면
        alt_dir = "Left" if natural_sign < 0 else "Right" 
        print(f"  [HeadingRec] Both blocked → Detour({alt_dir}) rot")
        return 0.0, -natural_sign * RECOVERY_W  # 일단 부딪히지 않기 위해 보정을 포기하고 반대 방향으로 제자리 우회 회전을 합니다.


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [알고리즘 3] 회피 방향 가중치 점수 평가
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def select_direction(left_clear, right_clear, heading_deg):
    # 왼쪽과 오른쪽의 빈 공간의 크기(각도)와 현재 헤딩 점수를 조합하여 최종적으로 장애물을 피할 방향(왼쪽/오른쪽)을 결정합니다.
    left_ok  = left_clear  >= MIN_VIABLE_CLEAR  # 왼쪽 연속 빈 공간이 로봇이 들어갈 수 있는 최소 폭(25도) 이상인지 평가합니다.
    right_ok = right_clear >= MIN_VIABLE_CLEAR  # 오른쪽 빈 공간이 25도 이상인지 평가합니다.

    # 갇힘 방지 특수 케이스 처리: 한쪽이 너무 좁으면 헤딩 점수가 어떻든 무조건 넓은 쪽으로 피하게 강제합니다.
    if left_ok and not right_ok:                # 왼쪽은 괜찮은데 오른쪽은 너무 좁다면
        print(f"  [Dir] Right blocked({right_clear}°) → Force Left")
        return 1.0                              # 무조건 왼쪽(1.0)으로 회전하도록 결정합니다.
    if right_ok and not left_ok:                # 오른쪽은 괜찮은데 왼쪽이 너무 좁다면
        print(f"  [Dir] Left blocked({left_clear}°) → Force Right")
        return -1.0                             # 무조건 오른쪽(-1.0)으로 회전하도록 결정합니다.
    if not left_ok and not right_ok:            # 양쪽 다 25도 미만으로 좁다면 (아주 좁은 골목)
        print(f"  [Dir] Both narrow → Select {'Left' if left_clear >= right_clear else 'Right'}")
        return 1.0 if left_clear >= right_clear else -1.0 # 어쩔 수 없이 1도라도 더 넓은 쪽을 울며 겨자먹기로 선택합니다.

    # 양쪽 다 통과할 만큼 충분히 넓다면, 공간의 여유도와 원래 궤도로 가려는 관성(헤딩)을 합산하여 점수를 매깁니다.
    # 왼쪽 점수 = 왼쪽 공간 여유도 + (헤딩이 음수(좌측 이탈)일 경우 원래 궤도로 가려는 왼쪽 가중치 점수 부여)
    left_score  = left_clear  + max(0.0, -heading_deg) * HEADING_WEIGHT
    # 오른쪽 점수 = 오른쪽 공간 여유도 + (헤딩이 양수(우측 이탈)일 경우 원래 궤도로 가려는 오른쪽 가중치 점수 부여)
    right_score = right_clear + max(0.0,  heading_deg) * HEADING_WEIGHT

    bonus_side = "R" if heading_deg > 0 else "L" # 보너스 점수가 어느 쪽에 들어갔는지 표시하기 위한 문자열
    bonus_val  = abs(heading_deg) * HEADING_WEIGHT # 실제 부여된 보너스 점수값
    # 계산된 좌/우 총점과 계산 내역을 출력합니다.
    print(f"  [DirScore] L={left_score:.0f}  R={right_score:.0f}  (Clear L={left_clear}° R={right_clear}°  HeadingBonus {bonus_side}+{bonus_val:.0f})")

    # 왼쪽 총점이 크거나 같으면 왼쪽(+1.0), 오른쪽이 더 크면 오른쪽(-1.0)을 최종 회피 방향으로 반환합니다.
    return 1.0 if left_score >= right_score else -1.0


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [알고리즘 4] 모터 제어 명령(v, w) 산출의 코어 로직
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def find_vw_command(scan_points, heading_deg):
    # 매 프레임마다 라이다 스캔 데이터와 헤딩을 바탕으로 아두이노에 보낼 전진 선속도(v)와 회전 각속도(w)를 최종적으로 도출합니다.
    global avoidance_w_sign, no_danger_count    # 회피 방향을 지속시키는 변수와, 평화로운 상태 지속 카운터를 가져옵니다.
    threshold = ROBOT_HALF_WIDTH + SAFETY_MARGIN # 로봇 중심선으로부터 피해야 할 장애물의 좌우 한계선(임계값, 110+10=120mm)을 설정합니다.

    # [1] 위협이 되는 장애물만 필터링하여 수집합니다.
    danger_points = []                          # 진짜 위험한 장애물 포인트만 담을 리스트입니다.
    for angle_norm, dist in scan_points:        # 전방 스캔 데이터를 모두 확인합니다.
        if dist <= 40 or dist > DETECTION_RANGE: # 센서 에러이거나 너무 멀리 있는(1.5m 초과) 데이터는 위협이 아니므로 버립니다.
            continue
        horiz, fwd = decompose(angle_norm, dist)# 극좌표계를 좌우(수평) 거리와 앞뒤(전방) 거리로 분리합니다.
        # 장애물이 내 앞쪽에 있고(fwd > 0), 전방 주시 구역(800mm) 안에 있으며, 좌우 한계선(120mm) 안으로 들어왔다면 (즉 로봇과 일직선상에서 충돌 예정이라면)
        if fwd > 0 and fwd <= FORWARD_RANGE and horiz < threshold:
            danger_points.append((angle_norm, dist, horiz, fwd)) # 위협 장애물 리스트에 추가합니다.

    NO_DANGER_RESET = 3                         # 3번(0.3초) 연속으로 전방에 장애물이 안 보여야 평화 상태로 완전히 인정합니다.
    if not danger_points:                       # 위협 리스트가 비어있다면 (내 경로 상에 아무것도 없다면)
        no_danger_count += 1                    # 평화 카운터를 1 증가시킵니다.
        if no_danger_count >= NO_DANGER_RESET:  # 평화가 3번 연속 유지되었다면
            avoidance_w_sign = 0.0              # 장애물을 피하기 위해 한 방향으로 꺾고 있던 관성(방향성)을 초기화시킵니다.
        return FORWARD_SPEED, 0.0               # 위협이 없으므로 최고 속도(FORWARD_SPEED)로 꺾지 않고 직진(w=0)하라고 반환합니다.
    no_danger_count = 0                         # 장애물이 단 한 개라도 감지되었다면 평화 카운터를 0으로 엎어버립니다.

    # [2] 기준 거리 계산 및 전진 선속도(v) 감속 비율 결정
    # 위협 장애물 중 전방 125mm, 수평 110mm 안에 들어온 '초근접(Stop)' 장애물들만 따로 모읍니다.
    stop_points = [p for p in danger_points if p[3] <= STOP_FWD_RANGE and p[2] <= STOP_HORIZ_RANGE]
    # 위협 장애물 중 측면보다 정면에 더 가까운 위치에 있는 장애물만 모읍니다.
    frontal   = [p for p in danger_points if p[3] >= p[2]]
    # 정면 장애물 중 가장 가까운 것의 앞뒤 거리를 구합니다. 없다면 감속 시작 거리(250mm)보다 조금 멀게 설정합니다.
    n_fwd_ref = min((p[3] for p in frontal), default=SLOW_START_DIST + 1)
    # 전체 위협 장애물 중 로봇 중심선(좌우)에 가장 바짝 붙은(horiz가 가장 작은) 장애물을 핵심 기준점으로 잡습니다.
    horiz_ref = min(danger_points, key=lambda p: p[2])
    nearest_angle, ref_dist, n_horiz, _ = horiz_ref # 핵심 기준점의 각도, 대각선 거리, 수평 거리 변수를 뽑아냅니다.

    print(f"  [Ref] Fwd:{n_fwd_ref:.0f}mm  Stop:{len(stop_points)}pts  Angle:{nearest_angle:.1f}°  Horiz:{n_horiz:.0f}mm")

    if stop_points:                             # 만약 초근접 구역(stop_points)에 장애물이 하나라도 들어왔다면
        v = 0.0                                 # 충돌 직전이므로 로봇의 직진 속도(v)를 즉시 0으로 만듭니다 (멈춤).
    elif n_fwd_ref >= SLOW_START_DIST:          # 가장 가까운 정면 장애물이 아직 감속 시작 거리(250mm)보다 멀리 있다면
        v = FORWARD_SPEED                       # 최고 속도(0.35 m/s)를 유지합니다.
    else:                                       # 장애물이 125mm ~ 250mm 사이에 위치하여 점차 멈춰야 하는 감속 구간이라면
        # 장애물이 가까워질수록 1.0에서 0.0으로 선형적으로 줄어드는 감속 비율(ratio)을 계산합니다.
        ratio = (n_fwd_ref - STOP_FWD_RANGE) / (SLOW_START_DIST - STOP_FWD_RANGE)
        v = max(FORWARD_SPEED * ratio, MIN_SPEED) # 계산된 비율을 곱해 속도를 줄이되, 최소 속도(0.07 m/s) 밑으로는 떨어지지 않게 하한선을 둡니다.

    # 로봇 중심 한계선(120mm)에서 기준 장애물이 좌우로 얼마나 깊숙이 침범했는지 '오차'를 구합니다.
    horiz_error = threshold - n_horiz           
    if horiz_error <= 0:                        # 한계선을 침범하지 않았다면 (수학적 예외 처리)
        avoidance_w_sign = 0.0                  # 회전할 필요 없이
        return v, 0.0                           # 직진만 하라고 반환합니다.

    # 좌우 여유 공간을 계산하기 위해 라이다 데이터를 5도 버킷 딕셔너리로 만듭니다.
    scan_dict = {}
    for angle_norm, dist in scan_points:
        if dist <= 40 or dist > DETECTION_RANGE: 
            continue
        bucket = round(angle_norm / ANGLE_STEP) * ANGLE_STEP
        if bucket not in scan_dict or dist < scan_dict[bucket]:
            scan_dict[bucket] = dist

    # [3] 방향(부호) 결정
    # 직진 속도(v)가 0이든 아니든, 항상 좌우 빈 공간을 먼저 계산하여 똑똑하게 방향을 정합니다.
    # (기존의 무조건 반대로 도망가는 'Direct decide' 로직을 삭제하여 좁은 길에서의 핑퐁 고착을 원천 차단합니다)
    left_clear = right_clear = 0 
    for a in range(-SCAN_HALF_ANGLE, 0, ANGLE_STEP): # 스캔의 왼쪽 절반(음수 각도)
        if scan_dict.get(a, DETECTION_RANGE + 1) >= ref_dist:
            left_clear += ANGLE_STEP 
    for a in range(ANGLE_STEP, SCAN_HALF_ANGLE + 1, ANGLE_STEP): # 오른쪽 절반(양수 각도)
        if scan_dict.get(a, DETECTION_RANGE + 1) >= ref_dist:
            right_clear += ANGLE_STEP 

    # 디버깅을 위해 정지 구역 발동 여부를 함께 출력합니다.
    if stop_points:
        print(f"  [StopZone] Active - Evaluating best clearance")
    
    print(f"  [Clearance] L:{left_clear}°  R:{right_clear}°  Heading:{heading_deg:.1f}°")

    if avoidance_w_sign == 0.0: 
        # 이전에 피하던 방향이 없었다면 (새로운 회피 시작)
        avoidance_w_sign = select_direction(left_clear, right_clear, heading_deg) 
        print(f"  [DirDecide] Locked to {'Left' if avoidance_w_sign>0 else 'Right'}")
    else: 
        # 이미 피하고 있던 중이라면 (방향 관성 존재)
        committed_clear = left_clear if avoidance_w_sign > 0 else right_clear 
        
        # 내가 피하려고 돌고 있는 쪽의 공간이 너무 비좁아졌다면(25도 미만) 방향 전환을 고려합니다.
        # (정지 상태이더라도 공간이 충분하다면 기존 방향으로 계속 안전하게 제자리 회전하며 빠져나갑니다)
        if committed_clear < MIN_VIABLE_CLEAR: 
            old = avoidance_w_sign 
            avoidance_w_sign = select_direction(left_clear, right_clear, heading_deg) 
            if avoidance_w_sign != old: 
                print(f"  [DirSwitch] Blocked({committed_clear}°) → {'Left' if avoidance_w_sign>0 else 'Right'}")

    # [4] 측면 안전 검사 (회전 시 옆구리 충돌 방지 클램프)
    def side_horiz_blocked(is_left):            # 특정 방향(좌/우)의 측면에 장애물이 바짝 붙어있는지 확인하는 내부 함수입니다.
        angles = (range(-ANGLE_STEP, -(SIDE_CHECK_ANGLE+ANGLE_STEP), -ANGLE_STEP) # 검사할 경우 왼쪽은 -5~-60도
                  if is_left else
                  range(ANGLE_STEP, SIDE_CHECK_ANGLE+ANGLE_STEP, ANGLE_STEP))     # 오른쪽은 +5~+60도 영역을 지정합니다.
        for a in angles:
            d = scan_dict.get(a, 0)
            if d <= 0: continue
            if d * abs(math.sin(math.radians(a))) < SIDE_ROTATE_SAFE: # 장애물의 측면 수직 거리가 안전 거리(150mm)보다 짧으면
                return True                     # 막혔음(True)을 반환합니다.
        return False                            # 다행히 측면이 뚫려있으면 False 반환

    left_close  = side_horiz_blocked(is_left=True)  # 돌기 전에 왼쪽 측면이 막혔는지 검사합니다.
    right_close = side_horiz_blocked(is_left=False) # 돌기 전에 오른쪽 측면이 막혔는지 검사합니다.
    
    if avoidance_w_sign > 0 and left_close and not right_close: # 왼쪽으로 돌려는데 왼쪽 측면에 뭐가 바짝 붙어있고 오른쪽은 비었다면
        print("  [SideBlock] Left → Force Right") 
        avoidance_w_sign = -1.0                 # 돌다가 긁히지 않도록 강제로 오른쪽(-1.0) 회전으로 부호를 뒤집습니다.
    elif avoidance_w_sign < 0 and right_close and not left_close: # 오른쪽으로 돌려는데 우측이 막히고 좌측이 비었다면
        print("  [SideBlock] Right → Force Left")
        avoidance_w_sign = 1.0                  # 좌회전으로 강제 전환합니다.

    # [5] P 제어를 통한 최종 각속도 크기(w_mag) 계산
    # 침범 오차(horiz_error)가 한계치(threshold) 대비 얼마나 큰지에 비례(W_GAIN)하여 회전 세기를 정합니다. 값이 MAX_W를 넘지 못하게 자릅니다.
    w_mag = max(min(W_GAIN * horiz_error / threshold, MAX_W), W_MIN_DANGER) # 오차가 작더라도 위험 상황에선 최소 W_MIN_DANGER(0.5) 속도로는 확실하게 돌아주도록 보장(max)합니다.
    w     = avoidance_w_sign * w_mag            # 확정된 부호(방향)에 계산된 세기를 곱해 최종 w 명령값을 산출합니다.

    print(f"  [Cmd] v:{v:.2f}  w:{w:.2f}  (HorizErr:{horiz_error:.0f}mm)") # 산출된 v, w 값을 터미널에 출력합니다.
    return v, w                                 # 메인 루프에서 사용할 수 있도록 v와 w를 반환합니다.


def main():
    global arduino_heading_deg, stuck_count, prev_w, avoidance_w_sign, stop_zone_entry_time, no_danger_count # 메인 상태 변수들을 전역으로 선언합니다.

    # 콘솔에 초기 설정값 안내 문구를 길게 출력하여 세팅이 잘 되었는지 확인시켜줍니다.
    print("=== RPLIDAR Obstacle Avoidance v5 ===")
    print(f"  LIDAR_PORT    : {LIDAR_PORT}")
    print(f"  ARDUINO_PORT  : {ARDUINO_PORT}")
    print(f"  LIDAR_OFFSET  : +{LIDAR_OFFSET}mm")
    print(f"  Danger Zone   : Fwd {FORWARD_RANGE}mm × Horiz {ROBOT_HALF_WIDTH+SAFETY_MARGIN}mm")
    print(f"  Speed         : Max {FORWARD_SPEED}m/s  Min {MIN_SPEED}m/s (No full stop)")
    print(f"  Lin Decel     : Decel from {SLOW_START_DIST}mm → Min speed in {STOP_FWD_RANGE}×{STOP_HORIZ_RANGE}mm zone")
    print(f"  Ang Control   : Horiz Err P-Control (Maintain while horiz < {ROBOT_HALF_WIDTH+SAFETY_MARGIN}mm)")
    print(f"  Block Detect  : Immed. escape rotation if fwd blocked")
    print(f"  Escape AngVel : {ESCAPE_W} rad/s (Optimal dir)")
    print("=" * 50)

    # 라이다와 아두이노 장치를 지정한 포트와 보드레이트(속도)로 엽니다. Timeout 1초를 설정하여 무한 대기를 막습니다.
    lidar   = serial.Serial(LIDAR_PORT,   BAUDRATE_LIDAR,   timeout=1)
    arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
    time.sleep(2)                               # 시리얼 포트가 열리고 안정화될 때까지 2초 기다립니다. (특히 아두이노는 포트 오픈 시 자동 리셋됨)

    lidar.write(bytes([0xA5, 0x40]))            # 라이다에게 모터 회전을 명령하는 바이너리 코드를 보냅니다.
    time.sleep(1)                               # 모터가 최고 속도로 돌 때까지 1초 기다립니다.
    lidar.write(bytes([0xA5, 0x20]))            # 라이다에게 레이저 스캔을 시작하고 데이터를 보내라고 명령합니다.
    print("Starting scan...")
    lidar.read(7)                               # 라이다가 스캔 시작 직후 보내는 7바이트짜리 응답 헤더 패킷을 읽어서 버퍼를 비웁니다.

    scan_points  = []                           # 한 바퀴(360도) 치의 라이다 점들을 모아둘 빈 리스트를 준비합니다.
    last_send    = time.time()                  # 아두이노에 제어 명령을 마지막으로 보낸 시간을 기록합니다.
    last_cmd_str = ""                           # 직전에 보낸 명령 문자열을 저장해 중복 출력을 방지합니다.

    try:                                        # Ctrl+C로 종료할 때 장치들을 안전하게 끄기 위해 try-except 블록을 사용합니다.
        while True:                             # 프로그램이 종료될 때까지 무한 반복하는 메인 루프입니다.
            read_arduino(arduino)               # 가장 먼저 아두이노 버퍼를 읽어 최신 헤딩 각도를 업데이트합니다.

            raw    = lidar.read(5)              # 라이다에서 들어오는 5바이트 점 데이터를 읽습니다.
            result = parse_packet(raw)          # 읽어온 바이너리를 파싱하여 각도, 거리로 만듭니다.
            if result is None:                  # 패킷이 깨졌다면
                continue                        # 이 점은 버리고 다음 데이터를 기다립니다.

            angle_raw, distance, quality = result # 정상적인 각도, 거리, 품질 데이터를 변수에 받습니다.
            s_flag = raw[0] & 0x01              # 첫 바이트에서 스캔 한 바퀴의 시작(동기화)을 알리는 s_flag를 추출합니다.

            if s_flag == 1 and scan_points:     # s_flag가 1이고 기존에 모아둔 점들이 있다면 (한 바퀴 360도 수집이 방금 완료됨을 의미)
                all_scan_points = list(scan_points) # 완성된 360도 전체 스냅샷을 복사해 둡니다 (탈출 분석 등에 사용). 
                front_points    = [             # 분석 속도를 높이기 위해, 360도 데이터 중 로봇 앞쪽 180도 데이터만 따로 추려냅니다.
                    (a, d) for a, d in scan_points
                    if is_in_front(a) and d > 0
                ]

                now = time.time()               # 현재 시간을 구합니다.
                if now - last_send >= SEND_INTERVAL: # 마지막으로 명령을 보낸 지 0.1초(SEND_INTERVAL) 이상이 지났다면 알고리즘을 굴립니다.

                    # ── ① 막힘 감지 → 탈출 시퀀스 ────
                    if is_path_blocked(front_points): # 추출해 둔 정면 180도 데이터를 넘겨 빠져나갈 폭이 있는지 확인합니다.
                        stuck_count += 1        # 막혔다는 판정이 나오면 누적 카운터를 1 올립니다.
                        print(f"  [BlockDetect] {stuck_count}/{STUCK_TRIGGER_COUNT} times")
                        if stuck_count >= STUCK_TRIGGER_COUNT: # 3번(0.3초) 연속으로 '진짜 갇혔다'고 판정이 내려지면
                            execute_escape_rotation(arduino, lidar, all_scan_points) # 동적 후진 및 제자리 탈출 회전 함수를 실행시킵니다.
                            
                            # 탈출이 끝난 후 다시 일반 주행을 시작해야 하니 변수들을 전부 0으로 리셋합니다.
                            stuck_count          = 0
                            avoidance_w_sign     = 0.0
                            stop_zone_entry_time = None
                            last_cmd_str = ""
                            last_send    = time.time()
                            scan_points  = []   # 그동안 라이다가 계속 돌아서 쌓인 쓰레기 데이터 버퍼도 싹 비워줍니다.
                            continue            # 이번 제어 주기를 건너뛰고 다음 데이터를 새롭게 받으러 루프 처음으로 갑니다.
                    else:                       # 통과할 폭이 충분하다고 판정되었는데
                        if stuck_count > 0:     # 기존에 누적된 막힘 카운터가 남아있다면
                            print(f"  [BlockCleared] Counter reset ({stuck_count} times)")
                        stuck_count = 0         # 막힘 판정이 취소된 것이므로 카운터를 0으로 깎아냅니다.

                    # ── ② 궤도 이탈 방지 (방향 보정) ──────────
                    if abs(arduino_heading_deg) > MISSION_HEADING_LIMIT: # 로봇이 요리조리 피하다가 원래 가려던 길에서 90도 넘게 틀어졌다면
                        execute_direction_correction(                    # 모든 전진을 멈추고 원래 궤도로 헤딩을 복귀시키는 함수를 돌립니다.
                            arduino, lidar, all_scan_points
                        )
                        avoidance_w_sign = 0.0  # 복귀 완료 후 제어 변수들을 리셋합니다.
                        last_cmd_str = ""
                        last_send    = time.time()
                        scan_points  = []
                        continue                # 마찬가지로 최신 데이터 확보를 위해 루프를 스킵합니다.

                    # ── ③ 정상 회피 명령 생성 및 송신 ────────────────────────────────────
                    v, w = find_vw_command(front_points, arduino_heading_deg) # 막히지도 않고 크게 이탈하지도 않았다면 코어 로직으로 정상 회피 (v, w) 속도를 계산합니다.

                    # 계산된 명령 중, 속도가 0이고 회전 각속도만 있으면(제자리에서 멈춰 돌고 있다면) 정지 구역에 들어간 것으로 판단합니다.
                    in_stop_zone = (v == 0.0 and abs(w) > 0.01)
                    if not in_stop_zone:        # 정지 상태가 아니라면
                        stop_zone_entry_time = None # (미사용 예비 타이머) 진입 시간을 초기화합니다.

                    # w 각속도 값이 갑자기 확 바뀌어 모터가 덜컥거리는 것을 방지하기 위해 LPF(로우패스필터)를 걸어 이전 명령값과 부드럽게 섞어줍니다.
                    w = W_SMOOTH * w + (1.0 - W_SMOOTH) * prev_w
                    prev_w = w                  # 다음 루프 필터링을 위해 현재 적용된 w 값을 저장합니다.

                    cmd = f"{v:.2f} {w:.2f}\n"  # 아두이노가 해석할 수 있도록 "v값 w값(엔터)" 형식의 텍스트 프로토콜로 문자열 조립합니다.
                    arduino.write(cmd.encode()) # 조립된 텍스트를 바이트 배열로 인코딩하여 아두이노 시리얼 포트로 전송합니다.

                    if cmd != last_cmd_str:     # 만약 직전(0.1초 전)에 보낸 명령과 소수점 둘째 자리까지의 값이 완전히 똑같지 않다면
                        print(f"[Send] v={v:.2f}  w={w:.2f}  Heading={arduino_heading_deg:.1f}°") # 갱신된 명령값과 로봇의 헤딩 상태를 터미널에 모니터링용으로 출력합니다.
                        last_cmd_str = cmd      # 중복 출력을 막기 위해 이번에 보낸 문자열을 백업해둡니다.

                    last_send = now             # 명령 송신 타이머를 현재 시간으로 갱신하여 다음 0.1초를 기다리게 합니다.

                scan_points = []                # 방금까지 모았던 360도 한 바퀴 데이터 처리가 끝났으니, 다음 한 바퀴 분량을 모으기 위해 버퍼를 비웁니다.

            # 읽은 데이터가 이번 바퀴(360도 스캔)의 점 중 하나라면, 각도를 정규화하고 라이다 보정 상수(20mm)를 더해 수집 버퍼 리스트에 차곡차곡 쌓아둡니다.
            scan_points.append((normalize_angle(angle_raw),
                                distance + LIDAR_OFFSET if distance > 0 else 0))

    except KeyboardInterrupt:                   # 터미널에서 사용자가 Ctrl+C를 눌러 강제 종료를 요청했다면
        print("\nShutting down...")             # 안전 종료 시퀀스로 넘어갑니다.
    finally:                                    # 프로그램이 에러가 나서 튕기거나 정상 종료될 때 반드시 실행되는 마무리 블록입니다.
        lidar.write(bytes([0xA5, 0x25]))        # 라이다 모터에 회전 중지 바이너리 명령을 보내서 센서를 끕니다.
        time.sleep(0.1)                         # 명령이 먹힐 때까지 잠시 대기
        lidar.close()                           # 점유하고 있던 라이다 시리얼 통신 포트를 해제합니다.
        arduino.write(b"0.00 0.00\n")           # 로봇이 미쳐 날뛰지 않도록 모터에 v=0, w=0 정지 명령을 막타로 보냅니다.
        arduino.close()                         # 점유하고 있던 아두이노 시리얼 통신 포트도 해제합니다.
        print("Shutdown complete.")             # 완전히 프로그램이 내려갔음을 알립니다.


if __name__ == "__main__":                      # 이 파이썬 파일이 모듈로 불려온 게 아니라 직접 실행(python main.py)되었을 때만
    main()                                      # 위에 짠 메인 루프 함수를 호출하여 프로그램을 시작하도록 하는 안전장치입니다.

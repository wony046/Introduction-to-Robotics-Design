# ── 1. 설정 및 파라미터 ─────────────────────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 9600

ROBOT_FRONT  = 110
ROBOT_BACK   = 150
ROBOT_HALF_W = 110
MARGIN       = 30       # 여유폭 약간 줄임 (35→30)

MAX_V = 0.35
MAX_W = 1.5

W_HEADING   = 2.0
W_CLEARANCE = 1.8
W_VELOCITY  = 1.0
BIAS_BONUS  = 0.3

# ── 4. DWA 코어 ────────────────────────────────────────────────

def generate_vw_window(current_v, current_w):
    # ★ 수정 1: 후보 더 촘촘하게, 저속 전진 추가
    v_cands = [0.0, 0.10, 0.15, 0.20, 0.25, MAX_V]
    w_cands = [-MAX_W, -1.2, -0.8, -0.5, -0.3, 0.0,
                0.3,   0.5,  0.8,  1.2,  MAX_W]
    return v_cands, w_cands


def check_collision_and_clearance(v_m_s, w_rad_s, scan_points,
                                   predict_t=0.6,   # ★ 수정 2: 1.0 → 0.6초
                                   step=0.1):        # ★ 수정 3: 0.2 → 0.1초 (정밀도↑)
    v_mm_s   = v_m_s * 1000.0
    max_dist = 1200.0

    local_pts = [
        (dist * math.cos(math.radians(ang)),
        -dist * math.sin(math.radians(ang)))
        for ang, dist in scan_points if 0 < dist <= max_dist
    ]
    if not local_pts:
        return 1000.0

    curr_x, curr_y, curr_th = 0.0, 0.0, 0.0
    t = 0.0
    min_clear_sq = 1000000.0

    front_bound = ROBOT_FRONT + MARGIN
    back_bound  = -(ROBOT_BACK  + MARGIN)
    side_bound  =  ROBOT_HALF_W + MARGIN

    while t <= predict_t:
        curr_x  += v_mm_s * math.cos(curr_th) * step
        curr_y  += v_mm_s * math.sin(curr_th) * step
        curr_th += w_rad_s * step
        t       += step

        cos_t = math.cos(-curr_th)
        sin_t = math.sin(-curr_th)

        for px, py in local_pts:
            dx, dy = px - curr_x, py - curr_y
            lx =  dx * cos_t - dy * sin_t
            ly =  dx * sin_t + dy * cos_t

            if back_bound <= lx <= front_bound and -side_bound <= ly <= side_bound:
                return -1.0

            dist_sq = dx**2 + dy**2
            if dist_sq < min_clear_sq:
                min_clear_sq = dist_sq

    return math.sqrt(min_clear_sq)


def run_dwa(scan_points, curr_heading, current_v, current_w):
    global last_w_sign
    v_cands, w_cands = generate_vw_window(current_v, current_w)

    best_v, best_w = 0.0, 0.0
    max_score      = -1.0

    for v in v_cands:
        for w in w_cands:
            clearance = check_collision_and_clearance(v, w, scan_points)
            if clearance <= 0:
                continue

            pred_turn   = math.degrees(w * 1.0)
            fut_heading = normalize_angle(curr_heading - pred_turn)

            score_heading   = max(0.0, 1.0 - (abs(fut_heading) / 180.0))
            score_clearance = min(1.0, clearance / 1000.0)
            score_velocity  = max(0.0, v / MAX_V)

            bias = BIAS_BONUS if (w * last_w_sign > 0) else 0.0

            # ★ 수정 4: v=0일 때 페널티 부여 → 웬만하면 정지 선택 안 함
            stop_penalty = -0.5 if (v == 0.0) else 0.0

            total_score = (W_HEADING   * score_heading   +
                           W_CLEARANCE * score_clearance +
                           W_VELOCITY  * score_velocity  +
                           bias + stop_penalty)

            if total_score > max_score:
                max_score      = total_score
                best_v, best_w = v, w

    # ★ 수정 5: 모든 전진이 막혔을 때 제자리 회전으로 fallback
    if best_v == 0.0 and best_w == 0.0:
        # 헤딩 기준으로 더 여유 있는 방향으로 제자리 회전
        best_w = MAX_W if curr_heading <= 0 else -MAX_W

    if best_w != 0:
        last_w_sign = 1.0 if best_w > 0 else -1.0

    return best_v, best_w

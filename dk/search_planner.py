"""
search_planner.py — 색지 탐색 플래너
  (후보 추종 + 퀸컹스 커버리지 스핀 + 색 좌표 메모리)

왜 스핀 스캔만으로는 부족한가:
  - 카메라 HFOV가 좁고(38.6°) 색지가 바닥에 누워 원근 압축됨
    → 멀고 약한 색지는 확정 검출에서 누락 → 한 지점 스핀의 명중 반경이 작다.
  - 4m²에 장애물 7개 → 어느 지점에서든 시야 일부가 가린다(occlusion).

전략 (우선순위 순):
  1) CANDIDATE 추종: 확정 검출은 없지만 후보(약한 신호)가 보이면
     그 방향으로 접근해 확정 검출을 유도한다. 헛것이면 가서 못 찾고 복귀.
     → 실질 탐지 반경이 늘어 스핀 명중률이 크게 오른다.
  2) MEMORY 직행: 미션이 넘어갈 때 그 색을 탐색 중 본 적이 있으면 그 좌표로.
  3) COVERAGE: 위 둘이 없으면 신뢰 탐지 반경 r로 경기장을 덮는
     퀸컹스(중심+4코너) 지점을 순회하며 각 지점에서 360° 스핀.

좌표계: jw_won.py 오도메트리와 동일.
  heading 0° = +y축,  x = dist*sin(hdg),  y = dist*cos(hdg),  bearing = atan2(dx, dy)
"""

import math
import time
import floor_verify   # 후보가 장애물(황토/남색)인지 라이다로 검증

# ── 측정 권장 파라미터 ─────────────────────────────────────────────────
DETECT_RADIUS_MM  = 900.0   # 한 지점 스핀으로 확정 검출 가능한 신뢰 반경 (실측)

# 퀸컹스 격자 코너가 중심(origin)에서 떨어진 거리.
# 경기장 중심을 알 수 없으므로 절대 좌표 클램프는 쓰지 않는다. 대신 격자
# 중심을 '목표를 잃은 위치'에 두고, 거기서 0.75m 떨어진 4 코너로 펼쳐
# 전체 1.5m × 1.5m 영역을 훑는다. 인접 점 간격 ≈1.06m로 DETECT_RADIUS
# 0.9m와 거의 겹쳐 빈틈이 적다.
QUINCUNX_RADIUS_MM = 750.0

SPIN_W            = 1.2     # 스핀 각속도 (rad/s) — 모션블러로 놓치지 않게 느리게
SPIN_TOTAL_DEG    = 380.0   # 1회 스핀 누적 회전각 (360 + 여유)

WP_ARRIVE_MM      = 300.0   # 웨이포인트/후보/메모리 도착 판정 거리
WP_TIMEOUT_SEC    = 12.0    # 이동 제한시간 (장애물로 막힐 때 포기)
CAND_TIMEOUT_SEC  = 8.0     # 후보 추종 제한시간 (헛것일 때 빠르게 포기)

# 한 번의 이동으로 현재 위치에서 벗어날 수 있는 최대 반경.
# 경기장 중심을 모르므로 절대 좌표로는 가둘 수 없다. 대신 "한 번에 멀리
# 가지 마라"로 제한한다 — 후보가 경기장 밖/헛것으로 멀리 잡혀도 이 반경까지만
# 다가가 재확인하므로 통제 불능으로 경기장을 벗어나지 않는다.
# (원래 가상경계도 경기장 중심이 아니라 '현재 위치 기준 원'이었음 → 같은 취지)
MAX_STEP_RADIUS_MM = 900.0

DEBUG             = 1

# ── 내부 상태 ─────────────────────────────────────────────────────────
_mode             = 'COVER'  # 'SPIN' | 'COVER' | 'CAND' | 'MEM'  (시작은 배회 우선)
_spin_accum_deg   = 0.0
_prev_heading     = None
_target_pos       = None
_move_start_time  = None
_origin           = None
_wp_index         = 0
_last_state_str   = ''


def _norm(a):
    return ((a + 180.0) % 360.0) - 180.0


def _quincunx(ox, oy):
    """중심(origin) + 4 코너 격자점.
    경기장 중심을 모르므로 절대 좌표 클램프 없음. 코너는 origin에서
    QUINCUNX_RADIUS_MM 떨어진 4 대각 방향에 둔다.
    경기장 밖으로 튀는 안전은 _clamp_step(MAX_STEP_RADIUS_MM)이 담당."""
    h = QUINCUNX_RADIUS_MM
    pts = [(ox, oy)]
    for sx in (+1, -1):
        for sy in (+1, -1):
            pts.append((ox + sx * h, oy + sy * h))
    return pts


def _polar_to_global(x, y, heading_deg, bearing_deg, dist_mm):
    g = math.radians(heading_deg + bearing_deg)
    return (x + dist_mm * math.sin(g), y + dist_mm * math.cos(g))


def notify_color_visible(x_mm=None, y_mm=None):
    """카메라가 목표 색을 확정 검출하는 동안 호출 — 탐색 상태 리셋.
    위치를 함께 받으면 그 좌표를 '목표를 마지막으로 본 곳'으로 기억해두고,
    다음번 색을 놓쳐 탐색이 재개될 때 격자 중심(origin)으로 쓴다."""
    global _mode, _spin_accum_deg, _prev_heading, _target_pos, _move_start_time
    global _origin, _wp_index
    _mode            = 'SPIN'
    _spin_accum_deg  = 0.0
    _prev_heading    = None
    _target_pos      = None
    _move_start_time = None
    if x_mm is not None and y_mm is not None:
        # 격자 중심을 '마지막 목표 위치'로 갱신하고 코너 순회를 처음부터.
        _origin   = (x_mm, y_mm)
        _wp_index = 0
        if DEBUG:
            print(f"[SEARCH] 목표 본 위치로 격자 중심 갱신 → "
                  f"({x_mm:.0f}, {y_mm:.0f})mm")


def _clamp_step(cur_x, cur_y, tx, ty, max_r=MAX_STEP_RADIUS_MM):
    """목표(tx,ty)가 현재 위치에서 max_r보다 멀면, 같은 방향으로 max_r 지점까지만.
    경기장 밖/헛것 후보로 멀리 튀어나가는 것을 막는다 (한 번에 한 발씩)."""
    dx, dy = tx - cur_x, ty - cur_y
    dist = math.hypot(dx, dy)
    if dist <= max_r or dist < 1e-6:
        return (tx, ty)
    s = max_r / dist
    return (cur_x + dx * s, cur_y + dy * s)


def _start_move(mode, pos, cur_x=None, cur_y=None):
    global _mode, _target_pos, _move_start_time
    if cur_x is not None and cur_y is not None:
        pos = _clamp_step(cur_x, cur_y, pos[0], pos[1])
    _mode            = mode
    _target_pos      = pos
    _move_start_time = time.time()


def _start_spin():
    global _mode, _spin_accum_deg, _prev_heading, _target_pos
    _mode           = 'SPIN'
    _spin_accum_deg = 0.0
    _prev_heading   = None
    _target_pos     = None


def _next_wp(x_mm, y_mm):
    """다음 커버리지 웨이포인트 선택.
    현재 위치에서 충분히 먼 것(WP_ARRIVE_MM*1.5 초과)을 순환하며 선택.
    모두 가깝다면 그 중 가장 먼 것을 선택 (제자리 맴돌기 방지)."""
    global _wp_index
    grid      = _quincunx(*_origin)
    best_idx  = (_wp_index + 1) % len(grid)
    best_dist = -1.0
    for i in range(len(grid)):
        idx = (_wp_index + 1 + i) % len(grid)
        d   = math.hypot(grid[idx][0] - x_mm, grid[idx][1] - y_mm)
        if d > WP_ARRIVE_MM * 1.5:
            _wp_index = idx
            return grid[idx]
        if d > best_dist:
            best_dist = d
            best_idx  = idx
    _wp_index = best_idx
    return grid[best_idx]


def update(x_mm, y_mm, heading_deg, camera, scan_points=None, pivot_ok=True):
    """
    목표 색 미감지 상태에서 매 제어 주기 호출.
    camera: camera_tracker 모듈
    scan_points: 라이다 스캔 [(angle, dist), ...] — 주어지면 후보를
                 floor_verify로 검증해 장애물 오인 후보를 추종하지 않는다.

    반환: (mode, target_bearing_deg, v, w)
      mode='SPIN' → (v, w)를 그대로 모터에 (제자리 회전)
      그 외      → target_bearing을 find_vw_command()에 전달
    """
    global _origin, _last_state_str, _spin_accum_deg, _prev_heading, _wp_index

    if _origin is None:
        _origin = (x_mm, y_mm)
        if DEBUG:
            print(f"[SEARCH] 격자 기준점: ({x_mm:.0f}, {y_mm:.0f})mm")

    state = camera.get_state()
    color = state.replace('SEEK_', '')

    # (2) 미션 전환 시 기억 좌표 직행
    if state != _last_state_str:
        _last_state_str = state
        mem = camera.get_seen_other(color)
        if mem is not None:
            mb, md = mem
            pos = _polar_to_global(x_mm, y_mm, heading_deg, mb, md)
            camera.clear_seen(color)
            _start_move('MEM', pos, x_mm, y_mm)
            if DEBUG:
                print(f"[SEARCH] {color} 기억 좌표 직행 → "
                      f"({pos[0]:.0f}, {pos[1]:.0f})mm")

    # (1) 후보 추종: 탐색 중 후보 포착 시 접근 — 단, 라이다 검증 통과한 후보만
    if _mode in ('SPIN', 'COVER'):
        cand = camera.get_candidate()
        if cand is not None:
            cb, cd = cand
            cand_ok = True
            if scan_points:
                cand_ok = floor_verify.is_floor_paper(scan_points, cb, cd)
                if not cand_ok and DEBUG:
                    print(f"[SEARCH] 후보 기각(장애물 추정) b={cb:+.1f}° d={cd:.0f}mm")
            if cand_ok:
                pos = _polar_to_global(x_mm, y_mm, heading_deg, cb, cd)
                _start_move('CAND', pos, x_mm, y_mm)
                if DEBUG:
                    print(f"[SEARCH] 후보 포착 b={cb:+.1f}° d={cd:.0f}mm → 접근")

    # SPIN: 제자리 회전
    if _mode == 'SPIN':
        # pivot_ok=False면 이번 주기엔 장애물 회피로 제자리 회전을 못 했다는 뜻.
        # 회피 주행으로 생긴 헤딩 변화를 스핀 진행으로 오카운트하면 한 바퀴를
        # 다 돌기 전에 스캔을 끝낸 것으로 착각하므로, 그런 주기엔 누적하지 않고
        # 기준 헤딩만 갱신한다(다음 실제 회전부터 정확히 누적).
        if _prev_heading is not None and pivot_ok:
            _spin_accum_deg += abs(_norm(heading_deg - _prev_heading))
        _prev_heading = heading_deg

        if _spin_accum_deg < SPIN_TOTAL_DEG:
            return ('SPIN', 0.0, 0.0, SPIN_W)

        cand_pt = _next_wp(x_mm, y_mm)
        _start_move('COVER', cand_pt)
        if DEBUG:
            print(f"[SEARCH] 스핀 완료(미발견) → 커버리지 "
                  f"({cand_pt[0]:.0f}, {cand_pt[1]:.0f})mm")

    # COVER 모드: 목표 미설정 시 첫 커버리지 포인트 선택 (초기 배회)
    if _mode == 'COVER' and _target_pos is None:
        cand_pt = _next_wp(x_mm, y_mm)
        _start_move('COVER', cand_pt)
        if DEBUG:
            print(f"[SEARCH] 초기 배회 → 커버리지 ({cand_pt[0]:.0f}, {cand_pt[1]:.0f})mm")

    # 이동 모드: 목표 좌표로 회피 주행
    tx, ty  = _target_pos
    dx, dy  = tx - x_mm, ty - y_mm
    dist    = math.hypot(dx, dy)
    limit   = CAND_TIMEOUT_SEC if _mode == 'CAND' else WP_TIMEOUT_SEC
    timeout = (time.time() - _move_start_time) > limit

    if dist < WP_ARRIVE_MM or timeout:
        if DEBUG:
            why = '도착' if dist < WP_ARRIVE_MM else '타임아웃'
            print(f"[SEARCH] {_mode} {why} → 스핀")
        _start_spin()
        return ('SPIN', 0.0, 0.0, SPIN_W)

    bearing_global = math.degrees(math.atan2(dx, dy))
    rel_bearing    = _norm(bearing_global - heading_deg)
    return (_mode, rel_bearing, None, None)

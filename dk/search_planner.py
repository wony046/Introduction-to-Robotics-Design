"""
search_planner.py — 색지 탐색 플래너
  (후보 추종 + 퀸컹스 커버리지 + 색 좌표 메모리)

전략 (우선순위 순):
  1) CANDIDATE 추종: 확정 검출은 없지만 후보(약한 신호)가 보이면
     그 방향으로 접근해 확정 검출을 유도한다.
  2) MEMORY 직행: 미션이 넘어갈 때 그 색을 탐색 중 본 적이 있으면 그 좌표로.
  3) COVERAGE: 위 둘이 없으면 퀸컹스(중심+4코너) 지점을 순회.

좌표계: jw_won.py 오도메트리와 동일.
  heading 0° = +y축,  x = dist*sin(hdg),  y = dist*cos(hdg),  bearing = atan2(dx, dy)
"""

import math
import time
import floor_verify   # 후보가 장애물(황토/남색)인지 라이다로 검증

# ── 측정 권장 파라미터 ─────────────────────────────────────────────────
DETECT_RADIUS_MM  = 900.0   # 한 지점 스핀으로 확정 검출 가능한 신뢰 반경 (실측)
ARENA_HALF_MM     = 900.0   # 경기장 절반폭(중심 기준). 2m 경기장이면 900~1000.

WP_ARRIVE_MM      = 300.0   # 웨이포인트/후보/메모리 도착 판정 거리
WP_TIMEOUT_SEC    = 12.0    # 이동 제한시간 (장애물로 막힐 때 포기)
CAND_TIMEOUT_SEC  = 8.0     # 후보 추종 제한시간 (헛것일 때 빠르게 포기)

DEBUG             = 1

# ── 내부 상태 ─────────────────────────────────────────────────────────
_mode             = 'COVER'  # 'COVER' | 'CAND' | 'MEM'
_target_pos       = None
_move_start_time  = None
_origin           = None
_wp_index         = 0
_last_state_str   = ''


def _norm(a):
    return ((a + 180.0) % 360.0) - 180.0


def _quincunx(ox, oy):
    """중심 + 4코너 격자점."""
    h = min(ARENA_HALF_MM, DETECT_RADIUS_MM)
    pts = [(ox, oy)]
    for sx in (+1, -1):
        for sy in (+1, -1):
            pts.append((ox + sx * h, oy + sy * h))
    return pts


def _polar_to_global(x, y, heading_deg, bearing_deg, dist_mm):
    g = math.radians(heading_deg + bearing_deg)
    return (x + dist_mm * math.sin(g), y + dist_mm * math.cos(g))


def notify_color_visible():
    """카메라가 목표 색을 확정 검출하는 동안 호출 — 탐색 상태 리셋."""
    global _mode, _target_pos, _move_start_time
    _mode            = 'COVER'
    _target_pos      = None
    _move_start_time = None


def _start_move(mode, pos):
    global _mode, _target_pos, _move_start_time
    _mode            = mode
    _target_pos      = pos
    _move_start_time = time.time()


def _next_cover_point(x_mm, y_mm):
    """퀸컹스 격자에서 다음 커버리지 포인트 선택."""
    global _wp_index
    grid = _quincunx(*_origin)
    _wp_index = (_wp_index + 1) % len(grid)
    for _ in range(len(grid)):
        cand_pt = grid[_wp_index]
        if math.hypot(cand_pt[0] - x_mm, cand_pt[1] - y_mm) > WP_ARRIVE_MM * 1.5:
            break
        _wp_index = (_wp_index + 1) % len(grid)
    return cand_pt


def update(x_mm, y_mm, heading_deg, camera, scan_points=None):
    """
    목표 색 미감지 상태에서 매 제어 주기 호출.
    camera: camera_tracker 모듈
    scan_points: 라이다 스캔 [(angle, dist), ...] — 주어지면 후보를
                 floor_verify로 검증해 장애물 오인 후보를 추종하지 않는다.

    반환: (mode, target_bearing_deg, v, w)
      target_bearing을 find_vw_command()에 전달 (v, w는 None)
    """
    global _origin, _last_state_str, _wp_index

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
            _start_move('MEM', pos)
            if DEBUG:
                print(f"[SEARCH] {color} 기억 좌표 직행 → "
                      f"({pos[0]:.0f}, {pos[1]:.0f})mm")

    # (1) 후보 추종: 탐색 중 후보 포착 시 접근 — 단, 라이다 검증 통과한 후보만
    if _mode == 'COVER':
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
                _start_move('CAND', pos)
                if DEBUG:
                    print(f"[SEARCH] 후보 포착 b={cb:+.1f}° d={cd:.0f}mm → 접근")

    # COVER 모드: 목표 미설정 시 다음 커버리지 포인트 선택
    if _mode == 'COVER' and _target_pos is None:
        cand_pt = _next_cover_point(x_mm, y_mm)
        _start_move('COVER', cand_pt)
        if DEBUG:
            print(f"[SEARCH] 커버리지 → ({cand_pt[0]:.0f}, {cand_pt[1]:.0f})mm")

    # 이동 모드: 목표 좌표로 회피 주행
    tx, ty  = _target_pos
    dx, dy  = tx - x_mm, ty - y_mm
    dist    = math.hypot(dx, dy)
    limit   = CAND_TIMEOUT_SEC if _mode == 'CAND' else WP_TIMEOUT_SEC
    timeout = (time.time() - _move_start_time) > limit

    if dist < WP_ARRIVE_MM or timeout:
        why = '도착' if dist < WP_ARRIVE_MM else '타임아웃'
        if DEBUG:
            print(f"[SEARCH] {_mode} {why} → 다음 커버리지")
        cand_pt = _next_cover_point(x_mm, y_mm)
        _start_move('COVER', cand_pt)
        if DEBUG:
            print(f"[SEARCH] 커버리지 → ({cand_pt[0]:.0f}, {cand_pt[1]:.0f})mm")
        tx, ty = cand_pt
        dx, dy = tx - x_mm, ty - y_mm

    bearing_global = math.degrees(math.atan2(dx, dy))
    rel_bearing    = _norm(bearing_global - heading_deg)
    return (_mode, rel_bearing, None, None)

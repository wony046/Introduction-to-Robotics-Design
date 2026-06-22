# -*- coding: utf-8 -*-
"""
project3_sim.py  ─  Project 3 주행 시뮬레이터 (Phase 1)

sim_logic.py 의 LiDAR 회피 로직을 그대로 호출해서, 가상 2m×2m 아레나에서
로봇이 장애물을 피하며 주행하는 모습을 시각화한다. 실제 하드웨어와 "동일한
부호 규약"으로 scan / heading 을 공급하므로, 알고리즘의 동작(정상이든 버그든)이
그대로 재현된다 → 디버깅용.

규약 (sim_logic.py 와 일치):
  · 월드 좌표 : 아레나 중심 = 원점, y 위쪽(+), 단위 mm
  · heading  : deg. 0 = +y(화면 위) 정면, +방향 = CCW(물리적 좌회전)
  · LiDAR a  : 0=정면, + = 물리적 우측, - = 좌측  (heading 과 반대 부호)
  · target_bearing : +가 좌측 (Phase 1 에서는 0 = 직진 유지)

기하 유도:
  forward_world(h) = (-sin h,  cos h)
  right_world(h)   = ( cos h,  sin h)
  로봇프레임 (lateral, fwd) → world = pos + right*lateral + forward*fwd
  LiDAR a 의 world 방향 = (sin(a-h), cos(a-h))      [a,h: deg→rad]

Phase 1 범위 : 에디터(장애물/패치/로봇) + 화면 이동/줌 + LiDAR 레이캐스트 +
회피 주행 + 시각화(레이어/STOP/갭/경계). 카메라·미션·CLOSE·접근경로(req9)는 Phase 2~3.

실행:  pip install pygame  후  python project3_sim.py
헤드리스 자가검증:  python project3_sim.py --selftest 120
"""

import math
import sys
import pygame
import sim_logic as S
import sim_camera as CAM

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  설정
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
WIN_W, WIN_H = 1300, 880
PANEL_W      = 330
MAP_W        = WIN_W - PANEL_W          # 지도 영역 폭
MAP_CX       = MAP_W // 2               # 지도 중심(스크린)
MAP_CY       = WIN_H // 2

ARENA_HALF   = 1000.0                   # 2m × 2m → ±1000mm
SIM_DT       = 0.05                     # 고정 timestep (20Hz)
V_TO_MM      = 1000.0                   # v[m/s]*dt*1000 = mm
PAN_PX       = 26                       # 방향키 1회 이동 픽셀
ROT_STEP     = 5.0                      # Q/E 회전 step(deg)
SCAN_HALF    = 180                      # 스캔 생성 범위 ±deg (1° 간격)

START_POSE   = (0.0, -800.0, 0.0)       # 시작: 아래쪽 중앙, 정면 위

HFOV_HALF    = CAM.HFOV_DEG / 2.0       # 감지콘 반각 (deg)

# 색
C_BG        = (24, 26, 32)
C_PANEL     = (16, 17, 21)
C_ARENA     = (64, 70, 86)
C_GRID      = (36, 40, 50)
C_OBST      = (92, 98, 112)
C_OBST_LINE = (140, 148, 166)
C_SEL       = (224, 184, 84)
C_ROBOT     = (70, 200, 122)
C_HEAD      = (240, 240, 240)
C_LIDAR     = (120, 220, 236)
C_GAP_PASS  = (80, 220, 200)
C_GAP_BLOCK = (164, 78, 78)
C_GAP_PICK  = (84, 240, 116)
C_FRONT     = (96, 156, 250)
C_INTENT    = (245, 220, 92)
C_BOUND     = (232, 152, 72)
C_TEXT      = (226, 229, 236)
C_DIM       = (140, 146, 158)
C_BTN       = (44, 48, 60)
C_BTN_ON    = (66, 110, 90)
C_BTN_HOV   = (58, 63, 78)

PATCH_RGB = {'R': (206, 64, 64), 'Y': (212, 192, 64), 'B': (72, 112, 212)}
PATCH_FULL = {'R': 'RED', 'Y': 'YELLOW', 'B': 'BLUE'}   # 패치 코드 → 미션 색명


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  기하 / 좌표 변환  (pygame 불필요 → 단독 테스트 가능)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def fwd_world(h_deg):
    r = math.radians(h_deg)
    return (-math.sin(r), math.cos(r))

def right_world(h_deg):
    r = math.radians(h_deg)
    return (math.cos(r), math.sin(r))

def robot_pt(robot, lateral, fwd):
    """로봇프레임 (lateral=+우, fwd=+전방) → world(mm)."""
    fx, fy = fwd_world(robot['h'])
    rx, ry = right_world(robot['h'])
    return (robot['x'] + rx * lateral + fx * fwd,
            robot['y'] + ry * lateral + fy * fwd)

def lidar_dir(a_deg, h_deg):
    """LiDAR 각 a(+우) 의 world 방향 단위벡터."""
    r = math.radians(a_deg - h_deg)
    return (math.sin(r), math.cos(r))

def w2s(wx, wy, cam, zoom):
    """world(mm) → screen(px). cam=화면중심의 world좌표, zoom=px/mm."""
    return (MAP_CX + (wx - cam[0]) * zoom,
            MAP_CY - (wy - cam[1]) * zoom)

def s2w(sx, sy, cam, zoom):
    return (cam[0] + (sx - MAP_CX) / zoom,
            cam[1] - (sy - MAP_CY) / zoom)


def ray_aabb(ox, oy, dx, dy, minx, miny, maxx, maxy):
    """원점 밖에서 쏜 ray 의 첫 교차 거리 t(mm). 원점이 박스 안이면 None(무시)."""
    if minx <= ox <= maxx and miny <= oy <= maxy:
        return None
    tmin, tmax = 0.0, float('inf')
    for o, d, lo, hi in ((ox, dx, minx, maxx), (oy, dy, miny, maxy)):
        if abs(d) < 1e-9:
            if o < lo or o > hi:
                return None
        else:
            t1 = (lo - o) / d
            t2 = (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return None
    if tmax < 0:
        return None
    return tmin if tmin >= 0 else None


def ray_obb(ox, oy, dx, dy, ob):
    """회전 장애물(OBB) 레이캐스트. ray 를 장애물 로컬 프레임으로 변환 후 AABB 재사용.
    회전은 거리를 보존하므로 반환 t 는 world 거리와 동일."""
    a = ob.get('a', 0.0)
    if not a:
        return ray_aabb(ox, oy, dx, dy,
                        ob['x'], ob['y'], ob['x'] + ob['w'], ob['y'] + ob['h'])
    cx = ob['x'] + ob['w'] / 2.0
    cy = ob['y'] + ob['h'] / 2.0
    rad = math.radians(a)
    c, s = math.cos(rad), math.sin(rad)
    # world → local: 회전 -a (R(-a) = [[c, s], [-s, c]])
    lx  = c * (ox - cx) + s * (oy - cy)
    ly  = -s * (ox - cx) + c * (oy - cy)
    ldx = c * dx + s * dy
    ldy = -s * dx + c * dy
    hw, hh = ob['w'] / 2.0, ob['h'] / 2.0
    return ray_aabb(lx, ly, ldx, ldy, -hw, -hh, hw, hh)


def obstacle_corners(ob):
    """장애물 4모서리 world 좌표 [TL,TR,BR,BL] (로컬→R(+a)→world)."""
    cx = ob['x'] + ob['w'] / 2.0
    cy = ob['y'] + ob['h'] / 2.0
    hw, hh = ob['w'] / 2.0, ob['h'] / 2.0
    rad = math.radians(ob.get('a', 0.0))
    c, s = math.cos(rad), math.sin(rad)
    local = [(-hw, hh), (hw, hh), (hw, -hh), (-hw, -hh)]   # 화면 기준 TL,TR,BR,BL
    return [(cx + c * lx - s * ly, cy + s * lx + c * ly) for lx, ly in local]


def point_in_obb(px, py, ob):
    """점이 회전 장애물 내부인지."""
    a = ob.get('a', 0.0)
    cx = ob['x'] + ob['w'] / 2.0
    cy = ob['y'] + ob['h'] / 2.0
    if not a:
        return ob['x'] <= px <= ob['x'] + ob['w'] and ob['y'] <= py <= ob['y'] + ob['h']
    rad = math.radians(a)
    c, s = math.cos(rad), math.sin(rad)
    lx = c * (px - cx) + s * (py - cy)
    ly = -s * (px - cx) + c * (py - cy)
    return abs(lx) <= ob['w'] / 2.0 and abs(ly) <= ob['h'] / 2.0


def generate_scan(robot, obstacles):
    """가상 LiDAR: ±SCAN_HALF° 를 1° 간격으로 모든 장애물 AABB 에 레이캐스트.
       패치는 평면이라 감지 안 함. 아레나 경계도 벽이 아님 → 장애물만."""
    scan = []
    rx, ry, h = robot['x'], robot['y'], robot['h']
    rng = S.DETECTION_RANGE
    for a in range(-SCAN_HALF, SCAN_HALF):
        dx, dy = lidar_dir(float(a), h)
        best = None
        for ob in obstacles:
            t = ray_obb(rx, ry, dx, dy, ob)
            if t is not None and (best is None or t < best):
                best = t
        if best is not None and best <= rng:
            scan.append((S.normalize_angle(float(a)), best))
    return scan


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  ★ 슬라이더 정의 (key, label, min, max)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SLIDER_DEFS = [
    ('FORWARD_SPEED',       'FWD speed',     0.05, 0.6),
    ('CLOSE_SPEED_MAX',     'CLOSE speed',   0.05, 0.4),
    ('MAX_W',               'MAX w',         0.5,  3.0),
    ('KP_GOAL',             'KP goal',       0.005, 0.15),
    ('KP_CLOSE_HDG',        'KP close hdg',  0.01, 0.3),
    ('STOP_FWD_MIN',        'STOP fwd min',  50,   200),
    ('STOP_FWD_MAX',        'STOP fwd max',  100,  300),
    ('STOP_HORIZ_TH',       'STOP horiz',    60,   200),
    ('STOP_ESCAPE_MIN_GAP', 'min passage',   150,  400),
    ('DETECTION_RANGE',     'detect range',  500,  2500),
    ('BOUNDARY_RADIUS',     'bound radius',  500,  2500),
    ('CLOSE_ENTER_MM',      'CLOSE enter',   200,  800),
    ('CLOSE_ARRIVE_MM',     'CLOSE arrive',  10,   100),
]
# 정수로 다뤄야 자연스러운 파라미터
INT_KEYS = {'STOP_FWD_MIN', 'STOP_FWD_MAX', 'STOP_HORIZ_TH', 'STOP_ESCAPE_MIN_GAP',
            'DETECTION_RANGE', 'CLOSE_ARRIVE_MM'}


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  경량 위젯 (패널 좌표 = 절대 rect, 그릴 때/판정 때 scroll 만큼 시프트)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Widget:
    def __init__(self, x, y, w, h):
        self.rect = pygame.Rect(x, y, w, h)

    def shifted(self, scroll):
        return self.rect.move(0, -scroll)


class Button(Widget):
    def __init__(self, x, y, w, h, label, group=None, value=None):
        super().__init__(x, y, w, h)
        self.label = label
        self.group = group     # 같은 group 이면 라디오처럼 동작
        self.value = value     # 콜백 식별자
        self.active = False

    def draw(self, surf, font, scroll, mouse):
        r = self.shifted(scroll)
        hov = r.collidepoint(mouse)
        col = C_BTN_ON if self.active else (C_BTN_HOV if hov else C_BTN)
        pygame.draw.rect(surf, col, r, border_radius=5)
        pygame.draw.rect(surf, (12, 13, 16), r, 1, border_radius=5)
        t = font.render(self.label, True, C_TEXT)
        surf.blit(t, (r.centerx - t.get_width() // 2,
                      r.centery - t.get_height() // 2))

    def hit(self, pos, scroll):
        return self.shifted(scroll).collidepoint(pos)


class Checkbox(Widget):
    def __init__(self, x, y, w, label, checked=True):
        super().__init__(x, y, w, 22)
        self.label = label
        self.checked = checked

    def draw(self, surf, font, scroll, mouse):
        r = self.shifted(scroll)
        box = pygame.Rect(r.x, r.y + 2, 16, 16)
        pygame.draw.rect(surf, (40, 44, 56), box, border_radius=3)
        pygame.draw.rect(surf, (90, 96, 110), box, 1, border_radius=3)
        if self.checked:
            pygame.draw.line(surf, (110, 220, 150), (box.x + 3, box.y + 8),
                             (box.x + 6, box.y + 12), 2)
            pygame.draw.line(surf, (110, 220, 150), (box.x + 6, box.y + 12),
                             (box.x + 13, box.y + 3), 2)
        t = font.render(self.label, True, C_TEXT)
        surf.blit(t, (box.right + 7, r.y + 3))

    def hit(self, pos, scroll):
        return self.shifted(scroll).collidepoint(pos)


class Slider(Widget):
    H = 34

    def __init__(self, x, y, w, key, label, lo, hi):
        super().__init__(x, y, w, Slider.H)
        self.key, self.label, self.lo, self.hi = key, label, lo, hi
        self.val = float(getattr(S, key))
        self.drag = False

    def _track(self, scroll):
        r = self.shifted(scroll)
        return pygame.Rect(r.x + 6, r.y + 22, r.w - 12, 6)

    def _knob_x(self, scroll):
        tr = self._track(scroll)
        f = (self.val - self.lo) / (self.hi - self.lo)
        f = max(0.0, min(1.0, f))
        return tr.x + int(f * tr.w)

    def draw(self, surf, font, scroll, mouse):
        r = self.shifted(scroll)
        disp = (f"{int(round(self.val))}" if self.key in INT_KEYS
                else f"{self.val:.3f}")
        lab = font.render(self.label, True, C_DIM)
        val = font.render(disp, True, C_TEXT)
        surf.blit(lab, (r.x + 4, r.y + 2))
        surf.blit(val, (r.right - val.get_width() - 4, r.y + 2))
        tr = self._track(scroll)
        pygame.draw.rect(surf, (44, 48, 60), tr, border_radius=3)
        kx = self._knob_x(scroll)
        fill = pygame.Rect(tr.x, tr.y, kx - tr.x, tr.h)
        pygame.draw.rect(surf, (74, 120, 160), fill, border_radius=3)
        pygame.draw.circle(surf, (210, 216, 226), (kx, tr.centery), 7)
        pygame.draw.circle(surf, (40, 44, 56), (kx, tr.centery), 7, 1)

    def hit(self, pos, scroll):
        tr = self._track(scroll)
        grab = pygame.Rect(tr.x - 8, tr.y - 8, tr.w + 16, tr.h + 16)
        return grab.collidepoint(pos)

    def set_from_x(self, mx, scroll):
        tr = self._track(scroll)
        f = (mx - tr.x) / max(1, tr.w)
        f = max(0.0, min(1.0, f))
        self.val = self.lo + f * (self.hi - self.lo)
        if self.key in INT_KEYS:
            self.val = float(round(self.val))


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  시뮬레이터 본체
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Sim:
    def __init__(self):
        # 월드 상태
        self.obstacles = []          # {'x','y','w','h'}  (min corner, mm)
        self.patches   = []          # {'x','y','w','h','color'}
        self.robot     = {'x': START_POSE[0], 'y': START_POSE[1], 'h': START_POSE[2]}
        self.start_pose = dict(self.robot)

        # 뷰
        self.cam  = [0.0, 0.0]
        self.zoom = 0.40             # px/mm (아레나 2m 가 화면에 적당히 들어옴)

        # 실행 상태
        self.running = False
        self.last_vw = (0.0, 0.0)
        self.scan    = []
        self.viz     = None
        self.prev_w  = 0.0
        self.speed   = 1.0           # 시뮬 속도 배율 (프레임당 step 수 환산)
        self._step_accum = 0.0

        # ── Phase 2: 카메라/미션/CLOSE 상태 ──
        self.detection = None        # 이번 프레임 기하 감지 결과(또는 None)
        self._close_target_x = None  # arduino 프레임 mm
        self._close_target_y = None
        self._close_initial_dist = None
        self._close_observe_elapsed = 0.0
        self.close_trail = []        # CLOSE 접근 중 로봇 world 경로 (req9 시각화)
        # ── SEEK 색지 소실 시 색지 위치 추정 + 오도메트리 헤딩 계산 추종 ──
        # 색 감지 중 (bearing, 추정거리)로 색지의 좌표(arduino 프레임)를 추정·저장.
        # 소실 시 현재 오도메트리 위치(x,y)·헤딩으로 그 점까지 헤딩을 매 프레임 재계산
        # (회전+병진 모두 보정)하여 마지막으로 본 색지 위치를 계속 추종. (CLOSE 와 동일 방식)
        self._seek_target_x = None   # mm(arduino): 추정 색지 x
        self._seek_target_y = None   # mm(arduino): 추정 색지 y
        self._seek_src = None        # 'TRACK'/'EST_POS'/'BOUNDARY'/'STRAIGHT' (HUD/viz)
        self._seek_tb  = 0.0         # 추종에 사용한 상대 bearing (viz)
        CAM.reset()
        S.reset_odom_state()

        # 에디터
        self.tool = 'select'         # select / obstacle / patch / robot
        self.patch_color = 'R'
        self.sel = None              # ('obstacle', i) / ('patch', i) / ('robot', None)
        self.drag_mode = None        # 'move' / 'resize' / 'create'
        self.drag_off  = (0.0, 0.0)
        self.resize_corner = 0
        self.create_anchor = None    # world (x,y) 시작점

        # 패널
        self.panel_scroll = 0
        self.panel_max    = 0
        self._build_widgets()
        self.apply_sliders()

    # ── 위젯 배치 ─────────────────────────────────────────────
    def _build_widgets(self):
        x = MAP_W + 14
        w = PANEL_W - 28
        y = 40
        self.tool_btns = []
        self.sim_btns  = []
        self.toggles   = {}
        self.sliders   = []

        # 도구
        bw = (w - 8) // 2
        self.tool_btns.append(Button(x, y, bw, 26, 'Select', 'tool', 'select'))
        self.tool_btns.append(Button(x + bw + 8, y, bw, 26, 'Robot', 'tool', 'robot'))
        y += 32
        self.tool_btns.append(Button(x, y, bw, 26, 'Obstacle', 'tool', 'obstacle'))
        self.tool_btns.append(Button(x + bw + 8, y, bw, 26, 'Patch', 'tool', 'patch'))
        y += 32
        cw = (w - 16) // 3
        self.tool_btns.append(Button(x, y, cw, 24, 'R', 'pcolor', 'R'))
        self.tool_btns.append(Button(x + cw + 8, y, cw, 24, 'Y', 'pcolor', 'Y'))
        self.tool_btns.append(Button(x + 2 * (cw + 8), y, cw, 24, 'B', 'pcolor', 'B'))
        y += 34

        # 실행
        tw = (w - 16) // 3
        self.sim_btns.append(Button(x, y, tw, 28, 'Start', 'sim', 'start'))
        self.sim_btns.append(Button(x + tw + 8, y, tw, 28, 'Stop', 'sim', 'stop'))
        self.sim_btns.append(Button(x + 2 * (tw + 8), y, tw, 28, 'Restart', 'sim', 'restart'))
        y += 34

        # 속도 배율
        self.speed_btns = []
        sw = (w - 24) // 4
        for k, (lab, val) in enumerate([('0.5x', 0.5), ('1x', 1.0), ('2x', 2.0), ('4x', 4.0)]):
            self.speed_btns.append(Button(x + k * (sw + 8), y, sw, 24, lab, 'speed', val))
        y += 34

        # 토글
        defs = [('camera', 'Camera FOV/detect', True),
                ('preview', 'Camera view box', True),
                ('approach', 'CLOSE approach', True),
                ('layers', 'Layer zones', True),
                ('stop', 'STOP zone', True),
                ('lidar', 'LiDAR points', True),
                ('gaps', 'Gaps / escape', True),
                ('intent', 'Intent (v,w)', True),
                ('bound', 'Boundary circle', True),
                ('grid', 'Grid', True)]
        for key, lab, on in defs:
            self.toggles[key] = Checkbox(x, y, w, lab, on)
            y += 22
        y += 10

        # 슬라이더
        for key, lab, lo, hi in SLIDER_DEFS:
            self.sliders.append(Slider(x, y, w, key, lab, lo, hi))
            y += Slider.H
        self.panel_max = max(0, y + 20 - WIN_H)

    def all_panel_widgets(self):
        for b in self.tool_btns: yield b
        for b in self.sim_btns:  yield b
        for b in self.speed_btns: yield b
        for c in self.toggles.values(): yield c
        for s in self.sliders:   yield s

    # ── 파라미터 ─────────────────────────────────────────────
    def apply_sliders(self):
        S.apply_params({s.key: s.val for s in self.sliders})

    # ── 실행 제어 ─────────────────────────────────────────────
    def start(self):
        self.start_pose = dict(self.robot)
        S.reset_state()
        S.reset_odom_state()
        CAM.reset()
        self._close_reset()
        self._seek_reset()
        self.prev_w = 0.0
        self.close_trail = []
        self.running = True

    def stop(self):
        self.running = False

    def restart(self):
        self.robot = dict(self.start_pose)
        S.reset_state()
        S.reset_odom_state()
        CAM.reset()
        self._close_reset()
        self._seek_reset()
        self.prev_w = 0.0
        self.last_vw = (0.0, 0.0)
        self.close_trail = []
        self.running = False

    def _close_reset(self):
        self._close_target_x = self._close_target_y = self._close_initial_dist = None
        self._close_observe_elapsed = 0.0

    def _seek_reset(self):
        """SEEK 색지 위치 추정 폐기 (색지 도달/재시작 시). 다음 색은 새 추정으로 탐색."""
        self._seek_target_x = None
        self._seek_target_y = None
        self._seek_src = None
        self._seek_tb  = 0.0

    # ── 기하 카메라 감지 (모델 b: FOV콘 + 범위 + occlusion, true bearing/dist) ──
    def _bearing_to(self, px, py):
        """로봇 기준 (px,py) 의 bearing(deg, 좌+, camera/heading 규약)."""
        vx, vy = px - self.robot['x'], py - self.robot['y']
        fx, fy = fwd_world(self.robot['h'])         # 전방
        rx, ry = right_world(self.robot['h'])       # 우측
        fwd_c  = vx * fx + vy * fy
        left_c = -(vx * rx + vy * ry)               # 좌측 성분 = -우측
        return math.degrees(math.atan2(left_c, fwd_c))

    def _project_to_cam(self, wx, wy):
        """world 바닥점 (wx,wy) → 카메라 이미지 픽셀 (cx,cy). camera_tracker 투영모델.
        카메라 뒤/지평선 위(depression<=0)면 None.
          cx = EFF_W/2 - f_px*tan(bearing)            (bearing 좌+)
          depression = atan2(CAM_HEIGHT, ground_dist)
          cy = EFF_H/2 + f_px*tan(depression - tilt)
        """
        vx, vy = wx - self.robot['x'], wy - self.robot['y']
        fx, fy = fwd_world(self.robot['h'])
        rx, ry = right_world(self.robot['h'])
        fwd_c  = vx * fx + vy * fy
        if fwd_c <= 1.0:                       # 카메라 뒤
            return None
        left_c = -(vx * rx + vy * ry)
        bearing = math.atan2(left_c, fwd_c)    # rad, 좌+
        gdist   = math.hypot(vx, vy)
        fp      = CAM.f_px()
        cx = CAM.EFF_W / 2.0 - fp * math.tan(bearing)
        depression = math.atan2(CAM.CAM_HEIGHT_MM, gdist)   # rad, >0
        delta_v = depression - math.radians(CAM.CAM_TILT_DEG)
        cy = CAM.EFF_H / 2.0 + fp * math.tan(delta_v)
        return (cx, cy)

    def _occluded(self, px, py, dist):
        """로봇→(px,py) 선분이 장애물 AABB 에 가리면 True."""
        if dist < 1e-6:
            return False
        dx, dy = (px - self.robot['x']) / dist, (py - self.robot['y']) / dist
        for ob in self.obstacles:
            t = ray_obb(self.robot['x'], self.robot['y'], dx, dy, ob)
            if t is not None and t < dist - 1.0:
                return True
        return False

    def _detect_target(self):
        """현재 미션 타깃 색 패치 중 FOV+범위+비가림 조건을 만족하는 가장 가까운 것.
        반환: {'bearing','distance','is_close','patch_idx'} 또는 None."""
        if CAM.is_done():
            return None
        color = CAM.get_target_color()           # 'RED'/'YELLOW'/'BLUE'
        best = None
        for i, p in enumerate(self.patches):
            if PATCH_FULL.get(p['color']) != color:   # 'R'→'RED' 매핑 비교
                continue
            cx = p['x'] + p['w'] / 2.0
            cy = p['y'] + p['h'] / 2.0
            dist = math.hypot(cx - self.robot['x'], cy - self.robot['y'])
            if dist < CAM.CAM_NEAR_MM or dist > CAM.CAM_FAR_MM:
                continue
            bearing = self._bearing_to(cx, cy)
            if abs(bearing) > HFOV_HALF:
                continue
            if self._occluded(cx, cy, dist):
                continue
            if best is None or dist < best[0]:
                best = (dist, bearing, i)
        if best is None:
            return None
        dist, bearing, idx = best
        return {'bearing': bearing, 'distance': dist,
                'is_close': dist < S.CLOSE_ENTER_MM, 'patch_idx': idx}

    def _set_arduino_odom(self):
        """월드(우+X) → arduino 프레임(좌+X) 미러링하여 전역 세팅."""
        S.arduino_heading_deg = self.robot['h']
        S.arduino_x_mm        = -self.robot['x']
        S.arduino_y_mm        =  self.robot['y']

    # ── 오케스트레이션 (jw_won._motor_controller 본문 이식) ──────────────────
    def _orchestrate(self):
        """DWELL/DONE > CLOSE > SEEK 3-상태. (v, w_raw) 반환.
        self.prev_w 는 분기 내에서 jw_won 과 동일하게 갱신(아래 스무딩 정합용)."""
        # 상태 1: DWELL / DONE → 정지
        if CAM.is_done() or CAM.is_dwelling():
            self.prev_w = 0.0
            self._close_reset()
            self._seek_reset()      # 색지 도달 → 직전 색 추종 기억 폐기
            return 0.0, 0.0

        # 상태 2: CLOSE → 관측 후 오도메트리 위치 제어
        if CAM.is_close() or self._close_target_x is not None:
            # 2a: 관측 단계 (CLOSE_OBSERVE_SEC 동안 정지)
            if self._close_target_x is None:
                self._close_observe_elapsed += SIM_DT
                if self._close_observe_elapsed < S.CLOSE_OBSERVE_SEC:
                    self.prev_w = 0.0
                    return 0.0, 0.0
                # 관측 완료 → 목표 좌표 확정 (true bearing/dist 사용)
                cb = CAM.get_last_close_bearing()
                dm = CAM.get_estimated_distance_mm()
                self._close_target_x, self._close_target_y = S.compute_close_target(cb, dm)
                self._close_initial_dist = None
                self.close_trail = [(self.robot['x'], self.robot['y'])]

            ex = self._close_target_x - S.arduino_x_mm
            ey = self._close_target_y - S.arduino_y_mm
            dist_err = math.sqrt(ex ** 2 + ey ** 2)
            if self._close_initial_dist is None:
                self._close_initial_dist = max(dist_err, 1.0)

            if dist_err < S.CLOSE_ARRIVE_MM:
                self.prev_w = 0.0
                CAM.signal_arrival()
                return 0.0, 0.0
            else:
                target_hdg = math.degrees(math.atan2(ex, ey))
                hdg_err    = S.normalize_angle(target_hdg - S.arduino_heading_deg)
                w = max(min(S.KP_CLOSE_HDG * hdg_err, S.MAX_W), -S.MAX_W)
                v = S.CLOSE_SPEED_MAX
                self.prev_w = w     # CLOSE 내 스무딩 관성 제거 (아래에서 identity)
                return v, w

        # 상태 3: SEEK → 카메라 bearing + 라이다 회피 / 미감지 시 추정 색지 위치 추종
        self._close_reset()
        bearing = CAM.get_bearing()
        if bearing is not None:
            S.clear_boundary_center()
            # 색지 위치 추정: (bearing, 추정거리) → 색지 좌표(arduino 프레임).
            # 현재 오도메트리 위치/헤딩 기준으로 환산 → 소실 시 이 점을 계속 추종.
            dist = CAM.get_estimated_distance_mm()
            self._seek_target_x, self._seek_target_y = \
                S.compute_close_target(bearing, dist)
            self._seek_src, self._seek_tb = 'TRACK', bearing
            return S.find_vw_command(self.scan, S.arduino_heading_deg, bearing)
        else:
            # 색 미감지: 추정 색지 위치까지의 헤딩을 오도메트리(x,y,heading)로 매 프레임
            # 재계산(회전+병진 보정)해 추종. 경계 초과 시엔 중심 복귀가 우선(안전).
            S.set_boundary_center()
            tb, v_scale, exceeded = S.get_boundary_correction()
            if exceeded:
                seek_tb, self._seek_src = tb, 'BOUNDARY'   # 경계 밖 → 중심 복귀
            elif self._seek_target_x is not None:
                # 오도메트리 헤딩 계산: 추정점까지 글로벌 헤딩 → 로봇 상대 bearing
                ex = self._seek_target_x - S.arduino_x_mm
                ey = self._seek_target_y - S.arduino_y_mm
                target_hdg = math.degrees(math.atan2(ex, ey))
                seek_tb = S.normalize_angle(target_hdg - S.arduino_heading_deg)
                self._seek_src = 'EST_POS'                 # 위치 추정 기반 추종
            else:
                seek_tb, self._seek_src = 0.0, 'STRAIGHT'  # 추정 없음 → 직진
            self._seek_tb = seek_tb
            v, w = S.find_vw_command(self.scan, S.arduino_heading_deg, seek_tb)
            return v * v_scale, w

    # ── 한 스텝 ──────────────────────────────────────────────
    def step(self):
        self.scan = generate_scan(self.robot, self.obstacles)
        self.detection = self._detect_target()       # 시각화/HUD 용 (항상 갱신)

        if self.running:
            self._set_arduino_odom()
            CAM.update(self.detection, SIM_DT)
            v, w_raw = self._orchestrate()
            # 최종 스무딩 (jw_won 하단부와 동일): CLOSE/정지는 prev_w 가 분기에서
            # 세팅돼 identity, SEEK 만 직전 w 와 블렌딩.
            w = S.W_SMOOTH * w_raw + (1.0 - S.W_SMOOTH) * self.prev_w
            self.prev_w = w
            self.last_vw = (v, w)
            # 적분 (월드 프레임)
            self.robot['h'] = S.normalize_angle(self.robot['h'] + math.degrees(w * SIM_DT))
            fx, fy = fwd_world(self.robot['h'])
            d = v * SIM_DT * V_TO_MM
            self.robot['x'] += fx * d
            self.robot['y'] += fy * d
            self._set_arduino_odom()                  # 적분 후 갱신(시각화 일관)
            # CLOSE 경로 기록
            if self._close_target_x is not None:
                self.close_trail.append((self.robot['x'], self.robot['y']))
                if len(self.close_trail) > 400:
                    self.close_trail.pop(0)

        # 회피 시각화용 분석 (target_bearing 은 표시 목적상 감지 bearing 또는 0)
        tb_viz = self.detection['bearing'] if self.detection else 0.0
        self.viz = S.analyze_scan(self.scan, self.robot['h'], tb_viz)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  렌더링
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def world_rect_to_screen(x, y, w, h, cam, zoom):
    """world AABB(min corner x,y / size w,h) → pygame.Rect(screen)."""
    sx0, sy0 = w2s(x, y + h, cam, zoom)     # 좌상(최대 y)
    sx1, sy1 = w2s(x + w, y, cam, zoom)     # 우하
    return pygame.Rect(int(sx0), int(sy0), int(sx1 - sx0), int(sy1 - sy0))


def draw_map(screen, overlay, sim, font, font_s):
    cam, zoom = sim.cam, sim.zoom
    tg = sim.toggles
    overlay.fill((0, 0, 0, 0))

    # 그리드 (0.5m 간격)
    if tg['grid'].checked:
        step = 500
        n = int(ARENA_HALF // step)
        for i in range(-n, n + 1):
            x0, y0 = w2s(i * step, -ARENA_HALF, cam, zoom)
            x1, y1 = w2s(i * step, ARENA_HALF, cam, zoom)
            pygame.draw.line(screen, C_GRID, (x0, y0), (x1, y1), 1)
            x0, y0 = w2s(-ARENA_HALF, i * step, cam, zoom)
            x1, y1 = w2s(ARENA_HALF, i * step, cam, zoom)
            pygame.draw.line(screen, C_GRID, (x0, y0), (x1, y1), 1)

    # 아레나 경계 (벽 아님, 참조용)
    ar = world_rect_to_screen(-ARENA_HALF, -ARENA_HALF, 2 * ARENA_HALF, 2 * ARENA_HALF, cam, zoom)
    pygame.draw.rect(screen, C_ARENA, ar, 2)

    # 패치 (바닥 색, LiDAR 미감지)
    for i, p in enumerate(sim.patches):
        r = world_rect_to_screen(p['x'], p['y'], p['w'], p['h'], cam, zoom)
        col = PATCH_RGB[p['color']]
        s = pygame.Surface((max(1, r.w), max(1, r.h)), pygame.SRCALPHA)
        s.fill((*col, 120))
        overlay.blit(s, (r.x, r.y))
        pygame.draw.rect(screen, col, r, 2)
        if sim.sel == ('patch', i):
            pygame.draw.rect(screen, C_SEL, r.inflate(6, 6), 2)

    # 장애물 (회전 지원: 4모서리 폴리곤)
    for i, ob in enumerate(sim.obstacles):
        corners = [w2s(px, py, cam, zoom) for px, py in obstacle_corners(ob)]
        pygame.draw.polygon(screen, C_OBST, corners)
        pygame.draw.polygon(screen, C_OBST_LINE, corners, 1)
        if sim.sel == ('obstacle', i):
            pygame.draw.polygon(screen, C_SEL, corners, 2)
            if not ob.get('a', 0.0):     # 핸들은 미회전 시에만 (a=0)
                for cx, cy in corners:
                    pygame.draw.rect(screen, C_SEL, (cx - 4, cy - 4, 8, 8))

    robot = sim.robot
    rpx, rpy = w2s(robot['x'], robot['y'], cam, zoom)

    # 경계 원 (동적 중심: 색 미감지 전환 시 찍힌 위치. arduino 프레임 → world 미러)
    if tg['bound'].checked:
        if S._boundary_center_x is not None:
            bcx, bcy = -S._boundary_center_x, S._boundary_center_y    # 미러 복원
            ox, oy = w2s(bcx, bcy, cam, zoom)
            pygame.draw.circle(screen, C_BOUND, (int(ox), int(oy)),
                               int(S.BOUNDARY_RADIUS * zoom), 1)
            pygame.draw.line(screen, C_BOUND, (ox - 5, oy), (ox + 5, oy), 1)
            pygame.draw.line(screen, C_BOUND, (ox, oy - 5), (ox, oy + 5), 1)
        else:
            # 미설정: 현재 위치 기준 미리보기 (희미하게)
            pygame.draw.circle(screen, (90, 70, 45), (int(rpx), int(rpy)),
                               int(S.BOUNDARY_RADIUS * zoom), 1)

    # 카메라 FOV 콘 + 감지 패치 강조
    if tg['camera'].checked:
        def _edge(beta, r):
            a = robot['h'] + beta
            fx, fy = fwd_world(a)
            return (robot['x'] + fx * r, robot['y'] + fy * r)
        nl = _edge(+HFOV_HALF, CAM.CAM_NEAR_MM); fl = _edge(+HFOV_HALF, CAM.CAM_FAR_MM)
        fr = _edge(-HFOV_HALF, CAM.CAM_FAR_MM);  nr = _edge(-HFOV_HALF, CAM.CAM_NEAR_MM)
        cone = [w2s(px, py, cam, zoom) for px, py in (nl, fl, fr, nr)]
        detected = sim.detection is not None
        pygame.draw.polygon(overlay, (235, 230, 120, 34) if detected else (150, 150, 160, 22), cone)
        pygame.draw.polygon(screen, (210, 205, 110) if detected else (110, 112, 120), cone, 1)
        # 감지된 패치 강조 + 로봇→패치 선
        if detected:
            di = sim.detection['patch_idx']
            if 0 <= di < len(sim.patches):
                p = sim.patches[di]
                pcx, pcy = p['x'] + p['w'] / 2.0, p['y'] + p['h'] / 2.0
                spx, spy = w2s(pcx, pcy, cam, zoom)
                pygame.draw.line(screen, (240, 235, 130), (rpx, rpy), (spx, spy), 1)
                pygame.draw.circle(screen, (250, 245, 140), (int(spx), int(spy)),
                                   max(6, int(max(p['w'], p['h']) * 0.5 * zoom)) + 4, 2)

    # SEEK 색지 소실 → 추정 색지 위치(오도메트리 헤딩 계산) 추종 시각화
    if (tg['camera'].checked and sim.running and sim._seek_src == 'EST_POS'
            and sim._seek_target_x is not None):
        tx, ty = -sim._seek_target_x, sim._seek_target_y      # arduino → world 미러
        stx, sty = w2s(tx, ty, cam, zoom)
        pygame.draw.line(screen, (255, 110, 180), (rpx, rpy), (stx, sty), 2)
        _arrow_head(screen, (rpx, rpy), (stx, sty), (255, 110, 180))
        # 추정 색지 위치 마커 (◇)
        for a, b in (((-8, 0), (0, -8)), ((0, -8), (8, 0)),
                     ((8, 0), (0, 8)), ((0, 8), (-8, 0))):
            pygame.draw.line(screen, (255, 150, 200),
                             (stx + a[0], sty + a[1]), (stx + b[0], sty + b[1]), 2)

    # CLOSE 접근 경로 (req9): 목표점 + 추종 궤적
    if tg['approach'].checked and sim._close_target_x is not None:
        tx, ty = -sim._close_target_x, sim._close_target_y       # arduino → world 미러
        stx, sty = w2s(tx, ty, cam, zoom)
        if len(sim.close_trail) >= 2:
            pts = [w2s(px, py, cam, zoom) for px, py in sim.close_trail]
            pygame.draw.lines(screen, (250, 130, 200), False, pts, 2)
        pygame.draw.line(screen, (250, 160, 210), (rpx, rpy), (stx, sty), 1)
        # 목표 X 마커
        pygame.draw.line(screen, (255, 120, 210), (stx - 7, sty - 7), (stx + 7, sty + 7), 2)
        pygame.draw.line(screen, (255, 120, 210), (stx - 7, sty + 7), (stx + 7, sty - 7), 2)
        pygame.draw.circle(screen, (255, 120, 210), (int(stx), int(sty)),
                           max(3, int(S.CLOSE_ARRIVE_MM * zoom)), 1)

    # 레이어 zone
    if tg['layers'].checked and sim.viz:
        active = sim.viz['active_names']
        for L in S.LAYERS:
            th = L['horiz_th']
            pts = [robot_pt(robot, -th, L['fwd_min']),
                   robot_pt(robot,  th, L['fwd_min']),
                   robot_pt(robot,  th, L['fwd_max']),
                   robot_pt(robot, -th, L['fwd_max'])]
            sp = [w2s(px, py, cam, zoom) for px, py in pts]
            on = L['name'] in active
            col = (255, 150, 60, 70) if on else (70, 90, 120, 26)
            pygame.draw.polygon(overlay, col, sp)
            pygame.draw.polygon(screen, (255, 170, 90) if on else (60, 76, 100), sp, 1)

    # STOP zone
    if tg['stop'].checked and sim.viz:
        th = S.STOP_HORIZ_TH
        pts = [robot_pt(robot, -th, S.STOP_FWD_MIN),
               robot_pt(robot,  th, S.STOP_FWD_MIN),
               robot_pt(robot,  th, S.STOP_FWD_MAX),
               robot_pt(robot, -th, S.STOP_FWD_MAX)]
        sp = [w2s(px, py, cam, zoom) for px, py in pts]
        trig = sim.viz['stop_triggered']
        pygame.draw.polygon(overlay, (220, 60, 60, 95) if trig else (150, 60, 60, 30), sp)
        pygame.draw.polygon(screen, (240, 80, 80) if trig else (150, 70, 70), sp, 2 if trig else 1)

    # LiDAR 점
    if tg['lidar'].checked:
        for a, d in sim.scan:
            wx = robot['x'] + math.sin(math.radians(a - robot['h'])) * d
            wy = robot['y'] + math.cos(math.radians(a - robot['h'])) * d
            sx, sy = w2s(wx, wy, cam, zoom)
            screen.set_at((int(sx), int(sy)), C_LIDAR)
            pygame.draw.circle(screen, C_LIDAR, (int(sx), int(sy)), 1)

    # 갭 / 탈출
    if tg['gaps'].checked and sim.viz:
        viz = sim.viz
        for g in viz['gap_info']:
            ca = g['center_angle']
            dirx, diry = lidar_dir(ca, robot['h'])
            ln = max(120.0, min(g['depth'], S.DETECTION_RANGE))
            ex, ey = robot['x'] + dirx * ln, robot['y'] + diry * ln
            sx, sy = w2s(ex, ey, cam, zoom)
            if g.get('chosen'):
                col, wdt = C_GAP_PICK, 3
            elif g['passable']:
                col, wdt = C_GAP_PASS, 1
            else:
                col, wdt = C_GAP_BLOCK, 1
            pygame.draw.line(screen, col, (rpx, rpy), (sx, sy), wdt)
        # 정상주행 타깃 front gap
        cf = viz['chosen_front']
        if cf is not None:
            ca = cf['center_angle']
            dirx, diry = lidar_dir(ca, robot['h'])
            ln = max(150.0, min(cf['depth'], S.DETECTION_RANGE))
            ex, ey = robot['x'] + dirx * ln, robot['y'] + diry * ln
            sx, sy = w2s(ex, ey, cam, zoom)
            pygame.draw.line(screen, C_FRONT, (rpx, rpy), (sx, sy), 2)

    # 로봇
    pygame.draw.circle(screen, C_ROBOT, (int(rpx), int(rpy)),
                       max(3, int(S.ROBOT_HALF_WIDTH * zoom)))
    pygame.draw.circle(screen, (20, 60, 40), (int(rpx), int(rpy)),
                       max(3, int(S.ROBOT_HALF_WIDTH * zoom)), 1)
    hx, hy = robot_pt(robot, 0, S.ROBOT_HALF_WIDTH + 120)
    shx, shy = w2s(hx, hy, cam, zoom)
    pygame.draw.line(screen, C_HEAD, (rpx, rpy), (shx, shy), 2)
    if sim.sel == ('robot', None):
        pygame.draw.circle(screen, C_SEL, (int(rpx), int(rpy)),
                           max(6, int(S.ROBOT_HALF_WIDTH * zoom)) + 5, 2)

    # 의도 화살표 (v,w 출력 시각화)
    if tg['intent'].checked:
        v, w = sim.last_vw
        # 전방 성분
        flen = 60 + v * 600
        fx2, fy2 = robot_pt(robot, 0, flen)
        sfx, sfy = w2s(fx2, fy2, cam, zoom)
        pygame.draw.line(screen, C_INTENT, (rpx, rpy), (sfx, sfy), 2)
        _arrow_head(screen, (rpx, rpy), (sfx, sfy), C_INTENT)
        # 회전 방향(부호) : 전방벡터를 sign(w) 만큼 살짝 회전
        turn = max(-50.0, min(50.0, math.degrees(w) * 0.6))
        # +w = 좌회전 → heading+turn 방향
        hh = robot['h'] + turn
        tfx, tfy = fwd_world(hh)
        tx = robot['x'] + tfx * (flen * 0.7)
        ty = robot['y'] + tfy * (flen * 0.7)
        stx, sty = w2s(tx, ty, cam, zoom)
        pygame.draw.line(screen, (250, 180, 70), (rpx, rpy), (stx, sty), 1)

    screen.blit(overlay, (0, 0))


def _arrow_head(surf, p0, p1, col):
    ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    for off in (math.radians(150), -math.radians(150)):
        x = p1[0] + 9 * math.cos(ang + off)
        y = p1[1] + 9 * math.sin(ang + off)
        pygame.draw.line(surf, col, p1, (x, y), 2)


def draw_panel(screen, sim, font, font_s):
    panel = pygame.Rect(MAP_W, 0, PANEL_W, WIN_H)
    pygame.draw.rect(screen, C_PANEL, panel)
    pygame.draw.line(screen, (8, 9, 11), (MAP_W, 0), (MAP_W, WIN_H), 2)
    mouse = pygame.mouse.get_pos()
    sc = sim.panel_scroll
    prev_clip = screen.get_clip()
    screen.set_clip(panel)

    def hdr(text, y):
        t = font_s.render(text, True, C_DIM)
        screen.blit(t, (MAP_W + 14, y - sc))

    hdr('TOOL', 26)
    for b in sim.tool_btns:
        b.draw(screen, font_s, sc, mouse)
    hdr('RUN', sim.sim_btns[0].rect.y - 14)
    for b in sim.sim_btns:
        b.draw(screen, font_s, sc, mouse)
    hdr('SPEED', sim.speed_btns[0].rect.y - 14)
    for b in sim.speed_btns:
        b.draw(screen, font_s, sc, mouse)
    first_tg = next(iter(sim.toggles.values()))
    hdr('VIEW', first_tg.rect.y - 14)
    for c in sim.toggles.values():
        c.draw(screen, font_s, sc, mouse)
    hdr('PARAMS  (\u2605 live tuning)', sim.sliders[0].rect.y - 14)
    for s in sim.sliders:
        s.draw(screen, font_s, sc, mouse)

    screen.set_clip(prev_clip)


def draw_camera_preview(screen, sim, font_s):
    """카메라가 보는 영상을 별도 박스로 렌더. 색지를 이미지 픽셀(cx,cy)로 투영.
    프레임 폭 전체 = 수평 FOV(±HFOV/2). 가까운 색지=하단·큼, 먼 색지=상단·작음."""
    EW, EH = CAM.EFF_W, CAM.EFF_H
    pw = 168
    ph = int(pw * EH / EW)
    title_h = 18
    px0 = MAP_W - pw - 12
    py0 = 12

    outer = pygame.Rect(px0, py0, pw, title_h + ph)
    pygame.draw.rect(screen, (16, 18, 24), outer, border_radius=4)
    pygame.draw.rect(screen, (70, 76, 92), outer, 1, border_radius=4)
    tgt = CAM.get_target_color()
    tcol = PATCH_RGB.get(tgt, (200, 200, 200))
    screen.blit(font_s.render('CAMERA  (HFOV %.0f\u00b0)' % CAM.HFOV_DEG, True, (170, 176, 190)),
                (px0 + 6, py0 + 2))
    pygame.draw.circle(screen, tcol, (px0 + pw - 12, py0 + 9), 5)

    img = pygame.Rect(px0, py0 + title_h, pw, ph)
    pygame.draw.rect(screen, (8, 9, 12), img)

    def to_scr(cx, cy):
        sx = px0 + cx / EW * pw
        sy = py0 + title_h + cy / EH * ph
        sx = max(px0 - 2 * pw, min(px0 + 3 * pw, sx))   # 폭주 방지 클램프
        sy = max(py0 - 2 * ph, min(py0 + 3 * ph, sy))
        return (int(sx), int(sy))

    prev_clip = screen.get_clip()
    screen.set_clip(img)

    # 중심선 (광축)
    cxs = to_scr(EW / 2.0, 0)[0]
    pygame.draw.line(screen, (44, 48, 60), (cxs, img.top), (cxs, img.bottom), 1)
    cys = to_scr(0, EH / 2.0)[1]
    pygame.draw.line(screen, (32, 35, 44), (img.left, cys), (img.right, cys), 1)

    # 색지 투영 (front + 비가림 + |bearing|<35° 인 것)
    det_idx = sim.detection['patch_idx'] if sim.detection else None
    for i, p in enumerate(sim.patches):
        cxw = p['x'] + p['w'] / 2.0
        cyw = p['y'] + p['h'] / 2.0
        cproj = sim._project_to_cam(cxw, cyw)
        if cproj is None:
            continue
        # 중심 bearing 게이트 (측면 과대투영 방지)
        if abs(sim._bearing_to(cxw, cyw)) > 35.0:
            continue
        cdist = math.hypot(cxw - sim.robot['x'], cyw - sim.robot['y'])
        if sim._occluded(cxw, cyw, cdist):
            continue
        corners_w = [(p['x'], p['y']), (p['x'] + p['w'], p['y']),
                     (p['x'] + p['w'], p['y'] + p['h']), (p['x'], p['y'] + p['h'])]
        proj = [sim._project_to_cam(wx, wy) for wx, wy in corners_w]
        if any(q is None for q in proj):
            continue
        poly = [to_scr(cx, cy) for cx, cy in proj]
        col = PATCH_RGB[p['color']]
        surf = pygame.Surface((pw, ph), pygame.SRCALPHA)
        loc = [(sx - px0, sy - py0 - title_h) for sx, sy in poly]
        pygame.draw.polygon(surf, (*col, 150), loc)
        screen.blit(surf, (px0, py0 + title_h))
        pygame.draw.polygon(screen, col, poly, 1)
        if i == det_idx:                       # 감지된 타깃: centroid 마커
            mcx, mcy = to_scr(*cproj)
            pygame.draw.circle(screen, (40, 255, 120), (mcx, mcy), 5, 2)
            pygame.draw.line(screen, (40, 255, 120), (mcx - 8, mcy), (mcx + 8, mcy), 1)
            pygame.draw.line(screen, (40, 255, 120), (mcx, mcy - 8), (mcx, mcy + 8), 1)

    screen.set_clip(prev_clip)

    # 하단 상태줄
    if sim.detection:
        msg = '%s  b=%+.1f\u00b0  d=%.0fmm' % (tgt, sim.detection['bearing'],
                                               sim.detection['distance'])
        mcol = (150, 230, 160)
    else:
        msg = '%s  (no signal)' % tgt
        mcol = (150, 150, 160)
    screen.blit(font_s.render(msg, True, mcol), (px0 + 5, img.bottom - 16))


def draw_hud(screen, sim, font, font_s):
    v, w = sim.last_vw
    state = 'RUNNING' if sim.running else 'STOPPED'
    scol = (90, 220, 130) if sim.running else (220, 160, 90)
    cam_state = CAM.get_state()
    # 오케스트레이션이 CLOSE 오도메트리 추종 중이면(카메라가 패치를 잃어도) 명확히 표기
    if sim._close_target_x is not None and not (CAM.is_dwelling() or CAM.is_done()):
        cam_state = f'CLOSE*odom_{CAM.get_target_color()}'
    tgt = CAM.get_target_color()
    tgt_col = PATCH_RGB.get(tgt, (200, 200, 200))
    if sim.detection:
        det = (f"detect {tgt}: bearing={sim.detection['bearing']:+.1f}  "
               f"dist={sim.detection['distance']:.0f}mm  close={sim.detection['is_close']}", C_TEXT)
    else:
        det = (f"detect {tgt}: --- (no patch in FOV)", C_DIM)
    lines = [
        (f'{state}    mission: {cam_state}', scol),
        (det[0], det[1]),
        (f'tool: {sim.tool}' + (f' ({sim.patch_color})' if sim.tool == 'patch' else ''), C_DIM),
        (f'v = {v:+.3f}   w = {w:+.3f} (>0=left)', C_TEXT),
        (f'heading = {sim.robot["h"]:+.1f} deg', C_TEXT),
        (f'pos = ({sim.robot["x"]:.0f}, {sim.robot["y"]:.0f}) mm', C_DIM),
        (f'scan pts = {len(sim.scan)}   zoom = {sim.zoom:.3f}   speed = {sim.speed:g}x', C_DIM),
    ]
    if sim.sel and sim.sel[0] == 'obstacle':
        ob = sim.obstacles[sim.sel[1]]
        lines.append((f'obstacle #{sim.sel[1]}: {ob["w"]:.0f}x{ob["h"]:.0f}mm  '
                      f'angle={ob.get("a", 0.0):+.0f} deg', (160, 200, 240)))
    if sim._close_target_x is not None and sim._close_initial_dist:
        ex = sim._close_target_x - S.arduino_x_mm
        ey = sim._close_target_y - S.arduino_y_mm
        derr = math.hypot(ex, ey)
        pct = (1.0 - derr / sim._close_initial_dist) * 100.0
        lines.append((f'CLOSE: remain={derr:.0f}mm  {pct:4.0f}%', (250, 160, 210)))
    if sim.running and sim._seek_src and sim._seek_src != 'TRACK':
        est = (f"est=({-sim._seek_target_x:.0f},{sim._seek_target_y:.0f})mm"
               if sim._seek_target_x is not None else "est=--")
        lines.append((f'SEEK lost: {sim._seek_src}  tb={sim._seek_tb:+.1f}  {est}',
                      (250, 180, 120)))
    if sim.viz:
        lines.append((f'STOP = {sim.viz["stop_triggered"]}   '
                      f'layers = {sorted(sim.viz["active_names"])}', C_DIM))
    bg = pygame.Surface((400, 20 * len(lines) + 10), pygame.SRCALPHA)
    bg.fill((10, 12, 16, 175))
    screen.blit(bg, (8, 8))
    y = 14
    for txt, col in lines:
        screen.blit(font_s.render(txt, True, col), (16, y))
        y += 20

    help_txt = ('arrows: pan | wheel: zoom | Q/E: rotate robot | , .: rotate obstacle | '
                'Del / right-click: delete | Space: run/stop')
    t = font_s.render(help_txt, True, C_DIM)
    screen.blit(t, (16, WIN_H - 24))


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  입력 처리
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def in_panel(pos):
    return pos[0] >= MAP_W


def hit_handle(ob, pos, cam, zoom):
    """선택된 (미회전) 장애물의 코너 핸들 hit → corner index or None.
    회전된(a≠0) 장애물은 핸들 비활성 (리사이즈는 a=0에서만)."""
    if ob.get('a', 0.0):
        return None
    r = world_rect_to_screen(ob['x'], ob['y'], ob['w'], ob['h'], cam, zoom)
    corners = [r.topleft, r.topright, r.bottomleft, r.bottomright]
    for idx, (cx, cy) in enumerate(corners):
        if abs(pos[0] - cx) <= 6 and abs(pos[1] - cy) <= 6:
            return idx
    return None


def pick_object(sim, wpos):
    """world 좌표에서 로봇/장애물/패치 hit 테스트 (위에 그려진 순서 역순)."""
    rx, ry = sim.robot['x'], sim.robot['y']
    if (wpos[0] - rx) ** 2 + (wpos[1] - ry) ** 2 <= (S.ROBOT_HALF_WIDTH + 40) ** 2:
        return ('robot', None)
    for i in range(len(sim.obstacles) - 1, -1, -1):
        if point_in_obb(wpos[0], wpos[1], sim.obstacles[i]):
            return ('obstacle', i)
    for i in range(len(sim.patches) - 1, -1, -1):
        p = sim.patches[i]
        if p['x'] <= wpos[0] <= p['x'] + p['w'] and p['y'] <= wpos[1] <= p['y'] + p['h']:
            return ('patch', i)
    return None


def handle_panel_click(sim, pos):
    sc = sim.panel_scroll
    for b in sim.tool_btns:
        if b.hit(pos, sc):
            if b.group == 'tool':
                sim.tool = b.value
            elif b.group == 'pcolor':
                sim.patch_color = b.value
                sim.tool = 'patch'
            _sync_buttons(sim)
            return True
    for b in sim.sim_btns:
        if b.hit(pos, sc):
            {'start': sim.start, 'stop': sim.stop, 'restart': sim.restart}[b.value]()
            return True
    for b in sim.speed_btns:
        if b.hit(pos, sc):
            sim.speed = b.value
            return True
    for c in sim.toggles.values():
        if c.hit(pos, sc):
            c.checked = not c.checked
            return True
    for s in sim.sliders:
        if s.hit(pos, sc):
            s.drag = True
            s.set_from_x(pos[0], sc)
            sim.apply_sliders()
            return True
    return True   # 패널 안 클릭은 항상 소비


def _sync_buttons(sim):
    for b in sim.tool_btns:
        if b.group == 'tool':
            b.active = (b.value == sim.tool)
        elif b.group == 'pcolor':
            b.active = (b.value == sim.patch_color)


def _sync_sim_buttons(sim):
    for b in sim.sim_btns:
        b.active = (b.value == 'start' and sim.running)
    for b in sim.speed_btns:
        b.active = (abs(b.value - sim.speed) < 1e-6)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  메인
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def make_default_scene(sim):
    """데모: RED→YELLOW→BLUE 를 전방에 일렬 배치(전체 미션 파이프라인 시연) +
    측면 장애물 + 상단 경로에 살짝 걸치는 장애물(회피 시연)."""
    # 로봇 시작 (0,-800) 정면 위. 패치는 중앙을 따라 위로.
    sim.patches.append({'x': -100, 'y': -350, 'w': 200, 'h': 200, 'color': 'R'})  # center (0,-250)
    sim.patches.append({'x': -100, 'y':  100, 'w': 200, 'h': 200, 'color': 'Y'})  # center (0, 200)
    sim.patches.append({'x': -100, 'y':  560, 'w': 200, 'h': 200, 'color': 'B'})  # center (0, 660)
    # 측면 장애물 (하나는 회전 배치로 기능 시연)
    sim.obstacles.append({'x':  350, 'y': -250, 'w': 200, 'h': 200, 'a':  0.0})
    sim.obstacles.append({'x': -560, 'y':  150, 'w': 260, 'h': 120, 'a': 35.0})
    # YELLOW→BLUE 경로에 살짝 걸치는 회전 장애물 → 회피 유도
    sim.obstacles.append({'x':   80, 'y':  360, 'w': 200, 'h': 120, 'a': 25.0})


def main():
    selftest = 0
    if '--selftest' in sys.argv:
        try:
            selftest = int(sys.argv[sys.argv.index('--selftest') + 1])
        except (ValueError, IndexError):
            selftest = 120
        import os
        os.environ['SDL_VIDEODRIVER'] = 'dummy'
        os.environ['SDL_AUDIODRIVER'] = 'dummy'

    pygame.init()
    pygame.display.set_caption('Project 3 Driving Sim  (Phase 1)')
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    overlay = pygame.Surface((MAP_W, WIN_H), pygame.SRCALPHA)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont('consolas,menlo,monospace', 16)
    font_s = pygame.font.SysFont('consolas,menlo,monospace', 13)

    sim = Sim()
    make_default_scene(sim)
    _sync_buttons(sim)

    running = True
    frame = 0
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                pos = event.pos
                if event.button in (4, 5):           # 휠
                    if in_panel(pos):
                        sim.panel_scroll = max(0, min(sim.panel_max,
                                        sim.panel_scroll + (40 if event.button == 5 else -40)))
                    else:
                        old = sim.zoom
                        sim.zoom *= 1.12 if event.button == 4 else 1 / 1.12
                        sim.zoom = max(0.08, min(3.0, sim.zoom))
                        wx, wy = s2w(pos[0], pos[1], sim.cam, old)   # 커서 고정 줌
                        nx, ny = s2w(pos[0], pos[1], sim.cam, sim.zoom)
                        sim.cam[0] += wx - nx
                        sim.cam[1] += wy - ny
                    continue
                if event.button == 3:                # 우클릭 → 오브젝트 즉시 삭제
                    if not in_panel(pos):
                        wpos = s2w(pos[0], pos[1], sim.cam, sim.zoom)
                        hit = pick_object(sim, wpos)
                        if hit and hit[0] == 'obstacle':
                            sim.obstacles.pop(hit[1]); sim.sel = None
                        elif hit and hit[0] == 'patch':
                            sim.patches.pop(hit[1]); sim.sel = None
                    continue
                if event.button != 1:
                    continue
                if in_panel(pos):
                    handle_panel_click(sim, pos)
                    continue
                # ── 지도 클릭 ──
                wpos = s2w(pos[0], pos[1], sim.cam, sim.zoom)
                if sim.tool == 'select':
                    # 핸들(리사이즈) 우선
                    if sim.sel and sim.sel[0] == 'obstacle':
                        ci = hit_handle(sim.obstacles[sim.sel[1]], pos, sim.cam, sim.zoom)
                        if ci is not None:
                            sim.drag_mode = 'resize'
                            sim.resize_corner = ci
                            continue
                    picked = pick_object(sim, wpos)
                    sim.sel = picked
                    if picked:
                        sim.drag_mode = 'move'
                        if picked[0] == 'robot':
                            sim.drag_off = (wpos[0] - sim.robot['x'], wpos[1] - sim.robot['y'])
                        elif picked[0] == 'obstacle':
                            ob = sim.obstacles[picked[1]]
                            sim.drag_off = (wpos[0] - ob['x'], wpos[1] - ob['y'])
                        else:
                            p = sim.patches[picked[1]]
                            sim.drag_off = (wpos[0] - p['x'], wpos[1] - p['y'])
                elif sim.tool == 'robot':
                    sim.robot['x'], sim.robot['y'] = wpos
                    sim.sel = ('robot', None)
                    sim.drag_mode = 'move'
                    sim.drag_off = (0.0, 0.0)
                elif sim.tool in ('obstacle', 'patch'):
                    sim.create_anchor = wpos
                    sim.drag_mode = 'create'

            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if sim.drag_mode == 'create' and sim.create_anchor:
                    wpos = s2w(event.pos[0], event.pos[1], sim.cam, sim.zoom)
                    x0, y0 = sim.create_anchor
                    x, y = min(x0, wpos[0]), min(y0, wpos[1])
                    w, h = abs(wpos[0] - x0), abs(wpos[1] - y0)
                    if w < 60 or h < 60:                 # 클릭만 → 기본 크기
                        w = h = 220
                        x, y = x0 - 110, y0 - 110
                    if sim.tool == 'obstacle':
                        sim.obstacles.append({'x': x, 'y': y, 'w': w, 'h': h, 'a': 0.0})
                        sim.sel = ('obstacle', len(sim.obstacles) - 1)
                    else:
                        sim.patches.append({'x': x, 'y': y, 'w': w, 'h': h,
                                            'color': sim.patch_color})
                        sim.sel = ('patch', len(sim.patches) - 1)
                for s in sim.sliders:
                    s.drag = False
                sim.drag_mode = None
                sim.create_anchor = None

            elif event.type == pygame.MOUSEMOTION:
                if not (event.buttons[0]):
                    continue
                pos = event.pos
                for s in sim.sliders:
                    if s.drag:
                        s.set_from_x(pos[0], sim.panel_scroll)
                        sim.apply_sliders()
                if in_panel(pos):
                    continue
                wpos = s2w(pos[0], pos[1], sim.cam, sim.zoom)
                if sim.drag_mode == 'move' and sim.sel:
                    if sim.sel[0] == 'robot':
                        sim.robot['x'] = wpos[0] - sim.drag_off[0]
                        sim.robot['y'] = wpos[1] - sim.drag_off[1]
                    elif sim.sel[0] == 'obstacle':
                        ob = sim.obstacles[sim.sel[1]]
                        ob['x'] = wpos[0] - sim.drag_off[0]
                        ob['y'] = wpos[1] - sim.drag_off[1]
                    else:
                        p = sim.patches[sim.sel[1]]
                        p['x'] = wpos[0] - sim.drag_off[0]
                        p['y'] = wpos[1] - sim.drag_off[1]
                elif sim.drag_mode == 'resize' and sim.sel and sim.sel[0] == 'obstacle':
                    ob = sim.obstacles[sim.sel[1]]
                    # 고정될 반대 코너 계산 후 재구성
                    corners = {0: (ob['x'], ob['y'] + ob['h']),          # TL(world 좌상)
                               1: (ob['x'] + ob['w'], ob['y'] + ob['h']),  # TR
                               2: (ob['x'], ob['y']),                     # BL
                               3: (ob['x'] + ob['w'], ob['y'])}           # BR
                    opp = {0: 3, 1: 2, 2: 1, 3: 0}[sim.resize_corner]
                    fx, fy = corners[opp]
                    nx, ny = wpos
                    ob['x'] = min(fx, nx); ob['y'] = min(fy, ny)
                    ob['w'] = max(40, abs(fx - nx)); ob['h'] = max(40, abs(fy - ny))

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    sim.cam[0] -= PAN_PX / sim.zoom
                elif event.key == pygame.K_RIGHT:
                    sim.cam[0] += PAN_PX / sim.zoom
                elif event.key == pygame.K_UP:
                    sim.cam[1] += PAN_PX / sim.zoom
                elif event.key == pygame.K_DOWN:
                    sim.cam[1] -= PAN_PX / sim.zoom
                elif event.key == pygame.K_q:
                    sim.robot['h'] = S.normalize_angle(sim.robot['h'] + ROT_STEP)
                elif event.key == pygame.K_e:
                    sim.robot['h'] = S.normalize_angle(sim.robot['h'] - ROT_STEP)
                elif event.key == pygame.K_COMMA:        # ',' 선택 장애물 반시계 회전
                    if sim.sel and sim.sel[0] == 'obstacle':
                        ob = sim.obstacles[sim.sel[1]]
                        ob['a'] = S.normalize_angle(ob.get('a', 0.0) + ROT_STEP)
                elif event.key == pygame.K_PERIOD:       # '.' 선택 장애물 시계 회전
                    if sim.sel and sim.sel[0] == 'obstacle':
                        ob = sim.obstacles[sim.sel[1]]
                        ob['a'] = S.normalize_angle(ob.get('a', 0.0) - ROT_STEP)
                elif event.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
                    if sim.sel and sim.sel[0] == 'obstacle':
                        sim.obstacles.pop(sim.sel[1]); sim.sel = None
                    elif sim.sel and sim.sel[0] == 'patch':
                        sim.patches.pop(sim.sel[1]); sim.sel = None
                elif event.key == pygame.K_SPACE:
                    sim.stop() if sim.running else sim.start()
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    pr = [0.5, 1.0, 2.0, 4.0]
                    i = min(range(4), key=lambda k: abs(pr[k] - sim.speed))
                    sim.speed = pr[max(0, i - 1)]
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
                    pr = [0.5, 1.0, 2.0, 4.0]
                    i = min(range(4), key=lambda k: abs(pr[k] - sim.speed))
                    sim.speed = pr[min(3, i + 1)]
                elif event.key == pygame.K_ESCAPE:
                    running = False

        # ── 업데이트 & 렌더 ──
        if sim.running:
            # speed 배율: 프레임당 step 수 (누적). <1x 면 일부 프레임은 건너뜀(슬로모).
            sim._step_accum += sim.speed
            n_steps = int(sim._step_accum)
            sim._step_accum -= n_steps
            for _ in range(min(n_steps, 8)):     # 폭주 방지 상한
                sim.step()
        else:
            sim._step_accum = 0.0
            sim.step()                            # 정지 중에도 에디터 라이브 프리뷰
        _sync_sim_buttons(sim)
        screen.fill(C_BG)
        draw_map(screen, overlay, sim, font, font_s)
        draw_panel(screen, sim, font, font_s)
        draw_hud(screen, sim, font, font_s)
        if sim.toggles['preview'].checked:
            draw_camera_preview(screen, sim, font_s)
        pygame.display.flip()
        clock.tick(60)

        frame += 1
        if selftest and frame == 1:
            sim.start()      # 자가검증: 바로 주행 시작
        if selftest and frame >= selftest:
            print(f"[selftest] {frame} frames OK | "
                  f"pos=({sim.robot['x']:.0f},{sim.robot['y']:.0f}) "
                  f"h={sim.robot['h']:.1f} v,w={sim.last_vw} "
                  f"scan={len(sim.scan)} stop={sim.viz['stop_triggered']}")
            running = False

    pygame.quit()


if __name__ == '__main__':
    main()
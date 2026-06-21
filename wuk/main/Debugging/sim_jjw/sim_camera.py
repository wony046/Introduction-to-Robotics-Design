# -*- coding: utf-8 -*-
"""
sim_camera.py  ─  카메라 + 미션 상태머신 시뮬 (camera_tracker.py 이식)

camera_tracker.py 의 public API(get_bearing / is_close / is_dwelling / is_done /
get_estimated_distance_mm / get_last_close_bearing / signal_arrival / get_state)를
그대로 모사한다. 단, OpenCV 비전 대신 "기하 감지"를 외부(project3_sim)에서
계산해 update() 로 주입한다 → 미션 시퀀싱 / dwell / 도착 / CLOSE 플래그 같은
'상태 로직'만 충실히 재현.

감지(detection) dict 형식 (project3_sim 이 만들어 넣음):
    {'bearing': float(deg, 좌+), 'distance': float(mm),
     'is_close': bool, 'patch_idx': int}
  또는 None (미감지).

도착(arrival)은 camera_tracker 와 동일하게 USE_ROI_ARRIVE=0 가정 →
오직 signal_arrival()(외부 CLOSE 오도메트리 제어가 호출) 로만 트리거.
"""

import math

# ── 카메라 기하 상수 (camera_tracker.py 값 + 시뮬 가시범위) ──────────────────
HFOV_DEG     = 38.6      # 수평 FOV (보정 후 _EFF_W=480 기준 실측). 감지콘 = ±HFOV/2
ARRIVE_HOLD_SEC = 1.2    # 도착 후 정지 유지(dwell) 시간

# 카메라 투영 모델 (camera_tracker.py 와 동일) — 프리뷰 창 렌더용
CAM_HEIGHT_MM = 430.0    # 카메라~바닥(색지) 수직 높이
CAM_TILT_DEG  = 34.5     # 광축 하향 틸트(수평=0, 아래로+)
EFF_W = 480.0            # 90° 회전 마운트 후 실효 가로 (_EFF_W) = raw FRAME_H, 유지
EFF_H = 848.0            # 90° 회전 마운트 후 실효 세로 (_EFF_H). [16:9] raw FRAME_W 640→848

def f_px():
    """초점거리(px). HFOV 가 EFF_W 기준이므로 EFF_W 로 산출."""
    return (EFF_W / 2.0) / math.tan(math.radians(HFOV_DEG / 2.0))

# 카메라 바닥 가시 범위(직선거리, mm). 틸트 34.5° + 수직FOV~50° + 높이 430mm 에서
# 근거리 한계 ~250mm(아래로 시야 벗어남) / 원거리 ~2600mm 추정.
# ★ CLOSE 모드의 dead-reckoning 을 충실히 테스트하려면 이 근거리 한계가 중요:
#   거리<250mm 면 카메라가 색지를 잃고 → CLOSE 오도메트리 추종이 인계받음.
CAM_NEAR_MM  = 250.0     # [16:9] EFF_H 변경으로 수직FOV 변동 → 재산출 권장
CAM_FAR_MM   = 2600.0

MISSION_ORDER = ['RED', 'YELLOW', 'BLUE']

# ── 상태 ────────────────────────────────────────────────────────────────────
_mission_idx        = 0
_dwell_elapsed      = 0.0
_dwelling           = False
_done               = False
_close              = False
_arrival_signal     = False

_target_bearing     = None    # 현재 감지 bearing (deg, 좌+) 또는 None
_last_close_bearing = 0.0      # CLOSE 진입 시 bearing — compute_close_target 용
_last_stable_bearing = 0.0
_last_distance      = 500.0    # 마지막 추정 거리 (mm)
_detected_idx       = None     # 현재 감지된 패치 index (시각화용)


def reset():
    """미션/도착/CLOSE 상태 전체 초기화 (재시작 시)."""
    global _mission_idx, _dwell_elapsed, _dwelling, _done, _close, _arrival_signal
    global _target_bearing, _last_close_bearing, _last_stable_bearing, _last_distance, _detected_idx
    _mission_idx = 0
    _dwell_elapsed = 0.0
    _dwelling = False
    _done = False
    _close = False
    _arrival_signal = False
    _target_bearing = None
    _last_close_bearing = 0.0
    _last_stable_bearing = 0.0
    _last_distance = 500.0
    _detected_idx = None


def update(detection, dt):
    """매 스텝 호출. detection(dict 또는 None) 주입 + 미션/dwell FSM 진행.
    camera_tracker._camera_loop 의 상태 전이를 그대로 따른다."""
    global _mission_idx, _dwell_elapsed, _dwelling, _done, _close, _arrival_signal
    global _target_bearing, _last_close_bearing, _last_stable_bearing, _last_distance, _detected_idx

    # 완주 → 정지 유지
    if _done or _mission_idx >= len(MISSION_ORDER):
        _target_bearing = None
        _detected_idx = None
        _close = False
        _dwelling = True
        _done = True
        return

    # ── 감지 반영 ──
    if detection is not None:
        _target_bearing      = detection['bearing']
        _last_stable_bearing = detection['bearing']
        _last_distance       = detection['distance']
        _close               = detection['is_close']
        _detected_idx        = detection['patch_idx']
        if _close:
            _last_close_bearing = detection['bearing']
    else:
        _target_bearing = None
        _close          = False
        _detected_idx   = None
        # _last_close_bearing / _last_distance 는 유지 (CLOSE 인계용)

    # ── 도착(dwell) FSM : USE_ROI_ARRIVE=0 → 외부 신호만 ──
    if _arrival_signal:
        _dwelling = True
        _dwell_elapsed += dt
        if _dwell_elapsed >= ARRIVE_HOLD_SEC:
            _mission_idx += 1
            _dwell_elapsed = 0.0
            _dwelling = False
            _arrival_signal = False
            _close = False
            _last_stable_bearing = 0.0
            if _mission_idx >= len(MISSION_ORDER):
                _done = True
    else:
        _dwell_elapsed = 0.0
        _dwelling = False


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Public API (camera_tracker.py 와 동일)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def get_bearing():
    return _target_bearing

def is_dwelling():
    return _dwelling

def is_done():
    return _done

def is_close():
    return _close

def get_estimated_distance_mm():
    return _last_distance

def get_last_close_bearing():
    return _last_close_bearing

def get_last_stable_bearing():
    return _last_stable_bearing

def signal_arrival():
    """외부(CLOSE 오도메트리 제어)가 도착을 알릴 때 호출."""
    global _arrival_signal
    _arrival_signal = True


# ── 시뮬 보조 (시각화/오케스트레이션용) ─────────────────────────────────────
def get_target_color():
    if _done or _mission_idx >= len(MISSION_ORDER):
        return 'DONE'
    return MISSION_ORDER[_mission_idx]

def get_mission_idx():
    return _mission_idx

def get_detected_idx():
    return _detected_idx

def get_state():
    if _done:
        return 'DONE'
    if _dwelling:
        return f'DWELL_{get_target_color()}'
    if _close:
        return f'CLOSE_{get_target_color()}'
    return f'SEEK_{get_target_color()}'
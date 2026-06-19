"""
floor_verify.py — 색지(바닥) vs 장애물(입체) 구분 검증

문제:
  황토색 장애물은 노랑과, 남색 장애물은 파랑과 색상(H)이 겹쳐
  HSV만으로는 분리 불가.

해결 (라이다+카메라 거리 비교):
  - 카메라가 본 색 blob의 cy → "바닥에 누워 있다고 가정한 거리"(cam_dist).
  - 같은 bearing 방향의 라이다 거리(lidar_dist).
  - RPLIDAR C1이 바닥 근처 낮은 높이에서 거의 수평 스캔하므로:
      · 세워진 장애물  → 라이다 빔이 옆면에 맞음 → lidar_dist ≈ cam_dist
      · 바닥의 색지    → 빔이 색지 위를 지나 더 멀리/무응답 → lidar_dist >> cam_dist
  → lidar_dist - cam_dist > MARGIN  또는  해당 방향 라이다 무응답 → 색지(인정)
    그 외(라이다가 카메라 거리쯤에서 막힘) → 장애물(기각)

좌표/각도 가정 (jw_won.py와 동일):
  - scan_points: [(angle_deg, dist_mm), ...], angle은 normalize_angle(±180), 0=정면
  - 카메라 bearing도 0=정면, 부호 동일 (사용자 확인: 같은 방향)
  - cam_dist는 camera_tracker.get_estimated_distance_mm() / get_candidate()[1]과 동일 식
"""

import math

# ── 파라미터 ──────────────────────────────────────────────────────────
ANGLE_TOL_DEG    = 8.0     # bearing ± 이 각도 안의 라이다 포인트를 같은 방향으로 본다
DIST_MARGIN_MM   = 300.0   # 라이다가 카메라거리보다 이만큼 이상 멀면 '색지'(빔이 통과)
LIDAR_MIN_VALID  = 100.0   # jw_won.py와 동일 (노이즈 하한)
NO_RETURN_AS_PAPER = True   # 해당 방향 라이다 리턴 없음 → 색지로 인정(빔이 통과한 것)


def lidar_dist_at(scan_points, bearing_deg, angle_tol=ANGLE_TOL_DEG):
    """
    bearing_deg ± angle_tol 범위 라이다 포인트 중 '가장 가까운 유효 거리' 반환.
    유효 리턴이 없으면 None.
    (가장 가까운 값을 쓰는 이유: 그 방향에 막는 물체가 있으면 가까운 면이 잡힘)
    """
    best = None
    for a, d in scan_points:
        if d < LIDAR_MIN_VALID:
            continue
        if abs(((a - bearing_deg) + 180) % 360 - 180) <= angle_tol:
            if best is None or d < best:
                best = d
    return best


def is_floor_paper(scan_points, bearing_deg, cam_dist_mm,
                   dist_margin=DIST_MARGIN_MM, angle_tol=ANGLE_TOL_DEG):
    """
    카메라가 본 색 blob이 '바닥 색지'면 True, '세워진 장애물'이면 False.

    판정:
      ld = 해당 방향 라이다 최근접 거리
      ld is None              → 빔이 통과(아무것도 안 막음) → 색지 (NO_RETURN_AS_PAPER)
      ld - cam_dist > margin  → 라이다가 색지 너머를 봄        → 색지
      그 외                   → 라이다가 cam_dist쯤에서 막힘   → 장애물
    """
    ld = lidar_dist_at(scan_points, bearing_deg, angle_tol)
    if ld is None:
        return NO_RETURN_AS_PAPER
    return (ld - cam_dist_mm) > dist_margin


def verify_detection(scan_points, bearing_deg, cam_dist_mm, debug=False):
    """
    편의 래퍼. (is_paper, lidar_dist) 반환 — 로깅에 라이다 거리도 같이 준다.
    """
    ld = lidar_dist_at(scan_points, bearing_deg)
    if ld is None:
        ok = NO_RETURN_AS_PAPER
    else:
        ok = (ld - cam_dist_mm) > DIST_MARGIN_MM
    if debug:
        ld_s = f"{ld:.0f}" if ld is not None else "None"
        verdict = "색지(인정)" if ok else "장애물(기각)"
        print(f"[VERIFY] bearing={bearing_deg:+.1f}° cam={cam_dist_mm:.0f}mm "
              f"lidar={ld_s}mm → {verdict}")
    return ok, ld

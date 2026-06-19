#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
부호 검증 테스트 — 라이다 각도 부호 / 헤딩 회전 부호

목적:
  STOP 피봇의 회전 부호를 확정하기 위해, 두 가지 '하드웨어 규약'을 실측한다.
    [A] 라이다 : 로봇 '왼쪽'에 둔 장애물이 음수 각도로 잡히는가?  → LEFT_IS_NEG
    [B] 모터   : '+w' 명령이 아두이노 헤딩을 '증가'시키는가?       → W_SIGN
  이 둘을 조합하면
    (1) 새 피봇 공식에 박을 상수(LIDAR_SIGN, W_SIGN)
    (2) 기존 분기①(카메라 직진)/분기②(갭추종)/기존 STOP 피봇 중
        어느 쪽 부호가 뒤집혀 있었는지
  가 한 번에 드러난다.

주의:
  - main 제어 스크립트는 '종료'한 상태에서 실행할 것 (시리얼 포트 충돌 방지).
  - [B] 테스트는 로봇이 제자리 회전한다. 바퀴를 들거나 충분한 공간을 확보할 것.
  - 라이다/아두이노 포트·통신값은 아래 상수가 main 코드와 동일해야 한다.
"""

import sys
import time
import math

try:
    import serial
except ImportError:
    print("pyserial이 필요해:  pip install pyserial")
    sys.exit(1)

# ── 포트 / 통신 (main 코드와 동일하게 유지) ──────────────────────────
LIDAR_PORT       = "/dev/ttyUSB0"
ARDUINO_PORT     = "/dev/ttyAMA3"
BAUDRATE_LIDAR   = 460800
BAUDRATE_ARDUINO = 115200

LIDAR_OFFSET     = 10      # mm
LIDAR_MIN_VALID  = 100     # mm
DETECTION_RANGE  = 1500    # mm

# ── [B] 스핀 파라미터 ────────────────────────────────────────────────
SPIN_W    = 1.0    # rad/s, +w 명령 (스티션 확실히 넘기려고 1.0)
SPIN_TIME = 1.0    # sec  (≈ SPIN_W*SPIN_TIME rad ≈ 57° 회전 예상)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 유틸 (main 코드와 동일)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def normalize_angle(angle):
    return ((angle + 180) % 360) - 180

def is_in_front_90(a):
    return -90 <= a <= 90

def parse_packet(data):
    if len(data) != 5:
        return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        return None
    if (data[1] & 0x01) != 1:
        return None
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    distance_q2 = data[3] | (data[4] << 8)
    return (angle_q6 / 64.0), (distance_q2 / 4.0)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [A] 라이다 각도 부호 테스트
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def test_lidar_angle(duration=2.5):
    print("\n[A] 라이다 각도 부호 테스트")
    print("    → 로봇 '정면 왼쪽'에 장애물(박스/벽)을 가까이(30~60cm) 두고,")
    print("      정면 오른쪽은 가능하면 비워둘 것 (최근접 포인트가 왼쪽이 되도록).")
    input("    준비되면 Enter > ")

    try:
        lidar = serial.Serial(LIDAR_PORT, BAUDRATE_LIDAR, timeout=1)
    except Exception as e:
        print(f"    [에러] 라이다 포트 열기 실패: {e}")
        print(f"           - main 스크립트가 켜져있지 않은지 / 포트({LIDAR_PORT}) 확인")
        print(f"           - 권한 문제면 sudo 또는 dialout 그룹 확인")
        return None

    try:
        time.sleep(2)
        lidar.write(bytes([0xA5, 0x40])); time.sleep(1.0)   # RESET
        lidar.reset_input_buffer()                          # 부팅 메시지 비우기
        lidar.write(bytes([0xA5, 0x20])); time.sleep(0.1)   # SCAN
        lidar.read(7)                                       # 응답 descriptor

        buckets = {}   # {각도(int): 최소거리}
        t0 = time.time()
        while time.time() - t0 < duration:
            raw = lidar.read(5)
            r = parse_packet(raw)
            if r is None:
                continue
            ang_raw, dist = r
            if dist <= 0:
                continue
            d = dist + LIDAR_OFFSET
            if not (LIDAR_MIN_VALID < d < DETECTION_RANGE):
                continue
            a = normalize_angle(ang_raw)
            if not is_in_front_90(a):
                continue
            b = int(round(a))
            if b not in buckets or d < buckets[b]:
                buckets[b] = d
    finally:
        try:
            lidar.write(bytes([0xA5, 0x25])); time.sleep(0.1)  # STOP
            lidar.close()
        except Exception:
            pass

    if not buckets:
        print("    [실패] 전방에서 유효 포인트를 못 받음.")
        print("           - 장애물 거리/위치 확인, main 종료 여부 확인 후 재시도.")
        return None

    closest = sorted(buckets.items(), key=lambda kv: kv[1])
    near_ang, near_d = closest[0]
    print("    가까운 포인트 5개 (각도°, 거리mm):")
    for ang, dd in closest[:5]:
        print(f"        {ang:+4d}°    {dd:5.0f} mm")
    print(f"    → 최근접: {near_ang:+d}°, {near_d:.0f} mm")

    if abs(near_ang) < 8:
        print("    [주의] 최근접 각도가 거의 0°(정면)야.")
        print("           장애물을 좀 더 '왼쪽'으로 치우쳐서 다시 측정 권장.")

    left_is_neg = (near_ang < 0)
    side = "음수(−)" if left_is_neg else "양수(+)"
    print(f"    [결과] 왼쪽 장애물 → 라이다 각도 {side}")
    print(f"           ⇒ LEFT_IS_NEG = {left_is_neg}")
    return left_is_neg


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# [B] 헤딩 회전 부호 테스트
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def read_latest_heading(ar, timeout=1.0, want=3):
    """'O:x,y,hdg' 또는 'H:hdg' 라인에서 최신 heading(°) 추출.
    want개 샘플 받으면 조기 종료. 반환: (heading 또는 None, 샘플 수)."""
    t0 = time.time()
    h = None
    n = 0
    while time.time() - t0 < timeout:
        line = ar.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('O:'):
            parts = line[2:].split(',')
            if len(parts) == 3:
                try:
                    h = float(parts[2]); n += 1
                except ValueError:
                    pass
        elif line.startswith('H:'):
            try:
                h = float(line[2:]); n += 1
            except ValueError:
                pass
        if n >= want:
            break
    return h, n


def test_heading_sign(spin_w=SPIN_W, spin_time=SPIN_TIME):
    print("\n[B] 헤딩 회전 부호 테스트")
    print(f"    ★ 로봇이 제자리 회전한다 (약 {math.degrees(spin_w*spin_time):.0f}° 예상).")
    print("      바퀴를 들거나 공간을 확보한 뒤 Enter.")
    input("    준비되면 Enter > ")

    try:
        ar = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=0.2)
    except Exception as e:
        print(f"    [에러] 아두이노 포트 열기 실패: {e}")
        print(f"           - main 스크립트 종료했는지 / 포트({ARDUINO_PORT}) 확인")
        return None

    try:
        time.sleep(0.5)
        ar.reset_input_buffer()
        ar.write(b"R\n"); time.sleep(0.3)     # 헤딩/위치 0 리셋
        ar.reset_input_buffer()               # 리셋 처리 후 옛 라인 제거
        h0, n0 = read_latest_heading(ar, timeout=1.0)

        if n0 == 0 or h0 is None:
            print("    [실패] 아두이노에서 'O:'/'H:' 텔레메트리가 안 옴.")
            print("           - 펌웨어/배선(Serial1) 확인 후 재시도.")
            ar.write(b"0.00 0.00\n")
            return None
        print(f"    시작 헤딩 h0 = {h0:+.1f}°")

        # +w 스핀: 아두이노 CMD_TIMEOUT(500ms)을 이기려고 짧은 주기로 재전송
        series = []
        t0 = time.time()
        while time.time() - t0 < spin_time:
            ar.write(f"0.00 {spin_w:.2f}\n".encode())
            line = ar.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('O:'):
                p = line[2:].split(',')
                if len(p) == 3:
                    try:
                        series.append((time.time() - t0, float(p[2])))
                    except ValueError:
                        pass
            time.sleep(0.05)

        ar.write(b"0.00 0.00\n"); time.sleep(0.3)   # 정지
        h1, n1 = read_latest_heading(ar, timeout=1.0)
        ar.write(b"0.00 0.00\n")                    # 정지 한 번 더 (안전)
    finally:
        try:
            ar.close()
        except Exception:
            pass

    if h1 is None:
        print("    [실패] 종료 헤딩을 못 읽음.")
        return None

    if series:
        print("    스핀 중 헤딩 변화 (초, °):")
        step = max(1, len(series) // 6)
        for t, hh in series[::step]:
            print(f"        t={t:4.2f}s   hdg={hh:+.1f}°")

    delta = normalize_angle(h1 - h0)
    print(f"    종료 헤딩 h1 = {h1:+.1f}°    Δ = {delta:+.1f}°")

    if abs(delta) < 5:
        print("    [주의] 회전량이 너무 작아(<5°).")
        print("           바퀴 공중/모터 동작 확인하고 SPIN_W 또는 SPIN_TIME ↑ 후 재시도.")

    w_sign = +1 if delta > 0 else -1
    sgn = "증가(+)" if w_sign > 0 else "감소(−)"
    print(f"    [결과] +w 명령 → 헤딩 {sgn}")
    print(f"           ⇒ W_SIGN = {w_sign:+d}")
    return w_sign


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 종합 판정
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def interpret(left_is_neg, w_sign):
    print("\n" + "=" * 60)
    print("  종합 판정")
    print("=" * 60)
    if left_is_neg is None or w_sign is None:
        print("  두 테스트(A, B)를 모두 완료해야 판정 가능. (옵션 3 권장)")
        print("=" * 60)
        return

    # LIDAR_SIGN: 라이다가 CCW(+=왼쪽) 규약이면 +1, CW(−=왼쪽)면 −1
    lidar_sign = -1 if left_is_neg else +1

    print(f"   측정:  LEFT_IS_NEG = {left_is_neg}   →  LIDAR_SIGN = {lidar_sign:+d}")
    print(f"          W_SIGN      = {w_sign:+d}")
    print("-" * 60)

    print("  [피봇 구현 상수] — 아래 두 상수를 코드에 그대로 박으면 됨:")
    print(f"        LIDAR_SIGN = {lidar_sign:+d}")
    print(f"        W_SIGN     = {w_sign:+d}")
    print("    target_global = normalize(heading + LIDAR_SIGN * escape_angle)")
    print("    err           = normalize(target_global - arduino_heading)")
    print("    w             = W_SIGN * KP_PIVOT * err     # clamp ±MAX_W")
    print("-" * 60)

    # 부호 진단
    branch1_ok   = (w_sign == +1)            # 분기① 카메라:  w = +KP*bearing(=CCW규약)
    branch2_ok   = (w_sign == lidar_sign)    # 분기② 갭추종:  w = +KP*center_angle(라이다규약)
    old_pivot_ok = (w_sign != lidar_sign)    # 기존 STOP 피봇: -copysign(MAX_W, target)

    def mark(ok):
        return "정상 ✅" if ok else "뒤집힘 ⚠️"

    print("  [기존 코드 부호 진단]")
    print(f"     분기① 목표직진 (카메라 bearing) : {mark(branch1_ok)}")
    print(f"     분기② 갭추종   (라이다 center)   : {mark(branch2_ok)}")
    print(f"     기존 STOP 피봇 (-copysign)       : {mark(old_pivot_ok)}")
    print("-" * 60)

    correct_one = "분기② 갭추종" if branch2_ok else "기존 STOP 피봇"
    print("  ⇒ 갭추종과 기존 STOP 피봇은 '구조적으로 항상' 서로 반대 부호.")
    print(f"     이번 측정 기준 올바른 방향: {correct_one}")
    print("     새 피봇 공식(위 상수)으로 통일하면 둘 다 정렬됨.")
    if not branch1_ok:
        print("  ⚠️ 카메라 직진(분기①)까지 뒤집힘 → 모터/엔코더 배선부터 점검 필요.")
    print("=" * 60)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 메뉴
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def main():
    last = {'left_is_neg': None, 'w_sign': None}
    while True:
        print("\n──────────── 부호 검증 ────────────")
        print("  1) [A] 라이다 각도 부호  (왼쪽에 장애물)")
        print("  2) [B] 헤딩 회전 부호    (+w → 헤딩 증감)")
        print("  3) 둘 다 실행 + 종합 판정")
        print("  4) 현재 측정값으로 종합 판정")
        print("  q) 종료")
        sel = input("  선택 > ").strip().lower()

        if sel == '1':
            last['left_is_neg'] = test_lidar_angle()
        elif sel == '2':
            last['w_sign'] = test_heading_sign()
        elif sel == '3':
            last['left_is_neg'] = test_lidar_angle()
            last['w_sign'] = test_heading_sign()
            interpret(last['left_is_neg'], last['w_sign'])
        elif sel == '4':
            interpret(last['left_is_neg'], last['w_sign'])
        elif sel == 'q':
            print("종료.")
            break
        else:
            print("  1 / 2 / 3 / 4 / q 중에서 선택.")


if __name__ == "__main__":
    main()
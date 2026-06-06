#!/usr/bin/env python3
# ============================================================================
#  ppm_calibrate.py  —  평균 스케일(pulses/m) 캘리브레이션 (라즈베리파이)
#
#  RPi 가 UART 로 "LINE <거리>" 전송 → 아두이노 자율 직진 →
#  "LINE_RESULT:odom,encL,encR,avg" 수신 → 줄자 실측값 입력 → true_ppm 계산.
#
#  실행 전: 평소 돌리던 네비게이션/모터 노드는 모두 꺼두세요
#           (UART 를 이 스크립트가 독점해야 함).
#  설치:    pip3 install pyserial
# ============================================================================
import sys
import time
import serial

# ── 설정 ───────────────────────────────────────────────────────────────────
PORT        = "/dev/ttyAMA3"   # CM5 UART. 환경따라 ttyAMA0 / ttyS0 / ttyUSB0
BAUD        = 115200
DIST_M      = 3.0              # 명령 거리(오도메트리 기준). 길수록 측정오차↓
RUNS        = 3               # 반복 횟수 (슬립 평균)
CURRENT_PPM = 4795.0          # 현재 오도메트리가 쓰는 ppm (참고용 표시)
# ──────────────────────────────────────────────────────────────────────────


def open_port():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.0)
    except serial.SerialException as e:
        print(f"[오류] 포트 열기 실패: {PORT}\n  {e}")
        print("  → PORT 값을 ls /dev/tty* 로 확인해 맞추세요.")
        sys.exit(1)
    time.sleep(2.0)            # 아두이노 리셋 대기
    ser.reset_input_buffer()
    return ser


def send(ser, s):
    ser.write((s + "\n").encode())


def wait_result(ser, timeout=120):
    """LINE_RESULT 한 줄을 기다린다. 그 외(O: 등)는 무시."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("LINE_RESULT:"):
            try:
                odom, dL, dR, avg = line[len("LINE_RESULT:"):].split(",")
                return float(odom), int(dL), int(dR), int(avg)
            except ValueError:
                print(f"  [경고] 파싱 실패: {line}")
    return None


def main():
    ser = open_port()
    print("=" * 56)
    print(f" 평균 스케일 캘리브레이션  |  명령거리 {DIST_M} m  |  {RUNS}회")
    print("=" * 56)

    results = []
    for i in range(RUNS):
        input(f"\n[{i+1}/{RUNS}] 출발점에 기준점 표시 → 전방 {DIST_M+1:.0f}m 비우고 Enter...")
        ser.reset_input_buffer()
        send(ser, f"LINE {DIST_M}")
        print("  주행 중... (정지까지 대기)")
        r = wait_result(ser)
        if r is None:
            print("  [실패] 결과 미수신. 이 회차 건너뜀.")
            continue
        odom, dL, dR, avg = r
        imbalance = (dR - dL) / avg * 100 if avg else 0
        print(f"  odom={odom:.3f}m  encL={dL}  encR={dR}  avg={avg}"
              f"  (L/R 불균형 {imbalance:+.1f}%)")
        try:
            phys = float(input("  줄자 실측 [출발점~종료점 직선거리, m]: "))
        except ValueError:
            print("  [실패] 숫자 입력 아님. 건너뜀.")
            continue
        if phys <= 0:
            print("  [실패] 거리 0 이하. 건너뜀.")
            continue
        ppm = avg / phys
        results.append(ppm)
        print(f"  → run{i+1} true_ppm = {ppm:.1f}")

    if not results:
        print("\n측정값이 없습니다.")
        ser.close()
        return

    mean_ppm = sum(results) / len(results)
    spread = max(results) - min(results)

    print("\n" + "=" * 56)
    print(" 결과")
    for j, p in enumerate(results):
        print(f"   run{j+1}:  {p:.1f}")
    print(f"   ----------------------------")
    print(f"   평균 true_ppm = {mean_ppm:.1f}   (편차 {spread:.1f})")
    print(f"   현재값 {CURRENT_PPM:.0f} 대비 {(mean_ppm/CURRENT_PPM-1)*100:+.2f}%")
    if spread > mean_ppm * 0.01:
        print("   ⚠ 회차간 편차 1%↑ → 슬립 의심. 더 천천히/평평한 바닥에서 재측정 권장.")
    print("-" * 56)
    print(" 아두이노에 반영 (둘 다 이 값으로 통일):")
    print(f"   M_TO_PULSE = {mean_ppm:.1f}f;")
    print(f"   // MPP 분모의 4795  →  {mean_ppm:.1f}")
    print(f"   // (애드온 [A] 사용시)  MPP_BASE = 1.0f / {mean_ppm:.1f}f;")
    print("=" * 56)

    ser.close()


if __name__ == "__main__":
    main()
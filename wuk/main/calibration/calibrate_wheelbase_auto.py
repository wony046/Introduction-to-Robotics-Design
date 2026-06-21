#!/usr/bin/env python3
"""
WHEEL_BASE 자동 회전 보정 스크립트 (폐루프 N바퀴 자동 회전 방식)
====================================================================
calibrate_wheelbase.py 가 "사람이 보고 멈추는" 수동 방식이라면, 이 코드는
로봇이 오도메트리 기준 지정한 바퀴 수(N×360°)만큼 스스로 회전·자동 정지하고,
우리는 그 결과 값을 눈으로 확인한 뒤 실제 물리 회전각만 알려주면 됩니다.

[보정 원리] (calibrate_wheelbase.py 와 동일)
  엔코더가 측정한 좌우 바퀴 이동량 차이(Δarc = dsR - dsL)는 물리적 사실로 고정.

      θ_odom     = Δarc / WHEEL_BASE_현재    (아두이노가 보고한 누적 헤딩)
      θ_physical = Δarc / WHEEL_BASE_참값     (실제 물리 회전각)

  ∴ WHEEL_BASE_참값 = WHEEL_BASE_현재 × θ_odom / θ_physical

  · 로봇은 θ_odom 이 N×360° 가 될 때까지 폐루프로 회전 후 자동 정지한다.
  · 실제 물리 회전각 θ_physical 은 우리가 마크로 확인한다:
      (a) 로봇 정면 표식이 시작 기준선에 정확히 정렬됨  → θ_phys = N×360° (정수)
      (b) 어긋났다면 어긋난 각도(잔차)를 입력           → θ_phys = N×360° + 잔차

[준비물]
  · 바닥 기준선(테이프) + 로봇 정면 표식(테이프). 회전 전 둘을 정렬.
  · (선택) 피벗 중심에 각도기/각도 눈금 → 잔차 측정이 쉬워짐.

[사용법]
  SSH로 라즈베리파이 접속 후:
      python3 calibrate_wheelbase_auto.py                # 기본 포트 /dev/ttyAMA3
      python3 calibrate_wheelbase_auto.py /dev/ttyS0     # 포트 직접 지정

  1. 로봇 정면 표식을 바닥 기준선에 정렬한다.
  2. 바퀴 수 N 입력 → 로봇이 자동으로 N바퀴 회전 후 정지.
  3. 화면에 표시된 값(θ_odom, 위치 드리프트, 추정 바퀴)을 확인.
  4. 물리 결과 입력:
       a            → 마크에 정확히 정렬됨 (θ_phys = N×360°)
       <숫자>       → 마크에서 어긋난 각도[deg] (회전방향 +, 반대 −)
       s            → 이번 측정 버림
  5. 여러 회 반복 후 [q] → 평균 WHEEL_BASE 결과 출력.
  6. 출력값을 arduino_code.cpp 의 `const float WHEEL_BASE` 에 반영 후 재업로드.
"""

import serial
import threading
import math
import sys
import time

# ── 설정 ──────────────────────────────────────────────────────────────────────
UART_PORT = '/dev/ttyAMA3'
BAUD_RATE = 115200

# 현재 아두이노에 업로드된 값과 반드시 동일하게! (arduino_code.cpp 의 WHEEL_BASE)
WHEEL_BASE_CURRENT = 0.1802   # [m]

DEFAULT_TURNS = 3       # 기본 회전 바퀴 수
DIRECTION     = +1      # +1: CCW(반시계, heading 증가) / -1: CW(시계)

# ── 폐루프 회전 제어 파라미터 ─────────────────────────────────────────────────
W_MAX         = 1.0     # 순항 각속도 [rad/s]
W_MIN         = 0.40    # 목표 근처 크리프 각속도 [rad/s] (모터 데드밴드 위)
KP_W          = 0.04    # 헤딩오차[deg] → 각속도[rad/s] 비례게인 (25°에서 W_MAX 포화)
STOP_TOL_DEG  = 2.0     # 목표 도달 판정 허용오차 [deg]
CTRL_DT       = 0.02    # 제어 주기 [s] (50Hz)
SEND_DT       = 0.05    # 명령 재전송 주기 [s] (아두이노 타임아웃 500ms 보다 짧게)
SETTLE_S      = 0.7     # 정지 후 오도메트리 안정화 대기 [s]
STALL_TIMEOUT_S = 2.5   # 이 시간 동안 헤딩 진전 없으면 스톨로 보고 중단
SPIN_TIMEOUT_S  = 120.0 # 한 번의 회전 전체 타임아웃 [s]

# ── 공유 상태 ─────────────────────────────────────────────────────────────────
_lock      = threading.Lock()         # _state / _cmd 보호
_io_lock   = threading.Lock()         # 시리얼 write 직렬화 (R 라인과 v/w 라인 섞임 방지)
_running   = threading.Event()
_tx_paused = threading.Event()        # set 동안 tx_thread 송신 일시정지 (리셋 시 R 단독 전송)
_state     = {'heading': 0.0, 'x': 0.0, 'y': 0.0, 'got': False}
_cmd       = {'v': 0.0, 'w': 0.0}


def ser_write(ser: serial.Serial, data: bytes):
    """모든 시리얼 쓰기를 _io_lock 으로 직렬화 — 두 스레드의 바이트 인터리빙 방지."""
    with _io_lock:
        ser.write(data)


# ── 수신 스레드: 'O:x,y,heading' 파싱 ─────────────────────────────────────────
def rx_thread(ser: serial.Serial):
    while _running.is_set():
        try:
            raw = ser.readline()
            if not raw:
                continue
            if not raw.endswith(b'\n'):
                continue              # 타임아웃으로 중간에 끊긴 부분 프레임 → 버림
            line = raw.decode('utf-8', errors='replace').strip()
            if not line.startswith('O:'):
                if line:
                    print(f'\r[ARD] {line}')
                continue
            parts = line[2:].split(',')
            if len(parts) != 3:
                continue
            with _lock:
                _state['x']       = float(parts[0])
                _state['y']       = float(parts[1])
                _state['heading'] = float(parts[2])
                _state['got']     = True
        except serial.SerialException:
            break
        except Exception:
            pass


# ── 송신 스레드: 현재 명령을 주기적으로 재전송 (타임아웃 방지) ────────────────
def tx_thread(ser: serial.Serial):
    while _running.is_set():
        if _tx_paused.is_set():
            time.sleep(SEND_DT)
            continue
        with _lock:
            v, w = _cmd['v'], _cmd['w']
        try:
            ser_write(ser, f'{v:.2f} {w:.2f}\n'.encode())
        except serial.SerialException:
            break
        time.sleep(SEND_DT)


# ── 헬퍼 ──────────────────────────────────────────────────────────────────────
def set_cmd(v: float, w: float):
    with _lock:
        _cmd['v'] = v
        _cmd['w'] = w


def read_state():
    with _lock:
        return _state['heading'], _state['x'], _state['y']


def reset_odom(ser: serial.Serial) -> bool:
    """
    헤딩/위치 0 리셋 (R 명령). tx 를 잠시 멈춰 R 라인을 단독 전송하고,
    리셋 직후의 O: 텔레메트리가 heading≈0 인지 확인(최대 3회 재시도).
    성공 True / 검증 실패 False.
    """
    set_cmd(0.0, 0.0)
    _tx_paused.set()
    time.sleep(SEND_DT * 2)        # 진행 중인 tx write 가 드레인될 시간
    try:
        for _ in range(3):
            ser_write(ser, b'R\n')
            time.sleep(0.15)       # 아두이노 R 처리 + 직전(리셋 前) O: 라인 드레인
            with _lock:
                _state['got'] = False
            t0 = time.time()
            while time.time() - t0 < 0.5:
                with _lock:
                    got, hdg = _state['got'], _state['heading']
                if got:
                    if abs(hdg) < 2.0:
                        with _lock:
                            _state['x'] = 0.0
                            _state['y'] = 0.0
                        return True
                    break          # 새 샘플인데 heading≉0 → R 재전송
                time.sleep(0.02)
        print('  [경고] 헤딩 리셋 확인 실패 — 결과 신뢰도 낮음')
        return False
    finally:
        _tx_paused.clear()


def rotate_to(target_deg: float) -> float:
    """
    odom heading 이 target_deg 에 도달할 때까지 폐루프 회전 후 정지.
    회전 방향은 (target - 현재헤딩) 부호로 고정 → 목표 통과 시 즉시 정지(헌팅 방지).
    스톨/타임아웃 시 안전 정지. 정지·안정화 후의 실제 odom heading 을 반환.
    """
    start_hdg, _, _ = read_state()
    direction = 1.0 if (target_deg - start_hdg) >= 0 else -1.0

    t0 = time.time()
    last_prog_t = t0
    last_hdg    = start_hdg

    while True:
        hdg, x, y = read_state()
        err = target_deg - hdg

        # 목표 도달(허용오차 이내) 또는 목표 통과(부호 반전) → 정지
        if abs(err) <= STOP_TOL_DEG or (err * direction) <= 0.0:
            break

        mag = KP_W * abs(err)
        mag = max(W_MIN, min(W_MAX, mag))
        set_cmd(0.0, direction * mag)

        now = time.time()
        # 진행 감시(스톨/엔코더 무응답)
        if abs(hdg - last_hdg) > 1.0:
            last_hdg = hdg
            last_prog_t = now
        if now - last_prog_t > STALL_TIMEOUT_S:
            set_cmd(0.0, 0.0)
            print('\n  [경고] 회전 진전 없음(스톨/엔코더 무응답) → 안전 정지')
            time.sleep(SETTLE_S)
            return read_state()[0]
        if now - t0 > SPIN_TIMEOUT_S:
            set_cmd(0.0, 0.0)
            print('\n  [경고] 회전 타임아웃 → 안전 정지')
            time.sleep(SETTLE_S)
            return read_state()[0]

        sys.stdout.write(
            f'\r  [회전중] odom={hdg:+8.1f}°  목표={target_deg:+8.1f}°  '
            f'남음={err:+7.1f}°  pos=({x:+5.0f},{y:+5.0f})mm   ')
        sys.stdout.flush()
        time.sleep(CTRL_DT)

    set_cmd(0.0, 0.0)
    time.sleep(SETTLE_S)        # 관성 정지 + 오도메트리 안정화
    sys.stdout.write('\n')
    return read_state()[0]


def ask_turns() -> int:
    while True:
        raw = input(f'  회전할 바퀴 수 N 입력 [기본 {DEFAULT_TURNS}]: ').strip()
        if raw == '':
            return DEFAULT_TURNS
        try:
            n = int(raw)
            if n > 0:
                return n
        except ValueError:
            pass
        print('  → 1 이상의 정수를 입력하세요.')


def run_trial(ser: serial.Serial, idx: int):
    """자동 회전 1회. 성공 시 (WHEEL_BASE_참값, θ_odom, N) 반환, 버리면 None."""
    print(f'\n──────────────  자동 회전 측정 #{idx}  ──────────────')
    print('  로봇 정면 표식을 바닥 기준선에 정렬하세요.')
    if input('  준비되면 Enter (건너뛰기: s+Enter): ').strip().lower() == 's':
        return None

    n_turns = ask_turns()
    target  = DIRECTION * n_turns * 360.0

    reset_odom(ser)
    dir_txt = 'CCW(반시계)' if DIRECTION > 0 else 'CW(시계)'
    print(f'  ▶ {n_turns}바퀴 {dir_txt} 자동 회전 시작 (목표 odom {target:+.0f}°)...')

    theta_odom = rotate_to(target)
    _, x, y = read_state()
    est_turns = theta_odom / 360.0
    drift     = math.hypot(x, y)

    print(f'  ── 결과 확인 ──────────────────────────────────')
    print(f'    목표 odom      : {target:+.1f}°')
    print(f'    측정 odom θ    : {theta_odom:+.1f}°  (≈ {est_turns:+.3f}바퀴)')
    print(f'    위치 드리프트  : ({x:+.0f}, {y:+.0f}) mm  |거리 {drift:.0f}mm|'
          f'  ← 제자리 회전이면 0 에 가까워야 함')

    # 물리 회전각 입력 → WHEEL_BASE 역산
    print('  실제 물리 회전 결과 입력:')
    print('     a       → 마크에 정확히 정렬됨 (실제 = N×360°)')
    print('     <숫자>  → 마크에서 어긋난 각도[deg] (회전방향 +, 반대 −)')
    print('     s       → 이번 측정 버림')
    while True:
        ans = input('  > ').strip().lower()
        if ans == 's':
            return None
        if ans == 'a':
            residual = 0.0
            break
        try:
            residual = float(ans)
            break
        except ValueError:
            print('  → a / 숫자 / s 중 하나를 입력하세요.')

    theta_phys = n_turns * 360.0 + residual
    if abs(theta_phys) < 1e-6:
        print('  [오류] 물리 회전각이 0 — 측정 버림.')
        return None

    # 부호 무관, 크기로 계산 (회전 방향에 독립)
    wb_true = WHEEL_BASE_CURRENT * abs(theta_odom) / abs(theta_phys)
    err_per_turn = (abs(theta_odom) - abs(theta_phys)) / n_turns
    print(f'  → θ_phys = {theta_phys:+.1f}°   '
          f'WHEEL_BASE 참값 = {wb_true:.5f} m   '
          f'(현재 {WHEEL_BASE_CURRENT:.5f} m, 오차 {err_per_turn:+.2f}°/바퀴)')
    return (wb_true, theta_odom, n_turns)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else UART_PORT

    print('=' * 62)
    print('  WHEEL_BASE 자동 회전 보정 (폐루프 N바퀴 자동 회전)')
    print(f'  포트: {port}   보드레이트: {BAUD_RATE}')
    print(f'  현재 WHEEL_BASE = {WHEEL_BASE_CURRENT:.5f} m  '
          f'(arduino_code.cpp 값과 일치해야 함)')
    print(f'  회전 방향 = {"CCW(반시계)" if DIRECTION > 0 else "CW(시계)"}')
    print('=' * 62)

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    except serial.SerialException as e:
        print(f'[오류] 포트 열기 실패: {e}')
        sys.exit(1)

    time.sleep(0.3)

    _running.set()
    rx = threading.Thread(target=rx_thread, args=(ser,), daemon=True)
    tx = threading.Thread(target=tx_thread, args=(ser,), daemon=True)
    rx.start()
    tx.start()

    print('\n[INFO] 오도메트리 수신 확인 중...')
    t0 = time.time()
    while time.time() - t0 < 3.0:
        with _lock:
            ok = _state['got']
        if ok:
            print('[INFO] 오도메트리 수신 OK.')
            break
        time.sleep(0.1)
    else:
        print('[WARN] 오도메트리(O:) 미수신. 배선/포트/아두이노를 확인하세요.')

    results = []
    try:
        idx = 1
        while True:
            r = run_trial(ser, idx)
            if r is not None:
                results.append(r)
                idx += 1
            if input('\n  계속 측정? [Enter=예 / q=종료 후 결과]: ').strip().lower() == 'q':
                break
    except KeyboardInterrupt:
        print('\n[INFO] Ctrl+C — 측정 종료.')
    finally:
        set_cmd(0.0, 0.0)
        time.sleep(0.2)
        _running.clear()
        time.sleep(SEND_DT * 2)         # tx_thread 종료 대기
        try:
            ser_write(ser, b'0.00 0.00\n')   # 확실히 정지
        except serial.SerialException:
            pass
        ser.close()

    # ── 결과 요약 ─────────────────────────────────────────────────────────────
    print('\n' + '=' * 62)
    if not results:
        print('  유효한 측정 없음. 종료.')
        print('=' * 62)
        return

    wbs = [r[0] for r in results]
    avg = sum(wbs) / len(wbs)
    vmin, vmax = min(wbs), max(wbs)
    spread = vmax - vmin
    scale = avg / WHEEL_BASE_CURRENT

    print(f'  측정 횟수      : {len(results)}')
    for i, (wb, th, n) in enumerate(results, 1):
        print(f'    #{i}: WB={wb:.5f} m  (θ_odom={th:+.1f}°, N={n})')
    print('  ' + '-' * 58)
    print(f'  WHEEL_BASE 평균: {avg:.5f} m')
    print(f'  분포(min~max)  : {vmin:.5f} ~ {vmax:.5f}  (편차 {spread*1000:.2f} mm)')
    print(f'  보정 배율      : ×{scale:.4f}  (현재 {WHEEL_BASE_CURRENT:.5f} m 대비)')
    print('  ' + '-' * 58)
    print('  → arduino_code.cpp 수정:')
    print(f'       const float WHEEL_BASE = {avg:.4f}f;')
    print('     수정 후 아두이노 재업로드하면 회전 오도메트리가 보정됩니다.')
    if spread * 1000 > 3.0:
        print('  [주의] 측정 편차가 큽니다(>3mm). 회전을 더 천천히(W_MIN↓) 하거나')
        print('         바퀴 수 N 을 늘려(5바퀴+) 재측정을 권장합니다.')
    print('=' * 62)


if __name__ == '__main__':
    main()

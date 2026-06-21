#!/usr/bin/env python3
"""
WHEEL_BASE (좌우 바퀴 간격) 오도메트리 회전 보정 스크립트
===========================================================
로봇을 제자리에서 정확히 N바퀴(N×360°) 회전시킨 뒤, 아두이노 오도메트리가
보고한 누적 헤딩(θ_odom)과 실제 물리 회전각(N×360°)을 비교해 WHEEL_BASE 참값을
역산합니다.

[보정 원리]
  엔코더가 측정한 좌우 바퀴 이동량 차이(Δarc = dsR - dsL)는 물리적 사실로 고정.
  같은 Δarc 가 두 식에 동시에 들어간다:

      θ_odom      = Δarc / WHEEL_BASE_현재     (아두이노가 보고한 헤딩)
      θ_physical  = Δarc / WHEEL_BASE_참값      (실제 회전, = N×360°)

  ∴ WHEEL_BASE_참값 = WHEEL_BASE_현재 × θ_odom / (360 × N)

  · θ_odom > 360·N  →  실제 바퀴 간격이 현재값보다 넓음 (값을 키워야 함)
  · θ_odom < 360·N  →  실제 바퀴 간격이 현재값보다 좁음 (값을 줄여야 함)
  · N바퀴를 돌릴수록 수동 정렬 오차가 1/N 로 줄어든다 → 3~5바퀴 권장.

[준비물]
  · 바닥 기준선(테이프) 1개 + 로봇 정면을 가리키는 표식(테이프) 1개.
  · 회전 시작 시 둘을 정렬해두고, N바퀴 후 다시 정렬되는 순간을 눈으로 포착.

[사용법]
  SSH로 라즈베리파이 접속 후:
      python3 calibrate_wheelbase.py                 # 기본 포트 /dev/ttyAMA3
      python3 calibrate_wheelbase.py /dev/ttyS0      # 포트 직접 지정

  1. 로봇 정면 표식을 바닥 기준선에 정렬한다.
  2. Enter → 헤딩 리셋 후 제자리 회전 시작 (천천히 회전함).
  3. N바퀴 돈 뒤 표식이 기준선에 다시 정렬되는 순간 Enter → 정지.
  4. 실제로 돈 정수 바퀴 수 N 입력 (화면의 odom 추정치를 참고).
  5. 여러 회 반복 후 [q] → 평균 WHEEL_BASE 결과 출력.
  6. 출력값을 arduino_code.cpp 의 `const float WHEEL_BASE` 에 반영 후 재업로드.
"""

import serial
import threading
import sys
import time

# ── 설정 ──────────────────────────────────────────────────────────────────────
UART_PORT = '/dev/ttyAMA3'
BAUD_RATE = 115200

# 현재 아두이노에 업로드된 값과 반드시 동일하게! (arduino_code.cpp 의 WHEEL_BASE)
WHEEL_BASE_CURRENT = 0.1802  # [m]

PIVOT_W      = 0.8    # 제자리 회전 각속도 [rad/s] (+:CCW / 느릴수록 슬립↓ 정렬↑)
SEND_DT      = 0.05   # 명령 전송 주기 [s] (아두이노 타임아웃 500ms 보다 짧게)
SETTLE_S     = 0.6    # 정지 후 오도메트리 안정화 대기 [s]
DEFAULT_TURNS = 3     # 권장 회전 바퀴 수

# ── 공유 상태 ─────────────────────────────────────────────────────────────────
_lock      = threading.Lock()         # _state / _cmd 보호
_io_lock   = threading.Lock()         # 시리얼 write 직렬화 (R 라인과 v/w 라인 섞임 방지)
_running   = threading.Event()        # 송신 스레드 생존 플래그
_tx_paused = threading.Event()        # set 동안 tx_thread 송신 일시정지 (리셋 시 R 단독 전송)
_state     = {'heading': 0.0, 'x': 0.0, 'y': 0.0, 'got': False}
_cmd       = {'v': 0.0, 'w': 0.0}      # 송신 스레드가 계속 내보내는 현재 명령


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
                # 아두이노 디버그 메시지는 그대로 흘려보냄
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


def read_heading() -> float:
    with _lock:
        return _state['heading']


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
                        return True
                    break          # 새 샘플인데 heading≉0 → R 재전송
                time.sleep(0.02)
        print('  [경고] 헤딩 리셋 확인 실패 — 결과 신뢰도 낮음')
        return False
    finally:
        _tx_paused.clear()


def live_status_thread(stop_evt: threading.Event):
    """회전 중 odom 헤딩/추정 바퀴 수를 실시간 표시 (참고용)."""
    while not stop_evt.is_set():
        hdg = read_heading()
        turns = hdg / 360.0
        sys.stdout.write(f'\r  [회전중] odom={hdg:+8.1f}°  ≈ {turns:+5.2f}바퀴   ')
        sys.stdout.flush()
        time.sleep(0.1)
    sys.stdout.write('\n')


def run_trial(ser: serial.Serial, idx: int):
    """한 번의 회전 측정. 성공 시 (WHEEL_BASE_참값, θ_odom, N) 반환, 취소 시 None."""
    print(f'\n──────────────  측정 #{idx}  ──────────────')
    print('  로봇 정면 표식을 바닥 기준선에 정렬하세요.')
    ans = input('  준비되면 Enter (이 측정 건너뛰기: s+Enter): ').strip().lower()
    if ans == 's':
        return None

    reset_odom(ser)
    print(f'  ▶ 회전 시작! {DEFAULT_TURNS}바퀴 권장 — '
          f'표식이 기준선에 다시 정렬되면 Enter.')

    # 회전 시작 + 실시간 상태 표시
    set_cmd(0.0, PIVOT_W)
    stop_evt = threading.Event()
    live = threading.Thread(target=live_status_thread, args=(stop_evt,), daemon=True)
    live.start()

    try:
        input()                      # 정렬되는 순간 Enter
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        set_cmd(0.0, 0.0)            # 즉시 정지
        stop_evt.set()
        live.join(timeout=1.0)

    time.sleep(SETTLE_S)             # 오도메트리 안정화
    theta_odom = read_heading()
    est_turns  = theta_odom / 360.0
    print(f'  odom 누적 헤딩 θ_odom = {theta_odom:+.1f}°  (≈ {est_turns:+.2f}바퀴)')

    # 실제 물리 바퀴 수 입력
    while True:
        raw = input(f'  실제로 돈 바퀴 수 N 입력 [기본 {round(abs(est_turns)) or DEFAULT_TURNS}]: ').strip()
        if raw == '':
            n_turns = round(abs(est_turns)) or DEFAULT_TURNS
            break
        try:
            n_turns = int(raw)
            if n_turns != 0:
                break
        except ValueError:
            pass
        print('  → 0 이 아닌 정수를 입력하세요.')

    theta_phys = 360.0 * n_turns
    # 회전 방향 부호 정합: θ_odom 와 θ_phys 의 부호를 맞춰 크기로 계산
    wb_true = WHEEL_BASE_CURRENT * abs(theta_odom) / abs(theta_phys)

    err_deg_per_turn = (abs(theta_odom) - abs(theta_phys)) / n_turns
    print(f'  → WHEEL_BASE 참값 = {wb_true:.5f} m   '
          f'(현재 {WHEEL_BASE_CURRENT:.5f} m, '
          f'오차 {err_deg_per_turn:+.2f}°/바퀴)')
    return (wb_true, theta_odom, n_turns)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else UART_PORT

    print('=' * 60)
    print('  WHEEL_BASE (좌우 바퀴 간격) 회전 오도메트리 보정')
    print(f'  포트: {port}   보드레이트: {BAUD_RATE}')
    print(f'  현재 WHEEL_BASE = {WHEEL_BASE_CURRENT:.5f} m  '
          f'(arduino_code.cpp 값과 일치해야 함)')
    print(f'  회전 각속도 = {PIVOT_W:+.2f} rad/s  '
          f'({"CCW(반시계)" if PIVOT_W > 0 else "CW(시계)"})')
    print('=' * 60)

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    except serial.SerialException as e:
        print(f'[오류] 포트 열기 실패: {e}')
        sys.exit(1)

    time.sleep(0.3)  # UART 안정화

    _running.set()
    rx = threading.Thread(target=rx_thread, args=(ser,), daemon=True)
    tx = threading.Thread(target=tx_thread, args=(ser,), daemon=True)
    rx.start()
    tx.start()

    # 오도메트리 수신 확인
    print('\n[INFO] 오도메트리 수신 확인 중...')
    t0 = time.time()
    while time.time() - t0 < 3.0:
        with _lock:
            ok = _state['got']
        if ok:
            print('[INFO] 오도메트리 수신 OK.\n')
            break
        time.sleep(0.1)
    else:
        print('[WARN] 오도메트리(O:) 미수신. 배선/포트/아두이노를 확인하세요.\n')

    results = []
    try:
        idx = 1
        while True:
            r = run_trial(ser, idx)
            if r is not None:
                results.append(r)
                idx += 1
            cont = input('\n  계속 측정? [Enter=예 / q=종료 후 결과]: ').strip().lower()
            if cont == 'q':
                break
    except KeyboardInterrupt:
        print('\n[INFO] Ctrl+C — 측정 종료.')
    finally:
        set_cmd(0.0, 0.0)
        time.sleep(0.2)
        _running.clear()
        time.sleep(SEND_DT * 2)
        try:
            ser_write(ser, b'0.00 0.00\n')   # 확실히 정지
        except serial.SerialException:
            pass
        ser.close()

    # ── 결과 요약 ─────────────────────────────────────────────────────────────
    print('\n' + '=' * 60)
    if not results:
        print('  유효한 측정 없음. 종료.')
        print('=' * 60)
        return

    wbs = [r[0] for r in results]
    avg = sum(wbs) / len(wbs)
    vmin, vmax = min(wbs), max(wbs)
    spread = vmax - vmin
    scale = avg / WHEEL_BASE_CURRENT

    print(f'  측정 횟수      : {len(results)}')
    for i, (wb, th, n) in enumerate(results, 1):
        print(f'    #{i}: WB={wb:.5f} m  (θ_odom={th:+.1f}°, N={n})')
    print('  ' + '-' * 56)
    print(f'  WHEEL_BASE 평균: {avg:.5f} m')
    print(f'  분포(min~max)  : {vmin:.5f} ~ {vmax:.5f}  (편차 {spread*1000:.2f} mm)')
    print(f'  보정 배율      : ×{scale:.4f}  (현재 {WHEEL_BASE_CURRENT:.5f} m 대비)')
    print('  ' + '-' * 56)
    print('  → arduino_code.cpp 수정:')
    print(f'       const float WHEEL_BASE = {avg:.4f}f;')
    print('     수정 후 아두이노 재업로드하면 회전 오도메트리가 보정됩니다.')
    if spread * 1000 > 3.0:
        print('  [주의] 측정 편차가 큽니다(>3mm). 회전을 더 천천히 하거나')
        print('         바퀴 수 N 을 늘려(5바퀴+) 재측정을 권장합니다.')
    print('=' * 60)


if __name__ == '__main__':
    main()

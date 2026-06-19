#!/usr/bin/env python3
"""
모터 FF 캘리브레이션 원격 제어 스크립트
SSH로 라즈베리파이에 접속한 후 실행
아두이노와 UART(Serial1, /dev/ttyAMA0)로 통신

사용법:
  python3 cal_ff_remote.py              # 기본 포트 /dev/ttyAMA0
  python3 cal_ff_remote.py /dev/ttyS0  # 포트 직접 지정
"""

import serial
import threading
import sys
import time

# ── 설정 ──────────────────────────────────────────────────────────────────────
UART_PORT  = '/dev/ttyAMA3'
BAUD_RATE  = 115200
CAL_STEPS  = 18  # 아두이노와 동일

# ── 상태 공유 ─────────────────────────────────────────────────────────────────
_waiting_for_n = threading.Event()   # 아두이노가 'n' 대기 중일 때 set
_cal_done      = threading.Event()   # 캘리브레이션 완료
_lock          = threading.Lock()

_step_info = {'pwm': 0, 'round': 0}  # 현재 진행 단계 표시용


def rx_thread(ser: serial.Serial):
    """
    아두이노 출력을 실시간으로 수신하여 터미널에 표시.
    [준비] 메시지를 감지하면 _waiting_for_n 이벤트를 set.
    """
    while not _cal_done.is_set():
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='replace').rstrip()
            if not line:
                continue

            # 오도메트리는 캘리브레이션 중 화면에 표시하지 않음
            if line.startswith('O:'):
                continue

            print(f'\r[ARD] {line}')

            # [준비] — 아두이노가 'n' 대기 중
            if '[준비]' in line:
                # "PWM 50 (1/3)" 파싱해서 저장
                try:
                    parts = line.split()
                    pwm_idx   = parts.index('PWM') + 1
                    round_str = parts[pwm_idx + 1]          # "(1/3)"
                    with _lock:
                        _step_info['pwm']   = float(parts[pwm_idx])
                        _step_info['round'] = int(round_str[1])
                except Exception:
                    pass
                _waiting_for_n.set()

            # 캘리브레이션 완료 감지
            elif '[CAL] 완료' in line:
                _cal_done.set()

        except serial.SerialException:
            break
        except Exception:
            pass


def send(ser: serial.Serial, cmd: str):
    ser.write((cmd + '\n').encode())


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else UART_PORT

    print('=' * 55)
    print('  모터 FF 캘리브레이션 원격 제어')
    print(f'  포트: {port}  /   보드레이트: {BAUD_RATE}')
    print()
    print('  조작 방법:')
    print('    Enter      → 다음 측정 진행 (n)')
    print('    q + Enter  → 캘리브레이션 중단')
    print('    Ctrl+C     → 강제 종료')
    print('=' * 55)

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    except serial.SerialException as e:
        print(f'[오류] 포트 열기 실패: {e}')
        sys.exit(1)

    time.sleep(0.3)  # UART 안정화

    # RX 스레드 시작
    t = threading.Thread(target=rx_thread, args=(ser,), daemon=True)
    t.start()

    print('\n[INFO] 캘리브레이션 시작 명령 전송...')
    send(ser, 'CAL')

    step_count = 0
    total_steps = CAL_STEPS * 3  # 18 PWM 단계 × 3회

    try:
        while not _cal_done.is_set():
            # [준비] 메시지 대기 (최대 120초 — 측정+이동 시간 고려)
            triggered = _waiting_for_n.wait(timeout=120)
            if not triggered:
                print('\n[WARN] 120초 응답 없음. 아두이노 상태를 확인하세요.')
                break

            _waiting_for_n.clear()

            with _lock:
                pwm   = _step_info['pwm']
                rnd   = _step_info['round']

            step_count += 1
            print(f'\n  [{step_count}/{total_steps}] PWM={pwm:.0f}  측정 {rnd}/3 회')
            if rnd > 1:
                print('  → 로봇을 출발선으로 이동하세요.')

            try:
                user_in = input('  [?] 준비되면 Enter  (중단: q+Enter): ').strip().lower()
            except EOFError:
                user_in = 'q'

            if user_in == 'q':
                send(ser, 'q')
                print('\n[INFO] 캘리브레이션 중단 신호 전송.')
                break
            else:
                send(ser, 'n')

    except KeyboardInterrupt:
        print('\n[INFO] Ctrl+C — 중단 신호 전송.')
        send(ser, 'q')
    finally:
        _cal_done.set()
        time.sleep(0.5)
        ser.close()
        print('[INFO] 종료.')


if __name__ == '__main__':
    main()

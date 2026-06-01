"""
RPLIDAR C1 직접 시리얼 드라이버
===============================
표준 `rplidar` 라이브러리는 A1/A2용으로 만들어져 C1의 GET_INFO/GET_HEALTH
응답과 호환되지 않음. 이 모듈은 C1 프로토콜에 맞춰 최소 기능만 구현.

표준 라이브러리와 동일한 API를 제공하므로 import만 바꾸면 됨:
    # from rplidar import RPLidar          ← 기존
    from rplidar_c1 import RPLidar         ← 변경

사용법은 동일:
    lidar = RPLidar('/dev/ttyUSB0')        # baudrate 자동 460800
    for scan in lidar.iter_scans():
        for (quality, angle_deg, distance_mm) in scan:
            ...
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
"""

import time
import serial


# ============================================================
# SLAMTEC RPLIDAR C1 프로토콜 상수
# ============================================================
SYNC_BYTE      = 0xA5
SYNC_BYTE2     = 0x5A

CMD_STOP       = 0x25
CMD_RESET      = 0x40
CMD_SCAN       = 0x20
CMD_FORCE_SCAN = 0x21


class RPLidarException(Exception):
    pass


class RPLidar:
    """RPLIDAR C1용 최소 드라이버 (표준 rplidar 라이브러리 호환)."""

    DEFAULT_BAUDRATE = 460800   # C1 전용 기본 baudrate

    def __init__(self, port, baudrate=None, timeout=1, logger=None):
        if baudrate is None:
            baudrate = self.DEFAULT_BAUDRATE

        self.port     = port
        self.baudrate = baudrate
        self.timeout  = timeout

        self._serial = serial.Serial(port, baudrate, timeout=timeout)
        self._scan_started = False

        # 잔여 명령/버퍼 정리
        self.stop()
        time.sleep(0.1)

    # --------------------------------------------------------
    # 저수준 명령
    # --------------------------------------------------------
    def _send_cmd(self, cmd):
        self._serial.write(bytes([SYNC_BYTE, cmd]))
        self._serial.flush()

    def _read_descriptor(self):
        """7-byte descriptor (스캔 시작 시 헤더). C1도 동일 포맷."""
        descriptor = self._serial.read(7)
        if len(descriptor) != 7:
            raise RPLidarException(
                f'Descriptor length mismatch: got {len(descriptor)} bytes')
        if descriptor[0] != SYNC_BYTE or descriptor[1] != SYNC_BYTE2:
            raise RPLidarException(
                f'Bad sync bytes: {descriptor[:2].hex()}')
        return descriptor

    # --------------------------------------------------------
    # 공개 API
    # --------------------------------------------------------
    def stop(self):
        """스캔 중지."""
        self._send_cmd(CMD_STOP)
        time.sleep(0.01)
        self._scan_started = False
        self._serial.reset_input_buffer()

    def stop_motor(self):
        """C1은 모터 PWM 별도 제어선이 없어서 stop()으로 충분."""
        self.stop()

    def disconnect(self):
        """연결 종료."""
        try:
            self.stop()
        except Exception:
            pass
        time.sleep(0.05)
        self._serial.close()

    def reset(self):
        """디바이스 리셋."""
        self._send_cmd(CMD_RESET)
        time.sleep(0.5)
        self._serial.reset_input_buffer()
        self._scan_started = False

    def get_info(self):
        """호환성용 더미 반환 (C1은 응답 포맷이 달라 신뢰성 낮음)."""
        return {'model': 'RPLIDAR C1', 'firmware': '?.?',
                'hardware': '?', 'serialnumber': '?'}

    def get_health(self):
        """호환성용 더미 반환 — 항상 healthy로 가정."""
        return ('Good', 0)

    # --------------------------------------------------------
    # 스캔 데이터 파싱
    # --------------------------------------------------------
    def _start_scan(self):
        if self._scan_started:
            return
        self._send_cmd(CMD_SCAN)
        self._read_descriptor()
        self._scan_started = True

    def iter_measurements(self):
        """5-byte 표준 스캔 패킷을 파싱해서
        (new_scan, quality, angle_deg, distance_mm) 튜플을 yield.

        패킷 포맷:
            byte0: [Q7..Q2 | ~S | S]     S=new_scan, ~S=inverted (parity)
            byte1: [A6..A0 | C]          C=check bit (반드시 1)
            byte2: [A14..A7]
            byte3: distance_lo
            byte4: distance_hi
            angle    = ((byte2<<7) | (byte1>>1)) / 64.0    [degrees]
            distance = ((byte4<<8) |  byte3   ) / 4.0      [mm]
        """
        self._start_scan()

        while True:
            packet = self._serial.read(5)
            if len(packet) != 5:
                continue  # 타임아웃 - 재시도

            b0, b1, b2, b3, b4 = packet

            # Parity 체크: byte0의 bit0와 bit1는 서로 반대여야 함
            new_scan     = b0 & 0x01
            inv_new_scan = (b0 >> 1) & 0x01
            if new_scan == inv_new_scan:
                # 동기 깨짐 - 1바이트 흘려보내고 재시도
                self._serial.read(1)
                continue

            # Check bit (byte1 bit0) must be 1
            if not (b1 & 0x01):
                self._serial.read(1)
                continue

            quality  = b0 >> 2
            angle    = ((b2 << 7) | (b1 >> 1)) / 64.0
            distance = ((b4 << 8) | b3) / 4.0

            yield (bool(new_scan), quality, angle, distance)

    def iter_scans(self, max_buf_meas=2000, min_len=5):
        """한 회전(360°) 분량의 측정값을 list로 yield.

        반환 형식: list of (quality, angle_deg, distance_mm)
                   ← 표준 rplidar 라이브러리와 동일
        """
        scan = []
        for new_scan, quality, angle, distance in self.iter_measurements():
            if new_scan and len(scan) >= min_len:
                yield scan
                scan = []
            if len(scan) >= max_buf_meas:
                yield scan
                scan = []
            if quality > 0 and distance > 0:
                scan.append((quality, angle, distance))


# ============================================================
# 단독 실행 시 간단 테스트
# ============================================================
if __name__ == '__main__':
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    print(f"[Test] {port} 연결 중...")

    lidar = RPLidar(port)
    print(f"[Test] info: {lidar.get_info()}")
    print(f"[Test] health: {lidar.get_health()}")
    print("[Test] 5회 스캔 출력 (Ctrl+C 종료)")

    try:
        for i, scan in enumerate(lidar.iter_scans()):
            print(f"  Scan {i}: {len(scan)} measurements, "
                  f"first 3 = {scan[:3]}")
            if i >= 4:
                break
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        lidar.disconnect()
        print("[Test] 종료")

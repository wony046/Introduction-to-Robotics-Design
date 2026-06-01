"""
RPLIDAR C1 직접 시리얼 드라이버
===============================
표준 `rplidar` 라이브러리는 A1/A2용으로 만들어져 C1의 GET_INFO/GET_HEALTH
응답과 호환되지 않음. 이 모듈은 C1 프로토콜에 맞춰 최소 기능만 구현.

표준 라이브러리와 동일한 API를 제공하므로 import만 바꾸면 됨:
    # from rplidar import RPLidar          ← 기존
    from rplidar_c1 import RPLidar         ← 변경
"""

import time
import serial

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

    DEFAULT_BAUDRATE = 460800

    def __init__(self, port, baudrate=None, timeout=1, logger=None):
        if baudrate is None:
            baudrate = self.DEFAULT_BAUDRATE
        self.port     = port
        self.baudrate = baudrate
        self.timeout  = timeout
        self._serial = serial.Serial(port, baudrate, timeout=timeout)
        self._scan_started = False
        self.stop()
        time.sleep(0.1)

    def _send_cmd(self, cmd):
        self._serial.write(bytes([SYNC_BYTE, cmd]))
        self._serial.flush()

    def _read_descriptor(self):
        descriptor = self._serial.read(7)
        if len(descriptor) != 7:
            raise RPLidarException(
                f'Descriptor length mismatch: got {len(descriptor)} bytes')
        if descriptor[0] != SYNC_BYTE or descriptor[1] != SYNC_BYTE2:
            raise RPLidarException(
                f'Bad sync bytes: {descriptor[:2].hex()}')
        return descriptor

    def stop(self):
        self._send_cmd(CMD_STOP)
        time.sleep(0.01)
        self._scan_started = False
        self._serial.reset_input_buffer()

    def stop_motor(self):
        self.stop()

    def disconnect(self):
        try:
            self.stop()
        except Exception:
            pass
        time.sleep(0.05)
        self._serial.close()

    def reset(self):
        self._send_cmd(CMD_RESET)
        time.sleep(0.5)
        self._serial.reset_input_buffer()
        self._scan_started = False

    def get_info(self):
        return {'model': 'RPLIDAR C1', 'firmware': '?.?',
                'hardware': '?', 'serialnumber': '?'}

    def get_health(self):
        return ('Good', 0)

    def _start_scan(self):
        if self._scan_started:
            return
        self._send_cmd(CMD_SCAN)
        self._read_descriptor()
        self._scan_started = True

    def iter_measurements(self):
        self._start_scan()
        while True:
            packet = self._serial.read(5)
            if len(packet) != 5:
                continue
            b0, b1, b2, b3, b4 = packet
            new_scan     = b0 & 0x01
            inv_new_scan = (b0 >> 1) & 0x01
            if new_scan == inv_new_scan:
                self._serial.read(1)
                continue
            if not (b1 & 0x01):
                self._serial.read(1)
                continue
            quality  = b0 >> 2
            angle    = ((b2 << 7) | (b1 >> 1)) / 64.0
            distance = ((b4 << 8) | b3) / 4.0
            yield (bool(new_scan), quality, angle, distance)

    def iter_scans(self, max_buf_meas=2000, min_len=5):
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
            print(f"  Scan {i}: {len(scan)} measurements, first 3 = {scan[:3]}")
            if i >= 4:
                break
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        lidar.disconnect()
        print("[Test] 종료")

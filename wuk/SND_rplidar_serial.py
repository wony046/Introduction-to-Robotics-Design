"""
rplidar_serial.py
==================
RPLIDAR C1 직접 시리얼 통신 드라이버.
SDK / rplidar 라이브러리 없이 protocol 명세에 따라 packet 단위로 통신.

수업자료 reference
-----------------
- 실습_04 LiDAR와 Serial통신.pdf
  · practice1: RESET, SCAN 명령 송신 + raw hex 출력
  · practice2: 5바이트 measurement packet 파싱 (quality, angle_q6, distance_q2)
  · practice3: S, S̄, C 검증 비트 적용
  · practice4: 시각화

지원 명령
---------
- STOP   (0xA5 0x25) : No response.  ≥10 ms 대기 필요
- RESET  (0xA5 0x40) : No response.  ≥500 ms 대기 필요
- SCAN   (0xA5 0x20) : Multiple response. 5-byte measurement stream

Measurement packet (5 byte)
---------------------------
  Byte 0 : Quality(7..2) | S̄(1) | S(0)
  Byte 1 : angle_q6[6:0](7..1) | C(0)
  Byte 2 : angle_q6[14:7]
  Byte 3 : distance_q2[7:0]
  Byte 4 : distance_q2[15:8]

  angle°    = angle_q6 / 64.0
  distance  = distance_q2 / 4.0   (mm)
"""

import time
import serial


# ============================================================
# Protocol constants
# ============================================================
SYNC_BYTE       = 0xA5
SYNC_BYTE2      = 0x5A
CMD_STOP        = 0x25
CMD_RESET       = 0x40
CMD_SCAN        = 0x20

DESCRIPTOR_LEN  = 7
SCAN_PACKET_LEN = 5

STOP_DELAY_MS   = 15        # spec: ≥10
RESET_DELAY_MS  = 800       # spec: ≥500, 여유 두고 800
SCAN_WARMUP_MS  = 200       # 모터 회전 안정화 후 data 출력 시작까지 여유


# ============================================================
# 드라이버 클래스
# ============================================================
class RPLidarSerial:
    """RPLIDAR C1 / A1 호환 직접 시리얼 드라이버."""

    def __init__(self, port="/dev/ttyUSB0", baudrate=460800, timeout=1.0):
        self.port     = port
        self.baudrate = baudrate
        self._scanning = False
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(0.1)

    # ----------------------------------------------------------
    # 저수준 송수신
    # ----------------------------------------------------------
    def _send_cmd(self, cmd_byte):
        """Request packet: [0xA5 | CMD]  (payload-less 명령)."""
        self.ser.write(bytes([SYNC_BYTE, cmd_byte]))
        self.ser.flush()

    def _read_descriptor(self):
        """
        Multiple Response Mode 명령 후 7 byte response descriptor 읽기.

        Format:
            A5 5A  |  30-bit length | 2-bit mode  |  1 byte data_type
            (SCAN은 length=5, mode=0x01(multiple), type=0x81)
        """
        data = self.ser.read(DESCRIPTOR_LEN)
        if len(data) != DESCRIPTOR_LEN:
            raise IOError(f"Descriptor 짧음: {len(data)} byte 받음 ({data.hex()})")
        if data[0] != SYNC_BYTE or data[1] != SYNC_BYTE2:
            raise IOError(f"Descriptor sync 불량: {data.hex()}")
        return data

    def _drain_buffer(self, idle_ms=150, max_wait_ms=1500):
        """
        boot text 등 잔여 데이터 완전 비움.
        idle_ms 동안 새 데이터가 없으면 종료.
        """
        deadline  = time.time() + max_wait_ms / 1000.0
        last_data = time.time()
        while time.time() < deadline:
            n = self.ser.in_waiting
            if n > 0:
                self.ser.read(n)
                last_data = time.time()
            elif (time.time() - last_data) * 1000 > idle_ms:
                return
            time.sleep(0.01)

    # ----------------------------------------------------------
    # 공개 명령
    # ----------------------------------------------------------
    def stop(self):
        """STOP: scanning 종료, idle 상태로. 응답 없음."""
        self._send_cmd(CMD_STOP)
        self._scanning = False
        time.sleep(STOP_DELAY_MS / 1000.0)
        self.ser.reset_input_buffer()

    def reset(self):
        """
        RESET: core 재부팅. 응답 없음.
        부팅 후 firmware banner 텍스트가 잠시 출력되므로 drain 까지 수행.
        """
        self._send_cmd(CMD_RESET)
        time.sleep(RESET_DELAY_MS / 1000.0)
        self._drain_buffer()

    def start_scan(self):
        """
        SCAN 시작.
        1) (이미 scanning이면) STOP
        2) 입력 buffer flush
        3) SCAN 명령 송신
        4) 7-byte descriptor 검증
        5) 모터 안정화 시간 대기
        """
        if self._scanning:
            self.stop()
        self.ser.reset_input_buffer()
        self._send_cmd(CMD_SCAN)
        self._read_descriptor()
        time.sleep(SCAN_WARMUP_MS / 1000.0)
        self._scanning = True

    # ----------------------------------------------------------
    # 데이터 iterator
    # ----------------------------------------------------------
    def iter_measurements(self):
        """
        5-byte measurement packet을 yield.

        Yields
        ------
        (quality, angle_deg, distance_mm, start_flag)
            quality    : 0~63
            angle_deg  : 0.0 ~ 359.99
            distance_mm: 0이면 invalid
            start_flag : 1이면 새 360° scan의 첫 번째 점
        """
        if not self._scanning:
            raise RuntimeError("start_scan() 호출 후에만 사용 가능")

        while True:
            data = self.ser.read(SCAN_PACKET_LEN)
            if len(data) != SCAN_PACKET_LEN:
                continue

            # --- 검증 (practice3 패턴) ---
            s_flag = data[0] & 0x01
            s_inv  = (data[0] >> 1) & 0x01
            if s_inv != (1 - s_flag):
                # S, S̄ 불일치 → 손상.
                # 1 byte 밀어서 재동기 (간단한 byte-level resync)
                self.ser.read(1)
                continue
            c_bit = data[1] & 0x01
            if c_bit != 1:
                continue

            # --- 파싱 ---
            quality = data[0] >> 2
            angle_q6 = (data[1] >> 1) | (data[2] << 7)
            angle    = angle_q6 / 64.0
            dist_q2  = data[3] | (data[4] << 8)
            distance = dist_q2 / 4.0   # mm

            yield (quality, angle, distance, s_flag)

    def iter_scans(self, min_quality=0, min_dist_mm=10.0):
        """
        한 바퀴(360°)씩 묶어서 yield.

        S=1 (start flag)이 검출되면 이전까지 누적된 scan을 반환.
        반환 형식: [(quality, angle_deg, distance_mm), ...]
        """
        buf = []
        for q, a, d, s in self.iter_measurements():
            if s == 1 and buf:
                yield buf
                buf = []
            if d >= min_dist_mm and q >= min_quality:
                buf.append((q, a, d))

    # ----------------------------------------------------------
    def close(self):
        try:
            if self._scanning:
                self.stop()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


# ============================================================
# 단독 실행: 3초간 raw 데이터 출력하는 테스트
# ============================================================
if __name__ == "__main__":
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    print(f"[TEST] port = {port}")

    lidar = RPLidarSerial(port)
    try:
        print("[TEST] RESET ...")
        lidar.reset()
        print("[TEST] SCAN  ...")
        lidar.start_scan()

        print("[TEST] 3초간 measurement 수신 (50개마다 출력)")
        t_end = time.time() + 3.0
        n = 0
        for q, a, d, s in lidar.iter_measurements():
            n += 1
            if n % 50 == 0:
                tag = "  ◀ START" if s == 1 else ""
                print(f"  #{n:5d}  θ={a:6.2f}°  d={d:7.1f}mm  Q={q:2d}{tag}")
            if time.time() > t_end:
                break
        print(f"[TEST] 총 {n}개 measurement")
    finally:
        lidar.close()
        print("[TEST] 종료")

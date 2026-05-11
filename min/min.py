import serial
import time
import math

# ==========================================
# 1. 통신 포트 설정
# ==========================================
arduino = serial.Serial('/dev/ttyS0', 115200, timeout=0.1)
lidar_ser = serial.Serial('/dev/ttyUSB0', 460800, timeout=1)

# ==========================================
# 2. 파라미터 및 가중치 설정 (핑퐁 모드)
# ==========================================
MAX_SPEED = 180
MIN_SPEED = 120
MAX_STEER = 120

PREDICT_TIME = 0.8

WEIGHT_CLEARANCE = 2.0   
WEIGHT_HEADING = 3.0     
WEIGHT_VELOCITY = 1.0    
WEIGHT_OPPOSITE = 2.5    # 청개구리(지그재그) 가중치

# ==========================================
# 3. DWA 핵심 알고리즘
# ==========================================
def calculate_dwa(scan_data, recent_turn_dir):
    obstacles = []
    
    for angle, distance in scan_data:
        if angle > 180: angle -= 360
        
        if 0 < distance < 1200 and -90 <= angle <= 90:
            rad = angle * (math.pi / 180.0)
            x = distance * math.cos(rad)
            y = distance * math.sin(rad)
            
            # 내 몸체 마스킹
            if abs(x) < 160 and abs(y) < 160:
                continue
                
            obstacles.append((x, y))

    v_cands = [120, 150, 180] 
    w_cands = [-80, -60, -40, -20, 0, 20, 40, 60, 80]

    best_v, best_w = 0, 0
    best_score = -999999

    for v in v_cands:
        for w in w_cands:
            pred_x, pred_y, pred_theta = 0.0, 0.0, 0.0
            min_dist = 1200
            collision = False
            
            sim_v = v * 2.5
            sim_w = w * 0.015
            dt = PREDICT_TIME / 8.0
            
            for _ in range(8):
                pred_x += sim_v * math.cos(pred_theta) * dt
                pred_y += sim_v * math.sin(pred_theta) * dt
                pred_theta += sim_w * dt
                
                for ox, oy in obstacles:
                    d = math.hypot(ox - pred_x, oy - pred_y)
                    if d < min_dist: 
                        min_dist = d
                    if d < 110: 
                        collision = True
                        break
                if collision: break

            if collision:
                score = -1000 + min_dist 
            else:
                clearance_score = min_dist / 1200.0
                heading_score = 1.0 - (abs(w) / 80.0) 
                velocity_score = v / MAX_SPEED
                
                opposite_score = 0.0
                # 방금 회전한 방향과 반대로 가려고 하면 보너스 점수
                if recent_turn_dir > 0 and w < 0:
                    opposite_score = abs(w) / 80.0
                elif recent_turn_dir < 0 and w > 0:
                    opposite_score = abs(w) / 80.0

                score = (WEIGHT_CLEARANCE * clearance_score) + \
                        (WEIGHT_HEADING * heading_score) + \
                        (WEIGHT_VELOCITY * velocity_score) + \
                        (WEIGHT_OPPOSITE * opposite_score)
                        
            if score > best_score:
                best_score = score
                best_v = v
                best_w = w

    return best_v, best_w

# ==========================================
# 4. 메인 루프 
# ==========================================
def main():
    print("[INFO] Lidar Initialization...")
    lidar_ser.write(bytes([0xA5, 0x40])) 
    time.sleep(1)
    lidar_ser.write(bytes([0xA5, 0x20])) 
    time.sleep(0.5)
    print("[INFO] Ping-Pong DWA Mode Started")

    scan_data = []
    recent_turn_dir = 1  

    try:
        while True:
            data = lidar_ser.read(5)
            if len(data) != 5: continue

            s_flag = data[0] & 0x01
            s_inv_flag = (data[0] & 0x02) >> 1
            if s_inv_flag != (1 - s_flag): continue
                
            check_bit = data[1] & 0x01
            if check_bit != 1: continue

            angle_q6 = ((data[1] >> 1) | (data[2] << 7))
            angle = angle_q6 / 64.0
            distance_q2 = (data[3] | (data[4] << 8))
            distance = distance_q2 / 4.0

            if s_flag == 1:
                if len(scan_data) > 50: 
                    v, w = calculate_dwa(scan_data, recent_turn_dir)
                    
                    if w > 20: recent_turn_dir = 1
                    elif w < -20: recent_turn_dir = -1
                    
                    command = f"{v},{w}\n"
                    arduino.write(command.encode('utf-8'))
                    print(f"-> Send: v={v}, w={w} (Points: {len(scan_data)})")
                    
                    time.sleep(0.05) 
                
                scan_data = []

            if distance > 0:
                scan_data.append((angle, distance))

    except KeyboardInterrupt:
        print("\n[INFO] Stop Command Received. Motor OFF.")
        arduino.write("0,0\n".encode('utf-8'))
        time.sleep(0.1)
        lidar_ser.write(bytes([0xA5, 0x25])) 
        arduino.close()
        lidar_ser.close()

if __name__ == '__main__':
    main()

def run_dwa(scan_points, curr_heading, current_v, current_w):
    global last_w_sign
    v_cands, w_cands = generate_vw_window(current_v, current_w)
    
    best_v, best_w = 0.0, 0.0
    max_score = -1.0
    
    for v in v_cands:
        for w in w_cands:
            clearance = check_collision_and_clearance(v, w, scan_points)
            if clearance <= 0: continue 
            
            # ★ 버그 수정: 현재 각도에 예측 회전량을 '더해야' 미래 각도가 됨
            pred_turn = math.degrees(w * 1.0)
            fut_heading = normalize_angle(curr_heading + pred_turn) 
            
            # 0도(정면)에 가까울수록 높은 점수
            score_heading = max(0.0, 1.0 - (abs(fut_heading) / 180.0))
            score_clearance = min(1.0, clearance / 1000.0)
            score_velocity = max(0.0, v / MAX_V)
            
            bias = BIAS_BONUS if (w * last_w_sign > 0) else 0.0
            total_score = (W_HEADING * score_heading) + (W_CLEARANCE * score_clearance) + (W_VELOCITY * score_velocity) + bias
                          
            if total_score > max_score:
                max_score = total_score
                best_v, best_w = v, w
                
    if best_w != 0: last_w_sign = 1.0 if best_w > 0 else -1.0
    return best_v, best_w

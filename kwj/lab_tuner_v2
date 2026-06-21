"""
LAB 색 검출 튜너 (다중 영역 추가/제거 지원)

조작법:
  r / y / b : 튜닝할 색상 전환 (RED / YELLOW / BLUE)
  = (또는 +) : [ADD 모드] 화면 클릭 시 해당 위치의 색상 영역 추가
  -         : [REMOVE 모드] 화면 클릭 시 가장 가까운 색상 영역 삭제
  m         : [MODIFY 모드] 화면 클릭 시 현재 선택된 영역 갱신
  [ / ]     : 선택된 영역(Active) 변경 (여러 개일 경우)
  p         : 지금까지 튜닝한 값을 콘솔에 출력 (복사/붙여넣기용)
  q / ESC   : 종료
"""

import cv2
import numpy as np

# ── 카메라 설정 (camera_tracker.py 와 완벽히 동일하게 일치) ────────────
CAMERA_INDEX = 0
FRAME_W      = 640
FRAME_H      = 480
FRAME_ROTATE = cv2.ROTATE_90_COUNTERCLOCKWISE

SAMPLE_WIN   = 9     # 클릭 시 샘플링할 윈도우 크기
MIN_AREA     = 500

WIN = 'LAB Tuner (Multi-Region)'

# 초기값 세팅
COLOR_PARAMS = {
    'RED':    [{'a': 180, 'b': 160, 'tol': 35}],
    'YELLOW': [{'a': 102, 'b': 160, 'tol': 29}],
    'BLUE':   [{'a': 111, 'b':  80, 'tol': 35}],
}
L_MIN = 30
COLORS = ['RED', 'YELLOW', 'BLUE']

_state = {
    'color': 'RED',
    'idx': 0,
    'mode': 'MODIFY', # 'ADD', 'REMOVE', 'MODIFY'
    'lab_img': None
}

# camera_tracker.py와 동일한 전처리 필터 적용
_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

def _to_lab(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = _clahe.apply(l)
    return cv2.merge([l, a, b])

def _nothing(_): pass

def _sync_trackbars_to_state():
    c = _state['color']
    idx = _state['idx']
    if len(COLOR_PARAMS[c]) > 0:
        p = COLOR_PARAMS[c][idx]
        cv2.setTrackbarPos('a_ref', WIN, p['a'])
        cv2.setTrackbarPos('b_ref', WIN, p['b'])
        cv2.setTrackbarPos('TOL',   WIN, p['tol'])
    cv2.setTrackbarPos('L_min', WIN, L_MIN)

def _on_mouse(event, x, y, flags, param):
    global L_MIN
    if event != cv2.EVENT_LBUTTONDOWN or _state['lab_img'] is None:
        return

    lab = _state['lab_img']
    h, w = lab.shape[:2]
    half = SAMPLE_WIN // 2
    x0, x1 = max(0, x - half), min(w, x + half + 1)
    y0, y1 = max(0, y - half), min(h, y + half + 1)
    patch = lab[y0:y1, x0:x1].reshape(-1, 3)
    
    a_val = int(np.median(patch[:, 1]))
    b_val = int(np.median(patch[:, 2]))
    c = _state['color']
    
    if _state['mode'] == 'ADD':
        current_tol = cv2.getTrackbarPos('TOL', WIN) if len(COLOR_PARAMS[c]) > 0 else 35
        COLOR_PARAMS[c].append({'a': a_val, 'b': b_val, 'tol': current_tol})
        _state['idx'] = len(COLOR_PARAMS[c]) - 1
        _sync_trackbars_to_state()
        print(f"[ADD] {c} 영역 추가 완료: a={a_val}, b={b_val}, tol={current_tol}")
        
    elif _state['mode'] == 'REMOVE':
        if len(COLOR_PARAMS[c]) > 0:
            best_i, min_d = -1, float('inf')
            for i, p in enumerate(COLOR_PARAMS[c]):
                d = (p['a'] - a_val)**2 + (p['b'] - b_val)**2
                if d < min_d:
                    min_d = d
                    best_i = i
            if best_i != -1:
                removed = COLOR_PARAMS[c].pop(best_i)
                print(f"[REMOVE] {c} 영역 제거됨: a={removed['a']}, b={removed['b']}")
                _state['idx'] = max(0, len(COLOR_PARAMS[c]) - 1)
                if len(COLOR_PARAMS[c]) > 0: _sync_trackbars_to_state()
        else:
            print(f"[REMOVE] 삭제할 {c} 영역이 없습니다.")
            
    elif _state['mode'] == 'MODIFY':
        if len(COLOR_PARAMS[c]) == 0:
            current_tol = cv2.getTrackbarPos('TOL', WIN)
            COLOR_PARAMS[c].append({'a': a_val, 'b': b_val, 'tol': current_tol})
            _state['idx'] = 0
        else:
            idx = _state['idx']
            COLOR_PARAMS[c][idx]['a'] = a_val
            COLOR_PARAMS[c][idx]['b'] = b_val
        _sync_trackbars_to_state()
        print(f"[MODIFY] {c} 현재 영역 갱신: a={a_val}, b={b_val}")

def _print_config():
    print("\n" + "=" * 60)
    print(" ▼▼ camera_tracker.py 에 붙여넣을 설정 코드 ▼▼\n")
    print("COLOR_PARAMS = {")
    for c in COLORS:
        print(f"    '{c}': [")
        for p in COLOR_PARAMS[c]:
            print(f"        {{'a': {p['a']}, 'b': {p['b']}, 'tol': {p['tol']}}},")
        print("    ],")
    print("}")
    print(f"L_MIN = {L_MIN}\n")
    
    print("COLOR_RANGES = {}")
    print("for color in ['RED', 'YELLOW', 'BLUE']:")
    print("    COLOR_RANGES[color] = []")
    print("    for p in COLOR_PARAMS[color]:")
    print("        lower = (L_MIN, max(0, p['a'] - p['tol']), max(0, p['b'] - p['tol']))")
    print("        upper = (255,   min(255, p['a'] + p['tol']), min(255, p['b'] + p['tol']))")
    print("        COLOR_RANGES[color].append((lower, upper))")
    print("\n" + "=" * 60 + "\n")

def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        return

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.createTrackbar('a_ref', WIN, 128, 255, _nothing)
    cv2.createTrackbar('b_ref', WIN, 128, 255, _nothing)
    cv2.createTrackbar('TOL',   WIN, 35,  100, _nothing)
    cv2.createTrackbar('L_min', WIN, 30,  255, _nothing)
    cv2.setMouseCallback(WIN, _on_mouse)

    _sync_trackbars_to_state()
    print("[TUNER] 실행 완료! (단축키: r,y,b / +, -, m / p)")

    eff_w, eff_h = (FRAME_H, FRAME_W) if FRAME_ROTATE in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE) else (FRAME_W, FRAME_H)

    while True:
        ret, frame = cap.read()
        if not ret: continue
        if FRAME_ROTATE is not None:
            frame = cv2.rotate(frame, FRAME_ROTATE)

        lab = _to_lab(frame)
        _state['lab_img'] = lab
        L, A, B = cv2.split(lab.astype(np.float32))

        # 트랙바 값을 현재 Active 영역에 반영
        global L_MIN
        L_MIN = cv2.getTrackbarPos('L_min', WIN)
        c = _state['color']
        idx = _state['idx']
        if len(COLOR_PARAMS[c]) > 0:
            COLOR_PARAMS[c][idx]['a'] = cv2.getTrackbarPos('a_ref', WIN)
            COLOR_PARAMS[c][idx]['b'] = cv2.getTrackbarPos('b_ref', WIN)
            COLOR_PARAMS[c][idx]['tol'] = cv2.getTrackbarPos('TOL', WIN)

        # 다중 영역 마스크 병합
        mask = np.zeros(lab.shape[:2], dtype=np.uint8)
        for p in COLOR_PARAMS[c]:
            dist = np.sqrt((A - p['a'])**2 + (B - p['b'])**2)
            mask |= ((dist < p['tol']) & (L > L_MIN)).astype(np.uint8) * 255

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 오버레이 표시
        overlay = frame.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area = 0.0
        centroid = None
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area >= MIN_AREA:
                M = cv2.moments(largest)
                if M['m00'] != 0:
                    centroid = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
                cv2.drawContours(overlay, [largest], -1, (0, 255, 0), 2)
                if centroid:
                    cv2.circle(overlay, centroid, 5, (0, 255, 0), -1)

        cv2.line(overlay, (eff_w // 2, 0), (eff_w // 2, eff_h), (180, 180, 180), 1)
        
        mode_str = "ADD (+)" if _state['mode'] == 'ADD' else "REMOVE (-)" if _state['mode'] == 'REMOVE' else "MODIFY (m)"
        cv2.putText(overlay, f"COLOR: {c}  |  MODE: {mode_str}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(overlay, f"Regions: {len(COLOR_PARAMS[c])}  |  Active: {idx+1}", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if len(COLOR_PARAMS[c]) > 0:
            p = COLOR_PARAMS[c][idx]
            cv2.putText(overlay, f"a={p['a']} b={p['b']} TOL={p['tol']} Lmin={L_MIN}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(overlay, "NO REGIONS (Click to Add)", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.putText(overlay, f"Area={area:.0f}", (10, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        combo = np.hstack([overlay, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        cv2.imshow(WIN, combo)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27): break
        elif key == ord('r'): _state['color'] = 'RED'; _state['idx'] = 0; _sync_trackbars_to_state()
        elif key == ord('y'): _state['color'] = 'YELLOW'; _state['idx'] = 0; _sync_trackbars_to_state()
        elif key == ord('b'): _state['color'] = 'BLUE'; _state['idx'] = 0; _sync_trackbars_to_state()
        elif key in (ord('='), ord('+')): _state['mode'] = 'ADD'; print("[MODE] ADD 모드")
        elif key == ord('-'): _state['mode'] = 'REMOVE'; print("[MODE] REMOVE 모드")
        elif key == ord('m'): _state['mode'] = 'MODIFY'; print("[MODE] MODIFY 모드")
        elif key == ord('['): 
            if len(COLOR_PARAMS[c])>0: _state['idx'] = max(0, _state['idx']-1); _sync_trackbars_to_state()
        elif key == ord(']'): 
            if len(COLOR_PARAMS[c])>0: _state['idx'] = min(len(COLOR_PARAMS[c])-1, _state['idx']+1); _sync_trackbars_to_state()
        elif key == ord('p'): _print_config()

    cap.release()
    cv2.destroyAllWindows()
    _print_config()

if __name__ == '__main__':
    main()

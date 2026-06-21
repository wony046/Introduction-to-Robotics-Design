"""
LAB 색 검출 튜너 — camera_tracker.py 의 detect_color_lab() 파라미터 맞춤 도구

검출 모델 (detect_color_lab 와 동일):
    dist = sqrt((a - a_ref)^2 + (b - b_ref)^2)
    mask = (dist < TOL) AND (L > L_MIN)

화면 구성 : [ 원본+검출표시 | 마스크 ] 가로로 나란히
실행      : python lab_tuner.py

────────────────────────────────────────────────────────────────────
조작법
  마우스 클릭 : 그 지점 주변(중앙값)의 a·b 를 트랙바에 자동 입력
  트랙바      : a_ref / b_ref / TOL / L_min / WB(0=off,1=on)
  r / y / b   : 튜닝 대상 색 전환 (RED / YELLOW / BLUE) — 저장값 자동 로드
  s           : 현재 트랙바 값을 그 색 슬롯에 저장
  p           : 지금까지 저장된 전체 설정을 콘솔에 출력 (붙여넣기용)
  q / ESC     : 종료 (종료 시 자동으로 전체 설정 출력)
────────────────────────────────────────────────────────────────────
"""

import cv2
import numpy as np

# ── 카메라 설정 (camera_tracker.py 와 동일하게 맞출 것) ────────────────
CAMERA_INDEX = 0
FRAME_W      = 848   # 16:9 (848×480) 로 통일 (기존 1280×720 — 타 파일과 불일치였음)
FRAME_H      = 480
FRAME_ROTATE = cv2.ROTATE_90_COUNTERCLOCKWISE   # None 이면 회전 없음

SAMPLE_WIN   = 9     # 클릭 시 샘플링할 정사각 윈도우 한 변(px)
MIN_AREA     = 500   # 표시용 최소 blob 면적

WIN = 'LAB Tuner'

# 색마다 저장 슬롯. 초기값은 추정치 — 튜닝으로 덮어쓰면 됨.
PARAMS = {
    'RED':    {'a': 180, 'b': 160, 'tol': 35, 'lmin': 30},
    'YELLOW': {'a': 120, 'b': 200, 'tol': 35, 'lmin': 30},
    'BLUE':   {'a': 120, 'b':  80, 'tol': 35, 'lmin': 30},
}
COLORS = ['RED', 'YELLOW', 'BLUE']
_cur = {'name': 'RED'}          # 현재 튜닝 대상
_last_lab = {'img': None}       # 클릭 샘플링용 최신 LAB 프레임


# ── 전처리 (camera_tracker 의 gray_world_wb 와 동일) ──────────────────
def gray_world_wb(bgr):
    b, g, r = cv2.split(bgr.astype(np.float32))
    mb, mg, mr = b.mean() + 1e-6, g.mean() + 1e-6, r.mean() + 1e-6
    mgray = (mb + mg + mr) / 3.0
    b *= mgray / mb
    g *= mgray / mg
    r *= mgray / mr
    return cv2.merge([b, g, r]).clip(0, 255).astype(np.uint8)


def _nothing(_):
    pass


def _load_to_trackbars(name):
    p = PARAMS[name]
    cv2.setTrackbarPos('a_ref',  WIN, p['a'])
    cv2.setTrackbarPos('b_ref',  WIN, p['b'])
    cv2.setTrackbarPos('TOL',    WIN, p['tol'])
    cv2.setTrackbarPos('L_min',  WIN, p['lmin'])


def _save_from_trackbars(name):
    PARAMS[name] = {
        'a':    cv2.getTrackbarPos('a_ref', WIN),
        'b':    cv2.getTrackbarPos('b_ref', WIN),
        'tol':  cv2.getTrackbarPos('TOL',   WIN),
        'lmin': cv2.getTrackbarPos('L_min', WIN),
    }
    print(f"[SAVE] {name} ← {PARAMS[name]}")


def _print_config():
    print("\n" + "=" * 56)
    print("# camera_tracker.py 에 붙여넣기")
    print("REF_AB = {")
    for c in COLORS:
        print(f"    '{c}': ({PARAMS[c]['a']}, {PARAMS[c]['b']}),")
    print("}")
    print("TOL   = {" + ", ".join(f"'{c}': {PARAMS[c]['tol']}" for c in COLORS) + "}")
    lmins = [PARAMS[c]['lmin'] for c in COLORS]
    if len(set(lmins)) == 1:
        print(f"L_MIN = {lmins[0]}")
    else:
        print("# L_min 이 색마다 다름 → detect_color_lab 를 색별 L_min 사용하도록 수정 필요:")
        print("L_MIN = {" + ", ".join(f"'{c}': {PARAMS[c]['lmin']}" for c in COLORS) + "}")
    print("=" * 56 + "\n")


def _on_mouse(event, x, y, flags, param):
    """클릭 지점 주변의 a·b 중앙값을 트랙바에 입력."""
    if event != cv2.EVENT_LBUTTONDOWN or _last_lab['img'] is None:
        return
    lab = _last_lab['img']
    h, w = lab.shape[:2]
    half = SAMPLE_WIN // 2
    x0, x1 = max(0, x - half), min(w, x + half + 1)
    y0, y1 = max(0, y - half), min(h, y + half + 1)
    patch = lab[y0:y1, x0:x1].reshape(-1, 3)
    L = int(np.median(patch[:, 0]))
    a = int(np.median(patch[:, 1]))
    b = int(np.median(patch[:, 2]))
    cv2.setTrackbarPos('a_ref', WIN, a)
    cv2.setTrackbarPos('b_ref', WIN, b)
    print(f"[CLICK] {_cur['name']}  L={L} a={a} b={b}  → a_ref,b_ref 설정")


def main():
    eff_w, eff_h = (FRAME_H, FRAME_W) if FRAME_ROTATE in (
        cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE) else (FRAME_W, FRAME_H)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다. CAMERA_INDEX 확인.")
        return

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.createTrackbar('a_ref', WIN, PARAMS['RED']['a'],    255, _nothing)
    cv2.createTrackbar('b_ref', WIN, PARAMS['RED']['b'],    255, _nothing)
    cv2.createTrackbar('TOL',   WIN, PARAMS['RED']['tol'],  100, _nothing)
    cv2.createTrackbar('L_min', WIN, PARAMS['RED']['lmin'], 255, _nothing)
    cv2.createTrackbar('WB',    WIN, 1,                       1, _nothing)
    cv2.setMouseCallback(WIN, _on_mouse)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    print("[TUNER] 시작. r/y/b 색 전환, 클릭=샘플, s=저장, p=출력, q=종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        if FRAME_ROTATE is not None:
            frame = cv2.rotate(frame, FRAME_ROTATE)

        use_wb = cv2.getTrackbarPos('WB', WIN) == 1
        proc = gray_world_wb(frame) if use_wb else frame

        lab = cv2.cvtColor(proc, cv2.COLOR_BGR2LAB)
        _last_lab['img'] = lab
        L, A, B = cv2.split(lab.astype(np.float32))

        a_ref = cv2.getTrackbarPos('a_ref', WIN)
        b_ref = cv2.getTrackbarPos('b_ref', WIN)
        tol   = cv2.getTrackbarPos('TOL',   WIN)
        lmin  = cv2.getTrackbarPos('L_min', WIN)

        dist = np.sqrt((A - a_ref) ** 2 + (B - b_ref) ** 2)
        mask = ((dist < tol) & (L > lmin)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 최대 blob 표시
        overlay = proc.copy()
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
        cv2.putText(overlay, f"TUNING: {_cur['name']}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        cv2.putText(overlay, f"a={a_ref} b={b_ref} TOL={tol} Lmin={lmin}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(overlay, f"area={area:.0f}", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combo = np.hstack([overlay, mask_bgr])
        cv2.imshow(WIN, combo)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('r'):
            _cur['name'] = 'RED';    _load_to_trackbars('RED')
        elif key == ord('y'):
            _cur['name'] = 'YELLOW'; _load_to_trackbars('YELLOW')
        elif key == ord('b'):
            _cur['name'] = 'BLUE';   _load_to_trackbars('BLUE')
        elif key == ord('s'):
            _save_from_trackbars(_cur['name'])
        elif key == ord('p'):
            _print_config()

    cap.release()
    cv2.destroyAllWindows()
    _print_config()


if __name__ == '__main__':
    main()

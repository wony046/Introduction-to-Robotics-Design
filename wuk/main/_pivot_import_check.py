import sys, types
# 하드웨어 의존 모듈 목 (import 만 통과시키면 됨 — 모듈 로드는 main() 호출 안 함)
m_serial = types.ModuleType('serial'); m_serial.Serial = lambda *a, **k: None
sys.modules['serial'] = m_serial
sys.modules['camera_tracker'] = types.ModuleType('camera_tracker')

import importlib
jw = importlib.import_module('jw_won')

assert hasattr(jw, 'PIVOT_CONFIRM_SEC'), 'PIVOT_CONFIRM_SEC 누락'
assert abs(jw.PIVOT_CONFIRM_SEC - 0.15) < 1e-9
assert hasattr(jw, '_pivot_confirm_start') and jw._pivot_confirm_start is None
assert hasattr(jw, 'ARRIVE_PIVOT_W')
assert hasattr(jw, '_seek_last_global_bearing')
print(f"OK: jw_won 모듈 로드 정상  PIVOT_CONFIRM_SEC={jw.PIVOT_CONFIRM_SEC} "
      f"_pivot_confirm_start={jw._pivot_confirm_start!r}")

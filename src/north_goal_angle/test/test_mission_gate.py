#!/usr/bin/env python3
"""자율 시작 게이트 순수 로직 테스트. python3 src/north_goal_angle/test/test_mission_gate.py

north_goal_angle.py 는 rclpy 를 import 하므로 그대로 못 읽는다(Mac). 순수 함수만 스텁으로 뽑는다."""

import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# north_goal_angle.py 통째 import 하면 rclpy 가 필요하다 → mission_should_run 만 추출.
_src = os.path.join(os.path.dirname(__file__), '..', 'north_goal_angle', 'north_goal_angle.py')
_ns = {}
_code = open(_src, encoding='utf-8').read()
# MODE_AUTO 와 mission_should_run 정의부만 실행 (import 없이 되는 순수 조각)
import re
m = re.search(r"^MODE_AUTO = 2\n\n\ndef mission_should_run.*?return boat_mode == mode_auto",
              _code, re.S | re.M)
exec(m.group(0), _ns)
mission_should_run = _ns['mission_should_run']
MODE_AUTO = _ns['MODE_AUTO']

_p = _t = 0


def check(name, fn):
    global _p, _t
    _t += 1
    try:
        fn(); _p += 1; print(f"  ✅ {name}")
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")


# ---------------- require_auto=True (기본, 안전)
def test_auto_runs():
    assert mission_should_run(MODE_AUTO, require_auto=True) is True


def test_manual_holds():
    assert mission_should_run(1, require_auto=True) is False   # MANUAL


def test_wait_holds():
    assert mission_should_run(0, require_auto=True) is False   # WAIT


def test_none_holds():
    """모드 아직 모름(/boat_mode 미수신) → 안전하게 대기."""
    assert mission_should_run(None, require_auto=True) is False


# ---------------- require_auto=False (벤치)
def test_bench_always_runs():
    for m in (None, 0, 1, 2):
        assert mission_should_run(m, require_auto=False) is True, m


def main():
    print("=== mission_gate 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())

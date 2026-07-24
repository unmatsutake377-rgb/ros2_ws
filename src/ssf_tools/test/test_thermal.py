#!/usr/bin/env python3
"""thermal 순수 로직 테스트. python3 src/ssf_tools/test/test_thermal.py"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ssf_tools.thermal import (  # noqa: E402
    LVL_OK, LVL_WARN, LVL_HOT,
    temp_state, throttle_detected, power_state, summarize,
)

_p = _t = 0


def check(name, fn):
    global _p, _t
    _t += 1
    try:
        fn(); _p += 1; print(f"  ✅ {name}")
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")


def test_temp_bands():
    assert temp_state(50.0)[0] == LVL_OK
    assert temp_state(80.0)[0] == LVL_WARN, "임계는 이상(>=)"
    assert temp_state(94.9)[0] == LVL_WARN
    assert temp_state(95.0)[0] == LVL_HOT
    assert temp_state(120.0)[0] == LVL_HOT


def test_temp_unknown():
    assert temp_state(None) == ("UNKNOWN", None)
    assert temp_state(float('nan')) == ("UNKNOWN", None)
    assert temp_state(float('inf')) == ("UNKNOWN", None)


def test_temp_custom_thresholds():
    assert temp_state(70.0, warn_c=65.0, hot_c=85.0)[0] == LVL_WARN


def test_throttle():
    thr, frac = throttle_detected(1000, 4000, ratio=0.6)
    assert thr and abs(frac - 0.25) < 1e-9, "25% < 60% = 스로틀 의심"
    thr, frac = throttle_detected(3000, 4000, ratio=0.6)
    assert not thr and abs(frac - 0.75) < 1e-9


def test_throttle_bad_input():
    assert throttle_detected(None, 4000)[0] is False
    assert throttle_detected(1000, 0)[0] is False, "0 나눗셈 방지"
    assert throttle_detected(1000, None)[0] is False


def test_power():
    assert power_state(True) == "AC"
    assert power_state(False) == "BATTERY"
    assert power_state(None) == "UNKNOWN"


def test_summarize_all_ok():
    s = summarize(temp_c=55.0, cur_khz=3800, max_khz=4000, ac_online=True)
    assert not s["alert"]
    assert s["temp_lvl"] == LVL_OK and s["power"] == "AC"


def test_summarize_hot_alerts():
    s = summarize(temp_c=97.0, cur_khz=3800, max_khz=4000, ac_online=True)
    assert s["alert"] and s["temp_lvl"] == LVL_HOT


def test_summarize_battery_alerts():
    s = summarize(temp_c=55.0, cur_khz=3800, max_khz=4000, ac_online=False)
    assert s["alert"] and s["power"] == "BATTERY"


def test_summarize_throttle_alerts():
    s = summarize(temp_c=55.0, cur_khz=1000, max_khz=4000, ac_online=True)
    assert s["alert"] and s["throttle"]


def test_summarize_missing_data_no_false_alarm():
    """값을 못 읽으면(전부 None) 경고하지 않는다 — UNKNOWN 은 alert 아님."""
    s = summarize()
    assert not s["alert"], "데이터 없음을 경고로 만들면 늑대소년이 된다"
    assert s["temp_lvl"] == "UNKNOWN" and s["power"] == "UNKNOWN"


def main():
    print("=== thermal 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())

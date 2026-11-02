"""hold_logic 단위 테스트 — ROS 없이 돈다.

    python3 src/ship_back/test/test_hold_logic.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ship_back.hold_logic import station_keep_action, HoldTimer, should_reset_on_mode  # noqa: E402

fails = 0


def check(name, cond, extra=""):
    global fails
    print(f"  {'✅' if cond else '❌'} {name} {extra}")
    if not cond:
        fails += 1
    assert cond, name


# ─────────────── station_keep_action ───────────────
def test_too_far_forward():
    check("멀다(4.0>2.5+0.3) → 전진", station_keep_action(4.0, 2.5, 0.3) == 'forward')


def test_too_close_reverse():
    check("가깝다(1.0<2.5-0.3) → 후진", station_keep_action(1.0, 2.5, 0.3) == 'reverse')


def test_in_band_hold():
    check("데드밴드 안 → 유지", station_keep_action(2.5, 2.5, 0.3) == 'hold')
    check("데드밴드 경계 안 → 유지", station_keep_action(2.7, 2.5, 0.3) == 'hold')


def test_unknown_dist():
    check("거리 미상 → unknown", station_keep_action(None, 2.5, 0.3) == 'unknown')


def test_band_edges():
    # 2.5±0.3 = [2.2, 2.8]
    check("2.81 → 전진", station_keep_action(2.81, 2.5, 0.3) == 'forward')
    check("2.19 → 후진", station_keep_action(2.19, 2.5, 0.3) == 'reverse')


# ─────────────── HoldTimer ───────────────
def test_hold_accumulates():
    t = HoldTimer(keep_radius=5.0, hold_time=5.0)
    check("0s 진입", abs(t.update(3.0, 100.0)) < 1e-9)
    check("3s 후 3초 유지", abs(t.update(3.0, 103.0) - 3.0) < 1e-9)
    check("5s 후 만족", t.satisfied(3.0, 105.0))


def test_hold_resets_when_leaving():
    t = HoldTimer(5.0, 5.0)
    t.update(3.0, 100.0)
    t.update(3.0, 104.0)              # 4초 유지
    check("반경 벗어나면 리셋", abs(t.update(6.0, 104.5)) < 1e-9)   # 6m > 5m
    # 다시 들어오면 처음부터
    check("재진입은 0부터", abs(t.update(3.0, 105.0)) < 1e-9)
    check("아직 만족 안 함", not t.satisfied(3.0, 106.0))


def test_hold_unknown_resets():
    t = HoldTimer(5.0, 5.0)
    t.update(3.0, 100.0)
    check("거리 미상 → 리셋", abs(t.update(None, 102.0)) < 1e-9)


def test_hold_boundary_radius():
    t = HoldTimer(5.0, 5.0)
    check("정확히 5.0m 는 반경 안(포함)", t.update(5.0, 100.0) == 0.0 and t._enter is not None)


# ─────────────── should_reset_on_mode (/wp_mode 전이 리셋 판정) ───────────────
def test_mode_reset_hold_2_to_3():
    # hold([2,3]): 2→3 은 반드시 리셋 — `prev not in modes` 로 쓰면 여기서 깨진다(계약서 위험 2)
    check("[2,3] 2→3 리셋", should_reset_on_mode(2, 3, [2, 3]))


def test_mode_reset_same_mode_no_reset():
    check("[2,3] 3→3 비리셋(래치 유지)", not should_reset_on_mode(3, 3, [2, 3]))
    check("[2,3] 2→2 비리셋", not should_reset_on_mode(2, 2, [2, 3]))
    check("[2] 2→2 비리셋", not should_reset_on_mode(2, 2, [2]))


def test_mode_reset_enter_from_other():
    check("[2,3] 5→2 리셋", should_reset_on_mode(5, 2, [2, 3]))
    check("[2] 5→2 리셋", should_reset_on_mode(5, 2, [2]))
    check("[2] -1(부팅)→2 리셋", should_reset_on_mode(-1, 2, [2]))


def test_mode_reset_leaving_inactive():
    check("[2,3] 3→5 비활성(리셋 없음)", not should_reset_on_mode(3, 5, [2, 3]))
    check("[2] 2→3 비활성(orbit 기본: 3 은 ship_turn)", not should_reset_on_mode(2, 3, [2]))


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in tests:
        try:
            fn()
        except AssertionError as e:
            fails += 1
            print(f"  ❌ {fn.__name__}: {e}")
    print(f"\n{'PASS' if fails == 0 else str(fails) + ' FAIL'}")
    sys.exit(1 if fails else 0)

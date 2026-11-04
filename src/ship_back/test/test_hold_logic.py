"""hold_logic 단위 테스트 — ROS 없이 돈다.

    python3 src/ship_back/test/test_hold_logic.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ship_back.hold_logic import station_keep_action, HoldTimer, should_reset_on_mode  # noqa: E402
from ship_back.hold_logic import target_source, counts_as_seen, search_timed_out  # noqa: E402
from ship_back.hold_logic import unknown_giveup, search_timeout_warn  # noqa: E402
from ship_back.hold_logic import hold_total_exceeded  # noqa: E402

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
    check("[2] 2→3 비활성(orbit 옵션: 3 은 ship_turn)", not should_reset_on_mode(2, 3, [2]))


# ─────────────── target_source / counts_as_seen (계약 전체코스 §2-5) ───────────────
def test_source_vision_fresh_wins():
    for st in ('SEARCH', 'HOLD'):
        for la in (False, True):
            check(f"{st} lidar_acquire={la}: 비전 신선 → vision",
                  target_source(st, True, None, 2.0, la) == 'vision')


def test_source_search_no_vision_no_lidar():
    # 수정 전: SEARCH 에서 비전이 없으면 LiDAR 전방 최근접을 부표로 보고 HOLD 했다(색 무관)
    check("SEARCH 비전 없음 → None(획득 안 함)", target_source('SEARCH', False, None, 2.0, False) is None)
    check("SEARCH 비전 없음, 방금 봤어도 → None", target_source('SEARCH', False, 0.0, 2.0, False) is None)


def test_source_hold_lidar_aid_window():
    check("HOLD 직전 비전 1.5 s 전 → lidar 보조",
          target_source('HOLD', False, 1.5, 2.0, False) == 'lidar')
    check("HOLD 경계 2.0 s(포함) → lidar", target_source('HOLD', False, 2.0, 2.0, False) == 'lidar')
    check("HOLD 2.01 s → None(lost 와 맞물림)", target_source('HOLD', False, 2.01, 2.0, False) is None)
    check("HOLD 본 적 없음 → None", target_source('HOLD', False, None, 2.0, False) is None)


def test_source_other_states_none():
    for st in ('INIT', 'DONE'):
        check(f"{st} 비전 없음 → None", target_source(st, False, 0.5, 2.0, False) is None)


def test_source_lidar_acquire_old_behavior():
    check("lidar_acquire=true SEARCH 비전 없음 → lidar(옛 동작)",
          target_source('SEARCH', False, None, 2.0, True) == 'lidar')
    check("lidar_acquire=true HOLD lost 뒤에도 → lidar",
          target_source('HOLD', False, 9.0, 2.0, True) == 'lidar')


def test_counts_as_seen():
    check("vision 은 봤다", counts_as_seen('vision', False) and counts_as_seen('vision', True))
    check("lidar 보조는 안 셈(기본)", not counts_as_seen('lidar', False))
    check("lidar_acquire=true 면 lidar 도 셈", counts_as_seen('lidar', True))
    check("None 은 안 셈", not counts_as_seen(None, False) and not counts_as_seen(None, True))


# ─────────────── search_timed_out (계약 전체코스 §2-4) ───────────────
def test_search_timeout_default_never_latches():
    check("기본(false) 25 s > 20 s → 래치 안 함", not search_timed_out(25.0, 20.0, False))
    check("기본(false) 1e6 s → 래치 안 함", not search_timed_out(1e6, 20.0, False))


def test_search_timeout_warn_once():
    check("false 20.1 s 미발행 → WARN", search_timeout_warn(20.1, 20.0, False, False))
    check("false 20.0 s → 아직(초과만)", not search_timeout_warn(20.0, 20.0, False, False))
    check("이미 냄 → 안 냄", not search_timeout_warn(30.0, 20.0, False, True))
    check("true(옛 동작) → 이 WARN 안 냄", not search_timeout_warn(30.0, 20.0, True, False))


def test_search_timeout_switch_old_behavior():
    check("true 25 s > 20 s → DONE", search_timed_out(25.0, 20.0, True))
    check("true 정확히 20 s → 아직(초과만)", not search_timed_out(20.0, 20.0, True))


# ─────────────── unknown_giveup (계약 전체코스 §10-추가 결정 4) ───────────────
def test_unknown_giveup_boundary():
    check("unknown 9.9 s → 아직", not unknown_giveup(100.0, 109.9, 10.0))
    check("unknown 10.0 s(포함) → 포기", unknown_giveup(100.0, 110.0, 10.0))
    check("unknown 30 s → 포기", unknown_giveup(100.0, 130.0, 10.0))


def test_unknown_giveup_not_unknown():
    check("unknown 아님(t0 None) → 포기 안 함", not unknown_giveup(None, 1e6, 10.0))


# ─────────────── hold_total_exceeded (계약 전체코스 §10-추가 결정 5) ───────────────
def test_hold_total_boundary():
    check("HOLD 59.9 s → 아직", not hold_total_exceeded(0.0, 100.0, 159.9, 60.0))
    check("HOLD 60.0 s(포함) → 포기", hold_total_exceeded(0.0, 100.0, 160.0, 60.0))


def test_hold_total_sums_segments():
    check("끝난 30 s + 지금 29.9 s → 아직", not hold_total_exceeded(30.0, 100.0, 129.9, 60.0))
    check("끝난 30 s + 지금 30 s → 포기", hold_total_exceeded(30.0, 100.0, 130.0, 60.0))
    check("HOLD 아님, 합 59.9 → 아직", not hold_total_exceeded(59.9, None, 1e6, 60.0))
    check("HOLD 아님, 합 60.0 → 포기", hold_total_exceeded(60.0, None, 1e6, 60.0))


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

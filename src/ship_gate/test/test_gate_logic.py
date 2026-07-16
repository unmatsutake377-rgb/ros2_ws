"""GateCounter / gate_candidate 단위 테스트 — ROS 없이 돈다.

    python3 src/ship_gate/test/test_gate_logic.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ship_gate.gate_logic import (  # noqa: E402
    GateCounter, gate_candidate, circular_mid, clamp_pm180)


fails = 0


def check(name, cond, extra=""):
    global fails
    print(f"  {'✅' if cond else '❌'} {name} {extra}")
    if not cond:
        fails += 1


# ─────────────── gate_candidate: 쌍/단일/제약 ───────────────
def test_pair_midpoint():
    """빨강 -20°, 초록 +20° → 중점 0° (정면), 유효 쌍."""
    steer, valid = gate_candidate(-20.0, 20.0, True, 5.0, 80.0, 40.0)
    check("정상 쌍 → 중점 정면", valid and abs(steer) < 1e-6, f"steer={steer}")


def test_pair_too_wide_rejected():
    """빨강 -60°, 초록 +60° → 분리 120° > max 80° → 쌍 아님, 가까운 쪽만."""
    steer, valid = gate_candidate(-60.0, 60.0, True, 5.0, 80.0, 40.0)
    check("너무 벌어진 쌍 → 쌍 거부", not valid, f"steer={steer}")


def test_pair_too_narrow_rejected():
    """분리 3° < min 5° → 같은 게이트 아님(중복 검출) → 쌍 거부."""
    _, valid = gate_candidate(10.0, 13.0, True, 5.0, 80.0, 40.0)
    check("너무 붙은 쌍 → 쌍 거부", not valid)


def test_single_red_on_port():
    """빨강만 -30° 보임, red_on_port → 중심은 빨강 우측 → -30+40 = +10°."""
    steer, valid = gate_candidate(-30.0, None, True, 5.0, 80.0, 40.0)
    check("빨강만(좌현) → 우측으로 오프셋", (not valid) and abs(steer - 10.0) < 1e-6, f"steer={steer}")


def test_single_green_on_port():
    """초록만 +30° 보임, red_on_port(초록=우현) → 중심은 초록 좌측 → 30-40 = -10°."""
    steer, valid = gate_candidate(None, 30.0, True, 5.0, 80.0, 40.0)
    check("초록만(우현) → 좌측으로 오프셋", (not valid) and abs(steer + 10.0) < 1e-6, f"steer={steer}")


def test_red_on_port_flips():
    """red_on_port=False 면 오프셋 방향이 뒤집힌다."""
    s_true, _ = gate_candidate(-30.0, None, True, 5.0, 80.0, 40.0)
    s_false, _ = gate_candidate(-30.0, None, False, 5.0, 80.0, 40.0)
    check("red_on_port 뒤집으면 방향 반대", (s_true > 0) and (s_false < 0), f"{s_true} vs {s_false}")


def test_none_visible():
    """둘 다 안 보임 → 게이트 없음."""
    steer, valid = gate_candidate(None, None, True, 5.0, 80.0, 40.0)
    check("둘 다 안 보임 → None", steer is None and not valid)


def test_wide_pair_uses_nearer():
    """벌어진 쌍(빨강 -75°, 초록 +25°, 분리 100°>80) → 쌍 거부, 가까운 초록(+25)만."""
    steer, valid = gate_candidate(-75.0, 25.0, True, 5.0, 80.0, 40.0)
    # 초록(우현) → 좌측 오프셋 → 25-40 = -15
    check("벌어진 쌍 → 가까운 부표만", (not valid) and abs(steer + 15.0) < 1e-6, f"steer={steer}")


def test_pair_boundary_inclusive():
    """분리각이 정확히 max_sep(80°)면 유효 쌍(경계 포함)."""
    _, valid = gate_candidate(-70.0, 10.0, True, 5.0, 80.0, 40.0)
    check("분리 80°(경계) → 유효 쌍", valid)


# ─────────────── GateCounter: 통과 계수 ───────────────
def test_count_one_gate():
    """게이트를 전방에서 보다가 부표가 뒤로(±75° 초과) 가면 1 카운트."""
    c = GateCounter(75.0)
    for _ in range(5):
        c.update(gate_ahead=True, buoy_abs_angles=[10.0, 12.0])   # 전방 접근
    check("접근 중엔 카운트 0", c.count == 0)
    c.update(gate_ahead=False, buoy_abs_angles=[80.0])            # 한쪽이 뒤로
    check("부표가 뒤로 → 1 카운트", c.count == 1)


def test_no_double_count():
    """같은 게이트로 여러 틱 뒤에 있어도 두 번 세지 않는다."""
    c = GateCounter(75.0)
    c.update(True, [10.0, 12.0])
    c.update(False, [82.0])       # 통과 → 1
    for _ in range(10):
        c.update(False, [88.0])   # 계속 뒤에 있어도
    check("이중 카운트 없음", c.count == 1, f"count={c.count}")


def test_two_gates_sequential():
    """두 게이트를 순차 통과 → 2. (사이에 재-arming)"""
    c = GateCounter(75.0)
    c.update(True, [8.0, 9.0]); c.update(False, [80.0])          # 1
    c.update(True, [7.0, 6.0]); c.update(False, [79.0])          # 2 (새 게이트 다시 전방에서 봄)
    check("두 게이트 순차 → 2", c.count == 2, f"count={c.count}")


def test_no_count_without_arming():
    """전방에서 유효 게이트를 못 봤으면(arming 없음) 부표가 뒤에 있어도 안 센다.
    (먼 부표가 잠깐 크게 잡힌 오탐 등)"""
    c = GateCounter(75.0)
    for _ in range(5):
        c.update(gate_ahead=False, buoy_abs_angles=[85.0])
    check("arming 없이는 카운트 0", c.count == 0)


# ─────────────── helpers ───────────────
def test_circular_mid_wrap():
    """±180 경계를 넘는 중점."""
    check("circular_mid(170,-170)=180 근처",
          abs(abs(circular_mid(170.0, -170.0)) - 180.0) < 1e-6)


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in tests:
        try:
            fn()
        except AssertionError as e:
            fails += 1
            print(f"  ❌ {fn.__name__}: {e}")
    print(f"\n{len(tests) - fails}/{len(tests)} 통과")
    sys.exit(1 if fails else 0)

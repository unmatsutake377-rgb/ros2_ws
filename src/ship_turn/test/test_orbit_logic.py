"""orbit_logic 단위 테스트 — ROS 없이 돈다.

    python3 src/ship_turn/test/test_orbit_logic.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ship_turn.orbit_logic import (  # noqa: E402
    orbit_steer, OrbitProgress, orbit_direction_cw, clamp_pm180)

fails = 0


def check(name, cond, extra=""):
    global fails
    print(f"  {'✅' if cond else '❌'} {name} {extra}")
    if not cond:
        fails += 1


# ─────────────── 방향 매핑 ───────────────
def test_color_direction():
    check("red → 시계(CW)", orbit_direction_cw("red") is True)
    check("green → 시계(CW)", orbit_direction_cw("GREEN") is True)
    check("white → 반시계(CCW)", orbit_direction_cw("white") is False)
    check("모르는 색 → None", orbit_direction_cw("blue") is None)
    check("빈 문자열 → None", orbit_direction_cw("") is None)


# ─────────────── orbit_steer: 반경 유지 시 접선 ───────────────
def test_steer_on_circle_cw():
    """CW, 부표 우현(+90°), 반경 정확 → 접선(정면 0°)."""
    s = orbit_steer(90.0, 3.0, 3.0, True, 30.0, 45.0)
    check("CW 반경 정확 → 정면(접선)", abs(s) < 1e-6, f"steer={s}")


def test_steer_on_circle_ccw():
    """CCW, 부표 좌현(-90°), 반경 정확 → 접선(정면 0°)."""
    s = orbit_steer(-90.0, 3.0, 3.0, False, 30.0, 45.0)
    check("CCW 반경 정확 → 정면(접선)", abs(s) < 1e-6, f"steer={s}")


def test_steer_too_far_turns_inward():
    """CW, 너무 멀다(dist>radius) → 부표(우현) 쪽으로(+) 튼다."""
    s = orbit_steer(90.0, 5.0, 3.0, True, 30.0, 45.0)
    check("CW 너무 멀다 → 안쪽(우)로", s > 0, f"steer={s}")


def test_steer_too_close_turns_outward():
    """CW, 너무 가깝다(dist<radius) → 바깥(좌)으로 튼다."""
    s = orbit_steer(90.0, 1.0, 3.0, True, 30.0, 45.0)
    check("CW 너무 가깝다 → 바깥(좌)으로", s < 0, f"steer={s}")


def test_steer_radial_clamped():
    """반경 오차가 커도 gain 은 radial_limit 로 제한된다."""
    s = orbit_steer(90.0, 100.0, 3.0, True, 30.0, 45.0)   # err=97, k*err 큼
    # 접선 0 + gain(<=45) → 45 이하
    check("반경 보정 clamp", abs(s) <= 45.0 + 1e-6, f"steer={s}")


def test_steer_ccw_far_turns_inward_left():
    """CCW, 부표 좌현(-90°), 너무 멀다 → 부표(좌) 쪽으로(-) 튼다."""
    s = orbit_steer(-90.0, 5.0, 3.0, False, 30.0, 45.0)
    check("CCW 너무 멀다 → 안쪽(좌)로", s < 0, f"steer={s}")


# ─────────────── OrbitProgress: 한 바퀴 누적 ───────────────
def test_progress_full_lap_cw():
    """월드 방위가 0→360 증가(CW 한 바퀴) → 진행각 ~360."""
    p = OrbitProgress()
    prog = 0.0
    for b in range(0, 361, 10):
        prog = p.update(float(b % 360))
    check("CW 한 바퀴 → 360°", abs(prog - 360.0) < 1e-3, f"progress={prog:.1f}")


def test_progress_ccw():
    """월드 방위가 감소(CCW) → 절대 진행각 누적."""
    p = OrbitProgress()
    prog = 0.0
    for b in range(360, -1, -10):
        prog = p.update(float(b % 360))
    check("CCW 한 바퀴 → 360°", abs(prog - 360.0) < 1e-3, f"progress={prog:.1f}")


def test_progress_wraps_360_boundary():
    """350°→10° 같은 경계 넘김도 +20°로 올바르게 누적."""
    p = OrbitProgress()
    p.update(350.0)
    prog = p.update(10.0)
    check("360 경계 넘김 → +20°", abs(prog - 20.0) < 1e-6, f"progress={prog:.1f}")


def test_progress_jitter_cancels():
    """앞뒤 지터는 상쇄된다(순 진행만 남음)."""
    p = OrbitProgress()
    p.update(100.0)
    p.update(110.0)
    prog = p.update(100.0)   # 되돌아옴
    check("지터 상쇄 → 0°", abs(prog) < 1e-6, f"progress={prog:.1f}")


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

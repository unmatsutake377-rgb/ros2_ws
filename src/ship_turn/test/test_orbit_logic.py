"""orbit_logic 단위 테스트 — ROS 없이 돈다.

    python3 src/ship_turn/test/test_orbit_logic.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ship_turn.orbit_logic import (  # noqa: E402
    orbit_steer, orbit_expected_rel, buoy_world_bearing, OrbitProgress, orbit_direction_cw,
    is_active, clamp)

fails = 0


def check(name, cond, extra=""):
    global fails
    print(f"  {'✅' if cond else '❌'} {name} {extra}")
    if not cond:
        fails += 1
    assert cond, name


# ─────────────── 활성 게이트 (enabled × wp_mode) ───────────────
def test_is_active_default_mode3():
    check("enabled=True, mode 3 → 활성", is_active(True, 3, 3) is True)


def test_is_active_other_mode():
    check("enabled=True, mode 2 → 비활성", is_active(True, 2, 3) is False)


def test_is_active_disabled():
    """search_behavior:=hold — enabled=False 면 활성 모드여도 침묵."""
    check("enabled=False, mode 3 → 비활성", is_active(False, 3, 3) is False)


def test_is_active_boot_sentinel():
    """부팅 초기값 wp_mode=-1 은 /wp_mode 첫 수신 전 → 비활성(위험 1)."""
    check("부팅 -1 → 비활성", is_active(True, -1, 3) is False)
    check("active_wp_mode=-1 센티널은 부팅값과 겹친다(금지 이유)", is_active(True, -1, -1) is True)


# ─────────────── 방향 매핑 ───────────────
def test_color_direction():
    check("red → 시계(CW)", orbit_direction_cw("red") is True)
    check("green → 시계(CW)", orbit_direction_cw("GREEN") is True)
    check("white → 반시계(CCW)", orbit_direction_cw("white") is False)
    # 2026-11 경기규정: 노랑 = 좌현선회(반시계). 수정 전엔 None 이라 노랑 공지 때 SEARCH 타임아웃.
    check("yellow → 반시계(CCW)", orbit_direction_cw("yellow") is False)
    check("YELLOW 공백 정리 → 반시계", orbit_direction_cw(" YELLOW ") is False)
    # 검정 = 위치유지 전용, 선회 방향 없음.
    check("black → None", orbit_direction_cw("black") is None)
    check("모르는 색 → None", orbit_direction_cw("blue") is None)
    check("빈 문자열 → None", orbit_direction_cw("") is None)


# ─────────────── orbit_steer: 좌 양수(/image_angle·/candidate_angle 규약) ───────────────
# 🚨 2026-11-04 전엔 "우 양수"(CW·부표 +90 → 접선 0)를 고정하고 있었다. 입·출력이 좌 양수라 CW 요청에
#    CCW 로 돌았다(계약서_ship_turn선회_20261104 §0-3). 아래는 전부 **부호 비대칭** 케이스라 되돌리면 깨진다.
def test_steer_on_circle_cw():
    """CW, 부표 우현(−90°, 좌 양수), 반경 정확 → 접선(정면 0°)."""
    s = orbit_steer(-90.0, 3.0, 3.0, True, 30.0, 45.0)
    check("CW 반경 정확(부표 우현 −90) → 정면(접선)", abs(s) < 1e-6, f"steer={s}")


def test_steer_on_circle_ccw():
    """CCW, 부표 좌현(+90°), 반경 정확 → 접선(정면 0°)."""
    s = orbit_steer(90.0, 3.0, 3.0, False, 30.0, 45.0)
    check("CCW 반경 정확(부표 좌현 +90) → 정면(접선)", abs(s) < 1e-6, f"steer={s}")


def test_steer_orbit_entry_cw_turns_left():
    """ORBIT 진입 틱: 부표 정면(rel 0)·dist 4·R 3·CW → **+60(좌조향)** 이어야 부표가 우현으로 가 CW 가 된다.
    옛 코드는 −60(우조향 → 부표가 좌현 → CCW)을 냈다. 되돌림 방어."""
    s = orbit_steer(0.0, 4.0, 3.0, True, 30.0, 45.0)
    check("CW 진입(rel 0, dist 4) → +60 좌조향", abs(s - 60.0) < 1e-6, f"steer={s}")
    check("되돌림 방어: 음수(우조향)면 CCW 로 돈다", s > 0, f"steer={s}")


def test_steer_orbit_entry_ccw_turns_right():
    """CCW 진입(rel 0, dist 4) → −60(우조향 → 부표가 좌현으로)."""
    s = orbit_steer(0.0, 4.0, 3.0, False, 30.0, 45.0)
    check("CCW 진입(rel 0, dist 4) → −60 우조향", abs(s + 60.0) < 1e-6, f"steer={s}")


def test_steer_too_far_turns_inward():
    """CW, 부표 우현(−90), 너무 멀다(dist>radius) → 부표 쪽 = 우(−)로 튼다."""
    s = orbit_steer(-90.0, 5.0, 3.0, True, 30.0, 45.0)
    check("CW 너무 멀다 → 안쪽(우, −)로", s < 0, f"steer={s}")


def test_steer_too_close_turns_outward():
    """CW, 부표 우현(−90), 너무 가깝다(dist<radius) → 바깥 = 좌(+)로 튼다."""
    s = orbit_steer(-90.0, 1.0, 3.0, True, 30.0, 45.0)
    check("CW 너무 가깝다 → 바깥(좌, +)으로", s > 0, f"steer={s}")


def test_steer_radial_clamped():
    """반경 오차가 커도 gain 은 radial_limit 로 제한된다."""
    s = orbit_steer(-90.0, 100.0, 3.0, True, 30.0, 45.0)   # err=97, k*err 큼
    # 접선 0 − gain(<=45) → |s| 45 이하
    check("반경 보정 clamp", abs(s) <= 45.0 + 1e-6, f"steer={s}")


def test_steer_ccw_far_turns_inward_left():
    """CCW, 부표 좌현(+90°), 너무 멀다 → 부표(좌) 쪽으로(+) 튼다."""
    s = orbit_steer(90.0, 5.0, 3.0, False, 30.0, 45.0)
    check("CCW 너무 멀다 → 안쪽(좌, +)로", s > 0, f"steer={s}")


def test_steer_mirror_symmetry():
    """좌우 대칭: steer_ccw(rel, d) == −steer_cw(−rel, d). 부호 규약을 한쪽만 고치면 깨진다."""
    for rel, d in ((0.0, 4.0), (-90.0, 3.0), (-60.0, 2.0), (30.0, 5.0)):
        a = orbit_steer(rel, d, 3.0, True, 30.0, 45.0)
        b = orbit_steer(-rel, d, 3.0, False, 30.0, 45.0)
        check(f"대칭 rel={rel} d={d}", abs(a + b) < 1e-6, f"cw={a} ccw={b}")


def test_expected_rel():
    """설계 자세: CW → 부표 우현 −90, CCW → 좌현 +90 (LiDAR 추적의 초기 예상 방위)."""
    check("CW 예상 −90", orbit_expected_rel(True) == -90.0)
    check("CCW 예상 +90", orbit_expected_rel(False) == 90.0)


# ─────────────── 월드 방위 합성 = yaw − rel ───────────────
def test_world_bearing_sign():
    """yaw(CW+) 와 rel(좌+)은 부호가 반대: 북향(0)에서 부표가 좌현 90 → 서쪽(270)."""
    check("yaw 0, rel +90(좌) → 270(서)", abs(buoy_world_bearing(0.0, 90.0) - 270.0) < 1e-6)
    check("yaw 0, rel −90(우) → 90(동)", abs(buoy_world_bearing(0.0, -90.0) - 90.0) < 1e-6)
    check("yaw 350, rel −20 → 10", abs(buoy_world_bearing(350.0, -20.0) - 10.0) < 1e-6)


def test_world_bearing_yaw_in_place_no_progress():
    """제자리 요잉: 부표는 북(0)에 고정, 배만 우(CW)로 40° 돌면 부표는 좌현으로 가 rel = +yaw(좌 양수).
    월드 방위 yaw − rel 은 0 그대로 → 진행각 0. 옛 합성 yaw + rel 은 80° 를 찍어 조기 COOLDOWN(계약서 §0-3 b)."""
    p = OrbitProgress()
    prog = 0.0
    old = OrbitProgress()
    old_prog = 0.0
    for yaw in range(0, 41, 5):
        prog = p.update(buoy_world_bearing(float(yaw), float(yaw)))
        old_prog = old.update((float(yaw) + float(yaw)) % 360.0)     # 옛 식 yaw + rel
    check("제자리 요잉 40° → 진행 0", abs(prog) < 1e-6, f"progress={prog:.1f}")
    check("옛 식 yaw+rel 은 80 (2배 과계수, 되돌림 방어 근거)", abs(old_prog - 80.0) < 1e-6, f"old={old_prog:.1f}")


# ─────────────── 발행 전 클램프 ±80 (ship_direction 매핑 0~160) ───────────────
def test_steer_clamp_80():
    """|steer|>80 은 ship_direction 에서 STOP_HOLD 가 된다 → 노드가 ±80 으로 자른다(clamp 로)."""
    check("+95 → +80", clamp(95.0, -80.0, 80.0) == 80.0)
    check("−95 → −80", clamp(-95.0, -80.0, 80.0) == -80.0)
    check("+60 그대로", clamp(60.0, -80.0, 80.0) == 60.0)
    # 진입 직후 rel 0·dist 3.2(err 0.2 → gain 6) → 84 → 클램프 필요한 실제 케이스
    s = orbit_steer(0.0, 3.2, 3.0, True, 30.0, 45.0)
    check("진입 rel 0·dist 3.2 → 84 (>80, 클램프 대상)", s > 80.0, f"steer={s}")


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

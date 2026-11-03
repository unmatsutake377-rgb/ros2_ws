"""dock_fsm_logic 단위 테스트 — ROS 없이 돈다.

    python3 src/ship_dock/test/test_dock_fsm_logic.py
    tools/run_tests.sh ship_dock
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ship_dock.dock_fsm_logic import (  # noqa: E402
    approach_decision, check_realign_min, CONTACT, ALIGN_STANDOFF, ALIGN, STEER)

fails = 0

# 기본값 묶음 = ship_dock.yaml / 코드 기본값 (contact 1.0, standoff 3.0, realign_min 1.5, align_tol 8, realign_tol 25)
P = dict(contact_dist=1.0, standoff_dist=3.0, realign_min_dist=1.5, align_tol=8.0, realign_tol=25.0)


def decide(dist, angle, standoff_aligned=False, **over):
    p = dict(P)
    p.update(over)
    return approach_decision(dist, angle, standoff_aligned, **p)


def check(name, cond, extra=""):
    global fails
    print(f"  {'✅' if cond else '❌'} {name} {extra}")
    if not cond:
        fails += 1
    assert cond, name


# ─────────────── 하한 밖: 종전 동작 그대로 ───────────────
def test_outside_floor_standoff_realign():
    # 2.5 m(standoff 3.0 안, 하한 1.5 밖), 10° > align_tol 8 → standoff 1회 재정렬, 플래그 True
    check("2.5 m 10° 플래그 F → ALIGN_STANDOFF, 플래그 True",
          decide(2.5, 10.0) == (ALIGN_STANDOFF, True))
    check("음수 각도도 |angle| 로 본다: 2.5 m -10° → ALIGN_STANDOFF",
          decide(2.5, -10.0) == (ALIGN_STANDOFF, True))


def test_outside_floor_realign_tol():
    # 플래그 True 라 standoff 분기는 안 타고, 30° > realign_tol 25 → ALIGN
    check("2.0 m 30° 플래그 T → ALIGN, 플래그 유지",
          decide(2.0, 30.0, standoff_aligned=True) == (ALIGN, True))
    check("standoff 밖(3.5 m) 30° → ALIGN (realign_tol 은 거리 무관)",
          decide(3.5, 30.0) == (ALIGN, False))


def test_outside_floor_steer_when_aligned():
    check("2.0 m 5° → STEER", decide(2.0, 5.0) == (STEER, False))
    check("standoff 밖(3.5 m) 10° → STEER (standoff 재정렬은 standoff 안에서만)",
          decide(3.5, 10.0) == (STEER, False))
    check("align_tol 경계 8.0° 는 정면(> 아님) → STEER", decide(2.5, 8.0) == (STEER, False))


# ─────────────── 하한 안: 두 종류 재정렬 모두 억제 ───────────────
def test_inside_floor_suppresses_standoff_realign():
    # 1.2 m 10°, 플래그 F — 하한 없었으면 ALIGN_STANDOFF 였을 조건
    check("1.2 m 10° 플래그 F → STEER (standoff 재정렬 억제)", decide(1.2, 10.0) == (STEER, False))
    check("억제 시 플래그는 안 바뀐다(False 유지)", decide(1.2, 10.0)[1] is False)


def test_inside_floor_suppresses_realign_tol():
    # 1.2 m 30°, 플래그 T — 하한 없었으면 ALIGN 이었을 조건 (시뮬 실패 3 런의 경로)
    check("1.2 m 30° 플래그 T → STEER (realign_tol 재정렬 억제)",
          decide(1.2, 30.0, standoff_aligned=True) == (STEER, True))
    check("1.2 m 30° 플래그 F → STEER (둘 다 억제)", decide(1.2, 30.0) == (STEER, False))
    check("1.04 m(실패 런 유실 거리) 30° → STEER", decide(1.04, 30.0) == (STEER, False))


# ─────────────── contact 우선 ───────────────
def test_contact_first():
    check("0.9 m 30° → CONTACT (하한·각도보다 먼저)", decide(0.9, 30.0)[0] == CONTACT)
    check("정확히 contact 1.0 m → CONTACT (<=)", decide(1.0, 0.0)[0] == CONTACT)
    check("CONTACT 는 플래그를 안 건드린다", decide(0.9, 30.0, standoff_aligned=True) == (CONTACT, True))


# ─────────────── standoff 1회 플래그 ───────────────
def test_standoff_once():
    d1, flag = decide(2.8, 10.0, standoff_aligned=False)
    check("1회차: ALIGN_STANDOFF", d1 == ALIGN_STANDOFF and flag is True)
    d2, flag2 = decide(2.5, 10.0, standoff_aligned=flag)
    check("2회차(같은 조건, 플래그 T): 10° 는 realign_tol 25 미만 → STEER", (d2, flag2) == (STEER, True))
    d3, _ = decide(2.5, 30.0, standoff_aligned=flag)
    check("2회차 30° 는 ALIGN(일반 재정렬)이지 ALIGN_STANDOFF 아님", d3 == ALIGN)


# ─────────────── 경계값 ───────────────
def test_floor_boundary():
    # in_floor = dist < realign_min (strict) → dist == realign_min 은 하한 밖
    check("dist == realign_min 1.5 m, 10° → ALIGN_STANDOFF (하한 밖)", decide(1.5, 10.0) == (ALIGN_STANDOFF, True))
    check("dist == realign_min 1.5 m, 30° 플래그 T → ALIGN", decide(1.5, 30.0, standoff_aligned=True)[0] == ALIGN)
    check("1.4999 m 10° → STEER (하한 안)", decide(1.4999, 10.0) == (STEER, False))
    check("1.4999 m 30° 플래그 T → STEER", decide(1.4999, 30.0, standoff_aligned=True) == (STEER, True))


def test_standoff_boundary():
    check("dist == standoff 3.0 m 10° → ALIGN_STANDOFF (<=)", decide(3.0, 10.0) == (ALIGN_STANDOFF, True))
    check("3.01 m 10° → STEER", decide(3.01, 10.0) == (STEER, False))


def test_floor_disabled_when_below_contact():
    # 하한 <= contact 로 설정되면 contact 가 먼저라 하한 안 구간이 없다 → 종전 동작(재정렬)으로 조용히 복귀.
    # 이것이 ERROR 로 드러나야 하는 이유 (check_realign_min).
    check("하한 1.0(=contact) 1.2 m 30° → ALIGN (하한 무효)",
          decide(1.2, 30.0, standoff_aligned=True, realign_min_dist=1.0)[0] == ALIGN)


# ─────────────── check_realign_min (부팅 방어) ───────────────
def test_check_realign_min():
    check("1.5 / contact 1.0 / standoff 3.0 → ok", check_realign_min(1.5, 1.0, 3.0) == "ok")
    check("하한 == contact → below_contact", check_realign_min(1.0, 1.0, 3.0) == "below_contact")
    check("하한 < contact → below_contact", check_realign_min(0.5, 1.0, 3.0) == "below_contact")
    check("하한 == standoff → above_standoff", check_realign_min(3.0, 1.0, 3.0) == "above_standoff")
    check("하한 > standoff → above_standoff", check_realign_min(4.0, 1.0, 3.0) == "above_standoff")
    check("contact 바로 위(1.01) → ok", check_realign_min(1.01, 1.0, 3.0) == "ok")


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

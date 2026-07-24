#!/usr/bin/env python3
"""
mode_gate 순수 로직 테스트 + 3-4 구조 검증(발행자 충돌 정적 검사).
    python3 src/color_shape_detector/test/test_mode_gate.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from color_shape_detector.mode_gate import (  # noqa: E402
    ModeGate, check_publisher_conflicts,
)

_passed = 0
_total = 0


def check(name, fn):
    global _passed, _total
    _total += 1
    try:
        fn()
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")
        return
    _passed += 1
    print(f"  ✅ {name}")


# ------------------------------------------------------------------ 게이트
def test_no_mode_is_inactive():
    """부팅 직후 /wp_mode 를 못 받았으면 비활성. 모르면 발행하지 않는다."""
    g = ModeGate([0, 1])
    active, why = g.state(0.0)
    assert not active and why == ModeGate.R_NO_MODE
    assert g.mode is None


def test_active_when_mode_matches():
    g = ModeGate([0, 1], stale_sec=2.0)
    g.update(1, 100.0)
    assert g.is_active(100.5)
    g.update(0, 101.0)
    assert g.is_active(101.0)


def test_inactive_on_other_mode():
    g = ModeGate([0, 1], stale_sec=2.0)
    g.update(3, 100.0)
    active, why = g.state(100.0)
    assert not active and why == ModeGate.R_OTHER_MODE


def test_stale_is_inactive():
    """
    FSM 이 죽으면 옛 모드가 얼어붙는다. 그대로 발행하면 배가 지난 미션을 계속 한다.
    경계는 '초과' — 정확히 stale_sec 이면 아직 유효.
    """
    g = ModeGate([1], stale_sec=2.0)
    g.update(1, 100.0)
    assert g.is_active(102.0), "정확히 임계면 유효"
    active, why = g.state(102.001)
    assert not active and why == ModeGate.R_STALE


def test_stale_then_recovers():
    g = ModeGate([1], stale_sec=2.0)
    g.update(1, 100.0)
    assert not g.is_active(103.0)
    g.update(1, 103.0)
    assert g.is_active(103.0), "복구는 즉시"


def test_bad_input_ignored():
    """None/문자열이 슬롯을 오염시키면 안 된다."""
    g = ModeGate([1], stale_sec=2.0)
    g.update(1, 100.0)
    assert not g.update(None, 100.1)
    assert not g.update("dock", 100.2)
    assert g.mode == 1, "마지막 정상값이 남아야 한다"
    assert g.is_active(100.3)


def test_int_like_accepted():
    g = ModeGate([7], stale_sec=2.0)
    assert g.update(7.0, 0.0)
    assert g.is_active(0.0)


def test_empty_active_modes_never_active():
    g = ModeGate([], stale_sec=2.0)
    g.update(1, 0.0)
    assert not g.is_active(0.0)


# ------------------------------------------------------------------ 🚨 발행자 충돌
# 3-4 구조의 실제 배치. 이 표가 곧 계약이다.
#   (권위 출처: 각 미션 노드의 active_wp_mode / healthcheck 매핑표)
LAYOUT = {
    "gate": ({0, 1}, {"/red_angle", "/green_angle"}),
    "turn": ({2, 3}, {"/image_angle", "/image_color"}),
    "dock": ({7}, {"/image_angle"}),
}


def test_no_publisher_conflict_in_layout():
    """
    🚨 dock 과 turn 은 둘 다 /image_angle 을 발행한다.
    상주시키면서 모드가 겹치면 한 토픽에 발행자 2개가 되어 값이 섞인다 — 에러 없이.
    이 배치에서 겹침이 없는지 정적으로 못 박는다.
    """
    conflicts = check_publisher_conflicts(LAYOUT)
    assert not conflicts, f"발행자 충돌: {conflicts}"


def test_conflict_detector_actually_detects():
    """검사기가 진짜로 잡는지 — 통과만 하는 검사는 검사가 아니다."""
    bad = dict(LAYOUT)
    bad["dock"] = ({3, 7}, {"/image_angle"})     # turn 의 모드 3 과 겹치게
    conflicts = check_publisher_conflicts(bad)
    assert conflicts, "겹치는데 못 잡았다"
    tp, mode, names = conflicts[0]
    assert tp == "/image_angle" and mode == 3
    assert names == ["dock", "turn"], names


def test_layout_matches_mission_nodes():
    """
    비전 노드의 담당 모드가 미션 노드의 active_wp_mode 와 맞는지.
    🚨 작년엔 여기가 어긋나 있었다:
       · mode 0: ship_gate 가 담당(6b 에서 ship_last 인수)인데 매니저는 비전 OFF
       · mode 7: ship_dock 이 담당(9→7 수정 완료)인데 매니저는 9 로 띄우고 7 은 비전 OFF
       → 게이트(mode 0)와 도킹(mode 7)이 눈 없이 돌았다.
    """
    mission = {                      # 각 노드가 스스로 선언한 값
        "ship_gate": {0, 1},         # active_wp_modes=[0,1]
        "ship_back": {2},            # active_wp_mode=2   (/image_angle 소비)
        "ship_turn": {3},            # active_wp_mode=3   (/image_angle 소비)
        "ship_dock": {7},            # active_wp_mode=7   (/image_angle 소비)
    }
    need_red = mission["ship_gate"]
    need_image = mission["ship_back"] | mission["ship_turn"] | mission["ship_dock"]

    have_red = LAYOUT["gate"][0]
    have_image = LAYOUT["turn"][0] | LAYOUT["dock"][0]

    assert need_red <= have_red, \
        f"/red_angle 이 필요한 모드 {sorted(need_red - have_red)} 에 검출기가 없다"
    assert need_image <= have_image, \
        f"/image_angle 이 필요한 모드 {sorted(need_image - have_image)} 에 검출기가 없다"


def test_avoidance_modes_have_no_detector():
    """mode 5, 8 은 순수 회피 구간(담당 노드 없음). 검출기를 돌릴 이유가 없다."""
    for m in (5, 8):
        for name, (modes, _t) in LAYOUT.items():
            assert m not in modes, f"회피 모드 {m} 에 {name} 이 활성이다(불필요 CPU)"


def main():
    print("=== mode_gate 테스트 ===")
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            check(name[5:], fn)
    print(f"\n{_passed}/{_total} 통과")
    return 0 if _passed == _total else 1


if __name__ == "__main__":
    sys.exit(main())

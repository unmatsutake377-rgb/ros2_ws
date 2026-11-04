"""GateCounter / gate_candidate 단위 테스트 — ROS 없이 돈다.

    python3 src/ship_gate/test/test_gate_logic.py      (직접 실행)
    tools/run_tests.sh ship_gate                        (pytest)

🚨 check() 는 assert 를 던진다. 예전엔 print 만 해서 pytest 가 ❌ 를 통과로 세었다(11-04 계약서 §0 덤).

부호 규약(11-04 확정): /red_angle·/green_angle 은 상대각, **왼쪽 양수**(vision_geom.angle_from_pixel).
  빨강이 좌현이면 /red_angle > 0. 아래 단일 부표 테스트는 전부 **비대칭** 입력이라 부호가 뒤집히면 실패한다.
"""

import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ship_gate.gate_logic import (  # noqa: E402
    GateCounter, gate_candidate, circular_mid)

def check(name, cond, extra=""):
    """실패하면 AssertionError — pytest·직접 실행 둘 다 잡는다."""
    print(f"  {'✅' if cond else '❌'} {name} {extra}")
    assert cond, f"{name} {extra}"


# ─────────────── gate_candidate: 쌍/단일/제약 ───────────────
def test_pair_midpoint():
    """빨강 +20°(좌현), 초록 -20°(우현) → 중점 0° (정면), 유효 쌍.
    (대칭 입력이라 부호 오류는 못 잡는다 — 그건 아래 단일 부표 테스트의 몫)"""
    steer, valid = gate_candidate(20.0, -20.0, True, 5.0, 80.0, 40.0)
    check("정상 쌍 → 중점 정면", valid and abs(steer) < 1e-6, f"steer={steer}")


def test_pair_too_wide_rejected():
    """빨강 +60°, 초록 -60° → 분리 120° > max 80° → 쌍 아님, 가까운 쪽만."""
    steer, valid = gate_candidate(60.0, -60.0, True, 5.0, 80.0, 40.0)
    check("너무 벌어진 쌍 → 쌍 거부", not valid, f"steer={steer}")


def test_pair_too_narrow_rejected():
    """분리 3° < min 5° → 같은 게이트 아님(중복 검출) → 쌍 거부."""
    _, valid = gate_candidate(13.0, 10.0, True, 5.0, 80.0, 40.0)
    check("너무 붙은 쌍 → 쌍 거부", not valid)


def test_single_red_on_port():
    """빨강만 +30°(좌현 = 양수) 보임, red_on_port → 중심은 빨강 우측 → 30-40 = -10°."""
    steer, valid = gate_candidate(30.0, None, True, 5.0, 80.0, 40.0)
    check("빨강만(좌현 +) → 우측(-)으로 오프셋", (not valid) and abs(steer + 10.0) < 1e-6, f"steer={steer}")


def test_single_green_on_port():
    """초록만 -30°(우현 = 음수) 보임, red_on_port(초록=우현) → 중심은 초록 좌측 → -30+40 = +10°."""
    steer, valid = gate_candidate(None, -30.0, True, 5.0, 80.0, 40.0)
    check("초록만(우현 -) → 좌측(+)으로 오프셋", (not valid) and abs(steer - 10.0) < 1e-6, f"steer={steer}")


def test_red_on_port_flips():
    """red_on_port=False 면 오프셋 방향이 뒤집힌다. 빨강 +30: true → 음수, false → 양수."""
    s_true, _ = gate_candidate(30.0, None, True, 5.0, 80.0, 40.0)
    s_false, _ = gate_candidate(30.0, None, False, 5.0, 80.0, 40.0)
    check("red_on_port 뒤집으면 방향 반대", (s_true < 0) and (s_false > 0), f"{s_true} vs {s_false}")


def test_single_steers_toward_center_not_buoy():
    """빨강 +30 하나만 → 조향 |각| < 30 (부표 쪽으로 '더' 꺾지 않는다). 1글자 되돌림(+offset) 방어."""
    steer, _ = gate_candidate(30.0, None, True, 5.0, 80.0, 40.0)
    check("단일 부표 → 부표보다 안쪽으로", abs(steer) < 30.0, f"steer={steer}")
    steer_g, _ = gate_candidate(None, -30.0, True, 5.0, 80.0, 40.0)
    check("단일 초록 → 부표보다 안쪽으로", abs(steer_g) < 30.0, f"steer={steer_g}")


def test_none_visible():
    """둘 다 안 보임 → 게이트 없음."""
    steer, valid = gate_candidate(None, None, True, 5.0, 80.0, 40.0)
    check("둘 다 안 보임 → None", steer is None and not valid)


def test_wide_pair_uses_nearer():
    """벌어진 쌍(빨강 +75°, 초록 -25°, 분리 100°>80) → 쌍 거부, 가까운 초록(-25)만.
    초록(우현) → 중심은 좌측(+) → -25+40 = +15."""
    steer, valid = gate_candidate(75.0, -25.0, True, 5.0, 80.0, 40.0)
    check("벌어진 쌍 → 가까운 부표만", (not valid) and abs(steer - 15.0) < 1e-6, f"steer={steer}")


def test_pair_boundary_inclusive():
    """분리각이 정확히 max_sep(80°)면 유효 쌍(경계 포함)."""
    _, valid = gate_candidate(70.0, -10.0, True, 5.0, 80.0, 40.0)
    check("분리 80°(경계) → 유효 쌍", valid)


# ─────────────── GateCounter: 통과 계수 ───────────────
# (세 곳 일치 검사용 — 노드 파일·yaml 을 정규식으로 읽는다)
_HERE = os.path.dirname(os.path.abspath(__file__))
_NODE_SRC = os.path.join(_HERE, '..', 'ship_gate', 'ship_gate.py')
_NODE_YAML = os.path.join(_HERE, '..', 'config', 'ship_gate.yaml')
_VISION_YAML = os.path.join(_HERE, '..', '..', 'color_shape_detector', 'config', 'vision.yaml')


def _node_default(param):
    """ship_gate.py 의 declare_parameter("<param>", <기본값>) 기본값. 노드는 rclpy 라 import 불가 → 정규식."""
    src = open(_NODE_SRC, encoding='utf-8').read()
    m = re.search(r'declare_parameter\("%s",\s*([-0-9.]+)\)' % re.escape(param), src)
    assert m, f"ship_gate.py 에 declare_parameter(\"{param}\", <숫자>) 가 없다"
    return float(m.group(1))


def _yaml_value(path, key):
    """yaml 의 `key: <숫자>` 한 줄(pyyaml 없이). 없으면 None."""
    if not os.path.exists(path):
        return None
    m = re.search(r'^\s*%s:\s*([-0-9.]+)' % re.escape(key), open(path, encoding='utf-8').read(), re.M)
    return float(m.group(1)) if m else None


# 전제: 카메라 HFOV 63.9°(OAK-1 PoE 640×360 ISP 1/3, 11-04 전환) → 부표 |각| 은 반화각 31.95° 를 못 넘는다
# (밖이면 10000 = 안 보임). 임계 T 는 그 안이어야 하고, 아래 입력 각도는 전부 ≤ HALF 로 둔다.
#   T   = 대회 기본 임계(세 곳 일치 검사와 같은 값이어야 뜻이 있다 → test_fov_constants 가 검사)
#   OUT = 임계는 넘었지만 아직 화면 안(T < OUT < HALF) — "통과" 틱의 입력
#   IN  = 전방(≤ T) 틱의 입력 중 가장 큰 값
# 숫자를 바꿀 땐 이 세 개만. (D455 시절: HALF 40.16, T 35, OUT 36~39)
HALF = 31.95
T = 28.0
OUT = 29.0
OUT2 = 30.0     # OUT 보다 더 바깥(여전히 < HALF) — 정체·두 번째 게이트용
IN = 25.0


def test_fov_constants():
    """테스트 상수가 실제 설정과 어긋나면 아래 시나리오가 '통과' 해도 뜻이 없다."""
    hfov_vision = _yaml_value(_VISION_YAML, "hfov_deg")
    check("HALF == vision.yaml hfov/2", hfov_vision is not None and abs(HALF - hfov_vision / 2.0) < 1e-6,
          f"{HALF} vs {hfov_vision}")
    check("T == 대회 기본 pass_behind", T == GateCounter().pass_behind, f"{T} vs {GateCounter().pass_behind}")
    check("T < OUT < OUT2 < HALF", T < OUT < OUT2 < HALF)
    check("IN ≤ T", IN <= T)
    check("여유 HALF−T ≥ 3°", HALF - T >= 3.0, f"{HALF - T:.2f}")


def test_count_within_fov():
    """게이트를 전방에서 보다가 부표가 임계(±T) 를 넘으면 1 카운트 — FOV 안 각도로."""
    c = GateCounter(T)
    for _ in range(5):
        c.update(gate_ahead=True, buoy_abs_angles=[10.0, 12.0])   # 전방 접근
    check("접근 중엔 카운트 0", c.count == 0)
    c.update(gate_ahead=False, buoy_abs_angles=[OUT, 8.0])       # 한쪽이 임계 밖으로
    check("부표가 임계 넘음 → 1 카운트", c.count == 1)


def test_default_threshold_inside_fov():
    """기본 임계가 세 곳(gate_logic 기본 인자·ship_gate.py 기본값·yaml) 같고 카메라 반화각 안인지.
    대회 launch 는 yaml 을 안 넘기므로 코드 기본값이 대회값(계약서 §4)."""
    logic_default = GateCounter().pass_behind
    node_default = _node_default("pass_behind_deg")
    yaml_val = _yaml_value(_NODE_YAML, "pass_behind_deg")
    hfov_node = _node_default("cam_hfov_deg")
    hfov_yaml = _yaml_value(_NODE_YAML, "cam_hfov_deg")
    hfov_vision = _yaml_value(_VISION_YAML, "hfov_deg")
    check("GateCounter 기본 == ship_gate.py 기본", logic_default == node_default, f"{logic_default} vs {node_default}")
    check("ship_gate.py 기본 == yaml", node_default == yaml_val, f"{node_default} vs {yaml_val}")
    check("cam_hfov_deg 코드 == yaml", hfov_node == hfov_yaml, f"{hfov_node} vs {hfov_yaml}")
    if hfov_vision is not None:
        check("cam_hfov_deg == vision.yaml hfov_deg", hfov_node == hfov_vision, f"{hfov_node} vs {hfov_vision}")
    check("기본 임계 < 카메라 반화각", logic_default < hfov_node / 2.0, f"{logic_default} vs {hfov_node / 2.0}")


def test_realistic_sequence_never_exceeds_fov():
    """실제 접근 시퀀스(10→20→IN→OUT2→사라짐): 75 임계는 영원히 0(11-04 발견 재현), T 는 1."""
    seq = [[10.0, 11.0], [20.0, 19.0], [IN, 23.0], [OUT2, 26.0], []]
    old = GateCounter(75.0)
    new = GateCounter(T)
    for i, angles in enumerate(seq):
        ahead = i == 0
        old.update(ahead, angles)
        new.update(ahead, angles)
    check("75° 임계는 FOV 밖 → 0 (문서화)", old.count == 0, f"count={old.count}")
    check("FOV 안 임계 → 1", new.count == 1, f"count={new.count}")


def test_no_double_count_plateau():
    """같은 게이트로 여러 틱 임계 밖에 있어도 두 번 세지 않는다."""
    c = GateCounter(T)
    c.update(True, [10.0, 12.0])
    c.update(False, [OUT])       # 통과 → 1
    for _ in range(10):
        c.update(False, [OUT2])   # 계속 임계 밖
    check("이중 카운트 없음", c.count == 1, f"count={c.count}")


def test_no_double_count_yaw_oscillation():
    """카운트 뒤 요잉으로 같은 게이트가 다시 '전방 유효 쌍' 이 돼도 재-arming 안 함 —
    같은 부표라 LiDAR 거리가 카운트 시점 거리(d_ref) 근처(≤ d_ref + margin)."""
    c = GateCounter(T, 1.0)
    c.update(True, [10.0, 12.0], 5.0)
    c.update(True, [20.0, 10.0], 4.0)           # d_ref 가 될 마지막 전방 쌍 거리
    c.update(False, [OUT, 5.0])                # 1
    check("첫 카운트", c.count == 1, f"count={c.count}")
    c.update(True, [IN, 5.0], 3.5)            # 요잉 복귀 — 노드는 both_ahead 를 매 틱 재평가해 True 를 준다
    c.update(False, [OUT, 5.0])                # 다시 임계 밖
    check("같은 거리면 재카운트 없음", c.count == 1, f"count={c.count}")
    for d in (4.9, 5.0, 3.0, 4.5):              # d_ref+margin = 5.0 까지(경계 포함)는 같은 게이트
        c.update(True, [IN, 5.0], d)
        c.update(False, [OUT, 5.0])
    check("반복 요잉에도 1 유지", c.count == 1, f"count={c.count}")


def test_rearm_after_gap():
    """(i) 둘 다 안 보이는 틱(빈 리스트)을 한 번 지나면 다음 게이트로 재-arming → 2 (거리 없어도)."""
    c = GateCounter(T)
    c.update(True, [10.0, 12.0], 4.0)
    c.update(False, [OUT, 5.0])       # 1
    c.update(True, [IN, 5.0], 3.5)   # 요잉 복귀(gap 전) — 무시돼야 함
    c.update(False, [])                # gap: 둘 다 안 보임
    c.update(True, [8.0, 9.0])         # 다음 게이트 전방
    c.update(False, [OUT])
    check("gap 뒤 재-arming → 2", c.count == 2, f"count={c.count}")


def test_rearm_after_farther_new_pair():
    """(ii) 카운트 뒤 전방 쌍의 거리가 d_ref + margin 을 넘으면 새 게이트 → 재-arming → 2.
    다음 게이트가 카메라 거리 안이라 '둘 다 안 보임' 틱이 안 생기는 경우(11-04 시뮬 누락 1/40)."""
    c = GateCounter(T, 1.0)
    c.update(True, [10.0, 12.0], 4.0)           # d_ref = 4.0
    c.update(False, [OUT, 5.0])                # 1
    c.update(True, [IN, 5.0], 3.0)            # 요잉 복귀 — 아님
    c.update(False, [OUT, 5.0])
    check("요잉 복귀로는 재카운트 없음", c.count == 1, f"count={c.count}")
    c.update(True, [12.0, 10.0], 9.0)           # 다음 게이트 9 m > 4+1 → 재-arming
    c.update(False, [OUT])
    check("더 먼 새 쌍 → 2", c.count == 2, f"count={c.count}")


def test_rearm_margin_boundary():
    """경계: d_ref + margin 과 같으면 같은 게이트, 그보다 크면 새 게이트."""
    for d, expect in ((5.0, 1), (5.01, 2)):
        c = GateCounter(T, 1.0)
        c.update(True, [10.0, 12.0], 4.0)
        c.update(False, [OUT, 5.0])            # 1, d_ref 4.0
        c.update(True, [12.0, 10.0], d)
        c.update(False, [OUT])
        check(f"pair_dist={d} → count {expect}", c.count == expect, f"count={c.count}")


def test_d_ref_is_min_not_last():
    """d_ref 는 arming 뒤 측정 최소값: [8, 3, None] 뒤 카운트 → d_ref 3 → 새 쌍 4.5 는 3+1 < 4.5 로 재-arming.
    (마지막 값 8 을 썼다면 4.5 ≤ 9 라 '같은 게이트' 로 묶여 누락 — LiDAR 가 근접 부표를 놓치는 경우, 리뷰 11-04)"""
    c = GateCounter(T, 1.0)
    c.update(True, [10.0, 12.0], 8.0)
    c.update(True, [20.0, 10.0], 3.0)
    c.update(True, [IN, 12.0], None)          # 근접에서 LiDAR 가 놓침 — 최소값 3 은 유지
    c.update(False, [OUT, 5.0])                # 1, d_ref = 3
    c.update(True, [12.0, 10.0], 4.5)           # 다음 쌍 4.5 > 3 + 1 → 재-arming
    c.update(False, [OUT])
    check("d_ref 최소값 → 4.5 쌍에 재-arming → 2", c.count == 2, f"count={c.count}")
    # min 과 '마지막 값' 이 갈리는 케이스: [3, 8] — 근접 부표를 놓치고 먼 반사(8)가 마지막에 오면
    # 마지막 값 기준(d_ref 8)은 다음 쌍 4.5 를 '같은 게이트' 로 묶어 누락, 최소값 기준(d_ref 3)은 재-arming.
    c2 = GateCounter(T, 1.0)
    c2.update(True, [10.0, 12.0], 3.0)
    c2.update(True, [20.0, 10.0], 8.0)
    c2.update(False, [OUT, 5.0])               # 1, d_ref = min(3, 8) = 3
    c2.update(True, [12.0, 10.0], 4.5)          # 4.5 > 3 + 1 → 재-arming (마지막 값이면 4.5 ≤ 9 → 차단)
    c2.update(False, [OUT])
    check("[3, 8] 뒤 4.5 쌍 → 최소값 기준으로 2", c2.count == 2, f"count={c2.count}")


def test_unknown_distance_rearms():
    """거리 미상이면 막지 않는다: d_ref 가 None(카운트 전 LiDAR 못 봄) 또는 pair_dist 가 None 이면 전방 쌍 틱에서
    재-arming. (이 경우 요잉 복귀 이중 카운트는 못 막는다 — 설계상 '카운트를 죽이지 않는' 쪽을 택함)"""
    c = GateCounter(T, 1.0)
    c.update(True, [10.0, 12.0])                # 거리 없음 → d_ref None
    c.update(False, [OUT, 5.0])                # 1
    c.update(True, [8.0, 9.0], 3.0)
    c.update(False, [OUT])
    check("d_ref 미상 → 재-arming → 2", c.count == 2, f"count={c.count}")
    c2 = GateCounter(T, 1.0)
    c2.update(True, [10.0, 12.0], 4.0)
    c2.update(False, [OUT, 5.0])               # 1, d_ref 4.0
    c2.update(True, [8.0, 9.0])                 # 새 쌍인데 LiDAR 가 못 봄
    c2.update(False, [OUT])
    check("pair_dist 미상 → 재-arming → 2", c2.count == 2, f"count={c2.count}")


def test_arming_has_no_distance_condition():
    """넓은 게이트: 전방 유효 쌍이 멀리(12 m) 보여도 거리 조건 없이 armed → 통과 때 카운트.
    (옛 gate_arm_dist_m 6 m 조건은 폭 7 m 이상에서 arming 창이 아예 안 생겼다)"""
    c = GateCounter(T)
    c.update(True, [IN, IN], 12.0)          # 폭 넓어 둘 다 IN 근처(≤ T), 멀다
    c.update(False, [OUT, OUT], None)
    check("거리 조건 없이 arming → 1", c.count == 1, f"count={c.count}")


def test_rearm_margin_three_places():
    """rearm_far_margin_m 기본값이 GateCounter·ship_gate.py·yaml 세 곳 같고, 노드가 counter.update 에 거리를 넘긴다.
    옛 gate_arm_dist_m 는 죽은 파라미터로 남지 않았는지도."""
    logic_default = GateCounter().rearm_far_margin
    node_default = _node_default("rearm_far_margin_m")
    yaml_val = _yaml_value(_NODE_YAML, "rearm_far_margin_m")
    check("margin 세 곳 일치", logic_default == node_default == yaml_val, f"{logic_default}/{node_default}/{yaml_val}")
    src = open(_NODE_SRC, encoding='utf-8').read()
    check("counter.update 에 거리 전달", re.search(r"self\.counter\.update\(gate_ahead, abs_angles, pair_dist\)", src))
    check("gate_arm_dist_m 잔존 없음(코드)", "gate_arm_dist_m" not in src.replace("옛 gate_arm_dist_m", ""))
    check("gate_arm_dist_m 잔존 없음(yaml)", _yaml_value(_NODE_YAML, "gate_arm_dist_m") is None)


def test_two_gates_sequential():
    """두 게이트를 순차 통과(사이에 둘 다 안 보임) → 2."""
    c = GateCounter(T)
    c.update(True, [8.0, 9.0])
    c.update(False, [OUT])          # 1
    c.update(False, [])              # 지나감
    c.update(True, [7.0, 6.0])
    c.update(False, [OUT2])          # 2
    check("두 게이트 순차 → 2", c.count == 2, f"count={c.count}")


def test_grace_plateau_counts_once():
    """검출기 폴백(grace 2 s)이 같은 각을 반복 발행해도 한 번만 센다."""
    c = GateCounter(T)
    c.update(True, [10.0, 12.0])
    for _ in range(20):
        c.update(False, [OUT2])
    check("폴백 반복 → 1", c.count == 1, f"count={c.count}")


def test_no_count_without_arming():
    """전방에서 유효 게이트를 못 봤으면(arming 없음) 부표가 임계 밖에 있어도 안 센다.
    (먼 부표가 잠깐 크게 잡힌 오탐 등)"""
    c = GateCounter(T)
    for _ in range(5):
        c.update(gate_ahead=False, buoy_abs_angles=[OUT2])
    check("arming 없이는 카운트 0", c.count == 0)


def test_gap_before_first_count_is_harmless():
    """첫 카운트 전엔 gap 요구가 없다 — 빈 틱이 섞여도 arming 그대로."""
    c = GateCounter(T)
    c.update(True, [10.0, 12.0])
    c.update(False, [])
    c.update(False, [OUT])
    check("첫 게이트는 gap 없이도 카운트", c.count == 1, f"count={c.count}")


# ─────────────── helpers ───────────────
def test_circular_mid_wrap():
    """±180 경계를 넘는 중점."""
    check("circular_mid(170,-170)=180 근처",
          abs(abs(circular_mid(170.0, -170.0)) - 180.0) < 1e-6)


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    fails = 0
    for fn in tests:
        try:
            fn()
        except AssertionError as e:
            fails += 1
            print(f"  ❌ {fn.__name__}: {e}")
    print(f"\n{len(tests) - fails}/{len(tests)} 통과")
    sys.exit(1 if fails else 0)

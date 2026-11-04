#!/usr/bin/env python3
"""ship_dock APPROACH 조향 ±80° 클램프 테스트 — 실제 ShipDock.timer_cb 를 스텁 노드로 돌린다.

실행: python3 src/ship_dock/test/test_dock_steer_clamp.py
      tools/run_tests.sh ship_dock

계약서 docs/문제와작업/계약서_각도클램프_20261104.md §1 B1 · §6 · §10.
규약: /candidate_angle 180.0 정확 일치 = 후진(ship_direction (B) 예외). APPROACH 조향이 우연히 180.0 이
  되지 않도록 발행 직전에만 ±STEER_LIMIT_DEG(80) 로 자른다(부호 그대로, rel_to_raw 없음).
  BACKOFF 180.0·SPIN 5000/6000·STOP 50000·DONE(원인별) 은 그대로. ALIGN 판정·approach_decision 은 클램프 전 값.

ship_dock.py 는 rclpy 를 import 하므로 Mac(ROS 없음)에선 그대로 못 읽는다.
→ tools/sim/dock_sim.py 와 같은 rclpy 스텁 노드로 **실제 노드 코드**를 import 한다.
  스텁은 import 하는 동안만 sys.modules 에 넣고 되돌린다(다른 테스트 파일의 스텁과 섞이지 않게).
  import 뒤 모듈 전역 Float32·time 을 테스트용으로 덮어써 실행 순서 의존을 없앤다.
"""

import importlib.util
import math
import os
import sys
import types

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, PKG)    # ship_dock.py 의 `from ship_dock.dock_fsm_logic import …` 용


# ---- rclpy 스텁 노드 (dock_sim.py:43-96 방식) ----
class _Param:
    def __init__(self, v):
        self.value = v


class _Logger:
    def __init__(self):
        self.lines = []    # (심각도, 메시지, kwargs)

    def info(self, m, **kw):
        self.lines.append(('INFO', m, kw))

    def warn(self, m, **kw):
        self.lines.append(('WARN', m, kw))

    def error(self, m, **kw):
        self.lines.append(('ERROR', m, kw))


class _Pub:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg.data)


class _Node:
    """파라미터는 코드 기본값(= 대회값, launch 가 ship_dock 에 yaml 을 안 넘긴다)."""

    def __init__(self, name):
        self._logger = _Logger()
        self.pubs = []

    def declare_parameter(self, name, default):
        return _Param(default)

    def create_publisher(self, typ, topic, qos):
        p = _Pub()
        self.pubs.append(p)
        return p

    def create_subscription(self, *a, **k):
        return None

    def create_timer(self, *a, **k):
        return None

    def get_logger(self):
        return self._logger


class _Float32:
    """테스트용 Float32 (모듈 전역을 이걸로 덮어쓴다)."""

    def __init__(self, data=0.0):
        self.data = data


def _qos_cls(n):
    return type(n, (), {'__init__': lambda self, **k: None,
                        'BEST_EFFORT': 1, 'RELIABLE': 2, 'KEEP_LAST': 1})


def _load_node_module():
    mods = {
        'rclpy': dict(init=lambda *a, **k: None, ok=lambda: True, shutdown=lambda: None),
        'rclpy.node': dict(Node=_Node),
        'rclpy.executors': dict(MultiThreadedExecutor=object),
        'rclpy.qos': {n: _qos_cls(n) for n in
                      ('QoSProfile', 'ReliabilityPolicy', 'HistoryPolicy')},
        'std_msgs': {},
        'std_msgs.msg': dict(Float32=_Float32, Int32=_Float32),
        'sensor_msgs': {},
        'sensor_msgs.msg': dict(LaserScan=object),
    }
    saved = {k: sys.modules.get(k) for k in mods}
    try:
        for name, attrs in mods.items():
            m = types.ModuleType(name)
            for k, v in attrs.items():
                setattr(m, k, v)
            sys.modules[name] = m
        spec = importlib.util.spec_from_file_location(
            'ship_dock_steer_clamp_under_test', os.path.join(PKG, 'ship_dock', 'ship_dock.py'))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return mod


SD = _load_node_module()
SD.Float32 = _Float32


class _Clock:
    """ship_dock 모듈의 time 대용. monotonic() 만 쓴다."""

    def __init__(self, t=1000.0):
        self.t = t

    def monotonic(self):
        return self.t


CLOCK = _Clock()
SD.time = CLOCK

L = 80.0
SPECIAL = (SD.SPIN_RIGHT, SD.SPIN_LEFT, SD.CANDIDATE_INVALID, SD.STOP_HOLD, SD.REVERSE_REL)


class _Scan:
    """LaserScan 흉내. 0~359°(LiDAR 프레임, 80 = 정면) 1° 간격, 전부 같은 거리."""

    def __init__(self, dist):
        self.angle_min = math.radians(0.0)
        self.angle_increment = math.radians(1.0)
        self.ranges = [float(dist)] * 360


def make_node(state, image_angle, dist, standoff_aligned=False):
    """state 에서 한 틱 돌릴 준비가 된 노드. 표식·스캔은 방금 들어온 것(신선)."""
    n = SD.ShipDock()
    now = CLOCK.t
    n.wp_mode = n.active_wp_mode
    n.state = state
    n.state_t = now
    n.standoff_aligned = standoff_aligned
    n.image_angle = float(image_angle)
    n.last_image_t = now
    n.latest_scan = _Scan(dist)
    n.last_scan_t = now
    return n


def tick(state, image_angle, dist, standoff_aligned=False):
    """한 틱 → (노드, 발행값 목록)."""
    n = make_node(state, image_angle, dist, standoff_aligned)
    n.timer_cb()
    return n, n.pub_candidate.values


def warns(n):
    return [ln for ln in n.get_logger().lines if ln[0] == 'WARN']


# ---------------- APPROACH: realign_min 안(1.2 m) → STEER, 조향만 클램프
def test_constant_is_80():
    assert SD.STEER_LIMIT_DEG == 80.0


def test_approach_180_clamped_to_plus80():
    """180 → 80.0. 수정 전에는 180.0 이 그대로 나가 ship_direction 이 후진(260)으로 받았다."""
    _, out = tick('APPROACH', 180.0, 1.2)
    assert out == [80.0], f"out={out}"


def test_approach_minus180_clamped_to_minus80():
    _, out = tick('APPROACH', -180.0, 1.2)
    assert out == [-80.0], f"out={out}"


def test_approach_just_inside_and_outside():
    for a in (179.9, 81.0):
        _, out = tick('APPROACH', a, 1.2)
        assert out == [80.0], f"{a} → {out}"


def test_approach_within_limit_unchanged():
    for a in (80.0, -80.0, 30.0, -30.0):
        _, out = tick('APPROACH', a, 1.2)
        assert out == [a], f"{a} → {out}"


def test_approach_outside_realign_floor_aligned_20():
    """dist 2.0(하한 밖, standoff 안), standoff 재정렬 끝남, 20 < realign_tol 25 → STEER 20."""
    n, out = tick('APPROACH', 20.0, 2.0, standoff_aligned=True)
    assert out == [20.0], f"out={out}"
    assert n.state == 'APPROACH'


def test_approach_decision_unchanged_regression():
    """판단 결과 불변 확인(회귀 방지): dist 2.0 에서 100° → realign_tol 초과 → ALIGN, 발행 없음.

    ±80 클램프는 realign_tol(25) 등 판단 임계를 넘지 않으므로 이 테스트로는
    '클램프 전 값 사용' 여부를 구별하지 못한다 — 판단 결과가 안 바뀌었는지만 본다.
    """
    n, out = tick('APPROACH', 100.0, 2.0, standoff_aligned=True)
    assert n.state == 'ALIGN' and out == [], f"state={n.state} out={out}"


def test_warn_only_when_clamped():
    n, _ = tick('APPROACH', 180.0, 1.2)
    w = warns(n)
    assert len(w) == 1 and w[0][2].get('throttle_duration_sec') == 1.0, f"warns={w}"
    for a in (80.0, -80.0, 30.0):
        n, _ = tick('APPROACH', a, 1.2)
        assert warns(n) == [], f"{a}: warns={warns(n)}"


# ---------------- 다른 상태는 그대로
def test_backoff_exact_180():
    _, out = tick('BACKOFF', 0.0, 1.2)
    assert out == [180.0], f"out={out}"


def test_search_spin_5000():
    n = make_node('SEARCH', SD.IMAGE_INVALID, 5.0)
    n.timer_cb()
    assert n.pub_candidate.values == [5000.0], f"out={n.pub_candidate.values}"


def test_align_spin_unchanged_regression():
    """판단 결과 불변 확인(회귀 방지): ALIGN 에서 180 은 좌(양수) → 6000.

    클램프해도 부호가 같아(+80) 선회 방향이 안 바뀌므로 '클램프 전 값 사용' 여부는 구별하지 못한다.
    """
    _, out = tick('ALIGN', 180.0, 5.0)
    assert out == [SD.SPIN_LEFT], f"out={out}"


def test_contact_stop_and_done_gps():
    """CONTACT 는 정지 50000. DONE 은 원인별(계약서_전체코스_20261104 C4 · §10-추가):
    후진 이탈 완료 20000(GPS 가 다음 WP 로), SEARCH 타임아웃·후진 타임아웃·원인 모름 50000."""
    _, out = tick('CONTACT', 0.0, 1.2)
    assert out == [50000.0], f"CONTACT → {out}"
    for reason, want in (('backoff', 20000.0), ('search_timeout', 50000.0),
                         ('backoff_timeout', 50000.0), (None, 50000.0)):
        n = make_node('DONE', 0.0, 1.2)
        n.done_reason = reason
        n.timer_cb()
        assert n.pub_candidate.values == [want], f"DONE({reason}) → {n.pub_candidate.values}"


def test_nan_not_published():
    """NaN 은 상류(marker_ok)에서 걸러져 APPROACH → SEARCH, 이 틱 발행 없음. 가드 유지 확인."""
    n, out = tick('APPROACH', float('nan'), 1.2)
    assert out == [] and n.state == 'SEARCH', f"state={n.state} out={out}"


def test_all_steer_within_limit_and_never_180():
    vals = [x * 0.5 for x in range(-720, 721)]       # −360 ~ 360, 0.5° 간격
    for a in vals:
        _, out = tick('APPROACH', a, 1.2)
        assert len(out) == 1, f"{a} → {out}"
        p = out[0]
        if p in SPECIAL:
            assert p != 180.0, f"{a} → 180.0(후진)"
            continue
        assert abs(p) <= L, f"{a} → {p}"


if __name__ == '__main__':
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith('test_') and callable(v)]
    fails = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  ✅ {name}")
        except AssertionError as e:
            fails += 1
            print(f"  ❌ {name}\n     {e}")
        except AttributeError as e:
            fails += 1
            print(f"  ❌ {name}\n     {e!r}")
    print(f"{len(tests) - fails}/{len(tests)} 통과")
    sys.exit(1 if fails else 0)

#!/usr/bin/env python3
"""ship_back HOLD forward 조향 ±80° 클램프 테스트 — 실제 ShipBack.timer_cb 를 스텁 노드로 돌린다.

실행: python3 src/ship_back/test/test_back_steer_clamp.py
      tools/run_tests.sh ship_back

계약서 docs/문제와작업/계약서_각도클램프_20261104.md §1 B2 · §6 · §10.
규약: /candidate_angle 180.0 정확 일치 = 후진(ship_direction (B) 예외). HOLD forward 조향이 우연히 180.0 이
  되지 않도록 발행 직전에만 rel_to_raw(clamp(rel, −80, 80)) (ship_turn 과 같은 순서).
  buoy_rel(clamp_pm180)·_dist_at 의 LiDAR 매칭 방위는 클램프 전 값. 후진 180.0·STOP 50000 그대로.

ship_back.py 는 rclpy 를 import 하므로 Mac(ROS 없음)에선 그대로 못 읽는다.
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
sys.path.insert(0, PKG)    # ship_back.py 의 `from ship_back.hold_logic import …` 용


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
    """파라미터는 코드 기본값(= config/ship_back.yaml 과 같게 유지되는 값)."""

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
            'ship_back_steer_clamp_under_test', os.path.join(PKG, 'ship_back', 'ship_back.py'))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return mod


SB = _load_node_module()
SB.Float32 = _Float32


class _Clock:
    """ship_back 모듈의 time 대용. monotonic() 만 쓴다."""

    def __init__(self, t=1000.0):
        self.t = t

    def monotonic(self):
        return self.t


CLOCK = _Clock()
SB.time = CLOCK

L = 80.0
# 기본 스캔은 실제보다 넓다(−200 ~ 599°). 실제 LiDAR(−180 ~ 179°, 1° × 360)는 _dist_at 이 각을 감지
# 않으므로 target = 80 + rel 이 179 를 넘는 rel > 약 100(좌현) 쪽만 거리를 못 찾는다(→ STOP_HOLD,
# 기존 동작). 우현 쪽(rel −180 → target −100)은 실제 스캔에서도 잡힌다. 넓힌 스캔은 rel > 100 입력의
# 클램프 경로를 보려는 것뿐이다 — 실제 기하 케이스는 REAL_* 로 따로 둔다.
A_MIN = -200.0       # 넓힌 스캔 시작각(LiDAR 프레임)
N_BEAMS = 800        # 1° 간격 → −200 ~ 599°
REAL_A_MIN = -180.0  # 실제 LiDAR 기하: angle_min −180°
REAL_N_BEAMS = 360   # 1° 간격 → −180 ~ 179°


class _Scan:
    """LaserScan 흉내(1° 간격). obj=None 이면 전 방위 dist, 아니면 LiDAR 각 obj 한 빔만 dist."""

    def __init__(self, dist, obj=None, a_min=A_MIN, n_beams=N_BEAMS):
        self.angle_min = math.radians(a_min)
        self.angle_increment = math.radians(1.0)
        if obj is None:
            self.ranges = [float(dist)] * n_beams
        else:
            self.ranges = [float('inf')] * n_beams
            self.ranges[int(round(obj - a_min))] = float(dist)


def make_hold(image_angle, dist=3.5, obj=None, scan=None):
    """HOLD 에서 한 틱 돌릴 준비가 된 노드. image_angle=None 이면 비전 방위 없음(LiDAR 대체).

    이미지 각은 실제 콜백(angle_cb)으로 넣는다 — clamp_pm180·NaN 가드를 그대로 거친다.
    """
    n = SB.ShipBack()
    n.wp_mode = n.active_wp_modes[0]
    n.state = 'HOLD'
    n.state_t = CLOCK.t
    n.last_seen_t = CLOCK.t
    if image_angle is not None:
        n.angle_cb(_Float32(float(image_angle)))
    n.latest_scan = scan if scan is not None else _Scan(dist, obj)
    n.t_scan = CLOCK.t
    return n


def tick(image_angle, dist=3.5, obj=None, scan=None):
    n = make_hold(image_angle, dist, obj, scan)
    n.timer_cb()
    return n, n.pub_candidate.values


def warns(n):
    return [ln for ln in n.get_logger().lines if ln[0] == 'WARN']


def _one(out):
    assert len(out) == 1, f"out={out}"
    return out[0]


# ---------------- HOLD forward (레인지 3.5 m > hold 2.5 + tol 0.4)
def test_constant_is_80():
    assert SB.STEER_LIMIT_DEG == 80.0


def test_forward_180_family_clamped_to_280():
    """180·−180·540 → clamp_pm180 = −180 → −80 → 280.0. 수정 전에는 180.0(후진)이 나갔다."""
    for a in (180.0, -180.0, 540.0):
        _, out = tick(a)
        assert _one(out) == 280.0, f"{a} → {out}"


def test_forward_180_real_lidar_geometry():
    """실제 LiDAR 기하(−180°, 1° × 360)에서 이미지 180 → 280.0.

    clamp_pm180(180) = −180 → target −100 이라 실제 스캔에서도 거리가 잡혀 전진 분기로 간다.
    수정 전에는 rel_to_raw(−180) = 180.0(후진 신호)이 나갔다.
    """
    real = _Scan(3.5, a_min=REAL_A_MIN, n_beams=REAL_N_BEAMS)
    _, out = tick(180.0, scan=real)
    assert _one(out) == 280.0, f"out={out}"


def test_forward_left_over_limit_to_80():
    """179.9·90 → 80.0.

    179.9 는 실제 ±180 스캔에서는 target 259.9 라 거리를 못 찾아 STOP_HOLD(기존 동작).
    여기선 넓힌 스캔으로 클램프 경로만 확인.
    """
    for a in (179.9, 90.0):
        _, out = tick(a)
        assert _one(out) == 80.0, f"{a} → {out}"


def test_forward_right_over_limit_to_280():
    _, out = tick(-90.0)
    assert _one(out) == 280.0, f"out={out}"


def test_forward_within_limit_unchanged():
    for a, want in ((80.0, 80.0), (-80.0, 280.0), (30.0, 30.0), (-30.0, 330.0), (0.0, 0.0)):
        _, out = tick(a)
        assert _one(out) == want, f"{a} → {out} (기대 {want})"


def test_warn_only_when_clamped():
    n, _ = tick(180.0)
    w = warns(n)
    assert len(w) == 1 and w[0][2].get('throttle_duration_sec') == 1.0, f"warns={w}"
    for a in (80.0, -80.0, 30.0):
        n, _ = tick(a)
        assert warns(n) == [], f"{a}: warns={warns(n)}"


# ---------------- 다른 분기는 그대로
def test_reverse_exact_180():
    """dist 1.5 < 2.5 − 0.4 → 후진 180.0 그대로."""
    _, out = tick(0.0, dist=1.5)
    assert _one(out) == 180.0, f"out={out}"


def test_deadband_stop():
    _, out = tick(0.0, dist=2.5)
    assert _one(out) == 50000.0, f"out={out}"


def test_lidar_fallback_pm30_unchanged():
    """비전 방위 없음 → LiDAR 전방 ±30 최근접. +30(LiDAR 110°) → 30, −30(LiDAR 50°) → 330."""
    for obj, want in ((110.0, 30.0), (50.0, 330.0)):
        _, out = tick(None, dist=3.5, obj=obj)
        assert abs(_one(out) - want) < 1e-6, f"LiDAR {obj}° → {out} (기대 {want})"


def test_lidar_match_uses_unclamped_bearing():
    """이미지 90 + 물체 LiDAR 170°(= 80 + 90) → 거리를 찾아 전진, 조향만 80.

    buoy_rel 을 80 으로 잘랐다면 LiDAR 160±3° 를 봐서 거리를 못 찾고 STOP_HOLD 였다.
    """
    _, out = tick(90.0, dist=3.5, obj=170.0)
    assert _one(out) == 80.0, f"out={out}"


def test_nan_guard_kept():
    """NaN 은 angle_cb 에서 걸러져 buoy_rel=None → LiDAR 대체(전 방위 3.5 → 전방 최근접 −30)."""
    n = make_hold(float('nan'))
    assert n.buoy_rel is None
    n.timer_cb()
    p = _one(n.pub_candidate.values)
    assert abs(p - 330.0) < 1e-6, f"out={p} (NaN 이 조향으로 새면 80.0)"


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

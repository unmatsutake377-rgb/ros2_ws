#!/usr/bin/env python3
"""ship_turn FSM 테스트 — 실제 ShipTurn.timer_cb 를 rclpy 스텁 노드로 돌린다.

실행: python3 src/ship_turn/test/test_turn_fsm.py
      tools/run_tests.sh ship_turn

계약서 docs/문제와작업/계약서_탐색orbit_WP넘김_20261111.md (D1):
  A. SEARCH = 20000(손 뗌) — 제자리 선회 SPIN 5000/6000 을 내지 않고, 시간 초과로 상태를 바꾸지 않는다.
  B. 한 바퀴(또는 orbit_max_sec) 뒤 DONE — 모드 3 이 끝날 때까지 20000. 3→3 래치 유지, 3→5→3 은 INIT.
  C. ORBIT 에선 LiDAR 추적이 카메라보다 먼저 — 검출기 grace 의 묵은 시야 끝 각을 이긴다.
     APPROACH 는 카메라 먼저(그대로).
스텁 방식은 ship_back/test/test_back_steer_clamp.py 와 같다(import 동안만 sys.modules 에 넣고 되돌린다).
"""

import importlib.util
import math
import os
import sys
import types

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, PKG)    # ship_turn.py 의 `from ship_turn.orbit_logic import …` 용

PARAM_OVERRIDES = {'enabled': True}   # launch search_behavior:=orbit 이 넘기는 값


class _Param:
    def __init__(self, v):
        self.value = v


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, m, **kw):
        self.lines.append(('INFO', m))

    def warn(self, m, **kw):
        self.lines.append(('WARN', m))

    def error(self, m, **kw):
        self.lines.append(('ERROR', m))


class _Pub:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg.data)


class _Node:
    def __init__(self, name):
        self._logger = _Logger()
        self.pubs = []

    def declare_parameter(self, name, default):
        return _Param(PARAM_OVERRIDES.get(name, default))

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


class _Msg:
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
        'std_msgs.msg': dict(Float32=_Msg, Int32=_Msg, String=_Msg, Float64=_Msg, Float32MultiArray=_Msg),
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
            'ship_turn_fsm_under_test', os.path.join(PKG, 'ship_turn', 'ship_turn.py'))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return mod


ST = _load_node_module()
ST.Float32 = _Msg


class _Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def monotonic(self):
        return self.t


CLOCK = _Clock()
ST.time = CLOCK

FWD = 80.0
INVALID = 20000.0


class _Scan:
    """LaserScan 흉내: −180~179.5°, 0.5° 간격. points = [(rel 좌 양수, dist)]."""

    def __init__(self, points=()):
        self.angle_min = math.radians(-180.0)
        self.angle_increment = math.radians(0.5)
        self.ranges = [math.inf] * 720
        for rel, d in points:
            a = FWD + rel
            i = int(round(((a + 180.0) % 360.0) / 0.5)) % 720
            self.ranges[i] = float(d)


def _node():
    n = ST.ShipTurn()
    n.wp_cb(_Msg(3))
    assert n.state == 'INIT'
    return n


def _pub(n):
    return n.pubs[0].values


def _tick(n, dt=0.1):
    CLOCK.t += dt
    n.timer_cb()


def _orbit_node(cam_rel=None, scan_points=(), last_rel=-31.0):
    """ORBIT 한가운데(CW) 노드. 카메라·스캔은 지금 시각 기준 신선."""
    n = _node()
    n.state = 'ORBIT'
    n.state_t = CLOCK.t
    n.dir_cw = True
    n.last_rel = last_rel
    n.last_seen_t = CLOCK.t
    n.latest_scan = _Scan(scan_points)
    n.t_scan = CLOCK.t + 1.0           # 다음 틱(0.1 s 뒤)에도 신선
    if cam_rel is not None:
        n.angle_cb(_Msg(float(cam_rel)))
        n.t_angle = CLOCK.t + 1.0
    return n


# ─────────────── A. SEARCH = 20000 ───────────────
def test_search_publishes_invalid_not_spin():
    n = _node()
    for _ in range(25):                 # INIT hold_sec 2 s → SEARCH
        _tick(n)
    assert n.state == 'SEARCH', n.state
    del _pub(n)[:]
    for _ in range(600):                # 60 s — 옛 search_timeout_sec 20 s 를 훨씬 넘긴다
        _tick(n)
    vals = set(_pub(n))
    assert vals == {INVALID}, f'SEARCH 발행값 {vals} — 20000 만이어야 한다(SPIN 5000/6000 금지)'
    assert n.state == 'SEARCH', f'시간 초과로 상태가 바뀌었다: {n.state}'


def test_search_with_color_and_angle_goes_approach():
    n = _node()
    for _ in range(25):
        _tick(n)
    n.color_cb(_Msg('yellow'))
    n.angle_cb(_Msg(5.0))
    _tick(n)
    assert n.state == 'APPROACH' and n.dir_cw is False, (n.state, n.dir_cw)


def test_search_with_black_goes_approach_ccw():
    """2026 OT(11-12): 검정 = 좌현선회. 수정 전엔 None 이라 SEARCH 에 머물러 선회 0회."""
    n = _node()
    for _ in range(25):
        _tick(n)
    n.color_cb(_Msg('black'))
    n.angle_cb(_Msg(5.0))
    _tick(n)
    assert n.state == 'APPROACH' and n.dir_cw is False, (n.state, n.dir_cw)


# ─────────────── B. DONE 래치 ───────────────
def _done_node():
    n = _orbit_node(cam_rel=-10.0, scan_points=[(-10.0, 3.0)], last_rel=-10.0)
    n.state_t = CLOCK.t - (n.orbit_max_sec + 1.0)      # 안전 타임아웃 → 완료 처리
    _tick(n)
    return n


def test_orbit_max_sec_goes_done():
    n = _done_node()
    assert n.state == 'DONE', n.state


def test_done_latches_invalid_through_3_to_3():
    n = _done_node()
    del _pub(n)[:]
    for _ in range(300):                # 30 s — 옛 cooldown_sec 5 s 를 넘긴다
        _tick(n)
    n.wp_cb(_Msg(3))                    # 다음 탐색 WP(3→3) — 래치 유지
    for _ in range(50):
        _tick(n)
    assert n.state == 'DONE', n.state
    assert set(_pub(n)) == {INVALID}, set(_pub(n))


def test_done_resets_on_mode_reentry():
    n = _done_node()
    n.wp_cb(_Msg(5))
    n.wp_cb(_Msg(3))
    assert n.state == 'INIT', n.state


def test_progress_complete_goes_done():
    n = _orbit_node(cam_rel=-90.0, scan_points=[(-90.0, 3.0)], last_rel=-90.0)
    n.yaw = 0.0
    n.t_yaw = CLOCK.t + 1.0
    n.progress.update(0.0)
    n.progress.total = n.orbit_target_deg - 1.0     # 1° 남음
    n.progress.prev = 90.0                          # 월드 방위 = yaw − rel = 0 − (−90) = 90
    n.yaw = 5.0                                     # → 95 : +5° → 목표 도달
    _tick(n)
    assert n.state == 'DONE', n.state


# ─────────────── C. ORBIT 방위 소스 ───────────────
def test_orbit_prefers_lidar_over_stale_camera():
    """검출기 grace: 카메라는 시야 끝 −31° 를 반복, 실제 부표는 LiDAR −45° 3 m."""
    n = _orbit_node(cam_rel=-31.0, scan_points=[(-45.0, 3.0)], last_rel=-31.0)
    _tick(n)
    assert n.rel_src == 'lidar', n.rel_src
    assert abs(n.last_rel - (-45.0)) <= 0.5, n.last_rel


def test_orbit_uses_camera_when_lidar_has_nothing():
    n = _orbit_node(cam_rel=-20.0, scan_points=[], last_rel=-20.0)
    _tick(n)
    assert n.rel_src == 'cam', n.rel_src


def test_approach_keeps_camera_first():
    n = _orbit_node(cam_rel=10.0, scan_points=[(-5.0, 3.0)], last_rel=-5.0)
    n.state = 'APPROACH'
    _tick(n)
    assert n.rel_src == 'cam', n.rel_src


if __name__ == '__main__':
    fails = 0
    for k, fn in sorted(globals().items()):
        if k.startswith('test_'):
            try:
                fn()
                print(f'  ✅ {k}')
            except AssertionError as e:
                fails += 1
                print(f'  ❌ {k}: {e}')
    print(f"\n{'PASS' if fails == 0 else str(fails) + ' FAIL'}")
    sys.exit(1 if fails else 0)

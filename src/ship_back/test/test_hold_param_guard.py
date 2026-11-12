#!/usr/bin/env python3
"""ship_back hold_dist_m·dist_tol_m 시작 값 가드 테스트 — ROS 없이 돈다.

실행: python3 src/ship_back/test/test_hold_param_guard.py
      tools/run_tests.sh ship_back

계약서 docs/문제와작업/계약서_옆비킴_현장조정_20261106.md 결정 ⑤:
  hold − tol ≤ 0·비유한·음수 → ERROR + 두 값 기본값(2.5/0.4), hold + tol ≥ keep_radius → WARN(값 유지),
  hold + tol > 2.9(north los_logic.SHIP_BACK_HOLD_OUTER_M) → north 반경 확인 INFO. 코드 기본값·yaml 불변.
노드 쪽은 test_back_steer_clamp.py 와 같은 rclpy 스텁으로 실제 ShipBack.__init__ 을 돈다.
"""

import importlib.util
import math
import os
import sys
import types

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, PKG)

from ship_back.hold_logic import check_hold_params  # noqa: E402
from ship_back.hold_logic import DEFAULT_HOLD_DIST_M, DEFAULT_DIST_TOL_M  # noqa: E402


def _levels(issues):
    return [lv for lv, _ in issues]


# ─────────────── 순수 함수 ───────────────
def test_defaults_no_issue():
    h, t, issues = check_hold_params(2.5, 0.4, 5.0)
    assert (h, t) == (2.5, 0.4) and issues == [], issues


def test_hold_35_north_info_only():
    h, t, issues = check_hold_params(3.5, 0.4, 5.0)
    assert (h, t) == (3.5, 0.4)
    assert _levels(issues) == ['INFO'], issues
    assert 'arrive_radius_hold_modes_m' in issues[0][1] and '3.9' in issues[0][1], issues


def test_outer_ge_keep_warn_value_kept():
    h, t, issues = check_hold_params(4.6, 0.4, 5.0)
    assert (h, t) == (4.6, 0.4), "WARN 은 값을 그대로 둔다(현장 판단)"
    assert 'WARN' in _levels(issues) and 'ERROR' not in _levels(issues), issues
    warn = [m for lv, m in issues if lv == 'WARN'][0]
    assert '5 m ≥ 규정 반경 5 m' in warn and 'HoldTimer' in warn, warn
    # 바깥 끝이 반경 바로 아래면 WARN 없음
    _, _, issues = check_hold_params(4.5, 0.4, 5.0)
    assert 'WARN' not in _levels(issues), issues


def test_hold_minus_tol_le_zero_error_defaults():
    for hold, tol in ((0.3, 0.4), (0.4, 0.4), (0.0, 0.0)):
        h, t, issues = check_hold_params(hold, tol, 5.0)
        assert (h, t) == (DEFAULT_HOLD_DIST_M, DEFAULT_DIST_TOL_M), (hold, tol, h, t)
        assert _levels(issues) == ['ERROR'], (hold, tol, issues)


def test_nonfinite_negative_error_defaults():
    nan, inf = float('nan'), float('inf')
    for hold, tol in ((nan, 0.4), (2.5, nan), (inf, 0.4), (2.5, inf), (-inf, 0.4),
                      (-2.5, 0.4), (2.5, -0.4), ('abc', 0.4), (None, 0.4)):
        h, t, issues = check_hold_params(hold, tol, 5.0)
        assert (h, t) == (2.5, 0.4), (hold, tol, h, t)
        assert _levels(issues) == ['ERROR'], (hold, tol, issues)
        assert math.isfinite(h) and math.isfinite(t)


def test_tol_zero_is_valid():
    """tol 0 은 데드밴드 없음일 뿐 잘못된 값 아님(hold − 0 > 0)."""
    h, t, issues = check_hold_params(2.5, 0.0, 5.0)
    assert (h, t) == (2.5, 0.0) and issues == [], issues


def test_info_threshold_is_default_outer():
    """INFO 기준 = 기본 hold + tol = 2.9 (north SHIP_BACK_HOLD_OUTER_M 과 같은 값)."""
    assert abs(DEFAULT_HOLD_DIST_M + DEFAULT_DIST_TOL_M - 2.9) < 1e-9
    _, _, issues = check_hold_params(2.5, 0.4, 5.0)
    assert 'INFO' not in _levels(issues)
    _, _, issues = check_hold_params(2.51, 0.4, 5.0)
    assert _levels(issues) == ['INFO'], issues


def test_code_defaults_equal_yaml():
    import yaml
    with open(os.path.join(PKG, 'config', 'ship_back.yaml'), encoding='utf-8') as f:
        y = yaml.safe_load(f)['/ship_back']['ros__parameters']
    assert DEFAULT_HOLD_DIST_M == 2.5 and DEFAULT_DIST_TOL_M == 0.4
    assert y['hold_dist_m'] == DEFAULT_HOLD_DIST_M and isinstance(y['hold_dist_m'], float)
    assert y['dist_tol_m'] == DEFAULT_DIST_TOL_M and isinstance(y['dist_tol_m'], float)
    assert y['keep_radius_m'] == 5.0
    _, _, issues = check_hold_params(y['hold_dist_m'], y['dist_tol_m'], y['keep_radius_m'])
    assert issues == [], "저장소 yaml 값으로는 가드 줄이 하나도 안 난다"


# ─────────────── 노드(스텁) ───────────────
OVERRIDE = {}    # 파라미터 이름 → yaml 이 준 것처럼 덮어쓸 값
DECLARED = {}    # 노드가 선언한 (이름 → 코드 기본값)


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


class _Node:
    def __init__(self, name):
        self._logger = _Logger()

    def declare_parameter(self, name, default):
        DECLARED[name] = default
        return _Param(OVERRIDE.get(name, default))

    def create_publisher(self, *a, **k):
        return types.SimpleNamespace(publish=lambda msg: None)

    def create_subscription(self, *a, **k):
        return None

    def create_timer(self, *a, **k):
        return None

    def get_logger(self):
        return self._logger


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
        'std_msgs.msg': dict(Float32=object, Int32=object, Float32MultiArray=object),
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
            'ship_back_hold_guard_under_test', os.path.join(PKG, 'ship_back', 'ship_back.py'))
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


def _make(**over):
    OVERRIDE.clear()
    OVERRIDE.update(over)
    try:
        return SB.ShipBack()
    finally:
        OVERRIDE.clear()


def _start_line(n):
    return [m for lv, m in n.get_logger().lines if lv == 'INFO' and '위치유지: hold_dist=' in m]


def test_node_declared_defaults_equal_guard_defaults():
    """declare_parameter 리터럴 = hold_logic.DEFAULT_* (yaml 과는 test_code_defaults_equal_yaml)."""
    _make()
    assert DECLARED['hold_dist_m'] == DEFAULT_HOLD_DIST_M, DECLARED['hold_dist_m']
    assert DECLARED['dist_tol_m'] == DEFAULT_DIST_TOL_M, DECLARED['dist_tol_m']
    assert DECLARED['keep_radius_m'] == 5.0


def test_node_defaults_silent():
    n = _make()
    assert (n.hold_dist_m, n.dist_tol_m) == (2.5, 0.4)
    assert [ln for ln in n.get_logger().lines if ln[0] != 'INFO'] == [], n.get_logger().lines
    assert len(n.get_logger().lines) == 1, n.get_logger().lines   # 시작 INFO 한 줄뿐
    assert 'hold_dist=2.5m' in _start_line(n)[0]


def test_node_bad_value_error_once_defaults():
    n = _make(hold_dist_m=0.3, dist_tol_m=0.4)
    assert (n.hold_dist_m, n.dist_tol_m) == (2.5, 0.4)
    errs = [m for lv, m in n.get_logger().lines if lv == 'ERROR']
    assert len(errs) == 1 and 'hold_dist_m=0.3' in errs[0], n.get_logger().lines
    assert 'hold_dist=2.5m' in _start_line(n)[0] and 'tol 0.4' in _start_line(n)[0], \
        "시작 INFO 는 가드 뒤 값"


def test_node_nan_error_defaults():
    n = _make(hold_dist_m=float('nan'))
    assert (n.hold_dist_m, n.dist_tol_m) == (2.5, 0.4)
    assert [lv for lv, _ in n.get_logger().lines].count('ERROR') == 1


def test_node_outer_ge_keep_warn_once_value_kept():
    n = _make(hold_dist_m=4.6, dist_tol_m=0.4)
    assert (n.hold_dist_m, n.dist_tol_m) == (4.6, 0.4)
    lv = [x for x, _ in n.get_logger().lines]
    assert lv.count('WARN') == 1 and lv.count('ERROR') == 0, n.get_logger().lines
    assert 'hold_dist=4.6m' in _start_line(n)[0]


def test_node_hold_35_info_once():
    n = _make(hold_dist_m=3.5)
    assert n.hold_dist_m == 3.5
    lines = n.get_logger().lines
    assert [x for x, _ in lines] == ['INFO', 'INFO'], lines
    assert sum('arrive_radius_hold_modes_m' in m for _, m in lines) == 1, lines
    assert 'hold_dist=3.5m' in _start_line(n)[0]


def test_node_guard_before_hold_logic_use():
    """가드 줄이 시작 INFO 보다 먼저 나온다(가드 뒤 값으로 시작 줄을 찍는다)."""
    n = _make(hold_dist_m=-1.0)
    lines = n.get_logger().lines
    assert lines[0][0] == 'ERROR' and '위치유지: hold_dist=' in lines[-1][1], lines


if __name__ == '__main__':
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print(f"  ok  {name}")
            except AssertionError as e:
                fails += 1
                print(f"  FAIL {name}: {e}")
    sys.exit(1 if fails else 0)

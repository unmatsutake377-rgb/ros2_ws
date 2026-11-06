#!/usr/bin/env python3
"""motor_control 목표(WP) 근접 감속 테스트 (계약서_목표근접감속_20261106 §검증 ①~⑮).
실행: python3 src/motor_control/test/test_motor_goal_slow.py  (또는 tools/run_tests.sh motor_control)

고정하는 규약 (A-2): /goal_distance 가 가까우면 **전진(2,4,6) 중심 PWM 만** 1500 쪽으로 당기고
조향 차동(pwm_l − pwm_r)은 유지한다. 그 뒤 기존 min(근접비, L1) 균일 축소. 미수신·stale → 1.0.
SPIN·후진·STOP·워치독 불변. /motor_reverse 는 좌우 평균 판정 → 전진 감속 중 False.
🚨 균일 축소(apply_slow 로 좌·우 같이 깎기)로 구현하면 ⑪(차동 보존)이 깨진다.
🚨 stale 을 안 보면 ⑤ 가, 후진에도 걸면 ⑥ 이 깨진다.

스텁 방식은 test_motor_l1_reverse 와 같다(import 동안만 rclpy·std_msgs 스텁, 바로 복원).
"""

import importlib.util
import math
import os
import sys
import time
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, '..', 'motor_control', 'motor_control.py')

# 기본값 (motor_control.py declare_parameter 리터럴 = motor_control.yaml)
BASE = 1360          # base_pwm → 전진 성분 140
START, FULL, MINR = 8.0, 3.0, 0.5


class _Msg:
    def __init__(self, data=None):
        self.data = data


class _Pub:
    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


class _Logger:
    def __init__(self):
        self.lines = []

    def warn(self, msg, **kw):
        self.lines.append(msg)

    info = error = warning = warn


class _Node:
    PARAMS = {}

    def __init__(self, name):
        self._logger = _Logger()
        self.pubs = {}
        self.subs = {}

    def declare_parameter(self, name, default=None):
        return types.SimpleNamespace(value=_Node.PARAMS.get(name, default))

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = (cb, qos)
        return None

    def create_publisher(self, typ, topic, qos):
        p = _Pub()
        self.pubs[topic] = p
        return p

    def create_timer(self, *a, **k):
        return None

    def get_logger(self):
        return self._logger


def _load():
    names = ('rclpy', 'rclpy.node', 'std_msgs', 'std_msgs.msg')
    saved = {n: sys.modules.get(n) for n in names}
    rclpy = types.ModuleType('rclpy')
    rclpy.init = rclpy.shutdown = rclpy.spin = lambda *a, **k: None
    rclpy.ok = lambda: True
    node = types.ModuleType('rclpy.node')
    node.Node = _Node
    std = types.ModuleType('std_msgs')
    stdm = types.ModuleType('std_msgs.msg')
    for n in ('Bool', 'Float32', 'Float32MultiArray', 'Int32'):
        setattr(stdm, n, _Msg)
    std.msg = stdm
    sys.modules.update({'rclpy': rclpy, 'rclpy.node': node, 'std_msgs': std, 'std_msgs.msg': stdm})
    try:
        spec = importlib.util.spec_from_file_location('motor_control_goal_under_test', _SRC)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for n, m in saved.items():
            if m is None:
                sys.modules.pop(n, None)
            else:
                sys.modules[n] = m
    mod.Int32 = mod.Bool = mod.Float32 = mod.Float32MultiArray = _Msg
    return mod


MC = _load()


def make(**params):
    _Node.PARAMS = dict(params)
    try:
        return MC.MotorController()
    finally:
        _Node.PARAMS = {}


def step(mc, angle, level=0, nearest=None, cmd_age=None, goal=None, goal_age=None):
    """한 틱. goal 을 주면 /goal_distance 수신(goal_age 초 전에 받은 것으로 묵힘).
    → (pwm_r, pwm_l, /motor_reverse bool)"""
    mc.failsafe_callback(_Msg(level))
    if nearest is not None:
        mc.obstacle_callback(_Msg([nearest, 80.0]))
    if goal is not None:
        mc.goal_distance_cb(_Msg(float(goal)))
        if goal_age is not None and mc.goal_dist_t is not None:
            mc.goal_dist_t = time.monotonic() - goal_age
    mc.angle_callback(_Msg(float(angle)))
    if cmd_age is not None:
        mc.last_cmd_t = time.monotonic() - cmd_age
    mc.timer_callback()
    code = mc.pubs['Motor_run'].last.data
    return code // 10000, code % 10000, mc.pubs['/motor_reverse'].last.data


def center(d):
    """기대 중심 PWM: 1500 − 140 × 비율(d)."""
    if d >= START:
        r = 1.0
    elif d <= FULL:
        r = MINR
    else:
        r = MINR + (1.0 - MINR) * (d - FULL) / (START - FULL)
    return int(round(1500 - (1500 - BASE) * r))


# ① 미수신 → 1360/1360 (기능 없을 때와 동일), /motor_reverse False
def test_01_no_goal_distance_unchanged():
    assert step(make(), 80.0) == (BASE, BASE, False)
    assert step(make(), 20000.0) == (BASE, BASE, False)


# ② d ≥ start(8 m) → 1.0 → 1360
def test_02_far_goal_full_speed():
    for d in (8.0, 8.5, 20.0, 100.0):
        assert step(make(), 80.0, goal=d) == (BASE, BASE, False), d


# ③ 중간 선형: 5.5 m → 0.75 → 1395, 4.0 m → 0.6 → 1416, 7.0 m → 0.9 → 1374
def test_03_linear_between():
    for d, exp in ((5.5, 1395), (4.0, 1416), (7.0, 1374), (3.5, 1423)):
        assert center(d) == exp, (d, center(d))
        r, l, _ = step(make(), 80.0, goal=d)
        assert (r, l) == (exp, exp), f"d={d}: {r}/{l}, 기대 {exp}"


# ④ d=0 → 하한 0.5 → 중심 1430
def test_04_zero_distance_floor():
    assert step(make(), 80.0, goal=0.0) == (1430, 1430, False)
    assert step(make(), 20000.0, goal=0.0) == (1430, 1430, False)   # (2) 폴백 전진도


# ⑤ stale: d=0 을 3.0 s 전에 받음 → 1.0 → 1360 (2.0 s 넘으면 해제)
def test_05_stale_goal_distance_releases():
    assert step(make(), 80.0, goal=0.0, goal_age=3.0) == (BASE, BASE, False)
    assert step(make(), 80.0, goal=0.0, goal_age=1.0) == (1430, 1430, False)   # 아직 신선
    # 파라미터로 stale 시간을 늘리면 그만큼 유지
    assert step(make(goal_dist_stale_sec=5.0), 80.0, goal=0.0, goal_age=3.0) == (1430, 1430, False)


# ⑥ 후진 260 + d=0 → 1590 불변, /motor_reverse True (L1 이면 1563 그대로)
def test_06_reverse_unchanged():
    assert step(make(), 260.0, goal=0.0) == (1590, 1590, True)
    assert step(make(), 260.0, level=1, goal=0.0) == (1563, 1563, True)


# ⑦ SPIN + d=0 → 1600/1400, 1400/1600 불변
def test_07_spin_unchanged():
    assert step(make(), 5000.0, goal=0.0) == (1600, 1400, False)
    assert step(make(), 6000.0, goal=0.0) == (1400, 1600, False)


# ⑧ STOP + d=0 → 1500/1500
def test_08_stop_unchanged():
    assert step(make(), 50000.0, goal=0.0) == (1500, 1500, False)


# ⑨ 근접 0.5 m 합성: 중심 1430 → ×(0.7+0.3×0.5/1.2=0.825) → 1500−70×0.825=1442
def test_09_compose_with_obstacle_slow():
    assert step(make(), 80.0, goal=0.0, nearest=0.5) == (1442, 1442, False)
    # 근접만(기존 동작): 1500−140×0.825 = 1384.5 → 1384
    assert step(make(), 80.0, nearest=0.5) == (1384, 1384, False)


# ⑩ L1 합성: 중심 1430 → ×0.7 → 1451 (중심 축소 뒤 균일 축소)
def test_10_compose_with_l1():
    assert step(make(), 80.0, level=1, goal=0.0) == (1451, 1451, False)
    assert step(make(), 80.0, level=1) == (1402, 1402, False)   # 기존 L1 전진 불변


# ⑪ 조향 차동 보존 (A-2): pwm_l − pwm_r 이 감속 전후 같다. 안쪽 추진기는 1500 을 넘어도 됨
def test_11_steer_differential_preserved():
    for ang in (161.0, -1.0, 120.0, 40.0, 80.0):
        r0, l0, _ = step(make(), ang)
        r1, l1, _ = step(make(), ang, goal=0.0)
        assert l1 - r1 == l0 - r0, f"angle {ang}: 차동 {l0 - r0} → {l1 - r1} (균일 축소로 구현했나?)"
        assert (r1 + l1) == 2 * 1430, f"angle {ang}: 중심 {(r1 + l1) / 2} ≠ 1430"
    assert step(make(), 161.0, goal=0.0)[:2] == (1330, 1530)   # 최대 좌조향: 1430∓100
    assert step(make(), -1.0, goal=0.0)[:2] == (1530, 1330)
    assert step(make(), 161.0)[:2] == (1260, 1460)             # 감속 없음(기존)


# ⑫ 전진 감속 중 /motor_reverse False (좌우 평균 1430 < 1500 — 한쪽이 1530 이어도)
def test_12_motor_reverse_false_while_slowing():
    for ang in (161.0, -1.0, 80.0, 20000.0):
        assert step(make(), ang, goal=0.0)[2] is False, ang


# ⑬ 경계: d=8.0 → 1360, d=3.0 → 1430, 바로 안쪽/바깥쪽
def test_13_boundaries():
    assert step(make(), 80.0, goal=8.0)[:2] == (BASE, BASE)
    assert step(make(), 80.0, goal=3.0)[:2] == (1430, 1430)
    assert step(make(), 80.0, goal=2.9)[:2] == (1430, 1430)
    r, _, _ = step(make(), 80.0, goal=3.1)
    assert 1428 <= r <= 1429, r                                   # 0.51 → 1428.6
    r, _, _ = step(make(), 80.0, goal=7.9)
    assert r == 1361, r                                           # 0.99 → 1361.4
    # 파라미터를 바꾸면 그 값으로 (start 6, full 2, 하한 0.8)
    mc = make(goal_slow_start_m=6.0, goal_slow_full_m=2.0, goal_slow_min_ratio=0.8)
    assert step(mc, 80.0, goal=6.0)[:2] == (BASE, BASE)
    assert step(mc, 80.0, goal=2.0)[:2] == (1388, 1388)           # 1500−140×0.8
    assert step(mc, 80.0, goal=4.0)[:2] == (1374, 1374)           # 0.9


# ⑭ 비유한·음수 무시: 값 갱신 없음(직전 값 유지), WARN 한 줄
def test_14_invalid_values_ignored():
    mc = make()
    for bad in (float('nan'), float('inf'), -float('inf'), -1.0):
        assert step(mc, 80.0, goal=bad)[:2] == (BASE, BASE), bad
        assert mc.goal_dist is None
    assert any('/goal_distance 무시' in ln for ln in mc.get_logger().lines)
    # 유효값 뒤 잘못된 값 → 직전 값 유지(덮어쓰지 않음)
    mc = make()
    assert step(mc, 80.0, goal=0.0)[:2] == (1430, 1430)
    assert step(mc, 80.0, goal=float('nan'))[:2] == (1430, 1430)
    assert mc.goal_dist == 0.0


# ⑮ 워치독 우선: d=0 이어도 /desired_angle 0.6 s 묵음 → 1500/1500
def test_15_watchdog_first():
    assert step(make(), 80.0, goal=0.0, cmd_age=0.6) == (1500, 1500, False)
    mc = make()
    mc.goal_distance_cb(_Msg(0.0))
    mc.timer_callback()                                           # 첫 명령 전
    assert mc.pubs['Motor_run'].last.data == 15001500


# 보조 A: 파라미터 가드 — 잘못된 값이면 ERROR 로그 + 기본값, 노드는 산다
def test_16_param_guard():
    mc = make(goal_slow_start_m=2.0, goal_slow_full_m=3.0)      # start ≤ full
    assert (mc.goal_slow_start_m, mc.goal_slow_full_m) == (8.0, 3.0)
    assert any('goal_slow_start_m' in ln for ln in mc.get_logger().lines)
    mc = make(goal_slow_min_ratio=1.5)
    assert mc.goal_slow_min_ratio == 0.5
    mc = make(goal_slow_min_ratio=-0.1)
    assert mc.goal_slow_min_ratio == 0.5
    mc = make(goal_slow_min_ratio=0.0)                           # 0 금지(전진 0 → 도착 반경 못 듦)
    assert mc.goal_slow_min_ratio == 0.5
    assert any('goal_slow_min_ratio' in ln for ln in mc.get_logger().lines)
    assert step(mc, 80.0, goal=0.0)[:2] == (1430, 1430)
    mc = make(goal_dist_stale_sec=0.0)
    assert mc.goal_dist_stale_sec == 2.0
    mc = make(goal_slow_start_m=float('nan'))
    assert mc.goal_slow_start_m == 8.0
    mc = make(goal_slow_full_m=-1.0)
    assert mc.goal_slow_full_m == 3.0
    assert step(mc, 80.0, goal=0.0)[:2] == (1430, 1430)
    assert not any('이상' in ln for ln in make().get_logger().lines)      # 기본값엔 ERROR 없음


# 보조 B: 끄는 노브 — goal_slow_min_ratio 1.0 이면 d=0 에서도 1360
def test_17_min_ratio_one_disables():
    assert step(make(goal_slow_min_ratio=1.0), 80.0, goal=0.0) == (BASE, BASE, False)


# 보조 C: 구독 토픽·콜백 이름 (full_course_sim 이 mc.goal_distance_cb 로 배선한다)
def test_18_subscription_wiring():
    mc = make()
    cb, qos = mc.subs['/goal_distance']
    assert cb == mc.goal_distance_cb and qos == 2
    assert math.isclose(mc.goal_ratio(time.monotonic()), 1.0)


def main():
    p = t = 0
    for n, f in sorted(globals().items()):
        if n.startswith('test_') and callable(f):
            t += 1
            try:
                f()
                p += 1
                print(f'  ok   {n}')
            except AssertionError as e:
                print(f'  FAIL {n}: {e}')
    print(f'\n{p}/{t} 통과')
    return 0 if p == t else 1


if __name__ == '__main__':
    sys.exit(main())

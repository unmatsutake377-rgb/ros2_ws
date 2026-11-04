#!/usr/bin/env python3
"""motor_control 분기 (5) 후진의 페일세이프 L1 상한 테스트 (계약서_후진감속_L1_20261104 §7 ①~⑫).
실행: python3 src/motor_control/test/test_motor_l1_reverse.py  (또는 tools/run_tests.sh motor_control)

고정하는 규약: **L1 상한 = 전진·후진 공통, 근접 감속 = 전진만, SPIN·STOP·워치독 불변.**
🚨 분기 (5) 에 slow_ok = True 를 넣는 구현이면 ④(L0 + 근접 0.5 m → 1590)가 깨진다.

motor_control.py 는 rclpy 를 import 하므로 Mac(ROS 없음)에선 그대로 못 읽는다.
→ rclpy·std_msgs 최소 스텁을 **import 하는 동안만** sys.modules 에 넣고 바로 복원한다
  (다른 테스트 파일의 스텁과 섞이지 않게). import 뒤 모듈 전역 Int32·Bool·Float32·
  Float32MultiArray 를 .data 대입이 되는 테스트용 클래스로 덮어, 어떤 스텁이 먼저
  등록됐든(실행 순서) 결과가 같게 한다.
"""

import importlib.util
import os
import sys
import time
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, '..', 'motor_control', 'motor_control.py')


class _Msg:
    """std_msgs 대용 — .data 만 있으면 된다."""

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
    """class MotorController(Node) 상속용. PARAMS 로 파라미터를 바꿔 끼운다."""
    PARAMS = {}

    def __init__(self, name):
        self._logger = _Logger()
        self.pubs = {}

    def declare_parameter(self, name, default=None):
        return types.SimpleNamespace(value=_Node.PARAMS.get(name, default))

    def create_subscription(self, *a, **k):
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
        spec = importlib.util.spec_from_file_location('motor_control_l1_under_test', _SRC)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for n, m in saved.items():
            if m is None:
                sys.modules.pop(n, None)
            else:
                sys.modules[n] = m
    # 실행 순서 의존 제거: 모듈 전역 메시지 클래스를 이 파일 것으로 고정
    mod.Int32 = mod.Bool = mod.Float32 = mod.Float32MultiArray = _Msg
    return mod


MC = _load()


def make(**params):
    """MotorController 생성. params 가 없으면 코드 기본값(= 대회값, launch 가 yaml 외엔 안 넘김)."""
    _Node.PARAMS = dict(params)
    try:
        return MC.MotorController()
    finally:
        _Node.PARAMS = {}


def step(mc, angle, level=0, nearest=None, cmd_age=None):
    """한 틱: /failsafe_level·/obstacle_distance_array·/desired_angle 을 넣고 timer_callback.
    매 호출 전 last_cmd_t 를 갱신한다(워치독 0.5 s 에 안 걸리게). cmd_age 를 주면 그만큼 묵힌다.
    → (Motor_run 정수, /motor_reverse bool)"""
    mc.failsafe_callback(_Msg(level))
    if nearest is not None:
        mc.obstacle_callback(_Msg([nearest, 80.0]))
    mc.angle_callback(_Msg(float(angle)))
    if cmd_age is not None:
        mc.last_cmd_t = time.monotonic() - cmd_age
    mc.timer_callback()
    return mc.pubs['Motor_run'].last.data, mc.pubs['/motor_reverse'].last.data


# ① L0 후진 → 1590 그대로, /motor_reverse True
def test_01_l0_reverse_full():
    assert step(make(), 260.0, level=0) == (15901590, True)


# ② L1 후진 → 1500 + 90×0.7 = 1563 (ssf_bridge 시리얼 1437)
def test_02_l1_reverse_capped():
    code, rev = step(make(), 260.0, level=1)
    assert code == 15631563, f"Motor_run={code}, 기대 15631563 (수정 전 = 상한 미적용 15901590)"
    assert rev is True


# ③ L1 + 근접 0.5 m → 1563 (근접 감속이 겹쳐 더 깎이면 안 됨, 이중 적용이면 1544)
def test_03_l1_reverse_with_obstacle_not_doubled():
    code, _ = step(make(), 260.0, level=1, nearest=0.5)
    assert code == 15631563, code


# ④ L0 + 근접 0.5 m → 1590 (근접 감속은 전진만. slow_ok=True 식 구현이면 1574)
def test_04_l0_reverse_obstacle_no_proximity_slow():
    code, _ = step(make(), 260.0, level=0, nearest=0.5)
    assert code == 15901590, f"Motor_run={code} — 근접 감속이 후진에 걸렸다(slow_ok 게이트 확인)"


# ⑤ L2 후진 → 1590 (L2 는 ship_direction 이 STOP_HOLD 로 처리, 여기 상한 아님)
def test_05_l2_reverse_unchanged():
    assert step(make(), 260.0, level=2)[0] == 15901590


# ⑥ L1 전진 정면 → 1500 − 140×0.7 = 1402 (기존 동작 불변)
def test_06_l1_forward_unchanged():
    assert step(make(), 80.0, level=1)[0] == 14021402


# ⑦ L1 SPIN → 1600/1400 (SPIN 은 L1 에서도 100%)
def test_07_l1_spin_unchanged():
    code, rev = step(make(), 5000.0, level=1)
    assert code == 16001400 and rev is False, (code, rev)
    assert step(make(), 6000.0, level=1)[0] == 14001600


# ⑧ L1 STOP_HOLD → 1500/1500
def test_08_l1_stop_unchanged():
    assert step(make(), 50000.0, level=1) == (15001500, False)


# ⑨ L1 후진 명령이 0.6 s 묵음 → 워치독 중립 1500/1500, /motor_reverse False
def test_09_l1_reverse_stale_watchdog():
    assert step(make(), 260.0, level=1, cmd_age=0.6) == (15001500, False)


# ⑩ 분기 경계: 161.0 = 조향(4), 161.1·4999 = 후진(5)
def test_10_branch_boundary():
    code, rev = step(make(), 161.0, level=0)
    assert code == 12601460 and rev is False, code          # 최대 좌조향, 전진
    for a in (161.1, 4999.0):
        assert step(make(), a, level=0)[0] == 15901590, a
        assert step(make(), a, level=1)[0] == 15631563, a


# ⑪ failsafe_l1_speed 1.0 → L1 후진도 1590 (끄는 노브가 후진에도 듣는다)
def test_11_l1_speed_one_disables():
    assert step(make(failsafe_l1_speed=1.0), 260.0, level=1)[0] == 15901590


# ⑫ reverse_pwm 1600 → L1 1500 + 100×0.7 = 1570 (reverse_pwm 파라미터 기준)
def test_12_reverse_pwm_param():
    assert step(make(reverse_pwm=1600), 260.0, level=1)[0] == 15701570
    assert step(make(reverse_pwm=1600), 260.0, level=0)[0] == 16001600


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

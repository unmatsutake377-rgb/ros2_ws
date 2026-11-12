#!/usr/bin/env python3
"""motor_control 분기 (5) 후진 좌우 트림 reverse_trim_us 테스트 (계약서_후진트림_20261112 §5).
실행: python3 src/motor_control/test/test_motor_reverse_trim.py  (또는 tools/run_tests.sh motor_control)

고정하는 규약:
  · t > 0 → pwm_r = reverse_pwm + t, pwm_l = reverse_pwm − t (= apply_steer(−t, center=reverse_pwm)).
    후진 중 물리적 시계 방향 보정(좌선회 쏠림 보정 = 우측을 더 후진). steer_invert 면 같이 뒤집힌다.
  · L1 상한은 좌우 **각각** apply_slow → 트림도 0.7배 (t=10 → 1570/1556).
    🚨 `pwm_r = pwm_l = apply_slow(reverse_pwm, cap)` 단일 대입을 남기면 ③ 이 깨진다(L1 중 트림 사라짐).
  · 기본 0 → 지금 출력 그대로. 전진·SPIN·STOP·워치독은 트림과 무관.
  · |t| ≥ 65(ESC 데드밴드 실효 65 µs) 또는 비정수 → ERROR 1회 + 0 사용(노드는 죽이지 않는다).
  · motor_control.yaml 의 reverse_trim_us 는 정수(0.0 이면 rclpy 가 부팅 때 노드를 죽인다).

스텁·make·step 은 test_motor_l1_reverse 것을 파일 경로로 불러 재사용한다(import 동안만 rclpy 스텁).
"""

import importlib.util
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_YAML = os.path.join(_HERE, '..', 'config', 'motor_control.yaml')


def _load_l1():
    path = os.path.join(_HERE, 'test_motor_l1_reverse.py')
    spec = importlib.util.spec_from_file_location('motor_l1_reverse_for_trim', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


L1 = _load_l1()
make, step = L1.make, L1.step


class _SplitLogger:
    """ERROR 를 따로 모으는 로거 (L1 스텁 로거는 심각도를 한 목록에 섞는다)."""

    def __init__(self):
        self.lines = []
        self.errors = []

    def info(self, msg, **kw):
        self.lines.append(msg)

    warn = warning = info

    def error(self, msg, **kw):
        self.errors.append(msg)
        self.lines.append(msg)


def make_logged(**params):
    """make() 와 같고, 노드 로거를 _SplitLogger 로 바꿔 끼운다(생성 중 ERROR 를 잡으려고)."""
    saved = L1._Logger
    L1._Logger = _SplitLogger
    try:
        return make(**params)
    finally:
        L1._Logger = saved


# ① t=0(기본·명시) → 1590/1590, 지금 출력 그대로
def test_01_zero_trim_unchanged():
    assert step(make(), 260.0) == (15901590, True)
    assert step(make(reverse_trim_us=0), 260.0) == (15901590, True)
    assert step(make(reverse_trim_us=0), 260.0, level=1)[0] == 15631563


# ② t=10 → Motor_run 16001580 (pwm_r 1600 = 우측 더 후진), /motor_reverse True
def test_02_trim_plus10():
    code, rev = step(make(reverse_trim_us=10), 260.0)
    assert code == 16001580, f"Motor_run={code}, 기대 16001580 (수정 전 = 트림 무시 15901590)"
    assert rev is True


# ③ t=10 + L1 → 좌우 각각 apply_slow: 1500+100×0.7=1570 / 1500+80×0.7=1556
def test_03_trim_plus10_l1_each_side():
    code, rev = step(make(reverse_trim_us=10), 260.0, level=1)
    assert code == 15701556, f"Motor_run={code} — L1 단일 대입이면 15631563(트림 사라짐)"
    assert rev is True


# ④ t=−10 대칭
def test_04_trim_minus10_symmetric():
    assert step(make(reverse_trim_us=-10), 260.0) == (15801600, True)
    assert step(make(reverse_trim_us=-10), 260.0, level=1)[0] == 15561570


# ⑤ steer_invert + t=10 → 1580/1600 (apply_steer 경유라 같이 뒤집힘)
def test_05_steer_invert_flips_trim():
    assert step(make(reverse_trim_us=10, steer_invert=True), 260.0) == (15801600, True)


# ⑥ 분기 (5) 전 구간(161.1·4999)이 같은 트림, L2 는 상한 없음
def test_06_trim_whole_reverse_range():
    for a in (161.1, 180.0, 260.0, 4999.0):
        assert step(make(reverse_trim_us=10), a)[0] == 16001580, a
    assert step(make(reverse_trim_us=10), 260.0, level=2)[0] == 16001580


# ⑦ 근접 감속은 여전히 후진에 안 걸림 (L0 + 0.5 m → 트림 그대로)
def test_07_no_proximity_slow_with_trim():
    assert step(make(reverse_trim_us=10), 260.0, nearest=0.5)[0] == 16001580


# ⑧ 전진·조향·SPIN·STOP·워치독 은 트림 무관
def test_08_other_branches_ignore_trim():
    mk = make(reverse_trim_us=30)
    assert step(mk, 80.0)[0] == 13601360
    assert step(mk, 161.0)[0] == 12601460
    assert step(mk, 30000.0)[0] == 13601360
    assert step(mk, 5000.0) == (16001400, False)
    assert step(mk, 6000.0)[0] == 14001600
    assert step(mk, 50000.0) == (15001500, False)
    assert step(mk, 260.0, cmd_age=0.6) == (15001500, False)
    assert step(mk, 80.0, level=1)[0] == 14021402


# ⑨ /motor_reverse — 좌우 평균이 reverse_pwm 그대로라 판정 불변
def test_09_motor_reverse_true_with_trim():
    for t in (-64, -10, 10, 64):
        for lv in (0, 1):
            assert step(make(reverse_trim_us=t), 260.0, level=lv)[1] is True, (t, lv)


# ⑩ |t| ≥ 65 → ERROR 1회 + 0 사용. 64 는 받아들인다
def test_10_guard_out_of_range():
    for t in (65, -65, 100, -1000):
        mc = make_logged(reverse_trim_us=t)
        errs = [m for m in mc.get_logger().errors if 'reverse_trim_us' in m]
        assert len(errs) == 1, (t, mc.get_logger().errors)
        assert mc.reverse_trim_us == 0, t
        assert step(mc, 260.0)[0] == 15901590, t
    mc = make_logged(reverse_trim_us=64)
    assert mc.get_logger().errors == [], mc.get_logger().errors
    assert step(mc, 260.0)[0] == 16541526


# ⑪ 비정수(소수·bool·문자열) → ERROR 1회 + 0
def test_11_guard_non_integer():
    for t in (10.0, 0.5, True, '10'):
        mc = make_logged(reverse_trim_us=t)
        errs = [m for m in mc.get_logger().errors if 'reverse_trim_us' in m]
        assert len(errs) == 1, (t, mc.get_logger().errors)
        assert mc.reverse_trim_us == 0, t
        assert step(mc, 260.0)[0] == 15901590, t


# ⑫ 정상 값은 ERROR 없음, 시작 로그에 트림 값
def test_12_start_log_has_trim():
    mc = make_logged(reverse_trim_us=12)
    assert mc.get_logger().errors == []
    assert any('reverse_trim_us=12' in m for m in mc.get_logger().lines), mc.get_logger().lines


# ⑬ motor_control.yaml 의 reverse_trim_us 는 정수 0 (0.0 이면 부팅 사망)
def test_13_yaml_value_is_int_zero():
    with open(_YAML, encoding='utf-8') as f:
        text = f.read()
    m = re.findall(r'^\s+reverse_trim_us:\s*([^\s#]+)', text, re.MULTILINE)
    assert len(m) == 1, m
    assert re.match(r'^[-+]?\d+$', m[0]), f"reverse_trim_us: {m[0]} — 정수로 써야 한다"
    assert int(m[0]) == 0, m[0]


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

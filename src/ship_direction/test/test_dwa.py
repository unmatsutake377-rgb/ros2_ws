#!/usr/bin/env python3
"""(C) DWA 순수 로직 테스트 (ship_direction/dwa.py, ROS 불필요).

실행: tools/run_tests.sh ship_direction

근거: docs/문제와작업/계약서_DWA_20261106.md 검증 계획.
원본 교차(tools/sim/boat3dof·sim/vrx make_boat·convert, motor_control 소스·yaml)는 저장소 경로로 읽고,
파일이 없으면 skip 한다(노드는 tools/·sim/ 을 import 하지 않는다).
시제품 동치: 환경변수 SSF_DWA_PROTO 에 시제품(sd_dwa.py) 경로를 주면 같은 입력에 같은 각인지 본다.
"""

import ast
import importlib.util
import math
import os
import random
import sys
import time

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, PKG)

import ship_direction.dwa as D  # noqa: E402
from ship_direction.dwa import (  # noqa: E402
    DwaPlanner, Hull3DOF, HULLS, pwm_from_cmd, pwm_to_thrust, rect_dist)

SIM_DIR = os.path.join(WS, 'tools', 'sim')
VRX_PKG = os.path.join(WS, 'sim', 'vrx', 'src', 'ssf_vrx', 'ssf_vrx')
MC_PY = os.path.join(WS, 'src', 'motor_control', 'motor_control', 'motor_control.py')
MC_YAML = os.path.join(WS, 'src', 'motor_control', 'config', 'motor_control.yaml')

ANG = [i * 0.5 for i in range(321)]           # LiDAR 프레임 0~160, 0.5° 간격
INF = [math.inf] * len(ANG)


def scene(cells):
    """cells = [(lo, hi, r)] 구간 셀만 r, 나머지 inf."""
    d = list(INF)
    for lo, hi, r in cells:
        for i, a in enumerate(ANG):
            if lo <= a <= hi:
                d[i] = r
    return d


def _load(path, name):
    if not os.path.exists(path):
        pytest.skip(f'{path} 없음')
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _boat3dof():
    if not os.path.exists(os.path.join(SIM_DIR, 'boat3dof.py')) or not os.path.isdir(VRX_PKG):
        pytest.skip('tools/sim/boat3dof.py 또는 sim/vrx 없음')
    if SIM_DIR not in sys.path:
        sys.path.insert(0, SIM_DIR)
    import boat3dof
    return boat3dof


# ======================================================================
# 원본 교차
# ======================================================================
def test_hull_table_equals_make_boat():
    """질량·loa·boa·hydro(6개)·추진기 == make_boat.BOATS. 접촉 직사각형 = loa × 실측 폭(결정 4)."""
    mb = _load(os.path.join(VRX_PKG, 'make_boat.py'), '_ssf_make_boat_for_dwa')
    for k in ('A', 'B'):
        src, mine = mb.BOATS[k], HULLS[k]
        for key in ('mass', 'loa', 'boa'):
            assert mine[key] == src[key], (k, key)
        for key, v in mine['hydro'].items():
            assert v == src['hydro'][key], (k, key)
        assert set(mine['hydro']) == {'xU', 'xUU', 'yV', 'yVV', 'nR', 'nRR'}
        assert [tuple(t) for t in mine['thrusters']] == [tuple(t) for t in src['thrusters']], k
        assert mine['rect_l'] == src['loa'], k
    assert (HULLS['A']['rect_b'], HULLS['B']['rect_b']) == (0.57, 0.535)


def test_pwm_to_thrust_equals_convert():
    cv = _load(os.path.join(VRX_PKG, 'convert.py'), '_ssf_convert_for_dwa')
    for p in list(range(1000, 2001)) + [1474.5, 1475, 1475.5, 1524.5, 1525, 1525.5]:
        assert pwm_to_thrust(p) == cv.pwm_to_thrust(p), p


@pytest.mark.parametrize('hull', ['A', 'B'])
def test_hull_equals_boat3dof(hull):
    """같은 PWM 순서로 굴리면 boat3dof.Boat3DOF 와 같다(정상상태·선회 포함). substeps 2·10."""
    b3 = _boat3dof()
    seq = [(1360, 1360)] * 200 + [(1260, 1460)] * 150 + [(1590, 1590)] * 30 + [(1460, 1260)] * 100
    for sub in (2, 10):
        ref = b3.Boat3DOF(hull, substeps=sub)
        me = Hull3DOF(hull, substeps=sub)
        for pl, pr in seq:
            ref.step(pl, pr, 0.1)
            me.step(pl, pr, 0.1)
            for key in ('u', 'v', 'r', 'x', 'y', 'psi'):
                assert math.isclose(getattr(me, key), getattr(ref, key),
                                    rel_tol=1e-9, abs_tol=1e-9), (hull, sub, key)
    u, w = b3.steady_state(hull, 1360, 1360)
    me = Hull3DOF(hull, substeps=10)
    for _ in range(400):
        me.step(1360, 1360, 0.1)
    assert math.isclose(me.u, u, rel_tol=1e-9) and abs(w) < 1e-9


def _mc_source():
    if not os.path.exists(MC_PY):
        pytest.skip('motor_control.py 없음')
    with open(MC_PY, encoding='utf-8') as f:
        return f.read()


def _mc_declared(src):
    out = {}
    for n in ast.walk(ast.parse(src)):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and n.func.attr == 'declare_parameter' and len(n.args) >= 2):
            out[n.args[0].value] = ast.literal_eval(n.args[1])
    return out


def test_pwm_constants_equal_motor_control():
    """1360/100/1590 == motor_control declare 기본값 == motor_control.yaml, max_angle 81(위험 1)."""
    src = _mc_source()
    dec = _mc_declared(src)
    assert (dec['base_pwm'], dec['max_diff'], dec['reverse_pwm']) == \
        (D.BASE_PWM, D.MAX_DIFF, D.REVERSE_PWM)
    with open(MC_YAML, encoding='utf-8') as f:
        p = yaml.safe_load(f)['/motor_control']['ros__parameters']
    assert (p['base_pwm'], p['max_diff'], p['reverse_pwm']) == \
        (D.BASE_PWM, D.MAX_DIFF, D.REVERSE_PWM)
    assert 'self.max_angle = 81' in src and D.MAX_ANGLE == 81


def test_pwm_from_cmd_equals_motor_control_branches():
    """분기 (4)(5) 를 motor_control 소스의 linear_diff·apply_steer 로 직접 계산해 비교."""
    src = _mc_source()
    assert 'elif -1.0 <= angle <= 161.0:' in src
    assert 'elif 161.0 < angle < SPIN_RIGHT:' in src
    assert 'SPIN_RIGHT = 5000' in src
    tree = ast.parse(src)
    ns = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.FunctionDef) and n.name in ('linear_diff', 'apply_steer'):
            exec(compile(ast.Module(body=[n], type_ignores=[]), MC_PY, 'exec'), ns)
    fake = type('F', (), {'max_angle': 81, 'max_diff': D.MAX_DIFF, 'base_pwm': D.BASE_PWM,
                          'steer_invert': False})()
    angles = [x * 0.25 for x in range(-4, 645)]       # −1 ~ 161
    for a in angles:
        offset = a - 80.0
        diff = ns['linear_diff'](fake, offset)
        pwm_r, pwm_l = ns['apply_steer'](fake, diff if offset > 0 else -diff,
                                         center=D.BASE_PWM)
        assert pwm_from_cmd(a) == (pwm_l, pwm_r), a
    for a in (161.01, 180.0, 260.0, 4999.9):
        assert pwm_from_cmd(a) == (D.REVERSE_PWM, D.REVERSE_PWM), a
    for a in (-1.01, 5000.0, 6000.0, 20000.0, 50000.0, math.nan, math.inf, None):
        assert pwm_from_cmd(a) == (1500, 1500), a


def test_pwm_from_cmd_sign():
    """40(우) → 왼쪽이 더 전진(작은 PWM), 120(좌) → 오른쪽이 더 전진. 0/160 은 ±98."""
    pl, pr = pwm_from_cmd(40.0)
    assert pl < 1360 < pr
    pl, pr = pwm_from_cmd(120.0)
    assert pr < 1360 < pl
    assert pwm_from_cmd(0.0) == (1262, 1458) and pwm_from_cmd(160.0) == (1458, 1262)


# ======================================================================
# 모델·배율
# ======================================================================
def test_agility_one_is_identity_and_formula():
    """a = 1.0 → 원본 계수 그대로. a → 감쇠 ÷a, 추력 ×a, 질량·관성 그대로(결정 5)."""
    for k in ('A', 'B'):
        one = Hull3DOF(k, 1.0)
        assert one.h == HULLS[k]['hydro']
        assert one.targets(1300, 1400) == Hull3DOF(k).targets(1300, 1400)
        for a in (0.5, 0.7, 1.4, 2.0):
            h = Hull3DOF(k, a)
            assert h.m == one.m and h.Iz == one.Iz
            for key, v in one.h.items():
                assert math.isclose(h.h[key], v / a), (k, a, key)
            for t1, ta in zip(one.targets(1300, 1400), h.targets(1300, 1400)):
                assert math.isclose(ta, t1 * a)
    ref = DwaPlanner('A')
    one = DwaPlanner('A', 1.0)
    d = scene([(84.0, 88.0, 3.0)])
    assert one.choose(ANG, d, 80.0) == ref.choose(ANG, d, 80.0)


def test_iz_from_make_boat_dims():
    for k in ('A', 'B'):
        s = HULLS[k]
        assert Hull3DOF(k).Iz == s['mass'] / 12.0 * (s['loa'] ** 2 + s['boa'] ** 2)


def test_observe_converges():
    """observe(80) 반복 → 1360 정상상태 속도, r → 0. 40 → 우선회(r < 0), 120 → 좌선회(r > 0)."""
    for k in ('A', 'B'):
        ref = Hull3DOF(k)
        for _ in range(400):
            ref.step(1360, 1360, 0.1)
        p = DwaPlanner(k)
        for _ in range(400):
            p.observe(80.0)
        assert math.isclose(p.state[0], ref.u, rel_tol=1e-6) and abs(p.state[2]) < 1e-9
        assert 0.8 < p.state[0] < 1.2, p.state
        assert p.last == 80.0
        for _ in range(50):
            p.observe(40.0)
        assert p.state[2] < 0.0 and p.last == 40.0
        for _ in range(80):
            p.observe(120.0)
        assert p.state[2] > 0.0


def test_observe_last_only_for_steering_values():
    """last 는 유한한 0~160 일 때만 바뀐다. NaN·260·50000·None·inf·범위 밖은 유지, 모델은 굴린다."""
    p = DwaPlanner('A')
    p.observe(40.0)
    assert p.last == 40.0
    for cmd in (math.nan, 260.0, 50000.0, None, math.inf, -math.inf, -1.0, 161.0, 5000.0):
        s0 = p.state
        p.observe(cmd)
        assert p.last == 40.0, cmd
        assert p.state != s0, cmd                              # 추정은 계속 굴린다
    for cmd in (0.0, 160.0, 80.0):
        p.observe(cmd)
        assert p.last == cmd


def test_nonfinite_last_treated_as_init():
    """이중 방어: last 가 비유한이면 80 으로 본다 → 비용이 NaN 이 되지 않아 0° 로 쏠리지 않는다."""
    p, q = DwaPlanner('A'), DwaPlanner('A')
    p.last = math.nan
    for d in (INF, scene([(82.0, 86.0, 3.0)])):
        assert p.choose(ANG, d, 80.0) == q.choose(ANG, d, 80.0)
    assert p.choose(ANG, INF, 80.0) == 80.0


def test_reset_restores_initial():
    p = DwaPlanner('B')
    for _ in range(30):
        p.observe(260.0)
    assert p.state != D.INIT_STATE and p.last == D.INIT_LAST     # 260 은 last 를 안 바꾼다
    p.observe(120.0)
    assert p.last == 120.0
    p.reset()
    assert p.state == D.INIT_STATE and p.last == D.INIT_LAST
    assert all(t == 0.0 for t in p.est.T)


def test_constants():
    p = DwaPlanner('A')
    assert len(p.cands) == 33 and p.cands[0] == 0.0 and p.cands[-1] == 160.0
    assert list(p.cands) == sorted(p.cands)                 # 동률이면 작은 각(앞쪽)
    assert p.n_steps == 25 and D.SUBSTEPS == 2 and p.est.substeps == 2
    assert (D.D_HARD_M, D.D_SOFT_M, D.W_CLEAR, D.W_SMOOTH) == (0.25, 0.8, 0.8, 0.1)
    assert (D.R_MIN_M, D.R_MAX_M, D.INIT_STATE) == (0.05, 5.0, (0.9, 0.0, 0.0))
    with pytest.raises(ValueError):
        DwaPlanner('C')


# ======================================================================
# 직사각형 거리·점 필터
# ======================================================================
def test_rect_dist_side_corner_inside():
    L, B = 1.66, 0.57
    hx, hy = L / 2, B / 2
    wx = np.array([0.0, hy + 0.5, -(hy + 0.3), 0.1, 0.0, hy + 0.3])
    wy = np.array([hx + 1.0, 0.0, -(hx + 0.4), -0.2, -(hx + 0.2), hx + 0.4])
    d = rect_dist(wx, wy, 0.0, 0.0, 0.0, L, B)
    assert d.shape == (6,)
    want = [1.0, 0.5, 0.5, 0.0, 0.2, 0.5]
    assert np.allclose(d, want), d
    # 동쪽(90°)을 보면 앞 = +x
    d = rect_dist(np.array([hx + 1.0, 0.0]), np.array([0.0, hy + 0.5]), 0.0, 0.0, 90.0, L, B)
    assert np.allclose(d, [1.0, 0.5]), d
    # 벡터화: (K,) 자세 → (K, N)
    d = rect_dist(wx, wy, np.array([0.0, 10.0]), np.array([0.0, 0.0]), np.array([0.0, 0.0]), L, B)
    assert d.shape == (2, 6) and np.allclose(d[0], want) and np.all(d[1] > 8.0)


def test_point_filter_bounds():
    """0.05 < r ≤ 5.0 만. inf·nan·None·음수·0 은 버린다."""
    p = DwaPlanner('A')
    rs = [0.05, 0.0500001, 5.0, 5.0000001, math.inf, math.nan, None, -1.0, 0.0, 2.0]
    P = p.points([80.0] * len(rs), rs)
    assert sorted(P[:, 0].tolist()) == [0.0500001, 2.0, 5.0]
    assert np.allclose(P[:, 1], 0.0)
    P = p.points([170.0, 80.0, -10.0], [1.0, 1.0, 1.0])   # 각 → x 앞, y 왼쪽
    assert np.allclose(P, [[0.0, 1.0], [1.0, 0.0], [0.0, -1.0]], atol=1e-12)
    assert p.points([], []).shape == (0, 2)


def test_lidar_x_shift():
    """dwa_lidar_x_m 은 점 x 에 더한다. 앞 점 r + lidar_x 0.3 == lidar_x 0 의 r + 0.3."""
    p0, p3 = DwaPlanner('A'), DwaPlanner('A', lidar_x_m=0.3)
    P = p3.points([80.0, 170.0], [2.0, 1.0])
    assert np.allclose(P, [[2.3, 0.0], [0.3, 1.0]])
    for r in (2.5, 2.7, 3.0):
        assert p3.choose(ANG, scene([(84.0, 88.0, r)]), 80.0) == \
            p0.choose(ANG, scene([(84.0, 88.0, r + 0.3)]), 80.0), r


# ======================================================================
# choose
# ======================================================================
def test_no_obstacle_goes_to_goal():
    p = DwaPlanner('A')
    assert p.choose(ANG, INF, 80.0) == 80.0
    left, right = p.choose(ANG, INF, 120.0), p.choose(ANG, INF, 40.0)
    assert left > 80.0 and right < 80.0 and left + right == 160.0
    assert p.choose(ANG, [], 80.0) == 80.0                 # 점 없음


@pytest.mark.parametrize('hull', ['A', 'B'])
def test_front_buoy_goes_other_side_mirror(hull):
    """정면 약간 왼쪽 부표 3.0 m → 오른쪽, 거울 → 왼쪽(합 160).

    1.5 m 는 추정 0.9 m/s 에서 33개 모두 2.5 s 안에 0.25 m 안으로 들어 None(폴백) — 그것도 본다.
    """
    p = DwaPlanner(hull)
    a = p.choose(ANG, scene([(82.0, 86.0, 3.0)]), 80.0)
    b = p.choose(ANG, scene([(74.0, 78.0, 3.0)]), 80.0)
    assert a is not None and a < 80.0, a
    assert b is not None and b > 80.0, b
    assert a + b == 160.0
    assert p.choose(ANG, scene([(82.0, 86.0, 1.5)]), 80.0) is None


def test_surrounded_or_inside_is_none():
    p = DwaPlanner('A')
    assert p.choose(ANG, scene([(0.0, 160.0, 0.3)]), 80.0) is None      # 사방 0.3 m
    assert p.choose(ANG, scene([(80.0, 80.0, 0.2)]), 80.0) is None      # 선체 안(0.2 m) 점
    assert p.choose(ANG, scene([(80.0, 80.0, 0.04)]), 80.0) == 80.0     # 0.05 이하는 버림


def test_nonfinite_yaw_is_none():
    p = DwaPlanner('A')
    for y in (math.nan, math.inf, -math.inf, None):
        assert p.choose(ANG, INF, y) is None, y


def test_tie_prefers_smaller_angle(monkeypatch):
    """비용이 정확히 같으면 작은 각(우현). 굴림을 모두 같게(1500) 만들고 직전 82.5 → 80·85 동률."""
    monkeypatch.setattr(D, 'pwm_from_cmd', lambda a: (1500, 1500))
    p = DwaPlanner('A')
    p.state = (0.0, 0.0, 0.0)
    p.last = 82.5
    assert p.choose(ANG, INF, 80.0) == 80.0
    p.last = 77.5
    assert p.choose(ANG, INF, 80.0) == 75.0


def test_inputs_not_mutated():
    p = DwaPlanner('A')
    ang = list(ANG)
    d = scene([(82.0, 86.0, 3.0), (10.0, 10.0, math.nan), (20.0, 20.0, -1.0)])
    snap = (list(ang), list(d), p.state, p.last)
    p.choose(ang, d, 80.0)
    assert ang == snap[0] and d == snap[1]                # 같은 객체(nan 포함) 그대로
    assert (p.state, p.last) == snap[2:]                    # choose 는 추정을 안 바꾼다


def test_choose_time_once():
    """1회 계산 시간(로그 전용, 결정 12 기준 50 ms). Mac 실측 약 1~2 ms — 여기선 1 s 넘으면 실패."""
    p = DwaPlanner('B')
    d = scene([(30.0, 40.0, 2.5), (82.0, 86.0, 3.0), (120.0, 140.0, 4.0)])
    ts = []
    for _ in range(20):
        t0 = time.perf_counter()
        p.choose(ANG, d, 100.0)
        ts.append(time.perf_counter() - t0)
    ts.sort()
    print(f'DWA 1회 p50 {ts[10] * 1e3:.2f} ms, 최대 {ts[-1] * 1e3:.2f} ms')
    assert ts[-1] < 1.0


# ======================================================================
# 시제품 동치 (선택) — 폭만 시제품 값(make_boat 0.55/0.522)으로 되돌리면 같은 각
# ======================================================================
def _proto_cls():
    path = os.environ.get('SSF_DWA_PROTO', '')
    if not path or not os.path.exists(path):
        pytest.skip('SSF_DWA_PROTO(시제품 sd_dwa.py 경로) 없음')
    b3 = _boat3dof()
    with open(path, encoding='utf-8') as f:
        src = f.read()
    cls = [n for n in ast.parse(src).body if isinstance(n, ast.ClassDef) and n.name == '_DWA']
    assert cls, '시제품에 _DWA 없음'
    ns = {'_os': os, '_sys': sys, '_m': math, '_np': np, '_B3': b3.Boat3DOF}
    exec(compile(ast.Module(body=cls, type_ignores=[]), path, 'exec'), ns)
    return ns['_DWA']


@pytest.mark.parametrize('hull', ['A', 'B'])
def test_equals_prototype(hull, monkeypatch):
    proto_cls = _proto_cls()
    for k in ('DWA_DAMP', 'DWA_THR', 'DWA_H', 'DWA_DHARD', 'DWA_DSOFT', 'DWA_WC', 'DWA_STEP'):
        monkeypatch.delenv(k, raising=False)
    monkeypatch.setenv('DWA_HULL', hull)
    proto = proto_cls()
    me = DwaPlanner(hull)
    me.rect_b = HULLS[hull]['boa']                          # 결정 4 이전 폭
    rng = random.Random(7)
    for _ in range(80):
        d = list(INF)
        for _k in range(rng.randint(0, 4)):
            c, w, r = rng.uniform(0, 160), rng.uniform(2, 15), rng.uniform(0.3, 5.5)
            for i, a in enumerate(ANG):
                if abs(a - c) < w:
                    d[i] = r
        yaw = rng.uniform(-10.0, 170.0)
        a1, a2 = proto.choose(ANG, d, yaw), me.choose(ANG, d, yaw)
        assert a1 == a2, (yaw, a1, a2)
        # 폴백 명령은 정상 조향값만 — 260·50000·NaN 의 last 처리만 시제품과 다르다(observe 주석)
        cmd = a1 if a1 is not None else rng.choice([40.0, 120.0])
        proto.observe(cmd)
        me.observe(cmd)
        assert proto.state == me.state and proto.last == me.last

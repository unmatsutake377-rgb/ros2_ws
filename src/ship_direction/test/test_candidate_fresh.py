#!/usr/bin/env python3
"""candidate 전체 신선도 테스트 — effective_candidate / CandidateStaleLog / control_cb / wp_mode_cb.
실행: python3 src/ship_direction/test/test_candidate_fresh.py

계약서 docs/문제와작업/계약서_전체코스_20261104.md §2-6·§4·§6·§10-3.
규약: /candidate_angle 수신 후 candidate_stale_sec(1.0 s) 넘게 갱신 없음 → control_cb 가 _compute
  전에 20000(GPS 주행+회피)으로 바꾼다. 수신 기록 없음(None)도 20000. 180(후진)은 0.5 s 규칙 유지
  (0.5~1.0 s 는 _compute 가 STOP_HOLD, 1.0 s 넘으면 20000). wp_mode 가 바뀌면 20000·None 초기화.
  로그: 진입 ERROR 1회 + 5 s 주기 ERROR, 복구 INFO 1회 — 심각도마다 호출 줄이 다르다(CLAUDE.md §1-7).

ship_direction.py 는 rclpy 를 import 하므로 test_candidate_reverse.py 와 같은 ROS 스텁 + fake self 로
**실제 메서드**를 언바운드 호출한다. 시간은 모듈의 time 을 가짜 시계로 바꿔 제어한다.
"""

import math
import os
import re
import sys
import threading
import types

import yaml

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))

# ---- ROS 모듈 최소 스텁 (import 통과용, 로직엔 안 쓰임) ----


def _stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class _Node:                       # class ShipDirection(Node) 상속용
    pass


_pol = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2, KEEP_LAST=1)
_stub('rclpy', init=None, ok=lambda: True, shutdown=None)
_stub('rclpy.node', Node=_Node)
_stub('rclpy.executors', MultiThreadedExecutor=object)
_stub('rclpy.qos',
      QoSProfile=lambda **k: types.SimpleNamespace(**k),
      ReliabilityPolicy=_pol, HistoryPolicy=_pol)


def _msg(*a, **k):
    return types.SimpleNamespace(data=k.get('data', a[0] if a else None))


_stub('std_msgs', msg=None)
_stub('std_msgs.msg', Int32=_msg, Float32=_msg, Float32MultiArray=_msg)
_stub('sensor_msgs', msg=None)
_stub('sensor_msgs.msg', LaserScan=_msg)

import ship_direction.ship_direction as SD  # noqa: E402
from ship_direction.ship_direction import (  # noqa: E402
    ShipDirection, CANDIDATE_INVALID, REVERSE_ANGLE, STOP_HOLD, SPIN_RIGHT, SPIN_LEFT,
    CANDIDATE_STALE_LOG_PERIOD_SEC, YAW_MISSING_LOG_PERIOD_SEC)
from ship_direction.failsafe import (  # noqa: E402
    effective_candidate, candidate_is_stale, CandidateStaleLog)

STALE = 1.0          # candidate_stale_sec 코드 기본값(=대회값)
REV_STALE = 0.5      # candidate_reverse_stale_sec 코드 기본값
OBST = object()      # _closest_obstacle 대용 센티널
SCAN = object()      # latest_scan 대용 센티널
VALUES = (30.0, 280.0, 180.0, SPIN_RIGHT, SPIN_LEFT, STOP_HOLD, CANDIDATE_INVALID)
PKG = os.path.join(HERE, '..')

_p = _t = 0


def check(name, fn):
    global _p, _t
    _t += 1
    try:
        fn()
        _p += 1
        print(f"  ✅ {name}")
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")


class _Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def monotonic(self):
        return self.t


class _Logger:
    """심각도와 **호출 줄**을 같이 기록 — rclpy 처럼 한 줄이 두 심각도를 쓰면 실패시킨다."""

    def __init__(self):
        self.lines = []          # (severity, msg)
        self.sites = {}          # (file, line) → severity

    def _log(self, sev, msg):
        f = sys._getframe(2)
        site = (f.f_code.co_filename, f.f_lineno)
        old = self.sites.setdefault(site, sev)
        assert old == sev, f"{site} 가 {old} 와 {sev} 를 같이 씀 (§1-7)"
        self.lines.append((sev, msg))

    def warn(self, msg, **kw):
        self._log('WARN', msg)

    def info(self, msg, **kw):
        self._log('INFO', msg)

    def error(self, msg, **kw):
        self._log('ERROR', msg)

    def count(self, sev):
        return sum(1 for s, _ in self.lines if s == sev)


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def _f32(data=None):
    return types.SimpleNamespace(data=data)


def with_clock(fn):
    """SD.time 을 가짜 시계로, SD.Float32 를 값 보존 메시지로 바꿔 fn(clock) 실행 후 되돌린다.
    (pytest 한 프로세스에선 먼저 import 한 다른 테스트의 스텁 Float32(None 반환)가 모듈에 남아 있다)"""
    real_t, real_f = SD.time, SD.Float32
    clk = _Clock()
    SD.time, SD.Float32 = clk, _f32
    try:
        return fn(clk)
    finally:
        SD.time, SD.Float32 = real_t, real_f


def fake_self(wp_mode=7):
    """control_cb·_effective_candidate·_compute(0)~(B)·candidate_callback·wp_mode_cb 가 읽는 것만."""
    log = _Logger()
    fs = types.SimpleNamespace(
        lock=threading.Lock(),
        failsafe_level=0,
        latest_scan=SCAN,
        watch=types.SimpleNamespace(fault=False),
        wp_mode=wp_mode,
        wp4_enter_time=None,
        yaw_error=0.0,
        detection_distance=3.0,
        detection_distance_default=3.0,
        detection_distance_gate=2.0,
        gate_wp_modes=[0, 1],
        candidate_angle=CANDIDATE_INVALID,
        candidate_t=None,
        candidate_stale_sec=STALE,
        candidate_reverse_stale_sec=REV_STALE,
        candidate_stale_log=CandidateStaleLog(CANDIDATE_STALE_LOG_PERIOD_SEC),
        # [N] 로그 상태. 여기선 yaw_error 0.0(받음)이라 막히지 않는다
        yaw_missing_log=CandidateStaleLog(YAW_MISSING_LOG_PERIOD_SEC),
        geofence=None,
        last_geofence_t=None,
        geofence_stale_sec=2.0,
        geofence_stop_margin_m=2.0,
        geofence_stop_cone_deg=60.0,
        reverse_state='READY',
        reverse_start_time=None,
        reverse_cooldown_start_time=None,
        prev_final_angle=None,
        pub_desired_angle=_Pub(),
        pub_obstacle_distance=_Pub(),
        get_logger=lambda: log,
        log=log,
        seen=[],                 # control_cb 가 _compute 에 넘긴 candidate
    )
    fs._closest_obstacle = lambda msg: OBST
    fs._reset_reverse_sequence = lambda: ShipDirection._reset_reverse_sequence(fs)
    fs._geofence_blocks = lambda rel: ShipDirection._geofence_blocks(fs, rel)
    fs._apply_rate_limit = lambda a: a
    fs._effective_candidate = (
        lambda c, ct, now, wm: ShipDirection._effective_candidate(fs, c, ct, now, wm))

    def _compute(msg, wp4, cand, yaw, dd, cand_t):
        fs.seen.append(cand)
        if cand == CANDIDATE_INVALID:
            return 'AUTO', OBST          # (C) 자율 회피 — 스캔 계산은 이 테스트 범위 밖
        return ShipDirection._compute(fs, msg, wp4, cand, yaw, dd, cand_t)
    fs._compute = _compute
    return fs


def receive(fs, clk, raw):
    ShipDirection.candidate_callback(fs, types.SimpleNamespace(data=raw))
    assert fs.candidate_t == clk.t


def tick(fs):
    """control_cb 1회 → (_compute 가 받은 candidate, 발행한 desired)."""
    ShipDirection.control_cb(fs)
    return fs.seen[-1], fs.pub_desired_angle.msgs[-1].data


def after(raw, age):
    """raw 수신 → age 초 뒤 control_cb 1회 → (_compute 가 받은 candidate, desired, fake self)."""
    def run(clk):
        fs = fake_self()
        receive(fs, clk, raw)
        clk.t += age
        cand, des = tick(fs)
        return cand, des, fs
    return with_clock(run)


# ---------------- 순수 함수
def test_pure_fresh_keeps_value():
    for v in VALUES:
        for age in (0.0, 0.5, 0.99, STALE):          # 경계 포함(초과일 때만 stale)
            got = effective_candidate(v, 100.0, 100.0 + age, STALE, CANDIDATE_INVALID)
            assert got == v, f"{v} age {age} → {got}"
            assert not candidate_is_stale(100.0, 100.0 + age, STALE)


def test_pure_stale_to_invalid():
    for v in VALUES:
        for age in (STALE + 1e-6, 1.1, 5.0, 1e6):
            got = effective_candidate(v, 100.0, 100.0 + age, STALE, CANDIDATE_INVALID)
            assert got == CANDIDATE_INVALID, f"{v} age {age} → {got}"
            assert candidate_is_stale(100.0, 100.0 + age, STALE)


def test_pure_none_is_invalid_not_stale():
    for v in VALUES:
        assert effective_candidate(v, None, 100.0, STALE, CANDIDATE_INVALID) == CANDIDATE_INVALID
    assert not candidate_is_stale(None, 100.0, STALE), "수신 기록 없음은 stale 아님(로그 없음)"


def test_initial_state_is_invalid_none():
    """노드 초기값(candidate 20000·candidate_t None) = 수신 기록 없음 대체값과 같다.
    실제 __init__ 을 돌려 확인한다(소스 문자열 검사는 wp_mode_cb 초기화 줄과도 맞아 판별력 없음)."""
    node, _ = _construct({})
    assert CANDIDATE_INVALID == 20000.0
    assert node.candidate_angle == 20000.0, node.candidate_angle
    assert node.candidate_t is None, node.candidate_t


# ---------------- control_cb 경유
def test_control_fresh_passes_value():
    for v, want in [(30.0, 110.0), (280.0, 0.0), (SPIN_RIGHT, SPIN_RIGHT),
                    (SPIN_LEFT, SPIN_LEFT), (STOP_HOLD, STOP_HOLD)]:
        for age in (0.0, 0.5, STALE):
            cand, des, fs = after(v, age)
            assert cand == v and des == want, f"{v} age {age} → cand {cand} des {des}"
            assert fs.log.count('ERROR') == 0


def test_control_stale_becomes_invalid():
    for v in (30.0, 280.0, SPIN_RIGHT, SPIN_LEFT, STOP_HOLD):
        cand, des, fs = after(v, 1.1)
        assert cand == CANDIDATE_INVALID and des == 'AUTO', f"{v} → cand {cand} des {des}"
        assert fs.log.count('ERROR') == 1


def test_control_none_is_invalid_without_log():
    def run(clk):
        fs = fake_self()
        for _ in range(100):
            clk.t += 0.1
            cand, des = tick(fs)
            assert cand == CANDIDATE_INVALID and des == 'AUTO'
        return fs
    fs = with_clock(run)
    assert fs.log.lines == [], fs.log.lines


def test_reverse_180_windows():
    """180: ≤0.5 s 260 / 0.5~1.0 s STOP_HOLD(_compute) / 1.0 s 넘으면 20000(effective_candidate)."""
    for age, want_cand, want_des in [(0.0, 180.0, REVERSE_ANGLE), (0.5, 180.0, REVERSE_ANGLE),
                                     (0.6, 180.0, STOP_HOLD), (0.9, 180.0, STOP_HOLD),
                                     (1.0, 180.0, STOP_HOLD),
                                     (1.1, CANDIDATE_INVALID, 'AUTO'),
                                     (3.0, CANDIDATE_INVALID, 'AUTO')]:
        cand, des, _ = after(180.0, age)
        assert cand == want_cand and des == want_des, \
            f"180 age {age} → cand {cand} des {des}, 기대 {want_cand}/{want_des}"


def test_publisher_death_timeline():
    """10 Hz 발행 노드가 STOP_HOLD 를 내다 죽음 → 1.0 s 넘는 첫 틱부터 20000."""
    def run(clk):
        fs = fake_self()
        out = []
        for _ in range(10):
            receive(fs, clk, STOP_HOLD)
            out.append(tick(fs)[0])
            clk.t += 0.1
        t_dead = clk.t - 0.1                 # 마지막 수신 시각
        while clk.t - t_dead < 2.0:
            out.append((clk.t - t_dead, tick(fs)[0]))
            clk.t += 0.1
        return out
    out = with_clock(run)
    assert all(c == STOP_HOLD for c in out[:10])
    # 0.1 s 틱 누적의 부동소수 오차 때문에 정확히 1.0 s 틱은 어느 쪽이든 허용(실제론 측도 0)
    for age, c in out[10:]:
        if age < STALE - 1e-6:
            assert c == STOP_HOLD, f"age {age:.3f} → {c}, 기대 STOP_HOLD"
        elif age > STALE + 1e-6:
            assert c == CANDIDATE_INVALID, f"age {age:.3f} → {c}, 기대 20000"
    first = min(a for a, c in out[10:] if c == CANDIDATE_INVALID)
    assert STALE - 1e-6 <= first <= STALE + 0.1 + 1e-6, f"20000 전환 {first:.3f} s"


# ---------------- wp_mode 변경 초기화
def _wp(fs, m):
    ShipDirection.wp_mode_cb(fs, types.SimpleNamespace(data=m))


def test_wp_mode_change_resets():
    def run(clk):
        fs = fake_self(wp_mode=7)
        receive(fs, clk, 30.0)
        _wp(fs, 7)                                         # 같은 모드 재발행 → 유지
        assert fs.candidate_angle == 30.0 and fs.candidate_t == clk.t
        _wp(fs, 8)                                         # 바뀜 → 초기화
        assert fs.candidate_angle == CANDIDATE_INVALID and fs.candidate_t is None
        cand, des = tick(fs)
        assert cand == CANDIDATE_INVALID and des == 'AUTO'
        receive(fs, clk, SPIN_LEFT)                        # 새 모드 발행자 → 그대로
        assert tick(fs)[0] == SPIN_LEFT
        fs.wp_mode = -1                                    # 부팅 첫 /wp_mode 도 바뀜
        _wp(fs, 0)
        assert fs.candidate_angle == CANDIDATE_INVALID and fs.candidate_t is None
        assert fs.log.lines == [], fs.log.lines
    with_clock(run)


# ---------------- 로그 (진입 ERROR 1회·5 s 주기 ERROR·복구 INFO 1회, 줄별 단일 심각도)
def test_stale_log_sequence():
    def run(clk):
        fs = fake_self()
        receive(fs, clk, 30.0)
        errs = []
        for _ in range(130):                               # 수신 뒤 13 s
            clk.t += 0.1
            tick(fs)
            errs.append(fs.log.count('ERROR'))
        assert fs.log.count('INFO') == 0
        # 진입(1.0~1.1 s) 1 + 주기 5 s 마다(~6.1·~11.1 s) = 3
        assert fs.log.count('ERROR') == 3, fs.log.lines
        first = (errs.index(1) + 1) * 0.1                 # 수신 뒤 진입 ERROR 시각
        assert STALE - 1e-6 <= first <= STALE + 0.1 + 1e-6, f"진입 ERROR 시각 {first:.2f}"
        assert '20000' in fs.log.lines[0][1]
        receive(fs, clk, 30.0)                             # 새 수신 → 복구 INFO 1회
        clk.t += 0.1
        tick(fs)
        tick(fs)
        assert fs.log.count('INFO') == 1 and fs.log.count('ERROR') == 3, fs.log.lines
        assert '새 수신' in fs.log.lines[-1][1]
        clk.t += 1.05                                      # 다시 stale → 진입 ERROR 다시 1회
        tick(fs)
        assert fs.log.count('ERROR') == 4
        _wp(fs, 8)                                         # 모드 변경 초기화 → 복구 INFO
        tick(fs)
        assert fs.log.count('INFO') == 2 and 'wp_mode 변경' in fs.log.lines[-1][1]
        # 호출 줄 3개(진입 ERROR·주기 ERROR·복구 INFO), 줄마다 심각도 하나(_Logger 가 검사)
        assert len(fs.log.sites) == 3, fs.log.sites
    with_clock(run)


def test_stale_log_pure():
    g = CandidateStaleLog(5.0)
    seq = [(False, 0.0, None), (True, 1.0, 'enter'), (True, 5.9, None), (True, 6.0, 'repeat'),
           (True, 10.9, None), (True, 11.0, 'repeat'), (False, 11.1, 'recover'),
           (False, 20.0, None), (True, 21.0, 'enter'), (False, 21.1, 'recover')]
    for stale, now, want in seq:
        got = g.update(stale, now)
        assert got == want, f"stale={stale} t={now} → {got}, 기대 {want}"


def test_stale_log_while_level2():
    """레벨 2(정지) 중에도 신선도 로그는 같은 기준으로 나간다(출력은 STOP_HOLD 그대로)."""
    def run(clk):
        fs = fake_self()
        fs.failsafe_level = 2
        receive(fs, clk, 30.0)
        clk.t += 1.1
        ShipDirection.control_cb(fs)
        assert fs.pub_desired_angle.msgs[-1].data == STOP_HOLD
        assert fs.log.count('ERROR') == 1
    with_clock(run)


# ---------------- 파라미터: 코드 기본값 == yaml, 이상값 방어
def _construct(overrides):
    """ShipDirection.__init__ 을 실제로 돌린다 — Node 메서드만 잠깐 끼운다."""
    base = ShipDirection.__bases__[0]
    log = _Logger()
    names = ('__init__', 'declare_parameter', 'create_subscription', 'create_publisher',
             'create_timer', 'get_logger')
    saved = {n: base.__dict__[n] for n in names if n in base.__dict__}
    base.__init__ = lambda self, name: None
    base.declare_parameter = lambda self, n, d: types.SimpleNamespace(value=overrides.get(n, d))
    base.create_subscription = lambda self, *a, **k: None
    base.create_publisher = lambda self, *a, **k: _Pub()
    base.create_timer = lambda self, *a, **k: None
    base.get_logger = lambda self: log
    try:
        node = ShipDirection()
    finally:
        for n in names:
            if n in saved:
                setattr(base, n, saved[n])
            elif n in base.__dict__:
                delattr(base, n)
    return node, log


def test_code_default_equals_yaml():
    with open(os.path.join(PKG, 'config', 'ship_direction.yaml'), encoding='utf-8') as f:
        p = yaml.safe_load(f)['/ship_direction']['ros__parameters']
    node, log = _construct({})
    assert node.candidate_stale_sec == STALE == p['candidate_stale_sec'], \
        (node.candidate_stale_sec, p['candidate_stale_sec'])
    assert node.candidate_reverse_stale_sec == REV_STALE == p['candidate_reverse_stale_sec']
    src = open(os.path.join(PKG, 'ship_direction', 'ship_direction.py'), encoding='utf-8').read()
    m = re.search(r"declare_parameter\('candidate_stale_sec', ([0-9.]+)\)", src)
    assert m and float(m.group(1)) == p['candidate_stale_sec']
    assert log.count('WARN') == 0


def test_bad_param_falls_back():
    for bad in (0.0, -1.0, math.nan, math.inf):
        node, log = _construct({'candidate_stale_sec': bad})
        assert node.candidate_stale_sec == 1.0, (bad, node.candidate_stale_sec)
        assert any('candidate_stale_sec' in m for s, m in log.lines if s == 'WARN'), log.lines
    node, _ = _construct({'candidate_stale_sec': 2.5})
    assert node.candidate_stale_sec == 2.5


def _stale_warns(log):
    return [m for s, m in log.lines if s == 'WARN' and 'candidate_stale_sec' in m]


def test_short_stale_sec_warns_but_keeps():
    """north 발행 주기(0.5 s)보다 짧으면 거짓 stale 위험 WARN 1회, 값은 그대로. 1.0 은 WARN 없음."""
    node, log = _construct({'candidate_stale_sec': 0.3})
    assert node.candidate_stale_sec == 0.3, node.candidate_stale_sec
    w = _stale_warns(log)
    assert len(w) == 1 and '거짓 stale' in w[0], log.lines
    for ok in (1.0, 0.5):
        node, log = _construct({'candidate_stale_sec': ok})
        assert node.candidate_stale_sec == ok and _stale_warns(log) == [], (ok, log.lines)


def main():
    print("=== candidate 전체 신선도 테스트 (계약서_전체코스_20261104 §2-6) ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())

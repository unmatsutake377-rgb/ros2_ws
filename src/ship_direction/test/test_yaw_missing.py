#!/usr/bin/env python3
"""부팅 뒤 /yaw_error 미수신 + candidate 20000 → STOP_HOLD 테스트.
실행: python3 src/ship_direction/test/test_yaw_missing.py

계약서 docs/문제와작업/계약서_yaw미수신정지_20261105.md (안 A).
규약: yaw_error 를 한 번도 못 받았고(None) 실효 candidate 가 20000(갭 선택)이면 control_cb 가 _compute
  전에 candidate 를 STOP_HOLD(50000)로 바꾼다. 판정은 `is None` — 0.0 은 '정확히 정렬'인 정상값.
  임무 노드가 주는 각도·5000·6000·180 은 yaw_error 를 안 읽으므로 그대로. /failsafe_level 은 안 건드림.
  로그: 진입 ERROR 1회 + 5 s 주기 ERROR(막힌 틱에서만), 복구 INFO 1회(yaw_error 첫 수신에서만).
  심각도마다 호출 줄이 다르다(CLAUDE.md §1-7).

ship_direction.py 는 rclpy 를 import 하므로 test_candidate_fresh.py 와 같은 ROS 스텁 + fake self 로
**실제 메서드**를 언바운드 호출한다. 시간은 모듈의 time 을 가짜 시계로 바꿔 제어한다.
"""

import os
import sys
import threading
import types

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
from ship_direction.failsafe import heading_missing, CandidateStaleLog  # noqa: E402

STALE = 1.0          # candidate_stale_sec 코드 기본값(=대회값)
REV_STALE = 0.5      # candidate_reverse_stale_sec 코드 기본값
OBST = object()      # _closest_obstacle 대용 센티널
SCAN = object()      # latest_scan 대용 센티널
YAW_TAG = '/yaw_error'   # 이 기능의 로그 문구에 들어가는 표식(candidate stale 로그와 구분)
AUTO = 77.0          # (C) 갭 선택 결과 대용 — 0~160 안 값이라 실제 레이트 리미터를 그대로 통과

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

    def yaw(self, sev):
        """이 기능(yaw_error 미수신) 로그만 센다."""
        return sum(1 for s, m in self.lines if s == sev and YAW_TAG in m)


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def _f32(data=None):
    return types.SimpleNamespace(data=data)


def with_clock(fn):
    """SD.time 을 가짜 시계로, SD.Float32 를 값 보존 메시지로 바꿔 fn(clock) 실행 후 되돌린다.
    (pytest 한 프로세스에선 먼저 import 한 다른 테스트의 스텁 Float32 가 모듈에 남아 있을 수 있다)"""
    real_t, real_f = SD.time, SD.Float32
    clk = _Clock()
    SD.time, SD.Float32 = clk, _f32
    try:
        return fn(clk)
    finally:
        SD.time, SD.Float32 = real_t, real_f


def fake_self(wp_mode=7, yaw_error=None):
    """control_cb 와 그 안에서 부르는 것(실제 메서드)이 읽는 것만. 기본은 부팅 직후(yaw_error None)."""
    log = _Logger()
    fs = types.SimpleNamespace(
        lock=threading.Lock(),
        failsafe_level=0,
        latest_scan=SCAN,
        watch=types.SimpleNamespace(fault=False, feed=lambda name, now: None),
        wp_mode=wp_mode,
        wp4_enter_time=None,
        yaw_error=yaw_error,
        detection_distance=3.0,
        detection_distance_default=3.0,
        detection_distance_gate=2.0,
        gate_wp_modes=[0, 1],
        candidate_angle=CANDIDATE_INVALID,
        candidate_t=None,
        candidate_stale_sec=STALE,
        candidate_reverse_stale_sec=REV_STALE,
        candidate_stale_log=CandidateStaleLog(CANDIDATE_STALE_LOG_PERIOD_SEC),
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
        rate_limit_max_deg_per_tick=15.0,
        pub_desired_angle=_Pub(),
        pub_obstacle_distance=_Pub(),
        get_logger=lambda: log,
        log=log,
        seen=[],                 # control_cb 가 _compute 에 넘긴 (candidate, yaw_error)
    )
    fs._closest_obstacle = lambda msg: OBST
    fs._reset_reverse_sequence = lambda: ShipDirection._reset_reverse_sequence(fs)
    fs._geofence_blocks = lambda rel: ShipDirection._geofence_blocks(fs, rel)
    fs._apply_rate_limit = lambda a: ShipDirection._apply_rate_limit(fs, a)
    fs._effective_candidate = (
        lambda c, ct, now, wm: ShipDirection._effective_candidate(fs, c, ct, now, wm))

    def _compute(msg, wp4, cand, yaw, dd, cand_t):
        fs.seen.append((cand, yaw))
        if cand == CANDIDATE_INVALID and yaw is not None:
            return AUTO, OBST            # (C) 갭 선택 — 스캔 계산은 이 테스트 범위 밖
        return ShipDirection._compute(fs, msg, wp4, cand, yaw, dd, cand_t)
    fs._compute = _compute
    return fs


def receive(fs, raw):
    ShipDirection.candidate_callback(fs, types.SimpleNamespace(data=raw))


def receive_yaw(fs, raw):
    ShipDirection.yaw_error_callback(fs, types.SimpleNamespace(data=raw))


def tick(fs):
    """control_cb 1회 → 발행한 desired (발행 없으면 None)."""
    n = len(fs.pub_desired_angle.msgs)
    ShipDirection.control_cb(fs)
    if len(fs.pub_desired_angle.msgs) == n:
        return None
    return fs.pub_desired_angle.msgs[-1].data


# ---------------- 순수 함수
def test_pure_heading_missing():
    inv = CANDIDATE_INVALID
    assert heading_missing(inv, None, inv) is True
    assert heading_missing(inv, 0.0, inv) is False, "0.0 은 정상값(정확히 정렬) — is None 이어야"
    assert heading_missing(inv, 123.4, inv) is False
    for c in (SPIN_RIGHT, SPIN_LEFT, 180.0, 30.0, STOP_HOLD, 0.0):
        assert heading_missing(c, None, inv) is False, f"candidate {c} 는 yaw_error 를 안 읽는다"


# ---------------- control_cb: 부팅 정지 + 로그
def test_boot_stop_and_log_sequence():
    def run(clk):
        fs = fake_self()
        out = []
        for _ in range(110):                                # 부팅 뒤 11 s, yaw_error 없음
            out.append(tick(fs))
            clk.t += 0.1
        assert all(d == STOP_HOLD for d in out), set(out)
        assert all(c == STOP_HOLD for c, _ in fs.seen), "control_cb 가 _compute 에 50000 을 넘겨야"
        assert fs.log.yaw('ERROR') == 3, fs.log.lines           # 0 s 진입 + 5 s·10 s 주기
        assert fs.log.yaw('INFO') == 0
        assert '50000' in fs.log.lines[0][1] and '20000' in fs.log.lines[0][1]
        receive_yaw(fs, 0.0)                                # 0.0 = 정확히 정렬(정상값)
        assert fs.yaw_error == 0.0
        d = tick(fs)
        assert d == AUTO and fs.seen[-1] == (CANDIDATE_INVALID, 0.0), (d, fs.seen[-1])
        assert fs.log.yaw('INFO') == 1 and fs.log.yaw('ERROR') == 3, fs.log.lines
        for _ in range(100):                                # 그 뒤엔 로그 없음, 계속 계산
            clk.t += 0.1
            assert tick(fs) == AUTO
        assert fs.log.yaw('INFO') == 1 and fs.log.yaw('ERROR') == 3
        # 호출 줄 3개(진입 ERROR·주기 ERROR·복구 INFO), 줄마다 심각도 하나(_Logger 가 검사)
        assert len(fs.log.sites) == 3, fs.log.sites
    with_clock(run)


def test_first_error_on_first_tick():
    def run(clk):
        fs = fake_self()
        t0 = clk.t
        assert tick(fs) == STOP_HOLD
        assert fs.log.yaw('ERROR') == 1
        clk.t = t0 + YAW_MISSING_LOG_PERIOD_SEC - 0.01
        tick(fs)
        assert fs.log.yaw('ERROR') == 1
        clk.t = t0 + YAW_MISSING_LOG_PERIOD_SEC
        tick(fs)
        assert fs.log.yaw('ERROR') == 2
    with_clock(run)


def test_zero_yaw_is_not_missing():
    """처음부터 0.0 을 받았으면 정지도 로그도 없다(`not yaw_error` 로 쓰면 여기서 영구 정지)."""
    def run(clk):
        fs = fake_self()
        receive_yaw(fs, 0.0)
        for _ in range(60):
            assert tick(fs) == AUTO
            clk.t += 0.1
        assert fs.log.lines == [], fs.log.lines
    with_clock(run)


def test_mission_values_pass_while_missing():
    """yaw_error None 이어도 임무 노드 값은 그대로 — 거짓 정지 없음, 로그 없음."""
    for raw, want in [(30.0, 110.0), (180.0, REVERSE_ANGLE), (SPIN_RIGHT, SPIN_RIGHT),
                      (SPIN_LEFT, SPIN_LEFT), (STOP_HOLD, STOP_HOLD)]:
        def run(clk, raw=raw):
            fs = fake_self()
            receive(fs, raw)
            clk.t += 0.1
            return tick(fs), fs
        d, fs = with_clock(run)
        assert d == want, f"{raw} → {d}, 기대 {want}"
        assert fs.log.yaw('ERROR') == 0, (raw, fs.log.lines)


def test_stale_candidate_becomes_stop():
    """1.1 s 무수신 → 20000 대체 → yaw_error 없으니 50000."""
    def run(clk):
        fs = fake_self()
        receive(fs, 30.0)
        clk.t += 1.1
        d = tick(fs)
        assert d == STOP_HOLD, d
        assert fs.log.yaw('ERROR') == 1
    with_clock(run)


def test_wp_mode_change_becomes_stop():
    def run(clk):
        fs = fake_self(wp_mode=7)
        receive(fs, SPIN_LEFT)
        assert tick(fs) == SPIN_LEFT
        ShipDirection.wp_mode_cb(fs, types.SimpleNamespace(data=8))   # → 20000·None 초기화
        assert tick(fs) == STOP_HOLD
        assert fs.log.yaw('ERROR') == 1
    with_clock(run)


def test_flicker_no_error_flood():
    """게이트처럼 후보각이 20000 ↔ 30.0 으로 깜빡여도 5 s 안 ERROR 는 1회, INFO 는 0회."""
    def run(clk):
        fs = fake_self()
        out = []
        for k in range(49):                                 # 4.9 s
            receive(fs, CANDIDATE_INVALID if k % 2 == 0 else 30.0)
            out.append(tick(fs))
            clk.t += 0.1
        assert out[0::2] == [STOP_HOLD] * len(out[0::2]), out
        assert out[1::2] == [110.0] * len(out[1::2]), out
        assert fs.log.yaw('ERROR') == 1, fs.log.lines
        assert fs.log.yaw('INFO') == 0, fs.log.lines
        # 5 s 를 넘긴 첫 '막힌' 틱에서 주기 ERROR
        receive(fs, CANDIDATE_INVALID)
        clk.t += 0.2
        tick(fs)
        assert fs.log.yaw('ERROR') == 2, fs.log.lines
    with_clock(run)


def test_level1_still_stops():
    """레벨 1(경고)은 [N] 앞에서 빠지지 않는다 → yaw None + 20000 이면 50000, ERROR 1회."""
    def run(clk):
        fs = fake_self()
        fs.failsafe_level = 1
        d = tick(fs)
        assert d == STOP_HOLD, d
        assert fs.seen == [(STOP_HOLD, None)], fs.seen
        assert fs.log.yaw('ERROR') == 1, fs.log.lines
    with_clock(run)


def test_first_yaw_on_angle_tick():
    """막힌 뒤 첫 yaw_error 가 후보각이 각도(30.0)인 틱에 들어와도 INFO 1회, 그 뒤 20000 은 안 막힘."""
    def run(clk):
        fs = fake_self()
        assert tick(fs) == STOP_HOLD                        # 20000 + None → 막힘, ERROR
        assert fs.log.yaw('ERROR') == 1
        clk.t += 0.1
        receive(fs, 30.0)
        receive_yaw(fs, 0.0)                                # 각도 틱에 첫 수신
        assert tick(fs) == 110.0
        assert fs.log.yaw('INFO') == 1, fs.log.lines
        out = []
        for _ in range(60):                                 # 6 s: 20000 으로 돌아와도 계산
            clk.t += 0.1
            receive(fs, CANDIDATE_INVALID)
            out.append(tick(fs))
            assert fs.seen[-1] == (CANDIDATE_INVALID, 0.0), fs.seen[-1]
        # 110 → 77 은 레이트 리미터(15°/틱)로 내려온다. 50000 은 한 번도 없다.
        assert STOP_HOLD not in out and out[-1] == AUTO, out[:5]
        assert fs.log.yaw('INFO') == 1 and fs.log.yaw('ERROR') == 1, fs.log.lines
    with_clock(run)


def test_level2_and_bad_scan_unchanged():
    """레벨 2·이상 스캔은 기존 경로(STOP_HOLD) 그대로 — 이 판정·로그보다 먼저 빠진다."""
    for attr in ('level', 'bad'):
        def run(clk, attr=attr):
            fs = fake_self()
            if attr == 'level':
                fs.failsafe_level = 2
            else:
                fs.watch.fault = True
            d = tick(fs)
            return d, fs
        d, fs = with_clock(run)
        assert d == STOP_HOLD, (attr, d)
        assert fs.seen == [], "레벨 2·bad 는 _compute 를 안 부른다"
        assert fs.log.yaw('ERROR') == 0, (attr, fs.log.lines)


def test_scan_none_no_publish():
    def run(clk):
        fs = fake_self()
        fs.latest_scan = None
        assert tick(fs) is None
        assert fs.pub_desired_angle.msgs == []
        assert fs.log.yaw('ERROR') == 0
    with_clock(run)


def test_stop_resets_reverse_and_rate_limiter():
    """50000 은 (0) 경로 → 후진 상태기계 READY, 레이트 리미터 상태 리셋."""
    def run(clk):
        fs = fake_self()
        fs.reverse_state = 'COOLDOWN'
        fs.reverse_cooldown_start_time = clk.t - 1.0
        fs.prev_final_angle = 100.0
        assert tick(fs) == STOP_HOLD
        assert fs.reverse_state == 'READY'
        assert fs.reverse_cooldown_start_time is None and fs.reverse_start_time is None
        assert fs.prev_final_angle is None
    with_clock(run)


# ---------------- _compute (C) 방어 가드
def test_compute_guard_direct():
    """control_cb 를 거치지 않고 _compute(20000, None) 을 불러도 예외 없이 50000."""
    def run(clk):
        fs = fake_self()
        fs.reverse_state = 'REVERSING'
        fs.reverse_start_time = clk.t
        angle, obst = ShipDirection._compute(fs, None, None, CANDIDATE_INVALID, None, 3.0, None)
        assert angle == STOP_HOLD and obst is OBST, angle
        assert fs.reverse_state == 'READY'
    with_clock(run)


# ---------------- __init__ 초기값
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


def test_initial_yaw_error_none():
    node, _ = _construct({})
    assert node.yaw_error is None, node.yaw_error
    assert isinstance(node.yaw_missing_log, CandidateStaleLog)
    assert node.yaw_missing_log.period_sec == YAW_MISSING_LOG_PERIOD_SEC == 5.0


def main():
    print("=== yaw_error 미수신 정지 테스트 (계약서_yaw미수신정지_20261105) ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())

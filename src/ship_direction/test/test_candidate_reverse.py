#!/usr/bin/env python3
"""(B) candidate 후진(180.0) 통과 테스트 — _compute / candidate_callback / _apply_rate_limit.
실행: python3 src/ship_direction/test/test_candidate_reverse.py

계약서 docs/문제와작업/계약서_후진통과_20261104.md §7 + §10-4.
규약: /candidate_angle **180.0 정확 일치** = 후진 요청 → /desired_angle 260(REVERSE_ANGLE, 분기 5).
  단 수신 후 candidate_reverse_stale_sec(0.5 s) 넘게 갱신이 없으면 STOP_HOLD(묵은 180 무한 후진 방지).
  그 밖의 값은 수정 전과 같다(범위 0~160 밖이면 STOP_HOLD, 신선도 무관).

ship_direction.py 는 rclpy 를 import 하므로 Mac(ROS 없음)에선 그대로 못 읽는다.
→ test_geofence_merge.py 와 같은 ROS 스텁 + fake self 로 **실제 메서드**를 언바운드 호출한다.
시간은 모듈의 time 을 가짜 시계로 바꿔 제어한다(_Clock).
"""

import os
import struct
import sys
import threading
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

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
    return None


_stub('std_msgs', msg=None)
_stub('std_msgs.msg', Int32=_msg, Float32=_msg, Float32MultiArray=_msg)
_stub('sensor_msgs', msg=None)
_stub('sensor_msgs.msg', LaserScan=_msg)

import ship_direction.ship_direction as SD  # noqa: E402
from ship_direction.ship_direction import (  # noqa: E402
    ShipDirection, REVERSE_ANGLE, REVERSE_REL, STOP_HOLD, SPIN_RIGHT, SPIN_LEFT)

INF = float('inf')
N_RAYS = 161                                   # geofence rel −80..80 / 1°
OBST = object()                                # _closest_obstacle 대용 센티널
STALE = 0.5                                    # candidate_reverse_stale_sec 코드 기본값(=대회값)

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
    """ship_direction 모듈의 time 대용. monotonic() 만 쓴다."""

    def __init__(self, t=1000.0):
        self.t = t

    def monotonic(self):
        return self.t


class _Logger:
    def __init__(self):
        self.lines = []

    def warn(self, msg, **kw):
        self.lines.append(msg)

    info = error = warn


def fake_self(gf=None, cone=60.0):
    """_compute (0)·[I]·(A)·(B) 와 candidate_callback·_apply_rate_limit 가 읽는 것만 채운 가짜 self.
    파라미터는 코드 기본값(=대회값). 시계는 SD.time 을 _Clock 으로 바꿔 쓴다(with_clock)."""
    log = _Logger()
    fs = types.SimpleNamespace(
        lock=threading.Lock(),
        geofence=gf,
        last_geofence_t=None,
        geofence_stale_sec=2.0,
        geofence_stop_margin_m=2.0,
        geofence_stop_cone_deg=cone,
        candidate_reverse_stale_sec=STALE,
        candidate_angle=SD.CANDIDATE_INVALID,
        candidate_t=None,
        reverse_state='READY',
        reverse_start_time=None,
        reverse_cooldown_start_time=None,
        prev_final_angle=None,
        rate_limit_max_deg_per_tick=15.0,
        get_logger=lambda: log,
        log=log,
    )
    fs._closest_obstacle = lambda msg: OBST
    fs._reset_reverse_sequence = lambda: ShipDirection._reset_reverse_sequence(fs)
    fs._geofence_blocks = lambda rel: ShipDirection._geofence_blocks(fs, rel)
    return fs


def with_clock(fn):
    """SD.time 을 가짜 시계로 바꿔 fn(clock) 실행 후 되돌린다."""
    real = SD.time
    clk = _Clock()
    SD.time = clk
    try:
        return fn(clk)
    finally:
        SD.time = real


def compute(fs, cand, age=0.0, wp4_age=None):
    """수신 후 age 초 지난 candidate 로 _compute. age=None 이면 수신 기록 없음. → desired."""
    def run(clk):
        cand_t = None if age is None else clk.t - age
        wp4 = None if wp4_age is None else clk.t - wp4_age
        if fs.geofence is not None:
            fs.last_geofence_t = clk.t
        angle, obst = ShipDirection._compute(fs, None, wp4, cand, 0.0, 3.0, cand_t)
        assert obst is OBST
        return angle
    return with_clock(run)


def via_callback(fs, raw, age=0.0):
    """candidate_callback(raw) 수신 → age 초 뒤 _compute. 수신 시각은 콜백이 기록한 것을 쓴다."""
    def run(clk):
        ShipDirection.candidate_callback(fs, types.SimpleNamespace(data=raw))
        assert fs.candidate_t == clk.t, f"candidate_t={fs.candidate_t} — 콜백이 수신 시각을 안 남겼다"
        clk.t += age
        angle, _ = ShipDirection._compute(
            fs, None, None, fs.candidate_angle, 0.0, 3.0, fs.candidate_t)
        return angle
    return with_clock(run)


def gf_wall(idx, dist):
    gr = [INF] * N_RAYS
    gr[idx] = dist
    return (-80.0, 1.0, gr)


# ---------------- 180.0 → 260
def test_reverse_180_maps_to_260():
    assert REVERSE_REL == 180.0 and REVERSE_ANGLE == 260.0
    assert compute(fake_self(), 180.0) == REVERSE_ANGLE


def test_reverse_180_is_exact_in_float32():
    """Float32 왕복으로도 180.0 은 정확히 180.0 (정확 일치 비교의 전제)."""
    f32 = struct.unpack('f', struct.pack('f', 180.0))[0]
    assert f32 == REVERSE_REL
    assert compute(fake_self(), f32) == REVERSE_ANGLE


def test_reverse_via_callback_wraps():
    """candidate_callback(−180)·(540)·(180) → val % 360 = 180 → 260."""
    for raw in (-180.0, 540.0, 180.0):
        fs = fake_self()
        a = via_callback(fs, raw)
        assert a == REVERSE_ANGLE, f"raw={raw} → {a}"


# ---------------- 경계 (정확 일치만 예외, 나머지는 수정 전 동작)
def test_boundaries_unchanged():
    cases = [(179.9, STOP_HOLD), (180.1, STOP_HOLD), (80.0, 160.0), (80.1, STOP_HOLD),
             (280.0, 0.0), (0.0, 80.0), (30.0, 110.0), (330.0, 50.0)]
    for c, want in cases:
        got = compute(fake_self(), c)
        assert abs(got - want) < 1e-9, f"candidate {c} → {got}, 기대 {want}"


# ---------------- 특수값
def test_special_values_unchanged():
    assert compute(fake_self(), SPIN_RIGHT) == SPIN_RIGHT
    assert compute(fake_self(), SPIN_LEFT) == SPIN_LEFT
    assert compute(fake_self(), STOP_HOLD) == STOP_HOLD


def test_wp4_first_5s_overrides_reverse():
    """WP4 진입 5 s 안이면 (0) 이 먼저 → 180 이어도 50000."""
    assert compute(fake_self(), 180.0, wp4_age=1.0) == STOP_HOLD
    assert compute(fake_self(), 180.0, wp4_age=6.0) == REVERSE_ANGLE


# ---------------- [I] geofence 가드 (순서 불변: [I] 가 (B) 보다 먼저)
def test_geofence_front_wall_does_not_block_reverse():
    """정면(rel 0) 1 m 경계 + 180 → diff 180 > cone 60 → 가드 안 걸림 → 260."""
    assert compute(fake_self(gf=gf_wall(80, 1.0)), 180.0) == REVERSE_ANGLE


def test_geofence_wide_cone_side_wall_blocks_reverse_first():
    """cone 120 + 옆 경계 rel −80(인덱스 0) 1 m + 180 → diff 100 ≤ 120 → [I] 가 먼저 STOP_HOLD."""
    assert compute(fake_self(gf=gf_wall(0, 1.0), cone=120.0), 180.0) == STOP_HOLD


def test_geofence_blocks_normal_candidate():
    """일반 candidate(정면 0) + 정면 1 m 경계 → STOP_HOLD (기존 동작)."""
    assert compute(fake_self(gf=gf_wall(80, 1.0)), 0.0) == STOP_HOLD


# ---------------- [K] rate limiter
def test_rate_limiter_passes_260_and_resets():
    fs = fake_self()
    assert ShipDirection._apply_rate_limit(fs, 80.0) == 80.0
    assert ShipDirection._apply_rate_limit(fs, REVERSE_ANGLE) == REVERSE_ANGLE
    assert fs.prev_final_angle is None, fs.prev_final_angle
    assert ShipDirection._apply_rate_limit(fs, 30.0) == 30.0, "260 다음 전진은 램프 없이 즉시"


# ---------------- [L] 상태기계
def test_reverse_candidate_resets_L_state():
    fs = fake_self()
    fs.reverse_state = 'COOLDOWN'
    fs.reverse_cooldown_start_time = 0.0
    assert compute(fs, 180.0) == REVERSE_ANGLE
    assert fs.reverse_state == 'READY', fs.reverse_state


# ---------------- §10-4 신선도
def test_fresh_reverse_is_260():
    assert compute(fake_self(), 180.0, age=0.0) == REVERSE_ANGLE
    assert compute(fake_self(), 180.0, age=0.4) == REVERSE_ANGLE
    assert compute(fake_self(), 180.0, age=STALE) == REVERSE_ANGLE     # 경계 포함


def test_stale_reverse_is_stop():
    fs = fake_self()
    assert compute(fs, 180.0, age=0.6) == STOP_HOLD
    assert any('180' in ln for ln in fs.log.lines), "stale 후진 warn 이 없다"
    assert compute(fake_self(), 180.0, age=None) == STOP_HOLD, "수신 기록 없음 = 모름 → 정지"


def test_stale_reverse_via_callback():
    """콜백 수신 후 0.6 s 갱신 없음 → 50000, 0.3 s 면 260."""
    assert via_callback(fake_self(), 180.0, age=0.6) == STOP_HOLD
    assert via_callback(fake_self(), 180.0, age=0.3) == REVERSE_ANGLE


def test_other_values_ignore_staleness():
    """다른 candidate 값은 수신 시각과 무관(수정 전과 같음)."""
    for c, want in [(30.0, 110.0), (280.0, 0.0), (179.9, STOP_HOLD),
                    (SPIN_RIGHT, SPIN_RIGHT), (STOP_HOLD, STOP_HOLD)]:
        for age in (0.0, 10.0, None):
            got = compute(fake_self(), c, age=age)
            assert abs(got - want) < 1e-9, f"candidate {c} age {age} → {got}, 기대 {want}"


def main():
    print("=== candidate 후진(180) 통과 테스트 (계약서_후진통과_20261104) ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())

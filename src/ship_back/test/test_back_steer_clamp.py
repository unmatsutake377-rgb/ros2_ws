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


DECLARED = {}    # 마지막 노드가 선언한 (이름 → 코드 기본값)


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


# launch 가 넘기는 값으로 덮는 파라미터. 이 파일의 mode 3 시나리오(3→3 래치 등)는 search_behavior:=hold
#   옵션 경로다 — 2026-11-11 기본이 orbit(코드 기본 [2])으로 바뀌어, hold 때 launch 가 넘기는 [2, 3] 을 준다
#   (계약서_탐색선회_부표색_20261111 C3). DECLARED 에는 코드 기본값을 그대로 기록한다(yaml 대조용).
PARAM_OVERRIDES = {'active_wp_modes': [2, 3], 'hold_source': 'vision'}  # vision 경로 고정(11-13 기본 gps)


class _Node:
    """파라미터는 코드 기본값(= config/ship_back.yaml 과 같게 유지되는 값) — PARAM_OVERRIDES 만 예외."""

    def __init__(self, name):
        self._logger = _Logger()
        self.pubs = []

    def declare_parameter(self, name, default):
        DECLARED[name] = default
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
        'std_msgs.msg': dict(Float32=_Float32, Int32=_Float32, Float32MultiArray=_Float32),
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


def clamp_warns(n):
    """클램프 WARN 만. 기본 distance_source=fused(11-14 G-9)라 /fused_objects 미수신 WARN(5 s 주기)이
    같이 찍힌다 — 그건 test_fused_match 가 본다."""
    return [ln for ln in warns(n) if '/fused_objects' not in ln[1]]


def test_warn_only_when_clamped():
    n, _ = tick(180.0)
    w = clamp_warns(n)
    assert len(w) == 1 and w[0][2].get('throttle_duration_sec') == 1.0, f"warns={w}"
    for a in (80.0, -80.0, 30.0):
        n, _ = tick(a)
        assert clamp_warns(n) == [], f"{a}: warns={clamp_warns(n)}"


# ---------------- 다른 분기는 그대로
def test_reverse_exact_180():
    """dist 1.5 < 2.5 − 0.4 → 후진 180.0 그대로."""
    _, out = tick(0.0, dist=1.5)
    assert _one(out) == 180.0, f"out={out}"


def test_deadband_stop():
    _, out = tick(0.0, dist=2.5)
    assert _one(out) == 50000.0, f"out={out}"


def test_lidar_fallback_pm30_unchanged():
    """HOLD 중 비전이 1.5 s 전에 끊김(< lost_sec 2.0) → LiDAR 전방 ±30 최근접 보조.

    +30(LiDAR 110°) → 30, −30(LiDAR 50°) → 330 — 클램프·매핑 기대값은 그대로.
    계약 전체코스 §2-5 로 바뀐 점: LiDAR 는 'HOLD 유지 중 직전 비전이 lost_sec 안'일 때만 보조하고,
    last_seen_t 를 갱신하지 않는다(갱신하면 전방에 무엇이든 있는 한 HOLD 가 안 끝난다).
    수정 전에는 비전과 무관하게 LiDAR 가 last_seen_t 를 now 로 당겼다.
    """
    for obj, want in ((110.0, 30.0), (50.0, 330.0)):
        n = make_hold(None, dist=3.5, obj=obj)
        n.last_seen_t = CLOCK.t - 1.5
        n.timer_cb()
        out = n.pub_candidate.values
        assert abs(_one(out) - want) < 1e-6, f"LiDAR {obj}° → {out} (기대 {want})"
        assert n.state == 'HOLD' and n.last_seen_t == CLOCK.t - 1.5, (n.state, n.last_seen_t)


def test_lidar_aid_ends_after_lost_sec():
    """직전 비전이 lost_sec 넘게 지났으면 LiDAR 가 앞에 있어도 HOLD → SEARCH(이 틱 발행 없음).

    수정 전에는 LiDAR 최근접이 last_seen_t 를 갱신해 HOLD 가 유지되고 30 을 발행했다.
    """
    n = make_hold(None, dist=3.5, obj=110.0)
    n.last_seen_t = CLOCK.t - 2.1
    n.timer_cb()
    assert n.state == 'SEARCH' and n.pub_candidate.values == [], (n.state, n.pub_candidate.values)


def test_lidar_match_uses_unclamped_bearing():
    """이미지 90 + 물체 LiDAR 170°(= 80 + 90) → 거리를 찾아 전진, 조향만 80.

    buoy_rel 을 80 으로 잘랐다면 LiDAR 160±3° 를 봐서 거리를 못 찾고 STOP_HOLD 였다.
    """
    _, out = tick(90.0, dist=3.5, obj=170.0)
    assert _one(out) == 80.0, f"out={out}"


def test_nan_guard_kept():
    """NaN 은 angle_cb 에서 걸러져 buoy_rel=None → LiDAR 보조(전 방위 3.5 → 전방 최근접 −30).

    make_hold 는 last_seen_t = now(방금 비전으로 봄)라 §2-5 보조 창 안 — 기대값 330 그대로.
    """
    n = make_hold(float('nan'))
    assert n.buoy_rel is None
    n.timer_cb()
    p = _one(n.pub_candidate.values)
    assert abs(p - 330.0) < 1e-6, f"out={p} (NaN 이 조향으로 새면 80.0)"


# ---------------- 계약 전체코스 §2-4·§2-5 (C5): 획득은 비전 색 확인으로만, SEARCH 타임아웃 비래치
def make_search(elapsed=1.5, scan_dist=3.0):
    """SEARCH 에 elapsed 초 머문 노드. 비전 없음, LiDAR 전 방위에 scan_dist(벽·다른 색 부표)."""
    n = SB.ShipBack()
    n.wp_mode = n.active_wp_modes[-1]          # 3 = 탐색
    n.state = 'SEARCH'
    n.state_t = CLOCK.t - elapsed
    n.latest_scan = _Scan(scan_dist)
    n.t_scan = CLOCK.t
    return n


def test_switch_defaults():
    n = SB.ShipBack()
    assert n.search_timeout_ends is False and n.lidar_acquire is False
    assert n.search_timeout_sec == 20.0


def test_code_defaults_equal_yaml():
    """코드 기본값 = config/ship_back.yaml.

    대회 launch 는 ship_back 에 [이 yaml, search_behavior dict(맨 뒤)] 를 넘긴다(11-05).
    """
    import yaml
    SB.ShipBack()
    with open(os.path.join(PKG, 'config', 'ship_back.yaml'), encoding='utf-8') as f:
        y = yaml.safe_load(f)['/ship_back']['ros__parameters']
    assert y['search_timeout_ends'] is False and y['lidar_acquire'] is False
    assert set(DECLARED) == set(y), (set(DECLARED) ^ set(y))
    diff = {k: (DECLARED[k], y[k]) for k in y if DECLARED[k] != y[k]}
    assert not diff, diff


def test_search_no_vision_lidar_wall_stays_search():
    """비전 없이 전방 벽(LiDAR 3 m) → SEARCH 유지·20000. 수정 전에는 LiDAR 최근접으로 HOLD 획득."""
    n = make_search()
    n.timer_cb()
    assert n.state == 'SEARCH' and _one(n.pub_candidate.values) == 20000.0, \
        (n.state, n.pub_candidate.values)
    assert n.last_seen_t is None


def test_search_vision_acquires_hold():
    n = make_search()
    n.angle_cb(_Float32(10.0))
    n.timer_cb()
    assert n.state == 'HOLD' and n.last_seen_t == CLOCK.t, (n.state, n.last_seen_t)


def test_search_timeout_not_done_by_default():
    """SEARCH 25 s(> 20) → DONE 래치하지 않고 SEARCH·20000. 수정 전에는 DONE."""
    n = make_search(elapsed=25.0)
    n.timer_cb()
    assert n.state == 'SEARCH' and _one(n.pub_candidate.values) == 20000.0, \
        (n.state, n.pub_candidate.values)
    n2 = make_search(elapsed=25.0)
    n2.angle_cb(_Float32(0.0))                 # 늦게라도 보이면 획득
    n2.timer_cb()
    assert n2.state == 'HOLD', n2.state


def test_search_timeout_warn_once_per_entry_no_latch():
    """search_timeout_ends=false: 20 s 넘으면 WARN 1회, 상태·발행 유지. 다시 SEARCH 진입하면 또 1회."""
    n = make_search(elapsed=19.9)
    n.timer_cb()
    assert warns(n) == []
    t0 = CLOCK.t
    try:
        for k in range(1, 6):                     # 20.0 ~ 24 s 지점
            CLOCK.t = t0 + k
            n.t_scan = CLOCK.t
            n.timer_cb()
        w = [ln for ln in warns(n) if '부표 못 찾음' in ln[1]]
        assert len(w) == 1 and '계속 탐색(20000)' in w[0][1], warns(n)
        assert n.state == 'SEARCH' and set(n.pub_candidate.values) == {20000.0}
        CLOCK.t = t0 + 30.0
        n.set_state('HOLD')
        n.set_state('SEARCH')                     # 재진입 → 다시 셈
        n.t_scan = CLOCK.t
        n.timer_cb()
        assert len([ln for ln in warns(n) if '부표 못 찾음' in ln[1]]) == 1   # 0 s 경과
        CLOCK.t = t0 + 50.1
        n.t_scan = CLOCK.t
        n.timer_cb()
        assert len([ln for ln in warns(n) if '부표 못 찾음' in ln[1]]) == 2
    finally:
        CLOCK.t = t0


def test_search_same_mode_3_to_3_keeps_search_then_vision_holds():
    """WP3 실패 → WP4(같은 mode 3) 계속을 노드 단위로 고정.

    /wp_mode 3 재수신은 리셋 없이 SEARCH·20000 유지, 그 뒤 비전(빨강)이 오면 HOLD.
    """
    n = make_search(elapsed=25.0)
    n.timer_cb()
    n.wp_cb(_Float32(3))
    assert n.state == 'SEARCH'
    n.timer_cb()
    assert n.state == 'SEARCH' and n.pub_candidate.values[-1] == 20000.0
    n.angle_cb(_Float32(5.0))
    n.timer_cb()
    assert n.state == 'HOLD', n.state


def test_search_timeout_switch_true_old_done():
    n = make_search(elapsed=25.0)
    n.search_timeout_ends = True
    n.timer_cb()
    assert n.state == 'DONE', n.state
    assert any('SEARCH 타임아웃' in ln[1] for ln in warns(n)), warns(n)


def test_lidar_acquire_true_old_behavior():
    """lidar_acquire=true: 비전 없이 LiDAR 최근접으로 SEARCH→HOLD, HOLD 에서 last_seen 갱신."""
    n = make_search()
    n.lidar_acquire = True
    n.timer_cb()
    assert n.state == 'HOLD' and n.last_seen_t == CLOCK.t, (n.state, n.last_seen_t)
    h = make_hold(None, dist=3.5, obj=110.0)
    h.lidar_acquire = True
    h.last_seen_t = CLOCK.t - 5.0
    h.timer_cb()
    assert h.state == 'HOLD' and _one(h.pub_candidate.values) == 30.0 \
        and h.last_seen_t == CLOCK.t, (h.state, h.pub_candidate.values)


def test_hold_vision_refreshes_last_seen():
    n = make_hold(0.0, dist=3.5)
    n.last_seen_t = CLOCK.t - 1.9
    n.timer_cb()
    assert n.last_seen_t == CLOCK.t and _one(n.pub_candidate.values) == 0.0


def test_done_only_from_hold_and_3_to_3_keeps_latch():
    """HOLD 성공(5 m 안 5 s) → DONE. 같은 모드 3→3 재수신은 리셋 안 함(DONE·20000 유지)."""
    n = make_hold(0.0, dist=2.5)
    n.wp_mode = 3
    n.hold_timer.update(2.5, CLOCK.t - 5.0)
    n.timer_cb()
    assert n.state == 'DONE', n.state
    n.wp_cb(_Float32(3))
    n.timer_cb()
    assert n.state == 'DONE' and n.pub_candidate.values[-1] == 20000.0


# ---------------- 계약 전체코스 §10-추가 결정 4: HOLD 거리 unknown 이 unknown_giveup_sec 이면 포기
def errors(n):
    return [ln for ln in n.get_logger().lines if ln[0] == 'ERROR']


def _step(n, t, lidar_sees):
    """시각 t 로 옮겨 비전(정면 0°)을 주고 한 틱. lidar_sees=False 면 비전 방위 ±3° 에 LiDAR 없음."""
    CLOCK.t = t
    n.angle_cb(_Float32(0.0))
    n.latest_scan = _Scan(3.5, obj=80.0 if lidar_sees else 170.0)   # 80 = 정면, 170 = 좌 90°
    n.t_scan = t
    n.timer_cb()
    return n.pub_candidate.values[-1] if n.pub_candidate.values else None


def _unknown_hold():
    n = make_hold(0.0, dist=3.5, obj=170.0)
    n.wp_mode = 3
    return n, CLOCK.t


def test_unknown_giveup_default_and_yaml():
    assert SB.ShipBack().unknown_giveup_sec == 10.0


def test_unknown_giveup_boundary_9_9_vs_10_0():
    n, t0 = _unknown_hold()
    try:
        assert _step(n, t0, False) == 50000.0 and n.state == 'HOLD'
        assert _step(n, t0 + 9.9, False) == 50000.0 and n.state == 'HOLD', n.state
        assert errors(n) == []
        _step(n, t0 + 10.0, False)
        assert n.state == 'DONE', n.state
        e = errors(n)
        assert len(e) == 1 and 'LiDAR 거리 없음' in e[0][1] and '포기' in e[0][1], e
    finally:
        CLOCK.t = t0


def test_unknown_giveup_done_persists_20000_and_3_to_3_latch():
    n, t0 = _unknown_hold()
    try:
        _step(n, t0, False)
        _step(n, t0 + 10.0, False)
        assert n.state == 'DONE'
        for k in range(1, 4):
            assert _step(n, t0 + 10.0 + k, True) == 20000.0 and n.state == 'DONE'
        n.wp_cb(_Float32(3))                      # 같은 모드 3→3 재수신 → 리셋 안 함
        assert _step(n, t0 + 15.0, True) == 20000.0 and n.state == 'DONE'
        assert len(errors(n)) == 1, errors(n)     # ERROR 는 포기 전이에서 1회
    finally:
        CLOCK.t = t0


def test_unknown_timer_resets_when_distance_returns():
    n, t0 = _unknown_hold()
    try:
        _step(n, t0, False)
        _step(n, t0 + 6.0, False)
        assert _step(n, t0 + 6.1, True) == 0.0    # 거리 3.5 m 잡힘 → 전진, 타이머 리셋
        assert n.unknown_t0 is None
        _step(n, t0 + 6.2, False)                 # unknown 다시 시작
        assert _step(n, t0 + 16.1, False) == 50000.0 and n.state == 'HOLD', n.state   # 9.9 s
        _step(n, t0 + 16.3, False)                # 10.1 s (부동소수 경계 피함 — 경계는 위 테스트)
        assert n.state == 'DONE' and len(errors(n)) == 1
    finally:
        CLOCK.t = t0


# ---------------- 계약 전체코스 §10-추가 결정 5: 한 WP 의 HOLD 총시간 hold_max_sec 이면 포기
def _hstep(n, t, dist):
    """시각 t, 비전 정면 0°, LiDAR 전 방위 dist 로 한 틱. 마지막 발행값."""
    CLOCK.t = t
    n.angle_cb(_Float32(0.0))
    n.latest_scan = _Scan(dist)
    n.t_scan = t
    n.timer_cb()
    return n.pub_candidate.values[-1] if n.pub_candidate.values else None


def _hold_node(mode=3):
    """담당 모드 진입(wp_cb) → set_state('HOLD') 로 HOLD 구간 시작. (노드, 시작 시각)."""
    n = SB.ShipBack()
    n.wp_cb(_Float32(mode))
    n.set_state('HOLD')
    return n, CLOCK.t


def _max_errors(n):
    return [ln for ln in errors(n) if 'HOLD 총' in ln[1]]


def test_hold_max_default():
    assert SB.ShipBack().hold_max_sec == 60.0


def test_hold_max_boundary_and_done_persists():
    """6 m(5 m 밖 → 성공 못 함, 거리 있음 → unknown 아님): 59.9 s HOLD, 60.0 s DONE·ERROR 1회."""
    n, t0 = _hold_node()
    try:
        assert _hstep(n, t0 + 59.9, 6.0) == 0.0 and n.state == 'HOLD'
        assert _max_errors(n) == []
        _hstep(n, t0 + 60.0, 6.0)
        assert n.state == 'DONE', n.state
        e = _max_errors(n)
        assert len(e) == 1 and '60 s 넘음' in e[0][1] and '포기' in e[0][1], e
        for k in range(1, 4):
            assert _hstep(n, t0 + 60.0 + k, 2.5) == 20000.0 and n.state == 'DONE'
        n.wp_cb(_Float32(3))                      # 3→3 래치 유지
        assert _hstep(n, t0 + 70.0, 2.5) == 20000.0 and n.state == 'DONE'
        assert len(errors(n)) == 1, errors(n)
    finally:
        CLOCK.t = t0


def test_hold_max_sums_hold_search_hold():
    n, t0 = _hold_node()
    try:
        CLOCK.t = t0 + 30.0
        n.set_state('SEARCH')                     # 30 s HOLD 끝
        CLOCK.t = t0 + 40.0
        n.set_state('HOLD')                       # 다시 HOLD (SEARCH 10 s 는 안 셈)
        assert _hstep(n, t0 + 69.9, 6.0) == 0.0 and n.state == 'HOLD'   # 30 + 29.9
        _hstep(n, t0 + 70.0, 6.0)                 # 30 + 30
        assert n.state == 'DONE' and len(_max_errors(n)) == 1
    finally:
        CLOCK.t = t0


def test_hold_max_reset_only_on_mode_entry():
    n, t0 = _hold_node(mode=2)
    try:
        CLOCK.t = t0 + 50.0
        n.set_state('SEARCH')                     # 이번 WP HOLD 합 50 s
        n.wp_cb(_Float32(3))                      # 담당 모드 진입(2→3) → 0
        assert n.hold_total == 0.0 and n.hold_enter_t is None
        n.set_state('HOLD')
        assert _hstep(n, t0 + 109.9, 6.0) == 0.0 and n.state == 'HOLD'  # 새 WP 59.9 s
        assert _max_errors(n) == []
    finally:
        CLOCK.t = t0


def test_hold_max_normal_success_unaffected():
    """정상 성공(5 m 안 5 s) — 상한과 같은 틱(앞서 55 s + 5 s)이어도 성공이 우선, ERROR 0."""
    n, t0 = _hold_node()
    try:
        n.hold_total = 55.0
        _hstep(n, t0, 2.5)
        assert n.state == 'HOLD'
        _hstep(n, t0 + 5.0, 2.5)
        assert n.state == 'DONE' and errors(n) == [], (n.state, errors(n))
    finally:
        CLOCK.t = t0


# ---------------- 계약 전체코스 §10-추가 결정 6: HOLD 에서 놓치면 reacquire_stop_sec 동안 정지
def _nstep(n, t, scan=None):
    """시각 t 로 옮겨 비전 없음(10000)·LiDAR scan(기본 전 방위 6 m)으로 한 틱. 마지막 발행값."""
    CLOCK.t = t
    n.angle_cb(_Float32(10000.0))
    n.latest_scan = scan if scan is not None else _Scan(6.0)
    n.t_scan = t
    n.timer_cb()
    return n.pub_candidate.values[-1] if n.pub_candidate.values else None


def _lost_warns(n):
    return [ln for ln in warns(n) if '부표 놓침' in ln[1]]


def _fail_warns(n):
    return [ln for ln in warns(n) if '재획득 실패' in ln[1]]


def _hold_then_lost(t0):
    """t0 HOLD 시작, t0+10 마지막 비전(6 m → 전진), t0+12.5 lost → SEARCH(창 시작). 노드."""
    n, _ = _hold_node()
    assert _hstep(n, t0 + 10.0, 6.0) == 0.0 and n.state == 'HOLD'
    k = len(n.pub_candidate.values)
    _nstep(n, t0 + 12.5)
    assert n.state == 'SEARCH' and len(n.pub_candidate.values) == k   # 전이 틱은 발행 없음
    assert n.reacquire_t0 == t0 + 12.5
    return n


def test_reacquire_stop_default_and_yaml():
    import yaml
    assert SB.ShipBack().reacquire_stop_sec == 5.0
    with open(os.path.join(PKG, 'config', 'ship_back.yaml'), encoding='utf-8') as f:
        y = yaml.safe_load(f)['/ship_back']['ros__parameters']
    assert y['reacquire_stop_sec'] == 5.0 and isinstance(y['reacquire_stop_sec'], float)
    assert isinstance(DECLARED['reacquire_stop_sec'], float)
    n = SB.ShipBack()
    assert any('reacquire_stop=5.0s' in ln[1] for ln in n.get_logger().lines
               if ln[0] == 'INFO'), n.get_logger().lines


def test_reacquire_window_stop_then_vision_holds_sum_continues():
    """HOLD 12.5 s → lost → 창 4 s 50000 → 재획득 HOLD(합 12.5 + 창 4 = 16.5) → 이어서 60 s 에 포기."""
    t0 = CLOCK.t
    try:
        n = _hold_then_lost(t0)
        assert len(_lost_warns(n)) == 1 and '5s 정지 후 재탐색' in _lost_warns(n)[0][1]
        assert abs(n.hold_total - 12.5) < 1e-9 and n.hold_enter_t is None
        assert _nstep(n, t0 + 13.0) == 50000.0 and n.state == 'SEARCH'
        assert _nstep(n, t0 + 16.0) == 50000.0 and n.state == 'SEARCH'
        assert abs(n.hold_total - 12.5) < 1e-9          # 창 중엔 HOLD 아님 → 합 그대로
        _hstep(n, t0 + 16.5, 6.0)                       # 창 안 재획득
        assert n.state == 'HOLD' and n.reacquire_t0 is None
        assert abs(n.hold_total - 16.5) < 1e-9          # 0 으로 안 돌아감 + 창 4 s 합산(보충 ①)
        assert _hstep(n, t0 + 16.5 + 43.4, 6.0) == 0.0 and n.state == 'HOLD'   # 59.9 s
        _hstep(n, t0 + 16.5 + 43.6, 6.0)                                        # 60.1 s
        assert n.state == 'DONE' and len(_max_errors(n)) == 1
        assert len(_lost_warns(n)) == 1 and _fail_warns(n) == []
    finally:
        CLOCK.t = t0


def test_reacquire_window_end_20000_and_warn_once():
    """창 4.9 s 정지, 5.0 s 부터 20000(SEARCH 유지)·WARN 1회."""
    t0 = CLOCK.t
    try:
        n = _hold_then_lost(t0)
        assert _nstep(n, t0 + 17.4) == 50000.0 and _fail_warns(n) == []    # 4.9 s
        assert _nstep(n, t0 + 17.5) == 20000.0 and n.state == 'SEARCH'      # 5.0 s
        w = _fail_warns(n)
        assert len(w) == 1 and '탐색 계속(20000)' in w[0][1], warns(n)
        assert n.reacquire_t0 is None
        for k in range(1, 4):
            assert _nstep(n, t0 + 17.5 + k) == 20000.0 and n.state == 'SEARCH'
        assert len(_fail_warns(n)) == 1 and len(_lost_warns(n)) == 1
    finally:
        CLOCK.t = t0


def test_reacquire_search_timeout_warn_counts_window():
    """SEARCH 20 s WARN 은 SEARCH 진입(lost)부터 — 창 5 s 포함. 상태·발행은 안 바뀜."""
    t0 = CLOCK.t
    try:
        n = _hold_then_lost(t0)
        assert _nstep(n, t0 + 32.4) == 20000.0                  # SEARCH 19.9 s
        assert [ln for ln in warns(n) if '부표 못 찾음' in ln[1]] == []
        assert _nstep(n, t0 + 32.6) == 20000.0                  # 20.1 s (창 끝 뒤 15.1 s)
        assert len([ln for ln in warns(n) if '부표 못 찾음' in ln[1]]) == 1
        assert n.state == 'SEARCH'
    finally:
        CLOCK.t = t0


def test_reacquire_first_search_from_init_is_20000():
    """INIT → SEARCH(모드 진입 뒤 첫 SEARCH)는 창 없음 — 처음부터 20000, 놓침 WARN 없음."""
    t0 = CLOCK.t
    try:
        n = SB.ShipBack()
        n.wp_cb(_Float32(3))
        assert _nstep(n, t0) == 20000.0 and n.state == 'INIT'
        _nstep(n, t0 + 1.0)
        assert n.state == 'SEARCH' and n.reacquire_t0 is None
        for k in (1.1, 2.0, 5.0):
            assert _nstep(n, t0 + k) == 20000.0 and n.state == 'SEARCH'
        assert set(n.pub_candidate.values) == {20000.0}
        assert _lost_warns(n) == [] and _fail_warns(n) == []
    finally:
        CLOCK.t = t0


def test_reacquire_window_closed_by_new_mode():
    """창 중 새 담당 모드(3→2) → 리셋: 창 닫힘, INIT → 첫 SEARCH 20000."""
    t0 = CLOCK.t
    try:
        n = _hold_then_lost(t0)
        assert _nstep(n, t0 + 13.0) == 50000.0
        n.wp_cb(_Float32(2))
        assert n.state == 'INIT' and n.reacquire_t0 is None and n.hold_total == 0.0
        assert _nstep(n, t0 + 13.5) == 20000.0
        _nstep(n, t0 + 14.0)                            # INIT 1 s → SEARCH
        assert n.state == 'SEARCH'
        assert _nstep(n, t0 + 14.1) == 20000.0          # 옛 창(→ t0+17.5)이 살았다면 50000
        assert _fail_warns(n) == []
    finally:
        CLOCK.t = t0


def test_reacquire_window_kept_on_same_mode_3_to_3():
    """창 중 /wp_mode 3 재수신(리셋 없음) → 창 유지, 끝나면 20000."""
    t0 = CLOCK.t
    try:
        n = _hold_then_lost(t0)
        n.wp_cb(_Float32(3))
        assert n.state == 'SEARCH' and n.reacquire_t0 == t0 + 12.5
        assert _nstep(n, t0 + 14.0) == 50000.0
        assert _nstep(n, t0 + 17.5) == 20000.0 and len(_fail_warns(n)) == 1
    finally:
        CLOCK.t = t0


def _win_max_errors(n):
    return [ln for ln in _max_errors(n) if '재획득 정지 창 포함' in ln[1]]


def test_reacquire_window_counts_toward_hold_max():
    """결정 6 보충 ①: HOLD 합 57.5 s 에서 lost → 창 2.4 s(합 59.9) SEARCH 50000,
    2.5 s(합 60.0) DONE·ERROR 1회(창 포함 표시) → 20000 래치. 창 중 3→3 은 합 이어짐."""
    t0 = CLOCK.t
    try:
        n, _ = _hold_node()
        n.hold_total = 50.0
        assert _hstep(n, t0 + 5.0, 6.0) == 0.0
        _nstep(n, t0 + 7.5)                             # lost → 합 57.5, 창 시작
        assert n.state == 'SEARCH' and abs(n.hold_total - 57.5) < 1e-9
        n.wp_cb(_Float32(3))                            # 3→3: 리셋 없음 → 창·합 그대로
        assert n.reacquire_t0 == t0 + 7.5 and abs(n.hold_total - 57.5) < 1e-9
        assert _nstep(n, t0 + 9.9) == 50000.0 and n.state == 'SEARCH'    # 창 2.4 s
        assert errors(n) == []
        _nstep(n, t0 + 10.0)                                              # 창 2.5 s → 60.0
        assert n.state == 'DONE', n.state
        e = _win_max_errors(n)
        assert len(e) == 1 and '60 s 넘음' in e[0][1] and '포기' in e[0][1], errors(n)
        for k in range(1, 4):
            assert _nstep(n, t0 + 10.0 + k) == 20000.0 and n.state == 'DONE'
        n.wp_cb(_Float32(3))                            # 3→3 래치 유지
        assert _hstep(n, t0 + 15.0, 2.5) == 20000.0 and n.state == 'DONE'
        assert len(errors(n)) == 1 and _fail_warns(n) == [], errors(n)
    finally:
        CLOCK.t = t0


def test_reacquire_window_end_tick_counts_full_window():
    """합 55 s 에서 lost → 창 4.9 s(59.9) 50000, 창 끝 틱 5.05 s 는 창 길이 5 s 로 잘라 60.0 → DONE."""
    t0 = CLOCK.t
    try:
        n, _ = _hold_node()
        n.hold_total = 47.5
        assert _hstep(n, t0 + 5.0, 6.0) == 0.0
        _nstep(n, t0 + 7.5)                             # lost → 합 55.0
        assert abs(n.hold_total - 55.0) < 1e-9
        assert _nstep(n, t0 + 12.4) == 50000.0 and n.state == 'SEARCH'
        _nstep(n, t0 + 12.55)
        assert n.state == 'DONE' and len(_win_max_errors(n)) == 1, errors(n)
        assert _fail_warns(n) == []
    finally:
        CLOCK.t = t0


def test_reacquire_window_after_end_not_counted_then_sum_continues():
    """합 50 s 에서 lost → 창 5 s 만 합산(55), 창 끝 뒤 20000 구간은 안 늚 → 재획득 HOLD 5 s 에 포기."""
    t0 = CLOCK.t
    try:
        n, _ = _hold_node()
        n.hold_total = 42.5
        assert _hstep(n, t0 + 5.0, 6.0) == 0.0
        _nstep(n, t0 + 7.5)                             # lost → 합 50.0
        assert abs(n.hold_total - 50.0) < 1e-9
        assert _nstep(n, t0 + 12.4) == 50000.0
        assert _nstep(n, t0 + 12.5) == 20000.0 and n.state == 'SEARCH'   # 창 끝
        assert abs(n.hold_total - 55.0) < 1e-9 and n.reacquire_t0 is None
        for t in (13.5, 20.0, 30.0):                    # 20000 구간 17.5 s — 합 그대로
            assert _nstep(n, t0 + t) == 20000.0 and n.state == 'SEARCH'
            assert abs(n.hold_total - 55.0) < 1e-9
        assert errors(n) == []
        _hstep(n, t0 + 31.0, 6.0)                       # 재획득 → HOLD, 합 이어짐
        assert n.state == 'HOLD' and abs(n.hold_total - 55.0) < 1e-9
        assert _hstep(n, t0 + 35.9, 6.0) == 0.0 and n.state == 'HOLD'    # 59.9
        _hstep(n, t0 + 36.0, 6.0)                                         # 60.0
        assert n.state == 'DONE' and len(_max_errors(n)) == 1
        assert _win_max_errors(n) == []                 # HOLD 쪽 상한(창 아님)
    finally:
        CLOCK.t = t0


def test_reacquire_window_unknown_giveup_not_counted():
    """거리 unknown 이 HOLD 에서 이어지다 lost → 창 중엔 unknown 포기 없음(SEARCH, ERROR 0).

    재획득 뒤 unknown 은 새로 센다(set_state 리셋, 결정 4 그대로) → 그로부터 10 s 에 포기.
    """
    t0 = CLOCK.t
    no_dist = _Scan(3.5, obj=170.0)                    # 비전 0° ±3 에도 전방 ±30 에도 없음
    try:
        n, _ = _hold_node()
        assert _step(n, t0, False) == 50000.0 and n.state == 'HOLD'
        assert _step(n, t0 + 7.0, False) == 50000.0 and n.state == 'HOLD'
        assert n.unknown_t0 == t0
        _nstep(n, t0 + 9.5, no_dist)                   # lost
        assert n.state == 'SEARCH' and n.unknown_t0 is None
        assert _nstep(n, t0 + 11.0, no_dist) == 50000.0 and n.state == 'SEARCH'   # 11 s
        assert errors(n) == []
        _step(n, t0 + 12.0, False)                     # 재획득 → HOLD
        assert n.state == 'HOLD'
        assert _step(n, t0 + 12.5, False) == 50000.0 and n.unknown_t0 == t0 + 12.5
        assert _step(n, t0 + 22.4, False) == 50000.0 and n.state == 'HOLD'   # 9.9 s
        _step(n, t0 + 22.6, False)
        assert n.state == 'DONE' and len(errors(n)) == 1
    finally:
        CLOCK.t = t0


def test_reacquire_stop_zero_old_behavior():
    """reacquire_stop_sec=0 → 옛 동작: lost 다음 틱부터 20000, 놓침/실패 WARN 없음."""
    t0 = CLOCK.t
    try:
        n, _ = _hold_node()
        n.reacquire_stop_sec = 0.0
        assert _hstep(n, t0 + 10.0, 6.0) == 0.0
        _nstep(n, t0 + 12.5)
        assert n.state == 'SEARCH' and n.reacquire_t0 is None
        assert _nstep(n, t0 + 12.6) == 20000.0 and _nstep(n, t0 + 14.0) == 20000.0
        assert _lost_warns(n) == [] and _fail_warns(n) == []
    finally:
        CLOCK.t = t0


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

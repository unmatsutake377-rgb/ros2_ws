#!/usr/bin/env python3
"""ship_dock TRANSIT·DONE 원인별 발행 테스트 — 실제 ShipDock 콜백·timer_cb 를 스텁 노드로 돌린다.

실행: python3 src/ship_dock/test/test_dock_transit.py
      tools/run_tests.sh ship_dock

계약서 docs/문제와작업/계약서_전체코스_20261104.md C4(U2) · §2-3 · §5-1 · §6 · §10.
  · mode 7 진입 첫 상태 = TRANSIT: 매 틱 /candidate_angle 20000(GPS 주행에 맡김).
  · /goal_distance 가 신선(goal_dist_stale_sec 2.0 이내)하고 dock_engage_dist_m(12.0) 미만이면 INIT.
  · 진입 후 1.0 s(GOAL_DIST_ENTRY_GUARD_SEC) 지나 받은 값만 쓴다 — 직전 WP 거리 배제
    (north 는 한 틱에 거리를 먼저, /wp_mode 를 나중에 낸다). 모르면 TRANSIT 유지.
  · DONE 원인별(§10-추가·추가 결정 2): 후진 이탈 완료 = 20000. SEARCH 타임아웃 = WP8 동안 50000,
    north 가 WP9 로 넘어가면(/wp_remaining_time 리셋 감지) 20000. 후진 타임아웃·원인 모름 = 50000.
    7→7 은 리셋하지 않는다(원인·WP 전환 기록 유지).
  · 코드 기본값 = config/ship_dock.yaml (대회 launch 가 ship_dock 에 이 yaml 을 넘긴다, 11-05).

계약서 docs/문제와작업/계약서_도킹거리None_20261106.md (LiDAR 전방 거리 None):
  · APPROACH None → 50000 + WARN 2 s, 표식 유실→SEARCH 는 안 봄, 30 s 접근 타임아웃은 진행 → BACKOFF.
  · CONTACT None → 50000 유지(INFO 5 s), 3 s 뒤 BACKOFF.
  · BACKOFF None: /scan 신선(전방 반환만 없음) → 180 + WARN 2 s, 8 s 초과 → backoff_timeout(50000)+ERROR;
    /scan 없음·stale → 50000 + ERROR 2 s. 거리가 돌아와 >= 2.5 m 면 그 틱에 DONE(backoff) → 20000.
  · None 만들기: `_Scan(inf)`(원인 ② 신선·반환 없음) / last_scan_t 없음·묵음(원인 ① LiDAR 부재).

계약서 docs/문제와작업/계약서_도킹물보라필터_20261109.md (전방 거리 공간 median, 검증 계획 (c)):
  · 섹터 한 광선만 0.5 m(나머지 5 m) → APPROACH 가 CONTACT 안 감 / front_median_kernel 1 이면 CONTACT(롤백).
  · BACKOFF 에서 고립 반사 점만 → 180 발행·DONE 아님(8 s 뒤 backoff_timeout), WARN '고립 반사' + 버린 점 수.
  · kernel 이상값(음수·9 초과) → 부팅 ERROR 1회 + 5 대체. 짝수 kernel 은 부팅 로그에 실효 창 k+1.

스텁 방식은 test_dock_steer_clamp.py 와 같다(import 동안만 sys.modules 에 넣고 되돌림).
"""

import importlib.util
import math
import os
import sys
import types

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, PKG)    # ship_dock.py 의 `from ship_dock.dock_fsm_logic import …` 용

OVERRIDES = {}             # 파라미터 덮어쓰기(비정상값 방어 테스트용). 비우면 코드 기본값.
DECLARED = {}              # 마지막 노드가 선언한 (이름 → 코드 기본값)


# ---- rclpy 스텁 노드 ----
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


class _Node:
    def __init__(self, name):
        self._logger = _Logger()
        self.subs = []
        DECLARED.clear()

    def declare_parameter(self, name, default):
        DECLARED[name] = default
        return _Param(OVERRIDES.get(name, default))

    def create_publisher(self, typ, topic, qos):
        return _Pub()

    def create_subscription(self, typ, topic, cb, qos):
        self.subs.append((topic, cb))
        return None

    def create_timer(self, *a, **k):
        return None

    def get_logger(self):
        return self._logger


class _Float32:
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
        'std_msgs.msg': dict(Float32=_Float32, Int32=_Float32),
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
            'ship_dock_transit_under_test', os.path.join(PKG, 'ship_dock', 'ship_dock.py'))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return mod


SD = _load_node_module()
SD.Float32 = _Float32


class _Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def monotonic(self):
        return self.t


CLOCK = _Clock()
SD.time = CLOCK

T0 = 1000.0           # mode 7 진입 시각
GPS = 20000.0
STOP = 50000.0


class _Scan:
    """LaserScan 흉내. 0~359°(80 = 정면) 1° 간격, 전부 같은 거리."""

    def __init__(self, dist):
        self.angle_min = 0.0
        self.angle_increment = math.radians(1.0)
        self.ranges = [float(dist)] * 360


def new_node():
    OVERRIDES.clear()
    CLOCK.t = T0 - 5.0
    n = SD.ShipDock()
    n.wp_mode_cb(_Float32(5))          # 직전 WP(mode 5) 동안
    return n


def enter(n, t=T0):
    CLOCK.t = t
    n.wp_mode_cb(_Float32(7))


def goal(n, d, t):
    CLOCK.t = t
    n.goal_distance_cb(_Float32(d))


def tick(n, t):
    """t 에 한 틱 → 이 틱의 발행값 목록."""
    CLOCK.t = t
    before = len(n.pub_candidate.values)
    n.timer_cb()
    return n.pub_candidate.values[before:]


def lines(n, lvl):
    return [ln for ln in n.get_logger().lines if ln[0] == lvl]


def fresh_sensors(n, t, dist=5.0, image=SD.IMAGE_INVALID):
    n.latest_scan = _Scan(dist)
    n.last_scan_t = t
    n.image_angle = float(image)
    n.last_image_t = t


# ---------------- TRANSIT: 20000, 진입 판단
def test_first_state_is_transit_and_publishes_20000():
    n = new_node()
    assert n.state == 'TRANSIT', n.state
    enter(n)
    assert tick(n, T0) == [GPS]
    assert tick(n, T0 + 0.1) == [GPS]
    assert n.state == 'TRANSIT'


def test_silent_outside_mode7():
    n = new_node()
    assert tick(n, T0) == []


def test_distance_before_entry_ignored():
    """진입 1.0 s 전에 받은 3 m(직전 WP 거리)는 진입 뒤 신선해도 쓰지 않는다."""
    n = new_node()
    goal(n, 3.0, T0 - 1.0)
    enter(n)
    for dt in (0.0, 0.5, 0.9):
        assert tick(n, T0 + dt) == [GPS], dt       # 지금 − 수신 ≤ 1.9 s = 신선, 그래도 무시
    assert n.state == 'TRANSIT'


def test_distance_just_after_entry_ignored():
    """진입 직후(0.5 s, 0.99 s) 받은 값도 무시 — north 가 거리 → /wp_mode 순서라 옛 WP 거리일 수 있다."""
    for dt_recv in (0.0, 0.5, 0.99):
        n = new_node()
        enter(n)
        goal(n, 3.0, T0 + dt_recv)
        assert tick(n, T0 + dt_recv + 0.1) == [GPS], dt_recv
        assert n.state == 'TRANSIT', dt_recv


def test_distance_after_guard_engages():
    """진입 후 정확히 1.0 s 에 받은 값은 쓴다(경계) → INIT, 그 틱에 STOP_HOLD."""
    n = new_node()
    enter(n)
    goal(n, 3.0, T0 + SD.GOAL_DIST_ENTRY_GUARD_SEC)
    assert SD.GOAL_DIST_ENTRY_GUARD_SEC == 1.0
    out = tick(n, T0 + 1.0)
    assert n.state == 'INIT' and out == [STOP], (n.state, out)


def test_engage_boundary_12m():
    n = new_node()
    enter(n)
    goal(n, 12.0, T0 + 1.5)
    assert tick(n, T0 + 1.5) == [GPS] and n.state == 'TRANSIT'    # 12.0 은 '미만' 아님
    goal(n, 11.99, T0 + 2.0)
    assert tick(n, T0 + 2.0) == [STOP] and n.state == 'INIT'


def test_far_distance_stays_transit():
    n = new_node()
    enter(n)
    for k in range(10):
        t = T0 + 1.0 + 0.5 * k
        goal(n, 30.0, t)
        assert tick(n, t) == [GPS]
    assert n.state == 'TRANSIT'


def test_stale_distance_stays_transit():
    """받은 지 2.0 s 는 신선(경계), 2.1 s 는 stale → TRANSIT 유지."""
    n = new_node()
    enter(n)
    goal(n, 5.0, T0 + 1.0)
    assert tick(n, T0 + 1.0 + 2.1) == [GPS] and n.state == 'TRANSIT'
    n = new_node()
    enter(n)
    goal(n, 5.0, T0 + 1.0)
    assert tick(n, T0 + 1.0 + 2.0) == [STOP] and n.state == 'INIT'


def test_no_distance_stays_transit_with_warn():
    n = new_node()
    enter(n)
    for k in range(60):
        assert tick(n, T0 + 0.1 * k) == [GPS]
    assert n.state == 'TRANSIT'
    w = lines(n, 'WARN')
    assert w and all(x[2].get('throttle_duration_sec') == 5.0 for x in w), w
    assert lines(n, 'ERROR') == []


def test_no_warn_in_first_guard_plus_stale():
    n = new_node()
    enter(n)
    for k in range(30):                    # 0 ~ 2.9 s < guard 1.0 + stale 2.0
        tick(n, T0 + 0.1 * k)
    assert lines(n, 'WARN') == []


def test_invalid_distance_stays_transit():
    for d in (float('nan'), float('inf'), -1.0):
        n = new_node()
        enter(n)
        goal(n, d, T0 + 1.5)
        assert tick(n, T0 + 1.5) == [GPS], d
        assert n.state == 'TRANSIT', d


def test_invalid_fresh_distance_warns():
    """신선하지만 NaN·inf·음수가 계속 오면 TRANSIT 유지 + 5 s 간격 WARN(침묵 금지)."""
    for d in (float('nan'), float('inf'), -1.0):
        n = new_node()
        enter(n)
        for k in range(60):
            t = T0 + 0.1 * k
            if k % 5 == 0:
                goal(n, d, t)
            assert tick(n, t) == [GPS], (d, k)
        assert n.state == 'TRANSIT', d
        w = lines(n, 'WARN')
        assert w and all(x[2].get('throttle_duration_sec') == 5.0 for x in w), (d, w)
        assert lines(n, 'ERROR') == [], d


def test_engage_log_is_info_only():
    n = new_node()
    enter(n)
    goal(n, 8.0, T0 + 1.5)
    tick(n, T0 + 1.5)
    assert n.state == 'INIT'
    info = [x for x in lines(n, 'INFO') if '도킹 시작' in x[1]]
    assert len(info) == 1, lines(n, 'INFO')
    assert lines(n, 'WARN') == [] and lines(n, 'ERROR') == []


def test_init_after_transit_waits_then_search():
    n = new_node()
    enter(n)
    goal(n, 8.0, T0 + 1.5)
    tick(n, T0 + 1.5)
    t = T0 + 1.5 + n.init_wait_sec - 0.1
    fresh_sensors(n, t)
    assert tick(n, t) == [STOP] and n.state == 'INIT'
    t = T0 + 1.5 + n.init_wait_sec
    fresh_sensors(n, t)
    tick(n, t)
    assert n.state == 'SEARCH'


def test_direct_wp_mode_set_first_tick_is_entry():
    """wp_mode 를 콜백 없이 7 로 둔 경우(시뮬·기존 테스트) 첫 틱이 진입 시각."""
    OVERRIDES.clear()
    CLOCK.t = T0
    n = SD.ShipDock()
    n.wp_mode = n.active_wp_mode
    assert tick(n, T0) == [GPS]
    assert n.mode_enter_t == T0
    goal(n, 3.0, T0 + 0.5)
    assert tick(n, T0 + 0.5) == [GPS]
    goal(n, 3.0, T0 + 1.0)
    assert tick(n, T0 + 1.0) == [STOP]


# ---------------- DONE: 원인별 (후진 이탈 완료 20000, 그 밖 50000)
def _ticks50(n, t0):
    out = []
    for k in range(50):
        out += tick(n, t0 + 0.1 * (k + 1))
    return out


def test_done_after_backoff_publishes_20000():
    n = new_node()
    enter(n)
    n.state, n.state_t = 'BACKOFF', T0 + 20.0
    fresh_sensors(n, T0 + 21.0, dist=3.0)              # 3.0 ≥ backoff 2.5 → DONE
    tick(n, T0 + 21.0)
    assert n.state == 'DONE' and n.done_reason == 'backoff'
    assert _ticks50(n, T0 + 21.0) == [GPS] * 50
    assert lines(n, 'ERROR') == []
    assert any('후진 이탈 완료' in x[1] for x in lines(n, 'INFO'))


def test_done_after_search_timeout_publishes_stop():
    """SEARCH 타임아웃 DONE 은 50000 유지 — 현재 WP 가 칸 입구(WP8)라 20000 이면 칸 안으로 몬다."""
    n = new_node()
    enter(n)
    n.state, n.state_t = 'SEARCH', T0 + 2.0
    t = T0 + 2.0 + n.search_timeout_sec + 0.1
    fresh_sensors(n, t)
    tick(n, t)
    assert n.state == 'DONE' and n.done_reason == 'search_timeout'
    e = lines(n, 'ERROR')
    assert len(e) == 1 and 'SEARCH 타임아웃' in e[0][1] and 'WP8' in e[0][1], e
    assert _ticks50(n, t) == [STOP] * 50


def test_done_after_backoff_timeout_publishes_stop():
    """후진 8 s 안에 이탈 거리를 확인 못 하면 50000 유지(아직 도크 앞일 수 있다) + ERROR 1회."""
    n = new_node()
    enter(n)
    n.state, n.state_t = 'BACKOFF', T0 + 20.0
    t = T0 + 20.0 + n.backoff_timeout_sec + 0.1
    fresh_sensors(n, t, dist=1.5)                      # 1.5 < 2.5 → 거리 미달
    tick(n, t)
    assert n.state == 'DONE' and n.done_reason == 'backoff_timeout'
    e = lines(n, 'ERROR')
    assert len(e) == 1 and '후진 타임아웃' in e[0][1], e
    assert _ticks50(n, t) == [STOP] * 50


def test_done_unknown_reason_is_stop():
    """원인 모름(상태만 DONE) → 50000. 모르면 GPS 에 넘기지 않는다."""
    n = new_node()
    enter(n)
    n.state, n.state_t = 'DONE', T0
    assert _ticks50(n, T0) == [STOP] * 50


# ---------------- 7→7 리셋 안 함, 다른 모드 → 리셋
def test_mode7_to_mode7_keeps_state():
    n = new_node()
    enter(n)
    n.state, n.state_t, n.standoff_aligned = 'APPROACH', T0 + 5.0, True
    CLOCK.t = T0 + 9.0
    n.wp_mode_cb(_Float32(7))                         # WP8 → WP9 (둘 다 mode 7)
    assert (n.state, n.state_t, n.standoff_aligned) == ('APPROACH', T0 + 5.0, True)
    assert n.mode_enter_t == T0
    n.state, n.done_reason = 'DONE', 'backoff'
    n.wp_mode_cb(_Float32(7))
    assert n.state == 'DONE' and n.done_reason == 'backoff' and tick(n, T0 + 9.1) == [GPS]
    n.done_reason = 'search_timeout'
    n.wp_mode_cb(_Float32(7))
    assert n.done_reason == 'search_timeout' and tick(n, T0 + 9.2) == [STOP]


def test_leave_mode7_resets_to_transit():
    n = new_node()
    enter(n)
    n.state, n.done_reason = 'DONE', 'search_timeout'
    CLOCK.t = T0 + 30.0
    n.wp_mode_cb(_Float32(8))
    assert n.state == 'TRANSIT' and n.mode_enter_t is None and n.done_reason is None
    assert tick(n, T0 + 30.1) == []                   # mode 8 → 발행 안 함
    enter(n, T0 + 40.0)
    assert n.mode_enter_t == T0 + 40.0
    assert tick(n, T0 + 40.0) == [GPS]


# ---------------- 순수 함수 경계
def test_transit_engage_pure():
    f = SD.transit_engage
    a = dict(engage_dist=12.0, stale_sec=2.0, entry_guard_sec=1.0)
    assert f(5.0, 11.0, 10.0, 11.0, **a) is True
    assert f(5.0, 10.99, 10.0, 11.0, **a) is False         # guard 전 수신
    assert f(5.0, 11.0, 10.0, 13.0, **a) is True           # 2.0 = 신선
    assert f(5.0, 11.0, 10.0, 13.01, **a) is False         # stale
    assert f(12.0, 11.0, 10.0, 11.0, **a) is False
    assert f(None, 11.0, 10.0, 11.0, **a) is False
    assert f(5.0, None, 10.0, 11.0, **a) is False
    assert f(5.0, 11.0, None, 11.0, **a) is False
    assert f(0.0, 11.0, 10.0, 11.0, **a) is True


# ---------------- 파라미터
def _yaml_params():
    import yaml
    p = os.path.join(PKG, 'config', 'ship_dock.yaml')
    with open(p, encoding='utf-8') as f:
        return yaml.safe_load(f)['/ship_dock']['ros__parameters']


def test_code_defaults_equal_yaml():
    OVERRIDES.clear()
    SD.ShipDock()
    y = _yaml_params()
    assert DECLARED['dock_engage_dist_m'] == 12.0
    assert DECLARED['goal_dist_stale_sec'] == 2.0
    assert set(DECLARED) == set(y), (set(DECLARED) ^ set(y))
    diff = {k: (DECLARED[k], y[k]) for k in y if DECLARED[k] != y[k]}
    assert diff == {}, diff


def test_bad_params_fall_back_with_warn():
    for bad in (float('nan'), float('inf'), 0.0, -3.0):
        OVERRIDES.clear()
        OVERRIDES.update(dock_engage_dist_m=bad, goal_dist_stale_sec=bad)
        n = SD.ShipDock()
        assert n.dock_engage_dist_m == 12.0 and n.goal_dist_stale_sec == 2.0, bad
        w = [x for x in lines(n, 'WARN') if 'dock_engage_dist_m' in x[1]
             or 'goal_dist_stale_sec' in x[1]]
        assert len(w) == 2, (bad, w)
    OVERRIDES.clear()


def test_subscribes_goal_distance():
    OVERRIDES.clear()
    n = SD.ShipDock()
    topics = [t for t, _ in n.subs]
    assert topics == ['/wp_mode', '/image_angle', '/scan', '/goal_distance',
                      '/wp_remaining_time'], topics


# ---------------- SEARCH 타임아웃 DONE: WP8 동안 50000, WP9 전환(/wp_remaining_time 리셋) 뒤 20000
def remain(n, v, t):
    CLOCK.t = t
    n.wp_remaining_cb(_Float32(v))


def search_timeout_done(n):
    """진입 T0, SEARCH 를 T0+4 에 시작해 타임아웃 → DONE. DONE 시각 반환."""
    n.state, n.state_t = 'SEARCH', T0 + 4.0
    t = T0 + 4.0 + n.search_timeout_sec + 0.1
    fresh_sensors(n, t)
    tick(n, t)
    assert n.state == 'DONE' and n.done_reason == 'search_timeout'
    return t


def test_wp_reset_detected_pure():
    f = SD.wp_reset_detected
    assert SD.WP_RESET_EPS_SEC == 0.25
    assert f(0.0, 150.0) is True            # WP8 시간 초과 → WP9 150
    assert f(149.0, 150.0) is True          # 도착(dwell 1.0) → 다음 WP: 최소 실제 증가 ≈ 1.0
    assert f(100.0, 100.25) is False        # eps 경계: 같으면 아님
    assert f(100.0, 100.26) is True
    assert f(100.0, 99.5) is False          # 한 WP 안에서는 줄기만 한다
    assert f(None, 150.0) is False          # 기준 없음 → 모름
    assert f(float('nan'), 150.0) is False and f(10.0, float('inf')) is False


def test_search_timeout_stop_during_wp8_then_20000_after_wp9():
    n = new_node()
    enter(n)
    r = 150.0
    for k in range(1, 9):                   # 진입 후 0.5 s 간격, WP8 남은 시간이 준다
        remain(n, r - 0.5 * k, T0 + 0.5 * k)
    t = search_timeout_done(n)
    for k in range(20):                     # WP8 동안: 남은 시간 계속 줄어듦 → 정지 유지
        remain(n, 140.0 - 0.5 * k - 30.0, t + 0.5 * k)
    assert _ticks50(n, t + 10.0) == [STOP] * 50
    assert n.wp_resets == 0
    remain(n, 0.0, t + 20.0)                # WP8 시간 초과
    remain(n, 150.0, t + 20.5)              # north 가 WP9 로 → 리셋
    assert n.wp_resets == 1
    assert _ticks50(n, t + 20.5) == [GPS] * 50
    det = [x for x in lines(n, 'INFO') if 'WP 전환 감지:' in x[1]]
    assert len(det) == 1 and '0.0 → 150.0' in det[0][1], det       # 감지 순간 INFO 1회
    rel = [x for x in lines(n, 'INFO') if '칸 입구 지남' in x[1]]
    assert len(rel) == 1, rel                                          # 20000 전환 INFO 1회


def test_previous_wp_remain_inside_guard_ignored():
    """진입 직후(guard 1.0 s 안) 받은 직전 모드 WP 값(100)이 기준이 되면 WP8 150 을 '리셋'으로 오판한다."""
    n = new_node()
    enter(n)
    remain(n, 100.0, T0 + 0.1)              # 늦게 온 WP7(mode 5) 값
    remain(n, 150.0, T0 + 0.5)              # WP8 시작 값 (guard 안)
    remain(n, 149.0, T0 + 1.0)              # guard 뒤 첫 값 = 기준
    remain(n, 148.5, T0 + 1.5)
    assert n.wp_resets == 0 and n.remain_prev == 148.5
    t = search_timeout_done(n)
    assert _ticks50(n, t) == [STOP] * 50


def test_goal_distance_jump_does_not_release_stop():
    """GPS 점프로 /goal_distance 가 크게 바뀌어도 WP 전환으로 보지 않는다(거리는 판단에 안 씀)."""
    n = new_node()
    enter(n)
    remain(n, 149.0, T0 + 1.0)
    t = search_timeout_done(n)
    for k, d in enumerate((4.0, 9.5, 0.3, 30.0, 4.0)):
        goal(n, d, t + 0.5 * k)
        remain(n, 149.0 - 30.0 - 0.5 * k, t + 0.5 * k)
    assert _ticks50(n, t + 3.0) == [STOP] * 50


def test_wp9_before_search_timeout_gives_20000_at_done():
    n = new_node()
    enter(n)
    remain(n, 149.0, T0 + 1.0)
    remain(n, 148.5, T0 + 1.5)
    remain(n, 150.0, T0 + 2.0)              # WP8 도착 → WP9 (탐색 중, 간격 없음)
    t = search_timeout_done(n)
    assert _ticks50(n, t) == [GPS] * 50


def test_backoff_timeout_stays_stop_after_wp_reset():
    n = new_node()
    enter(n)
    remain(n, 149.0, T0 + 1.0)
    n.state, n.state_t = 'BACKOFF', T0 + 20.0
    t = T0 + 20.0 + n.backoff_timeout_sec + 0.1
    fresh_sensors(n, t, dist=1.5)
    tick(n, t)
    assert n.done_reason == 'backoff_timeout'
    remain(n, 0.0, t + 0.5)
    remain(n, 150.0, t + 1.0)
    assert n.wp_resets == 1
    assert _ticks50(n, t + 1.0) == [STOP] * 50


def test_no_remaining_time_stays_stop_with_warn():
    """/wp_remaining_time 이 안 오면(시간 초과 표에 7 없음·north 정지) 전환을 모른다 → 정지 + WARN."""
    n = new_node()
    enter(n)
    t = search_timeout_done(n)
    assert _ticks50(n, t) == [STOP] * 50
    w = [x for x in lines(n, 'WARN') if '/wp_remaining_time' in x[1]]
    assert w and all(x[2].get('throttle_duration_sec') == 5.0 for x in w), w


def test_wp_track_kept_on_7_to_7_and_reset_on_leave():
    n = new_node()
    enter(n)
    remain(n, 149.0, T0 + 1.0)
    remain(n, 150.0, T0 + 1.5)
    assert n.wp_resets == 1
    CLOCK.t = T0 + 2.1
    n.wp_mode_cb(_Float32(7))
    assert n.wp_resets == 1 and n.remain_prev == 150.0
    n.wp_mode_cb(_Float32(8))
    assert n.wp_resets == 0 and n.remain_prev is None
    remain(n, 10.0, T0 + 3.0)               # mode 7 밖 값은 무시
    assert n.remain_prev is None


def test_remain_gap_then_150_is_not_reset():
    """MANUAL→AUTO 복귀: remain 이 끊겼다(> 0.8 s) 다시 150 → 리셋 아님(WP8 인데 20000 금지)."""
    assert SD.WP_REMAIN_GAP_SEC == 0.8
    n = new_node()
    enter(n)
    remain(n, 149.0, T0 + 1.0)
    remain(n, 148.5, T0 + 1.5)
    t = search_timeout_done(n)
    remain(n, 90.0, t)                      # (기준 갱신은 간격 때문에 버림 → 기준 90)
    remain(n, 89.5, t + 0.5)
    remain(n, 150.0, t + 0.5 + 0.81)        # 0.81 s 끊긴 뒤 150 (AUTO 복귀)
    assert n.wp_resets == 0 and n.remain_prev == 150.0
    remain(n, 149.5, t + 1.81)
    assert n.wp_resets == 0
    assert _ticks50(n, t + 2.0) == [STOP] * 50
    assert any('끊김' in x[1] for x in lines(n, 'WARN'))


def test_remain_no_gap_0_to_150_is_reset_with_info():
    """간격 없이(0.5 s) 0 → 150 = 실제 전환 → 리셋, INFO 1회(이전 → 새 값). 0.8 s 간격까지는 유지."""
    n = new_node()
    enter(n)
    remain(n, 0.5, T0 + 1.0)
    remain(n, 0.0, T0 + 1.5)
    remain(n, 150.0, T0 + 2.0)
    assert n.wp_resets == 1
    info = [x for x in lines(n, 'INFO') if 'WP 전환 감지:' in x[1]]
    assert len(info) == 1 and '0.0 → 150.0' in info[0][1], info
    n = new_node()
    enter(n)
    remain(n, 0.0, T0 + 1.0)
    remain(n, 150.0, T0 + 1.8)              # 간격 0.8 s = 경계, 기준 유지
    assert n.wp_resets == 1


# ---------------- 거리 None (계약서_도킹거리None_20261106): APPROACH 만 정지, CONTACT·BACKOFF 는 진행
REV = 180.0
INF = float('inf')


def no_return_scan(n, t):
    """원인 ②: /scan 은 신선한데 전방 ±15° 에 유한 반환이 없다(전부 inf) → front_min_dist None."""
    n.latest_scan = _Scan(INF)
    n.last_scan_t = t


def marker(n, t, angle=0.0):
    n.image_angle = float(angle)
    n.last_image_t = t


def lidar_lines(n, lvl):
    """거리 None 로그만(부팅 안내문 'ship_dock 시작' 도 'LiDAR' 를 담고 있어 제외)."""
    return [x for x in lines(n, lvl) if 'LiDAR' in x[1] and 'ship_dock 시작' not in x[1]]


def test_none_scan_variants_give_none_distance():
    n = new_node()
    no_return_scan(n, T0)
    assert n.front_min_dist(T0) is None and n.scan_fresh(T0) is True
    fresh_sensors(n, T0, dist=1.0)
    assert n.front_min_dist(T0 + 1.0) == 1.0 and n.scan_fresh(T0 + 1.0) is True   # 1.0 s = 신선(경계)
    assert n.front_min_dist(T0 + 1.01) is None and n.scan_fresh(T0 + 1.01) is False
    n.latest_scan, n.last_scan_t = None, None
    assert n.front_min_dist(T0) is None and n.scan_fresh(T0) is False


def test_contact_none_holds_stop_then_backoff_by_time():
    """CONTACT 중 None(원인 ①·② 둘 다): 매 틱 50000, 3.0 s 틱에 BACKOFF. INFO 5 s 한 줄, WARN·ERROR 0."""
    for how in ('no_return', 'no_scan'):
        n = new_node()
        enter(n)
        n.state, n.state_t = 'CONTACT', T0 + 20.0
        for k in range(30):                                   # 0.0 ~ 2.9 s
            t = T0 + 20.0 + k / 10.0
            if how == 'no_return':
                no_return_scan(n, t)
            assert tick(n, t) == [STOP], (how, k)
            assert n.state == 'CONTACT', (how, k)
        t = T0 + 20.0 + 3.0
        if how == 'no_return':
            no_return_scan(n, t)
        assert tick(n, t) == [STOP], how
        assert n.state == 'BACKOFF', how
        info = lidar_lines(n, 'INFO')
        assert info and all(x[2].get('throttle_duration_sec') == 5.0 for x in info), (how, info)
        assert lidar_lines(n, 'WARN') == [] and lines(n, 'ERROR') == [], how


def test_backoff_none_fresh_scan_reverses_then_timeout_stop():
    """원인 ②(스캔 신선·전방 반환 없음): 매 틱 180 + WARN 2 s, 8.0 s 초과 틱 DONE(backoff_timeout)
    + ERROR 1회('없음'), 이후 매 틱 50000."""
    n = new_node()
    enter(n)
    n.state, n.state_t = 'BACKOFF', T0 + 20.0
    for k in range(81):                                       # 0.0 ~ 8.0 s (8.0 은 경계, 아직 후진)
        t = T0 + 20.0 + k / 10.0
        no_return_scan(n, t)
        assert tick(n, t) == [REV], k
        assert n.state == 'BACKOFF', k
    assert lines(n, 'ERROR') == []
    w = lidar_lines(n, 'WARN')
    assert w and all(x[2].get('throttle_duration_sec') == 2.0 for x in w), w
    assert all('후진' in x[1] for x in w), w
    t = T0 + 20.0 + 8.1
    no_return_scan(n, t)
    tick(n, t)
    assert n.state == 'DONE' and n.done_reason == 'backoff_timeout'
    e = lines(n, 'ERROR')
    assert len(e) == 1 and '후진 타임아웃' in e[0][1] and '없음' in e[0][1], e
    for k in range(50):
        t2 = t + (k + 1) / 10.0
        no_return_scan(n, t2)
        assert tick(n, t2) == [STOP], k
    assert len(lines(n, 'ERROR')) == 1


def test_backoff_none_then_distance_returns_done_backoff_20000():
    """None 으로 후진하다 전방 3.0 m 가 돌아오면 그 틱에 DONE(backoff) → 20000."""
    n = new_node()
    enter(n)
    n.state, n.state_t = 'BACKOFF', T0 + 20.0
    for k in range(20):
        t = T0 + 20.0 + k / 10.0
        no_return_scan(n, t)
        assert tick(n, t) == [REV], k
    t = T0 + 22.0
    fresh_sensors(n, t, dist=3.0)
    tick(n, t)
    assert n.state == 'DONE' and n.done_reason == 'backoff'
    assert _ticks50(n, t) == [GPS] * 50
    assert lines(n, 'ERROR') == []
    assert any('후진 이탈 완료' in x[1] for x in lines(n, 'INFO'))


def test_backoff_none_then_short_distance_keeps_reversing():
    """None 으로 후진하다 1.5 m(< 2.5)가 돌아오면 계속 180 (타임아웃 전)."""
    n = new_node()
    enter(n)
    n.state, n.state_t = 'BACKOFF', T0 + 20.0
    no_return_scan(n, T0 + 20.0)
    assert tick(n, T0 + 20.0) == [REV]
    fresh_sensors(n, T0 + 20.1, dist=1.5)
    assert tick(n, T0 + 20.1) == [REV] and n.state == 'BACKOFF'


def test_backoff_scan_stale_stops_with_error_then_timeout():
    """원인 ①(/scan 없음·stale, 결정 ②): 뒤를 못 본다 → 50000 + ERROR 2 s. 시계는 흘러 8 s 뒤 backoff_timeout."""
    for how in ('no_scan', 'stale'):
        n = new_node()
        enter(n)
        n.state, n.state_t = 'BACKOFF', T0 + 20.0
        if how == 'stale':
            fresh_sensors(n, T0 + 18.0, dist=1.0)             # 2.0 s 전 스캔 = stale(1.0 s 초과)
        for k in range(81):
            t = T0 + 20.0 + k / 10.0
            assert tick(n, t) == [STOP], (how, k)
            assert n.state == 'BACKOFF', (how, k)
        e = lidar_lines(n, 'ERROR')
        assert e and all(x[2].get('throttle_duration_sec') == 2.0 for x in e), (how, e)
        assert all('후진 중단' in x[1] for x in e), (how, e)
        assert lidar_lines(n, 'WARN') == [], how
        t = T0 + 20.0 + 8.1
        tick(n, t)
        assert n.state == 'DONE' and n.done_reason == 'backoff_timeout', how
        assert _ticks50(n, t) == [STOP] * 50


def test_backoff_scan_returns_resumes_reverse():
    """/scan 이 끊겼다(50000) 다시 오면(전방 반환 없음이라도) 그 틱부터 180."""
    n = new_node()
    enter(n)
    n.state, n.state_t = 'BACKOFF', T0 + 20.0
    for k in range(10):
        assert tick(n, T0 + 20.0 + k / 10.0) == [STOP], k
    no_return_scan(n, T0 + 21.0)
    assert tick(n, T0 + 21.0) == [REV] and n.state == 'BACKOFF'
    assert len(lidar_lines(n, 'ERROR')) == 10


def test_approach_none_stops_but_timeout_runs_to_backoff():
    """APPROACH 중 None(표식 유효): 매 틱 50000 + WARN 2 s, 30.0 s 까지 APPROACH, 30.1 s 에 BACKOFF(①-a)."""
    n = new_node()
    enter(n)
    n.state, n.state_t = 'APPROACH', T0 + 20.0
    for k in range(301):                                      # 0.0 ~ 30.0 s (30.0 은 경계)
        t = T0 + 20.0 + k / 10.0
        no_return_scan(n, t)
        marker(n, t)
        assert tick(n, t) == [STOP], k
        assert n.state == 'APPROACH', k
    w = lidar_lines(n, 'WARN')
    assert w and all(x[2].get('throttle_duration_sec') == 2.0 for x in w), w
    assert lines(n, 'ERROR') == []
    t = T0 + 20.0 + 30.1
    no_return_scan(n, t)
    marker(n, t)
    tick(n, t)
    assert n.state == 'BACKOFF'
    e = lines(n, 'ERROR')
    assert len(e) == 1 and '접근 타임아웃' in e[0][1], e


def test_approach_none_blocks_marker_loss_until_distance_returns():
    """결정 ①: 거리 None 동안은 표식 유실→SEARCH 를 안 본다(현행 유지). 거리가 돌아오면 그 틱에 SEARCH."""
    n = new_node()
    enter(n)
    n.state, n.state_t = 'APPROACH', T0 + 20.0
    for k in range(20):
        t = T0 + 20.0 + k / 10.0
        no_return_scan(n, t)                                  # 표식 없음(IMAGE_INVALID·수신 없음)
        assert tick(n, t) == [STOP], k
        assert n.state == 'APPROACH', k
    t = T0 + 22.0
    fresh_sensors(n, t, dist=2.0)                             # 거리 복귀, 표식은 여전히 없음
    tick(n, t)
    assert n.state == 'SEARCH'


def test_approach_none_then_contact_distance_returns_contact():
    """APPROACH 중 None 이었다가 0.9 m(<= contact 1.0)가 돌아오면 그 틱에 CONTACT, 다음 틱 50000."""
    n = new_node()
    enter(n)
    n.state, n.state_t = 'APPROACH', T0 + 20.0
    for k in range(20):
        t = T0 + 20.0 + k / 10.0
        no_return_scan(n, t)
        marker(n, t)
        assert tick(n, t) == [STOP], k
    t = T0 + 22.0
    fresh_sensors(n, t, dist=0.9, image=0.0)
    tick(n, t)
    assert n.state == 'CONTACT'
    fresh_sensors(n, t + 0.1, dist=0.9, image=0.0)
    assert tick(n, t + 0.1) == [STOP] and n.state == 'CONTACT'


def test_approach_scan_dead_then_backoff_stop_no_scan_then_timeout():
    """복합(권고): APPROACH 중 /scan 완전 사망(stale, 표식은 유효) → 매 틱 50000 + WARN 2 s
    → 30 s 초과 틱에 BACKOFF → BACKOFF 첫 틱부터 STOP_NO_SCAN(50000 + ERROR 2 s, 180 발행 0회)
    → 8 s 초과 틱에 DONE(backoff_timeout, ERROR 1회) → 이후 50000 유지."""
    n = new_node()
    enter(n)
    fresh_sensors(n, T0 + 18.0, dist=1.5, image=0.0)       # 마지막 스캔. 이후 /scan 끊김(T0+19 부터 stale)
    n.state, n.state_t = 'APPROACH', T0 + 20.0
    for k in range(301):                                      # 0.0 ~ 30.0 s (30.0 = 경계, APPROACH)
        t = T0 + 20.0 + k / 10.0
        marker(n, t)
        assert n.scan_fresh(t) is False, k
        assert tick(n, t) == [STOP], k
        assert n.state == 'APPROACH', k
    w = lidar_lines(n, 'WARN')
    assert w and all(x[2].get('throttle_duration_sec') == 2.0 for x in w), w
    assert lines(n, 'ERROR') == []
    t = T0 + 20.0 + 30.1                                      # 타임아웃 초과 틱 → BACKOFF (전이 틱)
    marker(n, t)
    tick(n, t)
    assert n.state == 'BACKOFF'
    e = lines(n, 'ERROR')
    assert len(e) == 1 and '접근 타임아웃' in e[0][1], e
    tb = n.state_t
    t = tb + 0.1                                              # 전이 직후 첫 BACKOFF 틱: 반드시 STOP
    marker(n, t)
    assert tick(n, t) == [STOP]
    assert n.state == 'BACKOFF'
    first = lidar_lines(n, 'ERROR')
    assert len(first) == 1 and '후진 중단' in first[0][1], first
    assert first[0][2].get('throttle_duration_sec') == 2.0, first
    for k in range(2, 80):                                    # 0.2 ~ 7.9 s: 계속 STOP, 180 없음
        t = tb + k / 10.0
        marker(n, t)
        assert tick(n, t) == [STOP], k
        assert n.state == 'BACKOFF', k
    assert REV not in n.pub_candidate.values
    e = lidar_lines(n, 'ERROR')
    assert e and all(x[2].get('throttle_duration_sec') == 2.0 for x in e), e
    t = tb + 8.1                                              # 8 s 초과 틱 → DONE(backoff_timeout)
    marker(n, t)
    tick(n, t)
    assert n.state == 'DONE' and n.done_reason == 'backoff_timeout'
    to = [x for x in lines(n, 'ERROR') if '후진 타임아웃' in x[1]]
    assert len(to) == 1 and '없음' in to[0][1], to
    assert _ticks50(n, t) == [STOP] * 50
    assert REV not in n.pub_candidate.values
    assert len([x for x in lines(n, 'ERROR') if '후진 타임아웃' in x[1]]) == 1


def test_approach_distance_known_steers_as_before():
    """회귀: 거리·표식이 있으면 종전대로 조향 발행(2.0 m, 5°, standoff 재정렬 끝남 → 5.0)."""
    n = new_node()
    enter(n)
    n.state, n.state_t, n.standoff_aligned = 'APPROACH', T0 + 20.0, True
    fresh_sensors(n, T0 + 20.0, dist=2.0, image=5.0)
    assert tick(n, T0 + 20.0) == [5.0] and n.state == 'APPROACH'
    assert lidar_lines(n, 'WARN') == []


# ---------------- 전방 거리 공간 median (계약서_도킹물보라필터_20261109) — 검증 계획 (c)
FWD = int(SD.LIDAR_FORWARD_DEG)        # _Scan 은 1° 간격, 0° 시작 → 인덱스 80 = 정면


def new_node_with(**over):
    """new_node 와 같되 파라미터를 덮어쓴다(new_node 는 OVERRIDES 를 비운다)."""
    OVERRIDES.clear()
    OVERRIDES.update(over)
    CLOCK.t = T0 - 5.0
    n = SD.ShipDock()
    n.wp_mode_cb(_Float32(5))
    return n


def spike_scan(n, t, base, points):
    """전부 base 인 스캔에 {인덱스: 거리} 만 바꿔 넣는다. 신선(t)."""
    sc = _Scan(base)
    for i, d in points.items():
        sc.ranges[i] = float(d)
    n.latest_scan = sc
    n.last_scan_t = t


def boot_info(n):
    return [x[1] for x in lines(n, 'INFO') if 'ship_dock 시작' in x[1]]


def test_median_kernel_default_5_in_code_and_yaml():
    OVERRIDES.clear()
    n = SD.ShipDock()
    y = _yaml_params()
    assert DECLARED['front_median_kernel'] == 5 and type(DECLARED['front_median_kernel']) is int
    assert y['front_median_kernel'] == 5 and type(y['front_median_kernel']) is int   # 정수(5.0 이면 노드 안 뜸)
    assert n.front_median_kernel == 5 and n.front_median_window == 5
    assert lines(n, 'ERROR') == []
    b = boot_info(n)
    assert len(b) == 1 and 'front_median_kernel=5' in b[0] and '실효 창 5점' in b[0], b


def test_approach_single_spike_does_not_contact():
    """정면 섹터 한 광선만 0.5 m(물보라), 나머지 5 m → front_min_dist 5.0, CONTACT 안 감·조향 계속."""
    n = new_node()
    enter(n)
    n.state, n.state_t, n.standoff_aligned = 'APPROACH', T0 + 20.0, True
    for k in range(20):
        t = T0 + 20.0 + k / 10.0
        spike_scan(n, t, 5.0, {FWD + (k % 7) - 3: 0.5})      # 스파이크 위치를 섹터 안에서 옮겨 본다
        marker(n, t, 0.0)
        assert n.front_min_dist(t) == 5.0, k
        assert tick(n, t) == [0.0], k
        assert n.state == 'APPROACH', k
    assert lidar_lines(n, 'WARN') == [] and lines(n, 'ERROR') == []


def test_approach_single_spike_kernel1_contacts_rollback():
    """롤백 확인: front_median_kernel 1(또는 0) = raw → 같은 한 광선 0.5 m 에 CONTACT(옛 동작)."""
    for k in (1, 0):
        n = new_node_with(front_median_kernel=k)
        assert n.front_median_window == 1 and lines(n, 'ERROR') == [], k
        b = boot_info(n)
        assert len(b) == 1 and f'front_median_kernel={k}' in b[0] and 'raw' in b[0], (k, b)
        enter(n)
        n.state, n.state_t, n.standoff_aligned = 'APPROACH', T0 + 20.0, True
        spike_scan(n, T0 + 20.0, 5.0, {FWD: 0.5})
        marker(n, T0 + 20.0, 0.0)
        assert n.front_min_dist(T0 + 20.0) == 0.5, k
        tick(n, T0 + 20.0)
        assert n.state == 'CONTACT', k


def test_approach_three_point_object_still_contacts():
    """연속 3 광선 0.8 m(실제 물체·벤치 가짜 /scan 최소 폭)는 median 에서도 남는다 → CONTACT."""
    n = new_node()
    enter(n)
    n.state, n.state_t, n.standoff_aligned = 'APPROACH', T0 + 20.0, True
    spike_scan(n, T0 + 20.0, 5.0, {FWD - 1: 0.8, FWD: 0.8, FWD + 1: 0.8})
    marker(n, T0 + 20.0, 0.0)
    assert n.front_min_dist(T0 + 20.0) == 0.8
    tick(n, T0 + 20.0)
    assert n.state == 'CONTACT'


def test_backoff_isolated_points_only_reverses_not_done():
    """BACKOFF 중 전방이 전부 inf + 고립 반사 3.0 m 3 점: raw 면 3.0 ≥ 2.5 → DONE 이었지만 median 은 None.
    스캔 신선 → 매 틱 180(DONE 아님), WARN 은 '고립 반사' + 버린 점 수. 8 s 초과 → backoff_timeout(50000)."""
    iso = {FWD - 10: 3.0, FWD: 3.0, FWD + 10: 3.0}
    n = new_node()
    enter(n)
    n.state, n.state_t = 'BACKOFF', T0 + 20.0
    for k in range(81):                                       # 0.0 ~ 8.0 s
        t = T0 + 20.0 + k / 10.0
        spike_scan(n, t, INF, iso)
        assert n.front_min_dist(t) is None and n.front_rejected == 3, k
        assert tick(n, t) == [REV], k
        assert n.state == 'BACKOFF', k
    assert lines(n, 'ERROR') == []
    w = lidar_lines(n, 'WARN')
    assert w and all(x[2].get('throttle_duration_sec') == 2.0 for x in w), w
    assert all('고립 반사' in x[1] and '버린 점 3' in x[1] and '후진' in x[1] for x in w), w
    t = T0 + 20.0 + 8.1
    spike_scan(n, t, INF, iso)
    tick(n, t)
    assert n.state == 'DONE' and n.done_reason == 'backoff_timeout'
    assert tick(n, t + 0.1) == [STOP]
    assert GPS not in n.pub_candidate.values[-3:]


def test_backoff_isolated_points_kernel1_done_backoff():
    """대조(롤백): kernel 1 이면 같은 고립 반사 3.0 m 로 그 틱에 DONE(backoff) → 20000(옛 동작)."""
    n = new_node_with(front_median_kernel=1)
    enter(n)
    n.state, n.state_t = 'BACKOFF', T0 + 20.0
    spike_scan(n, T0 + 20.0, INF, {FWD - 10: 3.0, FWD: 3.0, FWD + 10: 3.0})
    tick(n, T0 + 20.0)
    assert n.state == 'DONE' and n.done_reason == 'backoff'
    assert tick(n, T0 + 20.1) == [GPS]


def test_approach_isolated_points_only_stops_with_rejected_count():
    """APPROACH 중 섹터가 전부 고립 반사(inf + 떨어진 0.5 m 3 점) + 표식 유효 → 거리 None = STOP_BLIND.
    매 틱 50000, WARN(2 s throttle)에 '버린 점 3' — LiDAR 고장(버린 점 0)과 현장에서 가른다."""
    iso = {FWD - 10: 0.5, FWD: 0.5, FWD + 10: 0.5}
    n = new_node()
    enter(n)
    n.state, n.state_t, n.standoff_aligned = 'APPROACH', T0 + 20.0, True
    for k in range(10):
        t = T0 + 20.0 + k / 10.0
        spike_scan(n, t, INF, iso)
        marker(n, t, 0.0)
        assert tick(n, t) == [STOP], k
        assert n.state == 'APPROACH', k
    w = lidar_lines(n, 'WARN')
    assert w and all(x[2].get('throttle_duration_sec') == 2.0 for x in w), w
    assert all('버린 점 3' in x[1] and '접근 중단' in x[1] for x in w), w
    assert lines(n, 'ERROR') == []
    # 대조: /scan 이 전부 inf(반환 없음)면 버린 점 0
    n = new_node()
    enter(n)
    n.state, n.state_t, n.standoff_aligned = 'APPROACH', T0 + 20.0, True
    no_return_scan(n, T0 + 20.0)
    marker(n, T0 + 20.0, 0.0)
    assert tick(n, T0 + 20.0) == [STOP]
    w = lidar_lines(n, 'WARN')
    assert len(w) == 1 and '버린 점 0' in w[0][1], w


def test_median_kernel_bad_values_error_and_fallback_5():
    for bad in (-1, -5, 10, 100):
        n = new_node_with(front_median_kernel=bad)
        assert n.front_median_kernel == 5 and n.front_median_window == 5, bad
        e = [x for x in lines(n, 'ERROR') if 'front_median_kernel' in x[1]]
        assert len(e) == 1 and str(bad) in e[0][1] and '5' in e[0][1], (bad, e)
        assert len(lines(n, 'ERROR')) == 1, bad
        b = boot_info(n)
        assert len(b) == 1 and 'front_median_kernel=5' in b[0] and '실효 창 5점' in b[0], (bad, b)
        # 대체값 5 가 실제로 쓰인다: 한 광선 스파이크를 지운다
        spike_scan(n, T0, 5.0, {FWD: 0.5})
        assert n.front_min_dist(T0) == 5.0, bad
    OVERRIDES.clear()


def test_median_kernel_even_logs_effective_window():
    for k, win in ((2, 3), (4, 5), (8, 9), (9, 9), (3, 3)):
        n = new_node_with(front_median_kernel=k)
        assert lines(n, 'ERROR') == [], k
        assert n.front_median_kernel == k and n.front_median_window == win, k
        b = boot_info(n)
        assert len(b) == 1 and f'front_median_kernel={k}' in b[0] and f'실효 창 {win}점' in b[0], (k, b)
    OVERRIDES.clear()


if __name__ == '__main__':
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith('test_') and callable(v)]
    fails = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  ✅ {name}")
        except (AssertionError, AttributeError, KeyError) as e:
            fails += 1
            print(f"  ❌ {name}\n     {e!r}")
    print(f"{len(tests) - fails}/{len(tests)} 통과")
    sys.exit(1 if fails else 0)

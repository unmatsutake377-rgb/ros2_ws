"""SensorWatch / TemporalVote 단위 테스트 — ROS 없이 돈다.

    python3 -m pytest src/ship_direction/test/test_failsafe_logic.py -q
    python3 src/ship_direction/test/test_failsafe_logic.py          # pytest 없이도 실행됨

시각(now)을 주입하므로 시간을 마음대로 돌릴 수 있다 (sleep 없음).
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ship_direction.failsafe import SensorWatch, TemporalVote, median_min  # noqa: E402

INF = float('inf')


# ─────────────────────────── SensorWatch ───────────────────────────

def test_armed_boot_no_false_trigger():
    """부팅 중(데이터 한 번도 못 받음)엔 아무리 시간이 흘러도 발동하지 않는다."""
    w = SensorWatch(['scan', 'yaw_error'])
    for t in range(0, 100):
        assert w.update(float(t)) == 0, "ARMED 전에 페일세이프가 발동했다(부팅 오발동)"


def test_warn_then_stop_on_scan_loss():
    """스캔이 끊기면 0.7s 경고 → 3.0s 정지."""
    w = SensorWatch(['scan'], warn_sec=0.7, stop_sec=3.0, confirm_n=3)
    w.feed('scan', 0.0)
    assert w.update(0.0) == 0

    # 0.7s 이내: 정상
    assert w.update(0.5) == 0

    # 0.7s 초과 → 연속 3회 확인 후 레벨1
    assert w.update(0.8) == 0        # 1회
    assert w.update(0.9) == 0        # 2회
    assert w.update(1.0) == 1        # 3회 → 승격
    assert w.level == 1

    # 3.0s 초과 → 연속 3회 확인 후 레벨2
    assert w.update(3.1) == 1
    assert w.update(3.2) == 1
    assert w.update(3.3) == 2
    assert w.level == 2


def test_imu_death_detected():
    """★ 회귀 방지: /scan 은 살아있는데 /yaw_error 만 끊겨도 잡아야 한다.
    (이걸 놓쳐서 IMU 사망 시 정지율 0%, 접촉 3회, 14.8m 계속 주행했다)"""
    w = SensorWatch(['scan', 'yaw_error'], warn_sec=0.7, stop_sec=3.0, confirm_n=3)
    w.feed('scan', 0.0)
    w.feed('yaw_error', 0.0)
    assert w.update(0.0) == 0

    t = 0.0
    for _ in range(60):              # 6초 동안 scan 만 계속 살아있게 먹인다
        t += 0.1
        w.feed('scan', t)            # LiDAR 는 멀쩡
        w.update(t)                  # yaw_error 는 0.0 이후로 안 옴

    assert w.level == 2, f"IMU 사망을 못 잡았다 (level={w.level})"
    assert 'yaw_error' in (w.worst or ''), f"원인 센서가 yaw_error 여야 한다 (worst={w.worst})"


def test_worst_sensor_wins():
    """가장 나쁜 센서가 레벨을 정한다."""
    w = SensorWatch(['scan', 'yaw_error'], warn_sec=0.7, stop_sec=3.0, confirm_n=1)
    w.feed('scan', 0.0)
    w.feed('yaw_error', 0.0)
    # scan 은 신선, yaw_error 는 4초 묵음 → 레벨2 (나쁜 쪽)
    w.feed('scan', 4.0)
    assert w.update(4.0) == 2


def test_no_false_trigger_on_jitter():
    """순간 지터(1회)로는 레벨이 안 올라간다 — confirm_n 회 연속이어야 한다."""
    w = SensorWatch(['scan'], warn_sec=0.7, stop_sec=3.0, confirm_n=3)
    w.feed('scan', 0.0)
    w.update(0.0)

    assert w.update(0.8) == 0        # 1회만 늦음
    w.feed('scan', 0.85)             # 곧바로 회복
    assert w.update(0.9) == 0        # 승격 안 됨
    assert w.level == 0


def test_auto_recovery_is_immediate():
    """센서가 돌아오면 즉시 복구 (올릴 땐 느리게, 풀 땐 빠르게)."""
    w = SensorWatch(['scan'], warn_sec=0.7, stop_sec=3.0, confirm_n=3)
    w.feed('scan', 0.0)
    for t in (0.8, 0.9, 1.0):
        w.update(t)
    assert w.level == 1

    w.feed('scan', 1.1)              # 스캔 복귀
    assert w.update(1.1) == 0, "자동 복구가 즉시 되지 않았다"


def test_hard_fault_is_immediate_no_confirm():
    """angle_increment 이상(하드 폴트)은 확인 N회 없이 즉시 레벨2."""
    w = SensorWatch(['scan'], confirm_n=3)
    w.feed('scan', 0.0)
    assert w.update(0.0) == 0

    w.set_fault(True)
    assert w.update(0.01) == 2, "하드 폴트가 즉시 정지시키지 않았다"

    w.set_fault(False)               # 정상 스캔 복귀
    w.feed('scan', 0.02)
    assert w.update(0.02) == 0, "폴트 해제 후 스스로 풀리지 않았다"


# ─────────────────────────── TemporalVote ───────────────────────────

def test_single_frame_spray_rejected():
    """★ 물보라: 한 프레임만 튄 셀은 장애물로 인정하지 않는다."""
    tv = TemporalVote(frames=3, votes=2)
    tv.apply([0, 0, 0, 0])           # 깨끗
    tv.apply([0, 0, 0, 0])
    out = tv.apply([0, 1, 0, 0])     # 셀1 에 물보라 1회
    assert out == [0, 0, 0, 0], f"한 프레임짜리 물보라를 장애물로 믿었다: {out}"


def test_persistent_obstacle_accepted():
    """진짜 부표: 2프레임 연속 나타나면 인정한다."""
    tv = TemporalVote(frames=3, votes=2)
    tv.apply([0, 1, 0, 0])
    out = tv.apply([0, 1, 0, 0])
    assert out == [0, 1, 0, 0], f"진짜 부표를 놓쳤다: {out}"


def test_obstacle_survives_one_dropout():
    """진짜 부표가 한 프레임 누락돼도, 최근 3프레임 중 2표면 살아남는다."""
    tv = TemporalVote(frames=3, votes=2)
    tv.apply([1])
    tv.apply([0])                    # 한 프레임 누락
    out = tv.apply([1])              # 최근 3프레임 중 2표
    assert out == [1], f"부표가 한 프레임 누락으로 사라졌다: {out}"


def test_disabled_passthrough():
    """frames 나 votes 가 1 이면 꺼진다 (그대로 통과)."""
    for tv in (TemporalVote(frames=1, votes=2), TemporalVote(frames=3, votes=1)):
        assert not tv.enabled
        assert tv.apply([0, 1, 1, 0]) == [0, 1, 1, 0]


def test_varying_scan_length_is_safe():
    """스캔 길이가 바뀌어도 IndexError 없이 동작한다."""
    tv = TemporalVote(frames=3, votes=2)
    tv.apply([1, 1, 1, 1, 1])
    out = tv.apply([1, 1])           # 갑자기 짧아짐
    assert out == [1, 1]


# ─────────────────────────── median_min (감속 신호 필터) ───────────────────────────

def test_median_kills_isolated_spike():
    """★ 핵심: 물보라 고립 스파이크(0.3m)가 이웃(5m)과 어긋나면 지워진다.
    raw-min 이면 0.3 을 믿고 감속했다."""
    r = [5.0, 5.0, 0.3, 5.0, 5.0]
    d, _ = median_min(r, 0, 4, kernel=5)
    assert abs(d - 5.0) < 1e-9, f"고립 스파이크가 안 지워졌다: d={d}"


def test_raw_min_would_be_fooled():
    """대조군: kernel=0(raw-min 폴백)이면 같은 데이터에 속는다."""
    r = [5.0, 5.0, 0.3, 5.0, 5.0]
    d, _ = median_min(r, 0, 4, kernel=0)
    assert abs(d - 0.3) < 1e-9, f"raw-min 폴백이 동작 안 함: d={d}"


def test_median_keeps_real_obstacle():
    """★ 진짜 장애물(연속 군집 2m)은 살아남는다 — 필터가 부표를 지우면 안 된다."""
    r = [5.0, 5.0, 2.0, 2.0, 2.0, 5.0, 5.0]
    d, i = median_min(r, 0, 6, kernel=5)
    assert abs(d - 2.0) < 1e-9, f"진짜 장애물을 지웠다(위험!): d={d}"


def test_isolated_spike_with_no_returns_ignored():
    """주변이 전부 무반사(inf)인 고립 반사 → 유효점 3개 미만 → 무시(=장애물 없음)."""
    r = [INF, INF, 0.3, INF, INF]
    d, i = median_min(r, 0, 4, kernel=5)
    assert d is None, f"고립 반사를 장애물로 믿었다: d={d}"


def test_min_valid_threshold():
    """유효점이 2개뿐이면(min_valid=3 미만) 그 인덱스는 무시된다."""
    r = [INF, 4.0, 0.3, INF, INF]
    d, _ = median_min(r, 0, 4, kernel=5)
    assert d is None, f"유효점 부족한데 채택했다: d={d}"


def test_even_window_uses_upper_median():
    """짝수 개일 땐 위쪽 median → 거리가 크게 나와 '가짜 감속' 쪽으로 안 기운다."""
    # 창 끝단(i=0)은 [0,2] = 3점이라 홀수 → 경계에서 4점 되는 지점을 본다
    r = [1.0, 9.0, 9.0, 9.0]
    d, _ = median_min(r, 0, 3, kernel=5)   # i=0 창=[1,9,9] → med 9 / i=1 창=[1,9,9,9] → 위쪽 9
    assert abs(d - 9.0) < 1e-9, f"위쪽 median 아님: d={d}"


def test_all_invalid_returns_none():
    d, i = median_min([INF, INF, INF], 0, 2, kernel=5)
    assert d is None and i is None


def test_empty_range_returns_none():
    d, i = median_min([], 0, 5, kernel=5)
    assert d is None and i is None


def test_index_returned_for_angle():
    """최소 지점의 인덱스를 돌려줘야 각도를 계산할 수 있다."""
    r = [9.0, 9.0, 3.0, 3.0, 3.0, 9.0, 9.0]
    d, i = median_min(r, 0, 6, kernel=5)
    assert i is not None and abs(d - 3.0) < 1e-9, f"d={d}, i={i}"


# ─────────────────────── D6: 기각수 반환(관측 전용) ───────────────────────
def test_return_rejected_backward_compatible():
    """🚨 기본값 False 면 기존 호출부는 2-튜플 그대로 받아야 한다(회귀 방지)."""
    r = [9.0, 9.0, 3.0, 9.0, 9.0]
    res = median_min(r, 0, 4, kernel=5)
    assert len(res) == 2, "기본 호출은 (거리,인덱스) 2-튜플이어야 한다"


def test_return_rejected_shape():
    r = [9.0, 9.0, 3.0, 9.0, 9.0]
    res = median_min(r, 0, 4, kernel=5, return_rejected=True)
    assert len(res) == 3, "return_rejected=True 면 (거리,인덱스,기각수) 3-튜플"


def test_rejected_counts_isolated_spike():
    """
    고립 스파이크(주변에 유효점이 min_valid 미만)를 기각으로 센다.
    한복판에 유효점 1개뿐이면 median 창 유효점 부족 → 기각.
    """
    # 전부 무한대인데 인덱스 3만 유효(0.3m) = 완전 고립 물보라
    r = [INF, INF, INF, 0.3, INF, INF, INF]
    d, i, rej = median_min(r, 0, 6, kernel=5, min_valid=3, return_rejected=True)
    assert d is None, "고립점은 무시되어 최소거리 없음"
    assert rej == 1, f"고립 스파이크 1개를 기각으로 세야 한다, got {rej}"


def test_rejected_zero_when_dense():
    """유효점이 촘촘하면 기각 0."""
    r = [3.0, 3.0, 3.0, 3.0, 3.0]
    d, i, rej = median_min(r, 0, 4, kernel=5, min_valid=3, return_rejected=True)
    assert rej == 0 and abs(d - 3.0) < 1e-9


def test_rejected_ignores_already_invalid():
    """원래 무효(inf)였던 점은 기각으로 세지 않는다 — median 이 '버린' 게 아니다."""
    r = [INF, INF, INF, INF, INF]
    d, i, rej = median_min(r, 0, 4, kernel=5, min_valid=3, return_rejected=True)
    assert rej == 0, "원래 inf 였던 점은 기각이 아니다"


def test_rejected_kernel_off_no_reject():
    """kernel 0/1(필터 OFF)이면 기각 개념이 없다 → 0."""
    r = [INF, INF, 0.3, INF, INF]
    d, i, rej = median_min(r, 0, 4, kernel=0, return_rejected=True)
    assert rej == 0, "필터 OFF 면 아무것도 기각 안 한다"


# ─────────────────────────── 러너 ───────────────────────────

if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    failed = 0
    for fn in tests:
        try:
            fn()
            print(f"  ✅ {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"  ❌ {fn.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} 통과")
    sys.exit(1 if failed else 0)

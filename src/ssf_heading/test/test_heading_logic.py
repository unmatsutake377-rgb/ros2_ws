#!/usr/bin/env python3
"""
heading_logic 순수 로직 테스트. ROS 없이 그냥 실행된다:
    python3 src/ssf_heading/test/test_heading_logic.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ssf_heading.heading_logic import (  # noqa: E402
    ALL_SOURCES, SOURCE_IS_MAGNETIC, SILENT_STATES, PUBLISH_STATES,
    SRC_IMU_RELATIVE, SRC_COG_OFFSET, SRC_DUAL_GPS, SRC_IMU_ABSOLUTE, SRC_GPS_COG,
    ST_OK, ST_NO_DATA, ST_STALE, ST_NOT_IMPLEMENTED, ST_NOT_CONVERGED,
    ST_HOLD, ST_INIT_GOAL, ST_INIT, ST_IMU_ASSIST,
    SPIN_RIGHT, SPIN_LEFT, GPS_COG_INIT_GOAL, BOAT_MODE_AUTO,
    HeadingMux, COGOffsetEstimator, GPSCOGTracker, cog_from_velocity, saturation_samples,
    gps_cog_status_text,
    wrap360, wrap180, ang_diff, is_finite, apply_corrections, is_spin_code,
)

import math  # noqa: E402

_passed = 0
_total = 0


def check(name, fn):
    global _passed, _total
    _total += 1
    try:
        fn()
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")
        return
    _passed += 1
    print(f"  ✅ {name}")


def near(a, b, tol=1e-9):
    return abs(a - b) < tol


# ------------------------------------------------------------------ 각도 유틸
def test_wrap360():
    assert near(wrap360(0.0), 0.0)
    assert near(wrap360(360.0), 0.0)
    assert near(wrap360(361.0), 1.0)
    assert near(wrap360(-1.0), 359.0)
    assert near(wrap360(-361.0), 359.0)
    assert near(wrap360(720.5), 0.5)


def test_wrap180():
    assert near(wrap180(0.0), 0.0)
    assert near(wrap180(180.0), -180.0), "180 은 -180 으로 접힌다(반열린구간)"
    assert near(wrap180(181.0), -179.0)
    assert near(wrap180(-181.0), 179.0)


def test_ang_diff():
    assert near(ang_diff(10.0, 350.0), 20.0), "북쪽 넘는 차가 -340 이 되면 안 된다"
    assert near(ang_diff(350.0, 10.0), -20.0)
    assert near(ang_diff(0.0, 0.0), 0.0)


def test_is_finite():
    assert is_finite(0.0)
    assert is_finite(-12)
    assert not is_finite(None)
    assert not is_finite(float('nan'))
    assert not is_finite(float('inf'))
    assert not is_finite("90")


# ------------------------------------------------------------------ 보정식
def test_corrections_identity():
    assert near(apply_corrections(123.0), 123.0), "기본값은 아무것도 안 바꾼다"


def test_corrections_invert():
    assert near(apply_corrections(90.0, invert_yaw=True), 270.0)
    assert near(apply_corrections(0.0, invert_yaw=True), 0.0)


def test_corrections_mount_and_decl():
    assert near(apply_corrections(10.0, mount_offset_deg=30.0), 40.0)
    assert near(apply_corrections(10.0, declination_deg=-8.0), 2.0)
    # 랩어라운드
    assert near(apply_corrections(350.0, mount_offset_deg=20.0), 10.0)
    assert near(apply_corrections(5.0, declination_deg=-8.0), 357.0)


def test_corrections_order_invert_before_offset():
    """invert 는 오프셋 '전' 에 걸려야 한다. 순서 바뀌면 오프셋 부호가 뒤집힌다."""
    got = apply_corrections(90.0, invert_yaw=True, mount_offset_deg=10.0)
    assert near(got, 280.0), f"-90+10=-80→280 이어야 하는데 {got}"


# ------------------------------------------------------------------ 🚨 편각 이중보정
def test_declination_only_for_magnetic_source():
    """
    진북 기준 소스(듀얼GPS/COG)에 편각을 더하면 한국 기준 8° 가 통째로 틀어진다.
    이건 물 위에서 '왜인지 모르게 계속 옆으로 간다' 로 나타나는 유형이다.
    """
    for src in ALL_SOURCES:
        mux = HeadingMux(src, declination_deg=-8.0, stale_sec=10.0)
        mux.update_imu(100.0, 0.0)
        mux.update_dual_gps(100.0, 0.0)
        mux.update_mag(100.0, 0.0)
        yaw, st = mux.heading(0.0)
        if st != ST_OK:
            continue
        if SOURCE_IS_MAGNETIC[src]:
            assert near(yaw, 92.0), f"{src}: 자기북 소스인데 편각이 안 걸렸다 ({yaw})"
        else:
            assert near(yaw, 100.0), f"{src}: 진북 소스인데 편각이 걸렸다 ({yaw}) — 이중보정"


def test_magnetic_table_covers_every_source():
    for src in ALL_SOURCES:
        assert src in SOURCE_IS_MAGNETIC, f"{src} 가 SOURCE_IS_MAGNETIC 에 없다"


# ------------------------------------------------------------------ 믹서 기본
def test_unknown_source_raises():
    try:
        HeadingMux("gps_heading_override")
    except ValueError:
        return
    assert False, "모르는 소스를 조용히 받아들이면 안 된다"


def test_no_data_is_silent():
    mux = HeadingMux(SRC_IMU_RELATIVE)
    yaw, st = mux.heading(0.0)
    assert yaw is None and st == ST_NO_DATA


def test_imu_relative_ok():
    mux = HeadingMux(SRC_IMU_RELATIVE, mount_offset_deg=37.0, stale_sec=0.5)
    mux.update_imu(10.0, 100.0)
    yaw, st = mux.heading(100.1)
    assert st == ST_OK and near(yaw, 47.0)


def test_stale_boundary():
    """경계는 '초과' 다. 정확히 stale_sec 면 아직 살아있다."""
    mux = HeadingMux(SRC_IMU_RELATIVE, stale_sec=0.5)
    mux.update_imu(10.0, 100.0)
    assert mux.heading(100.5)[1] == ST_OK, "정확히 임계면 유효"
    assert mux.heading(100.5001)[1] == ST_STALE
    assert mux.heading(100.5001)[0] is None, "stale 이면 값을 주면 안 된다"


def test_stale_then_recovers():
    mux = HeadingMux(SRC_IMU_RELATIVE, stale_sec=0.5)
    mux.update_imu(10.0, 100.0)
    assert mux.heading(101.0)[1] == ST_STALE
    mux.update_imu(20.0, 101.0)
    yaw, st = mux.heading(101.1)
    assert st == ST_OK and near(yaw, 20.0), "복구는 즉시여야 한다"


def test_nan_input_ignored():
    """NaN 이 슬롯을 오염시키면 안 된다. 마지막 정상값이 남고 시각은 안 갱신된다."""
    mux = HeadingMux(SRC_IMU_RELATIVE, stale_sec=0.5)
    mux.update_imu(10.0, 100.0)
    mux.update_imu(float('nan'), 100.1)
    yaw, st = mux.heading(100.2)
    assert st == ST_OK and near(yaw, 10.0)
    # NaN 만 계속 오면 결국 stale 로 떨어져 침묵한다 — 이게 맞다
    for i in range(10):
        mux.update_imu(float('nan'), 100.2 + i * 0.1)
    assert mux.heading(101.5)[1] == ST_STALE


def test_input_wrapped_on_ingest():
    mux = HeadingMux(SRC_IMU_RELATIVE, stale_sec=10.0)
    mux.update_imu(-10.0, 0.0)
    assert near(mux.heading(0.0)[0], 350.0)


# ------------------------------------------------------------------ 미구현 소스
def test_unimplemented_sources_are_loud_not_zero():
    """
    구현 안 된 소스가 조용히 0.0(=정북)을 내면 배가 북쪽으로 달린다.
    반드시 None + NOT_IMPLEMENTED 여야 한다.
    (cog_offset 은 N2 에서 구현됐다 → 아래 별도 테스트)
    """
    for src in (SRC_DUAL_GPS, SRC_IMU_ABSOLUTE):
        mux = HeadingMux(src, stale_sec=10.0)
        mux.update_imu(123.0, 0.0)          # IMU 는 들어와도
        mux.update_cog(45.0, 2.0, 0.0)      # COG 도 들어와도
        yaw, st = mux.heading(0.0)
        assert yaw is None, f"{src}: 미구현인데 값을 냈다 ({yaw})"
        assert st == ST_NOT_IMPLEMENTED, f"{src}: 상태가 {st}"


def test_cog_offset_is_implemented_now():
    """N2 로 구현됨 — 미구현이 아니라 '아직 수렴 안 함' 이어야 한다. 둘 다 침묵은 같다."""
    mux = HeadingMux(SRC_COG_OFFSET, stale_sec=10.0)
    mux.update_imu(123.0, 0.0)
    yaw, st = mux.heading(0.0)
    assert yaw is None
    assert st == ST_NOT_CONVERGED, st
    assert st in SILENT_STATES


def test_cog_does_not_leak_into_imu_relative():
    """작년 버그 재발 방지: COG 가 유효해도 imu_relative 를 덮어쓰면 안 된다."""
    mux = HeadingMux(SRC_IMU_RELATIVE, stale_sec=10.0)
    mux.update_imu(10.0, 0.0)
    mux.update_cog(200.0, 3.0, 0.0)
    yaw, st = mux.heading(0.0)
    assert st == ST_OK and near(yaw, 10.0), f"COG 가 새어들어왔다 ({yaw})"


def test_silent_states_all_yield_none():
    assert ST_NO_DATA in SILENT_STATES
    assert ST_STALE in SILENT_STATES
    assert ST_NOT_IMPLEMENTED in SILENT_STATES
    assert ST_OK not in SILENT_STATES





# ================================================================ N2: COG 유도
def test_cog_from_velocity_enu():
    """ENU: x=동, y=북. 정북 전진 → COG 0, 정동 → 90."""
    assert near(cog_from_velocity(0.0, 2.0)[0], 0.0), "정북"
    assert near(cog_from_velocity(2.0, 0.0)[0], 90.0), "정동"
    assert near(cog_from_velocity(0.0, -2.0)[0], 180.0), "정남"
    assert near(cog_from_velocity(-2.0, 0.0)[0], 270.0), "정서"
    assert near(cog_from_velocity(3.0, 4.0)[1], 5.0), "속력=벡터크기"


def test_cog_from_velocity_ned():
    assert near(cog_from_velocity(2.0, 0.0, frame="ned")[0], 0.0), "NED x=북"
    assert near(cog_from_velocity(0.0, 2.0, frame="ned")[0], 90.0), "NED y=동"


def test_cog_from_velocity_degenerate():
    """정지·NaN 은 COG 를 못 준다. 0.0(정북)을 대신 내면 안 된다."""
    assert cog_from_velocity(0.0, 0.0)[0] is None
    assert cog_from_velocity(float('nan'), 1.0)[0] is None
    assert cog_from_velocity(None, 1.0)[0] is None


def test_cog_from_velocity_unknown_frame_raises():
    try:
        cog_from_velocity(1.0, 1.0, frame="enu_maybe")
    except ValueError:
        return
    assert False, "모르는 프레임을 조용히 받아들이면 안 된다"


# ================================================================ N2: 추정기
def _feed(est, true_off, n=40, t0=0.0, dt=0.1, imu0=10.0,
          speed=2.0, turn=0.0, noise=None, crab=0.0):
    """imu_yaw 를 조금씩 바꿔가며 COG = imu + off + crab 로 표본 투입."""
    t = t0
    for i in range(n):
        imu = wrap360(imu0 + i * 1.3)          # 배가 서서히 방향을 바꾼다
        err = 0.0 if noise is None else noise[i % len(noise)]
        cog = wrap360(imu + true_off + crab + err)
        est.update(imu, cog, speed, turn, t, reverse=False)
        t += dt
    return t


def test_estimator_recovers_offset():
    est = COGOffsetEstimator(min_samples=30, half_life_sec=0.0)
    _feed(est, 37.0)
    assert est.converged, f"수렴 실패 (n={est.samples}, R={est.resultant:.3f})"
    assert abs(ang_diff(est.offset_deg, 37.0)) < 0.5, est.offset_deg


def test_estimator_wraparound_offset():
    """오프셋이 0/360 경계여도 맞아야 한다. 산술평균이었으면 여기서 180 이 나온다."""
    est = COGOffsetEstimator(min_samples=30, half_life_sec=0.0)
    _feed(est, 359.0, noise=[+2.0, -2.0])      # 표본이 1° 와 357° 를 오간다
    assert est.converged
    assert abs(ang_diff(est.offset_deg, 359.0)) < 1.0, est.offset_deg


def test_estimator_noise_tolerated():
    est = COGOffsetEstimator(min_samples=30, min_resultant=0.9, half_life_sec=0.0)
    _feed(est, 20.0, n=60, noise=[-8.0, 3.0, 9.0, -5.0, 0.0])
    assert est.converged, f"R={est.resultant:.3f}"
    assert abs(ang_diff(est.offset_deg, 20.0)) < 3.0, est.offset_deg


def test_estimator_rejects_slow():
    """느리면 COG 는 노이즈다 — 작년 드라이버가 g_speed 를 안 봐서 당한 지점."""
    est = COGOffsetEstimator(min_speed_mps=0.8)
    ok = est.update(10.0, 47.0, 0.2, 0.0, 0.0)
    assert not ok and est.last_reject == "slow"
    assert est.samples == 0.0


def test_estimator_rejects_turning():
    """선회 중엔 COG 가 뱃머리를 못 따라온다."""
    est = COGOffsetEstimator(max_turn_rate_dps=8.0)
    assert not est.update(10.0, 47.0, 2.0, 25.0, 0.0)
    assert est.last_reject == "turning"
    assert not est.update(10.0, 47.0, 2.0, -25.0, 0.0), "반대 방향 선회도"
    assert est.update(10.0, 47.0, 2.0, 3.0, 0.0), "완만하면 통과"


def test_estimator_rejects_nan():
    est = COGOffsetEstimator()
    assert not est.update(float('nan'), 47.0, 2.0, 0.0, 0.0)
    assert not est.update(10.0, None, 2.0, 0.0, 0.0)
    assert est.samples == 0.0


def test_estimator_not_converged_returns_none_not_zero():
    """수렴 전에 0.0 을 내면 '보정 없음' 과 구분이 안 된다 — 반드시 None."""
    est = COGOffsetEstimator(min_samples=30, half_life_sec=0.0)
    _feed(est, 37.0, n=5)
    assert not est.converged
    assert est.offset_deg is None


def test_estimator_reverse_motion_kills_confidence():
    """
    후진하면 COG 가 뱃머리와 180° 뒤집힌다. 절반씩 섞이면 R 이 무너져
    '못 믿겠다' 로 떨어져야 한다 — 조용히 중간값(90° 틀림)을 내면 최악이다.
    """
    est = COGOffsetEstimator(min_samples=10, min_resultant=0.9, half_life_sec=0.0)
    t = _feed(est, 30.0, n=20)                  # 전진
    _feed(est, 210.0, n=20, t0=t, imu0=99.0)    # 후진 = +180
    assert est.resultant < 0.5, f"R={est.resultant:.3f} — 뒤집힘을 못 잡았다"
    assert not est.converged
    assert est.offset_deg is None


def test_estimator_crab_is_absorbed_knowingly():
    """
    게걸음(crab)은 offset 에 흡수된다. 이건 버그가 아니라 옵션 B 의 원리적 한계다.
    '모르고 당하는 것' 과 '알고 쓰는 것' 을 가르려고 테스트로 박아둔다.
    """
    est = COGOffsetEstimator(min_samples=30, half_life_sec=0.0)
    _feed(est, 37.0, crab=12.0)
    assert est.converged
    got = est.offset_deg
    assert abs(ang_diff(got, 49.0)) < 0.5, \
        f"장착37 + 게걸음12 = 49 로 나와야 한다(흡수). 실제 {got}"


def test_estimator_decay_follows_change():
    """감쇠가 있으면 오프셋이 변해도 따라간다(IMU 드리프트 대응)."""
    # min_samples 는 포화상한(dt=0.1, half_life=1.0 → ≈14.9) 아래여야 한다
    est = COGOffsetEstimator(min_samples=10, half_life_sec=1.0)
    assert est.min_samples_reachable(0.1), "이 설정이면 애초에 수렴 불가"
    t = _feed(est, 10.0, n=40)
    assert abs(ang_diff(est.offset_deg, 10.0)) < 1.0
    _feed(est, 80.0, n=120, t0=t, imu0=200.0)
    assert abs(ang_diff(est.offset_deg, 80.0)) < 3.0, \
        f"새 오프셋을 못 따라감: {est.offset_deg}"


# ================================================================ N2: 후진 게이트
def test_reverse_only_would_converge_wrong_without_gate():
    """
    게이트가 없으면 어떤 일이 벌어지는지 못 박아 둔다.
    후진만 지속되면 표본이 서로 '일치' 해서 R=1.0 인데 값은 정확히 180° 틀린다.
    신뢰도로는 절대 못 막는다 — 이래서 별도 게이트가 필요하다.
    """
    est = COGOffsetEstimator(min_samples=30, half_life_sec=0.0,
                             require_reverse_gate=False)
    _feed(est, 37.0 + 180.0, n=60)          # 후진: COG = 뱃머리 + 180
    assert est.converged, "게이트 없으면 자신만만하게 수렴해버린다"
    assert est.resultant > 0.99, est.resultant
    assert abs(ang_diff(est.offset_deg, 37.0)) > 179.0, \
        f"정확히 180° 틀려야 한다: {est.offset_deg}"


def test_reverse_gate_blocks_samples():
    """지속 후진 중에는 표본수가 늘지 않아야 한다."""
    est = COGOffsetEstimator(min_samples=30, half_life_sec=0.0)
    t = 0.0
    for i in range(100):
        est.update(wrap360(10.0 + i * 0.3), wrap360(10.0 + i * 0.3 + 217.0),
                   2.0, 3.0, t, reverse=True)
        t += 0.2
    assert est.samples == 0.0, f"후진 표본이 새어들어갔다: n={est.samples}"
    assert est.last_reject == "reverse"
    assert not est.converged
    assert est.offset_deg is None


def test_reverse_gate_unknown_blocks_by_default():
    """모르면 안 모은다. None(신호 없음/stale) 은 배제."""
    est = COGOffsetEstimator(min_samples=30, half_life_sec=0.0)
    assert not est.update(10.0, 47.0, 2.0, 0.0, 0.0, reverse=None)
    assert est.last_reject == "no_reverse_gate"
    assert est.samples == 0.0


def test_reverse_gate_can_be_disabled():
    """헤딩만 벤치할 때(motor_control 없이) 끌 수 있어야 한다."""
    est = COGOffsetEstimator(min_samples=30, half_life_sec=0.0,
                             require_reverse_gate=False)
    assert est.update(10.0, 47.0, 2.0, 0.0, 0.0, reverse=None)
    assert est.samples == 1.0


def test_reverse_gate_resumes_after_reverse_ends():
    """후진이 끝나면 다시 모은다. 오염 없이 정상 수렴해야 한다."""
    est = COGOffsetEstimator(min_samples=30, half_life_sec=0.0)
    t = 0.0
    for i in range(50):                     # 후진 구간 — 전부 배제
        est.update(wrap360(10.0 + i * 0.3), wrap360(10.0 + i * 0.3 + 217.0),
                   2.0, 3.0, t, reverse=True)
        t += 0.2
    assert est.samples == 0.0
    _feed(est, 37.0, n=40, t0=t, dt=0.2)    # 전진 재개
    assert est.converged
    assert abs(ang_diff(est.offset_deg, 37.0)) < 0.5, est.offset_deg


def test_mux_reverse_state_stale_becomes_unknown():
    """게이트가 끊기면 True/False 가 아니라 '모름' 이 되어야 한다."""
    mux = HeadingMux(SRC_COG_OFFSET, stale_sec=0.5)
    assert mux.reverse_state(0.0) is None, "신호 없으면 모름"
    mux.update_reverse(False, 10.0)
    assert mux.reverse_state(10.2) is False
    assert mux.reverse_state(10.5) is False, "경계는 초과"
    assert mux.reverse_state(11.0) is None, "stale 이면 모름"


def test_mux_blocks_collection_without_reverse_gate():
    """믹서 통합: 게이트가 없으면 COG 가 아무리 와도 표본이 안 쌓인다."""
    mux = HeadingMux(SRC_COG_OFFSET, stale_sec=10.0,
                     estimator=COGOffsetEstimator(min_samples=30, half_life_sec=0.0))
    t = _run_mux(mux, n=60, with_gate=False)   # motor_control 이 안 뜬 상황
    assert mux.estimator.samples == 0.0
    assert mux.estimator.last_reject == "no_reverse_gate"
    assert mux.heading(t)[1] == ST_NOT_CONVERGED


def test_mux_collects_when_gate_says_forward():
    mux = HeadingMux(SRC_COG_OFFSET, stale_sec=10.0,
                     estimator=COGOffsetEstimator(min_samples=30, half_life_sec=0.0))
    t = 0.0
    for i in range(60):
        imu = wrap360(10.0 + i * 0.3)
        mux.update_reverse(False, t)
        mux.update_imu(imu, t)
        mux.update_cog(wrap360(imu + 37.0), 2.0, t)
        t += 0.1
    yaw, st = mux.heading(t)
    assert st == ST_OK, st
    assert abs(ang_diff(mux.estimator.offset_deg, 37.0)) < 0.5


def test_saturation_formula():
    """감쇠가 있으면 표본수가 포화한다. 공식을 못 박아 둔다."""
    assert saturation_samples(0.1, 0.0) == float("inf"), "감쇠 없으면 무한"
    n = saturation_samples(0.1, 1.0)
    assert 14.0 < n < 15.0, n
    assert saturation_samples(0.2, 60.0) > 400.0, "실사용 기본값은 넉넉하다"


def test_min_samples_unreachable_is_detectable():
    """
    min_samples 가 포화상한보다 크면 영원히 NOT_CONVERGED 다. 에러도 안 난다.
    조용히 안 죽는 유형이라 '감지 가능한가' 를 테스트로 박는다.
    """
    bad = COGOffsetEstimator(min_samples=20, half_life_sec=1.0)
    assert not bad.min_samples_reachable(0.1), "도달 불가를 감지 못 했다"
    _feed(bad, 30.0, n=500)                     # 500 표본을 넣어도
    assert not bad.converged, "포화상한을 넘을 수 없어야 한다"
    assert bad.samples < 15.0, bad.samples

    good = COGOffsetEstimator(min_samples=20, half_life_sec=60.0)
    assert good.min_samples_reachable(0.1)
    _feed(good, 30.0, n=60)
    assert good.converged


def test_estimator_decay_zero_never_forgets():
    est = COGOffsetEstimator(min_samples=10, half_life_sec=0.0)
    _feed(est, 15.0, n=20)
    n_before = est.samples
    est._decay_to(1e6)
    assert est.samples == n_before, "half_life=0 이면 감쇠하지 않아야 한다"


# ================================================================ N2: 믹서 통합
def _run_mux(mux, n=40, t0=0.0, dt=0.1, imu0=10.0, true_off=37.0, speed=2.0,
             yaw_step=0.3, with_gate=True):
    """
    yaw_step=0.3°/0.1s = 3°/s. 직진 중의 자연스러운 흔들림이다.
    ⚠️ 이걸 1.3 으로 두면 13°/s 라 max_turn_rate_dps(8) 에 걸려 전 표본이 배제된다.
       (실제로 이 테스트를 짜다 걸렸다 — 게이팅이 제대로 도는 증거)

    with_gate=False 면 /motor_reverse 를 안 준다 = motor_control 이 안 뜬 상황.
    """
    t = t0
    for i in range(n):
        imu = wrap360(imu0 + i * yaw_step)
        if with_gate:
            mux.update_reverse(False, t)
        mux.update_imu(imu, t)
        mux.update_cog(wrap360(imu + true_off), speed, t)
        t += dt
    return t


def test_mux_cog_offset_converges_and_publishes():
    mux = HeadingMux(SRC_COG_OFFSET, stale_sec=10.0,
                     estimator=COGOffsetEstimator(min_samples=30, half_life_sec=0.0))
    assert mux.heading(0.0)[1] == ST_NO_DATA
    t = _run_mux(mux)
    yaw, st = mux.heading(t)
    assert st == ST_OK, st
    imu_now = mux._imu[0]
    assert abs(ang_diff(yaw, wrap360(imu_now + 37.0))) < 0.5, yaw


def test_mux_cog_offset_silent_before_convergence():
    mux = HeadingMux(SRC_COG_OFFSET, stale_sec=10.0,
                     estimator=COGOffsetEstimator(min_samples=30, half_life_sec=0.0))
    t = _run_mux(mux, n=5)
    yaw, st = mux.heading(t)
    assert yaw is None and st == ST_NOT_CONVERGED
    assert ST_NOT_CONVERGED in SILENT_STATES


def test_mux_cog_offset_holds_heading_when_stopped():
    """
    옵션 B 의 핵심 이득: 수렴 후엔 배가 멈춰도 IMU 가 헤딩을 유지한다.
    (작년 GPS override 는 정확히 여기서 노이즈로 튀었다)
    """
    mux = HeadingMux(SRC_COG_OFFSET, stale_sec=10.0,
                     estimator=COGOffsetEstimator(min_samples=30, half_life_sec=0.0))
    t = _run_mux(mux)
    imu_hold = 123.0
    for _ in range(20):                       # 정지: IMU 만 오고 COG 는 안 옴
        t += 0.1
        mux.update_imu(imu_hold, t)
    yaw, st = mux.heading(t)
    assert st == ST_OK, st
    assert abs(ang_diff(yaw, wrap360(imu_hold + 37.0))) < 0.5, yaw


def test_mux_cog_offset_goes_stale_if_imu_dies():
    """COG 가 계속 와도 IMU 가 죽으면 침묵해야 한다 — IMU 가 본체다."""
    mux = HeadingMux(SRC_COG_OFFSET, stale_sec=0.5,
                     estimator=COGOffsetEstimator(min_samples=30, half_life_sec=0.0))
    t = _run_mux(mux)
    assert mux.heading(t)[1] == ST_OK
    assert mux.heading(t + 1.0)[1] == ST_STALE
    assert mux.heading(t + 1.0)[0] is None


def test_mux_drops_sample_when_imu_stale():
    """오래된 yaw 와 지금 COG 를 빼면 그 차이가 그대로 오차다 — 표본을 버려야 한다."""
    mux = HeadingMux(SRC_COG_OFFSET, stale_sec=0.5)
    mux.update_imu(10.0, 0.0)
    mux.update_cog(47.0, 2.0, 5.0)            # IMU 는 5초 전
    assert mux.estimator.samples == 0.0
    assert mux.estimator.last_reject == "imu_stale"


def test_mux_turn_rate_wraparound():
    """359→1 을 -358°/s 로 읽으면 멀쩡한 표본이 전부 'turning' 으로 버려진다."""
    mux = HeadingMux(SRC_COG_OFFSET, stale_sec=10.0)
    mux.update_imu(359.0, 0.0)
    mux.update_imu(1.0, 1.0)                  # 실제로는 +2°/s
    assert abs(mux._turn_rate_dps - 2.0) < 1e-6, mux._turn_rate_dps


def test_mux_estimator_runs_even_on_other_source():
    """
    다른 소스로 도는 동안에도 추정은 돌아야 한다 — 물 위에서 '지금 전환하면 쓸 만한가' 를
    미리 볼 수 있어야 하기 때문. 단, 출력은 imu_relative 규칙을 따른다.
    """
    mux = HeadingMux(SRC_IMU_RELATIVE, stale_sec=10.0,
                     estimator=COGOffsetEstimator(min_samples=30, half_life_sec=0.0))
    t = _run_mux(mux)
    assert mux.estimator.converged, "관찰 중이어야 한다"
    yaw, st = mux.heading(t)
    assert st == ST_OK
    assert abs(ang_diff(yaw, mux._imu[0])) < 1e-6, "출력엔 offset 이 섞이면 안 된다"


# ================================================================ cog_offset 보정 순서
#  (계약서_cog_offset보정순서_20261030)
#  물리 모델: 진짜 뱃머리 H = s·r + m (s=±1 invert, m mount), 게걸음 c, COG = H + c.
#  🚨 _run_mux 는 COG 를 원시 영역(imu + off)에서 만들어 invert·mount 이중 적용을 못 본다.
#     아래는 **진짜 H 에서** r = s·(H − m), COG = H + c 로 거꾸로 만든다.
def _raw_from_true(heading, invert, mount):
    """진짜 뱃머리 H → 센서 원시 r. s·r + m = H 를 r 에 대해 푼 것."""
    s = -1.0 if invert else 1.0
    return wrap360(s * (heading - mount))


def _cog_mux(invert, mount, decl=0.0):
    return HeadingMux(SRC_COG_OFFSET, invert_yaw=invert, mount_offset_deg=mount,
                      declination_deg=decl, stale_sec=10.0,
                      estimator=COGOffsetEstimator(min_samples=30, half_life_sec=0.0))


def _drive(mux, headings, *, invert, mount, crab=0.0, t0=0.0, dt=0.1, speed=2.0,
           on_step=None):
    """진짜 H 열을 따라 IMU·COG·후진게이트(전진)를 넣는다. on_step(H, t) 로 중간 관찰."""
    t = t0
    for h in headings:
        mux.update_reverse(False, t)
        mux.update_imu(_raw_from_true(h, invert, mount), t)
        mux.update_cog(wrap360(h + crab), speed, t)
        if on_step is not None:
            on_step(h, t)
        t += dt
    return t


def _straight(h0, n, wobble=0.3):
    """직진 중 자연스러운 흔들림 ±(3°/s 이하). 선회율 게이트(8°/s)에 안 걸린다."""
    return [wrap360(h0 + (wobble if i % 2 else 0.0)) for i in range(n)]


def _turn(h_from, h_to, step):
    """
    h_from → h_to 를 균등 간격(≤ step °/표본)으로 선회. 마지막 표본이 정확히 h_to.
    dt=0.1 이면 step 1.5 = 15°/s → 전 표본이 선회율 게이트(8°/s)에 걸린다.
    """
    d = ang_diff(h_to, h_from)
    k = max(1, math.ceil(abs(d) / step))
    return [wrap360(h_from + d * i / k) for i in range(1, k + 1)]


def test_cog_offset_mount_not_applied_twice():
    """1) mount=37, invert=F, H=90 → 90 (수정 전 127: off 가 흡수한 mount 를 _correct 가 또 더함)."""
    mux = _cog_mux(False, 37.0)
    t = _drive(mux, _straight(90.0, 40, wobble=0.0), invert=False, mount=37.0)
    yaw, st = mux.heading(t)
    assert st == ST_OK, st
    assert abs(ang_diff(yaw, 90.0)) < 0.5, f"mount 이중 적용? yaw={yaw}"
    assert abs(ang_diff(mux.estimator.offset_deg, 37.0)) < 0.5, mux.estimator.offset_deg


def test_cog_offset_invert_single_segment_east():
    """2) invert=T, mount=0, 동쪽 단일 구간 → 90 (수정 전 270)."""
    mux = _cog_mux(True, 0.0)
    t = _drive(mux, _straight(90.0, 40), invert=True, mount=0.0)
    yaw, st = mux.heading(t)
    assert st == ST_OK, st
    assert abs(ang_diff(yaw, 90.3)) < 0.5, f"invert 를 못 다뤘다: yaw={yaw}"


def test_cog_offset_invert_mount_north_then_east():
    """
    3) invert=T, mount=37, 북 → 동 각 40 표본 이상. 두 구간 끝 모두 OK·출력 H, off≈37.
       한 방향 직진은 invert 오설정도 통과하므로 두 구간이 필요하다(계약서 위험 3).
    """
    mux = _cog_mux(True, 37.0)
    north = _straight(0.0, 40, wobble=0.0)
    t = _drive(mux, north, invert=True, mount=37.0)
    yaw, st = mux.heading(t)
    assert st == ST_OK, f"북 구간 끝: {st}"
    assert abs(ang_diff(yaw, 0.0)) < 0.5, f"북 구간 끝 yaw={yaw}"
    assert abs(ang_diff(mux.estimator.offset_deg, 37.0)) < 0.5, mux.estimator.offset_deg

    t = _drive(mux, _turn(0.0, 90.0, 1.5), invert=True, mount=37.0, t0=t)
    t = _drive(mux, _straight(90.0, 40, wobble=0.0), invert=True, mount=37.0, t0=t)
    yaw, st = mux.heading(t)
    assert st == ST_OK, f"동 구간 끝: {st} R={mux.estimator.resultant:.3f}"
    assert abs(ang_diff(yaw, 90.0)) < 0.5, f"동 구간 끝 yaw={yaw}"
    assert abs(ang_diff(mux.estimator.offset_deg, 37.0)) < 0.5, mux.estimator.offset_deg
    assert mux.estimator.resultant > 0.99, "두 구간 표본이 같은 off 여야 한다"


def test_cog_offset_grid_output_is_heading_plus_crab():
    """4) invert{F,T} × mount{0,37} × c{0,5} → 출력 = H + c. off = mount + c."""
    for invert in (False, True):
        for mount in (0.0, 37.0):
            for crab in (0.0, 5.0):
                tag = f"invert={invert} mount={mount} c={crab}"
                mux = _cog_mux(invert, mount)
                t = _drive(mux, _straight(0.0, 40, wobble=0.0), invert=invert,
                           mount=mount, crab=crab)
                t = _drive(mux, _turn(0.0, 123.0, 1.5), invert=invert, mount=mount,
                           crab=crab, t0=t)
                t = _drive(mux, _straight(123.0, 40), invert=invert, mount=mount,
                           crab=crab, t0=t)
                yaw, st = mux.heading(t)
                assert st == ST_OK, f"{tag}: {st}"
                # 마지막 표본은 _straight 의 홀수 번째(123.3)
                assert abs(ang_diff(yaw, 123.3 + crab)) < 0.5, f"{tag}: yaw={yaw}"
                assert abs(ang_diff(mux.estimator.offset_deg, mount + crab)) < 0.5, \
                    f"{tag}: off={mux.estimator.offset_deg}"


def test_cog_offset_invert_tracks_heading_during_fast_turn():
    """
    5) invert=T, 0° 수렴 뒤 >8°/s 로 90° 선회 — 선회 중 표본은 버려지므로 출력은
       IMU 가 혼자 끌고 간다. 매 표본 출력이 H 를 따라가야 한다.
       (수정 전 출력 = H − 2H₀ − c + m → H₀=0, c=0 이면 m 만큼 상수 오차. 그래서 mount=37)
    """
    mux = _cog_mux(True, 37.0)
    t = _drive(mux, _straight(0.0, 40, wobble=0.0), invert=True, mount=37.0)
    assert mux.heading(t)[1] == ST_OK
    n_before = mux.estimator.samples
    errs = []

    def watch(h, now):
        yaw, st = mux.heading(now)
        assert st == ST_OK, f"선회 중 H={h}: {st}"
        errs.append((h, ang_diff(yaw, h)))

    _drive(mux, _turn(0.0, 90.0, 1.5), invert=True, mount=37.0, t0=t, on_step=watch)
    bad = [(h, e) for h, e in errs if abs(e) >= 0.5]
    assert not bad, f"선회 중 출력이 H 를 못 따라감 (H, 오차): {bad[:5]}"
    assert abs(ang_diff(errs[-1][0], 90.0)) < 1e-9, "90° 까지 돌았어야 한다"
    # 선회 중 표본은 전부 버려졌다 = 출력은 IMU 가 혼자 끌고 간 것
    assert mux.estimator.samples == n_before, mux.estimator.samples
    assert mux.estimator.last_reject == "turning", mux.estimator.last_reject


def test_cog_offset_ignores_declination():
    """6) declination −8 이 cog_offset 출력에 안 들어간다 (COG 는 진북 기준)."""
    for invert, mount in ((False, 0.0), (True, 37.0)):
        mux = _cog_mux(invert, mount, decl=-8.0)
        t = _drive(mux, _straight(200.0, 40, wobble=0.0), invert=invert, mount=mount)
        yaw, st = mux.heading(t)
        assert st == ST_OK, st
        assert abs(ang_diff(yaw, 200.0)) < 0.5, \
            f"invert={invert} mount={mount}: 편각·보정이 새어들어감 yaw={yaw}"


def test_imu_relative_bit_identical_with_cog_input():
    """
    7) imu_relative 는 이번 변경에 비트 단위로 불변. _imu 슬롯은 원시값,
       출력은 apply_corrections(wrap360(r)) 와 == (슬롯이 수신 시 wrap360 하므로).
       COG 가 들어와 추정이 수렴한 중에도 같아야 한다.
    """
    for invert in (False, True):
        for mount in (0.0, 37.0, 359.9, -12.5):
            for r in (0.0, 0.1, 89.9, 180.0, 359.999, -10.0):
                want = apply_corrections(wrap360(r), invert_yaw=invert,
                                         mount_offset_deg=mount)
                tag = f"invert={invert} mount={mount} r={r}"

                mux = HeadingMux(SRC_IMU_RELATIVE, invert_yaw=invert,
                                 mount_offset_deg=mount, declination_deg=-8.0,
                                 stale_sec=10.0,
                                 estimator=COGOffsetEstimator(min_samples=30,
                                                              half_life_sec=0.0))
                mux.update_imu(r, 0.0)
                assert mux._imu[0] == wrap360(r), f"{tag}: 슬롯이 원시값이 아니다"
                yaw, st = mux.heading(0.0)
                assert st == ST_OK and yaw == want, f"{tag}: {yaw} != {want}"

                # COG 입력 중: 추정을 수렴시킨 뒤 같은 r 로 다시
                t = _drive(mux, _straight(50.0, 40), invert=invert, mount=mount,
                           crab=5.0, t0=1.0)
                assert mux.estimator.converged, tag
                mux.update_imu(r, t)
                mux.update_cog(77.0, 2.0, t)
                assert mux._imu[0] == wrap360(r), f"{tag}: 슬롯이 원시값이 아니다(COG 중)"
                yaw, st = mux.heading(t)
                assert st == ST_OK and yaw == want, f"{tag} (COG 중): {yaw} != {want}"

                # gps_cog 입력(계약서_GPS방위_20261114)이 들어와도 같아야 한다:
                # 선회 신호·초기 방위·정지(cog None)·후진 — imu_relative 출력엔 무관
                for code in (SPIN_RIGHT, SPIN_LEFT, 20000.0, 50000.0, 12.0):
                    mux.update_desired_angle(code, t)
                    mux.update_goal_bearing(222.0, t)
                    mux.update_cog(None, 0.0, t)
                    mux.update_reverse(True, t)
                    mux.update_cog(300.0, 3.0, t)
                    yaw, st = mux.heading(t)
                    assert st == ST_OK and yaw == want, \
                        f"{tag} (gps_cog 입력 {code}): {yaw} != {want}"

                # /boat_mode 입력(보완 A — AUTO 진입 초기화)도 imu_relative 출력엔 무관
                for bm in (0, 1, BOAT_MODE_AUTO, BOAT_MODE_AUTO, 1, BOAT_MODE_AUTO, 0, 7):
                    mux.update_boat_mode(bm)
                    yaw, st = mux.heading(t)
                    assert st == ST_OK and yaw == want, \
                        f"{tag} (boat_mode {bm}): {yaw} != {want}"

                # 보완 B: gps_cog 추적기가 뒤에서 IMU 보조(ASSIST)를 돌려도 imu_relative 출력은 불변.
                # 추적기에 앵커를 세우고(settle 지난 뒤 전진 COG) 선회 코드로 보조를 켠 채 IMU 를 돌린다.
                tb = t + 10.0
                mux.update_imu(r, tb)
                mux.update_reverse(False, tb)
                mux.update_desired_angle(12.0, tb)
                mux.update_cog(100.0, 2.0, tb)
                assert mux.gps_cog.heading(tb)[1] == ST_OK, f"{tag}: 추적기 앵커 없음"
                for k, r2 in enumerate((r, r + 5.0, r + 9.5, r + 400.0, r - 7.0, r)):
                    tk = tb + 0.01 * (k + 1)
                    mux.update_desired_angle(SPIN_RIGHT, tk)
                    mux.update_reverse(k % 2 == 0, tk)
                    mux.update_imu(r2, tk)
                    want2 = apply_corrections(wrap360(r2), invert_yaw=invert,
                                              mount_offset_deg=mount)
                    yaw, st = mux.heading(tk)
                    assert mux.gps_cog.assist_active, f"{tag}: 보조가 안 돌았다(검사 무의미)"
                    assert st == ST_OK and yaw == want2, \
                        f"{tag} (gps_cog 보조 중 r={r2}): {yaw} != {want2}"


def test_cog_offset_invert_turning_reject_uses_abs():
    """8) invert=T 에서도 선회 배제는 abs(선회율) — 좌·우 선회 모두 'turning'."""
    for direction in (+1.0, -1.0):
        mux = _cog_mux(True, 37.0)
        t = _drive(mux, _straight(0.0, 5, wobble=0.0), invert=True, mount=37.0)
        n_before = mux.estimator.samples
        _drive(mux, _turn(0.0, direction * 60.0, 2.0), invert=True, mount=37.0, t0=t)
        assert mux.estimator.last_reject == "turning", \
            f"방향 {direction}: last_reject={mux.estimator.last_reject}"
        assert abs(mux._turn_rate_dps) > 8.0, mux._turn_rate_dps
        assert mux.estimator.samples == n_before, \
            f"방향 {direction}: 선회 표본이 새어들어감 n={mux.estimator.samples}"
        # 같은 배에서 완만한 변화(3°/s)는 통과해야 한다 — 게이트가 다 막는 게 아니다
        _drive(mux, _straight(direction * 60.0, 4), invert=True, mount=37.0, t0=t + 10.0)
        assert mux.estimator.last_reject is None, mux.estimator.last_reject
        assert mux.estimator.samples > n_before


# ================================================================ gps_cog (계약서_GPS방위_20261114)
# 보완 B 메타 테스트가 False 로 바꿔 C1·보완 A 테스트를 IMU 보조 off 로 다시 돌린다.
_GMUX_ASSIST = [True]


def _gmux(require_gate=True, imu_assist=None, imu_assist_max_sec=10.0,
          imu_max_step_deg=10.0, **mux_kw):
    """계약 기본값(공통 yaml 과 같은 값)으로 gps_cog 믹서. IMU stale = 믹서 stale_sec (B-Q3)."""
    mux_kw.setdefault('stale_sec', 0.5)
    trk = GPSCOGTracker(min_speed_mps=0.5, fresh_sec=1.5, predict_max_sec=1.5,
                        max_rate_dps=30.0, settle_sec=2.0, jump_deg=90.0,
                        init=GPS_COG_INIT_GOAL, init_stale_sec=2.0,
                        require_gate=require_gate,
                        imu_assist=_GMUX_ASSIST[0] if imu_assist is None else imu_assist,
                        imu_assist_max_sec=imu_assist_max_sec,
                        imu_max_step_deg=imu_max_step_deg,
                        imu_stale_sec=mux_kw['stale_sec'])
    return HeadingMux(SRC_GPS_COG, gps_cog=trk, **mux_kw)


def _gfeed(mux, cog, t, speed=1.0, reverse=False, desired=12.0):
    """게이트(전진·선회 아님) 신선하게 주고 COG 표본 하나. 채택 여부를 돌려준다."""
    if reverse is not None:
        mux.update_reverse(reverse, t)
    if desired is not None:
        mux.update_desired_angle(desired, t)
    before = mux.gps_cog.age(t)
    mux.update_cog(cog, speed, t)
    return mux.gps_cog.age(t) == 0.0 and before != 0.0


def test_gps_cog_is_known_true_north_source():
    assert SRC_GPS_COG in ALL_SOURCES
    assert SOURCE_IS_MAGNETIC[SRC_GPS_COG] is False, "COG 는 진북 — 편각 이중 보정 금지"


def test_gps_cog_publish_and_silent_states_disjoint():
    """HOLD·INIT_GOAL 은 발행, INIT 은 침묵. 겹치면 발행 판정이 모호해진다."""
    assert ST_HOLD in PUBLISH_STATES and ST_INIT_GOAL in PUBLISH_STATES
    assert ST_OK in PUBLISH_STATES
    assert ST_HOLD not in SILENT_STATES and ST_INIT_GOAL not in SILENT_STATES
    assert ST_INIT in SILENT_STATES and ST_INIT not in PUBLISH_STATES
    assert not (set(PUBLISH_STATES) & set(SILENT_STATES))


def test_gps_cog_unknown_init_raises():
    try:
        GPSCOGTracker(init="imu")
    except ValueError:
        return
    assert False, "모르는 gps_cog_init 을 조용히 받아들이면 안 된다"


def test_spin_codes_match_contract_table():
    """CLAUDE.md §3 특수 신호표: 5000 우선회, 6000 좌선회 (ssf-reviewer 6번 점검)."""
    assert SPIN_RIGHT == 5000.0 and SPIN_LEFT == 6000.0
    assert is_spin_code(5000.0) and is_spin_code(6000.0)
    assert is_spin_code(5000.0001), "float32 왕복 오차"
    for v in (20000.0, 50000.0, 260.0, 0.0, -30.0, float('nan'), None):
        assert not is_spin_code(v), v


# ---- 1) 첫 COG 전 (Q1 (c))
def test_gps_cog_init_goal_before_first_cog():
    mux = _gmux()
    yaw, st = mux.heading(0.0)
    assert yaw is None and st == ST_INIT, "tp 도 없으면 침묵"

    mux.update_goal_bearing(123.0, 0.0)
    yaw, st = mux.heading(0.1)
    assert st == ST_INIT_GOAL and yaw == 123.0, (yaw, st)
    mux.update_goal_bearing(-10.0, 0.5)               # tp 를 따라간다(감아서)
    assert mux.heading(0.6) == (350.0, ST_INIT_GOAL)
    assert mux.heading(2.5) == (350.0, ST_INIT_GOAL), "경계 = 초과"
    yaw, st = mux.heading(2.5001)
    assert yaw is None and st == ST_INIT, "tp stale 이면 다시 침묵"

    # 채택 못 받는 표본(느림·후진)은 INIT_GOAL 을 안 바꾼다
    mux.update_goal_bearing(40.0, 3.0)
    _gfeed(mux, 200.0, 3.0, speed=0.2)
    _gfeed(mux, 200.0, 3.1, reverse=True)
    assert mux.heading(3.2) == (40.0, ST_INIT_GOAL)

    # 첫 COG 채택 → 그 뒤로 tp 는 안 쓴다 (첫 표본은 jump 검사 없음 — tp 는 측정이 아니다)
    _gfeed(mux, 250.0, 6.0)
    assert mux.heading(6.0) == (250.0, ST_OK)
    mux.update_goal_bearing(10.0, 6.5)
    assert mux.heading(6.6)[0] == 250.0


# ---- 2) 느림 → HOLD, 값 유지, 계속 발행
def test_gps_cog_slow_holds_forever_and_keeps_publishing():
    mux = _gmux()
    assert _gfeed(mux, 45.0, 0.0)
    assert _gfeed(mux, 45.0, 1.0)
    assert mux.heading(2.5) == (45.0, ST_OK), "fresh 경계 = 초과"
    assert mux.heading(2.5001) == (45.0, ST_HOLD)
    t = 2.0
    while t < 3600.0:                               # 한 시간 정지
        _gfeed(mux, 200.0, t, speed=0.3)            # 느린 COG 는 노이즈
        mux.update_cog(None, 0.0, t + 0.5)          # 속도 0 (cog_from_velocity → None)
        t += 1.0
        yaw, st = mux.heading(t)
        assert yaw == 45.0 and st == ST_HOLD, (t, yaw, st)
        assert st in PUBLISH_STATES
    assert mux.gps_cog.last_reject == "slow"
    assert mux.gps_cog.age(t) > 3500.0


# ---- 3) 후진 True / 모름 → 갱신 안 함
def test_gps_cog_reverse_true_or_unknown_blocks_update():
    mux = _gmux()
    assert _gfeed(mux, 45.0, 0.0)
    assert not _gfeed(mux, 225.0, 1.0, reverse=True)
    assert mux.gps_cog.last_reject == "reverse"
    assert mux.heading(1.0)[0] == 45.0

    mux2 = _gmux()                                  # motor_control 이 안 뜬 상황
    assert not _gfeed(mux2, 45.0, 0.0, reverse=None)
    assert mux2.gps_cog.last_reject == "no_reverse_gate"
    assert mux2.heading(0.0) == (None, ST_INIT)
    # 신호가 끊겨 stale 이 돼도 '모름'
    mux2.update_reverse(False, 1.0)
    mux2.update_desired_angle(12.0, 1.6)
    mux2.update_cog(45.0, 1.0, 1.6)                 # 후진 신호 0.6 s 전 → stale
    assert mux2.gps_cog.last_reject == "no_reverse_gate"


def test_gps_cog_gate_off_accepts_unknown():
    """벤치: cog_require_reverse_gate:=false 하나로 후진·선회 게이트 둘 다 끈다 (Q5)."""
    mux = _gmux(require_gate=False)
    assert _gfeed(mux, 45.0, 0.0, reverse=None, desired=None)
    assert mux.heading(0.0) == (45.0, ST_OK)
    # 게이트를 꺼도 '알려진' 후진·선회는 여전히 막는다
    assert not _gfeed(mux, 225.0, 1.0, reverse=True, desired=None)


# ---- 4) 5000/6000 수신 중 + settle 동안 갱신 안 함
def test_gps_cog_spin_and_settle_block_update():
    for code in (SPIN_RIGHT, SPIN_LEFT):
        mux = _gmux()
        assert _gfeed(mux, 45.0, 0.0)
        assert not _gfeed(mux, 100.0, 1.0, desired=code)
        assert mux.gps_cog.last_reject == "spin"
        mux.update_desired_angle(code, 1.4)          # 마지막 선회 명령
        mux.update_desired_angle(12.0, 1.5)          # 선회 끝
        for t in (2.0, 3.0, 3.39):
            assert not _gfeed(mux, 100.0, t), (code, t)
            assert mux.gps_cog.last_reject == "settle"
            assert mux.heading(t)[0] == 45.0
        assert _gfeed(mux, 100.0, 3.45), "settle 2.0 s 경과(1.4 + 2.0) → 채택"
        assert mux.heading(3.45) == (100.0, ST_OK)


def test_gps_cog_reverse_end_starts_settle():
    mux = _gmux()
    assert _gfeed(mux, 45.0, 0.0)
    mux.update_reverse(True, 0.5)
    mux.update_reverse(False, 0.6)
    assert not _gfeed(mux, 225.0, 1.5)              # 후진 관성으로 뒤로 밀리는 중
    assert mux.gps_cog.last_reject == "settle"
    assert mux.heading(1.5)[0] == 45.0


def test_gps_cog_desired_unknown_blocks_update():
    mux = _gmux()
    assert not _gfeed(mux, 45.0, 0.0, desired=None)
    assert mux.gps_cog.last_reject == "no_spin_gate"
    assert mux.spin_state(0.0) is None
    mux.update_desired_angle(12.0, 1.0)
    assert mux.spin_state(1.5) is False
    assert mux.spin_state(1.5001) is None, "stale 이면 모름"


# ---- 5) 359→1 랩어라운드 회전율
def test_gps_cog_rate_wraparound():
    mux = _gmux()
    assert _gfeed(mux, 359.0, 0.0)
    assert _gfeed(mux, 1.0, 1.0)                    # +2°/s 이지 −358°/s 가 아니다
    assert near(mux.heading(1.5)[0], 2.0), mux.heading(1.5)
    mux = _gmux()
    assert _gfeed(mux, 1.0, 0.0)
    assert _gfeed(mux, 359.0, 1.0)
    assert near(mux.heading(1.5)[0], 358.0), mux.heading(1.5)


# ---- 6) 외삽 상한·회전율 클램프
def test_gps_cog_predict_stops_at_max_and_rate_clamped():
    mux = _gmux()
    assert _gfeed(mux, 0.0, 0.0)
    assert _gfeed(mux, 20.0, 1.0)                   # 20°/s
    assert near(mux.heading(1.5)[0], 30.0)
    assert near(mux.heading(2.5)[0], 50.0), "predict_max 1.5 s 까지"
    assert near(mux.heading(100.0)[0], 50.0), "그 뒤엔 그 자리 값 유지"
    assert mux.heading(100.0)[1] == ST_HOLD

    mux = _gmux()
    assert _gfeed(mux, 0.0, 0.0)
    assert _gfeed(mux, 80.0, 1.0)                   # 80°/s → 30°/s 로 클램프
    assert near(mux.heading(2.0)[0], 110.0), mux.heading(2.0)
    mux = _gmux()
    assert _gfeed(mux, 80.0, 0.0)
    assert _gfeed(mux, 0.0, 1.0)                    # −80°/s → −30°/s
    assert near(mux.heading(2.0)[0], 330.0), mux.heading(2.0)

    mux = _gmux()
    assert _gfeed(mux, 0.0, 0.0)
    assert _gfeed(mux, 20.0, 1.6)                   # 간격 > fresh → 짝 없음, 회전율 0
    assert near(mux.heading(2.5)[0], 20.0)


# ---- 7) 거부·선회 시작 순간 출력 연속
def test_gps_cog_freeze_is_continuous():
    for how in ("slow", "zero", "spin_msg", "reverse_msg", "jump"):
        mux = _gmux()
        assert _gfeed(mux, 0.0, 0.0)
        assert _gfeed(mux, 20.0, 1.0)               # 20°/s 외삽 중
        before = mux.heading(1.4999)[0]
        if how == "slow":
            _gfeed(mux, 20.0, 1.5, speed=0.1)
        elif how == "zero":
            mux.update_cog(None, 0.0, 1.5)
        elif how == "spin_msg":
            mux.update_desired_angle(SPIN_RIGHT, 1.5)    # 선회 시작 — GPS 표본 기다리지 않고 얼림
        elif how == "reverse_msg":
            mux.update_reverse(True, 1.5)
        else:
            _gfeed(mux, 200.0, 1.5)                 # 180° 튐 1회
        at = mux.heading(1.5)[0]
        assert near(at, 30.0, 1e-6), (how, at)
        assert abs(ang_diff(at, before)) < 0.01, f"{how}: 튐 {before} → {at}"
        for t in (1.6, 2.0, 5.0, 60.0):
            assert near(mux.heading(t)[0], 30.0, 1e-6), f"{how}: 얼림이 안 됐다 t={t}"


def test_gps_cog_jump_needs_two_in_a_row():
    mux = _gmux()
    assert _gfeed(mux, 0.0, 0.0)
    assert not _gfeed(mux, 180.0, 1.0)
    assert mux.gps_cog.last_reject == "jump" and mux.heading(1.0)[0] == 0.0
    assert _gfeed(mux, 181.0, 2.0), "2회 연속 → 채택"
    assert mux.heading(2.0)[0] == 181.0

    # 사이에 정상 표본이 끼면 연속이 끊긴다
    mux = _gmux()
    assert _gfeed(mux, 0.0, 0.0)
    assert not _gfeed(mux, 180.0, 1.0)
    assert _gfeed(mux, 5.0, 2.0)
    assert not _gfeed(mux, 180.0, 3.0)
    # 사이에 다른 거부(느림)가 끼어도 끊긴다
    mux = _gmux()
    assert _gfeed(mux, 0.0, 0.0)
    assert not _gfeed(mux, 180.0, 1.0)
    assert not _gfeed(mux, 180.0, 2.0, speed=0.1)
    assert not _gfeed(mux, 180.0, 3.0)
    assert _gfeed(mux, 180.0, 4.0)
    # 튄 두 표본끼리도 jump_deg 안이어야 한다
    mux = _gmux()
    assert _gfeed(mux, 0.0, 0.0)
    assert not _gfeed(mux, 150.0, 1.0)
    assert not _gfeed(mux, 260.0, 2.0), "150 과 260 은 110° 차 — 연속으로 안 친다"
    assert _gfeed(mux, 250.0, 3.0)
    # 정확히 90° 는 '넘게' 가 아니다
    mux = _gmux()
    assert _gfeed(mux, 0.0, 0.0)
    assert _gfeed(mux, 90.0, 3.0)


# ---- 8) mount·invert·편각을 줘도 출력 = COG
def test_gps_cog_ignores_mount_invert_declination():
    for invert in (False, True):
        mux = _gmux(invert_yaw=invert, mount_offset_deg=40.3, declination_deg=-8.0)
        mux.update_imu(10.0, 0.0)
        mux.update_goal_bearing(77.0, 0.0)
        assert mux.heading(0.0) == (77.0, ST_INIT_GOAL), "INIT_GOAL 에도 안 건다"
        assert _gfeed(mux, 123.0, 0.0)
        assert mux.heading(0.0) == (123.0, ST_OK), (invert, mux.heading(0.0))


# ---- 9) IMU 를 줬다 끊어도 출력 불변 (IMU 슬롯을 안 읽는다)
def test_gps_cog_does_not_read_imu():
    seq = [(0.0, 10.0), (1.0, 14.0), (2.0, 17.0), (3.0, 17.5)]
    a = _gmux()
    b = _gmux()
    for t, c in seq:
        a.update_imu(300.0 + t * 37.0, t)           # IMU 는 엉뚱한 값을 내다가
        _gfeed(a, c, t)
        _gfeed(b, c, t)
    probes = [3.0, 3.5, 4.0, 4.6, 10.0]             # 3.0 뒤로 IMU 끊김(stale)
    for t in probes:
        assert a.heading(t) == b.heading(t), (t, a.heading(t), b.heading(t))
    a._imu = "읽으면 터진다"                         # 슬롯을 읽으면 예외
    for t in probes:
        assert a.heading(t) == b.heading(t)


# ---- 11) ENU 정북·정동 벡터 → 믹서 출력
def test_gps_cog_enu_north_and_east():
    for (vx, vy), want in (((0.0, 1.0), 0.0), ((1.0, 0.0), 90.0),
                           ((0.0, -1.0), 180.0), ((-1.0, 0.0), 270.0)):
        mux = _gmux()
        mux.update_reverse(False, 0.0)
        mux.update_desired_angle(12.0, 0.0)
        cog, spd = cog_from_velocity(vx, vy, "enu")
        mux.update_cog(cog, spd, 0.0)
        yaw, st = mux.heading(0.0)
        assert st == ST_OK and near(yaw, want), ((vx, vy), yaw, st)
    # 정지 벡터 → COG 정의 안 됨 → 채택 안 함 (0.0 = 정북을 대신 내지 않는다)
    mux = _gmux()
    mux.update_reverse(False, 0.0)
    mux.update_desired_angle(12.0, 0.0)
    cog, spd = cog_from_velocity(0.0, 0.0, "enu")
    mux.update_cog(cog, spd, 0.0)
    assert mux.heading(0.0) == (None, ST_INIT)
    assert mux.gps_cog.last_reject == "slow"


# ================================================================ 보완 A: AUTO 진입 초기화
def _manual_cog(mux, cog=40.0, t0=0.0, n=3):
    """MANUAL 로 들고 걷는 중 COG 채택 (C1 동작 그대로 채택된다). 마지막 시각을 돌려준다."""
    t = t0
    for i in range(n):
        t = t0 + i
        assert _gfeed(mux, cog, t), (cog, t)
    return t


# ---- (1) MANUAL 중 COG 채택 → AUTO 진입 → INIT_GOAL(tp)
def test_auto_entry_resets_to_init_goal():
    mux = _gmux()
    assert mux.update_boat_mode(1) is False
    t = _manual_cog(mux, 40.0)
    mux.update_goal_bearing(200.0, t)
    assert mux.heading(t)[1] == ST_OK, "MANUAL 중 COG 채택은 그대로(관측·경계용)"
    assert near(mux.heading(t)[0], 40.0)

    assert mux.update_boat_mode(BOAT_MODE_AUTO) is True
    assert mux.heading(t) == (200.0, ST_INIT_GOAL), mux.heading(t)
    assert mux.gps_cog.age(t) is None, "'첫 COG 받음' 표시가 남아 있다"
    # 시간이 지나도 HOLD 로 '걸어온 방향' 이 되살아나지 않는다
    mux.update_goal_bearing(201.0, t + 1.0)
    assert mux.heading(t + 1.0) == (201.0, ST_INIT_GOAL)
    # 진입 뒤 첫 COG → OK
    assert _gfeed(mux, 195.0, t + 2.0)
    assert mux.heading(t + 2.0) == (195.0, ST_OK)


# ---- (2) 첫 수신이 AUTO 면 진입
def test_auto_entry_on_first_message():
    mux = _gmux()
    t = _manual_cog(mux, 40.0)
    mux.update_goal_bearing(10.0, t)
    assert mux.boat_mode is None
    assert mux.update_boat_mode(BOAT_MODE_AUTO) is True
    assert mux.heading(t) == (10.0, ST_INIT_GOAL)
    assert mux.boat_mode == BOAT_MODE_AUTO


# ---- (3) AUTO→AUTO 반복은 재초기화 안 함
def test_auto_repeat_does_not_reset():
    mux = _gmux()
    mux.update_boat_mode(BOAT_MODE_AUTO)
    t = _manual_cog(mux, 40.0)
    mux.update_goal_bearing(200.0, t)
    for _ in range(10):                         # ssf_bridge 10 Hz 반복 발행
        assert mux.update_boat_mode(BOAT_MODE_AUTO) is False
    yaw, st = mux.heading(t)
    assert st == ST_OK and near(yaw, 40.0), (yaw, st)


# ---- (4) AUTO→MANUAL→AUTO 재진입마다 초기화 (WAIT 경유도)
def test_auto_reentry_resets_each_time():
    mux = _gmux()
    assert mux.update_boat_mode(BOAT_MODE_AUTO) is True
    t = 0.0
    for k, via in enumerate((1, 0, 1)):
        t = _manual_cog(mux, 40.0 + k, t0=t + 1.0)   # AUTO 중 COG 채택 → OK
        assert mux.heading(t)[1] == ST_OK
        assert mux.update_boat_mode(via) is False, "AUTO 해제는 초기화 아님"
        assert mux.heading(t)[1] == ST_OK, "해제 순간엔 그대로"
        mux.update_goal_bearing(300.0 + k, t)
        assert mux.update_boat_mode(BOAT_MODE_AUTO) is True, f"{k}번째 재진입"
        assert mux.heading(t) == (300.0 + k, ST_INIT_GOAL), (k, mux.heading(t))


# ---- (5) /boat_mode 미수신이면 C1 과 같다
def test_no_boat_mode_is_c1_behaviour():
    a = _gmux()
    b = _gmux()
    b.update_boat_mode(1)                       # MANUAL 만 계속 — 진입 없음
    seq = [(0.0, 40.0), (1.0, 42.0), (2.0, 44.0), (3.0, 45.0)]
    for t, c in seq:
        a.update_goal_bearing(200.0, t)
        b.update_goal_bearing(200.0, t)
        _gfeed(a, c, t)
        _gfeed(b, c, t)
    assert a.boat_mode is None
    for t in (3.0, 3.7, 5.0, 30.0):
        assert a.heading(t) == b.heading(t), (t, a.heading(t), b.heading(t))
    assert a.heading(30.0)[1] == ST_HOLD, "미수신이면 걸어온 방향 HOLD 그대로(C1)"


# ---- (6) 진입 때 tp stale 이면 INIT 침묵
def test_auto_entry_with_stale_goal_is_silent():
    mux = _gmux()
    mux.update_goal_bearing(200.0, 0.0)
    t = _manual_cog(mux, 40.0, t0=1.0)          # t=3.0, tp 는 3 s 묵음(init_stale 2.0 초과)
    assert mux.update_boat_mode(BOAT_MODE_AUTO) is True
    assert mux.heading(t) == (None, ST_INIT), mux.heading(t)
    # tp 가 다시 오면 INIT_GOAL
    mux.update_goal_bearing(210.0, t + 0.5)
    assert mux.heading(t + 0.5) == (210.0, ST_INIT_GOAL)


# ---- (7) 진입 뒤 첫 COG 는 jump 검사 없이 채택, 진입 전 jump 대기 표본은 버린다
def test_auto_entry_first_cog_skips_jump_and_drops_pending():
    mux = _gmux()
    mux.update_boat_mode(0)
    assert _gfeed(mux, 0.0, 0.0)
    assert not _gfeed(mux, 180.0, 1.0), "MANUAL 중 180° 튐 1회 = jump 대기"
    assert mux.gps_cog._pending is not None
    mux.update_goal_bearing(270.0, 1.0)
    assert mux.update_boat_mode(BOAT_MODE_AUTO)
    assert mux.gps_cog._pending is None, "jump 대기 표본이 남았다"
    # 진입 전 앵커(0°)와 비교하지 않는다 — 90° 넘게 달라도 한 번에 채택
    assert _gfeed(mux, 100.0, 2.0)
    assert mux.heading(2.0) == (100.0, ST_OK)
    # 진입 전 대기 표본(180°)이 '2회 연속' 짝으로 쓰이지 않는다: 200° 는 앵커 100° 와 100° 차 → jump 대기 1회째
    assert not _gfeed(mux, 200.0, 3.0)
    assert mux.gps_cog.last_reject == "jump"
    # 회전율 짝도 지워졌다: 진입 직후 첫 표본의 회전율은 0
    m2 = _gmux()
    assert _gfeed(m2, 0.0, 0.0)
    assert _gfeed(m2, 10.0, 1.0)                # 10°/s 짝
    m2.update_boat_mode(BOAT_MODE_AUTO)
    assert _gfeed(m2, 20.0, 1.5)
    assert m2.heading(2.5) == (20.0, ST_OK), "진입 전 표본과 회전율 짝을 지었다"


# ---- (8) 진입 순간 후진·선회 중이면 게이트·settle 유지
def test_auto_entry_keeps_reverse_and_spin_gates():
    # 후진 중 진입
    mux = _gmux()
    _manual_cog(mux, 40.0)                      # t=0,1,2
    mux.update_goal_bearing(200.0, 2.5)
    mux.update_reverse(True, 2.5)
    assert mux.update_boat_mode(BOAT_MODE_AUTO)
    assert mux.heading(2.5) == (200.0, ST_INIT_GOAL)
    assert not _gfeed(mux, 220.0, 2.6, reverse=True)
    assert mux.gps_cog.last_reject == "reverse"
    assert not _gfeed(mux, 220.0, 3.0), "후진 끝 직후 settle 이 지워졌다"
    assert mux.gps_cog.last_reject == "settle"
    assert mux.heading(3.0)[1] == ST_INIT_GOAL
    assert _gfeed(mux, 220.0, 4.7)              # 2.6 + 2.0 settle 경과
    # 선회 중 진입
    mux = _gmux()
    _manual_cog(mux, 40.0)
    mux.update_goal_bearing(200.0, 2.5)
    mux.update_desired_angle(SPIN_RIGHT, 2.5)
    assert mux.update_boat_mode(BOAT_MODE_AUTO)
    assert not _gfeed(mux, 220.0, 2.6, desired=SPIN_RIGHT)
    assert mux.gps_cog.last_reject == "spin"
    assert not _gfeed(mux, 220.0, 3.5)
    assert mux.gps_cog.last_reject == "settle"
    assert _gfeed(mux, 220.0, 4.7)


# ---- (10) AUTO 상수 = CLAUDE.md §3 /boat_mode 2 = 펌웨어 enum = north_goal_angle MODE_AUTO
def test_boat_mode_auto_matches_contract():
    import re
    assert BOAT_MODE_AUTO == 2, "CLAUDE.md §3: /boat_mode 0=WAIT 1=MANUAL 2=AUTO"
    ws = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                       '..', '..', '..'))
    ino = os.path.join(ws, 'arduino', 'ssf_boat', 'ssf_boat.ino')
    with open(ino, encoding='utf-8') as f:
        m = re.search(r'enum\s+Mode\s*\{[^}]*MODE_AUTO\s*=\s*(\d+)', f.read())
    assert m and int(m.group(1)) == BOAT_MODE_AUTO, f"ssf_boat.ino enum Mode MODE_AUTO ≠ {BOAT_MODE_AUTO}"
    ng = os.path.join(ws, 'src', 'north_goal_angle', 'north_goal_angle', 'north_goal_angle.py')
    with open(ng, encoding='utf-8') as f:
        m = re.search(r'^MODE_AUTO\s*=\s*(\d+)', f.read(), re.M)
    assert m and int(m.group(1)) == BOAT_MODE_AUTO, f"north_goal_angle MODE_AUTO ≠ {BOAT_MODE_AUTO}"


# ================================================================ 보완 B: 기동 구간 IMU 상대 회전량 보조 (C1b)
def _imu_ramp(mux, t0, t1, r0, rate, dt=0.01, desired=None, reverse=None, sig_every=10):
    """
    IMU 100 Hz 경사 r(t) = r0 + rate·(t − t0) 를 t0..t1 로 민다. sig_every 샘플마다(기본 0.1 s)
    desired·reverse 신호를 같은 시각에 먼저 준다. (마지막 시각, 마지막 r) 을 돌려준다.
    """
    n = int(round((t1 - t0) / dt))
    t = t0
    r = r0
    for i in range(n + 1):
        t = t0 + i * dt
        if i % sig_every == 0:
            if desired is not None:
                mux.update_desired_angle(desired, t)
            if reverse is not None:
                mux.update_reverse(reverse, t)
        r = r0 + rate * (i * dt)
        mux.update_imu(r, t)
    return t, r


def _anchored(mux, cog=30.0, t=1.0, imu=50.0):
    """COG 두 표본(같은 방위, 회전율 0)으로 앵커 + IMU 신선. 시각 t 에 OK."""
    assert _gfeed(mux, cog, t - 1.0)
    assert _gfeed(mux, cog, t)
    mux.update_imu(imu, t)
    assert mux.heading(t) == (cog, ST_OK)


# ---- B1) 선회 코드 수신 순간 출력 연속, 상태 ASSIST
def test_assist_starts_continuous_on_spin():
    for code in (SPIN_RIGHT, SPIN_LEFT):
        mux = _gmux()
        assert _gfeed(mux, 0.0, 0.0)
        assert _gfeed(mux, 20.0, 1.0)               # 20°/s 외삽 중
        mux.update_imu(50.0, 1.49)
        before = mux.heading(1.4999)[0]
        mux.update_desired_angle(code, 1.5)
        yaw, st = mux.heading(1.5)
        assert st == ST_IMU_ASSIST, (code, st)
        assert near(yaw, 30.0, 1e-6) and abs(ang_diff(yaw, before)) < 0.01, (code, before, yaw)
        # IMU 가 안 움직이면 값도 그대로 (외삽 계속 아님)
        mux.update_imu(50.0, 1.6)
        assert mux.heading(2.0) == (yaw, ST_IMU_ASSIST)


# ---- B2) 선회 중 IMU +90 → +90, 359→29 랩
def test_assist_follows_imu_delta_and_wraps():
    mux = _gmux()
    _anchored(mux, 30.0, 1.0, imu=50.0)
    t, _ = _imu_ramp(mux, 1.0, 3.0, 50.0, 45.0, desired=SPIN_RIGHT)   # +90°
    yaw, st = mux.heading(t)
    assert st == ST_IMU_ASSIST and near(yaw, 120.0, 1e-6), (yaw, st)

    # 출력 359 → 29, IMU 원시도 350 → 20 으로 감김 (+30°)
    mux = _gmux()
    _anchored(mux, 359.0, 1.0, imu=350.0)
    t, _ = _imu_ramp(mux, 1.0, 2.0, 350.0, 30.0, desired=SPIN_LEFT)
    yaw, st = mux.heading(t)
    assert st == ST_IMU_ASSIST and near(yaw, 29.0, 1e-6), (yaw, st)
    # 반대 방향(CCW)도
    mux = _gmux()
    _anchored(mux, 10.0, 1.0, imu=5.0)
    t, _ = _imu_ramp(mux, 1.0, 2.0, 5.0, -40.0, desired=SPIN_LEFT)
    assert near(mux.heading(t)[0], 330.0, 1e-6), mux.heading(t)


# ---- B3) 누적 400° (unwrapped) — 상태 문자열 d 에도 보존
def test_assist_accumulates_beyond_360():
    mux = _gmux()
    _anchored(mux, 30.0, 1.0, imu=0.0)
    t, _ = _imu_ramp(mux, 1.0, 11.0, 0.0, 40.0, desired=SPIN_RIGHT)  # 10 s × 40°/s
    yaw, st = mux.heading(t)
    assert st == ST_IMU_ASSIST and near(yaw, wrap360(30.0 + 400.0), 1e-6), (yaw, st)
    assert near(mux.gps_cog.assist_total_deg, 400.0, 1e-6)
    txt = gps_cog_status_text(mux.gps_cog, st, t)
    assert txt.startswith("gps_cog:ASSIST d=+400.0 t=10.0 "), txt


# ---- B4) invert_yaw=True 면 Δ 반대, mount 40.3·편각 −8 무영향
def test_assist_invert_flips_delta_mount_decl_ignored():
    for invert, want in ((False, 120.0), (True, 300.0)):
        mux = _gmux(invert_yaw=invert, mount_offset_deg=40.3, declination_deg=-8.0)
        _anchored(mux, 30.0, 1.0, imu=50.0)
        t, _ = _imu_ramp(mux, 1.0, 3.0, 50.0, 45.0, desired=SPIN_RIGHT)  # 원시 +90
        yaw, st = mux.heading(t)
        assert st == ST_IMU_ASSIST and near(yaw, want, 1e-6), (invert, yaw, st)


# ---- B5) 후진 True 도 보조 시작
def test_assist_starts_on_reverse():
    mux = _gmux()
    _anchored(mux, 30.0, 1.0, imu=50.0)
    mux.update_reverse(True, 1.5)
    assert mux.heading(1.5) == (30.0, ST_IMU_ASSIST)
    t, _ = _imu_ramp(mux, 1.5, 2.5, 50.0, -20.0, reverse=True)
    yaw, st = mux.heading(t)
    assert st == ST_IMU_ASSIST and near(yaw, 10.0, 1e-6), (yaw, st)
    # 후진 중 COG(뒤로 가는 방향)는 여전히 거부, 보조는 계속
    assert not _gfeed(mux, 190.0, t, reverse=True)
    assert mux.gps_cog.last_reject == "reverse"
    assert mux.heading(t)[1] == ST_IMU_ASSIST and near(mux.heading(t)[0], 10.0, 1e-6)


# ---- B6) settle → slow 에도 계속 → COG 채택으로 OK
def test_assist_continues_through_settle_and_slow_then_cog():
    mux = _gmux()
    _anchored(mux, 30.0, 1.0, imu=50.0)
    t, r = _imu_ramp(mux, 1.0, 2.0, 50.0, 60.0, desired=SPIN_RIGHT)    # +60 → 90
    assert near(mux.heading(t)[0], 90.0, 1e-6)
    # 선회 끝(12) — settle 동안 COG 거부(settle)지만 IMU 를 계속 따라간다
    t, r = _imu_ramp(mux, 2.01, 3.0, r + 0.6, 10.0, desired=12.0)
    assert not _gfeed(mux, 200.0, 3.0, speed=1.0)
    assert mux.gps_cog.last_reject == "settle"
    exp = 90.0 + 0.6 + 10.0 * 0.99
    assert mux.heading(3.0)[1] == ST_IMU_ASSIST and near(mux.heading(3.0)[0], exp, 1e-6)
    # settle 뒤 저속(규칙 2 의 '정지')이어도 보조 중이면 계속 (B-Q7)
    t, r = _imu_ramp(mux, 3.01, 5.0, r + 0.1, 5.0, desired=12.0)
    assert not _gfeed(mux, 200.0, 5.0, speed=0.3)
    assert mux.gps_cog.last_reject == "slow"
    exp = exp + 0.1 + 5.0 * 1.99
    yaw, st = mux.heading(5.0)
    assert st == ST_IMU_ASSIST and near(yaw, exp, 1e-6), (yaw, exp, st)
    # 전진 COG 채택 → 앵커 = COG, OK (보조 종료)
    assert _gfeed(mux, 115.0, 5.5)
    assert mux.heading(5.5) == (115.0, ST_OK)
    assert not mux.gps_cog.assist_active and mux.gps_cog.assist_ends["cog"] == 1
    # 채택 뒤 IMU 가 돌아도 출력 불변(보조 밖)
    _imu_ramp(mux, 5.51, 6.0, r, 30.0, desired=12.0)
    assert near(mux.heading(6.0)[0], 115.0, 1e-9)


# ---- B7) 보조 밖 slow 는 IMU 가 돌아도 불변 (C1 얼림)
def test_assist_not_started_by_slow_outside_maneuver():
    mux = _gmux()
    _anchored(mux, 30.0, 1.0, imu=50.0)
    assert not _gfeed(mux, 200.0, 2.0, speed=0.2)
    assert mux.gps_cog.last_reject == "slow"
    t, _ = _imu_ramp(mux, 2.0, 30.0, 50.0, 10.0, desired=12.0)
    yaw, st = mux.heading(t)
    assert yaw == 30.0 and st == ST_HOLD, (yaw, st)
    assert not mux.gps_cog.assist_active


# ---- B8) 상한(B-Q2 (iii) 마지막 기동 신호 뒤 10 s): 얼림·HOLD·다음 기동에 재시작
def test_assist_limit_from_last_maneuver_then_restart():
    mux = _gmux()
    _anchored(mux, 30.0, 1.0, imu=0.0)
    # 선회 신호 1.0~3.0 (마지막 신호 3.0), IMU 는 5°/s 로 계속 돈다
    t, r = _imu_ramp(mux, 1.0, 3.0, 0.0, 5.0, desired=SPIN_RIGHT)
    last_sig = t
    # 신호가 이어지는 동안엔 시작 기준이 아니라 마지막 신호 기준 — 3 s 지났어도 계속
    t, r = _imu_ramp(mux, 3.01, 12.9, r + 0.05, 5.0, desired=12.0)
    yaw, st = mux.heading(12.9)
    assert st == ST_IMU_ASSIST and near(yaw, 30.0 + 5.0 * 11.9, 1e-6), (yaw, st)
    # 마지막 신호 + 10 s 넘김 → 그 값에서 얼림, HOLD
    t, r = _imu_ramp(mux, 12.91, 20.0, r + 0.05, 5.0, desired=12.0)
    yaw, st = mux.heading(20.0)
    assert st == ST_HOLD, st
    assert mux.gps_cog.assist_ends["limit"] == 1
    frozen = 30.0 + 5.0 * (last_sig + 10.0 - 1.0)
    assert abs(yaw - frozen) <= 5.0 * 0.011, (yaw, frozen)
    assert mux.heading(25.0)[0] == yaw, "얼린 뒤 IMU 회전이 새어들어감"
    # 다음 기동 → 얼린 값에서 재시작
    mux.update_desired_angle(SPIN_LEFT, 20.0)
    assert mux.heading(20.0) == (yaw, ST_IMU_ASSIST)
    t, _ = _imu_ramp(mux, 20.01, 21.0, r + 0.05, 5.0, desired=SPIN_LEFT)
    assert near(mux.heading(t)[0], yaw + 5.0 * 1.0, 1e-6), (mux.heading(t), yaw)


# ---- B9) 보조 중 IMU stale → 마지막 값, 시작 때 IMU 없음 → C1
def test_assist_imu_stale_freezes_and_no_imu_is_c1():
    mux = _gmux()
    _anchored(mux, 30.0, 1.0, imu=50.0)
    t, r = _imu_ramp(mux, 1.0, 2.0, 50.0, 20.0, desired=SPIN_RIGHT)      # +20 → 50
    last = mux.heading(t)[0]
    assert near(last, 50.0, 1e-6)
    # IMU 끊김, 선회 신호는 계속
    for k in range(1, 30):
        mux.update_desired_angle(SPIN_RIGHT, 2.0 + 0.1 * k)
    yaw, st = mux.heading(4.9)
    assert yaw == last and st == ST_HOLD, (yaw, st)
    assert mux.gps_cog.assist_ends["imu_stale"] == 1
    assert not mux.gps_cog.imu_fresh(4.9)
    # 0.5 s 경계 안쪽은 아직 보조
    m2 = _gmux()
    _anchored(m2, 30.0, 1.0, imu=50.0)
    m2.update_desired_angle(SPIN_RIGHT, 1.2)
    assert m2.heading(1.5)[1] == ST_IMU_ASSIST
    assert m2.heading(1.5001) == (30.0, ST_OK), "IMU stale → 보조 끝(마지막 COG 1.0 s 전이라 OK)"

    # 시작 때 IMU 없음 → C1 과 같다 (보조 off 믹서와 같은 출력)
    a = _gmux()
    b = _gmux(imu_assist=False)
    for m in (a, b):
        assert _gfeed(m, 0.0, 0.0)
        assert _gfeed(m, 20.0, 1.0)
        m.update_desired_angle(SPIN_RIGHT, 1.5)
    for tt in (1.5, 2.0, 5.0, 30.0):
        assert a.heading(tt) == b.heading(tt), (tt, a.heading(tt), b.heading(tt))
    assert a.heading(30.0)[1] == ST_HOLD and near(a.heading(30.0)[0], 30.0, 1e-6)
    assert not a.gps_cog.assist_active


# ---- B10) 튐: 한 샘플(+82 원복)은 영향 0, 계단은 한 샘플만 손실
def test_assist_spike_dropped_step_loses_one_sample():
    mux = _gmux()
    _anchored(mux, 30.0, 1.0, imu=50.0)
    mux.update_desired_angle(SPIN_RIGHT, 1.0)
    for k, r in enumerate((50.0, 50.0, 132.0, 50.0, 50.0)):     # 한 샘플 +82 튀었다 원복
        mux.update_imu(r, 1.01 + 0.01 * k)
    yaw, st = mux.heading(1.06)
    assert st == ST_IMU_ASSIST and yaw == 30.0, (yaw, st)
    assert mux.gps_cog.imu_step_rejects == 2, "튀는 Δ·돌아오는 Δ 둘 다 버린다"
    # 기준 계단(+20 그대로 유지) → 그 한 샘플만 잃고 이후 회전은 따라간다
    for k, r in enumerate((70.0, 70.0, 71.0, 72.0)):
        mux.update_imu(r, 1.07 + 0.01 * k)
    yaw, st = mux.heading(1.11)
    assert st == ST_IMU_ASSIST and near(yaw, 32.0, 1e-9), (yaw, st)
    assert mux.gps_cog.imu_step_rejects == 3
    # 정확히 10° 는 '넘게' 가 아니다
    mux.update_imu(82.0, 1.12)
    assert near(mux.heading(1.12)[0], 42.0, 1e-9) and mux.gps_cog.imu_step_rejects == 3
    # NaN·inf 는 샘플 자체를 무시
    mux.update_imu(float('nan'), 1.13)
    mux.update_imu(float('inf'), 1.14)
    assert near(mux.heading(1.14)[0], 42.0, 1e-9)


# ---- B11) 180° 선회 뒤 첫 COG 즉시 채택 (jump 대기 없음)
def test_assist_180_spin_then_first_cog_accepted():
    for assist, accepted in ((True, True), (False, False)):
        mux = _gmux(imu_assist=assist)
        _anchored(mux, 0.0, 1.0, imu=10.0)
        t, r = _imu_ramp(mux, 1.0, 5.0, 10.0, 45.0, desired=SPIN_RIGHT)   # +180
        t, r = _imu_ramp(mux, 5.01, 7.5, r, 0.0, desired=12.0)             # settle 끝
        got = _gfeed(mux, 181.0, 7.5)
        assert got is accepted, (assist, got, mux.gps_cog.last_reject)
        if assist:
            assert mux.heading(7.5) == (181.0, ST_OK)
        else:
            assert mux.gps_cog.last_reject == "jump", "보조 off(C1)는 2회 연속 규칙"


# ---- B12) 보조 중 AUTO 진입 → INIT_GOAL (보조 상태도 지움)
def test_assist_cleared_by_auto_entry():
    mux = _gmux()
    mux.update_boat_mode(1)
    _anchored(mux, 30.0, 1.0, imu=50.0)
    t, r = _imu_ramp(mux, 1.0, 2.0, 50.0, 30.0, desired=SPIN_RIGHT)
    assert mux.heading(t)[1] == ST_IMU_ASSIST
    mux.update_goal_bearing(200.0, t)
    assert mux.update_boat_mode(BOAT_MODE_AUTO) is True
    assert mux.heading(t) == (200.0, ST_INIT_GOAL)
    assert not mux.gps_cog.assist_active and mux.gps_cog.assist_ends["reset"] == 1
    # IMU 가 계속 돌고 선회 신호가 와도 INIT_GOAL 그대로 (B-Q6)
    mux.update_goal_bearing(200.0, t + 0.5)
    _imu_ramp(mux, t + 0.01, t + 1.0, r, 30.0, desired=SPIN_RIGHT)
    assert mux.heading(t + 1.0) == (200.0, ST_INIT_GOAL)


# ---- B13) INIT_GOAL 중 기동 → C1 (보조 안 함)
def test_assist_not_during_init_goal():
    a = _gmux()
    b = _gmux(imu_assist=False)
    for m in (a, b):
        m.update_goal_bearing(77.0, 0.0)
        m.update_imu(10.0, 0.0)
        _imu_ramp(m, 0.0, 2.0, 10.0, 45.0, desired=SPIN_RIGHT)
        m.update_goal_bearing(77.0, 2.0)
    assert a.heading(2.0) == b.heading(2.0) == (77.0, ST_INIT_GOAL)
    assert not a.gps_cog.assist_active
    # 첫 COG 채택(settle 뒤) 뒤엔 앵커가 생겨 다음 기동부터 보조
    for m in (a, b):
        _imu_ramp(m, 2.01, 4.5, 100.0, 0.0, desired=12.0)
        assert _gfeed(m, 120.0, 4.5)
    assert a.heading(4.5) == b.heading(4.5) == (120.0, ST_OK)
    a.update_desired_angle(SPIN_RIGHT, 4.6)
    assert a.heading(4.6) == (120.0, ST_IMU_ASSIST)


# ---- B14) gps_cog_imu_assist=False → C1 과 비트 동일 (IMU·기동 다 줘도)
def _assist_scenario(mux, with_imu):
    """앵커 → 선회(IMU 회전) → settle → 저속 → 후진 → 전진 COG. 탐침 시각별 heading 을 돌려준다."""
    out = []
    probes = (0.5, 1.0, 1.5, 2.2, 3.0, 3.9, 5.0, 6.3, 7.0, 8.4, 9.9, 12.0, 15.0, 30.0)
    events = []
    for i in range(0, 3001):                    # 0~30 s, 100 Hz
        t = i * 0.01
        if with_imu:
            events.append((t, 'imu', 20.0 + 17.0 * t))
        if i % 10 == 0:
            if 1.5 <= t < 4.0:
                events.append((t, 'des', SPIN_RIGHT))
            else:
                events.append((t, 'des', 12.0))
            events.append((t, 'rev', 7.0 <= t < 8.0))
        if i % 100 == 0:
            spd = 0.3 if 9.0 <= t < 12.0 else 1.0
            events.append((t, 'cog', (10.0 + 3.0 * t, spd)))
    pi = 0
    for t, kind, v in events:
        while pi < len(probes) and probes[pi] < t:
            out.append((probes[pi], mux.heading(probes[pi])))
            pi += 1
        if kind == 'imu':
            mux.update_imu(v, t)
        elif kind == 'des':
            mux.update_desired_angle(v, t)
        elif kind == 'rev':
            mux.update_reverse(v, t)
        else:
            mux.update_cog(v[0], v[1], t)
    while pi < len(probes):
        out.append((probes[pi], mux.heading(probes[pi])))
        pi += 1
    return out


def test_assist_off_is_bit_identical_to_c1():
    off_with_imu = _assist_scenario(_gmux(imu_assist=False), with_imu=True)
    c1_no_imu = _assist_scenario(_gmux(imu_assist=False), with_imu=False)
    on_no_imu = _assist_scenario(_gmux(imu_assist=True), with_imu=False)
    assert off_with_imu == c1_no_imu, "보조 off 인데 IMU 가 출력에 새어들어감"
    assert on_no_imu == c1_no_imu, "IMU 가 없으면 보조 on 이어도 C1 과 같아야 한다"
    on_with_imu = _assist_scenario(_gmux(imu_assist=True), with_imu=True)
    assert on_with_imu != c1_no_imu, "보조 on + IMU 인데 차이가 없다(시나리오가 보조를 안 태움)"
    assert any(h[1] == ST_IMU_ASSIST for _, h in on_with_imu)


# ---- B16) ASSIST 는 발행 상태, 침묵 상태 아님, 접두 gps_cog:
def test_assist_state_publishes_and_status_prefix():
    assert ST_IMU_ASSIST == "ASSIST"
    assert ST_IMU_ASSIST in PUBLISH_STATES, "빠지면 기동마다 /imu/yaw 침묵 → 페일세이프"
    assert ST_IMU_ASSIST not in SILENT_STATES
    assert not (set(PUBLISH_STATES) & set(SILENT_STATES))
    for gate in (True, False):
        mux = _gmux(require_gate=gate)
        _anchored(mux, 30.0, 1.0, imu=50.0)
        mux.update_desired_angle(SPIN_RIGHT, 1.2)
        mux.update_imu(97.3, 1.3)
        mux.update_imu(97.3, 1.4)
        yaw, st = mux.heading(1.4)
        assert st == ST_IMU_ASSIST
        txt = gps_cog_status_text(mux.gps_cog, st, 1.4)
        assert txt.startswith("gps_cog:ASSIST "), txt
        assert txt.endswith(" gate=off") is (not gate), txt
        assert not txt.endswith(":OK")
    # 다른 상태 문자열은 예전 그대로
    mux = _gmux()
    assert gps_cog_status_text(mux.gps_cog, ST_INIT, 0.0) == "gps_cog:INIT rej=-"
    _anchored(mux, 30.0, 1.0, imu=50.0)
    assert gps_cog_status_text(mux.gps_cog, ST_OK, 1.0) == "gps_cog:OK"
    assert gps_cog_status_text(mux.gps_cog, ST_HOLD, 3.0) == "gps_cog:HOLD age=2.0 rej=-"


# ---- 보완 B 검증 계획 '기존 테스트': C1 gps_cog·보완 A 테스트가 보조 off 에서도 같게 통과
_C1_AND_A_TESTS = (
    "test_gps_cog_init_goal_before_first_cog", "test_gps_cog_slow_holds_forever_and_keeps_publishing",
    "test_gps_cog_reverse_true_or_unknown_blocks_update", "test_gps_cog_gate_off_accepts_unknown",
    "test_gps_cog_spin_and_settle_block_update", "test_gps_cog_reverse_end_starts_settle",
    "test_gps_cog_desired_unknown_blocks_update", "test_gps_cog_rate_wraparound",
    "test_gps_cog_predict_stops_at_max_and_rate_clamped", "test_gps_cog_freeze_is_continuous",
    "test_gps_cog_jump_needs_two_in_a_row", "test_gps_cog_ignores_mount_invert_declination",
    "test_gps_cog_does_not_read_imu", "test_gps_cog_enu_north_and_east",
    "test_auto_entry_resets_to_init_goal", "test_auto_entry_on_first_message",
    "test_auto_repeat_does_not_reset", "test_auto_reentry_resets_each_time",
    "test_no_boat_mode_is_c1_behaviour", "test_auto_entry_with_stale_goal_is_silent",
    "test_auto_entry_first_cog_skips_jump_and_drops_pending",
    "test_auto_entry_keeps_reverse_and_spin_gates",
)


def test_c1_and_auto_tests_pass_with_assist_off():
    g = globals()
    assert _GMUX_ASSIST[0] is True, "기본 실행은 보조 on (코드 기본값 true, B-Q8)"
    _GMUX_ASSIST[0] = False
    try:
        for name in _C1_AND_A_TESTS:
            g[name]()
    finally:
        _GMUX_ASSIST[0] = True


def main():
    print("=== heading_logic 테스트 ===")
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            check(name[5:], fn)
    print(f"\n{_passed}/{_total} 통과")
    return 0 if _passed == _total else 1


if __name__ == "__main__":
    sys.exit(main())

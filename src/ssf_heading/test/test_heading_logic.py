#!/usr/bin/env python3
"""
heading_logic 순수 로직 테스트. ROS 없이 그냥 실행된다:
    python3 src/ssf_heading/test/test_heading_logic.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ssf_heading.heading_logic import (  # noqa: E402
    ALL_SOURCES, SOURCE_IS_MAGNETIC, SILENT_STATES,
    SRC_IMU_RELATIVE, SRC_COG_OFFSET, SRC_DUAL_GPS, SRC_IMU_ABSOLUTE,
    ST_OK, ST_NO_DATA, ST_STALE, ST_NOT_IMPLEMENTED, ST_NOT_CONVERGED,
    HeadingMux, COGOffsetEstimator, cog_from_velocity, saturation_samples,
    wrap360, wrap180, ang_diff, is_finite, apply_corrections,
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
        est.update(imu, cog, speed, turn, t)
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
             yaw_step=0.3):
    """
    yaw_step=0.3°/0.1s = 3°/s. 직진 중의 자연스러운 흔들림이다.
    ⚠️ 이걸 1.3 으로 두면 13°/s 라 max_turn_rate_dps(8) 에 걸려 전 표본이 배제된다.
       (실제로 이 테스트를 짜다 걸렸다 — 게이팅이 제대로 도는 증거)
    """
    t = t0
    for i in range(n):
        imu = wrap360(imu0 + i * yaw_step)
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


def main():
    print("=== heading_logic 테스트 ===")
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            check(name[5:], fn)
    print(f"\n{_passed}/{_total} 통과")
    return 0 if _passed == _total else 1


if __name__ == "__main__":
    sys.exit(main())

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
    ST_OK, ST_NO_DATA, ST_STALE, ST_NOT_IMPLEMENTED,
    HeadingMux, wrap360, wrap180, ang_diff, is_finite, apply_corrections,
)

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
    """
    for src in (SRC_COG_OFFSET, SRC_DUAL_GPS, SRC_IMU_ABSOLUTE):
        mux = HeadingMux(src, stale_sec=10.0)
        mux.update_imu(123.0, 0.0)          # IMU 는 들어와도
        mux.update_cog(45.0, 2.0, 0.0)      # COG 도 들어와도
        yaw, st = mux.heading(0.0)
        assert yaw is None, f"{src}: 미구현인데 값을 냈다 ({yaw})"
        assert st == ST_NOT_IMPLEMENTED, f"{src}: 상태가 {st}"


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


def main():
    print("=== heading_logic 테스트 ===")
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            check(name[5:], fn)
    print(f"\n{_passed}/{_total} 통과")
    return 0 if _passed == _total else 1


if __name__ == "__main__":
    sys.exit(main())

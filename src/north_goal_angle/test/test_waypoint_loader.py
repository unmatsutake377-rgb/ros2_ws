#!/usr/bin/env python3
"""waypoint_loader 순수 로직 테스트. python3 src/north_goal_angle/test/test_waypoint_loader.py"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.waypoint_loader import (  # noqa: E402
    parse_waypoints, WaypointError, VALID_MODES,
)

_p = _t = 0


def check(name, fn):
    global _p, _t
    _t += 1
    try:
        fn(); _p += 1; print(f"  ✅ {name}")
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")


def _raises(raw):
    """parse 가 WaypointError 를 던지면 True."""
    try:
        parse_waypoints(raw)
        return False
    except WaypointError:
        return True


# 유효한 최소 예시 (대회장 근처 좌표)
GOOD_DICT = {"waypoints": [
    {"구역": "게이트", "lat": 35.1862, "lon": 128.5655, "mode": 0, "dwell": 3.0},
    {"구역": "도킹",   "lat": 35.1859, "lon": 128.5655, "mode": 7, "dwell": 60.0},
]}
GOOD_LIST = {"waypoints": [
    [35.1862, 128.5655, 0, 3.0],
    [35.1859, 128.5655, 7, 60.0],
]}


# ---------------------------------------------------- 정상 파싱
def test_parse_dict_form():
    wps = parse_waypoints(GOOD_DICT)
    assert wps == [[35.1862, 128.5655, 0, 3.0], [35.1859, 128.5655, 7, 60.0]], wps


def test_parse_list_form():
    wps = parse_waypoints(GOOD_LIST)
    assert len(wps) == 2 and wps[0][2] == 0 and wps[1][2] == 7


def test_output_is_float():
    """정수로 적어도 float 로 정규화 (다운스트림 계산 안전)."""
    wps = parse_waypoints({"waypoints": [{"lat": 35, "lon": 128, "mode": 0, "dwell": 3}]})
    assert isinstance(wps[0][0], float) and isinstance(wps[0][1], float)


def test_mode_order_preserved():
    """🚨 미션 순서 = mode 순서. 입력 순서 그대로 나와야 한다."""
    raw = {"waypoints": [
        {"lat": 35.1, "lon": 128.5, "mode": 0, "dwell": 1.0},
        {"lat": 35.1, "lon": 128.5, "mode": 2, "dwell": 1.0},
        {"lat": 35.1, "lon": 128.5, "mode": 7, "dwell": 1.0},
    ]}
    modes = [w[2] for w in parse_waypoints(raw)]
    assert modes == [0, 2, 7], f"순서 안 지킴: {modes}"


# ---------------------------------------------------- 🚨 잘못된 입력은 예외 (조용히 넘어가면 안 됨)
def test_no_waypoints_key():
    assert _raises({"foo": []})
    assert _raises({})
    assert _raises([35.1, 128.5, 0, 3.0])   # dict 아님


def test_empty_list():
    assert _raises({"waypoints": []})


def test_missing_field():
    assert _raises({"waypoints": [{"lat": 35.1, "lon": 128.5, "mode": 0}]})  # dwell 없음


def test_wrong_length_list():
    assert _raises({"waypoints": [[35.1, 128.5, 0]]})        # 3개
    assert _raises({"waypoints": [[35.1, 128.5, 0, 3, 9]]})  # 5개


def test_lat_out_of_range():
    """위도가 한국 범위 밖 — 위경도 뒤바꿈·오타·빈칸 탐지."""
    assert _raises({"waypoints": [{"lat": 128.5, "lon": 35.1, "mode": 0, "dwell": 3.0}]})  # 뒤바뀜
    assert _raises({"waypoints": [{"lat": 0.0, "lon": 128.5, "mode": 0, "dwell": 3.0}]})   # 빈칸(0)
    assert _raises({"waypoints": [{"lat": 135.0, "lon": 128.5, "mode": 0, "dwell": 3.0}]}) # 오타


def test_lon_out_of_range():
    assert _raises({"waypoints": [{"lat": 35.1, "lon": 200.0, "mode": 0, "dwell": 3.0}]})


def test_invalid_mode():
    assert _raises({"waypoints": [{"lat": 35.1, "lon": 128.5, "mode": 70, "dwell": 3.0}]})  # 오타
    assert _raises({"waypoints": [{"lat": 35.1, "lon": 128.5, "mode": 9, "dwell": 3.0}]})   # 작년 도킹 오류
    # 허용 mode 는 통과
    for m in VALID_MODES:
        parse_waypoints({"waypoints": [{"lat": 35.1, "lon": 128.5, "mode": m, "dwell": 3.0}]})


def test_mode_must_be_int():
    assert _raises({"waypoints": [{"lat": 35.1, "lon": 128.5, "mode": 0.5, "dwell": 3.0}]})
    assert _raises({"waypoints": [{"lat": 35.1, "lon": 128.5, "mode": "0", "dwell": 3.0}]})


def test_negative_dwell():
    assert _raises({"waypoints": [{"lat": 35.1, "lon": 128.5, "mode": 0, "dwell": -1.0}]})


def test_non_numeric_coord():
    assert _raises({"waypoints": [{"lat": "삼십오", "lon": 128.5, "mode": 0, "dwell": 3.0}]})


def test_error_message_has_index():
    """에러가 '몇 번째'인지 알려줘야 비전공자가 고칠 수 있다."""
    try:
        parse_waypoints({"waypoints": [
            {"lat": 35.1, "lon": 128.5, "mode": 0, "dwell": 3.0},
            {"lat": 999.0, "lon": 128.5, "mode": 0, "dwell": 3.0},   # 2번째가 틀림
        ]})
        assert False, "예외가 났어야 한다"
    except WaypointError as e:
        assert "1번째" in str(e), f"몇 번째인지 안 알려줌: {e}"


def main():
    print("=== waypoint_loader 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())

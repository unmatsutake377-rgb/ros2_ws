#!/usr/bin/env python3
"""waypoint_recorder 순수 로직 테스트. python3 src/north_goal_angle/test/test_waypoint_recorder.py"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.waypoint_recorder import (  # noqa: E402
    average_fixes, format_waypoints_yaml,
)

_p = _t = 0


def check(name, fn):
    global _p, _t
    _t += 1
    try:
        fn(); _p += 1; print(f"  ✅ {name}")
    except AssertionError as e:
        print(f"  ❌ {name}\n     {e}")


# ---------------- average_fixes
def test_avg_empty():
    assert average_fixes([]) is None


def test_avg_single():
    assert average_fixes([(35.0, 128.0)]) == (35.0, 128.0)


def test_avg_mean():
    lat, lon = average_fixes([(35.0, 128.0), (35.2, 128.4)])
    assert abs(lat - 35.1) < 1e-9 and abs(lon - 128.2) < 1e-9, (lat, lon)


def test_avg_reduces_jitter():
    """튀는 표본 하나가 평균에 섞여도 다수 표본이 끌어당긴다(단일값보다 안정)."""
    samples = [(35.1862, 128.5655)] * 9 + [(35.1900, 128.5655)]  # 1개만 튐
    lat, _ = average_fixes(samples)
    assert 35.1862 < lat < 35.1866, lat   # 튄 값(35.19) 쪽으로 거의 안 감


# ---------------- format_waypoints_yaml
def test_format_has_header():
    y = format_waypoints_yaml([[35.1, 128.5, 0, 3.0, "게이트"]])
    assert y.startswith("waypoints:"), y


def test_format_label_and_fields():
    y = format_waypoints_yaml([[35.1862375, 128.5655118, 7, 60.0, "도킹"]])
    assert '구역: "도킹"' in y
    assert "mode: 7" in y
    assert "dwell: 60.0" in y
    assert "lat: 35.1862375" in y


def test_format_auto_label():
    """라벨 비면 WP<i> 자동."""
    y = format_waypoints_yaml([[35.1, 128.5, 0, 3.0, ""], [35.2, 128.5, 1, 3.0, ""]])
    assert '구역: "WP0"' in y and '구역: "WP1"' in y, y


def test_format_roundtrip_parseable():
    """생성한 yaml 이 waypoint_loader 로 다시 파싱되는가(형식 계약 일치)."""
    try:
        import yaml  # Mac 엔 없을 수 있음 → 있을 때만 검증
    except ImportError:
        print("     (yaml 모듈 없음 — roundtrip 건너뜀)")
        return
    from north_goal_angle.waypoint_loader import parse_waypoints
    y = format_waypoints_yaml([[35.1862, 128.5655, 0, 3.0, "게이트"],
                               [35.1859, 128.5655, 7, 60.0, "도킹"]])
    wps = parse_waypoints(yaml.safe_load(y))
    assert wps == [[35.1862, 128.5655, 0, 3.0], [35.1859, 128.5655, 7, 60.0]], wps


def main():
    print("=== waypoint_recorder 테스트 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""tournament 템플릿 웨이포인트 경고(C2) 정적 검사.

실행: python3 src/north_goal_angle/test/test_tournament_template_warn.py

계약서 docs/문제와작업/계약서_지오펜스폴리곤타입_20261109.md C2 (결정 4 권고안):
  course=tournament 이고 실제 쓰는 웨이포인트 파일이 패키지 템플릿(share 의
  config/waypoints_tournament.yaml, realpath 비교)이면 시작 WARN 1회 — 거부하지 않는다.

north_goal_angle.py 는 rclpy·geopy 를 import 하므로(Mac 에 없음) 소스를 ast·문자열로 본다.
"""

import ast
import os
import sys

_PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
_NODE_SRC = os.path.join(_PKG, 'north_goal_angle', 'north_goal_angle.py')

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


def _src():
    return open(_NODE_SRC, encoding='utf-8').read()


def _node_methods():
    tree = ast.parse(_src())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'NorthGoalAngle')
    return {f.name: f for f in cls.body if isinstance(f, ast.FunctionDef)}


def test_template_path_is_share_tournament_file():
    src = _src()
    want = ("get_package_share_directory('north_goal_angle'), 'config', "
            "'waypoints_tournament.yaml')")
    assert want in src, "템플릿 경로가 share 의 config/waypoints_tournament.yaml 이 아니다"


def test_template_compare_by_realpath_only_in_tournament():
    src = _src()
    i = src.index("self._log_tournament_template_waypoints(template_wp)")
    cond = src[src.rindex("if ", 0, i):i]
    assert "self.course == 'tournament'" in cond, cond
    assert "os.path.realpath(self.waypoints_file) == os.path.realpath(template_wp)" in cond, cond


def test_template_warn_after_waypoint_load_before_timer():
    src = _src()
    i_load = src.index("self.waypoints = self._load_waypoints(self.waypoints_file)")
    i_warn = src.index("self._log_tournament_template_waypoints(template_wp)")
    i_timer = src.index("self.create_timer(self.period_sec")
    assert i_load < i_warn < i_timer, (i_load, i_warn, i_timer)
    assert src.count("self._log_tournament_template_waypoints(") == 1, "호출이 한 곳이 아니다(1회)"


def test_template_warn_is_warn_not_error_and_no_raise():
    m = _node_methods().get('_log_tournament_template_waypoints')
    assert m is not None, "_log_tournament_template_waypoints 메서드가 없다"
    levels = [n.func.attr for n in ast.walk(m)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and isinstance(n.func.value, ast.Call)
              and isinstance(n.func.value.func, ast.Attribute)
              and n.func.value.func.attr == 'get_logger']
    assert levels == ['warn'], levels
    assert not any(isinstance(n, ast.Raise) for n in ast.walk(m)), "템플릿이면 거부하지 않는다(결정 4)"


def main():
    print("=== tournament 템플릿 경고 정적 검사 ===")
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            check(n[5:], f)
    print(f"\n{_p}/{_t} 통과")
    return 0 if _p == _t else 1


if __name__ == "__main__":
    sys.exit(main())

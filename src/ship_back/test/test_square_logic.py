#!/usr/bin/env python3
"""ship_back 사각형 위치유지(hold_source=square) 순수 로직 테스트 — ROS 없이 돈다.

실행: tools/run_tests.sh ship_back

계약서 v2 위치유지 사각형(study_square_sim/CONTRACT_v2_ship_back_square.md) 변경 단위 (b) 검증 계획:
  (1) square_logic 순수: 2→3→4점, 평행사변형 보완, 중심 GPS, 선체 전체 내부(1.77×0.57), 5 s 연속·리셋,
      부표 GPS 오차 초과 시 성공 금지, 안전영역 소멸 시 성공 금지, 검출기 stale → 50000
  (2) 출력 집합 {±80, 5000, 6000, 50000, 180, 20000 = INIT·DONE 만} — 🚨 DONE 전 20000 금지
  + 파라미터 기본값 = 노드 리터럴 = yaml(키·값·타입), /buoy_bearings 형식 사본 = stop_detect_logic.
"""

import ast
import math
import os
import random
import re
import sys

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, PKG)
NODE_SRC = os.path.join(PKG, 'ship_back', 'ship_back.py')
YAML = os.path.join(PKG, 'config', 'ship_back.yaml')
STOP_LOGIC = os.path.join(PKG, '..', 'color_shape_detector', 'color_shape_detector', 'stop_detect_logic.py')

from ship_back import square_logic as SQ  # noqa: E402
from ship_back.square_logic import (  # noqa: E402
    DEFAULTS, SquareFSM, HeadingFuser, LandmarkMap, check_square_params, parse_buoy_bearings, fix_usable,
    ll_to_xy, xy_to_ll, gps_speed, project, fit_square, hull_inside, safe_region_exists, near_hazard,
    entry_target, seek_approach_point, spin_cmd, steer_value, is_steer, output_allowed, order_ccw,
    square_hold_applies, ContinuousTimer, wrap180, bearing_to)

INV, STOP, SR, SL, REV = 20000.0, 50000.0, 5000.0, 6000.0, 180.0
NAN, INF = float('nan'), float('inf')


def _levels(issues):
    return [lv for lv, _ in issues]


def _square(side=5.0, rot=0.0, cx=0.0, cy=0.0):
    c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    out = []
    for ux, uy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        x, y = ux * side / 2, uy * side / 2
        out.append((cx + x * c - y * s, cy + x * s + y * c))
    return out


# ─────────────── 파라미터: 기본값 = 노드 리터럴 = yaml ───────────────
def _node_literals():
    out = {}
    tree = ast.parse(open(NODE_SRC, encoding='utf-8').read())
    for n in ast.walk(tree):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'declare_parameter'
                and len(n.args) >= 2 and isinstance(n.args[0], ast.Constant)):
            out[n.args[0].value] = ast.literal_eval(n.args[1])
    return out


def test_defaults_equal_node_literals_and_yaml():
    import yaml
    lit = _node_literals()
    with open(YAML, encoding='utf-8') as f:
        y = yaml.safe_load(f)['/ship_back']['ros__parameters']
    for k, v in DEFAULTS.items():
        assert k in lit and k in y, k
        assert lit[k] == v and y[k] == v, (k, lit[k], y[k], v)
        assert type(lit[k]) is type(y[k]) is type(v), (k, type(lit[k]), type(y[k]), type(v))  # noqa: E721
    assert lit['hold_source'] == y['hold_source'] == 'square'          # Davidson 답 v2 6번
    assert (y['hold_time_sec'], y['unknown_giveup_sec']) == (5.0, 10.0)  # 재사용(불변)
    assert (DEFAULTS['square_side_m'], DEFAULTS['hull_length_m'], DEFAULTS['hull_beam_m']) == (5.0, 1.77, 0.57)


def test_yaml_square_unmeasured_marked():
    txt = open(YAML, encoding='utf-8').read()
    for k in ('buoy_radius_m', 'square_inside_margin_m', 'square_shape_tol_m', 'buoy_pos_max_err_m',
              'bearings_stale_sec', 'yaw_raw_invert', 'close_m', 'spin_pulse_sec'):
        line = next(ln for ln in txt.splitlines() if ln.strip().startswith(k + ':'))
        assert '⚠️ 실측 필요' in line, line


def test_param_guard_defaults_silent():
    out, issues = check_square_params({})
    assert out == DEFAULTS and issues == [], issues
    out, issues = check_square_params(dict(DEFAULTS))
    assert out == DEFAULTS and issues == []


def test_param_guard_bad_values_fall_back_one_by_one():
    for k, bad in (('square_side_m', 0.0), ('hull_length_m', -1.0), ('buoy_radius_m', -0.1), ('obs_max_range_m', NAN),
                   ('bearings_stale_sec', INF), ('look_dwell_sec', 'x'), ('buoy_confirm_obs', 0),
                   ('buoy_confirm_obs', 3.0), ('heading_use_raw', 1), ('yaw_raw_invert', 'false'),
                   ('fix_stale_sec', True)):
        out, issues = check_square_params({k: bad})
        assert out[k] == DEFAULTS[k], (k, bad, out[k])
        assert _levels(issues) == ['ERROR'] and k in issues[0][1], (k, bad, issues)
    out, issues = check_square_params({'spin_pause_sec': 0.0, 'init_min_sec': 0.0})
    assert issues == [] and out['spin_pause_sec'] == 0.0


def test_param_guard_bundles():
    out, iss = check_square_params({'hold_in_m': 0.8, 'hold_out_m': 0.8})
    assert (out['hold_in_m'], out['hold_out_m']) == (0.4, 0.8) and _levels(iss) == ['ERROR']
    out, iss = check_square_params({'steer_spin_deg': 85.0})
    assert out['steer_spin_deg'] == 60.0 and _levels(iss) == ['ERROR']
    out, iss = check_square_params({'square_engage_m': 5.4})          # < 3.5 + 2.0
    assert out['square_engage_m'] == 8.0 and 'north' in iss[0][1]
    out, iss = check_square_params({'square_engage_m': 5.5})
    assert out['square_engage_m'] == 5.5 and iss == []


def test_param_guard_safe_region_gone_error_keeps_values():
    """5 − 2×(0.9 + 0.8) = 1.6 < 1.77 → 안전영역 소멸 ERROR(값은 그대로 — 성공 금지로 드러난다)."""
    out, iss = check_square_params({'buoy_radius_m': 0.9, 'square_inside_margin_m': 0.8})
    assert out['buoy_radius_m'] == 0.9 and out['square_inside_margin_m'] == 0.8
    assert _levels(iss) == ['ERROR'] and '안전영역 소멸' in iss[0][1]


def test_north_radius_copy_matches_hold_logic():
    from ship_back import hold_logic
    assert SQ.NORTH_ARRIVE_HOLD_M == hold_logic.NORTH_ARRIVE_HOLD_M == 3.5   # north yaml 대조는 test_gps_hold
    assert DEFAULTS['square_engage_m'] >= SQ.NORTH_ARRIVE_HOLD_M + SQ.ENGAGE_MIN_MARGIN_M


def test_square_applies_mode_2_only():
    assert square_hold_applies('square', 2)
    for m in (0, 1, 3, 5, 7, 8, -1):
        assert not square_hold_applies('square', m)
    assert not square_hold_applies('gps', 2) and not square_hold_applies('vision', 2)


# ─────────────── /buoy_bearings 형식 사본 ───────────────
def test_bb_format_copy_matches_detector_source():
    src = open(STOP_LOGIC, encoding='utf-8').read()
    for name, val in (('BB_VER', SQ.BB_VER), ('BB_STRIDE', SQ.BB_STRIDE), ('BB_HEADER', SQ.BB_HEADER)):
        m = re.search(rf'^{name}\s*=\s*(\d+)', src, re.M)
        assert m and int(m.group(1)) == val, (name, m and m.group(1), val)


def test_bb_parse_roundtrip_and_errors():
    data = [1.0, 4.0, 2.0, 12.5, 300.0, 0.7, 0.05, -20.0, 80.0, 0.2, 0.1]
    rows, err = parse_buoy_bearings(data)
    assert err is None and rows == [(12.5, 300.0, 0.7, 0.05), (-20.0, 80.0, 0.2, 0.1)]
    assert parse_buoy_bearings([1.0, 4.0, 0.0]) == ([], None)            # 하트비트
    for bad in ([], [1.0, 4.0], [2.0, 4.0, 0.0], [1.0, 3.0, 0.0], [1.0, 4.0, 1.0, 1.0], [1.0, 4.0, 1.5],
                [1.0, 4.0, -1.0], ['a'], None, [1.0, 4.0, NAN]):
        rows, err = parse_buoy_bearings(bad)
        assert rows == [] and err is not None, bad


def test_bb_parse_matches_detector_pack_when_importable():
    """color_shape_detector 를 import 할 수 있으면(cv2 등) 실제 pack_bearings 와 왕복."""
    sys.path.insert(0, os.path.join(PKG, '..', 'color_shape_detector'))
    try:
        from color_shape_detector.stop_detect_logic import pack_bearings, unpack_bearings
    except ImportError:
        return
    rows = [(10.0, 500.0, 0.9, 0.03), (-31.0, 60.0, 0.1, 0.0)]
    data, dropped = pack_bearings(rows)
    assert dropped == 0
    assert parse_buoy_bearings(data) == (unpack_bearings(data)[0], None)
    assert parse_buoy_bearings(data)[0] == rows


# ─────────────── GPS·기하 ───────────────
def test_fix_usable_and_local_frame():
    assert fix_usable(2, 35.1, 128.5) and fix_usable(0, 35.1, 128.5)
    for st, la, lo in ((-1, 35.1, 128.5), (2, 0.0, 0.0), (2, NAN, 128.5), (2, 35.1, INF), (None, 35.1, 128.5)):
        assert not fix_usable(st, la, lo), (st, la, lo)
    x, y = ll_to_xy(35.18700, 128.56610, 35.18690, 128.56600)
    assert abs(y - 11.13) < 0.05 and abs(x - 9.10) < 0.05, (x, y)
    la, lo = xy_to_ll(x, y, 35.18690, 128.56600)
    assert abs(la - 35.18700) < 1e-9 and abs(lo - 128.56610) < 1e-9


def test_gps_speed():
    a = (35.0, 128.0, 2, 10.0)
    lat1, lon1 = xy_to_ll(0.0, 0.8, 35.0, 128.0)
    b = (lat1, lon1, 2, 11.0)
    assert abs(gps_speed(a, b, 2.0) - 0.8) < 1e-6
    assert gps_speed(None, b, 2.0) is None and gps_speed(a, (lat1, lon1, 2, 13.5), 2.0) is None
    assert gps_speed(a, (lat1, lon1, -1, 11.0), 2.0) is None and gps_speed(a, (lat1, lon1, 2, 10.0), 2.0) is None


def test_project_conventions():
    # 북쪽을 보고(0°) 왼쪽 90° 5 m → 서쪽(−x)
    x, y = project(0.0, 0.0, 0.0, 90.0, 5.0)
    assert abs(x + 5.0) < 1e-9 and abs(y) < 1e-9
    x, y = project(1.0, 2.0, 90.0, 0.0, 3.0)          # 동쪽을 보고 정면 3 m
    assert abs(x - 4.0) < 1e-9 and abs(y - 2.0) < 1e-9
    x, y = project(0.0, 0.0, 0.0, -30.0, 2.0)          # 오른쪽 30°
    assert x > 0 and abs(bearing_to(0, 0, x, y) - 30.0) < 1e-9


def test_hull_inside_and_safe_region():
    q = order_ccw(_square())
    inset = 0.25 + 0.3
    assert hull_inside(q, 0.0, 0.0, 0.0, 1.77, 0.57, inset)
    assert hull_inside(q, 0.0, 0.0, 45.0, 1.77, 0.57, inset)
    # 중심에서 1.0 m 옆(변까지 1.5 m) — 길이 방향이 변 쪽이면 0.885 + 0.55 = 1.435 < 1.5 안, 1.2 m 면 밖
    assert hull_inside(q, 1.0, 0.0, 90.0, 1.77, 0.57, inset)
    assert not hull_inside(q, 1.2, 0.0, 90.0, 1.77, 0.57, inset)
    assert hull_inside(q, 1.2, 0.0, 0.0, 1.77, 0.57, inset)      # 폭 방향이면 0.285 + 0.55 + 1.2 < 2.5
    assert not hull_inside(q, 0.0, 0.0, None, 1.77, 0.57, inset) and not hull_inside(None, 0, 0, 0, 1.77, 0.57, 0)
    assert safe_region_exists(q, inset, 1.77, 0.57)
    assert not safe_region_exists(order_ccw(_square(side=2.5)), inset, 1.77, 0.57)   # 2.5 − 1.1 = 1.4 < 1.77
    assert not safe_region_exists(q, 1.7, 1.77, 0.57)


def test_near_hazard():
    L, B, c = 1.77, 0.57, 0.4
    assert near_hazard(None, L, B, c) == (False, False)
    assert near_hazard((1.2, 0.0), L, B, c) == (True, True)       # 선수 앞 0.315 m
    f, s = near_hazard((1.4, 0.0), L, B, c)
    assert not f and not s                                         # 선수 앞 0.515 > 0.4, 1.4 > 0.93 + 0.4
    f, s = near_hazard((1.0, 90.0), L, B, c)                       # 옆 1.0 m — 정면 아님, 선회는 금지
    assert not f and s
    f, s = near_hazard((1.0, 180.0), L, B, c)                      # 뒤
    assert not f and s
    assert near_hazard((NAN, 0.0), L, B, c) == (False, False)


def test_entry_target_avoids_corner():
    q = order_ccw(_square())
    tgt, staging = entry_target((0.0, 8.0), q, (0.0, 0.0), 1.2, 1.5)      # 변 가운데로 → 바로 중심
    assert tgt == (0.0, 0.0) and not staging
    tgt, staging = entry_target((6.0, 6.0), q, (0.0, 0.0), 1.2, 1.5)      # 꼭짓점 쪽 → 변 바깥 중간점
    assert staging and min(abs(tgt[0]), abs(tgt[1])) < 1e-6 and max(abs(tgt[0]), abs(tgt[1])) == 4.0, tgt
    tgt, staging = entry_target((0.5, -0.5), q, (0.0, 0.0), 1.2, 1.5)     # 이미 안
    assert tgt == (0.0, 0.0) and not staging


def test_seek_approach_point():
    A, B = (-2.5, 2.5), (2.5, 2.5)
    p = seek_approach_point((A, B), (0.0, 9.0), 5.0, 1.5)        # 변 → 배 쪽 바깥 1.5 m
    assert abs(p[0]) < 1e-9 and abs(p[1] - 4.0) < 1e-9
    p = seek_approach_point(((-2.5, -2.5), (2.5, 2.5)), (0.0, 9.0), 5.0, 1.5)   # 대각선 → 중심
    assert abs(p[0]) < 1e-9 and abs(p[1]) < 1e-9


# ─────────────── 사각형 맞추기 ───────────────
def test_fit_four_points_any_order_with_outlier():
    pts = _square(rot=27.0, cx=3.0, cy=-4.0)
    rnd = random.Random(1)
    for _ in range(5):
        p = list(pts) + [(20.0, 20.0)]
        rnd.shuffle(p)
        f = fit_square(p, 5.0, 1.5)
        assert f.kind == 4 and abs(f.center[0] - 3.0) < 1e-9 and abs(f.center[1] + 4.0) < 1e-9
        assert len(f.corners) == 4 and f.resid < 1e-9


def test_fit_three_points_parallelogram_completion():
    a, b, c, d = _square(rot=10.0)
    for missing in range(4):
        three = [p for i, p in enumerate((a, b, c, d)) if i != missing]
        f = fit_square(three, 5.0, 1.5)
        want = (a, b, c, d)[missing]
        assert f.kind == 3 and math.hypot(f.predicted[0][0] - want[0], f.predicted[0][1] - want[1]) < 1e-9
        assert math.hypot(*f.center) < 1e-9


def test_fit_drifted_square_within_tol():
    rnd = random.Random(7)
    pts = [(x + rnd.uniform(-0.6, 0.6), y + rnd.uniform(-0.6, 0.6)) for x, y in _square()]
    assert fit_square(pts, 5.0, 2.0).kind == 4
    assert fit_square(pts[:3], 5.0, 2.0).kind == 3


def test_fit_two_points_side_far_hypothesis_first_and_diagonal():
    A, B = (-2.5, 2.5), (2.5, 2.5)
    f = fit_square([A, B], 5.0, 1.5, boat=(0.0, 9.0))
    assert f.kind == 2 and len(f.predicted) == 4
    assert all(p[1] < 0.0 for p in f.predicted[:2]) and all(p[1] > 5.0 for p in f.predicted[2:])
    f = fit_square([(-2.5, -2.5), (2.5, 2.5)], 5.0, 1.5)
    pr = sorted((round(p[0], 6), round(p[1], 6)) for p in f.predicted)
    assert f.kind == 2 and pr == [(-2.5, 2.5), (2.5, -2.5)]
    assert fit_square([(0.0, 0.0), (12.0, 0.0)], 5.0, 1.5).kind == 0
    assert fit_square([], 5.0, 1.5).kind == 0


# ─────────────── 지도 ───────────────
def test_landmark_map_confirm_and_error_cap():
    m = LandmarkMap(1.2, 5, 0.6)
    for i in range(4):
        m.add(1.0 + 0.01 * i, 2.0, i)
    assert m.confirmed() == []                      # 4 < 5
    m.add(1.0, 2.0, 5)
    assert len(m.confirmed()) == 1
    m.add(5.0, 5.0, 6)                              # 멀다 → 새 부표
    assert len(m.lms) == 2
    # 흩어짐이 오차 상한을 넘는 부표는 확정 안 함(성공 금지로 이어짐)
    m2 = LandmarkMap(3.0, 5, 0.6)
    for i in range(10):
        m2.add(0.9 if i % 2 else -0.9, 0.0, i)
    assert m2.lms[0].spread > 0.6 and m2.confirmed() == []


# ─────────────── 헤딩 ───────────────
def test_heading_fuser_hold_compensated_by_raw():
    f = HeadingFuser(use_raw=True, tau_sec=0.0)
    t = 0.0
    f.update_raw(100.0, t)
    f.update_yaw(30.0, t, speed=1.0)                       # 첫 offset = −70
    assert abs(f.heading(t) - 30.0) < 1e-9
    for k in range(1, 31):                                 # 3 s 동안 raw 가 +45°, /imu/yaw 는 HOLD(30 그대로)
        t = k * 0.1
        f.update_raw(100.0 + 1.5 * k, t)
        f.update_yaw(30.0, t, speed=0.0)
    assert abs(f.heading(t) - 75.0) < 1e-9
    g = HeadingFuser(use_raw=False)
    g.update_raw(100.0, 0.0)
    g.update_yaw(30.0, 0.0)
    for k in range(1, 31):
        g.update_raw(100.0 + 1.5 * k, k * 0.1)
        g.update_yaw(30.0, k * 0.1)
    assert g.heading(3.0) == 30.0                          # 옛 방식은 회전을 모른다


def test_heading_fuser_learns_only_when_moving_straight():
    f = HeadingFuser(use_raw=True, tau_sec=0.0, min_speed=0.6, max_turn_dps=5.0)
    f.update_raw(0.0, 0.0)
    f.update_yaw(10.0, 0.0, speed=1.0)
    f.update_raw(0.0, 0.5)
    f.update_yaw(20.0, 0.5, speed=0.3)                     # 느림 → 안 배움
    assert abs(f.heading(0.5) - 10.0) < 1e-9
    f.update_yaw(25.0, 0.6, speed=None)                    # 속도 모름 → 안 배움
    assert abs(f.heading(0.6) - 10.0) < 1e-9
    f.update_raw(0.0, 0.7)
    f.update_yaw(20.0, 0.7, speed=1.0)                     # 빠르고 곧게 → 배움(tau 0 = 그대로)
    assert abs(f.heading(0.7) - 20.0) < 1e-9
    f.update_raw(5.0, 0.8)                                 # 0.1 s 에 5° = 50°/s 선회
    f.update_yaw(60.0, 0.8, speed=1.0)                     # 선회 중 COG → 안 배움
    assert abs(f.heading(0.8) - 25.0) < 1e-9
    # 평활: tau 3 s, dt 1 s → 1 − e^(−1/3) ≈ 0.283 만큼
    h = HeadingFuser(use_raw=True, tau_sec=3.0)
    h.update_raw(0.0, 0.0)
    h.update_yaw(0.0, 0.0, speed=1.0)
    h.update_raw(0.0, 1.0)
    h.update_yaw(10.0, 1.0, speed=1.0)
    assert abs(h.heading(1.0) - 10.0 * (1 - math.exp(-1 / 3))) < 1e-6


def test_heading_fuser_stale_invert_interp_rate():
    f = HeadingFuser(use_raw=True, yaw_stale=1.0, raw_stale=0.5)
    assert f.heading(0.0) is None
    f.update_yaw(30.0, 0.0, speed=1.0)                     # raw 없음 → offset 없음
    assert f.heading(0.0) is None
    f.update_raw(10.0, 0.0)
    assert f.heading(0.0) is None                          # offset 은 다음 /imu/yaw 에서
    f.update_yaw(30.0, 0.1, speed=1.0)
    assert abs(f.heading(0.1) - 30.0) < 1e-9
    assert f.heading(0.61) is None                         # raw 0.5 s 넘게 묵음
    f.update_raw(10.0, 1.0)
    assert f.heading(1.0) is not None and f.heading(1.11) is None   # /imu/yaw 1.0 s 넘게 묵음
    inv = HeadingFuser(use_raw=True, invert=True, tau_sec=0.0)
    inv.update_raw(10.0, 0.0)
    inv.update_yaw(100.0, 0.0, speed=1.0)
    inv.update_raw(20.0, 0.1)                              # invert: raw +10 = 반시계 10
    assert abs(wrap180(inv.heading(0.1) - 90.0)) < 1e-9
    h = HeadingFuser(use_raw=True)
    h.update_raw(0.0, 0.0)
    h.update_yaw(0.0, 0.0, speed=1.0)                      # 배움(offset 0)
    for k in range(1, 11):
        h.update_raw(10.0 * k, k * 0.1)                    # 100 °/s
    h.update_yaw(0.0, 1.0, speed=0.0)                      # 신선도만(HOLD)
    assert abs(h.heading(1.0) - 100.0) < 1e-6
    assert abs(h.heading_at(0.55, 1.0) - 55.0) < 1e-6
    assert abs(h.heading_at(-5.0, 1.0) - 0.0) < 1e-6 and abs(h.heading_at(9.0, 1.0) - 100.0) < 1e-6
    assert abs(h.turn_rate(1.0) - 100.0) < 1e-6
    assert HeadingFuser(use_raw=False).turn_rate(0.0) is None


def test_heading_fuser_never_moved_is_unknown():
    """정지 출발: /imu/yaw 가 gps_cog INIT_GOAL(WP 방위)·HOLD 뿐이면 헤딩이 아니다 → None(모르면 입 다문다)."""
    f = HeadingFuser(use_raw=True)
    for k in range(50):
        f.update_raw(5.0, k * 0.1)
        f.update_yaw(123.0, k * 0.1, speed=0.0 if k % 2 else None)
    assert f.heading(4.9) is None and f.offset is not None and not f.learned
    f.update_yaw(124.0, 5.0, speed=0.8)                    # 처음으로 움직이며 → 그 값 그대로(평활 없이)
    assert f.learned and abs(f.heading(5.0) - 124.0) < 1e-9


# ─────────────── 명령 ───────────────
def test_commands_and_output_set():
    assert spin_cmd(10.0, 0.0, 0.3, 0.4) == SR and spin_cmd(-10.0, 0.1, 0.3, 0.4) == SL
    assert spin_cmd(10.0, 0.5, 0.3, 0.4) == STOP and spin_cmd(10.0, 0.5, 0.3, 0.0) == SR
    assert steer_value(30.0) == 30.0 and steer_value(-30.0) == 330.0
    assert steer_value(120.0) == 80.0 and steer_value(-170.0) == 280.0
    for v in (0.0, 80.0, 280.0, 359.9, STOP, SR, SL, REV):
        assert output_allowed(v, 'HOLD'), v
    for v in (INV, 81.0, 279.0, 90.0, 10000.0, 270.0):
        assert not output_allowed(v, 'DORI'), v
    assert output_allowed(INV, 'INIT') and output_allowed(INV, 'DONE') and not output_allowed(STOP, 'DONE')
    assert not is_steer(REV) and not is_steer(INV)


def test_continuous_timer_resets():
    t = ContinuousTimer()
    assert t.update(True, 0.0) == 0.0 and t.update(True, 4.9) == 4.9
    assert t.update(False, 5.0) == 0.0 and t.update(True, 5.1) == 0.0 and t.update(True, 10.1) == 5.0


# ─────────────── FSM: 합성 세계 ───────────────
P = dict(DEFAULTS, hold_time_sec=5.0, unknown_giveup_sec=10.0)


class World:
    """순수 운동학 세계 — 이 테스트 전용(ROS·노드 없이 FSM 출력만 본다).

    20000 = north LOS 대역(중심으로 1 m/s), 조향 = 0.6 m/s + 회전, 5000/6000 = 제자리 ±30 °/s, 180 = 후진 0.4 m/s.
    관측 = 카메라 ±32°·10 m 안 참 부표 + 잡음(σ 0.05 m). near = 가장 가까운 부표 표면(거리, 좌 + 방위).
    """

    def __init__(self, seed=0, start=(0.0, 12.0), heading=180.0, buoys=None, fov=32.0):
        self.rng = random.Random(seed)
        self.buoys = buoys or _square(rot=self.rng.uniform(0, 90))
        self.x, self.y = start
        self.h = heading
        self.fov = fov
        self.contacts = 0

    def obs(self):
        out = []
        for bx, by in self.buoys:
            d = math.hypot(bx - self.x, by - self.y)
            rel = wrap180(self.h - bearing_to(self.x, self.y, bx, by))
            if d <= 10.0 and abs(rel) <= self.fov:
                out.append((bx + self.rng.gauss(0, 0.05), by + self.rng.gauss(0, 0.05)))
        return out

    def near(self, br=0.25):
        best = None
        for bx, by in self.buoys:
            d = math.hypot(bx - self.x, by - self.y) - br
            if best is None or d < best[0]:
                best = (d, wrap180(self.h - bearing_to(self.x, self.y, bx, by)))
        return best

    def act(self, v, dt=0.1):
        if v == INV:                                   # north LOS 대역: 중심 쪽으로 최대 40 °/s 돌며 1 m/s
            spd, w = 1.0, max(-40.0, min(40.0, wrap180(bearing_to(self.x, self.y, 0.0, 0.0) - self.h) / dt))
        elif v == STOP:
            spd, w = 0.0, 0.0
        elif v == SR:
            spd, w = 0.0, 30.0
        elif v == SL:
            spd, w = 0.0, -30.0
        elif v == REV:
            spd, w = -0.4, 0.0
        else:
            rel = v if v <= 180.0 else v - 360.0
            spd, w = 0.6, max(-40.0, min(40.0, -rel))
        self.h = (self.h + w * dt) % 360.0
        self.x += spd * math.sin(math.radians(self.h)) * dt
        self.y += spd * math.cos(math.radians(self.h)) * dt
        for bx, by in self.buoys:
            if math.hypot(bx - self.x, by - self.y) < 0.25 + 0.285:
                self.contacts += 1


def _run_world(w, p=P, t_max=150.0, bearings_ok=lambda t: True, xy_ok=lambda t: True):
    fsm = SquareFSM(p)
    fsm.reset(0.0)
    outs, logs = [], []
    t = 0.0
    while t < t_max:
        gd = math.hypot(w.x, w.y)
        v, lg = fsm.step(t, goal_dist=gd, xy=(w.x, w.y) if xy_ok(t) else None, heading=w.h,
                         bearings_ok=bearings_ok(t), obs=w.obs() if bearings_ok(t) else [], near=w.near())
        outs.append((round(t, 1), fsm.state, v))
        logs += lg
        if fsm.state == 'DONE' and len(outs) > 2 and outs[-2][1] == 'DONE':
            break
        w.act(v)
        t += 0.1
    return fsm, outs, logs


def _check_output_contract(outs):
    for t, st, v in outs:
        assert output_allowed(v, st), (t, st, v)
    seen_engage = False
    for t, st, v in outs:
        if st != 'INIT':
            seen_engage = True
        if seen_engage and st != 'DONE':
            assert v != INV, ('DONE 전 20000', t, st)


def test_fsm_reaches_done_from_many_directions_and_obeys_output_set():
    for seed, (sx, sy) in enumerate(((0.0, 14.0), (10.0, 10.0), (-13.0, 2.0), (3.0, -15.0), (-9.0, -9.0))):
        w = World(seed=seed, start=(sx, sy), heading=bearing_to(sx, sy, 0.0, 0.0))
        fsm, outs, logs = _run_world(w)
        assert fsm.state == 'DONE' and fsm.success, (seed, fsm.done_reason, [x for x in logs if x[0] != 'INFO'])
        _check_output_contract(outs)
        assert w.contacts == 0, seed
        # DONE 순간 선체가 참 사각형 안
        assert hull_inside(order_ccw(w.buoys), w.x, w.y, w.h, 1.77, 0.57, 0.25), seed
        assert {'INIT', 'HOLD', 'DONE'} <= {s for _, s, _ in outs}


def test_fsm_dori_sweep_finds_buoys_behind():
    """진입 때 사각형이 등 뒤(카메라 밖)여도 도리도리(±40·80·120°)로 찾는다. 시작부터 8 m 안이라 INIT 1 s 뒤 바로."""
    w = World(seed=3, start=(0.0, 6.5), heading=0.0)         # 북쪽을 봄 = 사각형 등짐
    fsm, outs, logs = _run_world(w)
    states = [s for _, s, _ in outs]
    assert 'DORI' in states and fsm.state == 'DONE' and fsm.success, (fsm.done_reason, logs[-5:])
    _check_output_contract(outs)
    assert SR in [v for _, _, v in outs] or SL in [v for _, _, v in outs]


def test_fsm_init_waits_for_engage_distance_and_min_time():
    fsm = SquareFSM(P)
    fsm.reset(100.0)
    assert fsm.step(100.0, goal_dist=2.0)[0] == INV                    # init_min_sec 전
    assert fsm.step(100.5, goal_dist=2.0)[0] == INV
    assert fsm.step(101.0, goal_dist=None)[0] == INV                   # 거리 모름
    assert fsm.step(101.1, goal_dist=8.0)[0] == INV                    # 경계(8.0)는 아직
    v, logs = fsm.step(101.2, goal_dist=7.9)                           # 엔게이지 — 센서 없으면 정지
    assert fsm.state == 'DORI' and v == STOP and any(lv == 'ERROR' for lv, _ in logs)
    _, logs = fsm.step(101.3, goal_dist=7.9)
    assert not [x for x in logs if x[0] == 'ERROR'], '같은 ERROR 는 5 s 에 한 번'
    v, logs = fsm.step(106.3, goal_dist=7.9)
    assert v == STOP and [x for x in logs if x[0] == 'ERROR']


def test_fsm_init_warns_once_without_goal_distance():
    fsm = SquareFSM(P)
    fsm.reset(0.0)
    warns = []
    for k in range(260):
        v, lg = fsm.step(k * 0.1)
        assert v == INV
        warns += [m for lv, m in lg if lv == 'WARN']
    assert len(warns) == 1 and '/goal_distance' in warns[0]


def test_fsm_stale_bearings_stop_error_then_giveup_done():
    """검출기 침묵(stale) → 50000 + ERROR(5 s 주기), unknown_giveup_sec(10 s) 뒤 포기 DONE(성공 아님)."""
    w = World(seed=1, start=(0.0, 7.0))
    fsm, outs, logs = _run_world(w, bearings_ok=lambda t: t < 1.5, t_max=40.0)
    after = [(t, s, v) for t, s, v in outs if t >= 1.5 and s != 'DONE']
    assert after and all(v == STOP for _, _, v in after), after[:5]
    assert fsm.state == 'DONE' and fsm.success is False and '포기' in fsm.done_reason
    errs = [m for lv, m in logs if lv == 'ERROR']
    assert any('/buoy_bearings' in m for m in errs) and len(errs) >= 3
    t_done = next(t for t, s, _ in outs if s == 'DONE')
    assert 11.0 <= t_done <= 12.5, t_done
    _check_output_contract(outs)


def test_fsm_no_fix_stops():
    w = World(seed=1, start=(0.0, 7.0))
    fsm, outs, logs = _run_world(w, xy_ok=lambda t: False, t_max=5.0)
    assert all(v == STOP for t, s, v in outs if s != 'INIT')
    assert any('GPS fix' in m for lv, m in logs if lv == 'ERROR')


def _hold_fsm(p=P, buoys=None, spread=0.0, n=20):
    """4 부표가 이미 확정된 HOLD 직전 FSM. spread > 0 이면 부표마다 ±spread 로 흩어 관측(같은 부표로 묶이게 assoc 3 m)."""
    buoys = buoys or _square()
    if spread:
        p = dict(p, buoy_assoc_m=3.0)
    fsm = SquareFSM(p)
    fsm.reset(0.0)
    fsm.step(1.0, goal_dist=1.0)
    for i in range(n):
        for bx, by in buoys:
            fsm.map.add(bx + (spread if i % 2 else -spread), by, 1.0)
    return fsm


def test_fsm_inside_5s_continuous_and_reset_on_exit():
    fsm = _hold_fsm()
    t = 1.0
    hist = []
    # 중심 근처 3 s → 한 틱 밖(1.6 m 옆, 길이 방향 변 쪽) → 4.9 s 안 → 아직, 5 s 넘으면 DONE
    for k in range(30):
        t += 0.1
        hist.append(fsm.step(t, goal_dist=0.5, xy=(0.0, 0.0), heading=0.0, bearings_ok=True)[0])
    assert fsm.state == 'HOLD'
    t += 0.1
    v, _ = fsm.step(t, goal_dist=0.5, xy=(1.6, 0.0), heading=90.0, bearings_ok=True)
    assert fsm.state == 'HOLD' and not fsm.inside and v != INV
    for k in range(49):
        t += 0.1
        fsm.step(t, goal_dist=0.5, xy=(0.0, 0.0), heading=0.0, bearings_ok=True)
    assert fsm.state == 'HOLD', '리셋 뒤 4.9 s 는 아직'
    for k in range(3):
        t += 0.1
        v, logs = fsm.step(t, goal_dist=0.5, xy=(0.0, 0.0), heading=0.0, bearings_ok=True)
    assert fsm.state == 'DONE' and fsm.success and v == INV
    assert any('✅' in m for lv, m in logs if lv == 'INFO')
    assert all(x != INV for x in hist)


def test_fsm_buoy_error_over_cap_forbids_success():
    """부표 흩어짐 > buoy_pos_max_err_m → 확정 안 됨 → 4점 아님 → 아무리 머물러도 DONE(성공) 없음."""
    fsm = _hold_fsm(spread=0.9)
    t = 1.0
    for k in range(200):
        t += 0.1
        v, _ = fsm.step(t, goal_dist=0.5, xy=(0.0, 0.0), heading=0.0, bearings_ok=True)
        assert v != INV
    assert fsm.state != 'DONE' and fsm.fit.kind < 4


def test_fsm_safe_region_gone_forbids_success_error_once():
    p = dict(P, buoy_radius_m=0.9, square_inside_margin_m=0.8)
    fsm = _hold_fsm(p=p)
    t = 1.0
    errs = []
    for k in range(200):
        t += 0.1
        v, lg = fsm.step(t, goal_dist=0.5, xy=(0.0, 0.0), heading=0.0, bearings_ok=True)
        errs += [m for lv, m in lg if lv == 'ERROR']
        assert v != INV
    assert fsm.state == 'HOLD' and not fsm.inside
    assert len(errs) == 1 and '안전영역 소멸' in errs[0]


def test_fsm_square_max_giveup():
    fsm = SquareFSM(dict(P, square_max_sec=20.0))
    fsm.reset(0.0)
    fsm.step(1.0, goal_dist=5.0)
    t = 1.0
    while fsm.state != 'DONE' and t < 30.0:
        t += 0.1
        v, logs = fsm.step(t, goal_dist=5.0, xy=(0.0, 6.0), heading=180.0, bearings_ok=True)
    assert fsm.state == 'DONE' and fsm.success is False and 20.9 <= t <= 21.2
    assert any('포기' in m for lv, m in logs if lv == 'ERROR')


def test_fsm_close_front_reverses_briefly_side_does_not():
    fsm = _hold_fsm()
    t = 1.1
    v, _ = fsm.step(t, goal_dist=2.0, xy=(0.0, 1.5), heading=0.0, bearings_ok=True, near=(1.1, 0.0))
    assert v == REV
    outs = [v]
    for k in range(25):
        t += 0.1
        v, _ = fsm.step(t, goal_dist=2.0, xy=(0.0, 1.5), heading=0.0, bearings_ok=True, near=(1.1, 0.0))
        outs.append(v)
    # 1 s 후진 → 1 s 쉼(정면이 막혀 조향도 정지) → 다시 짧게 후진
    assert outs[:10] == [REV] * 10 and outs[11:20] == [STOP] * 9 and outs[21] == REV, outs
    fsm2 = _hold_fsm()
    v, _ = fsm2.step(1.1, goal_dist=2.0, xy=(0.0, 1.5), heading=0.0, bearings_ok=True, near=(1.0, 90.0))
    assert v != REV and v not in (SR, SL)          # 옆 = 후진 아님, 선회 휩쓸림 원 안 → 선회 금지


def test_fsm_mode_reset_clears_map_and_done():
    w = World(seed=0, start=(0.0, 10.0))
    fsm, outs, _ = _run_world(w)
    assert fsm.state == 'DONE'
    fsm.reset(500.0)
    assert fsm.state == 'INIT' and fsm.map.lms == [] and fsm.center is None and fsm.success is None
    assert fsm.step(500.0, goal_dist=1.0)[0] == INV

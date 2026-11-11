#!/usr/bin/env python3
"""obstacle_cam_logic 순수 로직 테스트 (계약서_카메라장애물_20261111 U2). ROS·OpenCV 없이 돈다.

실행: tools/run_tests.sh color_shape_detector
"""

import ast
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from color_shape_detector.obstacle_cam_logic import (  # noqa: E402
    BOTTOM_TOUCH_PX, DEFAULT_CONFIRM_FRAMES, DEFAULT_CONFIRM_WINDOW, DEFAULT_PARAMS, ConfirmWindow,
    bbox_center_bearing, bbox_range, cam_to_lidar, check_confirm, check_params, detections_to_array,
    horizon_row, out_grid, pixel_to_bearing, pixel_to_bearing_tilt, process_frame, row_to_range)
from color_shape_detector.vision_geom import DEFAULT_HFOV_DEG, angle_from_pixel  # noqa: E402

W, H = 640, 360
INF = math.inf
MOD = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'color_shape_detector',
                   'obstacle_cam_logic.py')


def P(**kw):
    p = dict(DEFAULT_PARAMS)
    p.update(kw)
    return p


def near(a, b, tol=1e-9):
    return abs(a - b) <= tol


def row_for_range(z, p):
    """역산: 수평거리 z(range_scale 전) 가 나오는 행."""
    fy = (W / 2.0) / math.tan(math.radians(p["hfov_deg"]) / 2.0)
    delta = math.atan(p["cam_height_m"] / z)
    return H / 2.0 + fy * math.tan(delta - math.radians(p["cam_tilt_deg"]))


def cells(arr):
    """배열 → [(상대방위, r)] 유한 칸만."""
    a0, inc = arr[0], arr[1]
    return [(a0 + k * inc, r) for k, r in enumerate(arr[2:]) if math.isfinite(r)]


# ------------------------------------------------------------------ 기본값·모듈
def test_defaults_valid_and_hfov_from_vision_geom():
    assert check_params(DEFAULT_PARAMS) == []
    assert DEFAULT_PARAMS["hfov_deg"] == DEFAULT_HFOV_DEG == 63.9
    want = {"cam_height_m": 0.35, "cam_tilt_deg": 11.0, "range_scale": 0.85, "cam_x_from_lidar_m": 0.0,
            "cam_y_from_lidar_m": 0.0, "max_range_m": 5.0, "min_range_out_m": 0.5,
            "min_sector_deg": 3.0, "out_angle_inc_deg": 0.5}
    want["max_bbox_width_deg"] = 25.0
    for k, v in want.items():
        assert DEFAULT_PARAMS[k] == v and type(DEFAULT_PARAMS[k]) is float, k
    assert DEFAULT_PARAMS["max_bboxes"] == 4 and type(DEFAULT_PARAMS["max_bboxes"]) is int
    assert (DEFAULT_CONFIRM_FRAMES, DEFAULT_CONFIRM_WINDOW) == (3, 5)
    assert type(DEFAULT_CONFIRM_FRAMES) is int and type(DEFAULT_CONFIRM_WINDOW) is int


def test_no_ros_or_cv2_import():
    """순수 모듈 — import 는 math·numbers·collections·vision_geom 뿐(시뮬 U3 가 그대로 부른다)."""
    with open(MOD, encoding='utf-8') as f:
        tree = ast.parse(f.read())
    mods = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            mods.add(node.module)
    assert mods <= {'math', 'numbers', 'collections', 'color_shape_detector.vision_geom'}, mods


# ------------------------------------------------------------------ (1) 픽셀 → 방위
def test_pixel_bearing_sign_left_positive():
    p = DEFAULT_PARAMS
    assert pixel_to_bearing(100, W, p["hfov_deg"]) > 0.0         # 화면 왼쪽 = +
    assert pixel_to_bearing(540, W, p["hfov_deg"]) < 0.0
    assert near(pixel_to_bearing(320, W, p["hfov_deg"]), 0.0)
    assert near(pixel_to_bearing(0, W, p["hfov_deg"]), 63.9 / 2.0, 1e-9)
    assert near(pixel_to_bearing(640, W, p["hfov_deg"]), -63.9 / 2.0, 1e-9)
    assert pixel_to_bearing(200, W, 63.9) == angle_from_pixel(200, W, 63.9)   # 재사용


def test_pixel_bearing_invalid_is_none():
    assert pixel_to_bearing(math.nan, W, 63.9) is None
    assert pixel_to_bearing(100, 0, 63.9) is None
    assert pixel_to_bearing(100, W, 0.0) is None
    assert pixel_to_bearing(100, W, 180.0) is None
    assert pixel_to_bearing(None, W, 63.9) is None


def _ray_plane_bearing(x, y, tilt_deg, h=0.35):
    """독립 검산: 픽셀 광선을 3D 로 돌려 수면(h 아래)과 만나는 점의 방위(좌 +). 회전행렬로 직접."""
    f = (W / 2.0) / math.tan(math.radians(DEFAULT_HFOV_DEG) / 2.0)
    d_cam = (x - W // 2) / f, (y - H / 2.0) / f, 1.0           # 카메라 프레임: 오른쪽, 아래, 광축
    t = math.radians(tilt_deg)
    # 수평 프레임(앞, 오른쪽, 아래): 광축은 아래로 t 숙임
    fwd = d_cam[2] * math.cos(t) - d_cam[1] * math.sin(t)
    down = d_cam[2] * math.sin(t) + d_cam[1] * math.cos(t)
    right = d_cam[0]
    assert down > 0.0
    s = h / down
    return -math.degrees(math.atan2(right * s, fwd * s))


def test_bearing_tilt_exact_matches_ray_plane():
    """틸트 11°: 바닥 근처 행·화면 가장자리에서 정확식 = 3D 광선-수면 교점 방위. 옛 식(틸트 무시)은 2° 넘게 틀린다."""
    for x in (0.0, 1.0, 40.0, 320.0, 600.0, 639.0, 640.0):
        for y in (120.0, 250.0, 340.0, 359.0, 360.0):
            got = pixel_to_bearing_tilt(x, y, W, H, 63.9, 11.0)
            assert near(got, _ray_plane_bearing(x, y, 11.0), 1e-9), (x, y)
    corner = pixel_to_bearing_tilt(0.0, 360.0, W, H, 63.9, 11.0)
    assert corner > 34.0 and corner - angle_from_pixel(0.0, W, 63.9) > 2.0     # 정정한 docstring 의 2.3°
    assert pixel_to_bearing_tilt(640.0, 360.0, W, H, 63.9, 11.0) == -corner      # 좌우 대칭, 왼쪽 +
    assert near(pixel_to_bearing_tilt(320.0, 360.0, W, H, 63.9, 11.0), 0.0)


def test_bearing_tilt_zero_equals_angle_from_pixel():
    for x in (0.0, 13.0, 100.5, 320.0, 333.0, 639.0, 640.0):
        for y in (0.0, 180.0, 360.0):
            assert pixel_to_bearing_tilt(x, y, W, H, 63.9, 0.0) == angle_from_pixel(x, W, 63.9), (x, y)
            assert pixel_to_bearing_tilt(x, y, W, H, 80.32, 0.0) == angle_from_pixel(x, W, 80.32)


def test_bearing_tilt_invalid_is_none():
    assert pixel_to_bearing_tilt(math.nan, 300.0, W, H, 63.9, 11.0) is None
    assert pixel_to_bearing_tilt(100.0, None, W, H, 63.9, 11.0) is None
    assert pixel_to_bearing_tilt(100.0, 300.0, 0, H, 63.9, 11.0) is None
    assert pixel_to_bearing_tilt(100.0, 300.0, W, H, 0.0, 11.0) is None
    assert pixel_to_bearing_tilt(100.0, 300.0, W, H, 63.9, math.inf) is None
    assert pixel_to_bearing_tilt(100.0, 1e6, W, H, 63.9, 80.0) is None          # 광선이 뒤로(분모 ≤ 0)


# ------------------------------------------------------------------ (2) 행 → 거리
def test_row_range_contract_numbers():
    """계약서 위험 4: h 0.345·틸트 11°·fy 513.5 → 수평선 ≈80행, 3 m ≈140행, 바닥(360) ≈0.6 m."""
    p = P(cam_height_m=0.345, range_scale=1.0)
    assert abs(horizon_row(W, H, p) - 80.0) < 1.0, horizon_row(W, H, p)
    assert abs(row_for_range(3.0, p) - 140.0) < 1.0
    assert abs(row_to_range(140.0, W, H, p) - 3.0) < 0.1
    assert abs(row_to_range(360.0, W, H, p) - 0.59) < 0.02


def test_row_range_horizon_and_above_inf():
    p = DEFAULT_PARAMS
    hz = horizon_row(W, H, p)
    assert row_to_range(hz, W, H, p) == INF
    assert row_to_range(hz - 10.0, W, H, p) == INF
    assert row_to_range(0.0, W, H, p) == INF
    assert math.isfinite(row_to_range(hz + 1.0, W, H, p))


def test_row_range_monotone_decreasing():
    p = DEFAULT_PARAMS
    hz = horizon_row(W, H, p)
    ys = [hz + 1.0 + i for i in range(int(H - hz - 1))]
    rs = [row_to_range(y, W, H, p) for y in ys]
    assert all(a > b for a, b in zip(rs, rs[1:]))


def test_row_range_scale_and_height_tilt_direction():
    """range_scale 은 곱. 높이↑ → 멀게(위험 쪽), 틸트↑ → 가깝게(보수 쪽)."""
    y = 200.0
    base = row_to_range(y, W, H, P(range_scale=1.0))
    assert near(row_to_range(y, W, H, P(range_scale=0.85)), base * 0.85, 1e-12)
    assert row_to_range(y, W, H, P(range_scale=1.0, cam_height_m=0.45)) > base
    assert row_to_range(y, W, H, P(range_scale=1.0, cam_tilt_deg=13.0)) < base


def test_row_range_invalid_is_none():
    for bad in (P(cam_height_m=0.0), P(cam_height_m=-0.3), P(cam_tilt_deg=math.nan),
                P(hfov_deg=0.0), P(range_scale=0.0), P(cam_height_m=True), P(cam_height_m='0.3')):
        assert row_to_range(200.0, W, H, bad) is None, bad
    p = dict(DEFAULT_PARAMS)
    del p["cam_tilt_deg"]
    assert row_to_range(200.0, W, H, p) is None
    assert row_to_range(200.0, 0, H, DEFAULT_PARAMS) is None
    assert row_to_range(math.inf, W, H, DEFAULT_PARAMS) is None


def test_bbox_touching_bottom_uses_bottom_row():
    """바닥에 닿은 bbox → 바닥 경계 행 거리(실제 접점은 화면 밖 = 더 가까움 → 이 값이 상한)."""
    p = DEFAULT_PARAMS
    bottom = row_to_range(float(H), W, H, p)
    assert bbox_range(H - 0.5, W, H, p) == bottom
    assert bbox_range(H - BOTTOM_TOUCH_PX, W, H, p) == bottom
    assert bbox_range(H + 40.0, W, H, p) == bottom             # 화면 밖 값도 경계로
    assert bbox_range(300.0, W, H, p) == row_to_range(300.0, W, H, p) > bottom


def test_bbox_ignore_bottom_band():
    """ignore_bottom_frac 0.2 → 경계 = 288행. 경계에 닿으면 288행 거리."""
    p = P(ignore_bottom_frac=0.2)
    edge = H * 0.8
    assert bbox_range(edge, W, H, p) == row_to_range(edge, W, H, p)
    assert bbox_range(H, W, H, p) == row_to_range(edge, W, H, p)
    assert bbox_range(250.0, W, H, p) == row_to_range(250.0, W, H, p)


# ------------------------------------------------------------------ (3) 카메라 → LiDAR 원점
def test_cam_to_lidar_identity_without_offset():
    for b, r in ((0.0, 2.0), (20.0, 3.0), (-31.0, 1.2)):
        b2, r2 = cam_to_lidar(b, r, 0.0, 0.0)
        assert near(b2, b, 1e-9) and near(r2, r, 1e-9)


def test_cam_to_lidar_offsets():
    b, r = cam_to_lidar(0.0, 2.0, 0.5, 0.0)              # 카메라가 LiDAR 보다 0.5 m 앞
    assert near(b, 0.0) and near(r, 2.5)
    b, r = cam_to_lidar(90.0, 1.0, 1.0, 0.0)             # 왼쪽 정횡 1 m, 카메라 1 m 앞
    assert near(b, 45.0, 1e-9) and near(r, math.sqrt(2.0), 1e-12)
    b, r = cam_to_lidar(0.0, 2.0, 0.0, 0.2)              # 카메라가 왼쪽 0.2 m → LiDAR 에선 좌 +
    assert b > 0.0 and near(r, math.hypot(2.0, 0.2))
    b, r = cam_to_lidar(10.0, INF, 0.5, 0.2)
    assert b == 10.0 and r == INF


# ------------------------------------------------------------------ (4) 검출 → 배열
def test_array_format_and_empty_detection_all_inf():
    p = DEFAULT_PARAMS
    arr = detections_to_array([], W, H, p)
    a0, inc, n = out_grid(p)
    assert (a0, inc, n) == (-31.5, 0.5, 127)
    assert arr[:2] == [-31.5, 0.5] and len(arr) == 2 + 127
    assert all(r == INF for r in arr[2:])
    assert a0 + (n - 1) * inc == 31.5                     # 화각(±31.95) 범위만, 0 을 지남
    assert all(type(v) is float for v in arr)


def test_array_invalid_params_or_size_empty():
    assert detections_to_array([(300, 340, 200)], W, H, P(cam_height_m=0.0)) == []
    assert detections_to_array([(300, 340, 200)], W, H, P(min_range_out_m=6.0)) == []
    assert detections_to_array([(300, 340, 200)], 0, H, DEFAULT_PARAMS) == []
    assert detections_to_array([(300, 340, 200)], W, None, DEFAULT_PARAMS) == []
    assert process_frame([(300, 340, 200)], W, H, P(out_angle_inc_deg=0.0)) == []


def test_array_left_bbox_lands_left_with_range():
    """화면 왼쪽 bbox → 양수 방위 칸만, 거리 = 바닥 행 거리, 그 밖은 inf."""
    p = DEFAULT_PARAMS
    y = row_for_range(2.0, p)
    arr = detections_to_array([(100.0, 160.0, y)], W, H, p)
    cs = cells(arr)
    assert cs and all(a > 0.0 for a, _ in cs)
    lo_b = pixel_to_bearing_tilt(160.0, y, W, H, 63.9, 11.0)                   # 바닥 행에서 정확식
    hi_b = pixel_to_bearing_tilt(100.0, y, W, H, 63.9, 11.0)
    assert all(lo_b - 0.5 <= a <= hi_b + 0.5 for a, _ in cs), (cs, lo_b, hi_b)
    assert all(near(r, 2.0 * 0.85, 1e-6) for _, r in cs)
    n_in = sum(1 for k in range(127) if lo_b <= -31.5 + 0.5 * k <= hi_b)
    assert len(cs) == n_in


def test_array_overlap_takes_min():
    p = DEFAULT_PARAMS
    near_y, far_y = row_for_range(1.5, p), row_for_range(3.0, p)
    arr = detections_to_array([(250.0, 330.0, far_y), (300.0, 380.0, near_y)], W, H, p)
    d = dict(cells(arr))
    b_mid = round(pixel_to_bearing(315.0, W, 63.9) * 2.0) / 2.0     # 둘 다 덮는 칸
    assert near(d[b_mid], 1.5 * 0.85, 1e-6)
    b_far_only = round(pixel_to_bearing(260.0, W, 63.9) * 2.0) / 2.0
    assert near(d[b_far_only], 3.0 * 0.85, 1e-6)


def test_array_min_sector_widens_narrow():
    """폭 2 px(≈0.2°) bbox → min_sector_deg 3° 로 넓혀 7칸(0.5° 간격)."""
    p = DEFAULT_PARAMS
    arr = detections_to_array([(319.0, 321.0, row_for_range(2.5, p))], W, H, p)
    cs = cells(arr)
    span = cs[-1][0] - cs[0][0]
    assert 2.5 <= span <= 3.0 + 1e-9 and len(cs) >= 6, cs
    assert any(near(a, 0.0) for a, _ in cs)
    arr0 = detections_to_array([(319.0, 321.0, row_for_range(2.5, p))], W, H, P(min_sector_deg=0.0))
    assert 1 <= len(cells(arr0)) <= 2                        # 넓히지 않아도 가장 가까운 칸 하나는 낸다


def test_array_max_range_inf_and_min_range_floor():
    p = DEFAULT_PARAMS
    far = detections_to_array([(300.0, 340.0, row_for_range(5.0 / 0.85 + 0.5, p))], W, H, p)
    assert not cells(far)                                     # 5 m 초과 → inf
    ok = detections_to_array([(300.0, 340.0, row_for_range(4.0 / 0.85, p))], W, H, p)
    assert cells(ok) and all(near(r, 4.0, 1e-6) for _, r in cells(ok))
    pz = P(cam_tilt_deg=40.0)                                 # 바닥이 아주 가깝게 나오는 설정
    close = detections_to_array([(300.0, 340.0, float(H))], W, H, pz)
    assert row_to_range(float(H), W, H, pz) < 0.5
    assert cells(close) and all(r == 0.5 for _, r in cells(close))


def test_array_above_horizon_and_invalid_bbox_skipped():
    p = DEFAULT_PARAMS
    hz = horizon_row(W, H, p)
    bad = [(300.0, 340.0, hz - 5.0), (340.0, 300.0, 300.0), (math.nan, 340.0, 300.0),
           (300.0, 340.0, None), (300.0, 340.0), 'x']
    arr = detections_to_array(bad, W, H, P(max_bboxes=10))
    assert len(arr) == 129 and not cells(arr)


def test_array_lidar_offset_applied():
    """카메라가 LiDAR 보다 0.5 m 앞 → 같은 bbox 의 거리가 0.5 m 늘어난다(정면)."""
    p0 = DEFAULT_PARAMS
    y = row_for_range(2.0, p0)
    r0 = min(r for _, r in cells(detections_to_array([(310.0, 330.0, y)], W, H, p0)))
    p1 = P(cam_x_from_lidar_m=0.5)
    r1 = min(r for _, r in cells(detections_to_array([(310.0, 330.0, y)], W, H, p1)))
    assert near(r1 - r0, 0.5, 1e-3), (r0, r1)


def test_array_edge_bbox_clipped_to_grid():
    """화면 끝까지 닿는 bbox → 격자 끝 칸까지만(인덱스 0 = 우현 끝)."""
    p = DEFAULT_PARAMS
    arr = detections_to_array([(600.0, 640.0, row_for_range(2.0, p))], W, H, p)
    assert math.isfinite(arr[2]) and arr[2 + 126] == INF


# ------------------------------------------------------------------ (5) 확인-N 창
def test_confirm_3_of_5():
    cw = ConfirmWindow(3, 5, 10.0)
    assert cw.update([5.0]) == [False]
    assert cw.update([6.0]) == [False]
    assert cw.update([4.0]) == [True]                         # 3 프레임째
    assert cw.update([]) == []
    assert cw.update([5.0]) == [True]                         # 최근 5 중 4
    for _ in range(3):
        cw.update([])
    assert cw.update([5.0]) == [False]                        # 창에서 밀려남(최근 5 중 2)


def test_confirm_bearing_matching():
    cw = ConfirmWindow(3, 5, 10.0)
    cw.update([0.0])
    cw.update([0.0])
    assert cw.update([25.0, 9.0]) == [False, True]            # 25° 는 다른 물체
    assert cw.update([None, math.nan]) == [False, False]


def test_confirm_reset_and_off():
    cw = ConfirmWindow(3, 5, 10.0)
    cw.update([0.0])
    cw.update([0.0])
    cw.reset()
    assert cw.update([0.0]) == [False]
    assert ConfirmWindow(1, 1, 10.0).update([3.0, -20.0]) == [True, True]


def test_confirm_param_guard():
    assert check_confirm(3, 5, 10.0) == []
    for bad in ((3.0, 5, 10.0), (3, 5.0, 10.0), (True, 5, 10.0), (0, 5, 10.0), (6, 5, 10.0),
                (3, 31, 10.0), (3, 5, 0.0), (3, 5, math.nan), (3, 5, 91.0)):
        assert check_confirm(*bad), bad
        with pytest.raises(ValueError):
            ConfirmWindow(*bad)


def test_process_frame_confirm_then_array():
    """확인 전 프레임은 전부 inf(살아 있음), 3 프레임째부터 낸다. confirm None 이면 바로."""
    p = DEFAULT_PARAMS
    box = (300.0, 340.0, row_for_range(2.0, p))
    cw = ConfirmWindow(3, 5, 10.0)
    a1 = process_frame([box], W, H, p, cw)
    a2 = process_frame([box], W, H, p, cw)
    a3 = process_frame([box], W, H, p, cw)
    assert len(a1) == 129 and not cells(a1) and not cells(a2)
    assert cells(a3)
    assert cells(process_frame([box], W, H, p)) == cells(a3)
    assert near(bbox_center_bearing(box, W, H, p), 0.0)
    assert near(bbox_center_bearing((100.0, 140.0, 300.0), W, H, p),
                pixel_to_bearing_tilt(120.0, 300.0, W, H, 63.9, 11.0))
    assert bbox_center_bearing((None, 1, 2), W, H, p) is None
    assert bbox_center_bearing((0.0, 640.0, 300.0), W, H, p) is None           # 각폭 초과


# ------------------------------------------------------------------ (6) 거짓 정지 대책 — 각폭·개수
def test_wide_bbox_dropped_full_frame_bottom():
    """화각 전체를 덮는 바닥 닿음 bbox(석양·반사) → 버림 → 그 방향 전부 inf(살아 있는 배열)."""
    p = DEFAULT_PARAMS
    for f in (detections_to_array, process_frame):
        arr = f([(0.0, 640.0, float(H))], W, H, p)
        assert len(arr) == 129 and not cells(arr), f
    ok = detections_to_array([(250.0, 390.0, float(H))], W, H, p)        # ≈15° → 남는다
    assert cells(ok)


def test_wide_bbox_threshold_and_other_box_survives():
    p = DEFAULT_PARAMS
    y = row_for_range(2.0, p)
    xl, xr = 320.0 - 140.0, 320.0 + 140.0
    width = pixel_to_bearing_tilt(xl, y, W, H, 63.9, 11.0) - pixel_to_bearing_tilt(xr, y, W, H, 63.9, 11.0)
    assert 25.0 < width < 32.0
    assert not cells(detections_to_array([(xl, xr, y)], W, H, p))
    assert cells(detections_to_array([(xl, xr, y)], W, H, P(max_bbox_width_deg=width + 0.1)))
    arr = detections_to_array([(0.0, 640.0, float(H)), (100.0, 140.0, y)], W, H, p)
    assert cells(arr) and all(a > 10.0 for a, _ in cells(arr))           # 넓은 것만 버리고 왼쪽 것은 낸다


def test_too_many_bboxes_frame_invalid_empty():
    """bbox 수 > max_bboxes(4) → 빈 배열(프레임 이상, 병합 안 함). 4 개는 정상."""
    p = DEFAULT_PARAMS
    y = row_for_range(2.5, p)
    four = [(40.0 + 120.0 * i, 60.0 + 120.0 * i, y) for i in range(4)]
    five = four + [(560.0, 580.0, y)]
    assert len(cells(detections_to_array(four, W, H, p))) > 0
    assert detections_to_array(five, W, H, p) == []
    assert process_frame(five, W, H, p) == []
    assert len(process_frame(five, W, H, P(max_bboxes=5))) == 129


def test_too_many_bboxes_confirm_window_flows_empty():
    """개수 초과 프레임은 창에 '본 것 없음' 으로 들어간다 — 그 뒤 2 프레임만으로는 확인 안 됨."""
    p = DEFAULT_PARAMS
    box = (300.0, 340.0, row_for_range(2.0, p))
    cw = ConfirmWindow(3, 5, 10.0)
    process_frame([box], W, H, p, cw)
    process_frame([box], W, H, p, cw)
    assert process_frame([box] * 5, W, H, p, cw) == []
    assert len(cw._hist) == 3 and cw._hist[-1] == []
    assert cells(process_frame([box], W, H, p, cw))                       # 최근 4 중 3 → 확인
    cw2 = ConfirmWindow(3, 5, 10.0)
    assert process_frame([box], W, H, P(cam_height_m=0.0), cw2) == [] and len(cw2._hist) == 0   # 파라미터 무효 = 안 건드림


def test_max_bboxes_and_width_param_guard():
    assert check_params(P(max_bboxes=1)) == [] and check_params(P(max_bboxes=50)) == []
    for bad in (4.0, True, 0, 51, '4', None):
        assert check_params(P(max_bboxes=bad)), bad
    for bad in (0.0, -1.0, 181.0, math.nan):
        assert check_params(P(max_bbox_width_deg=bad)), bad
    p = dict(DEFAULT_PARAMS)
    del p["max_bboxes"]
    assert check_params(p)
    assert detections_to_array([], W, H, P(max_bboxes=4.0)) == []

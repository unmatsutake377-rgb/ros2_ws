#!/usr/bin/env python3
"""obstacle_fusion_logic 순수 로직 테스트 (계약서_카메라장애물_20261111 보완 1 U2b). ROS·OpenCV 없이 돈다.

실행: tools/run_tests.sh color_shape_detector
"""

import ast
import math
import os
import re
import sys
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from color_shape_detector.obstacle_cam_logic import (  # noqa: E402
    DEFAULT_PARAMS, ConfirmWindow, cam_to_lidar, pixel_to_bearing_tilt, process_frame)
from color_shape_detector.obstacle_fusion_logic import (  # noqa: E402
    FUSION_DEFAULTS, LIDAR_FORWARD_DEG, boom_wall_segment, check_fusion_params, fuse_bbox_range,
    fused_detections_to_array, lidar_clusters, ray_segment_range)

W, H = 640, 360
INF = math.inf
HERE = os.path.dirname(os.path.abspath(__file__))
MOD = os.path.join(HERE, '..', 'color_shape_detector', 'obstacle_fusion_logic.py')
SD_FILE = os.path.join(HERE, '..', '..', 'ship_direction', 'ship_direction', 'ship_direction.py')

INC = 0.25          # 1번배 80BF 각분해능 근처


def P(**kw):
    p = dict(DEFAULT_PARAMS)
    p.update(FUSION_DEFAULTS)
    p.update(kw)
    return p


PF = P()


def near(a, b, tol=1e-9):
    return abs(a - b) <= tol


def make_scan(objs, inc=INC, a_min=-180.0):
    """LiDAR 원 각도(deg, 80 = 정면) 스캔. objs = [(lo_deg, hi_deg, r)] → 그 구간 점만 r, 나머지 inf."""
    n = int(round(360.0 / inc))
    ranges = [INF] * n
    for lo, hi, r in objs:
        for i in range(n):
            a = a_min + i * inc
            if lo - 1e-9 <= a <= hi + 1e-9:
                ranges[i] = r
    return types.SimpleNamespace(ranges=ranges, angle_min=math.radians(a_min),
                                 angle_increment=math.radians(inc))


def clusters_of(objs, p=PF, **kw):
    sc = make_scan(objs, **kw)
    return lidar_clusters(sc.ranges, sc.angle_min, sc.angle_increment, p)


def row_for_range(z, p=PF):
    fy = (W / 2.0) / math.tan(math.radians(p["hfov_deg"]) / 2.0)
    delta = math.atan(p["cam_height_m"] / z)
    return H / 2.0 + fy * math.tan(delta - math.radians(p["cam_tilt_deg"]))


def cells(arr):
    a0, inc = arr[0], arr[1]
    return [(a0 + k * inc, r) for k, r in enumerate(arr[2:]) if math.isfinite(r)]


def lidar_obj_for_cam(b_lo, b_hi, r, p=PF):
    """카메라 방위 구간을 LiDAR 원 각도 구간으로(오프셋 0 가정): a = rel + 80."""
    return (b_lo + LIDAR_FORWARD_DEG, b_hi + LIDAR_FORWARD_DEG, r)


# ------------------------------------------------------------------ 기본값·모듈
def test_defaults_and_param_check():
    assert FUSION_DEFAULTS == {"fusion_margin_deg": 3.0, "cluster_gap_deg": 2.0, "cluster_gap_m": 0.3,
                               "cluster_min_points": 2, "fusion_fallback_row": True,
                               "boom_wall_enabled": True, "boom_max_span_m": 4.0,
                               "fusion_behind_tol_deg": 3.0}
    assert type(FUSION_DEFAULTS["cluster_min_points"]) is int
    assert type(FUSION_DEFAULTS["fusion_fallback_row"]) is bool
    assert type(FUSION_DEFAULTS["boom_wall_enabled"]) is bool
    assert not set(FUSION_DEFAULTS) & set(DEFAULT_PARAMS)          # 공통 키와 안 겹친다
    assert check_fusion_params(PF) == []
    for bad in (dict(cluster_min_points=2.0), dict(cluster_min_points=True), dict(cluster_min_points=0),
                dict(fusion_fallback_row=1), dict(fusion_fallback_row='true'), dict(fusion_margin_deg=-1.0),
                dict(fusion_margin_deg=math.nan), dict(cluster_gap_deg=0.0), dict(cluster_gap_m=0.0),
                dict(cluster_gap_m=math.inf), dict(boom_wall_enabled=1), dict(boom_wall_enabled='true'),
                dict(boom_max_span_m=0.0), dict(boom_max_span_m=math.nan), dict(boom_max_span_m=True)):
        assert check_fusion_params(P(**bad)), bad
    p = dict(PF)
    del p["cluster_gap_m"]
    assert check_fusion_params(p)


def test_defaults_have_measure_marker():
    """FUSION_DEFAULTS 각 줄에 ⚠️ 실측 필요 주석(CLAUDE.md §1-4)."""
    src = open(MOD, encoding='utf-8').read()
    body = src[src.index('FUSION_DEFAULTS = {'):]
    body = body[:body.index('}')]
    for k in FUSION_DEFAULTS:
        line = next(ln for ln in body.splitlines() if f'"{k}"' in ln)
        assert '⚠️ 실측 필요' in line, line


def test_no_ros_or_cv2_import():
    with open(MOD, encoding='utf-8') as f:
        tree = ast.parse(f.read())
    mods = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            mods.add(node.module)
    assert mods <= {'math', 'color_shape_detector.obstacle_cam_logic'}, mods


def test_lidar_forward_matches_ship_direction():
    """상수 공유 모듈화 금지(§1-8) → 값 일치를 테스트로. ship_direction.py LIDAR_FORWARD_DEG = 80.0."""
    if not os.path.exists(SD_FILE):
        pytest.skip('ship_direction 소스 없음')
    src = open(SD_FILE, encoding='utf-8').read()
    m = re.search(r'^LIDAR_FORWARD_DEG\s*=\s*([0-9.]+)', src, re.M)
    assert m and float(m.group(1)) == LIDAR_FORWARD_DEG == 80.0


# ------------------------------------------------------------------ (1) 군집
def test_frame_sign_lidar100_is_left20():
    """LiDAR 원 각도 100° → 상대방위 +20(좌), 60° → −20(우). 정면 80° → 0."""
    cs = clusters_of([(99.5, 100.5, 2.0), (59.5, 60.5, 3.0), (79.75, 80.25, 4.0)])
    assert len(cs) == 3
    by_r = {c[2]: c for c in cs}
    left, right, front = by_r[2.0], by_r[3.0], by_r[4.0]
    assert near(left[0], 20.5, 1e-6) and near(left[1], 19.5, 1e-6) and left[3] == 5
    assert near(right[0], -19.5, 1e-6) and near(right[1], -20.5, 1e-6)
    assert near(front[0], 0.25, 1e-6) and near(front[1], -0.25, 1e-6)
    for c in cs:
        assert c[0] >= c[1]                                    # left = 최대 rel


def test_two_adjacent_objects_not_merged_by_range():
    """각으로 붙어 있어도 거리 차 > cluster_gap_m 이면 둘. ★ 참고 코드 버그(닫은 뒤 이어 붙임) 고정:
    두 번째 군집의 점 수가 자기 점만이어야 한다."""
    cs = clusters_of([(70.0, 71.0, 2.0), (71.25, 72.25, 3.5)])
    assert len(cs) == 2, cs
    a, b = cs
    assert (a[2], a[3]) == (2.0, 5) and (b[2], b[3]) == (3.5, 5)
    assert a[0] < b[1]                                         # 겹치지 않음


def test_three_objects_by_angle_gap_each_own_cluster():
    """같은 거리라도 방위 틈 > cluster_gap_deg(2°) 면 셋 — 합쳐지면 n_points 가 15 가 된다."""
    cs = clusters_of([(70.0, 71.0, 2.0), (74.0, 75.0, 2.0), (78.0, 79.0, 2.0)])
    assert [c[3] for c in cs] == [5, 5, 5], cs
    assert [round(c[1] + LIDAR_FORWARD_DEG, 6) for c in cs] == [70.0, 74.0, 78.0]


def test_gradual_range_change_stays_one_cluster():
    """점마다 거리 변화가 작으면(비스듬한 면) 끝 거리 차가 커도 한 군집 — 인접 점 기준."""
    sc = make_scan([])
    i0 = int(round((70.0 + 180.0) / INC))
    for k in range(20):
        sc.ranges[i0 + k] = 2.0 + 0.1 * k
    cs = lidar_clusters(sc.ranges, sc.angle_min, sc.angle_increment, PF)
    assert len(cs) == 1 and cs[0][3] == 20 and near(cs[0][2], 2.0)


def test_invalid_points_skipped():
    """inf·nan·0·음수·max_range 초과·숫자 아님은 건너뛴다. 한 점 구멍(0.5°)은 이어지고, 큰 구멍(>2°)은 끊긴다."""
    sc = make_scan([(70.0, 75.0, 2.0)])
    i0 = int(round((70.0 + 180.0) / INC))
    sc.ranges[i0 + 3] = math.nan
    sc.ranges[i0 + 4] = 0.0
    sc.ranges[i0 + 5] = -1.0
    cs = lidar_clusters(sc.ranges, sc.angle_min, sc.angle_increment, PF)
    assert len(cs) == 1 and cs[0][3] == 21 - 3
    for k in range(8, 17):                                     # 9 점(2.25°) 구멍 → 틈 2.5° > 2°
        sc.ranges[i0 + k] = INF
    cs = lidar_clusters(sc.ranges, sc.angle_min, sc.angle_increment, PF)
    assert len(cs) == 2, cs
    sc2 = make_scan([(70.0, 71.0, 6.0), (90.0, 91.0, 2.0)])     # 6 m > max_range 5 → 제외
    sc2.ranges[0] = 'x'
    sc2.ranges[1] = None
    cs2 = lidar_clusters(sc2.ranges, sc2.angle_min, sc2.angle_increment, PF)
    assert len(cs2) == 1 and cs2[0][2] == 2.0
    assert lidar_clusters([INF, math.nan, 0.0], 0.0, math.radians(INC), PF) == []


def test_min_points():
    sc = make_scan([])
    sc.ranges[int(round((80.0 + 180.0) / INC))] = 2.0          # 외톨이 한 점
    assert lidar_clusters(sc.ranges, sc.angle_min, sc.angle_increment, PF) == []
    one = lidar_clusters(sc.ranges, sc.angle_min, sc.angle_increment, P(cluster_min_points=1))
    assert len(one) == 1 and one[0][3] == 1 and near(one[0][0], 0.0, 1e-6)
    assert len(clusters_of([(80.0, 80.25, 2.0)])) == 1          # 2 점이면 남는다


def test_wrap_seam_splits_allowed():
    """LiDAR 255~265° = rel 175~185 → 바로 뒤 이음매(±180)에서 둘로 갈린다(허용, 문서화). rel 은 [−180, 180)."""
    cs = clusters_of([(255.0, 265.0, 2.0)], a_min=0.0)        # 0~359.75° 스캔
    assert len(cs) == 2, cs
    for c in cs:
        assert -180.0 <= c[1] <= c[0] < 180.0
    assert sum(c[3] for c in cs) == 41
    assert any(near(c[1], 175.0, 1e-6) for c in cs) and any(near(c[1], -180.0, 1e-6) for c in cs)


def test_invalid_header_or_params_none():
    sc = make_scan([(70.0, 71.0, 2.0)])
    for a_min, inc in ((sc.angle_min, 0.0), (sc.angle_min, -0.01), (math.nan, sc.angle_increment),
                       (sc.angle_min, math.inf), (None, sc.angle_increment)):
        assert lidar_clusters(sc.ranges, a_min, inc, PF) is None
    assert lidar_clusters(sc.ranges, sc.angle_min, sc.angle_increment, P(cluster_min_points=2.0)) is None
    assert lidar_clusters(sc.ranges, sc.angle_min, sc.angle_increment, P(max_range_m=0.0)) is None
    assert lidar_clusters(None, sc.angle_min, sc.angle_increment, PF) is None
    assert lidar_clusters([], sc.angle_min, sc.angle_increment, PF) == []


# ------------------------------------------------------------------ (2) 짝짓기
def test_fuse_margin():
    cl = [(12.0, 10.0, 2.5, 9)]
    assert fuse_bbox_range(7.5, 5.0, cl, PF) == (2.5, 'lidar')          # 틈 2.5° ≤ 3°
    assert fuse_bbox_range(6.5, 5.0, cl, PF) is None                   # 틈 3.5° > 3°
    assert fuse_bbox_range(6.5, 5.0, cl, P(fusion_margin_deg=4.0)) == (2.5, 'lidar')
    assert fuse_bbox_range(5.0, 7.5, cl, PF) == (2.5, 'lidar')          # 순서 바뀌어도
    assert fuse_bbox_range(11.0, 11.0, cl, P(fusion_margin_deg=0.0)) == (2.5, 'lidar')   # 안쪽


def test_fuse_nearest_of_overlapping():
    cl = [(5.0, -5.0, 3.0, 40), (2.0, 0.0, 1.8, 9), (30.0, 25.0, 0.9, 9)]   # 세 번째는 안 겹침
    assert fuse_bbox_range(3.0, -3.0, cl, PF) == (1.8, 'lidar')
    assert fuse_bbox_range(3.0, -3.0, [], PF) is None
    assert fuse_bbox_range(3.0, -3.0, None, PF) is None
    assert fuse_bbox_range(math.nan, -3.0, cl, PF) is None
    assert fuse_bbox_range(3.0, -3.0, [(2.0, 0.0, math.nan, 9), 'x', (1.0,)], PF) is None


def test_fuse_parallax_camera_offset():
    """카메라가 LiDAR 왼쪽 0.3 m: 카메라 정면 3 m 물체는 LiDAR 에선 +5.7°. 보정 없이는 여유 3° 로 못 짝짓는다."""
    p = P(cam_y_from_lidar_m=0.3)
    rel = math.degrees(math.atan2(0.3, 3.0))
    r = math.hypot(3.0, 0.3)
    cl = [(rel + 0.5, rel - 0.5, r, 5)]
    assert fuse_bbox_range(1.0, -1.0, cl, PF) is None                  # 오프셋 모름 → 4.7° 틈
    assert fuse_bbox_range(1.0, -1.0, cl, p) == (r, 'lidar')


# ------------------------------------------------------------------ (3) 프레임 → 배열
def test_output_format_and_no_detection_all_inf():
    arr = fused_detections_to_array([], W, H, make_scan([(70.0, 71.0, 2.0)]), PF)
    assert arr[:2] == [-31.5, 0.5] and len(arr) == 129
    assert all(r == INF for r in arr[2:]) and all(type(v) is float for v in arr)


def test_fused_lidar_range_beats_row_range():
    """행은 2.0×0.85 = 1.7 m 라 하지만 LiDAR 군집이 2.4 m → 2.4 m(range_scale·원점 변환 없음). 방위 칸은 같다.
    (2.4 m 는 bbox 바닥보다 1.6° 위 — 뒤쪽 짝 거부 3° 안. 3.2 m(3.7°)면 거부된다 → test_behind_* )"""
    y = row_for_range(2.0)
    box = (300.0, 340.0, y)
    b_l = pixel_to_bearing_tilt(300.0, y, W, H, 63.9, 11.0)
    b_r = pixel_to_bearing_tilt(340.0, y, W, H, 63.9, 11.0)
    sc = make_scan([lidar_obj_for_cam(b_r, b_l, 2.4)])
    fused = fused_detections_to_array([box], W, H, sc, PF)
    row = process_frame([box], W, H, PF)
    assert cells(row) and all(near(r, 1.7, 1e-6) for _, r in cells(row))
    assert cells(fused) and all(r == 2.4 for _, r in cells(fused))
    assert [a for a, _ in cells(fused)] == [a for a, _ in cells(row)]
    cl = clusters_of([lidar_obj_for_cam(b_r, b_l, 2.4)])
    assert fused_detections_to_array([box], W, H, cl, PF) == fused        # 군집 목록을 넘겨도 같다


def test_no_match_fallback_row_or_drop():
    """짝 없음(LiDAR 군집이 반대쪽) → fallback True: 행 기반(= process_frame), False: 그 bbox 버림(전부 inf)."""
    box = (100.0, 160.0, row_for_range(2.0))                   # 화면 왼쪽
    sc = make_scan([(55.0, 60.0, 2.0)])                         # rel −25~−20 (우현)
    on = fused_detections_to_array([box], W, H, sc, PF)
    assert on == process_frame([box], W, H, PF) and cells(on)
    off = fused_detections_to_array([box], W, H, sc, P(fusion_fallback_row=False))
    assert len(off) == 129 and not cells(off)


def test_scan_missing_fallback_or_silent():
    """스캔 없음(None·머리 무효): fallback True → 행 기반 전부, False → [] (거리를 모른다)."""
    box = (300.0, 340.0, row_for_range(2.0))
    assert fused_detections_to_array([box], W, H, None, PF) == process_frame([box], W, H, PF)
    assert fused_detections_to_array([box], W, H, None, P(fusion_fallback_row=False)) == []
    bad = types.SimpleNamespace(ranges=[2.0] * 10, angle_min=0.0, angle_increment=0.0)
    assert fused_detections_to_array([box], W, H, bad, P(fusion_fallback_row=False)) == []
    assert fused_detections_to_array([box], W, H, bad, PF) == process_frame([box], W, H, PF)


def test_mixed_fused_and_fallback_overlap_min():
    """bbox 둘: 왼쪽은 LiDAR 짝(2.4 m), 오른쪽은 짝 없음 → 행 기반. 각자 제 칸, 겹치는 칸은 min."""
    y_l, y_r = row_for_range(2.0), row_for_range(3.0)
    bl = (200.0, 260.0, y_l)
    br = (380.0, 440.0, y_r)
    b1 = pixel_to_bearing_tilt(200.0, y_l, W, H, 63.9, 11.0)
    b2 = pixel_to_bearing_tilt(260.0, y_l, W, H, 63.9, 11.0)
    arr = fused_detections_to_array([bl, br], W, H, make_scan([lidar_obj_for_cam(b2, b1, 2.4)]), PF)
    d = cells(arr)
    assert {r for a, r in d if a > 0} == {2.4}
    assert all(near(r, 3.0 * 0.85, 1e-6) for a, r in d if a < 0)


def test_fused_offset_bearing_uses_lidar_range():
    """카메라가 LiDAR 보다 0.5 m 앞: 거리는 LiDAR 값 그대로, 방위는 그 거리에 놓인 점의 LiDAR 원점 방위."""
    p = P(cam_x_from_lidar_m=0.5, min_sector_deg=0.0)
    y = row_for_range(2.0, p)
    box = (180.0, 200.0, y)
    bc = pixel_to_bearing_tilt(190.0, y, W, H, 63.9, 11.0)       # 카메라 방위 ≈ +14°
    R = 3.0
    # 카메라 광선 위에서 LiDAR 거리 R 인 점
    t = math.radians(bc)
    cu = 0.5 * math.cos(t)
    s = -cu + math.sqrt(cu * cu - 0.25 + R * R)
    want_b, want_r = cam_to_lidar(bc, s, 0.5, 0.0)
    assert near(want_r, R, 1e-9) and want_b < bc
    cl = [(want_b + 0.5, want_b - 0.5, R, 5)]
    arr = fused_detections_to_array([box], W, H, cl, p)
    d = cells(arr)
    assert d and all(r == R for _, r in d)
    mid = (d[0][0] + d[-1][0]) / 2.0
    assert abs(mid - want_b) <= 0.5, (mid, want_b, bc)


def test_fused_wide_bbox_and_count_filters():
    """퓨전 경로에도 거짓 정지 대책: 화각 전체 바닥 bbox(+ 정면 LiDAR 군집) → 버림 → 전부 inf / 5 개 → []."""
    sc = make_scan([(70.0, 90.0, 1.5)])
    arr = fused_detections_to_array([(0.0, 640.0, float(H))], W, H, sc, PF)
    assert len(arr) == 129 and not cells(arr)
    y = row_for_range(2.5)
    five = [(40.0 + 120.0 * i, 60.0 + 120.0 * i, y) for i in range(5)]
    assert fused_detections_to_array(five, W, H, sc, PF) == []
    assert len(fused_detections_to_array(five[:4], W, H, sc, PF)) == 129


def test_fused_confirm_same_path_as_process_frame():
    """확인-N 은 U2 ConfirmWindow 그대로: 1·2 프레임 inf, 3 프레임째 낸다. 개수 초과 프레임은 '본 것 없음'."""
    y = row_for_range(2.0)
    box = (300.0, 340.0, y)
    sc = make_scan([(78.0, 82.0, 2.6)])
    cw = ConfirmWindow(3, 5, 10.0)
    a = [fused_detections_to_array([box], W, H, sc, PF, cw) for _ in range(3)]
    assert not cells(a[0]) and not cells(a[1]) and cells(a[2])
    assert all(r == 2.6 for _, r in cells(a[2]))
    cw2 = ConfirmWindow(3, 5, 10.0)
    fused_detections_to_array([box], W, H, sc, PF, cw2)
    assert fused_detections_to_array([box] * 5, W, H, sc, PF, cw2) == []
    assert list(cw2._hist)[-1] == []


def test_fused_invalid_params_empty_window_untouched():
    box = (300.0, 340.0, row_for_range(2.0))
    sc = make_scan([(78.0, 82.0, 2.6)])
    for bad in (P(cluster_min_points=2.0), P(fusion_fallback_row='yes'), P(fusion_margin_deg=math.nan),
                P(cam_height_m=0.0), P(max_bboxes=4.0)):
        cw = ConfirmWindow(3, 5, 10.0)
        assert fused_detections_to_array([box], W, H, sc, bad, cw) == []
        assert len(cw._hist) == 0
    assert fused_detections_to_array([box], 0, H, sc, PF) == []
    p = dict(PF)
    del p["fusion_fallback_row"]
    assert fused_detections_to_array([box], W, H, sc, p) == []


# ------------------------------------------------------------------ U2c 붐 벽
def _wall_box(D, x_l=220.0, x_r=420.0, p=PF):
    """bbox (x_l, x_r, y) + 그 양 끝 방위에 부표 군집 둘 — 둘 다 정면 수직 벽 x = D 위(대표점 = (D, D·tan b))."""
    y = row_for_range(2.0, p)
    b_l = pixel_to_bearing_tilt(x_l, y, W, H, 63.9, 11.0)
    b_r = pixel_to_bearing_tilt(x_r, y, W, H, 63.9, 11.0)
    objs = [lidar_obj_for_cam(b - 0.5, b + 0.5, D / math.cos(math.radians(b))) for b in (b_l, b_r)]
    return (x_l, x_r, y), b_l, b_r, make_scan(objs)


def test_ray_segment_range_basic():
    seg = ((2.0, 1.0), (2.0, -1.0))
    assert near(ray_segment_range(0.0, seg), 2.0)
    assert near(ray_segment_range(20.0, seg), 2.0 / math.cos(math.radians(20.0)))
    assert ray_segment_range(40.0, seg) is None                # 선분 끝(26.6°) 밖
    assert ray_segment_range(180.0, seg) is None               # 뒤쪽
    assert ray_segment_range(90.0, ((1.0, 1.0), (2.0, 1.0))) is None    # 평행
    assert near(ray_segment_range(0.0, seg, origin=(0.5, 0.0)), 1.5)


def test_boom_wall_perpendicular_cosine():
    """정면 수직 벽(두 부표 x = 2.0 m): bbox 구간 칸마다 거리 = D / cos(방위). 가장 가까운 군집 거리(기존 규칙)보다 짧다."""
    D = 2.0
    box, b_l, b_r, sc = _wall_box(D)
    assert b_l > 0 > b_r and b_l - b_r < PF["max_bbox_width_deg"]
    arr = fused_detections_to_array([box], W, H, sc, PF)
    d = cells(arr)
    assert d
    assert min(a for a, _ in d) >= b_r - 0.5 - 1e-9 and max(a for a, _ in d) <= b_l + 0.5 + 1e-9
    for a, r in d:
        assert near(r, D / math.cos(math.radians(a)), 1e-3), (a, r)
    r_near = min(D / math.cos(math.radians(b)) for b in (b_l, b_r))
    front = [r for a, r in d if abs(a) < 0.3]
    assert front and near(front[0], D, 1e-3) and front[0] < r_near - 0.02
    old = fused_detections_to_array([box], W, H, sc, P(boom_wall_enabled=False))
    assert all(near(r, r_near, 1e-9) for _, r in cells(old))   # 끄면 U2b 그대로(가장 가까운 군집)


def test_boom_wall_sign_oblique():
    """비스듬한 붐: 왼쪽(+) 부표 1.5 m, 오른쪽(−) 부표 3.0 m → 왼쪽 칸이 더 가깝고 오른쪽으로 갈수록 멀어진다."""
    y = row_for_range(2.0)
    b_l = pixel_to_bearing_tilt(240.0, y, W, H, 63.9, 11.0)
    b_r = pixel_to_bearing_tilt(400.0, y, W, H, 63.9, 11.0)
    sc = make_scan([lidar_obj_for_cam(b_l - 0.5, b_l + 0.5, 1.5), lidar_obj_for_cam(b_r - 0.5, b_r + 0.5, 3.0)])
    d = cells(fused_detections_to_array([(240.0, 400.0, y)], W, H, sc, PF))
    assert d
    left = [r for a, r in d if a > 2.0]
    right = [r for a, r in d if a < -2.0]
    assert left and right and max(left) < min(right)
    rs = [r for _, r in sorted(d)]                             # 방위 오름차순 = 오른쪽 → 왼쪽
    assert all(rs[i] >= rs[i + 1] - 1e-9 for i in range(len(rs) - 1))
    seg = boom_wall_segment(b_l, b_r, clusters_of([lidar_obj_for_cam(b_l - 0.5, b_l + 0.5, 1.5),
                                                   lidar_obj_for_cam(b_r - 0.5, b_r + 0.5, 3.0)]), PF)
    (x1, y1), (x2, y2) = seg
    assert y1 > 0 > y2 and near(math.hypot(x1, y1), 1.5) and near(math.hypot(x2, y2), 3.0)


def test_boom_wall_span_limit():
    """두 부표 거리 > boom_max_span_m → 벽 안 만듦 = 가장 가까운 군집 거리(기존 규칙)."""
    box, b_l, b_r, sc = _wall_box(2.0)
    span = 2.0 * (math.tan(math.radians(b_l)) - math.tan(math.radians(b_r)))
    assert 0.5 < span < 1.0
    cl = clusters_of([lidar_obj_for_cam(b - 0.5, b + 0.5, 2.0 / math.cos(math.radians(b))) for b in (b_l, b_r)])
    assert boom_wall_segment(b_l, b_r, cl, P(boom_max_span_m=span + 0.01)) is not None
    assert boom_wall_segment(b_l, b_r, cl, P(boom_max_span_m=span - 0.01)) is None
    lim = fused_detections_to_array([box], W, H, sc, P(boom_max_span_m=span - 0.01))
    assert lim == fused_detections_to_array([box], W, H, sc, P(boom_wall_enabled=False))


def test_boom_wall_single_cluster_unchanged():
    """겹치는 군집 1개 → 기존 규칙과 비트 단위 같음. 0개 → 행 기반 대체 그대로."""
    y = row_for_range(2.0)
    box = (300.0, 340.0, y)
    sc = make_scan([(78.0, 82.0, 2.6)])
    assert boom_wall_segment(5.0, -5.0, clusters_of([(78.0, 82.0, 2.6)]), PF) is None
    assert (fused_detections_to_array([box], W, H, sc, PF)
            == fused_detections_to_array([box], W, H, sc, P(boom_wall_enabled=False)))
    far = make_scan([(150.0, 152.0, 2.0)])
    assert fused_detections_to_array([box], W, H, far, PF) == process_frame([box], W, H, PF)


def test_boom_wall_picks_end_clusters_of_three():
    """겹치는 군집 셋(양 끝 부표 + 가운데 앞 작은 점): 양 끝 군집을 잇고, 가운데 군집은 벽에 안 쓴다."""
    D = 3.0
    box, b_l, b_r, _ = _wall_box(D)
    objs = [lidar_obj_for_cam(b - 0.5, b + 0.5, D / math.cos(math.radians(b))) for b in (b_l, b_r)]
    objs.append(lidar_obj_for_cam(-0.5, 0.5, 4.5))
    seg = boom_wall_segment(b_l, b_r, clusters_of(objs), PF)
    (x1, y1), (x2, y2) = seg
    assert near(x1, D, 0.01) and near(x2, D, 0.01) and y1 > 0 > y2


# ------------------------------------------------------------------ 뒤쪽 짝 거부 (11-11 검토)
def test_behind_cluster_rejected_uses_row():
    """재현: 붐 1.5 m(행), 같은 방위 4.5 m 에 뒷줄 흰 부표(LiDAR) → 예전엔 4.5 m 출력(위험 쪽).
    이제 짝을 버리고 행 기반(= process_frame) — fusion_fallback_row False 여도 행 기반."""
    y = row_for_range(1.5)
    box = (300.0, 340.0, y)
    sc = make_scan([(79.0, 81.0, 4.5)])
    row = process_frame([box], W, H, PF)
    assert cells(row)
    assert fused_detections_to_array([box], W, H, sc, PF) == row
    assert fused_detections_to_array([box], W, H, sc, P(fusion_fallback_row=False)) == row
    assert min(fused_detections_to_array([box], W, H, sc, PF)[2:]) < 1.5


def test_behind_check_threshold_and_off():
    """3.2 m 짝(바닥보다 3.7° 위)은 기본 3° 에서 거부, 4° 면 짝 그대로. 수평선 위 bbox 는 검사 안 함."""
    from color_shape_detector.obstacle_cam_logic import _bbox_cam
    from color_shape_detector.obstacle_fusion_logic import lidar_behind
    y = row_for_range(2.0)
    box = (300.0, 340.0, y)
    cam = _bbox_cam(box, W, H, PF)
    assert lidar_behind(cam, 3.2, W, H, PF)
    assert not lidar_behind(cam, 3.2, W, H, P(fusion_behind_tol_deg=4.0))
    assert not lidar_behind(cam, 2.4, W, H, PF)
    assert not lidar_behind(cam, 1.0, W, H, PF)                      # LiDAR 가 더 가까움 = 거부 안 함
    sc = make_scan([(79.0, 81.0, 3.2)])
    assert all(r == 3.2 for _, r in cells(fused_detections_to_array([box], W, H, sc, P(fusion_behind_tol_deg=4.0))))
    sky = _bbox_cam((300.0, 340.0, 20.0), W, H, PF)                 # 수평선 위
    assert not lidar_behind(sky, 4.9, W, H, PF)


def test_behind_rejected_before_boom_wall():
    """bbox 양 끝에 군집 둘이 있어도 둘 다 한참 뒤(4.5 m)면 붐 벽을 만들지 않고 행 기반."""
    D = 1.5
    box, b_l, b_r, _ = _wall_box(D)
    objs = [lidar_obj_for_cam(b - 0.5, b + 0.5, 4.5) for b in (b_l, b_r)]
    sc = make_scan(objs)
    assert fused_detections_to_array([box], W, H, sc, PF) == process_frame([box], W, H, PF)


def test_cam_yaw_offset_restores_pairing_and_cells():
    """카메라 4° 오른쪽 틀어짐: 정면 물체가 화면에선 +4°. 오프셋 −4 를 넣으면 정면 칸에 칠하고 좁은 여유각에서도 짝."""
    from color_shape_detector.vision_geom import fx_from_hfov
    y = row_for_range(2.5)
    x0 = 320.0 - fx_from_hfov(W, 63.9) * math.tan(math.radians(4.0))
    box = (x0 - 10.0, x0 + 10.0, y)
    sc = make_scan([(79.4, 80.6, 2.5)])                               # LiDAR: 정면 ±0.6°, 2.5 m
    p0 = P(fusion_margin_deg=1.0)
    p4 = P(fusion_margin_deg=1.0, cam_yaw_offset_deg=-4.0)
    a0 = [a for a, _ in cells(fused_detections_to_array([box], W, H, sc, p0))]
    d4 = cells(fused_detections_to_array([box], W, H, sc, p4))
    assert min(a0) > 1.0                                              # 보정 없음: 정면이 아닌 왼쪽 칸
    assert all(r == 2.5 for _, r in d4)                              # 보정: LiDAR 짝(2.5 m)
    assert min(a for a, _ in d4) < 0.0 < max(a for a, _ in d4)        # 정면을 덮는다

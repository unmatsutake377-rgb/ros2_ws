#!/usr/bin/env python3
"""perception_fusion 순수 로직 테스트 (계약서_인식융합_20261112 §4·§5·§8, §10 결정). ROS 없이 돈다.

    tools/run_tests.sh perception_fusion

지키는 것
  · LiDAR: 상대 = a − 80(좌 +), inf/nan/≤range_min 무효, 정면 ±90° 만, 0.250°/0.223° 격자 모두
  · 군집: 간격 2°/0.3 m, 최소 2점, LiDAR 거리 = 최솟값
  · D455: 열 → 방위(오른쪽 열 = 음수, 좌 +), z→r = z/cos(방위), 0 = 모름, 유효 픽셀 하한, 0.5~6 m, 장착 오프셋
  · 연관: 방위 구간 ± fusion_margin 겹침, 한 덩어리는 한 번만
  · 거리: 일치 → 신뢰도 가중(w ∈ [0.3, 0.7]) src fused / 어긋남 → min + bit0 / 한쪽만 → 그 값
  · 신뢰도: 유효율 × (1 − 0.5·min(MAD/0.3,1) − 0.5·점프율), 반짝임 1프레임 → conf < 0.5
  · stale: LiDAR stale → 뎁스만 + bit1, 둘 다 stale → None(발행 안 함), 빈 목록 = 살아 있고 물체 없음
  · 색: 검출기 방위가 물체 구간 ±3° 안이면 그 색 id, 10000 센티넬은 무효
  · 출력: [VER=1, STRIDE=8, N, N×8], 길이 = 3 + 8N
  · 파라미터 가드: 무효 → 기본값 + 이슈 문자열
"""

import array
import math

import numpy as np
import pytest

from perception_fusion import fusion_logic as fl

P = dict(fl.DEFAULTS)


# ------------------------------------------------------------ 도우미

def _scan(rel_r, inc_deg=0.25, a_min_deg=-180.0, range_min=0.15):
    """{상대각(좌 +): 거리} → LaserScan 같은 (ranges, angle_min_rad, inc_rad, range_min)."""
    n = int(round(360.0 / inc_deg))
    ranges = [float('inf')] * n
    for rel, r in rel_r.items():
        a = fl.LIDAR_FORWARD_DEG + rel
        i = int(round((a - a_min_deg) / inc_deg)) % n
        ranges[i] = r
    return ranges, math.radians(a_min_deg), math.radians(inc_deg), range_min


def _buoy_scan(center_rel, r, half_width_deg=2.0, inc_deg=0.25):
    pts = {}
    k = -half_width_deg
    while k <= half_width_deg + 1e-9:
        pts[center_rel + k] = r
        k += inc_deg
    return _scan(pts, inc_deg=inc_deg)


def _lidar_track(frames, t0=0.0, dt=0.1, center_rel=0.0, half_width_deg=2.0):
    """frames = 거리 목록(None = 그 프레임 안 보임) → SourceTrack."""
    tr = fl.SourceTrack(P['conf_window_frames'])
    for k, r in enumerate(frames):
        if r is None:
            rel, rr = np.array([]), np.array([])
        else:
            ranges, amin, inc, rmin = _buoy_scan(center_rel, r, half_width_deg)
            rel, rr = fl.scan_points(ranges, amin, inc, rmin, P['lidar_half_fov_deg'])
        tr.push(t0 + k * dt, rel, rr)
    return tr


def _depth_track(frames, t0=0.0, dt=1.0 / 15.0, center_rel=0.0, half_width_deg=2.0):
    """뎁스 덩어리를 LiDAR 원점 (rel, r) 로 직접 넣는다(열 계산은 따로 테스트)."""
    tr = fl.SourceTrack(P['conf_window_frames'])
    for k, r in enumerate(frames):
        if r is None:
            rel, rr = np.array([]), np.array([])
        else:
            rel = np.arange(center_rel - half_width_deg, center_rel + half_width_deg + 1e-9, 0.2)
            rr = np.full(len(rel), float(r))
        tr.push(t0 + k * dt, rel, rr)
    return tr


def _empty_track():
    return fl.SourceTrack(P['conf_window_frames'])


# ------------------------------------------------------------ LiDAR 점

@pytest.mark.parametrize('inc_deg', [0.250, 0.223])
def test_scan_points_sign_and_grid(inc_deg):
    """a = 80 + rel: 왼쪽 30°(a=110) → +30, 오른쪽 30°(a=50) → −30. 0.250/0.223 격자 모두."""
    ranges, amin, inc, rmin = _scan({30.0: 4.0, -30.0: 5.0}, inc_deg=inc_deg)
    rel, r = fl.scan_points(ranges, amin, inc, rmin, 90.0)
    assert len(rel) == 2
    got = sorted(zip(rel.tolist(), r.tolist()))
    assert got[0][0] == pytest.approx(-30.0, abs=inc_deg) and got[0][1] == 5.0
    assert got[1][0] == pytest.approx(30.0, abs=inc_deg) and got[1][1] == 4.0


def test_scan_points_invalid_dropped():
    ranges, amin, inc, rmin = _scan({0.0: float('nan'), 5.0: 0.1, 10.0: 0.15, 15.0: 3.0})
    rel, r = fl.scan_points(ranges, amin, inc, rmin, 90.0)
    assert r.tolist() == [3.0], 'inf/nan/≤range_min 은 무효'


def test_scan_points_front_half_fov_only():
    ranges, amin, inc, rmin = _scan({89.0: 3.0, 95.0: 3.0, -120.0: 3.0, 179.0: 3.0})
    rel, r = fl.scan_points(ranges, amin, inc, rmin, 90.0)
    assert rel.tolist() == pytest.approx([89.0], abs=0.25)


# ------------------------------------------------------------ 군집

def test_cluster_gap_and_min_points():
    rel = np.array([-10.0, -9.75, -9.5, 0.0, 0.25, 0.5, 20.0])
    r = np.array([5.0, 4.9, 5.0, 3.0, 3.1, 3.0, 2.0])
    blobs = fl.cluster(rel, r, 2.0, 0.3, 2)
    assert len(blobs) == 2, '20° 한 점은 최소 2점 미달'
    b0, b1 = sorted(blobs, key=lambda b: b.bearing)
    assert (b0.lo, b0.hi) == (-10.0, -9.5) and b0.range_m == 4.9 and b0.bearing == pytest.approx(-9.75)
    assert (b1.lo, b1.hi) == (0.0, 0.5) and b1.range_m == 3.0


def test_cluster_splits_on_range_jump():
    rel = np.array([0.0, 0.25, 0.5, 0.75])
    r = np.array([3.0, 3.0, 4.0, 4.0])
    blobs = fl.cluster(rel, r, 2.0, 0.3, 2)
    assert [b.range_m for b in sorted(blobs, key=lambda b: b.bearing)] == [3.0, 4.0]


def test_cluster_median_for_depth():
    rel = np.array([0.0, 0.2, 0.4, 0.6, 0.8])
    r = np.array([4.0, 4.1, 4.2, 4.1, 4.0])
    (b,) = fl.cluster(rel, r, 2.0, 0.3, 2, reduce='median')
    assert b.range_m == pytest.approx(4.1)


def test_cluster_empty():
    assert fl.cluster(np.array([]), np.array([]), 2.0, 0.3, 2) == []


# ------------------------------------------------------------ D455 열 → 방위·거리

def test_column_bearings_sign():
    """오른쪽 열(u > cx) → 오른쪽 → 음수(좌 +). 가운데 = 0."""
    b = fl.column_bearings(5, fx=100.0, cx=2.0)
    assert b[2] == pytest.approx(0.0)
    assert b[4] == pytest.approx(-math.degrees(math.atan(2.0 / 100.0)))
    assert b[0] == pytest.approx(+math.degrees(math.atan(2.0 / 100.0)))


def test_depth_points_z_to_r_and_zero_unknown():
    h, w = 10, 5
    fx, cx = 2.0, 2.0                       # 열 4 → 방위 −45°
    img = np.zeros((h, w), dtype=np.uint16)
    img[:, 4] = 3000                         # z = 3 m
    img[:, 2] = 2000                         # 정면 z = 2 m
    p = dict(P, depth_row_lo_frac=0.0, depth_row_hi_frac=1.0)
    rel, r = fl.depth_points(img, fx, cx, p)
    got = dict(zip(np.round(rel, 3).tolist(), r.tolist()))
    assert got[0.0] == pytest.approx(2.0)
    assert got[-45.0] == pytest.approx(3.0 / math.cos(math.radians(45.0))), 'r = z / cos(방위)'
    assert len(got) == 2, '0 = 모름 → 열 없음'


def test_depth_points_range_limits_and_valid_fraction():
    h, w = 10, 3
    img = np.zeros((h, w), dtype=np.uint16)
    img[:, 0] = 300                          # 0.3 m < depth_min 0.5
    img[:, 1] = 7000                         # 7 m > depth_max 6
    img[:2, 2] = 2000                        # 유효 2/10 < 0.3
    p = dict(P, depth_row_lo_frac=0.0, depth_row_hi_frac=1.0)
    rel, r = fl.depth_points(img, 100.0, 1.0, p)
    assert len(r) == 0
    img[:3, 2] = 2000                        # 유효 3/10 = 0.3 → 통과, median 2.0
    rel, r = fl.depth_points(img, 100.0, 1.0, p)
    assert r.tolist() == pytest.approx([2.0], abs=1e-3)     # 열 2 = 방위 −0.57° → z/cos ≈ 2.0001


def test_depth_points_row_band():
    h, w = 10, 1
    img = np.zeros((h, w), dtype=np.uint16)
    img[8:, 0] = 1000                        # 아래(수면) 1 m — 띠 밖
    img[4:6, 0] = 4000                       # 가운데 띠 4 m
    p = dict(P, depth_row_lo_frac=0.4, depth_row_hi_frac=0.6, depth_min_valid_frac=0.5)
    rel, r = fl.depth_points(img, 100.0, 0.0, p)
    assert r.tolist() == pytest.approx([4.0])


def test_depth_points_mount_offset():
    """카메라가 LiDAR 앞 1 m, 요 오프셋 +10°: 카메라 정면 2 m → LiDAR 원점 (rel 10°보다 작게, r ≈ 2.98)."""
    img = np.full((4, 1), 2000, dtype=np.uint16)
    p = dict(P, depth_row_lo_frac=0.0, depth_row_hi_frac=1.0,
             cam_x_from_lidar_m=1.0, cam_y_from_lidar_m=0.0, cam_yaw_offset_deg=10.0)
    rel, r = fl.depth_points(img, 100.0, 0.0, p)
    x = 1.0 + 2.0 * math.cos(math.radians(10.0))
    y = 2.0 * math.sin(math.radians(10.0))
    assert rel[0] == pytest.approx(math.degrees(math.atan2(y, x)))
    assert r[0] == pytest.approx(math.hypot(x, y))


def test_depth_image_from_bytes_step_and_endian():
    w, h, step = 3, 2, 8                     # 줄 끝 패딩 2바이트
    rows = []
    for y in range(h):
        vals = [1000 * (y + 1) + x for x in range(w)]
        rows.append(np.array(vals, dtype='<u2').tobytes() + b'\x00\x00')
    img = fl.depth_image(b''.join(rows), h, w, step, False)
    assert img.shape == (2, 3) and img[1, 2] == 2002 and img.dtype == np.uint16
    big = b''.join(np.array([1000 * (y + 1) + x for x in range(w)], dtype='>u2').tobytes() + b'\x00\x00'
                   for y in range(h))
    assert fl.depth_image(big, h, w, step, True)[1, 2] == 2002
    assert fl.depth_image(b'\x00' * 5, h, w, step, False) is None, '길이 부족 → None'
    raw = b''.join(rows)
    assert fl.depth_image(array.array('B', raw), h, w, step, False)[1, 2] == 2002, 'rclpy uint8[] = array.array'
    assert fl.depth_image(list(raw), h, w, step, False)[1, 2] == 2002, '리스트도 받는다'


# ------------------------------------------------------------ 신뢰도

def test_conf_stable_object_is_high():
    tr = _lidar_track([3.0] * 10)
    assert fl.window_conf(tr.hist, 0.0, 3.0, 0.3, 0.5) == pytest.approx(1.0)


def test_conf_formula_valid_ratio_times_stability():
    # 10 프레임 중 8 보임, 값 3.0 3.0 3.6 3.0 ... → 차이 [0,0.6,0.6,0,0,0,0] → MAD 0, 점프율 2/7
    tr = _lidar_track([3.0, 3.0, 3.6, 3.0, 3.0, 3.0, 3.0, 3.0, None, None])
    c = fl.window_conf(tr.hist, 0.0, 3.0, 0.3, 0.5)
    stab = 1.0 - 0.5 * 0.0 - 0.5 * (2.0 / 7.0)
    assert c == pytest.approx(0.8 * stab)


def test_conf_glint_single_frame_below_half():
    """반짝임 1프레임(나머지 9프레임 그 방위에 아무것도 없음) → conf < 0.5 (덧셈 식이 통과시키던 경우)."""
    tr = _depth_track([None] * 9 + [1.2])
    c = fl.window_conf(tr.hist, 0.0, 3.0, 0.3, 0.5)
    assert c < 0.5
    objs, _ = fl.build_objects(0.6, _empty_track(), tr, [], P)
    assert len(objs) == 1 and objs[0]['conf'] < 0.5 and objs[0]['src_id'] == fl.SRC_DEPTH


def test_conf_fewer_than_three_values_stability_half():
    tr = _lidar_track([3.0, 3.0])
    assert fl.window_conf(tr.hist, 0.0, 3.0, 0.3, 0.5) == pytest.approx(0.5)


def test_conf_empty_history_zero():
    assert fl.window_conf([], 0.0, 3.0, 0.3, 0.5) == 0.0


def test_conf_noisy_mad_lowers():
    vals = [3.0, 3.2, 3.0, 3.2, 3.0, 3.2, 3.0, 3.2, 3.0, 3.2]     # MAD 0.2 → −0.333
    tr = _lidar_track(vals)
    c = fl.window_conf(tr.hist, 0.0, 3.0, 0.3, 0.5)
    assert c == pytest.approx(1.0 - 0.5 * (0.2 / 0.3), abs=1e-3)


# ------------------------------------------------------------ 연관·거리 융합

def test_fuse_pair_agree_weighted():
    r, src, conf, flags = fl.fuse_pair(3.0, 0.9, 3.2, 0.3, P)
    wl = 0.7                                  # 0.9/1.2 = 0.75 → 0.7 로 자름
    assert r == pytest.approx(wl * 3.0 + (1 - wl) * 3.2)
    assert src == fl.SRC_FUSED and flags == 0 and conf == pytest.approx(0.9)


def test_fuse_pair_weight_floor():
    r, _, _, _ = fl.fuse_pair(3.0, 0.1, 3.2, 0.9, P)
    assert r == pytest.approx(0.3 * 3.0 + 0.7 * 3.2)


def test_fuse_pair_agree_threshold_relative():
    # min 5.0 → 허용 max(0.3, 0.5) = 0.5: 5.0 vs 5.45 일치, 5.0 vs 5.6 어긋남
    assert fl.fuse_pair(5.0, 0.8, 5.45, 0.8, P)[1] == fl.SRC_FUSED
    assert fl.fuse_pair(5.0, 0.8, 5.6, 0.8, P)[3] & fl.FLAG_DISAGREE


P_MIN = dict(P, disagree_policy='min')
P_LIDAR = dict(P, disagree_policy='lidar')


@pytest.mark.parametrize('dL,dD,want_src', [(1.5, 4.0, 0), (4.0, 1.5, 1)])
def test_fuse_pair_disagree_min(dL, dD, want_src):
    """disagree_policy min: 어긋남 → 평균 금지(2.75 m = 물체 없는 거리), min + bit0, 감점 (11-12 규칙 그대로)."""
    r, src, conf, flags = fl.fuse_pair(dL, 0.9, dD, 0.8, P_MIN)
    assert r == 1.5 and src == want_src and flags & fl.FLAG_DISAGREE
    assert conf == pytest.approx(0.8 - P['disagree_penalty'])


@pytest.mark.parametrize('dL,dD', [(1.5, 4.0), (4.0, 1.5)])
def test_fuse_pair_disagree_lidar(dL, dD):
    """disagree_policy lidar(G-5): 어긋남 → 뎁스가 더 가깝든 멀든 LiDAR 값·src 0, bit0, 감점은 min 정책과 같음."""
    r, src, conf, flags = fl.fuse_pair(dL, 0.9, dD, 0.8, P_LIDAR)
    assert r == dL and src == fl.SRC_LIDAR and flags == fl.FLAG_DISAGREE
    assert conf == pytest.approx(0.8 - P['disagree_penalty'])
    assert fl.fuse_pair(dL, 0.9, dD, 0.8, P_MIN)[2] == conf, '감점 규칙은 정책과 무관'


def test_fuse_pair_default_policy_is_lidar():
    """기본(DEFAULTS) = lidar. 키가 없는 p(옛 호출자)도 lidar 로 본다."""
    assert fl.DEFAULTS['disagree_policy'] == 'lidar'
    assert fl.fuse_pair(4.0, 0.9, 1.5, 0.8, P)[:2] == (4.0, fl.SRC_LIDAR)
    p_old = {k: v for k, v in P.items() if k != 'disagree_policy'}
    assert fl.fuse_pair(4.0, 0.9, 1.5, 0.8, p_old)[:2] == (4.0, fl.SRC_LIDAR)


@pytest.mark.parametrize('dL,cL,dD,cD', [(3.0, 0.9, 3.2, 0.3), (3.0, 0.1, 3.2, 0.9), (5.0, 0.8, 5.45, 0.8),
                                         (2.0, 0.0, 2.1, 0.0), (1.0, 0.5, 0.75, 0.6)])
def test_fuse_pair_agree_unchanged_by_policy(dL, cL, dD, cD):
    """일치하면 정책과 무관하게 같은 가중 결과(비트 동일)."""
    got = {pol: fl.fuse_pair(dL, cL, dD, cD, dict(P, disagree_policy=pol)) for pol in fl.DISAGREE_POLICIES}
    assert got['lidar'] == got['min']
    assert got['lidar'][1] == fl.SRC_FUSED and got['lidar'][3] == 0


def test_associate_overlap_with_margin():
    L = [fl.Blob(-2.0, 2.0, 0.0, 3.0), fl.Blob(20.0, 22.0, 21.0, 8.0)]
    D = [fl.Blob(4.5, 6.0, 5.25, 3.1), fl.Blob(-40.0, -38.0, -39.0, 2.0)]
    pairs = fl.associate(L, D, 3.0)
    got = {(None if a is None else a.bearing, None if b is None else b.bearing) for a, b in pairs}
    assert got == {(0.0, 5.25), (21.0, None), (None, -39.0)}
    assert fl.associate(L[:1], D[:1], 2.0) == [(L[0], None), (None, D[0])], '여유 2° 면 안 겹침'


def test_associate_each_blob_once():
    L = [fl.Blob(-1.0, 1.0, 0.0, 3.0), fl.Blob(1.5, 3.0, 2.25, 3.0)]
    D = [fl.Blob(0.0, 2.0, 1.0, 3.0)]
    pairs = fl.associate(L, D, 3.0)
    paired = [(a, b) for a, b in pairs if a is not None and b is not None]
    assert len(paired) == 1 and len(pairs) == 2


# ------------------------------------------------------------ build_objects

def test_build_fused_agree():
    lt = _lidar_track([3.0] * 10)
    dt = _depth_track([3.1] * 10, t0=0.3)
    objs, stats = fl.build_objects(0.95, lt, dt, [], P)
    assert len(objs) == 1
    o = objs[0]
    assert o['src_id'] == fl.SRC_FUSED and o['flags'] == 0
    assert 3.0 < o['range_m'] < 3.1
    assert o['bearing_deg'] == pytest.approx(0.0, abs=0.3)
    assert o['track_id'] == -1 and o['color_id'] == 0
    assert stats['n'] == 1 and stats['agree'] == pytest.approx(1.0)


@pytest.mark.parametrize('policy,want_r,want_src', [('lidar', 3.0, fl.SRC_LIDAR), ('min', 1.5, fl.SRC_DEPTH)])
def test_build_disagree_policy(policy, want_r, want_src):
    """같은 방위 LiDAR 3.0 m · 뎁스 1.5 m(어긋남) → 정책별 거리·src, 둘 다 bit0·agree 0. 형식 VER·STRIDE 불변."""
    lt = _lidar_track([3.0] * 10)
    dt = _depth_track([1.5] * 10, t0=0.3)
    objs, stats = fl.build_objects(0.95, lt, dt, [], dict(P, disagree_policy=policy))
    assert len(objs) == 1
    o = objs[0]
    assert o['range_m'] == want_r and o['src_id'] == want_src and o['flags'] == fl.FLAG_DISAGREE
    assert stats['agree'] == 0.0
    data = fl.encode(objs)
    assert data[:3] == [float(fl.VER), float(fl.STRIDE), 1.0] and len(data) == 3 + fl.STRIDE
    assert fl.decode(data)[0]['range_m'] == want_r


def test_build_lidar_only_and_depth_only():
    lt = _lidar_track([3.0] * 10, center_rel=30.0)
    dt = _depth_track([2.0] * 10, center_rel=-20.0, t0=0.3)
    objs, _ = fl.build_objects(0.95, lt, dt, [], P)
    by_src = {o['src_id']: o for o in objs}
    assert set(by_src) == {fl.SRC_LIDAR, fl.SRC_DEPTH}
    assert by_src[fl.SRC_LIDAR]['range_m'] == 3.0
    assert by_src[fl.SRC_DEPTH]['range_m'] == 2.0


def test_build_lidar_stale_depth_only_bit1():
    lt = _lidar_track([3.0] * 10)            # 마지막 t = 0.9
    dt = _depth_track([3.1] * 10, t0=1.5)    # 마지막 t = 2.1
    objs, stats = fl.build_objects(2.15, lt, dt, [], P)
    assert len(objs) == 1
    o = objs[0]
    assert o['src_id'] == fl.SRC_DEPTH and o['flags'] & fl.FLAG_LIDAR_STALE
    assert o['range_m'] == pytest.approx(3.1)
    assert not stats['lidar_ok'] and stats['depth_ok']


def test_build_depth_stale_lidar_only_no_flag():
    lt = _lidar_track([3.0] * 10, t0=1.5)
    dt = _depth_track([3.1] * 10)
    objs, _ = fl.build_objects(2.45, lt, dt, [], P)
    assert [o['src_id'] for o in objs] == [fl.SRC_LIDAR] and objs[0]['flags'] == 0


def test_build_both_stale_returns_none():
    lt = _lidar_track([3.0] * 10)
    dt = _depth_track([3.1] * 10)
    objs, stats = fl.build_objects(5.0, lt, dt, [], P)
    assert objs is None, '둘 다 stale → 발행 중단'
    assert not stats['lidar_ok'] and not stats['depth_ok']
    objs, _ = fl.build_objects(0.0, _empty_track(), _empty_track(), [], P)
    assert objs is None, '한 번도 못 받음 = stale'


def test_build_alive_no_objects_is_empty_list():
    lt = _lidar_track([None] * 5)
    objs, stats = fl.build_objects(0.45, lt, _empty_track(), [], P)
    assert objs == [] and stats['n'] == 0


def test_build_age_s():
    lt = _lidar_track([3.0] * 10)            # 마지막 0.9
    objs, _ = fl.build_objects(1.2, lt, _empty_track(), [], P)
    assert objs[0]['age_s'] == pytest.approx(0.3)


# ------------------------------------------------------------ 색

def test_color_assigned_within_margin():
    lt = _lidar_track([3.0] * 10, center_rel=10.0)          # 구간 8~12
    objs, _ = fl.build_objects(0.95, lt, _empty_track(), [(14.5, 1)], P)
    assert objs[0]['color_id'] == 1, '구간 끝 12 + 3 = 15 안'
    objs, _ = fl.build_objects(0.95, lt, _empty_track(), [(15.5, 1)], P)
    assert objs[0]['color_id'] == 0


def test_color_goes_to_nearest_object_only():
    tr = fl.SourceTrack(10)
    for k in range(10):
        ranges, amin, inc, rmin = _scan({**{-2.0 + 0.25 * i: 3.0 for i in range(9)},
                                         **{3.0 + 0.25 * i: 6.0 for i in range(9)}})
        rel, r = fl.scan_points(ranges, amin, inc, rmin, 90.0)
        tr.push(k * 0.1, rel, r)
    objs, _ = fl.build_objects(0.95, tr, _empty_track(), [(2.5, 2)], P)
    colors = sorted((o['bearing_deg'], o['color_id']) for o in objs)
    assert [c for _, c in colors] == [0, 2], '방위가 더 가까운 물체(3~5°) 하나만'


@pytest.mark.parametrize('name,cid', [('red', 1), ('green', 2), ('white', 3), ('yellow', 4),
                                      ('black', 5), ('orange', 6), ('blue', 7), ('none', 0),
                                      ('', 0), ('RED ', 1), ('purple', 0)])
def test_color_id_enum(name, cid):
    assert fl.color_id(name) == cid


@pytest.mark.parametrize('v,ok', [(10000.0, False), (float('nan'), False), (float('inf'), False),
                                  (12.5, True), (0.0, True), (-31.0, True)])
def test_detector_angle_valid(v, ok):
    assert fl.detector_angle_valid(v) is ok


# ------------------------------------------------------------ 출력 형식

def test_encode_format():
    objs = [dict(color_id=1, bearing_deg=-5.0, range_m=3.0, conf=0.8, src_id=2, age_s=0.05,
                 track_id=-1, flags=0),
            dict(color_id=0, bearing_deg=20.0, range_m=7.5, conf=0.4, src_id=0, age_s=0.1,
                 track_id=-1, flags=1)]
    data = fl.encode(objs)
    assert data[:3] == [1.0, 8.0, 2.0]
    assert len(data) == 3 + 8 * 2
    assert data[3:11] == [1.0, -5.0, 3.0, 0.8, 2.0, 0.05, -1.0, 0.0]
    assert fl.decode(data)[1]['range_m'] == 7.5
    assert fl.encode([]) == [1.0, 8.0, 0.0]


def test_encode_constants():
    assert (fl.VER, fl.STRIDE) == (1, 8)
    assert (fl.SRC_LIDAR, fl.SRC_DEPTH, fl.SRC_FUSED, fl.SRC_ROW) == (0, 1, 2, 3)
    assert (fl.FLAG_DISAGREE, fl.FLAG_LIDAR_STALE) == (1, 2)


@pytest.mark.parametrize('bad', [[2.0, 8.0, 0.0], [1.0, 7.0, 0.0], [1.0, 8.0, 1.0], [1.0]])
def test_decode_rejects_bad_header(bad):
    assert fl.decode(bad) is None


# ------------------------------------------------------------ 파라미터 가드·상태·주기

def test_check_params_defaults_pass():
    params, issues = fl.check_params(dict(fl.DEFAULTS))
    assert params == fl.DEFAULTS and issues == []


@pytest.mark.parametrize('key,val', [('lidar_stale_sec', 0.0), ('publish_hz', -1.0),
                                     ('cluster_min_points', 0), ('conf_window_frames', 1),
                                     ('depth_min_m', 7.0), ('depth_row_lo_frac', 0.7),
                                     ('weight_min', 0.8), ('agree_rel', -0.1),
                                     ('depth_min_valid_frac', 0.0), ('depth_topic', ''),
                                     ('lidar_half_fov_deg', 200.0), ('cam_yaw_offset_deg', float('nan'))])
def test_check_params_invalid_falls_back(key, val):
    raw = dict(fl.DEFAULTS, **{key: val})
    params, issues = fl.check_params(raw)
    assert params[key] == fl.DEFAULTS[key]
    assert issues and any(key in s for s in issues)


@pytest.mark.parametrize('val', ['average', '', 'fused', None, 1, True, ['lidar']])
def test_check_params_disagree_policy_invalid(val):
    """이상값 → 이슈(노드가 ERROR) + 기본 lidar."""
    params, issues = fl.check_params(dict(fl.DEFAULTS, disagree_policy=val))
    assert params['disagree_policy'] == 'lidar'
    assert any('disagree_policy' in s for s in issues)


@pytest.mark.parametrize('val,want', [('min', 'min'), ('lidar', 'lidar'), (' MIN ', 'min'), ('Lidar', 'lidar')])
def test_check_params_disagree_policy_valid(val, want):
    params, issues = fl.check_params(dict(fl.DEFAULTS, disagree_policy=val))
    assert params['disagree_policy'] == want and issues == []


def test_format_status():
    s = fl.format_status({'n': 3, 'conf_l': 0.82, 'conf_d': 0.61, 'agree': 0.93,
                          'lidar_ok': True, 'depth_ok': True}, 2)
    assert s == 'L0.82 D0.61 agree0.93 lidar_ok depth_ok n=3 m=2'
    s = fl.format_status({'n': None, 'conf_l': None, 'conf_d': None, 'agree': None,
                          'lidar_ok': False, 'depth_ok': False}, -1)
    assert s == 'L- D- agree- lidar_stale depth_stale n=- m=-1 (발행 중단)'


def test_due():
    assert fl.due(None, 0.0, 5.0)
    assert not fl.due(0.0, 4.9, 5.0)
    assert fl.due(0.0, 5.0, 5.0)


def test_detections_from_fresh_and_color():
    det = fl.detections_from(10.0, (9.5, 4.0), (9.0, -6.0), (9.8, 1.0), (9.8, 'yellow'), 0.7)
    assert det == [(4.0, 1), (1.0, 4)], 'green 1.0 s 묵음 → 뺌, image 는 신선한 /image_color 색'
    det = fl.detections_from(10.0, None, None, (9.8, 1.0), (8.0, 'red'), 0.7)
    assert det == [], '/image_color 가 묵으면 색 모름 → 안 씀'
    det = fl.detections_from(10.0, None, None, (9.8, 1.0), (9.9, 'none'), 0.7)
    assert det == []

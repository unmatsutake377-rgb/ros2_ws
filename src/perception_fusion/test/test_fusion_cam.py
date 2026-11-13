#!/usr/bin/env python3
"""perception_fusion P1 카메라 경로 테스트 (계약서_흰부표_거리융합_v3 B-2 + '단안·삼각측량 수식 정밀화'). ROS 없이 돈다.

    tools/run_tests.sh perception_fusion

지키는 것
  · 비트 동일: 카메라 경로 없음(cams None·[] + cam_live False) = P1 이전 HEAD 11db5c9 출력 골든과 같다
  · /oak_detections 파서: VER·STRIDE·길이·N 불일치 = 통째로 버림, 줄 단위 무효 = 그 줄만
  · 단안: 정면·측면 20°·32°·롤 3°·피치 오차가 기하 합성 해석해와 일치, 수평선 None, σ 단조, σ 근사식
  · 삼각측량: 옆 지나가기 참값 일치, 정면 접근·제자리 회전 미발행, 기선·시차·시간차 하한 경계, 비FIXED 미사용,
    레버암, 뒤쪽 교점 None, σ ≈ √2·d²σα/(b·sinγ)
  · RTK FIXED 판정·평면 근사·자세 버퍼 보간
  · 융합: LiDAR 우선(정책별), LiDAR 없는 물체 역분산 가중·게이팅·부분 합의·전부 어긋남, src 4·5·2, 기여 비트
"""

import math

import numpy as np
import pytest

from perception_fusion import fusion_logic as fl

P = dict(fl.DEFAULTS)
H = 0.6


# ------------------------------------------------------------ 골든 장면 (HEAD 11db5c9 fusion_logic 으로 만든 값)

def _lidar(frames_by_center, t0=0.0, dt=0.1, inc=0.25, hw=2.0):
    tr = fl.SourceTrack(10)
    for k in range(len(next(iter(frames_by_center.values())))):
        n = int(round(360.0 / inc))
        ranges = [float('inf')] * n
        for c, frames in frames_by_center.items():
            r = frames[k]
            if r is None:
                continue
            j = -hw
            while j <= hw + 1e-9:
                ranges[int(round((80.0 + c + j + 180.0) / inc)) % n] = r
                j += inc
        rel, rr = fl.scan_points(ranges, math.radians(-180.0), math.radians(inc), 0.15, 90.0)
        tr.push(t0 + k * dt, rel, rr)
    return tr


def _depth(frames_by_center, t0=0.0, dt=1.0 / 15.0, hw=2.0):
    tr = fl.SourceTrack(10)
    for k in range(len(next(iter(frames_by_center.values())))):
        rel, rr = [], []
        for c, frames in frames_by_center.items():
            r = frames[k]
            if r is None:
                continue
            a = np.arange(c - hw, c + hw + 1e-9, 0.2)
            rel += a.tolist()
            rr += [float(r)] * len(a)
        tr.push(t0 + k * dt, np.array(rel), np.array(rr))
    return tr


def _scenes():
    return {
        'agree': (0.95, _lidar({0.0: [3.0] * 10}), _depth({0.0: [3.1] * 10}, t0=0.3), [(0.5, 1)], P),
        'disagree_lidar': (0.95, _lidar({0.0: [3.0] * 10}), _depth({0.0: [1.5] * 10}, t0=0.3), [], P),
        'disagree_min': (0.95, _lidar({0.0: [3.0] * 10}), _depth({0.0: [1.5] * 10}, t0=0.3), [],
                         dict(P, disagree_policy='min')),
        'lidar_stale': (2.15, _lidar({0.0: [3.0] * 10}), _depth({0.0: [3.1] * 10}, t0=1.5), [(1.0, 4)], P),
        'multi': (0.95, _lidar({-20.0: [4.0, 4.1, None, 4.0, 4.2, 4.0, 4.0, 4.1, 4.0, 4.0], 15.0: [2.5] * 10}),
                  _depth({15.5: [2.6] * 10, 40.0: [None] * 9 + [1.2]}, t0=0.3),
                  [(-19.0, 2), (16.0, 1), (40.5, 3)], P),
        'noisy': (0.95, _lidar({5.0: [3.0, 3.2, 3.0, 3.6, 3.0, 3.2, 3.0, 3.2, 3.0, 3.2]}),
                  _depth({5.0: [3.3, None, 3.2, 3.4, None, 3.3, 3.2, 3.3, 3.4, 3.3]}, t0=0.3), [], P),
    }


GOLDEN = {
    'agree': [1.0, 8.0, 1.0, 1.0, 0.0, 3.05, 1.0, 2.0, 0.050000000000000044, -1.0, 0.0],
    'disagree_lidar': [1.0, 8.0, 1.0, 0.0, 0.0, 3.0, 0.9, 0.0, 0.050000000000000044, -1.0, 1.0],
    'disagree_min': [1.0, 8.0, 1.0, 0.0, 0.0, 1.5, 0.9, 1.0, 0.050000000000000044, -1.0, 1.0],
    'lidar_stale': [1.0, 8.0, 1.0, 4.0, -4.440892098500626e-16, 3.1, 1.0, 1.0, 0.04999999999999982, -1.0, 2.0],
    'multi': [1.0, 8.0, 3.0, 3.0, 40.00000000000003, 1.2, 0.05, 1.0, 0.050000000000000044, -1.0, 0.0,
              1.0, 15.000000000000014, 2.55, 1.0, 2.0, 0.050000000000000044, -1.0, 0.0,
              2.0, -20.0, 4.0, 0.7500000000000006, 0.0, 0.04999999999999993, -1.0, 0.0],
    'noisy': [1.0, 8.0, 1.0, 0.0, 5.000000000000014, 3.254545454545455, 0.6666666666666665, 2.0,
              0.050000000000000044, -1.0, 0.0],
}


@pytest.mark.parametrize('name', sorted(GOLDEN))
@pytest.mark.parametrize('cams,live', [(None, False), ([], False)])
def test_bit_identical_golden_without_camera(name, cams, live):
    """카메라 경로가 없으면(스위치 꺼짐 = None, /oak_detections 없음 = cam_live False) HEAD 출력과 비트 동일."""
    now, lt, dt, dets, p = _scenes()[name]
    objs, _ = fl.build_objects(now, lt, dt, dets, p, cams=cams, cam_live=live)
    assert fl.encode(objs) == GOLDEN[name]


def test_cam_live_adds_only_source_bits():
    """카메라 경로가 살아 있고 측정이 0개면 바뀌는 것은 flags bit2~5(기여 소스)뿐 — 거리·src·conf 그대로."""
    now, lt, dt, dets, p = _scenes()['multi']
    base = fl.decode(GOLDEN['multi'])
    objs, stats = fl.build_objects(now, lt, dt, dets, p, cams=[], cam_live=True)
    got = fl.decode(fl.encode(objs))
    want_bits = [fl.FLAG_SRC_DEPTH, fl.FLAG_SRC_LIDAR | fl.FLAG_SRC_DEPTH, fl.FLAG_SRC_LIDAR]
    for a, b, bits in zip(base, got, want_bits):
        assert {k: v for k, v in a.items() if k != 'flags'} == {k: v for k, v in b.items() if k != 'flags'}
        assert int(b['flags']) == int(a['flags']) | bits
    assert stats['mono_n'] == 0 and stats['tri_used'] == 0


def test_source_bits_respect_disagree_choice():
    """L–D 어긋남: policy lidar → bit2 만, policy min(뎁스 선택) → bit3 만. bit0 은 그대로."""
    for name, bits in (('disagree_lidar', fl.FLAG_SRC_LIDAR), ('disagree_min', fl.FLAG_SRC_DEPTH)):
        now, lt, dt, dets, p = _scenes()[name]
        objs, _ = fl.build_objects(now, lt, dt, dets, p, cams=[], cam_live=True)
        assert objs[0]['flags'] == fl.FLAG_DISAGREE | bits


def test_format_constants():
    assert (fl.SRC_MONO, fl.SRC_TRI) == (4, 5)
    assert (fl.FLAG_SRC_LIDAR, fl.FLAG_SRC_DEPTH, fl.FLAG_SRC_MONO, fl.FLAG_SRC_TRI) == (4, 8, 16, 32)
    assert (fl.VER, fl.STRIDE) == (1, 8), 'VER·STRIDE 유지 — 배치가 같아 올리지 않는다'
    assert (fl.OAK_VER, fl.OAK_STRIDE) == (1, 6) and len(fl.OAK_FIELDS) == fl.OAK_STRIDE


# ------------------------------------------------------------ /oak_detections 파서

def _oak(rows, ver=1.0, stride=6.0, n=None):
    data = [ver, stride, float(len(rows) if n is None else n)]
    for r in rows:
        data += [float(v) for v in r]
    return data


GOOD = (3, 10.0, 0.1, 2.0, 0.8, 0.05)


def test_parse_oak_ok_and_empty():
    dets, bad, err = fl.parse_oak_detections(_oak([GOOD, (4, -20.0, 0.2, 3.0, 0.5, 0.0)]))
    assert err is None and bad == 0 and len(dets) == 2
    assert dets[0] == dict(color_id=3, bearing_left_deg=10.0, v_norm_bottom=0.1, ang_width_deg=2.0,
                           conf=0.8, age_s=0.05)
    assert isinstance(dets[0]['color_id'], int)
    assert fl.parse_oak_detections([1.0, 6.0, 0.0]) == ([], 0, None), 'N 0 = 살아 있고 물체 없음'


@pytest.mark.parametrize('data', [
    _oak([GOOD], ver=2.0), _oak([GOOD], stride=5.0), _oak([GOOD], n=2), _oak([GOOD], n=1.5),
    _oak([GOOD], n=-1), [1.0, 6.0], [], _oak([GOOD])[:-1], _oak([GOOD]) + [0.0], ['a', 6, 0],
    [1.0, 6.0, float('nan')],
])
def test_parse_oak_rejects_whole_message(data):
    dets, bad, err = fl.parse_oak_detections(data)
    assert dets is None and err


@pytest.mark.parametrize('row', [
    (3, 10.0, 0.1, 2.0, 0.8, -0.01),          # age < 0
    (3, float('nan'), 0.1, 2.0, 0.8, 0.0),
    (8, 10.0, 0.1, 2.0, 0.8, 0.0),            # 열거형 밖
    (2.5, 10.0, 0.1, 2.0, 0.8, 0.0),          # 정수 아님
    (-1, 10.0, 0.1, 2.0, 0.8, 0.0),
    (3, 90.0, 0.1, 2.0, 0.8, 0.0),            # |방위| ≥ 90
    (3, 10.0, 0.1, -1.0, 0.8, 0.0),           # 폭 < 0
    (3, 10.0, 0.1, 2.0, 1.2, 0.0),            # conf > 1
    (3, 10.0, float('inf'), 2.0, 0.8, 0.0),
])
def test_parse_oak_drops_bad_row_only(row):
    dets, bad, err = fl.parse_oak_detections(_oak([GOOD, row]))
    assert err is None and bad == 1 and len(dets) == 1 and dets[0]['bearing_left_deg'] == 10.0


def test_parse_oak_boundaries_accepted():
    dets, bad, _ = fl.parse_oak_detections(_oak([(0, 89.9, -0.5, 0.0, 0.0, 0.0), (7, -89.9, 0.3, 5.0, 1.0, 2.0)]))
    assert bad == 0 and [d['color_id'] for d in dets] == [0, 7]


# ------------------------------------------------------------ 단안 (기하 합성)

def _project(D, beta_deg, pitch_deg, roll_deg, h=H):
    """수면 점(수평 거리 D, 방위 β 좌 +) → 카메라 정규 좌표 (x 오른쪽, y 아래).

    카메라 축을 수평 좌표에서 직접 만든다: 롤 ρ 는 오른쪽 축을 아래로, 피치 θ 는 앞 축을 아래로 기울인다.
    (fusion_logic 의 역변환과 독립된 정변환 — 축 벡터의 내적으로 투영)
    """
    t, r = math.radians(pitch_deg), math.radians(roll_deg)
    # 롤만 걸린 축 (앞, 오른쪽, 아래)
    f0, r0, d0 = (1.0, 0.0, 0.0), (0.0, math.cos(r), math.sin(r)), (0.0, -math.sin(r), math.cos(r))

    def pitch(v):          # 앞–아래 평면 회전
        return (v[0] * math.cos(t) - v[2] * math.sin(t), v[1], v[0] * math.sin(t) + v[2] * math.cos(t))

    fwd, right, down = pitch(f0), pitch(r0), pitch(d0)
    pt = (D * math.cos(math.radians(beta_deg)), -D * math.sin(math.radians(beta_deg)), h)

    def dot(a, b):
        return sum(x * y for x, y in zip(a, b))

    z = dot(pt, fwd)
    return dot(pt, right) / z, dot(pt, down) / z


@pytest.mark.parametrize('D,beta,pitch,roll', [
    (3.0, 0.0, 5.0, 0.0), (3.0, 20.0, 5.0, 0.0), (3.0, 32.0, 5.0, 0.0), (3.0, -32.0, 5.0, 0.0),
    (3.0, 0.0, 0.0, 3.0), (4.0, 25.0, 4.0, 3.0), (6.0, -32.0, 8.0, -3.0), (1.5, 20.0, 10.0, 3.0),
])
def test_mono_matches_geometry(D, beta, pitch, roll):
    x, y = _project(D, beta, pitch, roll)
    rng, b = fl.mono_range(x, y, H, pitch, roll)
    assert rng == pytest.approx(D, abs=1e-9) and b == pytest.approx(beta, abs=1e-9)


def test_mono_frontal_closed_form():
    """정면: y = tan(atan(h/D) − θ) → D (계약 이전 식 h/tan(θ + atan y) 와 같아야 한다)."""
    y = math.tan(math.atan(H / 3.0) - math.radians(5.0))
    assert fl.mono_range(0.0, y, H, 5.0, 0.0)[0] == pytest.approx(3.0, abs=1e-12)


def test_mono_roll_closed_form():
    """롤 3°(우현 아래), 정면 3 m: 카메라에선 x = h sinρ / D, y = h cosρ / D 로 보인다 → 3 m, 방위 0."""
    rho = math.radians(3.0)
    rng, b = fl.mono_range(H * math.sin(rho) / 3.0, H * math.cos(rho) / 3.0, H, 0.0, 3.0)
    assert rng == pytest.approx(3.0, abs=1e-12) and b == pytest.approx(0.0, abs=1e-9)


def test_mono_lateral_beats_old_formula():
    """32° 측면: 수평 거리 t·√(f² + s²) 가 참값. 옛 식(h/tan(θ + atan y))은 약 15% 짧게 낸다."""
    x, y = _project(3.0, 32.0, 5.0, 0.0)
    old = H / math.tan(math.radians(5.0) + math.atan(y))
    assert fl.mono_range(x, y, H, 5.0, 0.0)[0] == pytest.approx(3.0, abs=1e-9)
    assert old == pytest.approx(2.544, abs=2e-3)


def test_mono_pitch_error_matches_first_order():
    """장착 피치를 1° 크게 알면(참 5°, 가정 6°) 정면 거리 = h/tan(φ + 1°), 1차 근사 Δd ≈ −(d² + h²)/h·δ."""
    y = math.tan(math.atan(H / 3.0) - math.radians(5.0))
    got = fl.mono_range(0.0, y, H, 6.0, 0.0)[0]
    assert got == pytest.approx(H / math.tan(math.atan(H / 3.0) + math.radians(1.0)), abs=1e-12)
    first = 3.0 - (9.0 + H * H) / H * math.radians(1.0)
    assert got == pytest.approx(first, abs=0.05)
    x, y = _project(3.0, 20.0, 5.0, 3.0)
    assert abs(fl.mono_range(x, y, H, 6.0, 3.0)[0] - 3.0) > 0.2, '측면·롤에서도 피치 1° = 약 0.24 m'


@pytest.mark.parametrize('y', [0.0, -0.1, math.tan(math.radians(0.49)), math.tan(math.radians(0.3))])
def test_mono_horizon_none(y):
    assert fl.mono_range(0.0, y, H, 0.0, 0.0) is None
    assert fl.mono_sigma(0.0, y, H, 0.0, 0.0, P) == math.inf


def test_mono_horizon_boundary():
    assert fl.mono_range(0.0, math.tan(math.radians(0.51)), H, 0.0, 0.0) is not None
    assert fl.mono_range(0.0, math.tan(math.radians(0.6)), H, -1.0, 0.0) is None, '카메라 1° 들림 → 실제 −0.4°'
    assert fl.mono_range(0.0, math.tan(math.radians(0.3)), H, 1.0, 0.0) is not None, '1° 숙임 → 1.3°'


def test_mono_sigma_pitch_only_matches_closed_form():
    """σ(피치 하나) = h/sin²φ·σθ (core.ground_plane_sigma 와 같은 식) — 정면, 롤·픽셀·높이 σ 0."""
    p = dict(P, oak_cam_h_sigma_m=0.0, mono_pixel_sigma=0.0)
    for D in (2.0, 3.0, 5.0, 8.0):
        y = H / D
        phi = math.atan2(H, D)
        assert fl.mono_sigma(0.0, y, H, 0.0, 0.0, p) == pytest.approx(
            H / math.sin(phi) ** 2 * math.radians(p['mono_pitch_sigma_deg']), rel=1e-6)


def test_mono_sigma_height_term():
    p = dict(P, mono_pitch_sigma_deg=0.0, mono_pixel_sigma=0.0, oak_cam_h_sigma_m=0.06)
    assert fl.mono_sigma(0.0, H / 3.0, H, 0.0, 0.0, p) == pytest.approx(3.0 * 0.06 / H, rel=1e-9)


def test_mono_sigma_monotone():
    sig_d = [fl.mono_sigma(*_project(D, 20.0, 5.0, 3.0), H, 5.0, 3.0, P) for D in (1.0, 2.0, 3.0, 5.0, 8.0)]
    assert all(a < b for a, b in zip(sig_d, sig_d[1:])), sig_d
    x, y = _project(3.0, 20.0, 5.0, 3.0)
    sig_s = [fl.mono_sigma(x, y, H, 5.0, 3.0, dict(P, mono_pitch_sigma_deg=s)) for s in (0.5, 1.0, 2.0, 4.0)]
    assert all(a < b for a, b in zip(sig_s, sig_s[1:])), sig_s
    sig_px = [fl.mono_sigma(x, y, H, 5.0, 3.0, dict(P, mono_pixel_sigma=s)) for s in (0.0, 2.0, 8.0)]
    assert all(a < b for a, b in zip(sig_px, sig_px[1:])), sig_px


def _det(bearing, v, color=3, width=2.0, conf=0.8, age=0.0):
    return dict(color_id=color, bearing_left_deg=bearing, v_norm_bottom=v, ang_width_deg=width, conf=conf, age_s=age)


def test_mono_measurements_offsets_and_filters():
    x, y = _project(3.0, 20.0, 5.0, 0.0)
    det = _det(-math.degrees(math.atan(x)), y)
    p = dict(P, oak_pitch_deg=5.0, oak_x_from_lidar_m=0.3, oak_y_from_lidar_m=-0.1, oak_yaw_offset_deg=-4.0)
    (m,) = fl.mono_measurements([det], 10.0, 10.1, 5.0, 0.0, p)
    th = math.radians(20.0 - 4.0)
    px, py = 0.3 + 3.0 * math.cos(th), -0.1 + 3.0 * math.sin(th)
    assert m['range_m'] == pytest.approx(math.hypot(px, py), abs=1e-9)
    assert m['bearing_deg'] == pytest.approx(math.degrees(math.atan2(py, px)), abs=1e-9)
    assert (m['lo'], m['hi']) == pytest.approx((m['bearing_deg'] - 1.0, m['bearing_deg'] + 1.0))
    assert m['src'] == fl.SRC_MONO and m['bit'] == fl.FLAG_SRC_MONO and m['color_id'] == 3 and m['t'] == 10.0
    assert fl.mono_measurements([dict(det, age_s=0.5)], 10.0, 10.25, 5.0, 0.0, p) == [], '촬영 0.75 s 전 > 0.7'
    assert fl.mono_measurements([_det(0.0, -0.05)], 10.0, 10.0, 0.0, 0.0, p) == [], '수평선 위'
    far = _project(9.0, 0.0, 0.0, 0.0)
    assert fl.mono_measurements([_det(0.0, far[1])], 10.0, 10.0, 0.0, 0.0, P) == [], 'mono_max_range_m 8'


def test_quat_pitch_roll_rep103():
    """FLU 쿼터니언: +y 축 5° = 선수 숙임 +5, +x 축 3° = 우현 아래 +3."""
    def q(axis, deg):
        s, c = math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2)
        return tuple(s * a for a in axis) + (c,)
    assert fl.quat_pitch_roll(*q((0, 1, 0), 5.0)) == pytest.approx((5.0, 0.0), abs=1e-9)
    assert fl.quat_pitch_roll(*q((1, 0, 0), 3.0)) == pytest.approx((0.0, 3.0), abs=1e-9)


def test_camera_attitude_mount_plus_imu_or_mount_only():
    p = dict(P, oak_pitch_deg=4.0, oak_roll_deg=-1.0)
    assert fl.camera_attitude((9.8, 1.5, 2.0), 10.0, p) == (5.5, 1.0, True)
    assert fl.camera_attitude((9.0, 1.5, 2.0), 10.0, p) == (4.0, -1.0, False), '묵음 → 장착값만'
    assert fl.camera_attitude(None, 10.0, p) == (4.0, -1.0, False)


# ------------------------------------------------------------ GPS·버퍼

def test_ll_to_local_flat():
    lat0, lon0 = 35.2, 128.6
    x, y = fl.ll_to_local(lat0 + 1e-4, lon0 + 1e-4, lat0, lon0)
    k = math.pi / 180.0 * 6371000.0
    assert y == pytest.approx(1e-4 * k) and x == pytest.approx(1e-4 * k * math.cos(math.radians(lat0)))
    assert y == pytest.approx(11.12, abs=0.01) and x == pytest.approx(9.09, abs=0.01)


@pytest.mark.parametrize('status,cov,ctype,want', [
    (2, [0.015 ** 2, 0, 0, 0, 0.015 ** 2, 0, 0, 0, 1.0], 2, (True, 0.015)),
    (2, [0.10 ** 2, 0, 0, 0, 0.10 ** 2, 0, 0, 0, 1.0], 2, (False, 0.10)),     # σ 큼 (float 수준)
    (0, [0.015 ** 2, 0, 0, 0, 0.015 ** 2, 0, 0, 0, 1.0], 2, (False, 0.015)),  # 일반 fix (carrSoln ≠ 2)
    (1, [0.015 ** 2, 0, 0, 0, 0.015 ** 2, 0, 0, 0, 1.0], 2, (False, 0.015)),
    (2, [0.0] * 9, 0, (False, None)),                                         # 공분산 모름
    (2, [float('nan')] * 9, 2, (False, None)),
    (2, [0.01 ** 2, 0, 0, 0, 0.06 ** 2, 0, 0, 0, 1.0], 2, (False, 0.06)),      # 큰 축으로 본다
])
def test_fix_quality(status, cov, ctype, want):
    fixed, sigma = fl.fix_quality(status, cov, ctype, P['tri_rtk_max_sigma_m'])
    assert fixed is want[0]
    assert (sigma is None) == (want[1] is None) and (sigma is None or sigma == pytest.approx(want[1]))


def test_timeseries_bracket_interp_and_gaps():
    ts = fl.TimeSeries(5.0)
    for t, v in ((0.0, 0.0), (0.2, 2.0), (1.0, 10.0)):
        ts.push(t, v)
    a, b, w = ts.bracket(0.1, 0.3)
    assert a + w * (b - a) == pytest.approx(1.0)
    assert ts.bracket(0.6, 0.3) is None, '앞뒤 표본 0.4 s 떨어짐 → 모름'
    assert ts.bracket(1.2, 0.3)[0] == 10.0, '마지막 표본 0.2 s 안 → 그 값'
    assert ts.bracket(1.4, 0.3) is None
    ts.push(7.0, 1.0)
    assert ts.buf[0][0] == 7.0 - 5.0 or ts.buf[0][0] >= 2.0, 'horizon 밖은 버림'


def test_yaw_at_wraps():
    ts = fl.TimeSeries(5.0)
    ts.push(0.0, 358.0)
    ts.push(0.2, 4.0)
    assert fl.yaw_at(ts, 0.1, 0.3) == pytest.approx(1.0)


# ------------------------------------------------------------ 삼각측량

LEVER = dict(P, gps_to_cam_x_m=0.5, gps_to_cam_y_m=0.2)


def _cam_pos(ax, ay, yaw, p):
    r = math.radians(yaw)
    return (ax + math.sin(r) * p['gps_to_cam_x_m'] - math.cos(r) * p['gps_to_cam_y_m'],
            ay + math.cos(r) * p['gps_to_cam_x_m'] + math.sin(r) * p['gps_to_cam_y_m'])


def _drive(buoy, path, yaws, p, truth_p=None, fixed=True, dt=0.2, n=16, sig=0.015):
    """안테나 경로·yaw 를 따라가며 부표를 본 검출을 합성 → (추정 목록, 참 거리(마지막 카메라 기준), 마지막 사유)."""
    truth_p = truth_p or p
    gps, yaw = fl.TimeSeries(10.0), fl.TimeSeries(10.0)
    tr = fl.TriTracker()
    why = None
    for k in range(n):
        t = k * dt
        ax, ay = path(t)
        ps = yaws(t) % 360.0
        gps.push(t, (ax, ay, sig, fixed))
        yaw.push(t, ps)
        cx, cy = _cam_pos(ax, ay, ps, truth_p)
        beta = math.degrees(math.atan2(buoy[0] - cx, buoy[1] - cy))
        b = (ps - beta + 180.0) % 360.0 - 180.0
        ray, why = fl.make_ray(_det(b, 0.1), t, gps, yaw, 0.0, 0.0, p)
        tr.add([ray] if ray is not None else [], p)
    true_r = math.hypot(buoy[0] - cx, buoy[1] - cy)
    return tr.estimates((n - 1) * dt, p), true_r, why


def test_tri_pass_by_matches_truth():
    """동쪽(yaw 90)으로 1 m/s, 부표 (6, 4) 를 왼쪽에 두고 지나감 → 교점·거리·방위가 참값과 일치."""
    est, true_r, _ = _drive((6.0, 4.0), lambda t: (t, 0.0), lambda t: 90.0, LEVER)
    (e,) = est
    assert (e['x'], e['y']) == pytest.approx((6.0, 4.0), abs=1e-9)
    assert e['range_m'] == pytest.approx(true_r, abs=1e-9)
    cx, cy = _cam_pos(3.0, 0.0, 90.0, LEVER)
    want_b = 90.0 - math.degrees(math.atan2(6.0 - cx, 4.0 - cy))
    assert e['bearing_deg'] == pytest.approx(want_b, abs=1e-9) and e['bearing_deg'] > 0, '왼쪽 = +'
    assert e['src'] == fl.SRC_TRI and e['color_id'] == 3 and e['n_rays'] >= 3
    assert 0.0 < e['sigma'] < 0.5


def test_tri_turning_with_lever_arm():
    """돌면서 지나감: 레버암을 알면 참값, 0 으로 두면 거리가 틀린다(레버암이 yaw 로 돈다)."""
    path, yaws = (lambda t: (t, 0.3 * t)), (lambda t: 80.0 - 5.0 * t)
    est, true_r, _ = _drive((5.0, 5.0), path, yaws, LEVER)
    assert (est[0]['x'], est[0]['y']) == pytest.approx((5.0, 5.0), abs=1e-9)
    assert est[0]['range_m'] == pytest.approx(true_r, abs=1e-9)
    no_lever = dict(LEVER, gps_to_cam_x_m=0.0, gps_to_cam_y_m=0.0)
    est2, _, _ = _drive((5.0, 5.0), path, yaws, no_lever, truth_p=LEVER)
    assert math.hypot(est2[0]['x'] - 5.0, est2[0]['y'] - 5.0) > 0.1, '레버암 0.54 m 를 빼먹으면 약 0.19 m 빗나감'


def test_tri_frontal_approach_silent():
    est, _, _ = _drive((10.0, 0.0), lambda t: (t, 0.0), lambda t: 90.0, LEVER)
    assert est == [], '정면 접근 = 시차 0'


def test_tri_stationary_and_spin_in_place_silent():
    est, _, _ = _drive((6.0, 4.0), lambda t: (0.0, 0.0), lambda t: 90.0, LEVER)
    assert est == [], '정지 = 기선 0'
    big = dict(LEVER, gps_to_cam_x_m=1.0)
    est, _, _ = _drive((3.0, 4.0), lambda t: (0.0, 0.0), lambda t: 60.0 + 20.0 * t, big)
    assert est == [], '제자리 회전: 카메라는 움직여도 안테나 기선 0 → 안 냄'


def test_tri_non_fixed_not_used():
    est, _, why = _drive((6.0, 4.0), lambda t: (t, 0.0), lambda t: 90.0, LEVER, fixed=False)
    assert est == [] and why == 'rtk'


def test_make_ray_missing_pose():
    gps, yaw = fl.TimeSeries(10.0), fl.TimeSeries(10.0)
    assert fl.make_ray(_det(0.0, 0.1), 1.0, gps, yaw, 0.0, 0.0, P) == (None, 'yaw')
    yaw.push(1.0, 90.0)
    assert fl.make_ray(_det(0.0, 0.1), 1.0, gps, yaw, 0.0, 0.0, P) == (None, 'gps')
    gps.push(0.5, (0.0, 0.0, 0.015, True))
    assert fl.make_ray(_det(0.0, 0.1), 1.0, gps, yaw, 0.0, 0.0, P) == (None, 'gps'), '0.5 s > 0.3 s 간격'


def _ray(t, ax, ay, alpha, yaw=0.0, sig=0.015):
    return fl.Ray(t, ax, ay, ax, ay, alpha % 360.0, yaw, sig, 3, 0.8, 1.0)


def _two(b, Y, dt=1.0):
    """안테나 (0,0)·(b,0), 부표 (b/2, Y) 를 보는 두 광선(북쪽 향함)."""
    return [_ray(0.0, 0.0, 0.0, math.degrees(math.atan2(b / 2, Y))),
            _ray(dt, b, 0.0, math.degrees(math.atan2(-b / 2, Y)))]


def test_tri_baseline_boundary():
    assert fl.tri_estimate(_two(0.79, 3.0), P) is None
    e = fl.tri_estimate(_two(0.81, 3.0), P)
    assert (e['x'], e['y']) == pytest.approx((0.405, 3.0), abs=1e-9)


def test_tri_parallax_boundary():
    """b = 1, 시차 γ = 2·atan(0.5/Y): 4.9° 는 안 내고 5.1° 는 낸다."""
    def y_for(g):
        return 0.5 / math.tan(math.radians(g / 2))
    assert fl.tri_estimate(_two(1.0, y_for(4.9)), P) is None
    e = fl.tri_estimate(_two(1.0, y_for(5.1)), P)
    assert e is not None and e['y'] == pytest.approx(y_for(5.1), abs=1e-6)


def test_tri_dt_boundary():
    assert fl.tri_estimate(_two(1.0, 3.0, dt=3.0), P) is not None
    assert fl.tri_estimate(_two(1.0, 3.0, dt=3.01), P) is None


def test_tri_behind_is_none():
    """두 광선이 뒤쪽에서 만나면(벌어지는 광선) 안 낸다."""
    rays = [_ray(0.0, 0.0, 0.0, -10.0), _ray(1.0, 1.0, 0.0, 10.0)]
    assert fl.tri_solve([(0.0, 0.0, 350.0), (1.0, 0.0, 10.0)]) is None
    assert fl.tri_estimate(rays, P) is None


def test_tri_two_ray_equals_least_squares():
    pts = [(0.0, 0.0, 30.0), (2.0, 0.0, -20.0)]
    X2 = fl.tri_solve(pts)
    X3 = fl.tri_solve(pts + [(1.0, -1.0, math.degrees(math.atan2(X2[0] - 1.0, X2[1] + 1.0)))])
    assert X3 == pytest.approx(X2, abs=1e-9)


@pytest.mark.parametrize('b', [1.0, 2.0])
def test_tri_sigma_matches_stereo_approx(b):
    """σd ≈ √2·d²·σα/(b·sinγ) (γ = 기선과 시선의 각, 두 광선 독립) — 위치 σ·픽셀 σ 0 으로 비교."""
    p = dict(P, mono_pixel_sigma=0.0)
    rays = [fl.Ray(t, x, 0.0, x, 0.0, a, 0.0, 0.0, 3, 0.8, 1.0) for t, x, a in
            ((0.0, 0.0, math.degrees(math.atan2(b / 2, 5.0))), (1.0, b, math.degrees(math.atan2(-b / 2, 5.0)) % 360))]
    e = fl.tri_estimate(rays, p)
    d = e['range_m']
    gam = math.atan2(5.0, b / 2)
    approx = math.sqrt(2.0) * d * d * math.radians(p['tri_yaw_sigma_deg']) / (b * math.sin(gam))
    assert e['sigma'] == pytest.approx(approx, rel=0.15)
    e2 = fl.tri_estimate(rays, dict(p, tri_yaw_sigma_deg=1.0))
    assert e2['sigma'] == pytest.approx(2.0 * e['sigma'], rel=1e-3), 'σα 에 선형'


def test_tri_tracker_association_by_color_and_bearing():
    tr = fl.TriTracker()
    tr.add([_ray(0.0, 0.0, 0.0, 30.0), fl.Ray(0.0, 0.0, 0.0, 0.0, 0.0, 30.0, 0.0, 0.015, 4, 0.8, 1.0)], P)
    assert len(tr.tracks) == 2, '색이 다르면 다른 트랙'
    tr.add([_ray(0.2, 0.2, 0.0, 33.0)], P)
    assert len(tr.tracks) == 2 and len(tr.tracks[0]['rays']) == 2
    tr.add([_ray(0.4, 0.4, 0.0, 45.0)], P)
    assert len(tr.tracks) == 3, 'α 차 12° > tri_assoc_deg 5° → 새 트랙'
    assert tr.estimates(10.0, P) == [] and tr.tracks == [], '오래된 트랙 정리'


# ------------------------------------------------------------ 융합 규칙

def _cam(src, r, sigma, bearing=0.0, conf=0.8, color=3, t=0.9, half=1.0):
    bit = fl.FLAG_SRC_MONO if src == fl.SRC_MONO else fl.FLAG_SRC_TRI
    return dict(src=src, bit=bit, range_m=r, sigma=sigma, bearing_deg=bearing, lo=bearing - half,
                hi=bearing + half, conf=conf, color_id=color, t=t)


def _build(lt, dt, cams, p=P, now=0.95):
    return fl.build_objects(now, lt, dt, [], p, cams=cams, cam_live=True)


def test_lidar_priority_agree_keeps_lidar_value():
    lt = _lidar({0.0: [3.0] * 10})
    objs, st = _build(lt, fl.SourceTrack(10), [_cam(fl.SRC_MONO, 3.4, 0.3)])
    (o,) = objs
    assert o['range_m'] == 3.0 and o['src_id'] == fl.SRC_LIDAR
    assert o['flags'] == fl.FLAG_SRC_LIDAR | fl.FLAG_SRC_MONO and o['color_id'] == 3, '색은 카메라 소스에서'
    assert o['conf'] == pytest.approx(1.0)
    assert st['mono_used'] == 1 and st['mono_n'] == 1


@pytest.mark.parametrize('policy,want_r,want_src,want_bits', [
    ('lidar', 3.0, fl.SRC_LIDAR, fl.FLAG_SRC_LIDAR), ('min', 1.5, fl.SRC_MONO, fl.FLAG_SRC_MONO)])
def test_lidar_priority_disagree_policy(policy, want_r, want_src, want_bits):
    lt = _lidar({0.0: [3.0] * 10})
    objs, _ = _build(lt, fl.SourceTrack(10), [_cam(fl.SRC_MONO, 1.5, 0.1, conf=0.7)],
                     p=dict(P, disagree_policy=policy))
    (o,) = objs
    assert o['range_m'] == want_r and o['src_id'] == want_src
    assert o['flags'] == fl.FLAG_DISAGREE | want_bits
    assert o['conf'] == pytest.approx(0.7 - P['disagree_penalty'])


def test_lidar_priority_min_policy_camera_farther_keeps_lidar():
    lt = _lidar({0.0: [3.0] * 10})
    objs, _ = _build(lt, fl.SourceTrack(10), [_cam(fl.SRC_TRI, 5.0, 0.1)], p=dict(P, disagree_policy='min'))
    assert objs[0]['range_m'] == 3.0 and objs[0]['src_id'] == fl.SRC_LIDAR and objs[0]['flags'] & fl.FLAG_DISAGREE


def test_lidar_gate_boundary():
    """|c − dL| ≤ 3·√(σc² + 0.05²): σc 0.1 → 허용 0.3354."""
    lt = _lidar({0.0: [3.0] * 10})
    tol = 3.0 * math.hypot(0.1, 0.05)
    ok, _ = _build(lt, fl.SourceTrack(10), [_cam(fl.SRC_MONO, 3.0 + tol - 1e-6, 0.1)])
    bad, _ = _build(lt, fl.SourceTrack(10), [_cam(fl.SRC_MONO, 3.0 + tol + 1e-6, 0.1)])
    assert not ok[0]['flags'] & fl.FLAG_DISAGREE and bad[0]['flags'] & fl.FLAG_DISAGREE


def test_no_lidar_depth_mono_inverse_variance():
    dt = _depth({0.0: [3.0] * 10}, t0=0.3)
    m = _cam(fl.SRC_MONO, 3.2, 0.3)
    objs, _ = _build(fl.SourceTrack(10), dt, [m], now=0.95)
    (o,) = objs
    sd = P['depth_sigma_k'] * 9.0
    w1, w2 = 1 / sd ** 2, 1 / 0.3 ** 2
    assert o['range_m'] == pytest.approx((w1 * 3.0 + w2 * 3.2) / (w1 + w2))
    assert o['src_id'] == fl.SRC_FUSED
    assert o['flags'] == fl.FLAG_LIDAR_STALE | fl.FLAG_SRC_DEPTH | fl.FLAG_SRC_MONO


def test_no_lidar_camera_only_sources():
    """LiDAR·뎁스 물체 없음(살아는 있음): mono 혼자 → src 4, tri 혼자 → src 5, mono+tri 합의 → src 2 가중."""
    lt = _lidar({0.0: [None] * 10})
    objs, _ = _build(lt, fl.SourceTrack(10), [_cam(fl.SRC_MONO, 4.0, 0.4, bearing=20.0),
                                              _cam(fl.SRC_TRI, 6.0, 0.3, bearing=-30.0)])
    by = {o['src_id']: o for o in objs}
    assert set(by) == {fl.SRC_MONO, fl.SRC_TRI}
    assert by[fl.SRC_MONO]['flags'] == fl.FLAG_SRC_MONO and by[fl.SRC_TRI]['flags'] == fl.FLAG_SRC_TRI
    objs, _ = _build(lt, fl.SourceTrack(10), [_cam(fl.SRC_MONO, 4.0, 0.4), _cam(fl.SRC_TRI, 4.3, 0.2)])
    (o,) = objs
    w1, w2 = 1 / 0.16, 1 / 0.04
    assert o['src_id'] == fl.SRC_FUSED and o['range_m'] == pytest.approx((w1 * 4.0 + w2 * 4.3) / (w1 + w2))
    assert o['flags'] == fl.FLAG_SRC_MONO | fl.FLAG_SRC_TRI


def test_no_lidar_all_disagree_min_rule():
    lt = _lidar({0.0: [None] * 10})
    objs, _ = _build(lt, fl.SourceTrack(10), [_cam(fl.SRC_MONO, 2.0, 0.1, conf=0.9),
                                              _cam(fl.SRC_TRI, 5.0, 0.1, conf=0.6)])
    (o,) = objs
    assert o['range_m'] == 2.0 and o['src_id'] == fl.SRC_MONO
    assert o['conf'] == pytest.approx(0.6 - P['disagree_penalty'])
    assert o['flags'] == fl.FLAG_DISAGREE | fl.FLAG_SRC_MONO


def test_no_lidar_partial_agreement_drops_outlier():
    """뎁스·단안 합의, 삼각측량 혼자 어긋남 → 둘만 가중(src 2), bit0(어긋남 있었음), 감점 없음."""
    dt = _depth({0.0: [3.0] * 10}, t0=0.3)
    objs, _ = _build(fl.SourceTrack(10), dt, [_cam(fl.SRC_MONO, 3.1, 0.3), _cam(fl.SRC_TRI, 6.0, 0.2)])
    (o,) = objs
    sd = P['depth_sigma_k'] * 9.0
    w1, w2 = 1 / sd ** 2, 1 / 0.09
    assert o['range_m'] == pytest.approx((w1 * 3.0 + w2 * 3.1) / (w1 + w2))
    assert o['src_id'] == fl.SRC_FUSED and o['flags'] & fl.FLAG_DISAGREE
    assert not o['flags'] & fl.FLAG_SRC_TRI and o['conf'] == pytest.approx(1.0)


def test_fuse_group_prefers_lowest_chi2_pair():
    ms = [dict(range_m=3.0, sigma=0.1, conf=0.8, src=fl.SRC_DEPTH, bit=8, t=0.0),
          dict(range_m=3.5, sigma=0.1, conf=0.8, src=fl.SRC_MONO, bit=16, t=0.0),
          dict(range_m=3.8, sigma=0.1, conf=0.8, src=fl.SRC_TRI, bit=32, t=0.0)]
    r, src, _, flags, bits, _ = fl.fuse_group(ms, P)
    assert bits == 16 | 32 and r == pytest.approx(3.65) and flags == fl.FLAG_DISAGREE


def test_camera_far_in_bearing_not_attached():
    lt = _lidar({0.0: [3.0] * 10})                     # 구간 −2~2
    objs, _ = _build(lt, fl.SourceTrack(10), [_cam(fl.SRC_MONO, 3.0, 0.3, bearing=7.0, half=1.0)])
    assert len(objs) == 2, '6° 밖(2 + 3 여유 < 6) → 따로 물체'
    assert {o['src_id'] for o in objs} == {fl.SRC_LIDAR, fl.SRC_MONO}


def test_camera_color_not_overwritten_by_detectors():
    lt = _lidar({0.0: [3.0] * 10})
    objs, _ = fl.build_objects(0.95, lt, fl.SourceTrack(10), [(0.5, 1)], P,
                               cams=[_cam(fl.SRC_MONO, 3.0, 0.3, color=3)], cam_live=True)
    assert objs[0]['color_id'] == 3
    objs, _ = fl.build_objects(0.95, lt, fl.SourceTrack(10), [(0.5, 1)], P,
                               cams=[_cam(fl.SRC_MONO, 3.0, 0.3, color=0)], cam_live=True)
    assert objs[0]['color_id'] == 1, '카메라 색 0 → 기존 검출기 색'


def test_both_stale_still_silent_with_cameras():
    objs, _ = fl.build_objects(5.0, _lidar({0.0: [3.0] * 10}), fl.SourceTrack(10), [], P,
                               cams=[_cam(fl.SRC_MONO, 3.0, 0.3)], cam_live=True)
    assert objs is None, '카메라만으로 발행을 다시 열지 않는다'


def test_encode_roundtrip_with_new_src_and_bits():
    lt = _lidar({0.0: [None] * 10})
    objs, _ = _build(lt, fl.SourceTrack(10), [_cam(fl.SRC_TRI, 6.0, 0.3)])
    data = fl.encode(objs)
    assert data[:3] == [1.0, 8.0, 1.0] and len(data) == 11
    assert fl.decode(data)[0]['src_id'] == 5.0 and fl.decode(data)[0]['flags'] == 32.0


# ------------------------------------------------------------ 파라미터 가드·상태

@pytest.mark.parametrize('key,val', [('use_mono', 1), ('use_mono', 'true'), ('use_triangulation', None),
                                     ('oak_cam_h_m', 0.0), ('oak_fy_px', -1.0), ('imu_pitch_sign', 0.5),
                                     ('tri_min_parallax_deg', 0.0), ('tri_min_baseline_m', 0.0),
                                     ('depth_sigma_k', 0.0), ('oak_pitch_deg', 80.0), ('tri_max_dt_s', float('nan'))])
def test_check_params_p1_invalid(key, val):
    params, issues = fl.check_params(dict(fl.DEFAULTS, **{key: val}))
    assert params[key] == fl.DEFAULTS[key] and any(key in s for s in issues)


def test_check_params_p1_bool_on():
    params, issues = fl.check_params(dict(fl.DEFAULTS, use_mono=True, use_triangulation=True))
    assert params['use_mono'] is True and params['use_triangulation'] is True and issues == []


def test_p1_switch_defaults_off():
    assert fl.DEFAULTS['use_mono'] is False and fl.DEFAULTS['use_triangulation'] is False
    assert (fl.DEFAULTS['tri_min_baseline_m'], fl.DEFAULTS['tri_min_parallax_deg'],
            fl.DEFAULTS['tri_max_dt_s']) == (0.8, 5.0, 3.0)
    assert fl.DEFAULTS['disagree_policy'] == 'lidar'


def test_format_status_camera_tokens():
    base = {'n': 2, 'conf_l': 0.8, 'conf_d': None, 'agree': None, 'lidar_ok': True, 'depth_ok': False}
    assert fl.format_status(base, 3) == 'L0.80 D- agree- lidar_ok depth_stale n=2 m=3', '꺼짐 = 옛 문자열'
    cam = {'use_mono': True, 'use_tri': False, 'oak_ok': True, 'imu_ok': False, 'rtk_fixed': True}
    s = fl.format_status(dict(base, cam=cam, mono_n=3, mono_used=1, tri_n=0, tri_used=0), 3)
    assert s == 'L0.80 D- agree- lidar_ok depth_stale n=2 m=3 oak_ok mono=1/3 tri=off imu_mount rtk_fixed'
    cam = dict(cam, use_tri=True, oak_ok=False, rtk_fixed=False)
    s = fl.format_status(dict(base, n=None, cam=cam), 3)
    assert s.endswith('oak_stale mono=- tri=- imu_mount rtk_no (발행 중단)')

"""ship_avoid 순수 로직 테스트 (ROS 불필요). 실행: tools/run_tests.sh ship_avoid"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ship_avoid.core import (  # noqa: E402
    AvoidCore, AvoidParams, LowMap, SPIN_LEFT, SPIN_RIGHT, STOP_HOLD,
    ground_plane_point, ground_plane_range, ground_plane_sigma, virtual_ranges, wls_slope)
from ship_avoid.lidar_buoys import cluster_buoys  # noqa: E402


def _core(**kw):
    c = AvoidCore(AvoidParams(**kw))
    c.set_pose(0.0, 0.0, 0.0, 0.0, yaw_rate=0.0)
    assert c.activate(0.0)
    return c


def _confirm(c, x, y, n=3):
    R = [[0.01, 0.0], [0.0, 0.01]]
    for _ in range(n):
        c.map.update(x, y, R, True, 0.0)


def test_wls_slope_linear_and_weighted_spike():
    assert abs(wls_slope([(t, 2.0 * t, 1.0) for t in range(5)]) - 2.0) < 1e-9
    s = [(t, 1.0 * t, 1.0) for t in range(6)] + [(6, 20.0, 0.01)]   # 가중치 낮은 튐 1개
    assert abs(wls_slope(s) - 1.0) < 0.2
    assert wls_slope([(0, 0, 1), (1, 1, 1)]) is None


def test_ground_plane_range():
    d = ground_plane_range(0.6, 0.0, math.tan(math.atan2(0.6, 3.0)))
    assert abs(d - 3.0) < 1e-6
    assert ground_plane_range(0.6, 0.0, -0.1) is None          # 수평선 위


def test_virtual_ranges_left_positive_and_empty():
    out = virtual_ranges(0.0, 0.0, 0.0, [(-1.0, 2.0, 0.2)])     # 북쪽 보고, 장애물은 왼쪽 앞
    a0, inc, r = out[0], out[1], out[2:]
    k = min(range(len(r)), key=lambda i: r[i])
    assert a0 + k * inc > 0                                      # 좌 양수
    assert all(math.isinf(v) for v in virtual_ranges(0, 0, 0, [])[2:])


def test_cluster_buoys_separates_and_rejects_wall():
    inc = 0.5
    rng = [math.inf] * 720
    for a in range(538, 543):        # 상대 +10° = 스캔각 90° (각 = −180 + i·0.5, 정면 80°)
        rng[a] = 2.0
    for a in range(558, 561):        # 상대 +20° = 스캔각 100°
        rng[a] = 3.0
    for a in range(400, 520):        # 60° 넓은 벽 → 부표 아님
        rng[a] = 4.0
    b = cluster_buoys(rng, -180.0, inc)
    assert len(b) == 2, b
    assert abs(b[0][0] - 10.0) < 1.0 and abs(b[1][0] - 20.0) < 1.0


def test_cluster_buoys_width_r_admits_wide_white_buoy():
    inc = 0.5
    rng = [math.inf] * 720
    for a in range(520, 553):        # 16° 폭 @ 3 m (지름 약 0.85 m)
        rng[a] = 3.0
    assert cluster_buoys(rng, -180.0, inc) == []                       # 노랑 반경만이면 버림
    b = cluster_buoys(rng, -180.0, inc, width_r=0.25)
    assert len(b) == 1 and abs(b[0][1] - 3.12) < 1e-9                  # 중심 = 표면 + buoy_r
    rng = [math.inf] * 720
    for a in range(400, 520):        # 60° 벽은 여전히 버림
        rng[a] = 4.0
    assert cluster_buoys(rng, -180.0, inc, width_r=0.25) == []


def test_lowmap_vote():
    m = LowMap()
    R = [[0.01, 0.0], [0.0, 0.01]]
    for red in (True, True, False, True):
        m.update(1.0, 2.0, R, red, 0.0)
    assert len(m.confirmed()) == 1
    m2 = LowMap()
    for red in (False, False, True):
        m2.update(1.0, 2.0, R, red, 0.0)
    assert m2.confirmed() == []


def _synth(D, beta_deg, h=0.6, pitch=0.0, roll=0.0):
    """기하 합성: 수면 위 점(거리 D, 방위 β 좌 +)을 피치·롤 자세 카메라에 투영 → (bearing_left_deg, v_norm).
    수평 좌표 (오른쪽, 아래, 앞) = (−D sinβ, h, D cosβ) 를 피치 역회전 → 롤 역회전 → 정규 좌표."""
    b = math.radians(beta_deg)
    r_, dn, fw = -D * math.sin(b), h, D * math.cos(b)
    p, q = math.radians(pitch), math.radians(roll)
    y1 = dn * math.cos(p) - fw * math.sin(p)
    z = dn * math.sin(p) + fw * math.cos(p)
    x = r_ * math.cos(q) + y1 * math.sin(q)
    y = -r_ * math.sin(q) + y1 * math.cos(q)
    return -math.degrees(math.atan(x / z)), y / z


def test_ground_plane_point_front_side_closed_form():
    # 정면: 예전 식 h / tan(θ + atan v) 와 같다(피치 5°)
    v = math.tan(math.atan2(0.6, 3.0) - math.radians(5.0))
    assert abs(ground_plane_range(0.6, 5.0, v) - 3.0) < 1e-9
    # 측면 20°·32° 피치·롤 0: v = h / (D cosβ) (앞 거리로 투영), 거리 = D·(옛 식은 D cosβ 로 짧게 냈다)
    for beta in (20.0, -32.0, 32.0):
        v = 0.6 / (4.0 * math.cos(math.radians(beta)))
        d, brg = ground_plane_point(0.6, 0.0, 0.0, beta, v)
        assert abs(d - 4.0) < 1e-9 and abs(brg - beta) < 1e-9


def test_ground_plane_point_matches_synthesis_with_pitch_roll():
    for D, beta, pitch, roll in [(3.0, 0.0, 8.0, 0.0), (4.0, 20.0, 5.0, 0.0), (2.5, 32.0, 0.0, 3.0),
                                 (5.0, -32.0, 6.0, 3.0), (3.5, 20.0, 4.0, -3.0)]:
        brg_cam, v = _synth(D, beta, pitch=pitch, roll=roll)
        d, brg = ground_plane_point(0.6, pitch, roll, brg_cam, v)
        assert abs(d - D) < 1e-6 and abs(brg - beta) < 1e-6, (D, beta, pitch, roll, d, brg)
    brg_cam, v = _synth(3.0, 32.0, roll=3.0)
    assert abs(brg_cam - 32.0) > 0.1, '롤이 있으면 카메라 방위 ≠ 수평 방위(다시 내는 이유)'


def test_ground_plane_pitch_error_closed_form():
    D, h, delta = 3.0, 0.6, 1.0
    _, v = _synth(D, 0.0, h=h, pitch=0.0)
    d = ground_plane_range(h, delta, v)                      # 피치 1° 틀림
    assert abs(d - h / math.tan(math.atan2(h, D) + math.radians(delta))) < 1e-9
    assert 0.2 < D - d < 0.3                                 # 3 m 에서 약 0.25 m(계약서 A-1 의 '약 0.4 m' 보다 작다)
    sig = ground_plane_sigma(h, 0.0, 0.0, 0.0, v, delta, 0.0, 0.0)
    assert abs(sig - (D - d)) / (D - d) < 0.1                # 1차 전파 ≈ 실제 오차(1°)


def test_ground_plane_horizon_none():
    assert ground_plane_range(0.6, 0.0, -0.1) is None          # 수평선 위
    assert ground_plane_range(0.6, 0.0, math.tan(math.radians(0.4))) is None   # 0.5° 안 = 수평선 근처
    assert ground_plane_range(0.6, -5.0, 0.05) is None         # 위로 든 카메라
    assert ground_plane_point(0.6, 0.0, 0.0, 10.0, float('nan')) is None
    assert ground_plane_sigma(0.6, 0.0, 0.0, 0.0, -0.1, 2.0, 0.004, 0.05) == math.inf


def test_ground_plane_sigma_pitch_only_equals_old_formula_and_monotone():
    h = 0.6
    for D in (1.5, 3.0, 6.0):
        _, v = _synth(D, 0.0, h=h)
        phi = math.atan2(h, D)
        old = h / math.sin(phi) ** 2 * math.radians(2.0)
        s = ground_plane_sigma(h, 0.0, 0.0, 0.0, v, 2.0, 0.0, 0.0)
        assert abs(s - old) / old < 0.01, (D, s, old)        # 롤 항은 정면에서 0
    sig = [ground_plane_sigma(h, 0.0, 0.0, 20.0, _synth(D, 20.0)[1], 0.5, 0.004, 0.05) for D in (1, 2, 3, 5, 8)]
    assert all(b > a for a, b in zip(sig, sig[1:])), sig    # 멀수록 커진다
    _, v = _synth(3.0, 20.0)
    base = ground_plane_sigma(h, 0.0, 0.0, 20.0, v, 0.5, 0.004, 0.05)
    assert ground_plane_sigma(h, 0.0, 0.0, 20.0, v, 2.0, 0.004, 0.05) > base
    assert ground_plane_sigma(h, 0.0, 0.0, 20.0, v, 0.5, 0.008, 0.05) > base
    assert ground_plane_sigma(h, 0.0, 0.0, 20.0, v, 0.5, 0.004, 0.10) > base
    s_h = ground_plane_sigma(h, 0.0, 0.0, 0.0, _synth(3.0, 0.0)[1], 0.0, 0.0, 0.06)
    assert abs(s_h - 3.0 * 0.06 / h) < 1e-6                  # σd/d = σh/h
    assert ground_plane_sigma(h, 0.0, 0.0, 32.0, _synth(3.0, 32.0)[1], 0.0, 0.0, 0.0) == 0.0


def _det(D, beta, cls=0, rng=float('nan'), h=0.6, brg_err=0.0):
    brg, v = _synth(D, beta, h=h)
    return (cls, brg + brg_err, v, rng, 1.0, 0.05)


def _course_buoys(width=3.0, n=10):
    return [(sx * width / 2, float(k)) for sx in (-1, 1) for k in range(n)]


def test_white_detection_on_lidar_buoy_confirms_at_lidar_position():
    """흰 검출 + LiDAR 부표 → LiDAR 위치(중심 obstacle_r 보정) 트랙 확정. 방위 4° 어긋나고 단안 거리 틀려도."""
    c = _core()
    walls = _course_buoys()
    c.set_buoys(walls + [(0.3, 3.0)])
    c.capture_ok = True
    used = c.add_detections(0.0, 0.0, 0.0, 0.0, [_det(2.7, -5.7, brg_err=4.0)] * 3)  # 단안 0.3 m 짧음
    assert used == 3
    conf = c.map.confirmed()
    assert len(conf) == 1 and conf[0].lidar == 3
    dl = math.hypot(0.3, 3.0)
    k = (c.p.obstacle_r - c.p.buoy_r) / dl
    assert abs(conf[0].x - 0.3 * (1 + k)) < 1e-6 and abs(conf[0].y - 3.0 * (1 + k)) < 1e-6
    assert c.obstacle_radius(conf[0]) == c.p.obstacle_r == 0.25
    # ② 그 자리 LiDAR 부표는 벽·부표 목록에서 빠지고 벽은 노랑만으로
    assert (0.3, 3.0) not in c.free_buoys() and len(c.free_buoys()) == len(walls)
    assert [o for o in c.obstacles() if o[3] == 'buoy' and o[:2] == (0.3, 3.0)] == []
    assert c.walls(3.0) == (-1.5, 1.5)
    assert all(math.hypot(px - 0.3, py - 3.0) > 0.5 for px, py, _ in c.wall_points())


def test_unconfirmed_white_lidar_buoy_still_counts_as_buoy():
    c = _core()
    c.set_buoys([(0.3, 3.0)])
    c.capture_ok = True
    c.add_detections(0.0, 0.0, 0.0, 0.0, [_det(3.0, -5.7)] * 2)          # 2회 = 아직 미확정
    assert c.map.confirmed() == [] and c.free_buoys() == [(0.3, 3.0)]


def test_white_without_lidar_is_ignored_by_default_and_low_r_when_allowed():
    c = _core()
    c.set_buoys(_course_buoys())
    c.capture_ok = True
    assert c.add_detections(0.0, 0.0, 0.0, 0.0, [_det(3.0, 0.0)] * 3) == 0    # ④ LiDAR 연관 없음 → 버림
    assert c.map.tracks == []
    c = _core(obstacle_requires_lidar=False)
    c.set_buoys(_course_buoys())
    c.capture_ok = True
    assert c.add_detections(0.0, 0.0, 0.0, 0.0, [_det(3.0, 0.0, rng=3.0)] * 3) == 3   # 예전 경로(카메라 거리)
    conf = c.map.confirmed()
    assert len(conf) == 1 and conf[0].lidar == 0 and abs(conf[0].y - 3.0) < 1e-6
    assert c.obstacle_radius(conf[0]) == c.p.low_r


def test_yellow_on_lidar_buoy_is_not_obstacle_and_stays_wall():
    c = _core()
    c.set_buoys(_course_buoys())
    c.capture_ok = True
    beta = math.degrees(math.atan2(1.5, 3.0))                     # 왼쪽 벽 (−1.5, 3)
    used = c.add_detections(0.0, 0.0, 0.0, 0.0, [_det(math.hypot(1.5, 3.0), beta, cls=2)] * 3
                            + [_det(math.hypot(1.5, 3.0), beta, cls=0)])   # 오분류 1회
    assert used == 4
    assert c.map.confirmed() == [] and c.map.tracks[0].lidar == 4  # 노랑 다수결 → 미확정
    assert (-1.5, 3.0) in c.free_buoys()


def test_association_uses_range_to_split_same_bearing_buoys():
    """흰 장애물(4 m)과 같은 방위 뒤쪽 노란 벽(6.3 m) — 거리로 가른다. 방위 허용 밖이면 연관 안 함."""
    c = _core()
    a = math.atan2(1.0, 4.0)
    near = (4.0 * math.tan(a) * 1.0, 4.0)
    far = (1.5, 1.5 / math.tan(a))
    c.set_buoys([far, near])
    c.capture_ok = True
    beta = -math.degrees(a)
    c.add_detections(0.0, 0.0, 0.0, 0.0, [_det(math.hypot(*near), beta, rng=math.hypot(*near) - 0.12)] * 3)
    conf = c.map.confirmed()
    assert len(conf) == 1 and abs(conf[0].y - near[1]) < 0.2
    c2 = _core(obstacle_match_deg=3.0)
    c2.set_buoys([near])
    c2.capture_ok = True
    assert c2.add_detections(0.0, 0.0, 0.0, 0.0, [_det(math.hypot(*near), beta, brg_err=4.0)] * 3) == 0


def test_cells_and_out_in_out_run_on_lidar_white_track():
    """⑤ 칸 세기·out-in-out 이 LiDAR 연관 흰 트랙으로 동작 (반경 obstacle_r)."""
    c = _core()
    c.state = "DRIVE"
    c.pending = False
    c.set_buoys(_course_buoys(n=12) + [(0.8, 6.0)])
    c.capture_ok = True
    dl = math.hypot(0.8, 6.0)
    c.add_detections(0.0, 0.0, 0.0, 0.0, [_det(dl, -math.degrees(math.atan2(0.8, 6.0)))] * 3)
    tr = c.map.confirmed()[0]
    assert tr.lidar
    nxt = c._next_obstacle(0.0)
    assert nxt is not None and nxt[0] is tr
    lt = c.target_lateral(1.0, 0.0, 0.0)          # 멀리서 = out = 지나갈 편(왼쪽) 벽
    assert lt < 0 and tr.side == -1
    lp = c.target_lateral(tr.y, 5.0, 0.0)          # 장애물 옆 = in
    assert abs(lp - (tr.x - (0.25 + 0.285 + 0.45))) < 1e-6   # 정점 = 장애물 l − (obstacle_r + half_w + clear_low)
    c.set_pose(1.0, 0.0, 6.0, 0.0, yaw_rate=0.0)
    c._update_cells()
    assert c.cells == 0                           # 옆 = 아직
    c.set_pose(2.0, 0.0, tr.y + 0.83 + 0.25 + 0.05, 0.0, yaw_rate=0.0)
    c._update_cells()
    assert c.cells == 1 and c.pending and tr.passed
    vo = c.virtual_obstacles(0.2)
    assert any(abs(r - (0.25 + 0.2)) < 1e-9 for _, _, r in vo)


def test_add_detections_uses_roll_for_range_and_bearing():
    c = _core()
    c.capture_ok = True
    c.set_roll(3.0)
    c.set_buoys([])
    c.p.obstacle_requires_lidar = False
    brg, v = _synth(3.0, 30.0, roll=3.0)
    c.add_detections(0.0, 0.0, 0.0, 0.0, [(0, brg, v, float('nan'), 1.0, 0.0)])
    tr = c.map.tracks[0]
    assert abs(math.hypot(tr.x, tr.y) - 3.0) < 1e-6
    assert abs(math.degrees(math.atan2(-tr.x, tr.y)) - 30.0) < 1e-6   # 왼쪽 30° = 세계 서쪽
    c.reset_keep_pose()
    assert c.roll == 3.0


def test_start_doridori_right_first():
    c = _core()
    cmds = []
    for k in range(1, 40):
        t = k * 0.1
        c.set_pose(t, 0.0, 0.0, 0.0, yaw_rate=0.0)
        cmds.append(c.step(t))
    assert c.state == "DORI"
    spins = [x for x in cmds if x in (SPIN_LEFT, SPIN_RIGHT)]
    assert spins and spins[0] == SPIN_RIGHT


def test_spin_aborts_when_not_clear():
    c = _core()
    for k in range(1, 40):
        c.set_pose(k * 0.1, 0.0, 0.0, 0.0, yaw_rate=0.0)
        if c.step(k * 0.1) in (SPIN_LEFT, SPIN_RIGHT):
            break
    c.set_buoys([(0.5, 0.0)])                    # 회전 원 안에 부표가 들어옴
    assert c.step(5.0) == STOP_HOLD
    assert c.state == "DRIVE" and c.n_skip == 1


def test_cell_counts_when_low_obstacle_passed():
    c = _core()
    c.state = "DRIVE"
    c.pending = False
    _confirm(c, 0.0, 2.0)
    c.set_pose(1.0, 0.0, 1.0, 0.0, yaw_rate=0.0)
    c._update_cells()
    assert c.cells == 0
    c.set_pose(2.0, 0.0, 3.2, 0.0, yaw_rate=0.0)
    c._update_cells()
    assert c.cells == 1 and c.pending


def test_out_side_is_pass_side_wall():
    c = _core()
    c.state = "DRIVE"
    _confirm(c, 0.8, 6.0)                         # 오른쪽 장애물 → 왼쪽으로 지나감
    lt = c.target_lateral(1.0, 0.0, 0.0)          # 멀리서 = out
    assert lt < 0                                 # 장애물 반대(지나갈 편) 벽 쪽
    lp = c.target_lateral(6.0, 5.0, 0.0)          # 장애물 옆 = in
    assert lp <= 0.8 - (0.2 + 0.285 + 0.45) + 1e-6

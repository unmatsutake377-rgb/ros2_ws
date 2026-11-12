"""ship_avoid 순수 로직 테스트 (ROS 불필요). 실행: tools/run_tests.sh ship_avoid"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ship_avoid.core import (  # noqa: E402
    AvoidCore, AvoidParams, LowMap, SPIN_LEFT, SPIN_RIGHT, STOP_HOLD,
    ground_plane_range, virtual_ranges, wls_slope)
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


def test_detection_matching_lidar_buoy_is_not_low_obstacle():
    c = _core()
    c.set_buoys([(0.0, 3.0)])
    c.capture_ok = True
    used = c.add_detections(0.0, 0.0, 0.0, 0.0, [(0, 0.0, 0.0, 3.0, 1.0)] * 3)
    assert used == 0 and c.map.tracks == []
    used = c.add_detections(0.0, 0.0, 0.0, 0.0, [(0, 0.0, 0.0, 5.0, 1.0)] * 3)
    assert used == 3 and len(c.map.confirmed()) == 1


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

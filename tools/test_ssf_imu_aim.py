"""tools/ssf_imu_aim.py 순수 함수 테스트 (ROS 불필요). python3 -m pytest -q tools/test_ssf_imu_aim.py"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ssf_imu_aim as m  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
T0 = (35.1847763, 128.5653979)
T1 = (35.1846002, 128.5651050)


def test_bearing_lane1_matches_planar():
    # 공유 레인 1 T0→T1 은 평면 근사로 234°, 33.0 m (11-14 계산)
    assert abs(m.bearing_deg(*T0, *T1) - 234.0) < 1.0
    assert abs(m.dist_m(*T0, *T1) - 33.0) < 0.3


def test_bearing_cardinals():
    assert abs(m.bearing_deg(35.0, 128.0, 35.001, 128.0) - 0.0) < 1e-6
    assert abs(m.bearing_deg(35.0, 128.0, 35.0, 128.001) - 90.0) < 0.01


@pytest.mark.parametrize('target,raw,inv', [(234.0, 12.5, False), (5.0, 350.0, False),
                                            (234.0, 12.5, True), (0.0, 0.0, False)])
def test_offset_reproduces_target_like_yaw_mux(target, raw, inv):
    off = m.mount_offset(target, raw, inv)
    y = -raw if inv else raw
    got = (y + off) % 360.0      # yaw_mux apply_corrections 식(편각 0)
    assert min(abs(got - target), 360 - abs(got - target)) < 0.06


def test_offset_example():
    assert m.mount_offset(234.0, 12.5, False) == 221.5


def test_circ_mean_wraps():
    mean, std = m.circ_mean_std([359.0, 1.0, 0.0])
    assert min(mean, 360 - mean) < 1e-6 and std < 1.0
    assert m.circ_mean_std([0.0, 90.0, 180.0, 270.0])[1] > 30


@pytest.mark.parametrize('boat', ['A', 'B'])
def test_set_offset_keeps_comment_and_float(boat):
    p = os.path.join(REPO, 'src', 'ssf_heading', 'config', f'ssf_heading_{boat}.yaml')
    text = open(p, encoding='utf-8').read()
    new = m.set_offset(text, 221)
    line = [ln for ln in new.splitlines() if 'mount_offset_deg:' in ln and not ln.strip().startswith('#')][0]
    assert 'mount_offset_deg: 221.0' in line and '#' in line
    assert new.count('\n') == text.count('\n')
    assert m.read_invert(text) is False
    import yaml
    v = yaml.safe_load(new)['yaw_mux']['ros__parameters']['mount_offset_deg']
    assert isinstance(v, float) and v == 221.0


def test_lane_file_prefers_own(tmp_path):
    shared = m.lane_file('A', '1', home=str(tmp_path))
    assert shared.endswith(os.path.join('tournament_lanes', 'waypoints_tournament_1.yaml'))
    (tmp_path / 'ssf_logs').mkdir()
    own = tmp_path / 'ssf_logs' / 'waypoints_tournament_A_1.yaml'
    own.write_text('waypoints: []\n')
    assert m.lane_file('A', '1', home=str(tmp_path)) == str(own)
    with pytest.raises(FileNotFoundError):
        m.lane_file('A', 'zz', home=str(tmp_path))


def test_last_wp_shared_lane():
    lat, lon = m.last_wp(m.lane_file('A', '1', home='/nonexistent'))
    assert math.isclose(lat, T1[0]) and math.isclose(lon, T1[1])

"""배 모델·월드 생성기 테스트 — XML 이 열리고, 우리 스택 계약(토픽·조인트·각도)과 맞는지. ROS 불필요."""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, PKG)
from ssf_vrx import make_boat, make_world  # noqa: E402

WP_FILE = make_world.default_waypoints_path()


# ───────────────── 배 모델 ─────────────────
@pytest.mark.parametrize('boat', ['A', 'B'])
def test_boat_sdf_parses_and_has_contract(boat):
    root = ET.fromstring(make_boat.boat_sdf(boat))
    model = root.find('model')
    assert model.get('name') == f'ssf_boat_{boat.lower()}'
    sensors = {s.get('name'): s for s in model.iter('sensor')}
    assert set(sensors) == {'lidar', 'oak', 'imu', 'navsat'}
    # 센서 토픽은 launch 의 브릿지 표와 같아야 한다
    assert sensors['lidar'].find('topic').text == '/ssf/scan'
    assert sensors['oak'].find('topic').text == '/ssf/oak/rgb/image_raw'
    assert sensors['imu'].find('topic').text == '/ssf/imu'
    assert sensors['navsat'].find('topic').text == '/ssf/navsat'
    # LiDAR: 스캔 80° 가 뱃머리 → 센서 yaw -80°
    yaw = float(sensors['lidar'].find('pose').text.split()[5])
    assert yaw == pytest.approx(-math.radians(80.0), abs=1e-4)
    assert int(sensors['lidar'].find('.//samples').text) == 1440
    # 카메라 화각 = vision.yaml hfov_deg 63.9
    hfov = float(sensors['oak'].find('.//horizontal_fov').text)
    assert math.degrees(hfov) == pytest.approx(63.9, abs=0.01)
    assert sensors['oak'].find('.//width').text == '640'
    assert sensors['oak'].find('.//height').text == '360'
    # 추진기 조인트 = sim_bridge DEFAULT_JOINTS 와 같아야 한다
    joints = {j.get('name') for j in model.findall('joint')}
    thrusters = [p for p in model.findall('plugin') if 'thruster' in p.get('filename')]
    joint_names = [p.find('joint_name').text for p in thrusters]
    expected = {'A': ['left_engine_propeller_joint', 'right_engine_propeller_joint'],
                'B': ['rear_left_engine_propeller_joint', 'rear_right_engine_propeller_joint',
                      'front_left_engine_propeller_joint', 'front_right_engine_propeller_joint']}
    assert sorted(joint_names) == sorted(expected[boat])
    assert set(joint_names) <= joints
    for p in thrusters:
        assert float(p.find('max_thrust_cmd').text) == pytest.approx(47.0)
        assert float(p.find('min_thrust_cmd').text) == pytest.approx(-35.0)
    # 부력(Surface) 2개 + 수력 1개 + odometry 1개
    names = [p.get('name') for p in model.findall('plugin')]
    assert names.count('vrx::Surface') == 2
    assert 'vrx::SimpleHydrodynamics' in names
    assert 'gz::sim::systems::OdometryPublisher' in names
    mass = float(model.find('link/inertial/mass').text)
    assert mass == pytest.approx({'A': 34.0, 'B': 30.0}[boat])


def test_boat_joints_match_sim_bridge_defaults():
    """make_boat 의 조인트 이름과 sim_bridge 의 기본 토픽이 어긋나면 배가 안 움직인다."""
    src = open(os.path.join(PKG, 'ssf_vrx', 'sim_bridge.py'), encoding='utf-8').read()
    for boat in ('A', 'B'):
        for name, _x, _y in make_boat.BOATS[boat]['thrusters']:
            assert f"'{name}_engine_propeller_joint'" in src, name


def test_launch_bridge_list_matches_model_joints():
    src = open(os.path.join(PKG, 'launch', 'ssf_vrx.launch.py'), encoding='utf-8').read()
    for boat in ('A', 'B'):
        for name, _x, _y in make_boat.BOATS[boat]['thrusters']:
            assert f"'{name}_engine_propeller_joint'" in src, name
    for topic in ('/ssf/scan', '/ssf/oak/rgb/image_raw', '/ssf/navsat', '/ssf/imu',
                  '/ssf/odometry'):
        assert topic in src


# ───────────────── 월드 ─────────────────
@pytest.fixture(scope='module')
def course():
    assert os.path.isfile(WP_FILE), WP_FILE
    return make_world.build_course(make_world.load_wps(WP_FILE), seed=1, bay=0)


def test_course_layout(course):
    kinds = [b['kind'] for b in course['buoys']]
    assert sum(k.startswith('gate') for k in kinds) == 8
    assert 'hold' in kinds
    assert sum(k.startswith('search_') for k in kinds) == 3
    assert sum(k.startswith('obst_') for k in kinds) >= 12
    assert course['dock']['bay'] == 0
    assert 0.0 <= course['psi0'] < 360.0
    # 같은 시드 → 같은 코스
    again = make_world.build_course(make_world.load_wps(WP_FILE), seed=1, bay=0)
    assert again['buoys'] == course['buoys']


def test_gate_red_is_port(course):
    """빨강 = 진행 방향 왼쪽(좌현). ship_gate red_on_port=true 와 같아야 한다."""
    g = course['gates'][0]
    c, red = g['c'], g['red']
    wps = course['wps']
    u = (wps[1]['x'] - wps[0]['x'], wps[1]['y'] - wps[0]['y'])
    cross = u[0] * (red[1] - c[1]) - u[1] * (red[0] - c[0])
    assert cross > 0     # 진행 방향 기준 왼쪽


def test_world_sdf_parses(course, tmp_path):
    sdf = make_world.world_sdf('kaboat_test', course, 'A', wave_gain=0.0, wind_mps=0.0)
    root = ET.fromstring(sdf)
    world = root.find('world')
    assert world.get('name') == 'kaboat_test'
    sc = world.find('spherical_coordinates')
    assert float(sc.find('latitude_deg').text) == pytest.approx(course['origin'][0])
    assert float(sc.find('longitude_deg').text) == pytest.approx(course['origin'][1])
    uris = [i.find('uri').text for i in world.findall('include')]
    assert 'coast_waves' in uris and 'ssf_boat_a' in uris
    assert sum(u.startswith('kaboat_buoy_') or u.startswith('kaboat_obst_') for u in uris) \
        == len(course['buoys'])
    assert world.find("model[@name='kaboat_dock']") is not None
    assert world.find("plugin[@name='vrx::PublisherPlugin']") is not None
    assert world.find("plugin[@name='vrx::USVWind']") is None      # wind 0 → 없음
    with_wind = ET.fromstring(make_world.world_sdf('w', course, 'B', wind_mps=3.0)).find('world')
    assert with_wind.find("plugin[@name='vrx::USVWind']") is not None


def test_buoy_models_write_and_parse(tmp_path):
    dirs = make_world.write_buoy_models(str(tmp_path))
    assert len(dirs) == 2 * len(make_world.COLORS)
    for d in dirs:
        ET.parse(os.path.join(d, 'model.sdf'))
        ET.parse(os.path.join(d, 'model.config'))


def test_main_writes_world_and_sidecar(tmp_path):
    make_world.main(['--boat', 'A', '--seed', '3', '--bay', '1', '--name', 'kt',
                     '--worlds-dir', str(tmp_path / 'w'), '--models-dir', str(tmp_path / 'm')])
    ET.parse(tmp_path / 'w' / 'kt.sdf')
    side = json.load(open(tmp_path / 'w' / 'kt.json', encoding='utf-8'))
    assert side['dock']['bay'] == 1 and side['seed'] == 3
    assert os.path.isdir(tmp_path / 'm' / 'kaboat_buoy_red')

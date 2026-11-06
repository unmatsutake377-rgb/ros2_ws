"""KABOAT 코스 월드 생성기 — waypoints.yaml → Gazebo(VRX) 월드 SDF + 부표/도크 모델.

  python3 -m ssf_vrx.make_world --boat A --seed 1 [--waypoints PATH] [--wave-gain 0.0] [--wind 0.0]
    → worlds/kaboat_a.sdf (+ kaboat_a.json 부표 좌표), models/kaboat_buoy_<색>/ (처음 한 번)

배치 규칙은 tools/sim/full_course_sim.py build_world 와 같다(같은 시드면 같은 코스):
  · 게이트 4개: WP0→WP1 선분 위 1/8·3/8·5/8·7/8, 폭 3~6 m 무작위, 빨강 = 진행 방향 왼쪽.
  · 위치유지 흰 부표 = WP2, 탐색 부표 = mode 3 WP(라벨의 색), 도크 = 첫 mode 7 WP(정면은 직전 WP 쪽).
  · 장애물(주황) = WP6→WP7 사이, 도크→WP10 사이에 무작위.
좌표: 월드 원점 = WP0(ENU, x 동 y 북). Gazebo navsat 이 이 원점 기준 위경도를 내므로 waypoints.yaml 을
그대로 쓰면 실제 코스와 같은 위경도가 나온다(월드 전체 수백 m 라 평면 근사 오차 수 cm).

부표: KABOAT 규격(높이 1.0 m, 폭 0.5 m) 원기둥, **static**(물리 없음) — 부딪히면 배가 밀리지만 부표는 안 움직인다.
떠다니는 부표(PolyhedraBuoyancyDrag)는 나중 과제.
도크: 정면 벽 3 m(1 m 칸 3개) + 칸막이 0.5 m 4개(dock_sim 과 같은 규격), 각 칸 뒤벽에 표식(색·형상).
"""
import argparse
import json
import math
import os
import random

import yaml

R_EARTH = 6378137.0
COLORS = {
    'red': (0.85, 0.05, 0.05), 'green': (0.05, 0.6, 0.1), 'white': (0.95, 0.95, 0.95),
    'black': (0.05, 0.05, 0.05), 'orange': (1.0, 0.45, 0.0), 'yellow': (0.95, 0.85, 0.05),
    'blue': (0.05, 0.2, 0.85),
}
BUOY_R, BUOY_H = 0.25, 1.0          # 폭 0.5 m, 높이 1.0 m
OBST_R, OBST_H = 0.20, 0.60
DOCK_BAY_W, DOCK_DIV_L, DOCK_T = 1.0, 0.5, 0.15
DEFAULT_MARKERS = {-1: ('green', 'Triangle'), 0: ('red', 'Square'), 1: ('blue', 'Circle')}


# ───────────────────────── 좌표 ─────────────────────────
class Geo:
    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0

    def xy(self, lat, lon):
        return (math.radians(lon - self.lon0) * R_EARTH * math.cos(math.radians(self.lat0)),
                math.radians(lat - self.lat0) * R_EARTH)


def load_wps(path):
    with open(path, encoding='utf-8') as f:
        raw = yaml.safe_load(f)['waypoints']
    return [dict(label=str(w.get('구역', '')), lat=float(w['lat']), lon=float(w['lon']),
                 mode=int(w['mode']), dwell=float(w['dwell'])) for w in raw]


def _unit(v):
    n = math.hypot(v[0], v[1])
    return (v[0] / n, v[1] / n)


def build_course(wps, seed, bay=None):
    """full_course_sim.build_world 와 같은 배치. 반환: dict(buoys, gates, dock, start, psi0)."""
    rng = random.Random(seed)
    geo = Geo(wps[0]['lat'], wps[0]['lon'])
    P = [geo.xy(w['lat'], w['lon']) for w in wps]
    buoys = []

    def add(p, color, kind):
        buoys.append(dict(x=float(p[0]), y=float(p[1]), color=color, kind=kind))

    # 게이트
    d01 = (P[1][0] - P[0][0], P[1][1] - P[0][1])
    L01 = math.hypot(*d01)
    u = _unit(d01)
    left = (-u[1], u[0])
    gates = []
    for k in range(4):
        s = L01 * (k + 0.5) / 4.0
        c = (P[0][0] + u[0] * s, P[0][1] + u[1] * s)
        w = rng.uniform(3.0, 6.0)
        red = (c[0] + left[0] * w / 2, c[1] + left[1] * w / 2)
        green = (c[0] - left[0] * w / 2, c[1] - left[1] * w / 2)
        gates.append(dict(c=c, w=w, red=red, green=green))
        add(red, 'red', f'gate{k}')
        add(green, 'green', f'gate{k}')
    # 미션 부표
    modes = [w['mode'] for w in wps]
    i2 = modes.index(2)
    add(P[i2], 'white', 'hold')
    col_kw = {'초록': 'green', '빨강': 'red', '하양': 'white', '흰': 'white',
              '노랑': 'yellow', '주황': 'orange', '파랑': 'blue'}
    for i, w in enumerate(wps):
        if w['mode'] == 3:
            col = next((v for kk, v in col_kw.items() if kk in w['label']), 'red')
            add(P[i], col, f'search_{col}')
    # 도크: 정면 벽 중심 = 첫 mode 7 WP, 정면(법선 n)은 직전 WP 쪽. t = n 을 시계로 90° 돌린 것(오른쪽)
    i7 = modes.index(7)
    O = P[i7]
    n = _unit((P[i7 - 1][0] - O[0], P[i7 - 1][1] - O[1]))
    t = (n[1], -n[0])
    bay_sel = rng.choice((-1, 0, 1)) if bay is None else int(bay)
    dock = dict(O=O, n=n, t=t, bay=bay_sel)

    # 장애물
    def scatter(a, b, t_lo, t_hi, tag, n_min=6, n_max=10):
        d = (b[0] - a[0], b[1] - a[1])
        L = math.hypot(*d)
        uu = _unit(d)
        ll = (-uu[1], uu[0])
        N = rng.randint(n_min, n_max)
        placed = tries = 0
        while placed < N and tries < 2000:
            tries += 1
            s = rng.uniform(t_lo, t_hi)
            off = rng.uniform(-3.0, 3.0)
            p = (a[0] + uu[0] * s + ll[0] * off, a[1] + uu[1] * s + ll[1] * off)
            if all(math.hypot(p[0] - q['x'], p[1] - q['y']) >= 1.5 for q in buoys) and \
               math.hypot(p[0] - O[0], p[1] - O[1]) > 5.0:
                add(p, 'orange', tag)
                placed += 1

    i5 = [i for i, m in enumerate(modes) if m == 5]
    a, b = P[i5[0]], P[i5[1]]
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    scatter(a, b, 0.1 * L, 0.9 * L, 'obst_5')
    i8 = modes.index(8)
    a, b = P[i7], P[i8]
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    scatter(a, b, max(0.0, L - 20.0), L - 3.0, 'obst_8')

    # 출발: WP0 에서 진행 반대 방향 8 m, 좌우 ±1 m, 헤딩 ±20°
    back = (-u[0], -u[1])
    lat_off = rng.uniform(-1.0, 1.0)
    s0 = (P[0][0] + back[0] * 8.0 + left[0] * lat_off, P[0][1] + back[1] * 8.0 + left[1] * lat_off)
    brg = math.degrees(math.atan2(P[0][0] - s0[0], P[0][1] - s0[1]))   # 나침반(0=북, 시계)
    psi0 = (brg + rng.uniform(-20.0, 20.0)) % 360.0
    return dict(origin=(wps[0]['lat'], wps[0]['lon']), buoys=buoys, gates=gates, dock=dock,
                start=s0, psi0=psi0, wps=[dict(x=p[0], y=p[1], mode=m) for p, m in zip(P, modes)])


# ───────────────────────── SDF 조각 ─────────────────────────
def _rgb(c):
    r, g, b = COLORS[c]
    return f'<ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'


def buoy_model_sdf(color, radius=BUOY_R, height=BUOY_H, name=None):
    name = name or f'kaboat_buoy_{color}'
    return f"""<?xml version="1.0" ?>
<!-- 생성 파일: ssf_vrx.make_world. KABOAT 부표 {color}: 높이 {height} m, 폭 {2 * radius} m, static -->
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <pose>0 0 {height / 2 - 0.2:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>{radius}</radius><length>{height}</length>
        </cylinder></geometry>
      </collision>
      <visual name="visual">
        <pose>0 0 {height / 2 - 0.2:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>{radius}</radius><length>{height}</length>
        </cylinder></geometry>
        <material>{_rgb(color)}</material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _model_config(name, desc):
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>SSF</name></author>
  <description>{desc}</description>
</model>
"""


def write_buoy_models(models_dir):
    out = []
    for c in COLORS:
        kinds = ((f'kaboat_buoy_{c}', BUOY_R, BUOY_H), (f'kaboat_obst_{c}', OBST_R, OBST_H))
        for nm, r, h in kinds:
            d = os.path.join(models_dir, nm)
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, 'model.sdf'), 'w', encoding='utf-8') as f:
                f.write(buoy_model_sdf(c, r, h, nm))
            with open(os.path.join(d, 'model.config'), 'w', encoding='utf-8') as f:
                f.write(_model_config(nm, f'KABOAT {nm} (생성 파일)'))
            out.append(d)
    return out


def _marker_visual(idx, color, shape, pose):
    """표식: 칸 뒤벽에 붙는 0.5 m 도형. 정면 = 도크 법선(-n 쪽에서 봄)."""
    mat = f'<material>{_rgb(color)}</material>'
    if shape == 'Square':
        geom = '<box><size>0.03 0.5 0.5</size></box>'
    elif shape == 'Circle':
        geom = '<cylinder><radius>0.25</radius><length>0.03</length></cylinder>'
        pose = pose + ' 0 1.5708 0'
    else:   # Triangle — VRX 의 symbol_triangle 메시(vrx_gz 모델, 리소스 경로에 있음)
        geom = '<mesh><uri>model://symbol_triangle/mesh/triangle.dae</uri>' \
               '<scale>0.5 0.5 0.5</scale></mesh>'
        pose = pose + ' 1.5708 0 1.5708'
    if shape != 'Triangle' and shape != 'Circle':
        pose = pose + ' 0 0 0'
    return (f'<visual name="marker_{idx}"><pose>{pose}</pose>'
            f'<geometry>{geom}</geometry>{mat}</visual>')


def dock_sdf(dock, markers):
    """도크 1개 모델(static). 모델 프레임: x = 법선 n(배가 들어오는 방향의 반대, 즉 뒤벽 → 바깥),
    y = t(오른쪽). 월드에서 yaw 로 돌려 놓는다. 뒤벽은 x=0, 칸막이는 x 0~+0.5(바깥쪽)."""
    parts = []
    # 뒤벽(정면 벽): 길이 3 m(y), 두께 0.15(x), 높이 0.6, 수면 위 0.5
    parts.append(f'<collision name="back_wall_c"><pose>{-DOCK_T / 2} 0 0.2 0 0 0</pose>'
                 f'<geometry><box><size>{DOCK_T} {3 * DOCK_BAY_W} 0.6</size></box></geometry>'
                 f'</collision>')
    parts.append(f'<visual name="back_wall_v"><pose>{-DOCK_T / 2} 0 0.2 0 0 0</pose>'
                 f'<geometry><box><size>{DOCK_T} {3 * DOCK_BAY_W} 0.6</size></box></geometry>'
                 f'<material><ambient>0.6 0.5 0.4 1</ambient><diffuse>0.6 0.5 0.4 1</diffuse>'
                 f'</material></visual>')
    for i, s in enumerate((-1.5, -0.5, 0.5, 1.5)):
        parts.append(f'<collision name="div{i}_c"><pose>{DOCK_DIV_L / 2} {s} 0.2 0 0 0</pose>'
                     f'<geometry><box><size>{DOCK_DIV_L} {DOCK_T} 0.6</size></box></geometry>'
                     f'</collision>')
        parts.append(f'<visual name="div{i}_v"><pose>{DOCK_DIV_L / 2} {s} 0.2 0 0 0</pose>'
                     f'<geometry><box><size>{DOCK_DIV_L} {DOCK_T} 0.6</size></box></geometry>'
                     f'<material><ambient>0.6 0.5 0.4 1</ambient><diffuse>0.6 0.5 0.4 1</diffuse>'
                     f'</material></visual>')
    for idx, (bay, (color, shape)) in enumerate(sorted(markers.items())):
        parts.append(_marker_visual(idx, color, shape, f'0.02 {bay * DOCK_BAY_W} 0.75'))
    body = '\n      '.join(parts)
    return f"""
    <model name="kaboat_dock">
      <static>true</static>
      <link name="link">
      {body}
      </link>
    </model>
"""


def world_sdf(name, course, boat, wave_gain=0.0, wind_mps=0.0, markers=None):
    markers = markers or DEFAULT_MARKERS
    lat0, lon0 = course['origin']
    sx, sy = course['start']
    yaw0 = math.radians((90.0 - course['psi0'] + 180.0) % 360.0 - 180.0)
    includes = []
    for i, b in enumerate(course['buoys']):
        family = 'kaboat_obst' if b['kind'].startswith('obst') else 'kaboat_buoy'
        uri = f"{family}_{b['color']}"
        includes.append(f'''    <include>
      <name>buoy_{i}_{b['kind']}</name>
      <pose>{b['x']:.3f} {b['y']:.3f} 0 0 0 0</pose>
      <uri>{uri}</uri>
    </include>''')
    d = course['dock']
    dock_yaw = math.atan2(d['n'][1], d['n'][0])
    dock_block = dock_sdf(d, markers).replace(
        '<model name="kaboat_dock">',
        f'<model name="kaboat_dock">\n      <pose>{d["O"][0]:.3f} {d["O"][1]:.3f} '
        f'0 0 0 {dock_yaw:.5f}</pose>')
    wind = ''
    if wind_mps > 0.0:
        wind = f"""
    <plugin filename="libUSVWind.so" name="vrx::USVWind">
      <wind_obj>
        <name>ssf_boat</name>
        <link_name>base_link</link_name>
        <coeff_vector>.5 .5 .33</coeff_vector>
      </wind_obj>
      <wind_direction>240</wind_direction>
      <wind_mean_velocity>{wind_mps}</wind_mean_velocity>
      <var_wind_gain_constants>0</var_wind_gain_constants>
      <var_wind_time_constants>2</var_wind_time_constants>
      <random_seed>10</random_seed>
      <update_rate>10</update_rate>
    </plugin>
"""
    inc = '\n'.join(includes)
    return f"""<?xml version="1.0" ?>
<!-- 생성 파일: python3 -m ssf_vrx.make_world (손으로 고치지 말 것). 코스 = waypoints.yaml, 배 = S.S.F {boat} -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="4ms" type="dart">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <scene>
      <sky></sky>
      <grid>false</grid>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8</background>
    </scene>

    <!-- 월드 원점 = WP0. ENU: x 동, y 북 -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{lat0}</latitude_deg>
      <longitude_deg>{lon0}</longitude_deg>
      <elevation>0.0</elevation>
      <heading_deg>0.0</heading_deg>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- 수면 (VRX 파도 셰이더). 파도 크기는 아래 wavefield gain -->
    <include>
      <name>Coast Waves</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>coast_waves</uri>
    </include>

    <!-- 배 -->
    <include>
      <name>ssf_boat</name>
      <pose>{sx:.3f} {sy:.3f} 0.02 0 0 {yaw0:.5f}</pose>
      <uri>ssf_boat_{boat.lower()}</uri>
    </include>

    <!-- 부표 -->
{inc}

    <!-- 도크 (표식: 칸 -1/0/+1 = {markers[-1]}/{markers[0]}/{markers[1]}; 목표 칸 {d['bay']}) -->
{dock_block}
{wind}
    <!-- 파도 파라미터 (모든 Surface/WaveVisual 이 이 토픽을 듣는다). gain 0 = 잔잔 -->
    <plugin filename="libPublisherPlugin.so" name="vrx::PublisherPlugin">
      <message type="gz.msgs.Param" topic="/vrx/wavefield/parameters" every="2.0">
        params {{
          key: "direction"
          value {{ type: DOUBLE  double_value: 0.0 }}
        }}
        params {{
          key: "gain"
          value {{ type: DOUBLE  double_value: {wave_gain} }}
        }}
        params {{
          key: "period"
          value {{ type: DOUBLE  double_value: 5 }}
        }}
        params {{
          key: "steepness"
          value {{ type: DOUBLE  double_value: 0 }}
        }}
      </message>
    </plugin>
  </world>
</sdf>
"""


def default_waypoints_path():
    """소스 트리(sim/vrx/src/ssf_vrx/ssf_vrx → 저장소 루트)의 waypoints.yaml. 없으면 설치된 패키지에서."""
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(here, '..', '..', '..', '..', '..'))
    p = os.path.join(root, 'src', 'north_goal_angle', 'config', 'waypoints.yaml')
    if os.path.isfile(p):
        return p
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('north_goal_angle'), 'config',
                            'waypoints.yaml')
    except Exception:
        return p


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.abspath(os.path.join(here, '..'))
    ap.add_argument('--boat', default='A', choices=['A', 'B'])
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--name', default=None, help='월드 이름 (기본 kaboat_<배 소문자>)')
    ap.add_argument('--waypoints', default=default_waypoints_path())
    ap.add_argument('--bay', type=int, default=None, choices=[-1, 0, 1], help='목표 칸(기본 무작위)')
    ap.add_argument('--wave-gain', type=float, default=0.0, help='파도 gain (0 잔잔, VRX 기본 0.3)')
    ap.add_argument('--wind', type=float, default=0.0, help='평균 풍속 m/s (0 이면 바람 플러그인 없음)')
    ap.add_argument('--worlds-dir', default=os.path.join(pkg, 'worlds'))
    ap.add_argument('--models-dir', default=os.path.join(pkg, 'models'))
    a = ap.parse_args(argv)

    name = a.name or f'kaboat_{a.boat.lower()}'
    wps = load_wps(a.waypoints)
    course = build_course(wps, a.seed, a.bay)
    os.makedirs(a.worlds_dir, exist_ok=True)
    write_buoy_models(a.models_dir)
    sdf_path = os.path.join(a.worlds_dir, f'{name}.sdf')
    with open(sdf_path, 'w', encoding='utf-8') as f:
        f.write(world_sdf(name, course, a.boat, a.wave_gain, a.wind))
    side = dict(seed=a.seed, boat=a.boat, waypoints=os.path.basename(a.waypoints),
                origin=course['origin'], start=course['start'], psi0=course['psi0'],
                dock=dict(O=course['dock']['O'], n=course['dock']['n'], bay=course['dock']['bay']),
                buoys=course['buoys'], gates=[dict(c=g['c'], w=g['w']) for g in course['gates']],
                wps=course['wps'])
    with open(os.path.join(a.worlds_dir, f'{name}.json'), 'w', encoding='utf-8') as f:
        json.dump(side, f, ensure_ascii=False, indent=1)
    print(f'wrote {sdf_path}  (buoys {len(course["buoys"])}, dock bay {course["dock"]["bay"]}, '
          f'start {course["start"][0]:.1f},{course["start"][1]:.1f} psi0 {course["psi0"]:.0f}°)')


if __name__ == '__main__':
    main()

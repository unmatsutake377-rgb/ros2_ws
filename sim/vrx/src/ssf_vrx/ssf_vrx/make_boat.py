"""배 모델 SDF 생성기 — ssf_boat_a / ssf_boat_b (VRX Surface + SimpleHydrodynamics + gz Thruster).

  python3 -m ssf_vrx.make_boat --out <models 폴더>     # 두 배 모두 생성 (저장소에 결과도 커밋해 둔다)

치수는 보고서(A 최종본·B 15차) 값. **수력 계수(xU·xUU·nR …)와 부력 원기둥은 가정값**이다 —
VRX roboboat01(10 kg) 값을 질량·목표 속도에 맞춰 늘렸다. 수조 실측(가속·선회·정지)이 나오면 여기 숫자만 바꾼다.

센서 토픽은 절대 이름으로 박아 둔다(/ssf/...). 브릿지가 우리 토픽(/scan, /oak/rgb/image_raw, …)으로 옮긴다.
LiDAR 프레임: ship_direction 은 'LiDAR 각 80° = 정면, 왼쪽 +' 를 쓴다(LIDAR_FORWARD_DEG). Gazebo 의 스캔 각은
센서 +x 에서 반시계 양수이므로 센서를 -80° 돌려 단다 → 스캔 80° 가 뱃머리.
"""
import argparse
import math
import os

LIDAR_FORWARD_DEG = 80.0     # ship_direction.LIDAR_FORWARD_DEG 와 같아야 한다

BOATS = {
    'A': dict(
        loa=1.66, boa=0.55, depth=0.22, mass=34.0,
        # 추진기 (x, y) — 선미 좌우 ±120 mm (보고서 Ⅱ-3.3)
        thrusters=[('left', -0.70, 0.12), ('right', -0.70, -0.12)],
        prop_z=-0.16,            # 선저(-0.11) 아래 50 mm
        hull_y=0.14, hull_r=0.15,
        # 수력 계수(가정): 32 N(base_pwm 1360 두 기)에서 약 1.0 m/s, 94 N 에서 약 1.75 m/s
        hydro=dict(xU=10.0, xUU=25.0, yV=40.0, yVV=60.0, zW=200.0,
                   kP=20.0, kPP=30.0, mQ=40.0, mQQ=60.0, nR=6.0, nRR=8.0),
        lidar=(0.65, 0.0, 0.20), camera=(0.15, 0.0, 0.90), gps=(-0.45, 0.0, 0.25),
    ),
    'B': dict(
        loa=1.77, boa=0.522, depth=0.19, mass=30.0,
        # V자 4기: 뒤쪽 쌍 간격 280, 앞쪽 쌍 125, 전후 365 mm (보고서 Ⅱ-3.3)
        thrusters=[('rear_left', -0.65, 0.14), ('rear_right', -0.65, -0.14),
                   ('front_left', -0.285, 0.0625), ('front_right', -0.285, -0.0625)],
        prop_z=-0.145,
        hull_y=0.13, hull_r=0.14,
        hydro=dict(xU=10.0, xUU=25.0, yV=40.0, yVV=60.0, zW=200.0,
                   kP=20.0, kPP=30.0, mQ=40.0, mQQ=60.0, nR=6.0, nRR=8.0),
        lidar=(0.70, 0.0, 0.18), camera=(0.15, 0.0, 0.90), gps=(-0.45, 0.0, 0.22),
    ),
}

T200 = dict(fwd_max_n=47.0, rev_max_n=35.0, prop_d=0.076)


def box_inertia(m, x, y, z):
    return (m / 12.0 * (y * y + z * z), m / 12.0 * (x * x + z * z), m / 12.0 * (x * x + y * y))


def _thruster_links_joints(name, x, y, z):
    """roboboat01 과 같은 구성: housing(고정) + prop(회전, 축 x) 두 링크, 조인트 두 개."""
    return f"""
    <link name="{name}_housing_link">
      <pose>{x} {y} {z} 0 0 0</pose>
      <inertial><mass>0.4</mass>
        <inertia><ixx>0.0005</ixx><iyy>0.0005</iyy><izz>0.0005</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <visual name="{name}_housing_visual">
        <geometry><cylinder><radius>0.05</radius><length>0.11</length></cylinder></geometry>
        <pose>0 0 0 0 1.5708 0</pose>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
    </link>
    <link name="{name}_prop_link">
      <pose>{x - 0.06} {y} {z} 0 0 0</pose>
      <inertial><mass>0.05</mass>
        <inertia><ixx>0.00002</ixx><iyy>0.00002</iyy><izz>0.00002</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <visual name="{name}_prop_visual">
        <geometry><cylinder><radius>0.038</radius><length>0.01</length></cylinder></geometry>
        <pose>0 0 0 0 1.5708 0</pose>
        <material><ambient>0.2 0.2 0.6 1</ambient><diffuse>0.2 0.2 0.6 1</diffuse></material>
      </visual>
    </link>
    <joint name="{name}_chasis_engine_joint" type="fixed">
      <parent>base_link</parent><child>{name}_housing_link</child>
    </joint>
    <joint name="{name}_engine_propeller_joint" type="revolute">
      <pose>0 0 0 1.5708 0 0</pose>
      <axis><xyz>1 0 0</xyz>
        <limit><effort>100</effort><velocity>300</velocity></limit>
        <dynamics><friction>0.05</friction><damping>0.05</damping></dynamics>
      </axis>
      <parent>{name}_housing_link</parent><child>{name}_prop_link</child>
    </joint>
"""


def _thruster_plugin(name):
    return f"""
    <plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster">
      <joint_name>{name}_engine_propeller_joint</joint_name>
      <thrust_coefficient>0.0001</thrust_coefficient>
      <fluid_density>1000</fluid_density>
      <propeller_diameter>{T200['prop_d']}</propeller_diameter>
      <velocity_control>true</velocity_control>
      <max_thrust_cmd>{T200['fwd_max_n']}</max_thrust_cmd>
      <min_thrust_cmd>-{T200['rev_max_n']}</min_thrust_cmd>
    </plugin>
"""


def _surface(hull_len, hull_r, y, z=0.0):
    return f"""
    <plugin filename="libSurface.so" name="vrx::Surface">
      <link_name>base_link</link_name>
      <hull_length>{hull_len:.3f}</hull_length>
      <hull_radius>{hull_r}</hull_radius>
      <fluid_level>0.0</fluid_level>
      <points>
        <point>{hull_len / 2:.3f} {y} {z}</point>
        <point>{-hull_len / 2:.3f} {y} {z}</point>
      </points>
      <wavefield><topic>/vrx/wavefield/parameters</topic></wavefield>
    </plugin>
"""


def _sensors(spec):
    lx, ly, lz = spec['lidar']
    cx, cy, cz = spec['camera']
    gx, gy, gz = spec['gps']
    lidar_yaw = -math.radians(LIDAR_FORWARD_DEG)
    hfov = math.radians(63.9)        # vision.yaml hfov_deg 와 같아야 한다 (OAK ISP 1/3 실측)
    return f"""
      <!-- RPLIDAR A3 대역: 360°, 0.25°, 11 Hz, 25 m. 스캔 80° = 뱃머리가 되게 -80° 회전 -->
      <sensor name="lidar" type="gpu_lidar">
        <pose>{lx} {ly} {lz} 0 0 {lidar_yaw:.5f}</pose>
        <topic>/ssf/scan</topic>
        <update_rate>11</update_rate>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <lidar>
          <scan>
            <horizontal>
              <samples>1440</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range><min>0.2</min><max>25.0</max><resolution>0.01</resolution></range>
          <noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise>
        </lidar>
      </sensor>
      <!-- OAK-1 PoE 대역: 640x360, hfov 63.9°, 30 fps. 마스트 위 -->
      <sensor name="oak" type="camera">
        <pose>{cx} {cy} {cz} 0 0 0</pose>
        <topic>/ssf/oak/rgb/image_raw</topic>
        <update_rate>30</update_rate>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <camera>
          <horizontal_fov>{hfov:.5f}</horizontal_fov>
          <image><width>640</width><height>360</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>300</far></clip>
          <noise type="gaussian"><mean>0.0</mean><stddev>0.007</stddev></noise>
        </camera>
      </sensor>
      <!-- IMU 대역 (iahrs): 100 Hz. ENU 쿼터니언 → sim_bridge 가 /imu/yaw_raw 로 -->
      <sensor name="imu" type="imu">
        <pose>0 0 0.3 0 0 0</pose>
        <topic>/ssf/imu</topic>
        <update_rate>100</update_rate>
        <always_on>1</always_on>
        <imu>
          <angular_velocity>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></z>
          </angular_velocity>
        </imu>
      </sensor>
      <!-- ZED-F9P 대역: 1 Hz (north_goal_angle gps_stale_sec 2.5 가 1 Hz 전제) -->
      <sensor name="navsat" type="navsat">
        <pose>{gx} {gy} {gz} 0 0 0</pose>
        <topic>/ssf/navsat</topic>
        <update_rate>1</update_rate>
        <always_on>1</always_on>
      </sensor>
"""


def boat_sdf(boat):
    spec = BOATS[boat]
    loa, boa, depth, m = spec['loa'], spec['boa'], spec['depth'], spec['mass']
    ixx, iyy, izz = box_inertia(m, loa, boa, depth)
    hull_len = round(loa * 0.9, 3)
    h = spec['hydro']
    thr = spec['thrusters']
    body = f"""<?xml version="1.0" ?>
<!-- 생성 파일: python3 -m ssf_vrx.make_boat (손으로 고치지 말 것). S.S.F {boat} -->
<sdf version="1.9">
  <model name="ssf_boat_{boat.lower()}">
    <link name="base_link">
      <inertial>
        <pose>0 0 -0.05 0 0 0</pose>
        <mass>{m}</mass>
        <inertia><ixx>{ixx:.4f}</ixx><iyy>{iyy:.4f}</iyy><izz>{izz:.4f}</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="hull_collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>{loa} {boa} {depth}</size></box></geometry>
      </collision>
      <visual name="hull_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>{loa} {boa} {depth}</size></box></geometry>
        <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
      </visual>
      <visual name="mast_visual">
        <pose>{spec['camera'][0]} 0 {spec['camera'][2] / 2 + 0.05} 0 0 0</pose>
        <geometry><cylinder><radius>0.02</radius>
                  <length>{spec['camera'][2] - 0.1:.2f}</length></cylinder></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
{_sensors(spec)}
    </link>
"""
    for name, x, y in thr:
        body += _thruster_links_joints(name, x, y, spec['prop_z'])
    body += _surface(hull_len, spec['hull_r'], spec['hull_y'])
    body += _surface(hull_len, spec['hull_r'], -spec['hull_y'])
    body += f"""
    <plugin filename="libSimpleHydrodynamics.so" name="vrx::SimpleHydrodynamics">
      <link_name>base_link</link_name>
      <xDotU>0.0</xDotU><yDotV>0.0</yDotV><nDotR>0.0</nDotR>
      <xU>{h['xU']}</xU><xUU>{h['xUU']}</xUU>
      <yV>{h['yV']}</yV><yVV>{h['yVV']}</yVV>
      <zW>{h['zW']}</zW>
      <kP>{h['kP']}</kP><kPP>{h['kPP']}</kPP>
      <mQ>{h['mQ']}</mQ><mQQ>{h['mQQ']}</mQQ>
      <nR>{h['nR']}</nR><nRR>{h['nRR']}</nRR>
    </plugin>
"""
    for name, _x, _y in thr:
        body += _thruster_plugin(name)
    body += """
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>ssf_boat</robot_base_frame>
      <odom_publish_frequency>10</odom_publish_frequency>
      <odom_topic>/ssf/odometry</odom_topic>
      <dimensions>3</dimensions>
    </plugin>
  </model>
</sdf>
"""
    return body


def model_config(boat):
    return f"""<?xml version="1.0"?>
<model>
  <name>ssf_boat_{boat.lower()}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>SSF</name></author>
  <description>S.S.F {boat} — KABOAT 2026 (생성 파일, make_boat.py)</description>
</model>
"""


def write_models(out_dir):
    written = []
    for boat in BOATS:
        d = os.path.join(out_dir, f'ssf_boat_{boat.lower()}')
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, 'model.sdf'), 'w', encoding='utf-8') as f:
            f.write(boat_sdf(boat))
        with open(os.path.join(d, 'model.config'), 'w', encoding='utf-8') as f:
            f.write(model_config(boat))
        written.append(d)
    return written


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument('--out', default=os.path.join(here, '..', 'models'),
                    help='models 폴더 (기본: 패키지 소스의 models/)')
    a = ap.parse_args(argv)
    for d in write_models(os.path.abspath(a.out)):
        print('wrote', d)


if __name__ == '__main__':
    main()

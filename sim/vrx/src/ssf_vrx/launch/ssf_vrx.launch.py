"""VRX(Gazebo Garden) 위에서 SSF 배 + 코스 + 우리 스택을 띄운다.

  ros2 launch ssf_vrx ssf_vrx.launch.py boat:=A                 # 월드 kaboat_a, 스택 포함
  ros2 launch ssf_vrx ssf_vrx.launch.py boat:=B world:=kaboat_b headless:=true
  ros2 launch ssf_vrx ssf_vrx.launch.py boat:=A with_stack:=false   # 시뮬+브릿지만 (스택은 따로)

구성:
  1) vrx_gz/vrx_environment.launch.py — gz sim + VRX 플러그인 경로. sim_mode:=sim, 경기 브릿지 끔.
     world 는 이 패키지 worlds/<world>.sdf 의 절대 경로(확장자 없이)로 넘긴다. 배는 월드 안 <include>.
  2) ros_gz_bridge parameter_bridge — 센서 4종(gz→ROS)·추진기(ROS→gz)·지상진실 odometry.
     ROS 쪽 이름은 우리 스택 토픽으로 remap: /scan, /oak/rgb/image_raw, /ublox_gps_node/fix.
  3) sim_bridge — Motor_run → cmd_thrust, /boat_* 상태, IMU → /imu/yaw_raw.
  4) (with_stack) ssf_stack_sim.launch.py — 드라이버·ssf_bridge 없는 대회 스택.

⚠️ 노드들은 벽시계(monotonic)로 돈다. Gazebo RTF 가 0.5 아래로 떨어지면 센서 간격이 벌어져
   페일세이프(0.7/3.0 s)·워치독(0.5 s)이 실제보다 쉽게 걸린다. `gz stats` 로 RTF 를 먼저 본다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

PKG = 'ssf_vrx'

# gz 토픽 ↔ ROS 토픽. '[' = gz→ROS, ']' = ROS→gz (ros_gz_bridge 규칙)
SENSOR_BRIDGES = [
    ('/ssf/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan', '[', '/scan'),
    ('/ssf/oak/rgb/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image', '[',
     '/oak/rgb/image_raw'),
    ('/ssf/navsat', 'sensor_msgs/msg/NavSatFix', 'gz.msgs.NavSat', '[', '/ublox_gps_node/fix'),
    ('/ssf/imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU', '[', '/vrx/imu'),
    ('/ssf/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', '[', '/vrx/ground_truth'),
    ('/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', '[', '/clock'),
]
THRUSTER_JOINTS = {
    'A': ['left_engine_propeller_joint', 'right_engine_propeller_joint'],
    'B': ['rear_left_engine_propeller_joint', 'rear_right_engine_propeller_joint',
          'front_left_engine_propeller_joint', 'front_right_engine_propeller_joint'],
}
MODEL_NAME = 'ssf_boat'


def bridge_args(boat):
    args = [f'{gz}@{ros_t}{d}{gz_t}' for gz, ros_t, gz_t, d, _ in SENSOR_BRIDGES]
    for j in THRUSTER_JOINTS[boat]:
        args.append(f'/model/{MODEL_NAME}/joint/{j}/cmd_thrust'
                    '@std_msgs/msg/Float64]gz.msgs.Double')
    return args


def bridge_remaps():
    return [(gz, ros) for gz, _t, _g, _d, ros in SENSOR_BRIDGES if gz != ros]


def _launch(context):
    boat = LaunchConfiguration('boat').perform(context).upper()
    world = LaunchConfiguration('world').perform(context)
    share = get_package_share_directory(PKG)
    if not world:
        world = f'kaboat_{boat.lower()}'
    world_path = world if os.path.isabs(world) else os.path.join(share, 'worlds', world)
    if world_path.endswith('.sdf'):
        world_path = world_path[:-4]
    if not os.path.isfile(world_path + '.sdf'):
        raise RuntimeError(
            f'월드 파일 없음: {world_path}.sdf — `ros2 run ssf_vrx make_world --boat {boat}` 로 만들고 '
            f'colcon build 를 다시 한다(또는 world:=/절대/경로)')

    vrx_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vrx_gz'), 'launch', 'vrx_environment.launch.py')),
        launch_arguments={
            'world': world_path,
            'sim_mode': 'sim',
            'bridge_competition_topics': 'False',
            'headless': LaunchConfiguration('headless'),
            'paused': LaunchConfiguration('paused'),
            'extra_gz_args': LaunchConfiguration('extra_gz_args'),
        }.items())

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ssf_gz_bridge',
        output='screen', arguments=bridge_args(boat), remappings=bridge_remaps())

    sim_bridge = Node(
        package=PKG, executable='sim_bridge', name='sim_bridge', output='screen',
        parameters=[os.path.join(share, 'config', 'ssf_vrx.yaml'),
                    {'boat': boat, 'model_name': MODEL_NAME}])

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'ssf_stack_sim.launch.py')),
        launch_arguments={
            'boat': boat,
            'search_behavior': LaunchConfiguration('search_behavior'),
            'record': LaunchConfiguration('record'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('with_stack')))
    return [vrx_env, bridge, sim_bridge, stack]


def generate_launch_description():
    share = get_package_share_directory(PKG)
    return LaunchDescription([
        # 우리 모델·월드를 Gazebo 리소스 경로에 (VRX 의 hooks 가 vrx_gz 경로를 이미 넣는다)
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            [os.path.join(share, 'models'), ':', os.path.join(share, 'worlds'), ':',
             EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')]),
        DeclareLaunchArgument('boat', choices=['A', 'B'], description='배 (필수)'),
        DeclareLaunchArgument('world', default_value='',
                              description='worlds/<이름>.sdf 또는 절대 경로. 기본 kaboat_<배>'),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('paused', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('extra_gz_args', default_value=''),
        DeclareLaunchArgument('with_stack', default_value='true', choices=['true', 'false'],
                              description='false 면 시뮬+브릿지만 (스택은 따로 띄운다)'),
        DeclareLaunchArgument('search_behavior', default_value='hold', choices=['hold', 'orbit']),
        DeclareLaunchArgument('record', default_value='true', choices=['true', 'false'],
                              description='blackbox CSV 기록'),
        OpaqueFunction(function=_launch),
    ])

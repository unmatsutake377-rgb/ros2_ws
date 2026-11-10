"""north_goal_angle launch — 코스(course)와 웨이포인트 파일을 인자로 바꿔 끼울 수 있다.

  ros2 launch north_goal_angle north_goal_angle_launch.py
      → course=main, config/waypoints.yaml (실전 파일)

  ros2 launch north_goal_angle north_goal_angle_launch.py \
      waypoints_file:=$HOME/ssf_logs/waypoints_recorded.yaml
      (🚨 ~/ 는 안 된다 — ':=' 뒤라 bash 가 안 풀고, 노드는 경로를 그대로 연다)
      → 방금 녹화한 파일을 **복사 없이** 바로 사용

  ros2 launch north_goal_angle north_goal_angle_launch.py course:=tournament \
      waypoints_file:=<레인별 녹화본.yaml>
      → 토너먼트(전부 mode 8). waypoints_file 을 비우면 config/waypoints_tournament.yaml(템플릿)

코스 (계약서_토너먼트분리_20261109 T1-3, §9-1):
   · course 기본 'main' — 단독 launch(녹화절차 §5)는 지금 명령 그대로 돈다.
     대회 launch(launch_files)는 course 를 필수로 받아 Include 로 넘긴다(T1-4).
   · waypoints_file 기본 '' → 코스 기본 파일
     (main = waypoints.yaml, tournament = waypoints_tournament.yaml).
   · parameters = [공통 north_goal_angle.yaml, north_goal_angle_{course}.yaml,
                   {course, waypoints_file}]
     — 🚨 순서가 의미다. 뒤가 이긴다(공통 → 코스 → launch 인자).
   · 모르는 course 는 choices 가 막고, north_goal_files 도 ValueError(노드도 따로 거부한다).

🚨 왜 waypoints_file 을 인자화했나 (2026-08-07 야외 시험에서 나온 필요)
   waypoint_recorder 는 안전을 위해 **새 파일**에만 쓴다(실전 waypoints.yaml 을 안 건드림).
   그런데 그 파일을 쓰려면 지금까지는 "복사 → 재빌드" 를 해야 했다.
   대회장에서 좌표를 찍고 바로 미션을 돌려야 하는데, 손이 많이 가고 실수 여지가 크다.
   노드는 이미 waypoints_file 파라미터를 받고 있었는데(north_goal_angle.py) launch 가
   그걸 노출하지 않아 **쓸 방법이 없었다** — ublox_gps launch 가 c94_m8p 에 하드코딩돼
   F9P 를 못 쓰던 것(c1b0d81)과 같은 유형이다.

   ★ 녹화 파일을 인자로 쓰면 config/waypoints.yaml 은 그대로 남는다 → 오입력해도 복구 가능.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

COURSES = ['main', 'tournament']
DEFAULT_COURSE = 'main'   # §9-1: 단독 launch 는 기본 main (대회 launch 만 필수)


def north_goal_files(share_dir, course, waypoints_file):
    """(course, waypoints_file 인자) → (params yaml 목록, 쓸 웨이포인트 경로). 순수 함수(ROS 없이 테스트).

    params = [공통 north_goal_angle.yaml, north_goal_angle_{course}.yaml] — 🚨 순서가 의미다(뒤가 이긴다).
    waypoints_file 이 비면(공백만도) 코스 기본 파일:
      main = waypoints.yaml, tournament = waypoints_tournament.yaml.
    모르는 course 는 ValueError — 조용히 main 으로 떨어지지 않는다.
    노드 쪽 허용 코스 = waypoint_loader.COURSE_MODES 키 (test_course_files.py 가 같은지 검사).
    """
    wp_default = {'main': 'waypoints.yaml', 'tournament': 'waypoints_tournament.yaml'}
    if course not in wp_default:
        raise ValueError(f"course={course!r} — 'main' 또는 'tournament'")
    cfg = os.path.join(share_dir, 'config')
    params = [os.path.join(cfg, 'north_goal_angle.yaml'),
              os.path.join(cfg, f'north_goal_angle_{course}.yaml')]
    wp = waypoints_file.strip() if waypoints_file else ''
    if not wp:
        wp = os.path.join(cfg, wp_default[course])
    return params, wp


def _north_node(context):
    course = LaunchConfiguration('course').perform(context)
    wp_arg = LaunchConfiguration('waypoints_file').perform(context)
    params, wp = north_goal_files(
        get_package_share_directory('north_goal_angle'), course, wp_arg)
    missing = [f for f in params if not os.path.isfile(f)]
    if missing:
        # 코스 yaml 없이 뜨면 코스별 값(경계 폴리곤)이 조용히 빠진다 → launch 를 세운다.
        raise RuntimeError(
            f'north_goal_angle params 파일 없음: {missing} — pull 뒤 colcon build 를 했는지 확인')
    return [
        LogInfo(msg=f'north_goal_angle: course={course}, 웨이포인트={wp}, params={params}'),
        Node(
            package='north_goal_angle',
            executable='north_goal_angle',
            name='north_goal_angle',   # config yaml 의 /north_goal_angle 키와 일치
            output='screen',
            # 🚨 순서가 중요하다. 뒤에 오는 것이 이긴다 —
            #    공통 yaml → 코스 yaml → launch 인자(dict). yaml 에 waypoints_file 이 있더라도 인자가 덮어쓴다.
            parameters=params + [{'course': course, 'waypoints_file': wp}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'course',
            default_value=DEFAULT_COURSE,
            choices=COURSES,
            description='코스: main(종합임무) | tournament(토너먼트, 웨이포인트 전부 mode 8)'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value='',
            description="웨이포인트 yaml 경로. 비우면 코스 기본 파일"
                        "(main=waypoints.yaml, tournament=waypoints_tournament.yaml). "
                        "녹화본을 바로 쓰려면 그 경로를 준다"),
        OpaqueFunction(function=_north_node),
    ])

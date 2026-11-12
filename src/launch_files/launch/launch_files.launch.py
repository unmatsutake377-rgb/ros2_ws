"""대회 launch — 배(boat)와 코스(course)는 필수 인자다(둘 다 기본값 없음, 빠지면 launch 거부).

  ros2 launch launch_files launch_files.launch.py boat:=A course:=main         (종합임무)
  ros2 launch launch_files launch_files.launch.py boat:=B course:=tournament \
      waypoints_file:=<레인별 녹화본.yaml>                                      (토너먼트)

코스 (계약서_토너먼트분리_20261109 T1-4, §9):
  · north_goal_angle 은 Include 로 course·waypoints_file 을 받아 north_goal_angle_{course}.yaml 과
    코스 기본 웨이포인트 파일을 스스로 고른다(waypoints_file 이 비면 코스 기본 파일).
  · ship_direction·motor_control 은 공통 yaml 뒤에 course_param_files() 를 붙인다
    (main = 붙이는 것 없음, tournament = <pkg>_tournament.yaml, 없으면 launch 를 세운다).
  · ship_direction 의 dwa_hull 은 boat 인자로 정한다(main·tournament 모두, §9-5).
  · 판 사이에는 launch 를 재시작한다(MANUAL→AUTO 재진입은 wp_idx 를 안 되돌린다).

인식 융합 (계약서_인식융합_20261112 §10): d455_depth·perception_fusion 기본 true(대회에서 켬).
  끄기(롤백): ... perception_fusion:=false d455_depth:=false  → 이 계약 전과 같은 노드 구성.
"""

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction, DeclareLaunchArgument,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os
from launch_ros.actions import Node


def yaw_mux_param_files(share_dir, boat):
    """yaw_mux params 파일 목록. 🚨 순서가 의미다: 공통 → 배별 (뒤 파일이 이긴다).

    ssf_heading.launch.py(벤치)에 같은 함수가 있다 — 바꿀 땐 둘 다 (test_boat_config.py 가 검사).
    """
    cfg = os.path.join(share_dir, 'config')
    return [os.path.join(cfg, 'ssf_heading.yaml'),
            os.path.join(cfg, f'ssf_heading_{boat}.yaml')]


def _yaw_mux(context):
    # 입력이 끊기거나 소스가 미구현이면 발행을 멈춘다(설계).
    # → ship_goal_angle 이 /yaw_error 를 멈추고, north_goal_angle geofence 가 침묵한다.
    boat = LaunchConfiguration('boat').perform(context)
    files = yaw_mux_param_files(get_package_share_directory('ssf_heading'), boat)  # 공통 → 배별
    missing = [f for f in files if not os.path.isfile(f)]
    if missing:
        # 일부러 launch 전체를 세운다 — 배별 값 없이 뜨면 헤딩이 조용히 틀린다.
        # 🚨 이 검사는 t=0, 어떤 Node 보다 먼저 돈다(목록에서 _course_check 다음, Node 0개인 그 뒤)
        #    → 아무 프로세스도 안 뜬 채 멈춘다.
        #    아래 12) 브릿지 주석의 "예외 금지" 는 선택 기능(ntrip·브릿지) 얘기다 — 이 검사를 지우지 말 것.
        raise RuntimeError(
            f'yaw_mux params 파일 없음: {missing} — pull 뒤 colcon build 를 했는지 확인')
    return [
        LogInfo(msg=f'boat={boat}, 파일={files[0]} → {files[1]}'),
        Node(
            package='ssf_heading',
            executable='yaw_mux',
            name='yaw_mux',
            output='screen',
            parameters=files,
        ),
    ]


def _pkg_yaml(pkg):
    """<pkg> 설치 share 의 config/<pkg>.yaml — 미션·제어 노드와 blackbox 의 파라미터 출처.

    🚨 11-05 부터 이 yaml 이 대회값이다(그 전엔 parameters= 없이 떠서 코드 기본값이 대회값).
       · 타입 오타(3.0 ↔ 3, 1360 ↔ 1360.0)는 그 노드를 부팅 때 죽인다(InvalidParameterType).
       · 노드가 선언 안 한 키는 에러 없이 버려진다.
       · 노트북에서 커밋 안 하고 고친 yaml 도 그대로 대회값이 된다(git status 로 확인).
    test_launch_params.py 가 yaml 키 ⊆ 선언 키, 타입 일치, 절 이름 = 노드 이름을 검사한다.
    """
    return os.path.join(get_package_share_directory(pkg), 'config', f'{pkg}.yaml')


# 코스 yaml 을 공통 yaml 뒤에 붙이는 패키지. north_goal_angle 은 자기 launch 가 붙인다(Include).
COURSE_PKGS = ('ship_direction', 'motor_control')


def course_param_files(pkg, course):
    """course → <pkg> 공통 yaml 뒤에 붙일 코스 yaml 목록 (LaunchContext 없이 테스트).

    main       = [] — 공통 <pkg>.yaml 그대로(main 코스 파일은 north_goal_angle 에만 있다).
    tournament = [share/<pkg>/config/<pkg>_tournament.yaml] — 덮어쓰는 키만 든 파일.
    🚨 파일이 없으면 RuntimeError: setup.py 누락·빌드 안 함이면 토너먼트 값 없이
       조용히 main 값으로 뜬다(계약서 위험 1) → launch 를 세운다.
    모르는 course 는 ValueError — 조용히 main 으로 떨어지지 않는다.
    """
    if course == 'main':
        return []
    if course != 'tournament':
        raise ValueError(f"course={course!r} — 'main' 또는 'tournament'")
    path = os.path.join(get_package_share_directory(pkg), 'config', f'{pkg}_tournament.yaml')
    if not os.path.isfile(path):
        raise RuntimeError(
            f'{pkg} 코스 파일 없음: {path} — pull 뒤 colcon build 를 했는지 확인')
    return [path]


def _course_check(context):
    # 코스 파일 확인(raise)을 launch 목록 첫 OpaqueFunction 으로 — _yaw_mux 보다도 앞, 어떤 Node
    # 보다 먼저 한다(launch 는 깊이 우선 방문: 뒤에 두면 앞 OpaqueFunction 의 Node 가 먼저 예약된다).
    # 여기서는 LogInfo 만 낸다(Node 없음) → 다음 _yaw_mux 의 확인도 여전히 어떤 Node 보다 먼저다.
    # 노드는 _control_nodes 가 띄운다 — 같은 course_param_files 를 다시 부른다.
    course = LaunchConfiguration('course').perform(context)
    wp = LaunchConfiguration('waypoints_file').perform(context)
    # 🚨 변수 이름을 files 로 쓰지 말 것 — ssf_heading test_boat_config 가 모듈 전체에서
    #    yaw_mux parameters 변수(files) 대입이 1번인지 본다.
    by_pkg = {pkg: course_param_files(pkg, course) for pkg in COURSE_PKGS}
    attached = ', '.join(f'{pkg} ← {by_pkg[pkg] or "없음(공통 yaml 만)"}' for pkg in COURSE_PKGS)
    return [LogInfo(msg=f'course={course}: 코스 파일 {attached} / north_goal_angle ← '
                        f'north_goal_angle_{course}.yaml, waypoints_file={wp or "(코스 기본)"}')]


def _control_nodes(context):
    """10) ship_direction · 11) motor_control — course·boat 인자로 parameters 가 정해진다."""
    course = LaunchConfiguration('course').perform(context)
    boat = LaunchConfiguration('boat').perform(context)
    return [
        # 10) Fusion: 라이다 기반 항로 유지
        Node(
            package='ship_direction',
            executable='ship_direction',
            name='ship_direction',
            output='screen',
            # 🚨 순서가 의미다(뒤가 이긴다): 공통 yaml → 코스 yaml → dwa_hull dict **맨 뒤**.
            #    dwa_hull 은 boat 가 정한다(§9-5, main·tournament 모두) — 공통 yaml 의 'A' 를
            #    덮는다. 2번배 yaml 손편집('B')은 이것으로 대신한다. test_launch_params T8 이 검사.
            parameters=[_pkg_yaml('ship_direction'),
                        *course_param_files('ship_direction', course),
                        {'dwa_hull': boat}],
        ),
        # 11) Motor Control
        Node(
            package='motor_control',
            executable='motor_control',
            name='motor_control',
            output='screen',
            # 공통 yaml → 코스 yaml (tournament: goal_slow_min_ratio 1.0, 결승 감속 끔 §9-3)
            parameters=[_pkg_yaml('motor_control'),
                        *course_param_files('motor_control', course)],
        ),
    ]


def search_behavior_params(behavior):
    """launch 인자 search_behavior → ship_back·ship_turn 에 넘길 파라미터 (순수 함수, ROS 없이 테스트).

    orbit(기본, 2026-11 경기규정: 탐색 = 공지 색 부표 원 선회) = 2→ship_back, 3→ship_turn.
    hold(옵션, 옛 2026 안내문 해석: 탐색 = 부표 앞 위치유지) = 2,3→ship_back, ship_turn 침묵.
    (계약서_탐색선회_부표색_20261111 C3 — 11-04 의 hold 기본을 규정에 맞춰 뒤집었다.)
    🚨 enabled 는 Python bool 로 — 문자열 'false' 는 rclpy 타입 불일치고 bool('false') 는 True 다.
    🚨 ship_turn 을 끄려고 active_wp_mode -1 을 넘기지 않는다(부팅 초기값과 겹쳐 SPIN 발행).
    test_search_behavior.py 가 두 행동 모두에서 모드별 /candidate_angle 담당이 하나인지 검사한다.
    """
    if behavior == 'orbit':
        return {'ship_back': {'active_wp_modes': [2]}, 'ship_turn': {'enabled': True}}
    if behavior == 'hold':
        return {'ship_back': {'active_wp_modes': [2, 3]}, 'ship_turn': {'enabled': False}}
    raise ValueError(f"search_behavior={behavior!r} — 'orbit' 또는 'hold'")


def _search_nodes(context):
    # 탐색(mode 3) 행동 스위치. 노드는 둘 다 항상 뜬다 — 바뀌는 건 파라미터뿐.
    # 기본 orbit(2026-11 경기규정 원 선회). 현장에서 위치유지로 돌리려면 search_behavior:=hold.
    behavior = LaunchConfiguration('search_behavior').perform(context)
    params = search_behavior_params(behavior)
    if behavior == 'hold':
        msg = 'search_behavior=hold(옵션): 탐색(3) = ship_back 위치유지(모드 2·3 담당), ship_turn 침묵'
    else:
        msg = 'search_behavior=orbit(기본, 2026-11 규정): 탐색(3) = ship_turn 선회, ship_back 은 모드 2'
    actions = [LogInfo(msg=msg)]
    # 🚨 parameters 순서가 의미다: yaml 먼저, search_behavior dict **맨 뒤** (뒤가 이긴다).
    #    거꾸로면 orbit 에서 ship_back 이 yaml [2,3] 으로 뜨고 ship_turn 도 켜져
    #    mode 3 /candidate_angle 발행자가 2개가 된다(에러 없음). test_launch_params T5 가 검사.
    actions += [
        Node(package='ship_turn', executable='ship_turn', name='ship_turn', output='screen',
             parameters=[_pkg_yaml('ship_turn'), params['ship_turn']]),
        # ship_last 제거됨(6b): mode 0 을 ship_gate 가 인수. 하던 일은 /candidate_angle 에 20000
        #   폴백을 내는 것뿐이라 중복이었다. (mode 5 는 ship_avoid(_ship_avoid), mode 8 폴백은 north_goal_angle)
        Node(package='ship_back', executable='ship_back', name='ship_back', output='screen',
             parameters=[_pkg_yaml('ship_back'), params['ship_back']]),
    ]
    return actions


# OAK 드라이버 respawn 간격[s]. 🚨 런북 10-31: 드라이버를 끄면 카메라가 재부팅(링크 끊김 2 s → 복귀 14 s)하고
#   그 안에 다시 띄우면 연결 실패(23·59 회 재시도 후 실패). "다시 띄울 땐 20 s 기다린다" → 20 s 이상.
#   강제 종료 뒤엔 ~90 s 재연결 불가라 respawn 을 켜도 그만큼 비전 공백이 있다(계약 위험 5).
OAK_RESPAWN_DELAY_S = 20.0


def _oak_camera(context):
    """7) OAK-1 PoE 카메라(depthai_ros_driver). 패키지가 없으면 **건너뛴다** — launch 전체를 세우지 않는다.

    ssf_bridge.launch.py 의 포트 존재 검사와 같은 뜻: 2번배 노트북에 depthai 가 아직 없어도(⚠️ 미확인)
    라이다·GPS·IMU·모터 체인은 떠야 한다. 검출기는 영상 없이 침묵하고 healthcheck 가 CAM ✗ 로 알린다.
    """
    try:
        get_package_share_directory('depthai_ros_driver')
    except PackageNotFoundError:
        return [LogInfo(msg='⏭️ depthai_ros_driver 없음 — 카메라 노드 건너뜀 '
                            '(sudo apt install ros-humble-depthai-ros). 검출기 3종은 영상 없이 침묵한다.')]
    oak_cfg = os.path.join(get_package_share_directory('launch_files'), 'config', 'oak_640x360_full.yaml')
    return [
        LogInfo(msg=f'OAK-1 PoE: depthai_ros_driver camera_node name=oak, 파일={oak_cfg} '
                    f'(i_ip 169.254.1.222, 640x360 ISP 1/3 → /oak/rgb/image_raw, HFOV 63.9°)'),
        Node(
            package='depthai_ros_driver',
            # ⚠️ 실측 필요(Mac 작성): depthai_ros_driver 의 단독 실행파일명. 런북은 camera.launch.py 로 띄웠다
            #    (컴포넌트 컨테이너). 없으면 `ros2 pkg executables depthai_ros_driver` 로 확인.
            executable='camera_node',
            # 🚨 name=oak + 네임스페이스 없음 → 노드 /oak → 토픽 /oak/rgb/image_raw, /oak/rgb/camera_info
            #    (런북 10-31 실측 이름 = vision.yaml·ssf_tools.yaml 의 image_topic). yaml 의 /oak 블록도 이 이름.
            name='oak',
            output='screen',
            parameters=[oak_cfg],
            respawn=True,
            respawn_delay=OAK_RESPAWN_DELAY_S,
        ),
    ]


def d455_depth_params():
    """D455 뎁스 전용 realsense2_camera 파라미터 (순수 함수, ROS 없이 테스트 — test_perception_launch.py).

    계약서_인식융합_20261112 §10 Q9 + 계약서_거리융합_20261112 §4: 컬러·IR·IMU 끔, 뎁스만.
    프로필 480x270x15 (계약서_도리도리회피_v2_20261114 U0): 예전 424x240x15 는 2번배 D455
    (FW 5.13.0.55, USB2) 지원 목록에 없어 기본 640x480 으로 되돌아갔다. 480x270@60/30/15/5 는 있다.
    16:9 로 424x240 과 종횡비가 거의 같다(1.767 대 1.778, 행 띠 비율 각도 유지).
    ⚠️ 1번배(FW 5.17.0.10, USB3.2)는 424x240x15 정상 확인(11-13), 480x270 은 미확인.
    🚨 값은 Python bool 로(문자열 'false' 는 타입 불일치). 래퍼 4.56.4 소스 확인(Mac, 11-12):
       · 이름 = depth_module.depth_profile (rs_launch.py:44 — 옛 depth_module.profile 아님)
       · 구분자 x·, 둘 다 받음 (profile_manager.cpp:453 정규식 [xX,]) → '480x270x15'
       ⚠️ 두 배 부팅 로그 확인: 'Open profile: ... Depth ... 480x270 ... 15' — 안 맞으면 대안 '480,270,15'.
       ⚠️ 1번배는 rs-enumerate-devices -c 로 480x270 Z16 15 를 본다 — 없으면 래퍼가 기본 프로필로 연다.
    hole_filling 끔: 메운 값은 지어낸 거리다(§1-6). initial_reset 끔: USB 재열거로 허브째 끊길 수 있다.
    emitter 는 기본값(켬) 그대로 ⚠️ 물 위 영향 미확인.
    """
    return {
        'enable_color': False,
        'enable_depth': True,
        'enable_infra1': False,
        'enable_infra2': False,
        'enable_gyro': False,
        'enable_accel': False,
        'depth_module.depth_profile': '480x270x15',
        'pointcloud.enable': False,
        'align_depth.enable': False,
        'hole_filling_filter.enable': False,
        'initial_reset': False,
    }


def _d455_depth(context):
    """7b) D455 뎁스 전용(realsense2_camera). d455_depth:=false 면 안 뜬다. 패키지가 없으면 **건너뛴다**.

    🚨 name='camera' + namespace='camera' = camera.launch.py 와 같은 이름 → 토픽 /camera/camera/depth/image_rect_raw,
       /camera/camera/depth/camera_info (= perception_fusion.yaml depth_topic·depth_info_topic).
    🚨 camera.launch.py(물가 영상·벤치, D455 컬러)를 대회 launch 와 **동시에 띄우지 말 것** — 같은 노드 이름
       /camera/camera 가 둘이 되고 한 장치를 두 드라이버가 연다. 물가 영상(D455 컬러)은 §10 Q9 로 폐기:
       ssf_cam_send.sh 의 ros 모드는 컬러 토픽을 구독하는데 뎁스 전용이라 그 토픽이 없다.
    ⚠️ 허브: 뎁스 켠 채 LiDAR·IMU·GPS 와 같은 허브 시험은 아직 안 했다(§10 Q9). 끊기면 d455_depth:=false.
    """
    if LaunchConfiguration('d455_depth').perform(context) != 'true':
        return [LogInfo(msg='d455_depth=false — D455 뎁스 안 띄움 (perception_fusion 은 LiDAR 만, 뎁스 stale WARN)')]
    try:
        get_package_share_directory('realsense2_camera')
    except PackageNotFoundError:
        return [LogInfo(msg='⏭️ realsense2_camera 없음 — D455 뎁스 건너뜀 (colcon build 확인). '
                            'perception_fusion 은 LiDAR 만으로 계속.')]
    return [
        LogInfo(msg='D455 뎁스 전용: realsense2_camera name=camera ns=camera, 480x270x15, 컬러·IR·IMU 끔 '
                    '→ /camera/camera/depth/image_rect_raw (⚠️ 시작 로그로 프로필 확인)'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[d455_depth_params()],
        ),
    ]


def _perception_fusion(context):
    """8b) perception_fusion — /fused_objects (계약서_인식융합_20261112). perception_fusion:=false 면 안 뜬다.

    미션 노드(ship_back·ship_turn·ship_gate)는 yaml distance_source 가 fill/fused 일 때만 이 토픽을 본다.
    노드가 없거나 침묵하면 미션 노드는 LiDAR 로 되돌아가고 WARN 을 낸다(§4). 패키지가 없으면 **건너뛴다**.
    """
    if LaunchConfiguration('perception_fusion').perform(context) != 'true':
        return [LogInfo(msg='perception_fusion=false — /fused_objects 없음. '
                            '미션 노드 distance_source fill/fused 는 LiDAR 로 되돌아간다(WARN)')]
    try:
        get_package_share_directory('perception_fusion')
    except PackageNotFoundError:
        return [LogInfo(msg='⏭️ perception_fusion 없음 — 건너뜀 (colcon build 확인). 미션 노드는 LiDAR 로.')]
    return [
        Node(
            package='perception_fusion',
            executable='perception_fusion',
            name='perception_fusion',          # = perception_fusion.yaml 의 /perception_fusion 절
            output='screen',
            parameters=[_pkg_yaml('perception_fusion')],
        ),
    ]


def _ship_avoid(context):
    """9b) ship_avoid — mode 5(장애물 회피) /candidate_angle 담당 (계약서_도리도리회피_launch_20261113 C2).

    🚨 원자 묶음: north_goal_angle FALLBACK_MODES (8,)(mode 5 폴백 뺌) · ssf_tools wp_nodes 5→ship_avoid ·
       test_avoid_mission 과 한 커밋. 하나만 되돌리면 mode 5 발행자가 2개(값 섞임)이거나 healthcheck 거짓 경보.
    방식은 yaml avoid_mode(기본 'fused' — /avoid_zone·검출기 전엔 매 틱 20000) — 바꾸면 launch 재시작.
    패키지가 없으면 **건너뛴다**(launch 전체를 세우지 않는다). 그때 mode 5 는 발행자 0 — ship_direction 이
    모드 전환 때 20000 으로 리셋하고 못 받은 동안 20000 을 쓰므로 주행은 같다. healthcheck 는 담당 침묵으로 알린다.
    """
    try:
        get_package_share_directory('ship_avoid')
    except PackageNotFoundError:
        return [LogInfo(msg='⏭️ ship_avoid 없음 — 건너뜀 (colcon build 확인). mode 5 /candidate_angle 발행자 0 → '
                            'ship_direction 이 20000(자율 회피)으로 간다. healthcheck 는 mode 5 담당 침묵으로 알린다.')]
    return [
        Node(
            package='ship_avoid',
            executable='ship_avoid',
            name='ship_avoid',                 # = ship_avoid.yaml 의 /ship_avoid 절
            output='screen',
            parameters=[_pkg_yaml('ship_avoid')],
        ),
    ]


def _blackbox(context):
    """13) blackbox(CSV 기록, 구독 전용·발행 0). ssf_tools 가 없으면 **건너뛴다** — launch 를 안 세운다.

    record:=false 면 안 뜬다(IfCondition). ~/ssf_logs/blackbox_<초>.csv, launch 마다 1개.
    🚨 대회 launch 와 ssf_tools.launch.py 를 같이 띄우지 말 것 — /blackbox 가 2개가 되고
       같은 초에 뜨면 파일명이 같아 CSV 를 덮어쓴다. healthcheck·mission_monitor 는 넣지 않는다.
    """
    try:
        cfg = _pkg_yaml('ssf_tools')
    except PackageNotFoundError:
        return [LogInfo(msg='⏭️ ssf_tools 없음 — blackbox 건너뜀 (colcon build 확인). '
                            '나머지 노드는 그대로 뜬다.')]
    return [
        Node(
            package='ssf_tools',
            executable='blackbox',
            name='blackbox',            # = ssf_tools.yaml 의 /blackbox 절
            output='screen',
            parameters=[cfg],
            condition=IfCondition(LaunchConfiguration('record')),
        ),
    ]


def generate_launch_description():

    # 패키지 경로
    dir_rplidar = get_package_share_directory('rplidar_ros')
    dir_north = get_package_share_directory('north_goal_angle')
    dir_ntrip = get_package_share_directory('ntrip_client')
    dir_gps = get_package_share_directory('ublox_gps')
    # ship_direction 은 Node(package=...) 가, ship_goal_angle 은 아래 Include 가 다시 조회한다.
    # 여기 대입은 쓰이지 않아 제거했다(2026-09-23). 카메라(depthai)는 _oak_camera 가 조회한다.
    dir_bridge = get_package_share_directory('ssf_bridge')

    # 비전 공통 설정 (image_topic, hfov_deg, debug_view).
    # 3-4 이후로는 launch 가 노드를 직접 띄우므로 파라미터가 **그대로 닿는다** —
    # V5 의 vision_* 중계(매니저가 --ros-args 로 넘기던 것)는 3-4 에서 제거됐다.
    VISION_CFG = os.path.join(
        get_package_share_directory('color_shape_detector'), 'config', 'vision.yaml')

    # NTRIP 2초 지연
    ntrip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dir_ntrip, 'launch', 'ntrip_client_launch.py')
        )
    )

    return LaunchDescription([
        # 🚨 배 (필수, 기본값 없음 — 빠지거나 A/B 가 아니면 launch 거부).
        #    yaw_mux 의 배별 IMU 값(ssf_heading_A/B.yaml)과 ship_direction 의 dwa_hull 을 고른다.
        #    GPS 는 아래 gps_config 따로.
        #    ros2 launch launch_files launch_files.launch.py boat:=A course:=main   (또는 boat:=B)
        #    기본값을 두면 다른 배에서 그 배 값으로 조용히 뜬다(gps_config 가 이미 그렇게 실패 중).
        DeclareLaunchArgument(
            'boat', choices=['A', 'B'],
            description='배 (필수): A 또는 B — yaw_mux 배별 IMU 값, ship_direction dwa_hull'),
        # 🚨 코스 (필수, 기본값 없음 — boat 와 같은 이유. 계약서_토너먼트분리_20261109 결정 ③).
        #    main = 종합임무, tournament = 토너먼트(웨이포인트 전부 mode 8, 코스 yaml 덮어쓰기).
        #    기본값을 두면 토너먼트 날 인자를 빠뜨려도 main 값으로 조용히 뜬다.
        #    ros2 launch launch_files launch_files.launch.py boat:=A course:=tournament
        DeclareLaunchArgument(
            'course', choices=['main', 'tournament'],
            description='코스 (필수): main(종합임무) 또는 tournament(토너먼트, 전부 mode 8)'),
        # 웨이포인트 파일. 비우면(기본) 코스 기본 파일(main=waypoints.yaml,
        #   tournament=waypoints_tournament.yaml 템플릿). 레인별 녹화본은 경로를 직접 준다.
        DeclareLaunchArgument(
            'waypoints_file', default_value='',
            description='웨이포인트 yaml 경로. 비우면 코스 기본 파일 (north_goal_angle 로 전달)'),
        DeclareLaunchArgument(
            'gps_config', default_value='zed_f9p_rover.yaml',
            description='GPS 설정 — 두 배 모두 ZED-F9P 라 기본값 zed_f9p_rover.yaml'),
        # 탐색(mode 3) 행동. 기본 orbit — 2026-11 경기규정 "공지 색 부표를 중심으로 원 선회"
        #   (빨강·초록 우현선회 CW / 노랑 좌현선회 CCW): ship_turn 이 모드 3, ship_back 은 모드 2.
        #   (11-04 엔 2026 안내문 코스도를 근거로 hold 기본이었다 — 계약서_탐색선회_부표색_20261111 C3.)
        #   hold 는 옵션(탐색 = 부표 앞 위치유지). 현장에서 위치유지로 돌리려면:
        #   ros2 launch launch_files launch_files.launch.py boat:=A course:=main \
        #       search_behavior:=hold
        DeclareLaunchArgument(
            'search_behavior', default_value='orbit', choices=['orbit', 'hold'],
            description='탐색(wp_mode 3) 행동: orbit=ship_turn 선회(기본, 2026-11 경기규정) / '
                        'hold=ship_back 위치유지, ship_turn 침묵(옵션)'),
        # blackbox 기록. 기본 true — 끄려면 record:=false (아래 13, _blackbox 참고).
        DeclareLaunchArgument(
            'record', default_value='true', choices=['true', 'false'],
            description='blackbox CSV 기록(~/ssf_logs): true(기본) / false'),
        # 인식 융합(계약서_인식융합_20261112 §10 Q8: 대회에서 켬 → 기본 true). 롤백(대회 당일):
        #   ros2 launch launch_files launch_files.launch.py boat:=A course:=main \
        #       perception_fusion:=false d455_depth:=false
        #   둘 다 false 면 이 계약 전과 같은 노드 구성. 미션 노드 쪽 롤백은 yaml distance_source: lidar.
        DeclareLaunchArgument(
            'd455_depth', default_value='true', choices=['true', 'false'],
            description='D455 뎁스 전용 드라이버(컬러·IR·IMU 끔, 480x270x15): true(기본) / false'),
        DeclareLaunchArgument(
            'perception_fusion', default_value='true', choices=['true', 'false'],
            description='perception_fusion(/fused_objects: LiDAR+D455 뎁스+검출기 방위): true(기본) / false'),

        # ============================================================
        # 0a) 코스 파일 확인 + LogInfo(course·붙인 파일). 없으면 raise
        # ============================================================
        # 🚨 목록의 첫 OpaqueFunction — 어떤 Node 보다 먼저 돈다(launch 는 깊이 우선 방문이라
        #    _yaw_mux 뒤에 두면 yaw_mux Node 가 먼저 예약된다). Node 를 만들지 않는다.
        OpaqueFunction(function=_course_check),

        # ============================================================
        # 0) yaw_mux — /imu/yaw 의 단독 발행자 (params: 공통 → boat 별, _yaw_mux 참고)
        # ============================================================
        # 🚨 맨 앞쪽(0a 다음)에 둔다: 배별 파일 확인(raise)이 어떤 노드도 예약되기 전에 돌게.
        #    iahrs_driver 보다 먼저 떠도 무해하다 — 원시 yaw 가 오기 전엔 발행하지 않는다(stale).
        OpaqueFunction(function=_yaw_mux),

        # ============================================================
        # 1) RPLIDAR
        # ============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dir_rplidar, 'launch', 'rplidar_a3_launch.py')
            )
        ),

        # ============================================================
        # 2) AHRS (IMU) — 보정 안 한 상대 yaw 를 /imu/yaw_raw 로만 낸다
        # ============================================================
        # 🚨 CLAUDE.md 3-5. 작년엔 이 노드가 /imu/yaw 를 직접 냈는데 그 값이 절대방위가 아니었다
        #    (부팅 0점화 + GPS COG override). 절대방위 합성은 아래 yaw_mux 가 전담한다.
        #    파라미터는 드라이버 기본값과 같지만 **일부러 명시**한다 —
        #    나중에 누가 yaml 로 되살려도 여기서 눈에 띄게 하려는 것이다.
        Node(
            package='iahrs_driver',
            executable='iahrs_driver',
            name='iahrs_driver',
            output='screen',
            parameters=[{
                'yaw_topic': '/imu/yaw_raw',        # ← /imu/yaw 로 되돌리면 발행자 2개가 된다
                'zero_yaw_on_boot': False,
                'use_gps_heading_override': False,
            }],
        ),

        # 2b) yaw_mux 는 맨 앞 0) 으로 옮겼다 (배별 파일 확인을 t=0 에 하려고).

        # ============================================================
        # 3) U-BLOX GPS
        # ============================================================
        # 🚨 배별 설정을 넘긴다. 예전엔 인자 없이 include 해서 ublox launch 의 기본값(B배 F9P)이
        #    A배(M8P)에서도 그대로 쓰였다 — 에러 없이 조용히 어긋나는 류. (2026-09-16)
        #    2026-11-01: 두 배 모두 ZED-F9P 로 확인 — 인자 없이 기본 zed_f9p_rover.yaml.
        #    M8P 를 달 일이 생기면 gps_config:=c94_m8p_rover.yaml.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dir_gps, 'launch', 'ublox_gps_node-launch.py')
            ),
            launch_arguments={'config': LaunchConfiguration('gps_config')}.items(),
        ),

        # ============================================================
        # 4) NTRIP 2초 딜레이
        # ============================================================
        TimerAction(
            period=2.0,
            actions=[ntrip_launch]
        ),

        # ============================================================
        # 5) NORTH 기준 각도 계산
        # ============================================================
        # course·waypoints_file 을 넘긴다 → north launch 가 north_goal_angle_{course}.yaml 과
        #   코스 기본 웨이포인트 파일을 고른다(waypoints_file 이 비면). 단독 north launch 의 기본
        #   course=main 은 여기서 안 쓰인다 — 대회 launch 는 course 가 필수(§9-1).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dir_north, 'launch', 'north_goal_angle_launch.py')
            ),
            launch_arguments={
                'course': LaunchConfiguration('course'),
                'waypoints_file': LaunchConfiguration('waypoints_file'),
            }.items(),
        ),

        # ============================================================
        # 6) SHIP goal angle
        # ============================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ship_goal_angle'),
                             'launch', 'ship_goal_angle_launch_file.launch.py')
            )
        ),

        # ============================================================
        # 7) 카메라 — OAK-1 PoE (depthai_ros_driver). 2026-11-04 D455 → OAK 전환(계약서_OAK전환_20261104 §10)
        # ============================================================
        # 파라미터는 config/oak_640x360_full.yaml (= tools/oak/oak_640x360_full.yaml, 런북 10-31 실측):
        #   i_ip 169.254.1.222 고정 연결(자동탐색 실패), RGB 단독(뎁스·NN 없음 — OAK-1 은 모노 카메라),
        #   1080P 센서 30 fps, i_low_bandwidth(MJPEG, 100 Mb 링크에서도 26 fps),
        #   🚨 i_isp_num 1 / i_isp_den 3 — 1080P 전체 → 640×360 축소 = HFOV 63.9°(vision.yaml hfov_deg).
        #      빠지면 가운데만 잘려 40.3° 인데 camera_info fx 는 같아 숫자로 안 드러난다. 시야 육안 확인 필수.
        #   test_oak_camera.py 가 이 값들을 정적으로 검사한다.
        # 발행 QoS 는 RELIABLE — 검출기·healthcheck 는 BEST_EFFORT 구독이라 호환(CLAUDE.md §3).
        # 연결에 13~17 s 걸린다. 그동안 검출기는 침묵, healthcheck 는 CAM 대기(image_wait_grace_s 안).
        # 패키지가 없으면 _oak_camera 가 LogInfo 만 찍고 건너뛴다(launch 전체를 세우지 않는다).
        #
        # D455 시절 기록(auto WB 실측, saturation 64, HID udev 로 IMU OFF, 뎁스 ON 시 USB 끊김)은
        # camera.launch.py 와 git 이력 참고 — RealSense 경로는 CLAUDE.md §1-8 대로 유지.
        # 11-12 부터 D455 는 **뎁스 전용**으로 다시 들어온다(아래 7b, 컬러 없음 — 비전 검출은 OAK 그대로).
        OpaqueFunction(function=_oak_camera),

        # ============================================================
        # 7b) D455 뎁스 전용 (d455_depth:=true 기본, 계약서_인식융합_20261112 §10 Q9)
        # ============================================================
        # 🚨 camera.launch.py(물가 영상·벤치)와 동시에 띄우지 말 것 — 같은 이름 /camera/camera (_d455_depth 참고).
        # 09-10 실측: 컬러+뎁스 동시엔 10.8 fps·끊김 13회 → 여기선 컬러를 끄고 뎁스만 480x270x15
        # (424x240 은 2번배 USB2 목록에 없음 → U0 에서 480x270, 계약서_도리도리회피_v2_20261114).
        # 패키지가 없으면 _d455_depth 가 LogInfo 만 찍고 건너뛴다.
        OpaqueFunction(function=_d455_depth),

        # ============================================================
        # 8) 비전 검출기 — **상주**. 각자 /wp_mode 를 보고 자기 차례일 때만 일한다
        # ============================================================
        # 🚨 3-4: 작년엔 subscriber_mode_manager 가 subprocess 로 이 노드들을 죽였다 살렸다.
        #    모드 전환마다 비전이 몇 초 멈추고, 좀비가 남으면 카메라가 잠기고, 추적기를 못 썼다.
        #    지금은 상주 + 모드 게이팅이다. 매니저는 폐기했다(entry point 도 제거).
        #
        #    담당 모드는 **각 노드가 소유**한다 (미션 노드와 같은 패턴).
        #    권위 출처 = 미션 노드의 active_wp_mode(s). 바꿀 땐 양쪽을 함께 바꿔라:
        #      mode 0,1 → ship_gate  ← gate 검출기 (/red_angle, /green_angle)
        #      mode 2   → ship_back  ← turn 검출기 (turn_color_mode2 부표, /image_angle)
        #      mode 3   → ship_turn(orbit 기본) / ship_back(search_behavior:=hold)
        #                            ← turn 검출기 (turn_color_mode3 부표) — 검출기 쪽은 스위치와 무관
        #      mode 7   → ship_dock  ← dock 검출기 (/image_angle)
        #      mode 5   → ship_avoid (avoid_mode 기본 fused, 구역 신호·검출기 전엔 20000) ← 검출기 없음(U3 전까지) → 검출기 전부 비활성
        #      mode 8   → 순수 회피(담당 없음, north 20000 폴백) → 검출기 전부 비활성
        #
        #    🚨 dock 과 turn 은 둘 다 /image_angle 을 발행한다. 모드가 겹치면 한 토픽에
        #       발행자 2개가 되어 **에러 없이** 값이 섞인다. 겹침 없음은
        #       test_mode_gate.py 의 check_publisher_conflicts 가 정적으로 검사한다.
        Node(package='color_shape_detector', executable='basic_image_subscribergate',
             name='image_subscriber_gate', output='screen', parameters=[VISION_CFG]),
        Node(package='color_shape_detector', executable='basic_image_subscriberturn',
             name='image_subscriber_turn', output='screen', parameters=[VISION_CFG]),
        Node(package='color_shape_detector', executable='basic_image_subscriberdock',
             name='image_subscriber_dock', output='screen', parameters=[VISION_CFG]),
        # basic_image_subscriberhsv 는 튜닝 전용 — 대회 launch 에 넣지 않는다.

        # ============================================================
        # 8b) perception_fusion — LiDAR 군집 + D455 뎁스 + 검출기 방위 → /fused_objects (구독·관찰 전용)
        # ============================================================
        # perception_fusion:=true 기본(§10 Q8). 미션 노드는 yaml distance_source(lidar|fill|fused)로 쓸지 정한다.
        OpaqueFunction(function=_perception_fusion),

        # ============================================================
        # 9) SHIP mission nodes (WP 루틴)
        # ============================================================
        # 9)~11) 의 파라미터 출처 = 각 패키지 config/<pkg>.yaml (_pkg_yaml, 11-05 부터).
        Node(package='ship_gate', executable='ship_gate', name='ship_gate', output='screen',
             parameters=[_pkg_yaml('ship_gate')]),
        Node(package='ship_dock', executable='ship_dock', name='ship_dock', output='screen',
             parameters=[_pkg_yaml('ship_dock')]),
        # ship_turn·ship_back 은 search_behavior 인자에 따라 파라미터가 달라 _search_nodes 가 띄운다.
        OpaqueFunction(function=_search_nodes),
        # 9b) ship_avoid — mode 5 담당(기본 avoid_mode fused, 구역 신호·검출기 전엔 20000). 패키지 없으면 _ship_avoid 가 건너뛴다.
        OpaqueFunction(function=_ship_avoid),

        # ============================================================
        # 10) Fusion: 라이다 기반 항로 유지 / 11) Motor Control
        # ============================================================
        # course(코스 yaml)·boat(dwa_hull) 에 따라 parameters 가 달라 _control_nodes 가 띄운다.
        OpaqueFunction(function=_control_nodes),

        # ============================================================
        # 12) 시리얼 브릿지 (Motor_run → 아두이노 Mega, 상태 → /boat_mode)
        # ============================================================
        # 🚨 이게 없으면 motor_control 이 Motor_run 을 내도 **받는 쪽이 없다.**
        #    사슬의 마지막 조각이다: motor_control → Motor_run → [브릿지] → Mega
        # 아두이노가 아직 없어도 안전하다 — 브릿지 launch 가 포트 존재를 먼저 보고
        # 없으면 노드를 안 띄우고 넘어간다(예외를 던지면 launch 전체가 죽는다.
        # ntrip_client 가 파일 하나 못 찾아 노드 15개를 같이 죽인 사고가 실제로 있었다).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dir_bridge, 'launch', 'ssf_bridge.launch.py')
            )
        ),

        # ============================================================
        # 13) blackbox — CSV 기록 (구독 전용, 죽어도 다른 노드 영향 없음)
        # ============================================================
        # record:=false 면 안 뜬다. ssf_tools 가 없으면 _blackbox 가 LogInfo 만 찍고 건너뛴다.
        OpaqueFunction(function=_blackbox),
    ])

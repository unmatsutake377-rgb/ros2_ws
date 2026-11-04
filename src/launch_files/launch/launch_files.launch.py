from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction, DeclareLaunchArgument,
                            LogInfo, OpaqueFunction)
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
        # 🚨 이 검사는 t=0, 어떤 Node 보다 먼저 돈다(목록 맨 앞) → 아무 프로세스도 안 뜬 채 멈춘다.
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


def search_behavior_params(behavior):
    """launch 인자 search_behavior → ship_back·ship_turn 에 넘길 파라미터 (순수 함수, ROS 없이 테스트).

    hold(기본, 2026 안내문: 탐색 = 부표 앞 위치유지) = 2,3→ship_back, ship_turn 침묵.
    orbit(2025 규정 선회 대비 옵션) = 2→ship_back, 3→ship_turn.
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
    # 기본 hold(2026 안내문). 현장에서 선회(2025 규정)로 돌리려면 search_behavior:=orbit.
    behavior = LaunchConfiguration('search_behavior').perform(context)
    params = search_behavior_params(behavior)
    if behavior == 'hold':
        msg = 'search_behavior=hold(기본): 탐색(3) = ship_back 위치유지(모드 2·3 담당), ship_turn 침묵'
    else:
        msg = 'search_behavior=orbit: 탐색(3) = ship_turn 선회(2025 규정 대비 옵션), ship_back 은 모드 2'
    actions = [LogInfo(msg=msg)]
    actions += [
        Node(package='ship_turn', executable='ship_turn', name='ship_turn', output='screen',
             parameters=[params['ship_turn']]),
        # ship_last 제거됨(6b): mode 0 을 ship_gate 가 인수. 하던 일은 /candidate_angle 에 20000
        #   폴백을 내는 것뿐이라 중복이었다. (mode 5,8 폴백은 north_goal_angle 이 담당)
        Node(package='ship_back', executable='ship_back', name='ship_back', output='screen',
             parameters=[params['ship_back']]),
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
        #    지금은 yaw_mux 의 배별 IMU 값(ssf_heading_A/B.yaml)만 고른다. GPS 는 아래 gps_config 따로.
        #    ros2 launch launch_files launch_files.launch.py boat:=A   (또는 boat:=B)
        #    기본값을 두면 다른 배에서 그 배 값으로 조용히 뜬다(gps_config 가 이미 그렇게 실패 중).
        DeclareLaunchArgument(
            'boat', choices=['A', 'B'],
            description='배 (필수): A 또는 B — yaw_mux 배별 IMU 값 파일 선택'),
        DeclareLaunchArgument(
            'gps_config', default_value='zed_f9p_rover.yaml',
            description='GPS 설정 — 두 배 모두 ZED-F9P 라 기본값 zed_f9p_rover.yaml'),
        # 탐색(mode 3) 행동. 기본 hold — 2026 안내문 코스도 "특정 색상 부표 탐지 후 그 위치에서 위치유지"
        #   (팀장 확정 2026-11-04): ship_back 이 모드 2·3 담당, ship_turn 침묵.
        #   orbit 은 2025 규정(원 선회) 대비 옵션. 현장에서 선회로 돌리려면:
        #   ros2 launch launch_files launch_files.launch.py boat:=A search_behavior:=orbit
        DeclareLaunchArgument(
            'search_behavior', default_value='hold', choices=['orbit', 'hold'],
            description='탐색(wp_mode 3) 행동: hold=ship_back 위치유지, ship_turn 침묵(기본, 2026 안내문) / '
                        'orbit=ship_turn 선회(2025 규정 대비 옵션)'),

        # ============================================================
        # 0) yaw_mux — /imu/yaw 의 단독 발행자 (params: 공통 → boat 별, _yaw_mux 참고)
        # ============================================================
        # 🚨 맨 앞에 둔다: 배별 파일 확인(raise)이 어떤 노드도 예약되기 전에 돌게.
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dir_north, 'launch', 'north_goal_angle_launch.py')
            )
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
        # camera.launch.py 와 git 이력 참고 — RealSense 경로는 CLAUDE.md §1-8 대로 유지, 대회 launch 에서만 뺐다.
        OpaqueFunction(function=_oak_camera),

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
        #      mode 2   → ship_back  ← turn 검출기 (white 부표, /image_angle)
        #      mode 3   → ship_back(hold 기본) / ship_turn(search_behavior:=orbit)
        #                            ← turn 검출기 (red 부표) — 검출기 쪽은 스위치와 무관
        #      mode 7   → ship_dock  ← dock 검출기 (/image_angle)
        #      mode 5,8 → 순수 회피(담당 없음) → 검출기 전부 비활성
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
        # 9) SHIP mission nodes (WP 루틴)
        # ============================================================
        Node(package='ship_gate', executable='ship_gate', name='ship_gate', output='screen'),
        Node(package='ship_dock', executable='ship_dock', name='ship_dock', output='screen'),
        # ship_turn·ship_back 은 search_behavior 인자에 따라 파라미터가 달라 _search_nodes 가 띄운다.
        OpaqueFunction(function=_search_nodes),

        # ============================================================
        # 10) Fusion: 라이다 기반 항로 유지
        # ============================================================
        Node(
            package='ship_direction',
            executable='ship_direction',
            name='ship_direction',
            output='screen'
        ),

        # ============================================================
        # 11) Motor Control
        # ============================================================
        Node(
            package='motor_control',
            executable='motor_control',
            name='motor_control',
            output='screen'
        ),

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
    ])

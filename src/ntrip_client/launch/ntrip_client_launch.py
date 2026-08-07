"""NTRIP 클라이언트 launch — 계정 정보는 **저장소 밖 로컬 파일**에서 읽는다.

  기본 경로 : ~/.ssf/ntrip_credentials.yaml
  템플릿    : src/ntrip_client/config/ntrip_credentials.example.yaml
  경로 변경 : 환경변수 SSF_NTRIP_CREDENTIALS 또는 credentials_file:= 인자

🚨 왜 이렇게 바꿨나 (2026-08-06)
   예전엔 작년 계정이 이 파일에 **평문으로 하드코딩**돼 공개 저장소에 커밋돼 있었다
   (host/mountpoint/username/password 전부). docs/절차/ubuntu_setup.md 9단계는
   "계정 정보는 GitHub 에 없다 — 노트북 로컬에만" 이라고 적어뒀는데 지켜지지 않았다.
   받아놓을 그릇이 없으면 사람은 있는 자리에 넣는다 → 저장소 밖에 그릇을 만든다.

🚨 파일이 없으면 **노드를 띄우지 않고 에러 로그만** 남긴다 (예외를 던지지 않는다).
   IncludeLaunchDescription 안에서 예외가 나면 **launch 전체가 내려간다** — 이미 뜬
   노드까지 SIGINT 로 같이 죽는다. 같은 날 ntrip launch 경로 문제로 실제로 당했다
   (노드 16개가 통째로 종료됐다). RTK 보정이 없는 것과 배 전체가 안 뜨는 것은 무게가 다르다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml

DEFAULT_CREDENTIALS = os.path.join(
    os.path.expanduser('~'), '.ssf', 'ntrip_credentials.yaml')


def _resolve_path(context):
    """credentials_file 인자 > 환경변수 > 기본 경로."""
    arg = LaunchConfiguration('credentials_file').perform(context).strip()
    if arg:
        return os.path.expanduser(arg)
    env = os.environ.get('SSF_NTRIP_CREDENTIALS', '').strip()
    if env:
        return os.path.expanduser(env)
    return DEFAULT_CREDENTIALS


def _missing(path, reason):
    """노드 없이 안내만 남긴다. launch 전체를 죽이지 않는다."""
    example = os.path.join(
        get_package_share_directory('ntrip_client'),
        'config', 'ntrip_credentials.example.yaml')
    return [LogInfo(msg=(
        f"\n🛰️ [ntrip_client] {reason}\n"
        f"   찾은 경로: {path}\n"
        f"   → NTRIP(RTK 보정) 없이 계속 진행합니다. 나머지 노드는 정상 동작합니다.\n"
        f"   계정을 받으셨으면:\n"
        f"     mkdir -p ~/.ssf\n"
        f"     cp {example} {DEFAULT_CREDENTIALS}\n"
        f"     nano {DEFAULT_CREDENTIALS}   # 실제 값으로 채우기\n"))]


def _launch_setup(context, *args, **kwargs):
    path = _resolve_path(context)

    if not os.path.isfile(path):
        return _missing(path, "계정 파일이 없습니다.")

    try:
        with open(path, 'r', encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
    except Exception as e:                       # noqa: BLE001 — 어떤 파싱 오류든 안내로 바꾼다
        return _missing(path, f"계정 파일을 읽지 못했습니다: {e}")

    cred = doc.get('ntrip') or {}
    host = str(cred.get('host', '') or '').strip()
    mountpoint = str(cred.get('mountpoint', '') or '').strip()
    if not host or not mountpoint:
        return _missing(path, "계정 파일에 host 또는 mountpoint 가 비어 있습니다.")

    # 빈 문자열은 이 드라이버가 'None' 문자열로 기대하는 자리들이 있다 → 그대로 맞춰준다.
    def opt(key):
        v = str(cred.get(key, '') or '').strip()
        return v if v else 'None'

    params = {
        'host': host,
        'port': int(cred.get('port', 2101)),
        'mountpoint': mountpoint,
        'ntrip_version': opt('ntrip_version'),
        'authenticate': bool(cred.get('authenticate', True)),
        'username': str(cred.get('username', '') or ''),
        'password': str(cred.get('password', '') or ''),
        'ssl': bool(cred.get('ssl', False)),
        'cert': opt('cert'),
        'key': opt('key'),
        'ca_cert': opt('ca_cert'),

        # ---- 아래는 계정과 무관한 고정 설정 (저장소에 남겨도 되는 것들) ----
        'rtcm_frame_id': 'odom',
        'nmea_max_length': 82,
        'nmea_min_length': 3,
        'rtcm_message_package': LaunchConfiguration('rtcm_message_package').perform(context),
        # 재연결: 5초 간격 10회. ⚠️ 이건 '세션이 끊겼을 때' 다 —
        #   404(마운트포인트 오류) 같은 설정 오류는 재시도 대상이 아니라 바로 종료된다.
        'reconnect_attempt_max': 10,
        'reconnect_attempt_wait_seconds': 5,
        'rtcm_timeout_seconds': 4,
    }

    return [
        LogInfo(msg=f"🛰️ [ntrip_client] 계정 로드: {path}  (host={host}, mount={mountpoint})"),
        Node(
            name=LaunchConfiguration('node_name').perform(context),
            namespace=LaunchConfiguration('namespace').perform(context),
            package='ntrip_client',
            executable='ntrip_ros.py',
            parameters=[params],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/'),
        DeclareLaunchArgument('node_name', default_value='ntrip_client'),
        DeclareLaunchArgument('debug', default_value='True'),
        DeclareLaunchArgument('rtcm_message_package', default_value='rtcm_msgs'),
        DeclareLaunchArgument(
            'credentials_file', default_value='',
            description=f'NTRIP 계정 yaml 경로 (비우면 {DEFAULT_CREDENTIALS})'),

        SetEnvironmentVariable(name='NTRIP_CLIENT_DEBUG',
                               value=LaunchConfiguration('debug')),

        OpaqueFunction(function=_launch_setup),
    ])

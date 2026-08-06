# Copyright 2020 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
# * Neither the name of {copyright_holder} nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the ublox gps node. 배별 config 를 인자로 고른다.

  ros2 launch ublox_gps ublox_gps_node-launch.py                              # B배 (기본, ZED-F9P)
  ros2 launch ublox_gps ublox_gps_node-launch.py config:=c94_m8p_rover.yaml   # A배 (C94-M8P)

🚨 원래는 c94_m8p_rover.yaml 이 하드코딩돼 있었다. 배가 2척(A=M8P, B=F9P)인데 launch 가
   한 배에 고정돼 있어서, F9P 를 꽂으면 **에러 없이 M8P 설정으로 돌았다**(2026-08-06 실기 발견).
   기본값을 실제 보유 장비(F9P)로 두는 이유: launch_files.launch.py 가 이 파일을 그대로
   include 하므로, 기본이 틀리면 대회 실행 경로에서 같은 사고가 조용히 재발한다.
"""

import os

import ament_index_python.packages
import launch
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ublox_gps'),
        'config')

    config_arg = launch.actions.DeclareLaunchArgument(
        'config',
        default_value='zed_f9p_rover.yaml',
        description='배별 GPS 설정 파일 (B배: zed_f9p_rover.yaml / A배: c94_m8p_rover.yaml)')

    params = launch.substitutions.PathJoinSubstitution(
        [config_directory, launch.substitutions.LaunchConfiguration('config')])

    ublox_gps_node = launch_ros.actions.Node(package='ublox_gps',
                                             executable='ublox_gps_node',
                                             output='both',
                                             parameters=[params])

    return launch.LaunchDescription([config_arg,
                                     ublox_gps_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=ublox_gps_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])

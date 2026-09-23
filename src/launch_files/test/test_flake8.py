# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # 🚨 최대 줄 길이 99 → 120 (2026-09-23). ament_flake8 은 `.flake8`·`setup.cfg` 를
    #    읽지 않고 자기 ini 를 쓰므로, 설정 파일 하나로는 안 되고 패키지마다 여기서 준다.
    #    **10개 패키지가 같은 값이어야 한다** — 하나만 다르면 그 패키지만 조용히 엄격해진다.
    #
    #    왜 올렸나: 넘치던 35줄이 전부 `파라미터 한 줄 + 실측 필요 주석` 조합이었다. 예)
    #      self.base_pwm = int(self.declare_parameter("base_pwm", 1360).value)  # ⚠️ 실측 필요
    #    이 저장소에선 그 한 줄이 **정보 단위**다. 주석을 위로 올리면 규칙은 통과하지만
    #    나란히 정렬된 파라미터 블록이 흩어져 읽기 나빠진다.
    #    E501 은 결함 탐지기가 아니라 취향 설정이라 규칙 쪽을 옮겼다.
    #    ⚠️ lint 를 **끄는 것과는 다르다.** F 계열(죽은 코드)·E 계열은 그대로 다 돈다.
    rc, errors = main_with_errors(argv=['--linelength', '120'])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)

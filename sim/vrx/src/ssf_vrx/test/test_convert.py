"""순수 변환 함수 테스트 — ROS 없이 돈다. 실행: cd sim/vrx && python3 -m pytest src/ssf_vrx/test -q"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from ssf_vrx.convert import (command_is_stale, decode_motor_run, heading_from_quat,  # noqa: E402
                             heading_from_yaw_enu, pwm_to_thrust, yaw_enu_from_heading)


def _quat_yaw(yaw_rad):
    return (0.0, 0.0, math.sin(yaw_rad / 2), math.cos(yaw_rad / 2))


def test_decode_motor_run_matches_bridge_rule():
    assert decode_motor_run(13841336) == (1336, 1384)      # r*10000 + l
    assert decode_motor_run(15001500) == (1500, 1500)
    assert decode_motor_run(9000900) == (1000, 1000)        # 아래로 자름
    assert decode_motor_run(25002500) == (2000, 2000)       # 위로 자름


def test_pwm_to_thrust_sign_and_deadband():
    assert pwm_to_thrust(1500) == 0.0
    assert pwm_to_thrust(1475) == 0.0 and pwm_to_thrust(1525) == 0.0
    assert pwm_to_thrust(1360) > 0.0           # ROS 규약: <1500 전진
    assert pwm_to_thrust(1590) < 0.0           # 후진
    assert pwm_to_thrust(1100) == pytest.approx(47.0)
    assert pwm_to_thrust(900) == pytest.approx(47.0)        # 범위 밖은 끝값
    assert pwm_to_thrust(1900) == pytest.approx(-35.0)
    assert pwm_to_thrust(2000) == pytest.approx(-35.0)


def test_pwm_to_thrust_base_and_reverse_values():
    # base_pwm 1360 → (1475-1360)/375 = 0.3067 → 14.4 N
    # reverse_pwm 1590 → (1590-1525)/375 → -6.1 N
    assert pwm_to_thrust(1360) == pytest.approx(47.0 * 115 / 375, rel=1e-6)
    assert pwm_to_thrust(1590) == pytest.approx(-35.0 * 65 / 375, rel=1e-6)
    # 단조: 더 작은 PWM 이 더 큰 전진 추력
    assert pwm_to_thrust(1300) > pwm_to_thrust(1360) > pwm_to_thrust(1400)


def test_heading_from_enu_quaternion():
    # ENU yaw 0 (동쪽) → 나침반 90°, yaw 90°(북) → 0°, yaw -90°(남) → 180°, yaw 180°(서) → 270°
    assert heading_from_quat(*_quat_yaw(0.0)) == pytest.approx(90.0)
    assert heading_from_quat(*_quat_yaw(math.pi / 2)) == pytest.approx(0.0, abs=1e-9)
    assert heading_from_quat(*_quat_yaw(-math.pi / 2)) == pytest.approx(180.0)
    assert heading_from_quat(*_quat_yaw(math.pi)) == pytest.approx(270.0)
    # 시계방향 증가: 북(0)에서 오른쪽으로 30° 돌면 30°
    assert heading_from_quat(*_quat_yaw(math.radians(60))) == pytest.approx(30.0)


@pytest.mark.parametrize('hdg', [0.0, 30.0, 90.0, 179.9, 180.0, 270.0, 359.0])
def test_heading_roundtrip(hdg):
    assert heading_from_yaw_enu(yaw_enu_from_heading(hdg)) == pytest.approx(hdg % 360.0, abs=1e-9)


def test_command_watchdog():
    assert command_is_stale(10.0, None, 0.5)            # 첫 명령 전
    assert not command_is_stale(10.4, 10.0, 0.5)
    assert command_is_stale(10.6, 10.0, 0.5)

"""추력 방향 반전 검산 — bridge._send 의 1500 대칭 변환.

ROS 규약(motor_control) :  1500=정지, <1500=전진, >1500=후진
실물(08-27 실측)        :  >1500 = 전진
→ ssf_bridge 가 시리얼로 내보낼 때만 1500 대칭으로 뒤집는다.

ROS 안쪽 규약(Motor_run·/motor_reverse·blackbox)은 건드리지 않는다.
"""
import pytest

from ssf_bridge.status_parser import PWM_MAX, PWM_MIN, clamp_pwm


def mirror(pwm):
    """bridge._send 와 같은 식."""
    return 3000 - pwm


def test_중립은_그대로다():
    """워치독·정지 경로가 흔들리면 안 된다."""
    assert mirror(1500) == 1500


@pytest.mark.parametrize("ros, serial_", [
    (1360, 1640),   # base_pwm 순항 전진
    (1590, 1410),   # reverse_pwm 후진
    (1100, 1900),   # 최대 전진
    (1900, 1100),   # 최대 후진
])
def test_전후진이_뒤집힌다(ros, serial_):
    assert mirror(ros) == serial_


def test_전진은_1500보다_커진다():
    """ROS 에서 전진(<1500) 이면 시리얼로는 >1500 이어야 한다 — 이게 이번 수정의 목적."""
    assert mirror(1360) > 1500
    assert mirror(1590) < 1500


def test_조향_차동이_보존된다():
    """전진+우회전: 좌가 더 전진해야 한다.

    ROS 에서는 '더 전진' = 더 작은 값, 시리얼에서는 '더 전진' = 더 큰 값.
    """
    ros_l, ros_r = 1300, 1400          # 좌가 더 전진
    s_l, s_r = mirror(ros_l), mirror(ros_r)
    assert ros_l < ros_r               # ROS 규약에서 좌가 더 전진
    assert s_l > s_r                   # 시리얼 규약에서도 좌가 더 전진
    assert (s_l - 1500) == -(ros_l - 1500)


def test_제자리선회가_보존된다():
    """우선회: 좌 전진 · 우 후진. 대칭 반전 뒤에도 같은 회전이어야 한다."""
    ros_l, ros_r = 1400, 1600          # 좌 전진 · 우 후진 (ROS 규약)
    s_l, s_r = mirror(ros_l), mirror(ros_r)
    assert s_l == 1600 and s_r == 1400  # 시리얼 규약에서 좌 전진 · 우 후진
    assert (s_l - 1500) * (s_r - 1500) < 0   # 여전히 서로 반대


def test_반전해도_클램프_범위를_안_벗어난다():
    """PWM_MIN~PWM_MAX 안의 값은 반전해도 안에 남는다 (1500 대칭이므로)."""
    for v in range(PWM_MIN, PWM_MAX + 1, 10):
        m = mirror(v)
        assert PWM_MIN <= m <= PWM_MAX
        assert clamp_pwm(m) == m       # 클램프가 값을 깎지 않는다


def test_두_번_반전하면_원래대로():
    for v in (1100, 1360, 1500, 1590, 1900):
        assert mirror(mirror(v)) == v

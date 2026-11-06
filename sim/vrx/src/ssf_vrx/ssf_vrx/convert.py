"""순수 변환 함수 — ROS 없이 테스트한다 (test/test_convert.py).

규약 (CLAUDE.md §3 와 같다):
  · Motor_run = pwm_r*10000 + pwm_l. ROS 안에서는 <1500 = 전진, >1500 = 후진 (실물 ESC 와 반대 —
    실물은 ssf_bridge 가 3000-pwm 으로 뒤집지만, 시뮬은 ssf_bridge 를 띄우지 않고 여기서 바로 N 으로 바꾼다).
  · 헤딩: 0~360°, 0 = 진북, 시계방향 (iahrs /imu/yaw_raw 와 같은 뜻). Gazebo IMU 는 ENU 쿼터니언
    (yaw 0 = 동쪽, 반시계 양수) → heading = (90 - yaw_enu) mod 360.
  · 추력: T200 14.8 V 보간 최대 약 47 N 전진(보고서 Ⅱ-3.4), 후진은 약 35 N 으로 둔다(제조사 역추력 비율).
    PWM→추력은 **1차 근사**다(실제 T200 곡선은 비선형). 수조 실측 뒤 표로 바꾼다.
"""
import math

PWM_NEUTRAL = 1500
PWM_MIN, PWM_MAX = 1100, 1900
DEFAULT_DEADBAND = 25          # 1475~1525 는 정지 (ESC 데드밴드)
DEFAULT_FWD_MAX_N = 47.0
DEFAULT_REV_MAX_N = 35.0


def decode_motor_run(data):
    """Motor_run 정수 → (pwm_l, pwm_r). ssf_bridge status_parser 와 같은 규칙(1000~2000 으로 자름)."""
    data = int(data)
    pwm_r = data // 10000
    pwm_l = data - pwm_r * 10000
    clamp = (lambda v: max(1000, min(2000, v)))
    return clamp(pwm_l), clamp(pwm_r)


def pwm_to_thrust(pwm, fwd_max_n=DEFAULT_FWD_MAX_N, rev_max_n=DEFAULT_REV_MAX_N,
                  deadband=DEFAULT_DEADBAND):
    """ROS 규약 PWM(µs) → 추력[N]. 양수 = 전진.

    전진: 1500-deadband 에서 0, 1100 에서 fwd_max. 후진: 1500+deadband 에서 0, 1900 에서 -rev_max.
    범위 밖은 끝값으로 자른다. 데드밴드 안은 0.
    """
    pwm = float(pwm)
    lo = PWM_NEUTRAL - deadband
    hi = PWM_NEUTRAL + deadband
    if pwm < lo:
        span = lo - PWM_MIN                      # 375
        f = min(1.0, (lo - pwm) / span)
        return fwd_max_n * f
    if pwm > hi:
        span = PWM_MAX - hi
        f = min(1.0, (pwm - hi) / span)
        return -rev_max_n * f
    return 0.0


def yaw_enu_from_quat(x, y, z, w):
    """쿼터니언 → ENU yaw[rad] (z축 회전, 반시계 양수)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def heading_from_yaw_enu(yaw_enu_rad):
    """ENU yaw(0=동, 반시계) → 나침반 헤딩(0=북, 시계, 0~360)."""
    return (90.0 - math.degrees(yaw_enu_rad)) % 360.0


def heading_from_quat(x, y, z, w):
    return heading_from_yaw_enu(yaw_enu_from_quat(x, y, z, w))


def yaw_enu_from_heading(heading_deg):
    """역변환: 나침반 헤딩[deg] → ENU yaw[rad]. 월드 생성기가 배 초기 자세에 쓴다."""
    return math.radians((90.0 - heading_deg + 180.0) % 360.0 - 180.0)


def command_is_stale(now, last_cmd_t, timeout_sec):
    """펌웨어 CMD_TIMEOUT_MS(500) 와 같은 뜻: 첫 명령 전이거나 timeout 넘게 안 오면 참(→ 추력 0)."""
    return last_cmd_t is None or (now - last_cmd_t) > timeout_sec

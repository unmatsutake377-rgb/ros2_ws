"""ship_turn 의 orbit 제어 로직 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

    from ship_turn.orbit_logic import orbit_steer, OrbitProgress, orbit_direction_cw

부표를 중심으로 원을 그리며 도는 orbit 기동. 제어법·진행각 누적은 틀리기 쉬워서
노드에서 떼어내 단위 테스트한다.

각도 규약: 상대각(0 = 정면, + = 우현/오른쪽). 시계방향(CW) = 위에서 볼 때 시계.
"""

import math


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def clamp_pm180(a):
    return (a + 180.0) % 360.0 - 180.0


def wrap360(a):
    return (a % 360.0 + 360.0) % 360.0


def orbit_direction_cw(color):
    """색 → orbit 방향. 규정: 빨강·초록 = 시계(CW) / 흰색 = 반시계(CCW).
    → True(CW) / False(CCW) / None(모름)."""
    c = (color or "").strip().lower()
    if c in ("red", "green"):
        return True
    if c in ("white",):
        return False
    return None


def orbit_steer(buoy_rel, dist, orbit_radius, direction_cw, k_radial, radial_limit):
    """orbit 조향각(상대, 0=정면)을 낸다. 순수 함수.

    buoy_rel     : 부표 상대각(deg, 0=정면, +=우현)
    dist         : 부표까지 거리(m, LiDAR)
    direction_cw : True=시계, False=반시계

    ★ orbit 원리:
      · 시계(CW)로 돌려면 부표(원 중심)를 **오른쪽(+90°)**에 둔다. 진행 방향(접선) = buoy_rel - 90.
        (반시계는 부표를 왼쪽 -90°, 접선 = buoy_rel + 90)
      · 반경 보정: 너무 멀면(dist>radius) 부표 쪽으로, 너무 가까우면 반대로 튼다.
        접선에 s*gain 을 더한다 (s=+1 CW). gain>0 이면 부표 쪽(원 안쪽)으로 향한다.
    """
    s = 1.0 if direction_cw else -1.0
    tangent = clamp_pm180(buoy_rel - 90.0 * s)
    err = dist - orbit_radius                      # >0 너무 멀다 → 안쪽으로
    gain = clamp(k_radial * err, -radial_limit, radial_limit)
    return clamp_pm180(tangent + s * gain)


class OrbitProgress:
    """부표의 **월드 방위**(yaw + 상대각) 변화를 누적해 orbit 진행각(deg)을 잰다.

    배가 부표를 CW 로 돌면 '부표를 향한 월드 방위'가 증가하고, CCW 면 감소한다.
    누적 절대값이 orbit_target_deg 에 닿으면 한 바퀴 완료.
    (yaw 가 없으면 진행각을 못 잰다 → 노드가 orbit_max_sec 안전 타임아웃으로 마무리한다.)
    """

    def __init__(self):
        self.prev = None
        self.total = 0.0        # 부호 있는 누적 (CW +, CCW -)

    def reset(self):
        self.prev = None
        self.total = 0.0

    def update(self, world_bearing_deg):
        """월드 방위(0~360)를 넣으면 진행각(deg, 절대값)을 반환."""
        if self.prev is None:
            self.prev = world_bearing_deg
            return 0.0
        d = clamp_pm180(world_bearing_deg - self.prev)   # 부호 있는 증분 [-180,180]
        self.prev = world_bearing_deg
        self.total += d
        return abs(self.total)

    def progress_deg(self):
        return abs(self.total)

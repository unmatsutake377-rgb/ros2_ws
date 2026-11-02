"""ship_back 의 위치유지 로직 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

    from ship_back.hold_logic import station_keep_action, HoldTimer, should_reset_on_mode

규정: '부표 5m 이내 5초 유지'. 작년 코드는 그냥 5초간 PWM 중립이라 조류에 밀려 실패했다.
→ LiDAR 로 잰 거리로 전진/후진을 결정해 능동적으로 위치를 유지한다(bang-bang + 데드밴드).
"""


def should_reset_on_mode(prev_mode, cur_mode, active_modes):
    """/wp_mode 전이 때 FSM(INIT·DONE 래치·HoldTimer)을 리셋할지. 순수 함수.

    prev_mode    : 직전 /wp_mode (부팅 초기값 -1 포함).
    cur_mode     : 방금 받은 /wp_mode.
    active_modes : 이 노드가 담당하는 모드 목록 (파라미터 active_wp_modes).

    규칙: `prev != cur and cur in active_modes`.
      ★ `prev not in active_modes` 로 쓰면 안 된다 — launch 가 hold 로 [2, 3] 을 넘길 때
        2→3 전이에서 prev(2)가 목록 안이라 리셋이 안 되고, mode 2 에서 DONE 으로 래치된 채
        mode 3 부표를 건너뛴다. 같은 모드가 연속으로 오면(3→3) 리셋하지 않는다(래치 유지).
    """
    return prev_mode != cur_mode and cur_mode in active_modes


def station_keep_action(dist, hold_dist, tol):
    """부표까지 거리 → 동작. 순수 함수.

    dist      : 부표까지 거리(m, LiDAR). None 이면 'unknown'.
    hold_dist : 유지할 목표 거리(m).
    tol       : 데드밴드(±). 이 안에서는 'hold'(중립) — 벗어나면 되돌린다.

    → 'forward'(너무 멀다, 부표로 전진) / 'reverse'(너무 가깝다, 후진) / 'hold'(유지) / 'unknown'

    ★ 데드밴드가 있어야 목표 근처에서 전/후진이 딸깍딸깍 진동하지 않는다.
      조류에 밀려 tol 을 벗어나면 그때 되돌린다 → 표류가 ~tol 로 제한된다(작년은 무제한 표류).
    """
    if dist is None:
        return 'unknown'
    if dist > hold_dist + tol:
        return 'forward'
    if dist < hold_dist - tol:
        return 'reverse'
    return 'hold'


class HoldTimer:
    """부표 keep_radius 이내에 **연속으로** 머문 시간을 잰다(규정 판정).

    keep_radius 를 벗어나면 리셋(연속이 끊긴다). 시각(now)은 주입 — 테스트 가능.
    """

    def __init__(self, keep_radius, hold_time):
        self.keep_radius = float(keep_radius)
        self.hold_time = float(hold_time)
        self._enter = None      # 반경 안에 처음 들어온 시각

    def reset(self):
        self._enter = None

    def update(self, dist, now):
        """반경 안이면 연속 유지시간(초)을 반환, 밖이면 0 으로 리셋."""
        if dist is None or dist > self.keep_radius:
            self._enter = None
            return 0.0
        if self._enter is None:
            self._enter = now
        return now - self._enter

    def satisfied(self, dist, now):
        return self.update(dist, now) >= self.hold_time

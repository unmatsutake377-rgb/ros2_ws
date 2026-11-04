"""ship_back 의 위치유지 로직 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

    from ship_back.hold_logic import station_keep_action, HoldTimer, should_reset_on_mode
    from ship_back.hold_logic import target_source, counts_as_seen, search_timed_out
    from ship_back.hold_logic import unknown_giveup, search_timeout_warn, hold_total_exceeded

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


def target_source(state, vision_fresh, since_seen, lost_sec, lidar_acquire):
    """부표 (거리, 방위)를 어디서 얻을지. 순수 함수. → 'vision' / 'lidar' / None(모른다).

    state         : FSM 상태 문자열('SEARCH'·'HOLD' …).
    vision_fresh  : /image_angle(검출기 turn 의 모드별 색) 방위가 angle_stale_sec 안에 있다.
    since_seen    : 마지막으로 부표를 '봤다'고 센 뒤 지난 시간(s). 본 적 없으면 None.
                    lidar_acquire=false 면 비전만 '봤다'로 세므로 = 직전 비전 방위 이후 시간.
    lost_sec      : 이 시간 넘게 못 보면 HOLD → SEARCH (파라미터 lost_sec).
    lidar_acquire : true 면 옛 동작 — 비전이 없으면 상태와 무관하게 LiDAR 전방 최근접.

    규칙(계약 전체코스 §2-5): 획득(SEARCH→HOLD)은 비전 색 확인으로만. LiDAR 전방 최근접은
      HOLD 유지 중 직전 비전 방위가 lost_sec 안일 때 거리·방위 보조로만 쓴다.
      ★ SEARCH 에서 LiDAR 최근접을 부표로 보면 색과 무관한 물체(다른 색 부표·벽)에 HOLD 한다.
      보조 창(since_seen <= lost_sec)은 HOLD 의 lost 판정(> lost_sec)과 정확히 맞물린다.
    """
    if vision_fresh:
        return 'vision'
    if lidar_acquire:
        return 'lidar'
    if state == 'HOLD' and since_seen is not None and since_seen <= lost_sec:
        return 'lidar'
    return None


def counts_as_seen(source, lidar_acquire):
    """이 출처의 방위를 '부표를 봤다'(SEARCH→HOLD 획득·last_seen 갱신)로 셀지. 순수 함수.

    lidar_acquire=false 면 비전만 센다 — LiDAR 보조가 last_seen 을 갱신하면 비전이 끊겨도
    HOLD 가 끝나지 않는다(전방에 무엇이든 있으면 영원히 유지).
    """
    return source == 'vision' or (bool(lidar_acquire) and source == 'lidar')


def search_timed_out(elapsed, timeout_sec, timeout_ends):
    """SEARCH 시간 초과로 DONE 래치할지. 순수 함수.

    timeout_ends=false(기본, 계약 전체코스 §2-4): 래치하지 않는다 — SEARCH·DONE 둘 다 20000 이라
      래치의 효과는 '더 안 찾기'뿐이다. 계속 찾고, 다음 WP 진행은 north 의 WP 시간 초과에 맡긴다.
    timeout_ends=true : 옛 동작(elapsed > timeout_sec 이면 DONE).
    """
    return bool(timeout_ends) and elapsed > timeout_sec


def search_timeout_warn(elapsed, timeout_sec, timeout_ends, warned):
    """래치하지 않는 SEARCH(timeout_ends=false)가 timeout_sec 를 넘겼다고 WARN 할지. 순수 함수.

    SEARCH 진입마다 1회(warned = 이번 SEARCH 에서 이미 냈다). 상태는 바꾸지 않는다 —
    로그 없이 무한정 SEARCH 가 이어지는 것만 막는다. timeout_ends=true 면 DONE 쪽 WARN 이 따로 난다.
    """
    return (not timeout_ends) and (not warned) and elapsed > timeout_sec


def hold_total_exceeded(total_before, hold_enter_t, now, max_sec):
    """한 WP(담당 모드 진입)에서 HOLD 에 머문 시간의 합이 max_sec 에 닿았는지. 순수 함수.

    total_before : 이번 모드 진입 뒤 이미 끝난 HOLD 구간들의 합(s). HOLD→SEARCH→HOLD 도 이어 합산.
    hold_enter_t : 지금 HOLD 구간의 시작 시각. None = 지금 HOLD 아님.
    max_sec      : 파라미터 hold_max_sec.

    계약 전체코스 §10-추가 결정 5: 성공(5 m 안 5 s) 못 하고 HOLD 를 오가며 WP 시간 초과까지
      붙잡는 것을 막는다 → 포기 DONE(20000). 경계 포함(합 ≥ max_sec).
    """
    cur = 0.0 if hold_enter_t is None else max(0.0, now - hold_enter_t)
    return (total_before + cur) >= max_sec


def unknown_giveup(unknown_t0, now, giveup_sec):
    """HOLD 에서 거리 'unknown' 이 giveup_sec 이어졌으면 포기(DONE)할지. 순수 함수.

    unknown_t0 : 'unknown' 이 연속으로 시작된 시각. None = 지금 unknown 아님(거리 잡힘 → 리셋).
    now        : 현재 시각(monotonic).
    giveup_sec : 파라미터 unknown_giveup_sec.

    계약 전체코스 §10-추가 결정 4: LiDAR 가 부표를 못 보면 비전이 이어지는 한 HOLD 'unknown'
      → STOP_HOLD 로 WP 시간 초과까지 정지했다(시뮬 코스 +210 s). 포기하면 DONE(20000) 으로
      손 뗀다. 경계 포함(now − t0 ≥ giveup_sec).
    """
    return unknown_t0 is not None and (now - unknown_t0) >= giveup_sec


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

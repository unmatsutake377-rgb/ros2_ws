"""ship_back 의 위치유지 로직 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

    from ship_back.hold_logic import station_keep_action, HoldTimer, should_reset_on_mode
    from ship_back.hold_logic import target_source, counts_as_seen, search_timed_out
    from ship_back.hold_logic import unknown_giveup, search_timeout_warn, hold_total_exceeded
    from ship_back.hold_logic import reacquire_stop_active, reacquire_window_elapsed
    from ship_back.hold_logic import check_hold_params, DEFAULT_HOLD_DIST_M, DEFAULT_DIST_TOL_M
    from ship_back.hold_logic import check_hold_source, gps_hold_applies, check_gps_hold_params
    from ship_back.hold_logic import goal_dist_usable, gps_hold_action

규정: '부표 5m 이내 5초 유지'. 작년 코드는 그냥 5초간 PWM 중립이라 조류에 밀려 실패했다.
→ LiDAR 로 잰 거리로 전진/후진을 결정해 능동적으로 위치를 유지한다(bang-bang + 데드밴드).
"""

import math
from collections import namedtuple

# 시작 가드(check_hold_params)의 복귀값 = ship_back.py declare_parameter 리터럴 = config/ship_back.yaml
#   (test_hold_param_guard 가 셋을 묶어 둔다). ⚠️ 실측 필요(hold_dist_m).
DEFAULT_HOLD_DIST_M = 2.5
DEFAULT_DIST_TOL_M = 0.4


def check_hold_params(hold_dist, dist_tol, keep_radius,
                      default_hold=DEFAULT_HOLD_DIST_M, default_tol=DEFAULT_DIST_TOL_M):
    """hold_dist_m·dist_tol_m 시작 가드. 순수 함수. → (쓸 hold, 쓸 tol, [(심각도, 메시지), …]).

    심각도는 'ERROR' / 'WARN' / 'INFO'. 노드는 시작 때 한 번만 부르고 줄마다 그 심각도로 찍는다.
    (계약서_옆비킴_현장조정_20261106 결정 ⑤ — 동작은 그대로, 값 확인만)

    ERROR: 두 값 중 하나라도 숫자 아님·비유한·음수, 또는 hold − tol ≤ 0(후진 경계가 부표 위·뒤)
           → **두 값 모두** 기본값(default_hold / default_tol)으로 복귀. 아래 검사는 복귀한 값으로.
    WARN : hold + tol ≥ keep_radius — 유지 띠 바깥 끝이 규정 반경에 닿거나 넘음 → HoldTimer 가
           리셋돼 위치유지가 성공하지 못함(hold_max_sec 뒤 포기). 값은 그대로(현장 판단).
    INFO : hold + tol > 기본 바깥 끝(default_hold + default_tol = 2.9 m) — north 는 이 2.9 를
           los_logic.SHIP_BACK_HOLD_OUTER_M 으로 따로 갖고 arrive_radius_hold_modes_m(3.5) 를 거기에
           맞춰 두었다. ship_back 은 north 값을 모르므로 숫자 비교 없이 같이 확인하라는 안내만.
    keep_radius 는 규정값이라 여기서 검사하지 않는다(비유한이면 WARN 비교가 거짓 → 안 남).
    """
    issues = []
    try:
        h = float(hold_dist)
        t = float(dist_tol)
        bad = (not math.isfinite(h) or not math.isfinite(t) or h < 0.0 or t < 0.0
               or h - t <= 0.0)
    except (TypeError, ValueError):
        bad = True
    if bad:
        issues.append(('ERROR',
                       f"🚨 hold_dist_m={hold_dist!r}, dist_tol_m={dist_tol!r} 는 잘못된 값"
                       f"(숫자 아님·NaN·inf·음수, 또는 hold − tol ≤ 0) → 두 값 모두 기본값 "
                       f"{default_hold:g}/{default_tol:g} m 로 돈다. ship_back.yaml 확인."))
        h, t = float(default_hold), float(default_tol)

    outer = h + t
    if outer >= keep_radius:
        issues.append(('WARN',
                       f"⚠️ 유지 띠 바깥 {outer:g} m ≥ 규정 반경 {keep_radius:g} m — HoldTimer 가 "
                       f"리셋돼 위치유지가 성공하지 못함. 값은 그대로 쓴다(ship_back.yaml 확인)."))
    base_outer = float(default_hold) + float(default_tol)
    if outer > base_outer + 1e-9:
        issues.append(('INFO',
                       f"유지 띠 바깥 {outer:g} m > 기본 {base_outer:g} m — north 도착 반경"
                       f"(north_goal_angle.yaml arrive_radius_hold_modes_m, 기본 3.5 m)이 이보다 "
                       f"큰지 같이 확인(north 는 los_logic.SHIP_BACK_HOLD_OUTER_M {base_outer:g} 를 봄)."))
    return h, t, issues


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


def hold_total_exceeded(total_before, hold_enter_t, now, max_sec, window_sec=0.0):
    """한 WP(담당 모드 진입)에서 HOLD 에 머문 시간의 합이 max_sec 에 닿았는지. 순수 함수.

    total_before : 이번 모드 진입 뒤 이미 끝난 HOLD 구간·재획득 정지 창들의 합(s).
                   HOLD→SEARCH→HOLD 도 이어 합산.
    hold_enter_t : 지금 HOLD 구간의 시작 시각. None = 지금 HOLD 아님.
    max_sec      : 파라미터 hold_max_sec.
    window_sec   : 지금 열린 재획득 정지 창의 경과(s, reacquire_window_elapsed). 창 없으면 0.

    계약 전체코스 §10-추가 결정 5: 성공(5 m 안 5 s) 못 하고 HOLD 를 오가며 WP 시간 초과까지
      붙잡는 것을 막는다 → 포기 DONE(20000). 경계 포함(합 ≥ max_sec).
    결정 6 보충 ①: 재획득 정지 창(STOP_HOLD) 시간도 합에 넣는다 — 놓침→정지→재획득 반복이
      WP 시간 초과까지 이어지지 않게. 창이 끝나 20000(SEARCH)으로 간 뒤 시간은 넣지 않는다.
    """
    cur = 0.0 if hold_enter_t is None else max(0.0, now - hold_enter_t)
    return (total_before + cur + max(0.0, window_sec)) >= max_sec


def reacquire_window_elapsed(t_lost, now, stop_sec):
    """재획득 정지 창이 HOLD 총시간 합에 보탤 시간(s). 순수 함수.

    t_lost   : 창 시작 시각(reacquire_stop_active 와 같음). None = 창 없음 → 0.
    stop_sec : 파라미터 reacquire_stop_sec. 0 이하면 창 없음 → 0.

    → clamp(now − t_lost, 0, stop_sec). 창 끝(stop_sec) 뒤 20000 구간은 넣지 않는다 —
      창이 끝나는 틱이 stop_sec 를 조금 넘겨도 stop_sec 만 더한다(결정 6 보충 ①).
    """
    if t_lost is None or stop_sec <= 0.0:
        return 0.0
    return min(max(0.0, now - t_lost), stop_sec)


def reacquire_stop_active(t_lost, now, stop_sec):
    """HOLD 에서 부표를 놓친 뒤 '재획득 정지 창' 안인지(SEARCH 가 STOP_HOLD 를 낼지). 순수 함수.

    t_lost   : HOLD → SEARCH(lost) 전이 시각 = 창 시작. None = 창 없음
               (첫 SEARCH·모드 진입 리셋 뒤·창 끝남).
    now      : 현재 시각(monotonic).
    stop_sec : 파라미터 reacquire_stop_sec. 0 이하면 기능 끔(옛 동작: 바로 20000).

    계약 전체코스 §10-추가 결정 6: 결정 3(north 도착 반경 3.5 m·dwell 0.5 s) 뒤에는 HOLD 중
      놓쳐 SEARCH 20000 을 내면 배가 이미 3.5 m 안이라 north 가 ~1 s 만에 다음 WP 로 간다.
      → 창 안에서는 그 자리 정지(STOP_HOLD)로 다시 찾는다. 경계 미포함(now − t_lost < stop_sec):
      4.9 s 는 정지, 5.0 s 부터 20000. 창 시간은 HOLD 총시간 합에 든다(reacquire_window_elapsed).
    """
    return t_lost is not None and stop_sec > 0.0 and (now - t_lost) < stop_sec


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


# ==================== GPS 위치유지 hold_source=gps (계약서_GPS위치유지_20261113) ====================
# 사용자 결정 11-13: 위치유지(mode 2) = 'WP 반경에 들어가면 GPS 기준 정지 유지'. 기본 gps, 부표 비전 경로는
#   hold_source=vision 옵션(지금 동작 그대로). ⚠️ CLAUDE.md §6-2 'LiDAR 유지, RTK 폴백' 과 반대 — 계약서가 근거.
# 아래 DEFAULT_* = ship_back.py declare_parameter 리터럴 = config/ship_back.yaml (test_gps_hold 가 묶음).
HOLD_SOURCES = ('vision', 'gps')
DEFAULT_HOLD_SOURCE = 'gps'
GPS_HOLD_MODES = (2,)               # search_behavior:=hold 의 mode 3 은 vision 경로 그대로(결정 4)
DEFAULT_GPS_ENTER_M = 3.5           # 이 거리 미만이면 정지 시작 / 재접근 끝
DEFAULT_GPS_RESUME_M = 4.0          # 정지 중 이 거리 초과면 재접근(20000)
DEFAULT_GPS_KEEP_RADIUS_M = 4.5     # 판정 반경(결정 3) — 규정 5 m 보다 GPS 필터 지연만큼 안쪽
DEFAULT_GPS_HOLD_TIME_SEC = 6.0     # 판정 시간(결정 3) — 규정 5 s + 여유
DEFAULT_GOAL_DIST_STALE_SEC = 2.0   # ship_dock·motor_control 과 같은 값
# 🚨 사본: north_goal_angle.yaml arrive_radius_hold_modes_m(= los_logic.DEFAULT_ARRIVE_RADIUS_HOLD_MODES_M).
#   gps_enter_m 이 이보다 작으면 배가 3.5 m 안에서 20000 인 동안 north 가 dwell 0.5 s 로 WP 를 넘겨
#   정지 전에 끝난다(wp_advance). 공유 모듈 금지(§1-8) → test_gps_hold 가 north yaml 과 대조.
NORTH_ARRIVE_HOLD_M = 3.5
# 모드 진입 뒤 이 시간 이상 지나서 받은 /goal_distance 만 쓴다 — 직전 WP(도착해 ≈0 m) 거리가 늦게 와서
#   새 WP 40 m 앞에서 정지·타이머를 시작하는 오판 방지. ship_dock GOAL_DIST_ENTRY_GUARD_SEC 와 같은 장치·값
#   (north 주기 0.5 s × 2). 계약서 §5 파라미터 목록에 없어 상수.
GOAL_DIST_ENTRY_GUARD_SEC = 1.0


def check_hold_source(value):
    """파라미터 hold_source → (쓸 값, 오류 메시지 or None). 순수 함수.

    vision|gps(앞뒤 공백·대소문자 무시) 밖이면 기본(gps, 결정 1 — 기본과 같은 쪽) + 메시지.
    """
    v = value.strip().lower() if isinstance(value, str) else None
    if v in HOLD_SOURCES:
        return v, None
    return DEFAULT_HOLD_SOURCE, (f"🚨 hold_source={value!r} 는 vision|gps 가 아님 → {DEFAULT_HOLD_SOURCE} 로 돈다"
                                 f"(mode 2 = /goal_distance 정지 유지). ship_back.yaml 확인.")


def gps_hold_applies(hold_source, wp_mode):
    """이 모드를 GPS 위치유지로 돌지. gps 이고 mode 2 일 때만(mode 3 hold 옵션은 vision)."""
    return hold_source == 'gps' and wp_mode in GPS_HOLD_MODES


def _finite_float(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def check_gps_hold_params(enter_m, resume_m, keep_m, hold_time, stale_sec,
                          north_arrive_m=NORTH_ARRIVE_HOLD_M):
    """gps_* 시작 가드. 순수 함수. → (enter, resume, keep, hold_time, stale, [('ERROR', 메시지), …]).

    거리 셋(한 묶음): 숫자 아님·비유한, enter < north_arrive_m(3.5), resume < enter, keep ≤ resume 중 하나라도
      → ERROR 1줄 + **셋 모두** 기본값(3.5/4.0/4.5). (계약서 §2-9)
    hold_time·stale_sec: 숫자 아님·비유한·0 이하 → 그 값만 기본값(6.0 / 2.0) + ERROR 1줄씩.
    노드는 시작 때 한 번만 부르고 줄마다 ERROR 로 찍는다.
    """
    issues = []
    e, r, k = _finite_float(enter_m), _finite_float(resume_m), _finite_float(keep_m)
    if e is None or r is None or k is None or e < north_arrive_m or r < e or k <= r:
        issues.append(('ERROR',
                       f"🚨 gps_enter_m={enter_m!r}, gps_resume_m={resume_m!r}, gps_keep_radius_m={keep_m!r} "
                       f"는 잘못된 값(숫자 아님·NaN·inf, enter < north 도착 반경 {north_arrive_m:g}, "
                       f"resume < enter, keep ≤ resume 중 하나) → 셋 모두 기본값 "
                       f"{DEFAULT_GPS_ENTER_M:g}/{DEFAULT_GPS_RESUME_M:g}/{DEFAULT_GPS_KEEP_RADIUS_M:g} m. "
                       f"ship_back.yaml 확인."))
        e, r, k = DEFAULT_GPS_ENTER_M, DEFAULT_GPS_RESUME_M, DEFAULT_GPS_KEEP_RADIUS_M
    ht = _finite_float(hold_time)
    if ht is None or ht <= 0.0:
        issues.append(('ERROR', f"🚨 gps_hold_time_sec={hold_time!r} 는 잘못된 값(양수 아님) → "
                                f"기본 {DEFAULT_GPS_HOLD_TIME_SEC:g} s."))
        ht = DEFAULT_GPS_HOLD_TIME_SEC
    st = _finite_float(stale_sec)
    if st is None or st <= 0.0:
        issues.append(('ERROR', f"🚨 goal_dist_stale_sec={stale_sec!r} 는 잘못된 값(양수 아님) → "
                                f"기본 {DEFAULT_GOAL_DIST_STALE_SEC:g} s."))
        st = DEFAULT_GOAL_DIST_STALE_SEC
    return e, r, k, ht, st, issues


def goal_dist_usable(dist, t_rx, mode_enter_t, now, stale_sec, guard_sec=GOAL_DIST_ENTRY_GUARD_SEC):
    """마지막 /goal_distance 를 쓸 수 있으면 그 값(m), 아니면 None(모른다). 순수 함수.

    dist·t_rx    : 마지막 값과 받은 시각(monotonic). 없으면 None.
    mode_enter_t : 담당 모드 진입(wp_cb 리셋) 시각.
    모르면 None: 값·시각 없음, 비유한·음수, 진입 뒤 guard_sec 전 수신, now − t_rx > stale_sec.
    경계(ship_dock transit_engage 와 같음): 수신 − 진입 == guard 는 사용, 지금 − 수신 == stale 은 신선.
    """
    if dist is None or t_rx is None or mode_enter_t is None:
        return None
    d = _finite_float(dist)
    if d is None or d < 0.0:
        return None
    if (t_rx - mode_enter_t) < guard_sec:
        return None
    if (now - t_rx) > stale_sec:
        return None
    return d


def gps_hold_action(dist, approaching, enter_m, resume_m):
    """HOLD(GPS) 한 틱 동작. 순수 함수. → 'stop'(50000) / 'approach'(20000) / 'unknown'(50000).

    dist        : goal_dist_usable 결과(None = 모름).
    approaching : 직전 틱이 재접근이었나(히스테리시스 기억). unknown 뒤에는 False(정지했으므로).
    정지 중이면 dist > resume_m 일 때만 재접근, 재접근 중이면 dist < enter_m 일 때만 정지.
    ★ 180(후진)·조향각은 없다 — GPS 1 Hz·필터 지연이라 후진 방향을 모르고 후진 추력 미실측(계약서 §2-7).
      재접근 방향은 north LOS(20000 → ship_direction (C)).
    """
    if dist is None:
        return 'unknown'
    if approaching:
        return 'stop' if dist < enter_m else 'approach'
    return 'approach' if dist > resume_m else 'stop'


# ==================== /fused_objects 거리 출처 (계약서_인식융합_20261112 §4·§10) ====================
# 🚨 형식 사본 — perception_fusion(발행)·ship_back·ship_turn·ship_gate 가 같은 값을 따로 갖는다
#    (공유 모듈 금지 CLAUDE.md §1-8). 어긋나면 엉뚱한 칸을 거리로 읽는다(계약 위험 1) → 머리(VER·STRIDE)
#    검사 + 각 패키지 test_fused_match 의 사본 일치 검사. 바꿀 때 네 곳 동시에.
# Float32MultiArray data = [VER, STRIDE, N, 그 뒤 N × STRIDE 칸:
#   color_id, bearing_deg(상대방위 0 = 정면, 좌 +, LiDAR 원점), range_m, conf(0~1), src_id, age_s,
#   track_id(-1 = 없음), flags]
FUSED_VER = 1
FUSED_STRIDE = 8
FUSED_HEAD = 3            # 머리 칸 수 [VER, STRIDE, N]
FUSED_I_COLOR = 0         # 물체 한 칸 안의 열 위치
FUSED_I_BEARING = 1
FUSED_I_RANGE = 2
FUSED_I_CONF = 3
FUSED_I_SRC = 4
FUSED_I_AGE = 5
FUSED_I_TRACK = 6
FUSED_I_FLAGS = 7
# color_id (🚨 /avoid_detections cls 열거형과 다르다)
FUSED_COLOR_IDS = {'unknown': 0, 'red': 1, 'green': 2, 'white': 3, 'yellow': 4,
                   'black': 5, 'orange': 6, 'blue': 7}
FUSED_SRC_IDS = {'lidar': 0, 'depth': 1, 'fused': 2, 'row': 3}
# distance_source: lidar = /scan 만(구독도 안 만듦) / fill = LiDAR 먼저, None 일 때만 fused
#   / fused = fused 먼저, 맞는 물체 없으면 LiDAR. (§10 Q2)
DISTANCE_SOURCES = ('lidar', 'fill', 'fused')

FusedObj = namedtuple('FusedObj', 'color_id bearing_deg range_m conf src_id')


def _fused_wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def check_distance_source(value):
    """파라미터 distance_source → (쓸 모드, 오류 메시지 or None). 순수 함수.

    lidar|fill|fused(앞뒤 공백·대소문자 무시) 밖이면 'lidar'(지금 동작, /fused_objects 구독 없음) + 메시지.
    노드는 시작 때 한 번 부르고 메시지가 있으면 ERROR 한 줄.
    """
    v = value.strip().lower() if isinstance(value, str) else None
    if v in DISTANCE_SOURCES:
        return v, None
    return 'lidar', (f"🚨 distance_source={value!r} 는 lidar|fill|fused 가 아님 → lidar 로 돈다"
                     f"(/fused_objects 구독 안 함). yaml 확인.")


def fused_allowed_src_ids(names):
    """파라미터 fused_allowed_sources(이름 배열) → (src_id 집합, [모르는 이름]). 순수 함수.

    배열이 아니면(문자열 하나 등) 빈 집합 + 오류 — 아무 출처도 허용하지 않는다(모르면 입 다문다).
    """
    if isinstance(names, str) or not isinstance(names, (list, tuple)):
        return set(), [repr(names)]
    ids, bad = set(), []
    for nm in names:
        key = nm.strip().lower() if isinstance(nm, str) else None
        if key in FUSED_SRC_IDS:
            ids.add(FUSED_SRC_IDS[key])
        else:
            bad.append(str(nm))
    return ids, bad


def fused_color_id(color):
    """색 문자열('red' …) → color_id. 모르는 색·빈 값·None → 0(모름 = 방위만 매칭)."""
    c = color.strip().lower() if isinstance(color, str) else ''
    return FUSED_COLOR_IDS.get(c, 0)


def parse_fused_objects(data):
    """/fused_objects data → ([FusedObj, …], None) 또는 (None, 오류 사유). 순수 함수.

    머리 VER·STRIDE 불일치, N 이 0 이상 정수가 아님, 길이 ≠ 3 + N × STRIDE, 숫자 아님 → (None, 사유):
      노드는 메시지를 버리고 ERROR(5 s 주기). N = 0 은 '살아 있고 물체 없음' → ([], None).
    거리가 없는(0·음수·비유한) 물체, 방위·conf 가 비유한인 물체는 거리로 쓰지 않으므로 뺀다.
    """
    try:
        vals = [float(x) for x in data]
    except (TypeError, ValueError):
        return None, '숫자 배열 아님'
    if len(vals) < FUSED_HEAD:
        return None, f'길이 {len(vals)} < 머리 {FUSED_HEAD}'
    ver, stride, n = vals[0], vals[1], vals[2]
    if ver != FUSED_VER:
        return None, f'VER {ver:g} ≠ {FUSED_VER}'
    if stride != FUSED_STRIDE:
        return None, f'STRIDE {stride:g} ≠ {FUSED_STRIDE}'
    if not math.isfinite(n) or n < 0 or n != int(n):
        return None, f'N {n!r} 이 0 이상 정수 아님'
    n = int(n)
    if len(vals) != FUSED_HEAD + n * FUSED_STRIDE:
        return None, f'길이 {len(vals)} ≠ {FUSED_HEAD} + {n}×{FUSED_STRIDE}'
    objs = []
    for k in range(n):
        row = vals[FUSED_HEAD + k * FUSED_STRIDE: FUSED_HEAD + (k + 1) * FUSED_STRIDE]
        col, brg, rng = row[FUSED_I_COLOR], row[FUSED_I_BEARING], row[FUSED_I_RANGE]
        conf, src = row[FUSED_I_CONF], row[FUSED_I_SRC]
        if not all(math.isfinite(x) for x in (col, brg, rng, conf, src)) or rng <= 0.0:
            continue
        objs.append(FusedObj(int(round(col)), _fused_wrap180(brg), rng, conf, int(round(src))))
    return objs, None


def fused_fresh(t_rx, now, stale_sec):
    """마지막 정상 /fused_objects 수신(monotonic) 뒤 stale_sec 안인가. 미수신(None)·비유한 → False."""
    return t_rx is not None and (now - t_rx) <= stale_sec


def match_fused(objs, rel_deg, color_id, match_deg, min_conf, allowed_src_ids):
    """노드의 카메라 방위 rel_deg 에 맞는 물체 → (range_m, src_id) 또는 None. 순수 함수.

    1) |방위 차| ≤ match_deg, color_id 가 0 이 아니면 같은 색만(0 = 색 모름 → 방위만).
    2) 그중 방위 최근접(동점이면 거리 짧은 것) **하나**를 고른다.
    3) 그 물체가 conf ≥ min_conf 이고 src_id ∈ allowed_src_ids 일 때만 거리. 아니면 None —
       더 먼 방위의 다른 물체로 넘어가지 않는다(같은 창 안 다른 물체 = 다른 부표일 수 있다).
    NaN 파라미터는 비교가 거짓이 되어 None(닫힌 쪽으로 실패).
    """
    if not objs or rel_deg is None or not math.isfinite(rel_deg):
        return None
    want = int(color_id) if color_id else 0
    best, best_key = None, None
    for o in objs:
        if want and o.color_id != want:
            continue
        d = abs(_fused_wrap180(o.bearing_deg - rel_deg))
        if not d <= match_deg:
            continue
        key = (d, o.range_m)
        if best is None or key < best_key:
            best, best_key = o, key
    if best is None or not best.conf >= min_conf or best.src_id not in allowed_src_ids:
        return None
    return best.range_m, best.src_id


def fused_src_label(src_id):
    """출처 로그 이름: fused 물체 → 'fused:<src 이름>'."""
    for k, v in FUSED_SRC_IDS.items():
        if v == src_id:
            return f'fused:{k}'
    return f'fused:src{src_id}'


def choose_distance(mode, lidar_d, fused_hit):
    """distance_source 분기 → (거리 or None, 출처 이름 or None). 순수 함수.

    lidar_d   : 노드 자신의 /scan 방위 매칭 거리(None = 못 봄).
    fused_hit : match_fused 결과 (range_m, src_id). /fused_objects 가 stale·미수신이면 None 으로 넘긴다.
    출처 이름: 'lidar' = 이 노드의 /scan, 'fused:<src>' = /fused_objects 물체.
    lidar: LiDAR 그대로 / fill: LiDAR 먼저, None 일 때만 fused / fused: fused 먼저, None 이면 LiDAR.
    """
    lid = (lidar_d, 'lidar') if lidar_d is not None else (None, None)
    fus = (fused_hit[0], fused_src_label(fused_hit[1])) if fused_hit is not None else (None, None)
    if mode == 'fill':
        return lid if lid[0] is not None else fus
    if mode == 'fused':
        return fus if fus[0] is not None else lid
    return lid

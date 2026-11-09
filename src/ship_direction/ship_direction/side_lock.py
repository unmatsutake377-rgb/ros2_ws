"""(C) 옆 잠금 — 나란히 가는 배 쪽으로 침로를 바꾸지 않는다 (ROS 비의존, 순수 함수).

근거: docs/문제와작업/계약서_토너먼트분리_20261109.md T2-1·§9-6·§9-7.
규정(토너먼트 1대1): 나란히 항주하다 침로를 바꾼 쪽이 접촉 책임. 그래서 배 옆 구역에 무언가 가까이
있으면 그쪽으로 크게 꺾는 후보(갭·DWA·비킴)를 뺀다. 상대 배인지 부표인지 구분하지 않는다(옆 부표에도
걸리지만 그쪽으로 안 꺾을 뿐이라 무해).

노드 스위치 side_lock_enabled(코드 기본 false). 끄면 노드는 이 모듈을 부르지 않는다(기존과 같다).
잠금은 새 스캔 연속 confirm_frames 장 확인 뒤 걸고 dist + release_margin 이상에서 푼다(SideLockFilter).
대기(SideLockWait)는 **잠금이 후보를 지운 틱**에만 — 잠금과 무관한 막다른 곳은 기존 동작(후진).

🚨 각도 규약(CLAUDE.md §3, geofence·/candidate_angle 과 같음): LiDAR 프레임 80° = 정면,
   상대방위 rel = a − 80 은 **좌 양수**. 조향각(/desired_angle 0~160)도 80 보다 크면 좌, 작으면 우
   (motor_control off = a − 80 > 0 = 좌조향, gap_select.side_escape '중심 > 정면 = 좌현').
   → 좌현 옆 구역 = rel +min~+max, 우현 옆 구역 = rel −max~−min.
     좌가 잠기면 80 + turn 초과 각을, 우가 잠기면 80 − turn 미만 각을 뺀다.
⚠️ RPLIDAR 장착·가림 미확인: 마스트·선체·스러스터가 옆·뒤 셀을 가리거나 자기 선체를 1.5 m 안으로
   잡으면 그쪽이 늘 잠긴다(잠기면 그쪽으로 안 꺾을 뿐 — 정면 회피는 남는다). 물보라·후방 반사도 ⚠️.
   물 위 확인(계약서 §10 현장 스위치 표) 전엔 켜지 않는다.
"""

import math

LOCKED, CLEAR, UNKNOWN = 'locked', 'clear', 'unknown'
_EPS = 1e-6

# 이상값 가드 범위(상수 — 바꾸려면 계약서부터). 밖이면 노드가 ERROR 1회 + 옆 잠금 끔.
DIST_MAX_M = 5.0       # side_lock_dist_m: 0 < d ≤ 5 (DWA 점 필터 상한과 같은 크기)
DEG_MAX = 180.0        # side_lock_min/max_deg: 0 ≤ min < max ≤ 180
TURN_MAX_DEG = 80.0    # side_lock_turn_deg: 0 ≤ t ≤ 80 (조향 0~160 안)
WAIT_MAX_SEC = 60.0    # side_lock_wait_max_sec: 0 ≤ w ≤ 60 (0 = 대기 없이 바로 기존 동작)
CONFIRM_MAX_FRAMES = 20     # side_lock_confirm_frames: 1 ≤ n ≤ 20 정수 (20 장 ≈ 2 s)
RELEASE_MARGIN_MAX_M = 2.0  # side_lock_release_margin_m: 0 ≤ m ≤ 2.0


def check_params(enabled, dist_m, min_deg, max_deg, turn_deg, wait_max_sec,
                 confirm_frames=3, release_margin_m=0.3):
    """→ 문제 목록(문자열). 비면 정상. 노드는 하나라도 있으면 ERROR 1회 + 끔."""
    bad = []
    if not isinstance(enabled, bool):
        bad.append(f"side_lock_enabled={enabled!r} bool 아님")

    def num(name, v, lo, hi, lo_open=False):
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            bad.append(f"{name}={v!r} 비유한/숫자 아님")
            return False
        if (v <= lo if lo_open else v < lo) or v > hi:
            bad.append(f"{name}={v} 범위({lo}{'<' if lo_open else '≤'}값≤{hi}) 밖")
            return False
        return True

    num('side_lock_dist_m', dist_m, 0.0, DIST_MAX_M, lo_open=True)
    ok_lo = num('side_lock_min_deg', min_deg, 0.0, DEG_MAX)
    ok_hi = num('side_lock_max_deg', max_deg, 0.0, DEG_MAX)
    if ok_lo and ok_hi and not min_deg < max_deg:
        bad.append(f"side_lock_min_deg={min_deg} ≥ side_lock_max_deg={max_deg}")
    num('side_lock_turn_deg', turn_deg, 0.0, TURN_MAX_DEG)
    num('side_lock_wait_max_sec', wait_max_sec, 0.0, WAIT_MAX_SEC)
    if isinstance(confirm_frames, bool) or not isinstance(confirm_frames, int) or \
            not 1 <= confirm_frames <= CONFIRM_MAX_FRAMES:
        bad.append(f"side_lock_confirm_frames={confirm_frames!r} 1~{CONFIRM_MAX_FRAMES} 정수 아님")
    if isinstance(release_margin_m, bool) or not isinstance(release_margin_m, (int, float)) or \
            not math.isfinite(release_margin_m) or \
            not 0.0 <= release_margin_m <= RELEASE_MARGIN_MAX_M:
        bad.append(f"side_lock_release_margin_m={release_margin_m!r} "
                   f"0~{RELEASE_MARGIN_MAX_M} 유한값 아님")
    return bad


def zone_min(ranges, angle_min_deg, inc_deg, forward_deg, lo_rel, hi_rel, range_min=0.0):
    """상대방위 lo_rel ≤ rel ≤ hi_rel(도, 좌 양수) 구역의 최소 유효 거리 → (덮였나, 최솟값).

    덮였나 False = 스캔이 그 구역을 다 덮지 못함(예: 0~160° 만 오는 스캔) → 호출자는 '모름'.
    한 바퀴(n × inc ≥ 360 − inc/2)면 늘 덮인다(−180/180 경계를 넘는 구역도 돌려서 본다).
    유효 셀 = 유한 · > 0 · ≥ range_min(LaserScan 규격 — 그보다 가까우면 무효값). 없으면 inf.
    """
    n = len(ranges)
    if n == 0 or not math.isfinite(inc_deg) or inc_deg <= 0.0 or not math.isfinite(angle_min_deg):
        return False, math.inf
    width = hi_rel - lo_rel
    full = n * inc_deg >= 360.0 - inc_deg / 2.0
    if not full:
        span = (n - 1) * inc_deg
        off = ((lo_rel + forward_deg) - angle_min_deg) % 360.0
        if off + width > span + _EPS:
            return False, math.inf
    rmin = range_min if (isinstance(range_min, (int, float)) and math.isfinite(range_min)) else 0.0
    best = math.inf
    for i in range(n):
        d = ((angle_min_deg + i * inc_deg - forward_deg) - lo_rel) % 360.0
        if d > width + _EPS:
            continue
        r = ranges[i]
        if r is None or not math.isfinite(r) or r <= 0.0 or r < rmin:
            continue
        if r < best:
            best = r
    return True, best


def side_states(ranges, angle_min_deg, inc_deg, forward_deg, dist_m, min_deg, max_deg,
                range_min=0.0):
    """→ ((좌 상태, 좌 최솟값), (우 상태, 우 최솟값)). 상태 = LOCKED / CLEAR / UNKNOWN.

    LOCKED = 그 옆 구역 최솟값 < dist_m. UNKNOWN(덮이지 않음)은 잠그지 않는다 — 기존 동작 쪽.
    """
    out = []
    for lo, hi in ((min_deg, max_deg), (-max_deg, -min_deg)):
        covered, m = zone_min(ranges, angle_min_deg, inc_deg, forward_deg, lo, hi, range_min)
        if not covered:
            out.append((UNKNOWN, m))
        elif m < dist_m:
            out.append((LOCKED, m))
        else:
            out.append((CLEAR, m))
    return out[0], out[1]


def allowed_range(left_locked, right_locked, forward_deg, turn_deg, lo=0.0, hi=160.0):
    """잠금을 지키는 조향각 구간 [a_lo, a_hi]. 좌 잠김 → a ≤ 정면 + turn, 우 잠김 → a ≥ 정면 − turn."""
    a_lo = forward_deg - turn_deg if right_locked else lo
    a_hi = forward_deg + turn_deg if left_locked else hi
    return a_lo, a_hi


def angle_allowed(angle, a_lo, a_hi):
    """조향각이 [a_lo, a_hi] 안이면 True. None·비유한은 False(잠금을 지킨다고 볼 수 없다)."""
    return angle is not None and math.isfinite(angle) and a_lo <= angle <= a_hi


def clip_candidates(candidates, angle_array, yaw_mapped, a_lo, a_hi):
    """갭 후보 (angle_diff, -arc, angle, s, e) 를 잠금 구간으로 자른 **새** 리스트. 원본은 안 바꾼다.

    zone [angle[s], angle[e]] ∩ [a_lo, a_hi] 가 비면 그 후보는 뺀다. 남으면 노드와 같은 방식으로
    목표각(yaw_mapped)을 그 안으로 clamp 해 angle·angle_diff 를 다시 쓴다(-arc·s·e 는 그대로 —
    zone 안의 각은 팽창 마스크상 다 열려 있다). 구간이 zone 을 다 덮으면 후보가 그대로 남는다.
    """
    out = []
    for _diff, neg_arc, _angle, s, e in candidates:
        z_lo = max(angle_array[s], a_lo)
        z_hi = min(angle_array[e], a_hi)
        if z_lo > z_hi:
            continue
        if yaw_mapped < z_lo:
            best = z_lo
        elif yaw_mapped > z_hi:
            best = z_hi
        else:
            best = yaw_mapped
        out.append((abs(best - yaw_mapped), neg_arc, best, s, e))
    return out


class SideLockFilter:
    """좌·우 잠금 확인(연속 프레임) + 풀림 히스테리시스. 물보라·깜빡임으로 잠금이 들락날락하지 않게.

    update(states, dist_m, new_frame) → (좌 잠김, 우 잠김). states = side_states() 결과(순간 판정).
      · 잠그기: 최솟값 < dist_m 이 **새 스캔 프레임** 연속 confirm_frames 번(같은 스캔 재처리는 안 셈).
      · 풀기: 잠긴 뒤 최솟값 ≥ dist_m + release_margin_m 인 새 프레임 한 번.
      · '모름'(구역 미덮임): 그쪽 잠김·카운터를 바로 초기화(잠그지 않음 — 기존 동작 쪽).
    reset() = 둘 다 안 잠김·카운터 0.
    """

    def __init__(self, confirm_frames, release_margin_m):
        self.confirm_frames = int(confirm_frames)
        self.release_margin_m = float(release_margin_m)
        self.reset()

    def reset(self):
        """둘 다 안 잠김, 카운터 0."""
        self.locked = [False, False]
        self.count = [0, 0]

    def update(self, states, dist_m, new_frame):
        """한 틱 반영 → (좌 잠김, 우 잠김). new_frame False 면 '모름' 초기화만 하고 그대로."""
        for k, (st, d) in enumerate(states):
            if st == UNKNOWN:
                self.locked[k] = False
                self.count[k] = 0
                continue
            if not new_frame:
                continue
            if self.locked[k]:
                if d >= dist_m + self.release_margin_m:
                    self.locked[k] = False
                    self.count[k] = 0
            elif d < dist_m:
                self.count[k] += 1
                if self.count[k] >= self.confirm_frames:
                    self.locked[k] = True
            else:
                self.count[k] = 0
        return self.locked[0], self.locked[1]


class SideLockWait:
    """옆 잠금이 후보를 지운 틱의 대기 (§9-7). 시각은 호출자가 준다(monotonic, 테스트 가능).

    update(blocked, now) → (동작, 사건). blocked = **잠금 때문에** 후보가 0 개가 됐다(잠금 없이는
    갭·DWA·비킴 중 하나라도 있었다). 잠금과 무관하게 막힌 곳은 blocked False → 기존 동작(후진).
      동작: 'go'       blocked 아님 → 잠금 지킨 결과(또는 기존 결과)를 쓴다
            'wait'     후진하지 않고 STOP_HOLD, 매 틱 다시 평가
            'fallback' 대기가 wait_max_sec 를 넘음(0 이면 바로) → 기존 동작(잠금 없이 고름)
      사건: 'wait_start'(대기 시작) / 'resume'(대기 끝 → 재출발) / 'timeout'(상한 → 폴백) /
            'rearm'(폴백 끝 → 다시 잠금 지킴) / None. 사건은 전이마다 한 번만 나온다.
    폴백은 blocked 가 풀릴 때까지 유지된다.
    reset() = 조용히 처음 상태((C) 틱이 끊겼을 때 — 대기 시간을 이어 세지 않는다).
    """

    def __init__(self, wait_max_sec):
        self.wait_max_sec = float(wait_max_sec)
        self.reset()

    def reset(self):
        """대기·폴백 없음."""
        self.wait_t0 = None
        self.fallback = False

    @property
    def waiting(self):
        """지금 대기 중인가."""
        return self.wait_t0 is not None

    def update(self, blocked, now):
        """한 틱 반영 → (동작, 사건)."""
        if not blocked:
            ev = 'resume' if self.wait_t0 is not None else ('rearm' if self.fallback else None)
            self.reset()
            return 'go', ev
        if self.fallback:
            return 'fallback', None
        if self.wait_t0 is None:
            if self.wait_max_sec <= 0.0:
                self.fallback = True
                return 'fallback', 'timeout'
            self.wait_t0 = now
            return 'wait', 'wait_start'
        if now - self.wait_t0 > self.wait_max_sec:
            self.wait_t0 = None
            self.fallback = True
            return 'fallback', 'timeout'
        return 'wait', None

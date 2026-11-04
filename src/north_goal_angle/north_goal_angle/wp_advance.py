"""WP 전진 판정 — ROS 비의존 순수 로직 (test_wp_advance.py).

계약서 docs/문제와작업/계약서_전체코스_20261104.md C2(U1a) §2-2.

dwell 대기 (새 결함): mode 2·3 에서 north 도착 반경(당시 속도 반경 2.5 m)과 ship_back 유지
거리(2.5±0.4 m)가 겹쳤다. 배가 부표 옆에서 위치유지를 하는 동안 north 의 dwell 이 다 차서 WP 를
넘기고, 그 순간 /wp_mode 가 바뀌어 ship_back 의 위치유지가 끊겼다.
지금 mode 2·3 반경은 3.5 m 고정(arrive_radius_hold_modes_m, 추가 결정 3 — 유지 거리 밖까지)이라
유지 위치가 항상 반경 안이다 → 이 대기가 없으면 HOLD 중 dwell 0.5 s 만에 넘긴다. 대기가 필수다.
→ dwell 은 '담당 노드가 손 뗀 동안'만 센다. 손 뗐다 = north 가 받은 마지막 /candidate_angle 이
  · 신선(≤ dwell_wait_stale_sec)한 20000(담당 노드 INIT·SEARCH·DONE = "GPS 로 가라"), 또는
  · dwell_wait_stale_sec 넘게 무수신(담당 노드 사망 — 기다려도 소용없다. 받은 적 없음도 같다).
  신선한 비-20000(HOLD 조향·STOP_HOLD 등) = 담당 노드가 일하는 중 → dwell 을 세지 않는다.

🚨 시간 초과(wp_timeout)는 dwell 대기와 무관하게 그대로 진행한다 — wp_step 이 먼저 본다.
   담당 노드가 영원히 손을 안 떼도 미션이 멈추지 않는 유일한 수단이다.

t_start 정책: 대기 중(세지 않음)에는 '반경 밖'과 똑같이 t_start 를 비운다. dwell 은 기존처럼
   '조건을 모두 만족한 연속 시간'이다(반경 안 + fix 신선 + 담당 노드 손 뗌). 손 떼기 전
   반경 안에 있던 시간은 쌓지 않는다 — 쌓으면 HOLD 전 SEARCH(20000) 동안 반경 안에 있던 시간이
   남아, HOLD 중 부표를 잠깐 놓쳐 SEARCH 로 돌아간 한 틱(20000)에 바로 WP 를 넘길 수 있다.

ROS2 파라미터 (north_goal_angle.yaml 과 코드 기본값 동일 — test_wp_advance.py 가 잡는다):
    dwell_wait_enabled: true
    dwell_wait_modes: [2, 3]        🚨 빈 리스트 금지(rclpy 가 타입을 못 정한다) — 끄려면 enabled false
    dwell_wait_stale_sec: 1.0       north 발행 주기 0.5 s × 2. 반드시 소수(.0)
"""

import math

from north_goal_angle.waypoint_loader import VALID_MODES

DEFAULT_DWELL_WAIT_ENABLED = True
DEFAULT_DWELL_WAIT_MODES = [2, 3]
DEFAULT_DWELL_WAIT_STALE_SEC = 1.0

# 담당 노드가 '손 뗐다'(GPS 주행)는 /candidate_angle 값. ship_back·ship_direction 의 CANDIDATE_INVALID.
CANDIDATE_RELEASED = 20000.0


def sanitize_wait_modes(value, default=DEFAULT_DWELL_WAIT_MODES, valid_modes=VALID_MODES):
    """dwell_wait_modes 검증 → (쓸 모드 tuple, 정상 여부).

    정수만, VALID_MODES 안, 비어 있지 않음. 틀리면 (default, False) — 노드는 ERROR 후 기본값으로
    돈다(gps_stale_sec 와 같은 방식, 대기 기능이 출발을 막지는 않는다).
    """
    try:
        modes = list(value)
    except TypeError:
        return tuple(default), False
    if not modes:
        return tuple(default), False
    for m in modes:
        if isinstance(m, bool) or not isinstance(m, int) or m not in valid_modes:
            return tuple(default), False
    return tuple(modes), True


def dwell_may_count(enabled, wp_mode, wait_modes, last_cand, last_cand_t, now, stale_sec,
                    released=CANDIDATE_RELEASED):
    """이번 틱에 dwell 을 세도 되나(담당 노드가 손 뗐나). True = 센다.

    enabled False·대상 모드 아님 → True(기존 동작). 받은 적 없음·stale_sec 초과 → True.
    신선한 값이 released(20000) → True. 신선한 그 밖의 값(NaN 포함) → False(대기).
    """
    if not enabled or wp_mode not in wait_modes:
        return True
    if last_cand is None or last_cand_t is None:
        return True
    if (now - last_cand_t) > stale_sec:
        return True
    v = float(last_cand)
    return math.isfinite(v) and abs(v - released) < 1e-3


def wp_step(now, wp_enter_t, timeout, dist, arrive_r, t_start, dwell, may_count):
    """WP 하나의 전진 판정 한 틱. → (event, t_start, remain).

    event : 'timeout'(시간 초과 → 다음 WP) | 'arrive'(반경 안 dwell 완료) | None
    t_start: 다음 틱에 들고 갈 dwell 시작 시각(None = 안 세는 중)
    remain : 남은 시간(초, /wp_remaining_time). 타임아웃 없는 모드면 None

    순서 = 노드의 옛 timer_cb 그대로: 시간 초과를 먼저 본다(dwell 대기·fix 유무와 무관).
    dist None(fix 없음·stale)·반경 밖·대기(may_count False) → t_start 비움.
    처음 조건을 만족한 틱은 t_start 만 찍고, dwell 경과는 다음 틱부터 본다.
    """
    remain = None
    if timeout is not None:
        remain = max(timeout - (now - wp_enter_t), 0.0)
        if remain <= 0:
            return 'timeout', None, remain
    if dist is None or dist >= arrive_r or not may_count:
        return None, None, remain
    if t_start is None:
        return None, now, remain
    if (now - t_start) >= dwell:
        return 'arrive', None, remain
    return None, t_start, remain


def candidate_after_course(last_mode, fallback_modes, released=CANDIDATE_RELEASED):
    """코스가 끝난 뒤(wp_idx ≥ len) north 가 /candidate_angle 로 낼 값. None = 안 낸다.

    마지막 WP 가 north 담당 모드(fallback 5·8)였으면 그 모드에서 내던 20000 을 계속 낸다 —
    끊으면 ship_direction 이 1.0 s 뒤 같은 20000 으로 대체하면서 5 s 마다 거짓
    'candidate 갱신 없음' ERROR 를 낸다(C1). 값이 같으니 조향은 그대로다.
    마지막 모드에 담당 노드(0·1 gate, 2·3 back, 7 dock)가 있으면 내지 않는다 — 소비자는 여전히
    그 모드로 알고 있어 담당 노드가 계속 발행 중이고, north 가 내면 발행자가 둘이 된다.
    """
    if last_mode in fallback_modes:
        return released
    return None

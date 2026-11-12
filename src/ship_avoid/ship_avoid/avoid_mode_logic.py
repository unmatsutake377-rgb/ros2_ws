"""ship_avoid 틱 판정 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

계약서: docs/문제와작업/계약서_도리도리회피_launch_20261113.md C1 ②③④

    from ship_avoid.avoid_mode_logic import check_avoid_mode, check_stale_sec, plan_tick, fixed_candidate

avoid_mode (yaml 스위치, 기본 'lidar')
  · 'lidar'    : 담당 모드 동안 매 틱 20000 만 낸다. GPS·yaw 와 상관없다 = 지금 north 폴백과 같은 동작.
  · 'doridori' : 도리도리 코어(AvoidCore)로 돈다. 단 /avoid_detections 가 살아 있을 때만.
                 한 번도 못 받았거나 detections_quiet_warn_sec 넘게 조용하면 20000 으로 넘기고
                 도리도리를 시작하지 않는다(코어 비활성). 다시 들어오면 정상 경로.

틱 상태 (plan_tick 반환)
  IDLE      담당 모드 아님 / /wp_mode 를 아직 못 받음 / 묵었는데 마지막 값이 담당 모드 아님 → 발행 안 함
  WP_STALE  /wp_mode 가 wp_mode_stale_sec 넘게 묵음 + 마지막 값이 담당 모드 → 발행 안 함 + ERROR
  LIDAR     20000
  DET_WAIT  doridori, 검출 없음 — 담당 모드 진입 후 아직 quiet 초 안(검출기가 깨어나는 중) → 20000, 로그 없음
  DET_DEAD  doridori, 검출 없음 — 진입 후 quiet 초 넘게 조용 → 20000 + ERROR
  DORI      도리도리 코어 경로(GPS·yaw stale 검사는 노드가 그대로 한다)
"""

import math

CANDIDATE_INVALID = 20000.0     # core.CANDIDATE_INVALID 와 같은 값 (ship_direction (C) 자율 회피에 맡김)

AVOID_MODES = ('lidar', 'doridori')
# 아래 셋 = ship_avoid.py declare_parameter 리터럴 = config/ship_avoid.yaml (test_node_params 가 묶는다).
#   잘못된 값일 때 돌아갈 값이다.
DEFAULT_AVOID_MODE = 'lidar'
DEFAULT_WP_MODE_STALE_SEC = 1.0     # north /wp_mode 주기(period_sec 0.5)의 2배
DEFAULT_DET_QUIET_SEC = 5.0

IDLE = 'IDLE'
WP_STALE = 'WP_STALE'
LIDAR = 'LIDAR'
DET_WAIT = 'DET_WAIT'
DET_DEAD = 'DET_DEAD'
DORI = 'DORI'

_FIXED = {LIDAR: CANDIDATE_INVALID, DET_WAIT: CANDIDATE_INVALID, DET_DEAD: CANDIDATE_INVALID}


def check_avoid_mode(value):
    """avoid_mode 값 검사 → (쓸 모드, 오류 메시지 또는 None). 허용값과 정확히 같아야 한다(대소문자 구분)."""
    if isinstance(value, str) and value in AVOID_MODES:
        return value, None
    return DEFAULT_AVOID_MODE, (f"🚨 avoid_mode={value!r} 는 잘못된 값(허용 {list(AVOID_MODES)}) "
                                f"→ '{DEFAULT_AVOID_MODE}' 로 돈다. ship_avoid.yaml 확인.")


def check_stale_sec(name, value, default):
    """양의 유한 초 검사 → (쓸 값, 오류 메시지 또는 None)."""
    try:
        v = float(value)
        ok = math.isfinite(v) and v > 0.0
    except (TypeError, ValueError):
        ok = False
    if ok:
        return v, None
    return float(default), f"🚨 {name}={value!r} 는 잘못된 값(양의 유한 초) → {default:g} s 로 돈다. ship_avoid.yaml 확인."


def _fresh(last_t, now, limit):
    return last_t is not None and now - last_t <= limit


def plan_tick(avoid_mode, wp_mode, wp_mode_t, active_wp_mode, now, wp_mode_stale_sec,
              last_det_t, active_since_t, det_quiet_sec):
    """이번 틱 상태(위 표). 순수 함수.

    wp_mode / wp_mode_t : 마지막으로 받은 /wp_mode 값과 받은 시각(못 받았으면 None)
    last_det_t          : 마지막 /avoid_detections 수신 시각(못 받았으면 None)
    active_since_t      : 이번 담당 구간 진입 시각(노드가 들고 있다). None 이면 지금 진입한 것으로 본다.
    """
    if wp_mode is None or wp_mode_t is None:
        return IDLE
    if not _fresh(wp_mode_t, now, wp_mode_stale_sec):
        return WP_STALE if wp_mode == active_wp_mode else IDLE
    if wp_mode != active_wp_mode:
        return IDLE
    if avoid_mode != 'doridori':
        return LIDAR
    if _fresh(last_det_t, now, det_quiet_sec):
        return DORI
    since = now if active_since_t is None else active_since_t
    return DET_DEAD if now - since > det_quiet_sec else DET_WAIT


def fixed_candidate(state):
    """상태가 정하는 /candidate_angle 값. 20000 이거나 None(= 발행 안 함 또는 DORI 면 코어가 정함)."""
    return _FIXED.get(state)


def is_active(state):
    """담당 구간 안인가(진입 시각을 들고 있을지). IDLE·WP_STALE 이면 아니다 — 코어도 비활성으로 돌린다."""
    return state not in (IDLE, WP_STALE)

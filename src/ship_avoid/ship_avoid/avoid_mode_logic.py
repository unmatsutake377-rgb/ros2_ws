"""ship_avoid 틱 판정·검출 형식 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

계약서: docs/문제와작업/계약서_도리도리회피_launch_20261113.md C1 ②③④
        docs/문제와작업/계약서_도리도리회피_v2_20261114.md U1 (/avoid_zone · ZONE_OFF · /avoid_detections VER/STRIDE)
        + 사용자 결정 11-14: 모드 이름 'doridori' → 'fused'(명세 1~8 전체), 기본값 'fused'

    from ship_avoid.avoid_mode_logic import check_avoid_mode, check_stale_sec, plan_tick, fixed_candidate

avoid_mode (yaml 스위치, 기본 'fused')
  · 'lidar'    : 담당 모드 동안 매 틱 20000 만 낸다. GPS·yaw·/avoid_zone 과 상관없다 = 지금 north 폴백과 같은 동작.
                 이상값일 때 돌아가는 값(안전 쪽).
  · 'fused'    : 도리도리 코어(AvoidCore)로 돈다(예전 이름 'doridori' — 들어오면 'fused' 로 받고 WARN). 단
                 ① /avoid_zone 이 avoid_zone_stale_sec 안에 들어왔고 true 일 때만(명세 8 — 장애물 구간 다리).
                    못 받았거나 묵었거나 false 면 20000(ZONE_OFF).
                 ② /avoid_detections 가 살아 있을 때만. 한 번도 못 받았거나 detections_quiet_warn_sec 넘게
                    조용하면 20000 으로 넘기고 도리도리를 멈춘다(코어 일시 중지 — 지도는 zone 구간 동안 유지).
                 🚨 기본이 fused 여도 north 폴리곤이 비면 /avoid_zone 이 늘 false → ZONE_OFF = 20000
                    (ERROR 없음) — 배 동작은 lidar 와 같다.

틱 상태 (plan_tick 반환, 판정 순서 그대로)
  IDLE      담당 모드 아님 / /wp_mode 를 아직 못 받음 / 묵었는데 마지막 값이 담당 모드 아님 → 발행 안 함
  WP_STALE  /wp_mode 가 wp_mode_stale_sec 넘게 묵음 + 마지막 값이 담당 모드 → 발행 안 함 + ERROR
  LIDAR     20000 (zone 무관)
  ZONE_OFF  fused, /avoid_zone 미수신·stale·false → 20000 (지도·코스 축 초기화)
  DET_WAIT  fused·zone 안, 검출 없음 — 담당 모드 진입 후 아직 quiet 초 안(검출기가 깨어나는 중) → 20000, 로그 없음
  DET_DEAD  fused·zone 안, 검출 없음 — 진입 후 quiet 초 넘게 조용 → 20000 + ERROR
  DORI      도리도리 코어 경로(GPS·yaw stale 검사는 노드가 그대로 한다)

  ZONE_OFF 를 DET_* 보다 먼저 보는 이유: zone 밖이면 도리도리를 안 하므로 검출기 상태는 상관없다
  (zone 밖에서 검출 침묵 ERROR 를 내면 거짓 경보 — 검출기·zone 발행자가 둘 다 없는 지금 기본 fused 에서
  5 s 마다 DET_DEAD ERROR 가 찍히지 않게). LIDAR 보다 뒤인 이유: lidar 는 zone 과 무관하다.

지도 수명 (keeps_map) = zone 구간
  DORI·DET_WAIT·DET_DEAD 가 이어지는 동안(= 담당 모드 + zone 신선·true) 지도·코스 축을 유지한다.
  검출 침묵(DET_*)은 코어를 잠깐 멈출 뿐 지도는 그대로 — 검출이 돌아오면 같은 지도로 이어 간다.
  그 밖의 상태(IDLE·WP_STALE·LIDAR·ZONE_OFF)가 한 틱이라도 오면 구간이 끝난 것으로 보고 지도를 버린다.

/avoid_detections (Float32MultiArray) 형식 — G1 (a)
  [VER=1, STRIDE=6, N, N × (cls, bearing_left_deg, v_norm, range_m, conf, age_s)]
    cls 0 = 장애물(검출기 obstacle_color, 기본 흰 — 계약서_흰부표_거리융합_v3 W1), 2 = 노랑. bearing 좌 +(deg).
    v_norm = (v_bottom − cy)/fy.
    range_m = D455 거리, 모르면 NaN. age_s = 촬영부터 발행까지 초.
  · VER·STRIDE·N·길이가 하나라도 안 맞거나 cls·bearing·v_norm·conf·age_s 가 유한수가 아니면 → 메시지 전체를 버린다
    (살아 있음으로도 안 친다 — 검출기 고장).
  · N = 0 은 정상(본 것 없음) = 살아 있음(하트비트).
  · age_s > det_max_age_s 인 검출은 그 검출만 버린다(메시지는 살아 있음). age_s < 0 도 형식 오류로 본다.
  · 예전 stride 5 형식([cls, …] × N, 머리 없음)은 받지 않는다 — 발행자가 없었다.
"""

import math

CANDIDATE_INVALID = 20000.0     # core.CANDIDATE_INVALID 와 같은 값 (ship_direction (C) 자율 회피에 맡김)

AVOID_MODES = ('lidar', 'fused')
AVOID_MODE_ALIASES = {'doridori': 'fused'}   # 옛 이름(11-14 전 앱·문서) → 받아들이고 WARN
INVALID_AVOID_MODE = 'lidar'                 # 이상값이면 이것으로 돈다(안전 쪽 — 매 틱 20000)
# 아래 = ship_avoid.py declare_parameter 리터럴 = config/ship_avoid.yaml (test_node_params 가 묶는다).
#   DEFAULT_AVOID_MODE 외의 초 값은 잘못된 값일 때 돌아갈 값이다.
DEFAULT_AVOID_MODE = 'fused'
DEFAULT_WP_MODE_STALE_SEC = 1.0     # north /wp_mode 주기(period_sec 0.5)의 2배
DEFAULT_DET_QUIET_SEC = 5.0
DEFAULT_ZONE_STALE_SEC = 1.0        # north /avoid_zone 주기(timer_cb, period_sec 0.5)의 2배 — /wp_mode 와 같은 기준
DEFAULT_DET_MAX_AGE_S = 0.5         # ⚠️ 실측 필요(검출기 U3 지연). 근거는 ship_avoid.yaml 주석
DEFAULT_IMU_STALE_SEC = 1.0         # /imu/data 이보다 묵으면 IMU 피치·롤 0(장착값만) — W2b ⑦

DET_VER = 1
DET_STRIDE = 6
DET_HEADER = 3                      # VER, STRIDE, N

IDLE = 'IDLE'
WP_STALE = 'WP_STALE'
LIDAR = 'LIDAR'
ZONE_OFF = 'ZONE_OFF'
DET_WAIT = 'DET_WAIT'
DET_DEAD = 'DET_DEAD'
DORI = 'DORI'

# /avoid_zone 상태 (zone_status 반환)
ZONE_NEVER = 'NEVER'     # 한 번도 못 받음 (U2 north 발행 전에는 늘 이 상태)
ZONE_STALE = 'STALE'     # 받은 적은 있는데 묵음
ZONE_FALSE = 'FALSE'
ZONE_TRUE = 'TRUE'

# /imu/data 상태 (imu_status 반환) — W2b ⑦
IMU_NEVER = 'NEVER'      # 한 번도 못 받음 — 지금 배의 정상(iahrs 미발행, 11-14 결정): 활성 때 WARN 1회
IMU_STALE = 'STALE'      # 받다가 끊김 → 피치·롤 0(장착값만) + 주기 WARN
IMU_OK = 'OK'

_FIXED = {LIDAR: CANDIDATE_INVALID, ZONE_OFF: CANDIDATE_INVALID,
          DET_WAIT: CANDIDATE_INVALID, DET_DEAD: CANDIDATE_INVALID}
_KEEPS_MAP = (DORI, DET_WAIT, DET_DEAD)


def check_avoid_mode(value):
    """avoid_mode 값 검사 → (쓸 모드, 오류 메시지 또는 None, 경고 메시지 또는 None).

    허용값과 정확히 같아야 한다(대소문자 구분). 옛 이름(AVOID_MODE_ALIASES)은 새 이름으로 받고 경고만 낸다.
    그 밖의 값은 오류 + INVALID_AVOID_MODE('lidar').
    """
    if isinstance(value, str) and value in AVOID_MODES:
        return value, None, None
    if isinstance(value, str) and value in AVOID_MODE_ALIASES:
        new = AVOID_MODE_ALIASES[value]
        return new, None, (f"avoid_mode={value!r} 는 이름 바뀜 → '{new}' 로 돈다(같은 동작). "
                           f"ship_avoid.yaml·앱을 '{new}' 로 고칠 것.")
    return INVALID_AVOID_MODE, (f"🚨 avoid_mode={value!r} 는 잘못된 값(허용 {list(AVOID_MODES)}) "
                                f"→ '{INVALID_AVOID_MODE}' 로 돈다. ship_avoid.yaml 확인."), None


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


def zone_status(zone, zone_t, now, zone_stale_sec):
    """/avoid_zone 상태 → ZONE_NEVER / ZONE_STALE / ZONE_FALSE / ZONE_TRUE."""
    if zone is None or zone_t is None:
        return ZONE_NEVER
    if not _fresh(zone_t, now, zone_stale_sec):
        return ZONE_STALE
    return ZONE_TRUE if zone else ZONE_FALSE


def imu_status(imu_t, now, imu_stale_sec):
    """/imu/data 상태 → IMU_NEVER / IMU_STALE / IMU_OK."""
    if imu_t is None:
        return IMU_NEVER
    return IMU_OK if _fresh(imu_t, now, imu_stale_sec) else IMU_STALE


def plan_tick(avoid_mode, wp_mode, wp_mode_t, active_wp_mode, now, wp_mode_stale_sec,
              last_det_t, active_since_t, det_quiet_sec, zone, zone_t, zone_stale_sec):
    """이번 틱 상태(위 표). 순수 함수.

    wp_mode / wp_mode_t : 마지막으로 받은 /wp_mode 값과 받은 시각(못 받았으면 None)
    last_det_t          : 마지막으로 **형식이 맞는** /avoid_detections 를 받은 시각(못 받았으면 None)
    active_since_t      : 이번 담당 구간 진입 시각(노드가 들고 있다). None 이면 지금 진입한 것으로 본다.
    zone / zone_t       : 마지막 /avoid_zone 값(bool)과 받은 시각(못 받았으면 None)
    """
    if wp_mode is None or wp_mode_t is None:
        return IDLE
    if not _fresh(wp_mode_t, now, wp_mode_stale_sec):
        return WP_STALE if wp_mode == active_wp_mode else IDLE
    if wp_mode != active_wp_mode:
        return IDLE
    if avoid_mode != 'fused':
        return LIDAR
    if zone_status(zone, zone_t, now, zone_stale_sec) != ZONE_TRUE:
        return ZONE_OFF
    if _fresh(last_det_t, now, det_quiet_sec):
        return DORI
    # 검출 유예는 담당 모드 진입 기준(검출기는 wp_mode 로 깨어난다 — zone 과 무관)
    since = now if active_since_t is None else active_since_t
    return DET_DEAD if now - since > det_quiet_sec else DET_WAIT


def fixed_candidate(state):
    """상태가 정하는 /candidate_angle 값. 20000 이거나 None(= 발행 안 함 또는 DORI 면 코어가 정함)."""
    return _FIXED.get(state)


def is_active(state):
    """담당 구간 안인가(진입 시각을 들고 있을지). IDLE·WP_STALE 이면 아니다 — 코어도 비활성으로 돌린다."""
    return state not in (IDLE, WP_STALE)


def keeps_map(state):
    """이 상태면 도리도리 지도·코스 축을 유지한다(zone 구간 안). 아니면 지도를 버린다."""
    return state in _KEEPS_MAP


def _finite(v):
    return isinstance(v, (int, float)) and math.isfinite(v)


def parse_detections(data, max_age_s):
    """/avoid_detections data → (dets, n_old, err).

    dets  : [(cls:int, bearing_left_deg, v_norm, range_m, conf, age_s)] — age_s ≤ max_age_s 인 것만.
            형식 오류면 None (메시지 전체 버림, 살아 있음으로 안 침).
    n_old : age_s 초과로 버린 검출 수
    err   : 형식 오류 설명(오류 없으면 None)
    range_m 은 유한하지 않으면 NaN 으로 넘긴다(= 모름, core 가 단안 거리로 대신).
    """
    try:
        d = [float(v) for v in data]
    except (TypeError, ValueError):
        return None, 0, '숫자가 아닌 값'
    if len(d) < DET_HEADER:
        return None, 0, f'길이 {len(d)} < 머리 {DET_HEADER}(VER, STRIDE, N)'
    ver, stride, n = d[0], d[1], d[2]
    if ver != DET_VER:
        return None, 0, f'VER={ver!r} ≠ {DET_VER}'
    if stride != DET_STRIDE:
        return None, 0, f'STRIDE={stride!r} ≠ {DET_STRIDE}'
    if not (math.isfinite(n) and n >= 0 and n == int(n)):
        return None, 0, f'N={n!r} 은 0 이상 정수가 아님'
    n = int(n)
    if len(d) != DET_HEADER + n * DET_STRIDE:
        return None, 0, f'길이 {len(d)} ≠ {DET_HEADER} + N({n}) × {DET_STRIDE}'
    dets, n_old = [], 0
    for i in range(n):
        cls, brg, v_norm, rng, conf, age = d[DET_HEADER + i * DET_STRIDE: DET_HEADER + (i + 1) * DET_STRIDE]
        if not all(_finite(v) for v in (cls, brg, v_norm, conf, age)) or cls != int(cls) or age < 0.0:
            return None, 0, (f'검출 {i}: 값 이상(cls={cls!r}, bearing={brg!r}, v_norm={v_norm!r}, '
                             f'conf={conf!r}, age_s={age!r})')
        if age > max_age_s:
            n_old += 1
            continue
        dets.append((int(cls), brg, v_norm, rng if math.isfinite(rng) else float('nan'), conf, age))
    return dets, n_old, None

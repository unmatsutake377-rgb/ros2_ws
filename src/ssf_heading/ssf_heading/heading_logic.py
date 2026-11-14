"""
헤딩(뱃머리 절대방위) 순수 로직 — ROS 비의존. 시간은 주입받는다.

배경 (CLAUDE.md 3-5):
  작년 iahrs_driver 는 /imu/yaw 를 이렇게 만들었다.
    ② yaw = imu_yaw_cw - yaw_initial_offset      ← 부팅 시점 뱃머리를 0 으로 (상대각)
    ③ if (GPS heading 유효) yaw = gps_heading    ← IMU 를 통째로 버리고 덮어씀

  ② 때문에 /imu/yaw 는 절대방위가 아니다. 그런데 north_goal_angle 은 절대방위를 계산하고
     ship_goal_angle 은 둘을 뺀다 → 뺄셈이 무의미하다.
  ③ 은 더 나쁘다. NavPVT.heading 은 'Heading of motion'(COG, 대지침로)이지 뱃머리가 아니다.
     - 정지/저속에서 COG 는 노이즈다 (접안·정지유지 구간에서 yaw 가 튄다)
     - 조류·바람에 게걸음하면 COG 와 뱃머리가 벌어진다 (그 차이를 '추정' 하는 게 옵션 B 인데
       작년 코드는 차이를 0 이라고 '가정' 해버렸다)
     - 게다가 g_speed 를 안 봐서 속도 0 에서도 통과했다 (head_acc 임계 30° 도 헐렁하다)

  → 드라이버는 '보정 안 한 상대 yaw' 만 /imu/yaw_raw 로 내고,
    절대방위 합성은 이 모듈 + yaw_mux 노드가 전담한다. /imu/yaw 발행자는 yaw_mux 하나뿐이다.

각도 규약:
  - 전부 도(degree), 0~360, **시계방향 증가**(나침반 규약). 0 = 진북.
  - 드라이버가 IMU 원본(CCW 증가)을 이미 CW 로 뒤집어 놓는다. invert_yaw 는 그게 틀렸을 때의
    탈출구이지 기본 경로가 아니다. **부호는 벤치에서 확정한다 — 지금 추측하지 않는다.**

⚠️ 자기편각은 소스마다 적용 여부가 다르다 (SOURCE_IS_MAGNETIC 참고).
   듀얼 GPS(relPosNED)와 COG 는 이미 진북 기준이다. 여기에 편각을 더하면 이중 보정이다.
"""

import math

# ---------------------------------------------------------------- 소스 이름
SRC_IMU_RELATIVE = "imu_relative"   # 지금 쓸 수 있는 것: IMU 상대 yaw + 고정 오프셋
SRC_COG_OFFSET   = "cog_offset"     # 옵션 B: GPS COG 로 IMU 오프셋을 추정 (N2)
SRC_DUAL_GPS     = "dual_gps"       # 옵션 A: 듀얼 F9P relPosNED (N3, 보드 확보 후)
SRC_IMU_ABSOLUTE = "imu_absolute"   # 옵션 C: IMU 지자기 절대 heading (N4, 드라이버 조사 후)
SRC_GPS_COG      = "gps_cog"        # GPS 도플러 진행방향(COG) 그대로. IMU 절대값 배제 — 기동 구간만
                                    # IMU 상대 회전량(Δ)으로 보조 (계약서_GPS방위_20261114 보완 B)

ALL_SOURCES = (SRC_IMU_RELATIVE, SRC_COG_OFFSET, SRC_DUAL_GPS, SRC_IMU_ABSOLUTE,
               SRC_GPS_COG)

# 🚨 자기북 기준 소스만 자기편각(declination)을 받는다.
#    진북 기준 소스에 편각을 더하면 한국 기준 약 8° 가 통째로 틀어진다.
SOURCE_IS_MAGNETIC = {
    SRC_IMU_RELATIVE: False,   # 기준축이 임의 → mount_offset_deg 가 전부 흡수한다
    SRC_COG_OFFSET:   False,   # COG = 진북 기준
    SRC_DUAL_GPS:     False,   # relPosNED = 진북 기준
    SRC_IMU_ABSOLUTE: True,    # 자기북 기준 → 편각 보정 필요
    SRC_GPS_COG:      False,   # COG = 진북 기준. mount 는 안 건다, invert 는 보조 Δ 부호에만 (HeadingMux.heading)
}

# ---------------------------------------------------------------- 상태 코드
ST_OK              = "OK"
ST_NO_DATA         = "NO_DATA"          # 아직 한 번도 안 들어옴
ST_STALE           = "STALE"            # 들어왔었는데 끊김
ST_NOT_IMPLEMENTED = "NOT_IMPLEMENTED"  # 소스는 골랐는데 구현 전 (N3/N4)
ST_NOT_CONVERGED   = "NOT_CONVERGED"    # 소스는 돌고 있는데 아직 추정이 안 섰다 (N2)
# gps_cog 전용 (계약서_GPS방위_20261114)
ST_HOLD            = "HOLD"             # 새 COG 가 gps_cog_fresh_sec 넘게 없다 → 마지막 방위 유지·발행
ST_INIT_GOAL       = "INIT_GOAL"        # 첫 유효 COG 전 → /north_goal_angle_tp 를 초기 방위로 발행
ST_INIT            = "INIT"             # 첫 유효 COG 전 + tp 도 없음(또는 stale) → 침묵
ST_IMU_ASSIST      = "ASSIST"           # 기동(선회·후진) 구간 — COG 앵커 + IMU 상대 회전량 ΣΔ 발행 (보완 B)

# 침묵해야 하는 상태. 틀린 방위를 내보내느니 안 내보낸다.
# (north_goal_angle 은 /imu/yaw 없으면 geofence 를 침묵시키고,
#  ship_goal_angle 은 imu_stale 이면 /yaw_error 발행을 멈춘다 — 둘 다 이미 그렇게 돼 있다)
SILENT_STATES = (ST_NO_DATA, ST_STALE, ST_NOT_IMPLEMENTED, ST_NOT_CONVERGED, ST_INIT)

# 발행하는 상태. HOLD·INIT_GOAL·ASSIST 는 SILENT_STATES 에 넣지 않는다(계약 — 출발 교착·정지 중 침묵 방지).
# 🚨 INIT_GOAL 은 CLAUDE.md §1-6·§3 "헤딩 없으면 발행하지 않는다" 의 **승인된 예외**다
#    (Davidson Q1 = (c), 11-14). 출발 때 뱃머리를 현재 WP 로 겨눈다는 전제. tp 도 없으면 INIT(침묵).
# 🚨 ASSIST 가 여기서 빠지면 기동마다 /imu/yaw 가 침묵 → /yaw_error 정지 → 페일세이프 (보완 B 위험 5).
PUBLISH_STATES = (ST_OK, ST_HOLD, ST_INIT_GOAL, ST_IMU_ASSIST)

# ---------------------------------------------------------------- /desired_angle 특수 신호
# 🚨 CLAUDE.md §3 특수 신호표와 값까지 같아야 한다 (공유 모듈 금지 §1-8 — ssf-reviewer 6번 점검 대상).
#    yaw_mux 는 선회 판정에만 쓴다. 260/180(후진)은 /motor_reverse 로 거른다.
SPIN_RIGHT = 5000.0         # 우선회
SPIN_LEFT = 6000.0          # 좌선회
SPIN_CODES = (SPIN_RIGHT, SPIN_LEFT)

# ---------------------------------------------------------------- /boat_mode (ssf_bridge)
# 🚨 CLAUDE.md §3 `/boat_*` 값 = ssf_boat.ino `enum Mode` 와 값까지 같다(0=WAIT 1=MANUAL 2=AUTO).
#    새 번호·변환표 금지. 공유 모듈 금지(§1-8) — 테스트가 펌웨어 enum·north_goal_angle MODE_AUTO 와 대조.
#    gps_cog 는 AUTO 진입 순간 초기화에만 쓴다 (계약서_GPS방위_20261114 보완 A).
BOAT_MODE_AUTO = 2

# gps_cog 첫 유효 COG 전 동작 (Q1). 지금은 (c) 하나 — 모르는 값은 조용히 넘기지 않고 거부한다.
GPS_COG_INIT_GOAL = "goal"
GPS_COG_INIT_CHOICES = (GPS_COG_INIT_GOAL,)

# 보완 B: 이 거부 사유가 나면 IMU 회전량 보조를 시작한다(기동 구간). 그 밖의 거부는 보조 '계속' 만.
ASSIST_START_REJECTS = ("reverse", "spin", "settle")


# ---------------------------------------------------------------- 각도 유틸
def wrap360(deg):
    """0 <= x < 360 으로 접는다."""
    return math.fmod(math.fmod(deg, 360.0) + 360.0, 360.0)


def wrap180(deg):
    """-180 <= x < 180 으로 접는다."""
    return wrap360(deg + 180.0) - 180.0


def ang_diff(a, b):
    """a - b 의 최단 부호차. -180 <= d < 180."""
    return wrap180(a - b)


def is_finite(x):
    """None / NaN / inf 를 한 번에 막는다. 센서값은 전부 이걸 통과시킨 뒤 쓴다."""
    return x is not None and isinstance(x, (int, float)) and math.isfinite(x)


def is_spin_code(value):
    """/desired_angle 값이 제자리 선회(5000/6000)인가. float32 왕복 오차를 0.5 로 흡수."""
    return is_finite(value) and any(abs(value - c) < 0.5 for c in SPIN_CODES)


def apply_corrections(yaw_deg, *, invert_yaw=False, mount_offset_deg=0.0,
                      declination_deg=0.0):
    """
    원시 yaw → 보정된 방위.

    순서가 중요하다:
      1) invert  : 센서 회전방향이 반대일 때만 (벤치에서 확정)
      2) mount   : IMU 를 선체에 삐뚤게 단 각도 + (상대모드에선) 기준축 오프셋
      3) 편각    : 자기북 → 진북. **자기북 기준 소스에만** 호출부에서 넘긴다.

    🚨 cog_offset 소스는 이 함수를 거치지 않는다 (HeadingMux.heading 참고).
       invert 는 추정기 '앞' 에서 걸고, mount 는 추정 오프셋이 이미 흡수하므로
       여기서 또 걸면 이중 적용이다.
    """
    y = -yaw_deg if invert_yaw else yaw_deg
    return wrap360(y + mount_offset_deg + declination_deg)


# ---------------------------------------------------------------- COG 유도
def cog_from_velocity(vx, vy, frame="enu"):
    """
    GPS 속도벡터 → (COG[deg, 0=북, 시계방향], 속력[m/s]).
    둘 다 못 구하면 (None, 0.0).

    🚨 프레임을 틀리면 헤딩이 90° 돌거나 좌우가 뒤집힌다. 벤치에서 확인할 것:
       배를 정북으로 천천히 전진시키고 COG 가 0 근처인지 본다.
       · enu (ROS 표준, ublox fix_velocity 기본): x=동, y=북  → COG = atan2(x, y)
       · ned                                   : x=북, y=동  → COG = atan2(y, x)
    """
    if not (is_finite(vx) and is_finite(vy)):
        return None, 0.0
    speed = math.hypot(vx, vy)
    if speed <= 0.0:
        return None, 0.0
    if frame == "enu":
        east, north = vx, vy
    elif frame == "ned":
        north, east = vx, vy
    else:
        raise ValueError(f"gps_vel_frame '{frame}' 를 모른다. 'enu' 또는 'ned'.")
    return wrap360(math.degrees(math.atan2(east, north))), speed


def saturation_samples(sample_dt_sec, half_life_sec):
    """
    지수감쇠 표본수가 포화하는 상한. half_life 가 0 이면 감쇠 없음 → 무한대.

        n_max = 1 / (1 - 0.5^(dt/half_life))

    🚨 이걸 모르면 조용히 안 죽는다.
       min_samples 를 n_max 보다 크게 잡으면 **영원히 수렴하지 않는다.**
       에러도 안 나고 그냥 계속 NOT_CONVERGED 다 — 이 프로젝트가 반복해 당한 침묵 실패 유형.
       예: GPS 10Hz(dt=0.1) + half_life 1s → n_max ≈ 14.9. min_samples=20 이면 절대 못 넘는다.
    """
    if half_life_sec <= 0.0:
        return float("inf")
    if sample_dt_sec <= 0.0:
        return float("inf")
    k = 0.5 ** (sample_dt_sec / half_life_sec)
    if k >= 1.0:
        return float("inf")
    return 1.0 / (1.0 - k)


# ---------------------------------------------------------------- 옵션 B 추정기
class COGOffsetEstimator:
    """
    IMU 상대 yaw 의 미지 오프셋을 GPS COG 로 추정한다 (옵션 B).

        offset  = COG - imu_yaw          (직진 중일 때만 표본)
        heading = imu_yaw + offset       (정지 중에도 IMU 가 유지한다)

    imu_yaw 는 **invert 만 적용한 값** wrap360(s·r) 이다 (s=±1, HeadingMux 가 넘긴다).
    mount·편각은 적용하지 않는다 — 그래서 offset = 장착 오프셋(mount) + 게걸음각 이 되고,
    출력 heading 에 mount 를 또 더하지 않는다(이중 적용 금지).

    🚨 원리적 한계 — 숨기지 말 것.
      COG 는 '배가 실제로 간 방향' 이지 '뱃머리 방향' 이 아니다. 조류·바람에 게걸음하면
      그 각도(crab angle)가 offset 에 통째로 흡수된다. 즉 이 추정기는
      **장착 오프셋 + 그날의 평균 게걸음각** 을 함께 재는 것이다.
      물살이 일정하면 실용상 문제없고, 방향이 자주 바뀌면 offset 이 흔들린다(R 이 떨어져 잡힌다).
      절대 헤딩이 정말 필요하면 옵션 A(듀얼 F9P)가 정답이다.

    각도 평균은 산술로 하면 안 된다 — 359° 와 1° 의 산술평균은 180°(정반대)다.
    단위벡터로 더한 뒤 atan2 로 되돌린다(원형통계).

    신뢰도 R = |벡터합| / 표본수 (원형 결과길이, 0~1):
      · R≈1 표본이 한 방향으로 모임 = 믿을 만함
      · R≈0 흩어짐 = 못 믿음. 게걸음 급변이 여기서 걸린다.

    🚨 후진은 R 로 못 막는다 — 별도 게이트가 필요하다.
      전·후진이 **섞이면** R 이 무너져 걸린다. 그런데 **후진만 일관되게** 지속되면
      표본이 서로 일치한 채 전부 180° 틀려서 **R=1.0 으로 정반대 offset 에 수렴**한다.
      (검산: 후진만 60표본 → R=1.000, offset 217°(참값 37°), 오차 정확히 180°)
      전진이 충분히 쌓인 뒤의 짧은 후진은 R 이 임계 아래로 내려가 '침묵' 으로 실패하지만
      (60s 전진 후 3s 후진 → R=0.868), **냉시작 직후 후진**은 그대로 뚫린다.
      → motor_control 이 발행하는 후진 상태를 게이트로 받는다. 모르면 안 모은다.
    """

    def __init__(self, *, min_speed_mps=0.8, max_turn_rate_dps=8.0,
                 min_samples=30.0, min_resultant=0.9, half_life_sec=60.0,
                 require_reverse_gate=True):
        self.min_speed_mps = float(min_speed_mps)
        self.max_turn_rate_dps = float(max_turn_rate_dps)
        self.min_samples = float(min_samples)
        self.min_resultant = float(min_resultant)
        self.half_life_sec = float(half_life_sec)
        # True 면 후진 상태를 모를 때(신호 없음/stale) 표본을 안 모은다 — 모르면 안 모은다.
        # False 로 끄면 게이트 없이도 수렴하지만, 지속 후진 시 180° 틀린 값에 수렴할 수 있다.
        self.require_reverse_gate = bool(require_reverse_gate)

        self._cx = 0.0          # 감쇠 벡터합
        self._cy = 0.0
        self._n = 0.0           # 감쇠 표본수(실수)
        self._last_t = None
        self.last_reject = None  # 왜 버렸는지 (진단용)

    # ------------------------------------------------ 표본 투입
    def update(self, imu_yaw_deg, cog_deg, speed_mps, turn_rate_dps, t,
               reverse=False):
        """
        조건을 통과한 표본만 누적한다. 통과 여부를 bool 로 돌려준다.

        reverse: True=후진 중, False=전진 중, None=모름(신호 없음/stale).
                 None 은 require_reverse_gate 가 True 면 배제한다.
        """
        if not (is_finite(imu_yaw_deg) and is_finite(cog_deg) and is_finite(speed_mps)):
            self.last_reject = "bad_input"
            return False
        if reverse is True:
            # 후진 중엔 COG 가 뱃머리와 180° 뒤집힌다. R 로는 못 걸러진다(위 docstring 참고).
            self.last_reject = "reverse"
            return False
        if reverse is None and self.require_reverse_gate:
            # 모르면 안 모은다. 다만 조용히 멈추면 '왜 수렴을 안 하지' 로 시간을 버린다
            # → 호출부가 last_reject 를 /heading_status 에 실어 보이게 한다.
            self.last_reject = "no_reverse_gate"
            return False
        if speed_mps < self.min_speed_mps:
            # 느리면 COG 는 노이즈다. 작년 드라이버가 g_speed 를 안 봐서 당한 바로 그 지점.
            self.last_reject = "slow"
            return False
        if is_finite(turn_rate_dps) and abs(turn_rate_dps) > self.max_turn_rate_dps:
            # 선회 중엔 COG 가 뱃머리를 따라오지 못한다(횡슬립). 표본이 오염된다.
            self.last_reject = "turning"
            return False

        self._decay_to(t)
        a = math.radians(wrap360(cog_deg - imu_yaw_deg))
        self._cx += math.cos(a)
        self._cy += math.sin(a)
        self._n += 1.0
        self.last_reject = None
        return True

    def _decay_to(self, t):
        """오래된 표본을 지수 감쇠시킨다 — IMU 가 느리게 드리프트해도 따라간다."""
        if self._last_t is not None and self.half_life_sec > 0.0:
            dt = t - self._last_t
            if dt > 0.0:
                k = 0.5 ** (dt / self.half_life_sec)
                self._cx *= k
                self._cy *= k
                self._n *= k
        self._last_t = t

    # ------------------------------------------------ 조회
    @property
    def samples(self):
        return self._n

    @property
    def resultant(self):
        """0~1. 표본이 한 방향으로 모인 정도."""
        if self._n <= 0.0:
            return 0.0
        return math.hypot(self._cx, self._cy) / self._n

    @property
    def converged(self):
        return self._n >= self.min_samples and self.resultant >= self.min_resultant

    def min_samples_reachable(self, sample_dt_sec):
        """주어진 표본 주기에서 min_samples 에 도달할 수 있나. False 면 영원히 수렴 못 한다."""
        return self.min_samples <= saturation_samples(sample_dt_sec, self.half_life_sec)

    @property
    def offset_deg(self):
        """수렴 전이면 None. 절대 '0.0' 을 대신 내지 않는다 — 0 은 '보정 없음' 처럼 보인다."""
        if not self.converged:
            return None
        return wrap360(math.degrees(math.atan2(self._cy, self._cx)))


# ---------------------------------------------------------------- gps_cog 추적기
class GPSCOGTracker:
    """
    GPS 도플러 진행방향(COG)을 그대로 방위로 쓴다 (계약서_GPS방위_20261114).
    IMU **절대값**은 안 쓴다. 기동(선회·후진) 구간에 한해 IMU **상대 회전량(Δ)만** 빌린다(보완 B, 11-15
    — 본문 사용자 규칙 6 "IMU 배제"·부록 R-Q9 를 이 범위에서 번복). 절대 방위의 앵커는 늘 COG 다.

    출력 = 앵커(값 v, 시각 t0, 회전율 w) 에서
        yaw(t) = v + w · min(max(t − t0, 0), predict_max_sec)
      · 채택 표본마다 앵커를 새로 놓는다. w = 직전 두 채택 표본(간격 ≤ fresh_sec)의 회전율,
        ±max_rate_dps 클램프. 짝이 없으면 0.
      · 거부·선회/후진 시작 순간엔 그 순간 출력값에서 **얼린다**(w=0). 마지막 COG 로 되돌아가며
        튀지 않게 — 출력은 연속이다.
      · 기동 구간 IMU 보조 (보완 B — 부록 R 의 R3 자리를 IMU 가 차지, R3 폐기·B-Q9):
          yaw(t) = 앵커 + ΣΔ,  Δ_i = ang_diff(s·r_i, s·r_{i−1})  (s = invert_yaw 부호, mount·편각은 상쇄)
          시작: 앵커 있음 + IMU 신선(imu_stale_sec) + imu_assist 이고, 기동 신호(note_maneuver)
                또는 update() 거부 사유 reverse/spin/settle.  INIT/INIT_GOAL(앵커 없음) 중엔 안 함(B-Q6).
          계속: 보조 중엔 slow·jump·no_*_gate·bad_input 거부에도 계속(B-Q7). 보조 밖 slow 는 C1 얼림.
          끝:   COG 채택 / 마지막 기동 신호 뒤 imu_assist_max_sec 초과(B-Q2 (iii)) / IMU stale(B-Q3)
                / reset_to_init / 스위치 off. 끝나는 순간 값에서 얼린다(안 튐).
          튐:   샘플당 |Δ| > imu_max_step_deg 인 Δ 만 버린다(B-Q4). °/s 기준은 안 쓴다(stamp 없음).

    표본 채택 조건 (계약 '구현 요구'):
      속도 ≥ min_speed_mps, 후진 아님, 최근 /desired_angle 이 5000/6000 아님,
      선회·후진이 끝난 뒤 settle_sec 경과, finite.
      후진·선회 상태를 모르면(None) require_gate 일 때 거부 (Q5 — 벤치는 게이트 하나로 둘 다 끔).
      앵커와 jump_deg 넘게 다른 표본은 **2회 연속**(서로 jump_deg 안)일 때만 채택
      — 후진 관성으로 뒤로 밀릴 때의 180° 오채택 방지 (Q5).

    상태 (우선순위 INIT/INIT_GOAL > ASSIST > HOLD > OK):
      INIT       첫 채택 전 + 초기 방위(tp) 없음/stale → None (침묵)
      INIT_GOAL  첫 채택 전 + tp 신선 → tp 를 그대로 (§1-6 승인된 예외, Q1 (c))
      ASSIST     기동 구간 IMU 보조 중 → 앵커 + ΣΔ (보완 B)
      OK         마지막 채택 표본 나이 ≤ fresh_sec
      HOLD       그보다 오래됨 → 마지막 방위를 **시간 제한 없이** 유지·발행 (사용자 규칙 2)

    🚨 COG 는 '간 방향' 이지 뱃머리가 아니다 — 게걸음각이 그대로 들어간다(계약서 토픽 표).
    """

    def __init__(self, *, min_speed_mps=0.5, fresh_sec=1.5, predict_max_sec=1.5,
                 max_rate_dps=30.0, settle_sec=2.0, jump_deg=90.0,
                 init=GPS_COG_INIT_GOAL, init_stale_sec=2.0, require_gate=True,
                 imu_assist=True, imu_assist_max_sec=10.0, imu_max_step_deg=10.0,
                 imu_stale_sec=0.5):
        if init not in GPS_COG_INIT_CHOICES:
            raise ValueError(
                f"gps_cog_init '{init}' 를 모른다. 가능한 값: {list(GPS_COG_INIT_CHOICES)}")
        self.min_speed_mps = float(min_speed_mps)
        self.fresh_sec = float(fresh_sec)
        self.predict_max_sec = float(predict_max_sec)
        self.max_rate_dps = float(max_rate_dps)
        self.settle_sec = float(settle_sec)
        self.jump_deg = float(jump_deg)
        self.init = init
        self.init_stale_sec = float(init_stale_sec)
        self.require_gate = bool(require_gate)

        self._anchor = None             # (값, 시각, 회전율) — 첫 채택 전엔 None
        self._prev_accept = None        # (cog, 시각) — 회전율 짝. 거부·선회 시 끊는다
        self._last_accept_t = None
        self._pending = None            # jump_deg 넘게 튄 표본 하나 (2회 연속 확인용)
        self._last_maneuver_t = None    # 마지막으로 선회/후진을 본 시각 (settle 기준)
        self._goal = (None, None)       # (/north_goal_angle_tp, 수신시각) — INIT_GOAL 용
        self.last_reject = None         # 왜 버렸는지 (진단용)

        # ---- 보완 B: 기동 구간 IMU 상대 회전량 보조 ----
        self.imu_assist = bool(imu_assist)                  # 스위치 = 런타임 되돌리기
        self.imu_assist_max_sec = float(imu_assist_max_sec)  # 마지막 기동 신호 뒤 경과 상한 (B-Q2 (iii))
        self.imu_max_step_deg = float(imu_max_step_deg)      # 샘플당 |Δ| 튐 기준 (B-Q4)
        self.imu_stale_sec = float(imu_stale_sec)            # yaw_mux stale_sec 재사용 (B-Q3)
        self._imu_last = (None, None)   # (s·r, 수신시각) — 직전 원시 샘플. 절대값으로는 안 쓴다
        self._assist = False
        self._assist_sum = 0.0          # 현재 앵커 이후 ΣΔ (얼림·재앵커마다 0 으로)
        self._assist_total = 0.0        # 이번 보조 시작 이후 ΣΔ (unwrapped, 진단용)
        self._assist_t0 = None          # 이번 보조 시작 시각
        # 보조 종료 사유별 누적 횟수·튐 거부 누적 — yaw_mux 가 증가분으로 로그를 낸다(§1-6 드러내기)
        self.assist_ends = {"cog": 0, "limit": 0, "imu_stale": 0, "reset": 0}
        self.imu_step_rejects = 0

    # ------------------------------------------------ 입력
    def update_goal(self, bearing_deg, t):
        if is_finite(bearing_deg):
            self._goal = (wrap360(bearing_deg), t)

    def reset_to_init(self):
        """
        첫 COG 전 상태로 되돌린다 (보완 A: AUTO 진입 초기화).
        앵커·회전율 짝·jump 대기 표본·'첫 COG 받음'(마지막 채택 시각)을 지운다
        → 다음 출력은 tp 신선하면 INIT_GOAL, 아니면 INIT(침묵).
        🚨 지우지 않는 것: 선회·후진 settle 시각(_last_maneuver_t)·tp(_goal) — 실제 기동 상태라 유지.
        진입 뒤 첫 COG 는 앵커가 없으니 jump 검사 없이 채택된다(진입 전 표본과 비교 안 함).
        보조(보완 B)도 끈다 — 앵커가 없으니 INIT_GOAL 중엔 보조하지 않는다(B-Q6).
        IMU 직전 샘플(_imu_last)은 센서 상태라 유지한다.
        """
        if self._assist:
            self.assist_ends["reset"] += 1
        self._stop_assist()
        self._anchor = None
        self._prev_accept = None
        self._last_accept_t = None
        self._pending = None
        self.last_reject = None

    def note_maneuver(self, t):
        """
        선회(5000/6000) 또는 후진을 봤다. 그 순간 값에서 얼리고 settle 시계를 다시 건다.
        보완 B: 앵커가 있고 IMU 가 신선하면 IMU 회전량 보조를 시작한다(이미 보조 중이면 계속).
        상한은 이 신호 시각부터 다시 잰다(B-Q2 (iii)).
        """
        self._expire_assist(t)          # 상한·IMU 공백은 이 신호 '전' 기준으로 먼저 판정
        self._last_maneuver_t = t
        self._pending = None
        self._prev_accept = None
        self._freeze(t)
        self._try_start_assist(t)

    def update_imu(self, yaw_cf_deg, t):
        """
        보완 B: IMU 원시 샘플 (invert 만 적용한 s·r — HeadingMux 가 넘긴다).
        절대값은 쓰지 않는다. 보조 중일 때만 직전 원시 샘플 대비 Δ 를 앵커에 더한다.
        mount_offset_deg·편각은 상수라 Δ 에서 상쇄된다(B 로컬 40.3 무관).
        """
        if not is_finite(yaw_cf_deg):
            return
        y = wrap360(yaw_cf_deg)
        self._expire_assist(t)          # 공백·상한을 넘긴 뒤에 온 샘플은 더하지 않는다
        prev = self._imu_last[0]
        self._imu_last = (y, t)
        if not self._assist or prev is None:
            return
        d = ang_diff(y, prev)
        if abs(d) > self.imu_max_step_deg:
            # 그 Δ 만 버린다. 한 샘플 튀었다 돌아오면 두 Δ 다 버려져 순증 0,
            # 기준이 계단으로 바뀌면 그 한 샘플만 잃는다(B-Q4).
            self.imu_step_rejects += 1
            return
        self._assist_sum += d
        self._assist_total += d

    def imu_fresh(self, t):
        """마지막 IMU 샘플이 imu_stale_sec 안에 있나 (보조 시작·유지 조건)."""
        it = self._imu_last[1]
        return it is not None and (t - it) <= self.imu_stale_sec

    @property
    def assist_active(self):
        return self._assist

    @property
    def assist_total_deg(self):
        """이번 보조 시작 이후 IMU 회전량 합 (unwrapped, CW+). 보조 밖이면 마지막 보조 값."""
        return self._assist_total

    def assist_elapsed(self, t):
        """이번 보조 시작 뒤 경과 [s]. 보조 밖이면 None."""
        if not self._assist or self._assist_t0 is None:
            return None
        return t - self._assist_t0

    def update(self, cog_deg, speed_mps, t, *, reverse, spin):
        """
        COG 표본 하나. 채택하면 True.
        reverse / spin: True / False / None(모름). None 은 require_gate 면 거부.
        보완 B: 채택하면 IMU 보조를 끝내고 COG 로 다시 앵커를 잡는다. jump 검사는 보조가 반영된
        출력과 비교한다(180° 선회 뒤 첫 COG 를 바로 채택).
        """
        self._expire_assist(t)
        if not is_finite(speed_mps):
            return self._reject("bad_input", t)
        if reverse is True:
            return self._reject("reverse", t)
        if reverse is None and self.require_gate:
            return self._reject("no_reverse_gate", t)
        if spin is True:
            return self._reject("spin", t)
        if spin is None and self.require_gate:
            return self._reject("no_spin_gate", t)
        if (self._last_maneuver_t is not None
                and (t - self._last_maneuver_t) < self.settle_sec):
            return self._reject("settle", t)
        if speed_mps < self.min_speed_mps:
            return self._reject("slow", t)
        if not is_finite(cog_deg):
            return self._reject("bad_input", t)

        c = wrap360(cog_deg)
        if self._anchor is not None and abs(ang_diff(c, self._output(t))) > self.jump_deg:
            if self._pending is None or abs(ang_diff(c, self._pending)) > self.jump_deg:
                self._pending = c
                return self._reject("jump", t, keep_pending=True)
            # 직전 표본도 같은 쪽으로 튀었다 = 2회 연속 → 진짜 방향 전환으로 보고 채택

        rate = 0.0
        if self._prev_accept is not None:
            pc, pt = self._prev_accept
            dt = t - pt
            if 0.0 < dt <= self.fresh_sec:
                rate = max(-self.max_rate_dps,
                           min(self.max_rate_dps, ang_diff(c, pc) / dt))
        if self._assist:
            self.assist_ends["cog"] += 1
        self._stop_assist()
        self._anchor = (c, t, rate)
        self._prev_accept = (c, t)
        self._last_accept_t = t
        self._pending = None
        self.last_reject = None
        return True

    # ------------------------------------------------ 출력
    def heading(self, t):
        """(yaw, 상태). yaw None = 발행하지 마라."""
        if self._anchor is None:
            g, gt = self._goal
            if gt is None or (t - gt) > self.init_stale_sec:
                return None, ST_INIT
            return g, ST_INIT_GOAL
        self._expire_assist(t)
        yaw = self._output(t)
        if self._assist:
            return yaw, ST_IMU_ASSIST
        if (t - self._last_accept_t) > self.fresh_sec:
            return yaw, ST_HOLD
        return yaw, ST_OK

    def age(self, t):
        """마지막 채택 표본 나이 [s]. 한 번도 채택 안 했으면 None."""
        if self._last_accept_t is None:
            return None
        return t - self._last_accept_t

    # ------------------------------------------------ 내부
    def _output(self, t):
        v, t0, rate = self._anchor
        if self._assist:
            # 보조 중 앵커는 얼려 있다(회전율 0) — 출력 = 앵커 + IMU ΣΔ
            return wrap360(v + self._assist_sum)
        dt = min(max(t - t0, 0.0), self.predict_max_sec)
        return wrap360(v + rate * dt)

    def _freeze(self, t):
        """그 순간 출력값(보조 중이면 ΣΔ 반영값)에서 앵커를 다시 잡고 회전율·ΣΔ 를 0 으로 → 출력 연속."""
        if self._anchor is not None:
            self._anchor = (self._output(t), t, 0.0)
            self._assist_sum = 0.0

    def _reject(self, reason, t, keep_pending=False):
        self.last_reject = reason
        if not keep_pending:
            self._pending = None
        self._prev_accept = None
        self._freeze(t)
        if reason in ASSIST_START_REJECTS:
            self._try_start_assist(t)       # 보완 B: 후진·선회·settle 거부도 보조 시작 신호
        return False

    # ------------------------------------------------ 내부: IMU 보조 (보완 B)
    def _try_start_assist(self, t):
        """조건이 맞으면 보조 시작. 이미 보조 중이면 그대로 계속(얼림이 이미 재앵커했다)."""
        if (self._assist or not self.imu_assist or self._anchor is None
                or not self.imu_fresh(t)):
            return
        if (self._last_maneuver_t is not None
                and (t - self._last_maneuver_t) > self.imu_assist_max_sec):
            return                          # 상한을 이미 넘긴 기동 신호로는 시작하지 않는다
        self._freeze(t)                     # 시작 순간 출력값 = 앵커 (연속)
        self._assist = True
        self._assist_sum = 0.0
        self._assist_total = 0.0
        self._assist_t0 = t

    def _expire_assist(self, t):
        """보조 중 IMU stale 또는 상한(마지막 기동 신호 뒤 경과) 초과면 그 값에서 얼리고 끝낸다."""
        if not self._assist:
            return
        if not self.imu_fresh(t):
            reason = "imu_stale"            # 마지막 IMU 샘플 시점 값에서 멈춘다(그 뒤 ΣΔ 없음)
        else:
            base = (self._last_maneuver_t if self._last_maneuver_t is not None
                    else self._assist_t0)
            if (t - base) <= self.imu_assist_max_sec:
                return
            reason = "limit"
        self.assist_ends[reason] += 1
        self._freeze(t)
        self._stop_assist()

    def _stop_assist(self):
        self._assist = False
        self._assist_sum = 0.0
        self._assist_t0 = None


def gps_cog_status_text(trk, status, t):
    """
    gps_cog 의 /heading_status 문자열 (yaw_mux 가 그대로 발행). 접두 '<source>:<state>' 유지.
    OK 는 뒤에 아무것도 안 붙인다 — healthcheck 가 endswith(':OK') 로 본다(게이트가 꺼졌을 때만 gate=off).
    ASSIST: 'gps_cog:ASSIST d=+47.3 t=3.2 rej=spin[ gate=off]' — d = 이번 보조 IMU 회전량(unwrapped),
    t = 보조 경과 [s] (보완 B-Q5). healthcheck 엔 진단 ⚠️ 하나 추가될 뿐 /health_ok 무영향.
    """
    gate = "" if trk.require_gate else " gate=off"
    rej = trk.last_reject or '-'
    if status == ST_OK:
        return f"{SRC_GPS_COG}:{ST_OK}{gate}"
    if status == ST_HOLD:
        age = trk.age(t)
        age_s = '--' if age is None else f'{age:.1f}'
        return f"{SRC_GPS_COG}:{ST_HOLD} age={age_s} rej={rej}{gate}"
    if status == ST_IMU_ASSIST:
        el = trk.assist_elapsed(t)
        el_s = '--' if el is None else f'{el:.1f}'
        return (f"{SRC_GPS_COG}:{ST_IMU_ASSIST} d={trk.assist_total_deg:+.1f} t={el_s} "
                f"rej={rej}{gate}")
    return f"{SRC_GPS_COG}:{status} rej={rej}{gate}"


def init_goal_overdue_sec(source, status, boat_mode, auto_enter_t, t, warn_after_sec):
    """
    C-Q5 (계약서_GPS방위_20261114 보완 C 위험 1): AUTO 진입 뒤 첫 COG 없이 INIT_GOAL 이 오래 가나.
    경고할 때면 AUTO 진입 뒤 경과 [s], 아니면 None. **부작용 없음** — 로그 판정만, 출력·상태 불변.

    경고 조건 (전부):
      · source == gps_cog          (imu_relative 등은 tp 를 안 쓴다)
      · status == INIT_GOAL        (첫 COG 채택 뒤면 OK/HOLD/ASSIST — 안 냄. INIT 침묵은
                                    yaw_mux '헤딩 없음' WARN 이 이미 드러낸다)
      · boat_mode == AUTO 이고 auto_enter_t 있음
            기준 시각 = 마지막 AUTO **진입** 시각(보완 A 의 초기화 시각과 같은 순간).
            AUTO 를 한 번도 못 받았으면(미수신·MANUAL·WAIT 만) 안 낸다 — 부팅 기준이면 벤치·
            MANUAL 대기 중 INIT_GOAL 은 정상인데 계속 울린다(거짓 경고). /boat_mode 미수신은
            yaw_mux 의 별도 WARN(BOAT_MODE_WARN_AFTER_SEC)이 드러낸다.
            AUTO 를 떠나 MANUAL/WAIT 이면 조종기가 몰고 있으니 안 낸다. 재진입마다 호출부가
            auto_enter_t 를 새 진입 시각으로 바꾼다(= 기준 재설정).
      · t − auto_enter_t > warn_after_sec (엄격히 큼)
    """
    if source != SRC_GPS_COG or status != ST_INIT_GOAL:
        return None
    if boat_mode != BOAT_MODE_AUTO or auto_enter_t is None:
        return None
    elapsed = t - auto_enter_t
    return elapsed if elapsed > warn_after_sec else None


# ---------------------------------------------------------------- 믹서
class HeadingMux:
    """
    여러 헤딩 소스 중 하나를 골라 절대방위를 낸다. A/B/C 를 갈아끼울 수 있는 공용 골격.

    사용:
        mux = HeadingMux(SRC_IMU_RELATIVE, mount_offset_deg=37.0)
        mux.update_imu(raw_yaw, t)
        yaw, status = mux.heading(t)
        if yaw is None (또는 status not in PUBLISH_STATES): 발행하지 않는다
    """

    def __init__(self, source, *, invert_yaw=False, mount_offset_deg=0.0,
                 declination_deg=0.0, stale_sec=0.5, estimator=None, gps_cog=None):
        if source not in ALL_SOURCES:
            raise ValueError(
                f"heading_source '{source}' 를 모른다. 가능한 값: {list(ALL_SOURCES)}")

        self.source = source
        self.invert_yaw = bool(invert_yaw)
        self.mount_offset_deg = float(mount_offset_deg)
        self.declination_deg = float(declination_deg)
        self.stale_sec = float(stale_sec)

        # 소스별 최신 입력 (값, 수신시각)
        self._imu = (None, None)        # /imu/yaw_raw       (상대 yaw)
        self._cog = (None, None)        # GPS COG            (N2)
        self._speed = 0.0               # GPS 대지속도 [m/s]  (N2 게이트용)
        self._dual = (None, None)       # 듀얼 GPS heading   (N3)
        self._mag = (None, None)        # 지자기 절대 heading (N4)

        # 옵션 B (N2). cog_offset 이 아니어도 만들어 둔다 — 다른 소스로 도는 동안에도
        # 추정을 돌려두면 /heading_status 로 '지금 전환하면 쓸 만한가' 를 미리 볼 수 있다.
        self.estimator = estimator if estimator is not None else COGOffsetEstimator()
        self._turn_rate_dps = 0.0       # imu yaw 미분 (선회 중 표본 배제용)
        self._reverse = (None, None)    # (후진중?, 수신시각) — motor_control 이 준다

        # gps_cog (계약서_GPS방위_20261114). 추정기처럼 소스와 무관하게 입력을 받지만
        # 출력은 source == gps_cog 일 때만 쓴다.
        self.gps_cog = gps_cog if gps_cog is not None else GPSCOGTracker()
        self._desired = (None, None)    # (/desired_angle, 수신시각) — ship_direction 이 준다
        self._boat_mode = None          # 마지막 /boat_mode (None = 한 번도 못 받음)

    # ------------------------------------------------ 입력
    def update_imu(self, raw_yaw_deg, t):
        if not is_finite(raw_yaw_deg):
            return
        y = wrap360(raw_yaw_deg)
        prev, prev_t = self._imu
        if prev is not None and prev_t is not None and t > prev_t:
            # 최단 부호차로 미분한다. 359→1 을 -358°/s 로 읽으면 안 된다.
            self._turn_rate_dps = ang_diff(y, prev) / (t - prev_t)
        self._imu = (y, t)
        # 보완 B: gps_cog 추적기에 invert 만 적용한 s·r 를 밀어 넣는다(Δ 부호 = invert_yaw 반영).
        #   _imu 슬롯·_turn_rate_dps 는 위 그대로 — imu_relative 등 다른 소스 출력은 비트 단위 불변.
        #   추적기는 기동 구간 보조 중일 때만 Δ 를 쓴다(절대값은 안 쓴다).
        self.gps_cog.update_imu(self._cog_frame_yaw(y), t)

    def update_cog(self, cog_deg, speed_mps, t):
        """COG 표본 도착. gps_cog 추적기에 넣고, IMU 와 짝지어 추정기에 넣는다.

        cog_deg=None, speed 0.0 (정지 — COG 정의 안 됨)도 받는다: gps_cog 는 그 순간 얼린다.
        추정기 경로는 예전처럼 무시한다.
        """
        # gps_cog 는 IMU 슬롯을 읽지 않는다 — 후진·선회 게이트만 본다(IMU Δ 는 update_imu 가 따로 민다).
        self.gps_cog.update(cog_deg, speed_mps, t,
                            reverse=self.reverse_state(t), spin=self.spin_state(t))
        if not (is_finite(cog_deg) and is_finite(speed_mps)):
            return
        self._cog = (wrap360(cog_deg), t)
        self._speed = float(speed_mps)

        # IMU 가 신선할 때만 짝지을 수 있다. 오래된 yaw 와 지금 COG 를 빼면 그 차이가
        # 그대로 오차가 된다 — 짝을 못 지으면 표본을 버린다(침묵이 오염보다 낫다).
        imu_yaw, imu_t = self._imu
        if imu_t is None or (t - imu_t) > self.stale_sec:
            self.estimator.last_reject = "imu_stale"
            return
        # invert 는 추정 '앞' 에서 건다. 원시 r 로 표본을 만들면 invert=T 일 때
        # 표본이 2H+c-m 이 되어 수렴 방위에서만 맞는다 (계약서_cog_offset보정순서_20261030).
        self.estimator.update(self._cog_frame_yaw(imu_yaw), self._cog[0], self._speed,
                              self._turn_rate_dps, t,
                              reverse=self.reverse_state(t))

    def update_dual_gps(self, heading_deg, t):
        if is_finite(heading_deg):
            self._dual = (wrap360(heading_deg), t)

    def update_reverse(self, is_reverse, t):
        """후진 상태 갱신. motor_control 이 PWM 규약의 소유자라 거기서 판정해 보낸다."""
        self._reverse = (bool(is_reverse), t)
        if self._reverse[0]:
            self.gps_cog.note_maneuver(t)     # gps_cog: 후진 시작 순간 얼림 + settle

    def update_desired_angle(self, value, t):
        """/desired_angle (ship_direction 최종 출력). 5000/6000 이면 제자리 선회 중."""
        if not is_finite(value):
            return
        self._desired = (float(value), t)
        if is_spin_code(value):
            self.gps_cog.note_maneuver(t)     # gps_cog: 선회 시작 순간 얼림 + settle

    def spin_state(self, t):
        """True/False/None. None = /desired_angle 없음 또는 stale → '모른다'."""
        val, ts = self._desired
        if ts is None or (t - ts) > self.stale_sec:
            return None
        return is_spin_code(val)

    def update_boat_mode(self, mode):
        """
        /boat_mode (ssf_bridge). AUTO **진입**이면 gps_cog 추적기를 첫 COG 전 상태로 되돌리고 True.
        진입 = 직전 값이 AUTO 아님(0/1/그 밖/None=미수신) 이고 지금 AUTO. 첫 수신이 AUTO 여도 진입.
        AUTO→AUTO 는 진입 아님. 한 번도 못 받으면 아무 일도 안 일어난다(= C1 동작).
        출력엔 source == gps_cog 일 때만 반영된다(다른 소스는 추적기를 안 읽는다).
        """
        prev = self._boat_mode
        self._boat_mode = int(mode)
        if self._boat_mode == BOAT_MODE_AUTO and prev != BOAT_MODE_AUTO:
            self.gps_cog.reset_to_init()
            return True
        return False

    @property
    def boat_mode(self):
        """마지막 /boat_mode. None = 한 번도 못 받음."""
        return self._boat_mode

    def update_goal_bearing(self, bearing_deg, t):
        """/north_goal_angle_tp (현재 WP 방위). gps_cog 첫 유효 COG 전 초기값 (Q1 (c))."""
        self.gps_cog.update_goal(bearing_deg, t)

    def reverse_state(self, t):
        """True/False/None. None = 신호 없음 또는 stale → '모른다'."""
        val, ts = self._reverse
        if ts is None or (t - ts) > self.stale_sec:
            return None
        return val

    def update_mag(self, heading_deg, t):
        if is_finite(heading_deg):
            self._mag = (wrap360(heading_deg), t)

    # ------------------------------------------------ 출력
    def heading(self, t):
        """
        (yaw_deg, status) 를 돌려준다. yaw_deg 가 None 이면 **발행하지 마라.**
        틀린 방위 하나가 배를 정반대로 보낸다 — 침묵이 낫다.
        """
        if self.source == SRC_IMU_RELATIVE:
            return self._from_slot(self._imu, t)

        if self.source == SRC_GPS_COG:
            # 🚨 IMU 슬롯(절대값)을 읽지 않는다(사용자 규칙 6 — 보완 B 로 'IMU 절대값 배제' 로 정정).
            #    기동 구간 IMU 상대 회전량(Δ)은 update_imu 가 추적기에 따로 밀어 넣는다.
            #    _correct 도 거치지 않는다 — mount_offset_deg 는 IMU 장착값이라 COG 에 걸면 그만큼
            #    틀어지고(B 로컬 40.3 이면 40°), 편각은 COG 가 진북 기준이라 이중 보정이다.
            #    invert_yaw 는 보조 Δ 의 부호에만 들어간다(_cog_frame_yaw).
            #    HOLD·INIT_GOAL·ASSIST 도 값을 낸다 → 호출부는 PUBLISH_STATES 로 발행을 판정한다.
            return self.gps_cog.heading(t)

        if self.source == SRC_DUAL_GPS:
            val, ts = self._dual
            if ts is None:
                return None, ST_NOT_IMPLEMENTED   # N3: 발행 노드가 아직 없다
            return self._from_slot(self._dual, t)

        if self.source == SRC_IMU_ABSOLUTE:
            val, ts = self._mag
            if ts is None:
                return None, ST_NOT_IMPLEMENTED   # N4: 드라이버가 아직 안 낸다
            return self._from_slot(self._mag, t)

        if self.source == SRC_COG_OFFSET:
            # IMU 가 본체다. COG 는 오프셋만 준다 — 그래서 정지 중에도 헤딩이 유지된다.
            val, ts = self._imu
            if ts is None:
                return None, ST_NO_DATA
            if (t - ts) > self.stale_sec:
                return None, ST_STALE
            off = self.estimator.offset_deg
            if off is None:
                # 아직 못 믿는다. 0 을 대신 내지 않는다 — 0 은 '보정 없음' 처럼 보인다.
                return None, ST_NOT_CONVERGED
            # 🚨 _correct 를 거치지 않는다. off 가 이미 mount + 게걸음을 흡수했고
            #    invert 는 추정 앞에서 걸었다. 여기서 또 걸면 invert·mount 이중 적용이다.
            #    편각도 안 건다 (COG 는 진북 기준, SOURCE_IS_MAGNETIC[cog_offset]=False).
            return wrap360(self._cog_frame_yaw(val) + off), ST_OK

        return None, ST_NOT_IMPLEMENTED

    # ------------------------------------------------ 내부
    def _cog_frame_yaw(self, raw_yaw_deg):
        """
        cog_offset 의 추정·출력이 쓰는 yaw = wrap360(s·r). invert 만 적용, mount·편각 미적용.

        🚨 _imu 슬롯 자체는 원시 r 로 둔다. 슬롯에 invert 를 넣으면 imu_relative 가
           _from_slot → _correct 에서 invert 를 한 번 더 건다(거울상 방위).
        """
        return wrap360(-raw_yaw_deg) if self.invert_yaw else raw_yaw_deg

    def _from_slot(self, slot, t):
        val, ts = slot
        if ts is None:
            return None, ST_NO_DATA
        if (t - ts) > self.stale_sec:
            return None, ST_STALE
        return self._correct(val), ST_OK

    def _correct(self, yaw_deg):
        decl = self.declination_deg if SOURCE_IS_MAGNETIC[self.source] else 0.0
        return apply_corrections(
            yaw_deg,
            invert_yaw=self.invert_yaw,
            mount_offset_deg=self.mount_offset_deg,
            declination_deg=decl,
        )

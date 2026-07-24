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

ALL_SOURCES = (SRC_IMU_RELATIVE, SRC_COG_OFFSET, SRC_DUAL_GPS, SRC_IMU_ABSOLUTE)

# 🚨 자기북 기준 소스만 자기편각(declination)을 받는다.
#    진북 기준 소스에 편각을 더하면 한국 기준 약 8° 가 통째로 틀어진다.
SOURCE_IS_MAGNETIC = {
    SRC_IMU_RELATIVE: False,   # 기준축이 임의 → mount_offset_deg 가 전부 흡수한다
    SRC_COG_OFFSET:   False,   # COG = 진북 기준
    SRC_DUAL_GPS:     False,   # relPosNED = 진북 기준
    SRC_IMU_ABSOLUTE: True,    # 자기북 기준 → 편각 보정 필요
}

# ---------------------------------------------------------------- 상태 코드
ST_OK              = "OK"
ST_NO_DATA         = "NO_DATA"          # 아직 한 번도 안 들어옴
ST_STALE           = "STALE"            # 들어왔었는데 끊김
ST_NOT_IMPLEMENTED = "NOT_IMPLEMENTED"  # 소스는 골랐는데 구현 전 (N3/N4)
ST_NOT_CONVERGED   = "NOT_CONVERGED"    # 소스는 돌고 있는데 아직 추정이 안 섰다 (N2)

# 침묵해야 하는 상태. 틀린 방위를 내보내느니 안 내보낸다.
# (north_goal_angle 은 /imu/yaw 없으면 geofence 를 침묵시키고,
#  ship_goal_angle 은 imu_stale 이면 /yaw_error 발행을 멈춘다 — 둘 다 이미 그렇게 돼 있다)
SILENT_STATES = (ST_NO_DATA, ST_STALE, ST_NOT_IMPLEMENTED, ST_NOT_CONVERGED)


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


def apply_corrections(yaw_deg, *, invert_yaw=False, mount_offset_deg=0.0,
                      declination_deg=0.0):
    """
    원시 yaw → 보정된 방위.

    순서가 중요하다:
      1) invert  : 센서 회전방향이 반대일 때만 (벤치에서 확정)
      2) mount   : IMU 를 선체에 삐뚤게 단 각도 + (상대모드에선) 기준축 오프셋
      3) 편각    : 자기북 → 진북. **자기북 기준 소스에만** 호출부에서 넘긴다.
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


# ---------------------------------------------------------------- 믹서
class HeadingMux:
    """
    여러 헤딩 소스 중 하나를 골라 절대방위를 낸다. A/B/C 를 갈아끼울 수 있는 공용 골격.

    사용:
        mux = HeadingMux(SRC_IMU_RELATIVE, mount_offset_deg=37.0)
        mux.update_imu(raw_yaw, t)
        yaw, status = mux.heading(t)
        if yaw is None: 발행하지 않는다
    """

    def __init__(self, source, *, invert_yaw=False, mount_offset_deg=0.0,
                 declination_deg=0.0, stale_sec=0.5, estimator=None):
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

    def update_cog(self, cog_deg, speed_mps, t):
        """COG 표본 도착. IMU 와 짝지어 추정기에 넣는다."""
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
        self.estimator.update(imu_yaw, self._cog[0], self._speed,
                              self._turn_rate_dps, t,
                              reverse=self.reverse_state(t))

    def update_dual_gps(self, heading_deg, t):
        if is_finite(heading_deg):
            self._dual = (wrap360(heading_deg), t)

    def update_reverse(self, is_reverse, t):
        """후진 상태 갱신. motor_control 이 PWM 규약의 소유자라 거기서 판정해 보낸다."""
        self._reverse = (bool(is_reverse), t)

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
            return self._correct(wrap360(val + off)), ST_OK

        return None, ST_NOT_IMPLEMENTED

    # ------------------------------------------------ 내부
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

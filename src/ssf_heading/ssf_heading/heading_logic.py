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
ST_NOT_IMPLEMENTED = "NOT_IMPLEMENTED"  # 소스는 골랐는데 구현 전 (N2/N3/N4)

# 침묵해야 하는 상태. 틀린 방위를 내보내느니 안 내보낸다.
# (north_goal_angle 은 /imu/yaw 없으면 geofence 를 침묵시키고,
#  ship_goal_angle 은 imu_stale 이면 /yaw_error 발행을 멈춘다 — 둘 다 이미 그렇게 돼 있다)
SILENT_STATES = (ST_NO_DATA, ST_STALE, ST_NOT_IMPLEMENTED)


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
                 declination_deg=0.0, stale_sec=0.5):
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

    # ------------------------------------------------ 입력
    def update_imu(self, raw_yaw_deg, t):
        if is_finite(raw_yaw_deg):
            self._imu = (wrap360(raw_yaw_deg), t)

    def update_cog(self, cog_deg, speed_mps, t):
        if is_finite(cog_deg) and is_finite(speed_mps):
            self._cog = (wrap360(cog_deg), t)
            self._speed = float(speed_mps)

    def update_dual_gps(self, heading_deg, t):
        if is_finite(heading_deg):
            self._dual = (wrap360(heading_deg), t)

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

        # N2 에서 채운다. 지금은 조용히 0 을 내지 않고 '구현 안 됨' 을 명시한다.
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

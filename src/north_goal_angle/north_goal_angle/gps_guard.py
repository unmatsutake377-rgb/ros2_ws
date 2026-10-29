"""GPS fix 유효성 판정 — ROS 비의존 순수 로직 (테스트 가능).

🚨 왜 필요한가 (2026-08-06 실기에서 발견):
    ublox 드라이버는 **fix 가 없어도 /fix 를 1Hz 로 계속 발행한다.**
    status=-1(NO_FIX), latitude=0.0, longitude=0.0 인 메시지가 그대로 온다.
    north_goal_angle 의 gps_cb 는 status 를 안 보고 have_fix=True 로 만들어서,
    **적도 대서양(0°N,0°E, 일명 Null Island)에서 대회장까지의 방위**를 계산해 발행했다.

    실측: fix 없는 상태에서 /north_goal_angle_tp = 47.96°
          calc_angle(0, 0, WP0) = 47.96°  ← 소수점까지 일치

    체인이 그대로 이어진다:
      /north_goal_angle_tp → ship_goal_angle → /yaw_error → ship_direction 이 그 방향으로 조향.
    u-blox 냉시작은 fix 까지 30초~수 분이 걸린다. 그 구간 내내 배가 엉뚱한 방위로 간다.
    값이 47.96 처럼 **그럴듯해 보여서** 눈으로는 절대 못 잡는다.

    CLAUDE.md 의 '모르면 입을 다문다' 를 GPS 만 어기고 있었다
    (yaw_mux 는 헤딩 없으면 발행을 멈추고, geofence 도 IMU stale 이면 침묵한다).

⚠️ 이건 **첫 방어선**이다. 공분산 필터(고오차 fix 거부)·점프 필터(순간이동 거부)는
   팀원 GPS LOS 이식 트랙의 `gps_filter.py` 로 별도로 들어온다
   (docs/전달용/회신_GPS코드_기능추출_수정제안.md). 그게 들어와도 이 가드는 그대로 남는다 —
   NO_FIX 는 '품질이 나쁜 fix' 가 아니라 '아예 fix 가 아닌 것' 이라 층이 다르다.
"""

import math

# sensor_msgs/NavSatStatus 상수 (여기서 재정의 — 이 모듈은 ROS 를 import 하지 않는다)
STATUS_NO_FIX = -1     # 위성 못 잡음
STATUS_FIX = 0         # 일반 fix
STATUS_SBAS_FIX = 1
STATUS_GBAS_FIX = 2

# (0,0) 은 대서양 한복판이다. 드라이버가 fix 없을 때 그대로 내보내는 값이라
# status 를 신뢰할 수 없는 구현을 만나도 한 번 더 걸러낸다.
NULL_ISLAND_EPS = 1e-7


def fix_is_usable(status, lat, lon):
    """이 fix 로 항법해도 되는가.

    Returns True 인 조건 (전부 만족):
      · status >= 0            — NO_FIX(-1) 거부
      · lat/lon 이 유한한 수   — NaN/inf 거부
      · (0, 0) 이 아님         — 드라이버 기본값(Null Island) 거부
      · 위도 |lat| <= 90, 경도 |lon| <= 180
    """
    try:
        status = int(status)
        lat = float(lat)
        lon = float(lon)
    except (TypeError, ValueError):
        return False

    if status < STATUS_FIX:
        return False
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return False
    if abs(lat) <= NULL_ISLAND_EPS and abs(lon) <= NULL_ISLAND_EPS:
        return False
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return False
    return True


# ───────────────────────── fix 신선도 (계약서_GPS묶음_20261029 C1) ─────────────────────────
# 🚨 왜 필요한가: have_fix 를 False 로 바꾸는 곳은 gps_cb 의 NO_FIX 분기 하나뿐이다.
#    /fix 가 아예 끊기면(ublox 사망·USB 분리) 콜백이 안 불리고, GPSFilter 가 계속 거부해도
#    return 만 한다 → have_fix 가 True 로 남아 **옛 위치로** 방위·거리·geofence 를 계속 낸다.
#    → 마지막으로 '받아들인' 프레임의 monotonic 시각을 보고, 묵었으면 입을 다문다.
# 2.5 s = 1 Hz 기준 한 프레임 빠짐(2.0 s)은 견디고 두 프레임 빠짐(3.0 s)부터 stale.
# 노드 코드 기본값과 config/north_goal_angle.yaml 의 gps_stale_sec 는 이 값과 같아야 한다
# (대회 launch 가 yaml 을 못 실어도 같은 값 — test_gps_fresh.py 가 검사).
DEFAULT_GPS_STALE_SEC = 2.5


def fix_is_fresh(last_fix_t, now, stale_sec):
    """마지막으로 받아들인 fix 가 아직 믿을 만큼 새것인가.

    last_fix_t: GPSFilter 가 받아들인 마지막 프레임의 monotonic 시각. None = 한 번도 없음.
    now:        지금 monotonic 시각.
    stale_sec:  이보다 오래되면 stale (경계 age == stale_sec 는 fresh).

    None·숫자 아님·NaN 이면 False — 모르면 입을 다문다.
    """
    if last_fix_t is None:
        return False
    try:
        age = float(now) - float(last_fix_t)
        stale_sec = float(stale_sec)
    except (TypeError, ValueError):
        return False
    if not (math.isfinite(age) and math.isfinite(stale_sec)):
        return False
    return age <= stale_sec


def sanitize_stale_sec(value, default=DEFAULT_GPS_STALE_SEC):
    """gps_stale_sec 파라미터 검증 → (쓸 값, 정상 여부).

    유한한 양수만 정상. inf(검사 끔)·0 이하·NaN·숫자 아님(문자열·bool·None)이면
    (default, False) — 노드는 세우지 않고 시작 ERROR 를 낸 뒤 기본값으로 돈다(Davidson 10-29).

    🚨 int 를 받아 주는 분기는 순수 함수 호출용이다. 노드에선 정수(0, 3)가 여기 오지 못한다 —
       DOUBLE 로 선언한 파라미터에 정수를 주면 Humble rclpy 가 InvalidParameterTypeException 으로
       노드를 세운다. 실제 yaml 값은 반드시 소수(.0)로 적는다.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return float(default), False
    v = float(value)
    if not math.isfinite(v) or v <= 0.0:
        return float(default), False
    return v, True

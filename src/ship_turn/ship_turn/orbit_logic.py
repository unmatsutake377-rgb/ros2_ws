"""ship_turn 의 orbit 제어 로직 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

    from ship_turn.orbit_logic import orbit_steer, OrbitProgress, orbit_direction_cw, is_active

부표를 중심으로 원을 그리며 도는 orbit 기동. 제어법·진행각 누적은 틀리기 쉬워서
노드에서 떼어내 단위 테스트한다. 활성 판정(is_active)도 여기 — 부팅 센티널 -1 과
enabled 스위치가 얽히는 자리라 노드 밖에서 표로 검증한다.

각도 규약: 상대각(0 = 정면, **+ = 좌현/왼쪽 양수**). /image_angle·/candidate_angle·LiDAR a−80 과 같다.
  🚨 [2026-11-04 정정] 종전엔 "+ = 우현" 으로 쓰여 있었다. 입력(/image_angle)·출력(/candidate_angle)은
  둘 다 좌 양수라서 CW(빨강·초록) 요청에 CCW 로 돌았다(계약서_ship_turn선회_20261104 §0-3).
시계방향(CW) = 위에서 볼 때 시계. 나침반 yaw 는 CW 양수(북 0, 동 90).
"""


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def clamp_pm180(a):
    return (a + 180.0) % 360.0 - 180.0


def wrap360(a):
    return (a % 360.0 + 360.0) % 360.0


def is_active(enabled, wp_mode, active_wp_mode):
    """이 노드가 /candidate_angle 을 낼 차례인가.

    enabled        : 파라미터 `enabled`. False 면 어떤 모드에서도 침묵
                     (launch search_behavior:=hold — 탐색은 ship_back 담당).
    wp_mode        : 마지막으로 받은 /wp_mode. 부팅 초기값 -1 은 어떤 모드와도 안 맞는다.
    active_wp_mode : 파라미터(기본 3). -1 같은 센티널로 끄지 마라 — 부팅값과 일치해
                     첫 /wp_mode 수신 전에 SPIN 을 내보낸다. 끄려면 enabled=False.
    """
    return bool(enabled) and int(wp_mode) == int(active_wp_mode)


def orbit_direction_cw(color):
    """색 → orbit 방향. → True(CW) / False(CCW) / None(모름).

    2026-11 경기규정: 빨강·초록 = 우현선회(시계, CW) / **노랑 = 좌현선회(반시계, CCW)**.
    흰색 = 반시계는 2025 규정 값 — 2026 에 없는 색이지만 충돌이 없어 남긴다(계약서_탐색선회_부표색_20261111 C2).
    검정은 위치유지 전용이라 방향 없음(None) — 검출기 turn_color_logic.NO_ORBIT_DIRECTION 과 맞출 것.
    """
    c = (color or "").strip().lower()
    if c in ("red", "green"):
        return True
    if c in ("yellow", "white"):
        return False
    return None


def orbit_source(state, cam_rel, lidar_hit):
    """이번 틱 부표 방위를 어디서 쓸지 → "lidar" / "cam" / None. 순수 함수(계약서_탐색orbit_WP넘김_20261111 C).

    state     : ship_turn FSM 상태 문자열
    cam_rel   : 신선한 카메라 방위(/image_angle) 또는 None
    lidar_hit : lidar_track 결과 (rel, dist) 또는 None — 노드가 ORBIT 에선 늘, APPROACH 에선 카메라가 없을 때만 구한다

    ORBIT 에선 **LiDAR 먼저**: turn 검출기는 부표를 놓친 뒤 grace_period_s(2 s) 동안 마지막 각(시야 끝 ±32°)을
    그대로 다시 낸다. 카메라를 먼저 믿으면 그 2 s 동안 LiDAR 추적이 안 켜지고, 끝나면 부표는 이미 추적 창
    밖이라 놓친다 — 전체 코스 시뮬에서 ORBIT 이 61–73° 에서 끊겼다(11-11).
    APPROACH 는 지금처럼 카메라 먼저(색으로 확인한 방위), 카메라가 없을 때만 LiDAR.
    """
    if state == "ORBIT" and lidar_hit is not None:
        return "lidar"
    if cam_rel is not None:
        return "cam"
    if state in ("APPROACH", "ORBIT") and lidar_hit is not None:
        return "lidar"
    return None


def orbit_expected_rel(direction_cw):
    """설계상 orbit 자세에서 부표가 있어야 할 상대각(좌 양수). CW → 우현 −90, CCW → 좌현 +90."""
    return -90.0 if direction_cw else 90.0


def orbit_steer(buoy_rel, dist, orbit_radius, direction_cw, k_radial, radial_limit):
    """orbit 조향각(상대, 0=정면, +=좌현)을 낸다. 순수 함수.

    buoy_rel     : 부표 상대각(deg, 0=정면, **+=좌현**)
    dist         : 부표까지 거리(m, LiDAR)
    direction_cw : True=시계, False=반시계

    ★ orbit 원리 (좌 양수 프레임):
      · 시계(CW)로 돌려면 부표(원 중심)를 **우현(−90°)**에 둔다. 진행 방향(접선) = buoy_rel + 90.
        (반시계는 부표를 좌현 +90°, 접선 = buoy_rel − 90)   → tangent = buoy_rel + 90·s, s=+1 CW
      · 반경 보정: 너무 멀면(dist>radius) 부표 쪽으로, 너무 가까우면 반대로 튼다.
        접선에서 s·gain 을 **뺀다** — CW 면 부표가 우현(음수 쪽)이라 gain>0 이면 음수(우) 쪽 = 원 안쪽.
    수치: rel 0·dist 4·R 3·CW → +60(좌조향 → 부표가 우현으로 → CW). rel −90·dist 3·CW → 0(접선).
    """
    s = 1.0 if direction_cw else -1.0
    tangent = clamp_pm180(buoy_rel + 90.0 * s)
    err = dist - orbit_radius                      # >0 너무 멀다 → 안쪽으로
    gain = clamp(k_radial * err, -radial_limit, radial_limit)
    return clamp_pm180(tangent - s * gain)


def buoy_world_bearing(yaw_deg, rel_deg):
    """부표의 월드(나침반) 방위 = yaw − rel. yaw 는 CW 양수, rel 은 좌(CCW) 양수라 부호가 반대다.

    🚨 [2026-11-04 정정] 종전 노드는 yaw + rel 로 합성했다. 정상 선회(rel≈일정)에선 Δ가 같아 티가 안 났지만
    제자리 요잉(부표 고정, rel = yaw)에선 진행각이 2배로 잡혀 조기 COOLDOWN 이 났다.
    """
    return wrap360(yaw_deg - rel_deg)


class OrbitProgress:
    """부표의 **월드 방위**(buoy_world_bearing = yaw − rel) 변화를 누적해 orbit 진행각(deg)을 잰다.

    배가 부표를 CW 로 돌면 '부표를 향한 월드 방위'가 증가하고, CCW 면 감소한다.
    누적 절대값이 orbit_target_deg 에 닿으면 한 바퀴 완료.
    (yaw 가 없으면 진행각을 못 잰다 → 노드가 orbit_max_sec 안전 타임아웃으로 마무리한다.)
    """

    def __init__(self):
        self.prev = None
        self.total = 0.0        # 부호 있는 누적 (CW +, CCW -)

    def reset(self):
        self.prev = None
        self.total = 0.0

    def update(self, world_bearing_deg):
        """월드 방위(0~360)를 넣으면 진행각(deg, 절대값)을 반환."""
        if self.prev is None:
            self.prev = world_bearing_deg
            return 0.0
        d = clamp_pm180(world_bearing_deg - self.prev)   # 부호 있는 증분 [-180,180]
        self.prev = world_bearing_deg
        self.total += d
        return abs(self.total)

    def progress_deg(self):
        return abs(self.total)

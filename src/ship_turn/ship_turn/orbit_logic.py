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

import math
from collections import namedtuple


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
    **검정 = 좌현선회(반시계, CCW)** — 2026 OT(11-12). 예전엔 None(위치유지 전용)이라 SEARCH 에서 못 넘어가
    선회 0회였다(계약서_탐색검정좌현_20261112 K1). 검출기 turn_color_logic.NO_ORBIT_DIRECTION 과 맞출 것.
    """
    c = (color or "").strip().lower()
    if c in ("red", "green"):
        return True
    if c in ("yellow", "white", "black"):
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
#   / fused = fused 먼저, 맞는 물체 없으면 LiDAR. (§10 Q2) 단 fused 의 depth(D455 단독) 물체는 노드 LiDAR 가
#   못 볼 때만 채운다(계약서_흰부표_거리융합_v3 G-5 'LiDAR 우선').
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
    🚨 fused 라도 고른 물체가 src depth(D455 단독)이고 노드 LiDAR 값이 있으면 LiDAR(계약서_흰부표_거리융합_v3
       G-5 'LiDAR 우선' — depth 단독은 LiDAR 가 못 볼 때 채우기 전용). perception_fusion 은 LiDAR 가 거리에
       기여하지 않은 물체를 src fused 로 내지 않는다(P1b-1: D455 가 끼면 src depth, 카메라끼리면 src 4·5 = 이
       사본이 모르는 번호 → allowed 밖 → None → LiDAR). 그래서 LiDAR stale(flags bit1)이어도 물체는 depth 또는
       4·5 뿐이라 같은 규칙으로 덮인다. src lidar·fused(= LiDAR 기여) 는 그대로 fused 먼저.
       출처 이름은 실제로 쓴 값('lidar')을 말한다.
    """
    lid = (lidar_d, 'lidar') if lidar_d is not None else (None, None)
    fus = (fused_hit[0], fused_src_label(fused_hit[1])) if fused_hit is not None else (None, None)
    if mode == 'fill':
        return lid if lid[0] is not None else fus
    if mode == 'fused':
        depth_only = fused_hit is not None and fused_hit[1] == FUSED_SRC_IDS['depth']
        if fus[0] is not None and not (depth_only and lid[0] is not None):
            return fus
        return lid if lid[0] is not None else fus
    return lid

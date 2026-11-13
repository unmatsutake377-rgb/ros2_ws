"""ship_gate 의 판정 로직 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

    from ship_gate.gate_logic import GateCounter, gate_candidate

계수·조향 결정은 틀리기 쉬워서 노드에서 떼어내 단위 테스트한다.
"""

import math
from collections import namedtuple


def clamp_pm180(a):
    """각도를 [-180, 180) 로."""
    return (a + 180.0) % 360.0 - 180.0


def rel_to_raw_0_360(a):
    """상대각(±180) → /candidate_angle 규약(0~360, 0=정면)."""
    return (a + 360.0) % 360.0


def circular_mid(a1, a2):
    """두 상대각의 중간 (±180 wrap 안전)."""
    if abs(a1 - a2) > 180.0:
        if a1 < a2:
            a1 += 360.0
        else:
            a2 += 360.0
    return clamp_pm180((a1 + a2) / 2.0)


def _single_buoy_target(angle, color, red_on_port, offset):
    """부표 하나만 보일 때 게이트 중심 쪽으로 offset 만큼 튼 상대각.

    🚨 부호 규약(2026-11-04 계약서_ship_gate부호_카운트 로 확정, 벤치 검증 항목 아님):
       /red_angle·/green_angle 은 상대각, **왼쪽 양수**(vision_geom.angle_from_pixel "왼쪽이 +").
       /candidate_angle(≤180 = 좌)·LiDAR `a−80` 과 같은 부호. 빨강이 좌현이면 /red_angle > 0.

    red_on_port=True  → 빨강은 좌현 = **양의** 상대각. 게이트 중심은 빨강의 '우측' → angle − offset.
                        초록은 우현 = 음의 상대각. 게이트 중심은 초록의 '좌측' → angle + offset.
    red_on_port=False → 좌우 반대.
    (수정 전엔 "좌현 = 음수" 로 가정해 부호가 반대였다 → 한쪽만 보이면 부표 쪽으로 40° 더 꺾었다.
     시뮬 gate_sim: 110/160·접촉 22 → 160/160·접촉 0. test_gate_logic 의 비대칭 케이스가 되돌림을 막는다.)
    """
    if color == 'red':
        return clamp_pm180(angle - offset) if red_on_port else clamp_pm180(angle + offset)
    else:  # green
        return clamp_pm180(angle + offset) if red_on_port else clamp_pm180(angle - offset)


def gate_candidate(red_rel, green_rel, red_on_port, min_sep, max_sep, single_offset):
    """게이트 조향각을 정한다. 순수 함수.

    red_rel, green_rel: 부표 상대각(deg) 또는 None(안 보임).
    반환 (steer_rel, valid_pair):
      · steer_rel : 조향할 상대각(0=정면). None = 둘 다 안 보임 → 게이트 없음.
      · valid_pair: 빨강·초록이 '게이트로서 말이 되는' 쌍인가 (통과 카운트 arming 에 씀).

    🚨 게이트 쌍 제약 (프롬프트 6b-[3]): 화면에 '다음 게이트의 빨강' 과 '이번 게이트의 초록'이
       같이 보이면 그 둘의 중간으로 가버린다(엉뚱). 두 부표 각도차가 [min_sep, max_sep] 일 때만
       쌍으로 인정. 벗어나면 '가까운(|각| 작은) 쪽 하나'만 쓴다.
    """
    r, g = red_rel, green_rel

    if r is None and g is None:
        return None, False

    if r is not None and g is not None:
        sep = abs(clamp_pm180(g - r))
        if min_sep <= sep <= max_sep:
            return circular_mid(r, g), True            # 유효 쌍 → 중점으로
        # 쌍이 아니다 → 가까운 쪽 하나만 (정면에 가까운 = |각| 작은)
        if abs(r) <= abs(g):
            return _single_buoy_target(r, 'red', red_on_port, single_offset), False
        return _single_buoy_target(g, 'green', red_on_port, single_offset), False

    if r is not None:
        return _single_buoy_target(r, 'red', red_on_port, single_offset), False
    return _single_buoy_target(g, 'green', red_on_port, single_offset), False


class GateCounter:
    """게이트 순차 통과 카운트. **ROS 비의존.**

    규정: "다수의 게이트를 순차적으로 통과. 누락하면 패널티."
    통과 판정: 유효 게이트를 전방에서 봤고(armed), 그 부표의 |상대각| 이 pass_behind_deg 를 넘으면
              1 증가. armed 는 한 게이트당 한 번만 카운트되게 막는다.

    🚨 pass_behind_deg 는 카메라 반화각(OAK hfov 63.9/2 = 31.95°, vision.yaml; 11-04 전환 전 D455 40.16°) **안**이어야 한다.
       부표는 그 밖으로 나가면 검출기가 10000(안 보임)을 내서 각이 아예 안 들어온다 — 옛 기본값 75 는
       물리적으로 도달 불가라 /gates_passed 가 영원히 0 이었다(2026-11-04 시뮬). 그래서 "뒤로 갔다" 가
       아니라 "통과 직전(게이트 선 ~2~6 m 앞)" 에 센다. 대회값은 ship_gate.yaml(대회 launch 가 넘김,
       11-05). 이 기본 인자와 ship_gate.py 의 declare_parameter 기본값·ship_gate.yaml 세 곳을
       같이 바꿔야 한다(test_gate_logic 이 검사).

    arming(⑥): 유효 쌍이 전방(두 |각| ≤ pass_behind)에 보이면 거리 조건 없이 armed.
       (옛 gate_arm_dist_m(6 m) 조건은 폭 6 m 게이트에서 '둘 다 ≤ 35° 이면서 ≤ 6 m' arming 창이 0.9 m 로 짧고
        7 m 이상이면 없으며, 게이트 사이 4 m 에선 다음 쌍 거리(≈6 m)가 기준을 못 넘어 재-arming 이 막혔다 —
        11-04 시뮬 간격 4 m 17/40·6 m 19/40. 제거.)

    재-arming 가드(⑥): 임계가 FOV 안이면 카운트 뒤 요잉으로 같은 게이트가 다시 '전방 유효 쌍' 이 돼
       (36° → 카운트 → 25° 로 복귀 → 재-arming → 36°) 이중 카운트가 난다. 그래서 카운트 뒤엔 다음 둘 중
       하나의 틱을 한 번 지나야 다시 arming 된다:
         (i)  두 부표 모두 안 보임(buoy_abs_angles 빈 리스트) — 지난 게이트가 정횡으로 빠짐.
         (ii) 전방 유효 쌍의 LiDAR 최소거리 pair_dist 가 **카운트 시점 기준 거리 d_ref + rearm_far_margin_m**
              보다 멀다(= 새 게이트). d_ref = arming 뒤 카운트까지 잰 전방 쌍 pair_dist 의 **최소값**
              (마지막 값이 아니다: LiDAR 가 근접 부표를 놓치면 멀리서 잰 값이 남아 다음 쌍이 전부 '같은
              게이트' 로 묶이는 것 방지 — 리뷰 11-04).
              d_ref 또는 pair_dist 가 None(LiDAR 가 부표를 못 봄)이면 멀다고 취급(거리 미상이면 막지 않는다).
       요잉 복귀는 같은 부표라 거리가 d_ref 근처 → (ii) 를 못 만족한다 — LiDAR 가 부표를 볼 때만. 거리 미상이면
       재-arming 을 막지 않는다(이중 카운트 가능). (i) 만으로는 부족하다: 다음 게이트가
       카메라 거리 안이면 검출기(색별 가장 큰 블롭 1개)가 바로 다음 부표를 내서 '둘 다 안 보임' 틱이 안 생긴다
       (11-04 시뮬 재-arming 막혀 일치 1/40).
    ⚠️ 한계: 카메라만으론 게이트 사이 거리 ≤ 6 m 에서 앞 게이트 부표가 보이는 채로 다음 쌍이 비슷한 거리에
       들어와 누락 가능(11-04 시뮬 간격 9 m 40/40·8 m 40/40·6 m 35/40·4 m 21/40, 이중 카운트 0).
       LiDAR 부표 추적은 대회 후 과제.
    """

    def __init__(self, pass_behind_deg=28.0, rearm_far_margin_m=1.0):
        self.pass_behind = float(pass_behind_deg)
        self.rearm_far_margin = float(rearm_far_margin_m)   # ⚠️ 실측 필요 (요잉 복귀 때 거리 흔들림 폭)
        self.count = 0
        self._armed = False
        self._peak = 0.0            # armed 이후 본 최대 |상대각|
        self._need_gap = False      # 카운트 뒤 (i)/(ii) 틱을 아직 못 봤다 → 재-arming 금지
        self._last_pair_dist = None  # arming 뒤 잰 전방 쌍 LiDAR 거리의 최소값 (None = 아직 못 잼)
        self._d_ref = None          # 카운트 시점의 _last_pair_dist (재-arming 기준 거리)

    def _is_new_gate(self, gate_ahead, pair_dist):
        """(ii): 전방 쌍이 카운트한 게이트보다 margin 이상 멀면 새 게이트. 거리 미상이면 막지 않는다."""
        if not gate_ahead:
            return False
        if self._d_ref is None or pair_dist is None:
            return True
        return pair_dist > self._d_ref + self.rearm_far_margin

    def update(self, gate_ahead, buoy_abs_angles, pair_dist=None):
        """gate_ahead: 이번 틱에 '유효 쌍이 전방(두 |각| ≤ pass_behind)에' 보이나 (거리 조건 없음).
        buoy_abs_angles: 지금 보이는 부표들의 |상대각| 리스트 (안 보이면 빈 리스트).
        pair_dist: gate_ahead 인 쌍의 LiDAR 최소거리 [m]. None = 쌍 없음 또는 거리 미상.
        → 이 틱에 통과가 확정되면 True (그때 count 가 1 증가)."""
        if gate_ahead and pair_dist is not None:
            # 최소값 추적: LiDAR 가 근접 부표를 놓치면 멀리서 잰 값이 남아 다음 쌍이 전부 '같은 게이트' 로
            # 묶이는 것 방지(리뷰 11-04)
            self._last_pair_dist = pair_dist if self._last_pair_dist is None else min(self._last_pair_dist, pair_dist)
        if self._need_gap and (not buoy_abs_angles or self._is_new_gate(gate_ahead, pair_dist)):
            self._need_gap = False          # (i) 둘 다 사라짐 / (ii) 더 먼 새 쌍 → 지난 게이트는 끝났다
        if gate_ahead and not self._need_gap:
            self._armed = True
        if self._armed and buoy_abs_angles:
            self._peak = max(self._peak, max(buoy_abs_angles))

        if self._armed and self._peak > self.pass_behind:
            self.count += 1
            self._armed = False
            self._peak = 0.0
            self._need_gap = True
            self._d_ref = self._last_pair_dist
            self._last_pair_dist = None
            return True
        return False


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

"""ship_gate 의 판정 로직 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

    from ship_gate.gate_logic import GateCounter, gate_candidate

계수·조향 결정은 틀리기 쉬워서 노드에서 떼어내 단위 테스트한다.
"""


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

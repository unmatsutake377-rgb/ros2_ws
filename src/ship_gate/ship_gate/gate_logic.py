"""ship_gate 의 판정 로직 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

    from ship_gate.gate_logic import GateCounter, gate_candidate

계수·조향 결정은 틀리기 쉬워서 노드에서 떼어내 단위 테스트한다.
"""

import math


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

    red_on_port=True  → 빨강은 좌현(음의 상대각). 게이트 중심은 빨강의 '우측' → +offset.
                        초록은 우현 → 게이트 중심은 초록의 '좌측' → -offset.
    red_on_port=False → 좌우 반대. (⚠️ 어느 쪽이 맞는지는 벤치 검증 — red_on_port 파라미터)
    """
    if color == 'red':
        return clamp_pm180(angle + offset) if red_on_port else clamp_pm180(angle - offset)
    else:  # green
        return clamp_pm180(angle - offset) if red_on_port else clamp_pm180(angle + offset)


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
    통과 판정: 유효 게이트를 전방에서 봤고(armed), 그 부표가 ±pass_behind_deg 를 넘어
              **뒤로 가면** 1 증가. armed 는 한 게이트당 한 번만 카운트되게 막는다
              (다음 게이트를 다시 전방에서 봐야 재-arming).
    """

    def __init__(self, pass_behind_deg=75.0):
        self.pass_behind = float(pass_behind_deg)
        self.count = 0
        self._armed = False
        self._peak = 0.0        # armed 이후 본 최대 |상대각|

    def update(self, gate_ahead, buoy_abs_angles):
        """gate_ahead: 이번 틱에 '유효 게이트가 전방(±pass_behind 안)에' 보이나.
        buoy_abs_angles: 지금 보이는 부표들의 |상대각| 리스트 (안 보이면 빈 리스트).
        → 이 틱에 통과가 확정되면 True (그때 count 가 1 증가)."""
        if gate_ahead:
            self._armed = True
        if self._armed and buoy_abs_angles:
            self._peak = max(self._peak, max(buoy_abs_angles))

        if self._armed and self._peak > self.pass_behind:
            self.count += 1
            self._armed = False
            self._peak = 0.0
            return True
        return False

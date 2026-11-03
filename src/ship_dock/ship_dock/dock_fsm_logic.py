"""ship_dock 의 APPROACH 판단 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

    from ship_dock.dock_fsm_logic import approach_decision, check_realign_min

APPROACH 상태에서 '접안 / 재정렬(ALIGN) / 조향 접근' 을 고르는 판단만 뗐다.
타임아웃·표식 유실·거리 미상(None→STOP) 분기는 노드(ship_dock.py timer_cb)에 그대로 있다 —
이 함수는 **dist 가 숫자일 때만** 부른다(None 을 넣지 않는다).

왜 뗐나: dock_sim 500 런 중 실패 3 런이 전부 접안 직전(약 1.0 m)에 APPROACH→ALIGN 제자리 선회
→ 관성 전진 → 표식 유실 → SEARCH 였다. 재정렬 전이에 거리 하한(realign_min_dist)을 넣으면서
판단을 ROS 없이 테스트할 수 있게 했다(test/test_dock_fsm_logic.py).
"""

CONTACT = "CONTACT"                 # 접안 판정 (dist <= contact)
ALIGN_STANDOFF = "ALIGN_STANDOFF"   # standoff 안에서 1회 정밀 재정렬
ALIGN = "ALIGN"                     # 크게 틀어짐(realign_tol) → 재정렬
STEER = "STEER"                     # 표식 방위로 조향하며 전진


def approach_decision(dist, angle, standoff_aligned,
                      contact_dist, standoff_dist, realign_min_dist, align_tol, realign_tol):
    """APPROACH 한 틱의 판단. 순수 함수.

    dist             : LiDAR 전방 최소거리(m). **숫자여야 한다** (None 은 노드가 먼저 걸러 STOP).
    angle            : 표식 상대 방위(도, 0=정면). 표식이 유효할 때만 부른다.
    standoff_aligned : standoff 1회 재정렬을 이미 했는가 (노드 상태 플래그).
    contact_dist     : 접안 판정 거리(m).
    standoff_dist    : 1회 정밀 재정렬을 하는 지점(m).
    realign_min_dist : 재정렬 하한(m). **이 거리 안(dist < 하한)에서는 어떤 재정렬도 안 한다.**
    align_tol        : '정면' 허용 각(도). standoff 재정렬 조건.
    realign_tol      : 접근 중 이만큼 틀어지면 재정렬(도).

    → (decision, standoff_aligned_new)
      decision ∈ {CONTACT, ALIGN_STANDOFF, ALIGN, STEER}
      standoff_aligned_new : ALIGN_STANDOFF 일 때만 True 로 바뀐다(1회 보장). 나머지는 입력 그대로.

    순서(바꾸지 말 것):
      1. dist <= contact → CONTACT (맨 앞. 하한보다 먼저라, 하한 <= contact 면 하한은 영구 무효 → check_realign_min)
      2. dist < realign_min → STEER (하한 안: 제자리 선회하면 관성으로 카메라 최소거리 아래로 들어가 표식을 잃는다.
         1.5 m 에서 25° 는 좌우 0.7 m 라 선회로는 좌우 오차를 못 줄이고, 조향 접근이 낫다)
      3. standoff 1회 재정렬 (standoff_aligned 가 False 이고 dist <= standoff 이고 |angle| > align_tol)
      4. |angle| > realign_tol → ALIGN
      5. STEER
    경계: dist == realign_min 은 하한 **밖**(재정렬 허용). dist == standoff 는 standoff **안**.
    """
    if dist <= contact_dist:
        return CONTACT, standoff_aligned

    if dist < realign_min_dist:
        return STEER, standoff_aligned

    if (not standoff_aligned) and dist <= standoff_dist and abs(angle) > align_tol:
        return ALIGN_STANDOFF, True

    if abs(angle) > realign_tol:
        return ALIGN, standoff_aligned

    return STEER, standoff_aligned


def check_realign_min(realign_min_dist, contact_dist, standoff_dist):
    """부팅 때 재정렬 하한 설정이 말이 되는지. 순수 함수. 침묵하는 무효 설정을 드러낸다(CLAUDE.md 1-6).

    → 'below_contact'  : 하한 <= contact. approach_decision 이 contact 를 먼저 보므로 하한 안 구간이 없다
                         = 하한이 **영구 무효**, 접안 직전 제자리 선회가 그대로 남는다 → 노드는 ERROR.
      'above_standoff' : 하한 >= standoff. standoff 안이 전부 하한 안이라 standoff 1회 재정렬도,
                         standoff 안 realign_tol 재정렬도 영영 안 걸린다 → 노드는 WARN.
      'ok'             : contact < 하한 < standoff.
    """
    if realign_min_dist <= contact_dist:
        return "below_contact"
    if realign_min_dist >= standoff_dist:
        return "above_standoff"
    return "ok"

"""ship_dock 의 상태 판정 — **ROS 비의존 순수 파이썬** (rclpy 임포트 없음).

    from ship_dock.dock_fsm_logic import (
        approach_decision, check_realign_min, approach_gate, contact_step, backoff_step,
        median_min, median_kernel_window, MEDIAN_MIN_VALID)

APPROACH 의 '접안 / 재정렬(ALIGN) / 조향 접근' 판단(approach_decision)과, LiDAR 전방 거리가
None 일 때 APPROACH·CONTACT·BACKOFF 가 무엇을 하는지(approach_gate·contact_step·backoff_step)를
뗐다. 시계(elapsed)·발행·로그는 노드(ship_dock.py timer_cb)에 있다.
approach_decision 은 **dist 가 숫자일 때만** 부른다(None 을 넣지 않는다 — approach_gate 가 먼저 거른다).

거리 None 규약 (docs/문제와작업/계약서_도킹거리None_20261106.md, 11-06 승인):
  · APPROACH: 정지(STOP_HOLD)하되 approach_timeout 은 흐른다(→BACKOFF). 표식 유실→SEARCH 는 안 본다.
  · CONTACT : 거리와 무관하게 정지 유지, contact_hold 가 지나면 BACKOFF.
  · BACKOFF : /scan 이 신선한데 전방 반환이 없으면(벽 밀착 추정) 거리 없이 후진,
              /scan 이 없거나 stale 이면 정지(뒤를 못 본다). 타임아웃 시계는 둘 다 흐른다.
  이전엔 세 상태 모두 None 이면 정지만 하고 전이 평가를 통째로 막아 CONTACT 3 s·BACKOFF 8 s 가
  거리 복귀까지 멈췄다(docs/문제와작업/시뮬3자유도_발견_20261106.md 발견 1).

왜 뗐나: dock_sim 500 런 중 실패 3 런이 전부 접안 직전(약 1.0 m)에 APPROACH→ALIGN 제자리 선회
→ 관성 전진 → 표식 유실 → SEARCH 였다. 재정렬 전이에 거리 하한(realign_min_dist)을 넣으면서
판단을 ROS 없이 테스트할 수 있게 했다(test/test_dock_fsm_logic.py).

전방 거리 공간 median (docs/문제와작업/계약서_도킹물보라필터_20261109.md, 11-09 승인):
  · median_min 은 ship_direction/failsafe.py median_min 의 **사본**이다(ship_direction import 금지 —
    package.xml 의존 추가·tools/sim 단독 실행이 깨진다). 원본을 고치면 여기도 같이 고친다.
    어긋남은 test/test_dock_median_min.py 가 원본을 경로 로드해 무작위 입력 수천 건으로 잡는다.
  · 노드는 front_min_dist 에서 median_min(kernel=front_median_kernel, min_valid=MEDIAN_MIN_VALID)을 부른다.
"""

import math

CONTACT = "CONTACT"                 # 접안 판정 (dist <= contact)
ALIGN_STANDOFF = "ALIGN_STANDOFF"   # standoff 안에서 1회 정밀 재정렬
ALIGN = "ALIGN"                     # 크게 틀어짐(realign_tol) → 재정렬
STEER = "STEER"                     # 표식 방위로 조향하며 전진


def approach_decision(dist, angle, standoff_aligned,
                      contact_dist, standoff_dist, realign_min_dist, align_tol, realign_tol):
    """APPROACH 한 틱의 판단. 순수 함수.

    dist             : LiDAR 전방 최소거리(m). **숫자여야 한다** (None 은 approach_gate 가 먼저 거른다).
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


# ─────────────── 거리 None 처리 (11-06) — 세 상태의 한 틱 판정 ───────────────
SEARCH = "SEARCH"                   # approach_gate: 표식 유실 → 재탐색 (거리를 알 때만)
BACKOFF = "BACKOFF"                 # approach_gate·contact_step: 시간이 다 됨 → 후진
STOP_BLIND = "STOP_BLIND"           # approach_gate: 거리 None → 정지 (시계는 흐른다)
DECIDE = "DECIDE"                   # approach_gate: 거리·표식 있음 → approach_decision 으로
HOLD = "HOLD"                       # contact_step: 접안 유지
DONE_BACKOFF = "DONE_BACKOFF"       # backoff_step: 이탈 거리 도달 → DONE(20000)
DONE_TIMEOUT = "DONE_TIMEOUT"       # backoff_step: 후진 타임아웃 → DONE(50000 유지)
REVERSE = "REVERSE"                 # backoff_step: 후진(180) 계속
STOP_NO_SCAN = "STOP_NO_SCAN"       # backoff_step: /scan 없음·stale → 뒤를 못 본다, 정지


def approach_gate(dist_known, has_marker, elapsed, timeout_sec):
    """APPROACH 한 틱의 앞단 — approach_decision 을 부르기 전에 무엇을 할지. 순수 함수.

    dist_known  : LiDAR 전방 거리가 숫자인가(None 이 아닌가).
    has_marker  : 표식이 유효(센티널 아님·신선)한가.
    elapsed     : APPROACH 에 머문 시간(s).  timeout_sec : approach_timeout_sec.

    순서(결정 ①, 바꾸지 말 것):
      1. 거리를 알고 표식이 없음 → SEARCH   (거리 None 동안은 표식 유실을 안 본다 = 11-06 이전 동작 유지)
      2. elapsed > timeout → BACKOFF        (거리 None 이어도 흐른다 — 이전엔 None 가드가 이것도 막았다)
      3. 거리 None → STOP_BLIND             (눈 감고 접근하지 않는다 — 정지)
      4. DECIDE                             (approach_decision 으로)
    경계: elapsed == timeout 은 아직 타임아웃 아님(>, 현행과 같다).
    """
    if dist_known and not has_marker:
        return SEARCH
    if elapsed > timeout_sec:
        return BACKOFF
    if not dist_known:
        return STOP_BLIND
    return DECIDE


def contact_step(elapsed, hold_sec):
    """CONTACT 한 틱. 거리와 무관 — 접안은 이미 판정됐고 여기선 시간만 센다. 순수 함수.

    → BACKOFF (elapsed >= hold_sec) | HOLD.  경계: elapsed == hold_sec 은 BACKOFF(>=, 현행과 같다).
    """
    return BACKOFF if elapsed >= hold_sec else HOLD


def backoff_step(dist, scan_fresh, elapsed, backoff_dist, timeout_sec):
    """BACKOFF 한 틱. 순수 함수.

    dist        : LiDAR 전방 최소거리(m) 또는 None(전방에 유한 반환 없음 / 스캔 없음·stale).
    scan_fresh  : /scan 이 scan_stale_sec 안에 왔는가(front_min_dist 의 stale 기준과 같은 값).
    elapsed     : BACKOFF 에 머문 시간(s).

    순서(바꾸지 말 것):
      1. dist 유효·>= backoff_dist → DONE_BACKOFF   (이탈 확인. 시간보다 먼저)
      2. elapsed > timeout_sec → DONE_TIMEOUT      (이탈 미확인 → 노드는 50000 유지. None 이어도 흐른다)
      3. dist 유효 또는 scan_fresh → REVERSE       (None + 신선 = 전방에 유한 반환 없음 = 벽 밀착 추정, 결정 ②)
      4. STOP_NO_SCAN                              (None + /scan 없음·stale = LiDAR 부재 → 뒤를 못 보니 정지)
    경계: dist == backoff_dist 는 DONE_BACKOFF(>=), elapsed == timeout 은 아직 아님(>) — 현행과 같다.
    """
    if dist is not None and dist >= backoff_dist:
        return DONE_BACKOFF
    if elapsed > timeout_sec:
        return DONE_TIMEOUT
    if dist is not None or scan_fresh:
        return REVERSE
    return STOP_NO_SCAN


# ─────────────── 전방 거리 공간 median (11-09) — 물보라 한 점의 거짓 CONTACT 방지 ───────────────
MEDIAN_MIN_VALID = 3                # 창 안 유효점이 이보다 적으면 그 인덱스는 고립 반사로 버린다(결정 2, 고정)
MEDIAN_KERNEL_MAX = 9               # front_median_kernel 허용 상한(결정 3). 0·1 = raw, 2~9 = median


def median_kernel_window(kernel):
    """front_median_kernel → 실효 창 점 수. 순수 함수. 부팅 검증·로그용.

    → 1    : kernel 0·1 (필터 없음 = raw 최소, 옛 동작)
      2h+1 : kernel 2~9 (h = kernel // 2 → 짝수 k 는 창 k+1. median_min 이 ±(k//2) 를 보기 때문)
      None : 정수가 아니거나 음수·MEDIAN_KERNEL_MAX 초과 (노드는 ERROR + 기본값 대체)
    """
    if isinstance(kernel, bool) or not isinstance(kernel, int):
        return None
    if kernel < 0 or kernel > MEDIAN_KERNEL_MAX:
        return None
    if kernel <= 1:
        return 1
    return 2 * (kernel // 2) + 1


def median_min(ranges, lo, hi, kernel=5, min_valid=3, return_rejected=False):
    """[lo,hi] 구간에 **공간 median** 을 걸고 최소거리를 찾는다. → (거리, 인덱스) 또는 (None, None).

    ★ ship_direction/failsafe.py median_min 의 사본 — 의미를 바꾸지 말 것(동일성 테스트가 잡는다).

    return_rejected=True 면 (거리, 인덱스, 기각수). 기각수 = raw 로는 유효한 점(finite, >0)인데
      median 창의 유효점이 min_valid 미만이라 고립 스파이크로 버려진 인덱스 개수(로그 전용).

    각 인덱스의 ±(kernel//2) 창에서 유효점(finite, >0)의 median 을 그 인덱스 값으로 쓴다.
    유효점이 min_valid 미만이면 그 인덱스는 **무시**한다 (주변에 아무것도 없는 고립 반사 = 물보라).
    kernel 이 0/1 이면 필터 없이 raw min (폴백).
    ※ 짝수 개일 때 위쪽 median 을 쓴다 → 거리가 더 크게 나온다(접안 판정이 물보라 쪽으로 안 기운다).
    """
    n = len(ranges)
    lo = max(0, lo)
    hi = min(n - 1, hi)
    if lo > hi:
        return (None, None, 0) if return_rejected else (None, None)

    half = (kernel // 2) if (kernel and kernel > 1) else 0

    best_d, best_i = None, None
    rejected = 0
    for i in range(lo, hi + 1):
        if half == 0:
            r = ranges[i]
            v = r if (math.isfinite(r) and r > 0.0) else None
        else:
            w = []
            for j in range(max(0, i - half), min(n - 1, i + half) + 1):
                r = ranges[j]
                if math.isfinite(r) and r > 0.0:
                    w.append(r)
            if len(w) < min_valid:
                ri = ranges[i]
                if math.isfinite(ri) and ri > 0.0:
                    rejected += 1
                continue
            w.sort()
            v = w[len(w) // 2]           # median (짝수면 위쪽)
        if v is None:
            continue
        if best_d is None or v < best_d:
            best_d, best_i = v, i
    return (best_d, best_i, rejected) if return_rejected else (best_d, best_i)

"""ship_direction 의 판정 로직 — **ROS 비의존 순수 파이썬.**

여기엔 rclpy 를 임포트하지 않는다. 그래야 ROS 없이도 단위 테스트할 수 있다:

    from ship_direction.failsafe import SensorWatch, TemporalVote, median_min
    from ship_direction.failsafe import effective_candidate, candidate_is_stale, CandidateStaleLog
    from ship_direction.failsafe import heading_missing

시각(now)은 호출자가 주입한다 → 테스트에서 시간을 마음대로 돌릴 수 있다.
"""

import math
from collections import deque


def median_min(ranges, lo, hi, kernel=5, min_valid=3, return_rejected=False):
    """[lo,hi] 구간에 **공간 median** 을 걸고 최소거리를 찾는다. → (거리, 인덱스) 또는 (None, None).

    return_rejected=True 면 (거리, 인덱스, 기각수) 를 돌려준다 (D6, 관측 전용).
      기각수 = raw 로는 유효한 점(finite, >0)인데 median 창의 유효점이 min_valid 미만이라
      고립 스파이크로 버려진 인덱스 개수. **물보라 노이즈의 대리 지표**다.
      제어에는 쓰지 않는다 — blackbox 가 CSV 에 기록해 '노이즈 수 ↔ 기상 열화' 상관 검증용.
      ⚠️ 기본값 False 로 두어 기존 호출부·테스트는 (거리, 인덱스) 2-튜플 그대로 받는다.

    ★ 왜 필요한가 — **감속 신호가 물보라 한 점에 속는다:**
      _closest_obstacle 이 필터 없는 raw min 이라, 물보라 반사 하나가 0.3m 로 튀면
      /obstacle_distance_array[0] = 0.3 → motor_control 이 그대로 감속한다.
      측정(시드 20, 물보라 30%): raw-min 가짜감속 **36.8% → median 8.9%**(기준선). 접촉은 전 조건 0.

    ★ 이건 **공간** 필터다 — 폐기한 TemporalVote(시간 투표)와 메커니즘이 다르다.
      한 프레임 안에서 이웃과 어긋나는 고립 스파이크를 range 값 수준에서 지운다.

    각 인덱스의 ±(kernel//2) 창에서 유효점(finite, >0)의 median 을 그 인덱스 값으로 쓴다.
    유효점이 min_valid 미만이면 그 인덱스는 **무시**한다 (주변에 아무것도 없는 고립 반사 = 물보라).
    kernel 이 0/1 이면 필터 없이 raw min (폴백).

    ※ 짝수 개일 때 위쪽 median 을 쓴다 → 거리가 더 크게 나와 '가짜 감속' 쪽으로 안 기운다.
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
                # 고립 스파이크 → 무시. raw 로는 유효했던 점이면 '기각'으로 센다(물보라 지표).
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


class SensorWatch:
    """센서 신선도 → 페일세이프 레벨.

    레벨: 0 정상 / 1 경고(warn_sec) / 2 정지(stop_sec)

    규칙 (CLAUDE.md §5 — 팀 최우선 우려는 '고장이 아닌데 스스로 멈추는 것'):
      · 센서별 ARMED — 첫 데이터를 받은 뒤에만 감시한다 (부팅 오발동 방지)
      · **가장 나쁜 센서**가 레벨을 정한다.
        (/scan 만 보면 IMU 사망을 놓친다: IMU 가 죽으면 ship_goal_angle 이 /yaw_error 를
         끊어주는데, 그 침묵을 들을 귀가 없으면 마지막 yaw_error 로 영원히 조향한다.)
      · 올릴 땐 연속 confirm_n 회 확인 (순간 지터로 레벨 안 올림)
      · 내릴 땐 즉시 (자동 복구 — 센서가 돌아왔다는 뜻)
      · 하드 폴트(set_fault)는 지터가 아니므로 확인 없이 즉시 레벨 2
    """

    def __init__(self, sensors, warn_sec=0.7, stop_sec=3.0, confirm_n=3):
        self.warn_sec = float(warn_sec)
        self.stop_sec = float(stop_sec)
        self.confirm_n = int(confirm_n)
        self.last_seen = {name: None for name in sensors}   # None = ARMED 전
        self.level = 0
        self.worst = None
        self._raise_count = 0
        self._fault = False

    def feed(self, name, now):
        """센서 데이터 수신 기록 (ARMED)."""
        self.last_seen[name] = now

    def set_fault(self, fault):
        """하드 폴트(예: angle_increment 이상 스캔). 확인 N회 없이 즉시 레벨 2."""
        self._fault = bool(fault)

    @property
    def fault(self):
        return self._fault

    def raw_level(self, now):
        """확인/히스테리시스 적용 **전**의 순간 레벨. → (level, worst_name, worst_age)"""
        if self._fault:
            return 2, 'fault', 0.0

        raw, worst, worst_age = 0, None, 0.0
        for name, t in self.last_seen.items():
            if t is None:
                continue                      # ARMED 전 — 이 센서는 아직 감시하지 않는다
            age = now - t
            if age > self.stop_sec:
                lvl = 2
            elif age > self.warn_sec:
                lvl = 1
            else:
                lvl = 0
            if lvl > raw:                     # 가장 나쁜 센서가 레벨을 정한다
                raw, worst, worst_age = lvl, name, age
        return raw, worst, worst_age

    def update(self, now):
        """레벨을 갱신하고 반환한다."""
        raw, worst, worst_age = self.raw_level(now)

        if self._fault:
            self.level = raw                  # 하드 폴트 → 즉시
            self._raise_count = 0
        elif raw > self.level:
            self._raise_count += 1            # 올릴 땐 연속 N회 확인
            if self._raise_count >= self.confirm_n:
                self.level = raw
                self._raise_count = 0
        elif raw < self.level:
            self.level = raw                  # 자동 복구 → 즉시
            self._raise_count = 0
        else:
            self._raise_count = 0

        self.worst = (f"{worst}({worst_age:.1f}s)"
                      if worst and worst != 'fault' else worst)
        return self.level


class TemporalVote:
    """최근 N 프레임의 이진 장애물 마스크를 투표해, 한 프레임짜리 오탐(물보라)을 지운다.

    최근 frames(기본 3) 프레임 중 votes(기본 2) 회 이상 나타난 셀만 진짜 장애물로 인정한다.
    LiDAR 10Hz 기준 0.3초짜리 기억 → 반응 지연은 무시할 수준.

    ★ 반드시 **dilate(팽창) 전, 원본 마스크**에 걸어야 한다.
      팽창 후에 걸면 진짜 부표의 팽창 영역까지 표가 갈려 부표가 얇아진다.

    측정(정면 부표 1개, 25판, 물보라 60%): 최소여유 0.17m(없음) → 0.40m(있음). 2.4배.

    frames<=1 또는 votes<=1 이면 꺼진다(그대로 통과).
    """

    def __init__(self, frames=3, votes=2):
        self.frames = int(frames)
        self.votes = int(votes)
        self.history = deque(maxlen=max(1, self.frames))

    @property
    def enabled(self):
        return self.frames > 1 and self.votes > 1

    def reset(self):
        self.history.clear()

    def apply(self, binary):
        if not self.enabled:
            return binary

        self.history.append(list(binary))

        n = len(binary)
        out = [0] * n
        for i in range(n):
            v = 0
            for h in self.history:
                if i < len(h) and h[i] == 1:   # 스캔 길이가 바뀌어도 안전
                    v += 1
            if v >= self.votes:
                out[i] = 1
        return out


def candidate_is_stale(candidate_t, now, stale_sec):
    """수신 후 stale_sec **초과**로 갱신 없음이면 True. 수신 기록 없음(None)은 stale 이 아니다(모름).
    로그 판정(노드)과 값 대체(effective_candidate)가 같은 기준을 쓰게 한 곳에 둔다."""
    return candidate_t is not None and (now - candidate_t) > stale_sec


def effective_candidate(candidate, candidate_t, now, stale_sec, invalid):
    """control_cb 가 _compute 에 넘길 candidate (계약서_전체코스_20261104 §2-6·§10-3).

    · 수신 기록 없음(candidate_t None) → invalid(20000). 부팅 초기값·wp_mode 변경 초기화와 같다.
    · 수신 후 stale_sec 넘게 갱신 없음 → invalid(20000) = GPS 주행 + LiDAR 회피.
      발행 노드가 죽으면 묵은 값(조향각·SPIN·STOP_HOLD)을 모드가 바뀔 때까지 붙들고 있던 결함.
      발행 주기 0.1 s(미션 노드)·0.5 s(north) → 1.0 s 임계면 정상 발행에선 안 걸린다.
    · 그 밖엔 받은 값 그대로. 180(후진)의 0.5 s 규칙은 _compute 가 따로 본다
      → 0.5~stale_sec 는 _compute 가 STOP_HOLD, stale_sec 를 넘으면 여기서 invalid.
    경계: age == stale_sec 는 신선(초과일 때만 stale). invalid 는 호출자가 넘긴다(상수 공유 안 함).
    """
    if candidate_t is None or candidate_is_stale(candidate_t, now, stale_sec):
        return invalid
    return candidate


def heading_missing(candidate, yaw_error, invalid):
    """갭 선택(20000)인데 목표 방위가 없다 → True (계약서_yaw미수신정지_20261105, 안 A).

    · candidate == invalid(20000) 이고 yaw_error 를 부팅 뒤 한 번도 못 받음(None) → True = STOP_HOLD.
      (C) 갭 선택은 yaw_error 를 목표 방위로 쓰는데, 예전엔 초기값 0.0 을 '목표가 정면'으로 읽어
      뱃머리 방향으로 갭을 골랐다. 모르면 입을 다문다.
    · 🚨 판정은 반드시 `is None`. 0.0 은 '정확히 정렬'인 정상값 — `not yaw_error` 면 정렬된 순간 영구 정지.
    · 그 밖의 candidate(각도·5000·6000·180·50000)는 yaw_error 를 안 읽는다 → False(거짓 정지 없음).
    · 받은 뒤 끊김은 여기 몫이 아니다 — SensorWatch(ARMED 뒤 stale → 레벨 2)가 모든 모드를 멈춘다.
    invalid 는 호출자가 넘긴다(상수 공유 안 함).
    """
    return candidate == invalid and yaw_error is None


class CandidateStaleLog:
    """candidate stale 로그 시점 판정 — ROS 비의존. update() 가 낼 로그 종류를 돌려준다.

      'enter'  : 신선 → stale 로 바뀐 틱 (ERROR 1회)
      'repeat' : stale 지속 중 마지막 ERROR 뒤 period_sec 이상 지남 (주기 ERROR)
      'recover': stale → 신선/기록 없음(새 수신 또는 wp_mode 변경 초기화) (INFO 1회)
      None     : 로그 없음
    호출 지점은 노드에서 심각도별로 나눈다(CLAUDE.md §1-7, 같은 줄 다른 심각도 금지).

    yaw_error 미수신(heading_missing) 로그에도 같은 클래스를 쓴다. 노드는 막힌 틱에서만 update(True),
    yaw_error 가 생긴 뒤 틱에서만 update(False)(첫 번만 'recover')를 부르고, 그 밖의 틱(yaw 없음 +
    후보각이 각도 등)은 부르지 않는다
    → 후보각이 20000 ↔ 각도로 깜빡여도 'recover'·'enter' 가 번갈아 나오지 않는다(ERROR 폭주 방지).
    """

    def __init__(self, period_sec=5.0):
        self.period_sec = float(period_sec)
        self.stale = False
        self._last_log_t = None

    def update(self, stale, now):
        if stale and not self.stale:
            self.stale = True
            self._last_log_t = now
            return 'enter'
        if stale:
            if now - self._last_log_t >= self.period_sec:
                self._last_log_t = now
                return 'repeat'
            return None
        if self.stale:
            self.stale = False
            self._last_log_t = None
            return 'recover'
        return None

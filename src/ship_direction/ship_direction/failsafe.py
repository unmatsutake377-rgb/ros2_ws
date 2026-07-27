"""ship_direction 의 판정 로직 — **ROS 비의존 순수 파이썬.**

여기엔 rclpy 를 임포트하지 않는다. 그래야 ROS 없이도 단위 테스트할 수 있다:

    from ship_direction.failsafe import SensorWatch, TemporalVote, median_min

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

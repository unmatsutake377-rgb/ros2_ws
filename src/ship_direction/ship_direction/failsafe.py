"""ship_direction 의 판정 로직 — **ROS 비의존 순수 파이썬.**

여기엔 rclpy 를 임포트하지 않는다. 그래야 ROS 없이도 단위 테스트할 수 있다:

    from ship_direction.failsafe import SensorWatch, TemporalVote

시각(now)은 호출자가 주입한다 → 테스트에서 시간을 마음대로 돌릴 수 있다.
"""

from collections import deque


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

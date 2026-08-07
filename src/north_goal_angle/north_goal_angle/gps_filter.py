"""GPS 필터 — ROS 비의존 순수 로직 (시간 주입식이라 배 없이 테스트).

fix_is_usable(gps_guard)가 status·Null Island 를 걸러 '쓸 수 있는 fix'만 넘겨준 뒤,
여기서 **품질·이상치·지터**를 다룬다:
  1) 공분산 필터  — RTK 품질이 나쁜(공분산 큰) fix 는 버린다
  2) 점프 필터    — 두 fix 사이 속도가 물리적 상한을 넘으면 순간이동(멀티패스)으로 보고 버린다
  3) 위치 스무딩  — 오차 크기에 따라 alpha 를 조절하는 저역통과(파도 지터 흡수)
부수 산출: estimated_speed_mps(정상 fix 간 이동/시간) — lookahead·도착반경 계산에 쓴다.

update(raw_lat, raw_lon, cov, now) → (accepted: bool, reason: str).
now(단조시각)를 인자로 받아 순수 함수로 만든다(노드가 time.monotonic() 을 넣는다).
"""

import math

_R_EARTH_M = 6371000.0


def _hav(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * _R_EARTH_M * math.asin(min(1.0, math.sqrt(a)))


class GPSFilter:
    def __init__(self, cov_threshold=2.0, max_speed_mps=15.0):
        self.cov_threshold = cov_threshold
        self.max_speed_mps = max_speed_mps

        self.have_fix = False
        self.filtered_lat = 0.0
        self.filtered_lon = 0.0
        self.estimated_speed_mps = 0.0

        self._prev_raw_lat = None
        self._prev_raw_lon = None
        self._prev_t = None

    def update(self, raw_lat, raw_lon, cov, now):
        """쓸 수 있는 fix 를 넣으면 필터링·스무딩. accepted=False 면 이번 값은 버려짐."""
        # 1) 공분산 필터 — cov 정보가 있고(>0) 임계 초과면 버린다. (음수/0 은 '정보 없음' 취급)
        if cov is not None and cov > self.cov_threshold > 0.0:
            return False, f"cov {cov:.2f} > {self.cov_threshold}"

        # 2) 점프 필터 — 직전 원시 fix 대비 속도 상한 초과면 버린다.
        if self._prev_raw_lat is not None and self._prev_t is not None:
            dt = now - self._prev_t
            if dt > 0.0:
                d = _hav(self._prev_raw_lat, self._prev_raw_lon, raw_lat, raw_lon)
                speed = d / dt
                if speed > self.max_speed_mps:
                    return False, f"jump {speed:.1f} m/s > {self.max_speed_mps}"
                self.estimated_speed_mps = speed

        self._prev_raw_lat, self._prev_raw_lon, self._prev_t = raw_lat, raw_lon, now

        # 3) 위치 스무딩 (Adaptive LFP) — 오차가 작으면 강하게(안정), 크면 약하게(따라감).
        if not self.have_fix:
            self.filtered_lat, self.filtered_lon = raw_lat, raw_lon
            self.have_fix = True
        else:
            err = _hav(raw_lat, raw_lon, self.filtered_lat, self.filtered_lon)
            if err < 0.5:
                alpha = 0.15
            elif err < 1.5:
                alpha = 0.30
            else:
                alpha = 0.80
            self.filtered_lat = alpha * raw_lat + (1.0 - alpha) * self.filtered_lat
            self.filtered_lon = alpha * raw_lon + (1.0 - alpha) * self.filtered_lon

        return True, "ok"

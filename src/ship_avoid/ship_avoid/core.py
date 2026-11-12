"""도리도리회피 — ship_avoid 순수 로직 (ROS 비의존). 낮은 장애물 회피 + 도리도리.

계약서: docs/문제와작업/계약서_도리도리회피_20261112.md (초안, 미승인)

좌표 규약
  · 세계: x = 동(m), y = 북(m), yaw = 나침반(deg, 시계방향 +, 0 = 북) — /imu/yaw 와 같다.
  · 상대방위: **좌 양수**(deg) — LiDAR a−80·/candidate_angle·/geofence_state 와 같다.
  · 코스 좌표: 미션 시작 순간 헤딩 ψ0 를 코스 축으로 잠금. s = 축 방향 전진(m), l = 축 오른쪽(+, m).

칸(cell) 정의
  · LiDAR 로 안 보이는 **빨간 낮은 장애물 하나를 지나갈 때마다 한 칸.** 칸이 바뀌면 도리도리.
  · 시작할 때 1회. 확정 장애물이 앞에 하나도 없으면 explore_m 마다 1회(정보가 없을 때 찾으러).

eunwootest 에서 가져온 아이디어
  · 가중 선형회귀 속도(gate.calculate_speed) → wls_slope: 배 전진속도·정지 판정·조류 밀림 추정.
  · 색 다수결 + 연속 확인(fusion 오검출 카운터 취지) → LowMap.confirmed.
  · LiDAR 클러스터(lidar_object_detector) → lidar_buoys.py (리셋 버그 수정 + 부표 크기 검사).
"""
import math
from dataclasses import dataclass, field

SPIN_RIGHT = 5000.0         # 우선회 (ship_direction·motor_control 과 같은 값)
SPIN_LEFT = 6000.0          # 좌선회
CANDIDATE_INVALID = 20000.0  # ship_direction (C) 자율 회피에 맡김
STOP_HOLD = 50000.0         # 정지


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def compass_bearing(x, y, tx, ty):
    return math.degrees(math.atan2(tx - x, ty - y)) % 360.0


def ground_plane_range(cam_h, pitch_down_deg, v_norm):
    """수면 기준 거리. v_norm = (v_bottom − cy) / fy (아래 +). 수평선 위·근처면 None."""
    dep = pitch_down_deg + math.degrees(math.atan(v_norm))
    if dep <= 0.5:
        return None
    return cam_h / math.tan(math.radians(dep))


def ground_plane_sigma(cam_h, d, pitch_sigma_deg):
    phi = math.atan2(cam_h, d)
    return cam_h / math.sin(phi) ** 2 * math.radians(pitch_sigma_deg)


def wls_slope(samples):
    """가중 선형회귀 기울기 (gate.calculate_speed). samples = [(t, value, weight)]. 3개 미만 None."""
    if len(samples) < 3:
        return None
    sw = sum(w for _, _, w in samples)
    if sw <= 0:
        return None
    mt = sum(w * t for t, _, w in samples) / sw
    mv = sum(w * v for _, v, w in samples) / sw
    num = sum(w * (t - mt) * (v - mv) for t, v, w in samples)
    den = sum(w * (t - mt) ** 2 for t, _, w in samples)
    return None if den <= 1e-9 else num / den


def hull_clear(x, y, yaw, ox, oy, orad, half_l, half_w):
    """직사각형 선체 ↔ 원 최소거리(음수 = 침범)."""
    s, c = math.sin(math.radians(yaw)), math.cos(math.radians(yaw))
    dx, dy = ox - x, oy - y
    fwd = dx * s + dy * c
    lat = dx * c - dy * s
    qx = min(max(fwd, -half_l), half_l)
    qy = min(max(lat, -half_w), half_w)
    return math.hypot(fwd - qx, lat - qy) - orad


# ── 2x2 행렬 (numpy 없이 — 노드 의존성 최소화) ─────────────────────────
def _add(a, b):
    return [[a[0][0] + b[0][0], a[0][1] + b[0][1]], [a[1][0] + b[1][0], a[1][1] + b[1][1]]]


def _inv(a):
    det = a[0][0] * a[1][1] - a[0][1] * a[1][0]
    if abs(det) < 1e-12:
        det = 1e-12
    return [[a[1][1] / det, -a[0][1] / det], [-a[1][0] / det, a[0][0] / det]]


def _mul(a, b):
    return [[a[0][0] * b[0][0] + a[0][1] * b[1][0], a[0][0] * b[0][1] + a[0][1] * b[1][1]],
            [a[1][0] * b[0][0] + a[1][1] * b[1][0], a[1][0] * b[0][1] + a[1][1] * b[1][1]]]


def _mv(a, v):
    return [a[0][0] * v[0] + a[0][1] * v[1], a[1][0] * v[0] + a[1][1] * v[1]]


@dataclass
class Track:
    tid: int
    x: float
    y: float
    P: list
    n: int = 1
    red: int = 0
    last_t: float = 0.0
    passed: bool = False
    side: int = 0          # 0 미정, −1 = 장애물 왼쪽으로 지나감, +1 = 오른쪽


class LowMap:
    """낮은 장애물 지도: 세계 좌표 정적 점 칼만 + 색 다수결."""

    def __init__(self, confirm_n=3, red_ratio=0.6, gate_m=0.35, chi2=13.8):
        self.tracks = []
        self.confirm_n, self.red_ratio = confirm_n, red_ratio
        self.gate_m, self.chi2 = gate_m, chi2
        self._next = 0

    def update(self, zx, zy, R, red, t):
        best, bd = None, 1e18
        for tr in self.tracks:
            S = _add(tr.P, R)
            d = [zx - tr.x, zy - tr.y]
            Si = _inv(S)
            m = d[0] * (Si[0][0] * d[0] + Si[0][1] * d[1]) + d[1] * (Si[1][0] * d[0] + Si[1][1] * d[1])
            if (m < self.chi2 or math.hypot(*d) < self.gate_m) and m < bd:
                best, bd = tr, m
        if best is None:
            self.tracks.append(Track(self._next, zx, zy, [row[:] for row in R], 1, int(red), t))
            self._next += 1
            return self.tracks[-1]
        K = _mul(best.P, _inv(_add(best.P, R)))
        dx = _mv(K, [zx - best.x, zy - best.y])
        best.x += dx[0]
        best.y += dx[1]
        IK = [[1 - K[0][0], -K[0][1]], [-K[1][0], 1 - K[1][1]]]
        best.P = _mul(IK, best.P)
        best.n += 1
        best.red += int(red)
        best.last_t = t
        return best

    def confirmed(self):
        return [tr for tr in self.tracks if tr.n >= self.confirm_n and tr.red / tr.n >= self.red_ratio]


@dataclass
class AvoidParams:
    # 선체 (1번배 실측 1.66 × 0.57)
    half_l: float = 0.83
    half_w: float = 0.285
    # 장애물·부표 (⚠️ 실측 필요)
    low_r: float = 0.20
    buoy_r: float = 0.12
    default_half_width_m: float = 1.5      # 노란부표 줄을 못 잡았을 때 통로 반폭 가정
    # 여유
    clear_low: float = 0.45                # 낮은 장애물(지도로만 피함) 선체–장애물 여유
    clear_wall: float = 0.35               # 노란부표 줄 여유 (시뮬 0.20 보다 3 m 통로 접촉 적음)
    guard_m: float = 0.15                  # 지금 또는 stop_lead_s 뒤 예상 위치에서 이보다 가까우면 반대로 최대 조향
    stop_lead_s: float = 0.6               # 관성 정지·반응 예측 시간 ⚠️ 실측(τ)
    # 도리도리
    dori_angle_deg: float = 60.0
    dori_tol_deg: float = 4.0
    dori_lead_s: float = 0.3               # 오버슈트 예측: |yaw rate| × lead 만큼 일찍 멈춤 ⚠️ 실측
    dori_settle_s: float = 0.3
    dori_hold_s: float = 1.2               # 정지 촬영(5프레임 투표 여유)
    dori_sweep_margin_m: float = 0.25
    dori_defer_max_m: float = 1.5          # 요청 후 이만큼 가도 안전 자리가 없으면 포기
    dori_explore_m: float = 3.0
    dori_timeout_s: float = 25.0
    stop_speed: float = 0.05
    # 주행 (out-in-out)
    look_m: float = 1.5
    turn_in_m: float = 2.5
    max_steer_deg: float = 60.0
    step_deg: float = 10.0                 # 안전 헤딩 탐색 간격
    # 거리 측정
    cam_h: float = 0.60                    # ⚠️ 실측 필요
    cam_pitch_deg: float = 0.0             # 아래로 숙인 각 ⚠️ 실측 필요
    bearing_sigma_deg: float = 0.7
    pitch_sigma_hold: float = 0.5
    pitch_sigma_move: float = 2.0
    pos_sigma: float = 0.03
    range_sigma_rel: float = 0.03          # 외부 거리(D455·LiDAR 매칭) 상대 σ
    max_range: float = 8.0
    # 지도
    confirm_n: int = 3
    red_ratio: float = 0.6
    wall_fill_step: float = 0.15
    buoy_match_m: float = 0.35             # LiDAR 부표와 이 안이면 그 부표로 본다(지도에 안 넣음)


@dataclass
class _Pose:
    t: float = -1.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    yaw_rate: float = 0.0
    hist: list = field(default_factory=list)   # (t, s, l)


class AvoidCore:
    """mode 5 미션 두뇌. 노드(또는 시뮬)가 set_* 로 넣고 step() 으로 /candidate_angle 값을 받는다."""

    def __init__(self, p=None):
        self.p = p or AvoidParams()
        self.reset()

    # ── 수명 ──
    def reset(self):
        self.active = False
        self.map = LowMap(self.p.confirm_n, self.p.red_ratio)
        self.pose = _Pose()
        self.pitch = 0.0
        self.buoys = []
        self.psi0 = 0.0
        self.ox = self.oy = 0.0
        self.state = "IDLE"
        self.seq = []
        self.seq_i = 0
        self.seq_t0 = 0.0
        self.dori_t0 = 0.0
        self.pending = False
        self.pending_s = 0.0
        self.last_dori_s = 0.0
        self.cells = 0
        self.n_dori = 0
        self.n_skip = 0
        self.capture_ok = False

    def activate(self, t):
        """wp_mode 가 5 가 된 첫 틱. 자세가 있어야 한다(없으면 False)."""
        if self.pose.t < 0:
            return False
        self.reset_keep_pose()
        self.active = True
        self.psi0 = self.pose.yaw
        self.ox, self.oy = self.pose.x, self.pose.y
        self.state = "DRIVE"
        self._request(0.0)          # 시작 도리도리
        return True

    def resume(self, t):
        """같은 zone 구간 안에서 잠깐 멈췄다(검출 침묵 → deactivate) 다시 켤 때.
        activate 와 달리 reset 하지 않는다 — 지도·코스 축 ψ0·원점·칸 수를 그대로 두고 DRIVE 로 돌아가
        도리도리를 1회 요청한다(멈춘 동안 배가 움직였으니 다시 본다). 자세가 있어야 한다(없으면 False).
        계약서_도리도리회피_v2_20261114 U1 (지도 수명 = zone 구간)."""
        if self.pose.t < 0:
            return False
        self.active = True
        self.state = "DRIVE"
        self.capture_ok = False
        self.pending = False
        self.pose.hist = []
        self._request(self.to_course(self.pose.x, self.pose.y)[0])
        return True

    def reset_keep_pose(self):
        pose, pitch, buoys = self.pose, self.pitch, self.buoys
        self.reset()
        self.pose, self.pitch, self.buoys = pose, pitch, buoys
        self.pose.hist = []

    def deactivate(self):
        self.active = False
        self.state = "IDLE"

    # ── 입력 ──
    def set_pose(self, t, x, y, yaw, yaw_rate=None):
        if yaw_rate is None and self.pose.t >= 0 and t > self.pose.t:
            yaw_rate = wrap180(yaw - self.pose.yaw) / (t - self.pose.t)
        self.pose.t, self.pose.x, self.pose.y, self.pose.yaw = t, x, y, yaw
        self.pose.yaw_rate = yaw_rate or 0.0
        s, l_ = self.to_course(x, y)
        self.pose.hist.append((t, s, l_))
        self.pose.hist = [h for h in self.pose.hist if t - h[0] <= 1.5]

    def set_pitch(self, pitch_down_deg):
        self.pitch = pitch_down_deg

    def set_buoys(self, pts):
        """LiDAR 노란부표 후보 (세계 x, y) 목록 — 매 스캔 갱신."""
        self.buoys = list(pts)

    def add_detections(self, t, x, y, yaw, dets):
        """카메라 검출 (촬영 순간 자세와 함께). dets = [(cls, bearing_left_deg, v_norm, range_m, conf[, age_s])].
        cls 0 = 빨강(낮은 장애물), 2 = 노랑. 그 외는 무시. capture_ok 가 아니면 버린다.
        age_s(노드 /avoid_detections VER 1)는 지금은 받기만 하고 쓰지 않는다(U5/U6 몫) — 5개짜리도 그대로 받는다."""
        if not (self.active and self.capture_ok):
            return 0
        moving = self.state == "DRIVE"
        used = 0
        for cls, brg, v_norm, rng, conf, *_age in dets:
            if cls not in (0, 2):
                continue
            if rng is not None and math.isfinite(rng) and rng > 0:
                d, sr = rng, max(0.03, self.p.range_sigma_rel * rng)
            else:
                d = ground_plane_range(self.p.cam_h, self.p.cam_pitch_deg + self.pitch, v_norm)
                if d is None:
                    continue
                sr = ground_plane_sigma(self.p.cam_h, d,
                                        self.p.pitch_sigma_move if moving else self.p.pitch_sigma_hold)
            if d > self.p.max_range:
                continue
            wb = math.radians(yaw - brg)
            ux, uy = math.sin(wb), math.cos(wb)
            vx, vy = uy, -ux
            sb = d * math.radians(self.p.bearing_sigma_deg)
            ps = self.p.pos_sigma ** 2
            R = [[sr * sr * ux * ux + sb * sb * vx * vx + ps, sr * sr * ux * uy + sb * sb * vx * vy],
                 [sr * sr * ux * uy + sb * sb * vx * vy, sr * sr * uy * uy + sb * sb * vy * vy + ps]]
            zx, zy = x + d * ux, y + d * uy
            # eunwootest sensor_fusion 의 카메라–LiDAR 매칭: LiDAR 가 보는 물체 = 노란부표 → 지도에 안 넣는다.
            #   매칭 안 되는 빨강 = LiDAR 가 못 보는 낮은 장애물(버리지 않는 게 이 미션의 핵심).
            if any(math.hypot(zx - bx, zy - by) < self.p.buoy_match_m for bx, by in self.buoys):
                continue
            self.map.update(zx, zy, R, cls == 0, t)
            used += 1
        return used

    # ── 좌표 ──
    def to_course(self, x, y):
        s0, c0 = math.sin(math.radians(self.psi0)), math.cos(math.radians(self.psi0))
        dx, dy = x - self.ox, y - self.oy
        return dx * s0 + dy * c0, dx * c0 - dy * s0

    def to_world(self, s, l_):
        s0, c0 = math.sin(math.radians(self.psi0)), math.cos(math.radians(self.psi0))
        return self.ox + s * s0 + l_ * c0, self.oy + s * c0 - l_ * s0

    def speed(self):
        v = wls_slope([(t, s, 1.0) for t, s, _ in self.pose.hist])
        return 0.0 if v is None else v

    # ── 장애물·벽 ──
    def obstacles(self):
        """(x, y, r, kind) — 확정 낮은 장애물 + LiDAR 노란부표."""
        out = [(tr.x, tr.y, self.p.low_r, "low") for tr in self.map.confirmed()]
        out += [(bx, by, self.p.buoy_r, "buoy") for bx, by in self.buoys]
        return out

    def walls(self, s_c):
        """s_c 근처(±3 m) 노란부표 가로 위치 중앙값 → (l_left, l_right)."""
        left, right = [], []
        for bx, by in self.buoys:
            s, l_ = self.to_course(bx, by)
            if abs(s - s_c) <= 3.0:
                (left if l_ < 0 else right).append(l_)
        med = (lambda v: sorted(v)[len(v) // 2])
        ll = med(left) if left else -self.p.default_half_width_m
        lr = med(right) if right else self.p.default_half_width_m
        return ll, lr

    def wall_points(self):
        """이웃 노란부표(0.6~1.4 m) 사이를 메운 가상 벽 점 (x, y, r)."""
        pts = []
        b = sorted(self.buoys, key=lambda q: self.to_course(*q))
        for i, (x1, y1) in enumerate(b):
            for x2, y2 in b[i + 1:]:
                d = math.hypot(x2 - x1, y2 - y1)
                if 0.6 <= d <= 1.4 and abs(self.to_course(x1, y1)[1] - self.to_course(x2, y2)[1]) < 0.5:
                    k = max(1, int(d / self.p.wall_fill_step))
                    pts += [(x1 + (x2 - x1) * j / k, y1 + (y2 - y1) * j / k, 0.08) for j in range(1, k)]
        return pts

    def virtual_obstacles(self, extra=0.20):
        """ship_direction 에 넘길 가상 장애물: 낮은 장애물(+extra) + 노란부표 벽."""
        v = [(tr.x, tr.y, self.p.low_r + extra) for tr in self.map.confirmed()]
        return v + self.wall_points()

    def _ahead(self, lead_s):
        d = max(0.0, self.speed()) * lead_s
        a = math.radians(self.pose.yaw)
        return self.pose.x + d * math.sin(a), self.pose.y + d * math.cos(a)

    def spin_clear(self, x=None, y=None):
        r_sw = math.hypot(self.p.half_l, self.p.half_w) + self.p.dori_sweep_margin_m
        if x is None:
            x, y = self.pose.x, self.pose.y
        return all(math.hypot(ox - x, oy - y) - r >= r_sw for ox, oy, r, _ in self.obstacles())

    # ── 칸 세기 ──
    def _update_cells(self):
        s_b, _ = self.to_course(self.pose.x, self.pose.y)
        passed_now = 0
        for tr in self.map.confirmed():
            if tr.passed:
                continue
            s_o, _ = self.to_course(tr.x, tr.y)
            if s_o < s_b - (self.p.half_l + self.p.low_r):
                tr.passed = True
                passed_now += 1
        if passed_now:
            self.cells += passed_now
            if self.state != "DORI":
                self._request(s_b)
        ahead = [tr for tr in self.map.confirmed() if not tr.passed]
        if not ahead and not self.pending and self.state != "DORI" and \
                s_b - self.last_dori_s >= self.p.dori_explore_m:
            self._request(s_b)

    def _request(self, s_b):
        if not self.pending:
            self.pending = True
            self.pending_s = s_b

    # ── 도리도리 ──
    def _start_dori(self, t):
        a = self.p.dori_angle_deg
        # 원안 순서: 오른쪽 → 원위치 → 왼쪽 → 원위치 (상대각 좌 양수라 오른쪽 = −a)
        self.seq = [("stop", 0.0), ("hold", 0.0), ("spin", -a), ("hold", -a), ("spin", 0.0),
                    ("spin", a), ("hold", a), ("spin", 0.0)]
        self.seq_i, self.seq_t0, self.dori_t0 = 0, t, t
        self.state = "DORI"
        self.pending = False
        self.n_dori += 1

    def _end_dori(self, aborted):
        self.state = "DRIVE"
        self.pending = False
        self.last_dori_s = self.to_course(self.pose.x, self.pose.y)[0]
        if aborted:
            self.n_skip += 1

    def _dori_step(self, t):
        kind, rel = self.seq[self.seq_i]
        target = (self.psi0 - rel) % 360.0           # 상대 좌 양수 → 나침반은 빼기
        err = wrap180(target - self.pose.yaw)
        cmd, done = STOP_HOLD, False
        self.capture_ok = False
        if kind == "stop":
            done = abs(self.speed()) < self.p.stop_speed and abs(self.pose.yaw_rate) < 2.0 and \
                t - self.seq_t0 > self.p.dori_settle_s
        elif kind == "hold":
            if t - self.seq_t0 > self.p.dori_settle_s and abs(self.pose.yaw_rate) < 2.0:
                self.capture_ok = True
            done = t - self.seq_t0 > self.p.dori_settle_s + self.p.dori_hold_s
        else:
            if abs(err) < self.p.dori_tol_deg + abs(self.pose.yaw_rate) * self.p.dori_lead_s:
                done = abs(self.pose.yaw_rate) < 2.0
            else:
                if not self.spin_clear():            # 회전 중 연속 가드
                    self._end_dori(True)
                    return STOP_HOLD
                cmd = SPIN_RIGHT if err > 0 else SPIN_LEFT
        if t - self.dori_t0 > self.p.dori_timeout_s:
            self._end_dori(True)
            return STOP_HOLD
        if done:
            self.seq_i += 1
            self.seq_t0 = t
            if self.seq_i >= len(self.seq):
                self._end_dori(False)
        return cmd

    # ── 주행 (out-in-out) ──
    def _next_obstacle(self, s_b):
        best = None
        for tr in self.map.confirmed():
            s_o, l_o = self.to_course(tr.x, tr.y)
            if s_o > s_b - (self.p.half_l + self.p.low_r) and (best is None or s_o < best[1]):
                best = (tr, s_o, l_o)
        return best

    def _pass_lateral(self, tr, s_o, l_o, l_b):
        ll, lr = self.walls(s_o)
        lo = ll + self.p.buoy_r + self.p.half_w + self.p.clear_wall
        hi = lr - self.p.buoy_r - self.p.half_w - self.p.clear_wall
        off = self.p.low_r + self.p.half_w + self.p.clear_low
        if tr.side == 0:   # 편 확정(한 번 정하면 유지 — 뒤집기 금지)
            free_l = (l_o - self.p.low_r) - (ll + self.p.buoy_r)
            free_r = (lr - self.p.buoy_r) - (l_o + self.p.low_r)
            tr.side = -1 if free_l >= free_r else 1
        lp = l_o + tr.side * off
        return min(max(lp, lo), hi), lo, hi

    def _forbidden(self, s_q, l_q):
        """s_q 에서 확정 장애물 겹침 구간이면 확정된 편 경계로 밀어낸다."""
        for tr in self.map.confirmed():
            s_o, l_o = self.to_course(tr.x, tr.y)
            if abs(s_o - s_q) < self.p.low_r + self.p.half_l + self.p.clear_low:
                off = self.p.low_r + self.p.half_w + self.p.clear_low
                if abs(l_q - l_o) < off:
                    side = tr.side or (1 if l_q >= l_o else -1)
                    l_q = l_o + side * off
        return l_q

    def target_lateral(self, s_q, s_b, l_b):
        nxt = self._next_obstacle(s_b)
        ll, lr = self.walls(s_q)
        lo = ll + self.p.buoy_r + self.p.half_w + self.p.clear_wall
        hi = lr - self.p.buoy_r - self.p.half_w - self.p.clear_wall
        if nxt is None:
            lt = 0.5 * (lo + hi)
        else:
            tr, s_o, l_o = nxt
            lp, _, _ = self._pass_lateral(tr, s_o, l_o, l_b)
            # out-in-out: 정점(in) = 장애물 옆 lp, 바깥(out) = 지나갈 편 쪽 벽. 장애물 쪽 벽엔 절대 안 붙는다.
            #   오른쪽으로 비켜 지나가야(side +1) 하면 미리 오른쪽 벽 → 정점 → 다시 바깥.
            out = hi if tr.side > 0 else lo
            d = s_o - s_q
            if d > self.p.turn_in_m:
                lt = out
            elif d > 0:
                lt = lp + (out - lp) * d / self.p.turn_in_m
            else:
                lt = lp
        lt = self._forbidden(s_q, min(max(lt, lo), hi))
        return lt

    def _clearance_along(self, heading, near):
        """heading(나침반) 으로 곧게 갈 때 예상 위치들(지금·0.4·0.8·1.2 m)의 최소 선체 여유."""
        x, y = self.pose.x, self.pose.y
        a = math.radians(heading)
        best = 1e9
        for d in (0.0, 0.4, 0.8, 1.2):
            px, py = x + d * math.sin(a), y + d * math.cos(a)
            for ox, oy, r in near:
                c = hull_clear(px, py, heading, ox, oy, r, self.p.half_l, self.p.half_w)
                if c < best:
                    best = c
        return best

    def _drive_step(self):
        """out-in-out 목표 방위 → 안전 헤딩 탐색으로 다듬어 (B) 각도로 낸다.
        후보 방위(±max_steer, step_deg)마다 짧게 직진 예측해 선체 여유 ≥ guard 인 것 중 목표에 가장 가까운 것.
        전부 안 되면 여유가 가장 큰 것. (C)(20000) 에는 넘기지 않는다 — 계약서 §7-2 10."""
        x, y, yaw = self.pose.x, self.pose.y, self.pose.yaw
        s_b, l_b = self.to_course(x, y)
        s_q = s_b + self.p.look_m
        tx, ty = self.to_world(s_q, self.target_lateral(s_q, s_b, l_b))
        nominal = wrap180(yaw - compass_bearing(x, y, tx, ty))
        nominal = max(-self.p.max_steer_deg, min(self.p.max_steer_deg, nominal))
        pts = [(ox, oy, r) for ox, oy, r, _ in self.obstacles()] + self.wall_points()
        near = [q for q in pts if math.hypot(q[0] - x, q[1] - y) < 3.5]
        if not near:
            return nominal % 360.0
        best_ok, best_any = None, None
        k = int(self.p.max_steer_deg // self.p.step_deg)
        for i in range(-k, k + 1):
            rel = i * self.p.step_deg
            c = self._clearance_along((yaw - rel) % 360.0, near)
            cost = abs(rel - nominal)
            if c >= self.p.guard_m and (best_ok is None or cost < best_ok[0]):
                best_ok = (cost, rel)
            if best_any is None or c > best_any[0]:
                best_any = (c, rel)
        rel = best_ok[1] if best_ok is not None else best_any[1]
        return rel % 360.0

    # ── 한 틱 ──
    def step(self, t):
        """반환: /candidate_angle 값. 비활성이면 None(발행하지 않음)."""
        if not self.active:
            return None
        self._update_cells()
        if self.state == "DRIVE":
            self.capture_ok = True               # 주행 중 전방 카메라도 지도에 넣는다(σ 크게)
            if self.pending:
                s_b = self.to_course(self.pose.x, self.pose.y)[0]
                if self.spin_clear() and self.spin_clear(*self._ahead(self.p.stop_lead_s)):
                    self._start_dori(t)     # 지금 + 멈춰 설 예상 위치 둘 다 회전 여유가 있어야
                elif s_b - self.pending_s > self.p.dori_defer_max_m:
                    self.pending = False
                    self.n_skip += 1
        if self.state == "DORI":
            return self._dori_step(t)
        return self._drive_step()

    def status(self):
        n_conf = len(self.map.confirmed())
        st = self.state if self.state != "DORI" else f"DORI_{self.seq[self.seq_i][0]}"
        return f"{st} cells={self.cells} low={n_conf} dori={self.n_dori} skip={self.n_skip}"


def virtual_ranges(x, y, yaw, objs, a_min=-80.0, inc=1.0, n=161, max_r=30.0):
    """가상 장애물 원 → /geofence_state 형식 거리 배열. 상대방위 좌 양수(인덱스 0 = 우현 −80).
    반환: [a_min, inc, r0, …] (멀면 inf)."""
    out = [a_min, inc]
    for k in range(n):
        th = math.radians(yaw - (a_min + k * inc))
        dx, dy = math.sin(th), math.cos(th)
        best = math.inf
        for ox, oy, r in objs:
            px, py = ox - x, oy - y
            tca = px * dx + py * dy
            if tca <= 0:
                continue
            d2 = px * px + py * py - tca * tca
            if d2 > r * r:
                continue
            tt = tca - math.sqrt(max(r * r - d2, 0.0))
            if 0 < tt < best:
                best = tt
        out.append(best if best <= max_r else math.inf)
    return out

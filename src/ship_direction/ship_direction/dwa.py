"""(C) DWA — 3자유도 굴림 + 직사각형 선체 vs LiDAR 점 (ROS 비의존, numpy).

근거: docs/문제와작업/시뮬3자유도_발견_20261106.md 발견 10·11, 계약서_DWA_20261106.md.
노드 스위치 dwa_enabled(기본 끔)가 켜지고 wp_mode 가 dwa_modes(5·8 중) 일 때만 쓴다.

틱마다 조향각 후보 0~160(5° 간격 33개)을 각각 2.5 s 굴려(3자유도 모델, 명령은 바로 바뀐다고 가정)
선체 직사각형과 LiDAR 점의 최소 거리를 본다. 0.25 m 안으로 들면 그 후보는 버리고, 남은 것 중
|목표 − 끝 헤딩| + 여유 벌점 + 직전 명령과의 차 벌점이 가장 작은 각을 고른다(동률이면 작은 각).
전부 불가 → None → 노드가 기존 결과(갭 정렬·비킴·후진)를 그대로 쓴다.

🚨 계수(질량·감쇠·추진기 위치)와 PWM→추력은 **가정값**이다. 원본은 sim/vrx make_boat.BOATS 와
   convert.pwm_to_thrust 이고 여기는 사본이다(노드는 tools/·sim/ 을 import 하지 않는다).
   test/test_dwa.py 가 원본과 같은지 본다. 수조 실측으로 원본을 바꾸면 여기도 같이 바꾼다.
🚨 PWM 상수(1360/100/81/1590)는 motor_control 의 사본이다. motor_control 의 base_pwm·max_diff·
   reverse_pwm 을 배에서 바꾸면 모델만 어긋난다(경고 없음) → 그 배는 dwa_enabled 를 끈다(현장 조정표).
   steer_invert 는 반영하지 않는다(물리 기준 좌/우).

좌표: 선체 x 앞, y 왼쪽(+), r 반시계 양수. 나침반 헤딩 ψ(0=북, 시계)는 ψ̇ = −r.
월드(굴림 시작 자세 기준): x 동, y 북. 시작 ψ = 0 이라 앞 = +y, 왼쪽 = −x.
"""

import math

import numpy as np

# ---- 선체 표 — 질량·loa·boa·hydro·thrusters 는 make_boat.BOATS 사본(가정값) ----
# rect_l·rect_b 는 접촉 판정 직사각형(결정 4): 길이 = loa, 폭 = 줄자 실측(A 0.57 09-28, B 0.535 10-24).
#   make_boat 의 boa(0.55/0.522)보다 넓다(보수적). 관성 Iz 는 make_boat 의 loa·boa 로 계산한다.
HULLS = {
    'A': {
        'mass': 34.0, 'loa': 1.66, 'boa': 0.55,
        'hydro': {'xU': 10.0, 'xUU': 25.0, 'yV': 40.0, 'yVV': 60.0, 'nR': 6.0, 'nRR': 8.0},
        'thrusters': [('left', -0.70, 0.12), ('right', -0.70, -0.12)],
        'rect_l': 1.66, 'rect_b': 0.57,
    },
    'B': {
        'mass': 30.0, 'loa': 1.77, 'boa': 0.522,
        'hydro': {'xU': 10.0, 'xUU': 52.0, 'yV': 50.0, 'yVV': 80.0, 'nR': 8.0, 'nRR': 12.0},
        'thrusters': [('rear_left', -0.65, 0.14), ('rear_right', -0.65, -0.14),
                      ('front_left', -0.285, 0.0625), ('front_right', -0.285, -0.0625)],
        'rect_l': 1.77, 'rect_b': 0.535,
    },
}

# ---- PWM → 추력 (convert.pwm_to_thrust 사본, ROS 규약 <1500 전진) ----
PWM_NEUTRAL = 1500
PWM_MIN, PWM_MAX = 1100, 1900
PWM_DEADBAND = 25            # 1475~1525 정지
FWD_MAX_N = 47.0             # T200 14.8 V 근사(가정)
REV_MAX_N = 35.0
THRUST_TAU_SEC = 0.15        # 추력 1차 지연(boat3dof 기본값)

# ---- 명령 → PWM (motor_control 분기 (4)(5) 사본, 그 밖은 1500) ----
BASE_PWM = 1360              # motor_control base_pwm 기본값
MAX_DIFF = 100               # motor_control max_diff 기본값
MAX_ANGLE = 81               # motor_control max_angle(매핑 상수)
REVERSE_PWM = 1590           # motor_control reverse_pwm 기본값
CMD_MIN, CMD_MAX = -1.0, 161.0   # 분기 (4) 정상 조향 구간
SPIN_RIGHT = 5000.0          # 분기 (5) 후진 구간 상한(161 < a < 5000)

# ---- DWA 상수 (계약서 '옮길 사양' — 바꾸려면 계약서부터) ----
LIDAR_FORWARD_DEG = 80.0     # LiDAR 프레임 정면
CAND_MAX_DEG = 160.0
CAND_STEP_DEG = 5.0          # 후보 0, 5, …, 160 (33개)
HORIZON_SEC = 2.5
DT_SEC = 0.1                 # 굴림 한 스텝 = 제어 주기(control_period_sec 0.1 가정)
SUBSTEPS = 2
D_HARD_M = 0.25              # 이보다 가까우면 후보 버림
D_SOFT_M = 0.8               # 이보다 가까우면 여유 벌점
W_CLEAR = 0.8
W_SMOOTH = 0.1
R_MIN_M = 0.05               # 점 필터: R_MIN < r ≤ R_MAX
R_MAX_M = 5.0
NO_POINT_CLEAR_M = 99.0      # 점이 없을 때 여유
INIT_STATE = (0.9, 0.0, 0.0)  # 추정 (u, v, r) 초기값
INIT_LAST = 80.0             # 직전 명령 초기값(정면)


def pwm_to_thrust(pwm):
    """ROS 규약 PWM(µs) → 추력[N], 양수 = 전진. convert.pwm_to_thrust 와 같은 숫자."""
    pwm = float(pwm)
    lo = PWM_NEUTRAL - PWM_DEADBAND
    hi = PWM_NEUTRAL + PWM_DEADBAND
    if pwm < lo:
        span = lo - PWM_MIN
        f = min(1.0, (lo - pwm) / span)
        return FWD_MAX_N * f
    if pwm > hi:
        span = PWM_MAX - hi
        f = min(1.0, (pwm - hi) / span)
        return -REV_MAX_N * f
    return 0.0


def pwm_from_cmd(angle):
    """/desired_angle 값 → (pwm_l, pwm_r). motor_control 분기 (4)(5) 사본, 그 밖(None·비유한 포함) 1500.

    (4) −1~161: off = a − 80 (±81 로 자름), d = int(|off|/81·100), 오른쪽 = 1360 − s, 왼쪽 = 1360 + s
        (s = d, off > 0 = 좌조향이면 +). (5) 161 < a < 5000: 1590/1590.
    목표 근접 감속·근접 감속·L1 상한·steer_invert 는 반영하지 않는다(위험 2).
    """
    if angle is None or not (CMD_MIN <= angle <= CMD_MAX):
        if angle is not None and CMD_MAX < angle < SPIN_RIGHT:
            return REVERSE_PWM, REVERSE_PWM
        return PWM_NEUTRAL, PWM_NEUTRAL
    off = max(-float(MAX_ANGLE), min(float(MAX_ANGLE), angle - LIDAR_FORWARD_DEG))
    d = int(abs(off) / MAX_ANGLE * MAX_DIFF)
    s = d if off > 0 else -d
    return BASE_PWM + s, BASE_PWM - s


def rect_dist(wx, wy, cx, cy, psi_deg, length, width):
    """선체 직사각형(length × width, 중심 (cx, cy), 나침반 헤딩 psi_deg)과 점 (wx, wy) 의 거리.

    안쪽 점은 0. wx·wy 는 점 배열 (N,), cx·cy·psi_deg 는 스칼라 또는 (K,) → 결과 (N,) 또는 (K, N).
    """
    wx = np.asarray(wx, dtype=float)
    wy = np.asarray(wy, dtype=float)
    cx = np.asarray(cx, dtype=float)[..., None]
    cy = np.asarray(cy, dtype=float)[..., None]
    p = np.radians(np.asarray(psi_deg, dtype=float))[..., None]
    fx, fy = np.sin(p), np.cos(p)                   # 앞 단위벡터(동, 북)
    dx, dy = wx - cx, wy - cy
    u = dx * fx + dy * fy                           # 앞 성분
    w = dx * (-fy) + dy * fx                        # 왼쪽 성분
    return np.hypot(np.maximum(np.abs(u) - length / 2, 0),
                    np.maximum(np.abs(w) - width / 2, 0))


class Hull3DOF:
    """3자유도 선체 — tools/sim/boat3dof.Boat3DOF 와 같은 식.

      m(u̇ − v r) = ΣT_i − (xU u + xUU |u| u)
      m(v̇ + u r) = −(yV v + yVV |v| v)
      Iz ṙ      = Σ(−y_i T_i) − (nR r + nRR |r| r)

    agility a(결정 5): 감쇠 계수 ÷a, 추력 ×a, 질량·관성 그대로. a = 1.0 이면 원본과 같다.
    상태(x, y, psi, u, v, r, T)는 스칼라 또는 같은 모양의 numpy 배열(후보 여러 개 동시 굴림).
    """

    def __init__(self, hull='A', agility=1.0, substeps=SUBSTEPS, thrust_tau=THRUST_TAU_SEC):
        spec = HULLS[hull]
        self.agility = float(agility)
        self.m = float(spec['mass'])
        L, B = float(spec['loa']), float(spec['boa'])
        self.Iz = self.m / 12.0 * (L * L + B * B)
        self.h = {k: float(v) / self.agility for k, v in spec['hydro'].items()}
        self.thrusters = [(name, float(tx), float(ty)) for name, tx, ty in spec['thrusters']]
        self.T = [0.0] * len(self.thrusters)          # 실제 추력(지연 반영)
        self.thrust_tau = float(thrust_tau)
        self.substeps = int(substeps)
        self.x = self.y = self.psi = 0.0
        self.u = self.v = self.r = 0.0

    def targets(self, pwm_l, pwm_r):
        """추진기별 목표 추력(×agility). y > 0(왼쪽) 추진기는 pwm_l, 나머지는 pwm_r."""
        if np.ndim(pwm_l) == 0:
            tl = pwm_to_thrust(pwm_l) * self.agility
            tr = pwm_to_thrust(pwm_r) * self.agility
        else:
            tl = np.array([pwm_to_thrust(p) for p in pwm_l]) * self.agility
            tr = np.array([pwm_to_thrust(p) for p in pwm_r]) * self.agility
        return [tl if ty > 0 else tr for _n, _tx, ty in self.thrusters]

    def step(self, pwm_l, pwm_r, dt):
        return self.step_targets(self.targets(pwm_l, pwm_r), dt)

    def step_targets(self, tgt, dt):
        h, m, Iz = self.h, self.m, self.Iz
        sub = dt / self.substeps
        for _ in range(self.substeps):
            a = sub / self.thrust_tau if self.thrust_tau > 0 else 1.0
            self.T = [T + (g - T) * min(1.0, a) for T, g in zip(self.T, tgt)]
            X = sum(self.T)
            N = sum(-ty * T for (_n, _tx, ty), T in zip(self.thrusters, self.T))
            u, v, r = self.u, self.v, self.r
            du = (X - (h['xU'] * u + h['xUU'] * abs(u) * u)) / m + v * r
            dv = (-(h['yV'] * v + h['yVV'] * abs(v) * v)) / m - u * r
            dr = (N - (h['nR'] * r + h['nRR'] * abs(r) * r)) / Iz
            self.u += du * sub
            self.v += dv * sub
            self.r += dr * sub
            self.psi = (self.psi - np.degrees(self.r) * sub) % 360.0
            p = np.radians(self.psi)
            fx, fy = np.sin(p), np.cos(p)          # 전진 단위벡터(동, 북)
            lx, ly = -fy, fx                       # 왼쪽 단위벡터
            self.x += (self.u * fx + self.v * lx) * sub
            self.y += (self.u * fy + self.v * ly) * sub
        return self.x, self.y, self.psi


def _wrap180(a):
    return ((a + 180.0) % 360.0) - 180.0


class DwaPlanner:
    """조향각 후보마다 굴려서 고른다. 노드가 DWA 모드 (C) 틱마다 choose → observe 순으로 부른다.

    추정 상태(u, v, r)는 observe(명령) 마다 0.1 s 앞으로 굴려 이어 간다(리미터 전 값, 추력 지연 유지).
    굴림은 매번 추력 T = 0 에서 시작하고 (u, v, r) 만 이어받는다.
    """

    def __init__(self, hull='A', agility=1.0, lidar_x_m=0.0):
        if hull not in HULLS:
            raise ValueError(f'dwa hull {hull!r} (A/B)')
        self.hull = hull
        self.agility = float(agility)
        self.lidar_x_m = float(lidar_x_m)
        self.rect_l = float(HULLS[hull]['rect_l'])
        self.rect_b = float(HULLS[hull]['rect_b'])
        self.cands = np.arange(0.0, CAND_MAX_DEG + 1e-6, CAND_STEP_DEG)
        self.n_steps = int(round(HORIZON_SEC / DT_SEC))
        self.reset()

    def reset(self):
        """추정 초기화 — (u, v, r) = (0.9, 0, 0), 직전 명령 80, 추력 0. DWA 모드 진입 때(결정 6)."""
        self.state = INIT_STATE
        self.last = INIT_LAST
        self.est = Hull3DOF(self.hull, self.agility)

    def observe(self, cmd):
        """이번 틱에 낸 명령(리미터 전)으로 추정을 0.1 s 굴린다.

        모델은 어떤 값이든 pwm_from_cmd 로 굴린다(NaN·50000 → 1500, 260 → 후진).
        직전 명령 last(변화 벌점 기준)는 유한한 정상 조향값 0~160 일 때만 바꾼다 — 260·50000·NaN 은
        유지. 시제품과 다른 점은 이것뿐: 시제품은 그대로 넣어 /yaw_error NaN 틱 뒤 last = NaN →
        다음 틱 비용이 전부 NaN → 첫 후보 0°(최대 우선회)를 냈다(리뷰 차단 1).
        """
        b = self.est
        b.u, b.v, b.r = self.state
        b.x = b.y = b.psi = 0.0
        pl, pr = pwm_from_cmd(cmd)
        b.step(pl, pr, DT_SEC)
        self.state = (float(b.u), float(b.v), float(b.r))
        if cmd is not None and math.isfinite(cmd) and 0.0 <= cmd <= CAND_MAX_DEG:
            self.last = float(cmd)

    def points(self, angle_array, distance_array):
        """LiDAR (각, 거리) → 선체 좌표 (x 앞, y 왼쪽) 배열 (N, 2). x 에 lidar_x_m 을 더한다."""
        pts = []
        for a, r in zip(angle_array, distance_array):
            if r is None or math.isinf(r) or math.isnan(r) or r <= R_MIN_M or r > R_MAX_M:
                continue
            t = math.radians(a - LIDAR_FORWARD_DEG)
            pts.append((r * math.cos(t) + self.lidar_x_m, r * math.sin(t)))
        return np.array(pts) if pts else np.zeros((0, 2))

    def choose(self, angle_array, distance_array, yaw_mapped):
        """후보 중 비용 최소 각(0~160). 다 불가·yaw_mapped 비유한 → None. 입력은 안 바꾼다."""
        if yaw_mapped is None or not math.isfinite(yaw_mapped):
            return None
        P = self.points(angle_array, distance_array)
        wx, wy = -P[:, 1], P[:, 0]                  # 월드: 앞 = +y, 왼쪽 = −x
        goal = yaw_mapped - LIDAR_FORWARD_DEG
        k = len(self.cands)

        b = Hull3DOF(self.hull, self.agility)
        b.h, b.m, b.Iz = dict(self.est.h), self.est.m, self.est.Iz
        u0, v0, r0 = self.state
        b.u, b.v, b.r = np.full(k, u0), np.full(k, v0), np.full(k, r0)
        b.x, b.y, b.psi = np.zeros(k), np.zeros(k), np.zeros(k)
        b.T = [np.zeros(k) for _ in b.thrusters]
        pwm = [pwm_from_cmd(float(a)) for a in self.cands]
        tgt = b.targets([p[0] for p in pwm], [p[1] for p in pwm])

        mc = np.full(k, NO_POINT_CLEAR_M)
        for _ in range(self.n_steps):
            b.step_targets(tgt, DT_SEC)
            if len(P):
                d = rect_dist(wx, wy, b.x, b.y, b.psi, self.rect_l, self.rect_b)
                mc = np.minimum(mc, d.min(axis=1))

        last = self.last if math.isfinite(self.last) else INIT_LAST   # 이중 방어(NaN 비용 금지)
        best = None
        for i in range(k):
            if mc[i] < D_HARD_M:
                continue
            a = self.cands[i]
            head = -_wrap180(b.psi[i])
            err = abs(_wrap180(goal - head)) / 180.0
            soft = max(0.0, D_SOFT_M - mc[i]) / D_SOFT_M
            cost = err + W_CLEAR * soft + W_SMOOTH * abs(a - last) / CAND_MAX_DEG
            if best is None or cost < best[0]:
                best = (cost, float(a))
        return None if best is None else best[1]

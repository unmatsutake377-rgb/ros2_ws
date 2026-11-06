"""3자유도(전진·횡이동·선회) 선체 모델 — Gazebo 모델(sim/vrx make_boat.py)과 **같은 계수**를 쓴다.

full_course_sim.py `--dyn fossen` 이 쓴다. 기존 모델(`--dyn legacy`, K_V·K_W 1차 지연)은 추력 차 → 선회율이 즉시
비례하는 가정이라 관성·횡이동·감속이 없다. 이 모델은 VRX/gz Hydrodynamics 와 같은 Fossen 형식이다:

  m(u̇ − v r) = ΣT_i − (xU u + xUU |u| u)
  m(v̇ + u r) = −(yV v + yVV |v| v)
  Iz ṙ      = Σ(−y_i T_i) − (nR r + nRR |r| r)

좌표: 선체 x 앞, y **왼쪽**(+), r 반시계 양수(수학 규약). 나침반 헤딩 ψ(0=북, 시계)는 ψ̇ = −r.
추진기: T_i = pwm_to_thrust(PWM)(ROS 규약 <1500 전진), 1차 지연 τ_thr. 왼쪽 추진기(y>0)가 세면 −yT<0 → 시계 → 우선회.

🚨 계수(xU·xUU·nR·nRR …)와 PWM→추력은 **가정값**이다(make_boat.BOATS·convert.pwm_to_thrust 한 곳). 수조 실측으로 바꾼다.
"""
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SSF_VRX = os.path.abspath(os.path.join(_HERE, '..', '..', 'sim', 'vrx', 'src', 'ssf_vrx'))
if _SSF_VRX not in sys.path:
    sys.path.insert(0, _SSF_VRX)
from ssf_vrx.convert import pwm_to_thrust  # noqa: E402
from ssf_vrx.make_boat import BOATS         # noqa: E402


class Boat3DOF:
    """상태: x, y (m, 월드: x 동 y 북), psi (deg, 나침반), u 전진, v 좌측 횡속(m/s), r 반시계 선회율(rad/s)."""

    def __init__(self, boat='A', x=0.0, y=0.0, psi=0.0, thrust_tau=0.15, substeps=10):
        spec = BOATS[boat]
        self.m = float(spec['mass'])
        L, B = float(spec['loa']), float(spec['boa'])
        self.Iz = self.m / 12.0 * (L * L + B * B)
        self.h = dict(spec['hydro'])
        self.thrusters = [(name, float(tx), float(ty)) for name, tx, ty in spec['thrusters']]
        self.T = [0.0] * len(self.thrusters)          # 실제 추력(지연 반영)
        self.thrust_tau = float(thrust_tau)
        self.substeps = int(substeps)
        self.x, self.y, self.psi = float(x), float(y), float(psi)
        self.u = self.v = self.r = 0.0

    # ---- 명령 → 추진기별 목표 추력 ----
    def _targets(self, pwm_l, pwm_r):
        tl, tr = pwm_to_thrust(pwm_l), pwm_to_thrust(pwm_r)
        return [tl if ty > 0 else tr for _n, _tx, ty in self.thrusters]

    def step(self, pwm_l, pwm_r, dt, current=(0.0, 0.0)):
        tgt = self._targets(pwm_l, pwm_r)
        h, m, Iz = self.h, self.m, self.Iz
        sub = dt / self.substeps
        for _ in range(self.substeps):
            # 추력 1차 지연
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
            self.psi = (self.psi - math.degrees(self.r) * sub) % 360.0
            p = math.radians(self.psi)
            fx, fy = math.sin(p), math.cos(p)          # 전진 단위벡터(동, 북)
            lx, ly = -fy, fx                           # 왼쪽 단위벡터
            self.x += (self.u * fx + self.v * lx + current[0]) * sub
            self.y += (self.u * fy + self.v * ly + current[1]) * sub
        return self.x, self.y, self.psi

    @property
    def speed(self):
        return math.hypot(self.u, self.v)

    @property
    def yaw_rate_dps(self):
        """나침반 기준 선회율(°/s, 시계 양수)."""
        return -math.degrees(self.r)


def steady_state(boat='A', pwm_l=1360, pwm_r=1360, seconds=40.0, dt=0.1):
    """정상상태 속도·선회율 — 계수 점검용. (u m/s, 선회율 °/s)"""
    b = Boat3DOF(boat)
    for _ in range(int(seconds / dt)):
        b.step(pwm_l, pwm_r, dt)
    return b.u, b.yaw_rate_dps


if __name__ == '__main__':
    for boat in ('A', 'B'):
        u1, _ = steady_state(boat, 1360, 1360)
        u2, _ = steady_state(boat, 1100, 1100)
        _, w1 = steady_state(boat, 1260, 1460)      # 왼쪽 세게(1260) → 우선회
        print(f"{boat}: 1360/1360 → {u1:.2f} m/s, 전출력 → {u2:.2f} m/s, "
              f"좌 1260/우 1460 → {w1:+.1f} °/s (양수 = 우선회)")

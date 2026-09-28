import numpy as np, math
from ssf_sim import SCAN_ANGLE_MIN_DEG, SCAN_INC_DEG, SCAN_N

_A = SCAN_ANGLE_MIN_DEG + np.arange(SCAN_N) * SCAN_INC_DEG   # 스캔각
_REL = _A - 80.0                                             # 배 기준 상대각(CCW+)

class FastWorld:
    """벡터화된 LiDAR 광선투사 (원형 부표)."""
    def __init__(self, buoys):
        self.buoys = buoys
        if buoys:
            arr = np.array(buoys, dtype=float)
            self.bx, self.by, self.br = arr[:,0], arr[:,1], arr[:,2]
        else:
            self.bx = self.by = self.br = np.zeros(0)

    def lidar_scan(self, x, y, psi, noise_spike=None, max_range=20.0):
        n = len(self.bx)
        if n == 0:
            return np.full(SCAN_N, np.inf)
        wb = psi - _REL                       # 세계 나침반 방위
        th = np.radians(wb)
        dx, dy = np.sin(th), np.cos(th)       # (S,)
        ox = self.bx - x                      # (N,)
        oy = self.by - y
        t_ca = dx[:,None]*ox[None,:] + dy[:,None]*oy[None,:]      # (S,N)
        d2 = (ox**2 + oy**2)[None,:] - t_ca**2
        r2 = self.br**2
        hit = (d2 <= r2) & (t_ca > 0)
        t_hc = np.sqrt(np.maximum(r2[None,:] - d2, 0.0))
        t = np.where(hit, t_ca - t_hc, np.inf)
        rng = t.min(axis=1)
        rng[rng > max_range] = np.inf
        return rng

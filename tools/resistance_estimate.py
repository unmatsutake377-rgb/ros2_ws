#!/usr/bin/env python3
"""저항 추정 — 보고서 Ⅱ-3.4. 선도와 같은 STL 에서 배수량에 맞는 수선을 찾고, 침수 표면적 S·수선 길이 L_WL 로
ITTC-1957 마찰 저항에 형상계수를 곱한 점성 저항을 낸다. 조파 저항은 넣지 않는다(Fn 0.35 이상에서 급증 — 하한 추정).

  R_V = (1+k) · ½ρV²S · C_F,  C_F = 0.075 / (log10 Re − 2)²,  Re = V·L_WL/ν
  가정: 담수 ρ 1000 kg/m³, ν 1.0e-6 m²/s(20 °C 근처), 1+k = 1.2(소형 단동선 통상 범위 1.1~1.3 의 가운데)
  배수량: 1번배 34 kg, 2번배 30 kg (실측 11-03 저녁 확정 — 종전 1번배 30 추정 / 2번배 35 가정·25·34 는 폐기)
  추력: T200 제조사 사양 12 V 3.71 kgf · 16 V 5.25 kgf 를 14.8 V 로 선형 보간 ≈ 4.79 kgf

재현: python3 tools/resistance_estimate.py  → 표 값 출력 + docs/전달용/배치도/resistance_A.png, resistance_B.png
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hull_lines_plan as hl

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
D = os.path.join(ROOT, "docs", "전달용")
RHO, NU, G, K1 = 1000.0, 1.0e-6, 9.81, 1.2
T200_N = (3.71 + (5.25 - 3.71) * (14.8 - 12) / (16 - 12)) * G


def outer_tris(stl, **kw):
    by, _ = hl.load_stl(stl, n_st=3, **kw)        # 좌표 정리(x 0 = 트랜섬, y CL, z 바닥 0)만 빌린다
    import struct
    raw = open(stl, "rb").read(); n = struct.unpack("<I", raw[80:84])[0]
    return by


def sections(stl, n=120, **kw):
    """x 방향 n 단면: 각 단면의 외판 폴리라인(현측 상단선~현측 상단선)."""
    import struct
    raw = open(stl, "rb").read(); n_t = struct.unpack("<I", raw[80:84])[0]
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    T = np.frombuffer(raw, dt, count=n_t, offset=84)["v"].astype(float) * kw.get("scale", 1.0)
    if kw.get("fit"):
        f = kw["fit"]; T[:, :, 0] *= f[0] / np.ptp(T[:, :, 0]); T[:, :, 1] *= f[1] / np.ptp(T[:, :, 1])
    N = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    if kw.get("flip_z"):
        T[:, :, 2] = T[:, :, 2].max() - T[:, :, 2]; N[:, 2] *= -1; N *= -1
    T[:, :, 2] -= T[:, :, 2].min(); T[:, :, 0] -= T[:, :, 0].min()
    T[:, :, 1] -= (T[:, :, 1].min() + T[:, :, 1].max()) / 2
    L, H = T[:, :, 0].max(), T[:, :, 2].max()
    C = np.array([L / 2, 0.0, 1.2 * H]); out = (N * (C - T.mean(1))).sum(1) < 0
    up = N[:, 2] / (np.linalg.norm(N, axis=1) + 1e-12) > 0.95
    To = T[out & ~up] / 1000.0                                  # m
    xs = np.linspace(0.02 * L, 0.995 * L, n) / 1000.0
    secs = []
    for x in xs:
        ch = hl._chain(hl._slice(To, 0, x))
        secs.append(max(ch, key=len) if ch else None)
    return xs, secs


def below(sec, zw):
    """단면 폴리라인에서 수선 zw 아래 넓이·침수 거스. 메시가 중심선 이음매에서 반쪽씩 끊겨 나오는 경우가 있어
    (2번배 Fusion STL) 넓게 잡힌 쪽 반폭 w(z) 를 1 mm 간격으로 적분하고 좌우 대칭으로 두 배 한다."""
    if sec is None: return 0.0, 0.0
    y, z = sec[:, 1], sec[:, 2]
    side = 1.0 if y.max() >= -y.min() else -1.0
    yy = side * y
    zs = np.arange(z.min(), zw, 0.001)
    if len(zs) == 0: return 0.0, 0.0
    w = np.zeros_like(zs)
    for a in range(len(y) - 1):
        z0, z1 = z[a], z[a + 1]
        if yy[a] < 0 and yy[a + 1] < 0: continue
        lo, hi = min(z0, z1), max(z0, z1)
        m = (zs >= lo) & (zs <= hi)
        if not m.any(): continue
        t = np.where(hi > lo, (zs[m] - z0) / (z1 - z0 if z1 != z0 else 1), 0.0)
        w[m] = np.maximum(w[m], np.clip(yy[a] + t * (yy[a + 1] - yy[a]), 0, None))
    area = 2 * w.sum() * 0.001
    seg = np.diff(sec[:, 1:3], axis=0); mid = (sec[:-1] + sec[1:]) / 2
    keep = (mid[:, 2] < zw) & (side * mid[:, 1] >= 0)
    girth = 2 * np.hypot(seg[keep, 0], seg[keep, 1]).sum()
    return area, girth


def hydro(xs, secs, zw):
    A = np.array([below(s, zw) for s in secs])
    V = np.trapz(A[:, 0], xs); S = np.trapz(A[:, 1], xs)
    wet = xs[A[:, 0] > 1e-6]; Lwl = wet.max() - wet.min() if len(wet) else 0.0
    return V, S, Lwl


def waterline(xs, secs, vol):
    lo, hi = 0.0, 0.25
    for _ in range(40):
        m = (lo + hi) / 2
        lo, hi = (m, hi) if hydro(xs, secs, m)[0] < vol else (lo, m)
    return (lo + hi) / 2


def plot(key, name, S, Lwl, n_thr):
    """속도별 점성 저항 곡선 + 가용 추력선 + Fn 0.4 경계 → docs/전달용/배치도/resistance_<A|B>.png"""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"; matplotlib.rcParams["axes.unicode_minus"] = False
    v = np.linspace(0.2, 2.4, 100); Re = v * Lwl / NU; Cf = 0.075 / (np.log10(Re) - 2) ** 2
    Rv = K1 * 0.5 * RHO * v * v * S * Cf; vh = 0.40 * np.sqrt(G * Lwl); T = n_thr * T200_N
    fig, ax = plt.subplots(figsize=(8.6, 4.2))
    ax.semilogy(v, Rv, color="#1f77b4", lw=2.2, label="점성 저항 R_V (ITTC-1957 × 1.2)")
    ax.axhline(T, color="#d62728", lw=2, ls="--", label=f"가용 추력 T200 ×{n_thr} ≈ {T:.0f} N")
    ax.axvspan(vh, v.max(), color="#ff7f0e", alpha=0.12, label="Fn > 0.4 — 조파 저항 급증(계산 제외)")
    ax.axvline(vh, color="#ff7f0e", lw=1.2)
    ax.text(vh + 0.03, 0.25, f"Fn 0.4 ≈ {vh:.2f} m/s", color="#c05000", fontsize=9)
    for vv in (1.0, 1.5, 2.0):
        r = K1 * 0.5 * RHO * vv * vv * S * 0.075 / (np.log10(vv * Lwl / NU) - 2) ** 2
        ax.plot(vv, r, "o", color="#1f77b4"); ax.text(vv, r * 1.35, f"{r:.1f} N", ha="center", fontsize=8.5, color="#1f77b4")
    ax.set_xlabel("속도 V [m/s]"); ax.set_ylabel("힘 [N] (로그 눈금)"); ax.set_ylim(0.1, 400); ax.set_xlim(0.2, 2.4)
    ax.set_yticks([0.1, 1, 10, 100]); ax.set_yticklabels(["0.1", "1", "10", "100"]); ax.minorticks_off()
    ax.grid(alpha=.3, which="both"); ax.legend(fontsize=8.5, loc="center left")
    ax.set_title(f"{name} 속도별 점성 저항과 가용 추력 (S {S:.3f} m², L_WL {Lwl:.3f} m)", fontsize=11)
    fig.tight_layout(); out = os.path.join(D, "배치도", f"resistance_{key}.png"); fig.savefig(out, dpi=130); print(out)


def run(name, stl, mass, n_thr, key=None, **kw):
    xs, secs = sections(stl, **kw)
    zw = waterline(xs, secs, mass / RHO)
    V, S, Lwl = hydro(xs, secs, zw)
    if key: plot(key, name, S, Lwl, n_thr)
    print(f"\n{name}: 배수량 {mass} kg → 수선 z {zw*1000:.1f} mm, 침수 표면적 S {S:.3f} m², L_WL {Lwl:.3f} m, 선체속도(Fn 0.40) {0.40*np.sqrt(G*Lwl):.2f} m/s")
    print("  V[m/s]  Fn     Re        C_F      R_V[N]   (가용 추력 T200×%d = %.0f N)" % (n_thr, n_thr * T200_N))
    for v in (0.5, 1.0, 1.5, 2.0):
        Re = v * Lwl / NU; Cf = 0.075 / (np.log10(Re) - 2) ** 2; Rv = K1 * 0.5 * RHO * v * v * S * Cf
        print(f"  {v:4.1f}   {v/np.sqrt(G*Lwl):.2f}  {Re:.2e}  {Cf:.5f}  {Rv:6.1f}")


if __name__ == "__main__":
    run("1번배", os.path.join(D, "선도_1번배", "1번배_킬제거_Fusion360.stl"), 34.0, 2, key="A", scale=1.66, flip_z=True)
    run("2번배", os.path.join(D, "선도_2번배", "2번배_Fusion360.stl"), 30.0, 4, key="B", fit=(1770.0, 522.0))

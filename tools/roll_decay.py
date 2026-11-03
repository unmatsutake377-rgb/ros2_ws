#!/usr/bin/env python3
"""롤 감쇠 — 수조 시험 phyphox 'Inclination' 저장 상태 파일에서 놓은 뒤 감쇠를 읽는다 (보고서 Ⅱ-3.2).

입력: docs/전달용/수조시험/<배>_롤감쇠_phyphox_inclination.phyphox (phyphox 1.14 saved state, tiltFlatLR = 롤, 0.5 s 간격)
한계: Inclination 은 0.5 s 샘플링이라 주기 ~1 s 는 에일리어싱(매 샘플 부호 반전) → 주기는 '약 1 s' 이상 못 쪼갬.
      감쇠(진폭 포락선)는 샘플 간격과 무관하게 읽힌다. 보고서에는 이 한계를 함께 적는다.
방법: 기준 = 처음 10 s 중앙값. 놓은 시각 = 누름 구간(|편차|>4°, 5 s 이상) 끝. 놓은 뒤 12 s 의 |편차| 국부 최대값에
      ln|θ| = a − t/τ 를 직선 맞춤 → 시간상수 τ, 10°→1° 시간 = τ·ln10, 대수감쇠율 δ = T/τ (T = 1.0 s 가정).
재현: python3 tools/roll_decay.py A  → 수치 출력 + docs/전달용/배치도/roll_decay_A.png
"""
import os, re, sys, statistics as st
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"; matplotlib.rcParams["axes.unicode_minus"] = False
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."); D = os.path.join(ROOT, "docs", "전달용")
NAME = {"A": "1번배", "B": "2번배"}

def load(path):
    s = open(path, encoding="utf-8").read()
    def arr(n):
        m = re.search(r'<container[^>]*init="([^"]*)"[^>]*>' + n + '</container>', s)
        return np.array([float(x) for x in m.group(1).split(",")])
    return arr("t"), arr("tiltFlatLR")

def releases(t, dev, thr=4.0, hold=1.0):
    """누름 구간(같은 부호로 |편차|>thr 가 hold 초 이상) 의 마지막 샘플 시각 = 놓은 시각."""
    big = np.abs(dev) > thr; out = []; i = 0
    while i < len(t):
        if big[i]:
            j = i
            while j + 1 < len(t) and big[j + 1] and np.sign(dev[j + 1]) == np.sign(dev[i]): j += 1
            if t[j] - t[i] >= hold: out.append(t[j])
            i = j + 1
        else: i += 1
    return out

def analyze(key):
    t, lr = load(os.path.join(D, "수조시험", f"{NAME[key]}_롤감쇠_phyphox_inclination.phyphox"))
    base = st.median(lr[t < 10]); dev = lr - base
    rel = releases(t, dev); res = []
    fig, axes = plt.subplots(1, len(rel), figsize=(4.2 * len(rel), 3.6), sharey=True)
    for ax, t0 in zip(np.atleast_1d(axes), rel):
        m = (t > t0) & (t <= t0 + 12); tt = t[m] - t0; a = np.abs(dev[m])
        # 0.5 s 샘플링은 위상이 제멋대로라 2 s 창의 |편차| 최대값을 그 창의 진폭으로 본다
        tp, ap = [], []
        for w0 in np.arange(0, 10, 2.0):
            mm = (tt >= w0) & (tt < w0 + 2)
            if mm.any() and a[mm].max() > 0.3: tp.append(w0 + 1.0); ap.append(a[mm].max())
        tp, ap = np.array(tp), np.array(ap)
        slope, icpt = np.polyfit(tp, np.log(ap), 1); tau = -1 / slope
        res.append(dict(t0=t0, a0=a[:2].max(), tau=tau, t10=tau * np.log(10)))
        ax.plot(tt, dev[m], ".-", lw=0.8, ms=4, color="#1f77b4", label="롤 편차 (0.5 s 샘플)")
        ax.plot(tp, ap, "o", ms=4, color="#d62728", label="2 s 창 최대 |편차|")
        xs = np.linspace(0, 12, 50); ax.plot(xs, np.exp(icpt + slope * xs), "--", color="#d62728", lw=1.2, label=f"포락선 τ = {tau:.1f} s")
        ax.plot(xs, -np.exp(icpt + slope * xs), "--", color="#d62728", lw=1.2)
        ax.axhline(0, color="#888", lw=0.6); ax.set_xlabel("놓은 뒤 시간 [s]"); ax.set_title(f"{len(res)}회차 (놓음 {t0:.1f} s)", fontsize=10)
        ax.grid(alpha=.3)
    np.atleast_1d(axes)[0].set_ylabel("롤 각 편차 [°]"); np.atleast_1d(axes)[0].legend(fontsize=7.5, loc="lower right")
    fig.suptitle(f"{NAME[key]} 롤 감쇠 — 폰 기울기계(phyphox Inclination, 0.5 s 간격), 10° 안팎 눌렀다 놓음 ×{len(rel)}", fontsize=11)
    fig.tight_layout(); out = os.path.join(D, "배치도", f"roll_decay_{key}.png"); fig.savefig(out, dpi=130); plt.close(fig)
    tau = st.mean(r["tau"] for r in res)
    print(f"{NAME[key]}: 기준 {base:.2f}°, 놓은 시각 {[round(r['t0'],1) for r in res]}, 초기 진폭 {[round(r['a0'],1) for r in res]}°")
    for k, r in enumerate(res, 1): print(f"  {k}회: τ {r['tau']:.2f} s, 10°→1° {r['t10']:.1f} s")
    print(f"  평균 τ {tau:.2f} s, 10°→1° {tau*np.log(10):.1f} s, 대수감쇠율 δ(T=1.0 s 가정) {1.0/tau:.2f}, 감쇠비 ζ≈δ/2π {1.0/tau/2/np.pi:.3f}")
    print(out)

if __name__ == "__main__":
    for k in (sys.argv[1:] or ["A"]): analyze(k)

#!/usr/bin/env python3
"""그림 Ⅲ-3 갭 선택 개념도 — ship_direction._compute 의 회피 파이프라인을 같은 순서·같은 파라미터로 재현.

    python3 tools/gap_selection_figure.py   → docs/전달용/배치도/gap_selection.png

가상 장면(설명용, 실측 아님): 부표 2개 + 물보라 한 점. 파라미터는 ship_direction.yaml 대회값
(detection 3.0 m, half_width 0.29, clearance 0.25, max_spike_ratio 0.01, border_margin 2, min_obstacle_cells 1),
각분해능 0.25°(1번배 개체 80BF). 단계 함수는 ship_direction.py 의 smooth_spikes / suppress_spike_edges /
dilate_obstacles / select_valid_zones / clamp 를 그대로 옮겼다(노드는 rclpy 를 import 해 Mac 에서 불러올 수 없음).
"""
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "전달용", "배치도", "gap_selection.png")

# ---- 대회 파라미터 (ship_direction.yaml) ----
DET, HW, CL = 3.0, 0.29, 0.25
SPIKE_RATIO, BORDER, MIN_CELLS = 0.01, 2, 1
INC = 0.25                      # deg, 80BF
ANG = [i * INC for i in range(int(160 / INC) + 1)]   # LiDAR 각 0~160 (80 = 정면)
N = len(ANG)
MIN_W = HW * 2 + CL             # 0.83 m


def scene():
    """부표(중심 각·거리·지름) 두 개 + 물보라 한 점. 나머지는 탐지거리 밖(inf)."""
    r = [math.inf] * N
    buoys = [(60.0, 2.5, 0.40), (100.0, 2.0, 0.40)]
    for ca, d, dia in buoys:
        half = math.degrees(math.atan((dia / 2) / d))
        for i, a in enumerate(ANG):
            if abs(a - ca) <= half:
                r[i] = d
    r[ANG.index(30.0)] = 0.45   # 물보라 반사 한 점
    return r, buoys


def mask(r):
    return [1 if (not math.isinf(v) and not math.isnan(v) and v < DET) else 0 for v in r]


def smooth_spikes(b):
    mx = int(len(b) * SPIKE_RATIO); out = b[:]; cnt = 0; st = None
    for i, v in enumerate(b):
        if v == 1:
            st = i if st is None else st; cnt += 1
        else:
            if st is not None and cnt <= mx:
                for j in range(st, i):
                    out[j] = 0
            st = None; cnt = 0
    if st is not None and cnt <= mx:
        for j in range(st, len(b)):
            out[j] = 0
    return out


def suppress_edges(b):
    out = b[:]
    for i, v in enumerate(b):
        if v == 1:
            for off in range(-BORDER, BORDER + 1):
                j = i + off
                if 0 <= j < len(b) and b[j] == 0:
                    out[i] = 0
                    break
    return out


def dilate(b, r):
    n = len(b); out = b[:]; i = 0
    while i < n:
        if b[i] == 1:
            s = i
            while i + 1 < n and b[i + 1] == 1:
                i += 1
            e = i
            if e - s + 1 >= MIN_CELLS:
                re = min([DET] + [r[k] for k in range(s, e + 1) if not math.isinf(r[k])])
                m = max(1, int(round(math.degrees(math.atan((HW + CL) / max(re, 0.01))) / INC)))
                for j in range(max(0, s - m), min(n - 1, e + m) + 1):
                    out[j] = 1
        i += 1
    return out


def zones(b):
    z = []; st = None
    for i, v in enumerate(b):
        if v == 0 and st is None:
            st = i
        elif v == 1 and st is not None:
            z.append((st, i - 1)); st = None
    if st is not None:
        z.append((st, len(b) - 1))
    return z


def edge(r, i):
    return DET if math.isinf(r[i]) else r[i]


def main():
    r, buoys = scene()
    b0 = mask(r); b1 = smooth_spikes(b0); b2 = suppress_edges(b1); b3 = dilate(b2, r)
    zs = zones(b3)
    inc_rad = math.radians(INC)
    info = []
    for s, e in zs:
        re = min(edge(r, s), edge(r, e)); arc = inc_rad * re * (e - s)
        info.append((s, e, re, arc, arc >= MIN_W))
    target = 96.0                       # 목표 방위(LiDAR 각) — 일부러 부표 쪽
    cands = []
    for s, e, re, arc, ok in info:
        if not ok:
            continue
        lo, hi = ANG[s], ANG[e]
        best = min(max(target, lo), hi)
        cands.append((abs(best - target), -arc, best))
    cands.sort()
    chosen = cands[0][2] if cands else None

    fig = plt.figure(figsize=(13, 9.2))
    gs = fig.add_gridspec(5, 2, width_ratios=[1.05, 1.6], hspace=0.55, wspace=0.18)
    # 제목은 보고서 캡션이 맡는다(그림 Ⅲ-3) — 그림 안에 넣지 않는다

    # (a) 위에서 본 장면 — 80° 를 위로
    ax = fig.add_subplot(gs[:, 0])
    def xy(a, d):
        t = math.radians(a - 80 + 90)
        return d * math.cos(t), d * math.sin(t)
    for a in (0, 80, 160):
        x, y = xy(a, 3.3); ax.plot([0, x], [0, y], color="#bbb", lw=0.8, ls=":")
        ax.text(*xy(a, 3.45), f"{a:.0f}°", ha="center", va="center", fontsize=8, color="#777")
    th = [xy(a, DET) for a in ANG]; ax.plot([p[0] for p in th], [p[1] for p in th], color="#999", lw=1, ls="--")
    ax.text(*xy(150, DET + 0.12), "탐지 3.0 m", fontsize=8, color="#777")
    pts = [xy(a, d) for a, d in zip(ANG, r) if not math.isinf(d)]
    ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=4, color="#c0392b", zorder=3, label="스캔 점")
    for s, e, re, arc, ok in info:
        col = "#27ae60" if ok else "#e67e22"
        arcp = [xy(ANG[k], re) for k in range(s, e + 1)]
        ax.plot([p[0] for p in arcp], [p[1] for p in arcp], color=col, lw=4, alpha=0.6)
    if chosen is not None:
        x, y = xy(chosen, 2.9); ax.annotate("", xy=(x, y), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color="#2471a3", lw=2.4))
        ax.text(x, y + 0.1, f"조향 {chosen:.1f}°", color="#2471a3", fontsize=9, ha="center", weight="bold")
    xt, yt = xy(target, 2.4); ax.plot([0, xt], [0, yt], color="#2471a3", lw=1, ls="--")
    ax.text(xt, yt - 0.18, f"목표 {target:.0f}°", color="#2471a3", fontsize=8, ha="center")
    ax.add_patch(plt.Rectangle((-HW, -0.5), HW * 2, 0.5, color="#555"))
    ax.text(0, -0.75, "배 (폭 2×half_width)", ha="center", fontsize=8)
    ax.set_aspect("equal"); ax.set_xlim(-3.6, 3.6); ax.set_ylim(-1.0, 3.7); ax.axis("off")
    ax.set_title("(a) 위에서 본 장면 · 초록 = 통과 가능 구간, 주황 = 폭 부족", fontsize=10, loc="left")

    rows = [(b0, "(b) 이진 마스크: r < 3.0 m → 장애물"),
            (b1, "(c) 스파이크 제거: 길이 ≤ int(n×0.01) 셀 구간 삭제 — 물보라 한 점 사라짐"),
            (b2, "(d) 가장자리 정리: 경계 ±2 셀 안의 장애물 셀 제거"),
            (b3, "(e) 팽창: m = round(atan((0.29+0.25)/r_min)/Δθ) 셀씩 양옆")]
    for k, (bb, title) in enumerate(rows):
        a2 = fig.add_subplot(gs[k, 1])
        a2.fill_between(ANG, 0, bb, step="mid", color="#c0392b", alpha=0.75, lw=0)
        a2.set_xlim(160, 0); a2.set_ylim(0, 1.2); a2.set_yticks([]); a2.axvline(80, color="#888", lw=0.8, ls=":")
        a2.set_title(title.replace("−", "-"), fontsize=9.5, loc="left"); a2.tick_params(labelsize=8)
    a3 = fig.add_subplot(gs[4, 1])
    a3.set_xlim(160, 0); a3.set_ylim(0, 1); a3.set_yticks([]); a3.axvline(80, color="#888", lw=0.8, ls=":")
    for s, e, re, arc, ok in info:
        col = "#27ae60" if ok else "#e67e22"
        a3.axvspan(ANG[s], ANG[e], color=col, alpha=0.35)
        a3.text((ANG[s] + ANG[e]) / 2, 0.55, f"호 {arc:.2f} m\n{'≥' if ok else '<'} 0.83", ha="center", va="center", fontsize=8)
    a3.axvline(target, color="#2471a3", lw=1, ls="--")
    if chosen is not None:
        a3.axvline(chosen, color="#2471a3", lw=2.4)
    a3.set_title("(f) 폭 판정 arc = Δθ·r_edge·(e-s) ≥ half_width×2 + clearance(0.83 m) → 목표를 구간 안으로 clamp, 가장 가까운 각", fontsize=9.5, loc="left")
    a3.set_xlabel("LiDAR 각 [°]  (80 = 정면)", fontsize=9); a3.tick_params(labelsize=8)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, dpi=150, bbox_inches="tight")
    print(os.path.abspath(OUT))
    for s, e, re, arc, ok in info:
        print(f"구간 {ANG[s]:6.2f}~{ANG[e]:6.2f}°  r_edge {re:.2f}  호 {arc:.2f} m  {'통과' if ok else '부족'}")
    print("목표", target, "→ 조향", chosen)


if __name__ == "__main__":
    main()

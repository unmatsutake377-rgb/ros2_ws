#!/usr/bin/env python3
"""회피 파라미터 스윕 그래프 — 보고서 Ⅲ-6.1. tools/sim/out/sweep_s1.csv (sweep_s1.py 결과, 2,400 런)를
half_width × clearance 칸마다 80 런(시드 20 × 탐지거리 2 × 게이트폭 2) 평균으로 그린다.
정체 = 구간 미도달(fails > 0) 런 비율.
재현: python3 tools/sim/sweep_figure.py  → docs/전달용/배치도/sim_sweep.png
"""
import csv, collections, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"; matplotlib.rcParams["axes.unicode_minus"] = False
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "out", "sweep_s1.csv")
OUT = os.path.join(HERE, "..", "..", "docs", "전달용", "배치도", "sim_sweep.png")
CHOSEN = (0.29, 0.25)   # ship_direction.yaml half_width · clearance

g = collections.defaultdict(list)
for r in csv.DictReader(open(SRC)):
    g[(float(r["half_w"]), float(r["clr"]))].append(r)
HW = sorted({k[0] for k in g}); CL = sorted({k[1] for k in g})
hits = {k: sum(int(r["hits"]) for r in v) / len(v) for k, v in g.items()}
stall = {k: 100 * sum(int(r["fails"]) > 0 for r in v) / len(v) for k, v in g.items()}

fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.6))
cols = plt.cm.viridis([i / (len(HW) - 1) for i in range(len(HW))])
for h, col in zip(HW, cols):
    lw = 2.6 if h == CHOSEN[0] else 1.3
    a1.plot(CL, [hits[(h, c)] for c in CL], "o-", color=col, lw=lw, label=f"선폭 {2*h:.2f} m (half_width {h})")
    a2.plot(CL, [stall[(h, c)] for c in CL], "o-", color=col, lw=lw)
a1.plot(CHOSEN[1], hits[CHOSEN], "*", color="#d62728", ms=18, zorder=5, label=f"채택 half_width {CHOSEN[0]} · clearance {CHOSEN[1]}")
a2.plot(CHOSEN[1], stall[CHOSEN], "*", color="#d62728", ms=18, zorder=5)
a1.set_xlabel("clearance [m]"); a1.set_ylabel("부표 접촉 [회 / 런]"); a1.set_title("(a) 평균 접촉 횟수", fontsize=11)
a2.set_xlabel("clearance [m]"); a2.set_ylabel("정체(구간 미도달) 런 비율 [%]"); a2.set_title("(b) 정체 비율 — 틈 요구가 너무 크면 길을 못 찾는다", fontsize=11)
for a in (a1, a2): a.grid(alpha=.3); a.set_xticks(CL)
a1.legend(fontsize=8.5, loc="upper right")
fig.suptitle(f"회피 파라미터 스윕 — {len(HW)} 선폭 × {len(CL)} clearance × 80 런 = {sum(len(v) for v in g.values()):,} 런 (시뮬레이션)", fontsize=12)
fig.tight_layout(); fig.savefig(OUT, dpi=130)
print(OUT, "채택", hits[CHOSEN], stall[CHOSEN])

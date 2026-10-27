#!/usr/bin/env python3
"""허브용 강압 컨버터(SZWENGAO WG-1224S0505) 리드 4가닥 결선 그림.
재현: python3 tools/hub_converter_wiring.py -> docs/절차/그림/전원분기_5_허브컨버터결선.png
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle

matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "절차", "그림", "전원분기_5_허브컨버터결선.png")
RED, BLK, YEL, ORG, GRY, BLU = "#c0392b", "#222", "#d4a017", "#b8860b", "#666", "#2471a3"


def box(ax, x, y, w, h, t, fc, ec="#333", fs=9.5, bold=False):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                                fc=fc, ec=ec, lw=1.4))
    ax.text(x, y, t, ha="center", va="center", fontsize=fs, weight="bold" if bold else "normal", linespacing=1.35)


def wire(ax, pts, c, lw=3.0):
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color=c, lw=lw, solid_capstyle="round", zorder=2)


def lbl(ax, x, y, t, c="#333", fs=8.5, ha="center", bold=False):
    ax.text(x, y, t, ha=ha, va="center", fontsize=fs, color=c, weight="bold" if bold else "normal",
            bbox=dict(fc="white", ec="none", pad=1.2), zorder=4)


fig, ax = plt.subplots(figsize=(13, 7.4))
ax.set_xlim(0, 13); ax.set_ylim(0, 7.4); ax.axis("off")
ax.set_title("그림 5 — 허브 컨버터 SZWENGAO WG-1224S0505 결선 : 입력 2가닥은 배터리 A 쪽, 출력 2가닥은 허브 플러그 쪽",
             fontsize=12.5, loc="left", weight="bold")

# ── 왼쪽: 배터리 쪽 (입력)
box(ax, 1.5, 5.6, 2.3, 1.1, "배터리 A ⊕\n접촉기 5번 둘째 나사\n(링단자) 또는 본선 납땜", "#fdecea", fs=8.8)
box(ax, 1.5, 2.2, 2.3, 1.1, "배터리 A ⊖\n기존 승압 컨버터 IN⊖\n납땜점 (ESC 우 ⊖ 선)", "#eeeeee", fs=8.8)
ax.add_patch(Rectangle((3.35, 5.42), 1.3, 0.36, fc="#fff7d6", ec=ORG, lw=1.6))
ax.text(4.0, 5.6, "BF303 + 3 A", ha="center", va="center", fontsize=9, color=ORG, weight="bold")
lbl(ax, 4.0, 6.05, '"3A 허브" 유성펜', ORG, 8)

# ── 가운데: 컨버터
ax.add_patch(FancyBboxPatch((5.4, 1.4), 2.4, 4.9, boxstyle="round,pad=0.02,rounding_size=0.12",
                            fc="#2b2b2b", ec="#111", lw=1.5))
ax.text(6.6, 5.85, "SZWENGAO\nWG-1224S0505", ha="center", va="center", fontsize=10, color="white", weight="bold")
ax.text(6.6, 3.85, "IN 8~36 V\n→\nOUT 5 V 5 A\n(IP68, 강압)", ha="center", va="center", fontsize=9.5, color="#ddd", linespacing=1.5)
for y, t in ((5.6, "IN +"), (2.2, "IN -")):
    ax.add_patch(Circle((5.4, y), 0.09, fc="white", ec="k", zorder=3)); ax.text(5.6, y, t, color="white", fontsize=8.5, va="center")
for y, t in ((5.6, "OUT +"), (2.2, "OUT -")):
    ax.add_patch(Circle((7.8, y), 0.09, fc="white", ec="k", zorder=3)); ax.text(7.6, y, t, color="white", fontsize=8.5, va="center", ha="right")
lbl(ax, 6.6, 1.05, "라벨의 Input / Output 인쇄로 확인 — 색만 믿지 말 것", RED, 8.5, bold=True)

# ── 입력 선
wire(ax, [(2.65, 5.6), (3.35, 5.6)], RED); wire(ax, [(4.65, 5.6), (5.31, 5.6)], RED)
lbl(ax, 3.0, 5.25, "분기 18AWG 빨강", RED, 8)
lbl(ax, 5.0, 5.95, "빨강 리드", RED, 8)
wire(ax, [(2.65, 2.2), (5.31, 2.2)], BLK)
lbl(ax, 3.9, 2.55, "분기 18AWG 검정 (퓨즈 없음)", BLK, 8)
lbl(ax, 5.0, 1.85, "검정 리드 ①", BLK, 8)

# ── 출력 선 → 플러그
wire(ax, [(7.89, 5.6), (9.6, 5.6), (9.6, 4.25), (10.05, 4.25)], YEL)
wire(ax, [(7.89, 2.2), (9.6, 2.2), (9.6, 3.55), (10.05, 3.55)], BLK)
lbl(ax, 8.6, 5.95, "노랑 리드", YEL, 8, bold=True)
lbl(ax, 8.6, 1.85, "검정 리드 ②", BLK, 8)
# 플러그 단면
ax.add_patch(Rectangle((10.05, 3.35), 1.05, 1.1, fc="#ddd", ec="k", lw=1.2))
ax.add_patch(Rectangle((11.1, 3.7), 0.95, 0.4, fc="#bbb", ec="k", lw=1.2))
ax.add_patch(Rectangle((11.1, 3.85), 0.95, 0.1, fc=RED, ec="none"))
lbl(ax, 10.6, 4.75, "DC 플러그 5.5×2.1\n(UH308 동봉 어댑터 선 재활용)", "#333", 8.3)
lbl(ax, 12.15, 4.35, "중심 핀 ⊕", RED, 8.5, ha="left", bold=True)
lbl(ax, 12.15, 3.45, "바깥 통 ⊖", BLK, 8.5, ha="left", bold=True)
box(ax, 11.6, 2.0, 1.9, 0.75, "ipTIME UH308\nDC IN 5 V 잭", "#eef3ff", fs=8.8, bold=True)
ax.annotate("", xy=(11.6, 2.4), xytext=(11.6, 3.33), arrowprops=dict(arrowstyle="-|>", color=BLU, lw=2.0))
lbl(ax, 11.95, 2.85, "꽂음", BLU, 8, ha="left")

# ── 하단 규칙
rules = ("① 검정이 두 가닥(IN-, OUT-)이다. 납땜 전에 각각 테이프로 \"IN-\" \"OUT-\" 표시.  "
         "② 어댑터 선 두 가닥 중 플러그 중심 핀과 도통되는 쪽 = ⊕ → 노랑(OUT+). 다른 쪽 → 검정(OUT-).\n"
         "③ 허브에 꽂기 「전」 테스터로 플러그 중심-바깥 4.9~5.1 V, 중심이 ⊕ 인지.  "
         "④ 입력 극성 반대 = 모듈 사망.  ⑤ 승압 컨버터 SZH-BPM001(24 V)과 혼동 금지 — 허브에 24 V 들어가면 즉사.")
ax.add_patch(FancyBboxPatch((0.25, 0.05), 12.5, 0.75, boxstyle="round,pad=0.02,rounding_size=0.05", fc="#fff7d6", ec=ORG, lw=1.2))
ax.text(0.4, 0.43, rules, fontsize=8.6, va="center", linespacing=1.6)

fig.tight_layout()
os.makedirs(os.path.dirname(OUT), exist_ok=True)
fig.savefig(OUT, dpi=150)
print(os.path.abspath(OUT))

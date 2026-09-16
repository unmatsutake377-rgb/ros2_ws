#!/usr/bin/env python3
"""GPS 안테나 위치 설명도 — docs/전달용/배치도/gps_antenna_position.png
근거: docs/전달용/하드웨어_배치_요구사항.md §2-1 (u-blox 매뉴얼: 접지판 Ø100~150, 하늘 가림, RF 이격)."""
import math
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon, Wedge
matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "전달용", "배치도", "gps_antenna_position.png")
L, W = 1660, 580
BX0, BX1 = 600, 1160          # 박스(=해치) 앞뒤 (선미 기준)
MX, LX = 1230, 1470
GX = 535                      # [O] 안테나 (09-16: 케이블 짧아 해치 바로 뒤로. 접지판 Ø120)
DECK = 240                    # 건현 가정
MAST_TOP = DECK + 220

fig, (a1, a2) = plt.subplots(2, 1, figsize=(15, 12), gridspec_kw={"height_ratios": [1, 1.1]})

# ───────── 위: 측면 — 마스트 가림 고도각 비교 ─────────
a1.set_title("① 왜 해치 뒤인가 — 마스트가 가리는 하늘 (측면, mm)", fontsize=13, loc="left")
a1.add_patch(Rectangle((0, DECK - 280), L, 280, fc="#dbe6f1", ec="k"))
a1.add_patch(Rectangle((0, DECK - 6), L, 6, fc="#8fa8c8", ec="k"))
a1.add_patch(Rectangle((BX0, DECK - 6), BX1 - BX0, 14, fc="#f0c419", ec="#b8860b")); a1.text((BX0 + BX1) / 2, DECK + 22, "해치(=박스 뚜껑)", ha="center", fontsize=10)
a1.add_patch(Rectangle((MX - 20, DECK), 40, 220, fc="#aaa", ec="k")); a1.text(MX, MAST_TOP + 15, "마스트 220", ha="center", fontsize=9)
a1.add_patch(Rectangle((LX - 38, DECK), 76, 40, fc="#9fd89f", ec="k"))
a1.axhline(0, color="#3b7dd8", ls="--", lw=1); a1.text(20, -30, "수면", color="#3b7dd8", fontsize=9)
# [O] 후보: 해치 뒤 500
for gx, col, tag in ((GX, "#2a9d2a", "[O] 해치 뒤 x=535"), (1190, "#d33", "[X] 해치 앞 x=1190")):
    a1.add_patch(Rectangle((gx - 60, DECK), 120, 4, fc="#777", ec="k"))
    a1.add_patch(Circle((gx, DECK + 20), 18, fc="#eee", ec=col, lw=2))
    dx = (MX - 20) - gx
    ang = math.degrees(math.atan2(220, dx))
    a1.plot([gx, MX - 20], [DECK + 20, MAST_TOP], color=col, ls="--", lw=1.5)
    a1.add_patch(Wedge((gx, DECK + 20), 160, 0, ang, fc=col, alpha=0.18))
    a1.text(gx, DECK + 60 if gx == GX else DECK + 260, f"{tag}\n마스트까지 {dx/10:.0f}cm → 가림 고도각 {ang:.0f}°", ha="center", fontsize=9, color=col)
a1.text(GX, DECK - 60, "접지판 Ø120 (강판 or 알루미늄+나사), 그로밋 x≈515 바로 뒤", ha="center", fontsize=8, color="#444")
a1.set_xlim(-60, L + 60); a1.set_ylim(-80, MAST_TOP + 90); a1.set_aspect("equal"); a1.grid(alpha=0.2); a1.set_xlabel("선미 → 선수 (mm)")

# ───────── 아래: 평면 — 후보 비교 + 이격 ─────────
a2.set_title("② 평면 — 후보 비교, RF 이격, 해치 경첩 (1번 배 166×58cm)", fontsize=13, loc="left")
hull = Polygon([[0, -W/2], [1150, -W/2], [1450, -W/2 + 120], [1620, -80], [1660, 0], [1620, 80], [1450, W/2 - 120], [1150, W/2], [0, W/2]], closed=True, fc="#dbe6f1", ec="k", lw=1.5)
a2.add_patch(hull)
a2.add_patch(Rectangle((BX0, -190), BX1 - BX0, 380, fc="#f0c419", ec="#b8860b", alpha=0.7)); a2.text((BX0 + BX1) / 2, 60, "해치 (600~1160)", ha="center", fontsize=10)
a2.plot([BX1, BX1], [-190, 190], color="#b8860b", lw=5); a2.text(BX1 - 120, -240, "경첩 = 앞쪽(마스트 쪽) [O]\n뒤쪽이면 열릴 때 안테나 덮음 [X]", fontsize=8, color="#b8860b")
a2.add_patch(Rectangle((MX - 80, -60), 160, 120, fc="#888", ec="k")); a2.text(MX + 60, -100, "마스트 베이스 1150~1310\n(해치 1160 과 겹침 → 마스트 ≥1270 또는 해치 ≤1120)", ha="center", fontsize=8)
a2.add_patch(Circle((LX, 0), 38, fc="#9fd89f", ec="k")); a2.text(LX, 60, "LiDAR", ha="center", fontsize=8)
# [O] GPS
a2.add_patch(Circle((GX, 0), 60, fc="#bbb", ec="k", alpha=0.6)); a2.add_patch(Circle((GX - 20, -60), 6, fc="#333")); a2.text(GX - 20, -95, "그로밋 → 박스 뒷벽 수신기", ha="center", fontsize=7); a2.add_patch(Rectangle((GX - 30, -41), 60, 82, fc="#eee", ec="#2a9d2a", lw=2))
a2.text(GX, 120, "[O] GPS 안테나 x=535 중심선\n접지판 Ø120 (앞끝 595 ↔ 해치 600) 케이블 최단", ha="center", fontsize=9, color="#2a9d2a")
a2.annotate("", xy=(GX, -60), xytext=(GX, -W/2 + 20), arrowprops=dict(arrowstyle="-", color="#2a9d2a", ls=":"))
# [X] 후보들
a2.add_patch(Rectangle((1165, -41), 60, 82, fc="none", ec="#d33", lw=1.5, ls="--")); a2.text(1195, -150, "[X] 해치 앞\n마스트 기둥 2cm → 85° 가림", ha="center", fontsize=8, color="#d33")
a2.add_patch(Rectangle((850, W/2 - 105), 82, 60, fc="none", ec="#d33", lw=1.5, ls="--")); a2.text(890, W/2 - 140, "[X] 해치 옆: 옆갑판 10cm < 안테나 60×82\n(2번 배 43cm 폭은 아예 불가)", ha="center", fontsize=8, color="#d33")
a2.add_patch(Rectangle((60, -41), 60, 82, fc="none", ec="#d33", lw=1.5, ls="--")); a2.text(90, -110, "[X] 선미 끝\nRC/LTE 자리, CG 와 50cm+", ha="center", fontsize=8, color="#d33")
# 이격
a2.add_patch(Rectangle((20, W/2 - 90), 60, 40, fc="#ccc", ec="k")); a2.text(50, W/2 - 120, "LTE 안테나(A트랙 시)\n선미 모서리", ha="center", fontsize=8)
d_lte = math.hypot(GX - 50, 0 - (W/2 - 70)); a2.plot([GX, 50], [0, W/2 - 70], color="#888", ls=":"); a2.text((GX + 50) / 2 - 40, (W/2 - 70) / 2 + 30, f"{d_lte/10:.0f}cm ≥ 50 [O]", fontsize=8, color="#555")
a2.add_patch(Rectangle((160, -W/2 + 50), 60, 40, fc="#ccc", ec="k")); a2.text(190, -W/2 + 28, "RC 수신기 (반대 현)", ha="center", fontsize=8)
# CG
a2.plot([700, 700], [-60, 60], color="k", lw=1); a2.text(700, -90, "CG ≈700 (추정)\n안테나와 16cm → LOS 오프셋 무시", ha="center", fontsize=8)
a2.set_xlim(-60, L + 60); a2.set_ylim(-W/2 - 90, W/2 + 90); a2.set_aspect("equal"); a2.grid(alpha=0.2); a2.set_xlabel("선미 → 선수 (mm)")

plt.tight_layout()
os.makedirs(os.path.dirname(OUT), exist_ok=True)
plt.savefig(OUT, dpi=110)
print(os.path.abspath(OUT))

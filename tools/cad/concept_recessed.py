#!/usr/bin/env python3
"""concept_recessed.py — 오픈 헐 위에 갑판을 덮고, 방수박스는 갑판 아래(선체 안), 케이블은 박스 그랜드 → 갑판 관통 → 마스트. 전부 추정치."""
import glob, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt, matplotlib.font_manager as fm
from matplotlib.patches import Rectangle, Polygon, Circle, FancyArrowPatch
for f in glob.glob("/usr/share/fonts/**/NotoSansCJK*.ttc", recursive=True): fm.fontManager.addfont(f)
plt.rcParams["font.family"] = ["Noto Sans CJK KR", "Noto Sans CJK JP", "DejaVu Sans"]; plt.rcParams["axes.unicode_minus"] = False

L, DEPTH, FREEBOARD = 1700, 180, 100         # 2번 배. 흘수 80 가정 → 갑판 수면 +100
BOX_X, BOX_W, BOX_H = 600, 550, 160          # 박스: 갑판 아래 선체 안. 뚜껑 위로 해치
MAST_X, MAST_H, TUBE = BOX_X + BOX_W + 90, 150, 40   # 마스트: 박스 앞쪽 갑판 위
Z_DECK = FREEBOARD
Z_OAK, Z_D455 = Z_DECK + 8 + 95, Z_DECK + 8 + MAST_H + 40   # 마스트 상한 220: OAK 아래끝 ≈ 갑판+6cm, LiDAR 는 갑판 직치(회전면 갑판+3cm)
LIDAR_X = 1420

fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(2, 1, 1)
# 선체 단면
ax.add_patch(Polygon([(0, -80), (L, -80), (L, FREEBOARD), (0, FREEBOARD)], closed=True, fc="#dfe7ee", ec="k", lw=1.5))
ax.add_patch(Rectangle((0, FREEBOARD - 10), L - 60, 10, fc="#b7c6d3", ec="k"))   # 갑판판 (거널 위에 덮음)
ax.add_patch(Rectangle((BOX_X + 40, FREEBOARD - 10), BOX_W - 80, 10, fc="#e2b007", ec="k"))    # 해치 (박스 뚜껑 접근)
ax.text(BOX_X + 60, FREEBOARD + 12, "갑판 해치 (노트북 접근)", fontsize=9)
# 마스트 밑 가로대 (갑판 아래, 양현 거널 사이) — 갑판판 휨 방지
ax.add_patch(Rectangle((MAST_X - 60, FREEBOARD - 40), 120, 30, fc="#8c8c8c", ec="k"))
ax.text(MAST_X + 80, FREEBOARD - 30, "가로대(알루미늄 평철) — 갑판 아래, 볼트가 이걸 물고 지나감", fontsize=9)
ax.axhline(0, color="#3a7bd5", ls="--", lw=1); ax.text(L + 15, 8, "수면", color="#3a7bd5")
# 박스 (선체 안, 뚜껑 플러시)
ax.add_patch(Rectangle((BOX_X, FREEBOARD - BOX_H), BOX_W, BOX_H, fc="#fff2cc", ec="k"))
ax.add_patch(Rectangle((BOX_X - 10, FREEBOARD - 10 - 14), BOX_W + 20, 12, fc="#ffe699", ec="k"))  # 박스 뚜껑 (갑판 바로 아래)
ax.text(BOX_X + 20, FREEBOARD - 80, "방수박스 (갑판 아래)\n노트북 · 배터리 · 허브 · IMU")
# 뚜껑 그랜드
gx = BOX_X + BOX_W - 60
gx = BOX_X + BOX_W - 40
ax.add_patch(Rectangle((gx - 10, FREEBOARD - 60), 20, 30, fc="#444", ec="k"))                 # 박스 옆면 그랜드
ax.add_patch(Circle((gx + 60, FREEBOARD - 5), 8, fc="#444", ec="k"))                          # 갑판 관통 그로밋
ax.text(gx - 560, FREEBOARD + 60, "박스 옆면 그랜드 → 갑판 그로밋 (구멍 1개) → 마스트 베이스", fontsize=9)
# 마스트 베이스 + 관
ax.add_patch(Rectangle((MAST_X - 80, FREEBOARD), 160, 8, fc="#c9c9c9", ec="k"))
for bx in (MAST_X - 65, MAST_X + 65):
    ax.add_patch(Rectangle((bx - 3, FREEBOARD - 30), 6, 40, fc="#333"))  # 볼트 (갑판 관통)
ax.add_patch(Rectangle((MAST_X - TUBE / 2, FREEBOARD + 8), TUBE, MAST_H, fc="#c9c9c9", ec="k"))
ax.text(MAST_X + 80, FREEBOARD - 90, "M5 볼트 4개 → 갑판 + 가로대 관통", fontsize=9)
# 케이블 경로
ax.add_patch(FancyArrowPatch((gx, FREEBOARD - 45), (gx + 60, FREEBOARD - 5), connectionstyle="arc3,rad=-0.3", arrowstyle="->", color="#c00000", lw=1.5)); ax.add_patch(FancyArrowPatch((gx + 60, FREEBOARD + 2), (MAST_X, FREEBOARD + 20), connectionstyle="arc3,rad=-0.3", arrowstyle="->", color="#c00000", lw=1.5))

# OAK, D455
ax.add_patch(Rectangle((MAST_X + TUBE / 2 + 6, Z_OAK - 41), 31, 82, fc="#f4b183", ec="k"))
ax.text(MAST_X - 460, Z_OAK + 90, f"OAK (비전) 아래 7°, 롤 0\n수면 ≈ {Z_OAK / 10:.0f} cm (아래끝 LiDAR 면 위)", ha="left")
ax.add_patch(Rectangle((MAST_X - 62, Z_D455 - 40), 124, 29, fc="#9dc3e6", ec="k"))
ax.text(MAST_X + 75, Z_D455 + 20, f"D455 (조종 화면) 아래 15°\n수면 ≈ {(Z_D455 - 40) / 10:.0f} cm  (마스트 갑판 위 22cm)")
# LiDAR (선수, 짧은 기둥)
ax.add_patch(Rectangle((LIDAR_X - 38, FREEBOARD + 2), 76, 41, fc="#a9d18e", ec="k"))
ax.text(LIDAR_X + 50, FREEBOARD + 30, f"LiDAR (갑판 직치)\n회전면 수면 ≈ {(FREEBOARD + 30) / 10:.0f} cm", fontsize=9)
# GPS (뚜껑 위)
ax.add_patch(Rectangle((BOX_X + 120, FREEBOARD + 2), 60, 23, fc="#ffd966", ec="k")); ax.text(BOX_X - 330, FREEBOARD + 60, "GPS 안테나 (뚜껑 위, 접지판)", fontsize=9)
ax.add_patch(Rectangle((BOX_X + 260, FREEBOARD - BOX_H + 10), 30, 30, fc="#c00000", ec="k")); ax.text(BOX_X + 300, FREEBOARD - BOX_H + 12, "IMU (박스 바닥)", color="#c00000", fontsize=9)
ax.add_patch(Rectangle((-60, -70), 60, 40, fc="#666")); ax.text(-60, -130, "T200")
ax.set_xlim(-120, L + 150); ax.set_ylim(-200, 520); ax.set_aspect("equal"); ax.set_xlabel("mm")
ax.set_title("구상 — 갑판 덮은 오픈 헐, 박스는 갑판 아래, 케이블은 박스→갑판→마스트 (2번 배, 흘수 8cm 가정)")

# 베이스 상세 (평면)
ax2 = fig.add_subplot(2, 1, 2)
ax2.add_patch(Rectangle((-100, -70), 200, 140, fc="#f5f5f5", ec="k", ls="--"))          # 갑판 영역
ax2.add_patch(Rectangle((-80, -60), 160, 120, fc="#c9c9c9", ec="k"))                     # 베이스
for x in (-65, 65):
    for y in (-45, 45):
        ax2.add_patch(Circle((x, y), 2.8, fc="w", ec="k"))
ax2.add_patch(Rectangle((-20, -20), 40, 40, fc="#999", ec="k"))                          # 관
ax2.add_patch(Circle((0, 0), 11, fc="w", ec="k"))                                        # 케이블 구멍
ax2.text(-95, 80, "베이스 160×120×8 — M5 ×4 갑판 관통, 중앙 Ø22 케이블 구멍", fontsize=10)
ax2.add_patch(Rectangle((-100, -230), 200, 130, fc="#e2b007", ec="k")); ax2.text(-95, -120, "갑판 해치 (아래가 박스)", fontsize=9)
ax2.add_patch(Circle((60, -85), 8, fc="#444", ec="k")); ax2.text(75, -90, "갑판 그로밋", fontsize=9)
ax2.add_patch(FancyArrowPatch((0, -12), (60, -78), connectionstyle="arc3,rad=-0.3", arrowstyle="->", color="#c00000", lw=1.5))
ax2.annotate("", xy=(120, 60), xytext=(120, -60), arrowprops=dict(arrowstyle="<->")); ax2.text(125, -5, "120", fontsize=9)
ax2.annotate("", xy=(-80, 100), xytext=(80, 100), arrowprops=dict(arrowstyle="<->")); ax2.text(-10, 105, "160", fontsize=9)
ax2.text(-95, -260, "↑ 뱃머리(+Y) 쪽에 카메라. 베이스와 해치 사이 ≥ 30mm — 해치 여닫을 때 안 걸리게. 베이스 밑에는 가로대", fontsize=9)
ax2.set_xlim(-140, 190); ax2.set_ylim(-280, 130); ax2.set_aspect("equal"); ax2.set_xlabel("mm"); ax2.set_title("베이스 고정 상세 (위에서 본 것)")
plt.tight_layout()
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "concept_recessed.png"); plt.savefig(out, dpi=130); print(out)

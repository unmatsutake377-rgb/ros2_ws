import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon, FancyArrowPatch
matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False

OUT = "/Users/leesongeon/ros2_ws/docs/전달용/배치도/mast_recommend_1beonbae.png"

fig, (ax, ax2) = plt.subplots(2, 1, figsize=(16, 13), gridspec_kw={"height_ratios": [1.15, 1]})

# ---------------- 측면도 (1번 배 166cm, 깊이 28, 건현 24 가정, 수면 y=0) ----------------
L = 1660; FB = 240; DEPTH = 280; DECK = FB
ax.set_title("1번 배 — 마스트·센서 배치 추천 (측면, mm). 실선=확정  점선/?=미확정", fontsize=14, loc="left")
# 선체
ax.add_patch(Rectangle((0, FB - DEPTH), L, DEPTH, fc="#dbe6f1", ec="k", lw=1.5))
ax.axhline(0, color="#3b7dd8", ls="--", lw=1); ax.text(60, -22, "수면 (건현 24cm 가정 → 진수 때 실측)", color="#3b7dd8", va="top", fontsize=10)
# 갑판
ax.add_patch(Rectangle((0, DECK - 6), L, 6, fc="#8fa8c8", ec="k", lw=1)); ax.text(60, DECK + 12, "갑판(FRP) — 두께 ? 실측", fontsize=10)
# T200
ax.add_patch(Rectangle((-50, -70), 50, 40, fc="#555")); ax.text(-50, -95, "T200", fontsize=9)
# 방수박스 (갑판 아래) + 뚜껑=해치
BX0, BX1 = 600, 1160
ax.add_patch(Rectangle((BX0, FB - DEPTH + 20), BX1 - BX0, DECK - 20 - (FB - DEPTH + 20), fc="#fff2cc", ec="#b8860b", lw=1.5))
ax.add_patch(Rectangle((BX0, DECK - 6), BX1 - BX0, 14, fc="#f0c419", ec="#b8860b", lw=1.5))
ax.text((BX0 + BX1) / 2, FB - DEPTH + 60, "방수박스 (갑판 아래)\n노트북·배터리·허브·인젝터 PSE5502G·MD-30A", ha="center", fontsize=10)
ax.text(BX0 + 40, DECK + 34, "박스 뚜껑 = 해치 (갑판 면 노출)[확정]", ha="left", fontsize=10, color="#7a5c00")
# 비상버튼 on lid
ax.add_patch(Circle((1100, DECK + 14), 14, fc="red", ec="k")); ax.add_patch(Rectangle((1090, DECK + 6), 20, 8, fc="#333"))
ax.text(1100, DECK + 62, "비상버튼\n(뚜껑 위)[확정]", ha="center", fontsize=9, color="red")
# IMU
# GPS 안테나 옆 고정 갑판
GX = 480
ax.add_patch(Rectangle((GX - 60, DECK), 120, 4, fc="#999", ec="k"))
ax.add_patch(Circle((GX, DECK + 22), 20, fc="#eee", ec="k")); ax.text(GX, DECK + 56, "GPS 안테나[확정]\n뚜껑 옆 고정 갑판 + 접지판Ø10cm+", ha="center", fontsize=9)
# 마스트
MX = 1230
ax.add_patch(Rectangle((MX - 80, DECK), 160, 8, fc="#888", ec="k"))  # base
ax.add_patch(Rectangle((MX - 20, DECK + 8), 40, 150, fc="#aaa", ec="k"))  # column
ax.add_patch(Rectangle((MX - 45, DECK + 158), 90, 12, fc="#7aa7e8", ec="k")); ax.text(MX + 130, DECK + 175, "D455 (조종) 아래 15°", fontsize=9)
ax.add_patch(Rectangle((MX + 20, DECK + 60), 31, 82, fc="#f4a261", ec="k")); ax.text(MX + 130, DECK + 120, "OAK-1 PoE 아래 7°, 롤 0\n뒷면 M4×4, 아래끝 갑판+60", fontsize=9, va="top")
ax.text(MX - 60, DECK + 250, "마스트 ≤220mm 프린트(PETG)[확정]\nx≈1230 ? (박스 자리 확정 후)\n갑판 관통 M5×4 + 가로대", ha="center", fontsize=9)
ax.annotate("", xy=(MX + 110, DECK), xytext=(MX + 110, DECK + 220), arrowprops=dict(arrowstyle="<->", color="k")); ax.text(MX + 116, DECK + 40, "220", fontsize=9)
# IMU (마스트 베이스 포켓)
ax.add_patch(Rectangle((MX - 70, DECK + 1), 40, 14, fc="#c00", ec="k")); ax.text(MX - 90, DECK + 22, "IMU 베이스 블록 포켓[확정]\n굵은 배선과 30cm+, |m| 확인", ha="right", fontsize=8, color="#c00")
# 표시등
ax.add_patch(Rectangle((MX - 30, DECK + 100), 8, 40, fc="#2ecc71", ec="k")); ax.text(MX - 34, DECK + 145, "표시등\n마스트 뒤", ha="right", fontsize=8)
# LiDAR
LX = 1470
ax.add_patch(Rectangle((LX - 38, DECK), 76, 40, fc="#9fd89f", ec="k"))
ax.plot([LX - 300, LX + 150], [DECK + 30, DECK + 30], color="#2a8a2a", lw=1, ls=":")
ax.text(LX + 130, DECK + 30, "LiDAR A3 갑판 직치[확정]\n회전면 갑판+30 = 수면 +270 ?\n슬롯 ±100 → 수조 5m 검출로 확정", ha="left", va="center", fontsize=9)
ax.annotate("", xy=(LX + 90, DECK - 60), xytext=(LX + 90, DECK + 40), arrowprops=dict(arrowstyle="<->", color="#2a8a2a", ls="--"))
ax.text(LX + 100, DECK - 80, "±100 슬롯 ?", fontsize=8, color="#2a8a2a")
# 부표 가정
ax.add_patch(Circle((L + 420, 0), 150, fc="#ffb347", ec="k", alpha=0.6)); ax.text(L + 420, 175, "둥근 부표 가정\n노출 15~30cm ?", ha="center", fontsize=9)
ax.add_patch(Polygon([[L + 680, -40], [L + 800, -40], [L + 750, 480], [L + 730, 480]], fc="#e74c3c", ec="k", alpha=0.6)); ax.text(L + 740, 500, "라바콘 게이트\n45~70cm ?", ha="center", fontsize=9)
# 케이블
ax.plot([MX, MX, MX - 60, BX1 + 10], [DECK + 8, DECK - 40, DECK - 70, DECK - 70], color="#c0392b", lw=2)
ax.plot([BX1 + 10, BX1 + 10, BX1], [DECK - 70, DECK - 95, DECK - 95], color="#c0392b", lw=2)
ax.text(1180, -110, "케이블(빨강): 베이스 Ø22 → 갑판 밑 → 박스 앞면 구멍(위쪽)\n실란트 마감 + 드립 루프[확정]  (Cat5e·D455 USB·LiDAR USB 한 다발)", ha="center", va="top", fontsize=9, color="#c0392b")
ax.plot([LX, LX, BX1 + 10], [DECK, DECK - 50, DECK - 50], color="#c0392b", lw=1, ls="--")
ax.set_xlim(-120, L + 860); ax.set_ylim(-160, 560); ax.set_aspect("equal"); ax.set_xlabel("선미 → 선수 (mm)"); ax.grid(alpha=0.2)

# ---------------- 평면도 ----------------
W = 580
ax2.set_title("1번 배 — 평면 (mm). 마스트·LiDAR 중심선, 박스 자리는 회로 작업 때 확정", fontsize=14, loc="left")
hull = Polygon([[0, -W/2], [1150, -W/2], [1450, -W/2 + 120], [1620, -80], [1660, 0], [1620, 80], [1450, W/2 - 120], [1150, W/2], [0, W/2]], closed=True, fc="#dbe6f1", ec="k", lw=1.5)
ax2.add_patch(hull)
ax2.add_patch(Rectangle((BX0, -190), BX1 - BX0, 380, fc="#f0c419", ec="#b8860b", lw=1.5, alpha=0.7)); ax2.text((BX0 + BX1) / 2, 0, "방수박스 뚜껑 = 해치\n(비상버튼 뚜껑 위)", ha="center", va="center", fontsize=10)
ax2.add_patch(Circle((1100, 130), 14, fc="red", ec="k"))
ax2.add_patch(Rectangle((GX - 40, W/2 - 120), 80, 60, fc="#ddd", ec="k")); ax2.text(GX, W/2 - 30, "GPS 안테나\n(뚜껑 옆 갑판)", ha="center", fontsize=9)
ax2.add_patch(Rectangle((MX - 80, -60), 160, 120, fc="#888", ec="k")); ax2.text(MX, -100, "마스트 베이스 160×120\n해치와 ≥30 이격 ?", ha="center", fontsize=9)
for yy in (-W/2 + 40, W/2 - 40): ax2.plot([MX - 40, MX + 40], [yy, yy], color="#555", lw=6, alpha=0.5)
ax2.text(MX, W/2 - 5, "가로대(알루미늄 평철, 갑판 밑) ×2", ha="center", fontsize=8)
ax2.add_patch(Circle((LX, 0), 38, fc="#9fd89f", ec="k")); ax2.text(LX, 70, "LiDAR A3\n선수 판 내폭 ? 실측", ha="center", fontsize=9)
ax2.add_patch(Rectangle((160, W/2 - 90), 60, 40, fc="#ccc", ec="k")); ax2.text(190, W/2 - 120, "LTE 라우터(A트랙 시)", ha="center", fontsize=8)
ax2.add_patch(Rectangle((160, -W/2 + 50), 60, 40, fc="#ccc", ec="k")); ax2.text(190, -W/2 + 30, "RC 수신기 (반대 현, ≥50cm)", ha="center", fontsize=8)
ax2.set_xlim(-60, L + 60); ax2.set_ylim(-W/2 - 60, W/2 + 60); ax2.set_aspect("equal"); ax2.set_xlabel("선미 → 선수 (mm)"); ax2.grid(alpha=0.2)

plt.tight_layout()
import os; os.makedirs(os.path.dirname(OUT), exist_ok=True)
plt.savefig(OUT, dpi=110)
print(OUT)

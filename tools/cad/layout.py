#!/usr/bin/env python3
"""layout.py — 센서·마스트 배치도 (측면 + 평면). 숫자는 전부 '대략치·실측 전' — 회로팀 박스 도면 나오면 갱신.

배: 1850×620mm, 전고 1m 이하(규정). 좌표 원점 = 선미, 수면.
"""
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.font_manager as fm  # noqa: E402
from matplotlib.patches import Rectangle, Circle, Polygon  # noqa: E402

for f in glob.glob("/usr/share/fonts/**/NotoSansCJK*.ttc", recursive=True):
    fm.fontManager.addfont(f)
plt.rcParams["font.family"] = ["Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic", "AppleGothic", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

L, W = 1700, 430          # 2번 배 (좁은 쪽 = 최악 조건). 1번 배는 1660×580
BOX_X, BOX_W, BOX_H, FREEBOARD = 650, 550, 200, 90      # ⚠️ 박스 위치·크기는 추정. 건현: 깊이 18cm 중 흘수 ~9cm 가정
MAST_X = BOX_X + BOX_W - 120
MAST_H = 264                                           # mast.py 출력값 (MAST_H=220 + 바닥판·캡)
Z_DECK = FREEBOARD + BOX_H                             # 박스 뚜껑 높이
Z_OAK, Z_D455 = Z_DECK + 99, Z_DECK + MAST_H
LIDAR_X, GPS_X, IMU_X = 1400, 850, 875

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 9.5))

# ── 측면
ax1.add_patch(Polygon([(0, -90), (L, -90), (L, FREEBOARD), (0, FREEBOARD)], closed=True, fc="#dfe7ee", ec="k"))
ax1.axhline(0, color="#3a7bd5", lw=1, ls="--"); ax1.text(L + 20, 12, "수면", color="#3a7bd5")
ax1.axhline(1000, color="r", ls=":", lw=1); ax1.text(20, 1012, "규정 전고 1m", color="r")
ax1.add_patch(Rectangle((BOX_X, FREEBOARD), BOX_W, BOX_H, fc="#fff2cc", ec="k"))
ax1.text(BOX_X + 20, FREEBOARD + BOX_H - 45, "노트북·배터리 방수박스 (⚠️ 크기 추정)")
ax1.add_patch(Rectangle((MAST_X - 20, Z_DECK), 40, MAST_H, fc="#c9c9c9", ec="k"))
ax1.text(MAST_X - 720, Z_DECK + 380, "마스트 (3D 출력, 220mm 한 마디)\n케이블: 베이스 구멍 → 박스 안", ha="left")
ax1.add_patch(Rectangle((MAST_X + 20, Z_OAK - 41), 31, 82, fc="#f4b183", ec="k"))
ax1.text(MAST_X + 65, Z_OAK - 20, f"OAK-1 W PoE (비전)\n아래 7°, 롤 0\n수면 ≈ {Z_OAK / 10:.0f} cm")
ax1.add_patch(Rectangle((MAST_X - 62, Z_D455), 124, 29, fc="#9dc3e6", ec="k"))
ax1.text(MAST_X + 75, Z_D455 + 60, f"D455 (조종 화면)\n아래 15°, 뱃머리가 화면 아래에\n수면 ≈ {Z_D455 / 10:.0f} cm")
ax1.add_patch(Rectangle((LIDAR_X - 15, FREEBOARD), 30, 180, fc="#bbb", ec="k"))
ax1.add_patch(Rectangle((LIDAR_X - 38, FREEBOARD + 180), 76, 41, fc="#a9d18e", ec="k"))
ax1.text(LIDAR_X + 60, FREEBOARD + 60, f"RPLiDAR A3\n수면 ≈ {(FREEBOARD + 200) / 10:.0f} cm\n같은 높이에 아무것도 없게")
ax1.add_patch(Rectangle((GPS_X - 30, Z_DECK), 60, 23, fc="#ffd966", ec="k"))
ax1.text(GPS_X - 620, Z_DECK + 120, "GPS 안테나: 박스 뚜껑, 금속 접지판\n(마스트 꼭대기 X — 롤 레버암 오차)")
ax1.add_patch(Rectangle((IMU_X - 15, FREEBOARD + 20), 30, 30, fc="#c00000", ec="k"))
ax1.text(BOX_X - 680, -60, "IMU: 박스 안 바닥, 무게중심 근처, 선체축 정렬\n모터·ESC·전원선에서 30cm+ (자기 헤딩)", color="#c00000")
ax1.add_patch(Rectangle((-60, -80), 60, 40, fc="#666")); ax1.text(-60, -150, "T200 ×4")
ax1.set_xlim(-150, L + 200); ax1.set_ylim(-250, 1080); ax1.set_aspect("equal")
ax1.set_title("측면 — 2번 배(170×43cm, 깊이 18cm) 기준, 선미(왼쪽)→선수(오른쪽), mm")

# ── 평면
ax2.add_patch(Polygon([(0, -W / 2), (L - 250, -W / 2), (L, 0), (L - 250, W / 2), (0, W / 2)], closed=True, fc="#dfe7ee", ec="k"))
ax2.add_patch(Rectangle((BOX_X, -150), BOX_W, 300, fc="#fff2cc", ec="k")); ax2.text(BOX_X + 20, -135, "방수박스")
ax2.add_patch(Rectangle((MAST_X - 20, -20), 40, 40, fc="#c9c9c9", ec="k")); ax2.text(MAST_X - 45, 55, "마스트")
ax2.add_patch(Circle((LIDAR_X, 0), 38, fc="#a9d18e", ec="k")); ax2.text(LIDAR_X - 45, 60, "LiDAR")
ax2.add_patch(Rectangle((GPS_X - 30, -40), 60, 80, fc="#ffd966", ec="k")); ax2.text(GPS_X - 35, 60, "GPS")
ax2.add_patch(Rectangle((IMU_X - 15, -15), 30, 30, fc="#c00000", ec="k")); ax2.text(IMU_X - 20, -70, "IMU", color="#c00000")
ax2.add_patch(Rectangle((300, -W / 2 - 10), 60, 40, fc="#d9d9d9", ec="k")); ax2.text(120, -W / 2 - 95, "RC 수신기 (선미 한쪽, 안테나 높이)")
ax2.add_patch(Rectangle((300, W / 2 - 30), 60, 40, fc="#d9d9d9", ec="k")); ax2.text(120, W / 2 + 45, "LTE 라우터/안테나 (반대쪽, RC와 50cm+)")
ax2.set_xlim(-150, L + 200); ax2.set_ylim(-420, 420); ax2.set_aspect("equal")
ax2.set_title("평면 — 중심선 위: 마스트·LiDAR·GPS·IMU / 무선기기는 양현으로 분리")
for a in (ax1, ax2):
    a.set_xlabel("mm")
plt.tight_layout()
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "layout.png")
plt.savefig(out, dpi=130)
print(out)

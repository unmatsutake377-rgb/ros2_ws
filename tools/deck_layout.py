#!/usr/bin/env python3
"""갑판 배치도 — 보고서 그림 Ⅳ-3 (hull_plan_A/B.png). 위에서 본 실제 선형 외곽 위에 장비 자리.

선형 외곽은 추진기 배치 그림과 같은 함수(thruster_layout.plan_outline)·같은 STL:
  1번배 킬 제거 STL ×1.66 / 2번배 Fusion STL 을 실측 1770×522 에 맞춤.
장비 x(선미 트랜섬 0 → 선수) 는 docs/전달용/하드웨어_배치_요구사항.md §2-1·§2-5 채택값.
  방수박스 400×300 (1번배 790~1190, 2번배 626~1026) · GPS 안테나 x 535 중심선(접지판 Ø120)
  팀장 11-03: 박스 뒤쪽 갑판을 뚫고 3D 프린팅 박스를 뒤로 덧붙여 연장 — 1번배 +80 (710~1190).
  2번배(팀장 11-03): 박스 앞면은 선수 끝에서 600 → x 1170, 본체 770~1170 + 연장 135 → 635~1170.
    마스트 1284, LiDAR 1460 (§2-5 제약 사슬 권장안, 팀장 11-03 채택). GPS 535 그대로(연장부 앞 40 mm).
  마스트 베이스 168×160 (1번배 1294, 2번배 1130) · LiDAR A3 Ø76 (1번배 1470, 2번배 1420 — 2번배는 미확정)
팀장 11-01: 마스트 가로대 없이 갑판에 직접 고정, RC 수신기·LTE 핫스팟 폰은 방수박스 안.
  폰은 GPS 안테나·RC 수신기에서 멀리 — 박스 앞쪽 좌현 모서리 (부품_추천목록 §9-2 RF 이격).
재현: python3 tools/deck_layout.py   →  docs/전달용/배치도/hull_plan_A.png, hull_plan_B.png
"""
import os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"; matplotlib.rcParams["axes.unicode_minus"] = False
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
DOCS = os.path.join(ROOT, "docs", "전달용")
OUTD = os.path.join(DOCS, "배치도")

# thruster_layout.py 는 import 하면 그림을 전부 다시 그리므로 plan_outline 만 떼어 쓴다
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import hull_lines_plan as hl


def plan_outline(stl, **kw):   # tools/thruster_layout.py plan_outline 과 같은 처리
    by, _ = hl.load_stl(stl, **kw)
    a, b = sorted(by["HULL_SHEER"], key=lambda p: -p[:, 1].mean())
    a = a[np.argsort(a[:, 0])]; b = b[np.argsort(b[:, 0])]
    out = []
    for q in (a, b):
        y = np.abs(q[:, 1]); y = np.array([y[max(0, i - 5):i + 6].max() for i in range(len(y))])
        y = np.convolve(np.pad(y, 5, mode="edge"), np.ones(11) / 11, mode="valid")
        q[:, 1] = np.sign(q[:, 1].mean()) * y
        if kw.get("fit"):   # 성긴 Fusion 메시(2번배): 선도 반폭도와 같은 45 mm 매듭 pchip 평활
            xs_, ys_ = hl._knot_smooth(q[:, 0], np.abs(q[:, 1]))
            q = np.column_stack([xs_, np.sign(q[:, 1].mean()) * ys_, np.full_like(xs_, q[:, 2].mean())])
        out.append(q)
    a, b = out
    return np.vstack([[0, a[0, 1]], a[:, :2], b[::-1, :2], [0, b[0, 1]]])


BOATS = {
    "A": dict(title="1번배 갑판 배치 (LOA 1660 × B 550 mm)", L=1660., B=550.,
              poly=lambda: plan_outline(os.path.join(DOCS, "선도_1번배", "1번배_킬제거_Fusion360.stl"), scale=1.66, flip_z=True),
              box=(790., 1190.), ext=80., gps=535., gps_note="", mast=1294., lidar=1470., lidar_note="", mast_note=""),
    "B": dict(title="2번배 갑판 배치 (LOA 1770 × B 522 mm)", L=1770., B=522.,
              poly=lambda: plan_outline(os.path.join(DOCS, "선도_2번배", "2번배_Fusion360.stl"), fit=(1770., 522.)),
              box=(770., 1170.), ext=135., gps=535., gps_note="", mast=1284., lidar=1460., lidar_note="", mast_note=""),
}
BOX_W = 300.; GPS_R = 60.; MAST_L, MAST_W = 168., 160.; LIDAR_R = 38.
PHONE_L, PHONE_W = 150., 75.   # 공기계 폰 개념 크기


def draw(k):
    c = BOATS[k]
    fig, ax = plt.subplots(figsize=(14, 4.6))
    ax.add_patch(Polygon(c["poly"](), closed=True, fc="#dfe7ef", ec="k", lw=1.3))
    x0, x1 = c["box"]; e = c["ext"]; xe = x0 - e; GPS_X = c["gps"]
    ax.add_patch(Rectangle((x0, -BOX_W / 2), x1 - x0, BOX_W, fc="#fff1cc", ec="k", lw=1.2))
    # 3D 프린팅 연장부 — 기존 박스 뒤쪽 갑판을 뚫고 덧붙임 (팀장 11-03)
    ax.add_patch(Rectangle((xe, -BOX_W / 2), e, BOX_W, fc="#ffe0a3", ec="k", lw=1.2, hatch="///"))
    ax.text(xe + e / 2, -BOX_W / 2 - 12, f"3D 프린팅\n연장부 +{e:.0f}", fontsize=8.3, ha="center", va="top")
    ax.text(xe, BOX_W / 2 + 8, f"방수박스 400×300 + 뒤쪽 연장 {e:.0f} → {400 + e:.0f}×300 (선체 안, 뚜껑 = 해치)", fontsize=9.5, va="bottom")
    ax.text(x0 + 12, BOX_W / 2 - 16, "노트북 · 배터리 ×2\n접촉기 · 허브\nArduino · ESC\nGPS 수신기", fontsize=8.3, va="top", color="#555", linespacing=1.4)
    # RC 수신기 — 박스 안
    ax.add_patch(Rectangle((x0 + 30, -BOX_W / 2 + 25), 47, 32, fc="#d9d9d9", ec="k", lw=1.0))
    ax.text(x0 + 88, -BOX_W / 2 + 41, "RC 수신기 (박스 안)", fontsize=9, va="center")
    # LTE 핫스팟 폰 — 박스 안 앞쪽 좌현 모서리 (GPS 안테나·RC 수신기와 최대한 떨어지게)
    px = x1 - 20 - PHONE_L; py = BOX_W / 2 - 20 - PHONE_W
    ax.add_patch(Rectangle((px, py), PHONE_L, PHONE_W, fc="#cfe3f7", ec="k", lw=1.1))
    ax.text(px + PHONE_L / 2, py + PHONE_W / 2, "LTE 핫스팟 폰\n(박스 안)", fontsize=8.5, ha="center", va="center")
    ax.text(x0 + 12, -BOX_W / 2 + 70, f"폰 ↔ GPS 안테나 약 {px - GPS_X:.0f} mm", fontsize=8.3, color="#555")
    # GPS 안테나 — 중심선, 접지판
    ax.add_patch(Circle((GPS_X, 0), GPS_R, fc="#ffd966", ec="k", lw=1.1))
    ax.text(GPS_X, GPS_R + 14, "GPS 안테나\n(접지판 Ø120)", fontsize=9, ha="center", va="bottom")
    if c["gps_note"]:
        ax.text(GPS_X, -GPS_R - 14, c["gps_note"].strip(), fontsize=8.3, ha="center", va="top", color="#a00")
    # 마스트 베이스 — 가로대 없이 갑판에 직접
    m = c["mast"]
    ax.add_patch(Rectangle((m - MAST_L / 2, -MAST_W / 2), MAST_L, MAST_W, fc="#c8c8c8", ec="k", lw=1.1))
    ax.add_patch(Rectangle((m - 20, -20), 40, 40, fc="#777", ec="k", lw=1.0))
    ax.text(m, -MAST_W / 2 - 14, "마스트\n(OAK 위 · D455 아래)\n베이스 갑판 직접 고정" + c["mast_note"], fontsize=9, ha="center", va="top")
    # LiDAR
    ax.add_patch(Circle((c["lidar"], 0), LIDAR_R, fc="#a9d18e", ec="k", lw=1.1))
    ax.text(c["lidar"], LIDAR_R + 14, "LiDAR A3" + c["lidar_note"], fontsize=9, ha="center", va="bottom")
    # 좌·우현 표시
    ax.text(40, c["B"] / 2 + 40, "좌현", fontsize=9, color="#666"); ax.text(40, -c["B"] / 2 - 40, "우현", fontsize=9, color="#666", va="top")
    ax.annotate("", xy=(c["L"] + 40, 0), xytext=(c["L"] - 60, 0), arrowprops=dict(arrowstyle="-|>", color="#666"))
    ax.text(c["L"] + 50, 0, "선수", fontsize=9, color="#666", va="center")
    ax.set_xlim(-60, c["L"] + 130); ax.set_ylim(-c["B"] / 2 - 110, c["B"] / 2 + 110)
    ax.set_aspect("equal"); ax.set_xlabel("선미 트랜섬 → 선수 (mm)"); ax.set_title(c["title"], fontsize=12)
    out = os.path.join(OUTD, f"hull_plan_{k}.png")
    fig.savefig(out, dpi=130, bbox_inches="tight"); plt.close(fig)
    print(out)


if __name__ == "__main__":
    for k in ("A", "B"):
        draw(k)

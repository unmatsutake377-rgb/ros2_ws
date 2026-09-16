#!/usr/bin/env python3
"""mast_preview.py — tools/cad/out/mast_*.stl 를 3면으로 렌더. docs/전달용/배치도/mast_v3_preview.png
Fusion 없이 형상을 눈으로 확인하는 용도(정밀 판정은 STEP 을 Fusion 에서)."""
import os, struct, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False
HERE = os.path.dirname(os.path.abspath(__file__))
OUTD = os.path.join(HERE, "out")
PNG = os.path.join(HERE, "..", "..", "docs", "전달용", "배치도", "mast_v3_preview.png")

def load(fn):
    d = open(fn, 'rb').read()
    if d[:5] == b'solid' and b'facet' in d[:300]:
        v = [list(map(float, l.split()[1:4])) for l in d.decode(errors='ignore').splitlines() if l.split()[:1] == ['vertex']]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack('<I', d[80:84])[0]
    a = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].reshape(-1, 3, 3).astype(float)

# 배치 높이 (mast.py 와 같은 규칙)
BASE_T, DECK_T, SEG, COLLAR_H = 20.0, 4.0, 122.0, 14.0
PARTS = [("mast_base.stl", (0, 0, 0), "#b9b9b9"),
         ("mast_seg1.stl", (0, 0, BASE_T + DECK_T), "#9fb8d4"),
         ("mast_cap.stl",  (0, 0, BASE_T + DECK_T + SEG + DECK_T), "#c9c9c9"),
         ("mast_imu_lid.stl", (-59.9, 0, BASE_T - 3), "#8fbf8f")]

def box(ax, w, d, h, c, col, a=0.35):
    x = [c[0] - w / 2, c[0] + w / 2]; y = [c[1] - d / 2, c[1] + d / 2]; z = [c[2] - h / 2, c[2] + h / 2]
    f = [[(x[0],y[0],z[0]),(x[1],y[0],z[0]),(x[1],y[1],z[0]),(x[0],y[1],z[0])],
         [(x[0],y[0],z[1]),(x[1],y[0],z[1]),(x[1],y[1],z[1]),(x[0],y[1],z[1])],
         [(x[0],y[0],z[0]),(x[1],y[0],z[0]),(x[1],y[0],z[1]),(x[0],y[0],z[1])],
         [(x[0],y[1],z[0]),(x[1],y[1],z[0]),(x[1],y[1],z[1]),(x[0],y[1],z[1])],
         [(x[0],y[0],z[0]),(x[0],y[1],z[0]),(x[0],y[1],z[1]),(x[0],y[0],z[1])],
         [(x[1],y[0],z[0]),(x[1],y[1],z[0]),(x[1],y[1],z[1]),(x[1],y[0],z[1])]]
    ax.add_collection3d(Poly3DCollection(f, facecolor=col, edgecolor='k', alpha=a, linewidths=0.4))

fig = plt.figure(figsize=(16, 6.5))
views = [(14, -62, "등각 (좌현 앞에서)"), (2, -90, "정면 (뱃머리 쪽에서 본 마스트)"), (2, 0, "측면 (우현에서, +Y=뱃머리 →)")]
for k, (el, az, title) in enumerate(views):
    ax = fig.add_subplot(1, 3, k + 1, projection='3d')
    for fn, off, col in PARTS:
        p = os.path.join(OUTD, fn)
        if not os.path.exists(p): continue
        t = load(p) + np.array(off)
        ax.add_collection3d(Poly3DCollection(t, facecolor=col, edgecolor='none', alpha=0.95))
    yf = 20.0
    box(ax, 81.9, 31, 81.9, (0, yf + 1 + 6 + 15.5, 175), "#f4a261")          # OAK
    box(ax, 27, 16, 35, (0, yf + 1 + 6 + 15.5, 175 - 41 - 17), "#e07b39", .5)  # OAK 그랜드
    box(ax, 124, 26, 29, (0, yf + 1 + 6 + 13, 72), "#7aa7e8")                # D455
    box(ax, 10, 8, 25, (37, yf + 1 + 6 + 13, 72 - 14.5 - 12), "#4a7bc8", .6)  # D455 USB-C
    box(ax, 35, 35, 10, (-69.9, 0, 20 - 3 - 13 + 2 + 5), "#d05050", .6)      # IMU
    ax.set_xlim(-115, 115); ax.set_ylim(-70, 160); ax.set_zlim(0, 235)
    ax.set_box_aspect((230, 230, 235)); ax.view_init(elev=el, azim=az)
    ax.set_title(title, fontsize=11); ax.set_xlabel("X (좌−/우+)"); ax.set_ylabel("Y (뱃머리+)"); ax.set_zlabel("Z (갑판 위)")
fig.suptitle("마스트 v3 — OAK 꼭대기 −11°(그랜드↓) / D455 뒷면 M4×2 판 −15°(USB-C↓) / IMU 좌현 포켓 · 최고점 218 ≤ 220", fontsize=12)
plt.tight_layout()
os.makedirs(os.path.dirname(PNG), exist_ok=True)
plt.savefig(PNG, dpi=105)
print(os.path.abspath(PNG))

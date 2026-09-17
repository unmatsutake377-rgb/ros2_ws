#!/usr/bin/env python3
"""mast_preview.py — tools/cad/out/mast_*.stl 를 3면으로 렌더. docs/전달용/배치도/mast_v4_preview.png
Fusion 없이 형상을 눈으로 확인하는 용도(정밀 판정은 STEP 을 Fusion 에서)."""
import os, struct, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False
HERE = os.path.dirname(os.path.abspath(__file__))
OUTD = os.path.join(HERE, "out")
PNG = os.path.join(HERE, "..", "..", "docs", "전달용", "배치도", "mast_v4_preview.png")

def load(fn):
    d = open(fn, 'rb').read()
    if d[:5] == b'solid' and b'facet' in d[:300]:
        v = [list(map(float, l.split()[1:4])) for l in d.decode(errors='ignore').splitlines() if l.split()[:1] == ['vertex']]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack('<I', d[80:84])[0]
    a = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].reshape(-1, 3, 3).astype(float)

# 배치값은 mast.py 가 out/placement.json 으로 뱉는다 (하드코딩 금지)
import json
P = json.load(open(os.path.join(OUTD, "placement.json")))
PARTS = [("mast_base.stl", (0, 0, 0), "#b9b9b9"),
         ("mast_seg1.stl", (0, 0, P["seg_z0"]), "#9fb8d4"),
         ("mast_cap.stl",  (0, 0, P["cap_z0"]), "#c9c9c9"),
         ("mast_imu_lid.stl", (P["pcx"], P["pcy"], P["BASE_T"] - P["LID_T"]), "#8fbf8f")]

def rot_box(ax, w, d, h, c, col, pivot, tilt, a=0.4):
    """pivot=(y,z) 기준 X축 −tilt 회전한 박스."""
    import numpy as _np
    cx, cy, cz = c
    pts = _np.array([[cx+sx*w/2, cy+sy*d/2, cz+sz*h/2] for sx in (-1,1) for sy in (-1,1) for sz in (-1,1)])
    t = _np.radians(-tilt); py, pz = pivot
    yy, zz = pts[:,1]-py, pts[:,2]-pz
    pts[:,1] = py + yy*_np.cos(t) - zz*_np.sin(t)
    pts[:,2] = pz + yy*_np.sin(t) + zz*_np.cos(t)
    idx = [[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]]
    ax.add_collection3d(Poly3DCollection([[tuple(pts[i]) for i in f] for f in idx],
                                         facecolor=col, edgecolor='k', alpha=a, linewidths=0.4))

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
    yf = P["yf"]
    ow, od, oh, ozb, otl, oso, opt, ocx, ocl, ocw = P["oak"]
    dw, dd, dh, dzb, dtl, dso, dpt, dux, dul = P["d455"]
    oy = yf + oso + opt + od / 2
    dy = yf + dso + dpt + dd / 2
    rot_box(ax, ow, od, oh, (0, oy, ozb + oh / 2), "#f4a261", (yf + oso, ozb), otl)      # OAK
    rot_box(ax, ocw, 16, ocl, (ocx, oy, ozb - ocl / 2), "#e07b39", (yf + oso, ozb), otl, .55)  # OAK 그랜드
    rot_box(ax, dw, dd, dh, (0, dy, dzb + dh / 2), "#7aa7e8", (yf + dso, dzb), dtl)      # D455
    rot_box(ax, 10, 8, dul, (dux, dy, dzb - dul / 2), "#4a7bc8", (yf + dso, dzb), dtl, .6)  # D455 USB-C
    ix, iy, iz, il, iw_, ih = P["imu"]
    rot_box(ax, il, iw_, ih, (ix, iy, iz), "#d05050", (0, 0), 0, .6)                      # IMU
    ax.set_xlim(-115, 115); ax.set_ylim(-70, 160); ax.set_zlim(0, 235)
    ax.set_box_aspect((230, 230, 235)); ax.view_init(elev=el, azim=az)
    ax.set_title(title, fontsize=11); ax.set_xlabel("X (좌−/우+)"); ax.set_ylabel("Y (뱃머리+)"); ax.set_zlabel("Z (갑판 위)")
fig.suptitle("마스트 v4 — 카메라 2대 U자 크레들(끼움) / IMU 중심선 선미 포켓(접착) / OAK↔D455 여유 20 · 최고점 %.0f ≤ 220" % P["top"] + "", fontsize=12)
plt.tight_layout()
os.makedirs(os.path.dirname(PNG), exist_ok=True)
plt.savefig(PNG, dpi=105)
print(os.path.abspath(PNG))

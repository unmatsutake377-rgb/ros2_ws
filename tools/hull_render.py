#!/usr/bin/env python3
"""선형 입체 그림 — 보고서 그림 Ⅱ-1(a). 위(선수 쪽)와 아래(선저)에서 본 음영 렌더 두 장.

입력은 선도(tools/hull_lines_plan.py)와 같은 원본:
  1번배: 킬 제거 Fusion360 STL (×1.66, 갑판이 z=0 으로 뒤집혀 나온 메시)
  2번배: Fusion360 STL (10-31). DXF 입력(스테이션 단면을 이어 면 생성)도 지원
재현:
  python3 tools/hull_render.py docs/전달용/선도_1번배/1번배_킬제거_Fusion360.stl --scale 1.66 --flip-z \
      --out docs/전달용/선도_1번배/1번배_선형_3D.png --title "1번배 선형 (킬 제거 STL ×1.66)"
  python3 tools/hull_render.py docs/전달용/선도_2번배/2번배_Fusion360.stl --fit 1770,522 \
      --out docs/전달용/선도_2번배/2번배_선형_3D.png --title "2번배 선형 (Fusion360 모델 형상, 실측 길이·폭)"
"""
import argparse
import io
import os
import struct
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image, ImageOps

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hull_lines_plan as hl   # 글꼴 설정·DXF 읽기 공용


def stl_tris(path, scale, flip_z, fit=None):
    """바깥 면 삼각형만 (N,3,3). 좌표: x 선미 0→선수, y CL=0, z 바닥 0→위."""
    raw = open(path, "rb").read()
    n = struct.unpack("<I", raw[80:84])[0]
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    T = np.frombuffer(raw, dt, count=n, offset=84)["v"].astype(float) * scale
    if fit:   # 형상은 모델, 치수는 실측 (2번배: Fusion 1691×404 → 실측 1770×522, 팀장 10-31)
        T[:, :, 0] *= fit[0] / np.ptp(T[:, :, 0]); T[:, :, 1] *= fit[1] / np.ptp(T[:, :, 1])
    N = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    if flip_z:
        T[:, :, 2] = T[:, :, 2].max() - T[:, :, 2]
        N[:, 2] *= -1; N *= -1
    T[:, :, 2] -= T[:, :, 2].min()
    T[:, :, 0] -= T[:, :, 0].min()
    T[:, :, 1] -= (T[:, :, 1].min() + T[:, :, 1].max()) / 2   # 중심선 y=0 (2번배 Fusion STL 은 CL 이 y≈125)
    L, H = T[:, :, 0].max(), T[:, :, 2].max()
    C = np.array([L / 2, 0.0, 1.2 * H])          # 선체 안쪽 기준점 (hull_lines_plan.load_stl 과 같음)
    out = (N * (C - T.mean(1))).sum(1) < 0
    up = N[:, 2] / (np.linalg.norm(N, axis=1) + 1e-12) > 0.95
    return T[out & ~up]


def dxf_tris(path):
    by, cl = hl.load(path)
    st = sorted(by["HULL_STATION_LINES"], key=lambda p: p[:, 0].mean())
    n = min(len(p) for p in st)
    S = [p[:n].copy() for p in st]
    for p in S:
        p[:, 1] -= cl
    z0 = min(p[:, 2].min() for p in S)
    x0 = min(p[:, 0].min() for p in S)
    tris = []
    for a, b in zip(S[:-1], S[1:]):
        for i in range(n - 1):
            q = [a[i], a[i + 1], b[i + 1], b[i]]
            tris += [q[:3], [q[0], q[2], q[3]]]
    T = np.array(tris)
    T[:, :, 2] -= z0; T[:, :, 0] -= x0
    return T


def shade(T, light):
    N = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
    k = np.abs(N @ light)                        # 감김 방향과 무관하게 양면 조명
    base = np.array([0.80, 0.84, 0.90])
    return np.clip(base[None] * (0.35 + 0.65 * k[:, None]), 0, 1)


def view(ax, T, elev, azim, label):
    L = np.radians(azim); E = np.radians(elev)
    light = np.array([np.cos(E) * np.cos(L), np.cos(E) * np.sin(L), np.sin(E)])
    pc = Poly3DCollection(T, facecolors=shade(T, light), edgecolors="none", linewidths=0)
    ax.add_collection3d(pc)
    lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(hi - lo)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="선형")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--flip-z", action="store_true")
    ap.add_argument("--fit", help="길이,폭 [mm] 실측에 맞춰 x·y 만 늘림")
    a = ap.parse_args()
    T = stl_tris(a.src, a.scale, a.flip_z, [float(v) for v in a.fit.split(',')] if a.fit else None) if a.src.lower().endswith(".stl") else dxf_tris(a.src)
    lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
    # 3D 축은 여백이 커서 시점마다 따로 그린 뒤 흰 여백을 잘라 붙인다
    imgs = []
    for elev, azim, label in ((22, -55, "위·선수 쪽에서 본 모습"), (-30, -125, "아래에서 본 모습 (선저)")):
        f = plt.figure(figsize=(9, 6))
        view(f.add_subplot(projection="3d"), T, elev, azim, label)
        buf = io.BytesIO(); f.savefig(buf, dpi=170, format="png"); plt.close(f)
        im = Image.open(buf).convert("RGB")
        im = im.crop(ImageOps.invert(im).getbbox())
        imgs.append((im, label))
    fig, axs = plt.subplots(1, 2, figsize=(13, 3.4))
    for ax, (im, label) in zip(axs, imgs):
        ax.imshow(im); ax.set_axis_off(); ax.set_title(label, fontsize=11)
    fig.suptitle(f"{a.title}   LOA {hi[0]-lo[0]:.0f} · B {hi[1]-lo[1]:.0f} · D {hi[2]-lo[2]:.0f} mm", fontsize=13)
    fig.tight_layout()
    fig.savefig(a.out, dpi=150, bbox_inches="tight")
    print(a.out, len(T), "faces")


if __name__ == "__main__":
    main()

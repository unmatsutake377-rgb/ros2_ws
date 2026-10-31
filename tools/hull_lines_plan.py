#!/usr/bin/env python3
"""선도(Lines plan) — 3D 선형 DXF 에서 정면도(Body plan)·측면도(Profile)·반폭도(Half-breadth) 를 뽑는다.

입력 DXF 레이어 (2번배 정상선형 3D, ezdxf 로 저장된 것):
  HULL_STATION_LINES      단면(스테이션) 닫힌 폴리라인 — x 고정
  HULL_LONGITUDINAL_LINES 종방향 선 (좌·우 쌍)
  HULL_SHEER              현측 상단선 (좌·우)
  HULL_WATERLINES         수선 (z 고정, 좌·우 반쪽)
  REFERENCE_CENTERLINE    중심선 (y = CL)
좌표: x = 길이(0 선미 트랜섬 → 최대 선수), y = 폭(CL 기준 대칭), z = 높이(0 = 용골 최저점). 단위 mm.

STL 모드 (두께 있는 선체 껍데기 메시, 예: 1번배 Fusion360 출력):
  바깥 면 삼각형만 골라(법선이 선체 안쪽 기준점에서 멀어지는 쪽) 평면으로 잘라 스테이션·수선·버톡을 만든다.
  --flip-z : 갑판이 z=0, 바닥이 +z 로 뒤집혀 나온 메시.  --scale : 실제 크기 배율.

재현:
  python3 tools/hull_lines_plan.py docs/전달용/선도_2번배/2번배_Fusion360.stl --fit 1770,522 --out docs/전달용/선도_2번배 --name 2번배
    (10-31: 정상선형 DXF 는 선수가 실물과 달라 Fusion360 STL 로 교체 — 팀장 확인)
  python3 tools/hull_lines_plan.py docs/전달용/선도_1번배/1번배_킬제거_Fusion360.stl --scale 1.66 --flip-z \
      --out docs/전달용/선도_1번배 --name 1번배
"""
import argparse
import os
from collections import defaultdict

import ezdxf
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

for _f in ("Apple SD Gothic Neo", "AppleGothic", "NanumGothic", "Noto Sans CJK KR"):
    if any(_f == f.name for f in font_manager.fontManager.ttflist):
        matplotlib.rcParams["font.family"] = _f
        break
matplotlib.rcParams["axes.unicode_minus"] = False

C_ST, C_WL, C_LG, C_SH, C_KL, C_CL = "#1f4e79", "#1a7f37", "#9a9a9a", "#b03a2e", "#111111", "#7d3c98"


def load(path):
    doc = ezdxf.readfile(path)
    by = defaultdict(list)
    cl_y = None
    for e in doc.modelspace():
        t = e.dxftype()
        if t == "POLYLINE":
            pts = np.array([v.dxf.location for v in e.vertices], float)
            if e.is_closed:   # 닫힌 단면은 마지막→첫 점 구간(트랜섬·선수 바닥의 직선)이 꼭짓점 목록에 없다
                pts = np.vstack([pts, pts[:1]])
            by[e.dxf.layer].append(pts)
        elif t == "LINE" and e.dxf.layer == "REFERENCE_CENTERLINE":
            cl_y = e.dxf.start[1]
    if cl_y is None:
        allp = np.vstack(by["HULL_STATION_LINES"])
        cl_y = (allp[:, 1].min() + allp[:, 1].max()) / 2
    return by, cl_y


def densify(p, step=1.0):
    """긴 직선 구간(트랜섬 바닥·용골 근처)이 CL 을 건너뛰지 않게 점을 촘촘히 채운다."""
    out = [p[:1]]
    for a, b in zip(p[:-1], p[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        t = np.linspace(0, 1, n + 1)[1:, None]
        out.append(a + (b - a) * t)
    return np.vstack(out)


def _slice(T, ax, c):
    """삼각형 묶음 T(n,3,3)를 좌표축 ax = c 평면으로 자른 선분들(m,2,3)."""
    d = T[:, :, ax] - c
    d = np.where(np.abs(d) < 1e-9, 1e-9, d)
    m = (d.min(1) < 0) & (d.max(1) > 0)
    t, d = T[m], d[m]
    pts = []
    for a, b in ((0, 1), (1, 2), (2, 0)):
        cr = d[:, a] * d[:, b] < 0
        with np.errstate(divide="ignore", invalid="ignore"):
            u = d[:, a] / (d[:, a] - d[:, b])
        q = t[:, a] + (t[:, b] - t[:, a]) * u[:, None]
        pts.append(np.where(cr[:, None], q, np.nan))
    P = np.stack(pts, 1)
    ok = (~np.isnan(P[:, :, 0])).sum(1) == 2
    return np.array([r[~np.isnan(r[:, 0])] for r in P[ok]])


def _chain(segs, tol=1e-4):
    """순서 없는 선분들을 이어 폴리라인 목록으로 (끝점 반올림으로 짝짓기)."""
    if len(segs) == 0:
        return []
    key = lambda p: tuple(np.round(p / tol).astype(np.int64))
    adj = defaultdict(list)
    for i, (a, b) in enumerate(segs):
        adj[key(a)].append((i, 0)); adj[key(b)].append((i, 1))
    used = np.zeros(len(segs), bool)
    lines = []
    ends = [k for k, v in adj.items() if len(v) == 1]
    order = [adj[k][0] for k in ends] + [(i, 0) for i in range(len(segs))]
    for i0, e0 in order:
        if used[i0]:
            continue
        used[i0] = True
        pts = [segs[i0][e0], segs[i0][1 - e0]]
        while True:
            nxt = [(i, e) for i, e in adj[key(pts[-1])] if not used[i]]
            if not nxt:
                break
            i, e = nxt[0]; used[i] = True; pts.append(segs[i][1 - e])
        lines.append(np.array(pts))
    return sorted(lines, key=len, reverse=True)


def load_stl(path, scale=1.0, flip_z=False, fit=None, n_st=11, wl_step=40.0, buttocks=(0.25, 0.5, 0.75)):
    """두께 있는 선체 메시 → load() 와 같은 레이어 구조. 좌표: x 선미 0→선수, y CL=0, z 바닥 0→위."""
    import struct
    raw = open(path, "rb").read()
    n = struct.unpack("<I", raw[80:84])[0]
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec = np.frombuffer(raw, dt, count=n, offset=84)
    T = rec["v"].astype(float) * scale
    if fit:   # 형상은 모델, 치수는 실측 (2번배: Fusion 1691×404 → 실측 1770×522, 팀장 10-31)
        T[:, :, 0] *= fit[0] / np.ptp(T[:, :, 0]); T[:, :, 1] *= fit[1] / np.ptp(T[:, :, 1])
    N = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    if flip_z:
        T[:, :, 2] = T[:, :, 2].max() - T[:, :, 2]
        N[:, 2] *= -1; N *= -1            # 한 축을 뒤집으면 감김 방향도 뒤집힌다
    T[:, :, 2] -= T[:, :, 2].min()
    T[:, :, 0] -= T[:, :, 0].min()
    T[:, :, 1] -= (T[:, :, 1].min() + T[:, :, 1].max()) / 2   # 중심선 y=0 (2번배 Fusion STL 은 CL 이 y≈125)
    L, H = T[:, :, 0].max(), T[:, :, 2].max()
    # 바깥 면 = 법선이 선체 안쪽 기준점(가운데, 갑판보다 위)에서 멀어지는 면
    C = np.array([L / 2, 0.0, H + 0.2 * H])
    cen = T.mean(1)
    out = (N * (C - cen)).sum(1) < 0
    up = N[:, 2] / (np.linalg.norm(N, axis=1) + 1e-12) > 0.95      # 테두리 윗면(두께 면) 제외
    To = T[out & ~up]
    by = defaultdict(list)
    xs = np.linspace(0, L, n_st)
    xs[0], xs[-1] = 0.005 * L, 0.995 * L
    for x in xs:
        ls = _chain(_slice(To, 0, x))
        if ls:
            by["HULL_STATION_LINES"].append(ls[0])
    for z in np.arange(wl_step, H - 0.5 * wl_step, wl_step):
        for l in _chain(_slice(To, 2, z)):
            if len(l) > 5:
                for side in (+1, -1):
                    q = l[(l[:, 1] * side >= 0) & (l[:, 0] > 0.01 * L)]   # 트랜섬 판 두께에 걸린 조각 제외
                    if len(q) > 2:
                        by["HULL_WATERLINES"].append(q[np.argsort(q[:, 0])])
    hb = np.abs(To[:, :, 1]).max()
    for f in buttocks:
        for side in (+1, -1):
            for l in _chain(_slice(To, 1, side * f * hb))[:1]:
                by["HULL_LONGITUDINAL_LINES"].append(l[np.argsort(l[:, 0])])
    for k in _chain(_slice(To, 1, 0.0)):     # 바닥·트랜섬·선수가 따로 끊겨 나올 수 있다 → 전부
        if len(k) > 2:
            by["KEEL"].append(k)
    # 현측 상단선: x 구간마다 바깥 면의 최대 |y| 와 그 높이
    V = To.reshape(-1, 3)
    bins = np.linspace(0, L, 160)
    idx = np.digitize(V[:, 0], bins)
    for side in (+1, -1):
        pts = []
        for i in range(1, len(bins)):
            sel = V[idx == i]
            sel = sel[sel[:, 1] * side > 0]
            if len(sel):
                j = np.argmax(np.abs(sel[:, 1])); pts.append([sel[j, 0], sel[j, 1], sel[:, 2].max()])
        pts = np.array(pts)
        # 성긴 메시(2번배 Fusion STL 선수부)는 구간에 높은 꼭짓점이 없어 현측선이 톱니처럼 꺼진다 → 이웃 ±3 구간 최댓값
        z = pts[:, 2].copy()
        pts[:, 2] = [z[max(0, i - 3):i + 4].max() for i in range(len(z))]
        by["HULL_SHEER"].append(pts)
    return by, 0.0


def half(p, cl, side):
    """CL 기준 한쪽(side=+1 우현 / -1 좌현) 점만, 끊긴 구간은 NaN 으로 분리."""
    q = densify(p)
    keep = (q[:, 1] - cl) * side >= -0.5
    q[~keep] = np.nan
    return q


def pchip(x, y, xf):
    """단조 3차 보간(Fritsch-Carlson) — scipy 없이. 용골선이 스테이션 사이에서 출렁이지 않게."""
    h = np.diff(x); d = np.diff(y) / h
    m = np.zeros_like(y)
    for k in range(1, len(x) - 1):
        if d[k - 1] * d[k] > 0:
            w1, w2 = 2 * h[k] + h[k - 1], h[k] + 2 * h[k - 1]
            m[k] = (w1 + w2) / (w1 / d[k - 1] + w2 / d[k])
    m[0], m[-1] = d[0], d[-1]
    i = np.clip(np.searchsorted(x, xf) - 1, 0, len(x) - 2)
    t = (xf - x[i]) / h[i]
    h00, h10, h01, h11 = 2*t**3 - 3*t**2 + 1, t**3 - 2*t**2 + t, -2*t**3 + 3*t**2, t**3 - t**2
    return h00 * y[i] + h10 * h[i] * m[i] + h01 * y[i + 1] + h11 * h[i] * m[i + 1]


def keel_profile(stations, cl):
    """스테이션 단면의 CL 부근 최저점을 이어 용골선(측면 하단 윤곽)을 만든다."""
    xs, zs = [], []
    for p in stations:
        near = np.abs(p[:, 1] - cl) <= max(15.0, 0.08 * np.ptp(p[:, 1]))
        z = p[near, 2].min() if near.any() else p[:, 2].min()
        xs.append(p[0, 0]); zs.append(z)
    o = np.argsort(xs)
    xs, zs = np.array(xs)[o], np.array(zs)[o]
    xf = np.linspace(xs[0], xs[-1], 400)
    return xs, zs, xf, pchip(xs, zs, xf)


def draw_profile(ax, by, cl, dims):
    st = sorted(by["HULL_STATION_LINES"], key=lambda p: p[0, 0])
    for p in by["HULL_LONGITUDINAL_LINES"]:
        if np.mean(p[:, 1]) > cl:
            ax.plot(p[:, 0], p[:, 2], color=C_LG, lw=0.8, label=dims.get("lg_label", "종방향 선"))
    for p in by["HULL_SHEER"]:
        if np.mean(p[:, 1]) > cl:
            ax.plot(p[:, 0], p[:, 2], color=C_SH, lw=1.8, label="현측 상단선(Sheer)")
    if by.get("KEEL"):
        for j, k in enumerate(by["KEEL"]):
            ax.plot(k[:, 0], k[:, 2], color=C_KL, lw=1.8, label="중심선 윤곽(Keel·트랜섬·선수)" if j == 0 else None)
    else:
        xs, zs, xf, zf = keel_profile(st, cl)
        ax.plot(xf, zf, color=C_KL, lw=1.8, label="용골선(Keel)")
        # 선미 트랜섬·선수 끝 닫기
        s0, s1 = st[0], st[-1]
        ax.plot([s0[0, 0]] * 2, [zf[0], s0[:, 2].max()], color=C_KL, lw=1.8)
        ax.plot([s1[0, 0]] * 2, [zf[-1], s1[:, 2].max()], color=C_KL, lw=1.8)
    for i, p in enumerate(st):
        x = p[0, 0]
        ax.plot([x, x], [p[:, 2].min(), p[:, 2].max()], color=C_ST, lw=0.7, alpha=0.8)
        ax.text(x, -14, str(i), ha="center", va="top", fontsize=7, color=C_ST)
    span = defaultdict(lambda: [np.inf, -np.inf])          # 같은 높이의 조각들을 합쳐 앞뒤 끝
    for p in by["HULL_WATERLINES"]:
        if np.mean(p[:, 1]) > cl:
            z = round(p[0, 2], 1)
            span[z][0] = min(span[z][0], p[:, 0].min()); span[z][1] = max(span[z][1], p[:, 0].max())
    for j, (z, (x0, x1)) in enumerate(sorted(span.items())):
        xl = np.array([x0, x1])
        ax.plot([xl.min(), xl.max()], [z, z], color=C_WL, lw=1.1, ls="--", label="수선(WL)" if j == 0 else None)
        ax.text(xl.max() + 12, z + (5 if j else -5), f"WL {z:.1f}", color=C_WL, fontsize=7.5, va="center")
    h, l = ax.get_legend_handles_labels()
    u = dict(zip(l, h))
    ax.legend(u.values(), u.keys(), fontsize=7.5, loc="upper left", bbox_to_anchor=(0.03, 0.93), ncol=4, framealpha=0.9)
    ax.set_title("측면도 (Profile)", loc="left", fontsize=11, weight="bold")
    ax.set_xlabel("길이 x [mm]  (0 = 선미 트랜섬 → 선수)")
    ax.set_ylabel("높이 z [mm]")
    ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_xlim(-40, dims["L"] + 90); ax.set_ylim(-35, dims["D"] + 30)


def draw_halfbreadth(ax, by, cl, dims):
    st = sorted(by["HULL_STATION_LINES"], key=lambda p: p[0, 0])
    for p in by["HULL_LONGITUDINAL_LINES"]:
        if np.mean(p[:, 1]) > cl:
            ax.plot(p[:, 0], p[:, 1] - cl, color=C_LG, lw=0.8, label=dims.get("lg_label") if dims.get("lg_label") else None)
    for p in by["HULL_SHEER"]:
        if np.mean(p[:, 1]) > cl:
            ax.plot(p[:, 0], p[:, 1] - cl, color=C_SH, lw=1.8, label="현측 상단선(Sheer)")
    for p in by["HULL_WATERLINES"]:
        if np.mean(p[:, 1]) > cl:
            q = half(p, cl, +1)
            z = p[0, 2]
            ax.plot(q[:, 0], q[:, 1] - cl, color=C_WL, lw=1.3, ls="--", label=f"수선 WL {z:.1f}")
    for i, p in enumerate(st):
        x = p[0, 0]
        ax.plot([x, x], [0, p[:, 1].max() - cl], color=C_ST, lw=0.7, alpha=0.8)
        ax.text(x, -10, str(i), ha="center", va="top", fontsize=7, color=C_ST)
    ax.axhline(0, color=C_CL, lw=1.0, ls="-.")
    ax.text(dims["L"] + 12, 0, "CL", color=C_CL, fontsize=8, va="center")
    ax.set_title("반폭도 (Half-breadth plan)", loc="left", fontsize=11, weight="bold")
    ax.set_xlabel("길이 x [mm]")
    ax.set_ylabel("반폭 y [mm]")
    ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_xlim(-40, dims["L"] + 90); ax.set_ylim(-30, dims["B"] / 2 + 25)
    h, l = ax.get_legend_handles_labels()
    u = dict(zip(l, h))
    ax.legend(u.values(), u.keys(), fontsize=7.5, loc="lower center", ncol=3, framealpha=0.9)


def draw_body(ax, by, cl, dims):
    st = sorted(by["HULL_STATION_LINES"], key=lambda p: p[0, 0])
    mid = dims["L"] / 2
    i_max = int(np.argmax([np.ptp(p[:, 1]) for p in st]))   # 최대 단면(중앙 단면) — 양쪽에 다 그린다
    aft = [i for i, p in enumerate(st) if p[0, 0] < mid]

    def outline(p, side):
        q = half(p, cl, side)
        q[q[:, 2] > np.nanmax(q[:, 2]) - 0.6] = np.nan      # 갑판 윗변은 빼고 외판만
        return q

    transom = dims.get("transom_station0", False)
    for i, p in enumerate(st):
        side = +1 if p[0, 0] >= mid else -1   # 관례: 선수 쪽 단면 = 오른쪽, 선미 쪽 단면 = 왼쪽
        q = outline(p, side)
        if i == 0 and transom:
            # 선미 트랜섬 판 윤곽(x≈0). 바로 앞 단면보다 10 mm 안쪽·바닥 33 mm 로 모서리가 깎여 있어 따로 표시
            ax.plot(q[:, 1] - cl, q[:, 2], color="#888", lw=1.1, ls="--")
            continue
        ax.plot(q[:, 1] - cl, q[:, 2], color=C_ST, lw=1.6 if i == i_max else 1.0)
        if side > 0:
            k = np.nanargmax(np.abs(q[:, 1] - cl))
            ax.text(q[k, 1] - cl + 6, q[k, 2], str(i), fontsize=7, color=C_ST, ha="left", va="center")
        elif not dims.get("group_aft", False):
            # 선미 쪽은 위가 모여 겹친다 → 바닥이 CL 에 닿는 곳(높이가 단면마다 다르다)에 번호
            k = np.nanargmin(np.abs(q[:, 1] - cl))
            ax.text(q[k, 1] - cl - 4, q[k, 2], str(i), fontsize=7, color=C_ST, ha="right", va="center")
    # 최대 단면을 선미 쪽에도 (비교 기준)
    q = outline(st[i_max], -1)
    ax.plot(q[:, 1] - cl, q[:, 2], color=C_ST, lw=1.6)
    if dims.get("group_aft", False):
        # 선미 쪽 라벨: 겹치는 평행 중앙부라 묶어서
        a1, a2 = aft[1], aft[-1]
        ax.annotate(f"{a1}~{a2} (안→밖)\n{i_max} 최대 단면(굵은 선)", xy=(-dims["B"] / 2 + 12, 120), xytext=(-dims["B"] / 2 + 40, 140),
                    fontsize=7, color=C_ST, arrowprops=dict(arrowstyle="-", color=C_ST, lw=0.6))
    else:
        ax.text(0, -0.06 * dims["D"], f"굵은 선 = {i_max} 최대 단면(양쪽)", fontsize=7, color=C_ST, ha="center", va="top")
    if transom:
        ax.annotate("0 선미 트랜섬 윤곽(점선)", xy=(-120, 33), xytext=(-150, 95), fontsize=7, color="#666",
                    arrowprops=dict(arrowstyle="-", color="#888", lw=0.6))
    for p in by["HULL_SHEER"]:
        side = 1 if np.mean(p[:, 1]) > cl else -1
        sel = (p[:, 0] >= mid) if side > 0 else ((p[:, 0] < mid) & (p[:, 0] > 30))
        ax.plot(p[sel, 1] - cl, p[sel, 2], color=C_SH, lw=1.4)
    for p in by["HULL_WATERLINES"]:
        z = p[0, 2]
        ax.plot([-dims["B"] / 2, dims["B"] / 2], [z, z], color=C_WL, lw=1.0, ls="--")
        ax.text(-dims["B"] / 2 - 4, z, f"WL {z:.1f}", color=C_WL, fontsize=6.5, ha="right", va="center")
    ax.axvline(0, color=C_CL, lw=1.0, ls="-.")
    ax.text(-dims["B"] / 2, dims["D"] + 14, f"← 선미 쪽 단면 (0~{aft[-1]})", fontsize=7.5, ha="left")
    ax.text(dims["B"] / 2, dims["D"] + 14, f"선수 쪽 단면 ({aft[-1] + 1}~{len(st) - 1}) →", fontsize=7.5, ha="right")
    ax.set_title("정면도 (Body plan)", loc="left", fontsize=11, weight="bold")
    ax.set_xlabel("폭 y [mm] (CL = 0)")
    ax.set_ylabel("높이 z [mm]")
    ax.set_aspect("equal"); ax.grid(alpha=0.25)
    ax.set_xlim(-dims["B"] / 2 - 60, dims["B"] / 2 + 30); ax.set_ylim(-0.12 * dims["D"], dims["D"] * 1.15)


def station_table(by):
    st = sorted(by["HULL_STATION_LINES"], key=lambda p: p[0, 0])
    return [(i, p[0, 0]) for i, p in enumerate(st)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dxf")
    ap.add_argument("--out", default=".")
    ap.add_argument("--name", default="hull")
    ap.add_argument("--scale", type=float, default=1.0, help="STL: 실제 크기 배율")
    ap.add_argument("--flip-z", action="store_true", help="STL: 갑판이 z=0·바닥이 +z 인 메시")
    ap.add_argument("--fit", help="STL: 길이,폭 [mm] 실측에 맞춰 x·y 만 늘림 (예: 1770,522)")
    ap.add_argument("--wl-step", type=float, default=40.0, help="STL: 수선 간격 [실제 mm]")
    a = ap.parse_args()
    stl = a.dxf.lower().endswith(".stl")
    if stl:
        by, cl = load_stl(a.dxf, a.scale, a.flip_z, fit=[float(v) for v in a.fit.split(',')] if a.fit else None, wl_step=a.wl_step)
    else:
        by, cl = load(a.dxf)
    allp = np.vstack([p for k, v in by.items() if k != "HULL_WATERLINES" for p in v])
    dims = {"L": allp[:, 0].max() - allp[:, 0].min(), "B": np.ptp(allp[:, 1]), "D": allp[:, 2].max() - allp[:, 2].min()}
    if stl:
        dims["lg_label"] = "버톡(Buttock, 반폭 25·50·75 %)"
    else:
        dims.update(transom_station0=True, group_aft=True)
    os.makedirs(a.out, exist_ok=True)
    sub = f"{a.name}  LOA {dims['L']:.0f} · B {dims['B']:.0f} · D {dims['D']:.0f} mm"
    if stl and a.scale != 1.0:
        sub += f"  (STL ×{a.scale:g})"
    if stl and a.fit:
        sub += "  (모델 형상을 실측 길이·폭에 맞춤)"
    outs = []

    for key, fn, size in (("정면도", draw_body, (6.4, 4.4)), ("측면도", draw_profile, (13, 3.0)),
                          ("반폭도", draw_halfbreadth, (13, 3.0))):
        fig, ax = plt.subplots(figsize=size)
        fn(ax, by, cl, dims)
        fig.suptitle(sub, fontsize=8.5, x=0.99, ha="right", color="#555")
        fig.tight_layout()
        p = os.path.join(a.out, f"{a.name}_{key}.png"); fig.savefig(p, dpi=200); plt.close(fig); outs.append(p)

    fig = plt.figure(figsize=(17, 5.4))
    gs = fig.add_gridspec(2, 2, width_ratios=[3.3, 1], height_ratios=[1, 1.05])
    draw_profile(fig.add_subplot(gs[0, 0]), by, cl, dims)
    draw_halfbreadth(fig.add_subplot(gs[1, 0]), by, cl, dims)
    draw_body(fig.add_subplot(gs[:, 1]), by, cl, dims)
    tbl = "  ".join(f"{i}:{x:.0f}" for i, x in station_table(by))
    fig.suptitle(f"선도 (Lines plan) — {sub}", fontsize=13, weight="bold", x=0.01, ha="left")
    fig.text(0.01, 0.01, f"스테이션 번호 : x[mm]   {tbl}", fontsize=8, color="#333")
    fig.tight_layout(rect=(0, 0.03, 1, 0.96))
    p = os.path.join(a.out, f"{a.name}_선도_종합.png"); fig.savefig(p, dpi=200); plt.close(fig); outs.append(p)
    for p in outs:
        print(os.path.abspath(p))


if __name__ == "__main__":
    main()

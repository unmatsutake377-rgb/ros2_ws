#!/usr/bin/env python3
"""lidar_mount.py — RPLIDAR A3 물막이 1단계: 받침(base) + 차양(hood) + 맞춤 쿠폰(fit) (2026-10-30, CadQuery).

왜: A3 는 방수가 아닌데 선수 끝(물보라 1순위)에 앉는다. 막지는 못해도 덜 젖게 한다
    (`docs/전달용/마스트_요구사항.md` §2-1 "스프레이 레일 + LiDAR 위 작은 차양").
  · 받침 — 갑판에서 PED_H 만큼 띄워 LiDAR 바닥이 고인 물에 안 잠기게. 갑판 볼트 4 + 차양 자리 8곳.
  · 차양 — 위에서 오는 비·물보라를 지붕으로. **기둥은 하나, 뒤쪽(마스트 쪽)에만** — 앞 ±80° 에는 아무것도 없다.
           치마(skirt) 아래끝은 레이저 창 위끝보다 SKIRT_MARGIN 위 → 빔을 안 가린다(빔은 수평).
  · 쿠폰 — 받침 윗판만 3 mm. 실물 A3 를 얹어 나사 구멍·맞춤 핀을 10 분 만에 확인.

A3 치수 출처: Slamtec "RPLIDAR A3M1 datasheet" v1.2 Figure 5-1 (p.16):
  베이스 Ø76, 회전 헤드 Ø72.5, 전체 높이 41, 레이저 창 중심 31(창 폭 6), 바닥 4×M3 깊이 4 (R29.5, 45°),
  위치결정 구멍 Ø3 깊이 1.0 (R29.5, 0°), 케이블은 반대쪽(180°) 평면부.
  🚨 데이터시트: "바닥 M3 나사는 4 mm 보다 길게 들어가면 내부 모듈 손상" → 윗판 3 mm + M3×6 (물림 3 mm).

사용:
  python3 tools/cad/lidar_mount.py   → tools/cad/out/lidar_{base,hood,fit}.stl|step + 판정 출력
                                      + docs/전달용/배치도/lidar_mount_preview.png
  확정본 STL 은 docs/전달용/라이다_stl/ 에 복사해 둔다(마스트와 같은 방식).
"""
import math
import os

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
os.makedirs(OUT, exist_ok=True)

# ── A3 (데이터시트) ─────────────────────────────────────────────
A3_D, A3_H = 76.0, 41.0
A3_HEAD_D = 72.5
A3_BASE_H, A3_HEAD_Z0 = 18.6, 19.6      # 고정 베이스 높이, 회전 헤드 시작(그 사이 1 mm 가 회전 틈)
A3_WIN_C, A3_WIN_W = 31.0, 6.0          # 레이저 창 중심 높이, 창 폭 → 위끝 34
A3_HOLE_R = 29.5                        # 바닥 M3 ×4 (45°, 135°, 225°, 315°)
A3_SCREW_MAX = 4.0                      # 🚨 이보다 깊게 물리면 내부 손상
A3_PIN_R, A3_PIN_HOLE_D, A3_PIN_HOLE_DEPTH = 29.5, 3.0, 1.0   # 위치결정 구멍 (0°)

# ── 받침 ───────────────────────────────────────────────────────
TOP_T = 3.0            # 윗판 두께 = 나사가 지나는 두께. M3×6 → 물림 3 ≤ 4
PED_H = 12.0           # 갑판 → LiDAR 바닥. 회전면 = 갑판 + PED_H + 31 (갑판 직치 31 대비 +PED_H)
FLANGE_R, FLANGE_T = 62.0, 5.0
M3_CLR, M3_HEAD_ACCESS = 3.4, 7.0       # 관통, 아래에서 드라이버·나사머리 들어갈 구멍
M3_NUT_AF, M3_NUT_T = 5.6, 2.6          # 육각너트 맞변(5.5 + 여유), 트랩 깊이 (차양 고정용, 갑판 쪽에서 미리 넣음)
DECK_HOLE_D, DECK_HOLE_R = 4.5, 56.0    # 갑판 볼트 M4 ×4 (차양 자리와 22.5° 어긋나게)
PIN_D, PIN_H = 2.6, 0.8                 # 위치결정 핀 (안 맞으면 칼로 깎아내도 된다)

# ── 차양 ───────────────────────────────────────────────────────
HOOD_POS_N = 8                          # 차양 기둥 자리 8곳(45° 간격) — LiDAR 를 어느 방향으로 달든 '마스트 쪽' 에 맞춘다
POST_R0, POST_R1 = 44.0, 56.0           # 기둥 반경 범위 (A3 R38 과 6 mm 띄움)
POST_W = 10.0                           # 기둥 접선 폭 → 가리는 뒤쪽 각도 = 2·atan(5/44)
FOOT_W, FOOT_T = 24.0, 4.0              # 발(받침 플랜지 위) 접선 폭·두께
FOOT_HOLE_T = 8.5                       # 발 나사 2개: 접선 ±8.5, 반경 (POST_R0+POST_R1)/2
GAP_TOP = 8.0                           # LiDAR 꼭대기 → 지붕 아래면
ROOF_R, ROOF_T = 50.0, 3.0
SKIRT_T = 3.0
SKIRT_MARGIN = 7.0                      # 치마 아래끝 - 레이저 창 위끝 (≥5 확보)

FOOT_HOLE_R = (POST_R0 + POST_R1) / 2


def pol(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)


# ═════════════════════════ 받침 ═════════════════════════
def make_base():
    # 플랜지 윗면 바깥 모서리 모따기 — 물이 고이지 않고 흘러내리게
    flange = cq.Workplane("XY").circle(FLANGE_R).extrude(FLANGE_T).faces(">Z").edges().chamfer(1.5)
    b = flange.union(cq.Workplane("XY").circle(A3_D / 2).extrude(PED_H))
    # LiDAR 나사 4개: 아래에서 드라이버가 들어갈 Ø7 구멍 + 윗판 3 mm 관통
    for deg in (45, 135, 225, 315):
        x, y = pol(A3_HOLE_R, deg)
        b = b.cut(cq.Workplane("XY").center(x, y).circle(M3_HEAD_ACCESS / 2).extrude(PED_H - TOP_T))
        b = b.cut(cq.Workplane("XY").center(x, y).circle(M3_CLR / 2).extrude(PED_H))
    # 위치결정 핀 (0°)
    px, py = pol(A3_PIN_R, 0)
    b = b.union(cq.Workplane("XY").workplane(offset=PED_H).center(px, py).circle(PIN_D / 2).extrude(PIN_H))
    # 갑판 볼트 M4 ×4
    for k in range(4):
        x, y = pol(DECK_HOLE_R, 22.5 + 90 * k)
        b = b.cut(cq.Workplane("XY").center(x, y).circle(DECK_HOLE_D / 2).extrude(FLANGE_T))
    # 차양 자리 8곳 × 나사 2 — 관통 + 아래 육각너트 트랩
    for k in range(HOOD_POS_N):
        th = 360.0 / HOOD_POS_N * k
        for s in (-1, 1):
            x, y = local_to_xy(FOOT_HOLE_R, s * FOOT_HOLE_T, th)
            b = b.cut(cq.Workplane("XY").center(x, y).circle(M3_CLR / 2).extrude(FLANGE_T))
            b = b.cut(cq.Workplane("XY").center(x, y).polygon(6, M3_NUT_AF / math.cos(math.pi / 6))
                      .extrude(M3_NUT_T))
    return b


def local_to_xy(r, t, th_deg):
    """반경 r, 접선 오프셋 t, 방위 th → XY."""
    a = math.radians(th_deg)
    return r * math.cos(a) - t * math.sin(a), r * math.sin(a) + t * math.cos(a)


# ═════════════════════════ 차양 (기둥 방위 0° 로 생성, 설치 때 8곳 중 마스트 쪽으로) ═════════════════════════
Z_FOOT = FLANGE_T                              # 발 아래면 = 플랜지 윗면
Z_A3_TOP = PED_H + A3_H
Z_WIN_TOP = PED_H + A3_WIN_C + A3_WIN_W / 2
Z_ROOF0 = Z_A3_TOP + GAP_TOP
Z_ROOF1 = Z_ROOF0 + ROOF_T
Z_SKIRT0 = Z_WIN_TOP + SKIRT_MARGIN


def make_hood():
    depth = POST_R1 - POST_R0
    # 발 + 45° 경사(뒤집어 프린트할 때 받침 없이 나오게): 접선(Y)-높이(Z) 단면을 반경(X)으로 밀어낸다
    taper = (FOOT_W - POST_W) / 2
    prof = [(-FOOT_W / 2, Z_FOOT), (FOOT_W / 2, Z_FOOT), (FOOT_W / 2, Z_FOOT + FOOT_T),
            (POST_W / 2, Z_FOOT + FOOT_T + taper), (-POST_W / 2, Z_FOOT + FOOT_T + taper),
            (-FOOT_W / 2, Z_FOOT + FOOT_T)]
    foot = cq.Workplane("YZ").workplane(offset=POST_R0).polyline(prof).close().extrude(depth)
    post = (cq.Workplane("XY").workplane(offset=Z_FOOT)
            .center((POST_R0 + POST_R1) / 2, 0).rect(depth, POST_W).extrude(Z_ROOF1 - Z_FOOT))
    roof = cq.Workplane("XY").workplane(offset=Z_ROOF0).circle(ROOF_R).extrude(ROOF_T)
    skirt = (cq.Workplane("XY").workplane(offset=Z_SKIRT0).circle(ROOF_R)
             .circle(ROOF_R - SKIRT_T).extrude(Z_ROOF0 - Z_SKIRT0))
    h = foot.union(post).union(roof).union(skirt)
    # 발 나사 2: 관통 + 위쪽 경사에 드라이버 구멍
    for s in (-1, 1):
        y = s * FOOT_HOLE_T
        h = h.cut(cq.Workplane("XY").workplane(offset=Z_FOOT).center(FOOT_HOLE_R, y)
                  .circle(M3_CLR / 2).extrude(FOOT_T))
        h = h.cut(cq.Workplane("XY").workplane(offset=Z_FOOT + FOOT_T).center(FOOT_HOLE_R, y)
                  .circle(6.5 / 2).extrude(taper + 1))
    return h


def make_fit():
    f = cq.Workplane("XY").circle(A3_D / 2).extrude(TOP_T)
    for deg in (45, 135, 225, 315):
        x, y = pol(A3_HOLE_R, deg)
        f = f.cut(cq.Workplane("XY").center(x, y).circle(M3_CLR / 2).extrude(TOP_T))
    px, py = pol(A3_PIN_R, 0)
    f = f.union(cq.Workplane("XY").workplane(offset=TOP_T).center(px, py).circle(PIN_D / 2).extrude(PIN_H))
    f = f.cut(cq.Workplane("XY").circle(20).extrude(TOP_T))      # 가운데 비워 출력 시간 줄임
    return f


def preview(base, hood):
    """측면 단면 + 등각 → docs/전달용/배치도/lidar_mount_preview.png"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
    matplotlib.rcParams["axes.unicode_minus"] = False

    def tris(part):
        v, f = part.val().tessellate(0.3, 0.3)
        v = np.array([(p.x, p.y, p.z) for p in v])
        return v[np.array(f)]

    fig = plt.figure(figsize=(15, 6.0))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.45, 1])
    ax = fig.add_subplot(gs[0])
    # 측면 (XZ 단면, 기둥 방위 0° = +X 가 뒤쪽/마스트 쪽)
    sec = cq.Workplane("XZ").rect(400, 400).extrude(0.01, both=True)
    for part, c in ((base, "#b9b9b9"), (hood, "#9fb8d4")):
        for t in tris(part.intersect(sec)):
            ax.fill(t[:, 0], t[:, 2], color=c, lw=0)
    r76, rh = A3_D / 2, A3_HEAD_D / 2
    z0 = PED_H
    ax.add_patch(plt.Rectangle((-r76, z0), A3_D, A3_BASE_H, fc="#a9d18e", ec="k", alpha=.6))
    ax.add_patch(plt.Rectangle((-rh, z0 + A3_HEAD_Z0), A3_HEAD_D, A3_H - A3_HEAD_Z0, fc="#a9d18e", ec="k", alpha=.6))
    zb = PED_H + A3_WIN_C
    ax.axhline(zb, color="#c0392b", ls="--", lw=1.2)
    ax.text(-122, zb + 2, f"레이저 회전면 갑판+{zb:.0f}", color="#c0392b", fontsize=9)
    ax.axhspan(PED_H + A3_WIN_C - A3_WIN_W / 2, Z_WIN_TOP, color="#c0392b", alpha=.12)
    ax.annotate(f"치마 아래끝 {Z_SKIRT0:.0f}\n창 위끝 {Z_WIN_TOP:.0f} (+{Z_SKIRT0 - Z_WIN_TOP:.0f})",
                xy=(-ROOF_R + 1.5, Z_SKIRT0), xytext=(-122, 66), fontsize=8.5,
                arrowprops=dict(arrowstyle="->", lw=.8))
    blocked = 2 * math.degrees(math.atan2(POST_W / 2, POST_R0))
    ax.text(POST_R1 + 3, 22, f"기둥 1개 = 뒤쪽(마스트 쪽)\n가림 {blocked:.0f}°, 앞쪽엔 아무것도 없음", fontsize=8.5)
    ax.text(-30, -8, "갑판", fontsize=8.5)
    ax.axhline(0, color="k", lw=1)
    ax.text(0, Z_A3_TOP - 7, "A3", ha="center", fontsize=9)
    ax.set_xlim(-125, 125); ax.set_ylim(-12, 80); ax.set_aspect("equal")
    ax.set_xlabel("← 앞(선수)        mm        뒤(마스트) →"); ax.set_ylabel("갑판 위 mm")
    ax.set_title("측면 단면 — 받침(회색) · 차양(파랑) · A3(초록)", loc="left", fontsize=11)

    ax3 = fig.add_subplot(gs[1], projection="3d")
    for part, c in ((base, "#b9b9b9"), (hood, "#9fb8d4")):
        t = tris(part)
        n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        lum = 0.55 + 0.45 * np.abs(n @ np.array([-0.4, -0.5, 0.77]))   # 감김 방향이 섞여 있어 abs
        rgb = np.array(matplotlib.colors.to_rgb(c))
        ax3.add_collection3d(Poly3DCollection(t, facecolors=np.clip(rgb * lum[:, None], 0, 1), edgecolor="none"))
    ax3.set_xlim(-65, 65); ax3.set_ylim(-65, 65); ax3.set_zlim(0, 80)
    ax3.set_box_aspect((130, 130, 80)); ax3.view_init(elev=25, azim=-135); ax3.set_axis_off()
    ax3.set_title(f"등각 — 지붕 Ø{2 * ROOF_R:.0f}, 받침 Ø{2 * FLANGE_R:.0f}, 높이 {Z_ROOF1:.0f}", fontsize=11, y=0.9)
    png = os.path.join(HERE, "..", "..", "docs", "전달용", "배치도", "lidar_mount_preview.png")
    fig.tight_layout(); fig.savefig(png, dpi=140)
    print("그림:", os.path.abspath(png))


def main():
    base, hood, fit = make_base(), make_hood(), make_fit()
    for name, part in (("lidar_base", base), ("lidar_hood", hood), ("lidar_fit", fit)):
        cq.exporters.export(part, os.path.join(OUT, name + ".stl"), tolerance=0.05, angularTolerance=0.1)
        cq.exporters.export(part, os.path.join(OUT, name + ".step"))

    # ── 판정 ─────────────────────────────────────────────
    a3 = (cq.Workplane("XY").workplane(offset=PED_H).circle(A3_D / 2).extrude(A3_WIN_C - A3_WIN_W / 2)
          .union(cq.Workplane("XY").workplane(offset=PED_H + A3_WIN_C - A3_WIN_W / 2)
                 .circle(A3_HEAD_D / 2).extrude(A3_H - A3_WIN_C + A3_WIN_W / 2)))
    inter_hood = hood.intersect(a3).val().Volume()
    inter_base = base.intersect(hood).val().Volume()
    blocked = 2 * math.degrees(math.atan2(POST_W / 2, POST_R0))
    screw_in = 6.0 - TOP_T
    print(f"회전면(빔) 높이: 갑판 + {PED_H + A3_WIN_C:.0f} mm  (갑판 직치 31 대비 +{PED_H:.0f})")
    print(f"차양 치마 아래끝 {Z_SKIRT0:.0f} > 레이저 창 위끝 {Z_WIN_TOP:.0f}  (여유 {Z_SKIRT0 - Z_WIN_TOP:.0f} mm) → "
          f"{'OK' if Z_SKIRT0 - Z_WIN_TOP >= 5 else '🚨 빔 가림'}")
    print(f"기둥이 가리는 뒤쪽 각도: {blocked:.1f}° (마스트 사각 ≈10° 와 같은 쪽에 둘 것)")
    print(f"간섭: 차양∩A3 {inter_hood:.1f} mm³, 받침∩차양 {inter_base:.1f} mm³ → "
          f"{'없음' if inter_hood < 1e-3 and inter_base < 1e-3 else '🚨 있음'}")
    print(f"LiDAR 나사: M3×6 → 물림 {screw_in:.0f} mm (한계 {A3_SCREW_MAX:.0f}) → "
          f"{'OK' if screw_in <= A3_SCREW_MAX else '🚨 내부 손상'}  · M3×8 금지(물림 5)")
    print(f"전체 높이: 갑판 + {Z_ROOF1:.0f} mm, 받침 지름 {2 * FLANGE_R:.0f}, 지붕 지름 {2 * ROOF_R:.0f}")
    print("출력:", ", ".join(f"out/{n}.stl" for n in ("lidar_base", "lidar_hood", "lidar_fit")))
    preview(base, hood)


if __name__ == "__main__":
    main()

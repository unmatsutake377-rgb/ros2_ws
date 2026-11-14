#!/usr/bin/env python3
"""장애물 구간(mode 5, WP6→WP7) 배치 — 긴 흰 벽 + 누운 흰 펜더 (full_course_sim --avoid-obs).

왜 (11-13 현장 사진, 2번배 세션):
  실제 장애물 구간은 둥근 부표가 아니라 '긴 흰 벽(양 끝 펜더 달린 부유 바) 5개 + 누운 흰 펜더 약 14개'를
  로프(노란 소형 부이)로 둘러싼 직사각형이다. 옛 시뮬(부표 6~10개 무작위)은 이 구조를 대표하지 못했다.
  사용자 판단: LiDAR 가 누운 흰 부표와 같은 높이라 벽·펜더도 본다 → 레이캐스트 대상(선분)으로 넣는다.

모양 (전부 2D 선분, 순수 함수 — ROS·노드 코드 안 씀)
  · 벽 = 선분(양 끝 펜더 포함 길이), 반두께 WALL_R. 펜더 = 길이 FENDER_L 선분, 반지름 FENDER_R.
  · LiDAR 는 선분에 레이캐스트(두께 0), 접촉 = 선체 중심선(±0.83 m)과 선분 거리 < 선체 반폭 + 반두께.

구간 좌표계 zone_frame
  · 폴리곤(geofence_avoid_polygon, 4점)이 있으면 직사각형 근사: 중심 = 꼭짓점 평균, 긴 변 방향(WP6→WP7 쪽) e_s,
    길이 Lz = 긴 두 변 평균, 폭 Wz = 짧은 두 변 평균. 없으면 WP6→WP7 선 기준(길이 + 3.2 m, 폭 10.9 m — 11-13 폴리곤 값).
  · s = 진입 끝(WP6 쪽)에서 잰 거리, l = 진행 방향 왼쪽(+).

photo (11-13 사진 1장 손 디지타이즈 — ⚠️ 추정. 실측 아님)
  · 사진 픽셀(1400×1050)에서 로프 네 모서리로 (u, v) 를 정하고, 원근은 먼 쪽/가까운 쪽 크기비 PHOTO_DEPTH_RATIO 1.3
    (펜더 겉보기 폭 28 px vs 37 px)으로 v 만 보정한다. 오른쪽 아래 모서리는 화면 밖이라 (1420, 648) 로 가정.
  · 사진을 어느 둑에서 찍었는지 모른다 → photo_entry 'right'(배가 사진 오른쪽 끝으로 들어감 = 남쪽 둑에서 찍음) /
    'left'(왼쪽 끝 = 북쪽 둑). 거울상은 물리적으로 불가능해 두 가지뿐.

random (시드마다) — 실제 규칙(11-13 사용자): 장애물은 부표이고, 몇 쌍의 부표 사이에 벽이 걸린다.
  · 부표(펜더) 16~24개를 서로 1.5 m 이상 무작위로 놓고, 중심 거리 2.0~3.5 m 인 쌍 중 3~6쌍을 벽으로 잇는다
    (사진: 벽 5·벽 끝 펜더 10·따로 떠 있는 펜더 12 = 부표 22).
  · 진입 끝에서 12~88 % 구간, 폭 방향 가장자리 1.0 m 안쪽, WP6·WP7 에서 2.5 m 이상.
    벽은 다른 부표·벽과 1.2 m 이상. 전체 통과 가능성은 보장하지 않는다(국소 틈만).
"""
import math

import numpy as np

WALL_R = 0.15          # 벽 반두께 m  ⚠️ 사진 추정
FENDER_R = 0.15        # 펜더 반지름 m ⚠️ 사진 추정
FENDER_L = 0.7         # 펜더 길이 m  ⚠️ 사진 추정
SEG_R = {"wall": WALL_R, "fender": FENDER_R}
ZONE_W_DEFAULT = 10.9  # 11-13 녹화 폴리곤 짧은 변
MIN_GAP = 1.5          # 부표(펜더 선분)끼리 최소 간격 — 옛 부표 규칙과 같음
WALL_CLEAR = 1.2       # 벽 ↔ 다른 부표·벽 최소 간격(배 폭 0.58 + 여유)
WP_CLEAR = 2.5

# ---- 사진 디지타이즈 (픽셀) ----
PHOTO_TOP = ((190.0, 545.0), (1160.0, 532.0))     # 먼 쪽 로프: 왼쪽 빨간 원통 → 오른쪽 빨간 원통
PHOTO_BOT = ((135.0, 688.0), (1420.0, 648.0))     # 가까운 쪽 로프: 왼쪽 빨간 원통 → (화면 밖 가정)
PHOTO_DEPTH_RATIO = 1.3
PHOTO_WALLS = [((440, 578), (595, 602)), ((452, 620), (512, 680)), ((663, 542), (708, 588)),
               ((925, 572), (1012, 620)), ((1083, 590), (1137, 630))]
PHOTO_FENDERS = [(163, 577), (160, 628), (438, 549), (565, 563), (427, 620), (607, 637), (722, 610),
                 (803, 560), (836, 592), (880, 543), (888, 632), (1037, 561)]


def zone_frame(wp6, wp7, poly_xy=None):
    """→ dict(c, e_s, e_l, Lz, Wz, entry). wp6·wp7 = 로컬 xy(np), poly_xy = [(x, y)×4] 또는 None."""
    d = np.asarray(wp7, float) - np.asarray(wp6, float)
    d = d / np.linalg.norm(d)
    if poly_xy is not None and len(poly_xy) == 4:
        P = [np.asarray(p, float) for p in poly_xy]
        edges = [(P[i], P[(i + 1) % 4]) for i in range(4)]
        lens = [float(np.linalg.norm(b - a)) for a, b in edges]
        order = sorted(range(4), key=lambda i: -lens[i])
        e = np.zeros(2)
        for i in order[:2]:
            v = edges[i][1] - edges[i][0]
            e += v if v @ d > 0 else -v
        e_s = e / np.linalg.norm(e)
        Lz = (lens[order[0]] + lens[order[1]]) / 2.0
        Wz = (lens[order[2]] + lens[order[3]]) / 2.0
        c = sum(P) / 4.0
    else:
        e_s = d
        Lz = float(np.linalg.norm(np.asarray(wp7, float) - np.asarray(wp6, float))) + 3.2
        Wz = ZONE_W_DEFAULT
        c = (np.asarray(wp6, float) + np.asarray(wp7, float)) / 2.0
    e_l = np.array([-e_s[1], e_s[0]])
    return dict(c=c, e_s=e_s, e_l=e_l, Lz=Lz, Wz=Wz, entry=c - e_s * Lz / 2.0)


def to_xy(F, s, l):
    return F["entry"] + F["e_s"] * s + F["e_l"] * l


def _photo_uv(x, y):
    """사진 픽셀 → (u 0=왼끝 1=오른끝, v_phys 0=먼 쪽 1=가까운 쪽). 원근은 v 만 보정."""
    (tx0, ty0), (tx1, ty1) = PHOTO_TOP
    (bx0, by0), (bx1, by1) = PHOTO_BOT
    yt = ty0 + (ty1 - ty0) * (x - tx0) / (tx1 - tx0)
    yb = by0 + (by1 - by0) * (x - bx0) / (bx1 - bx0)
    v = (y - yt) / (yb - yt)
    xl = tx0 + (bx0 - tx0) * v
    xr = tx1 + (bx1 - tx1) * v
    u = (x - xl) / (xr - xl)
    zf, zn = PHOTO_DEPTH_RATIO, 1.0                    # 먼 쪽 깊이 / 가까운 쪽 깊이
    z = 1.0 / (1.0 / zf + v * (1.0 / zn - 1.0 / zf))   # 영상 행은 1/깊이에 선형
    vp = (zf - z) / (zf - zn)
    return u, vp


def _photo_sl(F, x, y, entry):
    u, vp = _photo_uv(x, y)
    if entry == "right":                       # 남쪽 둑: 오른쪽 = 진입, 가까운 쪽 = 진행 왼쪽
        return (1.0 - u) * F["Lz"], (vp - 0.5) * F["Wz"]
    return u * F["Lz"], -(vp - 0.5) * F["Wz"]  # 북쪽 둑: 왼쪽 = 진입, 가까운 쪽 = 진행 오른쪽


def photo_objects(F, entry="right"):
    """사진 배치 → [(p, q, kind)]. 펜더는 구간 긴 방향으로 눕힌다(사진에서 대체로 가로)."""
    out = []
    for (x0, y0), (x1, y1) in PHOTO_WALLS:
        out.append((to_xy(F, *_photo_sl(F, x0, y0, entry)), to_xy(F, *_photo_sl(F, x1, y1, entry)), "wall"))
    h = F["e_s"] * FENDER_L / 2.0
    for x, y in PHOTO_FENDERS:
        p = to_xy(F, *_photo_sl(F, x, y, entry))
        out.append((p - h, p + h, "fender"))
    return out


def _pt_seg(p, a, b):
    ab = b - a; L2 = float(ab @ ab)
    s = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, float((p - a) @ ab) / L2))
    return float(np.linalg.norm(p - (a + s * ab)))


def seg_dist(a1, a2, b1, b2):
    d1, d2 = a2 - a1, b2 - b1
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) > 1e-12:
        r = b1 - a1
        s = (r[0] * d2[1] - r[1] * d2[0]) / den; u = (r[0] * d1[1] - r[1] * d1[0]) / den
        if 0 <= s <= 1 and 0 <= u <= 1:
            return 0.0
    return min(_pt_seg(a1, b1, b2), _pt_seg(a2, b1, b2), _pt_seg(b1, a1, a2), _pt_seg(b2, a1, a2))


def random_objects(F, wp6, wp7, rng, n_buoys=(16, 24), n_walls=(3, 6), wall_span=(2.0, 3.5)):
    """시드 rng(random.Random) → [(p, q, kind)].

    실제 규칙(11-13 사용자): 장애물은 부표(누운 흰 펜더)이고, 그중 몇 쌍 사이에 벽이 걸린다.
      1) 부표 n_buoys 개를 서로 MIN_GAP 이상 떨어뜨려 무작위로 놓는다(방향 무작위).
      2) 중심 거리 wall_span 안인 부표 쌍 중 무작위로 n_walls 쌍을 골라 중심끼리 벽으로 잇는다.
         한 부표엔 벽 하나만, 벽은 다른 물체와 WALL_CLEAR 이상(두 끝 부표 제외), 벽끼리도 같은 간격.
         벽이 걸린 부표는 벽 방향으로 눕힌다(사진처럼 벽 끝에 펜더).
    """
    keep = [np.asarray(wp6, float), np.asarray(wp7, float)]
    lmax = F["Wz"] / 2.0 - 1.0
    buoys = []                                   # [중심, 방향 단위벡터]
    for _ in range(rng.randint(*n_buoys)):
        for _t in range(4000):
            c = to_xy(F, rng.uniform(0.12, 0.88) * F["Lz"], rng.uniform(-lmax, lmax))
            th = rng.uniform(0.0, math.pi)
            u = np.array([math.cos(th), math.sin(th)])
            p, q = c - u * FENDER_L / 2.0, c + u * FENDER_L / 2.0
            if any(_pt_seg(k, p, q) < WP_CLEAR for k in keep):
                continue
            if any(seg_dist(p, q, c2 - u2 * FENDER_L / 2.0, c2 + u2 * FENDER_L / 2.0) < MIN_GAP for c2, u2 in buoys):
                continue
            buoys.append([c, u])
            break
    pairs = [(i, j) for i in range(len(buoys)) for j in range(i + 1, len(buoys))
             if wall_span[0] <= float(np.linalg.norm(buoys[i][0] - buoys[j][0])) <= wall_span[1]]
    rng.shuffle(pairs)
    used, walls = set(), []
    want = rng.randint(*n_walls)
    for i, j in pairs:
        if len(walls) >= want:
            break
        if i in used or j in used:
            continue
        a, b = buoys[i][0], buoys[j][0]
        if any(_pt_seg(buoys[k][0], a, b) < WALL_CLEAR for k in range(len(buoys)) if k not in (i, j)):
            continue
        if any(seg_dist(a, b, w0, w1) < WALL_CLEAR for w0, w1 in walls):
            continue
        walls.append((a, b)); used.update((i, j))
        d = (b - a) / float(np.linalg.norm(b - a))
        buoys[i][1] = d; buoys[j][1] = d
    out = [(a, b, "wall") for a, b in walls]
    out += [(c - u * FENDER_L / 2.0, c + u * FENDER_L / 2.0, "fender") for c, u in buoys]
    return out

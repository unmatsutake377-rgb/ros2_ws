"""도크 형상(선분) 생성 — 순수 함수 (numpy 만, ROS·노드 import 없음). full_course_sim 이 쓴다.

좌표 규약 (full_course_sim 과 같음, 단위 m)
  O = 칸 안쪽 벽(뒷벽) 중심.  n = 물 쪽(칸 입구 쪽) 단위 법선.  t = (n_y, −n_x) = n 을 시계로 90° 돌린 방향.
  칸 k(−1, 0, +1 …) 의 축 = O + t·k·pitch,  pitch = bay_w + gap (칸 중심 간격).
  칸 입구 = 칸 축 위 O + depth·n.  칸막이(덩어리) = 칸 사이·양 끝의 폭 gap, 깊이 depth 인 직사각형.

선분 종류 (kind)
  "front" : 뒷벽(칸 안쪽 벽). 양 끝 덩어리 바깥 끝까지 한 줄 (−half … +half).
  "div"   : 칸 옆벽(칸 가장자리, 깊이 0 → depth)과 양 끝 덩어리 바깥 벽.
  "cap"   : 덩어리의 물 쪽 끝면(깊이 depth, 칸 입구와 같은 선). gap = 0 이면 없다.
  half = n_bays·bay_w/2 + (n_bays+1)·gap/2  (양 끝 덩어리 바깥 끝까지).

기본값(bay_w 1.0, gap 0.0, depth 0.5, 칸 3개) = full_course_sim 의 옛 형상과 **선분·순서까지 같다**:
  front (O−1.5t → O+1.5t), div 4개(가로 −1.5, −0.5, 0.5, 1.5, 깊이 0 → 0.5). 같은 자리 선분은 하나로 합친다.
규정 그림(2025-11) 도크 = bay_w 2.0, gap 1.5, depth 3.0 → 전체 12 m.
"""
import numpy as np

DOCK_DEFAULT = dict(bay_w=1.0, gap=0.0, depth=0.5, n_bays=3)
_EPS = 1e-9


def bay_offsets(n_bays):
    """칸 번호(가로 단위 pitch). 3 → (-1, 0, 1)."""
    return [k - (n_bays - 1) / 2.0 for k in range(n_bays)]


def dock_geometry(O, n, bay_w=1.0, gap=0.0, depth=0.5, n_bays=3):
    """도크 선분과 기준점. → dict

      O, n, t, pitch, half, bay_w, gap, depth, n_bays
      bays      : [{k, center(뒷벽 위 칸 중심 = 표식 자리), entrance(칸 입구 중심), lo, hi(가로 경계)}]
      tips      : 칸 입구 양옆 칸막이 끝 점(깊이 depth, 가로 = 칸 가장자리) 목록 [(가로, 점)]
      segs      : [(a, b, kind)] — full_course_sim W["segs"] 형식
    """
    O = np.asarray(O, dtype=float)
    n = np.asarray(n, dtype=float)
    n = n / np.linalg.norm(n)
    t = np.array([n[1], -n[0]])
    if bay_w <= 0 or gap < 0 or depth <= 0 or n_bays < 1:
        raise ValueError(f"도크 치수 이상: bay_w={bay_w} gap={gap} depth={depth} n_bays={n_bays}")
    pitch = bay_w + gap
    half = n_bays * bay_w / 2.0 + (n_bays + 1) * gap / 2.0
    ks = bay_offsets(n_bays)
    bays, edges, tips = [], [], []
    for k in ks:
        c = k * pitch
        lo, hi = c - bay_w / 2.0, c + bay_w / 2.0
        bays.append(dict(k=k, center=O + t * c, entrance=O + t * c + depth * n, lo=lo, hi=hi))
        edges += [lo, hi]
        tips += [(lo, O + t * lo + depth * n), (hi, O + t * hi + depth * n)]
    segs = [(O - half * t, O + half * t, "front")]
    walls = []
    for s in [-half] + sorted(edges) + [half]:
        if not walls or abs(s - walls[-1]) > _EPS:
            walls.append(s)
    for s in walls:
        segs.append((O + s * t, O + s * t + depth * n, "div"))
    if gap > _EPS:
        # 덩어리 끝면: (−half, 첫 칸 lo), (칸 hi, 다음 칸 lo) …, (마지막 칸 hi, +half)
        stops = [-half] + [e for b in bays for e in (b["lo"], b["hi"])] + [half]
        for a0, b0 in zip(stops[0::2], stops[1::2]):
            if b0 - a0 > _EPS:
                segs.append((O + a0 * t + depth * n, O + b0 * t + depth * n, "cap"))
    return dict(O=O, n=n, t=t, pitch=pitch, half=half, bay_w=bay_w, gap=gap, depth=depth,
                n_bays=n_bays, bays=bays, tips=tips, segs=segs)


def extent(geom):
    """O 에서 도크 선분 끝까지 최대 거리(LiDAR·접촉 검사 반경 산정용)."""
    return float(np.hypot(geom["half"], geom["depth"]))

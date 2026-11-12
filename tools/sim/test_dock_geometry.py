"""도크 형상(dock_geometry) + full_course_sim LiDAR 레이캐스트 + 세계 파일 테스트 — ROS 불필요.

  python3 -m pytest tools/sim/test_dock_geometry.py -q

검증
  · 순수 함수: 칸 수·칸 중심 간격(pitch)·칸 입구·칸막이 끝 위치·전체 길이, 기본값 = 옛 full_course_sim 선분(순서까지).
  · full_course_sim.lidar() 로 ship_dock 과 같은 정의(정면 ±front_sector_half_deg 15°, 공간 median kernel 5 —
    ship_dock.dock_fsm_logic.median_min) 의 전방 최소거리가 닫힌 식과 맞는지:
      목표 칸 축 위 칸 입구 앞 2 m · 입구 · 칸 안 1 m (칸 폭 2 m·3 m, 칸 −1/0/+1).
  · 축에서 옆으로 e 만큼 비켜 칸 쪽으로 곧장 들어갈 때 전방 거리가 contact 1.0 m 이하가 되는 첫 위치와 맞은 물체:
      e 0 · 0.6 → 뒷벽(배 중심이 뒷벽 1.0 m), e 1.2 → 칸막이 끝면(cap, 입구 앞 1.0 m) = 거짓 CONTACT,
      경계 e* = 칸폭/2 − contact·sin(15°) (= 0.741 m) 바로 안·밖.
  · 세계 파일 arena_2025.yaml ↔ 위경도 사본 arena_2025_wp.yaml 일치, 13점 OT 순서(mode).
"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from dock_geometry import dock_geometry, DOCK_DEFAULT  # noqa: E402

fcs = pytest.importorskip("full_course_sim")            # 노드 9개를 rosstub 로 읽는다(ROS 불필요)
from ship_dock.dock_fsm_logic import median_min, MEDIAN_MIN_VALID  # noqa: E402

SECTOR = 15.0      # ship_dock front_sector_half_deg 기본
KERNEL = 5         # ship_dock front_median_kernel 기본
CONTACT = 1.0      # ship_dock contact_dist_m 기본


# ───────────────────────── 순수 함수 ─────────────────────────
def test_default_equals_old_segments():
    O = np.array([3.0, -2.0]); n = np.array([0.6, 0.8])
    g = dock_geometry(O, n, **{k: DOCK_DEFAULT[k] for k in ("bay_w", "gap", "depth", "n_bays")})
    t = np.array([n[1], -n[0]])
    old = [(O - 1.5 * t, O + 1.5 * t, "front")] + [(O + s * t, O + s * t + 0.5 * n, "div") for s in (-1.5, -0.5, 0.5, 1.5)]
    assert len(g["segs"]) == len(old)
    for (a, b, k), (a2, b2, k2) in zip(g["segs"], old):
        assert k == k2 and np.array_equal(a, a2) and np.array_equal(b, b2)      # 비트 단위로 같다
    assert g["pitch"] == 1.0 and g["half"] == 1.5


@pytest.mark.parametrize("bw,gap,depth", [(2.0, 1.5, 3.0), (3.0, 1.5, 3.0), (1.0, 0.5, 0.5)])
def test_rule_dock_layout(bw, gap, depth):
    O = np.array([68.0, 6.0]); n = np.array([-1.0, 0.0])          # 입구 서쪽 → t = (0, 1) 북
    g = dock_geometry(O, n, bw, gap, depth, 3)
    assert np.allclose(g["t"], [0.0, 1.0])
    assert len(g["bays"]) == 3 and g["pitch"] == pytest.approx(bw + gap)
    cy = [b["center"][1] for b in g["bays"]]
    assert np.allclose(np.diff(cy), bw + gap)
    assert g["half"] * 2 == pytest.approx(3 * bw + 4 * gap)       # 규정 그림: 2·3 + 1.5·4 = 12 m
    for b in g["bays"]:
        assert b["center"][0] == pytest.approx(68.0)
        assert np.allclose(b["entrance"], b["center"] + depth * n)
        assert b["hi"] - b["lo"] == pytest.approx(bw)
    # 칸막이 끝(칸 입구 양옆) = 깊이 depth, 가로 = 칸 가장자리
    assert len(g["tips"]) == 6
    for lat, p in g["tips"]:
        assert np.allclose(p, [68.0 - depth, 6.0 + lat])
    kinds = [k for _, _, k in g["segs"]]
    assert kinds.count("front") == 1 and kinds.count("div") == 8 and kinds.count("cap") == 4
    caps = sorted(((a[1], b[1]) for a, b, k in g["segs"] if k == "cap"))
    assert all(hi - lo == pytest.approx(gap) for lo, hi in caps)   # 덩어리 끝면 폭 = 칸막이 두께
    assert all(a[0] == pytest.approx(68.0 - depth) for a, _, k in g["segs"] if k == "cap")


def test_rule_dock_total_12m():
    g = dock_geometry((0, 0), (0, 1), 2.0, 1.5, 3.0, 3)
    assert g["half"] == pytest.approx(6.0)


# ───────────────────────── LiDAR 레이캐스트 vs 기하 ─────────────────────────
def make_world(bw, gap=1.5, depth=3.0):
    """뒷벽 중심 O = 원점, 물 쪽 n = +y(북) → 배는 남쪽(180°)으로 들어간다. t = (1, 0) 동."""
    g = dock_geometry((0.0, 0.0), (0.0, 1.0), bw, gap, depth, 3)
    W = dict(bx=np.array([1e4]), by=np.array([1e4]),                 # 부표 없음(멀리 하나 — 빈 배열 회피)
             buoys=[dict(x=1e4, y=1e4, kind="far", color="orange")],
             segs=g["segs"], seg_arr=np.array([[a[0], a[1], b[0], b[1]] for a, b, _ in g["segs"]]),
             dock=dict(O=g["O"], reach=max(3.0, math.hypot(g["half"], depth))))
    return W, g


def front(W, x, y, psi=180.0):
    r = fcs.lidar(W, x, y, psi)
    lo = int((80.0 - SECTOR - fcs.K.SCAN_ANGLE_MIN_DEG) / fcs.K.SCAN_INC_DEG)
    hi = int((80.0 + SECTOR - fcs.K.SCAN_ANGLE_MIN_DEG) / fcs.K.SCAN_INC_DEG)
    d, i = median_min(list(r), lo, hi, KERNEL, MEDIAN_MIN_VALID)
    return d, fcs.ray_hit_kind(W, x, y, psi, i)


def closed_form_on_axis(s, bw, depth, alpha=SECTOR):
    """칸 축 위, 뒷벽까지 s, 정면(뒷벽 수직) — 섹터 안 최소거리.
    정면 광선 = 뒷벽 s. 섹터 끝 광선(α)이 칸 옆벽(가로 bw/2)을 깊이 [0, depth] 안에서 맞으면 (bw/2)/sin α.
    (kernel 5 median 은 단조 구간에서 가운데 광선 값 = 끝 광선 값을 준다.)"""
    h = bw / 2.0; a = math.radians(alpha)
    d = s
    reach = h / math.tan(a)                 # 옆벽을 맞히는 데 필요한 전진 거리
    if 0.0 <= s - reach <= depth and (s <= depth or (s - depth) * math.tan(a) <= h):
        d = min(d, h / math.sin(a))
    return d


@pytest.mark.parametrize("bw", [2.0, 3.0])
@pytest.mark.parametrize("k", [-1, 0, 1])
@pytest.mark.parametrize("ahead", [2.0, 0.0, -1.0])     # 칸 입구 앞 2 m · 입구 · 칸 안 1 m
def test_front_sector_on_bay_axis(bw, k, ahead):
    W, g = make_world(bw)
    x = k * g["pitch"]; s = g["depth"] + ahead          # 배 중심 → 뒷벽 거리
    d, kind = front(W, x, s)
    exp = closed_form_on_axis(s, bw, g["depth"])
    assert d == pytest.approx(exp, abs=0.01), (bw, k, ahead, d, exp, kind)
    # 칸 폭 2 m 는 입구 앞 2 m 에서 옆벽(섹터 끝)이 뒷벽보다 가깝다: 1/sin15° = 3.86 m (뒷벽 5 m)
    if bw == 2.0 and ahead == 2.0:
        assert kind == "div" and d == pytest.approx(1.0 / math.sin(math.radians(SECTOR)), abs=0.01)
    else:
        assert kind == "front"


def first_contact(W, x, y0=6.0, y1=0.3, step=0.01):
    """배 중심을 (x, y0) → (x, y1) 로 곧장(헤딩 180°) 옮기며 전방 거리 ≤ CONTACT 가 되는 첫 y 와 맞은 물체."""
    for y in np.arange(y0, y1, -step):
        d, kind = front(W, x, float(y))
        if d is not None and d <= CONTACT:
            return float(y), kind, d
    return None, None, None


@pytest.mark.parametrize("k", [-1, 0, 1])
def test_lateral_offset_false_contact(k):
    W, g = make_world(2.0)
    xc = k * g["pitch"]; D = g["depth"]
    # 축 위·0.6 m 옆: 칸막이 끝이 1.0 m 섹터에 안 들어온다 → 뒷벽 1.0 m 에서 CONTACT (진짜 접안)
    for e in (0.0, 0.6, -0.6):
        y, kind, _ = first_contact(W, xc + e)
        assert kind == "front" and y == pytest.approx(CONTACT, abs=0.02), (e, y, kind)
    # 1.2 m 옆(칸 폭 2 m 의 반 1.0 밖): 칸막이 끝면 정면 → 입구 앞 1.0 m 에서 CONTACT (거짓)
    for e in (1.2, -1.2):
        y, kind, _ = first_contact(W, xc + e)
        assert kind == "cap" and y == pytest.approx(D + CONTACT, abs=0.02), (e, y, kind)


def test_lateral_threshold_closed_form():
    """칸막이 끝(가로 bw/2)이 '섹터 안 + 1.0 m 안' 에 드는 최소 옆 비킴 e* = bw/2 − contact·sin(α)."""
    W, g = make_world(2.0)
    e_star = 1.0 - CONTACT * math.sin(math.radians(SECTOR))
    assert e_star == pytest.approx(0.741, abs=1e-3)
    y_in, k_in, _ = first_contact(W, e_star - 0.05)          # 안: 뒷벽에서
    assert k_in == "front" and y_in == pytest.approx(CONTACT, abs=0.02)
    y_out, k_out, _ = first_contact(W, e_star + 0.05)        # 밖: 칸막이 끝(입구 근처, 배 중심 깊이 > D)
    assert k_out in ("div", "cap") and y_out > g["depth"], (y_out, k_out)


# ───────────────────────── 세계 파일 ─────────────────────────
WORLD = os.path.join(HERE, "worlds", "arena_2025.yaml")


def test_world_wp_file_matches():
    w = fcs.load_world(WORLD)
    with open(w["_wp_path"], encoding="utf-8") as f:
        assert f.read() == fcs.world_wp_text(w), "arena_2025_wp.yaml 이 세계 파일과 다르다 — --write-world-wp 로 다시 만들 것"
    wps = fcs.load_wps(w["_wp_path"])
    assert [p["mode"] for p in wps] == [0, 1, 2, 5, 7, 7, 5, 3, 3, 3, 5, 5, 8]
    geo = fcs.world_geo(w)
    for p, q in zip(wps, w["waypoints"]):
        x, y = geo.xy(p["lat"], p["lon"])
        assert math.hypot(x - q["xy"][0], y - q["xy"][1]) < 1e-3


def test_world_build_dock_and_align():
    w = fcs.load_world(WORLD)
    wps = fcs.load_wps(w["_wp_path"])
    W = fcs.build_world(wps, 1, fcs.resolve_dock_cfg(w, dict(target=1.0)), w)
    d = W["dock"]
    assert d["bay"] == 1.0 and np.allclose(d["marker"], [68.0, 9.5]) and d["bay_w"] == 2.0
    P = W["P"]
    # 정렬점 = 가운데 칸 입구 정면 6 m, WP8 = 가운데 칸 입구
    entr0 = d["O"] + d["depth"] * d["n"]
    assert np.linalg.norm(P[3] - entr0) == pytest.approx(6.0, abs=1e-3)
    assert np.linalg.norm(P[4] - entr0) < 1e-3
    assert len(W["gates"]) == 5 and not any(b["kind"] == "obst_8" for b in W["buoys"])
    p = fcs.align_target_file(w["_wp_path"], W)
    moved = fcs.load_wps(p)
    geo = W["geo"]
    for i, dy in ((3, 3.5), (4, 3.5), (5, 3.5), (2, 0.0), (6, 0.0)):
        x0, y0 = geo.xy(wps[i]["lat"], wps[i]["lon"]); x1, y1 = geo.xy(moved[i]["lat"], moved[i]["lon"])
        assert x1 - x0 == pytest.approx(0.0, abs=1e-3) and y1 - y0 == pytest.approx(dy, abs=1e-3)

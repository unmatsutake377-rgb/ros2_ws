#!/usr/bin/env python3
"""도킹 목표 칸 정면 접근 웨이포인트 계산기 (대회장용, ROS 불필요).

목표 칸 입구 중심 좌표와 '도크에서 물 쪽으로 나가는 방향'을 주면, 그 방향(도크 법선) 위
far·near 거리(기본 10 m·6 m)의 접근점 두 개와 WP8(칸 입구)·WP9(이탈점) 까지 네 줄을 찍는다.
waypoints.yaml 의 '도킹 접근 10m' ~ 'WP9 도킹 이탈' 네 줄(⚠️ 현장 교체 자리표시값)을 통째로 바꾼다.

  # 방향을 도크 정면 양 끝 두 점으로 (물에서 도크를 바라볼 때 왼쪽 끝 L, 오른쪽 끝 R)
  python3 tools/dock_wp.py --bay 35.1859269 128.5655428 \
      --left 35.18593 128.56551 --right 35.18592 128.56558

  # 방향을 나침반 방위로 (도크 → 물 쪽, 0 = 북, 시계방향)
  python3 tools/dock_wp.py --bay 35.1859269 128.5655428 --out-deg 200

  # 세 칸을 한 번에 (칸 중심 세 개를 각각 --bay 로)
  python3 tools/dock_wp.py --bay <왼칸> --bay <가운데칸> --bay <오른칸> --left ... --right ...

왜 두 점인가: 도킹이 아닌 웨이포인트는 2.5 m 안에 들면 도착이라(los_logic.get_dynamic_arrive_radius),
하나만 두면 옆에서 들어온 채 도킹 모드로 넘어간다. 법선 위 두 점이면 마지막 구간을 정면으로 들어온다.
시뮬(tools/sim/dock_sim.py 의 run, 300 런): 칸 안 접안 무작위 출발 186/300 → 법선 위 2점 300/300(횡오차 σ0.4 m 가정).
왜 WP9 = 가까운 접근점(mode 7): 도킹이 끝나면 칸 앞 near 거리로 되나온 뒤 WP10 으로 간다. mode 7 을
이어 두면(7→7) ship_dock 이 리셋되지 않아 도킹을 다시 시작하지 않는다. WP9 를 칸 좌표로 두면 도킹을
마친 배가 다시 칸으로 들어간다(계약서 docs/문제와작업/계약서_전체코스_20261104.md §2-3).
절차: docs/절차/도킹_칸별_웨이포인트.md
"""
import argparse
import math

R_EARTH = 6371000.0


def to_xy(lat0, lon0, lat, lon):
    """기준점 기준 동(x)·북(y) [m] (수십 m 범위 평면 근사)."""
    x = math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * R_EARTH
    return x, y


def to_ll(lat0, lon0, x, y):
    lat = lat0 + math.degrees(y / R_EARTH)
    lon = lon0 + math.degrees(x / (R_EARTH * math.cos(math.radians(lat0))))
    return lat, lon


def out_bearing_from_corners(left, right):
    """물에서 도크를 볼 때 왼쪽 끝 L → 오른쪽 끝 R. 물 쪽 법선 = L→R 을 시계방향 90° 회전."""
    x, y = to_xy(left[0], left[1], right[0], right[1])
    if math.hypot(x, y) < 0.5:
        raise SystemExit("❌ L·R 두 점이 0.5 m 도 안 떨어졌다 — 도크 양 끝을 다시 찍어라")
    nx, ny = y, -x                                  # 시계방향 90°
    return math.degrees(math.atan2(nx, ny)) % 360.0, math.hypot(x, y)


def main():
    ap = argparse.ArgumentParser(description="도킹 목표 칸 정면 접근 웨이포인트 계산")
    ap.add_argument("--bay", nargs=2, type=float, action="append", required=True, metavar=("LAT", "LON"),
                    help="목표 칸 입구 중심(도크 정면 선 위). 여러 칸이면 여러 번")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--out-deg", type=float, help="도크 → 물 쪽 방향, 나침반 방위 [deg]")
    g.add_argument("--left", nargs=2, type=float, metavar=("LAT", "LON"), help="물에서 볼 때 도크 정면 왼쪽 끝")
    ap.add_argument("--right", nargs=2, type=float, metavar=("LAT", "LON"), help="물에서 볼 때 도크 정면 오른쪽 끝")
    ap.add_argument("--far", type=float, default=10.0, help="먼 접근점 거리 [m] (기본 10)")
    ap.add_argument("--near", type=float, default=6.0, help="가까운 접근점 거리 [m] (기본 6, 회피 감지 3 m 보다 멀게)")
    ap.add_argument("--mode", type=int, default=5, help="접근점 mode (기본 5 = 회피 주행. 7 로 두면 다가가는 동안 도킹 노드가 켜진다)")
    ap.add_argument("--dwell", type=float, default=1.0,
                    help="접근점·WP8·WP9 머무는 시간 [s] (기본 1.0 = waypoints.yaml mode 5·7)")
    a = ap.parse_args()
    if a.left is not None and a.right is None:
        ap.error("--left 를 쓰면 --right 도 필요")
    if a.near <= 3.0:
        print("⚠️ near ≤ 3 m: 회피 모드(detection_distance_default 3.0 m)가 도크 벽을 장애물로 볼 수 있다")
    if a.far <= a.near + 2.5:
        print("⚠️ far − near ≤ 2.5 m: 도착 반경(2.5 m) 때문에 두 점이 사실상 하나처럼 동작한다")
    if a.mode == 7:
        print("⚠️ mode 7: 접근 중에도 ship_dock 이 켜진다 — 의도한 게 아니면 5")

    if a.out_deg is not None:
        brg = a.out_deg % 360.0
        print(f"물 쪽 방향 {brg:.1f}° (입력)")
    else:
        brg, span = out_bearing_from_corners(a.left, a.right)
        print(f"물 쪽 방향 {brg:.1f}° (도크 정면 폭 {span:.2f} m 에서 계산 — 폭이 실제 도크와 맞는지 확인)")
    ux, uy = math.sin(math.radians(brg)), math.cos(math.radians(brg))
    names = {1: ["목표칸"], 3: ["왼쪽칸", "가운데칸", "오른쪽칸"]}.get(len(a.bay), [f"칸{i + 1}" for i in range(len(a.bay))])
    if len(a.bay) > 1:
        print("※ 칸이 여럿이면 대회 당일 공지된 목표 칸 하나의 네 줄만 쓴다")
    for nm, (lat, lon) in zip(names, a.bay):
        f = to_ll(lat, lon, ux * a.far, uy * a.far)
        n = to_ll(lat, lon, ux * a.near, uy * a.near)
        print(f"\n# ── {nm}: waypoints.yaml 의 '도킹 접근 {a.far:.0f}m' ~ 'WP9 도킹 이탈' 네 줄을"
              f" 아래 네 줄로 통째로 교체 ──")
        rows = [(f"도킹 접근 {a.far:.0f}m({nm})", f, a.mode),
                (f"도킹 접근 {a.near:.0f}m({nm})", n, a.mode),
                (f"WP8 도킹 입구({nm})", (lat, lon), 7),             # 칸 입구 중심
                (f"WP9 도킹 이탈({nm})", n, 7)]                      # near 접근점으로 되나옴
        for label, (la, lo), md in rows:
            print(f'  - {{구역: "{label}", lat: {la:.7f}, lon: {lo:.7f}, '
                  f'mode: {md}, dwell: {a.dwell}}}')


if __name__ == "__main__":
    main()

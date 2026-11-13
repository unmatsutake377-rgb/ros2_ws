"""LiDAR 스캔 → 부표 후보(노랑 + 키 큰 흰 장애물) (ROS 비의존).

eunwootest lidar_object_detector 의 클러스터링(거리 점프·각도 간격·최소 점수)을 가져오되
  · 클러스터를 매번 새로 시작한다(원본은 리셋이 빠져 모든 점이 하나로 뭉쳤다).
  · 부표 크기 검사: 클러스터 각폭이 2·atan(r/거리) 와 맞는 것만(배·벽·물보라 배제).
    r = width_r(가장 큰 부표 반경 — 노드는 max(buoy_r, obstacle_r) 를 넘긴다, 흰 지름 ⚠️ 가정).
    classify: buoy_r 폭으론 안 되고 width_r 폭으로만 통과한 군집에 wide 표시(노랑 표·벽에 안 씀).
  · 거리는 표면 평균 + buoy_r = 중심 거리(노랑 가정 — 흰 장애물 중심은 core 가 obstacle_r 로 고친다).
"""
import math


def cluster_buoys(ranges, angle_min_deg, angle_inc_deg, lidar_forward_deg=80.0,
                  max_gap=0.25, max_angle_gap=3.0, min_points=3, min_r=0.5, max_r=10.0,
                  buoy_r=0.12, width_tol=2.5, width_r=None, classify=False):
    """반환: [(상대방위 좌양수 deg, 중심거리 m)]. width_r None = buoy_r(예전과 같음).
    classify True 면 [(방위, 거리, wide)] — wide = 노랑(buoy_r) 폭 검사로는 버려지고 흰 폭(width_r)으로만 통과한
    넓은 군집(흰 장애물이거나 붙어 보이는 부표 둘 — 단일 노랑일 수 없다, 계약서 v3 W2b ⑤ 흰 폭 허용 분리)."""
    wr = buoy_r if width_r is None else max(buoy_r, width_r)
    out = []
    cur = []

    def flush():
        if len(cur) < min_points:
            return
        i0, i1 = cur[0][0], cur[-1][0]
        d = sum(r for _, r in cur) / len(cur)
        width = (i1 - i0) * angle_inc_deg
        expect = math.degrees(2.0 * math.atan(wr / max(d, 0.1)))
        if width > expect * width_tol + angle_inc_deg:
            return
        a = angle_min_deg + 0.5 * (i0 + i1) * angle_inc_deg
        if classify:
            narrow = math.degrees(2.0 * math.atan(buoy_r / max(d, 0.1))) * width_tol + angle_inc_deg
            out.append((a - lidar_forward_deg, d + buoy_r, width > narrow))
        else:
            out.append((a - lidar_forward_deg, d + buoy_r))

    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and min_r < r < max_r):
            continue
        if cur:
            pi, pr = cur[-1]
            if abs(r - pr) < max_gap and (i - pi) * angle_inc_deg < max_angle_gap:
                cur.append((i, r))
                continue
            flush()
        cur = [(i, r)]
    if cur:
        flush()
    return out

"""LiDAR 스캔 → 노란부표 후보 (ROS 비의존).

eunwootest lidar_object_detector 의 클러스터링(거리 점프·각도 간격·최소 점수)을 가져오되
  · 클러스터를 매번 새로 시작한다(원본은 리셋이 빠져 모든 점이 하나로 뭉쳤다).
  · 부표 크기 검사: 클러스터 각폭이 2·atan(r/거리) 와 맞는 것만(배·벽·물보라 배제).
  · 거리는 표면 평균 + 반경 = 중심 거리.
"""
import math


def cluster_buoys(ranges, angle_min_deg, angle_inc_deg, lidar_forward_deg=80.0,
                  max_gap=0.25, max_angle_gap=3.0, min_points=3, min_r=0.5, max_r=10.0,
                  buoy_r=0.12, width_tol=2.5):
    """반환: [(상대방위 좌양수 deg, 중심거리 m)]."""
    out = []
    cur = []

    def flush():
        if len(cur) < min_points:
            return
        i0, i1 = cur[0][0], cur[-1][0]
        d = sum(r for _, r in cur) / len(cur)
        width = (i1 - i0) * angle_inc_deg
        expect = math.degrees(2.0 * math.atan(buoy_r / max(d, 0.1)))
        if width > expect * width_tol + angle_inc_deg:
            return
        a = angle_min_deg + 0.5 * (i0 + i1) * angle_inc_deg
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

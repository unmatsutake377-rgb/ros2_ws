#!/usr/bin/env python3
"""gps_guard.fix_is_usable 단위테스트 (ROS 불필요).

실행: python3 src/north_goal_angle/test/test_gps_guard.py

🚨 이 테스트가 고정하는 사실 (2026-08-06 실기 발견):
   ublox 드라이버는 fix 가 없어도 /fix 를 1Hz 로 계속 낸다 — status=-1, lat=lon=0.0.
   그걸 믿으면 (0,0) 기준 방위(실측 47.96°)를 발행해 배가 엉뚱한 방향으로 간다.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from north_goal_angle.gps_guard import (  # noqa: E402
    fix_is_usable, STATUS_NO_FIX, STATUS_FIX, STATUS_SBAS_FIX, STATUS_GBAS_FIX)

# 대회장 근처 실제 웨이포인트 좌표 (config/waypoints.yaml WP0)
GOOD_LAT, GOOD_LON = 35.1862375, 128.5655118

_fails = []


def check(name, cond):
    if cond:
        print(f"  ✅ {name}")
    else:
        print(f"  ❌ {name}")
        _fails.append(name)


print("=== gps_guard 테스트 ===")

# ---- 거부해야 하는 것 ----
check("no_fix_rejected",
      not fix_is_usable(STATUS_NO_FIX, GOOD_LAT, GOOD_LON))
check("null_island_rejected",          # 드라이버 기본값 그대로
      not fix_is_usable(STATUS_NO_FIX, 0.0, 0.0))
check("null_island_rejected_even_if_status_ok",   # status 를 못 믿는 구현 대비
      not fix_is_usable(STATUS_FIX, 0.0, 0.0))
check("nan_lat_rejected",
      not fix_is_usable(STATUS_FIX, float('nan'), GOOD_LON))
check("nan_lon_rejected",
      not fix_is_usable(STATUS_FIX, GOOD_LAT, float('nan')))
check("inf_rejected",
      not fix_is_usable(STATUS_FIX, float('inf'), GOOD_LON))
check("lat_out_of_range_rejected",
      not fix_is_usable(STATUS_FIX, 91.0, GOOD_LON))
check("lon_out_of_range_rejected",
      not fix_is_usable(STATUS_FIX, GOOD_LAT, 181.0))
check("none_rejected",
      not fix_is_usable(None, GOOD_LAT, GOOD_LON))
check("garbage_rejected",
      not fix_is_usable("x", GOOD_LAT, GOOD_LON))

# ---- 받아야 하는 것 ----
check("plain_fix_accepted",
      fix_is_usable(STATUS_FIX, GOOD_LAT, GOOD_LON))
check("sbas_fix_accepted",
      fix_is_usable(STATUS_SBAS_FIX, GOOD_LAT, GOOD_LON))
check("gbas_fix_accepted",            # RTK 는 여기로 온다
      fix_is_usable(STATUS_GBAS_FIX, GOOD_LAT, GOOD_LON))
check("southern_hemisphere_accepted",  # 좌표 부호 때문에 막지 않는지
      fix_is_usable(STATUS_FIX, -33.86, 151.20))
check("boundary_lat_90_accepted",
      fix_is_usable(STATUS_FIX, 90.0, 0.5))
check("boundary_lon_180_accepted",
      fix_is_usable(STATUS_FIX, 0.5, 180.0))
check("near_zero_but_real_accepted",   # 적도 근처 실제 좌표는 막으면 안 된다
      fix_is_usable(STATUS_FIX, 0.001, 0.001))

# ---- 회귀 고정: 실기에서 관측한 그 상황 ----
check("regression_observed_case",      # status=-1, lat=lon=0 → 반드시 거부
      not fix_is_usable(-1, 0.0, 0.0))

print()
n = 18
print(f"{n - len(_fails)}/{n} 통과")
if _fails:
    print("실패:", ", ".join(_fails))
    sys.exit(1)

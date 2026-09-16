#!/usr/bin/env python3
"""mast_parts.py — mast.py 와 fit_test.py 가 공유하는 형상 함수.
치수 중복 정의를 막으려고 분리했다 (v4, 2026-09-16)."""
import cadquery as cq


def cradle(cam_w, cam_d, cam_h, plate_h, plate_t, lip,
           t=4.0, clear=1.0, notch_xs=(), notch_w=22.0):
    """카메라를 끼우는 U자 크레들 (뒷판 + 바닥턱 + 좌우턱, 위 열림).
    로컬 원점 = 좌우 중심 / 뒷판 뒷면 y=0 / **카메라 바닥 z=0**.
    카메라는 y plate_t~plate_t+cam_d, z 0~cam_h 에 앉는다. 무게는 바닥턱(z −t~0)이 받는다.
      cam_*  : 카메라 실치수
      clear  : 카메라 ↔ 안쪽 벽 총 여유 (편측 clear/2)
      lip    : 좌우턱 높이
      notch_xs / notch_w : 바닥턱 커넥터 노치 (아래로 나오는 케이블)
    """
    iw = cam_w + clear
    idp = cam_d + clear
    ow = iw + 2 * t
    fy = plate_t + idp + t                       # 바닥·좌우턱 앞끝
    body = cq.Workplane("XY").box(ow, plate_t, plate_h).translate((0, plate_t / 2, cam_h / 2))
    floor = cq.Workplane("XY").box(ow, fy - plate_t, t).translate((0, (plate_t + fy) / 2, -t / 2))
    for nx in notch_xs:
        floor = floor.cut(cq.Workplane("XY").box(notch_w, fy - plate_t + 2, t + 2)
                          .translate((nx, (plate_t + fy) / 2, -t / 2)))
    body = body.union(floor)
    for sx in (-1, 1):
        body = body.union(cq.Workplane("XY").box(t, fy - plate_t, lip)
                          .translate((sx * (iw + t) / 2, (plate_t + fy) / 2, lip / 2)))
    return body

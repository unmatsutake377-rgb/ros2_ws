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
    # [2026-09-16 검토 반영] 뒷판 밑을 바닥턱 밑면(z=−t)까지 내린다.
    #   전에는 판이 z (cam_h−plate_h)/2 에서 끝나 바닥턱보다 3.5~4.0mm 떠 있었다 →
    #   어느 방향으로 눕혀도 서포트가 필요했다. 이제 STL 방향 그대로 평평히 앉는다.
    p_top = cam_h / 2 + plate_h / 2
    p_bot = -t
    body = cq.Workplane("XY").box(ow, plate_t, p_top - p_bot).translate((0, plate_t / 2, (p_top + p_bot) / 2))
    floor = cq.Workplane("XY").box(ow, fy - plate_t, t).translate((0, (plate_t + fy) / 2, -t / 2))
    for nx in notch_xs:
        floor = floor.cut(cq.Workplane("XY").box(notch_w, fy - plate_t + 2, t + 2)
                          .translate((nx, (plate_t + fy) / 2, -t / 2)))
    body = body.union(floor)
    for sx in (-1, 1):
        body = body.union(cq.Workplane("XY").box(t, fy - plate_t, lip)
                          .translate((sx * (iw + t) / 2, (plate_t + fy) / 2, lip / 2)))
    return body


def collar_ring(tube_od, margin, clear, height, z0=0.0):
    """각관 **바깥**을 잡는 칼라 (베이스↔마디 이중 끼움의 바깥쪽).
    보어 = tube_od + 2*clear. 본체와 쿠폰이 같은 형상을 쓰게 공용화했다
    (쿠폰에 이게 빠져 있어서 더 빡빡한 쪽을 시험 못 하고 있었다 — 2026-09-16 검토)."""
    ch = tube_od / 2 + margin + clear
    return (cq.Workplane("XY").workplane(offset=z0).rect(2 * ch, 2 * ch).extrude(height)
            .faces(">Z").workplane().rect(tube_od + 2 * clear, tube_od + 2 * clear).cutThruAll())

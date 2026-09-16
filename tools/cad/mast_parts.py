#!/usr/bin/env python3
"""mast_parts.py — mast.py 와 fit_test.py 가 공유하는 형상 함수.
치수 중복 정의를 막으려고 분리했다 (v4, 2026-09-16)."""
import math

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


def cradle_module(cam_w, cam_d, cam_h, plate_h, plate_t, lip, tilt,
                  back_t=6.0, t=4.0, clear=1.0, notch_xs=(), notch_w=22.0,
                  holes=(), hole_d=3.4, hole_depth=8.0):
    """[v5] 관에서 떼어낸 **볼트-온 크레들 모듈**.

    등판(mounting face)은 **수직**(y=0 평면)이라 관의 리브에 그대로 밀착한다.
    카메라만 tilt 만큼 아래로 기울고, 그 사이 틈은 쐐기로 채운다.
    → 등판을 바닥에 대고 뽑으면 관도 크레들도 각각 평평하게 앉는다(서포트 거의 0).
    로컬 원점: 좌우 중심 / 등판 뒷면 y=0 / 등판 밑 z=0.
    """
    c = cradle(cam_w, cam_d, cam_h, plate_h, plate_t, lip, t, clear, notch_xs, notch_w)
    c = c.translate((0, 0, t))                      # 바닥턱 밑 → z=0
    c = c.rotate((0, 0, 0), (1, 0, 0), -tilt)       # 등판 밑 모서리 기준으로 앞으로 기울임
    H = t + cam_h + 6.0                             # 등판이 덮어야 할 높이
    bw = cam_w + clear + 2 * t
    yb = H * math.sin(math.radians(tilt))           # 기울여서 앞으로 나간 양
    # 등판 + 쐐기: y −back_t..0 의 판 + 0..yb 의 삼각 쐐기
    back = cq.Workplane("XY").box(bw, back_t, H).translate((0, -back_t / 2, H / 2))
    wedge = (cq.Workplane("YZ").workplane(offset=-bw / 2)
             .polyline([(0.0, 0.0), (yb + 0.3, H), (0.0, H)]).close().extrude(bw))
    body = back.union(wedge).union(c)
    for (hx, hz) in holes:                          # 셀프탭 M4 맹공 (등판 뒷면에서)
        body = body.cut(cq.Workplane("XZ").workplane(offset=back_t + 1)
                        .center(hx, hz).circle(hole_d / 2).extrude(-(hole_depth + back_t + 1)))
    return body


def mount_rib(tube_od, width, proud, z0, z1, holes=(), hole_d=3.4, hole_depth=8.0):
    """[v5] 관 +Y 면에 붙는 **전 높이 리브** (크레들이 볼트로 붙는 자리).
    세로 프리즘이라 관을 세워 뽑을 때 아래보기 면이 생기지 않는다."""
    rib = cq.Workplane("XY").box(width, proud, z1 - z0).translate((0, tube_od / 2 + proud / 2, (z0 + z1) / 2))
    for (hx, hz) in holes:
        rib = rib.cut(cq.Workplane("XZ").workplane(offset=-(tube_od / 2 + proud + 1))
                      .center(hx, hz).circle(hole_d / 2).extrude(hole_depth + 1))
    return rib

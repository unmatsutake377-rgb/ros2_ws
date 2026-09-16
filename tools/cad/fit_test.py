#!/usr/bin/env python3
"""fit_test.py — 실험 프린트용 '맞춤 확인 쿠폰' 생성기 (2026-09-16).

통짜 마스트를 뽑기 전에 **맞는지 모르는 것만** 작게 뽑아 확인한다.
치수는 mast.py 가 뱉은 out/placement.json 에서 읽는다 → 본체와 항상 같은 값.

쿠폰 4종:
  1 cradle_d455 : D455 크레들 단품 (실물 D455 있음 → 끼움·나사간격·USB 노치 확인)
  2 pocket_imu  : IMU 포켓 블록 + 뚜껑 (실물 iAHRS 있음 → 포켓·뚜껑나사·케이블터널 확인)
  3 joint_plug / joint_socket : 각관 이음 한 쌍 (0.4 여유가 이 프린터에서 맞는지)
  4 cradle_oak_gauge : OAK 크레들 단품 + OAK 목업 블록 (카메라 미발주 → 목업으로 대신 확인)

사용:
  python3 tools/cad/mast.py        # 먼저 (placement.json 생성)
  python3 tools/cad/fit_test.py    # → tools/cad/out_fit/*.stl
"""
import json
import os

import cadquery as cq
from mast_parts import collar_ring, cradle

HERE = os.path.dirname(os.path.abspath(__file__))
P = json.load(open(os.path.join(HERE, "out", "placement.json")))
F = P["fit"]
OUT = os.path.join(HERE, "out_fit")
os.makedirs(OUT, exist_ok=True)

T, CLR = F["CRADLE_T"], F["CRADLE_CLEAR"]
parts = {}

# ── 1. D455 크레들 단품 ─────────────────────────────────────────────
dw, dd, dh, dph, dpt, dhp, dhd, dux = F["D455"]
c1 = cradle(dw, dd, dh, dph, dpt, F["LIP_D455"], T, CLR, (-dux, dux), F["NOTCH_W"])
for hx in (-dhp / 2, dhp / 2):
    c1 = c1.cut(cq.Workplane("XZ").workplane(offset=1).center(hx, dh / 2)
                .circle(dhd / 2).extrude(-(dpt + 2)))
parts["cradle_d455"] = c1

# ── 2. IMU 포켓 블록 + 뚜껑 ─────────────────────────────────────────
px, py, pz, lx, ly, lt, lsd, chd, chx, pad = F["pocket"]
bt = F["base"][0]
blk_x, blk_y = lx + 16, ly + 16
blk = cq.Workplane("XY").box(blk_x, blk_y, bt).translate((0, 0, bt / 2)).edges("|Z").fillet(6)
floor_z = bt - lt - pz
blk = blk.cut(cq.Workplane("XY").workplane(offset=floor_z).rect(px, py).extrude(pz + lt + 1))
blk = blk.cut(cq.Workplane("XY").workplane(offset=bt - lt).rect(lx, ly).extrude(lt + 1))
lm = F["LID_MARGIN"]   # 본체와 같은 값 (전에는 4.0 하드코딩 — 지금은 같지만 어긋날 수 있었다)
scr = [(sx * (px / 2 + lm / 2), sy * (py / 2 + lm / 2)) for sx in (-1, 1) for sy in (-1, 1)]
for (sx, sy) in scr:
    blk = blk.cut(cq.Workplane("XY").workplane(offset=bt - lt - 9).center(sx, sy).circle(lsd / 2).extrude(10))
# 케이블 터널 (포켓 +Y 벽 → 블록 밖) — 본체에선 중앙 구멍으로 가지만 쿠폰은 관통만 확인
# 터널은 포켓 안 → 블록 **밖**까지 관통해야 케이블을 실제로 꿰어볼 수 있다.
#   [2026-09-16] 전엔 extrude(-(blk_y/2+2)) 라 포켓 벽을 1mm 만 파고 막혔다(맹공).
#   본체는 중앙 Ø22 구멍까지 뚫려 정상. 쿠폰만 틀렸던 것.
blk = blk.cut(cq.Workplane("XZ").workplane(offset=py / 2 - 1).center(chx, floor_z + pad + chd / 2 + 0.5)
              .circle(chd / 2).extrude(-(blk_y / 2 + 2 + py / 2 - 1)))
parts["pocket_imu"] = blk
lid = cq.Workplane("XY").rect(lx - F["LID_FIT"], ly - F["LID_FIT"]).extrude(lt).edges("|Z").fillet(2)
for (sx, sy) in scr:
    lid = lid.cut(cq.Workplane("XY").center(sx, sy).circle(3.4 / 2).extrude(lt))
parts["pocket_lid"] = lid

# ── 3. 각관 이음 한 쌍 ──────────────────────────────────────────────
tod, tw, plen, pclr, phole, dckt, bd = F["plug"]
inner = tod - 2 * tw
plug = inner - 2 * pclr
# 플러그 쪽 (베이스 윗부분 흉내): 판 + 바닥판 + 플러그
a = cq.Workplane("XY").box(tod + 20, tod + 20, 6).translate((0, 0, 3))
a = a.union(cq.Workplane("XY").workplane(offset=6).rect(tod, tod).extrude(dckt)
            .faces(">Z").workplane().rect(phole, phole).cutThruAll())
a = a.union(cq.Workplane("XY").workplane(offset=6 + dckt).rect(plug, plug).extrude(plen)
            .faces(">Z").workplane().rect(phole, phole).cutThruAll())
a = a.cut(cq.Workplane("XY").rect(phole, phole).extrude(6 + dckt + 1))
# [2026-09-16 검토 반영] 칼라 추가 — 베이스↔마디는 플러그(안) + 칼라(바깥) 이중 끼움이고,
#   프린트 오차는 구멍이 작아지고 바깥면이 커지는 방향이라 **칼라 쪽이 더 빡빡하다**.
#   쿠폰에 칼라가 없으면 안쪽만 합격하고 본체에서 마디가 안 들어간다.
ch_h, ch_m = F["collar"]
a = a.union(collar_ring(tod, ch_m, pclr, ch_h, 6.0))
a = a.cut(cq.Workplane("YZ").workplane(offset=-tod).center(0, 6 + dckt + plen / 2).circle(bd / 2).extrude(2 * tod))
parts["joint_plug"] = a
# 소켓 쪽 (마디 아래끝 흉내)
b = (cq.Workplane("XY").rect(tod, tod).extrude(plen + 12)
     .faces(">Z").workplane().rect(inner, inner).cutThruAll())
b = b.cut(cq.Workplane("YZ").workplane(offset=-tod).center(0, plen / 2).circle(bd / 2).extrude(2 * tod))
parts["joint_socket"] = b

# ── 4. OAK 크레들 + 목업 (카메라 미발주) ────────────────────────────
ow, od, oh, oph, opt, ohx, ohz, ohd, ocx = F["OAK"]
c2 = cradle(ow, od, oh, oph, opt, F["LIP_OAK"], T, CLR, (ocx,), F["NOTCH_OAK"])
for hx in (-ohx / 2, ohx / 2):
    for hz in (-ohz / 2, ohz / 2):
        c2 = c2.cut(cq.Workplane("XZ").workplane(offset=1).center(hx, oh / 2 + hz)
                    .circle(ohd / 2).extrude(-(opt + 2)))
parts["cradle_oak"] = c2
# 목업: OAK 외형 + 뒷면 M4 자리 + 그랜드. 속을 비워 프린트 시간·재료 절약
mock = cq.Workplane("XY").box(ow, od, oh).translate((0, od / 2, oh / 2))
mock = mock.cut(cq.Workplane("XY").box(ow - 8, od - 8, oh - 8).translate((0, od / 2, oh / 2)))
mock = mock.union(cq.Workplane("XY").box(27, 16, 35).translate((ocx, od / 2, -35 / 2)))
parts["oak_mockup"] = mock

for name, p in parts.items():
    cq.exporters.export(p, os.path.join(OUT, f"fit_{name}.stl"), tolerance=0.05)
    bb = p.val().BoundingBox()
    print(f"  fit_{name:14s} {bb.xlen:6.1f} × {bb.ylen:6.1f} × {bb.zlen:6.1f} mm")
print("출력 폴더:", OUT)

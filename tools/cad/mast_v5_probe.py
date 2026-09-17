#!/usr/bin/env python3
"""mast_v5_probe.py — v5(크레들 분리형) 시제 생성기. **2026-09-16 측정 후 기각됨.**

기각 근거 (의도한 출력 방향 = 등판을 바닥에, X+90°):
    v5_cradle_d455  서포트 4455mm²   (v4 mast_seg1 4177mm² 보다 나쁨)
    v5_cradle_oak   서포트 7831mm²   (v4 mast_cap  3851mm² 의 2배)
  앞서 방향 전수탐색이 내놓은 2860mm² 는 X330°/X240° 같이 부품을 모서리로 세운 비현실적 각도였다.
  게다가 부피의 58~67% 가 등판+쐐기(순수 장착용 군살)이고, 마스트 상부에 +69g 이 실린다.
  → 크레들을 떼어내도 '기울어진 카메라를 수직면에 붙이는' 쐐기가 그대로 오버행을 만든다. 구조가 답이 아니다.

남겨두는 이유: 같은 아이디어를 다시 시도하지 않게 하려고. 재현하려면 이 파일을 그대로 돌리면 된다.

(원본 설명) v5(크레들 분리형) 출력성 측정용 시제 생성기.
기존 mast.py 는 건드리지 않는다. 서포트 면적이 실제로 줄어드는지 **측정해 보고** 채택 여부를 정한다."""
import json, math, os
import cadquery as cq
from mast_parts import cradle_module, mount_rib

HERE = os.path.dirname(os.path.abspath(__file__))
P = json.load(open(os.path.join(HERE, "out", "placement.json"))); F = P["fit"]
OUT = os.path.join(HERE, "out_v5"); os.makedirs(OUT, exist_ok=True)
TOD, TW, PLEN, PCLR, PHOLE, DCKT, BD = F["plug"]
T, CLR = F["CRADLE_T"], F["CRADLE_CLEAR"]
dw, dd, dh, dph, dpt, dhp, dhd, dux = F["D455"]
ow, od, oh, oph, opt, ohx, ohz, ohd, ocx = F["OAK"]
RIB_W, RIB_P = 34.0, 10.0
HOLES = [(x, z) for x in (-12, 12) for z in (14, 44)]   # 24 × 30 패턴
inner = TOD - 2 * TW; plug = inner - 2 * PCLR

def tube(L):
    return (cq.Workplane("XY").rect(TOD, TOD).extrude(L)
            .faces(">Z").workplane().rect(inner, inner).cutThruAll())

parts = {}
# 관(마디) + 리브
seg = tube(106.1)
seg = seg.union(cq.Workplane("XY").workplane(offset=106.1).rect(TOD, TOD).extrude(DCKT)
                .faces(">Z").workplane().rect(PHOLE, PHOLE).cutThruAll())
seg = seg.union(cq.Workplane("XY").workplane(offset=106.1 + DCKT).rect(plug, plug).extrude(PLEN)
                .faces(">Z").workplane().rect(PHOLE, PHOLE).cutThruAll())
parts["v5_seg1"] = seg.union(mount_rib(TOD, RIB_W, RIB_P, 2.0, 104.0, HOLES))
# 캡 + 리브
cap = tube(PLEN + 6)
cap = cap.union(cq.Workplane("XY").workplane(offset=PLEN).rect(TOD, TOD).extrude(6))
parts["v5_cap"] = cap.union(mount_rib(TOD, RIB_W, RIB_P, 2.0, PLEN + 4.0, HOLES))
# 크레들 모듈 2종
parts["v5_cradle_d455"] = cradle_module(dw, dd, dh, dph, dpt, F["LIP_D455"], 15.0,
                                        t=T, clear=CLR, notch_xs=(-dux, dux),
                                        notch_w=F["NOTCH_W"], holes=HOLES)
parts["v5_cradle_oak"] = cradle_module(ow, od, oh, oph, opt, F["LIP_OAK"], 11.0,
                                       t=T, clear=CLR, notch_xs=(ocx,),
                                       notch_w=F["NOTCH_OAK"], holes=HOLES)
for n, p in parts.items():
    cq.exporters.export(p, os.path.join(OUT, f"{n}.stl"), tolerance=0.05)
    bb = p.val().BoundingBox()
    print(f"  {n:18s} {bb.xlen:6.1f}×{bb.ylen:6.1f}×{bb.zlen:6.1f}mm  {p.val().Volume()/1000:6.1f}㎤")
print("출력:", OUT)

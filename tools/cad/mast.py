#!/usr/bin/env python3
"""mast.py v2 — 카메라 마스트 파라메트릭 CAD (CadQuery). STEP(Fusion 360) + STL(프린트) 출력.

v2 (2026-09-16, `docs/전달용/마스트_요구사항.md` A10 단일화 결론 반영):
  · **OAK 위(꼭대기, 아래 11°) / D455 아래(선반, 아래 15°)** — 자율 우선. 물 띠 = atan(높이/뱃머리 거리).
  · **IMU 는 베이스 블록 포켓** (박스 안은 노트북·MD-30A 와 동거 → 파손 위험). 베이스 두께 8→20.
    포켓 + 나일론 M3 뚜껑(`imu_lid`). 케이블은 포켓 벽 → 베이스 속 터널 → 중앙 Ø22 구멍.
  · 갑판(FRP) 위 최고점(OAK 상단) ≤ 220. 캡 상단 = 20 + 4 + MAST_H + 4 + 36.
  · OAK RJ45 그랜드는 **아래**로 (위로 두면 220 초과). D455 는 선반 앞쪽(관 면에서 40mm 뒤로 물림)에 놓아 그랜드·케이블과 안 겹침.
  · 재질 **PETG/ASA**(PLA ✗ — 여름 갑판 열로 각도 처짐), 밝은 색, 인필 ≥40%, 벽 ≥4.

부품 4개: base(IMU 포켓·칼라) / seg1(각관 + D455 선반) / cap(OAK 판) / imu_lid.
좌표: +Y = 뱃머리, Z = 갑판 윗면 기준 0. 카메라는 모두 +Y 를 본다.

⚠️ 실측 후 고칠 파라미터:
  · IMU_L / IMU_W / IMU_H — iAHRS 케이스 포함 실측 (지금은 ⚠️ 가정 50×50×20). 포켓 = 실측 +1.
    케이블 나오는 면이 포켓의 **관 쪽(−X)** 을 보게 넣는다. 다른 면이면 IMU_CH_DIR 만 바꾼다.
  · BASE_HOLE_PITCH — (130, 가로대 간격). 갑판 두께 t 면 볼트 M5 길이 = t + BASE_T(20) + 와셔·너트 ≈10 → t+30.
  · OAK_TILT / D455_TILT — 물 위 첫 시험 후.
  · BASE_W 는 포켓·뚜껑이 들어가게 자동으로 넓어진다(최소 160). 출력 로그 확인.

사용:
  pip install cadquery
  python3 tools/cad/mast.py            → tools/cad/out/mast_{base,seg1,cap,imu_lid}.stl/.step, mast_assembly.step
"""
import math
import os

import cadquery as cq

# ───────────────────────────── 파라미터 (mm, deg) ─────────────────────────────
MAST_H         = None    # 각관 마디 길이. None = 자동(OAK 판 하단이 캡 하단에 오도록: OAK_Z − 25 − BASE_T − 8 ≈ 122). 숫자를 주면 그 값
SEG_MAX        = 250.0
TUBE_OD        = 40.0
TUBE_WALL      = 5.0
PLUG_LEN       = 30.0
PLUG_CLEAR     = 0.4
BOLT_D         = 4.4     # M4 관통
DECK_T         = 4.0     # 플러그 밑 바닥판
LIMIT_H        = 220.0   # 팀 요구 상한 (갑판 위 최고점)

BASE_W_MIN, BASE_L, BASE_T_MIN = 160.0, 120.0, 20.0   # v2: 두께 8→20 (IMU 포켓). IMU 가 두꺼우면 자동으로 더 두꺼워진다(포켓 바닥 ≥4)
BASE_HOLE_PITCH = (130.0, 90.0)   # x=현 방향, y=90 = 가로대(평철) 2개 중심 간격 → 실측 후 수정
BASE_HOLE_D     = 5.5             # M5
CABLE_HOLE_D    = 22.0            # RJ45 헤드 통과
COLLAR_H        = 20.0
COLLAR_MARGIN   = 2.0             # 칼라 편측 두께 (v1 5 → 2: 포켓 자리)
GUSSET          = 30.0            # ±Y 거싯만 (±X 는 포켓)

# IMU (iAHRS RB-SDA-v1) — ⚠️ 가정. 케이스 포함 실측값으로 교체할 것
IMU_L, IMU_W, IMU_H = 50.0, 50.0, 20.0   # L=선체 좌우(X), W=선체 전후(Y), H=높이
IMU_CLEAR   = 1.0      # 편측 아님, 전체 +1
IMU_CH_D    = 8.0      # 케이블 터널 지름 (JST/듀폰 하네스 지나감)
IMU_CH_DIR  = "-X"     # 터널이 나가는 포켓 면: "-X"(관 쪽, 기본) / "+Y" / "-Y"
LID_T       = 3.0
LID_MARGIN  = 8.0      # 뚜껑이 포켓보다 편측 8 크다 (나사 자리)
LID_SCREW_D = 2.5      # 나일론 M3 탭 구멍 (베이스), 뚜껑은 3.4 관통
IMU_POCKET_C = None    # (x, y) 포켓 중심. None 이면 칼라 옆에 자동 배치

# D455: 124 × 26 × 29 (W×D×H). 밑면 1/4-20. 벨크로 고정
D455_W, D455_D, D455_H = 124.0, 26.0, 29.0
D455_Z      = 85.0    # 갑판 → 선반 윗면(관 면에서). [09-16] 95→85: OAK 그랜드 끝(≈97)과 2mm → 12mm 여유
D455_TILT   = 15.0    # 아래로
D455_SETBACK = 40.0   # 관 면 ↔ D455 뒤 턱. OAK 그랜드(아래 ≈35mm)·케이블이 이 틈으로 내려온다
SHELF_T     = 6.0
SHELF_RIM   = 3.0
SHELF_MARGIN = 3.0
SHELF_GUSSET_H = 25.0  # 선반 밑 거싯 높이. [09-16] 35→25: D455_Z=85 에서 베이스 칼라(상단 48)와 간섭 회피

# OAK-1 PoE (구매 확정 09-15, 고정초점 69°): 81.9 × 81.9 × 31, 뒷면 M4×4 가로 45 / 세로 37.5, 바닥 1/4-20
OAK_W, OAK_H, OAK_D = 81.9, 81.9, 31.0
OAK_HOLE_PITCH = (45.0, 37.5)
OAK_HOLE_D     = 4.4
OAK_TILT       = 11.0    # 아래로 (마스트_요구사항 §2-6)
OAK_Z          = 175.0   # 갑판 → OAK 중심. 상단 = OAK_Z + 41·cos + 15.5·sin ≈ 218
OAK_PLATE_T    = 6.0
OAK_PLATE_H    = 50.0
OAK_STANDOFF   = 1.0     # 판 뒷면 ↔ seg1 앞면 여유 (캡 아래로 내려온 판이 마디에 안 닿게)
OAK_CONN_LEN, OAK_CONN_W = 35.0, 27.0   # RJ45 그랜드 돌출 (아래)

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)

inner = TUBE_OD - 2 * TUBE_WALL
plug = inner - 2 * PLUG_CLEAR
R = math.radians


def tube(length):
    return (cq.Workplane("XY").rect(TUBE_OD, TUBE_OD).extrude(length)
            .faces(">Z").workplane().rect(inner, inner).cutThruAll())


def plug_top(body, z_top):
    """토막 위에 바닥판 + 플러그 + 관통 볼트 구멍. 케이블 구멍은 뚫려 있다."""
    hole = plug - 2 * TUBE_WALL
    deck = (cq.Workplane("XY").workplane(offset=z_top).rect(TUBE_OD, TUBE_OD).extrude(DECK_T)
            .faces(">Z").workplane().rect(hole, hole).cutThruAll())
    p = (cq.Workplane("XY").workplane(offset=z_top + DECK_T).rect(plug, plug).extrude(PLUG_LEN)
         .faces(">Z").workplane().rect(hole, hole).cutThruAll())
    body = body.union(deck).union(p)
    return body.cut(cq.Workplane("YZ").workplane(offset=-TUBE_OD).center(0, z_top + DECK_T + PLUG_LEN / 2)
                    .circle(BOLT_D / 2).extrude(2 * TUBE_OD))


def socket_bolts(body, z_bottom):
    return body.cut(cq.Workplane("YZ").workplane(offset=-TUBE_OD).center(0, z_bottom + PLUG_LEN / 2)
                    .circle(BOLT_D / 2).extrude(2 * TUBE_OD))


# ───────────────────────────── 1. 베이스 (IMU 포켓) ─────────────────────────────
pocket_x, pocket_y, pocket_z = IMU_L + IMU_CLEAR, IMU_W + IMU_CLEAR, IMU_H + IMU_CLEAR
lid_x, lid_y = pocket_x + 2 * LID_MARGIN, pocket_y + 2 * LID_MARGIN
collar_half = TUBE_OD / 2 + COLLAR_MARGIN + PLUG_CLEAR
if IMU_POCKET_C is None:
    IMU_POCKET_C = (collar_half + 1.5 + lid_x / 2, 0.0)   # 칼라 옆 +X (우현)
pcx, pcy = IMU_POCKET_C
BASE_W = max(BASE_W_MIN, 2 * (pcx + lid_x / 2 + 4.0))
BASE_T = max(BASE_T_MIN, math.ceil(pocket_z + LID_T + 4.0))   # 포켓(IMU+1) + 뚜껑 리베이트 + 바닥 ≥4mm
if BASE_T > BASE_T_MIN:
    print(f"ℹ️ IMU_H {IMU_H:.0f} → 베이스 두께 {BASE_T_MIN:.0f}→{BASE_T:.0f} (포켓 바닥 4mm 확보). 볼트 길이 = 갑판 t + {BASE_T:.0f} + 10")
if MAST_H is None:
    MAST_H = OAK_Z - OAK_PLATE_H / 2 - BASE_T - 2 * DECK_T   # OAK 판 하단 = 캡 하단
    print(f"ℹ️ MAST_H 자동 = {MAST_H:.0f}")

base = cq.Workplane("XY").rect(BASE_W, BASE_L).extrude(BASE_T).edges("|Z").fillet(8)
base = base.faces(">Z").workplane().rect(*BASE_HOLE_PITCH, forConstruction=True).vertices().hole(BASE_HOLE_D)
base = plug_top(base, BASE_T)
base = base.cut(cq.Workplane("XY").circle(CABLE_HOLE_D / 2).extrude(BASE_T + DECK_T + PLUG_LEN + 1))
for ang in (0, 180):   # ±Y 거싯
    g = (cq.Workplane("XZ").polyline([(collar_half - 0.1, BASE_T), (collar_half + GUSSET, BASE_T),
                                      (collar_half - 0.1, BASE_T + COLLAR_H)]).close().extrude(4, both=True)
         .rotate((0, 0, 0), (0, 0, 1), ang + 90))
    base = base.union(g)
collar = (cq.Workplane("XY").workplane(offset=BASE_T).rect(2 * collar_half, 2 * collar_half).extrude(COLLAR_H)
          .faces(">Z").workplane().rect(TUBE_OD + 2 * PLUG_CLEAR, TUBE_OD + 2 * PLUG_CLEAR).cutThruAll())
base = base.union(collar)

# 포켓 + 뚜껑 자리(리베이트) + 나사 탭 구멍
pocket_floor = BASE_T - LID_T - pocket_z   # 포켓 바닥 z (뚜껑 리베이트 아래로 pocket_z)
base = base.cut(cq.Workplane("XY").workplane(offset=pocket_floor).center(pcx, pcy).rect(pocket_x, pocket_y).extrude(pocket_z + LID_T + 1))
base = base.cut(cq.Workplane("XY").workplane(offset=BASE_T - LID_T).center(pcx, pcy).rect(lid_x, lid_y).extrude(LID_T + 1))
screw_pts = [(pcx + sx * (pocket_x / 2 + LID_MARGIN / 2), pcy + sy * (pocket_y / 2 + LID_MARGIN / 2))
             for sx in (-1, 1) for sy in (-1, 1)]
for (sx, sy) in screw_pts:
    base = base.cut(cq.Workplane("XY").workplane(offset=BASE_T - LID_T - 9).center(sx, sy).circle(LID_SCREW_D / 2).extrude(10))
# 케이블 터널: 포켓 벽 → 중앙 케이블 구멍 (포켓 바닥 바로 위)
ch_z = pocket_floor + IMU_CH_D / 2 + 0.5
if IMU_CH_DIR == "-X":
    ch = cq.Workplane("YZ").workplane(offset=-1).center(pcy, ch_z).circle(IMU_CH_D / 2).extrude(pcx)
elif IMU_CH_DIR in ("+Y", "-Y"):
    s = 1 if IMU_CH_DIR == "+Y" else -1
    ch = (cq.Workplane("XZ").workplane(offset=-(pcy + s * pocket_y / 2)).center(pcx, ch_z).circle(IMU_CH_D / 2).extrude(-s * 30)
          .union(cq.Workplane("YZ").workplane(offset=-1).center(pcy + s * (pocket_y / 2 + 25), ch_z).circle(IMU_CH_D / 2).extrude(pcx)))
else:
    raise SystemExit("IMU_CH_DIR 은 -X / +Y / -Y")
base = base.cut(ch)
# 포켓 바닥 화살표 (+Y = 뱃머리) 0.6 각인
arrow = (cq.Workplane("XY").workplane(offset=pocket_floor - 0.6).center(pcx, pcy)
         .polyline([(0, 12), (7, 4), (2.5, 4), (2.5, -12), (-2.5, -12), (-2.5, 4), (-7, 4)]).close().extrude(1.0))
base = base.cut(arrow)

# 뚜껑 (나일론 M3 ×4 관통, 손잡이 홈)
lid = cq.Workplane("XY").rect(lid_x - 0.4, lid_y - 0.4).extrude(LID_T).edges("|Z").fillet(2)
for (sx, sy) in screw_pts:
    lid = lid.cut(cq.Workplane("XY").center(sx - pcx, sy - pcy).circle(3.4 / 2).extrude(LID_T))
lid = lid.cut(cq.Workplane("XY").workplane(offset=LID_T - 0.8).center(0, 0).rect(12, 3).extrude(1))

# ───────────────────────────── 2. 마디 (D455 선반) ─────────────────────────────
n_seg = math.ceil(MAST_H / SEG_MAX)
seg_len = MAST_H / n_seg
seg_z0 = BASE_T + DECK_T     # 갑판 기준 seg1 시작
segments = []
shelf_w = D455_W + 2 * SHELF_MARGIN + 2 * SHELF_RIM
shelf_d = D455_SETBACK + D455_D + 2 * SHELF_MARGIN + 2 * SHELF_RIM
for i in range(n_seg):
    s = tube(seg_len)
    s = socket_bolts(s, 0)
    s = plug_top(s, seg_len)
    z0 = seg_z0 + i * (seg_len + DECK_T)
    if z0 <= D455_Z < z0 + seg_len:
        zl = D455_Z - z0
        # 선반 로컬: 뒤 가장자리 = (y=0, z=0), 앞으로 +Y. 윗면 z=0, 판은 아래로 SHELF_T
        sh = (cq.Workplane("XY").workplane(offset=-SHELF_T).center(0, shelf_d / 2).rect(shelf_w, shelf_d).extrude(SHELF_T + SHELF_RIM)
              .edges("|Z").fillet(4))
        # D455 자리(턱 안쪽) — 앞쪽에
        pad_y0 = D455_SETBACK + SHELF_RIM
        sh = sh.cut(cq.Workplane("XY").center(0, pad_y0 + (D455_D + 2 * SHELF_MARGIN) / 2)
                    .rect(shelf_w - 2 * SHELF_RIM, D455_D + 2 * SHELF_MARGIN).extrude(SHELF_RIM + 1))
        # 뒤쪽(관 면 ~ 턱) 은 케이블 통로: 턱 없이 평면 + 케이블 노치
        sh = sh.cut(cq.Workplane("XY").center(0, D455_SETBACK / 2).rect(shelf_w - 2 * SHELF_RIM, D455_SETBACK).extrude(SHELF_RIM + 1))
        sh = sh.cut(cq.Workplane("XY").workplane(offset=-SHELF_T - 1).center(0, 6 + 12).rect(36, 24).extrude(SHELF_T + SHELF_RIM + 2))
        sh = sh.faces("<Z").workplane().center(0, pad_y0 + SHELF_MARGIN + D455_D / 2).hole(6.6)   # 1/4-20 (선택)
        # 거싯 2개 (관 폭 안쪽 X=±14) 선반 밑
        for gx in (-14, 14):
            g = (cq.Workplane("YZ").workplane(offset=gx - 2)
                 .polyline([(0, -SHELF_T), (D455_SETBACK + 10, -SHELF_T), (0, -SHELF_T - SHELF_GUSSET_H)]).close().extrude(4))
            sh = sh.union(g)
        # 뒤 가장자리 기준 아래로 D455_TILT 회전 → 관 앞면으로 이동
        sh = sh.rotate((0, 0, 0), (1, 0, 0), -D455_TILT).translate((0, TUBE_OD / 2, zl))
        # 기울이면 거싯 아래 끝이 관 안으로 들어온다 → 관 앞면(y=TUBE_OD/2) 뒤쪽은 잘라낸다
        sh = sh.cut(cq.Workplane("XY").box(shelf_w + 20, 100, 300).translate((0, TUBE_OD / 2 - 50, zl)))
        # 관 면과 기운 선반·거싯 사이 쐐기 (뒤가 벌어지는 틈)
        h_w = SHELF_T + SHELF_GUSSET_H
        wedge = (cq.Workplane("YZ").workplane(offset=-TUBE_OD / 2)
                 .polyline([(TUBE_OD / 2, zl + 1), (TUBE_OD / 2, zl - h_w),
                            (TUBE_OD / 2 + h_w * math.sin(R(D455_TILT)) + 0.5, zl - h_w * math.cos(R(D455_TILT)))]).close()
                 .extrude(TUBE_OD))
        # 케이블 슬롯 (관 앞면, 선반 아래 → 관 내부)
        slot = cq.Workplane("XZ").workplane(offset=-(TUBE_OD / 2) - 20).center(0, zl - 50).rect(18, 14).extrude(TUBE_WALL + 25)
        s = s.union(wedge).union(sh).cut(slot)
    segments.append(s)
seg_top = seg_z0 + n_seg * seg_len + (n_seg - 1) * DECK_T   # 마지막 마디 관 윗면 (갑판 기준)

# ───────────────────────────── 3. 캡 (OAK 판) ─────────────────────────────
cap_z0 = seg_top + DECK_T
cap_h = PLUG_LEN + 6
cap = tube(cap_h)
cap = socket_bolts(cap, 0)
cap = cap.union(cq.Workplane("XY").workplane(offset=cap_h - 6).rect(TUBE_OD, TUBE_OD).extrude(6))
# OAK 판: 로컬 (y=0 판 뒷면, z=0 판 아래 가장자리). 아래 가장자리 기준 위쪽이 앞으로 기움(아래로 OAK_TILT)
pz0 = OAK_Z - OAK_PLATE_H / 2 - cap_z0          # 캡 로컬 판 하단
plate = (cq.Workplane("XY").center(0, OAK_PLATE_T / 2).rect(OAK_W + 10, OAK_PLATE_T).extrude(OAK_PLATE_H)
         .edges("|Y").fillet(4))
for hx in (-OAK_HOLE_PITCH[0] / 2, OAK_HOLE_PITCH[0] / 2):
    for hz in (-OAK_HOLE_PITCH[1] / 2, OAK_HOLE_PITCH[1] / 2):
        plate = plate.cut(cq.Workplane("XZ").workplane(offset=-OAK_PLATE_T - 1).center(hx, OAK_PLATE_H / 2 + hz)
                          .circle(OAK_HOLE_D / 2).extrude(OAK_PLATE_T + 2))
plate = plate.rotate((0, 0, 0), (1, 0, 0), -OAK_TILT).translate((0, TUBE_OD / 2 + OAK_STANDOFF, pz0))
# 캡 앞면 ↔ 기운 판 사이 쐐기 + 스탠드오프 (캡 높이 범위 안에서만)
wz1 = min(cap_h, pz0 + OAK_PLATE_H)
wedge = (cq.Workplane("YZ").workplane(offset=-TUBE_OD / 2)
         .polyline([(TUBE_OD / 2 - 0.1, max(0, pz0)), (TUBE_OD / 2 + OAK_STANDOFF + (max(0, pz0) - pz0) * math.tan(R(OAK_TILT)) + 0.3, max(0, pz0)),
                    (TUBE_OD / 2 + OAK_STANDOFF + (wz1 - pz0) * math.tan(R(OAK_TILT)) + 0.3, wz1), (TUBE_OD / 2 - 0.1, wz1)]).close()
         .extrude(TUBE_OD))
cap = cap.union(wedge).union(plate)

# ───────────────────────────── 4. 검산 ─────────────────────────────
oak_top = OAK_Z + (OAK_H / 2) * math.cos(R(OAK_TILT)) + (OAK_D / 2) * math.sin(R(OAK_TILT))
oak_bot = OAK_Z - (OAK_H / 2) * math.cos(R(OAK_TILT)) - (OAK_D / 2) * math.sin(R(OAK_TILT))
cap_top = cap_z0 + cap_h
top = max(oak_top, cap_top)
d455_top = D455_Z + (SHELF_RIM + D455_H) * math.cos(R(D455_TILT))
if pz0 < -1:
    print(f"⚠️ OAK 판이 캡 아래로 {-pz0:.0f}mm 내려옴 — seg1 앞면과 {OAK_STANDOFF}mm 간격. 조립 STEP 에서 간섭 확인")

# ───────────────────────────── 출력 ─────────────────────────────
parts = {"base": base, "imu_lid": lid, "cap": cap}
for i, s in enumerate(segments):
    parts[f"seg{i + 1}"] = s
for name, p in parts.items():
    cq.exporters.export(p, os.path.join(OUT, f"mast_{name}.stl"), tolerance=0.05)
    cq.exporters.export(p, os.path.join(OUT, f"mast_{name}.step"))

asm = cq.Assembly()
asm.add(base, name="base")
asm.add(lid, name="imu_lid", loc=cq.Location(cq.Vector(pcx, pcy, BASE_T - LID_T)), color=cq.Color(0.6, 0.8, 0.6, 0.8))
z = seg_z0
for i, s in enumerate(segments):
    asm.add(s, name=f"seg{i + 1}", loc=cq.Location(cq.Vector(0, 0, z)))
    z += seg_len + DECK_T
asm.add(cap, name="cap", loc=cq.Location(cq.Vector(0, 0, z)))
# 더미: D455 / OAK(+그랜드 아래) / IMU
d455_dummy = (cq.Workplane("XY").center(0, D455_SETBACK + SHELF_RIM + SHELF_MARGIN + D455_D / 2)
              .rect(D455_W, D455_D).extrude(D455_H).translate((0, 0, SHELF_RIM))
              .rotate((0, 0, 0), (1, 0, 0), -D455_TILT).translate((0, TUBE_OD / 2, D455_Z)))
asm.add(d455_dummy, name="D455_dummy", color=cq.Color(0.2, 0.5, 0.9, 0.5))
oak_body = cq.Workplane("XY").box(OAK_W, OAK_D, OAK_H).translate((0, OAK_PLATE_T + OAK_D / 2, OAK_PLATE_H / 2))
oak_conn = cq.Workplane("XY").box(OAK_CONN_W, 16, OAK_CONN_LEN).translate((0, OAK_PLATE_T + OAK_D / 2, OAK_PLATE_H / 2 - OAK_H / 2 - OAK_CONN_LEN / 2))
oak_dummy = (oak_body.union(oak_conn).rotate((0, 0, 0), (1, 0, 0), -OAK_TILT)   # 판과 같은 원점(판 뒷면 아래 가장자리)에서 회전
             .translate((0, TUBE_OD / 2 + OAK_STANDOFF, OAK_Z - OAK_PLATE_H / 2)))
asm.add(oak_dummy, name="OAK_dummy", color=cq.Color(0.9, 0.5, 0.2, 0.5))
imu_dummy = cq.Workplane("XY").box(IMU_L, IMU_W, IMU_H).translate((pcx, pcy, pocket_floor + 0.5 + IMU_H / 2))
asm.add(imu_dummy, name="IMU_dummy", color=cq.Color(0.8, 0.2, 0.2, 0.6))
asm.save(os.path.join(OUT, "mast_assembly.step"))

# 간섭 검사 (부품·더미 쌍)
placed = {"base": base, "imu_lid": lid.translate((pcx, pcy, BASE_T - LID_T)), "cap": cap.translate((0, 0, z)),
          "D455": d455_dummy, "OAK": oak_dummy, "IMU": imu_dummy}
zz = seg_z0
for i, s in enumerate(segments):
    placed[f"seg{i + 1}"] = s.translate((0, 0, zz)); zz += seg_len + DECK_T
names = list(placed)
bad = []
for a in range(len(names)):
    for b in range(a + 1, len(names)):
        try:
            v = placed[names[a]].intersect(placed[names[b]]).val().Volume()
        except Exception:
            v = 0.0
        if v > 5.0:
            bad.append(f"{names[a]}×{names[b]} {v:.0f}mm³")

print(f"마디 {n_seg}개 × {seg_len:.0f} (갑판 {seg_z0:.0f}~{seg_top:.0f}) / 캡 상단 {cap_top:.0f} / OAK 상단 {oak_top:.1f} (아래끝 {oak_bot:.1f}, 그랜드 끝 ≈{oak_bot - OAK_CONN_LEN:.0f}) / D455 선반 {D455_Z:.0f} 상단 {d455_top:.0f} / 한계 {LIMIT_H:.0f} → {'OK' if top <= LIMIT_H else '초과!'}")
print(f"IMU 포켓 {pocket_x:.0f}×{pocket_y:.0f}×{pocket_z:.0f} (IMU {IMU_L:.0f}×{IMU_W:.0f}×{IMU_H:.0f} +{IMU_CLEAR:.0f}) 중심 ({pcx:.1f},{pcy:.1f}) 터널 {IMU_CH_DIR} Ø{IMU_CH_D:.0f} / 뚜껑 {lid_x:.0f}×{lid_y:.0f}×{LID_T:.0f} 나일론 M3×4 / 베이스 {BASE_W:.0f}×{BASE_L:.0f}×{BASE_T:.0f}")
print("간섭:", ", ".join(bad) if bad else "없음")
print("출력:", ", ".join(sorted(f for f in os.listdir(OUT) if f.startswith("mast_"))))

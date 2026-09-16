#!/usr/bin/env python3
"""mast.py v3 — 카메라 마스트 파라메트릭 CAD (CadQuery). STEP(Fusion 360) + STL(프린트) 출력.

v3 (2026-09-16, `docs/전달용/케이블_출구_맵.md` 반영 — 제조사 도면으로 확인한 케이블 출구 기준):
  · **D455: 선반 삭제 → 뒷면 M4×2(간격 95) 판 마운트.** Intel 도면(337029-017 p150): USB-C 는 ¼-20 과 같은 **바닥면**(중심 +37).
    선반에 얹으면 플러그가 선반을 뚫는다. 판에 걸면 플러그가 아래로 자유롭게 나오고 관 앞면 슬롯으로 들어간다.
  · **IMU: 좌현(−X) 포켓, 터널 +X(관 쪽), 커넥터 쪽 20mm 연장.** iAHRS 도면: 케이스 35×35×10, 커넥터(micro-USB+4핀)는
    X 화살표(앞=뱃머리) 기준 오른쪽(−Y) 면 → 보드 앞을 뱃머리로 두면 커넥터가 우현(+X)을 본다 → 포켓을 좌현에 두어야 케이블이 관 쪽으로.
  · OAK 슬롯 폭 18→40 (그랜드 오프셋 실측 전 여유). 플러그 안쪽 구멍 22 (RJ45 헤드 13×16 통과).
  · v2 유지: OAK 꼭대기 −11°(그랜드 아래), 220 상한, 간섭 검사, PETG/ASA.

부품 4개: base(IMU 포켓·칼라) / seg1(각관 + D455 판) / cap(OAK 판) / imu_lid.
좌표: +Y = 뱃머리, Z = 갑판 윗면 0. 카메라는 +Y 를 본다.

⚠️ 실측 후 고칠 파라미터:
  · OAK_CONN_X — RJ45 그랜드가 OAK 바닥면 중앙에서 좌우로 얼마나 벗어나는지(도착일). 슬롯 40 안이면 무변경.
  · D455_USB_X — USB-C 가 ¼-20 기준 +37 인지 −37 인지(좌우). 더미 위치만 바뀜.
  · BASE_HOLE_PITCH — (130, 가로대 간격). 볼트 M5 길이 = 갑판 t + BASE_T + 10.
  · OAK_TILT / D455_TILT — 물 위 첫 시험 후.

사용:
  pip install cadquery
  python3 tools/cad/mast.py            → tools/cad/out/mast_{base,seg1,cap,imu_lid}.stl/.step, mast_assembly.step
"""
import math
import os

import cadquery as cq

# ───────────────────────────── 파라미터 (mm, deg) ─────────────────────────────
MAST_H         = None    # 각관 마디 길이. None = 자동(OAK 판 하단 = 캡 하단)
SEG_MAX        = 250.0
TUBE_OD        = 40.0
TUBE_WALL      = 5.0
PLUG_LEN       = 30.0
PLUG_CLEAR     = 0.4
PLUG_HOLE      = 22.0    # [v3] 플러그 안쪽 관통 구멍 — RJ45 헤드(13×16) 통과. plug 29.2 → 벽 3.6
BOLT_D         = 4.4     # M4 관통
DECK_T         = 4.0     # 플러그 밑 바닥판
LIMIT_H        = 220.0   # 팀 요구 상한 (갑판 위 최고점)

BASE_W_MIN, BASE_L, BASE_T_MIN = 160.0, 120.0, 20.0
BASE_HOLE_PITCH = (130.0, 90.0)   # x=현 방향, y=90 = 가로대 2개 중심 간격 → 실측 후 수정
BASE_HOLE_D     = 5.5             # M5
CABLE_HOLE_D    = 22.0
COLLAR_H        = 14.0   # [v3] 20→14: D455 USB 슬롯을 낮추려고 (USB 플러그 끝 ≈33)
COLLAR_MARGIN   = 2.0
GUSSET          = 30.0

# IMU (iAHRS RB-SDA-v1) — 매뉴얼 도면 확정값
IMU_L, IMU_W, IMU_H = 35.0, 35.0, 10.0   # 케이스 (L=선체 좌우 X, W=전후 Y, H=높이)
IMU_CLEAR   = 1.0      # 전체 +1
IMU_CONN_EXT = 20.0    # [v3] 커넥터 면(+X, 관 쪽) 쪽 포켓 연장 — micro-USB/4핀 플러그 + 굽힘
IMU_CH_D    = 8.0      # 케이블 터널 지름
IMU_PAD_T   = 2.0      # 포켓 바닥 고무 방진패드 (매뉴얼 권장) — 포켓 깊이에 포함
LID_T       = 3.0
LID_MARGIN  = 8.0
LID_SCREW_D = 2.5      # 나일론 M3 탭 (베이스), 뚜껑 3.4 관통

# D455: 124 × 26 × 29. 뒷면 M4×2 간격 95 (삽입 ≤4mm, 0.4Nm). 바닥 ¼-20 + USB-C(중심 +37) + M2×2 케이블 잠금
D455_W, D455_D, D455_H = 124.0, 26.0, 29.0
D455_HOLE_PITCH = 95.0
D455_HOLE_D     = 4.4
D455_ZC         = 72.0   # 갑판 → D455 중심(=M4 구멍 높이). 상단 ≈89 ↔ OAK 그랜드 끝 ≈97
D455_TILT       = 15.0   # 아래로
D455_PLATE_W, D455_PLATE_H, D455_PLATE_T = 115.0, 30.0, 6.0
D455_STANDOFF   = 1.0
D455_USB_X      = 37.0   # ¼-20 기준 USB-C 좌우 오프셋 (부호 실측)
D455_USB_LEN    = 25.0   # USB-C 플러그 + 몰드 돌출 (아래)
D455_SLOT_Z     = None   # 관 앞면 케이블 슬롯 중심 높이. None = 칼라 위 자동

# OAK-1 PoE: 81.9 × 81.9 × 31, 뒷면 M4×4 (45/37.5), 바닥 ¼-20, 그랜드 측면(아래로 장착)
OAK_W, OAK_H, OAK_D = 81.9, 81.9, 31.0
OAK_HOLE_PITCH = (45.0, 37.5)
OAK_HOLE_D     = 4.4
OAK_TILT       = 11.0
OAK_Z          = 175.0   # 갑판 → OAK 중심. 상단 ≈218
OAK_PLATE_T    = 6.0
OAK_PLATE_H    = 50.0
OAK_STANDOFF   = 1.0
OAK_CONN_LEN, OAK_CONN_W = 35.0, 27.0   # RJ45 그랜드 돌출(아래)·지름
OAK_CONN_X     = 0.0     # 그랜드 좌우 오프셋 (실측)
OAK_SLOT_W     = 40.0    # [v3] 18→40

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)

inner = TUBE_OD - 2 * TUBE_WALL
plug = inner - 2 * PLUG_CLEAR
R = math.radians


def tube(length):
    return (cq.Workplane("XY").rect(TUBE_OD, TUBE_OD).extrude(length)
            .faces(">Z").workplane().rect(inner, inner).cutThruAll())


def plug_top(body, z_top):
    hole = PLUG_HOLE
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


# ───────────────────────────── 1. 베이스 (IMU 포켓 — 좌현) ─────────────────────────────
pocket_x = IMU_L + IMU_CLEAR + IMU_CONN_EXT          # X: IMU + 커넥터 연장(+X 쪽)
pocket_y = IMU_W + IMU_CLEAR
pocket_z = IMU_H + IMU_CLEAR + IMU_PAD_T
lid_x, lid_y = pocket_x + 2 * LID_MARGIN, pocket_y + 2 * LID_MARGIN
collar_half = TUBE_OD / 2 + COLLAR_MARGIN + PLUG_CLEAR
# 포켓 +X 끝(연장부 끝)이 칼라 옆 1.5mm 에 오도록
pocket_xmax = -(collar_half + 1.5) - LID_MARGIN
pcx = pocket_xmax - pocket_x / 2            # 포켓 중심 x (음수 = 좌현)
pcy = 0.0
imu_cx = pcx - IMU_CONN_EXT / 2             # IMU 케이스 중심 (포켓의 −X 쪽에 붙음)
BASE_W = max(BASE_W_MIN, 2 * (abs(pcx) + lid_x / 2 + 4.0))
BASE_T = max(BASE_T_MIN, math.ceil(pocket_z + LID_T + 4.0))
if MAST_H is None:
    MAST_H = OAK_Z - OAK_PLATE_H / 2 - BASE_T - 2 * DECK_T
    print(f"ℹ️ MAST_H 자동 = {MAST_H:.0f}")

base = cq.Workplane("XY").rect(BASE_W, BASE_L).extrude(BASE_T).edges("|Z").fillet(8)
base = base.faces(">Z").workplane().rect(*BASE_HOLE_PITCH, forConstruction=True).vertices().hole(BASE_HOLE_D)
base = plug_top(base, BASE_T)
base = base.cut(cq.Workplane("XY").circle(CABLE_HOLE_D / 2).extrude(BASE_T + DECK_T + PLUG_LEN + 1))
for ang in (90, 270, 0):   # ±Y 거싯 + 우현(+X) 거싯 (좌현은 포켓)
    g = (cq.Workplane("XZ").polyline([(collar_half - 0.1, BASE_T), (collar_half + GUSSET, BASE_T),
                                      (collar_half - 0.1, BASE_T + COLLAR_H)]).close().extrude(4, both=True)
         .rotate((0, 0, 0), (0, 0, 1), ang))
    base = base.union(g)
collar = (cq.Workplane("XY").workplane(offset=BASE_T).rect(2 * collar_half, 2 * collar_half).extrude(COLLAR_H)
          .faces(">Z").workplane().rect(TUBE_OD + 2 * PLUG_CLEAR, TUBE_OD + 2 * PLUG_CLEAR).cutThruAll())
base = base.union(collar)

pocket_floor = BASE_T - LID_T - pocket_z
base = base.cut(cq.Workplane("XY").workplane(offset=pocket_floor).center(pcx, pcy).rect(pocket_x, pocket_y).extrude(pocket_z + LID_T + 1))
base = base.cut(cq.Workplane("XY").workplane(offset=BASE_T - LID_T).center(pcx, pcy).rect(lid_x, lid_y).extrude(LID_T + 1))
screw_pts = [(pcx + sx * (pocket_x / 2 + LID_MARGIN / 2), pcy + sy * (pocket_y / 2 + LID_MARGIN / 2))
             for sx in (-1, 1) for sy in (-1, 1)]
for (sx, sy) in screw_pts:
    base = base.cut(cq.Workplane("XY").workplane(offset=BASE_T - LID_T - 9).center(sx, sy).circle(LID_SCREW_D / 2).extrude(10))
# 케이블 터널: 포켓 +X 벽 → 중앙 케이블 구멍 (YZ 평면 법선 +X, offset = 시작 x)
ch_z = pocket_floor + IMU_PAD_T + IMU_CH_D / 2 + 0.5
x_start = pcx + pocket_x / 2 - 1.0
base = base.cut(cq.Workplane("YZ").workplane(offset=x_start).center(pcy, ch_z).circle(IMU_CH_D / 2).extrude(-x_start + 1.0))
# 포켓 바닥 화살표 (+Y = 뱃머리) 0.6 각인 — IMU 자리 중심에
arrow = (cq.Workplane("XY").workplane(offset=pocket_floor - 0.6).center(imu_cx, pcy)
         .polyline([(0, 12), (7, 4), (2.5, 4), (2.5, -12), (-2.5, -12), (-2.5, 4), (-7, 4)]).close().extrude(1.0))
base = base.cut(arrow)
# IMU 케이스 Ø2 홀 4개 → 포켓 바닥 M2 탭 구멍 (모서리 3, 간격 29) — 뚜껑 누름 대신 직접 고정 선택지
for sx in (-1, 1):
    for sy in (-1, 1):
        base = base.cut(cq.Workplane("XY").workplane(offset=pocket_floor - 6).center(imu_cx + sx * 14.5, pcy + sy * 14.5)
                        .circle(1.6 / 2).extrude(7))

lid = cq.Workplane("XY").rect(lid_x - 0.4, lid_y - 0.4).extrude(LID_T).edges("|Z").fillet(2)
for (sx, sy) in screw_pts:
    lid = lid.cut(cq.Workplane("XY").center(sx - pcx, sy - pcy).circle(3.4 / 2).extrude(LID_T))
lid = lid.cut(cq.Workplane("XY").workplane(offset=LID_T - 0.8).center(0, 0).rect(12, 3).extrude(1))

# ───────────────────────────── 2. 마디 (D455 뒷면 판) ─────────────────────────────
n_seg = math.ceil(MAST_H / SEG_MAX)
seg_len = MAST_H / n_seg
seg_z0 = BASE_T + DECK_T
segments = []
d455_slot_z = D455_SLOT_Z if D455_SLOT_Z is not None else BASE_T + COLLAR_H + 10.0
for i in range(n_seg):
    s = tube(seg_len)
    s = socket_bolts(s, 0)
    s = plug_top(s, seg_len)
    z0 = seg_z0 + i * (seg_len + DECK_T)
    if z0 <= D455_ZC < z0 + seg_len:
        zl = D455_ZC - z0
        yf = TUBE_OD / 2
        # 판: 관 앞면에 붙는 수직 판 (뒤면 y=yf+standoff), 구멍 2개 가로 95
        plate = cq.Workplane("XY").box(D455_PLATE_W, D455_PLATE_T, D455_PLATE_H) \
            .translate((0, yf + D455_STANDOFF + D455_PLATE_T / 2, zl)).edges("|Y").fillet(4)
        for hx in (-D455_HOLE_PITCH / 2, D455_HOLE_PITCH / 2):
            plate = plate.cut(cq.Workplane("XZ").workplane(offset=-(yf + D455_STANDOFF + D455_PLATE_T + 1))
                              .center(hx, zl).circle(D455_HOLE_D / 2).extrude(D455_PLATE_T + 2))
        # 스탠드오프 + 거싯 (판 뒤 ↔ 관 면), 판 폭 안쪽 x=±30
        for gx in (-30, 30):
            g = (cq.Workplane("YZ").workplane(offset=gx - 2)
                 .polyline([(yf - 0.1, zl + D455_PLATE_H / 2), (yf + D455_STANDOFF + 0.2, zl + D455_PLATE_H / 2),
                            (yf + D455_STANDOFF + 0.2, zl - D455_PLATE_H / 2), (yf - 0.1, zl - D455_PLATE_H / 2 - 20)]).close().extrude(4))
            plate = plate.union(g)
        # 아래로 D455_TILT: 판 뒤 중심선(y=yf, z=zl) 기준
        plate = plate.rotate((0, yf, zl), (1, yf, zl), -D455_TILT)
        # 기울인 판 위쪽과 관 면 사이 쐐기
        h2 = D455_PLATE_H / 2
        wedge = (cq.Workplane("YZ").workplane(offset=-D455_PLATE_W / 2)
                 .polyline([(yf - 0.1, zl - 2), (yf + D455_STANDOFF + h2 * math.sin(R(D455_TILT)) + 0.3, zl - 2),
                            (yf + D455_STANDOFF + h2 * math.sin(R(D455_TILT)) + 0.3, zl + h2 * math.cos(R(D455_TILT))),
                            (yf - 0.1, zl + h2 * math.cos(R(D455_TILT)))]).close().extrude(D455_PLATE_W))
        # 관 앞면 케이블 슬롯 (USB-C 플러그 아래에서 관 안으로)
        slot = cq.Workplane("XZ").workplane(offset=-(yf + 1)).center(0, d455_slot_z - z0).rect(18, 14).extrude(TUBE_WALL + 2)
        s = s.union(wedge).union(plate).cut(slot)
    segments.append(s)
seg_top = seg_z0 + n_seg * seg_len + (n_seg - 1) * DECK_T

# ───────────────────────────── 3. 캡 (OAK 판) ─────────────────────────────
cap_z0 = seg_top + DECK_T
cap_h = PLUG_LEN + 6
cap = tube(cap_h)
cap = socket_bolts(cap, 0)
cap = cap.union(cq.Workplane("XY").workplane(offset=cap_h - 6).rect(TUBE_OD, TUBE_OD).extrude(6))
pz0 = OAK_Z - OAK_PLATE_H / 2 - cap_z0
plate = (cq.Workplane("XY").center(0, OAK_PLATE_T / 2).rect(OAK_W + 10, OAK_PLATE_T).extrude(OAK_PLATE_H)
         .edges("|Y").fillet(4))
for hx in (-OAK_HOLE_PITCH[0] / 2, OAK_HOLE_PITCH[0] / 2):
    for hz in (-OAK_HOLE_PITCH[1] / 2, OAK_HOLE_PITCH[1] / 2):
        plate = plate.cut(cq.Workplane("XZ").workplane(offset=-OAK_PLATE_T - 1).center(hx, OAK_PLATE_H / 2 + hz)
                          .circle(OAK_HOLE_D / 2).extrude(OAK_PLATE_T + 2))
plate = plate.rotate((0, 0, 0), (1, 0, 0), -OAK_TILT).translate((0, TUBE_OD / 2 + OAK_STANDOFF, pz0))
wz1 = min(cap_h, pz0 + OAK_PLATE_H)
wedge = (cq.Workplane("YZ").workplane(offset=-TUBE_OD / 2)
         .polyline([(TUBE_OD / 2 - 0.1, max(0, pz0)), (TUBE_OD / 2 + OAK_STANDOFF + (max(0, pz0) - pz0) * math.tan(R(OAK_TILT)) + 0.3, max(0, pz0)),
                    (TUBE_OD / 2 + OAK_STANDOFF + (wz1 - pz0) * math.tan(R(OAK_TILT)) + 0.3, wz1), (TUBE_OD / 2 - 0.1, wz1)]).close()
         .extrude(TUBE_OD))
cap = cap.union(wedge).union(plate)
# OAK 케이블 슬롯 — 캡 앞면 아래쪽 (그랜드가 아래로 나와 관 안으로), 폭 40
oak_slot = cq.Workplane("XZ").workplane(offset=-(TUBE_OD / 2 + 1)).center(OAK_CONN_X, max(3, pz0 - 4) + 7).rect(OAK_SLOT_W, 14).extrude(TUBE_WALL + 2)
cap = cap.cut(oak_slot)

# ───────────────────────────── 4. 검산 ─────────────────────────────
oak_top = OAK_Z + (OAK_H / 2) * math.cos(R(OAK_TILT)) + (OAK_D / 2) * math.sin(R(OAK_TILT))
oak_bot = OAK_Z - (OAK_H / 2) * math.cos(R(OAK_TILT)) - (OAK_D / 2) * math.sin(R(OAK_TILT))
cap_top = cap_z0 + cap_h
top = max(oak_top, cap_top)
d455_top = D455_ZC + (D455_H / 2) * math.cos(R(D455_TILT)) + (D455_D / 2) * math.sin(R(D455_TILT))
d455_usb_end = D455_ZC - (D455_H / 2) * math.cos(R(D455_TILT)) - D455_USB_LEN

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
# 더미: D455(+USB 플러그 아래) / OAK(+그랜드 아래) / IMU
yf = TUBE_OD / 2
d455_body = cq.Workplane("XY").box(D455_W, D455_D, D455_H).translate((0, yf + D455_STANDOFF + D455_PLATE_T + D455_D / 2, D455_ZC))
d455_usb = cq.Workplane("XY").box(10, 8, D455_USB_LEN).translate((D455_USB_X, yf + D455_STANDOFF + D455_PLATE_T + D455_D / 2, D455_ZC - D455_H / 2 - D455_USB_LEN / 2))
d455_dummy = d455_body.union(d455_usb).rotate((0, yf, D455_ZC), (1, yf, D455_ZC), -D455_TILT)
asm.add(d455_dummy, name="D455_dummy", color=cq.Color(0.2, 0.5, 0.9, 0.5))
oak_body = cq.Workplane("XY").box(OAK_W, OAK_D, OAK_H).translate((0, OAK_PLATE_T + OAK_D / 2, OAK_PLATE_H / 2))
oak_conn = cq.Workplane("XY").box(OAK_CONN_W, 16, OAK_CONN_LEN).translate((OAK_CONN_X, OAK_PLATE_T + OAK_D / 2, OAK_PLATE_H / 2 - OAK_H / 2 - OAK_CONN_LEN / 2))
oak_dummy = (oak_body.union(oak_conn).rotate((0, 0, 0), (1, 0, 0), -OAK_TILT)
             .translate((0, TUBE_OD / 2 + OAK_STANDOFF, OAK_Z - OAK_PLATE_H / 2)))
asm.add(oak_dummy, name="OAK_dummy", color=cq.Color(0.9, 0.5, 0.2, 0.5))
imu_dummy = cq.Workplane("XY").box(IMU_L, IMU_W, IMU_H).translate((imu_cx, pcy, pocket_floor + IMU_PAD_T + 0.5 + IMU_H / 2))
asm.add(imu_dummy, name="IMU_dummy", color=cq.Color(0.8, 0.2, 0.2, 0.6))
asm.save(os.path.join(OUT, "mast_assembly.step"))

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

print(f"마디 {n_seg}개 × {seg_len:.0f} (갑판 {seg_z0:.0f}~{seg_top:.0f}) / 캡 상단 {cap_top:.0f} / OAK 상단 {oak_top:.1f} (아래끝 {oak_bot:.1f}, 그랜드 끝 ≈{oak_bot - OAK_CONN_LEN:.0f}) / "
      f"D455 중심 {D455_ZC:.0f} 상단 {d455_top:.1f} USB 끝 ≈{d455_usb_end:.0f} 슬롯 z {d455_slot_z:.0f} / 한계 {LIMIT_H:.0f} → {'OK' if top <= LIMIT_H else '초과!'}")
print(f"IMU 포켓 {pocket_x:.0f}×{pocket_y:.0f}×{pocket_z:.0f} (케이스 {IMU_L:.0f}×{IMU_W:.0f}×{IMU_H:.0f} + 커넥터 연장 {IMU_CONN_EXT:.0f}) 중심 ({pcx:.1f},{pcy:.1f}) IMU 중심 x {imu_cx:.1f} 터널 +X Ø{IMU_CH_D:.0f} / 뚜껑 {lid_x:.0f}×{lid_y:.0f}×{LID_T:.0f} / 베이스 {BASE_W:.0f}×{BASE_L:.0f}×{BASE_T:.0f}")
print("간섭:", ", ".join(bad) if bad else "없음")
print("출력:", ", ".join(sorted(f for f in os.listdir(OUT) if f.startswith("mast_"))))

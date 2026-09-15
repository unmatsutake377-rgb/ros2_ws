#!/usr/bin/env python3
"""mast.py — 카메라 마스트 파라메트릭 CAD (CadQuery). STEP(Fusion 360 에서 열어 수정) + STL(프린트) 출력.

[2026-09-15 v2] 팀 결정 반영 (docs/전달용/마스트_요구사항.md §2-5·2-6, 하드웨어_배치_요구사항.md)
  · 마스트 ≤ 220mm 프린트(PETG/ASA), 갑판 관통 M5×4 + 가로대. 두 배 공용.
  · **OAK 가 위(−11°), D455 가 아래(−15°)** — 물 띠 = atan(갑판 위 높이 / 뱃머리 거리). 높아야 화면에 물이 많다.
  · **IMU 는 베이스 블록 포켓** — 박스 안은 노트북·MD-30A 와 동거해 파손 위험. 베이스를 20mm 로 올리고 귀퉁이에 포켓+뚜껑.
    포켓 바닥에 선수(+Y) 화살표 양각, 포켓→중앙 케이블 구멍으로 Ø6 홈. 뚜껑 나사는 나일론 M3(자력계 옆 강철 ✗).
  · LiDAR·GPS·LTE 는 마스트에 안 올린다 (갑판).
  · 케이블: 베이스 중앙 Ø22 → 갑판 밑 → 박스 앞면.

⚠️ 실측 후 고칠 파라미터:
  · IMU_L/W/H — iAHRS RB-SDA-v1 보드(케이스 포함) 실측. 기본값은 넉넉히 잡은 추정.
  · OAK_* — OAK-1 PoE 데이터시트(81.9×81.9×31, 뒷면 M4 45/37.5). 도착 시 실측.
  · BASE_HOLE_PITCH — 가로대 평철 간격. 갑판 두께 나오면 볼트 길이.
  · OAK_TILT / D455_TILT — 물 위 첫 시험에서.
  · RJ45 그랜드가 OAK 위로 ≈35mm 돌출 — 220 상한은 마스트 본체 기준. 그랜드는 위로.

사용:
  pip install cadquery
  python3 tools/cad/mast.py            → tools/cad/out/mast_*.step|stl, mast_assembly.step (Fusion 360: 파일 > 열기)
"""
import math
import os

import cadquery as cq

# ───────────────────────────── 파라미터 (mm, deg) ─────────────────────────────
LIMIT_H        = 220.0   # 팀 요구: 갑판 윗면 → 마스트 최고점
MAST_H         = 156.0   # 각관 마디 길이 (베이스 위). 20+4+156+4+36(캡) = 220
SEG_MAX        = 250.0   # 프린터 Z 한계
TUBE_OD        = 40.0
TUBE_WALL      = 5.0
PLUG_LEN       = 30.0
PLUG_CLEAR     = 0.4
BOLT_D         = 4.4     # M4 관통

BASE_W, BASE_L, BASE_T = 160.0, 120.0, 20.0   # [v2] 8→20: IMU 포켓 + 볼트 뽑힘 여유
BASE_HOLE_PITCH = (130.0, 90.0)
BASE_HOLE_D     = 5.5             # M5
CABLE_HOLE_D    = 22.0            # RJ45 헤드 통과

# IMU 포켓 (베이스 +X+Y 귀퉁이). 좌표계: +Y = 뱃머리.
IMU_L, IMU_W, IMU_H = 34.0, 22.0, 10.0   # ⚠️ 실측. L=선체축(Y) 방향 아님 — 아래 배치에서 L 은 X 방향
IMU_POCKET_C   = (46.0, 24.0)     # 포켓 중심 (M5 구멍·거싯·칼라 피한 자리)
IMU_CLEAR      = 0.5
LID_T          = 3.0
LID_MARGIN     = 4.0              # 포켓보다 사방 4 크게 (나사 자리)
LID_SCREW_D    = 2.5              # M3 나일론 탭 (베이스) / 3.2 관통 (뚜껑)
IMU_CH_D       = 6.0              # 포켓→중앙 구멍 케이블 홈

# D455: 124 × 26 × 29 (W×D×H). 아래단 선반, 벨크로 + 1/4-20 선택.
D455_W, D455_D, D455_H = 124.0, 26.0, 29.0
D455_TILT   = 15.0    # 아래로
D455_Z      = 75.0    # 베이스 윗면 → 선반 밑면 (D455 상단 ≈ 갑판+124, OAK 아래끝 134 과 10mm 여유)
SHELF_RIM   = 3.0
SHELF_MARGIN = 3.0
SHELF_T     = 4.0

# OAK-1 PoE: 81.9 × 81.9 × 31, 뒷면 M4 가로 45 / 세로 37.5. 위단.
OAK_W, OAK_H, OAK_D = 81.9, 81.9, 31.0
OAK_HOLE_X, OAK_HOLE_Z = 45.0, 37.5
OAK_HOLE_D     = 4.4
OAK_TILT       = 11.0    # 아래로 (수평선이 화면 위 30%)
OAK_Z          = 155.0   # 베이스 윗면 → OAK 중심 (갑판+175, 상단 216 ≤ 220)
OAK_PLATE_T    = 6.0

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)

inner = TUBE_OD - 2 * TUBE_WALL
plug = inner - 2 * PLUG_CLEAR
DECK_T = 4.0   # 플러그 밑 바닥판


def tube(length):
    return (cq.Workplane("XY").rect(TUBE_OD, TUBE_OD).extrude(length)
            .faces(">Z").workplane().rect(inner, inner).cutThruAll())


def plug_top(body, z_top):
    hole = plug - 2 * TUBE_WALL
    deck = (cq.Workplane("XY").workplane(offset=z_top).rect(TUBE_OD, TUBE_OD).extrude(DECK_T)
            .faces(">Z").workplane().rect(hole, hole).cutThruAll())
    p = (cq.Workplane("XY").workplane(offset=z_top + DECK_T).rect(plug, plug).extrude(PLUG_LEN)
         .faces(">Z").workplane().rect(hole, hole).cutThruAll())
    body = body.union(deck).union(p)
    body = body.cut(cq.Workplane("YZ").workplane(offset=-TUBE_OD).center(0, z_top + DECK_T + PLUG_LEN / 2)
                    .circle(BOLT_D / 2).extrude(2 * TUBE_OD))
    return body


def socket_bolts(body, z_bottom):
    return body.cut(cq.Workplane("YZ").workplane(offset=-TUBE_OD).center(0, z_bottom + PLUG_LEN / 2)
                    .circle(BOLT_D / 2).extrude(2 * TUBE_OD))


# ───────────────────────────── 1. 베이스 (+ IMU 포켓) ─────────────────────────────
base = (cq.Workplane("XY").rect(BASE_W, BASE_L).extrude(BASE_T).edges("|Z").fillet(8))
base = base.faces(">Z").workplane().rect(*BASE_HOLE_PITCH, forConstruction=True).vertices().hole(BASE_HOLE_D)
base = plug_top(base, BASE_T)
base = base.cut(cq.Workplane("XY").circle(CABLE_HOLE_D / 2).extrude(BASE_T + DECK_T + PLUG_LEN + 1))
for ang in (0, 90, 180, 270):
    g = (cq.Workplane("XZ").polyline([(TUBE_OD / 2 - 0.1, BASE_T), (TUBE_OD / 2 + 40, BASE_T),
                                      (TUBE_OD / 2 - 0.1, BASE_T + 40)]).close().extrude(4, both=True)
         .rotate((0, 0, 0), (0, 0, 1), ang))
    base = base.union(g)
collar = cq.Workplane("XY").workplane(offset=BASE_T).rect(TUBE_OD + 10, TUBE_OD + 10).extrude(20) \
    .faces(">Z").workplane().rect(TUBE_OD + 2 * PLUG_CLEAR, TUBE_OD + 2 * PLUG_CLEAR).cutThruAll()
base = base.union(collar)

# IMU 포켓: 뚜껑 자리(3mm 함몰) + 본 포켓
px, py = IMU_POCKET_C
lid_w, lid_l = IMU_L + 2 * IMU_CLEAR + 2 * LID_MARGIN, IMU_W + 2 * IMU_CLEAR + 2 * LID_MARGIN
pocket_w, pocket_l = IMU_L + 2 * IMU_CLEAR, IMU_W + 2 * IMU_CLEAR
pocket_depth = LID_T + IMU_H + 2.0
base = base.cut(cq.Workplane("XY").workplane(offset=BASE_T - LID_T).center(px, py).rect(lid_w, lid_l).extrude(LID_T + 1))
base = base.cut(cq.Workplane("XY").workplane(offset=BASE_T - pocket_depth).center(px, py).rect(pocket_w, pocket_l).extrude(pocket_depth + 1))
# 뚜껑 나사 4개 (M3 탭 구멍, 베이스)
lid_pts = [(px + sx * (lid_w / 2 - 3.0), py + sy * (lid_l / 2 - 3.0)) for sx in (-1, 1) for sy in (-1, 1)]
base = base.cut(cq.Workplane("XY").workplane(offset=BASE_T - LID_T - 8).pushPoints(lid_pts).circle(LID_SCREW_D / 2).extrude(9))
# 선수(+Y) 화살표 양각 — 포켓 바닥에 0.8mm 파냄 (인쇄 후 보임)
floor_z = BASE_T - pocket_depth
arrow = (cq.Workplane("XY").workplane(offset=floor_z).center(px, py)
         .polyline([(0, 7), (-4, 1), (-1.5, 1), (-1.5, -7), (1.5, -7), (1.5, 1), (4, 1)]).close().extrude(0.8))
base = base.cut(arrow)
# 포켓 → 중앙 케이블 구멍 Ø6 홈 (포켓 바닥 높이, 대각선)
vx, vy = -px, -py
n = math.hypot(vx, vy)
ch_plane = cq.Plane(origin=(px, py, floor_z + IMU_CH_D / 2 + 0.5), xDir=(-vy / n, vx / n, 0), normal=(vx / n, vy / n, 0))
base = base.cut(cq.Workplane(ch_plane).circle(IMU_CH_D / 2).extrude(n))

# 뚜껑 (별도 파트) — 관통 3.2 구멍, 바닥에 IMU 누름 리브 없음(케이블 여유)
lid = (cq.Workplane("XY").rect(lid_w - 2 * 0.3, lid_l - 2 * 0.3).extrude(LID_T)
       .faces(">Z").workplane().pushPoints([(x - px, y - py) for x, y in lid_pts]).hole(3.2))

# ───────────────────────────── 2. 마디 (OAK 판 + D455 선반) ─────────────────────────────
n_seg = math.ceil(MAST_H / SEG_MAX)
seg_len = MAST_H / n_seg
segments = []
for i in range(n_seg):
    s = tube(seg_len)
    s = socket_bolts(s, 0)
    s = plug_top(s, seg_len)
    z0 = DECK_T + i * (seg_len + DECK_T)
    # --- OAK 판 (앞면 +Y, 위단) ---
    if z0 <= OAK_Z < z0 + seg_len:
        zl = OAK_Z - z0
        plate_h = OAK_HOLE_Z + 16
        plate = (cq.Workplane("XZ").workplane(offset=-(TUBE_OD / 2))
                 .center(0, zl).rect(OAK_W + 10, plate_h).extrude(-OAK_PLATE_T)
                 .edges("|Y").fillet(4))
        holes = [(sx * OAK_HOLE_X / 2, sz * OAK_HOLE_Z / 2) for sx in (-1, 1) for sz in (-1, 1)]
        plate = plate.cut(cq.Workplane("XZ").workplane(offset=-(TUBE_OD / 2) - OAK_PLATE_T - 1)
                          .center(0, zl).pushPoints(holes).circle(OAK_HOLE_D / 2).extrude(OAK_PLATE_T + 2))
        # 판 아래 삼각 거싯 2개
        for gx in (-25, 25):
            g = (cq.Workplane("YZ").workplane(offset=gx)
                 .polyline([(-(TUBE_OD / 2) + 0.1, zl - plate_h / 2), (-(TUBE_OD / 2) - OAK_PLATE_T, zl - plate_h / 2),
                            (-(TUBE_OD / 2) + 0.1, zl - plate_h / 2 - 25)]).close().extrude(3, both=True))
            plate = plate.union(g)
        slot = cq.Workplane("XZ").workplane(offset=-(TUBE_OD / 2) - OAK_PLATE_T - 1) \
            .center(0, zl - plate_h / 2 - 10).rect(18, 12).extrude(TUBE_WALL + OAK_PLATE_T + 2)
        plate = plate.rotate((0, TUBE_OD / 2, zl), (1, TUBE_OD / 2, zl), -OAK_TILT)
        s = s.union(plate).cut(slot)
    # --- D455 선반 (앞면 +Y, 아래단) ---
    if z0 <= D455_Z < z0 + seg_len:
        zl = D455_Z - z0
        shelf_w = D455_W + 2 * SHELF_MARGIN + 2 * SHELF_RIM
        shelf_d = D455_D + 2 * SHELF_MARGIN + 2 * SHELF_RIM
        shelf = (cq.Workplane("XY").rect(shelf_w, shelf_d).extrude(SHELF_T + SHELF_RIM)
                 .faces(">Z").workplane().rect(shelf_w - 2 * SHELF_RIM, shelf_d - 2 * SHELF_RIM).cutBlind(-SHELF_RIM))
        shelf = shelf.faces("<Z").workplane().hole(6.6)                                  # 1/4-20 선택
        shelf = shelf.cut(cq.Workplane("XY").center(0, -shelf_d / 2 + 6).rect(14, 8).extrude(20))  # USB 노치(뒤)
        # 선반 뒤 가장자리를 관 앞면에 붙인다: 뒤끝 y = TUBE_OD/2
        shelf = shelf.translate((0, TUBE_OD / 2 + shelf_d / 2, zl))
        # 선반 밑 거싯 2개
        for gx in (-40, 40):
            g = (cq.Workplane("YZ").workplane(offset=gx)
                 .polyline([(TUBE_OD / 2 - 0.1, zl), (TUBE_OD / 2 + shelf_d - 6, zl), (TUBE_OD / 2 - 0.1, zl - 30)])
                 .close().extrude(3, both=True))
            shelf = shelf.union(g)
        # 앞이 내려가게 회전 (뒤 가장자리 기준)
        shelf = shelf.rotate((0, TUBE_OD / 2, zl), (1, TUBE_OD / 2, zl), D455_TILT)
        usb_slot = cq.Workplane("XZ").workplane(offset=-(TUBE_OD / 2) - 1).center(0, zl + 6).rect(14, 10).extrude(TUBE_WALL + 2)
        s = s.union(shelf).cut(usb_slot)
    segments.append(s)

# ───────────────────────────── 3. 상단 캡 (막음) ─────────────────────────────
cap_h = PLUG_LEN + 6
cap = tube(cap_h)
cap = socket_bolts(cap, 0)
cap = cap.union(cq.Workplane("XY").workplane(offset=cap_h - 6).rect(TUBE_OD, TUBE_OD).extrude(6))

# ───────────────────────────── 출력 ─────────────────────────────
parts = {"base": base, "imu_lid": lid, "cap": cap}
for i, s in enumerate(segments):
    parts[f"seg{i + 1}"] = s
for name, p in parts.items():
    cq.exporters.export(p, os.path.join(OUT, f"mast_{name}.stl"), tolerance=0.05)
    cq.exporters.export(p, os.path.join(OUT, f"mast_{name}.step"))

asm = cq.Assembly()
asm.add(base, name="base")
asm.add(lid, name="imu_lid", loc=cq.Location(cq.Vector(px, py, BASE_T - LID_T)), color=cq.Color(0.6, 0.6, 0.6, 0.8))
z = BASE_T + DECK_T
for i, s in enumerate(segments):
    asm.add(s, name=f"seg{i + 1}", loc=cq.Location(cq.Vector(0, 0, z)))
    z += seg_len + DECK_T
asm.add(cap, name="cap", loc=cq.Location(cq.Vector(0, 0, z)))
top_z = z + cap_h
# 카메라·IMU 더미
oak_dummy = (cq.Workplane("XY").box(OAK_W, OAK_D, OAK_H).translate((0, TUBE_OD / 2 + OAK_PLATE_T + OAK_D / 2, 0))
             .rotate((0, TUBE_OD / 2, 0), (1, TUBE_OD / 2, 0), -OAK_TILT).translate((0, 0, BASE_T + OAK_Z)))
asm.add(oak_dummy, name="OAK_dummy", color=cq.Color(0.9, 0.5, 0.2, 0.5))
shelf_d = D455_D + 2 * SHELF_MARGIN + 2 * SHELF_RIM
d455_dummy = (cq.Workplane("XY").box(D455_W, D455_D, D455_H)
              .translate((0, TUBE_OD / 2 + shelf_d / 2, SHELF_T + SHELF_RIM + D455_H / 2))
              .rotate((0, TUBE_OD / 2, 0), (1, TUBE_OD / 2, 0), D455_TILT).translate((0, 0, BASE_T + D455_Z)))
asm.add(d455_dummy, name="D455_dummy", color=cq.Color(0.2, 0.5, 0.9, 0.5))
imu_dummy = cq.Workplane("XY").box(IMU_L, IMU_W, IMU_H).translate((px, py, floor_z + IMU_H / 2))
asm.add(imu_dummy, name="IMU_dummy", color=cq.Color(0.8, 0.1, 0.1, 0.6))
asm.save(os.path.join(OUT, "mast_assembly.step"))

oak_top = BASE_T + OAK_Z + (OAK_H / 2) * math.cos(math.radians(OAK_TILT))
print(f"마디 {n_seg}개 × {seg_len:.0f}mm | 캡 상단 {top_z:.0f} | OAK 중심 갑판+{BASE_T + OAK_Z:.0f}, OAK 상단 {oak_top:.0f} | "
      f"D455 선반 갑판+{BASE_T + D455_Z:.0f} | 한계 {LIMIT_H:.0f} → {'OK' if max(top_z, oak_top) <= LIMIT_H else '초과!'}")
print(f"IMU 포켓 {pocket_w:.1f}×{pocket_l:.1f}×{IMU_H + 2:.0f} @({px},{py}), 뚜껑 {lid_w:.0f}×{lid_l:.0f}×{LID_T:.0f}")
print("출력:", ", ".join(sorted(os.listdir(OUT))))

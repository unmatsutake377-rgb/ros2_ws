#!/usr/bin/env python3
"""mast.py — 카메라 마스트 파라메트릭 CAD (CadQuery). STEP(Fusion 360 에서 열어 수정) + STL(프린트) 출력.

왜 이 형태인가:
  · 마스트는 **노트북·배터리 박스 뚜껑 위**에 볼트로 서고, 케이블은 베이스 중앙 구멍으로 박스 안에 들어간다.
  · 올리는 건 **카메라 2대뿐** (D455 = 조종 화면, OAK = 비전). GPS 안테나는 레버암 오차 때문에,
    LiDAR 는 회전면 확보 때문에, IMU 는 진동 때문에 마스트에 올리지 않는다.
  · [09-14] 해풍·좁은 선체 때문에 **낮게(220mm)** — 마디 1개로 끝남. 250 넘기면 자동 분할(플러그 + M4).
  · 선체: 1번 배 166×58cm, 2번 배 170×43cm(깊이 18cm). 2번 배가 좁아 롤이 크다 → 카메라 무게중심을 낮게.
  · 각도(D455 아래 15°, OAK 아래 7°)는 물 위 첫 시험에서 바꿀 수 있게 파라미터.

⚠️ 실측 필요:
  · OAK-1 PoE(구매 확정 09-15, 고정초점 69°): 81.9×81.9×31, 뒷면 M4×4(가로 45 / 세로 37.5) + 바닥 1/4-20. RJ45 그랜드(케이블 돌출 ≈35mm) 는 위쪽.
  · ⚠️ 팀 `docs/전달용/마스트_요구사항.md`(09-15) 는 LiDAR·GPS·LTE 안테나까지 마스트에 올리는 알루미늄/카본 마스트를 전제 — 이 파일(카메라 2대만, PLA, ≤22cm) 과 개념이 다르다. 결정 전까지 둘 다 잠정.
  · 박스 뚜껑 두께·볼트 위치 (BASE_HOLE_PITCH) — 회로팀 박스 도면 나오면 수정.
  · MAST_H: 박스 뚜껑이 수면에서 얼마나 높은지에 따라. D455 가 수면 60~70cm 오게.

사용:
  pip install cadquery
  python3 tools/cad/mast.py            → out/mast_*.step, out/mast_*.stl
"""
import math
import os
import sys

import cadquery as cq

# ───────────────────────────── 파라미터 (mm, deg) ─────────────────────────────
# [09-15] 선체 STL 분석 결과 두 배 모두 갑판 없는 오픈 헐 → 마스트는 양현 거널을 가로지르는 **가로대(알루미늄 평철 25×3 ×2)** 위에 선다.
#   [09-15] 마스트 높이 상한 220mm(팀 요구) → 두 배 공용 한 설계. BOAT 변수는 출력 폴더 구분용으로만 남김.
BOAT = int(os.environ.get("BOAT", "2"))
MAST_H         = 150.0   # [09-15] 팀 요구: 마스트 전체(갑판~선반 최고점) ≤ 220mm → 150 + 바닥판·캡·기운 선반 ≈ 220. 두 배 공용
SEG_MAX        = 250.0   # 프린터 Z 한계(모르면 250)
TUBE_OD        = 40.0    # 각관 외경
TUBE_WALL      = 5.0     # [09-14] 4→5, 해풍 굽힘 여유
PLUG_LEN       = 30.0    # 마디 플러그 길이
PLUG_CLEAR     = 0.4     # 플러그 편측 여유 (PLA 기준, 헐거우면 0.3)
BOLT_D         = 4.4     # M4 관통

BASE_W, BASE_L, BASE_T = 160.0, 120.0, 8.0   # [09-14] 넓고 두껍게 — 볼트 뽑힘·판 휨 여유
BASE_HOLE_PITCH = (130.0, 90.0)   # x=현 방향(가로대 따라), y=90 = 평철 2개 중심 간격. 평철 폭 25 위에 구멍이 온다
BASE_HOLE_D     = 5.5             # M5
CABLE_HOLE_D    = 22.0            # 케이블 통과 (RJ45 커넥터 16mm 지나감)

# D455: 124 × 26 × 29 (W×D×H). 밑면 1/4-20. 벨크로 고정 예정.
D455_W, D455_D, D455_H = 124.0, 26.0, 29.0
D455_TILT   = 15.0    # 아래로 ([09-14] 마스트가 낮아져 뱃머리를 잡으려면 더 숙임)
SHELF_RIM   = 3.0     # 벨크로 밀림 방지 턱
SHELF_MARGIN = 3.0

# OAK-1 PoE (확정): 81.9 × 81.9 × 31, 뒷면 M4 가로 45mm(세로 37.5). 데이터시트 기준 — 도착일 실측
OAK_W, OAK_H, OAK_D = 81.9, 81.9, 31.0
OAK_HOLE_PITCH = 45.0
OAK_HOLE_D     = 4.4
OAK_TILT       = 7.0     # 아래로
OAK_Z          = 95.0    # 베이스 윗면에서 OAK 중심 (OAK 아래끝 ≈ 갑판+6cm). LiDAR 는 갑판에 직접 놓아 회전면을 그 아래(갑판+3cm)로
OAK_PLATE_T    = 6.0

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", f"boat{BOAT}")
os.makedirs(OUT, exist_ok=True)

inner = TUBE_OD - 2 * TUBE_WALL
plug = inner - 2 * PLUG_CLEAR


def tube(length):
    """각관 한 토막 (z: 0→length), 케이블 채널 관통."""
    return (cq.Workplane("XY").rect(TUBE_OD, TUBE_OD).extrude(length)
            .faces(">Z").workplane().rect(inner, inner).cutThruAll())


DECK_T = 4.0   # 플러그 밑 바닥판 두께 (관 속이 비어 있으므로 플러그가 설 자리)


def plug_top(body, z_top):
    """토막 위에 바닥판 + 플러그 + 관통 볼트 구멍 (다음 마디가 끼워짐). 케이블 구멍은 뚫려 있다."""
    hole = plug - 2 * TUBE_WALL
    deck = (cq.Workplane("XY").workplane(offset=z_top).rect(TUBE_OD, TUBE_OD).extrude(DECK_T)
            .faces(">Z").workplane().rect(hole, hole).cutThruAll())
    p = (cq.Workplane("XY").workplane(offset=z_top + DECK_T).rect(plug, plug).extrude(PLUG_LEN)
         .faces(">Z").workplane().rect(hole, hole).cutThruAll())
    body = body.union(deck).union(p)
    # 볼트: 플러그 중앙 높이, x 방향 관통
    body = body.cut(cq.Workplane("YZ").workplane(offset=-TUBE_OD).center(0, z_top + DECK_T + PLUG_LEN / 2)
                    .circle(BOLT_D / 2).extrude(2 * TUBE_OD))
    return body


def socket_bolts(body, z_bottom):
    """토막 아래쪽(플러그가 들어오는 자리)에 맞구멍."""
    return body.cut(cq.Workplane("YZ").workplane(offset=-TUBE_OD).center(0, z_bottom + PLUG_LEN / 2)
                    .circle(BOLT_D / 2).extrude(2 * TUBE_OD))


# 좌표계: +Y = 뱃머리(앞). 카메라는 모두 +Y 를 본다.
# ───────────────────────────── 1. 베이스 ─────────────────────────────
base = (cq.Workplane("XY").rect(BASE_W, BASE_L).extrude(BASE_T).edges("|Z").fillet(8))
base = base.faces(">Z").workplane().rect(*BASE_HOLE_PITCH, forConstruction=True).vertices().hole(BASE_HOLE_D)
# 각관 소켓 (베이스 위로 PLUG_LEN 만큼 플러그 돌출)
base = plug_top(base, BASE_T)
# 케이블 구멍 — 베이스 관통 (플러그 안쪽 구멍과 이어짐)
base = base.cut(cq.Workplane("XY").circle(CABLE_HOLE_D / 2).extrude(BASE_T + DECK_T + PLUG_LEN + 1))
# 보강 거싯 4개
for ang in (0, 90, 180, 270):
    g = (cq.Workplane("XZ").polyline([(TUBE_OD / 2 - 0.1, BASE_T), (TUBE_OD / 2 + 40, BASE_T),
                                      (TUBE_OD / 2 - 0.1, BASE_T + 40)]).close().extrude(4, both=True)   # [09-14] 거싯 확대
         .rotate((0, 0, 0), (0, 0, 1), ang))
    base = base.union(g)
# 거싯은 플러그가 아니라 관 바깥에 붙어야 하므로 관 자리에 짧은 칼라를 둔다
collar = cq.Workplane("XY").workplane(offset=BASE_T).rect(TUBE_OD + 10, TUBE_OD + 10).extrude(20) \
    .faces(">Z").workplane().rect(TUBE_OD + 2 * PLUG_CLEAR, TUBE_OD + 2 * PLUG_CLEAR).cutThruAll()
base = base.union(collar)

# ───────────────────────────── 2. 마디들 ─────────────────────────────
n_seg = math.ceil(MAST_H / SEG_MAX)
seg_len = MAST_H / n_seg
segments = []
for i in range(n_seg):
    s = tube(seg_len)
    s = socket_bolts(s, 0)
    s = plug_top(s, seg_len)          # 마지막 마디 위에도 플러그 → 상단 캡이 끼워짐
    z0 = DECK_T + i * (seg_len + DECK_T)   # 베이스 윗면 기준 이 마디의 시작 높이
    # OAK 브래킷: 해당 높이가 이 마디 안에 있으면 앞면(+Y)에 판을 붙인다
    if z0 <= OAK_Z < z0 + seg_len:
        zl = OAK_Z - z0
        plate = (cq.Workplane("XZ").workplane(offset=-(TUBE_OD / 2))   # 관 앞면
                 .center(0, zl).rect(OAK_W + 10, OAK_H * 0.6).extrude(-OAK_PLATE_T)
                 .edges("|Y").fillet(4))
        # 뒷면 M4 구멍 2개 (가로 45mm)
        plate = plate.cut(cq.Workplane("XZ").workplane(offset=-(TUBE_OD / 2) - OAK_PLATE_T - 1)
                          .center(0, zl).rect(OAK_HOLE_PITCH, 0.01, forConstruction=True)
                          .vertices().circle(OAK_HOLE_D / 2).extrude(OAK_PLATE_T + 2))
        # RJ45 케이블 슬롯 (판 아래쪽 → 관 내부)
        slot = cq.Workplane("XZ").workplane(offset=-(TUBE_OD / 2) - OAK_PLATE_T - 1) \
            .center(0, zl - OAK_H * 0.3 - 10).rect(18, 12).extrude(TUBE_WALL + OAK_PLATE_T + 2)
        # 기울기: 판을 X축 기준 회전 (아래로 OAK_TILT)
        plate = plate.rotate((0, TUBE_OD / 2, zl), (1, TUBE_OD / 2, zl), -OAK_TILT)   # 앞면(+Y) 기준 아래로 기울임
        s = s.union(plate).cut(slot)
    segments.append(s)

# ───────────────────────────── 3. 상단 캡 (D455 선반) ─────────────────────────────
shelf_w = D455_W + 2 * SHELF_MARGIN + 2 * SHELF_RIM
shelf_d = D455_D + 2 * SHELF_MARGIN + 2 * SHELF_RIM
cap_h = PLUG_LEN + 6
cap = tube(cap_h)                       # 마지막 플러그에 끼워지는 소켓
cap = socket_bolts(cap, 0)
cap = cap.union(cq.Workplane("XY").workplane(offset=cap_h - 6).rect(TUBE_OD, TUBE_OD).extrude(6))  # 막음
# 선반: 캡 위에, 앞으로 기울임
shelf = (cq.Workplane("XY").rect(shelf_w, shelf_d).extrude(4 + SHELF_RIM)
         .faces(">Z").workplane().rect(shelf_w - 2 * SHELF_RIM, shelf_d - 2 * SHELF_RIM).cutBlind(-SHELF_RIM))
shelf = shelf.faces("<Z").workplane().hole(6.6)                       # 1/4-20 (선택) — 벨크로 대신 쓸 때
shelf = shelf.cut(cq.Workplane("XY").center(0, -shelf_d / 2 + 6).rect(14, 8).extrude(20))  # 케이블 노치(뒤)
shelf = shelf.rotate((0, 0, 0), (1, 0, 0), -D455_TILT).translate((0, 0, cap_h + shelf_d / 2 * math.sin(math.radians(D455_TILT)) + 1))
# 선반과 캡 사이 쐐기
# 앞(+Y)이 낮고 뒤(-Y)가 높은 쐐기 — 선반이 앞으로 D455_TILT 만큼 기운다
wedge = cq.Workplane("YZ").polyline([(-TUBE_OD / 2, cap_h), (TUBE_OD / 2, cap_h),
                                     (TUBE_OD / 2, cap_h + 1),
                                     (-TUBE_OD / 2, cap_h + 1 + TUBE_OD * math.tan(math.radians(D455_TILT)))]).close().extrude(TUBE_OD / 2, both=True)
cap = cap.union(wedge).union(shelf)

# ───────────────────────────── 출력 ─────────────────────────────
parts = {"base": base, "cap_d455": cap}
for i, s in enumerate(segments):
    parts[f"seg{i + 1}"] = s

for name, p in parts.items():
    cq.exporters.export(p, os.path.join(OUT, f"mast_{name}.stl"), tolerance=0.05)
    cq.exporters.export(p, os.path.join(OUT, f"mast_{name}.step"))

# 조립 상태 STEP (검토용) — 실제 높이로 쌓는다
asm = cq.Assembly()
asm.add(base, name="base")
z = BASE_T + DECK_T
for i, s in enumerate(segments):
    asm.add(s, name=f"seg{i + 1}", loc=cq.Location(cq.Vector(0, 0, z)))
    z += seg_len + DECK_T
asm.add(cap, name="cap_d455", loc=cq.Location(cq.Vector(0, 0, z)))
# 카메라 더미 (위치 확인용)
d455_dummy = cq.Workplane("XY").box(D455_W, D455_D, D455_H).translate((0, 0, D455_H / 2 + 4 + SHELF_RIM)) \
    .rotate((0, 0, 0), (1, 0, 0), -D455_TILT).translate((0, 0, z + cap_h + 2))
asm.add(d455_dummy, name="D455_dummy", color=cq.Color(0.2, 0.5, 0.9, 0.5))
oak_dummy = cq.Workplane("XY").box(OAK_W, OAK_D, OAK_H).translate((0, (TUBE_OD / 2) + OAK_PLATE_T + OAK_D / 2, 0)) \
    .rotate((0, TUBE_OD / 2, 0), (1, TUBE_OD / 2, 0), -OAK_TILT).translate((0, 0, BASE_T + OAK_Z))
asm.add(oak_dummy, name="OAK_dummy", color=cq.Color(0.9, 0.5, 0.2, 0.5))
asm.save(os.path.join(OUT, "mast_assembly.step"))

print(f"마디 {n_seg}개 × {seg_len:.0f}mm, 전체 높이(베이스 윗면→D455 선반) ≈ {z - BASE_T + cap_h:.0f}mm")
print("출력:", ", ".join(sorted(os.listdir(OUT))))

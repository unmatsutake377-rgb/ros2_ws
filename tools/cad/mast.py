#!/usr/bin/env python3
"""mast.py v4 — 카메라 마스트 파라메트릭 CAD (CadQuery). STEP(Fusion 360) + STL(프린트) 출력.

v4 (2026-09-16, 팀 피드백 반영):
  · **카메라 2대 모두 "박스에 끼우는" U자 크레들** — 뒷판 + 바닥턱 + 좌우턱, 위 열림. 카메라 무게는 바닥턱이 받고
    M4 나사는 잠금만 한다(v3 는 나사 2개에 매달렸음). 바닥턱에는 커넥터 노치(D455 USB-C ±37 양쪽, OAK 그랜드 중앙).
  · **IMU 포켓을 중심선(X=0) 선미쪽**으로 — 좌우 대칭, 마스트 축에 가깝다. 베이스 폭 200→160 복귀, 길이 120→자동(≈152).
  · **IMU 는 접착식** — 포켓 바닥 M2 탭 구멍 삭제, VHB/방진패드로 붙인다(매뉴얼의 방진패드 권장과 겸함).
    바닥이 평평해야 하므로 뱃머리 화살표는 포켓 바닥이 아니라 **베이스 윗면**에 각인.
  · **D455 를 62 로 내림**(72→62) + L자 USB-C 가정(돌출 25→10) → OAK 그랜드 끝과 여유 7.6 → 17.6.
    ⚠️ OAK 쪽 "L자 RJ45" 는 불가 — 그랜드가 본체 일부라 길이 35 고정. 여유는 D455 를 내려서만 번다.

v3 (2026-09-16, `docs/전달용/케이블_출구_맵.md` 반영 — 제조사 도면으로 확인한 케이블 출구 기준):
  · **D455: 선반 삭제 → 뒷면 M4×2(간격 95) 판 마운트.** Intel 도면(337029-017 p150): USB-C 는 ¼-20 과 같은 **바닥면**(중심 +37).
    선반에 얹으면 플러그가 선반을 뚫는다. 판에 걸면 플러그가 아래로 자유롭게 나오고 관 앞면 슬롯으로 들어간다.
  · **IMU: 좌현(−X) 포켓, 터널 +X(관 쪽), 커넥터 쪽 20mm 연장.** iAHRS 도면: 케이스 35×35×10, 커넥터(micro-USB+4핀)는
    X 화살표(앞=뱃머리) 기준 오른쪽(−Y) 면 → 보드 앞을 뱃머리로 두면 커넥터가 우현(+X)을 본다 → 포켓을 좌현에 두어야 케이블이 관 쪽으로.
  · OAK 슬롯 폭 18→40 (그랜드 오프셋 실측 전 여유). 플러그 안쪽 구멍 22 (RJ45 헤드 13×16 통과).
  · v2 유지: OAK 꼭대기 −11°(그랜드 아래), 220 상한, 간섭 검사, PETG/ASA.

부품 4개: base(IMU 포켓·칼라) / seg1(각관 + D455 크레들) / cap(OAK 크레들) / imu_lid.
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
import json
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

BASE_W_MIN, BASE_L_MIN, BASE_T_MIN = 160.0, 120.0, 20.0   # [v4] BASE_L 은 포켓에 맞춰 자동
BASE_HOLE_PITCH = (130.0, 90.0)   # x=현 방향, y=90 = 가로대 2개 중심 간격 → 실측 후 수정
BASE_HOLE_D     = 5.5             # M5
CABLE_HOLE_D    = 22.0
COLLAR_H        = 14.0   # [v3] 20→14: D455 USB 슬롯을 낮추려고 (USB 플러그 끝 ≈33)
COLLAR_MARGIN   = 2.0
GUSSET          = 30.0

# IMU (iAHRS RB-SDA-v1) — 매뉴얼 도면 확정값
IMU_L, IMU_W, IMU_H = 35.0, 35.0, 10.0   # 케이스 (L=선체 좌우 X, W=전후 Y, H=높이)
IMU_CLEAR   = 1.0      # 전체 +1
IMU_CABLE_SPACE = 24.0 # [v4.1] 14→24. 14 면 IMU 끝(x=10.5)이 터널 입구(x 6~14)를 56% 막았다
IMU_CH_D    = 8.0      # 케이블 터널 지름
IMU_CH_X    = 10.0     # 터널 x 위치 (중앙 케이블 구멍 Ø22 안)
IMU_PAD_T   = 2.0      # [v4] VHB 양면 + 방진패드 두께 (붙이는 방식, 나사 없음)
LID_T       = 3.0
LID_MARGIN  = 8.0
LID_FIT     = 0.8      # [v4.3] 뚜껑↔리베이트 총 여유 (편측 0.4). 전엔 0.4 하드코딩=편측 0.2 라
                       #   리베이트는 작아지고 뚜껑 바깥면은 커지는 프린트 방향에서 안 들어갈 공산이 컸다
LID_SCREW_D = 2.5      # 나일론 M3 탭 (베이스), 뚜껑 3.4 관통

# D455: 124 × 26 × 29. 뒷면 M4×2 간격 95 (삽입 ≤4mm, 0.4Nm). 바닥 ¼-20 + USB-C(중심 +37) + M2×2 케이블 잠금
D455_W, D455_D, D455_H = 124.0, 26.0, 29.0
D455_HOLE_PITCH = 95.0
D455_HOLE_D     = 4.4
D455_ZC         = 62.0   # [v4.4] 높이는 62 유지, **틸트만 15→0°**: LiDAR(갑판+41, 240mm 앞)가 화면 35~49%(중앙부)를 가렸다
                         #   62/0° 면 LiDAR 가 화면 58~75%(아래 1/3)로 내려간다. 70mm 는 3%p 더 좋지만
                         #   OAK 그랜드 여유가 17.6→9.6mm 로 깎여 손해다. 뱃머리는 0° 에서도 보인다
D455_TILT       = 0.0    # [v4.4] 15→0. 틸트가 LiDAR 를 화면 중앙으로 끌어올리는 주범이었다
D455_PLATE_H, D455_PLATE_T = 30.0, 6.0
D455_STANDOFF   = 1.0
D455_USB_X      = 37.0   # ¼-20 기준 USB-C 좌우 오프셋 (부호 실측 → 노치는 ±37 둘 다 뚫는다)
D455_USB_LEN    = 10.0   # [v4] L자 USB-C 케이블 가정 (스트레이트는 25)

# 크레들 공통 (카메라를 끼우는 U자 — 뒷판 + 바닥턱 + 좌우턱, 위 열림)
CRADLE_T        = 4.0    # 턱 두께
CRADLE_CLEAR    = 1.0    # 카메라 ↔ 안쪽 벽 총 여유
CRADLE_LIP_D455 = 12.0   # 좌우턱 높이
CRADLE_LIP_OAK  = 18.0
CRADLE_NOTCH_W  = 22.0   # 바닥턱 커넥터 노치 기본 폭 (D455 USB-C 10 + 여유)
CRADLE_NOTCH_OAK = 35.0  # [v4] OAK 그랜드는 Ø27 → 노치 35 (22 면 양옆 2.5 씩 걸려 간섭)
D455_SLOT_Z     = None   # 관 앞면 케이블 슬롯 중심 높이. None = 칼라 위 자동

# OAK-1 PoE: 81.9 × 81.9 × 31, 뒷면 M4×4 (45/37.5), 바닥 ¼-20, 그랜드 측면(아래로 장착)
OAK_W, OAK_H, OAK_D = 81.9, 81.9, 31.0
OAK_HOLE_PITCH = (45.0, 37.5)
OAK_HOLE_D     = 4.4
OAK_TILT       = 11.0
OAK_Z          = 175.0   # 갑판 → OAK 중심. 상단 ≈218
OAK_PLATE_T    = 6.0
OAK_PLATE_H    = 82.0    # [v4] 50→82: 크레들 뒷판이 OAK 전체 높이를 덮는다
OAK_STANDOFF   = 1.0
OAK_CONN_LEN, OAK_CONN_W = 35.0, 27.0   # RJ45 그랜드 돌출(아래)·지름
OAK_CONN_X     = 0.0     # 그랜드 좌우 오프셋 (실측)
OAK_SLOT_W     = 40.0    # [v3] 18→40

# ── 통합(통짜) 출력용 리브 (v5, 2026-09-17) ──────────────────────────────────
#   통짜로 세워 뽑을 때 크레들 바닥턱 밑을 194mm 서포트탑이 받치다 무너졌다.
#   앞서 핀 2개(±50)를 넣었다가 실패한 이유는 **간격이 100mm** 라 그 사이가 그대로 떴기 때문.
#   → 브리지 가능한 간격(≤30mm)으로 촘촘히 넣으면 바닥턱이 리브 사이를 다리 놓듯 건너간다.
#   D455 는 베이스 판이 바로 밑이라 **수직 핀**(오버행 0°), OAK 는 공중이라 **45° 삼각 리브**.
RIB_T      = 5.0
RIB_XS_D455 = (-63.0, -50.0, -26.0, -12.0, 12.0, 26.0, 50.0, 63.0)  # USB-C(±32~42) 회피, 최대 간격 26
RIB_XS_OAK  = (-40.0, -22.0, 22.0, 40.0)                            # 그랜드 노치(±17.5) 바깥, 최대 간격 18
RIB_ANG    = 45.0        # OAK 리브 빗면 (45° = 자립 한계)

# 🚫 [v4.4 기각] 보강 브레이스(D455 수직 핀 ±50 / OAK 45° 삼각 웨브)를 넣어봤다가 뺐다.
#    목적은 강도가 아니라 **출력 중 서포트탑 대체**였는데, 측정하니 오히려 서포트가 늘었다:
#      seg1 4156→4622mm² (+11%) / cap 3312→4399mm² (+33%), 서포트탑 72→100mm, 재료 +6g
#    얇은 핀 사이·바깥의 바닥턱이 여전히 떠 있고, 45° 웨브는 자기 밑면이 또 오버행이 된다.
#    카메라 하중 자체는 응력 0.03~0.08MPa 로 PETG(40MPa)의 1/500 이라 보강이 필요 없다.
#    → 크레들 바닥턱은 **넓고 평평한 수평 캔틸레버**라 형상으로는 못 없앤다. 서포트를 받아들이고
#      **분할 출력으로 서포트탑을 낮추는** 쪽이 답이다(통짜 194mm → 분할 127/72mm).


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



from mast_parts import collar_ring, cradle   # 공용 형상 함수 (fit_test.py 와 공유)


# ───────────────────────────── 1. 베이스 (IMU 포켓 — 중심선 선미쪽, 접착식) ─────────────────────────────
pocket_x = IMU_L + IMU_CLEAR + IMU_CABLE_SPACE     # X: IMU + 케이블 꺾임 공간(+X 쪽)
pocket_y = IMU_W + IMU_CLEAR
pocket_z = IMU_H + IMU_CLEAR + IMU_PAD_T
lid_x, lid_y = pocket_x + 2 * LID_MARGIN, pocket_y + 2 * LID_MARGIN
collar_half = TUBE_OD / 2 + COLLAR_MARGIN + PLUG_CLEAR
pocket_ymax = -(collar_half + 1.5)                 # 포켓 +Y 끝 = 칼라 뒤 1.5
pcx, pcy = 0.0, pocket_ymax - pocket_y / 2         # [v4] 중심선(X=0), 선미쪽
imu_cx = pcx - IMU_CABLE_SPACE / 2                 # IMU 케이스 중심 (포켓 −X 쪽에 붙임)
BASE_W = max(BASE_W_MIN, lid_x + 24.0)
BASE_L = max(BASE_L_MIN, 2 * (abs(pcy) + lid_y / 2 + 6.0))
BASE_T = max(BASE_T_MIN, math.ceil(pocket_z + LID_T + 4.0))
if MAST_H is None:
    MAST_H = OAK_Z - OAK_H / 2 - BASE_T - 2 * DECK_T
    print(f"ℹ️ MAST_H 자동 = {MAST_H:.1f}")

# ── 통합 시험 출력 모드 (MAST_TEST=1) ────────────────────────────────────────
#   실물과 **같은 형상**을 유지하되 베이스 판만 최소로 줄인다. 갑판이 아직 없어 M5 볼트 구멍은
#   어차피 시험할 수 없으므로 뺀다. 목적: 조립·케이블 경로·서포트 자국을 실물 그대로 확인.
TEST = os.environ.get("MAST_TEST") == "1"
if TEST:
    _x = max(lid_x / 2, collar_half + GUSSET) + 4.0
    _ymin, _ymax = pcy - lid_y / 2 - 4.0, collar_half + GUSSET + 4.0
    BASE_W = 2 * _x
    BASE_L = _ymax - _ymin
    _yoff = (_ymax + _ymin) / 2
    OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out_test")
    os.makedirs(OUT, exist_ok=True)
    print(f"ℹ️ MAST_TEST: 베이스 판 {BASE_W:.0f}×{BASE_L:.0f} (중심 y{_yoff:+.1f}), M5 볼트 구멍 생략")
else:
    _yoff = 0.0

base = cq.Workplane("XY").rect(BASE_W, BASE_L).extrude(BASE_T).translate((0, _yoff, 0)).edges("|Z").fillet(8)
if not TEST:
    base = base.faces(">Z").workplane().rect(*BASE_HOLE_PITCH, forConstruction=True).vertices().hole(BASE_HOLE_D)
base = plug_top(base, BASE_T)
base = base.cut(cq.Workplane("XY").circle(CABLE_HOLE_D / 2).extrude(BASE_T + DECK_T + PLUG_LEN + 1))
for ang in (0, 90, 180):        # ±X + 뱃머리(+Y) 거싯 — 선미(−Y)는 포켓
    g = (cq.Workplane("XZ").polyline([(collar_half - 0.1, BASE_T), (collar_half + GUSSET, BASE_T),
                                      (collar_half - 0.1, BASE_T + COLLAR_H)]).close().extrude(4, both=True)
         .rotate((0, 0, 0), (0, 0, 1), ang))
    base = base.union(g)
base = base.union(collar_ring(TUBE_OD, COLLAR_MARGIN, PLUG_CLEAR, COLLAR_H, BASE_T))

pocket_floor = BASE_T - LID_T - pocket_z
base = base.cut(cq.Workplane("XY").workplane(offset=pocket_floor).center(pcx, pcy).rect(pocket_x, pocket_y).extrude(pocket_z + LID_T + 1))
base = base.cut(cq.Workplane("XY").workplane(offset=BASE_T - LID_T).center(pcx, pcy).rect(lid_x, lid_y).extrude(LID_T + 1))
screw_pts = [(pcx + sx * (pocket_x / 2 + LID_MARGIN / 2), pcy + sy * (pocket_y / 2 + LID_MARGIN / 2))
             for sx in (-1, 1) for sy in (-1, 1)]
for (sx, sy) in screw_pts:
    base = base.cut(cq.Workplane("XY").workplane(offset=BASE_T - LID_T - 9).center(sx, sy).circle(LID_SCREW_D / 2).extrude(10))
# 케이블 터널: 포켓 +Y 벽 → 중앙 케이블 구멍 (XZ 법선 −Y)
ch_z = pocket_floor + IMU_PAD_T + IMU_CH_D / 2 + 0.5
y_start = pocket_ymax - 1.0
base = base.cut(cq.Workplane("XZ").workplane(offset=-y_start).center(IMU_CH_X, ch_z).circle(IMU_CH_D / 2).extrude(y_start - 1.0))
# [v4] 포켓 바닥은 접착면이라 평평하게 둔다 → 뱃머리 화살표는 베이스 윗면(뱃머리 쪽)에 각인
arrow = (cq.Workplane("XY").workplane(offset=BASE_T - 0.6).center(0, collar_half + 14)
         .polyline([(0, 12), (7, 4), (2.5, 4), (2.5, -12), (-2.5, -12), (-2.5, 4), (-7, 4)]).close().extrude(1.0))
base = base.cut(arrow)

lid = cq.Workplane("XY").rect(lid_x - LID_FIT, lid_y - LID_FIT).extrude(LID_T).edges("|Z").fillet(2)
for (sx, sy) in screw_pts:
    lid = lid.cut(cq.Workplane("XY").center(sx - pcx, sy - pcy).circle(3.4 / 2).extrude(LID_T))
lid = lid.cut(cq.Workplane("XY").workplane(offset=LID_T - 0.8).center(0, 0).rect(12, 3).extrude(1))

# ───────────────────────────── 2. 마디 (D455 크레들) ─────────────────────────────
yf = TUBE_OD / 2
n_seg = math.ceil(MAST_H / SEG_MAX)
seg_len = MAST_H / n_seg
seg_z0 = BASE_T + DECK_T
d455_slot_z = D455_SLOT_Z if D455_SLOT_Z is not None else BASE_T + COLLAR_H + 10.0
d455_zb = D455_ZC - D455_H / 2                      # 카메라 바닥 (절대)
segments = []
for i in range(n_seg):
    s = tube(seg_len)
    s = socket_bolts(s, 0)
    s = plug_top(s, seg_len)
    z0 = seg_z0 + i * (seg_len + DECK_T)
    if z0 <= D455_ZC < z0 + seg_len:
        zb = d455_zb - z0                            # 마디 로컬 카메라 바닥
        cr = cradle(D455_W, D455_D, D455_H, D455_PLATE_H, D455_PLATE_T, CRADLE_LIP_D455,
                    CRADLE_T, CRADLE_CLEAR, (-D455_USB_X, D455_USB_X), CRADLE_NOTCH_W)
        for hx in (-D455_HOLE_PITCH / 2, D455_HOLE_PITCH / 2):   # 뒷판 M4 관통 2개
            cr = cr.cut(cq.Workplane("XZ").workplane(offset=1).center(hx, D455_H / 2)
                        .circle(D455_HOLE_D / 2).extrude(-(D455_PLATE_T + 2)))
        cr = cr.translate((0, yf + D455_STANDOFF, zb))
        for gx in (-34, 34):                         # 뒷판 ↔ 관 거싯
            g = (cq.Workplane("YZ").workplane(offset=gx - 2)
                 .polyline([(yf - 0.1, zb + D455_H / 2 + 12), (yf + D455_STANDOFF + 0.2, zb + D455_H / 2 + 12),
                            (yf + D455_STANDOFF + 0.2, zb - CRADLE_T), (yf - 0.1, zb - CRADLE_T - 16)]).close().extrude(4))
            cr = cr.union(g)
        cr = cr.rotate((0, yf + D455_STANDOFF, zb), (1, yf + D455_STANDOFF, zb), -D455_TILT)
        cr = cr.cut(cq.Workplane("XY").box(300, 120, 400).translate((0, yf - 60, zb)))   # 관 뒤로 넘어간 부분 제거
        wedge = (cq.Workplane("YZ").workplane(offset=-30)
                 .polyline([(yf - 0.1, zb - 2), (yf + D455_STANDOFF + 0.4, zb - 2),
                            (yf + D455_STANDOFF + D455_PLATE_H * math.sin(R(D455_TILT)) + 0.4, zb + D455_PLATE_H * math.cos(R(D455_TILT))),
                            (yf - 0.1, zb + D455_PLATE_H)]).close().extrude(60))
        # [v5] 수직 핀 — 베이스 판(z=BASE_T) 위에 서서 바닥턱 밑을 받친다. 오버행 0°.
        _fz  = zb - CRADLE_T                                   # 바닥턱 밑면 (마디 로컬)
        _fy0 = collar_half + 0.6                               # 칼라 바깥부터
        _fy1 = yf + D455_STANDOFF + D455_PLATE_T + D455_D + 1.0 + CRADLE_T
        for _rx in RIB_XS_D455:
            fin = (cq.Workplane("YZ").workplane(offset=_rx - RIB_T / 2)
                   .polyline([(_fy0, BASE_T - z0), (_fy1, BASE_T - z0), (_fy1, _fz), (_fy0, _fz)]).close()
                   .extrude(RIB_T))
            cr = cr.union(fin)
        slot = cq.Workplane("XZ").workplane(offset=-(yf + 1)).center(0, d455_slot_z - z0).rect(18, 14).extrude(TUBE_WALL + 2)
        s = s.union(wedge).union(cr).cut(slot)
    segments.append(s)
seg_top = seg_z0 + n_seg * seg_len + (n_seg - 1) * DECK_T

# ───────────────────────────── 3. 캡 (OAK 크레들) ─────────────────────────────
cap_z0 = seg_top + DECK_T
cap_h = PLUG_LEN + 6
cap = tube(cap_h)
cap = socket_bolts(cap, 0)
cap = cap.union(cq.Workplane("XY").workplane(offset=cap_h - 6).rect(TUBE_OD, TUBE_OD).extrude(6))
oak_zb = OAK_Z - OAK_H / 2
zb_l = oak_zb - cap_z0                               # 캡 로컬 카메라 바닥
cro = cradle(OAK_W, OAK_D, OAK_H, OAK_PLATE_H, OAK_PLATE_T, CRADLE_LIP_OAK,
             CRADLE_T, CRADLE_CLEAR, (OAK_CONN_X,), CRADLE_NOTCH_OAK)
for hx in (-OAK_HOLE_PITCH[0] / 2, OAK_HOLE_PITCH[0] / 2):
    for hz in (-OAK_HOLE_PITCH[1] / 2, OAK_HOLE_PITCH[1] / 2):
        cro = cro.cut(cq.Workplane("XZ").workplane(offset=1).center(hx, OAK_H / 2 + hz)
                      .circle(OAK_HOLE_D / 2).extrude(-(OAK_PLATE_T + 2)))
cro = cro.translate((0, yf + OAK_STANDOFF, zb_l))
for gx in (-30, 30):
    g = (cq.Workplane("YZ").workplane(offset=gx - 2)
         .polyline([(yf - 0.1, zb_l + OAK_H / 2), (yf + OAK_STANDOFF + 0.2, zb_l + OAK_H / 2),
                    (yf + OAK_STANDOFF + 0.2, zb_l - CRADLE_T), (yf - 0.1, zb_l - CRADLE_T - 18)]).close().extrude(4))
    cro = cro.union(g)
cro = cro.rotate((0, yf + OAK_STANDOFF, zb_l), (1, yf + OAK_STANDOFF, zb_l), -OAK_TILT)
cro = cro.cut(cq.Workplane("XY").box(300, 120, 500).translate((0, yf - 60, zb_l)))
wedge_o = (cq.Workplane("YZ").workplane(offset=-30)
           .polyline([(yf - 0.1, max(1.0, zb_l - 2)), (yf + OAK_STANDOFF + 0.4, max(1.0, zb_l - 2)),
                      (yf + OAK_STANDOFF + OAK_PLATE_H * math.sin(R(OAK_TILT)) + 0.4, zb_l + OAK_PLATE_H * math.cos(R(OAK_TILT))),
                      (yf - 0.1, zb_l + OAK_PLATE_H)]).close().extrude(60))
oak_slot = cq.Workplane("XZ").workplane(offset=-(yf + 1)).center(OAK_CONN_X, max(4.0, zb_l - 6)).rect(OAK_SLOT_W, 14).extrude(TUBE_WALL + 2)
# [v5] 45° 삼각 리브 — 공중에 뜬 OAK 바닥턱 밑을 자립 구조로 받친다 (회전 뒤 = 월드 기준 45°)
_ofz  = zb_l - CRADLE_T
_ofy1 = yf + OAK_STANDOFF + OAK_PLATE_T + OAK_D + 1.0 + CRADLE_T
_orun = _ofy1 - yf
for _rx in RIB_XS_OAK:
    # 리브 윗면 = **기울어진 바닥턱 밑면 그 자체**. 같은 부품이라 틈을 둘 이유가 없다.
    #   [2026-09-17] 앞서 카메라 간섭을 피하려고 9mm 내렸다가 리브가 바닥에서 떨어져 아무것도
    #   못 받쳤다(브리지 0mm²). 카메라는 바닥턱 **위**에 있으므로 틈이 필요 없다.
    #   간섭의 진짜 원인은 리브 윗면을 **수평**으로 뒀던 것 — 바닥이 앞으로 기울어 내려가는데
    #   리브가 수평이면 끝에서 바닥 위로 솟아 카메라를 파고든다. 기울기를 그대로 따라가면 해결.
    _tipdrop = _orun * math.sin(R(OAK_TILT))
    rib = (cq.Workplane("YZ").workplane(offset=_rx - RIB_T / 2)
           .polyline([(yf - 0.1, _ofz - _orun * math.tan(R(RIB_ANG))),
                      (_ofy1, _ofz - _tipdrop), (yf - 0.1, _ofz)]).close()
           .extrude(RIB_T))
    cro = cro.union(rib)

cap = cap.union(wedge_o).union(cro).cut(oak_slot)

# ───────────────────────────── 4. 더미 + 검산 ─────────────────────────────
def cam_dummy(cam_w, cam_d, cam_h, plate_t, standoff, zb, tilt, conn=None):
    yc = yf + standoff + plate_t + cam_d / 2
    d = cq.Workplane("XY").box(cam_w, cam_d, cam_h).translate((0, yc, zb + cam_h / 2))
    if conn:
        cw, cd, cl, cx = conn
        d = d.union(cq.Workplane("XY").box(cw, cd, cl).translate((cx, yc, zb - cl / 2)))
    return d.rotate((0, yf + standoff, zb), (1, yf + standoff, zb), -tilt)

d455_dummy = cam_dummy(D455_W, D455_D, D455_H, D455_PLATE_T, D455_STANDOFF, d455_zb, D455_TILT,
                       conn=(10, 8, D455_USB_LEN, D455_USB_X))
oak_dummy = cam_dummy(OAK_W, OAK_D, OAK_H, OAK_PLATE_T, OAK_STANDOFF, oak_zb, OAK_TILT,
                      conn=(OAK_CONN_W, 16, OAK_CONN_LEN, OAK_CONN_X))
imu_dummy = cq.Workplane("XY").box(IMU_L, IMU_W, IMU_H).translate((imu_cx, pcy, pocket_floor + IMU_PAD_T + IMU_H / 2))

def bb(o):
    return o.val().BoundingBox()

oak_bb, d455_bb = bb(oak_dummy), bb(d455_dummy)
cap_top = cap_z0 + cap_h
top = max(oak_bb.zmax, cap_top, cap_z0 + bb(cro).zmax)
gap = oak_bb.zmin - d455_bb.zmax

# ───────────────────────────── 출력 ─────────────────────────────
parts = {"base": base, "imu_lid": lid, "cap": cap}
for i, sg in enumerate(segments):
    parts[f"seg{i + 1}"] = sg
for name, pp in parts.items():
    cq.exporters.export(pp, os.path.join(OUT, f"mast_{name}.stl"), tolerance=0.05)
    cq.exporters.export(pp, os.path.join(OUT, f"mast_{name}.step"))

asm = cq.Assembly()
asm.add(base, name="base")
asm.add(lid, name="imu_lid", loc=cq.Location(cq.Vector(pcx, pcy, BASE_T - LID_T)), color=cq.Color(0.6, 0.8, 0.6, 0.8))
z = seg_z0
for i, sg in enumerate(segments):
    asm.add(sg, name=f"seg{i + 1}", loc=cq.Location(cq.Vector(0, 0, z)))
    z += seg_len + DECK_T
asm.add(cap, name="cap", loc=cq.Location(cq.Vector(0, 0, z)))
asm.add(d455_dummy, name="D455_dummy", color=cq.Color(0.2, 0.5, 0.9, 0.5))
asm.add(oak_dummy, name="OAK_dummy", color=cq.Color(0.9, 0.5, 0.2, 0.5))
asm.add(imu_dummy, name="IMU_dummy", color=cq.Color(0.8, 0.2, 0.2, 0.6))
asm.save(os.path.join(OUT, "mast_assembly.step"))

# ── 통짜 융합 출력 (MAST_FUSED=1) ────────────────────────────────────────────
#   base + seg + cap 을 하나로 붙인다. 이음(플러그·칼라·M4 볼트)이 사라지는 대신
#   크레들 캔틸레버가 전부 한 부품에 실린다. 출력성은 측정으로 판단한다.
if os.environ.get("MAST_FUSED") == "1":
    print("ℹ️ MAST_FUSED: base+seg+cap 융합 중 (OCCT 불리언, 시간 걸림)")
    fused = base
    _z = seg_z0
    for _s in segments:
        fused = fused.union(_s.translate((0, 0, _z))); _z += seg_len + DECK_T
    fused = fused.union(cap.translate((0, 0, _z)))
    cq.exporters.export(fused, os.path.join(OUT, "mast_fused.stl"), tolerance=0.05)
    _bb = fused.val().BoundingBox()
    print(f"   융합본 {_bb.xlen:.1f}×{_bb.ylen:.1f}×{_bb.zlen:.1f}mm  {fused.val().Volume()/1000:.1f}㎤")



placed = {"base": base, "imu_lid": lid.translate((pcx, pcy, BASE_T - LID_T)), "cap": cap.translate((0, 0, z)),
          "D455": d455_dummy, "OAK": oak_dummy, "IMU": imu_dummy}
zz = seg_z0
for i, sg in enumerate(segments):
    placed[f"seg{i + 1}"] = sg.translate((0, 0, zz)); zz += seg_len + DECK_T
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

print(f"마디 {n_seg}개 × {seg_len:.1f} (갑판 {seg_z0:.0f}~{seg_top:.1f}) / 캡 {cap_z0:.1f}~{cap_top:.1f}")
print(f"OAK  바닥 {oak_bb.zmin:.1f} 상단 {oak_bb.zmax:.1f} (그랜드 포함) / D455 상단 {d455_bb.zmax:.1f} 바닥 {d455_bb.zmin:.1f} (USB 포함)")
print(f"세로 여유 OAK바닥−D455상단 = {gap:.1f} / 최고점 {top:.1f} / 한계 {LIMIT_H:.0f} → {'OK' if top <= LIMIT_H else '초과!'}")
print(f"IMU 포켓 {pocket_x:.0f}×{pocket_y:.0f}×{pocket_z:.0f} 중심({pcx:.0f},{pcy:.1f}) IMU중심x {imu_cx:.1f} 터널 +Y Ø{IMU_CH_D:.0f} @x{IMU_CH_X:.0f} / 뚜껑 {lid_x:.0f}×{lid_y:.0f} / 베이스 {BASE_W:.0f}×{BASE_L:.0f}×{BASE_T:.0f}")
print("간섭:", ", ".join(bad) if bad else "없음")

# 배치값 덤프 — mast_preview.py 가 읽는다 (수치 하드코딩 방지)
json.dump({
    "yf": yf, "BASE_T": BASE_T, "BASE_W": BASE_W, "BASE_L": BASE_L, "LID_T": LID_T, "DECK_T": DECK_T,
    "seg_z0": seg_z0, "seg_len": seg_len, "n_seg": n_seg, "cap_z0": cap_z0, "cap_h": cap_h,
    "pcx": pcx, "pcy": pcy, "lid_x": lid_x, "lid_y": lid_y,
    "imu": [imu_cx, pcy, pocket_floor + IMU_PAD_T + IMU_H / 2, IMU_L, IMU_W, IMU_H],
    "d455": [D455_W, D455_D, D455_H, d455_zb, D455_TILT, D455_STANDOFF, D455_PLATE_T, D455_USB_X, D455_USB_LEN],
    "oak": [OAK_W, OAK_D, OAK_H, oak_zb, OAK_TILT, OAK_STANDOFF, OAK_PLATE_T, OAK_CONN_X, OAK_CONN_LEN, OAK_CONN_W],
    "top": top, "limit": LIMIT_H, "gap": gap,
    "fit": {   # fit_test.py 가 읽는 형상 파라미터 (쿠폰과 본체가 같은 값을 쓰게)
        "CRADLE_T": CRADLE_T, "CRADLE_CLEAR": CRADLE_CLEAR,
        "LIP_D455": CRADLE_LIP_D455, "LIP_OAK": CRADLE_LIP_OAK,
        "NOTCH_W": CRADLE_NOTCH_W, "NOTCH_OAK": CRADLE_NOTCH_OAK,
        "D455": [D455_W, D455_D, D455_H, D455_PLATE_H, D455_PLATE_T, D455_HOLE_PITCH, D455_HOLE_D, D455_USB_X],
        "OAK": [OAK_W, OAK_D, OAK_H, OAK_PLATE_H, OAK_PLATE_T, OAK_HOLE_PITCH[0], OAK_HOLE_PITCH[1], OAK_HOLE_D, OAK_CONN_X],
        "pocket": [pocket_x, pocket_y, pocket_z, lid_x, lid_y, LID_T, LID_SCREW_D, IMU_CH_D, IMU_CH_X, IMU_PAD_T],
        "imu": [IMU_L, IMU_W, IMU_H],
        "plug": [TUBE_OD, TUBE_WALL, PLUG_LEN, PLUG_CLEAR, PLUG_HOLE, DECK_T, BOLT_D],
        "collar": [COLLAR_H, COLLAR_MARGIN],
        "LID_MARGIN": LID_MARGIN, "LID_FIT": LID_FIT,
        "base": [BASE_T, BASE_HOLE_D],
    },
}, open(os.path.join(OUT, "placement.json"), "w"), indent=1)
print("출력:", ", ".join(sorted(f for f in os.listdir(OUT) if f.startswith("mast_"))))

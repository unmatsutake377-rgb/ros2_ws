#!/usr/bin/env python3
"""허브·카메라 4S 급전 설명서 그림 4장 (matplotlib).
  docs/절차/그림/전원분기_1_전체결선.png   전체 결선 — 배터리 A → 분기 → 퓨즈 → 인젝터/5V 모듈 → OAK/허브
  docs/절차/그림/전원분기_2_분기점.png     분기점 만드는 법 3가지 + 납땜 요령
  docs/절차/그림/전원분기_3_플러그측정.png  플러그 극성·어댑터 플러그 재활용·측정 4점
  docs/절차/그림/전원분기_4_순서.png       작업 순서 10단계
재현: python3 tools/power_branch_manual.py  (설명서 HTML 은 tools/make_power_manual_html.py)
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle, Rectangle, Polygon

matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "절차", "그림")
RED, BLK, BLU, GRN, ORG, GRY = "#c0392b", "#222", "#2471a3", "#1e8449", "#b8860b", "#666"


def box(ax, x, y, w, h, text, fc="#f4f6fa", ec="#334", fs=9, bold=False, lw=1.2):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.06", fc=fc, ec=ec, lw=lw))
    ax.text(x, y, text, ha="center", va="center", fontsize=fs, weight="bold" if bold else "normal", linespacing=1.35)


def wire(ax, pts, c=RED, lw=2.2, ls="-", arrow=False):
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    ax.plot(xs, ys, color=c, lw=lw, ls=ls, solid_capstyle="round", solid_joinstyle="round", zorder=1)
    if arrow:
        ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle="-|>", mutation_scale=14, color=c, lw=0))


def lbl(ax, x, y, t, c="#333", fs=8, ha="center", va="center", bold=False, bg=True):
    ax.text(x, y, t, ha=ha, va=va, fontsize=fs, color=c, weight="bold" if bold else "normal",
            bbox=dict(fc="white", ec="none", pad=1.5) if bg else None, linespacing=1.3)


def fuse(ax, x, y, text):
    ax.add_patch(Rectangle((x - 0.55, y - 0.16), 1.1, 0.32, fc="#fff7d6", ec=ORG, lw=1.4))
    ax.text(x, y, text, ha="center", va="center", fontsize=8, color=ORG, weight="bold")


def setup(w, h, title):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(0, w); ax.set_ylim(0, h); ax.axis("off")
    ax.set_title(title, fontsize=13, loc="left", weight="bold")
    return fig, ax


# ───────────────────────── 그림 1 전체 결선
def fig1():
    fig, ax = setup(13, 8.2, "그림 1 — 전체 결선: 배터리 A 한 곳에서 두 갈래 (접촉기 「앞」이라 비상정지와 무관하게 살아 있다)")
    # 배터리·접촉기·ESC (기존)
    box(ax, 1.4, 6.3, 2.0, 1.1, "배터리 A\n4S 14.8 V (12.0~16.8)\nEC5", fc="#fdecea", fs=9, bold=True)
    box(ax, 5.2, 6.3, 2.1, 1.1, "MD-30a 접촉기\n5번(⊕ 유입) → 6번", fc="#eef3ff", fs=9)
    box(ax, 8.6, 6.3, 1.7, 0.9, "ESC 우 → T200", fc="#e6f4ea", fs=9)
    wire(ax, [(2.4, 6.5), (4.15, 6.5)], RED, 3.2); wire(ax, [(6.25, 6.5), (7.75, 6.5)], RED, 3.2, arrow=True)
    wire(ax, [(2.4, 6.1), (3.0, 6.1), (3.0, 7.6), (8.6, 7.6), (8.6, 6.75)], BLK, 3.2)
    lbl(ax, 5.2, 7.75, "배터리 A ⊖ → ESC ⊖ 직결 (접촉기 안 지남)  10 AWG", BLK, 8)
    lbl(ax, 3.3, 6.75, "10 AWG ⊕ 본선", RED, 8)
    # 분기점
    ax.add_patch(Circle((3.4, 6.5), 0.13, fc=ORG, ec="k", zorder=3)); ax.add_patch(Circle((3.0, 7.0), 0.13, fc=ORG, ec="k", zorder=3))
    lbl(ax, 3.4, 5.95, "분기점 ⊕\n(5번보다 배터리 쪽)", ORG, 8, bold=True)
    lbl(ax, 2.1, 7.0, "분기점 ⊖", ORG, 8, bold=True)
    lbl(ax, 6.0, 5.55, "※ 승압 컨버터·무선 릴레이 입력도 이미 이 두 점에서 납땜 분기(08-25). 같은 자리, 같은 방법.", GRY, 8)
    # 두 갈래
    wire(ax, [(3.4, 6.5), (3.4, 4.9), (2.2, 4.9), (2.2, 3.4)], RED, 2.0); wire(ax, [(3.4, 4.9), (6.2, 4.9), (6.2, 3.4)], RED, 2.0)
    lbl(ax, 4.6, 5.05, "18 AWG 분기선 (합계 약 1.3 A)", RED, 8)
    fuse(ax, 2.2, 4.3, "BF303 + 3 A"); fuse(ax, 6.2, 4.3, "BF303 + 2 A")
    lbl(ax, 2.2, 4.62, "허브", ORG, 8); lbl(ax, 6.2, 4.62, "인젝터", ORG, 8)
    # ⊖ 갈래
    # ⊖ 분기선: 분기점 ⊖ → 모듈 IN⊖ (왼쪽) / 인젝터 DC IN ⊖ (아래). 퓨즈 없음
    wire(ax, [(3.0, 7.0), (0.5, 7.0), (0.5, 2.85), (1.0, 2.85)], BLK, 2.0, arrow=True)
    wire(ax, [(0.5, 2.85), (0.5, 2.05), (5.65, 2.05), (5.65, 2.35)], BLK, 2.0, arrow=True)
    lbl(ax, 0.5, 4.5, "⊖ 분기선 (검정)\n퓨즈 없음\n두 장치의 ⊖ 로", BLK, 8)
    lbl(ax, 1.35, 3.08, "IN⊖", BLK, 7.5); lbl(ax, 5.95, 2.2, "DC IN ⊖", BLK, 7.5)
    lbl(ax, 3.2, 2.2, "⊖ 공통선 (인젝터용)", BLK, 7.5)
    # 5V 모듈 → 허브
    box(ax, 2.2, 2.85, 2.4, 1.0, "SZWENGAO WG-1224S0505\nIN 8~36 V → OUT 5 V 5 A\nIP68 · 비절연", fc="#eafaf1", fs=8.5, bold=True)
    wire(ax, [(2.2, 2.35), (2.2, 1.75)], GRN, 2.0, arrow=True)
    lbl(ax, 4.75, 1.3, "5 V 출력선(OUT⊕·OUT⊖) → DC 플러그\n(동봉 어댑터 플러그 재활용, 그림 3)", GRN, 8)
    box(ax, 2.2, 1.25, 2.6, 0.85, "ipTIME UH308 허브  DC 5 V\n→ 노트북 USB-A ②", fc="#eef3ff", fs=8.5, bold=True)
    box(ax, 2.2, 0.35, 3.6, 0.5, "LiDAR A3 · IMU · GPS · D455 (뎁스 OFF)", fc="#ffffff", fs=8)
    wire(ax, [(2.2, 0.82), (2.2, 0.6)], BLU, 1.5)
    # 인젝터 → OAK
    box(ax, 6.2, 2.85, 2.6, 1.0, "PoE 인젝터 PSE5502G\nDC IN 12~48 V (5.5×2.1 중심⊕)\n→ 48 V 802.3af/at 30 W", fc="#eafaf1", fs=8.5, bold=True)
    lbl(ax, 7.2, 3.65, "DC IN ⊕ (2 A 퓨즈 뒤)  입력 0.4~0.8 A", GRY, 7.5)
    wire(ax, [(7.5, 2.85), (9.3, 2.85)], BLU, 2.0, arrow=True); lbl(ax, 8.4, 3.05, "PoE OUT  Cat5e 2 m(동봉)", BLU, 8)
    box(ax, 10.6, 2.85, 2.3, 1.0, "OAK-1 PoE FF\n802.3af Class 3 · ≤7.5 W\nRJ45 · 마스트", fc="#e6f4ea", fs=8.5, bold=True)
    wire(ax, [(6.2, 2.35), (6.2, 1.5), (9.3, 1.5)], BLU, 2.0, arrow=True); lbl(ax, 7.6, 1.7, "DATA/LAN  Cat5e", BLU, 8)
    box(ax, 10.6, 1.5, 2.3, 0.8, "NM-UCL02 기가비트\n→ 노트북 USB-C", fc="#eef3ff", fs=8.5)
    # 전압 표
    lbl(ax, 10.9, 6.9, "각 점의 전압 (정상)\n분기점 12.0~16.8 V\n퓨즈 뒤 = 분기점과 같음\n모듈 OUT 4.9~5.1 V\n인젝터 PoE 48 V (측정 불필요)",
        "#333", 8.5, ha="left", bold=False)
    ax.add_patch(Rectangle((10.75, 6.2), 2.2, 1.45, fc="none", ec=GRY, lw=1))
    lbl(ax, 0.3, 0.1, "색: 빨강 ⊕ · 검정 ⊖ · 초록 5 V · 파랑 이더넷/USB.  노트북 USB-A ① 은 Mega 직결(수신기 전원) — 허브에 꽂지 않는다.", GRY, 8, ha="left")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "전원분기_1_전체결선.png"), dpi=150); plt.close(fig)


# ───────────────────────── 그림 2 분기점
def fig2():
    fig, ax = setup(13, 7.6, "그림 2 — 분기점 만드는 법: ⊕ 는 둘 중 하나, ⊖ 는 기존 납땜점에 합류")
    # (가) 본선 납땜
    lbl(ax, 2.3, 7.0, "(가) ⊕ 본선 납땜 — 승압 컨버터와 같은 방법 (08-25 검증)", "#333", 10, bold=True, bg=False)
    wire(ax, [(0.4, 5.9), (4.4, 5.9)], RED, 5)
    lbl(ax, 0.9, 6.25, "배터리 EC5 쪽", GRY, 8); lbl(ax, 3.9, 6.25, "접촉기 5번 쪽", GRY, 8)
    ax.add_patch(Rectangle((1.9, 5.75), 0.9, 0.3, fc="#f5cba7", ec=ORG, lw=1.2)); lbl(ax, 2.35, 5.9, "피복 8 mm 벗김", ORG, 7, bg=False)
    wire(ax, [(2.35, 5.75), (2.35, 5.0), (2.35, 4.6)], RED, 2.0, arrow=True); lbl(ax, 3.1, 5.2, "18 AWG 분기선\n본선에 감아 납땜", RED, 8)
    ax.add_patch(Rectangle((1.7, 5.6), 1.3, 0.6, fc="none", ec=GRY, lw=1, ls="--")); lbl(ax, 2.35, 5.45, "수축튜브", GRY, 7)
    lbl(ax, 2.3, 4.2, "인두 60 W 이상 (10 AWG 가 열을 뺏는다)\n1) 본선 중간 피복만 벗김(자르지 않음)\n2) 분기선 끝을 본선에 3바퀴 감음\n3) 납이 본선에 스며들 때까지 가열\n4) 수축튜브 두 겹 또는 자기융착 테이프", "#333", 8, ha="center")
    # (나) 링단자
    lbl(ax, 7.3, 7.0, "(나) 접촉기 5번 둘째 나사에 링단자 — 납땜 없음", "#333", 10, bold=True, bg=False)
    box(ax, 7.3, 5.6, 2.6, 1.6, "", fc="#eef3ff")
    lbl(ax, 7.3, 6.2, "MD-30a  5번 단자", "#333", 9, bold=True, bg=False)
    for i, (x, t) in enumerate([(6.7, "나사 ①\n본선 10 AWG"), (7.9, "나사 ②\n분기선 링단자")]):
        ax.add_patch(Circle((x, 5.45), 0.22, fc="#ddd", ec="k")); ax.add_patch(Circle((x, 5.45), 0.07, fc="k"))
        lbl(ax, x, 4.95, t, "#333", 7.5)
    wire(ax, [(7.9, 5.45), (7.9, 4.3)], RED, 2.0, arrow=True)
    lbl(ax, 7.3, 4.0, "결선표: \"위·아래 나사는 같은 단자(한 점에 나사 2개)\"\n5번 = 배터리 A ⊕ 유입 = 접촉기 「앞」 → 비상정지 걸려도 살아 있음\nM4 링단자 18 AWG 압착 → 둘째 나사. 토크 확인, 흔들어 봄", "#333", 8)
    # (다) ⊖
    lbl(ax, 11.3, 7.0, "(다) ⊖ 분기", "#333", 10, bold=True, bg=False)
    wire(ax, [(9.9, 5.9), (12.7, 5.9)], BLK, 5); lbl(ax, 10.3, 6.25, "배터리 A ⊖", GRY, 8); lbl(ax, 12.3, 6.25, "ESC ⊖", GRY, 8)
    ax.add_patch(Circle((11.3, 5.9), 0.16, fc=ORG, ec="k", zorder=3))
    wire(ax, [(11.3, 5.9), (11.3, 4.6)], BLK, 2.0, arrow=True)
    lbl(ax, 11.3, 4.1, "기존 \"컨버터 IN⊖ · 릴레이 IN-\" 납땜점\n(⊖ 는 \"어디든 무방 — 안 끊기는 줄\")\n거기에 분기선 하나 더 납땜.\n퓨즈 없음.", "#333", 8)
    # 하단 공통
    box(ax, 6.5, 2.3, 12.4, 1.5,
        "공통 규칙\n• 작업 내내 배터리 A·B 모두 분리.  • ⊕ 선 빨강, ⊖ 선 검정 (옛 사고: 네 단자가 전부 빨간 선).\n"
        "• 분기선 끝은 바로 퓨즈홀더 리드에 — 홀더 통에 유성펜 \"3A 허브\" / \"2A 인젝터\".\n"
        "• 다 붙이고 배터리 꽂기 전: 테스터 저항 모드로 ⊕–⊖ 사이가 OL(무한대)인지. 0 Ω 이면 어딘가 붙었다.\n"
        "• EC5 암 피그테일은 커넥터가 어디에 꽂히는지 정해지지 않았다 — 성별이 맞을 때만 쓰고, 아니면 (가)/(나)로.",
        fc="#fbfbfb", ec=GRY, fs=8.5)
    box(ax, 6.5, 0.6, 12.4, 0.8, "전류 근거: 인젝터 0.4~0.8 A + 5 V 모듈 0.6 A + 기존 승압·릴레이 0.7 A ≈ 2 A → 18 AWG(정격 10 A 급) 충분. 10 AWG 는 분기선에 과하다.", fc="#fff7d6", ec=ORG, fs=8.5)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "전원분기_2_분기점.png"), dpi=150); plt.close(fig)


# ───────────────────────── 그림 3 플러그·측정
def fig3():
    fig, ax = setup(13, 7.6, "그림 3 — 플러그 극성·동봉 어댑터 플러그 재활용·측정 4점")
    # 플러그 단면
    lbl(ax, 2.4, 7.0, "DC 배럴 플러그 5.5×2.1 (인젝터 DC IN · 허브 DC IN 공통 규격)", "#333", 10, bold=True, bg=False)
    ax.add_patch(Rectangle((0.8, 5.2), 1.6, 0.9, fc="#ddd", ec="k")); ax.add_patch(Rectangle((2.4, 5.45), 1.3, 0.4, fc="#bbb", ec="k"))
    ax.add_patch(Rectangle((2.4, 5.6), 1.3, 0.1, fc="#e74c3c", ec="none"))
    lbl(ax, 4.3, 5.85, "중심 핀 = ⊕ (빨강)", RED, 9, bold=True); lbl(ax, 4.3, 5.4, "바깥 통 = ⊖ (검정)", BLK, 9, bold=True)
    lbl(ax, 2.4, 4.75, "외경 5.5 / 내경 2.1 mm.  라벨이 권위 — 기기 뒷면 ⊕—(•—⊖ 기호 확인.\n인젝터는 역극성 보호 명시 없음 → 꽂기 전 플러그 중심이 ⊕ 인지 테스터로.", "#333", 8)
    # 어댑터 재활용
    lbl(ax, 9.3, 7.0, "허브 동봉 어댑터 플러그 재활용 — 잭 치수·극성 고민 끝", "#333", 10, bold=True, bg=False)
    box(ax, 7.3, 5.6, 1.6, 0.9, "UH308 동봉\n5 V 어댑터", fc="#eee", fs=8.5)
    wire(ax, [(8.1, 5.6), (10.4, 5.6)], BLK, 3)
    ax.add_patch(Polygon([[9.1, 5.95], [9.4, 5.25], [9.5, 5.3], [9.2, 6.0]], fc=RED, ec="none")); lbl(ax, 9.25, 6.25, "여기서 자름", RED, 8, bold=True)
    ax.add_patch(Rectangle((10.4, 5.45), 0.9, 0.3, fc="#bbb", ec="k")); lbl(ax, 11.9, 5.6, "플러그 쪽 선 → 5 V 모듈 OUT", GRN, 8.5)
    lbl(ax, 9.3, 4.75, "잘린 선 두 가닥 중 어느 쪽이 중심 핀인지 테스터 도통으로 확인(흰 줄무늬 = ⊕ 인 경우 많지만 측정이 권위)\n→ 모듈 OUT⊕(노랑/빨강) 에 ⊕, OUT⊖(검정) 에 ⊖ 납땜 + 수축튜브.  어댑터 본체는 벤치용으로 보관.", "#333", 8)
    # 측정 4점
    lbl(ax, 6.5, 3.75, "측정 4점 — 순서대로. 하나라도 틀리면 다음 단계로 가지 않는다", "#333", 10, bold=True, bg=False)
    rows = [("①", "분기점 ⊕–⊖ (퓨즈 뒤)", "배터리 연결 직후, 부하 없이", "12.0~16.8 V", "0 V → 퓨즈·분기 납땜 확인. 음수 → ⊕⊖ 반대!"),
            ("②", "5 V 모듈 OUT⊕–OUT⊖", "허브 꽂기 「전」", "4.9~5.1 V", "0 V → 입력 극성 반대(모듈 사망) 또는 3 A 퓨즈. 5.3 V↑ → 모듈 불량, 허브 꽂지 말 것"),
            ("③", "DC 플러그 중심–바깥", "인젝터·허브에 꽂기 「전」", "중심 ⊕", "반대면 선 바꿔 납땜. 인젝터에 반대로 꽂으면 끝"),
            ("④", "LED", "꽂은 뒤", "모듈·허브·인젝터 LED, 10~20 s 뒤 OAK LED", "안 켜지면 §안 될 때 표")]
    y = 3.2
    for a, b, c, d, e in rows:
        ax.add_patch(Rectangle((0.5, y - 0.3), 12.2, 0.62, fc="#fbfbfb", ec=GRY, lw=0.8))
        lbl(ax, 0.8, y, a, ORG, 10, bold=True, bg=False); lbl(ax, 2.2, y, b, "#333", 8.5, bg=False); lbl(ax, 4.6, y, c, GRY, 8, bg=False)
        lbl(ax, 6.9, y, d, GRN, 8.5, bold=True, bg=False); lbl(ax, 10.3, y, e, "#333", 7.5, bg=False)
        y -= 0.72
    lbl(ax, 0.6, 0.25, "테스터: DC V 20 V 레인지. ① ② 는 검정 리드를 ⊖ 에 고정하고 빨강 리드로 찍는다. 저항(도통) 측정은 반드시 배터리 분리 상태.", GRY, 8, ha="left")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "전원분기_3_플러그측정.png"), dpi=150); plt.close(fig)


# ───────────────────────── 그림 4 순서
def fig4():
    fig, ax = setup(13, 8.0, "그림 4 — 작업 순서 10단계 (약 90분, 배터리는 8단계 전까지 분리)")
    steps = [
        ("1", "준비", "인두 60 W+, 테스터, 수축튜브, 18 AWG 적·흑, M4 링단자, 유성펜, 타이", "#fbfbfb"),
        ("2", "퓨즈홀더 2개", "BF303 에 2 A / 3 A 끼움. 통에 \"2A 인젝터\" \"3A 허브\" 표기", "#fff7d6"),
        ("3", "배터리 A·B 분리", "EC5 둘 다 뽑고 테스터로 본선 0 V 확인", "#fdecea"),
        ("4", "⊕ 분기", "그림 2 (가) 본선 납땜 또는 (나) 5번 둘째 나사. 분기선 → 홀더 2개", "#fbfbfb"),
        ("5", "⊖ 분기", "기존 컨버터 IN⊖ 납땜점에 분기선 2가닥 추가", "#fbfbfb"),
        ("6", "5 V 모듈", "홀더(3A) → 모듈 IN⊕, ⊖ → IN⊖. 라벨로 Input 확인. OUT 은 아직 아무 데도 안 붙임", "#eafaf1"),
        ("7", "플러그 2개", "허브: 동봉 어댑터 플러그 잘라 모듈 OUT 에. 인젝터: 수 플러그 또는 동봉 리드선에 홀더(2A)/⊖", "#eafaf1"),
        ("8", "무전원 도통", "⊕–⊖ 사이 OL 확인. 플러그 중심=⊕ 도통 확인(그림 3 ③)", "#fdecea"),
        ("9", "배터리 A 연결 → 측정 ①②", "분기점 12~16.8 V, 모듈 OUT 4.9~5.1 V. 그 다음에야 허브·인젝터에 꽂음", "#e6f4ea"),
        ("10", "부하 시험", "허브: lsusb 에 4종, LiDAR 회전음. 카메라: 인젝터 LED → OAK LED → ping 169.254.1.222. 스러스터 돌리며 5분 dmesg", "#e6f4ea"),
    ]
    y = 7.1
    for n, t, d, fc in steps:
        ax.add_patch(FancyBboxPatch((0.4, y - 0.27), 12.2, 0.56, boxstyle="round,pad=0.01,rounding_size=0.05", fc=fc, ec=GRY, lw=0.9))
        lbl(ax, 0.75, y, n, ORG, 11, bold=True, bg=False); lbl(ax, 2.3, y, t, "#333", 9.5, bold=True, bg=False)
        lbl(ax, 3.8, y, d, "#333", 8.5, ha="left", bg=False)
        y -= 0.64
    box(ax, 6.5, 0.45, 12.2, 0.6, "실패 시 교체 가능한 것: 퓨즈(2 A→3 A, 10개), 분기선, 플러그.  교체 불가능한 사고 = 극성 반대로 꽂기 → 8·9 단계 측정이 막는다.", fc="#fff7d6", ec=ORG, fs=8.5)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "전원분기_4_순서.png"), dpi=150); plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    fig1(); fig2(); fig3(); fig4()
    print("wrote", OUT)

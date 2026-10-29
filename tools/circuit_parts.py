#!/usr/bin/env python3
"""SSF 1번배 부분별 회로도 6장 (보고서 그림용).

    python3 tools/circuit_parts.py            → docs/전달용/배치도/회로_부분/회로_{1..6}_*.png

출처(사실은 전부 여기서 가져왔다 — 그림을 고치기 전에 출처부터 고칠 것):
  핀 배정        arduino/ssf_boat/ssf_boat.ino (PIN_*), 08-25·09-03 실물 확인
  접촉기·비상정지  docs/문제와작업/좌측스러스터_접촉기_20260819.md §2·§4·§13·§20
  허브·카메라     docs/절차/허브_카메라_전원_설명서.md (10-28 1번배 실측)
  3상 배선       docs/문제와작업/스러스터_배선_1번배_20261026.md 최종표
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle

matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
matplotlib.rcParams["axes.unicode_minus"] = False

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "전달용", "배치도", "회로_부분")
os.makedirs(OUT, exist_ok=True)

C_PWR = "#c0392b"   # 배터리 ⊕ (고전류)
C_GND = "#222222"   # ⊖ / GND
C_SIG = "#1f5fa8"   # 제어 신호
C_5V = "#d35400"    # 5V 보조 전원
C_24 = "#8e44ad"    # 24V 코일 회로
C_USB = "#138d75"   # USB 데이터
C_LAN = "#6c3483"   # 이더넷 / PoE
C_NOTE = "#7d6608"
FOOT = "S.S.F 1번배 · 2026-10-29 · 실물 확인 기준"


def new(title, sub, w=16, h=9.4):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(0, 100); ax.set_ylim(0, 60); ax.axis("off")
    fig.subplots_adjust(left=0.02, right=0.98, top=0.90, bottom=0.04)
    fig.text(0.02, 0.955, title, fontsize=19, fontweight="bold", ha="left", va="center")
    fig.text(0.02, 0.918, sub, fontsize=11.5, color="#444", ha="left", va="center")
    fig.text(0.98, 0.015, FOOT, fontsize=8.5, color="#888", ha="right")
    return fig, ax


def box(ax, x, y, w, h, title, body="", fc="#ffffff", ec="#333", tfs=12, bfs=9.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.25,rounding_size=0.8",
                                fc=fc, ec=ec, lw=1.4, zorder=3))
    if body:
        ax.text(x + w / 2, y + h - 1.6, title, ha="center", va="top", fontsize=tfs, fontweight="bold", zorder=4)
        ax.text(x + w / 2, y + h - 4.2, body, ha="center", va="top", fontsize=bfs, color="#333",
                linespacing=1.45, zorder=4)
    else:
        ax.text(x + w / 2, y + h / 2, title, ha="center", va="center", fontsize=tfs, fontweight="bold", zorder=4)


def wire(ax, pts, c, lw=2.6, ls="-", z=2):
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color=c, lw=lw, ls=ls,
            solid_capstyle="round", solid_joinstyle="round", zorder=z)


def dot(ax, x, y, c):
    ax.add_patch(Circle((x, y), 0.45, fc=c, ec=c, zorder=5))


def tag(ax, x, y, t, c="#222", fs=9.5, ha="center", va="center", bold=False, bg=True):
    kw = dict(boxstyle="round,pad=0.18", fc="white", ec="none", alpha=0.95) if bg else None
    ax.text(x, y, t, ha=ha, va=va, fontsize=fs, color=c, fontweight="bold" if bold else "normal",
            bbox=kw, zorder=6, linespacing=1.35)


def note(ax, x, y, t, c=C_NOTE, fs=9.5, ha="left"):
    ax.text(x, y, t, ha=ha, va="top", fontsize=fs, color=c, linespacing=1.5, zorder=6)


def legend(ax, items, x=1, y=2.2):
    cx = x
    for name, c, ls in items:
        wire(ax, [(cx, y), (cx + 3.2, y)], c, lw=2.6, ls=ls)
        ax.text(cx + 3.8, y, name, va="center", fontsize=9.5, color="#333")
        cx += 4.6 + len(name) * 0.95


def switch(ax, x1, x2, y, c, closed=True):
    """접점 기호: 두 단자 사이 기울어진 막대."""
    dot(ax, x1, y, c); dot(ax, x2, y, c)
    if closed:
        wire(ax, [(x1, y), (x2, y)], c, lw=2.6)
    else:
        wire(ax, [(x1, y), (x2 - 0.6, y + 2.0)], c, lw=2.6)


def save(fig, name):
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=200, facecolor="white")
    plt.close(fig)
    print("wrote", p)


# ═══════════════════════════════════════════════ ① 추진 전력
def d1():
    fig, ax = new("① 추진 전력 — 배터리 → 접촉기 → ESC → T200",
                  "접촉기는 각 배터리의 ⊕ 한 가닥만 끊는다. ⊖ 는 ESC 로 직결. 1번배: 측당 ESC·T200 1기 (2번배는 측당 2기 팬아웃)")
    box(ax, 1, 36, 15, 15, "배터리 A", "4S LiPo 14.8 V\n(만충 16.8 V)\nDXF 8400 mAh 120C", fc="#fdecea")
    box(ax, 1, 7, 15, 15, "배터리 B", "4S LiPo 14.8 V\n(만충 16.8 V)\nDXF 8400 mAh 120C", fc="#fdecea")
    # 접촉기
    ax.add_patch(FancyBboxPatch((33, 5), 22, 48, boxstyle="round,pad=0.25,rounding_size=0.8",
                                fc="#eef4fb", ec="#333", lw=1.4, zorder=1))
    ax.text(44, 51.3, "접촉기 MD-30a", ha="center", va="top", fontsize=12.5, fontweight="bold")
    ax.text(44, 48.6, "LS Metasol · DC 2극", ha="center", va="top", fontsize=9.5, color="#333")
    switch(ax, 37, 51, 43.5, C_PWR); tag(ax, 37, 40.8, "5", fs=12, bold=True); tag(ax, 51, 40.8, "6", fs=12, bold=True)
    switch(ax, 37, 51, 14.5, C_PWR); tag(ax, 37, 11.8, "2", fs=12, bold=True); tag(ax, 51, 11.8, "1", fs=12, bold=True)
    tag(ax, 44, 28, "코일 A1─A2 (24 V)\n하나가 두 극을 함께 연다\n→ ② 비상정지 회로", c=C_24, fs=9.5)
    wire(ax, [(44, 35.5), (44, 31.5)], C_24, lw=1.5, ls=":"); wire(ax, [(44, 24.5), (44, 16.5)], C_24, lw=1.5, ls=":")
    tag(ax, 44, 37.6, "6번 \"⊖\" 각인은 무시 — ⊕ 가 지나가는 자리", c="#a93226", fs=8.5)
    # ESC · T200
    box(ax, 64, 36, 14, 15, "ESC 우", "Blue Robotics\nBESC30-R3\n신호: Mega 핀 11", fc="#eafaf1")
    box(ax, 64, 7, 14, 15, "ESC 좌", "Blue Robotics\nBESC30-R3\n신호: Mega 핀 12", fc="#eafaf1")
    box(ax, 85, 36, 14, 15, "T200 우", "Blue Robotics\n3상 색 동일\n(흰·초·파)", fc="#eafaf1")
    box(ax, 85, 7, 14, 15, "T200 좌", "Blue Robotics\n3상 초↔파 교차\n(⑥ 참고)", fc="#eafaf1")
    # ⊕ 배선
    wire(ax, [(16.5, 47), (37, 47), (37, 43.5)], C_PWR)
    wire(ax, [(51, 43.5), (51, 47), (63.7, 47)], C_PWR)
    wire(ax, [(16.5, 18), (37, 18), (37, 14.5)], C_PWR)
    wire(ax, [(51, 14.5), (51, 18), (63.7, 18)], C_PWR)
    tag(ax, 22, 48.6, "A ⊕  10 AWG", c=C_PWR, bold=True); tag(ax, 26, 19.6, "B ⊕  10 AWG", c=C_PWR, bold=True)
    # 분기점
    dot(ax, 29, 47, C_PWR); wire(ax, [(29, 47), (29, 42)], C_PWR, lw=1.8)
    tag(ax, 24.5, 38.2, "분기(접촉기 앞)\n→ ② 코일 컨버터\n→ ④ 허브 5V\n→ ⑤ PoE 인젝터", c=C_PWR, fs=9.5, bold=True)
    # ⊖ 직결 — 배터리 위로 나가 접촉기 위로 돌아간다(접촉기를 지나지 않음을 그림으로도)
    wire(ax, [(8.5, 51.4), (8.5, 56.5), (60, 56.5), (60, 39), (63.7, 39)], C_GND)
    wire(ax, [(16.5, 11), (24, 11), (24, 3.8), (60, 3.8), (60, 10), (63.7, 10)], C_GND)
    tag(ax, 45, 58.2, "A ⊖ → ESC 우 ⊖ 직결 (접촉기를 지나지 않음)", c=C_GND, fs=9)
    tag(ax, 32, 2.2, "B ⊖ → ESC 좌 ⊖ 직결", c=C_GND, fs=9)
    # ESC → T200 3상
    for i, c in enumerate(["#999", "#27ae60", "#2e86c1"]):
        wire(ax, [(78.2, 41 + 2.5 * i), (84.8, 41 + 2.5 * i)], c, lw=2.2)
        wire(ax, [(78.2, 12 + 2.5 * i), (84.8, 12 + 2.5 * i)], c, lw=2.2)
    tag(ax, 81.5, 51.8, "3상", fs=9); tag(ax, 81.5, 22.8, "3상", fs=9)
    note(ax, 64, 31.5, "● 배터리 연결 순서: B(1·2번) 먼저 → A(5·6번) 나중\n   A 가 코일 컨버터를 먹여 꽂는 순간 접촉기가 닫힌다\n● ESC 는 전원이 들어오는 순간 1500 µs 를 봐야 시동",
         c="#333", fs=9)
    legend(ax, [("배터리 ⊕", C_PWR, "-"), ("⊖", C_GND, "-"), ("코일 제어(②)", C_24, ":")], x=64, y=1.4)
    save(fig, "회로_1_추진전력.png")


# ═══════════════════════════════════════════════ ② 비상정지
def d2():
    fig, ax = new("② 비상정지 — 코일 회로(직렬) · 상태 감지(보조접점)",
                  "버튼과 무선 릴레이는 굵은 전류가 아니라 접촉기 코일만 끊는다. 둘 중 하나만 열려도 두 극이 함께 열린다")
    box(ax, 1, 38, 15, 14, "배터리 A ⊕ / ⊖", "접촉기 5번보다\n배터리 쪽에서 분기", fc="#fdecea")
    box(ax, 23, 38, 17, 14, "승압 컨버터", "SZH-BPM001\n입력 9~20 V\n→ 24 V 고정 3 A", fc="#f4ecf7")
    box(ax, 47, 38, 16, 14, "접촉기 코일", "MD-30a A1 ─ A2\nDC 24 V · 실측 48 Ω", fc="#eef4fb")
    box(ax, 70, 38, 15, 14, "무선 릴레이", "SRD-12VDC-SL-C\n(리모컨 수신)\nCOM ─ NC 사용", fc="#eafaf1")
    box(ax, 70, 15, 15, 13, "비상정지 버튼", "한영넉스\nNC (누르면 열림)", fc="#eafaf1")
    # 입력
    wire(ax, [(16.5, 47), (22.7, 47)], C_PWR); tag(ax, 19.6, 48.6, "IN ⊕", c=C_PWR, fs=9, bold=True)
    wire(ax, [(16.5, 42), (22.7, 42)], C_GND); tag(ax, 19.6, 40.4, "IN ⊖", c=C_GND, fs=9, bold=True)
    # 직렬 루프 (24V)
    wire(ax, [(40.3, 47), (46.7, 47)], C_24); tag(ax, 43.5, 48.6, "OUT⊕→A1", c=C_24, fs=9, bold=True)
    wire(ax, [(63.3, 47), (69.7, 47)], C_24); tag(ax, 66.5, 48.6, "A2→COM", c=C_24, fs=9, bold=True)
    wire(ax, [(77.5, 37.7), (77.5, 28.3)], C_24); tag(ax, 82, 33, "NC → 버튼 1", c=C_24, fs=9, bold=True)
    wire(ax, [(77.5, 14.7), (77.5, 11), (36, 11), (36, 37.7)], C_24)
    tag(ax, 54, 9.4, "버튼 2 → 컨버터 OUT⊖   (한 줄 직렬: 버튼·무선 중 하나만 열려도 코일이 죽는다)", c=C_24, fs=9.5, bold=True)
    # 릴레이 모듈 전원
    wire(ax, [(8.5, 52.3), (8.5, 56), (88, 56), (88, 47), (85.3, 47)], C_PWR, lw=1.6)
    wire(ax, [(10.5, 52.3), (10.5, 54.5), (86.5, 54.5), (86.5, 42), (85.3, 42)], C_GND, lw=1.6)
    tag(ax, 60, 57.2, "릴레이 모듈 전원 (배터리 A 직결)  ⚠ 모듈 정격 12 V 인데 배터리 직결 = 만충 16.8 V(140%)", c=C_NOTE, fs=9)
    # 상태 감지
    ax.add_patch(FancyBboxPatch((1, 15), 30, 16, boxstyle="round,pad=0.25,rounding_size=0.8",
                                fc="#f8f9f9", ec="#888", lw=1.2, ls="--", zorder=1))
    ax.text(16, 30, "상태 감지 (표시 전용)", ha="center", va="top", fontsize=11, fontweight="bold")
    switch(ax, 4, 11, 22, C_SIG, closed=True)
    tag(ax, 4, 19.6, "21", fs=10, bold=True); tag(ax, 11, 19.6, "22", fs=10, bold=True)
    tag(ax, 7.5, 25.6, "보조 b접점(NC)", fs=9)
    wire(ax, [(11, 22), (19, 22)], C_SIG); tag(ax, 24.5, 22, "Mega 핀 38\nINPUT_PULLUP", c=C_SIG, fs=9, bold=True)
    wire(ax, [(4, 22), (4, 17.5), (19, 17.5)], C_GND); tag(ax, 22, 17.5, "Mega GND", c=C_GND, fs=9, bold=True)
    note(ax, 38.5, 32, "정상: 접촉기 붙음 → 21-22 열림 → 핀 38 HIGH\n         → estop 0 → LED 모드색\n"
                       "정지: 접촉기 떨어짐 → 21-22 닫힘 → 핀 38 LOW\n         → estop 1 → LED 빨강",
         c=C_SIG, fs=9.5)
    note(ax, 38.5, 22.5, "● 아두이노는 차단 경로에 없다\n   제어기가 죽어도 비상정지는 동작\n"
                         "● 두 선은 꼬아서 배선 · 1 kΩ 외부 풀업은\n   권장(현재 내부 풀업)",
         c="#333", fs=9)
    note(ax, 1, 7.4, "● 컨버터 입력을 접촉기 「뒤」(6번)에서 따면 코일이 접촉기를, 접촉기가 코일을 기다려 영원히 안 붙는다 (08-19 실제 사고)",
         c="#a93226", fs=9.5)
    legend(ax, [("배터리 ⊕", C_PWR, "-"), ("⊖ / GND", C_GND, "-"), ("24 V 코일 루프", C_24, "-"), ("상태 신호", C_SIG, "-")], x=1, y=2.0)
    save(fig, "회로_2_비상정지.png")


# ═══════════════════════════════════════════════ ③ 제어 신호
def d3():
    fig, ax = new("③ 제어 신호 — 노트북 · Arduino Mega · 조종기 수신기 · ESC · 표시등",
                  "핀 배정 출처: arduino/ssf_boat/ssf_boat.ino. Mega 는 노트북 USB 직결(허브 금지), 수신기·LED 는 Mega 5V 로 동작")
    box(ax, 1, 40, 15, 13, "배 노트북", "Lenovo IdeaPad Slim 3\nROS2 Humble · ssf_bridge\n115200 bps", fc="#eaf2fb")
    ax.add_patch(FancyBboxPatch((28, 8), 22, 45, boxstyle="round,pad=0.25,rounding_size=0.8",
                                fc="#eaf2fb", ec="#333", lw=1.4, zorder=1))
    ax.text(39, 51.5, "Arduino Mega 2560 R3", ha="center", va="top", fontsize=12.5, fontweight="bold")
    pins = [("5V · GND", 42.5), ("핀 2  RC 스로틀", 38), ("핀 3  RC 조향", 34.5), ("핀 19  RC 모드", 31),
            ("핀 12  ESC 좌", 25.5), ("핀 11  ESC 우", 22), ("핀 16  LED DIN", 17), ("핀 38  비상정지 감지", 12.5)]
    for t, y in pins:
        ax.text(48.8, y, t, ha="right", va="center", fontsize=9.5, color="#1b2631", zorder=4)
        dot(ax, 50, y, C_SIG if "핀" in t else (C_USB if t == "USB" else C_5V))
    ax.text(29.5, 31.5, "ID 핀 34·36\n미배선 = A배\n(둘 다 GND = FAULT\n→ 무조건 중립)", fontsize=8.5, color="#555", va="center")
    # 노트북 ↔ Mega (USB-B 는 보드 왼쪽 끝)
    dot(ax, 28, 46.5, C_USB); ax.text(29, 46.5, "USB-B", va="center", fontsize=9.5, color=C_USB, zorder=4)
    wire(ax, [(16.5, 46.5), (27.6, 46.5)], C_USB)
    tag(ax, 22, 48.8, "USB-A 직결", c=C_USB, fs=9, bold=True)
    note(ax, 1, 37, "● 허브 경유 금지: 수신기가 4.09 V 까지 처진다\n● 포트가 열릴 때 리셋 →\n   udev 98-ssf-serial-safety 로 차단\n   (ModemManager 제외 · -hupcl)",
         c="#a93226", fs=9)
    # 수신기
    box(ax, 66, 37, 16, 16, "RC 수신기", "FlySky FS-iA6B", fc="#fef9e7")
    for ch, y_pin, y_rx in [("CH2  오른쪽 스틱 상하", 38, 47), ("CH4  왼쪽 스틱 좌우", 34.5, 44), ("CH6  SwD 모드", 31, 41)]:
        wire(ax, [(50.4, y_pin), (56 + (47 - y_rx) * 0.4, y_pin), (56 + (47 - y_rx) * 0.4, y_rx), (65.7, y_rx)], C_SIG, lw=2)
        dot(ax, 66, y_rx, C_SIG); ax.text(67, y_rx, ch, va="center", fontsize=9.3, color=C_SIG, zorder=4)
    ax.text(83, 44, "← 채널마다 신호선만 Mega 로\n   수신기 전원은 Mega 5V·GND", fontsize=8.5, color="#555", va="center")
    wire(ax, [(50.4, 42.5), (54, 42.5), (54, 51.5), (65.7, 51.5)], C_5V, lw=2)
    tag(ax, 60, 53, "5V · GND (수신기 전원)", c=C_5V, fs=9)
    # ESC
    box(ax, 66, 19, 14, 10, "ESC 좌 · 우", "BESC30-R3 ×2\n서보 신호 1100~1900 µs", fc="#eafaf1", bfs=9)
    wire(ax, [(50.4, 25.5), (65.7, 25.5)], C_SIG, lw=2); wire(ax, [(50.4, 22), (65.7, 22)], C_SIG, lw=2)
    ax.text(81.5, 26.5, "1500 = 정지 · >1500 = 전진(실물)\nROS 는 반대 → ssf_bridge 가 3000−pwm", fontsize=9, color="#333", va="center")
    ax.text(81.5, 21, "● 핀 13 금지: 부트로더가 깜빡여\n   스러스터를 돌린다 (08-25 사고)", fontsize=9, color="#a93226", va="center")
    # LED
    box(ax, 66, 8, 14, 9, "WS2812 스트립", "34 px · 밝기 상한 40", fc="#fdf2e9", bfs=9)
    wire(ax, [(50.4, 17), (58, 17), (58, 13.5), (65.7, 13.5)], C_SIG, lw=2)
    ax.text(81.5, 13.5, "DIN 쪽에 연결(방향 있음 — \"IN\" 표시)\n전원은 Mega 5V (수신기와 같은 레일)\n초록=수동 · 노랑=자율 · 빨강=비상정지 · 꺼짐=대기",
            fontsize=9, color="#333", va="center")
    # 비상정지 감지
    wire(ax, [(50.4, 12.5), (54, 12.5), (54, 5), (65.5, 5)], C_SIG, lw=2)
    tag(ax, 66, 5, "접촉기 보조접점 21-22 (② 참고)", c=C_SIG, fs=9, ha="left")
    note(ax, 1, 25, "공통 GND:\nMega GND · 수신기 · LED ·\nESC 신호 GND ×2 ·\n배터리 A ⊖ · B ⊖",
         c=C_GND, fs=9)
    legend(ax, [("USB 데이터", C_USB, "-"), ("Mega 5V", C_5V, "-"), ("제어 신호", C_SIG, "-")], x=1, y=2.5)
    save(fig, "회로_3_제어신호.png")


# ═══════════════════════════════════════════════ ④ 센서 허브 전원
def d4():
    fig, ax = new("④ 센서 허브 전원 — 배터리 A → 3 A 퓨즈 → 5 V 컨버터 → USB 허브",
                  "접촉기 앞에서 분기하므로 비상정지를 눌러도 센서는 켜져 있다. 10-28 실측: 스러스터 1600·30 s 동안 USB 끊김 0")
    box(ax, 1, 38, 14, 14, "배터리 A", "⊕ 접촉기 5번 쪽 본선\n⊖ 컨버터 IN⊖ 납땜점", fc="#fdecea", bfs=9)
    box(ax, 20, 41, 10, 8, "BF303", "인라인 퓨즈\n3 A", fc="#fef9e7", tfs=11, bfs=9)
    box(ax, 36, 34, 18, 18, "5 V 컨버터", "SZWENGAO\nWG-1224S0505\n입력 8~36 V\n출력 5 V 고정 5 A · IP68", fc="#f4ecf7")
    box(ax, 62, 37, 12, 12, "DC 플러그", "5.5 × 2.1\n중심 ⊕ · 바깥 ⊖\n(UH308 동봉선 재활용)", fc="#fdf2e9", tfs=11, bfs=8.5)
    box(ax, 80, 30, 19, 23, "ipTIME UH308", "유전원 USB 허브\nUSB 3.0 ×4 · 2.0 ×3\n충전 전용 ×1 (데이터 없음)\n포트별 전원 버튼·LED", fc="#eaf2fb")
    wire(ax, [(15.3, 47), (19.7, 47)], C_PWR); tag(ax, 17.5, 48.6, "18AWG", c=C_PWR, fs=8.5)
    wire(ax, [(30.3, 47), (35.7, 47)], C_PWR); tag(ax, 33, 48.6, "IN+ 빨강", c=C_PWR, fs=9, bold=True)
    wire(ax, [(15.3, 41), (17, 41), (17, 37), (35.7, 37)], C_GND); tag(ax, 26, 35.4, "IN− 검정①", c=C_GND, fs=9, bold=True)
    wire(ax, [(54.3, 47), (61.7, 47)], C_5V); tag(ax, 58, 48.6, "OUT+ 노랑", c=C_5V, fs=9, bold=True)
    wire(ax, [(54.3, 39), (61.7, 39)], C_GND); tag(ax, 58, 37.4, "OUT− 검정②", c=C_GND, fs=9, bold=True)
    wire(ax, [(74.3, 43), (79.7, 43)], C_5V, lw=3.2); tag(ax, 77, 44.8, "DC IN 5V", c=C_5V, fs=9, bold=True)
    # 장치
    devs = [("D455 카메라 · USB 3.0 · 30 fps", 21), ("라이다 RPLIDAR A3 · /dev/ttyLiDAR (CP2102 0addfc38)", 15.5),
            ("IMU iAHRS RB-SDA-v1 · /dev/IMU (CP2102 0001)", 10), ("GPS u-blox · /dev/ttyGPS (CDC-ACM)", 4.5)]
    for i, (n, y) in enumerate(devs):
        box(ax, 46, y - 2.2, 30, 4.6, n, fc="#eafaf1", tfs=9.3)
        xb = 82 + i * 3.6
        wire(ax, [(xb, 29.8), (xb, y), (76.3, y)], C_USB, lw=2)
    box(ax, 1, 4, 15, 13, "배 노트북", "USB-A → 허브 업스트림\n(Mega 는 다른 포트에\n노트북 직결)", fc="#eaf2fb", bfs=9)
    wire(ax, [(16.3, 10.5), (40, 10.5), (40, 31.8), (79.7, 31.8)], C_USB, lw=2.4)
    tag(ax, 58, 30.3, "업스트림 USB (허브 동봉 1 m)", c=C_USB, fs=9, bold=True)
    note(ax, 1, 31.5, "● 퓨즈는 본선 가까이(10 cm 안)\n   그 앞 구간은 보호 못 한다\n● 컨버터 IN/OUT 극성을 바꾸면 컨버터 사망\n"
                      "● 24 V 코일 컨버터 출력과 혼동 금지 — 허브 즉사", c="#a93226", fs=9)
    note(ax, 1, 23.5, "● 장치가 하나도 안 잡히면 UH308\n   포트 전원 버튼부터 (10-28)\n"
                      "● 배터리 A 가 빠져도 허브는 노트북\n   USB 전원으로 유지 (10-28 실측)", c="#333", fs=9)
    legend(ax, [("배터리 ⊕", C_PWR, "-"), ("⊖", C_GND, "-"), ("5 V", C_5V, "-"), ("USB", C_USB, "-")], x=18, y=1)
    save(fig, "회로_4_센서허브전원.png")


# ═══════════════════════════════════════════════ ⑤ 카메라 PoE
def d5():
    fig, ax = new("⑤ 카메라 PoE — 배터리 A → 2 A 퓨즈 → PoE 인젝터 → OAK-1 PoE",
                  "인젝터는 배터리 전압을 직접 받는다(5 V 컨버터 뒤 금지). 랜선 한 가닥이 카메라에 전원과 데이터를 같이 보낸다")
    box(ax, 1, 36, 14, 14, "배터리 A", "⊕ 접촉기 5번 쪽 본선\n⊖ 컨버터 IN⊖ 납땜점", fc="#fdecea", bfs=9)
    box(ax, 20, 39, 10, 8, "BF303", "인라인 퓨즈\n2 A", fc="#fef9e7", tfs=11, bfs=9)
    box(ax, 37, 30, 20, 22, "PoE 인젝터", "PSE5502G 계열\nDC IN 12~48 V (5.5×2.1, 중심 ⊕)\n→ 48 V 802.3af/at · 30 W\n기가비트 · 비방수\n(10-27 주문, 배송 중)", fc="#f4ecf7", bfs=9)
    tag(ax, 61, 47.8, "PoE OUT", c=C_LAN, fs=9.5, bold=True); dot(ax, 57.3, 46, C_LAN)
    tag(ax, 61, 36.8, "DATA IN", c=C_LAN, fs=9.5, bold=True); dot(ax, 57.3, 35, C_LAN)
    box(ax, 79, 38, 20, 14, "OAK-1 PoE", "Luxonis · 표준 화각 · 고정초점\nIP67 · 4~7.5 W\n169.254.1.222 (폴백 기본)", fc="#eafaf1", bfs=9)
    box(ax, 66, 13, 16, 12, "USB 이더넷 어댑터", "NETmate NM-UCL02\nAX88179 · 1 Gbps\n169.254.1.10/16 고정", fc="#eaf2fb", bfs=9)
    box(ax, 87, 13, 12, 12, "배 노트북", "USB-C 직결\n(허브 안 거침)", fc="#eaf2fb", bfs=9)
    wire(ax, [(15.3, 45), (19.7, 45)], C_PWR); tag(ax, 17.5, 46.6, "18AWG", c=C_PWR, fs=8.5)
    wire(ax, [(30.3, 45), (36.7, 45)], C_PWR); tag(ax, 33.5, 46.6, "DC IN ⊕", c=C_PWR, fs=9, bold=True)
    wire(ax, [(15.3, 39), (17, 39), (17, 33), (36.7, 33)], C_GND); tag(ax, 26, 31.4, "DC IN ⊖", c=C_GND, fs=9, bold=True)
    wire(ax, [(57.3, 46), (78.7, 46)], C_LAN, lw=3.2); tag(ax, 71, 47.8, "Cat5e 2 m (전원 + 데이터)", c=C_LAN, fs=9, bold=True)
    wire(ax, [(57.3, 35), (61, 35), (61, 19), (65.7, 19)], C_LAN, lw=2.6); tag(ax, 65.5, 27, "Cat5e (데이터만)", c=C_LAN, fs=9, ha="left")
    wire(ax, [(82.3, 19), (86.7, 19)], C_USB, lw=2.6)
    note(ax, 1, 27, "● 포트 방향: PoE OUT → 카메라, DATA IN → 노트북.\n   바뀌면 카메라가 안 켜진다\n"
                    "● 인젝터는 역극성 보호 명시 없음 — 꽂기 전 플러그 중심 ⊕ 확인\n"
                    "● 입력 하한 12 V — 배터리 12.5 V 아래면 영상이 끊길 수 있다\n"
                    "● 켤 때 돌입전류로 2 A 가 끊기면 3 A 로", c="#a93226", fs=9.5)
    note(ax, 1, 9, "카메라는 접촉기 앞에서 분기 — 비상정지를 눌러도 영상은 유지된다", c="#333", fs=9.5)
    legend(ax, [("배터리 ⊕", C_PWR, "-"), ("⊖", C_GND, "-"), ("이더넷 / PoE", C_LAN, "-"), ("USB", C_USB, "-")], x=1, y=2.5)
    save(fig, "회로_5_카메라PoE.png")


# ═══════════════════════════════════════════════ ⑥ 스러스터 3상
def d6():
    fig, ax = new("⑥ ESC ↔ T200 3상 배선 — 회전 방향은 선 두 가닥으로 정한다",
                  "T200 프로펠러가 CW/CCW 2종이라 같은 명령에 둘 다 물을 뒤로 밀도록 한쪽만 교차. 판정은 대칭 명령 hold.py 1600 1600 으로만")
    cols = [("흰", "#999"), ("초", "#27ae60"), ("파", "#2e86c1")]

    def panel(x0, title, crossed, sub):
        ax.text(x0 + 17, 52, title, ha="center", fontsize=13.5, fontweight="bold")
        ax.text(x0 + 17, 48.8, sub, ha="center", fontsize=9.5, color="#a93226" if crossed else "#1e8449", fontweight="bold")
        box(ax, x0, 18, 9, 27, "ESC", fc="#eafaf1", tfs=11)
        box(ax, x0 + 25, 18, 9, 27, "T200", fc="#eafaf1", tfs=11)
        ys = [38, 31.5, 25]
        ax.text(x0 + 9.6, 42.5, "ESC 쪽 색", fontsize=8.5, color="#555")
        ax.text(x0 + 24.6, 42.5, "T200 쪽 색", fontsize=8.5, color="#555", ha="right")
        for i, (n, c) in enumerate(cols):
            ax.text(x0 + 9.6, ys[i] + 1.4, n, fontsize=10, color=c, fontweight="bold")
        order = [0, 2, 1] if crossed else [0, 1, 2]
        for i, (n, c) in enumerate(cols):
            j = order.index(i) if crossed else i
            wire(ax, [(x0 + 9.3, ys[i]), (x0 + 13, ys[i]), (x0 + 21, ys[j]), (x0 + 24.7, ys[j])], c, lw=3)
            ax.text(x0 + 21.2, ys[j] + 1.4, cols[j][0], fontsize=10, color=cols[j][1], fontweight="bold")

    panel(4, "1번배 좌 (Mega 핀 12 · 접촉기 1번)", True, "교차 — ESC 초 ↔ T200 파")
    panel(52, "1번배 우 (Mega 핀 11 · 접촉기 6번)", False, "색 동일 — 흰·초·파 그대로")
    note(ax, 4, 14, "● 10-26 저녁 대칭 시험(hold.py 1600 1600 10)으로 확정: 둘 다 뒤로 바람 = 전진.  \"왜 둘이 다르지\" 하고 맞추지 말 것 — 맞추면 그쪽이 역방향\n"
                    "● 수동 모드에서 눈으로 판정하지 말 것: 조종기 중심이 60 µs 만 밀려도 좌우에 반대 명령이 가서 한쪽만 틀려 보인다\n"
                    "● 2번배(4기)는 09-01 기록: 좌앞·우뒤 교차, 좌뒤·우앞 동일 — 조립 후 같은 대칭 시험으로 다시 확정\n"
                    "● 스러스터·ESC·프로펠러를 바꿔 달면 이 그림은 무효 — hold.py 로 다시 판정", c="#333", fs=9.5)
    save(fig, "회로_6_스러스터3상.png")


if __name__ == "__main__":
    for f in (d1, d2, d3, d4, d5, d6):
        f()

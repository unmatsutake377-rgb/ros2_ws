#!/usr/bin/env python3
"""SSF 회로도 — 최종 통합본. 단자 배정은 docs/전달용/회로도_20260824.html 결선표가 출처다.
재현: python3 tools/circuit_diagram.py  ->  docs/전달용/배치도/회로도_통합.png"""
import os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle
matplotlib.rcParams["font.family"]="Apple SD Gothic Neo"; matplotlib.rcParams["axes.unicode_minus"]=False

C_PWR="#c0392b"; C_SIG="#2471a3"; C_GND="#555"; C_WARN="#b8860b"
def box(ax,x,y,w,h,t,fc="#fff",ec="#333",fs=9,lw=1.3,bold=False):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle="round,pad=0.008",fc=fc,ec=ec,lw=lw))
    ax.text(x+w/2,y+h/2,t,ha="center",va="center",fontsize=fs,
            fontweight="bold" if bold else "normal",linespacing=1.35)
def wire(ax,pts,c=C_PWR,lw=2.0,ls="-"):
    ax.plot([p[0] for p in pts],[p[1] for p in pts],color=c,lw=lw,ls=ls,
            solid_capstyle="round",zorder=1)
def lbl(ax,x,y,t,c="#333",fs=8,ha="center",va="center",bold=False):
    ax.text(x,y,t,ha=ha,va=va,fontsize=fs,color=c,fontweight="bold" if bold else "normal")

fig=plt.figure(figsize=(15.5,19)); gs=fig.add_gridspec(3,1,height_ratios=[1,1,1.12],hspace=0.10)
for a in range(3):
    ax=fig.add_subplot(gs[a]); ax.set_xlim(0,100); ax.set_ylim(0,100); ax.axis("off")
    globals()[f"ax{a+1}"]=ax

# ═══ ① 주 전력 ═══
ax=ax1
ax.set_title("① 주 전력 — 접촉기가 (+) 한 가닥씩만 끊는다. (-) 는 ESC 직결",
             fontsize=13,loc="left",fontweight="bold",pad=6)
box(ax,2,64,15,17,"배터리 A\n4S 14.8V\nDXF 8400mAh 120C",fc="#fdecea",bold=True)
box(ax,2,16,15,17,"배터리 B\n4S 14.8V\nDXF 8400mAh 120C",fc="#fdecea",bold=True)
box(ax,34,40,17,42,"MD-30a 접촉기\n(LS Metasol, DC 2극)\n\n코일 A1·A2 하나가\n두 극을 동시에",fc="#eef4fb",bold=True)
lbl(ax,36.5,76,"5",fs=11,bold=True); lbl(ax,48.5,76,"6",fs=11,bold=True)
lbl(ax,36.5,46,"2",fs=11,bold=True); lbl(ax,48.5,46,"1",fs=11,bold=True)
lbl(ax,36.5,72,"⊕ 유입",fs=7); lbl(ax,48.5,72,"⊖ 유출",fs=7)
lbl(ax,36.5,42,"⊕ 유입",fs=7); lbl(ax,48.5,42,"⊖ 유출",fs=7)
box(ax,62,68,16,13,"ESC 우 ×2\nBESC30-R3",fc="#eafaf1")
box(ax,62,20,16,13,"ESC 좌 ×2\nBESC30-R3",fc="#eafaf1")
box(ax,84,68,13,13,"T200 ×2\n(우)",fc="#eafaf1")
box(ax,84,20,13,13,"T200 ×2\n(좌)",fc="#eafaf1")
wire(ax,[(17,76),(36.5,76)]); wire(ax,[(48.5,76),(62,76)])
wire(ax,[(17,28),(36.5,28),(36.5,46)]); wire(ax,[(48.5,46),(48.5,28),(62,28)])
wire(ax,[(78,76),(84,76)],c="#111"); wire(ax,[(78,28),(84,28)],c="#111")
wire(ax,[(17,68),(26,68),(26,89),(70,89),(70,81)],c=C_GND,lw=1.6)
wire(ax,[(17,20),(22,20),(22,8),(70,8),(70,20)],c=C_GND,lw=1.6)
lbl(ax,46,91.5,"배터리 A ⊖ → ESC 우 ⊖ ×2  직결 (접촉기를 지나지 않는다)",c=C_GND,fs=8.5)
lbl(ax,46,5.5,"배터리 B ⊖ → ESC 좌 ⊖ ×2  직결",c=C_GND,fs=8.5)
lbl(ax,26,60,"A ⊕ 는 5번 가기 전에\n컨버터·릴레이로 분기 →②",c=C_WARN,fs=8,bold=True)
lbl(ax,88,60,"8~10 AWG\n계통당 42A · 순간 60A",c=C_PWR,fs=8)
lbl(ax,88,38,"▲ A=우 / B=좌 는 결선표(A→5·6)와\n신규설계(A→우)를 합친 추론이다.\n통전으로 한 번 확인할 것",c=C_WARN,fs=7.5)

# ═══ ② 차단(코일) 회로 ═══
ax=ax2
ax.set_title("② 차단 회로 — 버튼도 무선도 굵은 전류를 안 끊는다. 코일만 끊으면 두 극이 함께 열린다",
             fontsize=13,loc="left",fontweight="bold",pad=6)
box(ax,2,62,15,16,"배터리 A ⊕\n접촉기 5번보다\n배터리 쪽에서 분기",fc="#fdecea",bold=True)
box(ax,24,64,17,14,"DC-DC 컨버터\n승압 14.8→24V",fc="#fff8dc",ec=C_WARN,bold=True)
lbl(ax,30,55,"● 현재 11~16V — 4S 만충 16.8V 가 범위 밖\n재구매 필요(구매목록 #32). 확보 전 16V 초과 충전 금지",c=C_WARN,fs=8,bold=True)
box(ax,50,64,15,14,"접촉기 코일\nA1 ─ 48Ω ─ A2\nDC 24V",fc="#eef4fb",bold=True)
box(ax,72,64,12,14,"무선 릴레이\nSRD-12VDC-SL-C\nCOM–NC",fc="#eafaf1")
box(ax,72,34,12,14,"비상정지 버튼\n한영넉스 NC",fc="#eafaf1")
wire(ax,[(17,71),(24,71)]); wire(ax,[(41,71),(50,71)]); lbl(ax,45.5,74,"OUT⊕→A1",fs=7.5,c=C_PWR)
wire(ax,[(65,71),(72,71)]); lbl(ax,68.5,74,"A2→COM",fs=7.5,c=C_PWR)
wire(ax,[(78,64),(78,48)]); lbl(ax,84,56,"릴레이 NC\n→ 버튼 NC(1)",fs=7.5,c=C_PWR)
wire(ax,[(72,41),(32.5,41),(32.5,64)]); lbl(ax,50,38,"버튼 NC(2) → 컨버터 OUT⊖   (직렬 — 하나만 열려도 코일이 죽는다)",fs=8,c=C_PWR)
lbl(ax,78,30,"▲ 모듈 입력 DC 12V 인데 배터리 직결\n= 최대 140%. 점퍼는 자기유지로",c=C_WARN,fs=7.5)
box(ax,24,10,22,13,"접촉기 보조접점\n21 · 22  (b접점 NC)",fc="#eef4fb")
box(ax,54,10,18,13,"Mega 핀 38 / GND\n(상태 읽기만)",fc="#eaf2fb")
wire(ax,[(46,16.5),(54,16.5)],c=C_SIG,lw=1.6)
lbl(ax,13,16.5,"접촉기 열림 = 접점 닫힘 = 핀 38 LOW",fs=8,c=C_SIG)
lbl(ax,50,2.5,"● 아두이노는 이 회로에 없다 — 제어기가 죽어도 비상정지는 들어야 한다 (룰북 §8)",
    fs=9,c=C_PWR,bold=True)

# ═══ ③ 제어·신호·카메라 ═══
ax=ax3
ax.set_title("③ 제어 · 신호 · 카메라 — 핀은 arduino/ssf_boat/ssf_boat.ino 가 단일 출처",
             fontsize=13,loc="left",fontweight="bold",pad=6)
box(ax,2,74,17,16,"배 노트북\nIdeaPad Slim 3\n자체 배터리 운용",fc="#eaf2fb",bold=True)
box(ax,30,74,18,16,"Arduino Mega 2560\n노트북 USB 직결",fc="#eaf2fb",bold=True)
wire(ax,[(19,82),(30,82)],c=C_SIG)
lbl(ax,24.5,92,"● 허브 금지\n수신기 4.09V (최소 4.0)",c=C_PWR,fs=7.5,bold=True)
box(ax,58,80,16,10,"RC 수신기\nFS-iA6B",fc="#eafaf1")
wire(ax,[(48,85),(58,85)],c=C_SIG); lbl(ax,53,88,"2 / 3 / 19",fs=8,c=C_SIG,bold=True)
lbl(ax,66,76,"스로틀 2 · 조향 3 · 모드 19",fs=7.5,c=C_SIG)
box(ax,58,64,16,9,"ESC 좌 ×2",fc="#eafaf1"); box(ax,78,64,16,9,"ESC 우 ×2",fc="#eafaf1")
wire(ax,[(39,74),(39,68.5),(58,68.5)],c=C_SIG); lbl(ax,48,71,"핀 12",fs=8,c=C_SIG,bold=True)
wire(ax,[(41,74),(41,60),(86,60),(86,64)],c=C_SIG); lbl(ax,64,57.5,"핀 11",fs=8,c=C_SIG,bold=True)
lbl(ax,76,53.5,"● 핀 13 금지 — 부트로더가 스러스터를 돌린다",c=C_PWR,fs=8,bold=True)
box(ax,58,40,16,9,"WS2812 스트립\n초록/노랑/빨강",fc="#eafaf1")
wire(ax,[(43,74),(43,44.5),(58,44.5)],c=C_SIG); lbl(ax,50,47,"핀 16",fs=8,c=C_SIG,bold=True)
lbl(ax,66,36.5,"밝기 상한 40 (USB 폴리퓨즈 0.5A)",fs=7.5,c=C_WARN)
ax.add_patch(Rectangle((2,24),94,7,fc="#f2f2f2",ec=C_GND,lw=1.4))
lbl(ax,49,27.5,"공통 GND 버스 —  Mega GND · ESC 신호GND ×4 · 수신기 · LED · 배터리A ⊖ · 배터리B ⊖   "
                "(굵은 전원은 분리, 신호 GND 는 한 점)",fs=9,c=C_GND,bold=True)
box(ax,2,7,20,12,"배터리 ⊕\n─ 2A 퓨즈 ─",fc="#fdecea")
box(ax,27,7,19,12,"PoE 인젝터\n802.3af/at active\n30W · 비방수",fc="#fff8dc",ec=C_WARN)
box(ax,52,7,16,12,"OAK-1 PoE\n(마스트 위)",fc="#eafaf1")
box(ax,74,7,22,12,"노트북 이더넷\nAX88179 어댑터",fc="#eaf2fb")
wire(ax,[(22,13),(27,13)]); wire(ax,[(46,13),(52,13)],c="#111")
wire(ax,[(37,7),(37,3),(85,3),(85,7)],c=C_SIG,lw=1.6)
lbl(ax,49,21,"동봉 2m Cat5e",fs=7.5); lbl(ax,61,1,"LAN (데이터)",fs=7.5,c=C_SIG)
lbl(ax,12,21.5,"● 인젝터는 접촉기 앞에서 — 비상정지는 추진만 끊는다",c=C_PWR,fs=8,bold=True)

fig.text(.5,.006,"출처: 결선표 docs/전달용/회로도_20260824.html · 설계근거 회로_신규설계.md · 핀 arduino/ssf_boat/ssf_boat.ino"
        "   |   현재 진행상황은 docs/문제와작업/좌측스러스터_접촉기_20260819.md 가 단일 출처",
        ha="center",fontsize=9,color="#666")
out=os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","docs","전달용","배치도","회로도_통합.png")
os.makedirs(os.path.dirname(out),exist_ok=True)
plt.savefig(out,dpi=100,bbox_inches="tight",facecolor="white")
print(os.path.abspath(out))

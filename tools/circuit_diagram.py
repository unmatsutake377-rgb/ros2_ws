#!/usr/bin/env python3
"""SSF 회로도 — 최종 통합본. 단자 배정은 docs/전달용/회로도_20260824.html 결선표가 출처다.
재현: python3 tools/circuit_diagram.py -> 회로도_통합.png (2번배 기준 통합본) · SSF_BOAT=A|B python3 tools/circuit_diagram.py -> 회로도_통합_A.png / _B.png"""
import os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle
matplotlib.rcParams["font.family"]="Apple SD Gothic Neo"; matplotlib.rcParams["axes.unicode_minus"]=False

C_PWR="#c0392b"; C_SIG="#2471a3"; C_GND="#555"; C_WARN="#b8860b"
# 배 선택: SSF_BOAT=A → 1번배(측당 T200·ESC 1기) / B → 2번배(측당 2기) / 미설정 → 통합본(2번배 기준, 기존 파일명)
BOAT=os.environ.get("SSF_BOAT","")
NS = 1 if BOAT=="A" else 2; TOT = 2*NS
BNAME = {"A":"1번배 (S.S.F A)","B":"2번배 (S.S.F B)"}.get(BOAT,"2번배 4기 기준 · 1번배는 측당 ESC·T200 1기, 핀·접촉기 동일")
def box(ax,x,y,w,h,t,fc="#fff",ec="#333",fs=9,lw=1.3,bold=False):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle="round,pad=0.008",fc=fc,ec=ec,lw=lw))
    ax.text(x+w/2,y+h/2,t,ha="center",va="center",fontsize=fs,
            fontweight="bold" if bold else "normal",linespacing=1.35)
def wire(ax,pts,c=C_PWR,lw=2.0,ls="-"):
    ax.plot([p[0] for p in pts],[p[1] for p in pts],color=c,lw=lw,ls=ls,
            solid_capstyle="round",zorder=1)
def lbl(ax,x,y,t,c="#333",fs=8,ha="center",va="center",bold=False):
    ax.text(x,y,t,ha=ha,va=va,fontsize=fs,color=c,fontweight="bold" if bold else "normal")

fig=plt.figure(figsize=(15.5,24)); gs=fig.add_gridspec(4,1,height_ratios=[1,1,1.38,1.05],hspace=0.10)
for a in range(4):
    ax=fig.add_subplot(gs[a]); ax.set_xlim(0,100)
    ax.set_ylim(-22,100) if a==2 else ax.set_ylim(0,100)
    ax.axis("off")
    globals()[f"ax{a+1}"]=ax

# ═══ ① 주 전력 ═══
ax=ax1
ax.set_title(f"① 주 전력 — 접촉기가 (+) 한 가닥씩만 끊는다. (-) 는 ESC 직결   [{BNAME}{'' if not BOAT else f' · T200 ×{TOT}, 측당 ESC·T200 {NS}기'}]",
             fontsize=13,loc="left",fontweight="bold",pad=6)
box(ax,2,64,15,17,"배터리 A\n4S 14.8V\nDXF 8400mAh 120C",fc="#fdecea",bold=True)
box(ax,2,16,15,17,"배터리 B\n4S 14.8V\nDXF 8400mAh 120C",fc="#fdecea",bold=True)
box(ax,34,40,17,42,"MD-30a 접촉기\n(LS Metasol, DC 2극)\n\n코일 A1·A2 하나가\n두 극을 동시에",fc="#eef4fb",bold=True)
lbl(ax,36.5,76,"5",fs=11,bold=True); lbl(ax,48.5,76,"6",fs=11,bold=True)
lbl(ax,36.5,46,"2",fs=11,bold=True); lbl(ax,48.5,46,"1",fs=11,bold=True)
lbl(ax,36.5,72,"⊕ 유입",fs=7); lbl(ax,48.5,72,"⊖ 유출",fs=7)
lbl(ax,36.5,42,"⊕ 유입",fs=7); lbl(ax,48.5,42,"⊖ 유출",fs=7)
box(ax,62,68,16,13,f"ESC 우 ×{NS}\nBESC30-R3",fc="#eafaf1")
box(ax,62,20,16,13,f"ESC 좌 ×{NS}\nBESC30-R3",fc="#eafaf1")
box(ax,84,68,13,13,f"T200 ×{NS}\n(우)",fc="#eafaf1")
box(ax,84,20,13,13,f"T200 ×{NS}\n(좌)",fc="#eafaf1")
wire(ax,[(17,76),(36.5,76)]); wire(ax,[(48.5,76),(62,76)])
wire(ax,[(17,28),(36.5,28),(36.5,46)]); wire(ax,[(48.5,46),(48.5,28),(62,28)])
wire(ax,[(78,76),(84,76)],c="#111"); wire(ax,[(78,28),(84,28)],c="#111")
wire(ax,[(17,68),(26,68),(26,89),(70,89),(70,81)],c=C_GND,lw=1.6)
wire(ax,[(17,20),(22,20),(22,8),(70,8),(70,20)],c=C_GND,lw=1.6)
lbl(ax,46,91.5,f"배터리 A ⊖ → ESC 우 ⊖ ×{NS}  직결 (접촉기를 지나지 않는다)",c=C_GND,fs=8.5)
lbl(ax,46,5.5,f"배터리 B ⊖ → ESC 좌 ⊖ ×{NS}  직결",c=C_GND,fs=8.5)
lbl(ax,26,60,"A ⊕ 는 5번에서 옆으로 분기(둘째 나사 링단자·본선 납땜)\n→ ② 코일 컨버터 · ③ 인젝터 · ③ 허브 5V",c=C_WARN,fs=8,bold=True)
lbl(ax,88,60,f"8~10 AWG\n계통당 {21*NS}A · 순간 {30*NS}A",c=C_PWR,fs=8)
lbl(ax,88,38,"▲ A=우 / B=좌 는 결선표(A→5·6)와\n신규설계(A→우)를 합친 추론이다.\n통전으로 한 번 확인할 것",c=C_WARN,fs=7.5)

# ═══ ② 차단(코일) 회로 ═══
ax=ax2
ax.set_title("② 차단 회로 — 버튼도 무선도 굵은 전류를 안 끊는다. 코일만 끊으면 두 극이 함께 열린다",
             fontsize=13,loc="left",fontweight="bold",pad=6)
box(ax,2,62,15,16,"배터리 A ⊕\n접촉기 5번보다\n배터리 쪽에서 분기",fc="#fdecea",bold=True)
box(ax,24,64,17,14,"승압 컨버터 SZH-BPM001\n입력 9~20V → 24V 고정 3A\n(10-20 도착 ×2)",fc="#eafaf1",bold=True)
lbl(ax,30,55,"✓ 입력 9~20V 라 4S 만충 16.8V 커버 — 옛 11~16V 컨버터 교체 완료.\n출력 24V 고정(트리머 없음) · 3A > 코일 흡인 2A",c="#1f7a3a",fs=8,bold=True)
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
box(ax,58,64,16,9,f"ESC 좌 ×{NS}",fc="#eafaf1"); box(ax,78,64,16,9,f"ESC 우 ×{NS}",fc="#eafaf1")
wire(ax,[(39,74),(39,68.5),(58,68.5)],c=C_SIG); lbl(ax,48,71,"핀 12",fs=8,c=C_SIG,bold=True)
wire(ax,[(41,74),(41,60),(86,60),(86,64)],c=C_SIG); lbl(ax,64,57.5,"핀 11",fs=8,c=C_SIG,bold=True)
lbl(ax,76,53.5,"● 핀 13 금지 — 부트로더가 스러스터를 돌린다",c=C_PWR,fs=8,bold=True)
box(ax,58,40,16,9,"WS2812 스트립\n초록/노랑/빨강",fc="#eafaf1")
wire(ax,[(43,74),(43,44.5),(58,44.5)],c=C_SIG); lbl(ax,50,47,"핀 16",fs=8,c=C_SIG,bold=True)
lbl(ax,66,36.5,"밝기 상한 40 (USB 폴리퓨즈 0.5A)",fs=7.5,c=C_WARN)
ax.add_patch(Rectangle((2,24),94,7,fc="#f2f2f2",ec=C_GND,lw=1.4))
lbl(ax,49,27.5,f"공통 GND 버스 —  Mega GND · ESC 신호GND ×{TOT} · 수신기 · LED · 배터리A ⊖ · 배터리B ⊖   "
                "(굵은 전원은 분리, 신호 GND 는 한 점)",fs=9,c=C_GND,bold=True)
box(ax,2,7,20,12,"배터리 A ⊕ (5번 둘째 나사)\n분기 18AWG ─ BF303 2A",fc="#fdecea")
box(ax,27,7,19,12,"PoE 인젝터 PSE5502G\nDC 12-48V in → 48V 802.3af/at\n30W · 비방수 · DC IN 5.5×2.1",fc="#eafaf1")
box(ax,52,7,16,12,"OAK-1 PoE FF\n(마스트 위, 10-20 도착)",fc="#eafaf1")
box(ax,74,7,22,12,"NM-UCL02 (AX88179)\n→ 노트북 USB-C 직결",fc="#eaf2fb")
wire(ax,[(22,13),(27,13)]); wire(ax,[(46,13),(52,13)],c="#111")
wire(ax,[(37,7),(37,3),(85,3),(85,7)],c=C_SIG,lw=1.6)
lbl(ax,49,21,"동봉 2m Cat5e (전원+데이터)",fs=7.5); lbl(ax,61,1,"LAN (데이터) — 허브 안 거침",fs=7.5,c=C_SIG)
lbl(ax,12,21.5,"● 인젝터는 접촉기 앞에서 — 비상정지는 추진만 끊는다",c=C_PWR,fs=8,bold=True)

# ── 허브 계통 ──
box(ax,2,-16,20,12,"배터리 A ⊕ (5번 둘째 나사)\n분기 18AWG ─ BF303 3A",fc="#fdecea")
box(ax,27,-16,19,12,"SZWENGAO WG-1224S0505\n8~36V → 5V 고정 5A · IP68\n(트리머 없음, 10-22 도착)",fc="#eafaf1")
box(ax,52,-16,18,12,"ipTIME UH308\n유전원 허브",fc="#eaf2fb")
box(ax,76,-16,22,12,"LiDAR · IMU · GPS · D455",fc="#eafaf1")
wire(ax,[(22,-10),(27,-10)]); wire(ax,[(46,-10),(52,-10)])
wire(ax,[(70,-10),(76,-10)],c=C_SIG)
lbl(ax,49,-2.5,"DC 플러그 5.5x2.1   ● 중심 + / 바깥 - 를 멀티미터로 확인   ● 5V 고정이라 조정 불필요 — 허브 꽂기 전 출력 4.9~5.1V 확인만",fs=7.5,c=C_PWR,bold=True)
lbl(ax,24,-19.5,"● 퓨즈는 입력측",fs=7.5,c=C_PWR)
lbl(ax,80,-19.5,"노트북 USB-A② → 허브 · USB-A① → Mega · USB-C → 어댑터",fs=7.5,c=C_SIG)


# ═══ ④ 결선표 ═══
ax=ax4
ax.set_title("④ 결선표 — 이 표를 보고 한 줄씩 체크한다. 지난 사고는 이 표가 없어서 났다",
             fontsize=13,loc="left",fontweight="bold",pad=6)

def tbl(ax,x0,y0,w,rows,head,colw,title,tc="#333"):
    lbl(ax,x0,y0+6,title,fs=11,ha="left",bold=True,c=tc)
    ax.add_patch(Rectangle((x0,y0-len(rows)*4.1),w,4.1,fc="#e8eef5",ec="#999",lw=0.8))
    yy=y0
    ax.add_patch(Rectangle((x0,yy),w,4.1,fc="#d4dde8",ec="#999",lw=0.9))
    cx=x0+1
    for t,cw in zip(head,colw):
        lbl(ax,cx,yy+2.05,t,fs=8.5,ha="left",bold=True); cx+=cw
    for i,r in enumerate(rows):
        yy=y0-(i+1)*4.1
        ax.add_patch(Rectangle((x0,yy),w,4.1,fc="#fff" if i%2==0 else "#f6f8fa",ec="#ccc",lw=0.6))
        cx=x0+1
        for t,cw in zip(r,colw):
            warn = t.startswith("●")
            lbl(ax,cx,yy+2.05,t,fs=8,ha="left",c=C_PWR if warn else "#333",bold=warn); cx+=cw

pins=[("2","RC 스로틀  ← 수신기","인터럽트 핀 (2·3·18·19·20·21 만)"),
      ("3","RC 조향  ← 수신기","08-25 실측. 옛 표 3/2 는 뒤바뀐 값"),
      ("19","RC 모드  ← 수신기","08-18 이설. 18 은 M 소켓과 한 핀 충돌"),
      ("11",f"ESC 우 ×{NS}  신호 →","08-25 실물 확정"),
      ("12",f"ESC 좌 ×{NS}  신호 →","08-25 실물 확정"),
      ("● 13","● 사용 금지","● 부트로더가 부팅마다 흔든다 = 스러스터가 튄다"),
      ("16","WS2812 스트립 Din →","=TX2. 09-03 확정. 밝기 상한 40"),
      ("22 / 24 / 26","단색 룰 표시등 초록/노랑/빨강","(예비) 스트립 쓰면 미사용"),
      ("28","점검 LED →","배 ID 깜빡임·워치독. 항상 사용"),
      ("34 / 36","배 ID — 배선 안 함","09-17 확정. 두 배 동일. 둘 다 GND 면 FAULT"),
      ("38","비상정지 감지  ← 접촉기 21","INPUT_PULLUP · LOW = 차단됨"),
      ("GND","공통 GND 버스",f"ESC 신호GND ×{TOT} · 수신기 · LED · 배터리 ⊖ ×2")]
tbl(ax,1,88,47,pins,("Mega 핀","연결","비고"),(9,17,21),"Arduino Mega 2560  —  단일 출처: arduino/ssf_boat/ssf_boat.ino")

term=[("5","배터리 A ⊕","전류 유입 (⊕ 인쇄)"),
      ("6",f"ESC 우 ⊕ ×{NS}","전류 유출 (⊖ 인쇄)"),
      ("2","배터리 B ⊕","전류 유입"),
      ("1",f"ESC 좌 ⊕ ×{NS}","전류 유출"),
      ("A1","컨버터 OUT ⊕","코일 (48Ω · DC 24V)"),
      ("A2","릴레이 COM","코일"),
      ("21","Mega 핀 38","b접점 — 09-09 완료"),
      ("22","Mega GND","b접점 — 09-09 완료"),
      ("13/14, 43/44, 31/32","미사용","NO ×2 · NC ×1 남음"),
      ("● 극성","● ⊕→⊖ 방향 지킬 것","● 거꾸로면 아크가 안 꺼지고 접점이 녹아 붙는다")]
tbl(ax,52,88,47,term,("MD-30a 단자","연결","비고"),(13,14,20),"MD-30a 접촉기  —  단일 출처: 회로도_20260824.html 결선표")

lbl(ax,1,36,f"배터리 ⊖ 는 접촉기를 지나지 않는다 :  A ⊖ → ESC 우 ⊖ ×{NS} 직결 (분기: 컨버터 IN⊖ · 릴레이 IN-)  /  B ⊖ → ESC 좌 ⊖ ×{NS} 직결",
    fs=9,ha="left",c=C_GND,bold=True)
lbl(ax,1,31,"코일 회로 :  배터리A ⊕ (접촉기 5번보다 배터리 쪽) → 컨버터 IN⊕ / 릴레이 IN+   ·   컨버터 OUT⊕ → A1   ·   A2 → 릴레이 COM → 릴레이 NC → 버튼 NC(1)   ·   버튼 NC(2) → 컨버터 OUT⊖",
    fs=9,ha="left",c=C_PWR)
lbl(ax,1,26,"보조 전원 :  배터리A ⊕ 접촉기 5번 둘째 나사(링단자) 또는 본선 납땜 ─ 분기 18AWG ┬ BF303 2A → PSE5502G DC IN → Cat5e → OAK-1 PoE (LAN → NM-UCL02 → 노트북 USB-C)   ┬ BF303 3A → SZWENGAO 5V → UH308 허브",
    fs=9,ha="left",c="#111")
lbl(ax,1,19,"● 배선 전 무전원 도통 시험 : 배터리 연결 전에 (+) 와 (-) 가 어디서도 안 만나는지 확인한다. 한쪽 (-) 와 다른 쪽 (+) 가 만나면 29.6V — T200(최대 20V)이 탄다.",
    fs=9,ha="left",c=C_PWR,bold=True)
lbl(ax,1,14,f"● 배터리 연결 순서 : 1·2번 쪽 먼저 → 5·6번 쪽 나중 → 삑 {TOT}개 확인 → 15초 대기.  순서가 바뀌면 일부 ESC 만 시동이 걸린다.",
    fs=9,ha="left",c=C_PWR,bold=True)
lbl(ax,1,9,"▲ 미해결(10-26) : 릴레이 점퍼 자기유지 재시험 · MD-30a 연속통전 Ith · 2번배 동일 회로 결선(Mega·무선스위치 도착) · 인젝터·EC5·DC잭·3A 퓨즈 도착 대기 · 퓨즈홀더 2개 부족",
    fs=9,ha="left",c=C_WARN,bold=True)

fig.text(.5,.006,"출처: 결선표 docs/전달용/회로도_20260824.html · 설계근거 회로_신규설계.md · 핀 arduino/ssf_boat/ssf_boat.ino"
        "   |   구매·도착 현황 docs/전달용/구매_목록.md §0-1   |   2026-10-26 갱신",
        ha="center",fontsize=9,color="#666")
out=os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","docs","전달용","배치도",f"회로도_통합{'_'+BOAT if BOAT else ''}.png")
os.makedirs(os.path.dirname(out),exist_ok=True)
plt.savefig(out,dpi=100,bbox_inches="tight",facecolor="white")
print(os.path.abspath(out))

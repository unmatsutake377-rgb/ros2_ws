#!/usr/bin/env python3
"""T200 스러스터 배치 해석 — 1번배 2기 / 2번배 4기.

근거·결론: docs/전달용/스러스터_배치_해석.md
CFD 아님. 강체 조종모델 + 횡류항력 요댐핑 + 작동원판 제트확산 + 정수압 트림.
단위는 전부 mm/N (과거 m/mm 혼용으로 제트반경이 1000배 틀린 적 있음 — 섞지 말 것).
출력: docs/전달용/배치도/thruster_layout.png"""
import math, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Wedge
matplotlib.rcParams["font.family"]="Apple SD Gothic Neo"; matplotlib.rcParams["axes.unicode_minus"]=False
T_F,T_R,DP,DB = 46.,36.,76.,100.          # N, N, mm, mm
RHO,CD,CB,m = 1000.,1.1,.50,35.
jet  = lambda mm: .707*DP/2 + mm*math.tan(math.radians(6.))     # mm in, mm out
K    = lambda L_m,d_m: RHO*CD*d_m*L_m**4/32.
rate = lambda N,k: math.degrees(math.sqrt(N/k))
Nm   = lambda ys,yb=0.: (ys+yb)/1e3*(T_F+T_R)                   # y in mm -> Nm

L1,B1=1660.,580.; d1=m/(RHO*(L1/1e3)*(B1/1e3)*CB)*1e3; k1=K(L1/1e3,d1/1e3); y1=B1/2-DB/2-20
L2,B2=1700.,430.; d2=m/(RHO*(L2/1e3)*(B2/1e3)*CB)*1e3; k2=K(L2/1e3,d2/1e3); y2=B2/2-DB/2-20
Y_MIN = DB/2+5                                                   # 좌우 본체 안 닿는 최소 y
X_S   = 150.

print(f"1번배 흘수 {d1:.0f} y한계 {y1:.0f} | 2번배 흘수 {d2:.0f} y한계 {y2:.0f} 최소y {Y_MIN:.0f}")
print("\n2번배 실행가능 조합 (본체간섭 없음 + 후류 회피):")
feas=[]
for xb in (450.,550.,650.,750.,850.):
    need=jet(xb-X_S)
    for ys in [v for v in range(int(Y_MIN),int(y2)+1,5)]:
        for yb in [v for v in range(int(Y_MIN),int(y2)+1,5)]:
            if abs(ys-yb) < need: continue
            feas.append((Nm(ys,yb),xb,ys,yb,need))
feas.sort(reverse=True)
for N,xb,ys,yb,need in feas[:6]:
    tag = "선미바깥(조향 뒤)" if ys>yb else "선수바깥(조향 앞 — 침로불안정)"
    print(f"  {N:5.1f}Nm {rate(N,k2):4.0f}°/s  선수 x={xb:.0f} y=±{yb:.0f} / 선미 x=150 y=±{ys:.0f}"
          f"  필요{need:.0f} 실제{abs(ys-yb):.0f}  {tag}")
best=[f for f in feas if f[2]>f[3]][0]
print(f"\n선미바깥 조건 최선: {best[0]:.1f}Nm {rate(best[0],k2):.0f}°/s "
      f"선수 x={best[1]:.0f} y=±{best[3]:.0f}, 선미 y=±{best[2]:.0f}")

fig=plt.figure(figsize=(14,12)); gs=fig.add_gridspec(4,1,height_ratios=[1.25,1,1,1],hspace=.5)
def hull(ax,L,B,t):
    ax.add_patch(Polygon([[0,-B/2],[L*.72,-B/2],[L*.92,-B/2+B*.22],[L,0],[L*.92,B/2-B*.22],[L*.72,B/2],[0,B/2]],
                closed=True,fc="#dbe6f1",ec="k",lw=1.4))
    ax.set_title(t,fontsize=12,loc="left"); ax.set_aspect("equal"); ax.grid(alpha=.18)
    ax.set_xlim(-70,L+360); ax.set_ylim(-B/2-95,B/2+95); ax.set_xticks([]); ax.set_yticks([])
def thr(ax,x,y,c):
    for s in(1,-1):
        ax.add_patch(Circle((x,s*y),DB/2,fc=c,ec="k",lw=1.3,alpha=.85))
        ax.add_patch(Circle((x,s*y),DP/2,fc="none",ec="k",ls=":",lw=.7))

a=fig.add_subplot(gs[0]); hull(a,L1,B1,"① 1번배 T200×2 — 선미 최대폭.  요모멘트 = y × 추력, x 는 무관")
thr(a,220,y1,"#2a7"); a.annotate("",xy=(220,y1),xytext=(220,-y1),arrowprops=dict(arrowstyle="<->",color="#2a7",lw=1.6))
a.text(300,0,f"y=±{y1:.0f}  간격 {2*y1:.0f}mm",fontsize=10,color="#2a7",weight="bold")
a.text(L1+20,30,f"{Nm(y1):.1f}Nm\n{rate(Nm(y1),k1):.0f}°/s",fontsize=10,color="#2a7")
a.text(800,0,"x 는 요권한에 영향 없음 —\n트림·환기·후류로만 정한다.",fontsize=10,ha="center",bbox=dict(fc="#fff8dc",ec="#b8860b"))
a.text(40,-B1/2-72,f"흘수 {d1:.0f}mm · 프로펠러 축을 선저 아래 50mm 이상 (h/D 0.96 → 1.6, 환기 회피)",fontsize=9.5,color="#c00")

opts=[("(가) 둘 다 최대폭 — 요권한 최대, 후류 간섭 감수",750.,y2,y2,"#c60"),
      (f"(나) 선수 x={best[1]:.0f} 안쪽 · 선미 바깥 — 후류 회피 + 선미 조향  [추천]",best[1],best[3],best[2],"#2a7"),
      ("(다) 선수 바깥 · 선미 안쪽 — 요권한은 같으나 조향이 앞 = 침로 불안정",550.,y2,75.,"#a33")]
for i,(t,xb,yb,ys,c) in enumerate(opts):
    ax=fig.add_subplot(gs[i+1]); hull(ax,L2,B2,f"② 2번배 T200×4   {t}")
    thr(ax,X_S,ys,"#2a7"); thr(ax,xb,yb,"#36c")
    need=jet(xb-X_S); N=Nm(ys,yb)
    for s in(1,-1): ax.add_patch(Wedge((xb,s*yb),xb-X_S,175,185,fc="#36c",alpha=.13))
    ax.text(X_S,-B2/2-52,f"선미 y=±{ys:.0f}",fontsize=9,color="#2a7",ha="center")
    ax.text(xb,B2/2+52,f"선수 y=±{yb:.0f}",fontsize=9,color="#36c",ha="center")
    ok = abs(ys-yb)>=need
    ax.text(L2+20,0,f"{N:.1f}Nm → {rate(N,k2):.0f}°/s\n후류 필요 {need:.0f} / 실제 {abs(ys-yb):.0f}mm"
            f"\n{'후류 회피' if ok else '후류 간섭'}",fontsize=9.5,color=c,va="center")
fig.text(.5,.004,f"2번배 흘수 {d2:.0f}mm, 건현 {185-d2:.0f}mm 로 낮음. 선체폭 430 → y 한계 {y2:.0f}mm, 좌우 본체 안 닿는 최소 y {Y_MIN:.0f}mm."
         "   CFD 아님: 강체 조종모델 + 횡류항력. 속도 예측은 이 모델 밖.",ha="center",fontsize=9.5,color="#555")
out=os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","docs","전달용","배치도","thruster_layout.png")
os.makedirs(os.path.dirname(out),exist_ok=True)
plt.savefig(out,dpi=110,bbox_inches="tight"); print("\n",os.path.abspath(out))

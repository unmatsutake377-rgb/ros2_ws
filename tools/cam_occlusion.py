#!/usr/bin/env python3
"""카메라 화면에서 LiDAR·뱃머리가 어디 보이나 — 마스트 x 위치를 바꿔가며 비교.

근거: docs/전달용/하드웨어_배치_요구사항.md §2-5, §3.
쓰는 법: python3 tools/cam_occlusion.py [마스트x ...]    (기본 1264 1328)

주의 — 부호 규약:
  ang_below = 수평선 아래로 내려다본 각(+가 아래).
  TILT 는 음수가 '카메라를 아래로 숙임'. 화면 중심축이 수평에서 |TILT| 만큼 아래를 본다.
  화면 비율 0% = 화면 맨 위, 100% = 맨 아래.
카메라가 LiDAR 보다 높으므로 **가까울수록 더 가파르게 내려다봐 LiDAR 가 화면 아래로 내려간다**
(가까워서 커 보이는 효과보다 이쪽이 크다). 2026-09-17 확인.
"""
import math, sys

L      = 1660.0   # 1번배 전장
LIDX   = 1470.0   # LiDAR 중심 x
LID_D  = 76.0     # A3 본체 지름
LID_H  = 41.0     # A3 본체 높이 (데이터시트 Ø76 x 41)
HFOV = {"D455 (조종·컬러)": 90.0, "OAK-1 (자율)": 69.0}
CAMS = {   # 이름: (갑판 위 높이, 수직 FOV, 틸트[음수=아래])
    # 🚨 높이는 tools/cad/mast.py 가 단일 출처다. D455_ZC=62, OAK_Z=175.
    #    하드웨어_배치_요구사항 §3 의 "D455 ≈갑판+110" 은 v2 선반 시절 낡은 값이다.
    "D455 (조종·컬러)": ( 62.0, 65.0,   0.0),   # RGB 90x65
    "OAK-1 (자율)":     (175.0, 54.5, -11.0),   # HFOV 69 + 4:3 -> V=2*atan(tan(34.5)*3/4)=54.5
}

def frac(ang_below, vfov, tilt):
    """수평 아래 ang_below 인 점의 화면 세로 위치(0=위, 1=아래). 잘렸는지도 돌려준다."""
    raw = (vfov / 2 + ang_below + tilt) / vfov
    return max(0.0, min(1.0, raw)), raw

def report(mx):
    d_lid, d_bow = LIDX - mx, L - mx
    print(f"\n■ 마스트 x={mx:.0f}  (LiDAR 까지 {d_lid:.0f}mm, 뱃머리까지 {d_bow:.0f}mm)")
    for name, (z, vfov, tilt) in CAMS.items():
        a_top = math.degrees(math.atan((z - LID_H) / d_lid))
        a_bot = math.degrees(math.atan(z / d_lid))
        a_bow = math.degrees(math.atan(z / d_bow))
        ft, _  = frac(a_top, vfov, tilt)
        fb, rb = frac(a_bot, vfov, tilt)
        fw, _  = frac(a_bow, vfov, tilt)
        out = " ← 아랫부분 화면 밖" if rb > 1 else ""
        hfov = HFOV[name]
        wdeg = 2 * math.degrees(math.atan(LID_D / 2 / d_lid))   # 가까울수록 커 보인다
        wfrac = wdeg / hfov
        area = (fb - ft) * wfrac
        print(f"   {name:<18} LiDAR 세로 {ft*100:5.1f}~{fb*100:5.1f}%{out}"
              f"  가로 {wfrac*100:4.1f}%  화면 점유 {area*100:4.1f}%")
        print(f"   {'':<18} 뱃머리 {fw*100:5.1f}%   (LiDAR 상단 {a_top:.1f}° / 하단 {a_bot:.1f}° / 뱃머리 {a_bow:.1f}° 아래)")

if __name__ == "__main__":
    xs = [float(a) for a in sys.argv[1:]] or [1264.0, 1328.0]
    print("화면 세로 위치 (0%=맨 위, 100%=맨 아래). LiDAR 가 아래일수록 좋지만, 가까울수록 커 보인다 — 화면 점유율로 판단할 것.")
    for x in xs: report(x)

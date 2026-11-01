#!/usr/bin/env python3
"""페일세이프 단계 시간축 — 보고서 Ⅲ-5 그림. Ⅲ-6.2 벤치 시험에서 잰 시각을 그대로 옮긴다(새로 지어낸 값 없음).

  LiDAR 노드 종료 : 0.70 s 경고(레벨 1) · 2.99 s 정지(레벨 2) · 재기동 즉시 정상(레벨 0)
  IMU 드라이버 종료: 0.3 s /imu/yaw 멈춤 · 0.8 s /yaw_error 멈춤 · 1.8 s 경고 · 4.1 s 정지
기준값(ship_direction.yaml): 경고 0.7 s · 정지 3.0 s 경과, 연속 3회 확인. 레벨 1 = 속도 0.7 배·조향 유지, 레벨 2 = STOP_HOLD.
재현: python3 tools/failsafe_timeline_figure.py  → docs/전달용/배치도/failsafe_timeline.png
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"; matplotlib.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "전달용", "배치도", "failsafe_timeline.png")
C0, C1, C2 = "#7cc47c", "#f2c94c", "#e05a4f"
END = 6.0

fig, ax = plt.subplots(figsize=(12, 3.9))
rows = [  # (y, 이름, [(시작, 끝, 색)], [(시각, 글)])
    (1.6, "LiDAR 노드 종료", [(-1.0, 1.0, C0), (0.0, 0.70, C0), (0.70, 2.29, C1), (2.99, END - 2.99 - 0.6, C2)],
     [(0.0, "노드 종료"), (0.70, "0.70 s 경고"), (2.99, "2.99 s 정지")]),
    (0.4, "IMU 드라이버 종료", [(-1.0, 1.0, C0), (0.0, 1.8, C0), (1.8, 2.3, C1), (4.1, END - 4.1 - 0.6, C2)],
     [(0.0, "드라이버 종료"), (0.3, "0.3 s /imu/yaw 멈춤"), (0.8, "0.8 s /yaw_error 멈춤"), (1.8, "1.8 s 경고"), (4.1, "4.1 s 정지")]),
]
for y, name, bars, evs in rows:
    for x0, w, c in bars:
        ax.broken_barh([(x0, w)], (y - 0.25, 0.5), color=c, ec="white")
    ax.text(-1.1, y, name, ha="right", va="center", fontsize=10.5, weight="bold")
    for k, (t, txt) in enumerate(evs):
        ax.plot([t, t], [y - 0.3, y + 0.3], color="#333", lw=1)
        dy = 0.42 if k % 2 == 0 else -0.42
        ax.text(t, y + dy, txt, ha="center", va="bottom" if dy > 0 else "top", fontsize=8.8)
# LiDAR 재기동: 시간축 밖(실측 시각 없음) — 끝에 표시
ax.broken_barh([(END - 0.6 + 0.05, 0.55)], (1.6 - 0.25, 0.5), color=C0, ec="white")
ax.text(END - 0.32, 1.6 + 0.42, "재기동 즉시 정상", ha="center", va="bottom", fontsize=8.8)
ax.plot([END - 0.6, END - 0.6], [1.25, 1.95], color="#333", lw=1, ls=":")
ax.set_xlim(-1.0, END); ax.set_ylim(-0.3, 2.35); ax.set_yticks([])
ax.set_xlabel("센서가 끊긴 뒤 경과 시간 [s] (벤치 시험 실측)")
for s in ("left", "right", "top"): ax.spines[s].set_visible(False)
handles = [plt.Rectangle((0, 0), 1, 1, color=C0, label="레벨 0 정상 — 계산한 각도, 속도 100 %"),
           plt.Rectangle((0, 0), 1, 1, color=C1, label="레벨 1 경고 — 각도 유지, 속도 0.7 배"),
           plt.Rectangle((0, 0), 1, 1, color=C2, label="레벨 2 정지 — STOP_HOLD")]
ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.28), ncol=3, fontsize=9, frameon=False)
ax.set_title("페일세이프 단계 전환 — 경고 기준 0.7 s · 정지 기준 3.0 s 경과(연속 3회 확인)", fontsize=11.5)
fig.tight_layout(); fig.savefig(OUT, dpi=130, bbox_inches="tight"); print(OUT)

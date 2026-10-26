#!/usr/bin/env python3
"""보고서용 그림 2장 생성 (matplotlib 만 사용).
  docs/전달용/배치도/mission_flowchart.png — 종합임무 상태기계 (wp_mode 0→1→2→3→5→7, CLAUDE.md §3 표 기준)
  docs/전달용/배치도/node_graph.png        — 노드·토픽 구조도 (CLAUDE.md §3 토픽 표 기준)
재현: python3 tools/mission_flowchart.py
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
matplotlib.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "전달용", "배치도")


def box(ax, x, y, w, h, text, fc="#f4f6fa", ec="#334", fs=9, bold=False, style="round,pad=0.02,rounding_size=0.08"):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle=style, fc=fc, ec=ec, lw=1.2))
    ax.text(x, y, text, ha="center", va="center", fontsize=fs, weight="bold" if bold else "normal", linespacing=1.3)


def arrow(ax, p, q, text=None, color="#334", rad=0.0, fs=8, tpos=0.5, toff=(0, 0.12)):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=12, lw=1.1, color=color,
                                 connectionstyle=f"arc3,rad={rad}"))
    if text:
        mx = p[0] + (q[0] - p[0]) * tpos + toff[0]
        my = p[1] + (q[1] - p[1]) * tpos + toff[1]
        ax.text(mx, my, text, ha="center", va="center", fontsize=fs, color=color,
                bbox=dict(fc="white", ec="none", pad=1))


def flowchart():
    fig, ax = plt.subplots(figsize=(11, 7.2))
    ax.set_xlim(0, 11); ax.set_ylim(0, 7.2); ax.axis("off")
    ax.set_title("종합임무 상태기계 — /wp_mode 가 담당 노드와 검출기를 정한다", fontsize=12, loc="left", weight="bold")

    # 상단: 공통 체인
    box(ax, 5.5, 6.55, 10.4, 0.62,
        "공통 체인 (모든 모드)   north_goal_angle(GPS 목표방위·지오펜스) → 미션 노드 → ship_direction(LiDAR 갭 선택·회피·페일세이프) "
        "→ motor_control(각도→PWM, 감속) → ssf_bridge → Mega → ESC",
        fc="#eef3ff", fs=8.5)

    # 미션 상자 (가로 흐름)
    xs = [1.1, 2.9, 4.7, 6.5, 8.3, 10.1]
    y = 4.6
    items = [
        ("mode 0·1\n항로추종\nship_gate", "gate 검출기\n/red_angle /green_angle", "#e6f4ea"),
        ("mode 2\n위치유지 ×2\nship_back", "turn(white)\n/image_angle", "#e6f4ea"),
        ("mode 3\n탐색(선회)\nship_turn", "turn(red)\n/image_angle /image_color", "#e6f4ea"),
        ("mode 5\n장애물 회피 50 s\n(미션 노드 없음)", "검출기 없음\n20000 → 전진 폴백", "#fff4e0"),
        ("mode 7\n도킹 60 s\nship_dock", "dock\n/image_angle", "#e6f4ea"),
        ("종료\nSTOP_HOLD 50000", "", "#f3e6e6"),
    ]
    for x, (t1, t2, fc) in zip(xs, items):
        box(ax, x, y, 1.6, 1.05, t1, fc=fc, fs=8.5, bold=True)
        if t2:
            box(ax, x, y - 1.2, 1.6, 0.7, t2, fc="#ffffff", ec="#888", fs=7.5)
            arrow(ax, (x, y - 0.85), (x, y - 0.53), color="#888")
    for a, b in zip(xs[:-1], xs[1:]):
        arrow(ax, (a + 0.8, y), (b - 0.8, y))
    ax.text(5.5, y + 0.75, "/wp_mode 전이 = north_goal_angle 의 웨이포인트 도달 (GPS RTK)   ·   mode 8 = 토너먼트 자율 회피(검출기 없음)",
            ha="center", fontsize=8.5, color="#333")

    # 하단: 모드 무관 규칙
    box(ax, 2.75, 1.55, 4.9, 1.6,
        "검출기 공통\n• /wp_mode 2 s 이상 stale → 발행 정지 + 5 s 주기 경고\n• 방위각만 발행, 거리는 LiDAR\n• 목표 색·형상 유효값 밖 → ERROR + 발행 정지",
        fc="#fbfbfb", ec="#888", fs=8)
    box(ax, 8.25, 1.55, 4.9, 1.6,
        "페일세이프 (ship_direction·motor_control)\n• 센서 stale 0.7 s → 감속 0.7배 / 3.0 s → STOP_HOLD\n• 명령 워치독 0.5 s (노트북) · 0.5 s (펌웨어) → 중립\n"
        "• 지오펜스 전방 ±60°·2 m 안 → STOP_HOLD · 비상정지 → /boat_estop",
        fc="#fbfbfb", ec="#888", fs=8)
    ax.text(0.3, 0.35, "특수 코드: 5000 우선회 · 6000 좌선회(예약) · 20000 미션 없음→전진 · 50000 정지.  모드 전환 통로는 조종기뿐(WAIT/MANUAL/AUTO).",
            fontsize=8, color="#555")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "mission_flowchart.png"), dpi=150)
    plt.close(fig)


def node_graph():
    fig, ax = plt.subplots(figsize=(11, 7.4))
    ax.set_xlim(0, 11); ax.set_ylim(0, 7.4); ax.axis("off")
    ax.set_title("ROS2 노드·토픽 구조 (CLAUDE.md §3 토픽 표)", fontsize=12, loc="left", weight="bold")
    S = "#e6f4ea"; N = "#eef3ff"; C = "#fff4e0"; H = "#f3e6e6"
    # 센서
    box(ax, 1.2, 6.3, 2.0, 0.6, "rplidar_ros\n(RPLIDAR A3)", fc=S)
    box(ax, 1.2, 5.3, 2.0, 0.6, "iahrs_driver\n(IMU)", fc=S)
    box(ax, 1.2, 4.3, 2.0, 0.6, "ublox + ntrip\n(GPS RTK)", fc=S)
    box(ax, 1.2, 3.3, 2.0, 0.6, "OAK / D455\n(카메라)", fc=S)
    # 처리
    box(ax, 4.2, 5.3, 2.2, 0.6, "yaw_mux\n(헤딩 단일 발행)", fc=N)
    box(ax, 4.2, 4.3, 2.2, 0.6, "north_goal_angle\n(GPS 목표·지오펜스)", fc=N)
    box(ax, 4.2, 3.3, 2.4, 0.7, "basic_image_subscriber\n{gate, turn, dock}", fc=N)
    box(ax, 7.0, 3.8, 2.3, 0.9, "미션 노드\nship_gate / ship_back\nship_turn / ship_dock", fc=N)
    box(ax, 7.0, 6.0, 2.3, 0.8, "ship_direction\n(갭 선택·회피·페일세이프)", fc=N, bold=True)
    box(ax, 9.8, 6.0, 1.9, 0.7, "motor_control\n(각도→PWM·감속)", fc=N)
    box(ax, 9.8, 4.6, 1.9, 0.7, "ssf_bridge\n(USB 시리얼)", fc=C)
    box(ax, 9.8, 3.3, 1.9, 0.7, "Arduino Mega\n→ ESC 좌/우", fc=C)
    box(ax, 7.0, 1.3, 4.6, 0.9, "진단·기록 (제어에 안 물림)\nhealthcheck → /health_ok · blackbox · mission_monitor", fc=H, ec="#a55")
    # 화살표
    arrow(ax, (2.2, 6.3), (5.85, 6.1), "/scan (BEST_EFFORT, depth 1)", rad=-0.05, tpos=0.45)
    arrow(ax, (2.2, 5.3), (3.1, 5.3), "/imu/yaw_raw")
    arrow(ax, (5.3, 5.3), (5.85, 5.75), "/imu/yaw", tpos=0.5, toff=(0.55, 0))
    arrow(ax, (5.3, 5.3), (5.85, 4.3), "/imu/yaw", rad=0.2, tpos=0.6, toff=(0.45, 0))
    arrow(ax, (2.2, 4.3), (3.1, 4.3), "/fix")
    arrow(ax, (5.3, 4.3), (5.85, 4.1), "/candidate_angle(목표방위)", tpos=0.5, toff=(-0.35, -0.3))
    arrow(ax, (5.3, 4.5), (5.85, 5.8), "/geofence_state", rad=-0.25, tpos=0.7, toff=(0.85, -0.05))
    arrow(ax, (2.2, 3.3), (3.0, 3.3), "image")
    arrow(ax, (5.4, 3.3), (5.85, 3.6), "/red_angle /green_angle\n/image_angle /image_color", tpos=0.5, toff=(0, -0.35))
    arrow(ax, (7.0, 4.25), (7.0, 5.6), "/candidate_angle", toff=(0.75, 0))
    arrow(ax, (8.15, 6.0), (8.85, 6.0), "/desired_angle\n/failsafe_level\n/obstacle_distance_array", toff=(0, 0.55), fs=7.5)
    arrow(ax, (9.8, 5.65), (9.8, 4.95), "Motor_run\n(pwm_r·10000+pwm_l)", toff=(0.9, 0), fs=7.5)
    arrow(ax, (9.8, 4.25), (9.8, 3.65), "L/R us · 상태줄 10 Hz", toff=(1.0, 0), fs=7.5)
    arrow(ax, (9.0, 4.4), (8.0, 1.75), "/boat_mode /boat_id\n/boat_estop /boat_cmd_watchdog", rad=0.3, tpos=0.75, toff=(-1.35, 0.0), fs=7.5, color="#a55")
    arrow(ax, (7.0, 5.6), (7.0, 1.75), "", color="#a55")
    ax.text(0.3, 0.45, "QoS: /scan·이미지 구독 BEST_EFFORT+KEEP_LAST+depth1, /wp_mode RELIABLE, /boat_* 발행 RELIABLE.  추력 반전은 ssf_bridge 한 곳.", fontsize=8, color="#555")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "node_graph.png"), dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    flowchart(); node_graph()
    print("wrote", OUT)

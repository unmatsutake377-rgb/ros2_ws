#!/usr/bin/env python3
"""LiDAR 끊김 → 페일세이프 L1(→ L2) 중 후진 명령의 Motor_run 시간축 (계약서_후진감속_L1_20261104 §10-4).

실제 노드 파일만 쓴다(로직 복제 없음): src/ship_direction/ship_direction/ship_direction.py 의 ShipDirection,
src/motor_control/motor_control/motor_control.py 의 MotorController 를 rosstub 으로 import 해 콜백·타이머를
손으로 돌린다. 비교용 '수정 전'은 같은 파일의 BASE_REV 판을 `git show` 로 꺼내 똑같이 import 한다.

시나리오
  A  도킹 BACKOFF: /candidate_angle 180 을 10 Hz 로 계속 발행(ship_dock BACKOFF 가 내는 값 — ShipDock 자체는
     안 띄움: 이 시험은 motor_control 출력만 보므로 180 의 출처는 무관). 전방 0.6 m 에 도크 벽(근접 감속이
     후진에 안 걸리는지 같이 본다). /scan 을 DROP_T0 부터 1.5 / 2.5 / 3.5 s 끊는다.
  B  ship_direction 자체 후진 [L]: candidate 20000, 전방 0.5 m 로 꽉 막힌 스캔 → 회피 경로 없음 → 260 0.5 s /
     STOP_HOLD 3 s 반복. 그 후진 창이 L1 구간에 걸리게 끊는다.

가정 (전부 노드 config yaml 값 — 대회 launch 와 같음)
  · ship_direction control/watchdog 10 Hz, motor_control 20 Hz(DT 0.05 s),
    /scan·/yaw_error·/candidate 10 Hz.
  · 한 틱 순서: (센서 입력) → watchdog_cb → control_cb → motor 콜백 → motor timer_callback.
  · L1 판정 = /scan 마지막 수신 후 failsafe_warn_sec(0.7 s) 초과가 failsafe_confirm_n(3)틱 연속 → 약 1.0 s 뒤.
    L2 = failsafe_stop_sec(3.0 s) 초과 3틱 → 약 3.3 s 뒤. 복구는 스캔 도착 다음 워치독 틱에 즉시.
  · /yaw_error 는 계속 들어온다(IMU 정상). 끊기는 건 /scan 뿐. 운동 모델 없음(PWM 만 본다).
  · '후진 편차 적분' Σ(pwm−1500)·dt [PWM·s] 는 후진 이동거리의 대용(추력 ∝ 편차 가정, ⚠️ 실측 아님).

재현: python3 tools/sim/l1_dropout_sim.py  → 표 stdout, 그림 tools/sim/out/l1_dropout_sim.png
"""
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
import rosstub                                   # noqa: E402
rosstub.install()
from rosstub import Clock                        # noqa: E402

BASE_REV = "cbc8b2a"     # 후진 L1 상한 적용 직전 커밋 (비교 기준 '수정 전')
MC_REL = "src/motor_control/motor_control/motor_control.py"
OUT_PNG = os.path.join(HERE, "out", "l1_dropout_sim.png")

rosstub.load_params(os.path.join(ROOT, "src/ship_direction/config/ship_direction.yaml"),
                    "/ship_direction")
rosstub.load_params(os.path.join(ROOT, "src/motor_control/config/motor_control.yaml"),
                    "/motor_control")
SD = rosstub.load_node(os.path.join(ROOT, "src/ship_direction/ship_direction/ship_direction.py"),
                       "ship_direction_real", extra_path=os.path.join(ROOT, "src/ship_direction"))
MC_NEW = rosstub.load_node(os.path.join(ROOT, MC_REL), "motor_control_real")
std = sys.modules["std_msgs.msg"]
Float32, Int32 = std.Float32, std.Int32


def load_base():
    """BASE_REV 판 motor_control.py 를 임시 파일로 꺼내 import. git 이 없으면 None."""
    try:
        src = subprocess.run(["git", "-C", ROOT, "show", f"{BASE_REV}:{MC_REL}"], check=True,
                             capture_output=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return None
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "motor_control_base.py")
        with open(p, "wb") as f:
            f.write(src)
        return rosstub.load_node(p, "motor_control_base")


MC_BASE = load_base()

DT = 0.05                 # motor_control 타이머
SD_EVERY = 2              # ship_direction 10 Hz = motor 2틱마다
DROP_T0 = 2.0


class Scan:
    """전방 0~160°(80 = 정면)를 dist 로 채운 스캔. ship_direction 이 0~160 인덱스만 본다."""
    def __init__(self, dist):
        self.angle_min = 0.0
        self.angle_increment = math.radians(1.0)
        self.ranges = [dist] * 161


def run(cand, wall, drop_t0, drop_len, t_end):
    Clock.t = 0.0
    sd = SD.ShipDirection()
    mcs = {"new": MC_NEW.MotorController()}
    if MC_BASE is not None:
        mcs["base"] = MC_BASE.MotorController()
    pub_a = sd.pubs["/desired_angle"]
    seen = 0
    rows = []
    for k in range(int(round(t_end / DT)) + 1):
        t = k * DT
        Clock.t = t
        if k % SD_EVERY == 0:
            if not (drop_t0 <= t < drop_t0 + drop_len - 1e-9):
                sd.scan_callback(Scan(wall))
            sd.yaw_error_callback(Float32(0.0))
            sd.candidate_callback(Float32(cand))
            sd.watchdog_cb()
            sd.control_cb()
            if pub_a.count != seen:              # 새로 발행된 /desired_angle 만 전달
                seen = pub_a.count
                for mc in mcs.values():
                    mc.angle_callback(pub_a.last)
                    mc.obstacle_callback(sd.pubs["/obstacle_distance_array"].last)
                    mc.failsafe_callback(sd.pubs["/failsafe_level"].last)
        row = dict(t=t, lvl=sd.failsafe_level,
                   ang=pub_a.last.data if pub_a.last is not None else None)
        for name, mc in mcs.items():
            mc.timer_callback()
            row[name] = mc.pubs["Motor_run"].last.data
        rows.append(row)
    return rows


def is_rev(a):
    """motor_control 분기 (5) 범위인지 — 표 표시용."""
    return a is not None and 161.0 < a < SD.SPIN_RIGHT


def ang_s(a):
    if a is None:
        return "-"
    return {50000.0: "STOP_HOLD", 20000.0: "INVALID"}.get(a, f"{a:g}")


def segments(rows):
    """(레벨, 각도, new, base) 가 같은 연속 구간으로 묶는다."""
    segs = []
    for r in rows:
        key = (r["lvl"], r["ang"], r["new"], r.get("base"))
        if segs and segs[-1][2] == key:
            segs[-1][1] = r["t"]
        else:
            segs.append([r["t"], r["t"], key])
    return segs


def rev_integral(rows, name):
    """Σ max(0, 평균pwm−1500)·dt — 후진 편차 적분 [PWM·s]."""
    s = 0.0
    for r in rows:
        if name not in r:
            return float("nan")
        c = r[name]
        s += max(0.0, (c // 10000 + c % 10000) / 2.0 - 1500.0) * DT
    return s


def print_table(title, rows):
    print(f"\n[{title}]")
    print(f"  {'구간 [s]':>13} | {'L':>1} | {'/desired_angle':>14} | {'Motor_run 수정 후':>17} | "
          f"{'수정 전(' + BASE_REV + ')':>16}")
    for t0, t1, (lvl, ang, new, base) in segments(rows):
        b = "-" if base is None else str(base)
        mark = "  ← L1 후진 상한" if (lvl == 1 and base is not None and new != base) else ""
        print(f"  {t0:5.2f}–{t1 + DT:5.2f} | {lvl} | {ang_s(ang):>14} | {new:>17} | {b:>16}{mark}")
    i_new, i_base = rev_integral(rows, "new"), rev_integral(rows, "base")
    print(f"  후진 편차 적분 Σ(pwm−1500)·dt: 수정 후 {i_new:.1f} / 수정 전 {i_base:.1f} PWM·s")
    return i_new, i_base


def main():
    if MC_BASE is None:
        print(f"⚠️ git show {BASE_REV} 실패 → '수정 전' 열 없이 진행")
    cases = []
    for L in (1.5, 2.5, 3.5):
        rows = run(180.0, 0.6, DROP_T0, L, DROP_T0 + L + 1.5)
        cases.append((f"A BACKOFF(/candidate_angle 180), /scan {L:g} s 끊김 @ {DROP_T0:g} s", rows))
    # B: [L] 은 첫 스캔부터 후진 0.5 s → STOP_HOLD 3 s → 후진(3.5–4.0 s) … 두 번째 후진 창이 L1 에 걸리게
    #    2.5 s 부터 2.5 s 끊는다(마지막 스캔 2.4 s → L1 ≈ 3.4 s, 복구 5.0 s).
    rows = run(SD.CANDIDATE_INVALID, 0.5, 2.5, 2.5, 6.0)
    cases.append(("B 자체 후진 [L](/desired_angle 260), /scan 2.5 s 끊김 @ 2.5 s", rows))

    print("LiDAR 끊김 주입 — 실제 ShipDirection + MotorController(수정 후 / 수정 전), 파라미터 = config yaml")
    summ = []
    for title, rows in cases:
        summ.append((title, *print_table(title, rows)))
    print("\n요약 — L1 중 후진 명령 시간, L2 시간, 후진 편차 적분(PWM·s, 이동거리 대용 ⚠️ 추력 ∝ 편차 가정)")
    for (title, i_new, i_base), (_, rows) in zip(summ, cases):
        l1r = sum(DT for r in rows if r["lvl"] == 1 and is_rev(r["ang"]))
        l2 = sum(DT for r in rows if r["lvl"] == 2)
        print(f"  {title.split(',')[0]:<34} {title.split(', ')[1]}")
        print(f"      L1 후진 {l1r:.2f} s, L2 {l2:.2f} s | 적분 수정 후 {i_new:.1f} / 수정 전 {i_base:.1f}"
              f" (차 {i_base - i_new:.1f})")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    matplotlib.rcParams["font.family"] = "Apple SD Gothic Neo"
    matplotlib.rcParams["axes.unicode_minus"] = False
    fig, axs = plt.subplots(len(cases), 1, figsize=(10, 2.3 * len(cases)), sharex=False)
    for ax, (title, rows) in zip(axs, cases):
        ts = [r["t"] for r in rows]
        for r in rows:
            if r["lvl"]:
                ax.axvspan(r["t"], r["t"] + DT, color=("#ffe08a" if r["lvl"] == 1 else "#f4a6a6"),
                           lw=0)
        if "base" in rows[0]:
            ax.step(ts, [r["base"] // 10000 for r in rows], where="post", color="#999999",
                    lw=1.6, label=f"수정 전 {BASE_REV}")
        ax.step(ts, [r["new"] // 10000 for r in rows], where="post", color="#1f77b4", lw=1.4,
                label="수정 후")
        ax.axhline(1500, color="k", lw=0.5)
        ax.set_ylim(1480, 1610)
        ax.set_ylabel("pwm_r")
        ax.set_title(title + "   (노랑 L1, 빨강 L2)", fontsize=9)
        ax.legend(loc="upper right", fontsize=8)
    axs[-1].set_xlabel("t [s]")
    fig.tight_layout()
    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
    fig.savefig(OUT_PNG, dpi=110)
    print(f"\n저장: {OUT_PNG}")


if __name__ == "__main__":
    main()

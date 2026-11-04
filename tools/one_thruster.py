#!/usr/bin/env python3
"""스러스터 한쪽만 돌려 보는 점검 도구 (대회용 아님 — 벤치·물가 확인 전용).

  python3 tools/one_thruster.py L 1580            # 좌측만 1580 µs, 8초
  python3 tools/one_thruster.py R 1420 5          # 우측만 1420 µs(후진 쪽), 5초
  python3 tools/one_thruster.py L 1550,1600,1650 5  # 값마다 5초씩, 사이사이 중립 3초
  python3 tools/one_thruster.py R 1600 --arm 0    # ESC 가 이미 깨어 있으면 시동 대기 생략

꼭 알아둘 것
  · 값은 **시리얼 원값**이다: 1500 = 정지, **>1500 = 전진**, <1500 = 후진 (펌웨어 머리말, 08-27 실측).
    ROS 쪽(motor_control, <1500 = 전진)과 반대다. 이 도구는 ssf_bridge 를 거치지 않으므로 3000−pwm 반전이 없다.
  · 신호선은 좌(핀 12)·우(핀 11) 두 개뿐이고 **한 신호를 그쪽 ESC 2개가 나눠 받는다.**
    "L" 은 좌측 2개가 함께 돈다. 정말 1개만 돌리려면 같은 쪽 다른 ESC 의 신호선(또는 전원)을 빼고 돌린다.
  · 모드 스위치가 **AUTO** 여야 펌웨어가 이 명령을 쓴다. MANUAL·WAIT 면 AUTO 가 될 때까지 중립만 보내며 기다린다.
  · ssf_bridge(대회 launch)가 같은 포트를 쓰고 있으면 두 프로그램이 번갈아 명령을 써서 위험하다 → 감지하면 시작하지 않는다.
  · 펌웨어 워치독 500 ms: 이 도구가 50 ms 마다 보낸다. 도구가 죽으면 0.5 s 안에 펌웨어가 중립으로 잡는다.
  · Ctrl+C 를 누르면 언제든 즉시 중립을 보내고 끝난다.
  · 기본 허용 범위 1500±200 µs. 넘기려면 --max-dev 300 처럼 준다(펌웨어 출구 클램프 1100~1900).
"""
import argparse
import os
import subprocess
import sys
import time

NEUTRAL = 1500
SEND_PERIOD = 0.05          # 20 Hz — 펌웨어 명령 워치독 500 ms 보다 충분히 짧게
FW_MIN, FW_MAX = 1100, 1900  # ssf_boat.ino ESC_OUT_MIN/MAX
MODE_NAMES = {0: "WAIT(대기)", 1: "MANUAL(RC)", 2: "AUTO"}


def parse_args():
    ap = argparse.ArgumentParser(description="스러스터 한쪽만 돌리는 점검 도구 (대회용 아님)")
    ap.add_argument("side", choices=["L", "R", "l", "r"], help="L = 좌측, R = 우측")
    ap.add_argument("pwm", help="µs 값. 여러 개면 쉼표로: 1550,1600,1650  (>1500 = 전진)")
    ap.add_argument("sec", nargs="?", type=float, default=8.0, help="값마다 돌리는 시간 [s] (기본 8)")
    ap.add_argument("--arm", type=float, default=15.0, help="시작 전 ESC 시동용 중립 유지 [s] (기본 15, 09-01 실측). 이미 깨어 있으면 0")
    ap.add_argument("--gap", type=float, default=3.0, help="값과 값 사이 중립 시간 [s] (기본 3)")
    ap.add_argument("--ramp", type=float, default=1.0, help="목표값까지 올리고 내리는 시간 [s] (기본 1, 0 = 계단)")
    ap.add_argument("--max-dev", type=int, default=200, help="허용 편차 |pwm−1500| 상한 (기본 200, 최대 400)")
    ap.add_argument("--port", default=None, help="시리얼 포트 (기본 /dev/ttyMEGA, 없으면 /dev/ttyACM0)")
    ap.add_argument("--wait-auto", type=float, default=60.0, help="AUTO 모드를 기다리는 최대 시간 [s] (기본 60)")
    ap.add_argument("--force", action="store_true", help="ssf_bridge 실행 감지를 무시 (권장하지 않음)")
    a = ap.parse_args()
    a.side = a.side.upper()
    try:
        a.values = [int(v) for v in a.pwm.split(",") if v.strip()]
    except ValueError:
        ap.error(f"pwm 값을 정수로 읽을 수 없음: {a.pwm}")
    if not a.values:
        ap.error("pwm 값이 없음")
    a.max_dev = max(0, min(400, a.max_dev))
    for v in a.values:
        if abs(v - NEUTRAL) > a.max_dev:
            ap.error(f"{v} µs 는 허용 범위 {NEUTRAL - a.max_dev}~{NEUTRAL + a.max_dev} 밖. 필요하면 --max-dev 를 올린다")
        if not FW_MIN <= v <= FW_MAX:
            ap.error(f"{v} µs 는 펌웨어 출구 범위 {FW_MIN}~{FW_MAX} 밖")
    if a.sec <= 0:
        ap.error("시간은 0보다 커야 함")
    return a


def bridge_running():
    """ssf_bridge 가 떠 있는지 (같은 포트에 쓰는 두 번째 프로그램 방지)."""
    try:
        out = subprocess.run(["pgrep", "-af", "ssf_bridge"], capture_output=True, text=True, timeout=2).stdout
    except Exception:
        return None
    me = str(os.getpid())
    lines = [l for l in out.splitlines() if l.strip() and not l.startswith(me + " ") and "one_thruster" not in l]
    return lines


class Link:
    def __init__(self, port):
        import serial  # pyserial
        self.s = serial.Serial(port, 115200, timeout=0, write_timeout=0.1)
        self.buf = b""
        self.status = None          # 마지막 상태줄 필드 리스트
        self.status_t = 0.0
        self.l = self.r = NEUTRAL
        self.last_send = 0.0

    def set(self, l, r):
        self.l, self.r = int(l), int(r)

    def pump(self):
        now = time.time()
        if now - self.last_send >= SEND_PERIOD:
            try:
                self.s.write(f"L{self.l},R{self.r}\n".encode("ascii"))
            except Exception as e:
                print(f"  ⚠️ 시리얼 쓰기 실패: {e}", flush=True)
            self.last_send = now
        try:
            self.buf += self.s.read(512)
        except Exception:
            pass
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            f = line.decode("utf-8", "replace").strip().split(",")
            if len(f) == 9 and f[0] == "S":
                try:
                    self.status = [int(x) for x in f[1:]]   # mode, wd, boatId, estop, FL, FR, RL, RR
                    self.status_t = now
                except ValueError:
                    pass
        time.sleep(0.01)

    def run_for(self, sec, l=None, r=None, every=None, label=""):
        if l is not None:
            self.set(l, r)
        t0 = time.time(); shown = -1
        while time.time() - t0 < sec:
            self.pump()
            el = int(time.time() - t0)
            if every and el != shown and el % every == 0:
                shown = el
                print(f"    {label}{el:>3d} s  {self.describe()}", flush=True)

    def describe(self):
        if self.status is None or time.time() - self.status_t > 1.0:
            return "상태줄 없음(펌웨어 응답 없음)"
        m, wd, bid, es, fl, fr, rl, rr = self.status
        extra = []
        if wd: extra.append("워치독 중립")
        if es: extra.append("비상정지!")
        if bid == 2: extra.append("배ID FAULT(펌웨어 강제 중립)")
        return f"모드 {MODE_NAMES.get(m, m)}  실제출력 좌 {fl} / 우 {fr}" + (f"  [{', '.join(extra)}]" if extra else "")

    def neutral_and_close(self):
        self.set(NEUTRAL, NEUTRAL)
        for _ in range(5):
            try:
                self.s.write(b"L1500,R1500\n"); self.s.flush()
            except Exception:
                pass
            time.sleep(0.02)
        try:
            self.s.close()
        except Exception:
            pass


def main():
    a = parse_args()
    port = a.port or ("/dev/ttyMEGA" if os.path.exists("/dev/ttyMEGA") else "/dev/ttyACM0")

    br = bridge_running()
    if br and not a.force:
        print("❌ ssf_bridge 가 실행 중이다. 같은 포트에 두 프로그램이 쓰면 명령이 섞인다.")
        for l in br[:3]:
            print("   ", l)
        print("   launch 를 끄고(Ctrl+C) 다시 실행하라.")
        sys.exit(2)

    try:
        link = Link(port)
    except ImportError:
        print("❌ pyserial 이 없다:  sudo apt install python3-serial"); sys.exit(2)
    except Exception as e:
        print(f"❌ 포트 열기 실패 {port}: {e}"); sys.exit(2)

    side_name = "좌측(핀 12)" if a.side == "L" else "우측(핀 11)"
    print(f"포트 {port} · {side_name} 만 · 값 {a.values} µs · 값마다 {a.sec:g} s  (>1500 = 전진)")
    print("⚠️ 프로펠러 주변 손·줄 치우기. 같은 쪽 ESC 2개가 함께 돈다(1개만이면 다른 쪽 신호선 분리).")
    print("   Ctrl+C = 즉시 중립.\n", flush=True)

    try:
        # 1) 펌웨어 응답 확인 (포트를 열 때 보드가 재시작하면 상태줄까지 ~5 s)
        t0 = time.time()
        while link.status is None and time.time() - t0 < 8.0:
            link.pump()
        if link.status is None:
            print("❌ 8 s 동안 상태줄(S,...)이 안 온다. 포트·펌웨어·USB 확인."); return 2
        print(f"펌웨어 응답 {time.time() - t0:.1f} s  ({link.describe()})", flush=True)

        # 2) ESC 시동 대기 — 중립 유지
        if a.arm > 0:
            print(f"\n▶ 중립 {a.arm:g} s — ESC 시동 대기 (삑삑 소리 확인)", flush=True)
            link.run_for(a.arm, NEUTRAL, NEUTRAL, every=5, label="중립 ")

        # 3) AUTO 모드 대기 — 아니면 펌웨어가 시리얼 명령을 안 쓴다
        t0 = time.time()
        if link.status[0] != 2:
            print(f"\n⏳ 지금 모드 {MODE_NAMES.get(link.status[0])} — 조종기 모드 스위치를 AUTO 로 (최대 {a.wait_auto:g} s 대기)", flush=True)
        while link.status[0] != 2:
            link.run_for(0.2, NEUTRAL, NEUTRAL)
            if time.time() - t0 > a.wait_auto:
                print("❌ AUTO 가 안 됐다. 중단."); return 3
        if link.status[3]:
            print("❌ 비상정지가 눌려 있다. 해제 후 다시."); return 3

        # 4) 값마다 램프 → 유지 → 램프 다운 → 중립
        for i, v in enumerate(a.values):
            print(f"\n▶ [{i + 1}/{len(a.values)}] {side_name} {v} µs  {a.sec:g} s", flush=True)
            steps = max(1, int(a.ramp / SEND_PERIOD)) if a.ramp > 0 else 1
            for k in range(1, steps + 1):
                p = round(NEUTRAL + (v - NEUTRAL) * k / steps)
                link.run_for(a.ramp / steps if a.ramp > 0 else 0.0, *((p, NEUTRAL) if a.side == "L" else (NEUTRAL, p)))
            link.run_for(a.sec, every=1, label="")
            # 실제 출력이 명령과 다르면 알려 준다(펌웨어 invert·gain·클램프 확인용)
            if link.status:
                got = link.status[4] if a.side == "L" else link.status[5]
                other = link.status[5] if a.side == "L" else link.status[4]
                if abs(got - v) > 3:
                    print(f"  ⚠️ 명령 {v} 인데 실제 출력 {got} — 펌웨어 motors[].invert/gain_num 또는 모드 확인", flush=True)
                if abs(other - NEUTRAL) > 3:
                    print(f"  ⚠️ 반대쪽 출력이 {other} (중립 아님)", flush=True)
            for k in range(steps - 1, -1, -1):
                p = round(NEUTRAL + (v - NEUTRAL) * k / steps)
                link.run_for(a.ramp / steps if a.ramp > 0 else 0.0, *((p, NEUTRAL) if a.side == "L" else (NEUTRAL, p)))
            if i < len(a.values) - 1:
                print(f"  중립 {a.gap:g} s", flush=True)
                link.run_for(a.gap, NEUTRAL, NEUTRAL)
        print("\n끝. 중립 복귀.")
        return 0
    except KeyboardInterrupt:
        print("\n⏹ Ctrl+C — 중립 복귀.")
        return 130
    finally:
        link.neutral_and_close()


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""fake_mega — 아두이노 없이 ssf_bridge 를 양방향 검증한다.

    source install/setup.bash
    python3 tools/fake_mega.py

pty(가상 시리얼)로 Mega 를 흉내내서, 브릿지가 실제로 쓰고 읽는지를 본다.
실물 Mega 가 오기 전까지 브릿지를 검증할 수 있는 **유일한 수단**이다.

⚠️ 이건 브릿지의 시리얼 입출력만 본다. **실물 검증이 아니다.**
   실제 배선·보드레이트·ESC 반응은 벤치에서 따로 확인해야 한다.
   (이 프로젝트는 "테스트 통과 = 안전" 이 아니라는 걸 실기 버그 8건으로 배웠다)

검증 항목:
  1) 부팅 시 중립(L1500,R1500) 발신
  2) Motor_run → 올바른 L/R 변환
  3) 펌웨어 상태 줄 → /boat_mode·/boat_id 발행
  4) 깨진 줄 무시
  5) 종료 시 중립 발신 (SIGTERM 포함)
"""
import os
import pty
import subprocess
import sys
import time

master, slave = pty.openpty()
port = os.ttyname(slave)
print(f"가짜 Mega 포트: {port}", flush=True)

env = dict(os.environ)
proc = subprocess.Popen(
    # 🚨 `ros2 run` 을 거치면 래퍼가 SIGTERM 을 자식에게 안 넘겨 종료 중립을 못 잰다.
    #    launch 는 실행파일을 직접 띄우므로, 시험도 같은 조건으로 맞춘다.
    [os.path.expanduser("~/ros2_ws/install/ssf_bridge/lib/ssf_bridge/bridge"),
     "--ros-args", "-p", f"port:={port}"],
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)

os.set_blocking(master, False)


def read_master(timeout=2.0):
    out = b""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            out += os.read(master, 4096)
        except BlockingIOError:
            time.sleep(0.05)
    return out.decode(errors="ignore")


results = []

# ── 1) 부팅 중립 ──
boot = read_master(4.0)
results.append(("부팅 시 중립 발신", "L1500,R1500" in boot, repr(boot[:60])))

# ── 3) 상태 줄 → 토픽 ──
# 🚨 echo --once 는 '지금 도착하는' 메시지를 기다린다. 먼저 다 밀어넣고 echo 하면
#    이미 지나가서 영원히 기다린다(처음에 이걸로 타임아웃 났다).
import threading
stop = threading.Event()


def feeder():
    while not stop.is_set():
        try:
            os.write(master, b"S,2,0,1,0,1400,1400,1400,1400\n")
        except OSError:
            return
        time.sleep(0.1)


t = threading.Thread(target=feeder, daemon=True)
t.start()

echo = subprocess.run(
    ["ros2", "topic", "echo", "/boat_mode", "std_msgs/Int32", "--once"],
    capture_output=True, text=True, timeout=20)
results.append(("상태 줄 → /boat_mode=2", "data: 2" in echo.stdout, echo.stdout.strip()[:60]))

echo = subprocess.run(
    ["ros2", "topic", "echo", "/boat_id", "std_msgs/Int32", "--once"],
    capture_output=True, text=True, timeout=20)
results.append(("상태 줄 → /boat_id=1(B배)", "data: 1" in echo.stdout, echo.stdout.strip()[:60]))

# ── 4) 깨진 줄 무시 ──
os.write(master, b"GARBAGE,,,\nS,99,0,0,0,1500,1500,1500,1500\n")
time.sleep(0.5)
echo = subprocess.run(
    ["ros2", "topic", "echo", "/boat_mode", "std_msgs/Int32", "--once"],
    capture_output=True, text=True, timeout=20)
results.append(("깨진 줄 뒤에도 모드 유지(2)", "data: 2" in echo.stdout, echo.stdout.strip()[:60]))

stop.set()
time.sleep(0.3)
read_master(0.5)   # 버퍼 비우기

# ── 2) Motor_run → 시리얼 ──
pub = subprocess.Popen(
    ["ros2", "topic", "pub", "-r", "5", "Motor_run", "std_msgs/Int32",
     "{data: 16001400}"],   # pwm_r=1600, pwm_l=1400
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
cmd = read_master(3.0)
pub.terminate()
results.append(("Motor_run 16001400 → L1400,R1600", "L1400,R1600" in cmd, repr(cmd[:60])))

# ── 5) 종료 시 중립 ──
read_master(0.5)
proc.terminate()
tail = read_master(3.0)
proc.wait(timeout=10)
results.append(("종료 시 중립 발신", "L1500,R1500" in tail, repr(tail[:60])))

print("\n" + "=" * 70)
fail = 0
for name, ok, detail in results:
    print(f"{'✅' if ok else '❌'}  {name}")
    if not ok:
        print(f"      실제: {detail}")
        fail += 1
print("=" * 70)
print(f"{len(results) - fail}/{len(results)} 통과")
sys.exit(1 if fail else 0)

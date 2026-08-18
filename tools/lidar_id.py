#!/usr/bin/env python3
"""lidar_id — 연결된 RPLIDAR 에게 **모델을 직접 물어본다.**

    python3 tools/lidar_id.py

왜 만들었나 (2026-08-18)
  라이다 본체 라벨이 지워져서 모델을 알 수 없었다. 그런데 모델을 모르면
  ① 교체 케이블을 못 고르고 ② `scan_mode` 를 맞게 설정할 수 없다.
  RPLIDAR 는 `GET_INFO`(0xA5 0x50) 에 모델·펌웨어·하드웨어·시리얼을 답한다.
  **라벨보다 이쪽이 확실하다** — 실물이 스스로 말하는 값이라 옮겨 적다 틀릴 일이 없다.

🚨 udev 심볼릭 링크(`/dev/ttyLiDAR`)를 믿지 않는다.
   그 규칙은 **USB 어댑터의 시리얼**로 매칭한다. 어댑터가 바뀌면 링크가 안 생기고,
   반대로 링크가 있어도 라이다가 살아있다는 뜻이 아니다(08-18 실제로 겪었다).
   → `/dev/ttyUSB*` 를 전부 훑는다.
"""

import glob
import subprocess
import sys
import time

import serial

BAUDS = (256000, 115200, 1000000, 460800)
# 모델 상위 4비트로 계열이 갈린다(Slamtec 관례). 하위는 세부 리비전.
FAMILY = {0x0: "A1 계열", 0x2: "A2 계열", 0x3: "A3 계열",
          0x4: "S1 계열", 0x6: "S2 계열", 0x7: "T1 계열"}


def dev_serial(port):
    try:
        out = subprocess.run(["udevadm", "info", "-q", "property", "-n", port],
                             capture_output=True, text=True, timeout=5).stdout
        for line in out.splitlines():
            if line.startswith("ID_SERIAL_SHORT="):
                return line.split("=", 1)[1]
    except Exception:                      # noqa: BLE001
        pass
    return "?"


def ask(port, baud):
    """GET_INFO 를 보내고 응답을 파싱한다. 실패하면 None."""
    try:
        s = serial.Serial(port, baud, timeout=1.0)
    except Exception:                      # noqa: BLE001
        return None
    try:
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b"\xA5\x25")               # STOP — 스캔 중이면 멈춘다
        time.sleep(0.1)
        s.reset_input_buffer()
        s.write(b"\xA5\x50")               # GET_INFO
        time.sleep(0.4)
        r = s.read(64)
    finally:
        s.close()
    if r[:2] != b"\xA5\x5A" or len(r) < 27:
        return None
    p = r[7:27]
    return {"model": p[0], "fw": f"{p[2]}.{p[1]}", "hw": p[3],
            "serial": p[4:20].hex().upper(), "baud": baud}


def main():
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        print("🚨 /dev/ttyUSB* 가 없다 — 라이다가 안 꽂혔거나 어댑터를 못 잡았다.")
        return 1
    print(f"검사할 포트 {len(ports)}개\n")
    found = False
    for port in ports:
        sn = dev_serial(port)
        print(f"▶ {port}  (어댑터 시리얼 {sn})")
        for b in BAUDS:
            info = ask(port, b)
            if not info:
                continue
            found = True
            fam = FAMILY.get(info["model"] >> 4, "미상")
            print(f"   ✅ 응답  보레이트 {info['baud']}")
            print(f"      모델    0x{info['model']:02X} ({info['model']})  → {fam}")
            print(f"      펌웨어  {info['fw']}    하드웨어 {info['hw']}")
            print(f"      시리얼  {info['serial']}")
            break
        else:
            print("   — 어느 보레이트에서도 무응답 (라이다가 아니거나 죽었다)")
    if not found:
        print("\n🚨 응답한 라이다가 없다.")
        return 1
    print("\n※ 모델 바이트는 계열만 확정한다. 세부 모델명(A2M12 등)은 "
          "펌웨어·하드웨어 번호와 함께 제조사 자료로 대조할 것.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

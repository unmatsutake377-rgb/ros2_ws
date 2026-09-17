import serial, sys, time

s = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
time.sleep(0.3)

log = []
def pump(dur, l, r, tag):
    t0 = time.time(); last = 0.0; buf = b''
    while time.time() - t0 < dur:
        if time.time() - last > 0.05:
            s.write(f"L{l},R{r}\n".encode()); last = time.time()
        buf += s.read(512)
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            line = line.decode('utf-8', 'replace').strip()
            if line.startswith('S,'):
                log.append((tag, line))
        time.sleep(0.01)

for i in (5, 4, 3, 2, 1):
    print(f"  {i}...", flush=True)
    time.sleep(1)

s.reset_input_buffer()
log.clear()

# 🚨 ESC 시동(arming)이 느리다. 6초 → 12초 → 25초로 계속 늘려왔다.
#    짧게 잡으면 "안 돈다"고 오판한다 — 08-25 에 두 번 겪었다.
ARM_SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0   # 09-01 실측: 15초면 시동된다
print(f"\n▶ 중립 유지 {ARM_SEC:.0f}초 — ESC 시동 대기 (삑삑 소리 확인)", flush=True)
for i in range(int(ARM_SEC), 0, -5):
    print(f"    {i}초 남음...", flush=True)
    pump(min(5.0, i), 1500, 1500, 'ARM')

print("\n▶ ① 좌측만 L1580,R1500  — 8초   (신호는 핀 12)", flush=True)
pump(8.0, 1580, 1500, 'P11')
print("▶ — 중립 — 5초", flush=True)
pump(5.0, 1500, 1500, 'N1')
print("\n▶ ② 우측만 L1500,R1580  — 8초   (신호는 핀 11)", flush=True)
pump(8.0, 1500, 1580, 'P12')
print("▶ — 중립 —", flush=True)
pump(2.0, 1500, 1500, 'N2')
s.write(b"L1500,R1500\n"); s.flush()
s.close()

print("\n=== 아두이노가 실제로 낸 출력 ===")
print("    (상태줄 순서는 FL,FR,RL,RR 이며 좌/우 배정은 08-25 실물 확정 — 좌=핀12, 우=핀11)")
for tag, name in [('ARM', '   시동대기'), ('P11', '① 좌측'), ('N1', '   중립'),
                  ('P12', '② 우측'), ('N2', '   중립')]:
    rows = [x for t, x in log if t == tag]
    if not rows:
        print(f"{name}: 상태줄 없음"); continue
    f = rows[-1].split(',')
    print(f"{name}: 모드={f[1]} 워치독={f[2]}  출력 {f[5]},{f[6]},{f[7]},{f[8]}   ({len(rows)}줄)")

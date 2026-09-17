"""스틱 하나만 움직였을 때 좌·우 출력이 어떻게 갈리는지 요약한다.

수동 믹싱은  cmdL = 1500 + thr + str,  cmdR = 1500 + thr - str  이다.
→ 스로틀만 움직이면 좌우가 **같아야** 하고, 조향만 움직이면 **반대로** 벌어져야 한다.
좌우가 같은데 한쪽만 안 돌면 그건 ESC·모터 쪽 문제다.
"""
import serial, sys, time

DUR = int(sys.argv[1]) if len(sys.argv) > 1 else 25

s = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
time.sleep(3.5)
s.reset_input_buffer()

print(f"{DUR}초간 기록합니다. 스틱 하나만 천천히 끝까지 밀었다 놓으십시오.\n")

buf = b''; t0 = time.time(); rows = []
while time.time() - t0 < DUR:
    buf += s.read(512)
    while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        line = line.decode('utf-8', 'replace').strip()
        if line.startswith('S,'):
            f = line.split(',')
            if len(f) >= 9 and f[1] == '1':          # 수동일 때만
                rows.append((int(f[5]), int(f[6])))
s.close()

if not rows:
    print("🚨 수동 모드 상태줄이 없습니다 — 모드 스위치가 자율에 있지 않은지 확인하십시오.")
    raise SystemExit

L = [r[0] for r in rows]; R = [r[1] for r in rows]
thr = [(a + b) / 2 - 1500 for a, b in rows]      # 스로틀 성분
str_ = [(a - b) / 2 for a, b in rows]            # 조향 성분

print(f"표본 {len(rows)}개\n")
print(f"{'':12}{'최소':>8}{'최대':>8}{'폭':>8}")
print("-" * 36)
print(f"{'좌 출력':12}{min(L):>8}{max(L):>8}{max(L)-min(L):>8}")
print(f"{'우 출력':12}{min(R):>8}{max(R):>8}{max(R)-min(R):>8}")
print(f"{'스로틀 성분':12}{min(thr):>8.0f}{max(thr):>8.0f}{max(thr)-min(thr):>8.0f}")
print(f"{'조향 성분':12}{min(str_):>8.0f}{max(str_):>8.0f}{max(str_)-min(str_):>8.0f}")

sw = max(thr) - min(thr)
ss = max(str_) - min(str_)
print()
if sw > 100 and ss < 40:
    print("→ 스로틀 축만 움직였습니다. 좌·우 명령이 같이 움직였습니다.")
    print("   그런데 한쪽만 돈다면 원인은 **아두이노 바깥**(ESC·모터·배선)입니다.")
elif ss > 100 and sw < 40:
    print("→ 조향 축만 움직였습니다. 좌·우가 반대로 벌어지는 게 정상입니다.")
elif sw > 100 and ss > 100:
    print("🚨 두 축이 같이 움직였습니다. 스틱 하나만 밀었다면 채널이 섞인 것입니다.")
else:
    print("→ 움직임이 작습니다. 스틱을 끝까지 밀어 주십시오.")

print(f"\n좌우 출력이 같았던 비율 : {sum(1 for a,b in rows if a==b)*100//len(rows)}%")

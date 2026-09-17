"""수동/자율 실시간 관찰 — 상태줄을 사람이 읽는 형태로 계속 찍는다.

명령은 한 줄도 보내지 않는다(읽기 전용). 자율에서는 워치독이 중립을 물고,
수동에서는 조종기 스틱이 그대로 출력에 나타난다.
"""
import serial, sys, time

DUR = int(sys.argv[1]) if len(sys.argv) > 1 else 40
MODE = {'0': '대기', '1': '수동', '2': '자율'}

s = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
time.sleep(3.5)          # 포트 열면 보드가 재시작한다 — 부팅 통과
s.reset_input_buffer()

print(f"{DUR}초간 관찰합니다. 스틱을 움직여 보십시오.\n")
print(f"{'모드':<5} {'워치독':<7} {'비상정지':<9} {'ESC (FL,FR,RL,RR)':<26} 중립?")
print("-" * 62)

buf = b''; t0 = time.time(); prev = None
while time.time() - t0 < DUR:
    buf += s.read(512)
    while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        line = line.decode('utf-8', 'replace').strip()
        if not line.startswith('S,'):
            continue
        f = line.split(',')
        if len(f) < 9:
            continue
        key = (f[1], f[2], f[4], tuple(f[5:9]))
        if key == prev:          # 바뀔 때만 찍는다 — 화면이 안 흐른다
            continue
        prev = key
        esc = ','.join(f[5:9])
        neutral = '✅' if all(x == '1500' for x in f[5:9]) else '🚨'
        wd = '중립강제' if f[2] == '1' else '정상'
        es = '작동' if f[4] == '1' else '정상'
        print(f"{MODE.get(f[1], f[1]):<5} {wd:<7} {es:<9} {esc:<26} {neutral}", flush=True)
    time.sleep(0.01)
s.close()
print("\n관찰 종료")

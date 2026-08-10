#!/usr/bin/env python3
"""펌웨어 상태 줄 파서 — 순수 로직 (ROS·시리얼 비의존, 테스트 대상).

펌웨어(`arduino/ssf_boat/ssf_boat.ino` publishStatus)가 10Hz 로 같은 시리얼에 되쏘는 줄:

    S,<mode>,<watchdog>,<boatId>,<estop>,<FL>,<FR>,<RL>,<RR>

  mode     0=WAIT(대기·중립) 1=MANUAL(RC 조종) 2=AUTO(브릿지 명령)
  watchdog 1 = 명령이 500ms 넘게 안 와서 펌웨어가 중립으로 잡고 있음
  boatId   0=A 1=B 2=FAULT(ID 핀 이상 → 펌웨어가 무조건 중립)
  estop    1 = 비상정지 눌림
  FL/FR/RL/RR  실제로 ESC 에 나간 µs (1000~2000)

🚨 **모르면 입을 다문다.** 형식이 조금이라도 어긋나면 None 을 돌려준다.
   추측해서 채우면 "모드 MANUAL" 같은 **틀린 확신**이 화면에 뜬다 —
   그건 아무것도 안 뜨는 것보다 나쁘다(사람이 그걸 믿고 배에 다가간다).
   펌웨어도 같은 규칙으로 깨진 명령 줄을 통째로 버린다(ino: parseCommandLine).
"""

# 펌웨어 enum 과 **값까지** 일치시킨다. 여기서 번호를 새로 매기면
# 펌웨어가 바뀌었을 때 두 곳이 조용히 어긋난다 (CLAUDE.md 3-1 과 같은 유형).
MODE_WAIT = 0
MODE_MANUAL = 1
MODE_AUTO = 2

BOAT_A = 0
BOAT_B = 1
BOAT_FAULT = 2

MODE_NAMES = {MODE_WAIT: "대기", MODE_MANUAL: "수동(RC)", MODE_AUTO: "자율(AUTO)"}
BOAT_NAMES = {BOAT_A: "A", BOAT_B: "B", BOAT_FAULT: "FAULT"}

PWM_MIN = 1000
PWM_MAX = 2000

_N_FIELDS = 9          # "S" + 8 개 값
_N_OUTPUTS = 4


def parse_status_line(line):
    """상태 줄 → dict. 형식이 어긋나면 None.

    반환: {mode, watchdog, boat_id, estop, outputs:[4개]}
    """
    if not isinstance(line, str):
        return None
    parts = line.strip().split(',')
    if len(parts) != _N_FIELDS or parts[0] != 'S':
        return None
    try:
        vals = [int(p) for p in parts[1:]]
    except ValueError:
        return None

    mode, watchdog, boat_id, estop = vals[0], vals[1], vals[2], vals[3]
    outputs = vals[4:]

    # 범위 검사 — 깨진 바이트가 정수로 파싱되는 경우가 실제로 있다
    # (시리얼은 노이즈로 문자가 섞여도 남은 부분이 숫자면 int() 를 통과한다).
    if mode not in MODE_NAMES:
        return None
    if boat_id not in BOAT_NAMES:
        return None
    if watchdog not in (0, 1) or estop not in (0, 1):
        return None
    if len(outputs) != _N_OUTPUTS:
        return None
    if any(o < PWM_MIN or o > PWM_MAX for o in outputs):
        return None

    return {
        "mode": mode,
        "watchdog": bool(watchdog),
        "boat_id": boat_id,
        "estop": bool(estop),
        "outputs": outputs,
    }


def mode_name(mode):
    """모드 번호 → 사람이 읽는 이름. 모르는 값이면 번호를 그대로 드러낸다."""
    return MODE_NAMES.get(mode, f"알수없음({mode})")


def boat_name(boat_id):
    return BOAT_NAMES.get(boat_id, f"알수없음({boat_id})")


def decode_motor_run(data):
    """Motor_run(Int32) → (pwm_l, pwm_r).

    motor_control 의 인코딩 `pwm_r*10000 + pwm_l` 의 역연산.
    🚨 이 규약은 motor_control·blackbox·펌웨어에 이미 있다. 여기서 다르게 쓰면
       한쪽만 낡는다 — 작년 리뷰(docs/기준/2025브리지_리뷰_회로AI전달.md §1)가
       확인한 식을 그대로 옮긴다.
    """
    data = int(data)
    pwm_r = data // 10000
    pwm_l = data - pwm_r * 10000
    return clamp_pwm(pwm_l), clamp_pwm(pwm_r)


def clamp_pwm(v):
    return PWM_MIN if v < PWM_MIN else PWM_MAX if v > PWM_MAX else int(v)


def format_command(pwm_l, pwm_r):
    """(좌, 우) → 펌웨어가 읽는 명령 줄. 형식은 ino:parseCommandLine 과 계약."""
    return f"L{clamp_pwm(pwm_l)},R{clamp_pwm(pwm_r)}\n"

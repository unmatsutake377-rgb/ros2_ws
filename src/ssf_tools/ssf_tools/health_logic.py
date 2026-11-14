"""healthcheck 순수 판정 — ROS 비의존 (test_health_logic.py 가 검사).

영상 상태 4가지 (계약서_OAK전환_20261104 결정 4):
  OK    : 마지막 수신이 timeout 안
  DEAD  : 받다가 끊김(timeout 초과) — 기존 동작
  WAIT  : 아직 한 번도 못 받았고 기동 후 유예(grace) 안 — 부팅 중(OAK PoE 연결 13~17 s)
  NEVER : 한 번도 못 받은 채 유예를 넘김 — **실패**. 예전엔 영원히 WAIT 라 depthai 연결 실패(IP·nmcli
          프로파일·패키지 미설치)에도 /health_ok 가 true 였다(계약 위험 2). 검출기는 그때 침묵한다.

⚠️ 다른 센서(scan/imu/gps)의 WAIT 는 그대로 '부팅 중' — 영상만 이렇게 본다(계약 범위).
"""

STATE_OK = "OK"
STATE_DEAD = "DEAD"
STATE_WAIT = "WAIT"
STATE_NEVER = "NEVER"

# /health_ok 를 false 로 만드는 상태들(영상용). 호출부가 다른 센서의 DEAD/NOFIX 와 합친다.
IMAGE_FAIL_STATES = (STATE_DEAD, STATE_NEVER)


def image_state(last_rx, now, t_boot, timeout_s, grace_s):
    """영상 수신 상태.

    last_rx : 마지막 수신 단조시각 (None = 아직 한 번도 못 받음)
    now     : 현재 단조시각
    t_boot  : 노드 기동 단조시각
    timeout_s : 받다가 이 시간 넘게 안 오면 DEAD
    grace_s   : 기동 후 이 시간 안에 첫 영상이 안 오면 NEVER (≤ 0 이면 유예 없음 = 첫 틱부터 NEVER)
    """
    if last_rx is None:
        return STATE_WAIT if (now - t_boot) <= grace_s else STATE_NEVER
    return STATE_OK if (now - last_rx) <= timeout_s else STATE_DEAD


# ---------------------------------------------------------------- §7-2 후진·선회 게이트 기계 검사
# cog_require_reverse_gate=false 가 '실제로 헤딩을 망치는' 소스들. yaw_mux 는 게이트가 꺼지면
# /heading_status 끝에 ' gate=off' 토큰을 붙인다(heading_logic.gps_cog_status_text, yaw_mux 추정기 줄).
#   cog_offset : IMU 오프셋을 COG 로 추정 — 후진 표본이 섞이면 180° 오수렴(R=1.0 이라 신뢰도로 못 거른다)
#   gps_cog    : COG 를 헤딩 본체로 씀(2026-11-15 공통 기본값, 64b8bf0f) — 게이트가 후진·선회 중 COG 를
#                거부하는 유일한 장치. 꺼지면 후진 순간 헤딩이 180° 튄다.
# 그 밖의 소스(imu_relative 등)에선 이 게이트가 헤딩에 안 쓰이므로 꺼져 있어도 무해 → 실패로 치지 않는다
# (거짓 정지 금지, CLAUDE.md §5).
GATE_CHECKED_SOURCES = ("cog_offset", "gps_cog")


def gate_off_source(heading_status):
    """/heading_status 가 '게이트가 의미 있는 소스인데 gate=off' 이면 그 소스 이름, 아니면 None.

    heading_status : '<source>:<state>[ 필드…][ gate=off]' 문자열 또는 None/''.
    접두는 'source:' 로 정확히 비교하고(비슷한 이름 오인 방지), gate=off 는 공백으로 나눈 토큰으로 본다
    (꼬리에 d=/t=/rej= 등 다른 필드가 있어도 된다).
    """
    if not heading_status:
        return None
    src, sep, _ = heading_status.partition(":")
    if not sep or src not in GATE_CHECKED_SOURCES:
        return None
    if "gate=off" not in heading_status.split():
        return None
    return src

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

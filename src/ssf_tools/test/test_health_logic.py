#!/usr/bin/env python3
"""health_logic 순수 로직 테스트 — ROS 없이 돈다.  tools/run_tests.sh ssf_tools

계약서_OAK전환_20261104 결정 4: 영상을 **한 번도** 못 받은 WAIT 가 기동 후 유예를 넘기면 실패(NEVER).
받다가 끊김(DEAD, 3 s) 은 전과 같다.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ssf_tools.health_logic import (  # noqa: E402
    IMAGE_FAIL_STATES, STATE_DEAD, STATE_NEVER, STATE_OK, STATE_WAIT, image_state,
)

TIMEOUT = 3.0
GRACE = 30.0
BOOT = 1000.0   # 단조시계 임의 기준


def test_wait_during_grace():
    """기동 직후~유예 안, 아직 한 장도 안 옴 → WAIT (부팅 중, 실패 아님). OAK 연결 13~17 s 가 여기 들어간다."""
    for dt in (0.0, 2.0, 15.0, 17.0, 29.9, 30.0):
        assert image_state(None, BOOT + dt, BOOT, TIMEOUT, GRACE) == STATE_WAIT, dt


def test_never_after_grace():
    """유예를 넘겨도 한 장도 안 옴 → NEVER (실패). 예전엔 영원히 WAIT 였다."""
    for dt in (30.01, 31.0, 60.0, 3600.0):
        assert image_state(None, BOOT + dt, BOOT, TIMEOUT, GRACE) == STATE_NEVER, dt


def test_ok_and_dead_unchanged():
    """한 번이라도 받은 뒤엔 기존 timeout 판정 그대로 — 유예와 무관."""
    now = BOOT + 100.0
    assert image_state(now - 0.1, now, BOOT, TIMEOUT, GRACE) == STATE_OK
    assert image_state(now - 3.0, now, BOOT, TIMEOUT, GRACE) == STATE_OK, "경계 포함(≤)"
    assert image_state(now - 3.01, now, BOOT, TIMEOUT, GRACE) == STATE_DEAD
    # 유예 안이어도 받았다 끊기면 DEAD (연결됐다 떨어진 경우)
    assert image_state(BOOT + 1.0, BOOT + 10.0, BOOT, TIMEOUT, GRACE) == STATE_DEAD


def test_first_frame_late_but_inside_grace_recovers():
    """29 s 에 첫 영상이 오면 그 뒤로는 OK — NEVER 로 굳지 않는다."""
    assert image_state(None, BOOT + 29.0, BOOT, TIMEOUT, GRACE) == STATE_WAIT
    assert image_state(BOOT + 29.0, BOOT + 31.0, BOOT, TIMEOUT, GRACE) == STATE_OK


def test_zero_grace_means_immediate_never():
    assert image_state(None, BOOT + 0.5, BOOT, TIMEOUT, 0.0) == STATE_NEVER


def test_fail_states():
    """/health_ok 를 false 로 만드는 영상 상태 = DEAD·NEVER. WAIT·OK 는 아니다."""
    assert set(IMAGE_FAIL_STATES) == {STATE_DEAD, STATE_NEVER}
    assert STATE_WAIT not in IMAGE_FAIL_STATES and STATE_OK not in IMAGE_FAIL_STATES

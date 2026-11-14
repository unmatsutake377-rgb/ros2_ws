#!/usr/bin/env python3
"""health_logic.gate_off_source 순수 로직 테스트 — ROS 없이 돈다.  tools/run_tests.sh ssf_tools

계약서_GPS방위_20261114 보완 C, C-Q6: §7-2 안전 스위치 cog_require_reverse_gate=false 를
cog_offset 뿐 아니라 gps_cog(두 배 공통 기본값)에서도 /health_ok=false 로 잡는다.
다른 소스(imu_relative 등)에선 무해 → 실패 아님(거짓 정지 금지).
문자열 형식은 yaw_mux 가 내는 그대로(heading_logic.gps_cog_status_text, yaw_mux 추정기 줄).
"""

import os
import re
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ssf_tools.health_logic import GATE_CHECKED_SOURCES, gate_off_source  # noqa: E402


def test_gps_cog_ok_gate_off_fails():
    assert gate_off_source("gps_cog:OK gate=off") == "gps_cog"


def test_gps_cog_ok_gate_on_passes():
    assert gate_off_source("gps_cog:OK") is None


def test_gps_cog_other_states_gate_off_fail():
    """꼬리에 다른 필드가 있어도 gate=off 토큰을 잡는다."""
    for hs in ("gps_cog:ASSIST d=+47.3 t=3.2 rej=spin gate=off",
               "gps_cog:HOLD age=1.2 rej=reverse gate=off",
               "gps_cog:INIT_GOAL rej=- gate=off"):
        assert gate_off_source(hs) == "gps_cog", hs


def test_gps_cog_other_states_gate_on_pass():
    for hs in ("gps_cog:ASSIST d=+47.3 t=3.2 rej=spin",
               "gps_cog:HOLD age=1.2 rej=reverse",
               "gps_cog:HOLD age=-- rej=-"):
        assert gate_off_source(hs) is None, hs


def test_cog_offset_gate_off_fails():
    """기존 동작 유지."""
    assert gate_off_source("cog_offset:OK n=40 R=0.98 off=12.3 rej=- gate=off") == "cog_offset"
    assert gate_off_source("cog_offset:WAIT n=3 R=0.50 off=-- rej=slow gate=off") == "cog_offset"


def test_cog_offset_gate_on_passes():
    assert gate_off_source("cog_offset:OK n=40 R=0.98 off=12.3 rej=-") is None


def test_imu_relative_gate_off_harmless():
    """imu_relative 는 이 게이트를 헤딩에 안 쓴다 → 꺼져 있어도 실패 아님(거짓 정지 금지)."""
    assert gate_off_source("imu_relative:OK n=0 R=0.00 off=-- rej=- gate=off") is None
    assert gate_off_source("imu_relative:OK gate=off") is None


def test_other_sources_gate_off_harmless():
    for src in ("dual_gps", "imu_absolute", "unknown"):
        assert gate_off_source(f"{src}:OK gate=off") is None, src


def test_empty_or_none_passes():
    assert gate_off_source(None) is None
    assert gate_off_source("") is None


def test_prefix_exact_not_substring():
    """'gps_cog' 가 접두가 아니라 다른 곳에 있거나, 콜론 없는 문자열은 무시."""
    assert gate_off_source("imu_relative:OK note=gps_cog: gate=off") is None
    assert gate_off_source("gps_cog gate=off") is None
    assert gate_off_source("gps_cog_x:OK gate=off") is None


def test_gate_token_exact():
    """gate=off 는 공백 토큰으로만 — 'gate=offset' 같은 우연 일치는 실패 아님."""
    assert gate_off_source("gps_cog:OK gate=offset") is None


_HEADING_PKG = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'ssf_heading', 'ssf_heading'))

# GATE_CHECKED_SOURCES 의 각 이름 ↔ heading_logic.py 의 상수 정의 줄
_SOURCE_CONST = {"cog_offset": "SRC_COG_OFFSET", "gps_cog": "SRC_GPS_COG"}


def _read_heading_file(name):
    path = os.path.join(_HEADING_PKG, name)
    if not os.path.exists(path):
        pytest.skip(f"ssf_heading 소스 없음(패키지 단독 체크아웃?): {path}")
    with open(path, encoding='utf-8') as f:
        return f.read()


def test_checked_sources_match_heading_logic_constants():
    """소스 이름이 yaw_mux 쪽 상수 정의 줄과 철자까지 같아야 한다(어긋나면 검사가 조용히 꺼진다)."""
    text = _read_heading_file('heading_logic.py')
    assert set(GATE_CHECKED_SOURCES) == set(_SOURCE_CONST)
    for src, const in _SOURCE_CONST.items():
        pat = rf'^{const}\s*=\s*"{re.escape(src)}"'
        assert re.search(pat, text, re.M), (const, src)


def test_gate_off_token_format_in_yaw_mux():
    """검사가 기대는 토큰 형식(공백 + gate=off)을 yaw_mux 쪽 두 발행 경로가 그대로 쓰는지."""
    for name in ('heading_logic.py', 'yaw_mux.py'):
        text = _read_heading_file(name)
        assert '" gate=off"' in text, name

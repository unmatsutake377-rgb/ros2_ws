#!/usr/bin/env python3
"""설정 파일 정합성 정적 테스트 — ROS 없이 돈다.  tools/run_tests.sh ssf_tools

계약서_OAK전환_20261104 §6-a: ssf_tools.yaml(healthcheck) 의 image_topic == vision.yaml(검출기) 의 image_topic.
한쪽만 바꾸면 healthcheck 가 아무도 안 보는 토픽을 감시해 CAM 이 영원히 WAIT/NEVER 이거나,
반대로 검출기가 침묵하는데 CAM ✅ 가 뜬다 — 런북 §4 "두 곳에 넣는다" 를 기계적으로.
"""

import os
import re

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.normpath(os.path.join(_HERE, '..', '..'))
_TOOLS_YAML = os.path.join(_HERE, '..', 'config', 'ssf_tools.yaml')
_VISION_YAML = os.path.join(_SRC, 'color_shape_detector', 'config', 'vision.yaml')
_HEALTHCHECK = os.path.join(_HERE, '..', 'ssf_tools', 'healthcheck.py')

OAK_IMAGE_TOPIC = "/oak/rgb/image_raw"   # 런북 10-31 실측(depthai name=oak)


def _yaml_str(path, key):
    """`key: "값"` 한 줄(pyyaml 없이, 블록 무관). 없으면 None."""
    m = re.search(r'^\s*%s:\s*["\']?([^"\'\s#]+)' % re.escape(key),
                  open(path, encoding='utf-8').read(), re.M)
    return m.group(1) if m else None


def _yaml_num(path, key):
    m = re.search(r'^\s*%s:\s*([-0-9.]+)' % re.escape(key), open(path, encoding='utf-8').read(), re.M)
    return float(m.group(1)) if m else None


def test_image_topic_same_in_tools_and_vision_yaml():
    tools = _yaml_str(_TOOLS_YAML, "image_topic")
    vision = _yaml_str(_VISION_YAML, "image_topic")
    assert tools is not None and vision is not None, f"image_topic 없음: tools={tools} vision={vision}"
    assert tools == vision, f"ssf_tools.yaml {tools} ≠ vision.yaml {vision} — healthcheck 가 엉뚱한 토픽을 본다"


def test_image_topic_is_oak():
    assert _yaml_str(_TOOLS_YAML, "image_topic") == OAK_IMAGE_TOPIC


def test_healthcheck_declares_grace_param_and_yaml_sets_it():
    """image_wait_grace_s: 코드가 선언하고(미선언 키는 rclpy 가 버린다) yaml 이 OAK 연결 시간(13~17 s)보다 길게 준다."""
    src = open(_HEALTHCHECK, encoding='utf-8').read()
    assert re.search(r'declare_parameter\(\s*["\']image_wait_grace_s["\']', src), \
        "healthcheck.py 가 image_wait_grace_s 를 declare_parameter 하지 않는다"
    grace = _yaml_num(_TOOLS_YAML, "image_wait_grace_s")
    assert grace is not None, "ssf_tools.yaml 에 image_wait_grace_s 없음"
    assert grace > 17.0, f"유예 {grace}s ≤ OAK 연결 상한 17 s — 정상 부팅도 CAM ✗ 로 찍힌다"
    timeout = _yaml_num(_TOOLS_YAML, "image_timeout_sec")
    assert timeout is not None and grace > timeout


def test_healthcheck_code_default_equals_yaml():
    """코드 기본 image_topic(yaml 이 안 닿는 단독 실행) == ssf_tools.yaml — 결정 3(코드 기본값도 OAK)."""
    src = open(_HEALTHCHECK, encoding='utf-8').read()
    m = re.search(r'declare_parameter\(\s*["\']image_topic["\']\s*,\s*["\']([^"\']+)["\']\s*\)', src, re.S)
    assert m, "healthcheck.py 에 declare_parameter(\"image_topic\", <문자열>) 가 없다"
    assert m.group(1) == _yaml_str(_TOOLS_YAML, "image_topic"), \
        f"healthcheck 코드 기본 {m.group(1)} ≠ ssf_tools.yaml {_yaml_str(_TOOLS_YAML, 'image_topic')}"

<!-- ─────────────────────────────────────────────────────────────
  🚨 SSF 내재화(vendoring) 기록 — 2026-07-23
  micro-ROS-Agent — 아두이노 ↔ ROS2 브리지 에이전트

  원본 upstream 커밋: 1cb265e752f73003a37cb66ad849f6ef1d9ce695

  왜 여기 있나:
    이 패키지는 원래 **gitlink(서브모듈 포인터, mode 160000)** 로 커밋돼 있었다.
    그런데 `.gitmodules` 가 없어서 **받아올 URL 이 없었다** — 로컬엔 파일이 다 있는데
    다른 머신에서 fresh clone 하면 **빈 폴더**로 온다. 라이다·아두이노 통신 경로가
    통째로 비는 것이라 멀티머신 작업에서 반복 사고가 났다.
    → 서브모듈을 풀고 파일을 저장소에 직접 넣었다. 드라이버 자체는 **바꾸지 않았다**
      (동작 변화 0). 위 해시가 이후 업스트림 패치 대조의 유일한 단서다.

  원칙: 이 디렉터리 내부는 수정하지 않는다.
        부득이 수정하면 그 지점과 이유를 이 블록 아래에 적을 것.
────────────────────────────────────────────────────────────── -->

# micro-ROS Agent

[![GitHub license](https://img.shields.io/github/license/microROS/micro-ROS-Agent.svg)](https://github.com/microROS/micro-ROS-Agent)
[![GitHub release](https://img.shields.io/github/release/microROS/micro-ROS-Agent.svg)](https://github.com/microROS/micro-ROS-Agent/releases)

ROS 2 package using Micro XRCE-DDS Agent.

## Overview

This repository contains the Micro-ROS Agent package.
Micro-ROS Agent is a ROS 2 node that wraps the Micro XRCE-DDS Agent.
For further information about Micro XRCE-DDS Agent click [here](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
This package is a part of the Micro-ROS project stack.
For more information about Micro-ROS project click [here](https://micro-ros.github.io/).

The node acts as a server between DDS Network and Micro-ROS nodes inside MCU.
It receives and send messages from Micro-ROS nodes, and keep track of the Micro-ROS nodes exposing them to the ROS 2 network.
The node interacts with DDS Global Data Space on behalf of the Micro-ROS nodes.

## Package features

### XML generation

During the build process, the package looks for all ROS 2 messages to generate an initial list of XML profiles.
These profiles can are referenced in the Agent-Client communication to avoid sending the full XML content.
This reference mechanism can be switched on and off from the Micro XRCE-DDS middleware layer.

### Agent-Client communication mechanism

Communication between the Micro-ROS Agent and the Micro-ROS nodes supports two types of transport:

- UDP and TCP over IPv4 and IPv6.
- Serial Port transports.

All available configurations are supported directly by the Micro XRCE-DDS agent.

## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open-source components included in this repository,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

Please notice the following issues/limitations:

* There is an unknown issue when dealing with serial ports shared with the micro-ROS agent running inside a Docker. Sometimes it works with a remarkable packet loss.
* There is an known issues with serial port communication on micro-ros-agent snap version. It is recommended to use the dockerized version or build it from source.
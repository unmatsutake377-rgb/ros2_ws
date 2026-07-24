<!-- ─────────────────────────────────────────────────────────────
  🚨 SSF 내재화(vendoring) 기록 — 2026-07-23
  micro_ros_msgs — micro-ROS 메시지 정의

  원본 upstream 커밋: 10be4d005fbc7d8dd60dbb213b65f4171419bfe9

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

# micro_ros_msgs

## Summary

Collection of ROS 2 message definitions used throughout the implementation of micro-ROS, both in the server ([micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent/)) and client ([micro-ROS RMW](https://github.com/micro-ROS/rmw-microxrcedds)) endpoints.

## Purpose of the Project

This software is not ready for production use.
It has neither been developed nor tested for a specific use case.
However, the license conditions of the applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software fulfills your requirements and adjust it according to any applicable safety standards, e.g., ISO 26262.

## License

This repository is open-sourced under the Apache-2.0 license. See the LICENSE file for details.

## Known Issues/Limitations

There are no known limitations.

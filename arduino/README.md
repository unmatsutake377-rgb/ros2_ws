# arduino — 회로(Arduino Due) 펌웨어

배에 탑재되는 Arduino Due 코드. 설계 근거와 결정 사항은 `docs/펌웨어_설계문서.md` 참고.

**컴파일 검증: ✅ 통과 (2026-07-22, Arduino IDE 2.x + SAM Boards 1.6.12 + micro_ros_arduino humble + platform.txt 패치).**
실기 동작 검증은 아직 — 벤치 테스트 대기 (docs/펌웨어_설계문서.md §6 목록).

- `ssf_boat/ssf_boat.ino` — 펌웨어 본체 (Arduino IDE로 열어서 업로드)
- 업로드: Arduino IDE → 보드 "Arduino Due (Programming Port)" → Programming 포트에 USB 연결 → 업로드
- 포트 역할: **Programming 포트 = 업로드 + 노트북 micro-ROS 에이전트 연결** / Native 포트 = 디버그 출력(시리얼 모니터)
- 노트북과의 토픽 계약: `Motor_run` 구독(Int32, pwm_r*10000+pwm_l, 1500=중립), `/firmware_status` 발행(Int32MultiArray, 10Hz)
- 배 A/B 구분은 코드가 아니라 **배에 달린 ID 핀(DIP 스위치)** 으로 자동 인식 — 펌웨어는 두 배 공용 1벌

## ⚠️ 새 컴퓨터에서 컴파일 시 필수 패치 (undefined reference 에러 대책)

Due의 SAM 보드 패키지가 오래돼서 micro-ROS의 precompiled 라이브러리를 링크 못 함.
`undefined reference to rclc_...` 에러가 뜨면 아래 파일을 패치:

`C:\Users\<사용자>\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\platform.txt`

1. 아무 곳(예: "# SAM3 compile patterns" 아래)에 한 줄 추가:
   `compiler.libraries.ldflags=`
2. `recipe.c.combine.pattern=` 줄에서 `"{build.path}/{archive_file}"` 바로 뒤에
   `{compiler.libraries.ldflags}` 삽입 (`-Wl,--end-group` 앞)

SAM 패키지를 업데이트/재설치하면 패치가 지워짐 — 같은 에러 재발 시 다시 적용.

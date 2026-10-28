# 지시서 02 — GPS (RTK · 웨이포인트 · 지오펜스)

## 담당 목차와 분량

| 목차 | 지금 원고 | 할 일 |
|---|---|---|
| Ⅲ-1.3 센서 표의 GPS 줄 | 한 줄 | 장비 사진 1장(A·B 각각), 용도 문장 보강 |
| **Ⅲ-2.2 GPS RTK** | 글머리표 2개 | **주 담당.** 공통 양식 8항목으로 확장. 아래 소절 구성 권장 |
| Ⅲ-2.3 의 지오펜스 문장 | 한 줄 | 지오펜스 **생성**은 여기서 씀. 병합은 LiDAR 담당 |

권장 소절: 2.2.1 수신기와 RTK 보정 · 2.2.2 fix 검증과 필터 · 2.2.3 웨이포인트와 LOS 유도 · 2.2.4 지오펜스

분량: **A4 2~2.5쪽** (표 3~4개, 그림 1~2장). 네 담당 중 코드가 가장 많다.

## 핵심 한 줄

GPS 는 위치를 받아 **다음 웨이포인트로 가는 방위**와 **현재 임무 번호(`/wp_mode`)**를 내고, 경기장 경계를 **가짜 LiDAR 광선**으로 바꿔 내보낸다. 올해는 fix 가 없는데도 항법하던 사고를 막는 검증, 이상치 필터, 경로선 추종(LOS)을 새로 넣었다.

## 반드시 답할 질문

1. A배·B배 수신기는 각각 무엇이고, 드라이버 설정 파일은 어느 것을 쓰나?
2. RTK 보정은 어떤 경로로 들어오나? `ntrip_client` → RTCM → 수신기. 계정은 어디서 읽나(경로만)?
3. 우리가 드라이버 설정에서 바꾼 것은? 해상용 동역학 모델, 토픽 이름 불일치, 배별 설정 인자, EEPROM 저장 제거.
4. `north_goal_angle` 은 무엇을 구독하고 무엇을 발행하나? 토픽·타입·주기.
5. fix 가 없을 때 작년 코드는 무엇을 했나? Null Island 사고와 실측값 47.96°, `gps_guard.py` 가 막는 방법.
6. `gps_filter.py` 의 세 단계(공분산·점프·스무딩)는 각각 무엇을 버리나? 파라미터 값은?
7. 웨이포인트 파일은 어떤 형식이고, 잘못된 좌표를 어떻게 거부하나(`waypoint_loader.py`)?
8. 대회장에서 좌표를 어떻게 녹화하나(`waypoint_recorder.py`, 녹화 절차 문서)?
9. LOS·ILOS 유도는 무엇을 계산하나? 경로선 이탈 거리, 전방주시 거리, 적분, 각 파라미터.
10. `/wp_mode` 와 `/candidate_angle` 은 언제 무엇을 내나? 담당 노드가 없는 모드(5, 8)의 폴백 20000 은 왜 그 모드에만 남겼나?
11. 지오펜스는 왜 원뿔이 아니라 **광선** 방식인가? `/geofence_state` 배열 형식은? IMU 가 끊기면?
12. `require_boat_mode_auto` 는 무엇을 막나?
13. 단위 테스트는 몇 개이고 무엇을 검증하나?
14. 아직 실측이 없는 것은? (야외 측위를 아직 한 번도 못 했다 — 과제 F1)

## 읽을 파일

| 파일 | 역할 | 우선 |
|---|---|---|
| `src/north_goal_angle/north_goal_angle/north_goal_angle.py` | 웨이포인트 상태기계, 방위 발행, 지오펜스. 맨 위 설명이 길고 중요 | ★★★ |
| `src/north_goal_angle/north_goal_angle/gps_guard.py` | fix 유효성 판정 | ★★★ |
| `src/north_goal_angle/north_goal_angle/gps_filter.py` | 공분산·점프·스무딩 | ★★★ |
| `src/north_goal_angle/north_goal_angle/los_logic.py` | LOS·ILOS 유도 | ★★★ |
| `src/north_goal_angle/config/north_goal_angle.yaml` | 파라미터 값 | ★★★ |
| `src/north_goal_angle/north_goal_angle/waypoint_loader.py`, `waypoint_recorder.py`, `config/waypoints.yaml` | 좌표 파일 검증·녹화 | ★★ |
| `src/ublox/ublox_gps/config/c94_m8p_rover.yaml` (A배), `zed_f9p_rover.yaml` (B배) | 수신기 설정. **우리가 고친 항목만** | ★★ |
| `src/ntrip_client/launch/ntrip_client_launch.py`, `config/ntrip_credentials.example.yaml` | 보정 수신 실행, 계정 파일 양식(값 없음) | ★ |
| `src/ublox/ublox_gps/launch/ublox_gps_node-launch.py`, `src/north_goal_angle/launch/north_goal_angle_launch.py`, `src/launch_files/launch/launch_files.launch.py` | 실행 설정 | ★ |
| `src/north_goal_angle/test/` | 단위 테스트 6개 파일 | ★ |

관련 문서: `CLAUDE.md` 3장·8장, `docs/전달용/팀원전달_GPS수정_병합안내.md`, `docs/전달용/회신_GPS코드_기능추출_수정제안.md`, `docs/전달용/회신_LOS강화_IMU추가_검토.md`, `docs/절차/웨이포인트_녹화절차.md`

## 코드 출처

| 패키지 | 출처 | 작년부터 | 올해 바꾼 것 (근거 커밋) |
|---|---|---|---|
| `ublox` | **외부.** u-blox ROS 드라이버(KumarRobotics 계열, `package.xml` 원저자 Gareth Cross) | 있음 | B배 F9P 설정 신설·배별 인자(`c1b0d81`), 토픽명 수정·해상 모델(`ed2a144`), 배별 설정 전달·EEPROM 저장 제거(`c7c0c59`) |
| `ntrip_client` | **외부.** LORD-MicroStrain `ntrip_client` (`package.xml` URL) | 있음 | 실행 경로 수정(`89818a5`), **계정을 저장소 밖 `~/.ssf/` 로**(`f98a8f9`) |
| `north_goal_angle` | 작년 우리 코드 | 있음 | fix 검증(`02944d8`), LOS·필터(`9221750`), 지오펜스, 웨이포인트 파일화 등 |

## 작년 대비 변경 보는 명령

```bash
cd ~/ros2_ws
git log --format='%h %ad %s' --date=short 9bff661..HEAD -- src/north_goal_angle src/ublox src/ntrip_client
git diff 9bff661 HEAD -- src/north_goal_angle src/ublox
```

🚨 **`src/ntrip_client` 는 diff 를 보지 않는다.** 작년 실행 파일에 NTRIP 계정이 평문으로 있었고, diff 의 지운 줄에 그 값이 그대로 다시 나온다. 커밋 메시지만 근거로 쓴다. 이 값은 AI 에도 원고에도 넣지 않는다.

## 경계

| 내용 | 누가 | 이 절에서는 |
|---|---|---|
| 지오펜스 광선 **생성** | **GPS** | 전부 씀 |
| 지오펜스를 실제 스캔과 합치는 `min` 병합, 정지 판정 | LiDAR | "ship_direction 이 실제 스캔과 합친다" 한 줄 |
| COG 로 IMU 헤딩 오프셋 잡기 | IMU | "COG 는 IMU 담당 절 참고" 한 줄 |
| 전체 임무 상태기계(Ⅲ-3) | 팀장 | `/wp_mode` 를 **누가 어떻게 바꾸는지**만 씀 |
| 위치유지 `ship_back` 의 RTK 폴백 | 팀장 | 언급만 |

## A배·B배

| | A배 (1번배) | B배 (2번배) |
|---|---|---|
| 수신기 | u-blox C94-M8P + ANN-MB | u-blox ZED-F9P + ANN-MB |
| 설정 파일 | `c94_m8p_rover.yaml` | `zed_f9p_rover.yaml` |

파라미터와 노드 코드는 같다. 수신기 사양(채널, 정밀도, 갱신율)은 데이터시트에서 가져오고 문서명을 출처로 단다.

## 빈칸으로 둘 실측

| 항목 | 언제 |
|---|---|
| 첫 fix 까지 걸린 시간, 위성 수 | 첫 야외 측위 (과제 F1) |
| RTK FLOAT / FIXED 위치 산포(m) | 같은 날, 정지 5분 기록 |
| 웨이포인트 도착 오차 | 물 위 시험 |
| 대회장 웨이포인트·경계 좌표 | 대회 당일 녹화 (과제 D7) |

## 그림 제안

1. `ntrip_client → 수신기 → /ublox_gps_node/fix → gps_guard → gps_filter → LOS → /north_goal_angle_tp, /candidate_angle, /wp_mode` 흐름도
2. 지오펜스 광선 그림: 경기장 모서리에서 원뿔 방식은 정면이 뚫리고 광선 방식은 막히는 비교 (`north_goal_angle.py` 맨 위 설명에 예시가 있다)

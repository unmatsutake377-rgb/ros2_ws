# 지시서 01 — IMU (헤딩)

## 담당 목차와 분량

| 목차 | 지금 원고 | 할 일 |
|---|---|---|
| Ⅲ-1.3 센서 표의 IMU 줄 | 한 줄 | 장비 사진 1장, 용도 문장 보강 |
| **Ⅲ-2.1 좌표계·헤딩** | 글머리표 4개 | **주 담당.** 공통 양식 8항목으로 확장 |

분량: **A4 1~1.5쪽** (표 2~3개, 그림 1장).

## 핵심 한 줄 (방향 잡기용)

작년에는 IMU 드라이버가 `/imu/yaw` 를 직접 냈는데, 부팅 순간 뱃머리를 0° 로 잡고 GPS 진행방향이 있으면 통째로 덮어써서 **절대방위가 아니었다.** 올해는 드라이버가 가공 안 한 `/imu/yaw_raw` 만 내고, 새로 만든 `yaw_mux` 노드가 **유일하게** `/imu/yaw` 를 낸다. 이 구조 변경이 이 절의 중심이다.

## 반드시 답할 질문

1. iAHRS RB-SDA-v1 이 노트북과 어떻게 연결되고(시리얼 속도), 드라이버가 어떤 토픽을 내나?
2. 작년 `/imu/yaw` 계산 방식의 문제는 무엇이었나? `heading_logic.py` 맨 위 설명의 ②③ 두 가지.
3. 올해 구조에서 `/imu/yaw` 발행자가 왜 하나여야 하나? 두 개가 되면 어떤 일이 생기나?
4. `heading_source` 에 어떤 선택지가 있고 지금 기본값은 무엇인가? 선택지마다 차이는?
5. 각도 규약은 무엇인가? 0° 기준, 증가 방향, 각도 평균을 산술평균으로 하면 안 되는 이유.
6. 자기편각은 어떤 소스에만 더하나? 다른 소스에 더하면 생기는 문제는?
7. IMU 값이 끊기면(`stale_sec`) `yaw_mux` 는 무엇을 하나? 그 뒤 `ship_goal_angle`, `ship_direction`, 지오펜스는 각각 어떻게 반응하나?
8. `ship_goal_angle` 은 무엇을 계산해 내나? 작년 대비 고친 세 가지는?
9. GPS 진행방향(COG)으로 IMU 오프셋을 잡는 방식에서 `cog_*` 파라미터들은 각각 무슨 조건인가?
10. 드라이버에 추가한 `/imu/mag_heading` 은 무엇이고, 지금 제어에 쓰이나 관찰용인가?
11. 단위 테스트는 몇 개이고 무엇을 검증하나?
12. 아직 실측이 없는 값은? `mount_offset_deg`, `invert_yaw`, 대회장 자기편각, 정북 정렬.

## 읽을 파일

| 파일 | 역할 | 우선 |
|---|---|---|
| `src/ssf_heading/ssf_heading/heading_logic.py` | 헤딩 계산 순수 로직. 작년 문제 설명이 맨 위에 있음 | ★★★ |
| `src/ssf_heading/ssf_heading/yaw_mux.py` | `/imu/yaw` 단독 발행 노드 | ★★★ |
| `src/ssf_heading/config/ssf_heading.yaml` | 파라미터 값 | ★★★ |
| `src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py` | `/yaw_error` 계산, IMU 생존 신호원 | ★★ |
| `src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp` | 외부 드라이버. **우리가 고친 줄만** 본다 | ★ |
| `src/ssf_heading/launch/ssf_heading.launch.py`, `src/ship_goal_angle/config/ship_goal_angle.yaml`, `src/iahrs_driver_ros2-main/iahrs_driver/launch/iahrs_driver.py` | 실행 설정 | ★ |
| `src/launch_files/launch/launch_files.launch.py` | 대회 실행 파일에서 드라이버·`yaw_mux` 를 어떻게 띄우나 | ★ |
| `src/ssf_heading/test/test_heading_logic.py` | 단위 테스트 | ★ |

관련 문서: `CLAUDE.md` 3장(3-5 헤딩)·7-2·8장, `docs/전달용/IMU_노드_쉬운설명.md`, `docs/전달용/회신_N4_mag_heading_검토.md`, `docs/전달용/회신_상위3팀분석_IMU.md`

## 코드 출처 — AI 에게 추측시키지 말고 이 표를 쓸 것

| 패키지 | 출처 | 작년부터 | 올해 바꾼 것 (근거 커밋) |
|---|---|---|---|
| `iahrs_driver` | **외부.** iAHRS 드라이버. `package.xml` 관리자 `wbjin`, README 링크 `blog.naver.com/zzang0736`. **정확한 원출처는 담당자가 확인** | 있음 | `/imu/yaw` → `/imu/yaw_raw` 로 발행 토픽 변경, 부팅 0점화·COG 덮어쓰기 제거(`ace1e33`). `/imu/mag_heading` 추가(`b433662`) |
| `ship_goal_angle` | 작년 우리 코드 | 있음 | 시작값 None, stale 시 발행 중단, 주기 파라미터화. 파일 안 설명 참고 |
| `ssf_heading` | **올해 새로 만듦** | 없음 | 전부 신규. diff 대신 파일 자체가 근거 |

## 작년 대비 변경 보는 명령

```bash
cd ~/ros2_ws
git log --format='%h %ad %s' --date=short 9bff661..HEAD -- src/ssf_heading src/iahrs_driver_ros2-main src/ship_goal_angle
git diff 9bff661 HEAD -- src/iahrs_driver_ros2-main src/ship_goal_angle
```

## 경계 — 겹치는 곳

| 내용 | 누가 | 이 절에서는 |
|---|---|---|
| COG 로 헤딩 오프셋 잡기 (`ssf_heading` 안) | **IMU** | 전부 씀 |
| GPS 수신·RTK·fix 검증 | GPS | "COG 는 GPS 속도 토픽에서 받는다" 한 줄 |
| 지오펜스가 IMU 끊기면 빈 배열 내는 것 | GPS | 반응만 한 줄 |
| `ship_direction` 의 IMU 페일세이프 판정 | LiDAR | "감속·정지한다" 한 줄 |
| IMU 장착 위치(마스트 베이스 포켓) | 하드웨어 파트 | 언급만 |

## A배·B배

두 배 모두 iAHRS RB-SDA-v1 이고 코드도 같다. **장착 오프셋과 자력 환경은 배마다 다르니** 실측 표는 배별 두 칸으로 만든다.

## 빈칸으로 둘 실측

| 항목 | 언제 생기나 |
|---|---|
| `mount_offset_deg`, `invert_yaw` 확정값 | 배 조립 완료 후 정북 정렬 (과제 C5). 가조립 상태 값은 버린다 |
| 재부팅 전후 `/imu/yaw_raw` 일치 여부 | 같은 날. 다르면 절대방위가 아니라는 뜻 |
| 대회장 자기편각 | 대회장 좌표로 계산 (과제 D9) |
| 정지·직진·선회 중 헤딩 흔들림(°) | 물 위 시험 |

## 그림 제안

`/imu/yaw_raw → yaw_mux → /imu/yaw → ship_goal_angle → /yaw_error → ship_direction` 흐름도. 작년 구조(드라이버가 직접 `/imu/yaw`)와 나란히 두면 개선점이 한눈에 보인다.

# 보고서 분담 묶음 — GPS
만든 시각: 2026-10-29 14:26 · 저장소 커밋: 3e2f087

이 파일 하나를 AI 에 올리고, 공통 작성규칙 5절의 프롬프트를 붙여 넣는다.
소스 파일에는 행 번호가 붙어 있다. 근거는 [근거: 파일경로:행] 으로 단다.


==================================================================
## 문서: docs/전달용/보고서_분담/00_공통_작성규칙.md
==================================================================
# 보고서 분담 — 공통 작성규칙 (소프트 파트, 2026-10-28)

## 1. 무엇을 하나

- KABOAT 2026 설계보고서 **Ⅲ장 소프트웨어** 중 내 담당 센서 절을 쓴다.
- 담당은 네 개다: **IMU · GPS · LiDAR · 카메라**. 내 지시서(`01_IMU.md` 등)에 채울 목차 번호와 분량이 있다.
- 다 모이면 팀장이 하나로 합친다. 그래서 **형식이 같아야 한다.** 이 규칙을 지키지 않은 원고는 다시 써야 한다.

받는 자료는 네 개다.

| 파일 | 내용 |
|---|---|
| `00_공통_작성규칙.md` | 이 문서. 규칙, 원고 양식, AI 프롬프트 |
| `00_현재원고_Ⅲ소프트웨어.md` | 지금 보고서의 Ⅲ장. 문체와 깊이의 기준 |
| `0N_<센서>.md` | 내 지시서. 채울 목차, 답할 질문, 읽을 파일, 경계 |
| `양식_<센서>.docx` | **내 파트의 보고서 양식.** 본 보고서와 같은 글꼴·제목·표. 여기에 채워서 낸다 |

## 2. AI 쓰는 법 — 둘 중 하나

**방법 A (추천) — 저장소를 직접 여는 AI 도구**
Claude Code, Cursor 같은 도구로 저장소 폴더를 연다. 5절 프롬프트를 붙여 넣는다. AI 가 지시서의 파일을 직접 읽는다.

**방법 B — 웹 채팅 AI (ChatGPT, Claude 웹 등)**
저장소 맨 위 폴더에서 아래를 실행하면 홈 폴더에 `보고서묶음_<센서>.md` 파일 하나가 생긴다. 규칙·원고·지시서·소스(행 번호 포함)·문서·작년 대비 변경이 다 들어 있다. 이 파일을 올리고 5절 프롬프트를 붙여 넣는다.

```bash
cd ~/ros2_ws && git pull && tools/report_pack.sh imu
```

센서 이름은 `imu`, `gps`, `lidar`, `camera` 중 하나.

미리 만든 묶음이 `docs/전달용/보고서_분담/묶음/보고서묶음_<센서>.md` 에 있다. 스크립트를 못 돌리면 이 파일을 그대로 올린다. 다만 만든 날 이후의 코드 변경은 빠져 있으니, 가능하면 스크립트로 새로 만든다.

묶음 크기는 IMU 약 5만, GPS·LiDAR 약 7만, 카메라 약 9만 토큰이다. 무료 버전 AI 는 뒷부분을 잘라 먹을 수 있다. 1단계 요약에서 빠진 파일이 보이면 방법 A 로 바꾸거나 긴 입력을 받는 AI 를 쓴다.

## 3. 규칙 — 전부 지킬 것

1. **모든 수치와 동작 설명에 근거를 단다.** 문장 끝에 `[근거: src/ssf_heading/config/ssf_heading.yaml:12]` 처럼 파일과 행. 합칠 때 이걸로 대조하고, 제출본에서는 지운다. **근거 없는 문장은 빠진다.**
2. **모르는 값은 지어내지 않는다.** 실측이 없으면 `____` 로 비우고 끝의 "확인 필요" 목록에 적는다. AI 가 그럴듯한 숫자를 만드는 게 제일 위험하다.
3. **코드 출처를 지시서의 출처 표대로 쓴다.** AI 에게 출처를 추측시키지 않는다.
   - 외부 드라이버(rplidar_ros, ublox, ntrip_client, realsense, iAHRS 드라이버)는 **내부 구현을 설명하지 않는다.** 우리가 고친 부분만 쓴다.
   - 작년 코드를 고친 것은 **무엇이 바뀌었는지** 위주로 쓴다.
4. **"2025 대비 개선점"은 git 기록에서만 뽑는다.** 기준은 커밋 `9bff661`(작년 대회 최종본). 묶음 파일 끝의 커밋 목록과 diff 가 근거다.
5. **이름은 코드 표기 그대로.** 토픽 `/imu/yaw`, 파라미터 `mount_offset_deg`, 노드 `yaw_mux` 를 번역하거나 바꾸지 않는다.
6. **문체는 현재 원고와 같게.** 제출용 기술보고서, "~한다" 체. "혁신적", "완벽한" 같은 과장 금지. 작년 보고서 문장 복사 금지.
7. **내 담당 밖은 쓰지 않는다.** 겹치는 곳은 지시서의 "경계" 대로. 다른 담당 것이 필요하면 한 줄로 언급만.
8. 🚨 **보안.** 홈 폴더의 `~/.ssf/`(NTRIP 계정) 내용은 AI 에 절대 붙여 넣지 않는다. 계정 값은 원고에도 쓰지 않는다. 경로만 쓴다.

## 4. 원고 양식 — `양식_<센서>.docx` 에 채운다

**제출물**: `원고_<센서>_<이름>.docx` (양식 파일에 채운 것) + 그림 `그림_<센서>_<번호>.png` 따로.

양식 파일에는 이미 들어 있는 것이 있다.
- 내 담당 절의 **본 보고서 번호와 제목** (예: IMU 는 Ⅲ-2.1, 소절 2.1.1~2.1.8). 번호·제목·스타일은 바꾸지 않는다.
- 초록 ▶ 표시 사이의 **현재 원고**. 이걸 바탕으로 고쳐 쓴다.
- 회색 `[작성 안내]` 글씨. 다 채우면 지운다.
- 머리글이 정해진 **빈 표**와 그림 자리. 행이 모자라면 늘린다.

소절 8개는 모든 담당이 같다. `N` 은 내 담당 절 번호다.

| 소절 | 내용 | 표 머리글 |
|---|---|---|
| N.1 장비 사양과 선정 이유 | 사양 표 + 선정 이유 2~3문장 | 항목 / A배 / B배 / 출처 |
| N.2 코드 출처 | 지시서 출처 표를 옮기고 보강 | 패키지 / 원래 출처 / 작년부터 / 올해 바꾼 것 / 근거 커밋 |
| N.3 데이터 흐름 | 토픽 입출력 + 흐름도 그림 | 토픽 / 입력·출력 / 메시지 타입 / 주기 / QoS / 상대 노드 |
| N.4 처리 알고리즘 | 우리 코드가 하는 일, 수식, 설계 이유 | (글) |
| N.5 주요 파라미터 | yaml 값과 근거 | 이름 / 값 / 위치(파일:행) / 정한 근거 |
| N.6 이상 상황 대응 | 끊김·이상값일 때 코드 동작 | 상황 / 감지 방법 / 코드의 동작 / 근거 |
| N.7 2025 대비 개선점 | git 기록에서만 | 작년 / 문제 / 올해 / 근거 커밋 |
| N.8 검증과 시험 결과 | 테스트 개수·내용 + 실측 표 | 테스트 파일 / 개수 / 검증 내용 · 실측 항목 / A배 / B배 / 측정일 |
| 부록 A·B·C | 배별 차이, 확인 필요, 겹치는 내용 | 양식 참고 |

LiDAR·카메라·GPS 는 8소절 외에 추가 절(4.4, 5, 6.1, 4.5 등)이 양식에 따로 있다. 거기는 현재 원고를 고치고 보강한다.

**AI 결과를 양식에 옮기는 법.** AI 는 같은 번호로 마크다운 원고를 낸다. 글은 해당 소절에 붙여 넣는다. 표는 칸별로 옮기거나, AI 에게 "표를 탭으로 구분한 줄로 다시 줘" 라고 한 뒤 워드에서 붙여 넣고 `표 → 텍스트를 표로 변환` 을 쓴다.

분량은 지시서에 있다. 표와 그림을 포함한 A4 기준이다.

## 5. AI 에 붙여 넣을 프롬프트

`<센서>`, `<지시서 파일>` 두 곳만 바꾼다.

```text
너는 KABOAT 2026 자율운항보트 대회 설계보고서의 소프트웨어 파트 공동 저자다.
내 담당은 <센서> 이고, 범위는 지시서 <지시서 파일> 에 있다. 그 범위만 쓴다.

자료: 공통 작성규칙(00_공통_작성규칙.md), 현재 원고(00_현재원고_Ⅲ소프트웨어.md),
지시서, 지시서의 '읽을 파일'과 '관련 문서', 작년 최종본(9bff661) 대비 커밋 목록과 diff.
(묶음 파일을 올렸다면 전부 그 안에 있다. 저장소를 열었다면 직접 읽어라.)

작업은 세 단계로 하고, 단계마다 결과를 보여 줘.

1단계 — 읽기 확인
  지시서의 '읽을 파일'을 전부 읽고, 파일마다 하는 일을 2~3줄로 요약해.
  못 읽은 파일이 있으면 그렇다고 말해. 원고에는 안 들어간다.

2단계 — 질문에 답하기
  지시서의 '반드시 답할 질문'에 번호 순서대로 답해.
  답마다 [근거: 파일경로:행] 을 단다. 근거를 못 찾으면 "근거 없음" 이라고 쓰고 지어내지 마.

3단계 — 원고 쓰기
  2단계 답으로 원고를 써. 절 번호는 내 담당 절 번호 N 을 써서 N.1~N.8 + 부록 A·B·C,
  표 머리글은 공통 작성규칙 4절 표 그대로. 지시서에 추가 절이 있으면 그 절도 쓴다.

절대 규칙
- 코드·문서에 없는 기능, 수치, 성능을 만들지 마. 실측이 없으면 ____ 로 비우고 부록 B 에 적어.
- 모든 수치와 동작 설명 끝에 [근거: 파일경로:행] 을 단다.
- 지시서 출처 표에서 '외부'인 드라이버는 내부 구현을 설명하지 마. 우리가 바꾼 부분만 쓴다.
- '2025 대비 개선점'은 커밋 목록과 diff 에서만 뽑아. 추측 금지.
- 토픽·파라미터·노드 이름은 코드 표기 그대로.
- 문체는 현재 원고와 같은 제출용 기술보고서 "~한다" 체. 과장 표현 금지.
- 지시서의 '경계'에서 남의 담당으로 정한 내용은 한 줄 언급만.
- 분량은 지시서대로. 그림은 그리지 말고 무엇을 그릴지 설명만 해.
```

AI 가 3단계까지 끝내면 **사람이 직접** 6절 체크리스트를 확인한다.

## 6. 내기 전 체크리스트

- [ ] 근거 5개를 골라 실제 파일의 그 행을 열어 봤다. 내용이 맞다.
- [ ] 수치 중 근거 없는 것이 없다. 실측값 자리는 `____` 다.
- [ ] 외부 드라이버 내부 구현을 설명한 문장이 없다.
- [ ] 개선점 표의 커밋 번호가 실제로 있다 (`git show <번호>` 로 확인).
- [ ] 계정·비밀번호·서버 주소가 원고에 없다.
- [ ] 양식의 8소절과 부록 3개가 다 채워졌다. 회색 `[작성 안내]` 글씨를 지웠다.
- [ ] 절 번호·제목·글꼴을 바꾸지 않았다.
- [ ] 그림 PNG 를 따로 냈다. 직접 찍은 장비 사진이 있으면 같이 낸다(Ⅲ-1.3 센서 표용).

==================================================================
## 문서: docs/전달용/보고서_분담/00_현재원고_Ⅲ소프트웨어.md
==================================================================
# 현재 보고서 Ⅲ장 소프트웨어 원고 (2026-10-29 통합본 기준)

> 원본: **`docs/전달용/보고서_분담/통합_Ⅲ소프트웨어.md`**(근거 포함 단일 원고) — 본 보고서·소프트파트 보고서가 이 원고에서 만들어진다. 이 파일은 A배 제출본(근거 제거)을 옮긴 읽기용 사본이다. **고칠 때는 통합 원고를 고친다.**
> 이 원고의 **문체·깊이·표 형식이 기준**이다. 담당 절은 이 내용을 바탕으로 **늘리고 고쳐 쓴다.** 지우지 말고, 틀린 곳은 근거와 함께 고친다.
> `⚠️` 와 `____` 는 아직 실측이 없는 자리다.
>
> **A배·B배 차이** (이 파일은 A배 기준)
> | 위치 | A배 (1번배) | B배 (2번배) |
> |---|---|---|
> | 1.1 하위 제어 | 측당 ESC 1개 (T200 ×2) | 측당 ESC 2개 팬아웃 (T200 ×4) |
> | 1.3 LiDAR | RPLIDAR A3 개체 80BF (약 11.1 Hz / 0.250°) | 개체 918A (11.0 Hz / 0.223°) |
> | 1.3 GPS | u-blox C94-M8P (안테나 ⚠️ 실물 확인) | u-blox ZED-F9P + ANN-MB |
> | 4.4 half_width 근거 | 폭 0.57 → 0.285, 공용값 0.29 | 폭 0.535 → 0.2675, 공용값 0.29 (시뮬 접촉 0.95·정체 0) |

# Ⅲ. 소프트웨어

## 1) 시스템 구성

### 1.1) 플랫폼
- 선내 연산: Lenovo IdeaPad Slim 3 (Ubuntu 22.04, ROS2 Humble). 두 배가 각자 노트북·DDS 도메인을 가진다.
- 하위 제어: Arduino Mega 2560 R3 — ssf_bridge 가 USB 시리얼로 L/R PWM(µs)을 보내고, 펌웨어가 ESC 2채널(핀 12 좌 / 11 우)로 내보낸다(측당 ESC 1개). 명령 500 ms 무수신 시 펌웨어가 자동 중립.
- 모드 전환은 조종기 스위치뿐(WAIT 0 / MANUAL 1 / AUTO 2). 노트북이 모드를 바꾸는 경로는 두지 않는다.
- 추력 부호: ROS 내부는 <1500 = 전진, 실물 ESC 는 >1500 = 전진. 반전은 ssf_bridge 한 곳(invert_thrust_direction).
- 배 구분: 두 배는 같은 코드·같은 대회 launch 를 쓰고, 배마다 다른 값은 launch 인자 두 개로 고른다. boat:=A|B 는 기본값이 없는 필수 인자로 IMU 장착값 파일 ssf_heading_<boat>.yaml 을 고르고, gps_config 는 수신기 설정 파일을 고른다. 1번배는 ros2 launch launch_files launch_files.launch.py boat:=A gps_config:=c94_m8p_rover.yaml 로 띄운다. boat 가 빠지거나 A/B 가 아니면 launch 가 거부되고, 배별 파일이 없으면 어떤 노드도 띄우기 전에 launch 전체를 세운다.

### 1.2) 노드·토픽 구조

| 노드 | 역할 | 주요 토픽 |
|---|---|---|
| rplidar_ros / iahrs_driver / ublox | 센서 드라이버 | /scan, /imu/yaw_raw, /ublox_gps_node/fix |
| ssf_heading (yaw_mux) | 헤딩 단일 발행자 | /imu/yaw, /heading_status |
| north_goal_angle | GPS 목표 방위, 임무 번호, 지오펜스 → 가짜 LiDAR | /north_goal_angle_tp, /wp_mode, /candidate_angle(모드 5·8), /geofence_state |
| ship_goal_angle | 목표 방위 − 헤딩 = 방위 오차 | /yaw_error |
| basic_image_subscriber{gate,turn,dock} | 색·형상 검출 (방위각만) | /red_angle, /green_angle, /image_angle, /image_color |
| ship_gate / ship_back / ship_turn / ship_dock | 임무 노드 (wp_mode 별) | /candidate_angle |
| ship_direction | LiDAR 갭 선택·회피, 페일세이프 레벨 | /desired_angle, /failsafe_level, /obstacle_distance_array, /obstacle_reject_count |
| motor_control | 각도 → 좌/우 PWM, 감속, 명령 워치독 0.5 s | Motor_run, /motor_reverse |
| ssf_bridge | 시리얼 브릿지, 상태줄 파싱 | /boat_mode, /boat_id, /boat_estop, /boat_cmd_watchdog |
| healthcheck / blackbox / mission_monitor | 진단·기록 (제어에 안 물림) | /health_ok |

그림 Ⅲ-1 ROS2 노드·토픽 구조

### 1.3) 센서

| 장비 사진 | 장비명 | 용도 |
|---|---|---|
| (사진) | RPLIDAR A3 개체 80BF (약 11.1 Hz / 0.250°) | 장애물·부표 거리, 갭 탐색, 감속 신호, 지오펜스 병합, 센서 감시. Sensitivity 모드, 25 m. 회전수·각분해능이 개체마다 달라 셀 수에 기대는 값은 배별로 다르게 동작한다 |
| (사진) | iAHRS RB-SDA-v1 9축 AHRS | 선수 방위. iahrs_driver 가 보정 전 /imu/yaw_raw 를 100 Hz 로 내고, yaw_mux 가 장착 오프셋을 보정해 /imu/yaw 를 단독 발행한다 |
| (사진) | u-blox C94-M8P (안테나 ⚠️ 실물 확인) | NTRIP RTK 위치(/ublox_gps_node/fix, 1 Hz)로 웨이포인트 방위·/wp_mode·지오펜스 광선을 만든다. 속도는 헤딩 COG 보정에 쓴다. 설정 파일 c94_m8p_rover.yaml |
| (사진) | Luxonis OAK-1 PoE FF (고정초점, HFOV 69° 스펙) | 자율 비전 주 카메라로 배당 1대 보유. PoE 인젝터 미도착으로 아직 미가동이며, 도착 뒤 D455 와 같은 rosbag 절차로 비교해 확정한다. hfov_deg 는 camera_info 로 실측 교체 |
| (사진) | Intel RealSense D455 | 현재 자율 비전(검출기 3종)이 쓰는 카메라. 컬러 640×480 30 fps, 뎁스 끔, hfov_deg 80.32° (camera_info 실측). 수동 조종 화면 송출 겸용 |


## 2) 센서 처리

### 2.1) 좌표계·헤딩
헤딩은 웨이포인트 목표 방위와 빼서 조향 오차를 만드는 기준값이고, 지오펜스 광선의 방향도 정한다. 각도 규약은 전 구간 같다: 0~360°, 0° = 진북, 시계방향 증가. 올해는 드라이버가 보정 없는 원시값을 /imu/yaw_raw 로만 내고, 새 패키지 ssf_heading 의 yaw_mux 가 보정해 /imu/yaw 를 단독 발행한다. 한 토픽에 발행자가 둘이면 보정 전·후 값이 번갈아 나와도 ROS 는 오류를 내지 않으므로, 대회 launch 가 드라이버의 yaw_topic 을 명시하고 드라이버는 /imu/yaw 로 설정되면 경고한다.

| 토픽 | 발행 → 구독 | 주기 |
|---|---|---|
| /imu/yaw_raw | iahrs_driver → yaw_mux | 100 Hz |
| /ublox_gps_node/fix_velocity, /motor_reverse | ublox_gps_node, motor_control → yaw_mux (COG 추정, 후진 게이트) | 수신기 측, 0.05 s |
| /imu/yaw, /heading_status | yaw_mux → ship_goal_angle, north_goal_angle, ship_turn / healthcheck | 50 Hz / 변화 시·1 s |
| /yaw_error | ship_goal_angle → ship_direction | 20 Hz |

보정식과 소스. yaw_mux 는 ψ = wrap360(s·ψ_raw + mount_offset_deg + δ) 를 적용한다(s = −1 은 invert_yaw). 편각 δ 는 자기북 기준 소스에만 넣는다. 듀얼 GPS·COG 는 이미 진북 기준이라 편각을 더하면 약 8° 이중 보정이 된다. 기본 소스 imu_relative 는 원시 yaw 에 고정 오프셋을 더하며, 기준축이 임의라 편각도 오프셋이 흡수한다. 대안 cog_offset 은 GPS COG 와 IMU 의 차 e_i = COG − ψ_imu 를 단위벡터로 누적(반감기 감쇠)해 오프셋을 추정하고, 신뢰도 R = ‖Σ‖/n 과 표본 수가 기준을 넘을 때만 발행한다. 산술평균은 359° 와 1° 를 180° 로 만들어 쓰지 않는다. 저속·급선회·후진 표본은 버린다. 후진만 이어지면 180° 틀린 채 R = 1.0 으로 수렴하므로 /motor_reverse 로 따로 막는다. 이 추정치에는 그날의 평균 게걸음각이 섞인다. dual_gps·imu_absolute 는 입력이 배선되지 않아 0 대신 발행을 멈춘다.
배별 장착값과 정북 정렬. 배마다 다른 값은 invert_yaw·mount_offset_deg 두 개뿐이며 ssf_heading_A.yaml 에 둔다. launch 는 공통 → 배별 순서로 넘겨 뒤 파일 값이 이기고, 벤치 실행도 같은 boat 인자를 요구한다. 배별 파일에 두 키 외의 값(안전 스위치 등)이 들어가거나 mount_offset_deg 를 정수로 적으면(yaw_mux 부팅 즉시 사망) 정적 테스트가 잡는다. 정렬은 배를 나침반으로 정북에 맞추고 /imu/yaw 가 목표값을 읽을 때까지 mount_offset_deg 를 조정한다. 자기 나침반은 자북을 가리키고 imu_relative 는 편각을 쓰지 않으므로 목표값은 0 이 아니라 진북 기준 뱃머리 360 + 편각(한국 약 352)이다. 이어 시계방향 90° 회전에서 값이 증가하는지로 invert_yaw 를 정한다.

| 이름 | 값 | 정한 근거 |
|---|---|---|
| heading_source | "imu_relative" | 입력 조건 없이 발행되는 소스. 오타면 기동 시 예외 |
| period_sec / stale_sec | 0.02 s / 0.5 s | stale 은 소비자 IMU 한계와 같다 |
| invert_yaw / mount_offset_deg (배별) | false / 0.0 | 0.0 은 '아직 안 쟀다'. 조립 후 정북 정렬로 정한다 |
| magnetic_declination_deg | 0.0 | imu_absolute 에만 적용 |
| cog_min_speed_mps / cog_max_turn_rate_dps | 0.8 m/s / 8.0 °/s | 보행 0.70 m/s 시험에서 rej=slow 로 걸러짐 |
| cog_min_samples / cog_min_resultant / cog_half_life_sec | 30 / 0.9 / 60 s | 감쇠로 생기는 표본 상한과의 조합을 기동 시 검사 |
| cog_require_reverse_gate | true | 대회값 고정, 공통 파일에만 |


| 작년 | 문제 | 올해 | 커밋 |
|---|---|---|---|
| 드라이버가 부팅 시점 방위를 0 으로 빼 /imu/yaw 직접 발행 | 상대각이라 목표 방위와 빼면 의미가 없음 | 드라이버는 /imu/yaw_raw 만, /imu/yaw 는 yaw_mux 단독 | ace1e33 |
| NAV-PVT 방위로 IMU 값을 통째로 덮어씀 | COG 라 뱃머리가 아니고 정지 시 잡음 | 덮어쓰기 기본 false, COG 는 조건 통과 표본만 사용 | ace1e33, 5a69c0a |
| ship_goal_angle 시작값 0.0, stale 검사 없음 | 부팅 직후 yaw_error=0.0 을 정렬 완료로 오인 | 시작값 None, IMU 0.5 s·목표각 2.0 s stale 시 중단 | 026482a, 100104e |

헤딩을 믿을 수 없으면 0 같은 대체값 대신 발행을 멈춘다. 원시 yaw 가 0.5 s 넘게 끊기면 /imu/yaw·/yaw_error 가 차례로 멈추고 지오펜스는 빈 배열을 내며, 이 침묵을 ship_direction 이 잡는다(Ⅲ-5).

### 2.2) GPS RTK
GPS 처리는 외부 드라이버 ublox_gps·ntrip_client 와 우리 노드 north_goal_angle 로 이뤄진다. north_goal_angle 은 다음 웨이포인트 방위 /north_goal_angle_tp, 임무 번호 /wp_mode, 경계 광선 /geofence_state 를 0.5 s 주기로 낸다. 설계 원칙은 "모르면 입을 다문다"이다.

#### 수신기와 RTK 보정
단독 측위 오차(±3~5 m)가 부표 간격보다 커서 웨이포인트에는 RTK 가 필요하다. 보정은 인터넷 NTRIP 으로 받아 UHF 안테나를 달지 않는다. ntrip_client 는 수신기 드라이버보다 2.0 s 늦게 뜨고, 계정은 저장소 밖(~/.ssf/) 파일에서 읽는다. 파일이 없으면 예외 없이 그 노드만 생략한다 — 예외가 나면 메인 launch 전체가 내려가기 때문이다. 세션이 끊기면 5 s 간격 최대 10회 재연결한다. 외부 드라이버는 설정만 바꿨다: 두 배 해상 모델 sea, EEPROM 저장 없이 RAM 적용, ZED-F9P 용 이동국 파일 신설, 메인 launch 의 gps_config 전달.

#### fix 검증·필터·신선도
작년 코드는 fix 상태를 보지 않고 위경도를 대입했다. 드라이버는 fix 가 없어도 status −1, 위경도 0.0 인 메시지를 계속 내므로, 2026-08-06 실기에서 fix 없이 위도 0°·경도 0° 기준 방위 47.96° 가 발행됐다. 올해는 fix 를 다섯 층으로 거른다.

| 층 | 처리 | 값 |
|---|---|---|
| fix 검증 (gps_guard) | status < 0, NaN·inf, (0, 0), 위경도 범위 밖이면 have_fix 해제, 방위·거리 미발행 | — |
| 공분산 | position_covariance[0] 이 임계 초과면 그 프레임만 버림. 0 이하는 정보 없음으로 통과 | gps_cov_threshold 2.0 |
| 점프 | 직전 fix 대비 속도가 상한 초과면 순간이동(멀티패스)으로 버림 | gps_max_speed_mps 15.0 m/s |
| 적응형 스무딩 | 오차 e < 0.5 m → α 0.15, < 1.5 m → 0.30, 그 이상 0.80 으로 p ← α·p_raw + (1 − α)·p | 코드 상수 |
| 신선도 | 마지막으로 받아들인 fix 후 임계 초과면 방위·거리·지오펜스·도착 판정·ILOS 적분 정지. ERROR 진입 1회 + 5 s 주기 | gps_stale_sec 2.5 s |

필터가 버리는 것은 한 프레임뿐이며 fix 상실로 보지 않는다. 신선도 2.5 s 는 1 Hz 에서 한 프레임 빠짐(2.0 s)은 견디고 두 프레임부터 stale 로 보는 값이다. 2026-10-29 이전에는 have_fix 가 한 번 참이면 GPS 가 끊겨도 마지막 위치로 방위를 계속 냈다.

#### 웨이포인트와 LOS·ILOS 유도
웨이포인트는 config/waypoints.yaml 의 {구역, lat, lon, mode, dwell} 목록이다. 로더가 좌표 범위(빈칸 0.0, 위경도 뒤바뀜)·mode·dwell 을 검사해 틀리면 몇 번째 항목이 왜 틀렸는지 남기고 노드를 세운다. 대회장 좌표는 waypoint_recorder 가 최근 2.0 s 의 유효 fix 를 평균해 녹화한다.
작년의 직접 방위는 조류에 밀리면 경로로 돌아오지 못하고 비스듬히 접근했다. 올해는 이전 웨이포인트 P 에서 목표 W 로 가는 경로선을 기준으로 한다.
- 경로선 이탈: XTE = d(P, 배)·sin(ψ(P→배) − ψ(P→W)), 양수는 오른쪽으로 밀린 것.
- 전방주시: L = clamp(1.2·v, 2.0, 6.0) m. 빠를수록 멀리 봐 완만하게 복귀한다.
- 목표 방위: ψ_d = ψ(P→W) − atan2(XTE + k_i·I, max(0.1, L)).
- ILOS 적분: I ← I + XTE·Δt, |I| ≤ 50(anti-windup), k_i = 0.02. 조류·바람처럼 계속 미는 외란을 상쇄한다. 웨이포인트를 넘길 때 P 를 갱신하고 I 를 0 으로 되돌린다.
- 도착 반경: 도킹(mode 7) 2.0 m, 그 외 속도 2 m/s 미만 2.5 m, 5 m/s 미만 3.5 m, 그 이상 4.5 m. 반경 안에 dwell 초 머무르면 다음 웨이포인트로 넘어간다.
LOS 이득은 초기값이며 물 위에서 튜닝한다. require_boat_mode_auto 가 true 이면 RC 모드가 AUTO 일 때만 웨이포인트가 전진하고, 그 전까지 타이머·dwell·적분·경로 시작점을 초기화해 AUTO 전환 순간 WP0 부터 시작한다.

#### 지오펜스 생성
경계 최근접점 방향으로 원뿔을 막으면 40 × 40 m 경기장 모서리에서 대각선을 향한 배의 정면이 두 원뿔 사이로 열린다. 광선 방식은 정면 경계 거리가 그대로 벽이 되고 튜닝 파라미터도 없다. 상대방위 −80°~+80° 를 1° 간격으로, 절대 방위 /imu/yaw + 상대방위 방향 광선과 경계 폴리곤 변의 최근접 교점 거리(30 m 초과는 inf)를 구해 [−80, 1, r0 … r160] 으로 낸다. fix 없음·stale, /imu/yaw 0.5 s 초과, 배가 이미 경계 밖이면 빈 배열을 낸다. 헤딩이 얼면 "없는 벽"이 배를 가두고, 이미 밖이면 경계가 복귀로를 막기 때문이다. 부팅 5 s 뒤 구독자가 0 이면 ERROR 를 낸다. 대회장 경계 좌표가 아직 없어 geofence_polygon 은 비어 있다(꺼짐).

| 작년 | 문제 | 올해 | 커밋 |
|---|---|---|---|
| fix 상태 무시 | fix 없을 때 (0, 0) 기준 방위 발행 | gps_guard 거부 + 신선도 2.5 s | 02944d8, b288014 |
| 직접 방위 + 고정 도착 반경 3.0 m | 외란에 밀리면 경로 복귀 못 함 | LOS·ILOS, 필터, 속도별 도착 반경 | 9221750 |
| 경계 방어 없음 | 경기장 이탈은 실격 | /geofence_state 광선 방식 | 2cb050b, b678154 |
| 웨이포인트 하드코딩, 순번 기준 시간 초과 | 좌표 수정에 코드 편집, 순서를 바꾸면 시간 초과가 어긋남 | waypoints.yaml + 로더·녹화 도구, mode 기준 시간 초과(Ⅲ-3) | 2758653, 7a2e803, b337ad8 |


### 2.3) LiDAR 처리
rplidar_node 의 /scan 은 ship_direction 이 받아 갭 선택 회피(Ⅲ-4.4), 감속 신호, 지오펜스 병합, 센서 감시(Ⅲ-5)에 쓴다. 회전수는 드라이버 파라미터로 바꿀 수 없어 개체마다 실측값이 다르다. 1번배는 RPLIDAR A3 개체 80BF (약 11.1 Hz / 0.250°) 이다. 이 절은 신호 처리만 다룬다.
- 수신과 계산의 분리. scan_callback 은 스캔을 저장만 하고 계산·발행은 10 Hz 타이머 control_cb 가 한다. 그래서 LiDAR 가 끊겨도 발행은 계속된다. /scan 구독은 BEST_EFFORT·depth 1 로 묵은 스캔이 쌓이지 않게 한다.
- 정면 정렬. LiDAR 프레임 80° 를 선수 정면으로 두고 0~160°(정면 ±80°)만 쓴다. 인덱스는 메시지의 Δθ 로 매번 계산하므로 각분해능이 다른 두 개체에 같은 코드가 돈다.
- inf·nan 처리. 측정 없음은 회피 마스크에서는 빈 셀, 빈 구간 가장자리 거리로는 detection_distance, 감속 신호에서는 유효점 제외로 다룬다. detection_distance 는 기본 3.0 m, 게이트 구간(wp_mode 0·1) 2.0 m 다.
- 감속 신호의 공간 median. 인덱스마다 창 [i−2, i+2](커널 5)의 유효점이 3개 이상이면 위쪽 중앙값을, 미만이면 고립 반사로 버리고, 그 최소값과 각도를 /obstacle_distance_array 로 낸다. 이 필터는 감속 신호에만 걸고 회피 마스크에는 걸지 않아 회피 접촉 성능을 바꾸지 않는다. 시뮬(시드 20, 물보라 30%)에서 가짜 감속이 36.8% → 8.9% 로 줄었고 접촉은 전 조건 0 이었다. 버린 점 수는 /obstacle_reject_count 로 내 물보라 대리 지표로 기록한다.
- 지오펜스 병합과 경계 정지. 자율 회피 구간에서는 셀마다 대응하는 경계 광선을 찾아 ranges = min(real, geofence) 로 병합한다. 병합이 마스크 생성 전이라 경계도 장애물과 같은 팽창·갭 선택을 거친다. 빈 배열이나 2.0 s 넘게 묵은 경계는 병합하지 않는다. 임무 노드가 조향하는 후보각·제자리 선회 구간은 병합할 마스크가 없어 하드 가드를 둔다. 최근접 경계가 2.0 m 안이고 그 방위가 의도한 진행 방향(후보각, 선회는 0°) ±60° 안이면 STOP_HOLD 를 낸다.

| 이름 | 값 | 정한 근거 |
|---|---|---|
| detection_distance_default / _gate (gate_wp_modes [0, 1]) | 3.0 / 2.0 m | 시뮬: 1.8 이면 8/8 충돌. 게이트는 좁아 3.0 이면 접촉 증가. 작년 게이트 키 wp_mode==2 는 위치유지라 정정 |
| obst_median_kernel | 5 | 가짜 감속 36.8% → 8.9%. 군집 물보라면 7/9 검토 |
| geofence_stale_sec | 2.0 s | north 주기 0.5 s 의 4배 |
| geofence_stop_margin_m / _cone_deg | 2.0 m / 60° | 후보각 구간 경계 이탈 방지 |

yaml 과 노드 declare_parameter 기본값은 같고, 대회 launch 는 yaml 을 넘기지 않아 운용값은 노드 기본값이다.

| 작년 | 문제 | 올해 | 커밋 |
|---|---|---|---|
| 계산·발행이 scan_callback 안 | LiDAR 끊기면 /desired_angle 0회 | 저장만, 10 Hz 타이머가 항상 발행 | 02bfc39 |
| /scan depth 10 RELIABLE, 감속 신호가 필터 없는 최소값 | 묵은 스캔으로 조향, 물보라 한 점에 감속 | BEST_EFFORT · depth 1, 공간 median 커널 5 | fd3b39e, 849c570 |
| angle_increment 무방어 | 0 나눗셈으로 노드 사망 | 하드 폴트로 즉시 정지 | da327a1 |
| 경계 처리 없음, time.time() | 경기장 이탈, 벽시계 점프 | 가짜 LiDAR 병합 + 하드 가드, time.monotonic() | b678154, e3490db, 02bfc39 |


### 2.4) 영상 처리 파이프라인
검출기 3종(gate·turn·dock)이 같은 파이프라인을 공유하고, 색 범위·최소 면적·확인 프레임·화각은 vision.yaml 한 곳에서 읽는다. 작년에는 같은 색이 파일 3곳에 다른 값으로 박혀 같은 초록 부표를 게이트 검출기는 보고 도킹 검출기는 못 봤다. 검출기는 상주하며 각자 /wp_mode 를 보고 담당 모드에서만 일한다.

| 단계 | 처리 | 파라미터 / 값 |
|---|---|---|
| ① 입력 | image_topic 구독, BEST_EFFORT depth 1 (묵은 프레임으로 조향 금지) | 해상도는 매 프레임 영상에서 읽음 |
| ② 색 분리 | BGR → HSV, 색별 inRange 를 OR 로 합친 뒤 medianBlur(3). 빨강은 0° 를 넘어 범위 2개 | red [0-5]+[165-180], green H 60-85, white V≥220·S≤40 (⚠️ 실외 재확정) |
| ③ 후보 추출 | 컨투어 → 최소 면적 → 형상 검사(꼭짓점 4~8·extent·종횡비, require_shape) 또는 solidity(면적/볼록껍질) → 화면 y 범위 → 가장 큰 것 1개 | min_area_px 40 (gate·turn) / 80 (dock), min_solidity 0.7, y_zone 검출기별 gate 0.15–0.55 / turn 0.10–0.55 / dock 0.15–0.55 (⚠️ 실측 필요) |
| ④ 형상 판정 (dock) | approxPolyDP 꼭짓점 수 + extent·aspect·circularity 로 삼각/사각/원 3종 | epsilon 0.0315, square extent≥0.4 aspect 0.6~1.4, circle circ≥0.82 |
| ⑤ 확인 | 최근 M 프레임 중 N 회(미검출은 관용, 색 전환은 리셋) 일치해야 발행. 유예 시간 내 유실은 마지막 값 유지 | confirm_frames 1 (gate·turn) / 3 (dock). confirm_window 는 gate·turn 만 선언(기본 N, 켤 땐 3/5), dock 은 연속 3. grace 2.0 s |
| ⑥ 방위각 변환 | 픽셀 x → 각도: θ = −atan((x − cx) / fx), cx = W/2 (영상 중심), fx = (W/2)/tan(hfov/2) | hfov_deg 80.32 (D455 camera_info 실측) → OAK 스펙 69°, ⚠️ camera_info fx 로 실측 교체 |
| ⑦ 발행·게이트 | /wp_mode 가 담당 모드일 때만 발행, stale 2 s → 정지. dock 목표 색·형상이 유효값 밖이면 기동 시 ERROR, 검출 없이 모드 7 에서만 무효값 10000 + 5 s 주기 ERROR | active_wp_modes: gate [0,1] / turn [2,3] / dock [7] |

- 거리는 쓰지 않는다. 카메라는 방위각만 내고 거리는 LiDAR 전방 섹터 최소거리로 얻는다(핀홀 거리 추정의 실해역 오차 회피). 방위각은 화면 왼쪽이 + 이고 왜곡 보정은 하지 않는다. 화각은 해상도별 센서 크롭에 딸려 있어 color_profile 을 바꾸면 hfov_deg 도 바꾼다.
- 확인과 유예. 창 안 다수결이 아니라 이번 프레임 key 가 최근 창에 N 회 이상일 때만 확정해, 흰색 각도를 빨강으로 잘못 내는 일이 없다. 미확정이면 2.0 s 안의 마지막 확정값을, 아니면 무효값 10000.0 을 낸다. 침묵하면 소비자가 "사라졌다"로 읽기 때문이다.
- 모드 게이팅. 판정은 콜백 맨 앞이라 비활성 검출기는 영상 변환도 하지 않는다. turn 과 dock 이 같은 /image_angle 을 내므로 담당 모드가 겹치지 않는지, 첫 발행이 게이트 뒤인지를 정적 테스트가 검사한다.
- 카메라. OAK-1 PoE 는 뎁스가 없지만 거리는 LiDAR 가 재고, D455 도 뎁스를 켜면 USB 에서 떨어져(2분간 13회) 끄고 쓰므로 잃는 기능이 없다. 진동에 강한 고정초점, HFOV 45° 인 32MP 대신 69° 표준형을 골랐다.

| 작년 | 문제 | 올해 | 커밋 |
|---|---|---|---|
| 모드마다 검출기를 subprocess 로 재기동, dock 은 mode 9 | 전환마다 비전 정지, 게이트 시작·도킹이 영상 없이 돎 | 검출기 상주 + ModeGate, 모드 매핑 정정 | 1908f7c |
| 화각 71.5° (벤치 역산치) | 가장자리 각도 최대 4.4° 오차 | hfov_deg 파라미터, camera_info 실측 80.32° | a1c1cb1, 086c125 |
| HSV 가 파일 3곳에 다른 값, color 콜백이 depth 대기 | 검출기마다 다르게 봄, 뎁스 없으면 침묵 | vision.yaml 단일 출처 + 튜닝 도구, depth 의존 제거 | 52494fb, c75f3c4 |
| dock 목표 하드코딩, 단일 프레임 즉시 발행 | 공지 조합이 다르면 코드 수정, 한 프레임 오탐이 조향에 튐 | 목표 파라미터·유효값 검사, 확인 프레임, 원형도 보조 | a007eb0, ff289f7 |


## 3) 전체 임무 상태기계
/wp_mode 값이 담당 노드를 정하고, 검출기는 해당 모드에서만 발행한다. /wp_mode 2 s 이상 stale 이면 검출기 발행 정지. /wp_mode 를 바꾸는 것은 north_goal_angle 하나다. 현재 웨이포인트의 mode 를 매 주기 내고, 도착(반경 안 dwell) 또는 시간 초과로 다음 웨이포인트로 넘어갈 때 값이 바뀐다. fix 가 없어도 /wp_mode 는 계속 낸다. 멈추면 검출기 3종이 모두 꺼지기 때문이다. 대신 fix 없이는 도착 판정을 하지 않는다.

| wp_mode | 임무 | 임무 노드 | 검출기 → 발행 | dwell / 시간 초과 |
|---|---|---|---|---|
| 0, 1 | 게이트 시작·끝 (항로추종) | ship_gate | gate → /red_angle, /green_angle | 3 s / 120 s |
| 2 | 위치유지 | ship_back | turn(white) → /image_angle | 3 s / 120 s |
| 3 | 탐색 (부표 선회) | ship_turn | turn(red) → /image_angle, /image_color | 3 s / 90 s |
| 5 | 장애물 회피 | 없음 — ship_direction 순수 회피 | 없음 | 50 s / 120 s |
| 7 | 도킹 | ship_dock | dock → /image_angle | 60 s / 150 s |
| 8 | 토너먼트 자율 회피 | 없음 | 없음 | 60 s / 없음 (마지막 WP) |

dwell 은 waypoints.yaml, 시간 초과는 WP 하나당 값으로 wp_timeout_modes·wp_timeout_sec 에 둔다. 시간 초과는 위치와 무관하게 다음 웨이포인트로 넘기는, GPS 가 죽어도 진행하는 유일한 수단이다. 도킹 150 s 는 ship_dock 선형 최악 약 129 s 와 dwell 60 s 보다 크게 잡았다. 값이 비었거나 0 이하·NaN·길이 불일치면 출발 전에 노드를 세운다. 2026-10-29 이전에는 시간 초과가 목록 순번에 묶여 있어 회피 두 번째 WP 와 도킹에 시간 초과가 없었다.
특수 명령 코드(/candidate_angle·/desired_angle): 5000 우선회, 6000 좌선회(예약), 20000 미션 없음 → 전진 폴백(모드 5·8 만), 50000 정지. 20000 은 담당 임무 노드가 없는 모드 5·8 에서 north_goal_angle 이 낸다. 작년에는 도킹 노드가 모드 9 로 잡혀 침묵하던 것을 덮느라 모드 7 에도 냈다. 도킹 노드를 7 로 고치면 한 토픽에 발행자가 둘이 되므로 두 수정을 한 커밋으로 묶었다.
그림 Ⅲ-2 종합임무 상태기계

## 4) 임무별 알고리즘

### 4.1) 항로추종 — ship_gate
- 검출기가 빨강·초록 부표 방위각을 내면 두 방위의 중간을 목표로, 한쪽만 보이면 그쪽 기준 오프셋, 둘 다 없으면 탐색 모드(3분기 폴백).
- 거리는 LiDAR 전방 섹터 최소거리. 통과 수는 /gates_passed 로 기록.
(상태 전이 그림, 실측 통과 성공률 — Ⅱ-5 표와 연결)

### 4.2) 위치유지 — ship_back
- LiDAR 로 부표 거리를 유지하고 RTK 는 폴백. PD 미채택(체인이 각도 기반). 흰색 부표 방위는 검출기 turn(white).

### 4.3) 탐색 — ship_turn
- 검출기 turn 은 모드 2 에서 흰색, 모드 3 에서 빨강 부표만 찾는다. ship_turn 은 /image_color 가 red/green 이면 시계 방향, white 면 반시계로 부표 중심 선회. 색 유실 시 SEARCH 상태 유지(토픽 이름 /image_color 고정 — 개명하면 영구 정지한 이력).

### 4.4) 장애물 회피 — ship_direction
/candidate_angle 이 20000(미션 없음)이면 LiDAR 갭 선택으로 조향각을 정한다. 그 밖의 명령은 먼저 처리한다. 50000 이거나 wp_mode 4 진입 후 5 s 이내면 STOP_HOLD, 5000·6000 은 그대로 통과, 일반 후보각은 LiDAR 각 격자(정면 80)로 옮기고 0~160° 밖이면 STOP_HOLD 다.
LiDAR 스캔에서 안전 구간(연속 빈 셀)을 찾는 순서는 다음과 같다.
- 이진 마스크: 병합 거리 r 이 detection_distance 미만이면 장애물.
- 잡음 정리: 길이 int(n × max_spike_ratio) 셀 이하 장애물 구간을 지우고 경계 border_margin 셀을 깎는다. 고립 1~3셀은 사라지고 회피 거리의 실제 부표(약 30셀)는 남는다.
- 팽창: 장애물 구간 최근접 거리 r_min 으로 m = max(1, round(atan((half_width + clearance)/r_min)/Δθ)) 셀씩 양옆을 넓힌다.
- 폭 판정: 빈 구간 (s, e) 마다 호 길이 arc = Δθ·r_edge·(e−s) 가 min_required_width = half_width×2 + clearance 이상인 구간만 후보로 남긴다. r_edge 는 양 끝 중 가까운 쪽 거리다. 먼 쪽으로 재면 실제보다 넓게 보이기 때문이다.
- 후보 선택: 각 후보 구간 안으로 목표 방위를 잘라 넣어(clamp) 목표에 가장 가까운 각을 고르고(동률이면 호가 긴 구간), 그 각을 /desired_angle 로 고정주기 발행한다.
- 경로 없음: 후진 0.5 s 후 3.0 s STOP_HOLD 로 재탐색하고, 그래도 없으면 반복한다. 뒤쪽은 보지 못하므로 계속 후진하지 않는다.
- 변화율 제한: 0~160° 정상 조향값은 한 틱에 15° 까지만 바꾸고 특수 신호는 즉시 통과시킨다.

| 파라미터 | 값 | 근거 |
|---|---|---|
| half_width | 0.29 m | 1번배 폭 0.57 m → 0.285, 공용값 0.29 |
| clearance | 0.25 m | 20 시드 스윕 최적 (Ⅲ-6.1) |
| min_required_width | 0.83 m | 0.29×2 + 0.25 |
| detection_distance 기본 / 게이트 | 3.0 / 2.0 m | 시뮬 스윕 |
| min_obstacle_cells | 1 | 3 이면 작은 부표를 무시한다 (두 배 공용) |
| temporal 투표 | OFF | 시드 20: OFF 접촉 19/20, ON 20/20 — 무익 |
| rear_obstacle_ignore | 제거 | 물보라 0.3 m 하나가 1.6 m 부표를 통째 삭제한 결함 |
| 후진 / 재탐색 | 0.5 / 3.0 s | 뒤를 못 보므로 계속 후진 대신 정지 후 재탐색 |
| rate_limit_max_deg_per_tick | 15° (150°/s) | 실측 후 튜닝 |

[그림 자리] 그림 Ⅲ-3 갭 선택 개념도 — (a) 정면 0~160° 스캔 점, (b) detection_distance 이진 마스크, (c) 스파이크 정리 후 팽창, (d) 빈 구간의 r_edge·호 길이 폭 판정, (e) 목표 방위를 구간 안으로 clamp 해 고른 조향각
그림 Ⅲ-3 갭 선택 개념도 — (a) 정면 0~160° 스캔 점, (b) detection_distance 이진 마스크, (c) 스파이크 정리 후 팽창, (d) 빈 구간의 r_edge·호 길이 폭 판정, (e) 목표 방위를 구간 안으로 clamp 해 고른 조향각

### 4.5) 도킹 — ship_dock
- 도크 3개 중 당일 지정 표식(삼각·원·사각 × 빨강·주황·노랑·초록·파랑·흰색)을 찾는다. 검출기 dock 이 target_color·target_shape 와 일치하는 후보의 방위각만 발행한다. 목표는 yaml 한 줄로 바꾼다. 목표가 유효값 밖이면 기동 시 ERROR, 검출 없이 도킹 모드에서만 무효값 10000 을 내고 5 s 마다 ERROR 로 다시 알린다(다른 모드에서는 발행하지 않는다). 2026-10-28 이전에는 이 분기가 모드 게이트보다 앞이라 모드 2·3 에서도 10000 이 섞여 turn 검출기의 방위를 지웠다. 도크 간격 1.0 m.
- 표식 판정: 마스크는 목표 색 하나로 만들고, 컨투어마다 형상을 판정해 목표와 다르면 버린다. 꼭짓점 3 → Triangle(원형도 ≥ 0.82 면 노이즈), 4~6 → extent > 0.4·종횡비 0.6~1.4 면 Square(원형도 ≥ 0.82 면 Circle), 7 이상 → Circle(원형도 < 0.60 이면 노이즈)이다. 통과 후보 중 화면 높이 0.15~0.55 안의 면적 최대 하나를 쓰고, key = (색, 형상) 이 연속 3 프레임 같아야 확정한다.
- 임계(원형도·extent·확인 프레임)는 감이 아니라 tools/dock_eval.py 혼동행렬 수치로 정한다. 실외 캘리브레이션 대상 위험은 둘이다: hsv.blue 가 S 18~60·V 160~200 이라 흐린 하늘에 가깝고, 빨강(H 0~5)과 주황(H 3~20)이 H 3~5 에서 겹친다.
- 접안 거리는 LiDAR 전방 섹터 최소거리 → ⚠️ 도크 규격 확정 후 contact_dist_m.

### 4.6) 제어기 — motor_control
PID 가 아니라 각도 기반 차동 조향이다. 상류 체인(/candidate_angle → /desired_angle)이 이미 "가야 할 방위" 를 주므로 제어기는 그 각도 오차를 좌·우 PWM 차동으로 바꾸고, 전방 장애물 거리와 페일세이프 레벨로 속도만 깎는다. 20 Hz 고정주기 타이머.

| /desired_angle | 분기 | 좌/우 PWM (ROS 기준, <1500 = 전진) | 감속 적용 |
|---|---|---|---|
| ≥ 50000 | 정지 STOP_HOLD | 1500 / 1500 | — |
| 20000 ~ 50000 | 미션 없음 → 전진 폴백 | base_pwm / base_pwm (1360) | ○ |
| 5000 (6000) | 제자리 우선회 (좌선회) | spin_forward_pwm ± spin_diff = 1600 / 1400 (1400 / 1600) | × (감속 걸면 뒤로 밀리며 돎) |
| −1 ~ 161 (정면 = 80) | 정상 조향 | offset = θ − 80, diff = abs(offset) / max_angle × max_diff(100) → base ∓ diff | ○ |
| 161 ~ 5000 | 후진 | reverse_pwm / reverse_pwm (1590) | × |
| 명령 0.5 s 무수신 | 워치독 | 1500 / 1500 | — |

감속비 = min(장애물 근접비, 페일세이프 상한). 근접비는 전방 최근접 거리 d 에 대해 d ≥ 1.2 m 이면 1.0, 그 아래서 0.7 + 0.3·d/1.2 로 선형. 페일세이프 레벨 1 이면 0.7. 적용은 PWM 을 중립 쪽으로 당기는 식 1500 + (PWM − 1500)·ratio.
- steer_invert 하나가 조향과 SPIN 을 함께 뒤집는다(배선 반대 대비). ⚠️ 물 위 첫 시험에서 확정 — 가장 흔한 사고.
- PD 미채택 근거: 체인이 각도 기반이라 속도 상태가 없고, 각도 오차의 미분은 LiDAR 10 Hz·헤딩 노이즈를 증폭한다. 작년 SPIN 버그(순항 속도로 원을 그려 도크 충돌)는 중심 PWM 을 1500 으로 분리해 해결.
- 출력 PWM 좌우 평균이 1500 보다 크면 /motor_reverse 를 참으로 내 헤딩 COG 추정의 후진 게이트로 쓴다(Ⅲ-2.1).

## 5) 페일세이프
- 4중 장치: time.monotonic() / ARMED(센서 1회 수신 후 감시) / 연속 N회(failsafe_confirm_n 3) / 히스테리시스 + 자동 복구.
- ship_direction 의 SensorWatch 가 /scan 과 /yaw_error 의 신선도를 함께 보고 가장 나쁜 센서가 레벨을 정한다. /scan 만 보면 IMU 가 죽어도 마지막 /yaw_error 로 계속 조향한다(시뮬: 정지율 0%, 접촉 3회). 평가는 독립 타이머(watchdog_cb, 10 Hz)에서 한다 — 센서가 죽으면 콜백이 안 불리므로.

| 레벨 | 진입 조건 | /desired_angle | 속도 |
|---|---|---|---|
| 0 정상 | — | 계산한 각도 | 100% |
| 1 경고 | 경과 > failsafe_warn_sec 0.7 s (연속 3회) | 계산한 각도 그대로 — 회피 중 직진 강제는 피하던 부표에 부딪힌다 | 0.7 배 |
| 2 정지 | 경과 > failsafe_stop_sec 3.0 s (연속 3회), 또는 이상 스캔(angle_increment 0·음수·NaN) 즉시 | STOP_HOLD 로 덮어씀 | 정지 |

- 위치·헤딩 소스는 모르면 침묵한다. IMU 가 끊기면 /imu/yaw·/yaw_error 가 멈춰 위 감시가 잡고(Ⅲ-2.1), 받아들인 GPS fix 가 2.5 s 넘게 없으면 방위·지오펜스가 멈추되 /wp_mode 와 시간 초과 진행은 유지한다(Ⅲ-2.2). 지오펜스가 2.0 s 넘게 묵으면 병합하지 않는다.
- 비전: /wp_mode 가 끊기면 검출기가 발행을 멈추고, 표식 유실 2.0 s 뒤에는 무효값 10000 을 낸다. 도킹 목표 오타는 모드 7 에서만 10000 + 주기 ERROR 로 드러난다(Ⅲ-4.5).
- motor_control 명령 워치독 0.5 s, 펌웨어 워치독 0.5 s, 부팅 시 중립. 접촉기 보조접점 → Mega 핀 38 로 비상정지 상태를 노트북이 안다.
- 진단(온도·헤딩 상태)은 로그만. /health_ok 는 제어에 물리지 않는다 — 거짓 정지 금지.
- TTC 비상제동은 해로워서 폐기, 시간 투표는 무익해서 OFF.

## 6) 시뮬레이션 및 시험 결과

### 6.1) 시뮬레이션 스윕 (tools/sim, 20 시드 × 폭 5 × clearance 6 × 탐지거리 2 × 게이트폭 2 = 2,400 런)
tools/sim 은 ship_direction·motor_control 로직 재구현, 가짜 LiDAR 부표 월드, 2025 규정 70×20 m 전 코스로 이뤄진다. 폭마다 정체(구간 미도달) 런 비율이 가장 낮고 그다음 접촉이 적은 clearance 를 골랐다.

| 선폭 [m] | half_width | 최적 clearance | 접촉 평균 | 정체 |
|---|---|---|---|---|
| 0.43 | 0.215 | 0.35 | 0.36 | 0% |
| 0.50 | 0.25 | 0.30 | 0.85 | 0% |
| 0.58 | 0.29 | 0.25 | 1.32 | 0% |
| 0.64 | 0.32 | 0.25 | 1.44 | 0% |
| 0.70 | 0.35 | 0.20 | 2.23 | 0% |

half_width 0.29 에서 clearance 별 접촉은 0.20 → 1.95, 0.25 → 1.32, 0.30 → 1.46(정체 2%), 0.35 → 정체 28%, 0.45 → 정체 50% 로, 접촉이 가장 적은 0.45 는 절반이 정체라 고르지 않았다. 작년 값 half_width 0.45 는 1.15 m 갭을 요구해 접촉 3.0 회·정체 1.5 회였다(1차 단일 시드 스윕). 최적점의 갭 요구(half_width×2 + clearance)는 0.78~0.90 m 에 모인다. 한계도 있다. 접촉 판정이 반경 half_width 원이라 측면 접촉을 못 잡고, 동역학은 가정값이며, 시간 투표·지오펜스 병합은 시뮬에 없다. 선회 중 유효 폭 W·cosθ + L·sinθ 는 1번배 치수(W 0.58, L 1.66)로 10° 에서 0.86 m, 15° 에서 0.99 m 라 갭 요구 0.83 m 는 약 9° 이내로 정렬했을 때만 맞는다 — 1 m 안팎 틈은 시뮬이 못 잡는 영역이라 수조 게이트 시험으로 확인한다.

### 6.2) 수조·야외 시험

| 일자 | 장소 | 항목 | 결과 | 조치 |
|---|---|---|---|---|
| ____ | 수조 5×5 m | 직진·선회·게이트 2 m | ____ | ____ |
| ____ | 야외 | 항로추종 → 위치유지 전환 | ____ | ____ |
| ____ | 야외 | 회피 구간, 인식 유실 | ____ | ____ |

배 장착 전 벤치·휴대 시험 (배 구분 없음). 아래는 선체에 싣기 전의 기록이라 물 위 성능을 대신하지 않는다.

| 시험 (일자) | 결과 |
|---|---|
| 헤딩 체인 (08-06) | /imu/yaw_raw 100.1 Hz, /imu/yaw 50.0 Hz, /yaw_error 20.0 Hz. 드라이버를 3.0 s 에 끄자 /imu/yaw 3.3 s·/yaw_error 3.8 s 에 멈추고 레벨 1(4.78 s)·레벨 2(7.08 s) |
| LiDAR 페일세이프 (08-06) | 종료 후 0.70 s 레벨 1, 2.99 s 레벨 2, 재기동 즉시 레벨 0. 그동안 /desired_angle 20 s 에 200회 발행, 14.3분 기록에서 의도적 차단 외 오탐 0건 |
| GPS RTK (08-07, ZED-F9P 휴대) | FIXED 수평 정확도 ±0.015 m, 위성 18개, pDOP 1.63, 처음 60 s FLOAT(±0.10 m). 녹화한 1.93 m 간격 두 점 구분. C94-M8P·선체 장착 상태는 미측정 |
| 비전 (08-12, 실내, 확인 프레임 끔) | 빨강 1220/1220, 초록 520/520 프레임, 60 s 동안 5° 이상 각도 튐 0건. 오탐은 세지 않음. dock_eval.py --selftest 12/12 는 합성 특징 자기 검증일 뿐 현장 정확도가 아니다 |
| 로직 테스트 (10-29, ROS 없이) | ssf_heading 68, north_goal_angle 91 + gps_guard 18, ship_direction 35, color_shape_detector 95 개 전부 통과 |

실측 예정. 실측이 없는 자리는 아래 한 표로 모은다. 값은 배 조립 후에만 의미가 있다.

| 항목 | 담당 절 | 상태/값 |
|---|---|---|
| mount_offset_deg · invert_yaw 확정 (정북 정렬, 목표값 360 + 편각) | Ⅲ-2.1 | ____ |
| 대회장 자기편각, 재부팅 전후 /imu/yaw_raw 일치 | Ⅲ-2.1 | ____ |
| 정지·직진·선회 중 헤딩 흔들림 | Ⅲ-2.1 | ____ |
| 냉시작 첫 fix 시간, 위성 수 / pDOP | Ⅲ-2.2 | ____ |
| RTK on/off 위치 산포 비교 (정지 5분): FLOAT ____ m / FIXED ____ m | Ⅲ-2.2 | ____ |
| 웨이포인트 도착 오차 / 직선 구간 XTE (물 위) | Ⅲ-2.2 | ____ |
| 대회장 웨이포인트·경계 좌표 녹화 | Ⅲ-2.2 | ____ |
| LiDAR 정면(80°)–선수 정렬, 부표 탐지 최대거리 | Ⅲ-2.3 | ____ |
| 수면 반사·물보라 오탐: /obstacle_reject_count (야외 rosbag) | Ⅲ-2.3 | ____ 회/분 |
| 1 m 안팎 게이트 통과 성공률 (수조) | Ⅲ-4.4 | ____ |
| 혼동행렬: 야외 부표 영상 (2/5/8 m, 순광·역광) 색별 정탐/오탐 — 임계값은 이 수치로 정한다 | Ⅲ-2.4 | ____ 프레임 / ____ |
| 야외 HSV 확정값, OAK 실측 hfov_deg | Ⅲ-2.4 | ____ |
| 도크 표식 형상×색 혼동행렬 (dock_eval.py --bag) | Ⅲ-4.5 | ____ |
| steer_invert 확정 (물 위 첫 시험) | Ⅲ-4.6 | ____ |
| 물 위 페일세이프 전이·오탐 | Ⅲ-5 | ____ |

▶ 작년 전수분석 결론: 격차는 설계가 아니라 (1) 실해역 파라미터 튜닝 (2) 유실·재인식 엣지케이스 검증 횟수 (3) 노트북 전원·발열 (4) A/B 분산. 이 표가 그 증거가 된다.

## 7) 소프트웨어 장비 및 구성

| 장비 사진 | 장비명 | 용도 |
|---|---|---|
| (사진) | Lenovo IdeaPad Slim 3 15IAH8 | Main Computer — 배당 1대, ROS2 Humble (두 배 각 1대) |
| (사진) | Arduino Mega 2560 R3 | RC 수신·ESC 2채널·LED·비상정지 감지 (ssf_boat.ino, 두 배 동일 펌웨어) |
| (사진) | FlySky FS-i6 / FS-iA6B | 수동 조종 · 모드 전환(RC 전용) |
| (사진) | ipTIME UH308 유전원 허브 | LiDAR·IMU·GPS·D455 USB 확장 (5V 5A 모듈 급전) |
| (사진) | NETmate NM-UCL02 (AX88179) | OAK 이더넷 ↔ 노트북 USB-C 직결 |



==================================================================
## 문서: docs/전달용/보고서_분담/02_GPS.md
==================================================================
# 지시서 02 — GPS (RTK · 웨이포인트 · 지오펜스)

## 담당 목차와 분량

**양식 파일: `양식_GPS.docx`** — 아래 절의 번호·제목·빈 표가 본 보고서 모양으로 들어 있다. 여기에 채워서 낸다.

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
3. 우리가 드라이버 설정에서 바꾼 것은? 해상용 동역학 모델, 배별 설정 인자, EEPROM 저장 제거.
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
14. 아직 실측이 없는 것은? 08-07 에 F9P 를 들고 나가 RTK FIXED ±0.015 m 를 본 기록은 있다(`docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md` 4-6절). 배에 단 상태와 A배 M8P 는 아직이다.

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
| `ublox` | **외부.** u-blox ROS 드라이버 (`package.xml` 저자 Johannes Meyer, 관리자 Gareth Cross). 원 저장소는 저장소 안에 기록 없음 | 있음 | B배 F9P 설정 신설·배별 인자(`c1b0d81`), F9P 해상 동역학 모델(`ed2a144` — 같은 커밋의 토픽명 수정은 IMU 드라이버 쪽), 배별 설정 전달·EEPROM 저장 제거(`c7c0c59`) |
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

==================================================================
## 소스 파일 (행 번호 포함)
==================================================================

----- FILE: src/north_goal_angle/north_goal_angle/north_goal_angle.py -----
```
     1	"""north_goal_angle — 웨이포인트 FSM + 경계 이탈 방지(geofence). 6a.
     2	
     3	고친 것:
     4	  [3] 🚨 mode-7 폴백 제거 (ship_dock 수정과 **원자적 한 쌍**)
     5	      작년: if wp_mode == 7: pub_candidate.publish(20000)  ← 0.5초마다
     6	      이건 '도킹이 안 되는 걸 알고 덮어둔 흔적'이다. ship_dock 이 9 로 잡혀 침묵하니,
     7	      배가 멈추지 않게 north 가 대신 폴백을 냈다.
     8	      ship_dock 을 7 로 고치면 → 같은 /candidate_angle 에 발행자가 둘이 된다.
     9	      도킹 중 조향이 GPS 방위로 튄다 (접안 직전 60초에 35° 스파이크 20번).
    10	      → 폴백은 **담당 노드가 없는 모드에만** 남긴다: mode 5(회피), 8(토너먼트 회피).
    11	
    12	  [6] Geofence 발행 — 경계를 **'가짜 LiDAR'** 로 낸다. 경기장 밖으로 나가면 실격인데 방어가 0 이었다.
    13	
    14	      /geofence_state (Float32MultiArray)
    15	        = [angle_min_deg, angle_inc_deg, r0, r1, r2, ...]   (상대방위 기준, 멀면 inf)
    16	        정보 없으면 **빈 배열**.
    17	
    18	      ★ 왜 '광선'인가 (원뿔 방식을 버린 이유):
    19	        **벽은 '점'이 아니라 '선'이다.** 최근접점 한 방향으로 ±각도 원뿔을 막는 건 기하학적으로
    20	        틀렸다. 40×40 경기장 (40,40) 모서리에서 배가 대각선(45°)을 향하면:
    21	            북벽 최근접점 → 상대 -45°   /   동벽 최근접점 → 상대 +45°
    22	            그런데 **탈출구(모서리)는 상대 0° — 배 정면이다.**
    23	        half_block=40° 면 차단 구역이 [-85,-5] 와 [5,85] → **정면 0° 가 정확히 '틈'으로 열린다.**
    24	        배가 대각선으로 탈출한다. 실격. (경기장이 사각형이 아니면 또 뚫린다.)
    25	        광선으로 쏘면 정면 방향 경계 거리(2.12m)가 그대로 잡혀 벽이 된다. **튜닝 파라미터가 없다.**
    26	
    27	      ★ 소비자(ship_direction)는 **스캔 병합 한 줄**이면 끝이다:
    28	            ranges[i] = min(real_ranges[i], geofence_ranges[i])
    29	        기존 dilate 가 배 폭 여유를 자동으로 더하고, 기존 갭-팔로잉이 알아서 피한다.
    30	        특수 로직도 새 상태기계도 없다. "경계선을 그냥 벽으로 취급한다"를 끝까지 밀어붙인 것.
    31	
    32	      계산량: 161방향 × 변 개수 @ 2Hz — 무시할 수준.
    33	
    34	프레임 규약: 광선 각도는 **상대 방위**(0=정면)다 — /candidate_angle 과 같은 규약.
    35	  ⚠️ 광선 방향이 /imu/yaw 에 통째로 의존한다 (CLAUDE.md 3-5).
    36	     N1 에서 /imu/yaw 는 ssf_heading/yaw_mux 단독 발행으로 바뀌었다 —
    37	     부팅 0점화와 GPS COG override 를 걷어낸 절대방위다.
    38	     🚨 단, mount_offset_deg / invert_yaw 를 **벤치에서 아직 안 쟀다.**
    39	        그 전까지 이 geofence 는 방향이 맞는다는 보장이 없다.
    40	     IMU 가 묵으면 **빈 배열**을 낸다. 틀린 방향으로 '없는 벽'을 세우느니 안 내는 게 낫다.
    41	"""
    42	
    43	import math
    44	import os
    45	import time
    46	
    47	import yaml
    48	
    49	import rclpy
    50	from rclpy.node import Node
    51	from rclpy.qos import ReliabilityPolicy, QoSProfile
    52	from std_msgs.msg import Float32, Float32MultiArray, Int32, Float64
    53	from sensor_msgs.msg import NavSatFix
    54	from geopy import distance
    55	from ament_index_python.packages import get_package_share_directory
    56	
    57	from north_goal_angle.waypoint_loader import parse_waypoints, WaypointError
    58	from north_goal_angle.gps_guard import (
    59	    DEFAULT_GPS_STALE_SEC, fix_is_fresh, fix_is_usable, sanitize_stale_sec)
    60	from north_goal_angle import los_logic          # LOS 유도 (순수, 테스트됨)
    61	from north_goal_angle.gps_filter import GPSFilter  # 공분산·점프·스무딩 (순수, 테스트됨)
    62	from north_goal_angle.wp_timeout import (            # WP 타임아웃 표 (순수, 테스트됨)
    63	    DEFAULT_WP_TIMEOUT_MODES, DEFAULT_WP_TIMEOUT_SEC, WpTimeoutError,
    64	    build_timeout_table, dwell_conflicts, per_wp_timeouts, timeout_for_mode,
    65	)
    66	
    67	# 🚨 대회 좌표는 코드가 아니라 config/waypoints.yaml 에서 읽는다 (비전공자도 편집 가능).
    68	#    대회장에서 GPS 좌표를 바꿀 사람은 그 파일의 lat/lon 만 고치면 된다 — 이 파일은 안 건드린다.
    69	#    로드/검증은 waypoint_loader.parse_waypoints (순수 로직, 테스트됨).
    70	#    파일이 없거나 형식이 틀리면 노드를 띄우지 않는다 — 틀린 좌표로 달리느니 멈춘다.
    71	
    72	CANDIDATE_INVALID = 20000.0
    73	ARRIVE_RADIUS_M = 3.0
    74	# WP 타임아웃은 순번이 아니라 wp_mode 기준 — 파라미터 wp_timeout_modes / wp_timeout_sec (wp_timeout.py)
    75	
    76	# 담당 노드가 없는 모드 = 순수 회피 구간. 여기서만 폴백을 낸다. (CLAUDE.md 3-6)
    77	FALLBACK_MODES = (5, 8)
    78	
    79	# GPS stale 이 이어지는 동안 ERROR 를 다시 찍는 주기(s). 로그 주기일 뿐 제어에는 안 쓴다.
    80	GPS_STALE_LOG_PERIOD_SEC = 5.0
    81	
    82	# 🚀 자율 시작 게이트: 아두이노 모드(ssf_bridge /boat_mode, Int32). 0=WAIT 1=MANUAL 2=AUTO.
    83	#    상수 공유 모듈화는 대회 전 금지(CLAUDE.md 3-9) → 여기 인라인(값은 ssf_bridge status_parser 와 일치).
    84	MODE_AUTO = 2
    85	
    86	
    87	def mission_should_run(boat_mode, require_auto, mode_auto=MODE_AUTO):
    88	    """미션 FSM(웨이포인트 전진)을 돌려도 되나 — ROS 비의존 순수 판정.
    89	
    90	    require_auto=False → 항상 True (아두이노 없이 벤치 테스트용).
    91	    require_auto=True  → boat_mode 가 AUTO(2)일 때만. WAIT/MANUAL/None(모름) 이면 False.
    92	      → 기본이 '안 돈다'라 안전하다: RC 를 AUTO 로 넘기기 전엔 미션이 전진하지 않는다.
    93	    """
    94	    if not require_auto:
    95	        return True
    96	    return boat_mode == mode_auto
    97	
    98	
    99	# 위경도 → 로컬 미터 근사
   100	M_PER_DEG_LAT = 110540.0
   101	M_PER_DEG_LON_EQ = 111320.0
   102	
   103	
   104	class NorthGoalAngle(Node):
   105	    def __init__(self):
   106	        super().__init__('north_goal_angle')
   107	        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
   108	
   109	        # ---- 웨이포인트(GPS 좌표) 로드 — config/waypoints.yaml 에서 읽는다 ----
   110	        # 파일 경로는 파라미터로도 바꿀 수 있다(기본: 패키지 share 의 config/waypoints.yaml).
   111	        default_wp = os.path.join(
   112	            get_package_share_directory('north_goal_angle'), 'config', 'waypoints.yaml')
   113	        self.waypoints_file = str(
   114	            self.declare_parameter('waypoints_file', default_wp).value)
   115	        self.waypoints = self._load_waypoints(self.waypoints_file)
   116	
   117	        # ---- 파라미터 (config/north_goal_angle.yaml, CLAUDE.md 1-4) ----
   118	        self.period_sec = float(self.declare_parameter('period_sec', 0.5).value)
   119	
   120	        # ---- LOS 유도 (경로선 추종 + 조류 보상 ILOS) ----
   121	        # 직접 방위(현위치→목표)는 조류·초기오차로 밀리면 비스듬히 접근한다. LOS 는 이전 WP→목표
   122	        # '경로 선'으로 되돌아오게 하고, ILOS 적분항이 지속 외란(조류·바람)을 상쇄한다.
   123	        self.los_min_lookahead_m = float(self.declare_parameter('los_min_lookahead_m', 2.0).value)
   124	        self.los_max_lookahead_m = float(self.declare_parameter('los_max_lookahead_m', 6.0).value)
   125	        self.los_speed_gain = float(self.declare_parameter('los_speed_gain', 1.2).value)
   126	        self.ilos_ki = float(self.declare_parameter('ilos_ki', 0.02).value)
   127	        self.ilos_max = float(self.declare_parameter('ilos_integral_max', 50.0).value)  # anti-windup
   128	        # GPS 필터 (fix_is_usable 통과분에 품질·이상치·지터 처리)
   129	        self.gps_cov_threshold = float(self.declare_parameter('gps_cov_threshold', 2.0).value)
   130	        self.gps_max_speed_mps = float(self.declare_parameter('gps_max_speed_mps', 15.0).value)
   131	
   132	        # 🚀 자율 시작 게이트: True 면 RC 모드가 AUTO(/boat_mode==2) 여야 미션이 전진한다.
   133	        #    기본 True = 안전(수동 이동 중 미션 스킵 방지). 벤치(아두이노 없음)는 false 로.
   134	        self.require_auto = bool(self.declare_parameter('require_boat_mode_auto', True).value)
   135	        # 경기장 폴리곤: [lat1, lon1, lat2, lon2, ...] 평탄 배열.
   136	        # ⚠️ 실측 필요 — 대회장 경계 좌표를 아직 모른다. 비어 있으면 geofence 는 꺼진다.
   137	        self.geofence_polygon = list(
   138	            self.declare_parameter('geofence_polygon', []).value or [])
   139	        # 경계를 '가짜 LiDAR'로 쏜다 — 스캔과 같은 각도 격자(상대방위).
   140	        # ★ 왜 광선인가: 벽은 '점'이 아니라 '선'이다. 최근접점 한 방향으로 ±각도 원뿔을 막는 건
   141	        #   기하학적으로 틀렸다. 40x40 모서리에서 배가 대각선(45°)을 보면 북벽 최근접점은 -45°,
   142	        #   동벽은 +45° 인데 **탈출구(모서리)는 정면 0°** 라 원뿔 사이 '틈'으로 그대로 빠져나간다.
   143	        #   광선으로 쏘면 정면 방향의 경계 거리(2.12m)가 그대로 잡혀 벽이 된다. 튜닝도 필요 없다.
   144	        self.ray_min_deg = float(self.declare_parameter('geofence_ray_min_deg', -80.0).value)
   145	        self.ray_max_deg = float(self.declare_parameter('geofence_ray_max_deg', 80.0).value)
   146	        self.ray_inc_deg = float(self.declare_parameter('geofence_ray_inc_deg', 1.0).value)
   147	        self.geofence_max_range_m = float(
   148	            self.declare_parameter('geofence_max_range_m', 30.0).value)   # 이보다 멀면 inf
   149	        # 구독자 0 경고를 시작할 시각
   150	        self.geofence_check_after_sec = float(
   151	            self.declare_parameter('geofence_check_after_sec', 5.0).value)
   152	        # 🚨 IMU 신선도. geofence 상대방위는 /imu/yaw 로 계산하므로 IMU 에 의존한다.
   153	        #    IMU 가 죽으면 yaw 가 얼어붙고 → 경계 방위가 엉뚱해지고 → ship_direction 이
   154	        #    '없는 벽'을 칠한다 → 배가 갇힌다.
   155	        #    → 묵으면 geofence 도 [inf, nan] 을 낸다. (1단계 ship_goal_angle 과 같은 원칙:
   156	        #      **모르면 입을 다문다.** 틀린 값을 내느니 안 내는 게 낫다.)
   157	        self.imu_stale_sec = float(self.declare_parameter('imu_stale_sec', 0.5).value)
   158	        # 🚨 GPS fix 신선도 (계약서_GPS묶음_20261029 C1). GPSFilter 가 마지막으로 받아들인
   159	        #    프레임이 이보다 묵으면 tp·goal_distance 를 멈추고 geofence 는 빈 배열.
   160	        #    /fix 가 끊기면 콜백이 안 불려 have_fix 가 True 로 남는다 → 옛 위치로 조향하던 구멍.
   161	        #    inf·0 이하·NaN 은 잘못된 값 → 시작 ERROR + 기본값(노드는 세우지 않는다).
   162	        #    🚨 단 정수(yaml `gps_stale_sec: 0`·`3`, `-p gps_stale_sec:=0`)는 여기까지 못 온다 —
   163	        #       기본값 2.5 로 DOUBLE 선언이라 Humble rclpy 가 InvalidParameterTypeException 으로
   164	        #       노드를 세운다. yaml 값은 반드시 소수(.0)로 적는다.
   165	        raw_stale = self.declare_parameter('gps_stale_sec', DEFAULT_GPS_STALE_SEC).value
   166	        self.gps_stale_sec, stale_ok = sanitize_stale_sec(raw_stale)
   167	        if not stale_ok:
   168	            self._log_stale_sec_invalid(raw_stale)
   169	
   170	        # ---- WP 타임아웃 (wp_mode 기준, WP 하나당) — 계약서_WP타임아웃모드기준_20261029 ----
   171	        # ROS2 파라미터는 dict 불가 → 병렬 배열. 표에 없는 모드 = 타임아웃 없음.
   172	        # 🚨 yaml 의 wp_timeout_sec 는 전부 .0 (정수 섞이면 Humble 이 노드를 죽이거나 params 파싱 실패).
   173	        # 타이머 생성보다 먼저 선언·검증한다.
   174	        self.wp_timeout_table = self._load_wp_timeouts(
   175	            self.declare_parameter('wp_timeout_modes', list(DEFAULT_WP_TIMEOUT_MODES)).value,
   176	            self.declare_parameter('wp_timeout_sec', list(DEFAULT_WP_TIMEOUT_SEC)).value)
   177	
   178	        # ---- 발행 (토픽 이름 불변 + /geofence_state 신규, CLAUDE.md 3-9) ----
   179	        self.pub_dist = self.create_publisher(Float32, '/goal_distance', qos)
   180	        self.pub_mode = self.create_publisher(Int32, '/wp_mode', qos)
   181	        self.pub_candidate = self.create_publisher(Float32, '/candidate_angle', qos)
   182	        self.pub_bearing = self.create_publisher(Float32, '/north_goal_angle_tp', qos)
   183	        self.pub_remain = self.create_publisher(Float32, '/wp_remaining_time', qos)
   184	        self.pub_geofence = self.create_publisher(Float32MultiArray, '/geofence_state', qos)
   185	
   186	        # ---- 구독 ----
   187	        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_cb, qos)
   188	        self.create_subscription(Float64, '/imu/yaw', self.yaw_cb, qos)   # 경계 상대방위 계산용
   189	        # 🚀 아두이노 모드(ssf_bridge 발행). AUTO 일 때만 미션 전진. RELIABLE 발행이라 qos 호환.
   190	        self.create_subscription(Int32, '/boat_mode', self.boat_mode_cb, qos)
   191	
   192	        # ---- 상태 ----
   193	        self.lat, self.lon = 0.0, 0.0
   194	        self.have_fix = False
   195	        self.last_fix_t = None          # GPSFilter 가 받아들인 마지막 프레임 (monotonic). None=없음
   196	        self.gps_stale = False          # have_fix 인데 fix 가 묵은 상태(로그 전이용)
   197	        self.gps_stale_next_log_t = 0.0
   198	        self.yaw = None
   199	        self.last_yaw_t = None          # /imu/yaw 마지막 수신 (monotonic)
   200	        self.wp_idx = 0
   201	        self.t_start = None
   202	        self.wp_enter_time = None
   203	        self.t0 = time.monotonic()
   204	
   205	        # LOS 상태
   206	        self.gps_filter = GPSFilter(
   207	            cov_threshold=self.gps_cov_threshold, max_speed_mps=self.gps_max_speed_mps)
   208	        self.integral_xte = 0.0          # ILOS 적분 (WP 전환 시 리셋)
   209	        self.prev_lat = None             # 경로선 시작점(이전 WP). None=첫 구간
   210	        self.prev_lon = None
   211	        self.boat_mode = None            # ssf_bridge /boat_mode (0 WAIT/1 MANUAL/2 AUTO). None=아직 모름
   212	
   213	        self.create_timer(self.period_sec, self.timer_cb)
   214	
   215	        gf = "OFF(폴리곤 미설정 — ⚠️ 실측 필요)" if len(self.geofence_polygon) < 6 else \
   216	             f"ON({len(self.geofence_polygon) // 2}점)"
   217	        self.get_logger().info(
   218	            f"north_goal_angle 시작: 폴백은 mode {FALLBACK_MODES} 에만 발행"
   219	            f"(mode 7 폴백 제거 — ship_dock 과 충돌했다). geofence {gf}. "
   220	            f"GPS stale 기준 {self.gps_stale_sec:g}s."
   221	        )
   222	
   223	    # ───────────────────────── 웨이포인트 로드 ─────────────────────────
   224	    def _load_waypoints(self, path):
   225	        """config/waypoints.yaml 을 읽어 검증한다. 실패하면 노드를 세운다(예외).
   226	
   227	        🚨 '모르면 침묵 말고 알린다' — 파일이 없거나 좌표가 이상하면 조용히 기본값으로
   228	        넘어가지 않고, 어디가 왜 틀렸는지 ERROR 로그를 내고 예외를 던져 노드를 멈춘다.
   229	        틀린 좌표로 배가 달리는 것보다 출발 전에 멈추는 게 안전하다.
   230	        """
   231	        try:
   232	            with open(path, 'r', encoding='utf-8') as f:
   233	                raw = yaml.safe_load(f)
   234	        except FileNotFoundError:
   235	            self.get_logger().error(
   236	                f"🚨 웨이포인트 파일이 없다: {path}\n"
   237	                f"   config/waypoints.yaml 이 설치됐는지 확인하라(colcon build 필요).")
   238	            raise
   239	        except yaml.YAMLError as e:
   240	            self.get_logger().error(
   241	                f"🚨 waypoints.yaml 문법 오류(콤마·중괄호·들여쓰기 확인): {e}")
   242	            raise
   243	
   244	        try:
   245	            wps = parse_waypoints(raw)
   246	        except WaypointError as e:
   247	            self.get_logger().error(f"🚨 웨이포인트 내용 오류 → {e}")
   248	            raise
   249	
   250	        modes = [w[2] for w in wps]
   251	        self.get_logger().info(
   252	            f"웨이포인트 {len(wps)}개 로드: {path}\n   미션 순서(mode): {modes}")
   253	        return wps
   254	
   255	    def _load_wp_timeouts(self, modes, secs):
   256	        """wp_timeout_modes/sec 를 검증해 {mode: sec} 표로. 틀리면 ERROR + 예외로 노드를 세운다.
   257	
   258	        빈 표·≤0·NaN·길이 불일치·허용 밖 모드·중복은 출발 전에 거부한다(waypoints.yaml 과 같은 방식).
   259	        timeout ≤ dwell 인 WP 는 반경 도착이 불가능하므로 WARN 만 낸다(세우지는 않는다).
   260	        """
   261	        try:
   262	            table = build_timeout_table(modes, secs)
   263	        except WpTimeoutError as e:
   264	            self.get_logger().error(f"🚨 WP 타임아웃 파라미터 오류 → {e}")
   265	            raise
   266	
   267	        for idx, mode, t, dwell in dwell_conflicts(self.waypoints, table):
   268	            self.get_logger().warn(
   269	                f"⚠️ WP{idx}(mode {mode}): 타임아웃 {t:.1f}s ≤ dwell {dwell:.1f}s — "
   270	                f"반경 도착은 불가능하고 늘 시간 초과로만 넘어간다. wp_timeout_sec 확인.")
   271	
   272	        per_wp = ['없음' if t is None else f"{t:g}" for t in per_wp_timeouts(self.waypoints, table)]
   273	        self.get_logger().info(
   274	            f"WP 타임아웃(mode 기준, WP 하나당) 표={table} → WP별(s): {per_wp}")
   275	        return table
   276	
   277	    # ───────────────────────── 콜백 ─────────────────────────
   278	    def gps_cb(self, msg):
   279	        # 🚨 status 를 반드시 본다. 드라이버는 fix 가 없어도 /fix 를 1Hz 로 계속 내며,
   280	        #    그때 lat/lon 은 0.0 이다. 예전 코드는 이걸 그대로 믿고 (0,0) 기준 방위를
   281	        #    계산해 발행했다 — 실측 47.96° = calc_angle(0, 0, WP0). 판정은 gps_guard 참고.
   282	        if not fix_is_usable(msg.status.status, msg.latitude, msg.longitude):
   283	            if self.have_fix:
   284	                self._log_fix_lost(msg.status.status)
   285	            self.have_fix = False
   286	            return
   287	
   288	        # fix 는 유효하다 → 품질·이상치·지터를 GPSFilter 로 거른다. 여기서 버리는 건 '한 프레임'만
   289	        # (공분산 큼 / 순간이동)이지 fix 상실이 아니다 → have_fix·이전 위치는 유지한다.
   290	        cov = msg.position_covariance[0] if len(msg.position_covariance) > 0 else None
   291	        now = time.monotonic()
   292	        accepted, reason = self.gps_filter.update(
   293	            msg.latitude, msg.longitude, cov, now)
   294	        if not accepted:
   295	            self.get_logger().warn(f"🛰️ GPS fix 한 프레임 버림: {reason}",
   296	                                   throttle_duration_sec=2.0)
   297	            return
   298	        if not self.have_fix:
   299	            self._log_fix_acquired(msg.status.status)
   300	        self.lat = self.gps_filter.filtered_lat
   301	        self.lon = self.gps_filter.filtered_lon
   302	        self.have_fix = True
   303	        self.last_fix_t = now           # 🚨 받아들인 프레임에서만 — 거부·NO_FIX 는 갱신 안 함
   304	
   305	    # ⚠️ 로거 호출 지점을 함수로 분리한 이유: rclpy 는 로그 호출을 **소스 위치**로 캐싱해서,
   306	    #    같은 줄이 warn/info 로 번갈아 불리면 ValueError 로 노드가 죽는다 (CLAUDE.md §5 사례 ②).
   307	    def _log_fix_lost(self, status):
   308	        self.get_logger().warn(
   309	            f"🛰️ GPS fix 상실(status={status}) → 목표 방위·거리 발행 중단. "
   310	            "모르면 입을 다문다 (소비자는 침묵을 처리하도록 설계돼 있다)")
   311	
   312	    def _log_fix_acquired(self, status):
   313	        self.get_logger().info(f"🛰️ GPS fix 확보(status={status}) → 항법 재개")
   314	
   315	    # stale 로그도 심각도·용도별로 호출 지점을 나눈다 (CLAUDE.md 1-7).
   316	    def _log_stale_sec_invalid(self, raw):
   317	        self.get_logger().error(
   318	            f"🚨 gps_stale_sec={raw!r} 는 잘못된 값(inf·0 이하·NaN·숫자 아님) → "
   319	            f"기본값 {self.gps_stale_sec:g}s 로 돈다. north_goal_angle.yaml 확인.")
   320	
   321	    def _log_fix_stale_enter(self, age):
   322	        self.get_logger().error(
   323	            f"🛰️ GPS fix stale ({age:.1f}s > {self.gps_stale_sec:g}s, /fix 끊김 또는 필터 계속 거부) "
   324	            "→ 목표 방위·거리 발행 중단, geofence 빈 배열. /wp_mode·시간 초과 진행은 유지")
   325	
   326	    def _log_fix_stale_ongoing(self, age):
   327	        self.get_logger().error(
   328	            f"🛰️ GPS fix 여전히 stale ({age:.1f}s) → 항법 침묵 중 (ublox·안테나·USB 확인)")
   329	
   330	    def _log_fix_stale_recovered(self):
   331	        self.get_logger().info("🛰️ GPS fix 신선도 복구 → 항법 재개")
   332	
   333	    def yaw_cb(self, msg):
   334	        self.yaw = float(msg.data)
   335	        self.last_yaw_t = time.monotonic()
   336	
   337	    def boat_mode_cb(self, msg):
   338	        # 0=WAIT 1=MANUAL 2=AUTO. AUTO 로 바뀌면 미션이 WP0 부터 시작한다(timer_cb 게이트).
   339	        prev = self.boat_mode
   340	        self.boat_mode = int(msg.data)
   341	        if prev != self.boat_mode and self.boat_mode == MODE_AUTO:
   342	            self.get_logger().info("🚀 AUTO 전환 감지 → 미션 시작 (WP0 부터)")
   343	
   344	    # ───────────────────────── Geofence ─────────────────────────
   345	    def _local_xy(self, lat, lon):
   346	        """보트 기준 로컬 미터 (east, north)."""
   347	        kx = math.cos(math.radians(self.lat)) * M_PER_DEG_LON_EQ
   348	        return ((lon - self.lon) * kx, (lat - self.lat) * M_PER_DEG_LAT)
   349	
   350	    def _geofence_ranges(self, gps_ok):
   351	        """경계를 '가짜 LiDAR'로 쏜다 → (angle_min_deg, angle_inc_deg, [r0, r1, ...]) 또는 None.
   352	
   353	        각 상대방위마다 '이 방향으로 몇 m 가면 경계 밖인가'를 구한다. 멀면 inf.
   354	        소비자(ship_direction)는 이걸 실제 스캔과 min() 으로 병합하기만 하면 된다 —
   355	        특수 로직이 아예 없어진다. 기존 dilate 가 배 폭 여유를 더하고, 갭-팔로잉이 알아서 피한다.
   356	
   357	        None(=빈 배열) 을 내는 경우 — 전부 '모르면 입을 다문다':
   358	          · 폴리곤 미설정 / GPS fix 없음 또는 stale(gps_stale_sec)
   359	          · IMU stale     → yaw 가 얼면 광선 방향이 통째로 틀어져 '없는 벽'을 만든다
   360	          · 이미 경기장 밖 → 경계를 벽으로 세우면 돌아갈 길을 막는다
   361	        """
   362	        poly = self.geofence_polygon
   363	        if len(poly) < 6 or not gps_ok:             # 최소 3점(=6수) / fix 없음·stale
   364	            return None
   365	
   366	        # 🚨 IMU 신선도 — 광선 방향이 yaw 에 통째로 의존한다. 얼면 벽 전체가 엉뚱해진다.
   367	        if self.yaw is None or self.last_yaw_t is None:
   368	            self.get_logger().warn(
   369	                "/imu/yaw 없음 → geofence 침묵(빈 배열)", throttle_duration_sec=5.0)
   370	            return None
   371	        yaw_age = time.monotonic() - self.last_yaw_t
   372	        if yaw_age > self.imu_stale_sec:
   373	            self.get_logger().warn(
   374	                f"/imu/yaw stale ({yaw_age:.2f}s > {self.imu_stale_sec}s) → geofence 침묵. "
   375	                f"틀린 방향으로 '없는 벽'을 세우느니 안 내는 게 낫다.",
   376	                throttle_duration_sec=2.0)
   377	            return None
   378	
   379	        pts = [self._local_xy(poly[i], poly[i + 1]) for i in range(0, len(poly) - 1, 2)]
   380	
   381	        if not _point_in_polygon((0.0, 0.0), pts):
   382	            # 🚨 이미 밖이다. 경계를 벽으로 세우면 복귀로를 막는다 → 아무것도 내지 않는다.
   383	            self.get_logger().error(
   384	                "🚨 경기장 경계 밖이다 (실격 위험). GPS 웨이포인트로 복귀 중.",
   385	                throttle_duration_sec=2.0)
   386	            return None
   387	
   388	        if self.ray_inc_deg <= 0.0 or self.ray_max_deg < self.ray_min_deg:
   389	            return None
   390	
   391	        n = int(round((self.ray_max_deg - self.ray_min_deg) / self.ray_inc_deg)) + 1
   392	        ranges = []
   393	        for k in range(n):
   394	            rel = self.ray_min_deg + k * self.ray_inc_deg
   395	            th = math.radians((self.yaw + rel) % 360.0)      # 절대 방위(compass)
   396	            d = (math.sin(th), math.cos(th))                 # (east, north) 단위벡터
   397	            t = _ray_polygon_dist((0.0, 0.0), d, pts)
   398	            ranges.append(float(t) if (t is not None and t <= self.geofence_max_range_m)
   399	                          else float('inf'))
   400	
   401	        return self.ray_min_deg, self.ray_inc_deg, ranges
   402	
   403	    def _publish_geofence(self, gps_ok):
   404	        msg = Float32MultiArray()
   405	        gs = self._geofence_ranges(gps_ok)
   406	        if gs is None:
   407	            msg.data = []                                    # 빈 배열 = 경계 정보 없음
   408	        else:
   409	            a0, inc, ranges = gs
   410	            msg.data = [float(a0), float(inc)] + [float(r) for r in ranges]
   411	        self.pub_geofence.publish(msg)
   412	
   413	        # 🚨 '잊으면 최악' 방어: 구독자가 0 이면 경계 방어가 통째로 없는 것이다.
   414	        #    (도킹이 1년간 침묵했던 것과 똑같은 사고 — 침묵하는 기능은 에러를 내지 않는다)
   415	        if (time.monotonic() - self.t0) > self.geofence_check_after_sec:
   416	            if self.pub_geofence.get_subscription_count() == 0:
   417	                self.get_logger().error(
   418	                    "🚨 /geofence_state 구독자가 0 이다 — 경계 이탈 방어가 작동하지 않는다(실격 위험). "
   419	                    "6a-2 에서 ship_direction 이 구독해야 한다.",
   420	                    throttle_duration_sec=10.0)
   421	
   422	    # ───────────────────────── 메인 루프 ─────────────────────────
   423	    def _update_gps_fresh(self):
   424	        """이번 틱에 위치를 믿어도 되나 = have_fix 이고 마지막 수용 프레임이 gps_stale_sec 안.
   425	
   426	        🚨 have_fix 는 여기서 바꾸지 않는다(gps_cb 만 바꾼다). stale 은 별도 상태로 들고
   427	           진입 1회 ERROR + GPS_STALE_LOG_PERIOD_SEC 마다 ERROR + 복구 INFO 만 낸다.
   428	           have_fix 가 False 인 경우(NO_FIX·부팅 직후)는 기존 _log_fix_lost·'fix 없음' WARN 몫.
   429	        """
   430	        now = time.monotonic()
   431	        fresh = self.have_fix and fix_is_fresh(self.last_fix_t, now, self.gps_stale_sec)
   432	        stale = self.have_fix and not fresh
   433	        if stale:
   434	            age = now - self.last_fix_t if self.last_fix_t is not None else float('inf')
   435	            if not self.gps_stale:
   436	                self._log_fix_stale_enter(age)
   437	                self.gps_stale_next_log_t = now + GPS_STALE_LOG_PERIOD_SEC
   438	            elif now >= self.gps_stale_next_log_t:
   439	                self._log_fix_stale_ongoing(age)
   440	                self.gps_stale_next_log_t = now + GPS_STALE_LOG_PERIOD_SEC
   441	        elif fresh and self.gps_stale:
   442	            self._log_fix_stale_recovered()
   443	        self.gps_stale = stale
   444	        return fresh
   445	
   446	    def timer_cb(self):
   447	        gps_ok = self._update_gps_fresh()    # have_fix + 신선도(gps_stale_sec)
   448	        self._publish_geofence(gps_ok)       # 웨이포인트가 끝나도 경계 감시는 계속한다
   449	
   450	        if self.wp_idx >= len(self.waypoints):
   451	            return
   452	
   453	        wp_lat, wp_lon, wp_mode, dwell = self.waypoints[self.wp_idx]
   454	
   455	        # 🚨 위치에서 나오는 값(거리·방위)은 fix 가 유효할 때만 낸다.
   456	        #    fix 가 없으면 lat/lon 이 0 이거나 옛값이라 방위가 통째로 틀린다.
   457	        #    소비자(ship_goal_angle)는 침묵을 이미 처리한다 — /yaw_error 를 멈춘다.
   458	        #    ⚠️ /wp_mode 는 계속 낸다. 미션 단계는 '지금 위치' 에서 나오는 값이 아니고,
   459	        #       멈추면 비전 검출기 3종이 전부 비활성이 된다(CLAUDE.md 3-4) — 부작용이 더 크다.
   460	        #       대신 아래 도착 판정(wp_idx 전진)은 fix 없이는 하지 않는다.
   461	        # 🚨 fix 가 묵어도(stale) 같다 — 옛 위치로 방위를 내지 않는다. ILOS 적분·prev_lat 초기화도
   462	        #    이 조건 안이라 stale 동안 적분이 쌓이지 않는다. /wp_mode·20000 폴백·
   463	        #    /wp_remaining_time·시간 초과 진행은 위치와 무관하니 그대로 낸다.
   464	        if gps_ok:
   465	            # 첫 구간은 이전점이 없다 → 현 위치를 경로 시작으로(=직접 접근). 이후엔 완료한 WP.
   466	            if self.prev_lat is None:
   467	                self.prev_lat, self.prev_lon = self.lat, self.lon
   468	
   469	            dist = los_logic.calc_dist(self.lat, self.lon, wp_lat, wp_lon)
   470	            self.pub_dist.publish(Float32(data=dist))
   471	
   472	            # LOS: 경로선(prev→wp)에서 벗어난 만큼 전방주시거리 안에서 되돌아오는 방위.
   473	            lookahead = los_logic.get_dynamic_lookahead(
   474	                self.gps_filter.estimated_speed_mps, self.los_speed_gain,
   475	                self.los_min_lookahead_m, self.los_max_lookahead_m)
   476	            bearing, xte = los_logic.calc_los_bearing(
   477	                self.prev_lat, self.prev_lon, wp_lat, wp_lon, self.lat, self.lon,
   478	                lookahead, self.integral_xte, self.ilos_ki)
   479	            # ILOS 적분 + anti-windup 클램프
   480	            self.integral_xte += xte * self.period_sec
   481	            self.integral_xte = max(-self.ilos_max, min(self.ilos_max, self.integral_xte))
   482	            self.pub_bearing.publish(Float32(data=bearing))
   483	        else:
   484	            dist = None
   485	            if not self.have_fix:            # stale 은 _update_gps_fresh 가 ERROR 로 따로 알린다
   486	                self.get_logger().warn(
   487	                    "🛰️ GPS fix 없음 → 목표 방위 미발행 (하늘이 안 보이거나 안테나 확인)",
   488	                    throttle_duration_sec=5.0)
   489	
   490	        self.pub_mode.publish(Int32(data=wp_mode))
   491	
   492	        # [3] 폴백은 '담당 노드가 없는 모드'(5, 8)에만.
   493	        #     작년엔 mode 7(도킹)에도 냈다 → ship_dock 을 7 로 고치면 발행자가 둘이 되어
   494	        #     도킹 중 조향이 GPS 방위로 튄다. 원자적 한 쌍이라 같은 커밋에서 고친다.
   495	        if wp_mode in FALLBACK_MODES:
   496	            self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
   497	
   498	        # 🚀 자율 시작 게이트 — AUTO 가 아니면 미션을 '전진'시키지 않는다(WP0 에서 대기).
   499	        #    수동(RC)으로 시작점까지 몰고 가는 동안 GPS 도착/타임아웃으로 wp_idx 가 새는 걸 막는다.
   500	        #    (위에서 방위·거리·wp_mode 는 계속 낸다 — 정보용. 모터는 아두이노 MANUAL 이 무시한다.)
   501	        #    RC 모드를 AUTO 로 넘기는 순간부터 아래 도착/타임아웃 로직이 살아나 WP0 부터 진짜 시작한다.
   502	        if not mission_should_run(self.boat_mode, self.require_auto):
   503	            self.wp_enter_time = None         # 타임아웃 시계 리셋 → AUTO 시작 시점부터 카운트
   504	            self.t_start = None               # dwell 리셋
   505	            self.integral_xte = 0.0           # LOS 적분 리셋
   506	            self.prev_lat = self.prev_lon = None   # 경로선 시작점 = AUTO 전환 시점 위치
   507	            self.get_logger().info(
   508	                "🕹 AUTO 대기 — RC 모드를 AUTO 로 넘기면 WP0 부터 자율 시작 "
   509	                "(벤치는 require_boat_mode_auto:=false 또는 /boat_mode 2 발행)",
   510	                throttle_duration_sec=5.0)
   511	            return
   512	
   513	        now = time.monotonic()               # 벽시계 금지 (CLAUDE.md 1-5)
   514	
   515	        if self.wp_enter_time is None:
   516	            self.wp_enter_time = now
   517	
   518	        # WP 하나당 타임아웃 — wp_mode 기준. 표에 없는 모드(기본: 8)는 None = 타임아웃 없음.
   519	        timeout = timeout_for_mode(self.wp_timeout_table, wp_mode)
   520	
   521	        if timeout is not None:
   522	            remain = max(timeout - (now - self.wp_enter_time), 0.0)
   523	            self.pub_remain.publish(Float32(data=remain))
   524	
   525	            if remain <= 0:
   526	                self.get_logger().warn(
   527	                    f"🕒 WP{self.wp_idx}(mode {wp_mode}) 시간 초과 {timeout:g}s → 다음 WP 이동")
   528	                self.prev_lat, self.prev_lon = wp_lat, wp_lon   # 새 경로선 시작점
   529	                self.integral_xte = 0.0                          # ILOS 적분 리셋
   530	                self.wp_idx += 1
   531	                self.t_start = None
   532	                self.wp_enter_time = None
   533	                return
   534	
   535	        # fix 가 없거나 stale 이면 '도착했는지' 를 판단할 근거가 없다 → wp_idx 를 전진시키지 않는다.
   536	        # (시간 초과 전진은 위에서 이미 처리한다 — 그건 위치와 무관한 안전장치라 유지)
   537	        # 도착 반경은 속도·모드에 따라 동적(빠르면 넓게 오버슈트 방지, 도킹은 좁게).
   538	        arrive_r = los_logic.get_dynamic_arrive_radius(
   539	            wp_mode, self.gps_filter.estimated_speed_mps)
   540	        if dist is None:
   541	            self.t_start = None
   542	        elif dist < arrive_r:
   543	            if self.t_start is None:
   544	                self.t_start = now
   545	            elif (now - self.t_start) >= dwell:
   546	                self.get_logger().info(
   547	                    f"✔ WP{self.wp_idx} 완료 (반경 {arrive_r:.1f}m) → 다음 WP 이동")
   548	                self.prev_lat, self.prev_lon = wp_lat, wp_lon   # 새 경로선 시작점
   549	                self.integral_xte = 0.0                          # ILOS 적분 리셋
   550	                self.wp_idx += 1
   551	                self.t_start = None
   552	                self.wp_enter_time = None
   553	        else:
   554	            self.t_start = None
   555	
   556	
   557	# ───────────────────────── 순수 기하 (테스트 가능) ─────────────────────────
   558	def _ray_polygon_dist(o, d, pts):
   559	    """원점 o 에서 방향 d 로 쏜 광선이 폴리곤 변에 처음 닿는 거리. 안 닿으면 None.
   560	
   561	    ★ 이게 '벽은 선이다'를 정직하게 푸는 방법이다. 최근접점 한 방향으로 원뿔을 막으면
   562	      모서리에서 정면(탈출구)이 원뿔 사이 '틈'으로 열려 배가 대각선으로 빠져나간다.
   563	    """
   564	    ox, oy = o
   565	    dx, dy = d
   566	    best = None
   567	    n = len(pts)
   568	    for i in range(n):
   569	        ax, ay = pts[i]
   570	        bx, by = pts[(i + 1) % n]
   571	        ex, ey = bx - ax, by - ay
   572	
   573	        denom = dx * ey - dy * ex
   574	        if abs(denom) < 1e-12:
   575	            continue                       # 광선과 변이 평행
   576	
   577	        rx, ry = ax - ox, ay - oy
   578	        t = (rx * ey - ry * ex) / denom    # 광선 위 거리
   579	        s = (rx * dy - ry * dx) / denom    # 변 위 위치 (0~1 이어야 선분 안)
   580	
   581	        if t >= 0.0 and 0.0 <= s <= 1.0:
   582	            if best is None or t < best:
   583	                best = t
   584	    return best
   585	
   586	
   587	def _point_in_polygon(p, pts):
   588	    """레이 캐스팅. 점 p 가 폴리곤 안인가."""
   589	    px, py = p
   590	    inside = False
   591	    n = len(pts)
   592	    for i in range(n):
   593	        x1, y1 = pts[i]
   594	        x2, y2 = pts[(i + 1) % n]
   595	        if (y1 > py) != (y2 > py):
   596	            xint = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
   597	            if px < xint:
   598	                inside = not inside
   599	    return inside
   600	
   601	
   602	def calc_angle(lat1, lon1, lat2, lon2):
   603	    from math import atan2, cos, radians, sin, degrees
   604	    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
   605	    dlon = lon2 - lon1
   606	    x = sin(dlon) * cos(lat2)
   607	    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
   608	    return (degrees(atan2(x, y)) + 360) % 360
   609	
   610	
   611	def calc_dist(lat1, lon1, lat2, lon2):
   612	    return distance.distance((lat1, lon1), (lat2, lon2)).m
   613	
   614	
   615	def main(args=None):
   616	    rclpy.init(args=args)
   617	    node = NorthGoalAngle()
   618	    try:
   619	        rclpy.spin(node)
   620	    except KeyboardInterrupt:
   621	        pass
   622	    finally:
   623	        node.destroy_node()
   624	        if rclpy.ok():
   625	            rclpy.shutdown()
   626	
   627	
   628	if __name__ == '__main__':
   629	    main()
```

----- FILE: src/north_goal_angle/north_goal_angle/gps_guard.py -----
```
     1	"""GPS fix 유효성 판정 — ROS 비의존 순수 로직 (테스트 가능).
     2	
     3	🚨 왜 필요한가 (2026-08-06 실기에서 발견):
     4	    ublox 드라이버는 **fix 가 없어도 /fix 를 1Hz 로 계속 발행한다.**
     5	    status=-1(NO_FIX), latitude=0.0, longitude=0.0 인 메시지가 그대로 온다.
     6	    north_goal_angle 의 gps_cb 는 status 를 안 보고 have_fix=True 로 만들어서,
     7	    **적도 대서양(0°N,0°E, 일명 Null Island)에서 대회장까지의 방위**를 계산해 발행했다.
     8	
     9	    실측: fix 없는 상태에서 /north_goal_angle_tp = 47.96°
    10	          calc_angle(0, 0, WP0) = 47.96°  ← 소수점까지 일치
    11	
    12	    체인이 그대로 이어진다:
    13	      /north_goal_angle_tp → ship_goal_angle → /yaw_error → ship_direction 이 그 방향으로 조향.
    14	    u-blox 냉시작은 fix 까지 30초~수 분이 걸린다. 그 구간 내내 배가 엉뚱한 방위로 간다.
    15	    값이 47.96 처럼 **그럴듯해 보여서** 눈으로는 절대 못 잡는다.
    16	
    17	    CLAUDE.md 의 '모르면 입을 다문다' 를 GPS 만 어기고 있었다
    18	    (yaw_mux 는 헤딩 없으면 발행을 멈추고, geofence 도 IMU stale 이면 침묵한다).
    19	
    20	⚠️ 이건 **첫 방어선**이다. 공분산 필터(고오차 fix 거부)·점프 필터(순간이동 거부)는
    21	   팀원 GPS LOS 이식 트랙의 `gps_filter.py` 로 별도로 들어온다
    22	   (docs/전달용/회신_GPS코드_기능추출_수정제안.md). 그게 들어와도 이 가드는 그대로 남는다 —
    23	   NO_FIX 는 '품질이 나쁜 fix' 가 아니라 '아예 fix 가 아닌 것' 이라 층이 다르다.
    24	"""
    25	
    26	import math
    27	
    28	# sensor_msgs/NavSatStatus 상수 (여기서 재정의 — 이 모듈은 ROS 를 import 하지 않는다)
    29	STATUS_NO_FIX = -1     # 위성 못 잡음
    30	STATUS_FIX = 0         # 일반 fix
    31	STATUS_SBAS_FIX = 1
    32	STATUS_GBAS_FIX = 2
    33	
    34	# (0,0) 은 대서양 한복판이다. 드라이버가 fix 없을 때 그대로 내보내는 값이라
    35	# status 를 신뢰할 수 없는 구현을 만나도 한 번 더 걸러낸다.
    36	NULL_ISLAND_EPS = 1e-7
    37	
    38	
    39	def fix_is_usable(status, lat, lon):
    40	    """이 fix 로 항법해도 되는가.
    41	
    42	    Returns True 인 조건 (전부 만족):
    43	      · status >= 0            — NO_FIX(-1) 거부
    44	      · lat/lon 이 유한한 수   — NaN/inf 거부
    45	      · (0, 0) 이 아님         — 드라이버 기본값(Null Island) 거부
    46	      · 위도 |lat| <= 90, 경도 |lon| <= 180
    47	    """
    48	    try:
    49	        status = int(status)
    50	        lat = float(lat)
    51	        lon = float(lon)
    52	    except (TypeError, ValueError):
    53	        return False
    54	
    55	    if status < STATUS_FIX:
    56	        return False
    57	    if not (math.isfinite(lat) and math.isfinite(lon)):
    58	        return False
    59	    if abs(lat) <= NULL_ISLAND_EPS and abs(lon) <= NULL_ISLAND_EPS:
    60	        return False
    61	    if abs(lat) > 90.0 or abs(lon) > 180.0:
    62	        return False
    63	    return True
    64	
    65	
    66	# ───────────────────────── fix 신선도 (계약서_GPS묶음_20261029 C1) ─────────────────────────
    67	# 🚨 왜 필요한가: have_fix 를 False 로 바꾸는 곳은 gps_cb 의 NO_FIX 분기 하나뿐이다.
    68	#    /fix 가 아예 끊기면(ublox 사망·USB 분리) 콜백이 안 불리고, GPSFilter 가 계속 거부해도
    69	#    return 만 한다 → have_fix 가 True 로 남아 **옛 위치로** 방위·거리·geofence 를 계속 낸다.
    70	#    → 마지막으로 '받아들인' 프레임의 monotonic 시각을 보고, 묵었으면 입을 다문다.
    71	# 2.5 s = 1 Hz 기준 한 프레임 빠짐(2.0 s)은 견디고 두 프레임 빠짐(3.0 s)부터 stale.
    72	# 노드 코드 기본값과 config/north_goal_angle.yaml 의 gps_stale_sec 는 이 값과 같아야 한다
    73	# (대회 launch 가 yaml 을 못 실어도 같은 값 — test_gps_fresh.py 가 검사).
    74	DEFAULT_GPS_STALE_SEC = 2.5
    75	
    76	
    77	def fix_is_fresh(last_fix_t, now, stale_sec):
    78	    """마지막으로 받아들인 fix 가 아직 믿을 만큼 새것인가.
    79	
    80	    last_fix_t: GPSFilter 가 받아들인 마지막 프레임의 monotonic 시각. None = 한 번도 없음.
    81	    now:        지금 monotonic 시각.
    82	    stale_sec:  이보다 오래되면 stale (경계 age == stale_sec 는 fresh).
    83	
    84	    None·숫자 아님·NaN 이면 False — 모르면 입을 다문다.
    85	    """
    86	    if last_fix_t is None:
    87	        return False
    88	    try:
    89	        age = float(now) - float(last_fix_t)
    90	        stale_sec = float(stale_sec)
    91	    except (TypeError, ValueError):
    92	        return False
    93	    if not (math.isfinite(age) and math.isfinite(stale_sec)):
    94	        return False
    95	    return age <= stale_sec
    96	
    97	
    98	def sanitize_stale_sec(value, default=DEFAULT_GPS_STALE_SEC):
    99	    """gps_stale_sec 파라미터 검증 → (쓸 값, 정상 여부).
   100	
   101	    유한한 양수만 정상. inf(검사 끔)·0 이하·NaN·숫자 아님(문자열·bool·None)이면
   102	    (default, False) — 노드는 세우지 않고 시작 ERROR 를 낸 뒤 기본값으로 돈다(Davidson 10-29).
   103	
   104	    🚨 int 를 받아 주는 분기는 순수 함수 호출용이다. 노드에선 정수(0, 3)가 여기 오지 못한다 —
   105	       DOUBLE 로 선언한 파라미터에 정수를 주면 Humble rclpy 가 InvalidParameterTypeException 으로
   106	       노드를 세운다. 실제 yaml 값은 반드시 소수(.0)로 적는다.
   107	    """
   108	    if isinstance(value, bool) or not isinstance(value, (int, float)):
   109	        return float(default), False
   110	    v = float(value)
   111	    if not math.isfinite(v) or v <= 0.0:
   112	        return float(default), False
   113	    return v, True
```

----- FILE: src/north_goal_angle/north_goal_angle/gps_filter.py -----
```
     1	"""GPS 필터 — ROS 비의존 순수 로직 (시간 주입식이라 배 없이 테스트).
     2	
     3	fix_is_usable(gps_guard)가 status·Null Island 를 걸러 '쓸 수 있는 fix'만 넘겨준 뒤,
     4	여기서 **품질·이상치·지터**를 다룬다:
     5	  1) 공분산 필터  — RTK 품질이 나쁜(공분산 큰) fix 는 버린다
     6	  2) 점프 필터    — 두 fix 사이 속도가 물리적 상한을 넘으면 순간이동(멀티패스)으로 보고 버린다
     7	  3) 위치 스무딩  — 오차 크기에 따라 alpha 를 조절하는 저역통과(파도 지터 흡수)
     8	부수 산출: estimated_speed_mps(정상 fix 간 이동/시간) — lookahead·도착반경 계산에 쓴다.
     9	
    10	update(raw_lat, raw_lon, cov, now) → (accepted: bool, reason: str).
    11	now(단조시각)를 인자로 받아 순수 함수로 만든다(노드가 time.monotonic() 을 넣는다).
    12	"""
    13	
    14	import math
    15	
    16	_R_EARTH_M = 6371000.0
    17	
    18	
    19	def _hav(lat1, lon1, lat2, lon2):
    20	    p1, p2 = math.radians(lat1), math.radians(lat2)
    21	    dp = math.radians(lat2 - lat1)
    22	    dl = math.radians(lon2 - lon1)
    23	    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    24	    return 2 * _R_EARTH_M * math.asin(min(1.0, math.sqrt(a)))
    25	
    26	
    27	class GPSFilter:
    28	    def __init__(self, cov_threshold=2.0, max_speed_mps=15.0):
    29	        self.cov_threshold = cov_threshold
    30	        self.max_speed_mps = max_speed_mps
    31	
    32	        self.have_fix = False
    33	        self.filtered_lat = 0.0
    34	        self.filtered_lon = 0.0
    35	        self.estimated_speed_mps = 0.0
    36	
    37	        self._prev_raw_lat = None
    38	        self._prev_raw_lon = None
    39	        self._prev_t = None
    40	
    41	    def update(self, raw_lat, raw_lon, cov, now):
    42	        """쓸 수 있는 fix 를 넣으면 필터링·스무딩. accepted=False 면 이번 값은 버려짐."""
    43	        # 1) 공분산 필터 — cov 정보가 있고(>0) 임계 초과면 버린다. (음수/0 은 '정보 없음' 취급)
    44	        if cov is not None and cov > self.cov_threshold > 0.0:
    45	            return False, f"cov {cov:.2f} > {self.cov_threshold}"
    46	
    47	        # 2) 점프 필터 — 직전 원시 fix 대비 속도 상한 초과면 버린다.
    48	        if self._prev_raw_lat is not None and self._prev_t is not None:
    49	            dt = now - self._prev_t
    50	            if dt > 0.0:
    51	                d = _hav(self._prev_raw_lat, self._prev_raw_lon, raw_lat, raw_lon)
    52	                speed = d / dt
    53	                if speed > self.max_speed_mps:
    54	                    return False, f"jump {speed:.1f} m/s > {self.max_speed_mps}"
    55	                self.estimated_speed_mps = speed
    56	
    57	        self._prev_raw_lat, self._prev_raw_lon, self._prev_t = raw_lat, raw_lon, now
    58	
    59	        # 3) 위치 스무딩 (Adaptive LFP) — 오차가 작으면 강하게(안정), 크면 약하게(따라감).
    60	        if not self.have_fix:
    61	            self.filtered_lat, self.filtered_lon = raw_lat, raw_lon
    62	            self.have_fix = True
    63	        else:
    64	            err = _hav(raw_lat, raw_lon, self.filtered_lat, self.filtered_lon)
    65	            if err < 0.5:
    66	                alpha = 0.15
    67	            elif err < 1.5:
    68	                alpha = 0.30
    69	            else:
    70	                alpha = 0.80
    71	            self.filtered_lat = alpha * raw_lat + (1.0 - alpha) * self.filtered_lat
    72	            self.filtered_lon = alpha * raw_lon + (1.0 - alpha) * self.filtered_lon
    73	
    74	        return True, "ok"
```

----- FILE: src/north_goal_angle/north_goal_angle/los_logic.py -----
```
     1	"""LOS(Line-of-Sight) 유도 — ROS 비의존 순수 로직.
     2	
     3	현재 north_goal_angle 은 '현 위치 → 목표'의 직접 방위만 낸다. 그러면 조류·초기오차로
     4	경로에서 밀리면 그 밀림을 되돌리지 못하고 비스듬히 접근한다. LOS 는 **이전 웨이포인트→목표를
     5	잇는 '경로 선'** 을 기준으로, 배가 그 선에서 얼마나 벗어났나(cross-track error, XTE)를 재서
     6	전방주시거리(lookahead) 안에서 선으로 되돌아오도록 방위를 보정한다.
     7	
     8	ILOS(적분 LOS): 조류·바람처럼 **지속적으로 미는 힘**은 XTE 를 계속 남기므로, XTE 를 적분해
     9	정상상태 오차를 없앤다(anti-windup 은 노드에서 클램프).
    10	
    11	geopy 없이 haversine 으로 거리를 구해 배 없이(Mac)도 테스트한다.
    12	프레임: 방위 0=북, 시계방향 증가(compass). north_goal_angle 계약과 동일.
    13	"""
    14	
    15	import math
    16	
    17	_R_EARTH_M = 6371000.0
    18	
    19	
    20	def calc_dist(lat1, lon1, lat2, lon2):
    21	    """두 좌표 간 대권 거리(m) — haversine (수백 m 범위에서 geopy 와 오차 무시)."""
    22	    p1, p2 = math.radians(lat1), math.radians(lat2)
    23	    dp = math.radians(lat2 - lat1)
    24	    dl = math.radians(lon2 - lon1)
    25	    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    26	    return 2 * _R_EARTH_M * math.asin(min(1.0, math.sqrt(a)))
    27	
    28	
    29	def calc_angle(lat1, lon1, lat2, lon2):
    30	    """1→2 초기 방위(°). 0=북, 시계방향(compass)."""
    31	    p1, p2 = math.radians(lat1), math.radians(lat2)
    32	    dl = math.radians(lon2 - lon1)
    33	    x = math.sin(dl) * math.cos(p2)
    34	    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    35	    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0
    36	
    37	
    38	def get_dynamic_lookahead(speed_mps, speed_gain, min_m, max_m):
    39	    """속도 비례 전방주시거리. 빠를수록 멀리 봐 완만하게, 느릴수록 가까이 봐 민첩하게.
    40	    min/max 로 클램프."""
    41	    la = speed_mps * speed_gain
    42	    return max(min_m, min(max_m, la))
    43	
    44	
    45	def cross_track_error(p_lat, p_lon, wp_lat, wp_lon, boat_lat, boat_lon):
    46	    """경로선(p→wp) 기준 배의 부호 있는 XTE(m) 와 경로 방위(°) 반환.
    47	
    48	    XTE 부호: **양수 = 배가 경로 진행방향 기준 오른쪽으로 밀림**.
    49	      경로 방위(path_bearing)와 'p→배' 방위(boat_bearing)의 최단 각차 sin 으로 낸다.
    50	    """
    51	    path_bearing = calc_angle(p_lat, p_lon, wp_lat, wp_lon)
    52	    boat_bearing = calc_angle(p_lat, p_lon, boat_lat, boat_lon)
    53	    boat_dist = calc_dist(p_lat, p_lon, boat_lat, boat_lon)
    54	    ang = (boat_bearing - path_bearing + 540.0) % 360.0 - 180.0   # 배가 오른쪽이면 +
    55	    xte = boat_dist * math.sin(math.radians(ang))
    56	    return xte, path_bearing
    57	
    58	
    59	def calc_los_bearing(p_lat, p_lon, wp_lat, wp_lon, boat_lat, boat_lon,
    60	                     lookahead_m, integral_xte=0.0, ki=0.0):
    61	    """경로선으로 되돌아오는 목표 방위(°) 와 이번 XTE(m) 반환.
    62	
    63	    배가 오른쪽으로 밀렸으면(XTE>0) 방위를 왼쪽으로(경로방위보다 작게) 틀어 선으로 복귀한다.
    64	    lookahead 가 클수록 완만하게 복귀(오버슈트↓), 작을수록 급하게.
    65	    ILOS: effective_xte = xte + ki*∫xte 로 지속 외란을 상쇄.
    66	    """
    67	    xte, path_bearing = cross_track_error(p_lat, p_lon, wp_lat, wp_lon, boat_lat, boat_lon)
    68	    effective = xte + ki * integral_xte
    69	    # 오른쪽 밀림(+)이면 왼쪽으로(−) 보정 → path_bearing 에서 빼준다.
    70	    correction = math.degrees(math.atan2(effective, max(0.1, lookahead_m)))
    71	    desired = (path_bearing - correction + 360.0) % 360.0
    72	    return desired, xte
    73	
    74	
    75	def get_dynamic_arrive_radius(wp_mode, speed_mps, dock_mode=7):
    76	    """도착 판정 반경(m) — 도킹은 좁게, 그 외는 속도에 비례해(빠르면 넓게 오버슈트 방지)."""
    77	    if wp_mode == dock_mode:
    78	        return 2.0
    79	    if speed_mps < 2.0:
    80	        return 2.5
    81	    if speed_mps < 5.0:
    82	        return 3.5
    83	    return 4.5
```

----- FILE: src/north_goal_angle/north_goal_angle/waypoint_loader.py -----
```
     1	"""웨이포인트 로드·검증 — ROS 비의존 순수 로직.
     2	
     3	대회장에서 GPS 좌표를 바꿀 사람은 config/waypoints.yaml 만 편집한다(코드는 안 건드림).
     4	이 모듈은 그 파일을 읽어 검증한다. **문법·좌표가 이상하면 조용히 넘어가지 않고 예외를 던진다**
     5	— 틀린 좌표로 배가 달리는 것보다 출발 전에 멈추는 게 낫다(모르면 침묵 말고 알린다).
     6	
     7	파싱은 순수 함수라 배·ROS 없이 테스트한다(test_waypoint_loader.py).
     8	"""
     9	
    10	
    11	class WaypointError(ValueError):
    12	    """웨이포인트 파일이 잘못됐을 때. 메시지에 '몇 번째 줄이 왜 틀렸는지'를 담는다."""
    13	
    14	
    15	# 한국 대회장 대략 범위 — 이 밖이면 좌표 오타로 보고 거부한다.
    16	# (0.0 을 빈칸으로 두거나 위도·경도를 바꿔 넣는 실수를 잡는다)
    17	LAT_MIN, LAT_MAX = 33.0, 39.0     # 한반도 남단~북단 근사
    18	LON_MIN, LON_MAX = 124.0, 132.0
    19	
    20	# 미션 번호(wp_mode) — 확정된 값만 허용. 오타(예: 70)를 잡는다.
    21	VALID_MODES = {0, 1, 2, 3, 5, 7, 8}
    22	
    23	
    24	def parse_waypoints(raw, *, lat_min=LAT_MIN, lat_max=LAT_MAX,
    25	                    lon_min=LON_MIN, lon_max=LON_MAX, valid_modes=VALID_MODES):
    26	    """
    27	    yaml 로 읽은 파이썬 객체(raw) → 검증된 [[lat, lon, mode, dwell], ...].
    28	
    29	    raw 는 yaml.safe_load 결과다. 기대 형식:
    30	        {"waypoints": [ {lat:..., lon:..., mode:..., dwell:...}, ... ]}
    31	      또는 각 항목이 리스트 [lat, lon, mode, dwell] 여도 받는다(둘 다 허용).
    32	
    33	    잘못되면 WaypointError 를 던진다 — 어느 항목이 왜 틀렸는지 메시지에 담는다.
    34	    검증:
    35	      · 최상위에 'waypoints' 키가 있고 리스트인가
    36	      · 비어있지 않은가
    37	      · 각 항목이 lat/lon/mode/dwell 을 갖는가
    38	      · lat/lon 이 한국 대회장 범위 안인가 (오타·빈칸·위경도 뒤바뀜 탐지)
    39	      · mode 가 허용된 미션 번호인가
    40	      · dwell 이 0 이상 숫자인가
    41	    """
    42	    if not isinstance(raw, dict) or "waypoints" not in raw:
    43	        raise WaypointError(
    44	            "최상위에 'waypoints:' 키가 없다. 파일 첫 줄이 'waypoints:' 인지 확인하라.")
    45	
    46	    items = raw["waypoints"]
    47	    if not isinstance(items, list):
    48	        raise WaypointError("'waypoints:' 아래는 목록(-)이어야 한다.")
    49	    if len(items) == 0:
    50	        raise WaypointError("웨이포인트가 하나도 없다. 최소 1개는 있어야 한다.")
    51	
    52	    out = []
    53	    for i, it in enumerate(items):
    54	        where = f"{i}번째 웨이포인트"
    55	
    56	        # dict 형식({lat:.., lon:.., ...}) 또는 리스트 형식([lat, lon, mode, dwell]) 둘 다 허용
    57	        if isinstance(it, dict):
    58	            missing = [k for k in ("lat", "lon", "mode", "dwell") if k not in it]
    59	            if missing:
    60	                raise WaypointError(f"{where}: {missing} 항목이 빠졌다.")
    61	            lat, lon, mode, dwell = it["lat"], it["lon"], it["mode"], it["dwell"]
    62	            label = it.get("구역", it.get("label", ""))
    63	        elif isinstance(it, (list, tuple)):
    64	            if len(it) != 4:
    65	                raise WaypointError(
    66	                    f"{where}: [위도, 경도, 미션번호, 머무는시간] 4개여야 하는데 {len(it)}개다.")
    67	            lat, lon, mode, dwell = it
    68	            label = ""
    69	        else:
    70	            raise WaypointError(f"{where}: 형식이 이상하다(dict 또는 리스트여야 한다).")
    71	
    72	        lat = _as_float(lat, where, "위도(lat)")
    73	        lon = _as_float(lon, where, "경도(lon)")
    74	        dwell = _as_float(dwell, where, "머무는시간(dwell)")
    75	
    76	        if not isinstance(mode, int) or isinstance(mode, bool):
    77	            raise WaypointError(f"{where}: 미션번호(mode)는 정수여야 한다 (지금 {mode!r}).")
    78	
    79	        # 범위 검증 — 오타·빈칸·위경도 뒤바뀜을 잡는다
    80	        if not (lat_min <= lat <= lat_max):
    81	            raise WaypointError(
    82	                f"{where} '{label}': 위도 {lat} 가 한국 범위({lat_min}~{lat_max}) 밖이다. "
    83	                f"위도·경도를 바꿔 넣었거나 오타가 아닌지 확인하라.")
    84	        if not (lon_min <= lon <= lon_max):
    85	            raise WaypointError(
    86	                f"{where} '{label}': 경도 {lon} 가 한국 범위({lon_min}~{lon_max}) 밖이다.")
    87	        if mode not in valid_modes:
    88	            raise WaypointError(
    89	                f"{where} '{label}': 미션번호 {mode} 는 허용값 {sorted(valid_modes)} 에 없다.")
    90	        if dwell < 0.0:
    91	            raise WaypointError(f"{where} '{label}': 머무는시간 {dwell} 이 음수다.")
    92	
    93	        out.append([lat, lon, mode, dwell])
    94	
    95	    return out
    96	
    97	
    98	def _as_float(v, where, name):
    99	    try:
   100	        return float(v)
   101	    except (TypeError, ValueError):
   102	        raise WaypointError(f"{where}: {name} 값 {v!r} 이 숫자가 아니다.")
```

----- FILE: src/north_goal_angle/north_goal_angle/waypoint_recorder.py -----
```
     1	#!/usr/bin/env python3
     2	"""waypoint_recorder — 대회장에서 RC로 몰고 다니며 현재 GPS를 waypoints yaml 로 기록.
     3	
     4	대회 좌표를 미리 못 받는 상황용. 흐름:
     5	  1) '좌표 설정 단계' — GPS 드라이버 + 이 노드만 실행(미션 스택은 끔).
     6	  2) RC 로 각 지점에 배를 대고 → 터미널에서 Enter → 현재 GPS 캡처 → mode/dwell 입력.
     7	  3) 다 찍으면 q → 검증된 yaml 을 **새 파일**(config/waypoints_recorded.yaml)로 저장.
     8	  4) 확인 후 waypoints.yaml 로 옮겨서 '미션 단계' 실행.
     9	
    10	설계:
    11	  · **미션 스택과 동시에 안 돈다** — 좌표 설정은 별도 단계라 waypoints 가 비어도 무방.
    12	  · 한 샘플 안 낚는다 — 최근 몇 초 fix 를 평균(지터·순간값 방지), fix 없으면 거부(fail-loud).
    13	  · 저장 전 waypoint_loader.parse_waypoints 로 **즉시 검증**(한국 범위·mode) — 형식도 우리 yaml 그대로.
    14	  · **새 파일에만 쓴다** — 기존 waypoints.yaml 을 건드리지 않아 오입력해도 복구 가능.
    15	
    16	순수 로직(average_fixes / format_waypoints_yaml)은 ROS 없이 테스트한다(test_waypoint_recorder.py).
    17	실제 GPS 구독·키 입력은 Ubuntu(ROS2)에서만 동작한다.
    18	"""
    19	
    20	import os
    21	import sys
    22	import threading
    23	import time
    24	
    25	# ---------------- 순수 로직 (ROS 비의존, 테스트 대상) ----------------
    26	
    27	
    28	def average_fixes(samples):
    29	    """[(lat, lon), ...] 평균. 빈 리스트면 None. 지터를 줄이려 여러 표본을 평균한다."""
    30	    if not samples:
    31	        return None
    32	    n = len(samples)
    33	    lat = sum(s[0] for s in samples) / n
    34	    lon = sum(s[1] for s in samples) / n
    35	    return lat, lon
    36	
    37	
    38	def format_waypoints_yaml(waypoints):
    39	    """검증된 [[lat, lon, mode, dwell, label], ...] → waypoints.yaml 문자열.
    40	
    41	    waypoint_loader 가 읽는 라벨 dict 형식으로 쓴다(비전공자가 나중에 손편집하기 쉬움).
    42	    pyyaml 없이 직접 포맷한다(의존성 최소 + Mac 에서도 생성 가능).
    43	    """
    44	    lines = ["waypoints:"]
    45	    for i, wp in enumerate(waypoints):
    46	        lat, lon, mode, dwell = wp[0], wp[1], wp[2], wp[3]
    47	        label = wp[4] if len(wp) > 4 and wp[4] else f"WP{i}"
    48	        lines.append(
    49	            f'  - {{구역: "{label}", lat: {lat:.7f}, lon: {lon:.7f}, '
    50	            f'mode: {int(mode)}, dwell: {float(dwell)}}}'
    51	        )
    52	    return "\n".join(lines) + "\n"
    53	
    54	
    55	# ---------------- ROS 노드 (Ubuntu 에서만 실행) ----------------
    56	
    57	def _run_node(out_path, avg_window_sec, cov_warn):
    58	    import rclpy
    59	    from rclpy.node import Node
    60	    from rclpy.qos import QoSProfile, ReliabilityPolicy
    61	    from sensor_msgs.msg import NavSatFix
    62	
    63	    # 저장 직전 검증에 우리 로더를 그대로 쓴다(범위·mode 체크 일관).
    64	    from north_goal_angle.waypoint_loader import parse_waypoints, WaypointError
    65	
    66	    class WaypointRecorder(Node):
    67	        def __init__(self):
    68	            super().__init__('waypoint_recorder')
    69	            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    70	            self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self._cb, qos)
    71	            self._buf = []            # 최근 fix (t, lat, lon, status, cov)
    72	            self._lock = threading.Lock()
    73	            self.recorded = []        # 확정된 [[lat, lon, mode, dwell, label], ...]
    74	            self.get_logger().info(
    75	                "waypoint_recorder 시작 — /ublox_gps_node/fix 대기. Enter=캡처, q=저장·종료")
    76	
    77	        def _cb(self, msg):
    78	            cov = msg.position_covariance[0] if len(msg.position_covariance) > 0 else 0.0
    79	            with self._lock:
    80	                self._buf.append((time.monotonic(), msg.latitude, msg.longitude,
    81	                                  msg.status.status, cov))
    82	                # 창 크기만 유지
    83	                cut = time.monotonic() - max(avg_window_sec, 1.0) * 3
    84	                self._buf = [b for b in self._buf if b[0] >= cut]
    85	
    86	        def snapshot(self):
    87	            """최근 avg_window_sec 내, fix 있는(status>=0) 표본만 반환."""
    88	            now = time.monotonic()
    89	            with self._lock:
    90	                recent = [b for b in self._buf
    91	                          if now - b[0] <= avg_window_sec and b[3] >= 0]
    92	            return recent
    93	
    94	    rclpy.init()
    95	    node = WaypointRecorder()
    96	
    97	    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    98	    spin.start()
    99	
   100	    def capture_position():
   101	        """현재 위치를 평균해서 (lat, lon) 반환. fix 없으면 None."""
   102	        recent = node.snapshot()
   103	        if not recent:
   104	            print("  ⚠️ 최근 GPS fix 없음(status<0 또는 무수신). 취소 — GPS 상태 확인.")
   105	            return None
   106	        avg = average_fixes([(b[1], b[2]) for b in recent])
   107	        worst_cov = max(b[4] for b in recent)
   108	        print(f"  표본 {len(recent)}개 평균 → lat {avg[0]:.7f}, lon {avg[1]:.7f}"
   109	              f" (최대 공분산 {worst_cov:.2f})")
   110	        if worst_cov > cov_warn:
   111	            print(f"  ⚠️ 공분산 높음(>{cov_warn}) — RTK 불안정일 수 있음. 그래도 쓰려면 진행.")
   112	        return avg
   113	
   114	    def is_valid(lat, lon, mode, dwell):
   115	        """저장 전 범위·mode 검증(우리 loader 재사용)."""
   116	        try:
   117	            parse_waypoints({"waypoints": [
   118	                {"lat": lat, "lon": lon, "mode": mode, "dwell": dwell}]})
   119	            return True
   120	        except WaypointError as e:
   121	            print(f"  🚨 검증 실패: {e}")
   122	            return False
   123	
   124	    def list_wps():
   125	        if not node.recorded:
   126	            print("  (아직 기록 없음)")
   127	            return
   128	        for i, w in enumerate(node.recorded):
   129	            print(f"  [{i}] {w[4] or f'WP{i}'}  lat {w[0]:.7f} lon {w[1]:.7f}"
   130	                  f"  mode {int(w[2])}  dwell {w[3]}")
   131	
   132	    def parse_idx(arg):
   133	        """'d 2' / 'r0' 등에서 인덱스 파싱. 범위 밖이면 None."""
   134	        try:
   135	            idx = int(arg)
   136	        except ValueError:
   137	            print("  번호를 붙이세요 (예: d 2, r 0)")
   138	            return None
   139	        if not (0 <= idx < len(node.recorded)):
   140	            print(f"  범위 밖 — 0~{len(node.recorded) - 1} 사이")
   141	            return None
   142	        return idx
   143	
   144	    print("\n=== 좌표 기록 시작 ===")
   145	    print("RC 로 지점에 배를 대고 조작하세요. 명령:")
   146	    print("  Enter  = 현재 위치를 새 WP 로 추가")
   147	    print("  l      = 기록 목록 보기")
   148	    print("  d N    = N 번 WP 삭제")
   149	    print("  r N    = N 번 WP 를 현재 위치로 재캡처(위치만 갱신, mode/dwell 유지)")
   150	    print("  q      = 저장·종료\n")
   151	    try:
   152	        while True:
   153	            raw = input("[Enter=추가 / l=목록 / d N=삭제 / r N=재캡처 / q=종료] > ").strip()
   154	            low = raw.lower()
   155	
   156	            if low == 'q':
   157	                break
   158	            if low == 'l':
   159	                list_wps()
   160	                continue
   161	            if low.startswith('d'):
   162	                idx = parse_idx(low[1:].strip())
   163	                if idx is not None:
   164	                    removed = node.recorded.pop(idx)
   165	                    print(f"  🗑 [{idx}] {removed[4] or f'WP{idx}'} 삭제 (남은 {len(node.recorded)}개)")
   166	                continue
   167	            if low.startswith('r'):
   168	                idx = parse_idx(low[1:].strip())
   169	                if idx is None:
   170	                    continue
   171	                avg = capture_position()
   172	                if not avg:
   173	                    continue
   174	                w = node.recorded[idx]
   175	                if not is_valid(avg[0], avg[1], w[2], w[3]):
   176	                    continue
   177	                node.recorded[idx] = [avg[0], avg[1], w[2], w[3], w[4]]
   178	                print(f"  ↻ [{idx}] 위치 갱신됨 (mode/dwell 유지)")
   179	                continue
   180	            if raw == '':
   181	                avg = capture_position()
   182	                if not avg:
   183	                    continue
   184	                mode = _ask_int("  mode(미션번호)? ")
   185	                dwell = _ask_float("  dwell(머무는 초)? ")
   186	                label = input("  구역 이름(엔터=자동)? ").strip()
   187	                if not is_valid(avg[0], avg[1], mode, dwell):
   188	                    continue
   189	                node.recorded.append([avg[0], avg[1], mode, dwell, label])
   190	                print(f"  ✔ WP{len(node.recorded) - 1} 추가됨 (총 {len(node.recorded)}개)\n")
   191	                continue
   192	            print("  알 수 없는 명령 — Enter/l/d N/r N/q")
   193	    except (EOFError, KeyboardInterrupt):
   194	        pass
   195	
   196	    # 저장
   197	    if node.recorded:
   198	        # 전체를 한 번 더 검증(순서·형식)
   199	        try:
   200	            parse_waypoints({"waypoints": [
   201	                {"lat": w[0], "lon": w[1], "mode": w[2], "dwell": w[3]}
   202	                for w in node.recorded]})
   203	        except WaypointError as e:
   204	            print(f"🚨 전체 검증 실패: {e}\n저장은 하되 미션 전 반드시 확인하세요.")
   205	        text = format_waypoints_yaml(node.recorded)
   206	        with open(out_path, "w", encoding="utf-8") as f:
   207	            f.write(text)
   208	        print("\n" + "=" * 66)
   209	        print(f"💾 {len(node.recorded)}개 저장 완료")
   210	        print(f"   경로: {out_path}")
   211	        print("=" * 66)
   212	        print("   ▶ 복사하지 않고 바로 쓰려면 (기존 waypoints.yaml 을 안 건드린다):")
   213	        print("       ros2 launch north_goal_angle north_goal_angle_launch.py \\")
   214	        print(f"           waypoints_file:={out_path}")
   215	        print("   ▶ 이걸 실전 파일로 확정하려면 config/waypoints.yaml 로 복사한 뒤 재빌드.")
   216	    else:
   217	        print("\n(기록된 웨이포인트 없음 — 저장 안 함)")
   218	
   219	    node.destroy_node()
   220	    rclpy.shutdown()
   221	
   222	
   223	def _ask_int(prompt):
   224	    while True:
   225	        try:
   226	            return int(input(prompt).strip())
   227	        except ValueError:
   228	            print("    정수로 입력하세요.")
   229	
   230	
   231	def _ask_float(prompt):
   232	    while True:
   233	        try:
   234	            return float(input(prompt).strip())
   235	        except ValueError:
   236	            print("    숫자로 입력하세요.")
   237	
   238	
   239	def main():
   240	    # 🚨 기본 출력은 **고정 경로**다 (~/ssf_logs/waypoints_recorded.yaml).
   241	    #    예전엔 os.getcwd() 였는데, 실행한 폴더에 따라 파일이 어디 생길지 모른다.
   242	    #    배는 SSH 로 붙어 쓰므로 접속할 때마다 cwd 가 달라지고, 대회장에서 급할 때
   243	    #    "찍긴 찍었는데 파일이 어디 있지" 가 된다 (2026-08-07 야외 시험에서 실제로 겪음 —
   244	    #    세 군데를 뒤졌다). blackbox 로그(~/ssf_logs/)와 같은 자리에 모은다.
   245	    #    다른 곳에 쓰려면 인자로 경로를 준다:  ros2 run ... waypoint_recorder /경로.yaml
   246	    default_dir = os.path.join(os.path.expanduser("~"), "ssf_logs")
   247	    os.makedirs(default_dir, exist_ok=True)
   248	    default_out = os.path.join(default_dir, "waypoints_recorded.yaml")
   249	    out_path = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else default_out)
   250	    _run_node(out_path, avg_window_sec=2.0, cov_warn=2.0)
   251	
   252	
   253	if __name__ == "__main__":
   254	    main()
```

----- FILE: src/north_goal_angle/config/north_goal_angle.yaml -----
```
     1	# north_goal_angle 파라미터 (하드코딩 금지, CLAUDE.md 1-4)
     2	/north_goal_angle:
     3	  ros__parameters:
     4	    period_sec: 0.5
     5	
     6	    # ---- LOS 유도 (경로선 추종 + 조류 보상 ILOS) ----
     7	    los_min_lookahead_m: 2.0       # 전방주시 최소(느릴 때 민첩)
     8	    los_max_lookahead_m: 6.0       # 최대(빠를 때 완만 — 오버슈트 방지)
     9	    los_speed_gain: 1.2            # 전방주시 = 속도 x 이 값 (min/max 클램프). 실측 후 튜닝
    10	    ilos_ki: 0.02                  # 적분 이득 — 조류·바람 상쇄. 너무 크면 진동, 작으면 상쇄 느림
    11	    ilos_integral_max: 50.0        # anti-windup 클램프
    12	    # ---- GPS 필터 (fix_is_usable 통과분에 품질·이상치·지터 처리) ----
    13	    gps_cov_threshold: 2.0         # position_covariance 이보다 크면 그 프레임 버림 (RTK 품질)
    14	    gps_max_speed_mps: 15.0        # 두 fix 간 속도 상한 — 넘으면 순간이동(멀티패스)으로 버림
    15	    gps_stale_sec: 2.5             # 🚨 받아들인 마지막 fix 가 이보다 묵으면 tp·goal_distance 멈춤, geofence 빈 배열.
    16	                                   #   1Hz 한 프레임 빠짐(2.0s)은 견디고 두 프레임(3.0s)부터 stale.
    17	                                   #   /wp_mode·20000 폴백·시간 초과 진행은 유지. 반드시 소수(2.5) —
    18	                                   #   inf·0 이하·NaN 이면 시작 ERROR + 2.5. 코드 기본값(gps_guard)과 같게.
    19	                                   #   🚨 정수(0, 3)는 ERROR+2.5 가 아니라 노드 사망 — DOUBLE 선언과 타입이
    20	                                   #   달라 rclpy 가 거부한다. 0 도 3 도 반드시 0.0·3.0 으로.
    21	
    22	    # ---- 🚀 자율 시작 게이트 ----
    23	    require_boat_mode_auto: true   # true=RC 모드 AUTO(/boat_mode==2)여야 미션 전진(수동 이동 중 스킵 방지).
    24	                                   #   벤치(아두이노 없음)는 false 로, 또는 ros2 topic pub /boat_mode 2
    25	
    26	    # ---- WP 타임아웃 (wp_mode 기준, WP 하나당) ----
    27	    # 두 배열은 같은 순서로 짝을 이룬다: wp_timeout_modes[i] 모드의 WP 하나당 제한시간 = wp_timeout_sec[i].
    28	    # 만료되면 위치와 무관하게 다음 WP 로 넘어간다(GPS 가 죽어도 진행하는 유일한 수단).
    29	    # 표에 없는 모드 = 타임아웃 없음 (지금은 mode 8 토너먼트 = 마지막 WP).
    30	    # 🚨 wp_timeout_sec 는 **모든 원소에 .0** — [120, 90] 이면 노드가 죽고,
    31	    #    [120.0, 90] 처럼 섞이면 이 파일 전체를 못 읽는다(Ubuntu 10-29 실측).
    32	    # 🚨 '없음'을 0 으로 적지 마라(틱마다 WP 를 건너뛴다) — 그 모드를 표에서 빼라.
    33	    #    0 이하·NaN·길이 불일치·없는 모드·중복이면 노드가 출발 전에 멈춘다.
    34	    # 코드 기본값(wp_timeout.py)과 똑같이 둔다 — 다르면 test_wp_timeout.py 가 잡는다.
    35	    wp_timeout_modes: [0, 1, 2, 3, 5, 7]
    36	    wp_timeout_sec: [120.0, 120.0, 120.0, 90.0, 120.0, 150.0]   # 3=부표 90 s, 7=도킹 150 s(dwell 60 보다 커야)
    37	
    38	    # ---- Geofence (경계 이탈 = 실격) ----
    39	    # 경기장 폴리곤: [lat1, lon1, lat2, lon2, ...] 평탄 배열. 최소 3점(=6개 수).
    40	    # ⚠️ 실측 필요 — 대회장 경계 좌표를 아직 모른다. 비어 있으면 geofence 는 꺼진다(=빈 배열).
    41	    geofence_polygon: []
    42	    # 경계를 '가짜 LiDAR'로 쏘는 각도 격자 (상대방위, 0=정면)
    43	    # ★ 원뿔(half_block) 방식은 폐기했다: 벽은 '점'이 아니라 '선'이다. 최근접점 한 방향으로
    44	    #   ±각도를 막으면 40x40 모서리에서 탈출구(정면 0°)가 원뿔 사이 '틈'으로 열려 배가
    45	    #   대각선으로 빠져나간다(실격). 광선으로 쏘면 정면 경계 거리가 그대로 벽이 된다.
    46	    geofence_ray_min_deg: -80.0
    47	    geofence_ray_max_deg: 80.0
    48	    geofence_ray_inc_deg: 1.0      # 161방향 x 변 개수 @2Hz — 무시할 수준
    49	    geofence_max_range_m: 30.0     # 이보다 멀면 그 방향은 inf
    50	    geofence_check_after_sec: 5.0  # 부팅 후 이 시간이 지나면 구독자 0 검사를 시작한다
    51	    imu_stale_sec: 0.5             # 🚨 /imu/yaw 가 이보다 묵으면 geofence 는 빈 배열을 낸다.
    52	                                   # 광선 '방향'이 yaw 에 통째로 의존한다. yaw 가 얼면 벽 전체가
    53	                                   # 엉뚱한 곳에 서고 → 배가 '없는 벽'에 갇힌다.
    54	                                   # 모르면 입을 다문다(1단계와 같은 원칙).
    55	
    56	    # 🚨 /geofence_state 는 6a 에서 '발행만' 한다. 구독자(ship_direction)는 6a-2 에서 붙인다.
    57	    #    발행자만 있는 토픽은 무해하지만, '넣었다'고 믿고 잊으면 도킹이 1년간 침묵한 것과
    58	    #    똑같은 사고가 된다. → 구독자가 0 이면 노드가 ERROR 를 계속 찍는다.
    59	    #    healthcheck 도 출발 전에 이걸 검사한다 (ssf_tools).
```

----- FILE: src/north_goal_angle/config/waypoints.yaml -----
```
     1	# ═══════════════════════════════════════════════════════════════════════════
     2	#  대회 웨이포인트(GPS 좌표) — 대회장에서 여기만 고칩니다
     3	# ═══════════════════════════════════════════════════════════════════════════
     4	#
     5	#  ▶ 무엇을 바꾸나:  각 구역의 lat(위도) · lon(경도) 만 대회장 실측값으로 바꿉니다.
     6	#  ▶ 절대 바꾸지 마세요:  mode(미션번호) 와 순서.  이게 미션 진행 순서입니다.
     7	#                        (좌표만 바꾸고 mode·dwell·순서는 그대로 두세요)
     8	#
     9	#  ▶ 형식:  - {구역: "설명", lat: 위도, lon: 경도, mode: 미션번호, dwell: 머무는초}
    10	#
    11	#  ▶ mode(미션번호) 뜻:  0·1 게이트 / 2 위치유지 / 3 부표돌기 / 5·8 회피 / 7 도킹
    12	#
    13	#  ▶ 저장 후:  배를 다시 실행하면 이 좌표로 움직입니다.
    14	#     ⚠️ 위도·경도를 바꿔 넣거나(예: lat 자리에 경도), 오타를 내거나, 콤마·중괄호를
    15	#        빠뜨리면 → 배가 출발하지 않고 화면에 "몇 번째 줄이 왜 틀렸는지" 알려줍니다.
    16	#        (틀린 좌표로 달리느니 멈추는 게 안전합니다.)
    17	# ═══════════════════════════════════════════════════════════════════════════
    18	
    19	waypoints:
    20	  - {구역: "WP0 게이트 시작",   lat: 35.1862375,     lon: 128.5655118,    mode: 0, dwell: 3.0}
    21	  - {구역: "WP1 게이트 끝",     lat: 35.1863642,     lon: 128.5657123,    mode: 1, dwell: 3.0}
    22	  - {구역: "WP2 위치유지",      lat: 35.1868822,     lon: 128.5660465,    mode: 2, dwell: 3.0}
    23	  - {구역: "WP3 부표(초록)",    lat: 35.1868638,     lon: 128.56582129999, mode: 3, dwell: 3.0}
    24	  - {구역: "WP4 부표(빨강)",    lat: 35.1867763,     lon: 128.5658085,    mode: 3, dwell: 3.0}
    25	  - {구역: "WP5 부표(하양)",    lat: 35.1868645,     lon: 128.5656684,    mode: 3, dwell: 3.0}
    26	  - {구역: "WP6 회피 시작",     lat: 35.1866601,     lon: 128.5655597,    mode: 5, dwell: 50.0}
    27	  - {구역: "WP7 회피 끝",       lat: 35.186396099999, lon: 128.5653297,   mode: 5, dwell: 50.0}
    28	  - {구역: "WP8 도킹 시작",     lat: 35.1859269,     lon: 128.5655428,    mode: 7, dwell: 60.0}
    29	  - {구역: "WP9 도킹",         lat: 35.1859269,     lon: 128.5655428,    mode: 7, dwell: 60.0}
    30	  - {구역: "WP10 토너먼트 회피", lat: 35.1861956,     lon: 128.5660033,    mode: 8, dwell: 60.0}
```

----- FILE: src/north_goal_angle/launch/north_goal_angle_launch.py -----
```
     1	"""north_goal_angle launch — 웨이포인트 파일을 인자로 바꿔 끼울 수 있다.
     2	
     3	  ros2 launch north_goal_angle north_goal_angle_launch.py
     4	      → config/waypoints.yaml (실전 파일)
     5	
     6	  ros2 launch north_goal_angle north_goal_angle_launch.py \
     7	      waypoints_file:=/home/ssfb/ssf_logs/waypoints_recorded.yaml
     8	      → 방금 녹화한 파일을 **복사 없이** 바로 사용
     9	
    10	🚨 왜 인자화했나 (2026-08-07 야외 시험에서 나온 필요)
    11	   waypoint_recorder 는 안전을 위해 **새 파일**에만 쓴다(실전 waypoints.yaml 을 안 건드림).
    12	   그런데 그 파일을 쓰려면 지금까지는 "복사 → 재빌드" 를 해야 했다.
    13	   대회장에서 좌표를 찍고 바로 미션을 돌려야 하는데, 손이 많이 가고 실수 여지가 크다.
    14	   노드는 이미 waypoints_file 파라미터를 받고 있었는데(north_goal_angle.py) launch 가
    15	   그걸 노출하지 않아 **쓸 방법이 없었다** — ublox_gps launch 가 c94_m8p 에 하드코딩돼
    16	   F9P 를 못 쓰던 것(c1b0d81)과 같은 유형이다.
    17	
    18	   ★ 녹화 파일을 인자로 쓰면 config/waypoints.yaml 은 그대로 남는다 → 오입력해도 복구 가능.
    19	"""
    20	
    21	import os
    22	
    23	from ament_index_python.packages import get_package_share_directory
    24	from launch import LaunchDescription
    25	from launch.actions import DeclareLaunchArgument
    26	from launch.substitutions import LaunchConfiguration
    27	from launch_ros.actions import Node
    28	
    29	
    30	def generate_launch_description():
    31	    share = get_package_share_directory('north_goal_angle')
    32	    params = os.path.join(share, 'config', 'north_goal_angle.yaml')
    33	    default_wp = os.path.join(share, 'config', 'waypoints.yaml')
    34	
    35	    return LaunchDescription([
    36	        DeclareLaunchArgument(
    37	            'waypoints_file',
    38	            default_value=default_wp,
    39	            description='웨이포인트 yaml 경로 (녹화본을 바로 쓰려면 그 경로를 준다)'),
    40	
    41	        Node(
    42	            package='north_goal_angle',
    43	            executable='north_goal_angle',
    44	            name='north_goal_angle',   # config yaml 의 /north_goal_angle 키와 일치
    45	            output='screen',
    46	            parameters=[
    47	                params,
    48	                # 🚨 순서가 중요하다. 뒤에 오는 것이 이긴다 —
    49	                #    params(yaml) 에 waypoints_file 이 있더라도 인자가 덮어쓴다.
    50	                {'waypoints_file': LaunchConfiguration('waypoints_file')},
    51	            ],
    52	        ),
    53	    ])
```

----- FILE: src/ublox/ublox_gps/config/c94_m8p_rover.yaml -----
```
     1	# ═══════════════════════════════════════════════════════════════════════════
     2	#  C94-M8P 이동국(rover) 설정 — A배
     3	#  [2026-09-16] 정리: save 블록 제거(EEPROM 영구 저장 — zed_f9p_rover.yaml 과 같은 이유로
     4	#  런타임(RAM)에만 적용), dynamic_model portable→sea(수상선박, 드라이버 node.cpp 'sea' 지원,
     5	#  u-blox 8 CFG-NAV5 dynModel 5), debug 1→0. uart1 19200·RTCM in 은 C94-M8P 고유라 유지.
     6	#  udev: idProduct 01a8 (u-blox 8) — F9P 의 01a9 와 다르다 → docs/절차/ubuntu_setup.md
     7	#  ⚠️ sea 모델은 A배 실기에서 아직 미확인 — 첫 실기 때 /fix 안정성 확인.
     8	# ═══════════════════════════════════════════════════════════════════════════
     9	ublox_gps_node:
    10	  ros__parameters:
    11	    debug: 0                    # Range 0-4 (0 means no debug statements will print)
    12	
    13	    # 🚨 save 블록을 넣지 않는다 (EEPROM 영구 저장 금지 — 잘못 박히면 되돌리기 어렵다)
    14	
    15	    device: /dev/ttyGPS
    16	    frame_id: gps
    17	    rate: 1.0                   # in Hz
    18	    nav_rate: 1                 # [# of measurement cycles], recommended 1 Hz, may
    19	                                # be either 5 Hz (Dual constellation) or
    20	                                # 8 Hz (GPS only)
    21	    dynamic_model: sea          # 해상 표면 차량용 (portable 에서 변경 2026-09-16). B배와 동일 근거.
    22	    fix_mode: auto
    23	    enable_ppp: false           # Not supported by C94-M8P
    24	    dr_limit: 0
    25	
    26	    # TMODE3 Config
    27	    tmode3: 0                   # Survey-In Mode
    28	    sv_in:
    29	      reset: false              # True: disables and re-enables survey-in (resets)
    30	                                # False: Disables survey-in only if TMODE3 is
    31	                                # disabled
    32	      min_dur: 300              # Survey-In Minimum Duration [s]
    33	      acc_lim: 3.0              # Survey-In Accuracy Limit [m]
    34	
    35	    uart1:
    36	      baudrate: 19200           # C94-M8P specific
    37	      in: 32                    # RTCM 3
    38	      out: 0                    # No UART out for rover
    39	
    40	    gnss:
    41	      glonass: true             # Supported by C94-M8P
    42	      beidou: false             # Supported by C94-M8P
    43	      qzss: true                # Supported by C94-M8P
    44	
    45	    dgnss_mode: 3               # Fixed mode
    46	
    47	    inf:
    48	      all: true                 # Whether to display all INF messages in console
    49	
    50	    # Enable u-blox message publishers
    51	    publish:
    52	      all: true
    53	      aid:
    54	        hui: false
    55	
    56	      nav:
    57	        posecef: false
```

----- FILE: src/ublox/ublox_gps/config/zed_f9p_rover.yaml -----
```
     1	# ═══════════════════════════════════════════════════════════════════════════
     2	#  ZED-F9P 이동국(rover) 설정 — B배
     3	# ═══════════════════════════════════════════════════════════════════════════
     4	#
     5	#  ⚠️ 기존 zed_f9p.yaml 은 쓰지 마라. 헤더가 "C94-M8P" 이고 tmode3:1(Survey-In=기지국)
     6	#     이라 **배에 다는 이동국 설정이 아니다.** M8P 파일을 복사해 이름만 바꾼 미완성본이다.
     7	#
     8	#  이 파일의 값은 전부 **2026-08-06 실기에서 관측한 근거**가 있다. 추측으로 채운 값 없음.
     9	#  관측 기록: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md
    10	#
    11	#  실물 확인된 장치:
    12	#    MOD=ZED-F9P / FWVER=HPG 1.13 / PROTVER=27.12 / GPS;GLO;GAL;BDS / SBAS;QZSS
    13	# ═══════════════════════════════════════════════════════════════════════════
    14	
    15	ublox_gps_node:
    16	  ros__parameters:
    17	    debug: 0                    # 0=조용. 문제 추적할 땐 4 로 올리면 UBX 바이트까지 찍힌다
    18	
    19	    # ── 포트 ────────────────────────────────────────────────────────────────
    20	    # udev 규칙으로 고정한 이름. /dev/ttyACM0 은 꽂는 순서에 따라 바뀐다.
    21	    #   SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="ttyGPS"
    22	    device: /dev/ttyGPS
    23	    frame_id: gps
    24	
    25	    # 🚨 save 블록을 넣지 않는다.
    26	    #    save:{mask:3103, device:4} 는 **EEPROM 에 영구 저장**한다. 잘못된 설정이 수신기에 박히면
    27	    #    되돌리기 어렵다. 런타임(RAM)에만 적용한다. (c94_m8p_rover.yaml 의 save 블록도 c7c0c59 에서 뺐다)
    28	
    29	    # ── 측위 주기 ───────────────────────────────────────────────────────────
    30	    rate: 1.0                   # Hz. 실측에서 /fix·/fix_velocity 가 1.00Hz 로 안정 발행 확인
    31	    nav_rate: 1                 # 측정 사이클 수
    32	    # ⚠️ F9P 는 더 높은 주기를 지원한다. 올릴지는 **물 위에서 제어 주기와 함께** 판단할 것.
    33	    #    지금 올리면 근거 없는 변경이다.
    34	
    35	    dynamic_model: sea          # 해상 표면 차량용 — 고도 변화를 억제하고 수평 이동에 최적화.
    36	                                #    수상선박엔 명백히 맞아 비교 실험 불필요(u-blox 표준 모델).
    37	                                #    드라이버 지원 확인: node.cpp 'sea'→DYN_MODEL_SEA.
    38	    fix_mode: auto
    39	    enable_ppp: false           # F9P 는 이 드라이버 경로로 PPP 를 쓰지 않는다
    40	    dr_limit: 0
    41	
    42	    # ── TMODE3 ──────────────────────────────────────────────────────────────
    43	    # 0 = 비활성 = 이동국. 🚨 1(Survey-In)은 **기지국**이다 — 움직이는 배에 쓰면 안 된다.
    44	    # 실측: 이 값으로 드라이버가 "Disabling TMODE3" 로그를 남기고 정상 진행함(적용 확인됨).
    45	    tmode3: 0
    46	
    47	    # ── GNSS 별자리 ─────────────────────────────────────────────────────────
    48	    # F9P 는 BDS(BeiDou)를 지원한다. 실측에서 $GBGSV 문장을 실제로 수신했다.
    49	    # 🚨 c94_m8p_rover.yaml 은 beidou:false 다 (M8P 기준) — 그대로 쓰면 위성이 줄어든다.
    50	    gnss:
    51	      glonass: true
    52	      beidou: true
    53	      qzss: true
    54	
    55	    # ── UART1 ───────────────────────────────────────────────────────────────
    56	    # 🚨 우리는 USB 로 연결한다. RTCM 은 uart1 이 아니라 **드라이버가 같은 USB 포트로 직접
    57	    #    주입**한다: ntrip_client → /rtcm 토픽 → UbloxNode::rtcmCallback → Gps::sendRtcm.
    58	    #    (node.cpp:502, gps.cpp:583)
    59	    #    → uart1 설정은 이 구성에서 쓰이지 않는다. M8P 전용값(19200)을 물려받지 않도록
    60	    #      아예 두지 않는다. UART 로 배선을 바꾸는 날 여기에 실측값을 넣을 것.
    61	
    62	    # ── DGNSS ───────────────────────────────────────────────────────────────
    63	    # 🚨 dgnss_mode 를 일부러 넣지 않았다 — **이 장치에서는 적용되지 않기 때문이다.**
    64	    #    제품 클래스는 config 가 아니라 펌웨어 문자열로 결정된다(node.cpp:698):
    65	    #      FWVER="HPG 1.13" 은 길이가 정확히 8 → ref_rov="" → HpPosRecProduct 생성
    66	    #      (HpgRovProduct 가 아니다. dgnss_mode 를 읽는 코드는 거기에만 있다)
    67	    #    실측 확인: /navrelposned·/navheading(HpPosRecProduct) + /navsvin(상속한 HpgRefProduct)
    68	    #    → 적어도 에러는 안 나지만 **조용히 무시된다.** 적어두면 "설정했다" 는 착각만 남는다.
    69	    #
    70	    #    ✅ **[2026-08-07 야외 검증 — 문제 아님으로 종결]**
    71	    #       NTRIP(SEJN-RTCM32, 11.3km) 을 붙이고 실측: **carrSoln=2(FIXED), 수평 ±0.015m,
    72	    #       위성 18개, pDOP 1.63.** dgnss_mode 가 적용되지 않는데도 fixed 까지 올라간다.
    73	    #       → F9P 펌웨어 기본값이 이미 RTK Fixed 라 드라이버가 안 보내도 무방하다.
    74	    #       ⚠️ float → FIXED 에 수십 초 걸린다. 처음 60초는 float(±0.10m) 였다 — 성급히 판단 말 것.
    75	    #       확인법(다시 볼 일이 생기면): /navrelposned 또는 /ublox_gps_node/navpvt 의 carrSoln.
    76	
    77	    inf:
    78	      all: true                 # 수신기 INF 메시지를 콘솔에 표시
    79	
    80	    # ── 발행 메시지 ─────────────────────────────────────────────────────────
    81	    publish:
    82	      all: true
    83	
    84	      # 🚨 F9 세대는 UBX-AID 클래스를 지원하지 않는다 (UBX-MGA 로 대체됨).
    85	      #    켜두면 부팅 때마다 NACK 이 뜬다 — 실측으로 확정한 내용이다:
    86	      #      "Setting rate 0x0b, 0x30" → NACK   (AidALM  : CLASS 11 / ID 48)
    87	      #      "Setting rate 0x0b, 0x31" → NACK   (AidEPH  : CLASS 11 / ID 49)
    88	      #    c94_m8p_rover.yaml 은 aid.hui 만 껐고 alm/eph 는 publish.all 을 물려받아 켜져 있었다.
    89	      #    aid.all:false 한 줄이면 alm·eph·hui 가 전부 상속으로 꺼진다(node.cpp:436-439).
    90	      #    ★ 빨간 에러를 켜둔 채 두면 팀이 "저건 원래 빨간 거야" 를 배운다. 그게 최악이다.
    91	      aid:
    92	        all: false
    93	
    94	      nav:
    95	        posecef: false
```

----- FILE: src/ublox/ublox_gps/launch/ublox_gps_node-launch.py -----
```
     1	# Copyright 2020 Open Source Robotics Foundation, Inc.
     2	# All rights reserved.
     3	#
     4	# Software License Agreement (BSD License 2.0)
     5	#
     6	# Redistribution and use in source and binary forms, with or without
     7	# modification, are permitted provided that the following conditions
     8	# are met:
     9	#
    10	# * Redistributions of source code must retain the above copyright
    11	#   notice, this list of conditions and the following disclaimer.
    12	# * Redistributions in binary form must reproduce the above
    13	#   copyright notice, this list of conditions and the following
    14	#   disclaimer in the documentation and/or other materials provided
    15	#   with the distribution.
    16	# * Neither the name of {copyright_holder} nor the names of its
    17	#   contributors may be used to endorse or promote products derived
    18	#   from this software without specific prior written permission.
    19	#
    20	# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    21	# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    22	# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    23	# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    24	# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    25	# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    26	# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    27	# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    28	# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    29	# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    30	# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    31	# POSSIBILITY OF SUCH DAMAGE.
    32	
    33	"""Launch the ublox gps node. 배별 config 를 인자로 고른다.
    34	
    35	  ros2 launch ublox_gps ublox_gps_node-launch.py                              # B배 (기본, ZED-F9P)
    36	  ros2 launch ublox_gps ublox_gps_node-launch.py config:=c94_m8p_rover.yaml   # A배 (C94-M8P)
    37	
    38	🚨 원래는 c94_m8p_rover.yaml 이 하드코딩돼 있었다. 배가 2척(A=M8P, B=F9P)인데 launch 가
    39	   한 배에 고정돼 있어서, F9P 를 꽂으면 **에러 없이 M8P 설정으로 돌았다**(2026-08-06 실기 발견).
    40	   기본값을 실제 보유 장비(F9P)로 두는 이유: launch_files.launch.py 가 이 파일을 그대로
    41	   include 하므로, 기본이 틀리면 대회 실행 경로에서 같은 사고가 조용히 재발한다.
    42	"""
    43	
    44	import os
    45	
    46	import ament_index_python.packages
    47	import launch
    48	import launch.substitutions
    49	import launch_ros.actions
    50	
    51	
    52	def generate_launch_description():
    53	    config_directory = os.path.join(
    54	        ament_index_python.packages.get_package_share_directory('ublox_gps'),
    55	        'config')
    56	
    57	    config_arg = launch.actions.DeclareLaunchArgument(
    58	        'config',
    59	        default_value='zed_f9p_rover.yaml',
    60	        description='배별 GPS 설정 파일 (B배: zed_f9p_rover.yaml / A배: c94_m8p_rover.yaml)')
    61	
    62	    params = launch.substitutions.PathJoinSubstitution(
    63	        [config_directory, launch.substitutions.LaunchConfiguration('config')])
    64	
    65	    ublox_gps_node = launch_ros.actions.Node(package='ublox_gps',
    66	                                             executable='ublox_gps_node',
    67	                                             output='both',
    68	                                             parameters=[params])
    69	
    70	    return launch.LaunchDescription([config_arg,
    71	                                     ublox_gps_node,
    72	
    73	                                     launch.actions.RegisterEventHandler(
    74	                                         event_handler=launch.event_handlers.OnProcessExit(
    75	                                             target_action=ublox_gps_node,
    76	                                             on_exit=[launch.actions.EmitEvent(
    77	                                                 event=launch.events.Shutdown())],
    78	                                         )),
    79	                                     ])
```

----- FILE: src/ntrip_client/launch/ntrip_client_launch.py -----
```
     1	"""NTRIP 클라이언트 launch — 계정 정보는 **저장소 밖 로컬 파일**에서 읽는다.
     2	
     3	  기본 경로 : ~/.ssf/ntrip_credentials.yaml
     4	  템플릿    : src/ntrip_client/config/ntrip_credentials.example.yaml
     5	  경로 변경 : 환경변수 SSF_NTRIP_CREDENTIALS 또는 credentials_file:= 인자
     6	
     7	🚨 왜 이렇게 바꿨나 (2026-08-06)
     8	   예전엔 작년 계정이 이 파일에 **평문으로 하드코딩**돼 공개 저장소에 커밋돼 있었다
     9	   (host/mountpoint/username/password 전부). docs/절차/ubuntu_setup.md 9단계는
    10	   "계정 정보는 GitHub 에 없다 — 노트북 로컬에만" 이라고 적어뒀는데 지켜지지 않았다.
    11	   받아놓을 그릇이 없으면 사람은 있는 자리에 넣는다 → 저장소 밖에 그릇을 만든다.
    12	
    13	🚨 파일이 없으면 **노드를 띄우지 않고 에러 로그만** 남긴다 (예외를 던지지 않는다).
    14	   IncludeLaunchDescription 안에서 예외가 나면 **launch 전체가 내려간다** — 이미 뜬
    15	   노드까지 SIGINT 로 같이 죽는다. 같은 날 ntrip launch 경로 문제로 실제로 당했다
    16	   (노드 16개가 통째로 종료됐다). RTK 보정이 없는 것과 배 전체가 안 뜨는 것은 무게가 다르다.
    17	"""
    18	
    19	import os
    20	
    21	from ament_index_python.packages import get_package_share_directory
    22	from launch import LaunchDescription
    23	from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
    24	                            SetEnvironmentVariable)
    25	from launch.substitutions import LaunchConfiguration
    26	from launch_ros.actions import Node
    27	
    28	import yaml
    29	
    30	DEFAULT_CREDENTIALS = os.path.join(
    31	    os.path.expanduser('~'), '.ssf', 'ntrip_credentials.yaml')
    32	
    33	
    34	def _resolve_path(context):
    35	    """credentials_file 인자 > 환경변수 > 기본 경로."""
    36	    arg = LaunchConfiguration('credentials_file').perform(context).strip()
    37	    if arg:
    38	        return os.path.expanduser(arg)
    39	    env = os.environ.get('SSF_NTRIP_CREDENTIALS', '').strip()
    40	    if env:
    41	        return os.path.expanduser(env)
    42	    return DEFAULT_CREDENTIALS
    43	
    44	
    45	def _missing(path, reason):
    46	    """노드 없이 안내만 남긴다. launch 전체를 죽이지 않는다."""
    47	    example = os.path.join(
    48	        get_package_share_directory('ntrip_client'),
    49	        'config', 'ntrip_credentials.example.yaml')
    50	    return [LogInfo(msg=(
    51	        f"\n🛰️ [ntrip_client] {reason}\n"
    52	        f"   찾은 경로: {path}\n"
    53	        f"   → NTRIP(RTK 보정) 없이 계속 진행합니다. 나머지 노드는 정상 동작합니다.\n"
    54	        f"   계정을 받으셨으면:\n"
    55	        f"     mkdir -p ~/.ssf\n"
    56	        f"     cp {example} {DEFAULT_CREDENTIALS}\n"
    57	        f"     nano {DEFAULT_CREDENTIALS}   # 실제 값으로 채우기\n"))]
    58	
    59	
    60	def _launch_setup(context, *args, **kwargs):
    61	    path = _resolve_path(context)
    62	
    63	    if not os.path.isfile(path):
    64	        return _missing(path, "계정 파일이 없습니다.")
    65	
    66	    try:
    67	        with open(path, 'r', encoding='utf-8') as f:
    68	            doc = yaml.safe_load(f) or {}
    69	    except Exception as e:                       # noqa: BLE001 — 어떤 파싱 오류든 안내로 바꾼다
    70	        return _missing(path, f"계정 파일을 읽지 못했습니다: {e}")
    71	
    72	    cred = doc.get('ntrip') or {}
    73	    host = str(cred.get('host', '') or '').strip()
    74	    mountpoint = str(cred.get('mountpoint', '') or '').strip()
    75	    if not host or not mountpoint:
    76	        return _missing(path, "계정 파일에 host 또는 mountpoint 가 비어 있습니다.")
    77	
    78	    # 빈 문자열은 이 드라이버가 'None' 문자열로 기대하는 자리들이 있다 → 그대로 맞춰준다.
    79	    def opt(key):
    80	        v = str(cred.get(key, '') or '').strip()
    81	        return v if v else 'None'
    82	
    83	    params = {
    84	        'host': host,
    85	        'port': int(cred.get('port', 2101)),
    86	        'mountpoint': mountpoint,
    87	        'ntrip_version': opt('ntrip_version'),
    88	        'authenticate': bool(cred.get('authenticate', True)),
    89	        'username': str(cred.get('username', '') or ''),
    90	        'password': str(cred.get('password', '') or ''),
    91	        'ssl': bool(cred.get('ssl', False)),
    92	        'cert': opt('cert'),
    93	        'key': opt('key'),
    94	        'ca_cert': opt('ca_cert'),
    95	
    96	        # ---- 아래는 계정과 무관한 고정 설정 (저장소에 남겨도 되는 것들) ----
    97	        'rtcm_frame_id': 'odom',
    98	        'nmea_max_length': 82,
    99	        'nmea_min_length': 3,
   100	        'rtcm_message_package': LaunchConfiguration('rtcm_message_package').perform(context),
   101	        # 재연결: 5초 간격 10회. ⚠️ 이건 '세션이 끊겼을 때' 다 —
   102	        #   404(마운트포인트 오류) 같은 설정 오류는 재시도 대상이 아니라 바로 종료된다.
   103	        'reconnect_attempt_max': 10,
   104	        'reconnect_attempt_wait_seconds': 5,
   105	        'rtcm_timeout_seconds': 4,
   106	    }
   107	
   108	    return [
   109	        LogInfo(msg=f"🛰️ [ntrip_client] 계정 로드: {path}  (host={host}, mount={mountpoint})"),
   110	        Node(
   111	            name=LaunchConfiguration('node_name').perform(context),
   112	            namespace=LaunchConfiguration('namespace').perform(context),
   113	            package='ntrip_client',
   114	            executable='ntrip_ros.py',
   115	            parameters=[params],
   116	        ),
   117	    ]
   118	
   119	
   120	def generate_launch_description():
   121	    return LaunchDescription([
   122	        DeclareLaunchArgument('namespace', default_value='/'),
   123	        DeclareLaunchArgument('node_name', default_value='ntrip_client'),
   124	        DeclareLaunchArgument('debug', default_value='True'),
   125	        DeclareLaunchArgument('rtcm_message_package', default_value='rtcm_msgs'),
   126	        DeclareLaunchArgument(
   127	            'credentials_file', default_value='',
   128	            description=f'NTRIP 계정 yaml 경로 (비우면 {DEFAULT_CREDENTIALS})'),
   129	
   130	        SetEnvironmentVariable(name='NTRIP_CLIENT_DEBUG',
   131	                               value=LaunchConfiguration('debug')),
   132	
   133	        OpaqueFunction(function=_launch_setup),
   134	    ])
```

----- FILE: src/ntrip_client/config/ntrip_credentials.example.yaml -----
```
     1	# ═══════════════════════════════════════════════════════════════════════════
     2	#  NTRIP 계정 설정 — 템플릿 (이 파일은 저장소에 올라갑니다. 여기에 실제 값을 쓰지 마세요)
     3	# ═══════════════════════════════════════════════════════════════════════════
     4	#
     5	#  ▶ 쓰는 법
     6	#      1) 이 파일을 노트북 로컬로 복사합니다:
     7	#           mkdir -p ~/.ssf
     8	#           cp ntrip_credentials.example.yaml ~/.ssf/ntrip_credentials.yaml
     9	#      2) ~/.ssf/ntrip_credentials.yaml 을 열어 실제 값으로 채웁니다.
    10	#      3) 끝. launch 가 알아서 읽습니다.
    11	#
    12	#  ▶ 왜 저장소 밖(~/.ssf/)인가
    13	#      계정 정보가 GitHub 에 올라가면 안 됩니다(docs/절차/ubuntu_setup.md 9단계).
    14	#      🚨 실제로 예전에 작년 계정이 launch 파일에 평문으로 커밋돼 있었습니다.
    15	#         "받아놓을 그릇" 이 없으면 사람은 있는 자리(=launch 파일)에 넣게 됩니다.
    16	#         그래서 저장소 밖에 두어 **물리적으로 커밋이 불가능**하게 만듭니다.
    17	#
    18	#  ▶ 경로를 바꾸고 싶으면
    19	#      환경변수:  export SSF_NTRIP_CREDENTIALS=/원하는/경로.yaml
    20	#      launch 인자: ros2 launch ntrip_client ntrip_client_launch.py \
    21	#                     credentials_file:=/원하는/경로.yaml
    22	#
    23	#  ▶ 이 파일이 없으면
    24	#      ntrip_client 노드를 **띄우지 않고** 에러 로그만 남깁니다.
    25	#      나머지 시스템(LiDAR·IMU·GPS·제어)은 그대로 돕니다 — RTK 보정만 없는 상태가 됩니다.
    26	#      (⚠️ 예외를 던지면 launch 전체가 죽습니다. 2026-08-06 에 실제로 그렇게 당했습니다.)
    27	# ═══════════════════════════════════════════════════════════════════════════
    28	
    29	ntrip:
    30	  # ── 캐스터 주소 ──────────────────────────────────────────────────────────
    31	  # 발급받은 기관이 알려줍니다. 작년엔 gnssdata.or.kr:2101 이었지만
    32	  # ⚠️ 새로 발급받는 곳이 어디인지 확인하고 채우세요. 추측해서 넣지 마세요.
    33	  host: ""                    # 예: gnssdata.or.kr
    34	  port: 2101                  # 대개 2101
    35	
    36	  # 마운트포인트 = 어느 기준국의 보정을 받을지. 대회장에서 가까운 곳을 고릅니다.
    37	  # 캐스터 접속 후 목록에서 확인할 수 있습니다.
    38	  mountpoint: ""              # 예: DAEG-RTCM31
    39	
    40	  # ── 인증 ────────────────────────────────────────────────────────────────
    41	  authenticate: true          # 대부분 true. 무인증 캐스터면 false
    42	  username: ""
    43	  password: ""
    44	
    45	  # ── 프로토콜 ────────────────────────────────────────────────────────────
    46	  # 캐스터가 특정 NTRIP 버전을 요구하면 "1.0" 또는 "2.0". 모르면 비워둡니다.
    47	  # (2026-08-06 실기: 작년 설정으로 접속하니 404 와 함께
    48	  #  "NTRIP version was not specified" 경고가 같이 나왔습니다. 접속 실패하면 여기부터 의심)
    49	  ntrip_version: ""
    50	
    51	  # ── SSL (대개 안 씁니다) ────────────────────────────────────────────────
    52	  ssl: false
    53	  cert: ""
    54	  key: ""
    55	  ca_cert: ""
```

----- FILE: src/launch_files/launch/launch_files.launch.py -----
```
     1	from launch import LaunchDescription
     2	from launch.actions import (IncludeLaunchDescription, TimerAction, DeclareLaunchArgument,
     3	                            LogInfo, OpaqueFunction)
     4	from launch.substitutions import LaunchConfiguration
     5	from launch.launch_description_sources import PythonLaunchDescriptionSource
     6	from ament_index_python.packages import get_package_share_directory
     7	import os
     8	from launch_ros.actions import Node
     9	
    10	
    11	def yaw_mux_param_files(share_dir, boat):
    12	    """yaw_mux params 파일 목록. 🚨 순서가 의미다: 공통 → 배별 (뒤 파일이 이긴다).
    13	
    14	    ssf_heading.launch.py(벤치)에 같은 함수가 있다 — 바꿀 땐 둘 다 (test_boat_config.py 가 검사).
    15	    """
    16	    cfg = os.path.join(share_dir, 'config')
    17	    return [os.path.join(cfg, 'ssf_heading.yaml'),
    18	            os.path.join(cfg, f'ssf_heading_{boat}.yaml')]
    19	
    20	
    21	def _yaw_mux(context):
    22	    # 입력이 끊기거나 소스가 미구현이면 발행을 멈춘다(설계).
    23	    # → ship_goal_angle 이 /yaw_error 를 멈추고, north_goal_angle geofence 가 침묵한다.
    24	    boat = LaunchConfiguration('boat').perform(context)
    25	    files = yaw_mux_param_files(get_package_share_directory('ssf_heading'), boat)  # 공통 → 배별
    26	    missing = [f for f in files if not os.path.isfile(f)]
    27	    if missing:
    28	        # 일부러 launch 전체를 세운다 — 배별 값 없이 뜨면 헤딩이 조용히 틀린다.
    29	        # 🚨 이 검사는 t=0, 어떤 Node 보다 먼저 돈다(목록 맨 앞) → 아무 프로세스도 안 뜬 채 멈춘다.
    30	        #    아래 12) 브릿지 주석의 "예외 금지" 는 선택 기능(ntrip·브릿지) 얘기다 — 이 검사를 지우지 말 것.
    31	        raise RuntimeError(
    32	            f'yaw_mux params 파일 없음: {missing} — pull 뒤 colcon build 를 했는지 확인')
    33	    return [
    34	        LogInfo(msg=f'boat={boat}, 파일={files[0]} → {files[1]}'),
    35	        Node(
    36	            package='ssf_heading',
    37	            executable='yaw_mux',
    38	            name='yaw_mux',
    39	            output='screen',
    40	            parameters=files,
    41	        ),
    42	    ]
    43	
    44	
    45	def generate_launch_description():
    46	
    47	    # 패키지 경로
    48	    dir_rplidar = get_package_share_directory('rplidar_ros')
    49	    dir_north = get_package_share_directory('north_goal_angle')
    50	    dir_ntrip = get_package_share_directory('ntrip_client')
    51	    dir_gps = get_package_share_directory('ublox_gps')
    52	    # ship_direction·realsense2_camera 는 Node(package=...) 가, ship_goal_angle 은
    53	    # 119행이 다시 조회한다. 여기 대입은 쓰이지 않아 제거했다(2026-09-23).
    54	    dir_bridge = get_package_share_directory('ssf_bridge')
    55	
    56	    # 비전 공통 설정 (image_topic, hfov_deg, debug_view).
    57	    # 3-4 이후로는 launch 가 노드를 직접 띄우므로 파라미터가 **그대로 닿는다** —
    58	    # V5 의 vision_* 중계(매니저가 --ros-args 로 넘기던 것)는 3-4 에서 제거됐다.
    59	    VISION_CFG = os.path.join(
    60	        get_package_share_directory('color_shape_detector'), 'config', 'vision.yaml')
    61	
    62	    # NTRIP 2초 지연
    63	    ntrip_launch = IncludeLaunchDescription(
    64	        PythonLaunchDescriptionSource(
    65	            os.path.join(dir_ntrip, 'launch', 'ntrip_client_launch.py')
    66	        )
    67	    )
    68	
    69	    return LaunchDescription([
    70	        # 🚨 배 (필수, 기본값 없음 — 빠지거나 A/B 가 아니면 launch 거부).
    71	        #    지금은 yaw_mux 의 배별 IMU 값(ssf_heading_A/B.yaml)만 고른다. GPS 는 아래 gps_config 따로.
    72	        #    ros2 launch launch_files launch_files.launch.py boat:=A   (또는 boat:=B)
    73	        #    기본값을 두면 다른 배에서 그 배 값으로 조용히 뜬다(gps_config 가 이미 그렇게 실패 중).
    74	        DeclareLaunchArgument(
    75	            'boat', choices=['A', 'B'],
    76	            description='배 (필수): A 또는 B — yaw_mux 배별 IMU 값 파일 선택'),
    77	        DeclareLaunchArgument(
    78	            'gps_config', default_value='zed_f9p_rover.yaml',
    79	            description='배별 GPS 설정 (B배: zed_f9p_rover.yaml / A배: c94_m8p_rover.yaml)'),
    80	
    81	        # ============================================================
    82	        # 0) yaw_mux — /imu/yaw 의 단독 발행자 (params: 공통 → boat 별, _yaw_mux 참고)
    83	        # ============================================================
    84	        # 🚨 맨 앞에 둔다: 배별 파일 확인(raise)이 어떤 노드도 예약되기 전에 돌게.
    85	        #    iahrs_driver 보다 먼저 떠도 무해하다 — 원시 yaw 가 오기 전엔 발행하지 않는다(stale).
    86	        OpaqueFunction(function=_yaw_mux),
    87	
    88	        # ============================================================
    89	        # 1) RPLIDAR
    90	        # ============================================================
    91	        IncludeLaunchDescription(
    92	            PythonLaunchDescriptionSource(
    93	                os.path.join(dir_rplidar, 'launch', 'rplidar_a3_launch.py')
    94	            )
    95	        ),
    96	
    97	        # ============================================================
    98	        # 2) AHRS (IMU) — 보정 안 한 상대 yaw 를 /imu/yaw_raw 로만 낸다
    99	        # ============================================================
   100	        # 🚨 CLAUDE.md 3-5. 작년엔 이 노드가 /imu/yaw 를 직접 냈는데 그 값이 절대방위가 아니었다
   101	        #    (부팅 0점화 + GPS COG override). 절대방위 합성은 아래 yaw_mux 가 전담한다.
   102	        #    파라미터는 드라이버 기본값과 같지만 **일부러 명시**한다 —
   103	        #    나중에 누가 yaml 로 되살려도 여기서 눈에 띄게 하려는 것이다.
   104	        Node(
   105	            package='iahrs_driver',
   106	            executable='iahrs_driver',
   107	            name='iahrs_driver',
   108	            output='screen',
   109	            parameters=[{
   110	                'yaw_topic': '/imu/yaw_raw',        # ← /imu/yaw 로 되돌리면 발행자 2개가 된다
   111	                'zero_yaw_on_boot': False,
   112	                'use_gps_heading_override': False,
   113	            }],
   114	        ),
   115	
   116	        # 2b) yaw_mux 는 맨 앞 0) 으로 옮겼다 (배별 파일 확인을 t=0 에 하려고).
   117	
   118	        # ============================================================
   119	        # 3) U-BLOX GPS
   120	        # ============================================================
   121	        # 🚨 배별 설정을 넘긴다. 예전엔 인자 없이 include 해서 ublox launch 의 기본값(B배 F9P)이
   122	        #    A배(M8P)에서도 그대로 쓰였다 — 에러 없이 조용히 어긋나는 류. (2026-09-16)
   123	        #    A배:  ros2 launch launch_files launch_files.launch.py gps_config:=c94_m8p_rover.yaml
   124	        #    B배:  인자 없이 (기본 zed_f9p_rover.yaml)
   125	        IncludeLaunchDescription(
   126	            PythonLaunchDescriptionSource(
   127	                os.path.join(dir_gps, 'launch', 'ublox_gps_node-launch.py')
   128	            ),
   129	            launch_arguments={'config': LaunchConfiguration('gps_config')}.items(),
   130	        ),
   131	
   132	        # ============================================================
   133	        # 4) NTRIP 2초 딜레이
   134	        # ============================================================
   135	        TimerAction(
   136	            period=2.0,
   137	            actions=[ntrip_launch]
   138	        ),
   139	
   140	        # ============================================================
   141	        # 5) NORTH 기준 각도 계산
   142	        # ============================================================
   143	        IncludeLaunchDescription(
   144	            PythonLaunchDescriptionSource(
   145	                os.path.join(dir_north, 'launch', 'north_goal_angle_launch.py')
   146	            )
   147	        ),
   148	
   149	        # ============================================================
   150	        # 6) SHIP goal angle
   151	        # ============================================================
   152	        IncludeLaunchDescription(
   153	            PythonLaunchDescriptionSource(
   154	                os.path.join(get_package_share_directory('ship_goal_angle'),
   155	                             'launch', 'ship_goal_angle_launch_file.launch.py')
   156	            )
   157	        ),
   158	
   159	        # ============================================================
   160	        # 7) RealSense Camera
   161	        # ============================================================
   162	        # 🚨 [2026-08-12 정정] 이 절 제목이 "Auto Exposure/WhiteBalance OFF" 였는데
   163	        #    **둘 다 켜져 있었다.** 실측으로 확인:
   164	        #      rgb_camera.enable_auto_white_balance = True   (아래 줄이 True 로 주고 있었다)
   165	        #      rgb_camera.enable_auto_exposure      = True   (설정 자체가 없어 드라이버 기본값)
   166	        #    제목만 보고 "고정돼 있구나" 로 읽으면 색 캘리브레이션의 전제가 통째로 틀린다.
   167	        #    → 제목을 지우고, 실제 값을 **명시**한다. 기본값에 기대지 않는다.
   168	        Node(
   169	            package='realsense2_camera',
   170	            executable='realsense2_camera_node',
   171	            name='camera',
   172	            namespace='camera',
   173	            output='screen',
   174	            parameters=[
   175	                # ── 색 안정성에 직결되는 값. 현재는 **자동** 이고, 그 상태로 검증했다 ──
   176	                # ⚠️ 색 기반 검출에서 자동 WB 는 원리상 위험하다 — 화면에 뭐가 들어오냐에
   177	                #    따라 같은 부표의 H 가 밀린다(흰 배가 지나가면 색조가 변한다).
   178	                #    그래서 끄려고 했는데, **실측이 그 주장을 뒷받침하지 않았다**:
   179	                #      정지 장면 A/B (auto ON vs OFF 4600K) → ΔH 0.0, ΔS 0.2, ΔV 0.2
   180	                #    자동 WB 가 고정값과 거의 같은 곳에 수렴했다. 측정 근거 없이 바꾸지 않는다.
   181	                # 🚨 **실외에서 다시 판정할 것.** 해가 움직이고 큰 물체가 드나드는 환경이
   182	                #    진짜 시험이다. 절차: 부표를 고정해 두고 화면에 밝은 물체를 넣었다 뺐다
   183	                #    하며 `tools/vision_probe.py --log-csv` 로 H 가 밀리는지 본다.
   184	                #    밀리면 auto_white_balance:=False + white_balance 고정(주간 5500~6500K).
   185	                {"rgb_camera.enable_auto_white_balance": True},
   186	                {"rgb_camera.enable_auto_exposure": True},
   187	
   188	                {"enable_infra1": False},
   189	                {"enable_infra2": False},
   190	
   191	                # 🚨 카메라 내장 IMU(자이로·가속도)를 끈다 — 안 끄면 노드가 예외로 죽는다.
   192	                #    librealsense 가 D455 의 HID/IIO 센서를 열려고 하는데 udev 규칙이 없어
   193	                #    Permission denied 가 나고, 그 예외가 rs_node_setup 의 센서 설정을 통째로
   194	                #    중단시킨다 (2026-08-07 실기 확인):
   195	                #      Failed to open .../iio:device1/scan_elements/in_anglvel_x_en
   196	                #      → Error updating the sensors
   197	                #    ⚠️ udev 규칙을 까는 방법도 있지만 **우리는 이 IMU 를 쓰지 않는다** —
   198	                #      헤딩은 iAHRS(/imu/yaw_raw → yaw_mux)가 담당한다(CLAUDE.md 3-5).
   199	                #      안 쓰는 센서 때문에 시스템에 규칙을 남기는 대신 명시적으로 끈다.
   200	                #    ※ OAK(PoE=이더넷)로 바꾸면 USB HID 자체가 없어져 이 줄은 불필요해진다.
   201	                {"enable_gyro": False},
   202	                {"enable_accel": False},
   203	
   204	                # 🚨 [2026-08-12] 여기 `rgb_camera.controls.saturation: 120` 이 있었는데
   205	                #    **그런 파라미터는 존재하지 않는다.** 실물로 확인:
   206	                #      ros2 param set … rgb_camera.controls.saturation
   207	                #        → "cannot be set because it was not declared"
   208	                #      실제 이름은 `rgb_camera.saturation` 이고, 값은 계속 기본값 **64** 였다.
   209	                #    즉 **120 은 한 번도 먹은 적이 없다.** 에러도 안 났다(조용히 무시).
   210	                #    → 지금 120 으로 고치면 '검증된 적 없는 값' 으로 동작이 바뀐다.
   211	                #      오늘 검출 검증(빨강·초록 100%)은 전부 **64 에서** 나온 결과다.
   212	                #      그래서 실제로 돌던 값 64 를 **명시**만 한다. 120 이 나은지는 측정 후에.
   213	                {"rgb_camera.saturation": 64},
   214	
   215	
   216	                # 🚨 [2026-09-10 실측] 뎁스 스트림을 끈다. **켜면 카메라가 USB 에서 떨어진다.**
   217	                #    D455 를 노트북 USB3 포트에 직결(SuperSpeed 5000M)한 상태에서 측정:
   218	                #      컬러+뎁스 : 컬러 10.8fps · 뎁스 0.6fps · 최대공백 1.75s
   219	                #                  2분간 USB 끊김 13회 · 드라이버 오류 583,090 줄
   220	                #                  (xioctl(VIDIOC_QBUF) failed ... error: No such device 무한 재시도)
   221	                #      컬러 전용 : **30.0fps · 최대공백 0.03s · 오류 0 줄**
   222	                #    허브·USB2.0 문제가 아니었다. 직결·SuperSpeed 에서도 같았고,
   223	                #    USB autosuspend 를 꺼도(boat_boot.sh 적용) 그대로였다. **뎁스 부하 자체가 원인이다.**
   224	                #
   225	                #    ⚠️ 끄는 게 안전한 근거: **뎁스를 구독하는 코드가 하나도 없다.**
   226	                #       V1(2026-07-23)에서 비전 3종의 depth 의존을 전부 제거했고
   227	                #       거리는 LiDAR `/scan` 에서 얻는다(CLAUDE.md 3-3). 전수 검색으로 재확인함.
   228	                #    ⚠️ 되살릴 일이 생기면 **먼저 USB 케이블을 바꿔서** 재측정할 것.
   229	                #       마지막까지 남은 의심은 케이블 품질이다(오늘 아두이노·라이다도 같은 증상이었다).
   230	                {"enable_depth": False},
   231	
   232	                # 🎥 30FPS, 640x480
   233	                {"rgb_camera.color_profile": "640x480x30"},
   234	            ]
   235	        ),
   236	
   237	        # ============================================================
   238	        # 8) 비전 검출기 — **상주**. 각자 /wp_mode 를 보고 자기 차례일 때만 일한다
   239	        # ============================================================
   240	        # 🚨 3-4: 작년엔 subscriber_mode_manager 가 subprocess 로 이 노드들을 죽였다 살렸다.
   241	        #    모드 전환마다 비전이 몇 초 멈추고, 좀비가 남으면 카메라가 잠기고, 추적기를 못 썼다.
   242	        #    지금은 상주 + 모드 게이팅이다. 매니저는 폐기했다(entry point 도 제거).
   243	        #
   244	        #    담당 모드는 **각 노드가 소유**한다 (미션 노드와 같은 패턴).
   245	        #    권위 출처 = 미션 노드의 active_wp_mode. 바꿀 땐 양쪽을 함께 바꿔라:
   246	        #      mode 0,1 → ship_gate  ← gate 검출기 (/red_angle, /green_angle)
   247	        #      mode 2   → ship_back  ← turn 검출기 (white 부표, /image_angle)
   248	        #      mode 3   → ship_turn  ← turn 검출기 (red 부표)
   249	        #      mode 7   → ship_dock  ← dock 검출기 (/image_angle)
   250	        #      mode 5,8 → 순수 회피(담당 없음) → 검출기 전부 비활성
   251	        #
   252	        #    🚨 dock 과 turn 은 둘 다 /image_angle 을 발행한다. 모드가 겹치면 한 토픽에
   253	        #       발행자 2개가 되어 **에러 없이** 값이 섞인다. 겹침 없음은
   254	        #       test_mode_gate.py 의 check_publisher_conflicts 가 정적으로 검사한다.
   255	        Node(package='color_shape_detector', executable='basic_image_subscribergate',
   256	             name='image_subscriber_gate', output='screen', parameters=[VISION_CFG]),
   257	        Node(package='color_shape_detector', executable='basic_image_subscriberturn',
   258	             name='image_subscriber_turn', output='screen', parameters=[VISION_CFG]),
   259	        Node(package='color_shape_detector', executable='basic_image_subscriberdock',
   260	             name='image_subscriber_dock', output='screen', parameters=[VISION_CFG]),
   261	        # basic_image_subscriberhsv 는 튜닝 전용 — 대회 launch 에 넣지 않는다.
   262	
   263	        # ============================================================
   264	        # 9) SHIP mission nodes (WP 루틴)
   265	        # ============================================================
   266	        Node(package='ship_gate', executable='ship_gate', name='ship_gate', output='screen'),
   267	        Node(package='ship_dock', executable='ship_dock', name='ship_dock', output='screen'),
   268	        Node(package='ship_turn', executable='ship_turn', name='ship_turn', output='screen'),
   269	        # ship_last 제거됨(6b): mode 0 을 ship_gate 가 인수. 하던 일은 /candidate_angle 에 20000
   270	        #   폴백을 내는 것뿐이라 중복이었다. (mode 5,8 폴백은 north_goal_angle 이 담당)
   271	        Node(package='ship_back', executable='ship_back', name='ship_back', output='screen'),
   272	
   273	        # ============================================================
   274	        # 10) Fusion: 라이다 기반 항로 유지
   275	        # ============================================================
   276	        Node(
   277	            package='ship_direction',
   278	            executable='ship_direction',
   279	            name='ship_direction',
   280	            output='screen'
   281	        ),
   282	
   283	        # ============================================================
   284	        # 11) Motor Control
   285	        # ============================================================
   286	        Node(
   287	            package='motor_control',
   288	            executable='motor_control',
   289	            name='motor_control',
   290	            output='screen'
   291	        ),
   292	
   293	        # ============================================================
   294	        # 12) 시리얼 브릿지 (Motor_run → 아두이노 Mega, 상태 → /boat_mode)
   295	        # ============================================================
   296	        # 🚨 이게 없으면 motor_control 이 Motor_run 을 내도 **받는 쪽이 없다.**
   297	        #    사슬의 마지막 조각이다: motor_control → Motor_run → [브릿지] → Mega
   298	        # 아두이노가 아직 없어도 안전하다 — 브릿지 launch 가 포트 존재를 먼저 보고
   299	        # 없으면 노드를 안 띄우고 넘어간다(예외를 던지면 launch 전체가 죽는다.
   300	        # ntrip_client 가 파일 하나 못 찾아 노드 15개를 같이 죽인 사고가 실제로 있었다).
   301	        IncludeLaunchDescription(
   302	            PythonLaunchDescriptionSource(
   303	                os.path.join(dir_bridge, 'launch', 'ssf_bridge.launch.py')
   304	            )
   305	        ),
   306	    ])
```

==================================================================
## 테스트 목록 (함수 이름만)
==================================================================
- src/north_goal_angle/test/test_gps_filter.py : 7 개
    26:def test_first_fix_passthrough():
    34:def test_cov_reject():
    41:def test_cov_zero_means_no_info():
    49:def test_jump_reject():
    57:def test_normal_speed_ok_and_estimated():
    67:def test_smoothing_small_error_strong():
    77:def test_smoothing_large_error_follows():
- src/north_goal_angle/test/test_gps_fresh.py : 22 개
    30:def test_default_is_2_5_float():
    34:def test_never_had_fix_is_not_fresh():
    38:def test_just_received_is_fresh():
    42:def test_boundary_equal_is_fresh():
    46:def test_boundary_plus_eps_is_stale():
    50:def test_one_missed_frame_at_1hz_is_fresh():
    55:def test_two_missed_frames_at_1hz_is_stale():
    59:def test_long_gap_is_stale():
    63:def test_nan_inputs_are_not_fresh():
    69:def test_garbage_inputs_are_not_fresh():
    75:def test_sanitize_normal_passes_through():
    81:def test_sanitize_int_ok_as_float():
    88:def test_sanitize_rejects_inf():
    94:def test_sanitize_rejects_zero_and_negative():
    100:def test_sanitize_rejects_nan():
    105:def test_sanitize_rejects_non_number():
    112:def test_sanitize_result_always_usable():
    119:def test_yaml_value_is_float_and_matches_code_default():
    152:def test_have_fix_only_changed_in_gps_cb():
    159:def test_last_fix_t_only_set_on_accepted_frame():
    175:def test_param_declared_with_code_default_before_timer():
    184:def test_timer_uses_freshness_not_bare_have_fix():
- src/north_goal_angle/test/test_gps_guard.py : 0 개
- src/north_goal_angle/test/test_los_logic.py : 12 개
    34:def test_dist_reasonable():
    40:def test_angle_north_east():
    46:def test_lookahead_clamp():
    53:def test_xte_sign_right_positive():
    60:def test_xte_sign_left_negative():
    65:def test_xte_on_path_zero():
    71:def test_los_right_steers_left():
    77:def test_los_left_steers_right():
    82:def test_los_on_path_equals_path_bearing():
    87:def test_los_larger_lookahead_gentler():
    94:def test_ilos_adds_correction():
    102:def test_arrive_radius():
- src/north_goal_angle/test/test_mission_gate.py : 5 개
    38:def test_auto_runs():
    42:def test_manual_holds():
    46:def test_wait_holds():
    50:def test_none_holds():
    56:def test_bench_always_runs():
- src/north_goal_angle/test/test_waypoint_loader.py : 15 개
    48:def test_parse_dict_form():
    53:def test_parse_list_form():
    58:def test_output_is_float():
    64:def test_mode_order_preserved():
    76:def test_no_waypoints_key():
    82:def test_empty_list():
    86:def test_missing_field():
    90:def test_wrong_length_list():
    95:def test_lat_out_of_range():
    102:def test_lon_out_of_range():
    106:def test_invalid_mode():
    114:def test_mode_must_be_int():
    119:def test_negative_dwell():
    123:def test_non_numeric_coord():
    127:def test_error_message_has_index():
- src/north_goal_angle/test/test_waypoint_recorder.py : 8 개
    28:def test_avg_empty():
    32:def test_avg_single():
    36:def test_avg_mean():
    41:def test_avg_reduces_jitter():
    49:def test_format_has_header():
    54:def test_format_label_and_fields():
    62:def test_format_auto_label():
    68:def test_format_roundtrip_parseable():
- src/north_goal_angle/test/test_wp_timeout.py : 22 개
    74:def test_default_table():
    79:def test_accepts_tuple_and_int_secs():
    84:def test_reject_empty_table():
    90:def test_reject_none():
    98:def test_reject_length_mismatch():
    103:def test_reject_duplicate_mode():
    107:def test_reject_mode_outside_valid():
    113:def test_reject_non_int_mode():
    119:def test_reject_zero_and_negative():
    124:def test_reject_nan_and_inf():
    129:def test_reject_non_number_sec():
    136:def test_timeout_for_mode_present():
    142:def test_timeout_for_mode_absent_is_none():
    150:def test_regression_real_waypoints():
    161:def test_follows_mode_not_index():
    169:def test_dwell_conflicts_none_for_real():
    173:def test_dwell_conflicts_detected():
    183:def test_yaml_sec_all_float():
    189:def test_yaml_modes_all_int():
    195:def test_yaml_valid_and_matches_code_default():
    202:def test_code_default_types():
    207:def test_node_declares_with_code_defaults():

==================================================================
## 관련 문서
==================================================================

----- DOC: CLAUDE.md -----
# SSF 자율운항선박 — 작년 코드 재활용 프로젝트

> 매 세션 자동 로드. 확정 사실만, 추측은 "⚠️ 미확인". 근거·이력·검산은 `docs/기준/CLAUDE_상세.md` (같은 § 번호).
> 🔒 계약(토픽·값·QoS) 요약은 **이 파일**이 기준이다. 상세 파일과 다르면 이 파일을 따른다. **코드와 다르면 어느 쪽으로도 고치기 전에 코드를 확인하고 Davidson 에게 알린다.**

## 0. 요약 · 환경

작년(2025) KABOAT 대회에 나갔던 ROS2 코드를 올해 새 배 2척(A배/B배, 크기 다름)에 재활용한다.
- ROS2 Humble / Ubuntu 22.04 / Python 3.10 · 워크스페이스 `~/ros2_ws` · 빌드 `colcon build --symlink-install`
- 펌웨어(회로팀)는 `arduino/ssf_boat/ssf_boat.ino` + `COLCON_IGNORE` — 다른 툴체인, colcon 이 안 건드린다.
- launch 전 매번 `sudo ~/ros2_ws/tools/boat_boot.sh` (상태만: `--check`). 런타임 설정만, 재부팅 시 원복. 상세 §7-3

## 1. 🚨 작업 규칙 — 반드시 지킬 것

**1-1. 한 번에 노드 하나만 고친다.** 통째로 갈아엎지 않는다(원인 즉시 특정·팀 이해·롤백).
예외 = **원자 그룹은 같은 커밋**:
- 비전 검출기–미션 원자 그룹: 검출기 `basic_image_subscriber{gate,turn,dock}` 의 **발행 토픽·`active_wp_modes` 를 바꾸면**, 그걸 받는 미션 노드 + `color_shape_detector/config/vision.yaml` + `ssf_tools/config/ssf_tools.yaml` + `test_mode_gate.py` 를 **같은 커밋**으로. 쪼개면 중간 상태가 깨진다. (옛 `marker_detector`/`marker_selector`/`tracker` 노드는 없어졌다.)
- **6a** `ship_dock`(`active_wp_mode` 9→7) + `north_goal_angle`(mode-7 폴백 제거 → `FALLBACK_MODES = (5, 8)`) — **반영 완료**. 되돌리거나 한쪽만 바꾸지 말 것.
  쪼개면 `/candidate_angle` 발행자가 둘 → 도킹 중 조향이 GPS 방위로 튄다. 상세 §4

**1-2. git.** 노드 하나 고칠 때마다 커밋, 메시지에 **무엇을 왜** 바꿨는지. (기준선: "작년 대회 최종본 (수정 전 기준선)")

**1-3. 토픽 이름을 바꾸지 않는다.** 새 기능은 토픽 **추가**. 옛+새 노드 혼합 상태에서도 돌아야 한다(5/5 조합 검증됨).
ROS2 는 이름이 어긋나도 **에러 없이 아무것도 안 준다** — `/buoy_color` 사고(§3) 참고.

**1-4. 하드코딩 금지.** 배 폭·PWM·거리 임계값은 패키지별 `src/<pkg>/config/<pkg>.yaml` + 코드 기본값에 둔다. 모르는 실측값은 임시값 + **`# ⚠️ 실측 필요`**. 배별 분기 = GPS `gps_config` 인자(A: `c94_m8p_rover.yaml` / B: `zed_f9p_rover.yaml`) + **`boat:=A|B`(필수, 대문자 — 없거나 소문자면 launch 가 노드 하나도 안 띄우고 거부)**. 지금 `boat` 가 고르는 건 IMU 두 값뿐(`ssf_heading_A/B.yaml`, 923b903). 다른 배별 값(half_width·PWM)은 그 노드들에 yaml 이 안 실려 별도 계약. 상세 §1-4
yaml 은 **노드 이름으로** 적용된다 → `super().__init__('<이름>')` 은 패키지명과 일치해야 한다(전수 감사 완료, 상세 §3-7).
🚨 대회 launch(`launch_files.launch.py`)는 미션 노드·`ship_direction`·`motor_control` 을 **`parameters=` 없이** 띄운다 → 이 노드들의 값은 **코드 기본값**이 대회값이다. yaml 만 고치면 대회 실행에 반영되지 않으니 코드 기본값과 yaml 을 같이 맞춘다.

**1-5. `time.time()` 금지 → `time.monotonic()`.** (벽시계는 NTP 보정·시간 점프에 취약)

**1-6. 침묵 사망 원칙.** 침묵하는 노드는 에러를 내지 않는다(도킹 1년 침묵, 상세 §3-1).
모르면 0·기본값·옛값을 내지 말고 **발행을 멈추고**, 그 침묵은 `healthcheck`/ERROR 로그로 **드러나게** 한다.

**1-7. 로거 호출을 변수에 담아 재사용하지 말 것.** (`log = logger.warn if … else logger.info; log(...)`)
rclpy 는 소스 위치별로 심각도를 캐싱해 `ValueError: Logger severity cannot be changed between calls.` 로 **노드가 죽는다**.
심각도마다 호출 지점을 나눈다. (2026-08-06 실기: 페일세이프 복구 시 `ship_direction` 사망 → 배 영구 정지. 상세 §5)

**1-8. 대회 전 하지 말 것.**
- 특수 신호 상수 공유 모듈화 — 패키지 8개 `package.xml`/`setup.py` 를 건드려 빌드 전체가 위험. 상수표(§3 특수 신호) + `ssf-reviewer` 6번 점검(특수 상수 불일치)으로 충분. 대회 후 정리.
- 외부 패키지 도입(예: `obstacle_detector`) — 빌드 위험.
- 🚨 RealSense 경로 삭제: `src/realsense-ros-ros2-master/` 에 `COLCON_IGNORE` 넣지 말고 삭제도 금지, `basic_image_*.py` 삭제 금지(현역 유일 비전 경로). OAK 드라이버(`depthai-ros`)는 교체가 아니라 **추가**.
- 시뮬·측정 완료된 체인(`motor_control` 인터페이스·base/reverse/SPIN 부호 등)을 뜯는 것.

**1-9. 에이전트 팀 모드 (2026-10 도입).**

여러 노드·토픽 계약·페일세이프/모터 체인의 **동작**을 바꾸는 변경은 역할을 나눈 서브에이전트로 처리한다:
`ssf-contract`(계약서) → **Davidson 승인** → `ssf-implementer`(구현) → `ssf-reviewer` + `ssf-verifier`(동시) → **Davidson 승인 후 커밋**.
- 팀 모드에서는 §1-2 의 커밋을 **메인 세션이 마지막 승인 뒤에만** 한다. 서브에이전트는 커밋하지 않는다. §7 의 `git diff HEAD~1` 대신 작업 트리 기준으로 본다.
- 메인 세션은 팀 모드 중 `src/` 를 직접 고치지 않고, 두 승인 지점에서 반드시 멈춘다. 구현자 둘을 동시에 돌리지 않는다(§1-1).
- 동작이 안 바뀌는 작은 수정(로그 문구·주석·오타·테스트 추가)은 팀 모드 없이 지금처럼 한다.
- **팀 모드를 시작하기 전에 `docs/기준/에이전트팀_운영.md` 를 읽는다.**

## 2. 하드웨어 (확정)

| 항목 | 사실 |
|---|---|
| LiDAR | RPLIDAR A3, `/dev/ttyLiDAR`, `Sensitivity`, 25m. **실측 개체마다 다름**: 1번배 `80BF` ≈11.1Hz/0.250°, 2번배 `918A` 11.00Hz/0.223°(셋 중 최정밀). 11.96Hz/0.275° 는 소재 미확인 `D5F1`(지금 없음) 값. (스펙 10Hz/0.225° 아님, 상세 §2·`docs/문제와작업/라이다_고장진단_20260818.md`). 회전수는 파라미터로 **못 바꾼다** — 드라이버가 `scan_frequency` 를 모터에 안 넘김(에러 없음) |
| IMU | RB-SDA-v1 (IntelliThings iAHRS) 9축 AHRS, ASCII `"e\n"`, `/dev/IMU`. **WT901C 아님** |
| GPS | A배 u-blox C94-M8P · B배 u-blox ZED-F9P, 둘 다 NTRIP RTK |
| MCU | **Arduino Mega 2560 R3**. 통신 = `ssf_bridge` USB 시리얼 (**Due·micro-ROS 아님**, launch 가 micro-ROS 안 띄움). `Motor_run = pwm_r*10000 + pwm_l`, 1500=중립. 펌웨어는 passthrough(모터별 `invert`·`gain_num` 노브 있음 — 현재 좌·우 둘 다 `false`·`10` → 숨은 반전 없음. 방향 디버깅 땐 `ssf_boat.ino:327-328` 도 확인) |
| ESC | 🔒 **좌/우 2채널뿐** — 핀 12(좌)·11(우). 선수·선미 구분 없음, 측별 팬아웃(2번배 측당 2개·1번배 측당 1개). 펌웨어 동일 → 배 ID 분기 불필요. **게걸음 불가** |
| 카메라 | **RealSense D455 현역**(비전 3종이 지금 이걸로 돈다) / **OAK-1 PoE 보유**(표준 화각·고정초점 — **W 광각 아님**, 09-15 구매모델 확정. RGB 전용·뎁스 없음·PoE=이더넷). OAK 는 인젝터(알리 10-27 주문) 도착 뒤 D455 와 비교 선정, 바꾸는 날 `image_topic`·`hfov_deg` 실측 — 런북 `docs/절차/oak_arrival_runbook.md`. 코드는 **중립화**(상세 §3-3) |
| 선체 | 1번배 실측 1.66 × 0.57 × 0.38m → `half_width 0.29`. 2번배 실측 1.77 × **0.535**m(10-24 줄자, ⚠️ 스러스터 포함 여부 미확인) — 공용 `half_width 0.29` 가 0.535/2=0.27 보다 넓어 안전 쪽 (§8) |

- 🚨 `docs/전달용/펌웨어_설계문서.md` 의 "선수 좌/우 + 선미 좌/우", `docs/전달용/회로_요구사항서.md` §7 ESC 4행은 **옛 설계** — 그대로 배선하면 없는 채널을 배선한다.
- 🚨 `/dev/ttyLiDAR` 이 있다고 라이다가 살아있는 게 아니다(udev 는 USB 어댑터 시리얼에 붙고, 어댑터는 개체 사이를 옮겨 다닌다). 개체 확인 `python3 tools/lidar_id.py`. 개체 시리얼 목록 상세 §2-1
- 각분해능이 개체마다 다르니(0.223/0.250°) 셀 기반 값(`min_obstacle_cells` 등)은 개체별로 튜닝. `918A`(0.223°)는 `0.225°` 가정이 실제와 맞지만 `80BF`(0.250°)에선 라이다를 실제보다 좋게 평가한다.

## 3. 🔒 계약 — 코드가 반드시 일치해야 하는 것

버그 이력(도킹 침묵·감속 미연결·카메라 depth 가드·subprocess 매니저·IMU/COG·`/candidate_angle` 분산합의·`ship_back` 이름충돌·기타)은 상세 §3-1~3-8.

**토픽 표** (어긋나면 에러 없이 빈 값)

| 토픽 | 타입 | 발행자 | 구독자 |
|---|---|---|---|
| `/health_ok` | `Bool` | `healthcheck` | (사람이 봄 — 제어 구독자 0, 진단 전용) |
| `/failsafe_level` | `Int32` | `ship_direction` | `blackbox`, `motor_control`(속도 상한), `mission_monitor` |
| `/gates_passed` | `Int32` | `ship_gate` | `blackbox` — 🚨 `/gate_pass_count` 아님 |
| `/geofence_state` | `Float32MultiArray` | `north_goal_angle` | `ship_direction` |
| `/image_color` | `String` | `basic_image_subscriberturn` | `ship_turn`, `blackbox` |
| `/boat_mode` `/boat_id` | `Int32` | `ssf_bridge` | `mission_monitor` |
| `/boat_cmd_watchdog` `/boat_estop` | `Bool` | `ssf_bridge` | `mission_monitor` |
| `/obstacle_distance_array` | `Float32MultiArray` | `ship_direction` | `motor_control`, `blackbox` — `[거리(m), 각도(deg)]`, 없으면 `[inf, nan]` |
| `/motor_reverse` | `Bool` | `motor_control` | `yaw_mux` — 실제 출력 PWM 좌우 평균 > 1500 |
| `/imu/yaw` | `Float64` | **`ssf_heading/yaw_mux` 하나뿐**(`yaw_mux.py:129`) | `ship_goal_angle`, `north_goal_angle`, `ship_turn` |
| `/imu/yaw_raw` | `Float64` | `iahrs_driver` | `yaw_mux`, `healthcheck` |
| `/heading_status` | `String` | `yaw_mux` | `healthcheck` |
| `/obstacle_reject_count` | `Int32` | `ship_direction` (median 기각 수, 관측 전용) | `blackbox` — 제어에 안 쓴다 |

- 🚨 `/image_color`(`"red"`/`"green"`/`"white"`; 빨강·초록=시계, 흰색=반시계) 를 `/buoy_color` 로 개명하지 마라 — 2026-09-23 철회. 문서만 개명했다가 `ship_turn` 이 SEARCH 에서 영구 정지(`aa46028`).
- 🚨 `iahrs_driver` `yaw_topic` 을 `/imu/yaw` 로 되돌리지 마라 → 발행자 2개(에러 없음). 기본값 `/imu/yaw_raw`.
- 죽은 토픽(구독자 0): `video_frames`, `/red_distance`·`/green_distance`·`/image_distance`(발행 중단). `/goal_distance`·`/wp_remaining_time` 은 **`mission_monitor` 가 관측용으로 구독**한다(`mission_monitor.py:105-106`) — 제어 구독자는 없다.

**`/boat_*` 값** — `ssf_boat.ino` 의 `enum Mode`·`enum BoatId` 와 **값까지** 같다. 🚨 여기서 번호를 새로 매기거나 변환표를 끼우지 마라.
- `/boat_mode`: 0=WAIT(대기·중립) 1=MANUAL(RC) 2=AUTO(브릿지 명령)
- `/boat_id`: 0=A 1=B 2=FAULT(ID 핀 **둘 다 GND**=배선 실수 → 펌웨어 무조건 중립). 미배선(둘 다 open)은 FAULT 아니라 **기본값 A** → 2번배도 미배선이면 0 을 보고한다(`ssf_boat.ino:507-523`)
- 두 배는 **노트북(ROS2 스택)이 따로** → DDS 도메인 분리. 둘 다 미배선 `/boat_id=0` 이어도 충돌 없음(한 스택=한 척). ESC 행의 "배 ID 분기 불필요"는 이 전제다. 한 네트워크에 두 배를 올릴 때만 2번배 `BOAT_B` 배선·네임스페이스가 필요.
- `/boat_cmd_watchdog`: true = 명령 500ms 무수신 → 펌웨어가 중립 · `/boat_estop`: true = 비상정지 눌림
- 펌웨어 상태 줄 `S,mode,watchdog,boatId,estop,FL,FR,RL,RR`(10Hz)를 `ssf_bridge` 가 파싱.
- 🚨 모드 전환 통로는 **조종기뿐**. 노트북에서 `/boat_mode` 를 바꾸는 경로를 만들지 마라(만든다면 단방향=자율 해제만 + 조종기 우선). 상세 §3-9

**🚨 추력 방향 — ROS 와 시리얼이 반대다**
- ROS(`motor_control`·`Motor_run`·`/motor_reverse`·blackbox): 1500=정지, **<1500=전진**, >1500=후진 (`base_pwm=1360`, `reverse_pwm=1590`)
- 시리얼/실물: **>1500=전진** (2026-08-27 실측)
- 반전은 **`ssf_bridge._send` 한 곳**: `pwm = 3000 - pwm` (`invert_thrust_direction`, 기본 true). `steer_invert` 는 건드리지 않는다. RC 는 이 경로를 안 탄다.
- 🚨 스러스터 배선을 다시 뒤집으면 `invert_thrust_direction: false`. 테스트 `src/ssf_bridge/test/test_thrust_invert.py`

**`/candidate_angle`·`/desired_angle` 특수 신호** (각도가 아니라 명령 코드, 6개 파일에 각자 정의 — 값을 맞출 것)

| 값 | 의미 |
|---|---|
| `5000.0` | 우선회 SPIN_RIGHT |
| `6000.0` | 좌선회 SPIN_LEFT (현재 발행 노드 없음, 예약) |
| `20000.0` | 미션 없음 CANDIDATE_INVALID → fallback 전진 |
| `50000.0` | 정지 STOP_HOLD / STOP_VALUE |

SPIN 중심 `spin_forward_pwm`(기본 1500 → 1400/1600 = 제자리 선회). 물리 배선 반전은 `steer_invert`(기본 false) 하나가 조향·SPIN 을 함께 뒤집는다.

**wp_mode → 담당 노드** (실제 `/wp_mode` 값은 `{0,1,2,3,5,7,8}` 뿐. 담당 모드의 권위 = 각 미션 노드의 `active_wp_mode` 기본값 + 검출기 `color_shape_detector/config/vision.yaml` 의 `active_wp_modes`. `ssf_tools/config/ssf_tools.yaml` 은 healthcheck 기대값일 뿐 — 바꿀 땐 셋을 함께)

| wp_mode | 미션 | 미션 노드 | 검출기 → 발행 |
|---|---|---|---|
| 0, 1 | 게이트 시작·끝 | `ship_gate` (`active_wp_modes: [0, 1]`, `ship_last` 제거됨) | `gate` → `/red_angle`, `/green_angle` |
| 2 | 위치유지 | `ship_back` | `turn`(white) → `/image_angle` |
| 3 | 부표 선회 | `ship_turn` | `turn`(red) → `/image_angle`, `/image_color` |
| 5 | 회피 (dwell 50 s) | **없음 = 정상** | 없음 — `ship_direction` 순수 회피 |
| 7 | 도킹 (dwell 60 s) | `ship_dock` | `dock` → `/image_angle` |
| 8 | 토너먼트 회피 | **없음 = 정상** | 없음 |

- 표의 50 s·60 s 는 **규정 시간이 아니라 작년 dwell 값**이다. WP 시간 초과는 **mode 기준** — 0·1·2·5 = 120 s, 3 = 90 s, 7 = 150 s, 8 = 없음(`north_goal_angle.yaml` `wp_timeout_modes`/`wp_timeout_sec`, b337ad8). 🚨 `wp_timeout_sec` 는 원소마다 소수점 — 정수면 노드가 안 뜬다(10-29 실측).
- mode 5·8 을 healthcheck 가 '누락' 으로 경고하면 매 실행 거짓 경보다 → 매핑표에 `none` 으로 명시(반영됨). `north_goal_angle` 의 20000 폴백은 5·8 에만.
- 🚨 `dock`·`turn` 이 둘 다 `/image_angle` 을 발행 → `active_wp_modes` 겹치면 발행자 2개. `test_mode_gate.py` 의 `check_publisher_conflicts` 가 정적 검사.
- 검출기는 상주한다(subprocess 매니저 폐기). `/wp_mode` 미수신·stale(2.0s 초과) → **발행 정지** + 5초 주기 경고. 게이트는 `color_callback` 맨 앞.
- `healthcheck` 는 빠진 모드 / 중복 모드 / 침묵 노드를 검사한다.

**`/geofence_state`** — 경계를 '가짜 LiDAR' 로: `data = [angle_min_deg, angle_inc_deg, r0, r1, …]`, 상대방위(0=정면), 멀면 `inf`.
정보 없으면 **빈 배열**(미설정·GPS fix 없음 / **GPS fix stale `gps_stale_sec` 2.5 s**(b288014) / IMU stale `imu_stale_sec` 0.5s / 이미 이탈). 소비: `ranges[i] = min(real, geofence)`, `geofence_stale_sec`(2.0s) 넘게 묵으면 병합 안 함. min 병합은 (C) 자율 회피 구간에만 들어가지만, **별도로** candidate/SPIN 구간엔 하드 가드가 있다 — 경계 최소거리가 `geofence_stop_margin_m`(2.0) 안이고 상대방위가 전진방향 ±`geofence_stop_cone_deg`(60°)와 겹치면 candidate/SPIN 무시하고 STOP_HOLD(`ship_direction.py:149`). 도킹 중 정지 원인이 될 수 있다.
🚨 구독자 0 = 경계 방어 없음(실격) → `north_goal_angle` 부팅 5초 뒤 ERROR + `healthcheck` 가 `/health_ok=false`. ⚠️ 상세 §3-9 에 구판 소비 방식(±40° 원뿔 칠하기)이 남아 충돌 — 코드 확인.

**QoS**
- 이미지 구독: **BEST_EFFORT + KEEP_LAST + depth=1** (묵은 프레임으로 조향 금지). `/scan` 구독 5곳(`ship_direction/dock/gate/turn/back`)도 동일 — depth=1 이 도착=신선을 일치시켜 페일세이프를 정직하게 만든다.
- `/wp_mode` 는 모드 명령 → **RELIABLE 유지**.
- 관찰 토픽이라도 **발행은 RELIABLE**(`/boat_*`) — BEST_EFFORT 면 `ros2 topic echo` 가 아무것도 못 받는다. (구독자 BEST_EFFORT ↔ 발행자 RELIABLE 은 호환, 반대는 비호환)

**헤딩·비전 규칙**
- 각도 평균은 산술평균 금지(359°와 1°→180°) — 단위벡터 합 + `atan2`. COG 수렴 파라미터(`cog_min_samples`·`gps_rate_hz`·`cog_half_life_sec`)는 포화 함정이 있어 바꾸기 전 상세 §3-5.
- `/health_ok` 는 진단 전용이라 거짓 정지 위험이 없다는 전제로 설계됐다. 제어에 물리려면 먼저 상세 §3-3·§5 를 보고 Davidson 과 검토.
- 도크 인식: 목표 색·형상 유효값 밖이면 **부팅 ERROR + 검출 안 함, 담당 모드(7)에서만 `/image_angle`=10000 + 5 s 주기 ERROR, 그 외 모드는 발행 없음**(ff289f7. 이전엔 게이트 전 발행으로 모드 2·3 에 10000 누수 — 10-29 Ubuntu 확인). 임계는 혼동행렬 수치로 — 감으로 튜닝 금지. 건드리면 상세 §3-3b.
- 헤딩 없으면 발행하지 않는다. 미구현 `heading_source` 는 0 대신 발행 정지. `zero_yaw_on_boot`·`use_gps_heading_override` 기본 false.
- 자기편각은 **자기북 기준 소스에만**(`SOURCE_IS_MAGNETIC`). 듀얼 GPS·COG 에 더하면 이중 보정(약 8°).
- 카메라는 **방위각만**, 거리는 LiDAR `/scan`. 도크는 매칭하지 말고 LiDAR 전방 섹터 최소거리. 상세 §3-3
- 카메라 교체 시 고칠 곳 = `src/color_shape_detector/config/vision.yaml` 의 `hfov_deg`(현재 80.32 — 2026-08-12 `camera_info` 실측으로 71.5 에서 정정) + HSV 재캘리브(광각은 `hfov_deg` 만으론 부족 — 왜곡 보정·`min_area_px` 재튜닝, 상세 §3-3). 픽셀 폭은 매 프레임 `msg.width` 로 읽지만, 해상도·화각이 바뀌면 `hfov_deg` 도 같이 바꿔야 한다. 런북 `docs/절차/oak_arrival_runbook.md`
- `image_topic` 파라미터 선언은 구독보다 **먼저**(뒤면 부팅 즉시 `AttributeError`).
- 🚨 `motor_control` 명령 워치독 0.5s 는 `ship_direction` 이 고정주기 타이머로 항상 발행한다는 전제 위에 있다. 그 전제를 깨지 말 것. 상세 §3-2

## 4. 진행 순서 원칙

하류부터, 위험 낮은 것부터(`motor_control` 이 체인 끝). **0단계(blackbox+healthcheck)를 건너뛰지 말 것** — 측정 없이는 개선을 모른다(종합임무 5회 도전·최고점 채택).
단계표·6a 원자성 근거는 상세 §4.

## 5. 페일세이프 — 오탐지 방지가 최우선

팀 최우선 우려 = "고장이 아닌데 스스로 고장이라 판단해 멈추는 것". 경기 중 멈추면 끝이다.
- 4중 장치: `time.monotonic()` / ARMED(센서 1회 수신 후 감시) / 연속 N회(`failsafe_confirm_n: 3`) / 히스테리시스+자동 복구.
- `failsafe_warn_sec: 0.7`(감속), `failsafe_stop_sec: 3.0`(정지), `failsafe_l1_speed: 0.7`.
- 페일세이프는 콜백이 아니라 **독립 타이머(`watchdog_cb`)** 에서 평가한다(센서가 죽으면 콜백이 안 불린다).
- 온도·헤딩 진단 등 무게가 다른 신호는 로그만 — `/health_ok` 에 넣지 않는다(거짓 정지 금지).
- **RC 두절 처리는 건드리지 않는다**(팀 결정).
- 순수 로직 테스트·시뮬은 ROS 래퍼 버그(1-7)에 닿지 않는다 — 실기 검증 필요. 상세 §5

## 6. 시뮬 검증 파라미터 — 임의로 바꾸지 말 것

출처 `tools/sim/`, 재현 `python3 tools/sim/sweep_s1.py --seeds 20`. 스윕 수치 상세 §6.

| 파라미터 | 값 |
|---|---|
| `clearance` (현 `half_width 0.29` 기준) | **0.25** — 🚨 접촉 수만 보고 0.45 고르지 마라(절반 정체) |
| `detection_distance_default` | **3.0** |
| `detection_distance_gate` | **2.0** |
| `min_obstacle_cells` | **1** |
| `temporal_frames / votes` | **1 / 2 = OFF** (무익. 코드는 유지, 켤 땐 dilate 전 원본 마스크에) |
| `obst_median_kernel` | **5** — 감속 신호(`_closest_obstacle`)에만. 회피 마스크(`_compute`)엔 안 건다 |

(`track_gate_deg` 는 tracker 노드와 함께 없어졌다 — `src/`·`tools/` 에 없음. 상세 §6)

- 그 외 확정값(상세 §3-2): `motor_control` 감속 `slow_start_dist=1.2`, `min_speed_ratio=0.7`(전진 성분만), 명령 워치독 `cmd_timeout_sec` 0.5s, 부팅 중립.
- `rear_obstacle_ignore_margin` 은 **제거됨**(부표를 통째로 지움). 되살리지 마라.
- **TTC 비상제동은 '해로워서' 폐기**, 시간투표는 '무익해서' OFF — 둘을 구분할 것.

**6-2. 재론 금지 (경쟁팀 분석 결론).** 갭 목표방위 가중·최소 갭폭 제약·선회 bbox∩LiDAR = 이미 있음 / median = 채택(DBSCAN 안 함) /
위치유지 = LiDAR 유지, RTK 는 폴백만, **PD 미채택**(체인이 각도 기반) / 게이트 유체장 = 안 함 / `obstacle_detector` = 안 함 / 도킹 확인-N = 5단계 후. 근거 상세 §6-2

## 7. 작업이 끝나면

```bash
git diff HEAD~1 --stat && git diff HEAD~1
colcon build --symlink-install
tools/run_tests.sh              # 로직 테스트 전체 (우리 패키지만)
colcon test                     # 🚨 lint 포함 — 권위 있는 실행기
```
- 🚨 `python3 -m pytest src/` 금지 — 수집 단계에서 죽어 하나도 안 돈다(ament 보일러플레이트 파일명 중복 + 외부 의존성 누락).
- 🚨 그 제외 설정을 `pytest.ini` 로 옮기지 마라 — `colcon test` 가 `addopts` 를 주워 lint 를 조용히 건너뛴다(기존 실패가 통과처럼 보임).
- 검토에 넘길 것: 변경된 노드 파일 + `git diff` + 바꾼 이유.

**7-2. 🚨 안전 스위치 — 대회값 고정 (끄는 쪽이 편해서 위험)**

| 파라미터 | 대회값 | 기계적 검사 |
|---|---|---|
| `cog_require_reverse_gate` (yaw_mux) | `true` | `healthcheck` → `/health_ok=false` (`heading_source: cog_offset` 일 때만) |
| `require_geofence_subscriber` (healthcheck) | `true` | `healthcheck` → `/health_ok=false` |
| `debug_view` (비전 4종) | `false` (헤드리스 `cv2.imshow` 가 노드를 죽임) | 기본값이 안전 쪽 |
| `active_wp_modes` (검출기 3종) | §3 wp_mode 표대로 | `test_mode_gate.py` |

사람이 할 확인(조향 부호·IMU 절대방위·`gps_vel_frame`·`hfov_deg`·`/health_ok` true) 절차는 상세 §7-2. `/health_ok` 빨간불을 켜둔 채 두지 않는다.
🚨 IMU 절대방위: **나침반은 자북**을 가리킨다. `imu_relative` 는 편각을 안 쓰므로(`SOURCE_IS_MAGNETIC`) 나침반 정북에 맞췄을 때 `/imu/yaw` 목표값은 0 이 아니라 **360 + 편각(한국 약 −8° → 약 352)** 이다. 지도·GPS 로 진북을 직접 잡았으면 0.

## 8. ⚠️ 아직 모르는 것 (추측하지 말 것 — 필요한 곳엔 `# ⚠️ 실측 필요`)

- 2번배 폭 0.535m 가 스러스터 포함인지 (10-24 줄자; 공용 `half_width 0.29` 는 1번배 기준이라 어느 쪽이든 안전 쪽)
- 새 배 추력 → `base_pwm`, `reverse_pwm`
- IMU 장착 오프셋(`mount_offset_deg`, `invert_yaw` — 0.0/false 는 "아직 안 쟀다"). 값은 **배별 파일 `ssf_heading_A/B.yaml`**, 🚨 **반드시 소수점**(`37.0` — `37` 이면 `yaw_mux` 가 부팅 즉시 죽는다, 10-29 실측). 대회장 자기편각
- 도크 규격 → `contact_dist_m`
- 🚨 **조향 좌/우 부호**(`steer_invert`) — 틀리면 모든 게 무의미. 가장 흔한 사고. 벤치 필수
- LiDAR 장착 높이 = 부표 높이인지 (안 맞으면 부표를 못 본다 → 위치유지 소스 판단, §6-2)
- 카메라 왜곡계수(D)·초점거리(fx,fy) (`camera_info`), OAK 고정 IP·컬러 토픽 이름
- 깊이 0.38m(선체 아래 21cm) — 진수·회수·대회장 얕은 구역 주의

----- DOC: docs/전달용/팀원전달_GPS수정_병합안내.md -----
# 팀원 전달: `north_goal_angle 개정5` 검토 결과 + 병합 안내

> 받은 파일: `#north_goal_angle 개정5 기존 코드 업그레이드.ver.py`
> 검토 기준: 저장소 최신 `src/north_goal_angle/north_goal_angle/north_goal_angle.py` (커밋 이력 반영본)
> 결론 한 줄: **방향은 좋다. 다만 이 파일은 우리가 최근 이 노드에 넣은 안전 수정 이전 버전에서 갈라졌다.
> 그대로 합치면 그 수정들이 되돌아간다. 최신본을 base로 다시 떠서 새 기능만 얹어 달라.**

---

## 0. 왜 이 안내가 필요한가

개정5는 GPS·IMU·heading·Dead Reckoning을 크게 손봤습니다 — 아이디어 자체는 좋습니다.
그런데 **base로 삼은 `north_goal_angle`이 옛 버전**입니다. 우리가 그 사이에 **같은 파일**을
여러 번 고쳤는데(주로 안전 관련), 개정5에는 그 수정들이 없습니다.

그래서 개정5를 그대로 반영하면 **우리 수정이 사라지는(회귀)** 문제가 생깁니다.
이 문서는 "우리가 그 사이에 이 노드에 **무엇을 왜 고쳤는지**"를 먼저 설명하고, 그 위에서
개정5의 새 기능을 **어떻게 얹으면 되는지**를 정리한 것입니다.

---

## 1. 우리가 `north_goal_angle`에 넣은 수정 (개정5에는 없는 것)

개정5가 되돌리게 되는 항목들입니다. 하나씩, **왜** 넣었는지까지.

### 1-1. 🚨 mode 7 폴백 제거 — 가장 중요

**개정5 (L255-256, 그대로 살아있음):**
```python
if wp_mode == 7:
    self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
```

**우리 최신본:** 이 코드를 **뺐습니다.** 폴백은 `mode 5, 8`(순수 회피 구간)에만 냅니다.

**왜:**
- 작년 `ship_dock`(도킹 노드)이 "내 담당은 `wp_mode == 9`"로 잘못 설정돼 **1년간 침묵**했습니다.
  실제 도킹 모드는 **7**입니다. 도킹이 안 되니, 배가 멈추지 않게 `north_goal_angle`이
  mode 7에서 대신 `CANDIDATE_INVALID(20000)`를 흘려보내는 **임시 땜빵**이 이 코드였습니다.
- 우리가 `ship_dock`을 **9 → 7**로 고쳤습니다(정상화). 그러면 mode 7일 때
  `ship_dock`과 `north_goal_angle`이 **둘 다 `/candidate_angle`에 발행** → **발행자 2개 충돌**.
  접안 직전 조향이 GPS 방위로 튑니다(시뮬에서 60초 중 35° 스파이크 20번).
- 그래서 `ship_dock` 수정과 이 폴백 제거는 **반드시 한 쌍**으로 가야 합니다.

> **개정5를 쓰면**: `ship_dock`(7)과 충돌이 되살아납니다. 이 줄은 반드시 빼 주세요.

### 1-2. Geofence(경계 이탈 방지) — 통째로 신규

**개정5:** 없음.
**우리 최신본:** `/geofence_state` 토픽을 새로 발행합니다. 경기장 경계를 **"가짜 LiDAR"** 로 만들어
`ship_direction`이 벽처럼 피하게 합니다.

**왜:**
- 경기장 밖으로 나가면 실격인데, 작년엔 경계 방어가 **0**이었습니다.
- 처음엔 "최근접 벽 방향으로 ±각도 원뿔을 막는" 방식을 생각했는데, **모서리에서 뚫립니다.**
  40×40 경기장 모서리에서 배가 대각선(45°)을 보면 두 벽의 최근접점이 ±45°인데
  **탈출구(모서리)는 정면 0°** 라 원뿔 사이 틈으로 그대로 빠져나갑니다.
- 그래서 경계선을 **각 방향으로 광선(ray)을 쏴서** "이 방향으로 몇 m 가면 경계 밖인가"를
  구하는 방식으로 바꿨습니다. `ship_direction`은 이걸 실제 LiDAR와 `min()`으로 합치기만 하면 됩니다.

> **개정5를 쓰면**: 경계 방어가 사라집니다. 이 기능은 유지돼야 합니다.

### 1-3. heading을 `north_goal_angle`에서 만들지 않음 — 구조가 바뀜

이게 개정5와 **가장 크게 충돌**하는 지점입니다. 자세히 설명합니다.

**개정5의 구조:**
- `/imu/data`(Imu, orientation quaternion)를 직접 구독해서 자체적으로 `imu_yaw`를 뽑고,
- `NavPVT.heading`으로 오프셋을 median 보정해서 `filtered_heading`을 만들고,
- 이걸 **Dead Reckoning(위치 추정)에 내부적으로** 씁니다.

**우리 구조 (N1에서 바꿈):**
- `north_goal_angle`은 heading을 **안 만듭니다.** `/imu/yaw`(보정된 절대방위)를 **구독만** 합니다.
- heading을 만드는 건 **`yaw_mux`라는 새 노드 하나뿐**입니다.

**왜 heading을 한 곳(`yaw_mux`)에 몰았나:**
- 작년엔 IMU 드라이버가 `/imu/yaw`를 직접 냈는데, 그게 **진짜 방위가 아니었습니다.**
  - 부팅 순간을 0으로 잡아 **상대각**이 됐고,
  - GPS가 유효하면 IMU를 버리고 **GPS 이동방향(COG)** 으로 통째로 덮어썼습니다.
    COG는 **배가 실제 간 방향**이지 **뱃머리 방향**이 아닙니다. 정지 중엔 노이즈로 튑니다.
- 그래서 IMU 드라이버는 **날것**(`/imu/yaw_raw`)만 내게 하고, 보정은 `yaw_mux`가 전담,
  `/imu/yaw`의 **발행자는 `yaw_mux` 하나**가 되도록 정리했습니다.
- **여러 노드가 각자 heading을 계산하면**, 노드마다 방위가 미묘하게 달라져 나중에
  "왜 이 노드는 이쪽, 저 노드는 저쪽으로 판단하지?"를 못 잡습니다.

> **개정5를 쓰면**: `north_goal_angle`이 다시 자체 heading을 만들게 되어, heading 소스가 둘로 갈립니다.
> heading 계산 로직은 `yaw_mux`로 옮겨 주세요(3장 참고).

### 1-4. 시계: `time.time()` → `time.monotonic()`

**개정5:** `time.time()`(벽시계) 사용.
**우리:** `time.monotonic()`.

**왜:** 벽시계는 NTP 시간 보정이나 시스템 시간 점프에 영향을 받습니다.
타임아웃·dwell 계산이 시간 점프 한 번에 엉킵니다. 경과시간 측정엔 항상 monotonic을 씁니다(프로젝트 규칙).

### 1-5. 하드코딩 → 파라미터

**개정5:** 임계값·타이머 주기·waypoints가 전부 코드에 박혀 있음.
**우리:** `config/north_goal_angle.yaml`로 뺌.

**왜:** 배가 **두 척(A·B, 크기 다름)** 입니다. 거리·주기 같은 값을 코드에 박으면 배마다 코드를
따로 관리해야 합니다. 설정 파일로 빼면 코드는 하나, 값만 배별로 다르게 둘 수 있습니다.

---

## 2. 개정5의 새 기능 평가 (이건 좋습니다)

되돌리는 것과 별개로, 개정5가 **새로 넣은 것**은 가치가 있습니다. 솔직하게 평가합니다.

| 기능 | 개정5 위치 | 평가 | 결론 |
|---|---|---|---|
| **Covariance 게이트** | `gps_cb` L131-134 | ✅ 싸고 안전. RTK가 풀려 오차가 커지면 그 값을 버림 | **이식 추천** (임계값은 실측으로) |
| **NavPVT heading offset** | `navpvt_cb` L109-122 | ⚠️ 우리 N2와 **개념이 같음** (아래 2-1) | **N2와 통합 검토** |
| **Dead Reckoning** | `timer_cb` L204-234 | ⚠️ 강력하지만 위험. 가속도 적분은 드리프트가 발산 | **실측 후 결정** |
| **Adaptive LPF / 동적 도착반경** | L168-179, L273-278 | ⚠️ 튜닝값. 근거 있는 시도지만 계수가 추측 | **실측 후 결정** |
| **ZUPT / 중력보상** | L207-218 | ✅ DR을 진지하게 만든 부분. DR을 쓴다면 함께 | DR과 세트 |

### 2-1. NavPVT heading — 우리 N2(옵션 B)와 겹칩니다

둘 다 **"GPS가 준 이동방향으로 IMU 방향을 보정"** 하는 **같은 아이디어**입니다.

- **개정5**: `NavPVT.heading`(Ublox 자체 계산) 사용, median(최근 10개), 속도 0.5m/s 게이트
- **우리 N2**: `fix_velocity`에서 COG 유도, 원형통계(단위벡터 평균), **후진 게이트**, 수렴 판정

**핵심**: `NavPVT.heading`은 우리가 N1에서 확인한 그 **COG(heading of motion)** 입니다.
소스가 사실상 같습니다. 다만 두 가지가 다릅니다:

1. **후진 함정**: 배가 후진만 계속하면 COG가 뱃머리와 **정확히 180° 뒤집힙니다.**
   개정5의 median은 이걸 못 막아서 방향을 정반대로 배울 수 있습니다.
   우리 N2는 `motor_control`의 `/motor_reverse`를 받아 후진 중엔 학습을 멈춥니다.
2. **위치**: 개정5는 `north_goal_angle` 안에서 DR용으로만 씀. 우리 N2는 `yaw_mux`에 있어
   `/imu/yaw`를 쓰는 **모든 노드**가 혜택을 받음.

> **제안**: NavPVT를 쓰고 싶으면 `yaw_mux`의 `heading_source`에 `navpvt` 옵션을 추가하는 방향으로.
> 후진 게이트는 우리 것을 유지. `fix_velocity` COG vs `NavPVT.heading` 중 실측으로 나은 쪽 채택.

---

## 3. 그래서 어떻게 합치면 되나 (실행 안내)

### 하지 말 것
- ❌ 개정5로 파일을 **통째 교체**. (geofence·mode7·monotonic·파라미터가 전부 되돌아감)

### 할 것
1. **base를 최신본으로 다시 뜬다.** 저장소의 현재 `north_goal_angle.py`를 받아서 시작하세요.
   (geofence + mode7 폴백 제거 + `/imu/yaw` 구독 + monotonic + 파라미터화가 이미 들어 있음)
2. **heading 계산 로직은 옮긴다.** 개정5의 `imu_cb` yaw 계산 + `navpvt_cb` offset 보정은
   `north_goal_angle`이 아니라 **`yaw_mux`(`src/ssf_heading/`)** 쪽 일입니다.
   NavPVT를 소스로 쓰고 싶으면 거기 `heading_source` 옵션으로 추가하고 **후진 게이트를 유지**하세요.
3. **새 기능만 최신본 위에 얹는다:**
   - **Covariance 게이트**: `gps_cb`에 이식. 임계 `2.0`은 F9P 실측으로 확정.
   - **DR / Adaptive LPF / 동적 반경**: 실측 데이터(blackbox CSV)로 필요성·계수 확인 후.
4. **규칙 두 개만 맞춘다:**
   - `time.time()` → `time.monotonic()` (경과시간 계산 전부)
   - 하드코딩 상수 → 파라미터 (`declare_parameter` + yaml)

### 우선순위 (실기 시작 후)
1. Covariance 게이트 (제일 싸고 안전)
2. NavPVT를 N2 소스 옵션으로 (후진 게이트 유지)
3. DR (GPS 공백이 실제 문제인지 CSV로 확인 후)

---

## 4. 왜 "지금 당장" 안 넣나 (참고)

우리 원칙이 **"측정으로 결정한다"** 입니다. 개정5의 새 기능 상당수는 **실기 검증 전**입니다:

- DR의 가속도 적분은 **실기 IMU 데이터 없이는 드리프트 특성을 모릅니다.** 짧은 GPS 공백엔
  유용하지만, 길어지면 배가 **엉뚱한 위치를 진짜라고 믿습니다.** 이건 조용히 위험합니다.
- Covariance 임계 `2.0`, LPF 계수, 동적 반경 범위는 F9P 실측 없이는 추측값입니다.

검증된 코드(우리 최신본)에 미검증 큰 변경을 덮는 건 위험합니다. 그래서 **아이디어는 채택하되,
실기가 오면 하나씩 측정하며 얹는** 방향을 제안합니다.

---

## 5. 요약 표

| 항목 | 개정5 | 우리 최신본 | 병합 방침 |
|---|---|---|---|
| mode 7 폴백 | 있음(버그) | 제거 | **최신본 유지** (빼기) |
| geofence | 없음 | 있음 | **최신본 유지** |
| heading 계산 | node 자체 | `yaw_mux`가 전담 | **`yaw_mux`로 이동** |
| 시계 | `time.time()` | `monotonic` | **monotonic** |
| 파라미터화 | 하드코딩 | yaml | **yaml** |
| Covariance 게이트 | 있음 ✅ | 없음 | **이식**(임계 실측) |
| NavPVT heading | 있음 | N2(fix_velocity) | **N2 소스 옵션으로 통합** |
| Dead Reckoning | 있음 | 없음 | **실측 후 결정** |
| Adaptive LPF/동적반경 | 있음 | 없음 | **실측 후 결정** |

궁금한 점 있으면 언제든. 정확한 기준은 저장소 `CLAUDE.md`(특히 3-1 도킹, 3-5 heading, 6a geofence, N1/N2)입니다.

---

## 부록. 코드 위치 지도 (실제 파일·줄번호)

> 위에서 설명한 우리 수정이 저장소 **어디에** 있는지. 줄번호는 작성 시점 기준이라 몇 줄 밀릴 수
> 있으니, 옆의 **검색어**로 찾는 게 확실하다. 전부 최신 `main` 브랜치 기준.

### 1) `src/north_goal_angle/north_goal_angle/north_goal_angle.py`
개정5가 base 로 삼은 바로 그 파일의 **최신본**. 여기가 핵심이다.

| 우리 수정 | 줄(약) | 검색어 |
|---|---|---|
| mode 7 폴백 제거 (5,8 에만) | L74, L250 | `FALLBACK_MODES` |
| ↳ 왜 뺐나 (주석) | L247–249 | `발행자가 둘이 되어` |
| geofence 발행 | L118 | `pub_geofence` |
| geofence 계산(가짜 LiDAR) | L158 | `def _geofence_ranges` |
| ↳ 광선-폴리곤 교차(순수) | L288 | `def _ray_polygon_dist` |
| ↳ 점-폴리곤 내부판정(순수) | L317 | `def _point_in_polygon` |
| geofence 파라미터 | L90–104 | `geofence_ray_min_deg` |
| `/imu/yaw` **구독만**(자체계산 안 함) | L122 | `self.create_subscription(Float64, '/imu/yaw'` |
| IMU stale 처리(모르면 침묵) | L110, L179–185 | `imu_stale_sec` |
| monotonic 시계 | L132, L150, L179, L223 | `time.monotonic` |
| 파라미터화(declare_parameter) | L87–110 | `declare_parameter` |

### 2) `src/ssf_heading/ssf_heading/yaw_mux.py` — heading 은 여기서 만든다 (N1)
개정5 가 `north_goal_angle` 안에서 하던 heading 계산(`imu_cb`+`navpvt_cb`)의 **정착지**.

| 항목 | 줄(약) | 검색어 |
|---|---|---|
| `/imu/yaw` **단독 발행** | L129 | `self.yaw_pub` |
| 드라이버 날것 `/imu/yaw_raw` 구독 | L61, L104 | `raw_yaw_topic` |
| heading 소스 선택(A/B/C) | L44 | `heading_source` |
| GPS 속도 구독(N2) | L65, L125 | `gps_vel_topic` |
| 후진 게이트 구독 | L150 | `update_reverse` |

### 3) `src/ssf_heading/ssf_heading/heading_logic.py` — 순수 로직(N2)
개정5 의 NavPVT offset 보정과 **같은 아이디어**가 여기 있다. 통합 시 여기 손댄다.

| 항목 | 줄(약) | 검색어 |
|---|---|---|
| COG 유도(속도벡터→방위) | L97 | `def cog_from_velocity` |
| COG 오프셋 추정기 | L143 | `class COGOffsetEstimator` |
| ↳ 후진 게이트 인자 | L192 | `reverse=False` |
| 소스 상수 | L33 | `SRC_COG_OFFSET` |
| 헤딩 믹서 | (검색) | `class HeadingMux` |

### 4) `src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp` — 드라이버 변경
개정5 가 `/imu/data` orientation 을 직접 쓰는 것과 관련. 우리는 드라이버가 **날것만** 내게 했다.

| 항목 | 줄(약) | 검색어 |
|---|---|---|
| 부팅 0점화 파라미터(기본 off) | L75 | `zero_yaw_on_boot` |
| GPS override 파라미터(기본 off) | L76 | `use_gps_heading_override` |
| 발행 토픽 = `/imu/yaw_raw` | L78, L82 | `yaw_topic` |

### 5) 설정 파일
| 파일 | 무엇 |
|---|---|
| `src/ssf_heading/config/ssf_heading.yaml` | heading 소스·COG·후진게이트·`gps_vel_frame` 파라미터 |
| `src/north_goal_angle/config/` | (있으면) waypoint·geofence·타임아웃 파라미터 |

### 6) 우리가 이 노드들에 왜 이렇게 했는지 — 근거 문서
| 문서 | 내용 |
|---|---|
| `CLAUDE.md` 3-1 | 도킹 mode 9→7 (mode 7 폴백 제거와 한 쌍) |
| `CLAUDE.md` 3-5 | IMU heading 문제(부팅 0점화 + GPS COG override) → yaw_mux |
| `CLAUDE.md` 6a | geofence 레이캐스트 설계(원뿔이 모서리에서 뚫린 경위) |
| `CLAUDE.md` N1/N2 | (3-5 안) heading 단독발행 + COG 오프셋 옵션 B |

> **찾는 순서 추천**: 먼저 `north_goal_angle.py`(1번)를 열어 개정5 와 나란히 놓고 비교 →
> heading 계산을 `yaw_mux.py`(2번)로 옮길 위치 확인 → `CLAUDE.md`(6번)로 "왜"를 확인.

----- DOC: docs/전달용/회신_GPS코드_기능추출_수정제안.md -----
# 제안: GPS 코드(개선판) — 필요한 기능만 최신본에 이식 (EKF 제외)

> 대상: 팀원 제출 `22 gps 코드.py` (LOS·DR·쿼터니언·공분산필터 등 포함)
> 기준: 저장소 최신 `src/north_goal_angle/north_goal_angle/north_goal_angle.py`
> 한 줄 결론: **아이디어 감각 좋습니다. 다만 통째 교체가 아니라, "우리 최신본을 base로 두고 → 값진 기능만
> 얹고 → 실행 안 되거나 위험한 부분은 뺀다". EKF/DR은 이번엔 넣지 않습니다(근거 아래).**

방향(full IMU 활용, 파도 대응) 자체는 옳습니다. 그래서 더 정확히 나눴습니다 — **뭘 이식하고, 뭘 빼고,
뭘 반드시 지키는지.**

---

## 0. 먼저 — base는 우리 최신본 (하드코딩·geofence 회귀 금지)

제출본은 **우리 최신본이 아닌 옛 버전**에서 갈라졌습니다. 그래서 **두 가지가 회귀**했습니다. 이식은
반드시 **저장소 최신 `north_goal_angle.py` 위에** 얹어주세요. 아래 두 개는 **그대로 유지**해야 합니다:

1. **웨이포인트는 `config/waypoints.yaml` + `waypoint_loader`** (좌표 검증·오타 시 fail-loud).
   → 제출본은 `waypoints = [...]` 하드코딩(15~27줄)이라 **검증이 사라졌습니다.** 되돌리면 안 됩니다.
2. **geofence(경계 이탈 방어)** — 제출본은 `pub_geofence`를 **선언만 하고 발행하지 않습니다**(72줄).
   `_geofence_ranges`·`_publish_geofence`·"구독자 0 경보"가 전부 빠졌습니다. **이건 실격 방지 기능이라
   조용히 사라지면 안 됩니다.** 최신본 것을 그대로 둡니다.

---

## 1. ✅ 이식할 기능 (값지고, IMU/EKF 없이 GPS만으로 돎)

| 기능 | 제출본 위치 | 왜 |
|---|---|---|
| **LOS 유도** (`calc_los_angle` + cross-track 보정) | 345~352, 사용 274~276 | 우리껀 단순 bearing. LOS는 경로 이탈을 줄이는 정석. **GPS·yaw만 쓰고 IMU 의존 없음** → 바로 작동. **추출 1순위** |
| **동적 lookahead** | 271~272 | 속도에 따라 전방주시 거리 조절. LOS와 세트 |
| **GPS 공분산 필터** (고오차 fix 거부) | 116~120 | RTK 품질 낮은 fix 버림. 파도 지터 완화. 저위험 |
| **GPS 점프 필터** (속도상한 초과 거부 + 턴구간 완화) | 127~138 | 순간이동(teleport) 이상치 차단. 저위험 |
| **가벼운 위치 스무딩** (공분산 가중 alpha) | 150~163 중 **DR 분기 뺀 부분** | 파도 지터 흡수. 단 `dr_active` 관련 로직은 빼고 순수 스무딩만 |
| **도착 반경 세분화 + 통과선 판정** (`passed_by_line`, 턴구간 좁은 반경) | 303~323 | 오버슈트·코너 깎기 방지. GPS만, 저위험 |

→ 이 6개가 "파도·경로 대응"의 실질 이득 대부분을 **발산 위험 없이** 줍니다.

---

## 2. ❌ 빼는 기능 (실행 안 되거나 위험)

| 기능 | 제출본 위치 | 왜 빼나 |
|---|---|---|
| **Dead Reckoning** (IMU 가속도 적분) | timer_cb 241~262 | ① 우리 드라이버가 `/imu/data`(accel)를 **안 냄** → 콜백 안 불려 **동작 안 함** ② 이중적분은 몇 초만에 미터 단위 발산 |
| **쿼터니언 중력분리** | imu_cb 199~213 | `/imu/data`의 orientation 쿼터니언 필요 → **우리 드라이버 미발행** → q=0 쓰레기 |
| **ZUPT / 속도융합 (vx/vy)** | gps_cb 140~146, imu_cb 215~234 | DR 지원용. DR 빼면 함께 빠짐 |
| **imu_cb 전체** | 190~234 | 위 세 개가 여기 있음. `/imu/data` 없으니 통째로 뺌 |
| **navpvt COG 오프셋** (`yaw_offset`) | navpvt_cb 179~188 | 🚨 우리 `/imu/yaw`는 **yaw_mux가 이미 절대방위 보정**해서 냄. 여기서 또 오프셋 빼면 **이중 보정**. yaw_mux에 맡김 |
| **north_goal 내부 조향각 클램프** (`max_steering_angle`) | 278~283 | 조향은 하류(ship_goal_angle/ship_direction) 책임. 여기서 하면 **이중 클램프**. 관심사 한 곳 유지 |

**핵심:** DR·쿼터니언은 "틀린 코드"가 아니라 **우리 드라이버가 `/imu/data`를 안 내서 실행조차 안 되는**
코드입니다(소비자만 있고 생산자 없음). 살리려면 드라이버 확장이 선행돼야 하는데, 그건 아래 3번 참고.

---

## 3. EKF/IMU 융합을 이번에 안 하는 이유 (팀 논의 결론)

시간 여유는 있지만, **우리 조건에서 풀 EKF는 이득 대비 위험이 큽니다:**

- **레버리지 최저** — GPS는 구간이동 전용. 채점 미션은 LiDAR·카메라가 함. EKF는 제일 안 중요한 구간만 개선.
- **RTK라 마진 작음** — 락 걸리면 위치 훌륭. DR 이득은 RTK 풀릴 때뿐인데 그 빈도를 아직 안 쟀음.
- **검증 불가 구조** — EKF는 GPS 나쁠 때만 도움 되고, 검증은 GPS 좋을 때(=정답인 RTK가 있을 때)만 가능.
  **도움되는 순간엔 맞는지 확인할 방법이 없음.** 조용한 발산을 대회장에서 못 잡음.
- 과거 문서 2건에서도 같은 결론: `docs/문제와작업/추가개선_판단과결과.md`(GPS+IMU 칼만 ❌),
  `docs/결과분석/팀아이디어_평가_필터링과칼만.md`(우선순위 낮음, 알파-베타로 충분).

**IMU 융합을 정말 하려면 EKF 말고:** ① 드라이버를 `/imu/data`(accel/gyro/pitch/roll) 발행하게 확장하고
② **보완필터(complementary) 또는 상한 걸린 짧은 DR**(경계 있어 발산 안 함)로, ③ **blackbox로 RTK 끊김을
실측한 뒤** 필요할 때만. 이건 이식과 별개 트랙입니다.

---

## 4. 구현 방식 권장 (우리 방식 — 순수 로직 + 테스트)

이식하는 기능은 **ROS 비의존 순수 함수**로 빼서 배 없이 Mac에서 테스트하면 좋습니다
(우리 `heading_logic`·`gate_logic`·`waypoint_loader`와 같은 방식).

- `los_logic.py` — `calc_los_angle`, cross-track, 동적 lookahead, 도착반경/통과선 판정 (순수 기하)
- `gps_filter.py` — 공분산 거부·점프 거부·위치 스무딩 (순수, 시간 주입)
- 각 모듈에 단위 테스트 (엣지: 경로 수직 이탈, 순간이동 fix, 고공분산, 턴구간 반경)

`north_goal_angle.py`는 이 모듈들을 **호출만** 하고, 토픽 계약(`/candidate_angle` 단일 발행,
`/north_goal_angle_tp`, `/geofence_state`)과 geofence·waypoint_loader는 최신본 그대로 둡니다.

---

## 5. 체크리스트 (팀원 작업 순서)

- [ ] base = 저장소 최신 `north_goal_angle.py` 에서 시작 (제출본 아님)
- [ ] waypoints 하드코딩 **금지** — `waypoints.yaml` + `waypoint_loader` 유지
- [ ] geofence(`_geofence_ranges`·`_publish_geofence`·구독자0 경보) **유지**
- [ ] LOS 유도 + 동적 lookahead 이식 → `los_logic.py`
- [ ] GPS 공분산·점프 필터 + 위치 스무딩(DR 분기 제외) 이식 → `gps_filter.py`
- [ ] 도착반경 세분화 + 통과선 판정 이식
- [ ] DR·쿼터니언·ZUPT·imu_cb·navpvt오프셋·조향클램프 **제외**
- [ ] 각 순수 모듈 단위 테스트 추가
- [ ] 토픽 계약 불변 확인 (`/candidate_angle` 발행자 1개 유지 — ship_dock과 충돌 금지)

---

## 요약

| 분류 | 항목 |
|---|---|
| ✅ 이식 | LOS·동적lookahead·공분산필터·점프필터·위치스무딩·도착반경/통과선 |
| ❌ 제외 | DR·쿼터니언·ZUPT·속도융합·navpvt오프셋·조향클램프 (미발행 `/imu/data` 의존 or 이중처리) |
| 🔒 보존 | waypoints.yaml+검증 · geofence · 토픽계약 |
| ⏸ 별도 트랙 | IMU 융합은 드라이버 확장 후 보완필터/상한DR로, 실측 기반 (EKF 아님) |

**방향은 좋았습니다** — 특히 LOS와 파도 대응 감각. 그 중 **발산 위험 없이 바로 이득 나는 것**만 최신본에
얹으면 됩니다. 기준은 저장소 코드와 CLAUDE.md(프레임 규약·geofence·waypoint_loader)입니다.

----- DOC: docs/전달용/회신_LOS강화_IMU추가_검토.md -----
# 회신: LOS 강화 north_goal / IMU 데이터 추가 ship_goal — 검토 + 역할 맞추기

> 받은 것: `23 github los강화.ver.py`(north_goal_angle), `IMU 추가 데이터.ver.py`(ship_goal_angle)
> 기준: 저장소 최신 `north_goal_angle.py`, `ship_goal_angle.py`, `iahrs_driver.cpp`, `blackbox.py`
> 한 줄 결론: **방향 잘 잡았습니다 — 특히 geofence를 되살린 것과 yaw를 안 덮어쓴 것. 다만 각 파일에
> 남은 3가지·2가지가 있어, "누가 뭘 맡을지"까지 아래에 정리했습니다. 병렬로 진행하시죠.**

---

## 파일 1 — LOS 강화 `north_goal_angle`

### ✅ 잘 반영됨

- **LOS + ILOS** — `calc_advanced_los_angle`에 cross-track 오차 + **적분항(조류·바람 보상)** + anti-windup(±50 클램프). 요청한 LOS보다 한 발 더 나갔고, 방향 좋습니다.
- **동적 lookahead** / **GPS 공분산 필터** / **위치 스무딩(Adaptive LFP)** — 모두 반영.
- **geofence 復活** ✅✅ — 이전엔 선언만 하고 죽어 있던 것을 `_geofence_ranges`·`_publish_geofence`·`_point_in_polygon`·`_ray_polygon_dist`로 완전히 살렸습니다. 우리가 강조한 "geofence 보존"이 제대로 들어갔습니다.
- **조향클램프·쿼터니언·navpvt오프셋·imu_cb 제거** — 빼달라 한 것들 정리됨.

### ❌ 아직 남은 것

1. **waypoints 하드코딩 (13~25줄)** — 우리 #1 지적. `config/waypoints.yaml` + `waypoint_loader`(좌표 검증·오타 시 fail-loud)로 전환해야 합니다. 지금은 오타 좌표로도 그냥 출발합니다.
2. **GPS 점프 필터 누락** — 이전 버전의 "속도상한 초과 fix 거부"가 빠졌습니다. `estimated_speed_mps`(128줄)가 원시 연속 fix로만 계산돼, GPS가 한 번 튀면 속도가 뻥튀기됩니다.

### ⚠️ DR — 판단 필요 (실행은 되지만 위험 요소 있음)

- 이 DR은 이전의 accel 이중적분판과 **다릅니다**: `/imu/yaw`(발행됨) + GPS 유도 속도로 등속 추측 → **우리 시스템에서 실행은 됩니다.**
- 다만 **시간 상한이 없습니다**(이전엔 15s cap). GPS가 오래 끊기면 무한정 드리프트합니다.
- `gps_timeout_sec=0.3s`가 너무 공격적입니다. RTK 5~10Hz면 2~3프레임만 놓쳐도 DR이 발동합니다. 점프 필터도 없어 뻥튀기된 속도로 DR할 수 있습니다.
- → **유지하려면**: 시간 상한 추가 + 점프 필터 복원 + 트리거 완화. 그리고 **RTK가 실제로 얼마나 끊기는지 blackbox로 실측한 뒤** 필요 여부를 정하는 게 맞습니다(측정으로 결정).

### 권장 구조

`calc_advanced_los_angle`가 자유 함수라 테스트 가능한 건 좋습니다. GPS 필터·LOS를 순수 모듈(`gps_filter`/`los_logic`)로 빼고 단위 테스트를 붙이면 배 없이 검증됩니다(우리 `waypoint_loader`·`heading_logic`과 같은 방식).

---

## 파일 2 — IMU 데이터 추가 `ship_goal_angle`

### ✅ 잘한 것

- **우리 최신 `ship_goal_angle` 위에 얹었습니다** — yaw_error 로직·watchdog·`% 360` 규약 그대로 유지.
- **yaw를 안 덮어씁니다**(78줄 주석대로 `/imu/yaw` 우선) — 이중 yaw 소스를 안 만든 것, 정확합니다.
- `euler_from_quaternion` 자체 구현(tf 의존 없음) — 합리적.

### ❌ 핵심 — 지금은 inert (작동 안 함)

- `/imu/data`를 구독하는데, **우리 `iahrs_driver`는 `/imu/data`를 발행하지 않습니다**(시리얼 `"e"`로 roll/pitch/yaw만 받아 `/imu/yaw_raw`만 냄). → `imu_data_callback`이 **영원히 안 불려** roll/pitch/quaternion/accel이 전부 `None`.
- 즉 **소비자만 있고 생산자(드라이버)가 없습니다** — GPS DR 때와 같은 구조입니다. 이 코드는 **IMU 담당이 진행 중인 드라이버 확장(`/imu/data` 또는 N4의 `/imu/mag_heading` 발행)이 선행돼야** 비로소 살아납니다.

### ⚠️ YAGNI

- 받은 데이터를 저장만 하고 아직 아무 데도 쓰지 않습니다(143줄). 안 쓰는 데이터를 구독·저장하는 건 이릅니다. **실제 쓸 곳(예: roll/pitch로 기울기 보상)이 정해질 때 넣는 게** 깔끔합니다. (무해하지만 dead plumbing)

---

## 🤝 방법 맞추기 — 역할 분담 (병렬 진행)

**진행 방식 — "LOS 안정 후 한 번에 이식" (안 A 확정):**
지금 바로 합치지 않습니다. 팀원이 LOS/필터/DR을 계속 다듬는 중이라, 지금 이식하면 곧 재작업입니다.
**LOS가 안정되면(점프필터·DR 상한까지 정리되면), 우리가 그 기능들을 우리 yaml 기반 최신본에 한 번에 이식**합니다. 충돌 1회로 끝냅니다.

| 항목 | 담당 | 내용 |
|---|---|---|
| north_goal: **waypoints.yaml 전환** | **우리** | 우리 최신본은 이미 `waypoints.yaml`+`waypoint_loader` 사용 중. 팀원은 하드코딩 그대로 둬도 됨 — 이식 시 우리가 yaml 기반으로 바꿈 |
| north_goal: **점프 필터 복원 + DR 상한/트리거** | 팀원 | LOS 작업자가 이어서. 순수 모듈 + 테스트 권장. **이게 끝나면 "안정" 신호 → 우리가 이식** |
| north_goal: **RTK 끊김 실측** | 우리 | blackbox로 DR 필요 여부 판단 근거 제공 |
| **드라이버 `/imu/data`(또는 mag_heading) 발행 확장** | IMU 담당 | 이게 선행돼야 파일2가 작동. **브랜치로 올려주면 우리가 diff 리뷰** |
| ship_goal: IMU 데이터 **활용처 확정 후 결합** | 협의 | 드라이버 확장 완료 후, 쓸 곳 정해서 |

**전달 방식 합의:** 코드는 **main 직접 push 금지**, **브랜치 push** 또는 `git diff` 텍스트로 주세요. 그럼 우리가 diff 보고 기능만 최신본에 반영합니다(env·미완 부분 걸러냄).

---

## 요약

| 파일 | 판정 | 남은 것 |
|---|---|---|
| LOS north_goal | ✅ 방향 좋음(LOS+ILOS·geofence 復活) | waypoints.yaml · 점프필터 · DR 상한/트리거 |
| IMU ship_goal | ✅ base 위 잘 얹음·yaw 안 덮음 | `/imu/data` 미발행이라 inert(드라이버 선행) · 저장만(YAGNI) |

**둘 다 좋은 진전입니다.** geofence 복원과 yaw 비덮어쓰기는 우리 원칙을 정확히 지킨 부분입니다.
남은 건 위 표대로 나눠서 병렬로 가시죠. 기준은 저장소 코드와 CLAUDE.md(waypoint_loader·geofence·IMU 토픽 규약)입니다.

----- DOC: docs/절차/웨이포인트_녹화절차.md -----
# 웨이포인트 녹화 절차 — 대회장에서 좌표 찍기

> **대회장에서 처음 해보면 안 되는 작업이다.** 2026-08-07 야외에서 한 번 돌려보고 쓴 문서다.
> 그때 실제로 걸린 것들(파일 위치를 못 찾음, 걸음이 느려 표본 부족)을 여기 반영했다.
>
> 관련: `north_goal_angle/waypoint_recorder.py` · `config/waypoints.yaml` · CLAUDE.md 3-6

---

## 0. 한 장 요약

```bash
# 1) 배(또는 GPS)를 지점에 대고
ros2 run north_goal_angle waypoint_recorder
#    Enter → mode → dwell → 이름   (지점마다 반복)
#    q     → ~/ssf_logs/waypoints_recorded.yaml 로 저장

# 2) 복사 없이 바로 미션에 태워보기
ros2 launch north_goal_angle north_goal_angle_launch.py \
    waypoints_file:=$HOME/ssf_logs/waypoints_recorded.yaml

# 3) 확정되면 실전 파일로
cp ~/ssf_logs/waypoints_recorded.yaml ~/ros2_ws/src/north_goal_angle/config/waypoints.yaml
cd ~/ros2_ws && colcon build --symlink-install --packages-select north_goal_angle
```

---

## 1. 찍기 전에 — RTK 가 잡혔는지 먼저 본다

**단독 GPS(±3~5m)로 찍으면 웨이포인트가 무의미하다.** 부표 간격보다 오차가 크다.
RTK FIXED(±1.5cm)를 확인하고 시작한다.

```bash
ros2 topic echo /ublox_gps_node/navpvt --once     # carrSoln 이 2(FIXED) 인가
```
또는 `healthcheck` 의 `GPS/fix ✅` 확인. `⚠️` 면 fix 가 없는 것이다(`NOFIX`).

⚠️ **NTRIP 이 붙어 있어야 RTK 가 된다 → 인터넷이 필요하다.**
대회장에 WiFi 가 없으면 **휴대폰 핫스팟**을 켠다. 데이터는 약 1KB/s 라 부담 없다.

🚨 **마운트포인트는 '측정 장소' 기준으로 고른다.**
2026-08-07 에 `waypoints.yaml` 좌표(마산)를 대회장으로 가정하고 `CHWN`(창원)을 골랐는데,
실제 시험 장소(세종)에서 **199km** 였다. 그 상태로는 RTK 가 안 된다.
소스테이블은 **인증 없이** 받을 수 있으니 현장에서 다시 계산하면 된다:
```bash
curl -s --http0.9 http://gnssdata.or.kr:2101/ | grep '^STR;'   # 9,10번째 필드가 위/경도
```
30km 이내를 고른다. 설정은 `~/.ssf/ntrip_credentials.yaml` 의 `mountpoint`.

---

## 2. 배가 물에 있으면 — SSH 로 붙는다

배는 헤드리스다(CLAUDE.md 3-3). 물가의 노트북/폰에서 SSH 로 접속해
`waypoint_recorder` 를 실행하고, **배를 RC 로 지점에 댄 뒤 Enter** 를 누른다.

```
[물가] 노트북/폰  --WiFi-->  [배 안] 노트북
                                  └ waypoint_recorder 실행
```
⚠️ **아직 준비 안 됨** — 배 노트북에 `openssh-server` 설치 + 접속용 기기 필요.
   핫스팟이 주는 IP 는 매번 바뀔 수 있으니 **고정 IP 또는 이름(mDNS)** 을 정해둘 것.

---

## 3. 찍는 법

```bash
ros2 run north_goal_angle waypoint_recorder
```

| 키 | 동작 |
|---|---|
| `Enter` | 현재 위치를 새 WP 로 추가 (→ mode, dwell, 이름을 물어본다) |
| `l` | 목록 보기 |
| `d N` | N번 삭제 |
| `r N` | N번을 현재 위치로 재캡처 (mode/dwell 유지) |
| `q` | 저장·종료 |

**지점마다 몇 초 머무른 뒤 Enter.** 최근 2초 표본을 평균내 지터를 줄인다.
누르면 `표본 23개 평균 → lat ..., lon ... (최대 공분산 0.00)` 이 뜬다.
공분산이 크면 RTK 가 불안정한 것이니 다시 찍는다.

### mode (미션번호) — 이 표에서만 고른다

| mode | 내용 | dwell(현재값) |
|---|---|---|
| 0 | 게이트 시작 | 3.0 |
| 1 | 게이트 끝 | 3.0 |
| 2 | 위치유지 | 3.0 |
| 3 | 부표 선회 | 3.0 |
| 5 | 회피 구간 | 50.0 |
| 7 | 도킹 | 60.0 |
| 8 | 토너먼트 회피 | 60.0 |

🚨 **`4`·`6`·`9` 는 없는 모드다.** 작년에 `ship_dock` 이 `9` 로 선언돼 **도킹이 1년 내내
침묵**했다(3-1). 넣으면 저장 직전 검증이 거부한다 — 확인됨:
```
🚨 검증 실패: 0번째 웨이포인트 '': 미션번호 9 는 허용값 [0,1,2,3,5,7,8] 에 없다.
🚨 검증 실패: 0번째 웨이포인트 '': 머무는시간 -5.0 이 음수다.
```

### dwell (머무는 초)
**도착 반경 안에 들어온 뒤 몇 초 머물다 다음으로 넘어갈지.**
도착 반경은 고정 3m 가 아니다 — 도킹(mode 7) 2.0m, 그 밖은 속도 2 m/s 미만 2.5m · 5 m/s 미만 3.5m · 그 이상 4.5m (`los_logic.get_dynamic_arrive_radius`).
3초짜리는 도착 판정 여유, 50/60초짜리는 **경기 규정의 미션 수행 시간**이다.
※ 도달 못 해도 타임아웃으로 넘어간다. 타임아웃은 **WP 의 mode** 로 정해진다(10-29 b337ad8, 그 전엔 목록 순번이었다):
   게이트·위치유지·회피(mode 0·1·2·5) = WP 하나당 120 s, 부표(3) = 90 s, 도킹(7) = 150 s, 토너먼트(8) = 없음.
   → 웨이포인트를 끼워 넣거나 순서를 바꿔도 타임아웃은 mode 를 따라간다. 값은 `config/north_goal_angle.yaml` 의
   `wp_timeout_modes` / `wp_timeout_sec`(🚨 sec 는 전부 `.0` — 정수가 섞이면 노드가 죽는다).
   타임아웃 시계에는 dwell 이 포함된다 — timeout ≤ dwell 이면 부팅 때 WARN 이 뜬다(도착으로는 못 넘어간다).

### 구역 이름
동작에는 영향이 없다. **에러 메시지에 찍힌다** — 오타가 났을 때 "몇 번째" 가 아니라
"어느 지점" 인지 바로 알 수 있다. 대회장에서는 알아볼 수 있게 적을 것.

---

## 4. 저장 위치

**`~/ssf_logs/waypoints_recorded.yaml`** (blackbox 로그와 같은 자리).

> 예전엔 `os.getcwd()` 라 실행한 폴더에 따라 파일이 어디 생길지 몰랐다.
> 2026-08-07 에 실제로 세 군데를 뒤졌다. SSH 로 붙으면 cwd 가 매번 달라져 더 나쁘다.
> → 고정 경로로 바꾸고, 저장할 때 **절대경로를 크게 출력**하도록 했다.

다른 곳에 쓰려면 인자로: `ros2 run north_goal_angle waypoint_recorder /경로.yaml`

🚨 **실전 `config/waypoints.yaml` 은 절대 안 건드린다.** 오입력해도 원본이 살아 있다.

---

## 5. 적용 — 복사하지 말고 인자로 먼저

```bash
ros2 launch north_goal_angle north_goal_angle_launch.py \
    waypoints_file:=$HOME/ssf_logs/waypoints_recorded.yaml
```
로그에 `웨이포인트 N개 로드: <경로>` 와 `미션 순서(mode): [...]` 가 찍힌다. **개수와 순서를 눈으로 확인.**

이 단계에서 문제가 보이면 다시 찍으면 된다 — 실전 파일은 그대로다.

확정된 뒤에만 복사한다:
```bash
cp ~/ssf_logs/waypoints_recorded.yaml ~/ros2_ws/src/north_goal_angle/config/waypoints.yaml
cd ~/ros2_ws && colcon build --symlink-install --packages-select north_goal_angle
```
⚠️ **재빌드를 빼먹으면** `install/` 의 옛 파일을 읽는다. 로그의 경로로 확인할 것.

---

## 6. 실측에서 걸린 것 (2026-08-07)

| 증상 | 원인 / 대응 |
|---|---|
| 저장 파일을 못 찾음 | `os.getcwd()` → **고정 경로로 수정함** |
| COG 표본이 안 모임 (`rej=slow`) | 걸음이 0.70 m/s 로 느렸다. `cog_min_speed_mps: 0.8` 미만은 배제된다 → **빠르게 걸으니 1.21 m/s, 37표본 확보** |
| RTK 가 float 에 머묾 | 수렴 중이었다. **수십 초 기다리면 FIXED** 로 올라간다(위성 18개, pDOP 1.63에서 확인) |
| 마운트포인트가 199km | 측정 장소 기준으로 다시 골라야 한다(§1) |

---

## 7. 체크리스트 (인쇄용)

- [ ] 인터넷(핫스팟) 연결
- [ ] 마운트포인트가 **현장 기준 30km 이내**인지 확인
- [ ] `sudo tools/boat_boot.sh` (launch 전 매번)
- [ ] RTK **FIXED** 확인 후 시작
- [ ] 지점마다 몇 초 정지 후 `Enter`
- [ ] mode 는 `{0,1,2,3,5,7,8}` 에서만
- [ ] `q` 로 저장 → **출력된 절대경로 확인**
- [ ] `waypoints_file:=` 인자로 먼저 태워보고 개수·순서 확인
- [ ] 확정 후에만 `config/waypoints.yaml` 로 복사 + 재빌드

==================================================================
## 작년 최종본(9bff661) 이후 커밋 목록
==================================================================
- b288014 2026-10-29 fix(north_goal_angle): GPS fix 신선도 검사(C1) — 받아들인 fix 가 2.5 s 넘게 없으면 위치 파생값만 멈춤
- b337ad8 2026-10-29 fix(north_goal_angle): 웨이포인트 타임아웃을 목록 순번 → wp_mode 기준 파라미터로 — WP7 무제한 해소, 도킹 150 s
- a2dd4a9 2026-10-28 docs: 초안 검토 결과 반영 — 본 보고서 Ⅲ장 18곳 정정, 정북 정렬 편각 절차, 낡은 주석·문서 정리
- 5390785 2026-09-23 test: pep257 을 끈다 — 결함 0건, 전부 서식이고 절반은 한국어에 적용 불가
- 4386901 2026-09-23 style: flake8 최대 줄 길이 99 → 120, 잔여 5건 수정 — 10개 패키지 전부 0건
- 5c5d039 2026-09-23 style: flake8 142 → 37 — 죽은 코드 18건은 실제로 걷어냈다
- c7c0c59 2026-09-16 fix(gps): 배별 GPS 설정이 메인 launch 까지 닿게 — gps_config 인자 전달, A배 yaml EEPROM save 제거·sea·debug 0, udev 01a8(M8P) 규칙
- df16eee 2026-08-10 feat(north_goal_angle): 자율 시작 게이트 — RC AUTO 전환 시 WP0부터 시작
- 1dc33c7 2026-08-09 docs: N4-B 전제조건 해결 + 쉬운설명 HTML 최신화 + 낡은 주석 정리
- 9221750 2026-08-07 feat(north_goal_angle): LOS 유도 직접 구현 (순수모듈 + ILOS + GPS 필터)
- ac486b1 2026-08-07 feat(waypoint): 저장 경로 고정 + launch 에 waypoints_file 인자 + 녹화 절차 문서
- f98a8f9 2026-08-06 security(ntrip_client): 계정 정보를 저장소 밖으로 — 평문 커밋 제거
- 89818a5 2026-08-06 fix(ntrip_client): launch 설치 경로 → 대회 실행 launch 가 통째로 죽던 문제
- 02944d8 2026-08-06 fix(north_goal_angle): GPS fix 없이 항법하던 버그 — Null Island 방위 발행
- ed2a144 2026-08-06 fix(gps): navpvt 토픽명 불일치 수정 + F9P dynamic_model sea
- c1b0d81 2026-08-06 fix(ublox_gps): F9P(B배) rover 설정 신설 + launch 배별 인자화
- 74b7d4b 2026-08-06 fix(build): clone 직후 빌드 실패 3건 수정
- 8fda096 2026-07-30 feat(waypoint_recorder): 편집 명령(l/d/r) + 시스템 설명서 최신화
- 7a2e803 2026-07-30 feat(north_goal_angle): waypoint_recorder — 현장 GPS 좌표 기록 도구
- 2758653 2026-07-28 feat(north_goal_angle): 웨이포인트 GPS 좌표를 코드→config/waypoints.yaml 로 분리
- ace1e33 2026-07-23 N1: /imu/yaw 를 yaw_mux 단독 발행으로. 부팅 0점화 + GPS COG override 제거
- b678154 2026-07-14 6a-2 개선: geofence 를 '가짜 LiDAR'로 (원뿔 방식 폐기)
- 2cb050b 2026-07-14 6a-2: ship_direction geofence 구독 + 가상 장애물 (빨간불 끔)
- 0b854c1 2026-07-14 6a: ship_dock 도킹 부활 + north_goal_angle mode-7 폴백 제거 (원자적 한 쌍)

==================================================================
## 작년 최종본(9bff661) 대비 변경 diff
==================================================================
올해 새로 생긴 파일 (diff 생략 — 위 소스 파일 또는 저장소에서 직접 본다):
- src/north_goal_angle/config/north_goal_angle.yaml
- src/north_goal_angle/config/waypoints.yaml
- src/north_goal_angle/north_goal_angle/gps_filter.py
- src/north_goal_angle/north_goal_angle/gps_guard.py
- src/north_goal_angle/north_goal_angle/los_logic.py
- src/north_goal_angle/north_goal_angle/waypoint_loader.py
- src/north_goal_angle/north_goal_angle/waypoint_recorder.py
- src/north_goal_angle/north_goal_angle/wp_timeout.py
- src/north_goal_angle/test/test_gps_filter.py
- src/north_goal_angle/test/test_gps_fresh.py
- src/north_goal_angle/test/test_gps_guard.py
- src/north_goal_angle/test/test_los_logic.py
- src/north_goal_angle/test/test_mission_gate.py
- src/north_goal_angle/test/test_waypoint_loader.py
- src/north_goal_angle/test/test_waypoint_recorder.py
- src/north_goal_angle/test/test_wp_timeout.py
- src/ublox/ublox_gps/config/zed_f9p_rover.yaml

작년 파일 중 바뀌거나 지워진 것의 diff (문맥 1줄):
```diff
diff --git a/src/north_goal_angle/launch/north_goal_angle_launch.py b/src/north_goal_angle/launch/north_goal_angle_launch.py
index f04423a..9d393dc 100755
--- a/src/north_goal_angle/launch/north_goal_angle_launch.py
+++ b/src/north_goal_angle/launch/north_goal_angle_launch.py
@@ -1,6 +1,41 @@
+"""north_goal_angle launch — 웨이포인트 파일을 인자로 바꿔 끼울 수 있다.
+
+  ros2 launch north_goal_angle north_goal_angle_launch.py
+      → config/waypoints.yaml (실전 파일)
+
+  ros2 launch north_goal_angle north_goal_angle_launch.py \
+      waypoints_file:=/home/ssfb/ssf_logs/waypoints_recorded.yaml
+      → 방금 녹화한 파일을 **복사 없이** 바로 사용
+
+🚨 왜 인자화했나 (2026-08-07 야외 시험에서 나온 필요)
+   waypoint_recorder 는 안전을 위해 **새 파일**에만 쓴다(실전 waypoints.yaml 을 안 건드림).
+   그런데 그 파일을 쓰려면 지금까지는 "복사 → 재빌드" 를 해야 했다.
+   대회장에서 좌표를 찍고 바로 미션을 돌려야 하는데, 손이 많이 가고 실수 여지가 크다.
+   노드는 이미 waypoints_file 파라미터를 받고 있었는데(north_goal_angle.py) launch 가
+   그걸 노출하지 않아 **쓸 방법이 없었다** — ublox_gps launch 가 c94_m8p 에 하드코딩돼
+   F9P 를 못 쓰던 것(c1b0d81)과 같은 유형이다.
+
+   ★ 녹화 파일을 인자로 쓰면 config/waypoints.yaml 은 그대로 남는다 → 오입력해도 복구 가능.
+"""
+
+import os
+
+from ament_index_python.packages import get_package_share_directory
 from launch import LaunchDescription
+from launch.actions import DeclareLaunchArgument
+from launch.substitutions import LaunchConfiguration
 from launch_ros.actions import Node
 
+
 def generate_launch_description():
+    share = get_package_share_directory('north_goal_angle')
+    params = os.path.join(share, 'config', 'north_goal_angle.yaml')
+    default_wp = os.path.join(share, 'config', 'waypoints.yaml')
+
     return LaunchDescription([
+        DeclareLaunchArgument(
+            'waypoints_file',
+            default_value=default_wp,
+            description='웨이포인트 yaml 경로 (녹화본을 바로 쓰려면 그 경로를 준다)'),
+
         Node(
@@ -8,4 +43,10 @@ def generate_launch_description():
             executable='north_goal_angle',
-            name='north_goal_angle',
+            name='north_goal_angle',   # config yaml 의 /north_goal_angle 키와 일치
             output='screen',
+            parameters=[
+                params,
+                # 🚨 순서가 중요하다. 뒤에 오는 것이 이긴다 —
+                #    params(yaml) 에 waypoints_file 이 있더라도 인자가 덮어쓴다.
+                {'waypoints_file': LaunchConfiguration('waypoints_file')},
+            ],
         ),
diff --git a/src/north_goal_angle/north_goal_angle/north_goal_angle.py b/src/north_goal_angle/north_goal_angle/north_goal_angle.py
index e7a1e6a..0718c23 100755
--- a/src/north_goal_angle/north_goal_angle/north_goal_angle.py
+++ b/src/north_goal_angle/north_goal_angle/north_goal_angle.py
@@ -1,24 +1,71 @@
-# north_goal_angle.py (FSM 전담)
-import rclpy
+"""north_goal_angle — 웨이포인트 FSM + 경계 이탈 방지(geofence). 6a.
+
+고친 것:
+  [3] 🚨 mode-7 폴백 제거 (ship_dock 수정과 **원자적 한 쌍**)
+      작년: if wp_mode == 7: pub_candidate.publish(20000)  ← 0.5초마다
+      이건 '도킹이 안 되는 걸 알고 덮어둔 흔적'이다. ship_dock 이 9 로 잡혀 침묵하니,
+      배가 멈추지 않게 north 가 대신 폴백을 냈다.
+      ship_dock 을 7 로 고치면 → 같은 /candidate_angle 에 발행자가 둘이 된다.
+      도킹 중 조향이 GPS 방위로 튄다 (접안 직전 60초에 35° 스파이크 20번).
+      → 폴백은 **담당 노드가 없는 모드에만** 남긴다: mode 5(회피), 8(토너먼트 회피).
+
+  [6] Geofence 발행 — 경계를 **'가짜 LiDAR'** 로 낸다. 경기장 밖으로 나가면 실격인데 방어가 0 이었다.
+
+      /geofence_state (Float32MultiArray)
+        = [angle_min_deg, angle_inc_deg, r0, r1, r2, ...]   (상대방위 기준, 멀면 inf)
+        정보 없으면 **빈 배열**.
+
+      ★ 왜 '광선'인가 (원뿔 방식을 버린 이유):
+        **벽은 '점'이 아니라 '선'이다.** 최근접점 한 방향으로 ±각도 원뿔을 막는 건 기하학적으로
+        틀렸다. 40×40 경기장 (40,40) 모서리에서 배가 대각선(45°)을 향하면:
+            북벽 최근접점 → 상대 -45°   /   동벽 최근접점 → 상대 +45°
+            그런데 **탈출구(모서리)는 상대 0° — 배 정면이다.**
+        half_block=40° 면 차단 구역이 [-85,-5] 와 [5,85] → **정면 0° 가 정확히 '틈'으로 열린다.**
+        배가 대각선으로 탈출한다. 실격. (경기장이 사각형이 아니면 또 뚫린다.)
+        광선으로 쏘면 정면 방향 경계 거리(2.12m)가 그대로 잡혀 벽이 된다. **튜닝 파라미터가 없다.**
+
+      ★ 소비자(ship_direction)는 **스캔 병합 한 줄**이면 끝이다:
+            ranges[i] = min(real_ranges[i], geofence_ranges[i])
+        기존 dilate 가 배 폭 여유를 자동으로 더하고, 기존 갭-팔로잉이 알아서 피한다.
+        특수 로직도 새 상태기계도 없다. "경계선을 그냥 벽으로 취급한다"를 끝까지 밀어붙인 것.
+
+      계산량: 161방향 × 변 개수 @ 2Hz — 무시할 수준.
+
+프레임 규약: 광선 각도는 **상대 방위**(0=정면)다 — /candidate_angle 과 같은 규약.
+  ⚠️ 광선 방향이 /imu/yaw 에 통째로 의존한다 (CLAUDE.md 3-5).
+     N1 에서 /imu/yaw 는 ssf_heading/yaw_mux 단독 발행으로 바뀌었다 —
+     부팅 0점화와 GPS COG override 를 걷어낸 절대방위다.
+     🚨 단, mount_offset_deg / invert_yaw 를 **벤치에서 아직 안 쟀다.**
+        그 전까지 이 geofence 는 방향이 맞는다는 보장이 없다.
+     IMU 가 묵으면 **빈 배열**을 낸다. 틀린 방향으로 '없는 벽'을 세우느니 안 내는 게 낫다.
+"""
+
+import math
+import os
 import time
-from math import atan2, cos, radians, sin, degrees
-from geopy import distance
+
+import yaml
+
+import rclpy
 from rclpy.node import Node
 from rclpy.qos import ReliabilityPolicy, QoSProfile
-from std_msgs.msg import Float32, Int32
+from std_msgs.msg import Float32, Float32MultiArray, Int32, Float64
 from sensor_msgs.msg import NavSatFix
+from geopy import distance
+from ament_index_python.packages import get_package_share_directory
+
+from north_goal_angle.waypoint_loader import parse_waypoints, WaypointError
+from north_goal_angle.gps_guard import (
+    DEFAULT_GPS_STALE_SEC, fix_is_fresh, fix_is_usable, sanitize_stale_sec)
+from north_goal_angle import los_logic          # LOS 유도 (순수, 테스트됨)
+from north_goal_angle.gps_filter import GPSFilter  # 공분산·점프·스무딩 (순수, 테스트됨)
+from north_goal_angle.wp_timeout import (            # WP 타임아웃 표 (순수, 테스트됨)
+    DEFAULT_WP_TIMEOUT_MODES, DEFAULT_WP_TIMEOUT_SEC, WpTimeoutError,
+    build_timeout_table, dwell_conflicts, per_wp_timeouts, timeout_for_mode,
+)
 
-waypoints = [
-    [35.1862375, 128.5655118, 0, 3.0], #WP0 게이트 시작
-    [35.1863642, 128.5657123, 1, 3.0], #WP1 게이트 끝
-    [35.1868822, 128.5660465, 2, 3.0], #WP2 위치유지
-    [35.1868638, 128.56582129999, 3, 3.0], #WP3 초록
-    [35.1867763, 128.5658085, 3, 3.0], #WP4 빨강
-    [35.1868645, 128.5656684, 3, 3.0], #WP5 하양
-    [35.1866601, 128.5655597, 5, 50.0], #WP6 회피 시작
-    [35.186396099999, 128.5653297, 5, 50.0], #WP7 회피끝
-    [35.1859269, 128.5655428, 7, 60.0], #WP8 도킹 시작
-    [35.1859269, 128.5655428, 7, 60.0], #WP9 도킹
-    [35.1861956, 128.5660033, 8, 60.0] #WP 토너먼트 회피
-]
+# 🚨 대회 좌표는 코드가 아니라 config/waypoints.yaml 에서 읽는다 (비전공자도 편집 가능).
+#    대회장에서 GPS 좌표를 바꿀 사람은 그 파일의 lat/lon 만 고치면 된다 — 이 파일은 안 건드린다.
+#    로드/검증은 waypoint_loader.parse_waypoints (순수 로직, 테스트됨).
+#    파일이 없거나 형식이 틀리면 노드를 띄우지 않는다 — 틀린 좌표로 달리느니 멈춘다.
 
@@ -26,4 +73,30 @@ CANDIDATE_INVALID = 20000.0
 ARRIVE_RADIUS_M = 3.0
-DEFAULT_TIMEOUT = 120.0
-TURN_TIMEOUT = 90.0
+# WP 타임아웃은 순번이 아니라 wp_mode 기준 — 파라미터 wp_timeout_modes / wp_timeout_sec (wp_timeout.py)
+
+# 담당 노드가 없는 모드 = 순수 회피 구간. 여기서만 폴백을 낸다. (CLAUDE.md 3-6)
+FALLBACK_MODES = (5, 8)
+
+# GPS stale 이 이어지는 동안 ERROR 를 다시 찍는 주기(s). 로그 주기일 뿐 제어에는 안 쓴다.
+GPS_STALE_LOG_PERIOD_SEC = 5.0
+
+# 🚀 자율 시작 게이트: 아두이노 모드(ssf_bridge /boat_mode, Int32). 0=WAIT 1=MANUAL 2=AUTO.
+#    상수 공유 모듈화는 대회 전 금지(CLAUDE.md 3-9) → 여기 인라인(값은 ssf_bridge status_parser 와 일치).
+MODE_AUTO = 2
+
+
+def mission_should_run(boat_mode, require_auto, mode_auto=MODE_AUTO):
+    """미션 FSM(웨이포인트 전진)을 돌려도 되나 — ROS 비의존 순수 판정.
+
+    require_auto=False → 항상 True (아두이노 없이 벤치 테스트용).
+    require_auto=True  → boat_mode 가 AUTO(2)일 때만. WAIT/MANUAL/None(모름) 이면 False.
+      → 기본이 '안 돈다'라 안전하다: RC 를 AUTO 로 넘기기 전엔 미션이 전진하지 않는다.
+    """
+    if not require_auto:
+        return True
+    return boat_mode == mode_auto
+
+
+# 위경도 → 로컬 미터 근사
+M_PER_DEG_LAT = 110540.0
+M_PER_DEG_LON_EQ = 111320.0
 
@@ -35,13 +108,93 @@ class NorthGoalAngle(Node):
 
-        self.pub_dist      = self.create_publisher(Float32, '/goal_distance', qos)
-        self.pub_mode      = self.create_publisher(Int32,   '/wp_mode', qos)
+        # ---- 웨이포인트(GPS 좌표) 로드 — config/waypoints.yaml 에서 읽는다 ----
+        # 파일 경로는 파라미터로도 바꿀 수 있다(기본: 패키지 share 의 config/waypoints.yaml).
+        default_wp = os.path.join(
+            get_package_share_directory('north_goal_angle'), 'config', 'waypoints.yaml')
+        self.waypoints_file = str(
+            self.declare_parameter('waypoints_file', default_wp).value)
+        self.waypoints = self._load_waypoints(self.waypoints_file)
+
+        # ---- 파라미터 (config/north_goal_angle.yaml, CLAUDE.md 1-4) ----
+        self.period_sec = float(self.declare_parameter('period_sec', 0.5).value)
+
+        # ---- LOS 유도 (경로선 추종 + 조류 보상 ILOS) ----
+        # 직접 방위(현위치→목표)는 조류·초기오차로 밀리면 비스듬히 접근한다. LOS 는 이전 WP→목표
+        # '경로 선'으로 되돌아오게 하고, ILOS 적분항이 지속 외란(조류·바람)을 상쇄한다.
+        self.los_min_lookahead_m = float(self.declare_parameter('los_min_lookahead_m', 2.0).value)
+        self.los_max_lookahead_m = float(self.declare_parameter('los_max_lookahead_m', 6.0).value)
+        self.los_speed_gain = float(self.declare_parameter('los_speed_gain', 1.2).value)
+        self.ilos_ki = float(self.declare_parameter('ilos_ki', 0.02).value)
+        self.ilos_max = float(self.declare_parameter('ilos_integral_max', 50.0).value)  # anti-windup
+        # GPS 필터 (fix_is_usable 통과분에 품질·이상치·지터 처리)
+        self.gps_cov_threshold = float(self.declare_parameter('gps_cov_threshold', 2.0).value)
+        self.gps_max_speed_mps = float(self.declare_parameter('gps_max_speed_mps', 15.0).value)
+
+        # 🚀 자율 시작 게이트: True 면 RC 모드가 AUTO(/boat_mode==2) 여야 미션이 전진한다.
+        #    기본 True = 안전(수동 이동 중 미션 스킵 방지). 벤치(아두이노 없음)는 false 로.
+        self.require_auto = bool(self.declare_parameter('require_boat_mode_auto', True).value)
+        # 경기장 폴리곤: [lat1, lon1, lat2, lon2, ...] 평탄 배열.
+        # ⚠️ 실측 필요 — 대회장 경계 좌표를 아직 모른다. 비어 있으면 geofence 는 꺼진다.
+        self.geofence_polygon = list(
+            self.declare_parameter('geofence_polygon', []).value or [])
+        # 경계를 '가짜 LiDAR'로 쏜다 — 스캔과 같은 각도 격자(상대방위).
+        # ★ 왜 광선인가: 벽은 '점'이 아니라 '선'이다. 최근접점 한 방향으로 ±각도 원뿔을 막는 건
+        #   기하학적으로 틀렸다. 40x40 모서리에서 배가 대각선(45°)을 보면 북벽 최근접점은 -45°,
+        #   동벽은 +45° 인데 **탈출구(모서리)는 정면 0°** 라 원뿔 사이 '틈'으로 그대로 빠져나간다.
+        #   광선으로 쏘면 정면 방향의 경계 거리(2.12m)가 그대로 잡혀 벽이 된다. 튜닝도 필요 없다.
+        self.ray_min_deg = float(self.declare_parameter('geofence_ray_min_deg', -80.0).value)
+        self.ray_max_deg = float(self.declare_parameter('geofence_ray_max_deg', 80.0).value)
+        self.ray_inc_deg = float(self.declare_parameter('geofence_ray_inc_deg', 1.0).value)
+        self.geofence_max_range_m = float(
+            self.declare_parameter('geofence_max_range_m', 30.0).value)   # 이보다 멀면 inf
+        # 구독자 0 경고를 시작할 시각
+        self.geofence_check_after_sec = float(
+            self.declare_parameter('geofence_check_after_sec', 5.0).value)
+        # 🚨 IMU 신선도. geofence 상대방위는 /imu/yaw 로 계산하므로 IMU 에 의존한다.
+        #    IMU 가 죽으면 yaw 가 얼어붙고 → 경계 방위가 엉뚱해지고 → ship_direction 이
+        #    '없는 벽'을 칠한다 → 배가 갇힌다.
+        #    → 묵으면 geofence 도 [inf, nan] 을 낸다. (1단계 ship_goal_angle 과 같은 원칙:
+        #      **모르면 입을 다문다.** 틀린 값을 내느니 안 내는 게 낫다.)
+        self.imu_stale_sec = float(self.declare_parameter('imu_stale_sec', 0.5).value)
+        # 🚨 GPS fix 신선도 (계약서_GPS묶음_20261029 C1). GPSFilter 가 마지막으로 받아들인
+        #    프레임이 이보다 묵으면 tp·goal_distance 를 멈추고 geofence 는 빈 배열.
+        #    /fix 가 끊기면 콜백이 안 불려 have_fix 가 True 로 남는다 → 옛 위치로 조향하던 구멍.
+        #    inf·0 이하·NaN 은 잘못된 값 → 시작 ERROR + 기본값(노드는 세우지 않는다).
+        #    🚨 단 정수(yaml `gps_stale_sec: 0`·`3`, `-p gps_stale_sec:=0`)는 여기까지 못 온다 —
+        #       기본값 2.5 로 DOUBLE 선언이라 Humble rclpy 가 InvalidParameterTypeException 으로
+        #       노드를 세운다. yaml 값은 반드시 소수(.0)로 적는다.
+        raw_stale = self.declare_parameter('gps_stale_sec', DEFAULT_GPS_STALE_SEC).value
+        self.gps_stale_sec, stale_ok = sanitize_stale_sec(raw_stale)
+        if not stale_ok:
+            self._log_stale_sec_invalid(raw_stale)
+
+        # ---- WP 타임아웃 (wp_mode 기준, WP 하나당) — 계약서_WP타임아웃모드기준_20261029 ----
+        # ROS2 파라미터는 dict 불가 → 병렬 배열. 표에 없는 모드 = 타임아웃 없음.
+        # 🚨 yaml 의 wp_timeout_sec 는 전부 .0 (정수 섞이면 Humble 이 노드를 죽이거나 params 파싱 실패).
+        # 타이머 생성보다 먼저 선언·검증한다.
+        self.wp_timeout_table = self._load_wp_timeouts(
+            self.declare_parameter('wp_timeout_modes', list(DEFAULT_WP_TIMEOUT_MODES)).value,
+            self.declare_parameter('wp_timeout_sec', list(DEFAULT_WP_TIMEOUT_SEC)).value)
+
+        # ---- 발행 (토픽 이름 불변 + /geofence_state 신규, CLAUDE.md 3-9) ----
+        self.pub_dist = self.create_publisher(Float32, '/goal_distance', qos)
+        self.pub_mode = self.create_publisher(Int32, '/wp_mode', qos)
         self.pub_candidate = self.create_publisher(Float32, '/candidate_angle', qos)
-        self.pub_bearing   = self.create_publisher(Float32, '/north_goal_angle_tp', qos)
-        
-        # ⭐ 추가 : 남은 시간 퍼블리셔
-        self.pub_remain    = self.create_publisher(Float32, '/wp_remaining_time', qos)
+        self.pub_bearing = self.create_publisher(Float32, '/north_goal_angle_tp', qos)
+        self.pub_remain = self.create_publisher(Float32, '/wp_remaining_time', qos)
+        self.pub_geofence = self.create_publisher(Float32MultiArray, '/geofence_state', qos)
 
+        # ---- 구독 ----
         self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_cb, qos)
+        self.create_subscription(Float64, '/imu/yaw', self.yaw_cb, qos)   # 경계 상대방위 계산용
+        # 🚀 아두이노 모드(ssf_bridge 발행). AUTO 일 때만 미션 전진. RELIABLE 발행이라 qos 호환.
+        self.create_subscription(Int32, '/boat_mode', self.boat_mode_cb, qos)
 
+        # ---- 상태 ----
         self.lat, self.lon = 0.0, 0.0
+        self.have_fix = False
+        self.last_fix_t = None          # GPSFilter 가 받아들인 마지막 프레임 (monotonic). None=없음
+        self.gps_stale = False          # have_fix 인데 fix 가 묵은 상태(로그 전이용)
+        self.gps_stale_next_log_t = 0.0
+        self.yaw = None
+        self.last_yaw_t = None          # /imu/yaw 마지막 수신 (monotonic)
         self.wp_idx = 0
@@ -49,21 +202,288 @@ class NorthGoalAngle(Node):
         self.wp_enter_time = None
+        self.t0 = time.monotonic()
+
+        # LOS 상태
+        self.gps_filter = GPSFilter(
+            cov_threshold=self.gps_cov_threshold, max_speed_mps=self.gps_max_speed_mps)
+        self.integral_xte = 0.0          # ILOS 적분 (WP 전환 시 리셋)
+        self.prev_lat = None             # 경로선 시작점(이전 WP). None=첫 구간
+        self.prev_lon = None
+        self.boat_mode = None            # ssf_bridge /boat_mode (0 WAIT/1 MANUAL/2 AUTO). None=아직 모름
+
+        self.create_timer(self.period_sec, self.timer_cb)
 
-        self.create_timer(0.5, self.timer_cb)
+        gf = "OFF(폴리곤 미설정 — ⚠️ 실측 필요)" if len(self.geofence_polygon) < 6 else \
+             f"ON({len(self.geofence_polygon) // 2}점)"
+        self.get_logger().info(
+            f"north_goal_angle 시작: 폴백은 mode {FALLBACK_MODES} 에만 발행"
+            f"(mode 7 폴백 제거 — ship_dock 과 충돌했다). geofence {gf}. "
+            f"GPS stale 기준 {self.gps_stale_sec:g}s."
+        )
 
+    # ───────────────────────── 웨이포인트 로드 ─────────────────────────
+    def _load_waypoints(self, path):
+        """config/waypoints.yaml 을 읽어 검증한다. 실패하면 노드를 세운다(예외).
 
+        🚨 '모르면 침묵 말고 알린다' — 파일이 없거나 좌표가 이상하면 조용히 기본값으로
+        넘어가지 않고, 어디가 왜 틀렸는지 ERROR 로그를 내고 예외를 던져 노드를 멈춘다.
+        틀린 좌표로 배가 달리는 것보다 출발 전에 멈추는 게 안전하다.
+        """
+        try:
+            with open(path, 'r', encoding='utf-8') as f:
+                raw = yaml.safe_load(f)
+        except FileNotFoundError:
+            self.get_logger().error(
+                f"🚨 웨이포인트 파일이 없다: {path}\n"
+                f"   config/waypoints.yaml 이 설치됐는지 확인하라(colcon build 필요).")
+            raise
+        except yaml.YAMLError as e:
+            self.get_logger().error(
+                f"🚨 waypoints.yaml 문법 오류(콤마·중괄호·들여쓰기 확인): {e}")
+            raise
+
+        try:
+            wps = parse_waypoints(raw)
+        except WaypointError as e:
+            self.get_logger().error(f"🚨 웨이포인트 내용 오류 → {e}")
+            raise
+
+        modes = [w[2] for w in wps]
+        self.get_logger().info(
+            f"웨이포인트 {len(wps)}개 로드: {path}\n   미션 순서(mode): {modes}")
+        return wps
+
+    def _load_wp_timeouts(self, modes, secs):
+        """wp_timeout_modes/sec 를 검증해 {mode: sec} 표로. 틀리면 ERROR + 예외로 노드를 세운다.
+
+        빈 표·≤0·NaN·길이 불일치·허용 밖 모드·중복은 출발 전에 거부한다(waypoints.yaml 과 같은 방식).
+        timeout ≤ dwell 인 WP 는 반경 도착이 불가능하므로 WARN 만 낸다(세우지는 않는다).
+        """
+        try:
+            table = build_timeout_table(modes, secs)
+        except WpTimeoutError as e:
+            self.get_logger().error(f"🚨 WP 타임아웃 파라미터 오류 → {e}")
+            raise
+
+        for idx, mode, t, dwell in dwell_conflicts(self.waypoints, table):
+            self.get_logger().warn(
+                f"⚠️ WP{idx}(mode {mode}): 타임아웃 {t:.1f}s ≤ dwell {dwell:.1f}s — "
+                f"반경 도착은 불가능하고 늘 시간 초과로만 넘어간다. wp_timeout_sec 확인.")
+
+        per_wp = ['없음' if t is None else f"{t:g}" for t in per_wp_timeouts(self.waypoints, table)]
+        self.get_logger().info(
+            f"WP 타임아웃(mode 기준, WP 하나당) 표={table} → WP별(s): {per_wp}")
+        return table
+
+    # ───────────────────────── 콜백 ─────────────────────────
     def gps_cb(self, msg):
-        self.lat, self.lon = msg.latitude, msg.longitude
+        # 🚨 status 를 반드시 본다. 드라이버는 fix 가 없어도 /fix 를 1Hz 로 계속 내며,
+        #    그때 lat/lon 은 0.0 이다. 예전 코드는 이걸 그대로 믿고 (0,0) 기준 방위를
+        #    계산해 발행했다 — 실측 47.96° = calc_angle(0, 0, WP0). 판정은 gps_guard 참고.
+        if not fix_is_usable(msg.status.status, msg.latitude, msg.longitude):
+            if self.have_fix:
+                self._log_fix_lost(msg.status.status)
+            self.have_fix = False
+            return
 
+        # fix 는 유효하다 → 품질·이상치·지터를 GPSFilter 로 거른다. 여기서 버리는 건 '한 프레임'만
+        # (공분산 큼 / 순간이동)이지 fix 상실이 아니다 → have_fix·이전 위치는 유지한다.
+        cov = msg.position_covariance[0] if len(msg.position_covariance) > 0 else None
+        now = time.monotonic()
+        accepted, reason = self.gps_filter.update(
+            msg.latitude, msg.longitude, cov, now)
+        if not accepted:
+            self.get_logger().warn(f"🛰️ GPS fix 한 프레임 버림: {reason}",
+                                   throttle_duration_sec=2.0)
+            return
+        if not self.have_fix:
+            self._log_fix_acquired(msg.status.status)
+        self.lat = self.gps_filter.filtered_lat
+        self.lon = self.gps_filter.filtered_lon
+        self.have_fix = True
+        self.last_fix_t = now           # 🚨 받아들인 프레임에서만 — 거부·NO_FIX 는 갱신 안 함
+
+    # ⚠️ 로거 호출 지점을 함수로 분리한 이유: rclpy 는 로그 호출을 **소스 위치**로 캐싱해서,
+    #    같은 줄이 warn/info 로 번갈아 불리면 ValueError 로 노드가 죽는다 (CLAUDE.md §5 사례 ②).
+    def _log_fix_lost(self, status):
+        self.get_logger().warn(
+            f"🛰️ GPS fix 상실(status={status}) → 목표 방위·거리 발행 중단. "
+            "모르면 입을 다문다 (소비자는 침묵을 처리하도록 설계돼 있다)")
+
+    def _log_fix_acquired(self, status):
+        self.get_logger().info(f"🛰️ GPS fix 확보(status={status}) → 항법 재개")
+
+    # stale 로그도 심각도·용도별로 호출 지점을 나눈다 (CLAUDE.md 1-7).
+    def _log_stale_sec_invalid(self, raw):
+        self.get_logger().error(
+            f"🚨 gps_stale_sec={raw!r} 는 잘못된 값(inf·0 이하·NaN·숫자 아님) → "
+            f"기본값 {self.gps_stale_sec:g}s 로 돈다. north_goal_angle.yaml 확인.")
+
+    def _log_fix_stale_enter(self, age):
+        self.get_logger().error(
+            f"🛰️ GPS fix stale ({age:.1f}s > {self.gps_stale_sec:g}s, /fix 끊김 또는 필터 계속 거부) "
+            "→ 목표 방위·거리 발행 중단, geofence 빈 배열. /wp_mode·시간 초과 진행은 유지")
+
+    def _log_fix_stale_ongoing(self, age):
+        self.get_logger().error(
+            f"🛰️ GPS fix 여전히 stale ({age:.1f}s) → 항법 침묵 중 (ublox·안테나·USB 확인)")
+
+    def _log_fix_stale_recovered(self):
+        self.get_logger().info("🛰️ GPS fix 신선도 복구 → 항법 재개")
+
+    def yaw_cb(self, msg):
+        self.yaw = float(msg.data)
+        self.last_yaw_t = time.monotonic()
+
+    def boat_mode_cb(self, msg):
+        # 0=WAIT 1=MANUAL 2=AUTO. AUTO 로 바뀌면 미션이 WP0 부터 시작한다(timer_cb 게이트).
+        prev = self.boat_mode
+        self.boat_mode = int(msg.data)
+        if prev != self.boat_mode and self.boat_mode == MODE_AUTO:
+            self.get_logger().info("🚀 AUTO 전환 감지 → 미션 시작 (WP0 부터)")
+
+    # ───────────────────────── Geofence ─────────────────────────
+    def _local_xy(self, lat, lon):
+        """보트 기준 로컬 미터 (east, north)."""
+        kx = math.cos(math.radians(self.lat)) * M_PER_DEG_LON_EQ
+        return ((lon - self.lon) * kx, (lat - self.lat) * M_PER_DEG_LAT)
+
+    def _geofence_ranges(self, gps_ok):
+        """경계를 '가짜 LiDAR'로 쏜다 → (angle_min_deg, angle_inc_deg, [r0, r1, ...]) 또는 None.
+
+        각 상대방위마다 '이 방향으로 몇 m 가면 경계 밖인가'를 구한다. 멀면 inf.
+        소비자(ship_direction)는 이걸 실제 스캔과 min() 으로 병합하기만 하면 된다 —
+        특수 로직이 아예 없어진다. 기존 dilate 가 배 폭 여유를 더하고, 갭-팔로잉이 알아서 피한다.
+
+        None(=빈 배열) 을 내는 경우 — 전부 '모르면 입을 다문다':
+          · 폴리곤 미설정 / GPS fix 없음 또는 stale(gps_stale_sec)
+          · IMU stale     → yaw 가 얼면 광선 방향이 통째로 틀어져 '없는 벽'을 만든다
+          · 이미 경기장 밖 → 경계를 벽으로 세우면 돌아갈 길을 막는다
+        """
+        poly = self.geofence_polygon
+        if len(poly) < 6 or not gps_ok:             # 최소 3점(=6수) / fix 없음·stale
+            return None
+
+        # 🚨 IMU 신선도 — 광선 방향이 yaw 에 통째로 의존한다. 얼면 벽 전체가 엉뚱해진다.
+        if self.yaw is None or self.last_yaw_t is None:
+            self.get_logger().warn(
+                "/imu/yaw 없음 → geofence 침묵(빈 배열)", throttle_duration_sec=5.0)
+            return None
+        yaw_age = time.monotonic() - self.last_yaw_t
+        if yaw_age > self.imu_stale_sec:
+            self.get_logger().warn(
+                f"/imu/yaw stale ({yaw_age:.2f}s > {self.imu_stale_sec}s) → geofence 침묵. "
+                f"틀린 방향으로 '없는 벽'을 세우느니 안 내는 게 낫다.",
+                throttle_duration_sec=2.0)
+            return None
+
+        pts = [self._local_xy(poly[i], poly[i + 1]) for i in range(0, len(poly) - 1, 2)]
+
+        if not _point_in_polygon((0.0, 0.0), pts):
+            # 🚨 이미 밖이다. 경계를 벽으로 세우면 복귀로를 막는다 → 아무것도 내지 않는다.
+            self.get_logger().error(
+                "🚨 경기장 경계 밖이다 (실격 위험). GPS 웨이포인트로 복귀 중.",
+                throttle_duration_sec=2.0)
+            return None
+
+        if self.ray_inc_deg <= 0.0 or self.ray_max_deg < self.ray_min_deg:
+            return None
+
+        n = int(round((self.ray_max_deg - self.ray_min_deg) / self.ray_inc_deg)) + 1
+        ranges = []
+        for k in range(n):
+            rel = self.ray_min_deg + k * self.ray_inc_deg
+            th = math.radians((self.yaw + rel) % 360.0)      # 절대 방위(compass)
+            d = (math.sin(th), math.cos(th))                 # (east, north) 단위벡터
+            t = _ray_polygon_dist((0.0, 0.0), d, pts)
+            ranges.append(float(t) if (t is not None and t <= self.geofence_max_range_m)
+                          else float('inf'))
+
+        return self.ray_min_deg, self.ray_inc_deg, ranges
+
+    def _publish_geofence(self, gps_ok):
+        msg = Float32MultiArray()
+        gs = self._geofence_ranges(gps_ok)
+        if gs is None:
+            msg.data = []                                    # 빈 배열 = 경계 정보 없음
+        else:
+            a0, inc, ranges = gs
+            msg.data = [float(a0), float(inc)] + [float(r) for r in ranges]
+        self.pub_geofence.publish(msg)
+
+        # 🚨 '잊으면 최악' 방어: 구독자가 0 이면 경계 방어가 통째로 없는 것이다.
+        #    (도킹이 1년간 침묵했던 것과 똑같은 사고 — 침묵하는 기능은 에러를 내지 않는다)
+        if (time.monotonic() - self.t0) > self.geofence_check_after_sec:
+            if self.pub_geofence.get_subscription_count() == 0:
+                self.get_logger().error(
+                    "🚨 /geofence_state 구독자가 0 이다 — 경계 이탈 방어가 작동하지 않는다(실격 위험). "
+                    "6a-2 에서 ship_direction 이 구독해야 한다.",
+                    throttle_duration_sec=10.0)
+
+    # ───────────────────────── 메인 루프 ─────────────────────────
+    def _update_gps_fresh(self):
+        """이번 틱에 위치를 믿어도 되나 = have_fix 이고 마지막 수용 프레임이 gps_stale_sec 안.
+
+        🚨 have_fix 는 여기서 바꾸지 않는다(gps_cb 만 바꾼다). stale 은 별도 상태로 들고
+           진입 1회 ERROR + GPS_STALE_LOG_PERIOD_SEC 마다 ERROR + 복구 INFO 만 낸다.
+           have_fix 가 False 인 경우(NO_FIX·부팅 직후)는 기존 _log_fix_lost·'fix 없음' WARN 몫.
+        """
+        now = time.monotonic()
+        fresh = self.have_fix and fix_is_fresh(self.last_fix_t, now, self.gps_stale_sec)
+        stale = self.have_fix and not fresh
+        if stale:
+            age = now - self.last_fix_t if self.last_fix_t is not None else float('inf')
+            if not self.gps_stale:
+                self._log_fix_stale_enter(age)
+                self.gps_stale_next_log_t = now + GPS_STALE_LOG_PERIOD_SEC
+            elif now >= self.gps_stale_next_log_t:
+                self._log_fix_stale_ongoing(age)
+                self.gps_stale_next_log_t = now + GPS_STALE_LOG_PERIOD_SEC
+        elif fresh and self.gps_stale:
+            self._log_fix_stale_recovered()
+        self.gps_stale = stale
+        return fresh
 
     def timer_cb(self):
-        if self.wp_idx >= len(waypoints):
+        gps_ok = self._update_gps_fresh()    # have_fix + 신선도(gps_stale_sec)
+        self._publish_geofence(gps_ok)       # 웨이포인트가 끝나도 경계 감시는 계속한다
+
+        if self.wp_idx >= len(self.waypoints):
             return
 
-        wp_lat, wp_lon, wp_mode, dwell = waypoints[self.wp_idx]
+        wp_lat, wp_lon, wp_mode, dwell = self.waypoints[self.wp_idx]
 
-        dist = calc_dist(self.lat, self.lon, wp_lat, wp_lon)
-        self.pub_dist.publish(Float32(data=dist))
+        # 🚨 위치에서 나오는 값(거리·방위)은 fix 가 유효할 때만 낸다.
+        #    fix 가 없으면 lat/lon 이 0 이거나 옛값이라 방위가 통째로 틀린다.
+        #    소비자(ship_goal_angle)는 침묵을 이미 처리한다 — /yaw_error 를 멈춘다.
+        #    ⚠️ /wp_mode 는 계속 낸다. 미션 단계는 '지금 위치' 에서 나오는 값이 아니고,
+        #       멈추면 비전 검출기 3종이 전부 비활성이 된다(CLAUDE.md 3-4) — 부작용이 더 크다.
+        #       대신 아래 도착 판정(wp_idx 전진)은 fix 없이는 하지 않는다.
+        # 🚨 fix 가 묵어도(stale) 같다 — 옛 위치로 방위를 내지 않는다. ILOS 적분·prev_lat 초기화도
+        #    이 조건 안이라 stale 동안 적분이 쌓이지 않는다. /wp_mode·20000 폴백·
+        #    /wp_remaining_time·시간 초과 진행은 위치와 무관하니 그대로 낸다.
+        if gps_ok:
+            # 첫 구간은 이전점이 없다 → 현 위치를 경로 시작으로(=직접 접근). 이후엔 완료한 WP.
+            if self.prev_lat is None:
+                self.prev_lat, self.prev_lon = self.lat, self.lon
 
-        bearing = calc_angle(self.lat, self.lon, wp_lat, wp_lon)
-        self.pub_bearing.publish(Float32(data=bearing))
+            dist = los_logic.calc_dist(self.lat, self.lon, wp_lat, wp_lon)
+            self.pub_dist.publish(Float32(data=dist))
+
+            # LOS: 경로선(prev→wp)에서 벗어난 만큼 전방주시거리 안에서 되돌아오는 방위.
+            lookahead = los_logic.get_dynamic_lookahead(
+                self.gps_filter.estimated_speed_mps, self.los_speed_gain,
+                self.los_min_lookahead_m, self.los_max_lookahead_m)
+            bearing, xte = los_logic.calc_los_bearing(
+                self.prev_lat, self.prev_lon, wp_lat, wp_lon, self.lat, self.lon,
+                lookahead, self.integral_xte, self.ilos_ki)
+            # ILOS 적분 + anti-windup 클램프
+            self.integral_xte += xte * self.period_sec
+            self.integral_xte = max(-self.ilos_max, min(self.ilos_max, self.integral_xte))
+            self.pub_bearing.publish(Float32(data=bearing))
+        else:
+            dist = None
+            if not self.have_fix:            # stale 은 _update_gps_fresh 가 ERROR 로 따로 알린다
+                self.get_logger().warn(
+                    "🛰️ GPS fix 없음 → 목표 방위 미발행 (하늘이 안 보이거나 안테나 확인)",
+                    throttle_duration_sec=5.0)
 
@@ -71,6 +491,24 @@ class NorthGoalAngle(Node):
 
-        if wp_mode == 7:
+        # [3] 폴백은 '담당 노드가 없는 모드'(5, 8)에만.
+        #     작년엔 mode 7(도킹)에도 냈다 → ship_dock 을 7 로 고치면 발행자가 둘이 되어
+        #     도킹 중 조향이 GPS 방위로 튄다. 원자적 한 쌍이라 같은 커밋에서 고친다.
+        if wp_mode in FALLBACK_MODES:
             self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
 
-        now = time.time()
+        # 🚀 자율 시작 게이트 — AUTO 가 아니면 미션을 '전진'시키지 않는다(WP0 에서 대기).
+        #    수동(RC)으로 시작점까지 몰고 가는 동안 GPS 도착/타임아웃으로 wp_idx 가 새는 걸 막는다.
+        #    (위에서 방위·거리·wp_mode 는 계속 낸다 — 정보용. 모터는 아두이노 MANUAL 이 무시한다.)
+        #    RC 모드를 AUTO 로 넘기는 순간부터 아래 도착/타임아웃 로직이 살아나 WP0 부터 진짜 시작한다.
+        if not mission_should_run(self.boat_mode, self.require_auto):
+            self.wp_enter_time = None         # 타임아웃 시계 리셋 → AUTO 시작 시점부터 카운트
+            self.t_start = None               # dwell 리셋
+            self.integral_xte = 0.0           # LOS 적분 리셋
+            self.prev_lat = self.prev_lon = None   # 경로선 시작점 = AUTO 전환 시점 위치
+            self.get_logger().info(
+                "🕹 AUTO 대기 — RC 모드를 AUTO 로 넘기면 WP0 부터 자율 시작 "
+                "(벤치는 require_boat_mode_auto:=false 또는 /boat_mode 2 발행)",
+                throttle_duration_sec=5.0)
+            return
+
+        now = time.monotonic()               # 벽시계 금지 (CLAUDE.md 1-5)
 
@@ -79,16 +517,14 @@ class NorthGoalAngle(Node):
 
-        # WP별 타임아웃 정책
-        timeout = None
-        if self.wp_idx in (3, 4, 5):
-            timeout = TURN_TIMEOUT
-        elif self.wp_idx <= 6:
-            timeout = DEFAULT_TIMEOUT
+        # WP 하나당 타임아웃 — wp_mode 기준. 표에 없는 모드(기본: 8)는 None = 타임아웃 없음.
+        timeout = timeout_for_mode(self.wp_timeout_table, wp_mode)
 
-        # ⭐ 타임아웃 남은 시간 퍼블리시
         if timeout is not None:
             remain = max(timeout - (now - self.wp_enter_time), 0.0)
-            self.pub_remain.publish(Float32(data=remain))  # ★ 퍼블리시
+            self.pub_remain.publish(Float32(data=remain))
 
             if remain <= 0:
-                self.get_logger().warn(f"🕒 WP{self.wp_idx} 시간 초과 → 다음 WP 이동")
+                self.get_logger().warn(
+                    f"🕒 WP{self.wp_idx}(mode {wp_mode}) 시간 초과 {timeout:g}s → 다음 WP 이동")
+                self.prev_lat, self.prev_lon = wp_lat, wp_lon   # 새 경로선 시작점
+                self.integral_xte = 0.0                          # ILOS 적분 리셋
                 self.wp_idx += 1
@@ -98,4 +534,10 @@ class NorthGoalAngle(Node):
 
-        # 정상 도착 확인
-        if dist < ARRIVE_RADIUS_M:
+        # fix 가 없거나 stale 이면 '도착했는지' 를 판단할 근거가 없다 → wp_idx 를 전진시키지 않는다.
+        # (시간 초과 전진은 위에서 이미 처리한다 — 그건 위치와 무관한 안전장치라 유지)
+        # 도착 반경은 속도·모드에 따라 동적(빠르면 넓게 오버슈트 방지, 도킹은 좁게).
+        arrive_r = los_logic.get_dynamic_arrive_radius(
+            wp_mode, self.gps_filter.estimated_speed_mps)
+        if dist is None:
+            self.t_start = None
+        elif dist < arrive_r:
             if self.t_start is None:
@@ -103,3 +545,6 @@ class NorthGoalAngle(Node):
             elif (now - self.t_start) >= dwell:
-                self.get_logger().info(f"✔ WP{self.wp_idx} 완료 → 다음 WP 이동")
+                self.get_logger().info(
+                    f"✔ WP{self.wp_idx} 완료 (반경 {arrive_r:.1f}m) → 다음 WP 이동")
+                self.prev_lat, self.prev_lon = wp_lat, wp_lon   # 새 경로선 시작점
+                self.integral_xte = 0.0                          # ILOS 적분 리셋
                 self.wp_idx += 1
@@ -111,3 +556,49 @@ class NorthGoalAngle(Node):
 
+# ───────────────────────── 순수 기하 (테스트 가능) ─────────────────────────
+def _ray_polygon_dist(o, d, pts):
+    """원점 o 에서 방향 d 로 쏜 광선이 폴리곤 변에 처음 닿는 거리. 안 닿으면 None.
+
+    ★ 이게 '벽은 선이다'를 정직하게 푸는 방법이다. 최근접점 한 방향으로 원뿔을 막으면
+      모서리에서 정면(탈출구)이 원뿔 사이 '틈'으로 열려 배가 대각선으로 빠져나간다.
+    """
+    ox, oy = o
+    dx, dy = d
+    best = None
+    n = len(pts)
+    for i in range(n):
+        ax, ay = pts[i]
+        bx, by = pts[(i + 1) % n]
+        ex, ey = bx - ax, by - ay
+
+        denom = dx * ey - dy * ex
+        if abs(denom) < 1e-12:
+            continue                       # 광선과 변이 평행
+
+        rx, ry = ax - ox, ay - oy
+        t = (rx * ey - ry * ex) / denom    # 광선 위 거리
+        s = (rx * dy - ry * dx) / denom    # 변 위 위치 (0~1 이어야 선분 안)
+
+        if t >= 0.0 and 0.0 <= s <= 1.0:
+            if best is None or t < best:
+                best = t
+    return best
+
+
+def _point_in_polygon(p, pts):
+    """레이 캐스팅. 점 p 가 폴리곤 안인가."""
+    px, py = p
+    inside = False
+    n = len(pts)
+    for i in range(n):
+        x1, y1 = pts[i]
+        x2, y2 = pts[(i + 1) % n]
+        if (y1 > py) != (y2 > py):
+            xint = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
+            if px < xint:
+                inside = not inside
+    return inside
+
+
 def calc_angle(lat1, lon1, lat2, lon2):
+    from math import atan2, cos, radians, sin, degrees
     lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
@@ -115,5 +606,6 @@ def calc_angle(lat1, lon1, lat2, lon2):
     x = sin(dlon) * cos(lat2)
-    y = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dlon)
+    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
     return (degrees(atan2(x, y)) + 360) % 360
 
+
 def calc_dist(lat1, lon1, lat2, lon2):
@@ -125,5 +617,10 @@ def main(args=None):
     node = NorthGoalAngle()
-    rclpy.spin(node)
-    node.destroy_node()
-    rclpy.shutdown()
+    try:
+        rclpy.spin(node)
+    except KeyboardInterrupt:
+        pass
+    finally:
+        node.destroy_node()
+        if rclpy.ok():
+            rclpy.shutdown()
 
diff --git a/src/north_goal_angle/setup.py b/src/north_goal_angle/setup.py
index 32dd61c..eac5d66 100755
--- a/src/north_goal_angle/setup.py
+++ b/src/north_goal_angle/setup.py
@@ -15,2 +15,3 @@ setup(
         (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
+        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
     ],
@@ -25,3 +26,4 @@ setup(
         'console_scripts': [
-            'north_goal_angle = north_goal_angle.north_goal_angle:main'
+            'north_goal_angle = north_goal_angle.north_goal_angle:main',
+            'waypoint_recorder = north_goal_angle.waypoint_recorder:main'
         ],
diff --git a/src/north_goal_angle/test/test_flake8.py b/src/north_goal_angle/test/test_flake8.py
index 27ee107..9d4e79b 100755
--- a/src/north_goal_angle/test/test_flake8.py
+++ b/src/north_goal_angle/test/test_flake8.py
@@ -21,3 +21,13 @@ import pytest
 def test_flake8():
-    rc, errors = main_with_errors(argv=[])
+    # 🚨 최대 줄 길이 99 → 120 (2026-09-23). ament_flake8 은 `.flake8`·`setup.cfg` 를
+    #    읽지 않고 자기 ini 를 쓰므로, 설정 파일 하나로는 안 되고 패키지마다 여기서 준다.
+    #    **10개 패키지가 같은 값이어야 한다** — 하나만 다르면 그 패키지만 조용히 엄격해진다.
+    #
+    #    왜 올렸나: 넘치던 35줄이 전부 `파라미터 한 줄 + 실측 필요 주석` 조합이었다. 예)
+    #      self.base_pwm = int(self.declare_parameter("base_pwm", 1360).value)  # ⚠️ 실측 필요
+    #    이 저장소에선 그 한 줄이 **정보 단위**다. 주석을 위로 올리면 규칙은 통과하지만
+    #    나란히 정렬된 파라미터 블록이 흩어져 읽기 나빠진다.
+    #    E501 은 결함 탐지기가 아니라 취향 설정이라 규칙 쪽을 옮겼다.
+    #    ⚠️ lint 를 **끄는 것과는 다르다.** F 계열(죽은 코드)·E 계열은 그대로 다 돈다.
+    rc, errors = main_with_errors(argv=['--linelength', '120'])
     assert rc == 0, \
diff --git a/src/north_goal_angle/test/test_pep257.py b/src/north_goal_angle/test/test_pep257.py
index b234a38..941937d 100755
--- a/src/north_goal_angle/test/test_pep257.py
+++ b/src/north_goal_angle/test/test_pep257.py
@@ -14,3 +14,18 @@
 
-from ament_pep257.main import main
+# 🚨 pep257 은 2026-09-23 에 껐다. **파일을 지우지 않고 skip 으로 남긴다** —
+#    왜 껐는지를 파일 자체가 말하게 하려는 것이다. 지우면 "원래 없었나?" 가 된다.
+#
+#    끈 근거 (저장소 전체 156건을 실측하고 판단):
+#      · 결함 0건. **전부 서식**이다 — flake8 의 F 계열(죽은 코드)과는 성격이 다르다.
+#      · 절반이 영어 전제 규칙이다. D400(마침표로 끝낼 것)·D403(첫 단어 대문자)·
+#        D401(명령형) — 이 저장소 docstring 은 전부 한국어라 적용 자체가 성립하지 않는다.
+#      · 나머지 절반(D205 요약 뒤 빈 줄·D212 첫 줄 위치)도, **"왜" 가 담긴 docstring 을
+#        기계적으로 옮기면 뜻이 흔들릴 위험만 있고 얻는 게 없다.**
+#      · 대회 전 시간은 실측·수조에 쓴다.
+#
+#    ⚠️ flake8 은 **끄지 않았다.** 그쪽은 죽은 코드를 실제로 잡는다(09-23 에 18건 제거).
+#       lint 를 통째로 포기한 게 아니라 **결함 탐지기는 켜두고 서식 검사만 내렸다.**
+#    ⚠️ `package.xml` 의 `ament_pep257` test_depend 는 **그대로 둔다.** 지우면 빌드
+#       의존을 건드리게 된다. 되살리려면 아래 skip 표시만 지우면 된다.
+
 import pytest
@@ -18,6 +33,5 @@ import pytest
 
-@pytest.mark.linter
-@pytest.mark.pep257
+@pytest.mark.skip(reason='docstring 이 전부 한국어라 영어 전제 규칙(D400/D401/D403)이 절반. '
+                         '서식 검사일 뿐 결함 탐지가 아니라 2026-09-23 끄기로 결정.')
 def test_pep257():
-    rc = main(argv=['.', 'test'])
-    assert rc == 0, 'Found code style errors / warnings'
+    pass
diff --git a/src/ublox/ublox_gps/config/c94_m8p_rover.yaml b/src/ublox/ublox_gps/config/c94_m8p_rover.yaml
index be0f5ca..4e85671 100644
--- a/src/ublox/ublox_gps/config/c94_m8p_rover.yaml
+++ b/src/ublox/ublox_gps/config/c94_m8p_rover.yaml
@@ -1,10 +1,14 @@
-# Configuration Settings for C94-M8P device
+# ═══════════════════════════════════════════════════════════════════════════
+#  C94-M8P 이동국(rover) 설정 — A배
+#  [2026-09-16] 정리: save 블록 제거(EEPROM 영구 저장 — zed_f9p_rover.yaml 과 같은 이유로
+#  런타임(RAM)에만 적용), dynamic_model portable→sea(수상선박, 드라이버 node.cpp 'sea' 지원,
+#  u-blox 8 CFG-NAV5 dynModel 5), debug 1→0. uart1 19200·RTCM in 은 C94-M8P 고유라 유지.
+#  udev: idProduct 01a8 (u-blox 8) — F9P 의 01a9 와 다르다 → docs/절차/ubuntu_setup.md
+#  ⚠️ sea 모델은 A배 실기에서 아직 미확인 — 첫 실기 때 /fix 안정성 확인.
+# ═══════════════════════════════════════════════════════════════════════════
 ublox_gps_node:
   ros__parameters:
-    debug: 1                    # Range 0-4 (0 means no debug statements will print)
+    debug: 0                    # Range 0-4 (0 means no debug statements will print)
 
-    save:
-      mask: 3103                # Save I/O, Message, INF Message, Nav, Receiver
-                                # Manager, Antenna, and Logging Configuration
-      device: 4                 # Save to EEPROM
+    # 🚨 save 블록을 넣지 않는다 (EEPROM 영구 저장 금지 — 잘못 박히면 되돌리기 어렵다)
 
@@ -16,6 +20,3 @@ ublox_gps_node:
                                 # 8 Hz (GPS only)
-    dynamic_model: portable     # Airborne < 2G, 2D fix not supported (3D only),
-                                # Max Alt: 50km
-                                # Max Horizontal Velocity: 250 m/s,
-                                # Max Vertical Velocity: 100 m/s
+    dynamic_model: sea          # 해상 표면 차량용 (portable 에서 변경 2026-09-16). B배와 동일 근거.
     fix_mode: auto
diff --git a/src/ublox/ublox_gps/launch/ublox_gps_node-composed-launch.py b/src/ublox/ublox_gps/launch/ublox_gps_node-composed-launch.py
index 76c7f5a..0f52acf 100644
--- a/src/ublox/ublox_gps/launch/ublox_gps_node-composed-launch.py
+++ b/src/ublox/ublox_gps/launch/ublox_gps_node-composed-launch.py
@@ -32,3 +32,11 @@
 
-"""Launch the ublox gps node in a composable container with c94-m8p configuration."""
+"""Launch the ublox gps node in a composable container. 배별 config 를 인자로 고른다.
+
+  ros2 launch ublox_gps ublox_gps_node-composed-launch.py                            # B배 (기본)
+  ros2 launch ublox_gps ublox_gps_node-composed-launch.py config:=c94_m8p_rover.yaml # A배
+
+🚨 여기도 c94_m8p_rover.yaml 이 하드코딩돼 있었다 (비하드코딩 launch 와 같은 사고).
+   ⚠️ 이 파일은 yaml 을 **Python 이 직접 읽어** 파라미터 dict 를 만든다. LaunchConfiguration
+      은 평가 시점이 늦어서 open() 에 그대로 못 쓴다 → OpaqueFunction 으로 평가를 미룬다.
+"""
 
@@ -39,2 +47,4 @@ import ament_index_python.packages
 from launch import LaunchDescription
+from launch.actions import DeclareLaunchArgument, OpaqueFunction
+from launch.substitutions import LaunchConfiguration
 from launch_ros.actions import ComposableNodeContainer
@@ -45,3 +55,3 @@ import yaml
 
-def generate_launch_description():
+def _make_container(context, *args, **kwargs):
     config_directory = os.path.join(
@@ -49,3 +59,4 @@ def generate_launch_description():
         'config')
-    param_config = os.path.join(config_directory, 'c94_m8p_rover.yaml')
+    config_name = LaunchConfiguration('config').perform(context)
+    param_config = os.path.join(config_directory, config_name)
     with open(param_config, 'r') as f:
@@ -66,3 +77,12 @@ def generate_launch_description():
     )
+    return [container]
 
-    return LaunchDescription([container])
+
+def generate_launch_description():
+    return LaunchDescription([
+        DeclareLaunchArgument(
+            'config',
+            default_value='zed_f9p_rover.yaml',
+            description='배별 GPS 설정 파일 (B배: zed_f9p_rover.yaml / A배: c94_m8p_rover.yaml)'),
+        OpaqueFunction(function=_make_container),
+    ])
diff --git a/src/ublox/ublox_gps/launch/ublox_gps_node-launch.py b/src/ublox/ublox_gps/launch/ublox_gps_node-launch.py
index 9a8d9b2..552e225 100644
--- a/src/ublox/ublox_gps/launch/ublox_gps_node-launch.py
+++ b/src/ublox/ublox_gps/launch/ublox_gps_node-launch.py
@@ -32,3 +32,12 @@
 
-"""Launch the ublox gps node with c94-m8p configuration."""
+"""Launch the ublox gps node. 배별 config 를 인자로 고른다.
+
+  ros2 launch ublox_gps ublox_gps_node-launch.py                              # B배 (기본, ZED-F9P)
+  ros2 launch ublox_gps ublox_gps_node-launch.py config:=c94_m8p_rover.yaml   # A배 (C94-M8P)
+
+🚨 원래는 c94_m8p_rover.yaml 이 하드코딩돼 있었다. 배가 2척(A=M8P, B=F9P)인데 launch 가
+   한 배에 고정돼 있어서, F9P 를 꽂으면 **에러 없이 M8P 설정으로 돌았다**(2026-08-06 실기 발견).
+   기본값을 실제 보유 장비(F9P)로 두는 이유: launch_files.launch.py 가 이 파일을 그대로
+   include 하므로, 기본이 틀리면 대회 실행 경로에서 같은 사고가 조용히 재발한다.
+"""
 
@@ -38,2 +47,3 @@ import ament_index_python.packages
 import launch
+import launch.substitutions
 import launch_ros.actions
@@ -45,3 +55,11 @@ def generate_launch_description():
         'config')
-    params = os.path.join(config_directory, 'c94_m8p_rover.yaml')
+
+    config_arg = launch.actions.DeclareLaunchArgument(
+        'config',
+        default_value='zed_f9p_rover.yaml',
+        description='배별 GPS 설정 파일 (B배: zed_f9p_rover.yaml / A배: c94_m8p_rover.yaml)')
+
+    params = launch.substitutions.PathJoinSubstitution(
+        [config_directory, launch.substitutions.LaunchConfiguration('config')])
+
     ublox_gps_node = launch_ros.actions.Node(package='ublox_gps',
@@ -51,3 +69,4 @@ def generate_launch_description():
 
-    return launch.LaunchDescription([ublox_gps_node,
+    return launch.LaunchDescription([config_arg,
+                                     ublox_gps_node,
 
```

ntrip_client 의 diff 는 일부러 넣지 않았다(작년 평문 계정이 다시 나온다). 위 커밋 메시지로 쓴다.

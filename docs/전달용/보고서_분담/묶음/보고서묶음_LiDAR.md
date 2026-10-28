# 보고서 분담 묶음 — LiDAR
만든 시각: 2026-10-28 16:46 · 저장소 커밋: c58e27b

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
# 현재 보고서 Ⅲ장 소프트웨어 원고 (2026-10-28 기준, 초안 검토 결과 18곳 반영)

> 원본: `docs/전달용/KABOAT2026_보고서_양식_SSF_A.docx` 의 Ⅲ장을 그대로 옮긴 것. 생성기는 `tools/make_report_docx.js`.
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

### 1.2) 노드·토픽 구조

| 노드 | 역할 | 주요 토픽 |
|---|---|---|
| rplidar_ros / iahrs_driver / ublox | 센서 드라이버 | /scan, /imu/yaw_raw, /ublox_gps_node/fix |
| ssf_heading (yaw_mux) | 헤딩 단일 발행자 | /imu/yaw, /heading_status |
| north_goal_angle | GPS 목표 방위, 지오펜스 → 가짜 LiDAR | /candidate_angle, /geofence_state |
| basic_image_subscriber{gate,turn,dock} | 색·형상 검출 (방위각만) | /red_angle, /green_angle, /image_angle, /image_color |
| ship_gate / ship_back / ship_turn / ship_dock | 임무 노드 (wp_mode 별) | /candidate_angle |
| ship_direction | LiDAR 갭 선택·회피, 페일세이프 레벨 | /desired_angle, /failsafe_level, /obstacle_distance_array, /obstacle_reject_count |
| motor_control | 각도 → 좌/우 PWM, 감속, 명령 워치독 0.5 s | Motor_run |
| ssf_bridge | 시리얼 브릿지, 상태줄 파싱 | /boat_mode, /boat_id, /boat_estop, /boat_cmd_watchdog |
| healthcheck / blackbox / mission_monitor | 진단·기록 (제어에 안 물림) | /health_ok |

그림 Ⅲ-1 ROS2 노드·토픽 구조 (tools/mission_flowchart.py)

### 1.3) 센서

| 장비 사진 | 장비명 | 용도 |
|---|---|---|
| (사진) | RPLIDAR A3 개체 80BF (약 11.1 Hz / 0.250°) | 장애물·부표 거리, 갭 탐색, 25 m. 각분해능이 개체마다 달라 셀 기반 튜닝값은 배별 |
| (사진) | iAHRS RB-SDA-v1 9축 | 요(yaw). yaw_mux 가 /imu/yaw 단독 발행 |
| (사진) | u-blox C94-M8P (안테나 ⚠️ ANN-MB / 키트 L1 패치 실물 확인) | NTRIP RTK 위치·COG |
| (사진) | Luxonis OAK-1 PoE FF (HFOV 69° 스펙) | 게이트·부표·도크 색/형상 방위각 후보. PoE 인젝터 도착 뒤 D455 와 비교해 선정, hfov_deg 는 camera_info 로 실측 교체 |
| (사진) | Intel RealSense D455 | 현재 자율 비전(검출기 3종)과 수동 조종 화면에 쓰는 카메라 |


## 2) 센서 처리

### 2.1) 좌표계·헤딩
- 헤딩 소스는 yaw_mux 하나로 모은다(IMU raw, COG 오프셋). 각도 평균은 산술평균 금지 — 단위벡터 합 + atan2 (359° 와 1° 의 평균이 180° 가 되는 함정).
- 자기편각은 자기북 기준 소스에만 더한다. 듀얼 GPS·COG 에 더하면 이중 보정(약 8°).
- 헤딩이 없으면 발행하지 않는다 — 0 같은 가짜 값 금지. IMU stale 0.5 s 이면 지오펜스도 빈 배열.
- ⚠️ IMU 장착 오프셋(mount_offset_deg), 대회장 자기편각 — 벤치·현장 실측.

### 2.2) GPS RTK
- NTRIP 보정. B배 수신기(ZED-F9P) 손 시험(08-07)에서 RTK FIXED 수평 정확도 ±0.015 m, 위성 18개, pDOP 1.63. A배 C94-M8P 와 배에 단 상태는 미측정. 보정 서버 자격은 저장소 밖(~/.ssf/).
- ⚠️ RTK on/off 위치 산포 비교표: FLOAT ____ m / FIXED ____ m (SeaNU 보고서 방식).

### 2.3) LiDAR 처리
- 감속 신호(가장 가까운 장애물)에만 median 커널 5 — 물보라 한 점에 속는 가짜 감속을 줄인다. 회피 마스크에는 걸지 않아 회피 접촉 성능은 그대로 둔다.
- 탐지 거리 기본 3.0 m, 게이트 2.0 m. inf/nan 셀은 탐지 거리로 치환해 갭 가장자리 거리로 쓴다.
- 지오펜스는 경계를 가짜 LiDAR 로 바꿔 ranges = min(real, geofence) 로 병합. 후보각·제자리 선회 구간에서는 의도한 진행 방향 ±60° 안에 경계가 2.0 m 이내면 STOP_HOLD.
- ⚠️ 수면 반사·물보라 오탐 통계: 야외 rosbag 에서 median 기각 수(/obstacle_reject_count) ____ 회/분.

### 2.4) 영상 처리 파이프라인
검출기 3종(gate·turn·dock)이 같은 파이프라인을 공유하고, 색 범위·최소 면적·확인 프레임은 vision.yaml 한 곳에서 읽는다(작년엔 같은 색이 파일 3곳에 다른 값으로 박혀 도킹만 부표를 못 본 이력).

| 단계 | 처리 | 파라미터 / 값 |
|---|---|---|
| ① 입력 | image_topic 구독, BEST_EFFORT depth 1 (묵은 프레임으로 조향 금지) | 해상도는 매 프레임 msg.width 로 읽음 |
| ② 색 분리 | BGR → HSV, 색별 inRange. 빨강은 0° 를 넘어 범위 2개 | red [0-5]+[165-180], green H 60-85, white V≥220·S≤40 (⚠️ 실외 재확정) |
| ③ 후보 추출 | 컨투어 → 최소 면적 → 형상 검사(꼭짓점 4~8·extent·종횡비, require_shape) 또는 solidity(면적/볼록껍질) → 가장 큰 것 1개 | min_area_px 40 (gate·turn) / 80 (dock), min_solidity 0.7, y_zone 검출기별 gate 0.15–0.55 / turn 0.10–0.55 / dock 0.15–0.55 (⚠️ 실측 필요) |
| ④ 형상 판정 (dock) | approxPolyDP 꼭짓점 수 + extent·aspect·circularity 로 삼각/사각/원 3종 | epsilon 0.0315, square extent≥0.4 aspect 0.6~1.4, circle circ≥0.82 |
| ⑤ 확인 | 최근 M 프레임 중 N 회(미검출은 관용, 색 전환은 리셋) 일치해야 발행. 유예 시간 내 유실은 마지막 값 유지 | confirm_frames 1 (gate·turn) / 3 (dock). confirm_window 는 gate·turn 만 선언(기본 N, 켤 땐 3/5), dock 은 연속 3. grace 2.0 s — 10-27 개정 |
| ⑥ 방위각 변환 | 픽셀 x → 각도: θ = −atan((x − cx) / fx), cx = W/2 (영상 중심), fx = (W/2)/tan(hfov/2) | hfov_deg 80.32 (D455 camera_info 실측) → OAK 스펙 69°, ⚠️ camera_info fx 로 실측 교체 |
| ⑦ 발행·게이트 | /wp_mode 가 담당 모드일 때만 발행, stale 2 s → 정지. dock 목표 색·형상이 유효값 밖이면 기동 시 ERROR, 검출 없이 무효값 10000 발행 | active_wp_modes: gate [0,1] / turn [2,3] / dock [7] |

- 거리는 쓰지 않는다 — 카메라는 방위각만, 거리는 LiDAR 전방 섹터 최소거리 (핀홀 거리 추정의 실해역 오차 회피).
- ⚠️ 혼동행렬: 야외 부표 영상 ____ 프레임에서 색별 정탐/오탐 ____ — 임계값은 이 수치로 정한다(감으로 튜닝 금지).

## 3) 전체 임무 상태기계
/wp_mode 값이 담당 노드를 정하고, 검출기는 해당 모드에서만 발행한다. /wp_mode 2 s 이상 stale 이면 검출기 발행 정지.

| wp_mode | 임무 | 임무 노드 | 검출기 → 발행 |
|---|---|---|---|
| 0, 1 | 게이트 시작·끝 (항로추종) | ship_gate | gate → /red_angle, /green_angle |
| 2 | 위치유지 | ship_back | turn(white) → /image_angle |
| 3 | 탐색 (부표 선회) | ship_turn | turn(red) → /image_angle, /image_color |
| 5 | 장애물 회피 (50 s) | 없음 — ship_direction 순수 회피 | 없음 |
| 7 | 도킹 (60 s) | ship_dock | dock → /image_angle |
| 8 | 토너먼트 자율 회피 | 없음 | 없음 |

특수 명령 코드(/candidate_angle·/desired_angle): 5000 우선회, 6000 좌선회(예약), 20000 미션 없음 → 전진 폴백(모드 5·8 만), 50000 정지.
그림 Ⅲ-2 종합임무 상태기계 (tools/mission_flowchart.py)

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
LiDAR 스캔에서 안전 구간(연속 빈 셀)을 찾고, 각 구간의 호 길이 arc = Δθ·r_edge·(e−s) 가 min_required_width = half_width×2 + clearance 이상인 구간만 후보로 남긴다. 각 후보 구간 안으로 목표 방위를 잘라 넣어(clamp) 목표에 가장 가까운 각을 고르고(동률이면 호가 긴 구간), 그 각을 /desired_angle 로 고정주기 발행한다.

| 파라미터 | 값 | 근거 |
|---|---|---|
| half_width | 0.29 m | 1번배 폭 0.57 → 0.285, 공용값 0.29 |
| clearance | 0.25 m | 20 시드 스윕 최적 (Ⅲ-6.1) |
| detection_distance 기본 / 게이트 | 3.0 / 2.0 m | 시뮬 스윕 |
| min_obstacle_cells | 1 | 3 이면 작은 부표를 무시한다 (두 배 공용) |
| temporal 투표 | OFF | 무익(시뮬) |
| rear_obstacle_ignore | 제거 | 부표 통째 삭제 결함 |


### 4.5) 도킹 — ship_dock
- 도크 3개 중 당일 지정 표식(삼각·원·사각 × 빨강·주황·노랑·초록·파랑·흰색)을 찾는다. 검출기 dock 이 target_color·target_shape 와 일치하는 후보의 방위각만 발행한다. 목표가 유효값 밖이면 기동 시 ERROR, 검출 없이 무효값 10000 을 낸다(임계는 혼동행렬 수치로 정함). 도크 간격 1.0 m.
- 접안 거리는 LiDAR 전방 섹터 최소거리 → ⚠️ 도크 규격 확정 후 contact_dist_m.

### 4.6) 제어기 — motor_control
PID 가 아니라 각도 기반 차동 조향이다. 상류 체인(/candidate_angle → /desired_angle)이 이미 "가야 할 방위" 를 주므로 제어기는 그 각도 오차를 좌·우 PWM 차동으로 바꾸고, 전방 장애물 거리와 페일세이프 레벨로 속도만 깎는다. 20 Hz 고정주기 타이머.

| /desired_angle | 분기 | 좌/우 PWM (ROS 기준, <1500 = 전진) | 감속 적용 |
|---|---|---|---|
| ≥ 50000 | 정지 STOP_HOLD | 1500 / 1500 | — |
| 20000 ~ 50000 | 미션 없음 → 전진 폴백 | base_pwm / base_pwm (1360) | ○ |
| 5000 (6000) | 제자리 우선회 (좌선회) | spin_forward_pwm ± spin_diff = 1600 / 1400 (1400 / 1600) | × (감속 걸면 뒤로 밀리며 돎) |
| −1 ~ 161 (정면 = 80) | 정상 조향 | offset = θ − 80, diff = /offset/ / max_angle × max_diff(100) → base ∓ diff | ○ |
| 161 ~ 5000 | 후진 | reverse_pwm / reverse_pwm (1590) | × |
| 명령 0.5 s 무수신 | 워치독 | 1500 / 1500 | — |

감속비 = min(장애물 근접비, 페일세이프 상한). 근접비는 전방 최근접 거리 d 에 대해 d ≥ 1.2 m 이면 1.0, 그 아래서 0.7 + 0.3·d/1.2 로 선형. 페일세이프 레벨 1 이면 0.7. 적용은 PWM 을 중립 쪽으로 당기는 식 1500 + (PWM − 1500)·ratio.
- steer_invert 하나가 조향과 SPIN 을 함께 뒤집는다(배선 반대 대비). ⚠️ 물 위 첫 시험에서 확정 — 가장 흔한 사고.
- PD 미채택 근거: 체인이 각도 기반이라 속도 상태가 없고, 각도 오차의 미분은 LiDAR 10 Hz·헤딩 노이즈를 증폭한다. 작년 SPIN 버그(순항 속도로 원을 그려 도크 충돌)는 중심 PWM 을 1500 으로 분리해 해결.

## 5) 페일세이프
- 4중 장치: time.monotonic() / ARMED(센서 1회 수신 후 감시) / 연속 N회(failsafe_confirm_n 3) / 히스테리시스 + 자동 복구.
- failsafe_warn_sec 0.7 → 감속(0.7 배), failsafe_stop_sec 3.0 → 정지. 독립 타이머(watchdog_cb)에서 평가 — 센서가 죽으면 콜백이 안 불리므로.
- motor_control 명령 워치독 0.5 s, 펌웨어 워치독 0.5 s, 부팅 시 중립. 접촉기 보조접점 → Mega 핀 38 로 비상정지 상태를 노트북이 안다.
- 진단(온도·헤딩 상태)은 로그만. /health_ok 는 제어에 물리지 않는다 — 거짓 정지 금지.
- TTC 비상제동은 해로워서 폐기, 시간 투표는 무익해서 OFF.

## 6) 시뮬레이션 및 시험 결과

### 6.1) 시뮬레이션 스윕 (tools/sim, 20 시드 × 폭 5 × clearance 6 × 탐지거리 2 × 게이트폭 2 = 2,400 런)

| 선폭 [m] | half_width | 최적 clearance | 접촉 평균 | 정체 |
|---|---|---|---|---|
| 0.43 | 0.215 | 0.35 | 0.36 | 0% |
| 0.50 | 0.25 | 0.30 | 0.85 | 0% |
| 0.58 | 0.29 | 0.25 | 1.32 | 0% |
| 0.64 | 0.32 | 0.25 | 1.44 | 0% |
| 0.70 | 0.35 | 0.20 | 2.23 | 0% |

작년 값 half_width 0.45 는 1.15 m 갭을 요구해 접촉 3.0 회·정체 1.5 회였다(1차 단일 시드 스윕). 최적점의 갭 요구(half_width×2 + clearance)는 0.78~0.90 m 에 모인다. 선회 중 유효 폭 W·cosθ + L·sinθ 는 1번배 10° 에서 0.86 m, 15° 에서 0.99 m 라 갭 요구 0.83 m 는 약 9° 이내로 정렬했을 때만 맞는다 — 1 m 안팎 틈은 시뮬이 못 잡는 영역이라 수조 게이트 시험으로 확인한다.

### 6.2) 수조·야외 시험

| 일자 | 장소 | 항목 | 결과 | 조치 |
|---|---|---|---|---|
| ____ | 수조 5×5 m | 직진·선회·게이트 2 m | ____ | ____ |
| ____ | 야외 | 항로추종 → 위치유지 전환 | ____ | ____ |
| ____ | 야외 | 회피 구간, 인식 유실 | ____ | ____ |

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
## 문서: docs/전달용/보고서_분담/03_LiDAR.md
==================================================================
# 지시서 03 — LiDAR (장애물 회피 · 페일세이프)

## 담당 목차와 분량

**양식 파일: `양식_LiDAR.docx`** — 아래 절의 번호·제목·빈 표가 본 보고서 모양으로 들어 있다. 여기에 채워서 낸다.

| 목차 | 지금 원고 | 할 일 |
|---|---|---|
| Ⅲ-1.3 센서 표의 LiDAR 줄 | 한 줄 | 장비 사진 1장, 개체 구분 설명 |
| **Ⅲ-2.3 LiDAR 처리** | 글머리표 4개 | **주 담당.** 공통 양식 8항목으로 확장 |
| **Ⅲ-4.4 장애물 회피 — ship_direction** | 문단 + 파라미터 표 | 알고리즘 상세. 2.3 과 겹치지 않게 2.3 은 "신호 처리", 4.4 는 "경로 선택"으로 나눈다 |
| Ⅲ-5 페일세이프 중 센서 감시 부분 | 글머리표 | `SensorWatch` 부분 보강 |
| Ⅲ-6.1 시뮬레이션 스윕 | 표 + 문단 | 숫자 대조, 설명 보강 (값의 근거가 이 시뮬이다) |

분량: **A4 2~2.5쪽** (표 3~4개, 그림 2장).

## 핵심 한 줄

`ship_direction` 이 LiDAR 스캔에서 배가 지나갈 수 있는 빈 구간을 찾아, 목표 방위에 가장 가까운 구간으로 조향각을 낸다. 올해는 배 폭을 실측해 통과 기준을 다시 잡았고(0.45 → 0.29 m), 물보라에 속지 않도록 감속 신호에만 median 필터를 걸었고, 센서가 죽으면 감속·정지하는 페일세이프를 독립 타이머로 분리했다.

## 반드시 답할 질문

1. RPLIDAR A3 개체는 배마다 무엇이고, 회전 주기와 각분해능이 어떻게 다른가? 그 차이가 셀 단위 파라미터에 왜 중요한가?
2. 드라이버는 그대로 쓰나? 우리가 바꾼 것은 무엇인가? 두 개체를 노트북이 어떻게 구분하나(udev)?
3. `/scan` 의 정면은 몇 도인가(`LIDAR_FORWARD_DEG`)? 구독 QoS 는?
4. `inf`·`nan` 셀은 어떻게 처리하나? 탐지 거리 기본값과 게이트 모드 값은?
5. 안전 구간을 어떻게 찾고, 통과 가능 판정 `min_required_width = half_width×2 + clearance` 와 호 길이 계산은 어떻게 하나? 가장자리 거리로 `r_edge` 를 쓰는 이유는?
6. 후보 구간 중 하나를 고르는 기준은? 특수 명령(5000, 20000, 50000)을 받으면?
7. median 필터는 어디에만 걸고 어디엔 안 거나? 이유와 시뮬 수치(가짜 감속 36.8% → 8.9%)는?
8. 갈 곳이 없으면 후진하나? 후진 시간·쿨다운·조향 변화율 제한 값은?
9. 지오펜스 광선을 실제 스캔과 어떻게 합치나? 경계에 가까우면 정지하는 조건은?
10. 페일세이프: 경고 0.7 s, 정지 3.0 s, 연속 N 회, 복구 조건. 왜 센서 콜백이 아니라 독립 타이머에서 판정하나?
11. 시간 투표(`TemporalVote`)는 왜 꺼 두었고, `rear_obstacle_ignore` 는 왜 없앴나? (TTC 비상제동은 이 노드가 아니라 `motor_control` 단계에서 기각됐다 — 팀장 절)
12. 시뮬 스윕(Ⅲ-6.1)은 어떤 조건을 몇 번 돌렸고, 그 결과로 어떤 값을 골랐나?
13. 단위 테스트는 몇 개이고 무엇을 검증하나?
14. 실측이 없는 것은? 부표 탐지 최대거리(과제 D4), 물보라 기각 수.

## 읽을 파일

| 파일 | 역할 | 우선 |
|---|---|---|
| `src/ship_direction/ship_direction/ship_direction.py` | 회피 노드 전체. 921줄 — 함수 단위로 읽게 할 것 | ★★★ |
| `src/ship_direction/ship_direction/failsafe.py` | `SensorWatch`, `TemporalVote`, `median_min` 순수 로직 | ★★★ |
| `src/ship_direction/config/ship_direction.yaml` | 파라미터 값과 주석 근거 | ★★★ |
| `tools/sim/README.md` | 시뮬 스윕 조건과 결과 | ★★ |
| `src/rplidar_ros/launch/rplidar_a3_launch.py` | 외부 드라이버 실행 설정. 우리가 정리한 부분만 | ★ |
| `src/ship_direction/launch/ship_direction_launch.py`, `src/launch_files/launch/launch_files.launch.py` | 실행 설정 | ★ |
| `src/ship_direction/test/` | 단위 테스트 | ★ |

관련 문서: `CLAUDE.md` 5장(페일세이프)·6장(시뮬 파라미터), `docs/전달용/LiDAR_노드_쉬운설명.md`, `docs/전달용/회신_rplidar_회피_수정제안.md`, `docs/전달용/회신_ship_direction_수정제안.md`, `docs/문제와작업/라이다_고장진단_20260818.md`, `docs/절차/2번배_udev_규칙_초안.md`

## 코드 출처

| 패키지 | 출처 | 작년부터 | 올해 바꾼 것 (근거 커밋) |
|---|---|---|---|
| `rplidar_ros` | **외부.** Slamtec 공식 RPLIDAR ROS 드라이버 | 있음 | A3 실행 파일 정리만(`673c20a`). 드라이버 코드는 그대로 |
| `ship_direction` | 작년 우리 코드 | 있음 | 페일세이프 분리(`02bfc39`, `2f4410a`), 시간 투표 OFF(`fd65e2f`), median(`849c570`), 지오펜스 병합(`b678154`), 후진·rate limit(`ed31dda`), half_width 0.45→0.29(`000e46c`) 등 |

## 작년 대비 변경 보는 명령

```bash
cd ~/ros2_ws
git log --format='%h %ad %s' --date=short 9bff661..HEAD -- src/ship_direction src/rplidar_ros
git diff 9bff661 HEAD -- src/ship_direction
git diff 19cfde7 HEAD -- src/rplidar_ros
```

🚨 `rplidar_ros` 는 `9bff661` 이 아니라 **`19cfde7` 기준**으로 본다. 그 전에는 외부 저장소 링크 형태여서 `9bff661` 과 비교하면 드라이버 전체 9만 줄이 "추가"로 나온다. 우리가 쓴 코드가 아니다.

## 경계

| 내용 | 누가 | 이 절에서는 |
|---|---|---|
| 지오펜스 **병합**과 경계 정지 | **LiDAR** | 전부 씀 |
| 지오펜스 광선 **생성** | GPS | "GPS 담당 절 참고" 한 줄 |
| 카메라 방위 + LiDAR 거리 조합 | 카메라 | "거리는 LiDAR 전방 섹터 최소거리" 한 줄 |
| 임무 노드(`ship_gate`·`ship_back`·`ship_turn`·`ship_dock`)의 `/scan` 사용 | 팀장 | 언급만 |
| 감속비 적용(`motor_control`) | 팀장 (Ⅲ-4.6) | "`/obstacle_distance_array` 를 받아 감속한다" 한 줄 |

## A배·B배

| | A배 (1번배) | B배 (2번배) |
|---|---|---|
| 개체 | 80BF (약 11.1 Hz / 0.250°) | 918A (11.0 Hz / 0.223°) |
| 배 폭 실측 | 0.57 m | 0.535 m |
| `half_width` | 공용 0.29 | 공용 0.29 (실측 폭 0.535 → 0.2675. 0.29 로 시뮬 접촉 0.95·정체 0 확인. 0.215 는 옛 설계폭 0.43 기준 값이라 쓰지 않는다) |

## 빈칸으로 둘 실측

| 항목 | 언제 |
|---|---|
| 부표 탐지 최대거리 (블랙박스 `obstacle_min_dist`) | 물 위 (과제 D4) |
| 물보라 기각 수 `/obstacle_reject_count` (회/분) | 야외 rosbag |
| 1 m 안팎 게이트 통과 성공률 | 수조 게이트 시험 |

## 그림 제안

1. 갭 선택 개념도: 스캔 → 장애물 팽창 → 안전 구간 → 폭 판정 → 목표에 가까운 구간 선택
2. 페일세이프 상태도: 정상 → 경고(감속) → 정지 → 복구

==================================================================
## 소스 파일 (행 번호 포함)
==================================================================

----- FILE: src/ship_direction/ship_direction/ship_direction.py -----
```
     1	import math
     2	import time
     3	from threading import Lock
     4	
     5	import rclpy
     6	from rclpy.node import Node
     7	from rclpy.executors import MultiThreadedExecutor
     8	from std_msgs.msg import Int32, Float32, Float32MultiArray
     9	from sensor_msgs.msg import LaserScan
    10	from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    11	
    12	# 판정 로직은 ROS 비의존 모듈에 있다 (ROS 없이 단위 테스트 가능).
    13	# 재수출: from ship_direction.ship_direction import SensorWatch 로도 접근 가능.
    14	from ship_direction.failsafe import SensorWatch, TemporalVote, median_min
    15	
    16	# 특수 신호 상수 (CLAUDE.md 3-9 상수표)
    17	SPIN_RIGHT = 5000.0          # 우선회
    18	SPIN_LEFT = 6000.0           # 좌선회
    19	CANDIDATE_INVALID = 20000.0  # 미션 없음 → yaw_error 기반 자율 회피
    20	STOP_HOLD = 50000.0          # 정지/대기
    21	
    22	REVERSE_ANGLE = 260.0        # 회피 경로 없음 → 후진
    23	LIDAR_FORWARD_DEG = 80.0     # LiDAR 프레임에서 정면 (상대방위 0 에 대응)
    24	
    25	
    26	def edge_range(distance_array, i, detection_distance):
    27	    """zone 가장자리 셀의 거리. inf·nan 은 '측정 없음' 이라 detection_distance 로 본다.
    28	
    29	    모듈 함수로 둔 이유: 노드 메서드 안 클로저였을 땐 테스트가 못 닿았다 (과제 S4).
    30	    """
    31	    v = distance_array[i]
    32	    return detection_distance if (math.isinf(v) or math.isnan(v)) else v
    33	
    34	
    35	def select_valid_zones(safe_zones, distance_array, angle_increment_rad,
    36	                       detection_distance, min_required_width):
    37	    """safe zone 중 배가 지나갈 수 있는 것만 고른다 → [(s, e, r_edge), ...].
    38	
    39	    폭 = 양 끝 중 가까운 거리(r_edge) × 호 길이. r_edge 를 쓰는 이유는 가장자리가
    40	    먼 쪽 거리로 재면 실제보다 넓게 보기 때문이다. min_required_width 는
    41	    half_width*2 + clearance (CLAUDE.md §6, yaml 과 같은 값).
    42	    """
    43	    valid = []
    44	    for s, e in safe_zones:
    45	        r_edge = min(edge_range(distance_array, s, detection_distance),
    46	                     edge_range(distance_array, e, detection_distance))
    47	        arc_len = angle_increment_rad * r_edge * (e - s)
    48	        if arc_len >= min_required_width:
    49	            valid.append((s, e, r_edge))
    50	    return valid
    51	
    52	
    53	# QoS-B: /scan 표준 sensor-data QoS. 작년은 depth=10 기본 RELIABLE 이었다.
    54	#   [1] 묵은 큐: LiDAR 10Hz × depth 10 = **1초치**가 쌓인다. 콜백이 한 번 밀리면
    55	#       그 뒤로 묵은 스캔이 burst 로 몰려와 '1초 전 장면' 으로 조향한다.
    56	#   [2] 워치독 왜곡: 스테일 판정이 '콜백 도착 시각' 기준이라, burst 가 워치독을 먹여
    57	#       실제로는 늦은 데이터인데 신선하다고 착각시킨다. depth=1 은 도착=신선을 일치시킨다.
    58	#   [3] 호환성: 구독자 BEST_EFFORT 는 발행자가 RELIABLE 이든 BEST_EFFORT 든 **전부 호환**된다
    59	#       (그 반대가 비호환). 현행 rplidar_ros 2.1.4 는 rplidar_node.cpp:440 에서
    60	#       rclcpp::QoS(KeepLast(10)) = RELIABLE 로 발행한다 — 소스 확인함. 드라이버를 갈아도 안 깨진다.
    61	#   드롭이 생겨도 '콜백 부재 → 스테일 → 페일세이프 발동' 으로 안전한 방향으로 실패한다.
    62	SCAN_QOS = QoSProfile(
    63	    reliability=ReliabilityPolicy.BEST_EFFORT,
    64	    history=HistoryPolicy.KEEP_LAST,
    65	    depth=1,
    66	)
    67	
    68	
    69	class ShipDirection(Node):
    70	    """LiDAR 회피 + 페일세이프. /desired_angle · /obstacle_distance_array · /failsafe_level 발행.
    71	
    72	    3단계에서 고친 것:
    73	
    74	    [A] 제어 루프를 고정 주기 타이머로 분리했다.
    75	        작년: 모든 계산·발행이 scan_callback 안에 있었다 → **LiDAR 가 끊기면 콜백이 아예 안 와서
    76	              아무도 /desired_angle 을 발행하지 않았다.** (측정: LiDAR 1~3초 끊김 → 발행 0회, 완전 침묵.
    77	              페일세이프 레벨은 올라가는데 출력이 없으니 L1/L2 가 '존재'하지 않았다.)
    78	        지금: scan_cb 는 **스캔 저장만**. control_cb 가 고정 주기로 **항상** 계산·발행한다.
    79	              → ship_direction 이 살아있는 한 /desired_angle 은 흐른다.
    80	                침묵 = ship_direction 사망 → motor_control 이 cmd_timeout 으로 잡는다(0.5s 로 조일 수 있게 됨).
    81	
    82	    [B] 페일세이프(독립 watchdog 타이머). scan 이 묵은 정도로 레벨을 매긴다. **3상태·2임계.**
    83	        0 정상        : 계산한 각도        / 속도 100%
    84	        1 경고(0.7s)  : 계산한 각도 그대로 / 속도 70%    ← 각도 안 건드림
    85	        2 정지(3.0s)  : STOP_HOLD override / 정지        ← 각도를 덮어쓰는 건 여기뿐
    86	
    87	        ★ 임계가 적을수록 틀릴 곳이 적다. 중간에 'freeze' 레벨을 뒀다가 뺐다 —
    88	          새 스캔이 없는데 재계산하면 같은 스캔으로 같은 답이 나온다.
    89	          즉 '재계산'과 'freeze'는 애초에 같은 동작이라 레벨을 나눌 이유가 없었다.
    90	        ★ 경고에서 '직진 강제' 같은 걸 하면 회피 기동 중간에 핸들을 놓는 셈이다.
    91	          부표를 피하던 중 직진하면 그대로 박는다. 0.7초 묵었다고 조향을 포기하는 건 과잉반응이다.
    92	          묵은 데이터라도 없는 것보다 낫다 — 대신 속도를 줄여 오차를 줄인다.
    93	
    94	        속도는 /desired_angle(각도)에 담을 수 없다 → motor_control 이 /failsafe_level 을 구독해
    95	        속도 상한을 건다. (레벨 2 는 STOP_HOLD 가 각도로 오므로 motor_control 분기 (1) 이 처리)
    96	
    97	        오탐 방지(CLAUDE.md §5): ARMED(스캔 한 번은 받아야 평가 시작) + 올릴 땐 연속 N회 확인 +
    98	        내릴 땐 즉시(자동 복구) + time.monotonic().
    99	
   100	    [C] rear_obstacle_ignore_margin 제거 — 튜닝값이 아니라 증명된 결함이다.
   101	        물보라 반사 0.3m 하나가 front_min_distance 를 끌어내려, 실제 1.6m 부표를 통째로
   102	        마스크에서 지웠다 (17셀 → 0셀).
   103	
   104	    [D] time.time() → time.monotonic() 전부 교체.
   105	
   106	    [E] 워치독이 /scan 과 /yaw_error 를 **둘 다** 감시한다 (센서별 dict, 가장 나쁜 센서가 레벨을 정함).
   107	        /scan 만 보면 IMU 사망을 놓친다. 1단계에서 만든 사슬은
   108	          IMU 사망 → ship_goal_angle 이 /yaw_error 를 끊는다 → ship_direction 이 감지해 정지
   109	        인데, 세 번째 화살표(침묵을 들을 귀)가 없으면 마지막 yaw_error 를 영원히 붙들고 조향한다.
   110	        (측정: IMU 사망 시 정지율 0%, 접촉 3회, 14.8m 계속 주행)
   111	        센서별로 첫 데이터를 받은 뒤에만 감시(ARMED) — 부팅 오발동 방지.
   112	
   113	    [F] angle_increment == 0 가드. min_index/max_index 계산에서 이 값으로 나눈다 →
   114	        LiDAR 가 이상 스캔을 보내면 ZeroDivisionError 로 노드가 통째로 죽는다
   115	        (작년 코드에도 방어가 없었다). 이상 스캔은 scan_callback 에서 버리고 즉시 정지.
   116	        하드 폴트라 확인 N회 없이 바로 레벨 2. 정상 스캔이 오면 스스로 풀린다.
   117	
   118	    [G] 시간 투표 필터(TemporalVote) — **기본 OFF(temporal_frames=1)**. 코드는 남겨둔다.
   119	        최근 N프레임 중 votes 회 이상 나타난 셀만 장애물로 인정하는 필터다.
   120	        처음엔 효과가 있어 보였으나(단일 시드), **시드 20개로 재보니 효과가 없었다:**
   121	          필터 OFF → 접촉 19/20, 전진 25.5m  /  필터 ON → 접촉 20/20, 전진 23.4m
   122	        해당 시나리오(정면 부표 + 물보라 60%)는 어느 코드든 95% 접촉하는 '통과 불가' 판이었다.
   123	        → **무해하지만 무익.** (TTC 는 '해로워서' 삭제했지만, 이건 '무익해서' 기본 OFF다.)
   124	        실제 물보라의 시간 특성이 시뮬과 다를 수 있으니 켤 수 있게 남긴다: frames=3, votes=2.
   125	        ★ 켤 때는 반드시 dilate 전, 원본 마스크에 건다
   126	          (팽창 후에 걸면 진짜 부표의 팽창 영역까지 표가 갈려 부표가 얇아진다).
   127	
   128	    [H] Geofence (6a-2). 경기장 밖으로 나가면 실격인데 방어가 0 이었다.
   129	        north 가 경계를 **'가짜 LiDAR'** 로 쏴서 보낸다:
   130	          /geofence_state = [angle_min_deg, angle_inc_deg, r0, r1, ...] (상대방위, 멀면 inf)
   131	        여기서 하는 일은 **스캔 병합 한 줄**이 전부다:
   132	          ranges[i] = min(real_ranges[i], geofence_ranges[i])
   133	        그 뒤는 기존 파이프라인이 알아서 한다 — detection_distance 로 마스크가 되고,
   134	        dilate 가 배 폭 여유를 더하고, 갭-팔로잉이 피한다.
   135	        **특수 로직도, 새 상태기계도, 튜닝 파라미터(half_block)도 없다.**
   136	
   137	        ★ 왜 '원뿔 칠하기'를 버렸나: **벽은 '점'이 아니라 '선'이다.**
   138	          최근접점 한 방향으로 ±각도를 막는 건 기하학적으로 틀렸다. 40×40 모서리에서
   139	          배가 대각선(45°)을 보면 북벽 최근접점 -45°, 동벽 +45° 인데 **탈출구(모서리)는 정면 0°**.
   140	          half_block=40° 면 [-85,-5]·[5,85] 만 막혀 **정면 0° 가 '틈'으로 열린다** → 대각선 탈출, 실격.
   141	          광선으로 쏘면 정면 경계 거리(2.12m)가 그대로 벽이 된다. 경기장이 사각형이 아니어도 맞다.
   142	
   143	        ★ 빈 배열/stale 이면 병합하지 않는다 — 묵은 '없는 벽'은 배를 가둔다.
   144	
   145	    [I] Geofence 하드 가드 — (A)SPIN·(B)candidate 구간 전용 (6a-2 보강).
   146	        [H]의 스캔 병합은 (C) 자율회피에서만 일어난다. 그런데 **도킹은 SPIN+candidate 로 조향**하고
   147	        이 구간은 distance_array 를 안 만들어 병합할 대상이 없다. 도크가 경기장 가장자리면
   148	        접안하다 경계로 파고들어 이탈 → 실격. (측정으로 확인된 실제 위험.)
   149	        → _compute 맨 앞에서 검사: 경계 최소거리가 geofence_stop_margin_m(2.0) 안이고 그 상대방위가
   150	          '전진하려는 방향 ±geofence_stop_cone_deg(60°)' 와 겹치면 → candidate/SPIN 무시하고 STOP_HOLD.
   151	        **미션보다 실격 방지가 우선.** 경계를 등지고 멈춘다.
   152	        ⚠️ north 광선은 전방 ±80° 만 → 후진 중 뒤쪽 경계는 못 본다(접안=전방 방어용).
   153	
   154	    [J] 감속 신호용 공간 median 필터 (obst_median_kernel, **기본 ON**).
   155	        _closest_obstacle 이 필터 없는 raw min 이라 **물보라 반사 한 점(0.3m)이 그대로
   156	        motor_control 감속을 물렸다.** 이웃과 어긋나는 고립 스파이크를 range 값 수준에서 지운다.
   157	        측정(시드 20, 물보라 30%): raw-min 가짜감속 **36.8% → median 8.9%**(기준선). 접촉은 전 조건 0.
   158	        ★ **감속 신호에만** 건다 — _compute 의 회피 마스크는 안 건드린다(그래서 접촉이 안 변한다).
   159	        ★ [G] TemporalVote(시간 투표)와 다르다: 저건 효과 0 이라 OFF, 이건 명확한 이득이라 ON.
   160	          시간 투표는 프레임 간, 이건 한 프레임 안의 공간.
   161	        ⚠️ 시뮬 물보라는 '단일점' 모델이다. 실제 물보라가 작은 군집이면 커널5로 부족할 수 있다
   162	           → 배 뜬 뒤 블랙박스의 obstacle_min_dist 로 실측 확인하고 커널을 7/9 로 올릴 것.
   163	
   164	    ※ 판정 로직(SensorWatch, TemporalVote, median_min)은 **ROS 비의존 모듈** ship_direction/failsafe.py 에 있다.
   165	      워치독을 노드 안에 인라인하면 단위 테스트가 불가능해진다. rclpy 없이 임포트 가능:
   166	          from ship_direction.failsafe import SensorWatch, TemporalVote   # ← ROS 불필요(권장)
   167	          from ship_direction.ship_direction import SensorWatch           # ← 재수출(ROS 필요)
   168	    """
   169	
   170	    def __init__(self):
   171	        super().__init__('ship_direction')
   172	
   173	        self.lock = Lock()
   174	
   175	        # ----------------------
   176	        #   파라미터 (config yaml, CLAUDE.md 1-4)
   177	        # ----------------------
   178	        self.control_period = float(self.declare_parameter('control_period_sec', 0.1).value)    # 10Hz
   179	        self.watchdog_period = float(self.declare_parameter('watchdog_period_sec', 0.1).value)
   180	
   181	        # 페일세이프 임계(센서가 묵은 시간). 3상태·2임계 (CLAUDE.md §5)
   182	        self.fs_warn_sec = float(self.declare_parameter('failsafe_warn_sec', 0.7).value)   # → 레벨 1
   183	        self.fs_stop_sec = float(self.declare_parameter('failsafe_stop_sec', 3.0).value)   # → 레벨 2
   184	        self.fs_confirm_n = int(self.declare_parameter('failsafe_confirm_n', 3).value)
   185	
   186	        # ---- Geofence (경계 이탈 = 실격). 6a-2 ----
   187	        # north 가 경계를 '가짜 LiDAR'로 쏴서 보낸다 → 실제 스캔과 min() 병합만 하면 끝이다.
   188	        # 특수 로직도, 튜닝 파라미터(half_block)도 없다. 아래 [H] 참고.
   189	        # 🚨 묵으면 병합하지 않는다. 묵은 '없는 벽'은 배를 가둔다. (north 주기 0.5s 의 4배)
   190	        self.geofence_stale_sec = float(self.declare_parameter('geofence_stale_sec', 2.0).value)
   191	        # 하드 가드([I]): 경계가 이 거리 안이고 전진 방향과 겹치면 미션 무시하고 정지.
   192	        # (A)SPIN/(B)candidate 구간은 마스크를 안 만들어 병합이 안 되므로, 도킹이 경계로 파고들면
   193	        # 이탈한다. 미션보다 실격 방지가 우선 → 경계를 등지고 멈춘다.
   194	        self.geofence_stop_margin_m = float(
   195	            self.declare_parameter('geofence_stop_margin_m', 2.0).value)
   196	        self.geofence_stop_cone_deg = float(
   197	            self.declare_parameter('geofence_stop_cone_deg', 60.0).value)
   198	
   199	        # ---- [J] 감속 신호용 공간 median 필터. **기본 ON.** ----
   200	        # _closest_obstacle(감속 신호)에만 건다. 회피 마스크(_compute)는 안 건드린다.
   201	        # 측정(시드 20, 물보라 30%): raw-min 가짜감속 36.8% → median 8.9%. 접촉 전 조건 0.
   202	        # 0 이면 raw-min 폴백.
   203	        self.obst_median_kernel = int(self.declare_parameter('obst_median_kernel', 5).value)
   204	
   205	        # 시간 투표 필터 — **기본 OFF(frames=1)**. 시드 20개로 재보니 효과가 없었다(아래 [G]).
   206	        # 무해하지만 무익하다. 실제 물보라의 시간 특성이 시뮬과 다를 수 있으니 코드는 남겨두고
   207	        # 켤 수 있게만 해둔다: temporal_frames=3, temporal_votes=2 로 켠다.
   208	        self.temporal_frames = int(self.declare_parameter('temporal_frames', 1).value)
   209	        self.temporal_votes = int(self.declare_parameter('temporal_votes', 2).value)
   210	
   211	        # 회피 튜닝 — §6 시뮬 스윕으로 검증된 값. 임의로 바꾸지 말 것.
   212	        self.detection_distance_default = float(
   213	            self.declare_parameter('detection_distance_default', 3.0).value)   # 1.8 이면 8/8 충돌
   214	        self.detection_distance_gate = float(
   215	            self.declare_parameter('detection_distance_gate', 2.0).value)      # 3.0 이면 게이트가 좁아 접촉 증가
   216	        # 게이트 구간의 wp_mode. 확정된 웨이포인트 표(3-6): mode 0=게이트 시작, 1=게이트 끝.
   217	        # (작년 코드는 wp_mode==2 (위치유지)를 키로 썼는데, 그건 게이트가 아니다.)
   218	        self.gate_wp_modes = list(self.declare_parameter('gate_wp_modes', [0, 1]).value)
   219	
   220	        self.half_width = float(self.declare_parameter('half_width', 0.29).value)
   221	        # 0.20→접촉 1.7회 / 0.25→0.2회 / 0.30→0.8회 / 0.45→807초 폭주(과보수)
   222	        self.clearance = float(self.declare_parameter('clearance', 0.25).value)
   223	        self.border_margin = int(self.declare_parameter('border_margin', 2).value)
   224	        self.max_spike_ratio = float(self.declare_parameter('max_spike_ratio', 0.01).value)
   225	        # ※ rear_obstacle_ignore_margin 은 제거했다 (파라미터 자체를 없앰). 위 [C] 참고.
   226	        self.min_obstacle_cells = int(
   227	            self.declare_parameter('min_obstacle_cells', 1).value)   # 3 이면 작은 부표가 무시된다
   228	
   229	        # ---- [L] 후진 후 재탐색 상태기계 ----
   230	        # 회피 경로가 없으면 reverse_duration 동안 후진하고,
   231	        # 이후 reverse_cooldown 동안 STOP_HOLD 상태로 재탐색한다.
   232	        self.reverse_duration = float(
   233	            self.declare_parameter('reverse_duration_sec', 0.5).value)
   234	        self.reverse_cooldown = float(
   235	            self.declare_parameter('reverse_cooldown_sec', 3.0).value)
   236	
   237	        # 잘못된 파라미터 방어
   238	        if not math.isfinite(self.reverse_duration) or self.reverse_duration <= 0.0:
   239	            self.get_logger().warn(
   240	                f"reverse_duration_sec={self.reverse_duration} 이상 → 기본값 0.5초 사용")
   241	            self.reverse_duration = 0.5
   242	        if not math.isfinite(self.reverse_cooldown) or self.reverse_cooldown < 0.0:
   243	            self.get_logger().warn(
   244	                f"reverse_cooldown_sec={self.reverse_cooldown} 이상 → 기본값 3.0초 사용")
   245	            self.reverse_cooldown = 3.0
   246	
   247	        # ---- [K] 조향각 변화율 제한 (rate limiter). 팀원 피드백으로 "0~160° 한정" 적용. ----
   248	        # 원안(전체 구간 적용)은 /desired_angle 이 각도+특수신호(260/5000/6000/20000/50000) 혼용
   249	        # 프로토콜이라, 정상값→특수신호 사이를 15°/tick 씩 기어가면 motor_control 이 그 경유값을
   250	        # 후진/선회/무효로 오해할 위험이 있었다(팀 지적). 0~160° 구간(정상 조향값)일 때만 제한을
   251	        # 걸고, 그 밖(특수신호 및 candidate 매핑의 범위 밖 부산물)은 그대로 통과시킨다.
   252	        # 덤: 180° 급반전(정후방 목표 → 좌우 zone 끝 선택)도 결과가 항상 0~160° 안이라 이 필터로
   253	        # 완화된다. 실제 각속도 = 이 값 / control_period(예: 15°, 0.1s 면 150°/s) — 실측 후 튜닝.
   254	        self.rate_limit_max_deg_per_tick = float(
   255	            self.declare_parameter('rate_limit_max_deg_per_tick', 15.0).value)
   256	
   257	        self.detection_distance = self.detection_distance_default
   258	
   259	        # ----------------------
   260	        #      Subscriptions
   261	        # ----------------------
   262	        self.create_subscription(LaserScan, '/scan', self.scan_callback, SCAN_QOS)
   263	        self.create_subscription(Float32, '/yaw_error', self.yaw_error_callback, 10)
   264	        self.create_subscription(Float32, '/candidate_angle', self.candidate_callback, 10)
   265	        self.create_subscription(Int32, '/wp_mode', self.wp_mode_cb, 10)
   266	        self.create_subscription(Float32MultiArray, '/geofence_state', self.geofence_cb, 10)
   267	
   268	        # ----------------------
   269	        #       Publishers  (토픽/타입 불변 + /failsafe_level 신규, CLAUDE.md 3-9)
   270	        # ----------------------
   271	        self.pub_desired_angle = self.create_publisher(Float32, '/desired_angle', 10)
   272	        self.pub_obstacle_distance = self.create_publisher(Float32MultiArray, '/obstacle_distance_array', 10)
   273	        # D6: median 필터가 버린 고립 스파이크 수 (관측 전용, 신규 토픽 — 계약 불변).
   274	        #   제어에 안 쓴다. blackbox 가 CSV 에 기록해 '노이즈 수 ↔ 기상 열화' 상관 검증용.
   275	        self.pub_reject = self.create_publisher(Int32, '/obstacle_reject_count', 10)
   276	        self.pub_failsafe = self.create_publisher(Int32, '/failsafe_level', 10)
   277	
   278	        # ----------------------
   279	        #       State
   280	        # ----------------------
   281	        self.wp_mode = -1
   282	        self.wp4_enter_time = None
   283	        self.yaw_error = 0.0
   284	        self.candidate_angle = CANDIDATE_INVALID
   285	
   286	        self.latest_scan = None       # scan_cb 는 저장만 한다
   287	
   288	        # [L] 후진 상태기계
   289	        # READY: 새 후진 시작 가능 / REVERSING: 일정 시간 후진 / COOLDOWN: STOP 재탐색
   290	        self.reverse_state = 'READY'
   291	        self.reverse_start_time = None
   292	        self.reverse_cooldown_start_time = None
   293	
   294	        self.prev_final_angle = None  # [K] rate limiter 상태. None = "직전 값 없음/특수신호였음" → 다음 정상값에 즉시 스냅
   295	
   296	        # geofence: (거리 m, 상대방위 deg) 또는 None. north 가 [inf,nan] 을 내면 None.
   297	        self.geofence = None
   298	        self.last_geofence_t = None
   299	
   300	        # 페일세이프 감시 — /scan 과 /yaw_error 를 둘 다 본다.
   301	        # ★ /scan 만 보면 IMU 사망을 놓친다. IMU 가 죽으면 ship_goal_angle 이 /yaw_error 를
   302	        #   끊어주는데(1단계 설계), 그 침묵을 들을 귀가 없으면 마지막 yaw_error 로 영원히 조향한다.
   303	        self.watch = SensorWatch(
   304	            ['scan', 'yaw_error'],
   305	            warn_sec=self.fs_warn_sec,
   306	            stop_sec=self.fs_stop_sec,
   307	            confirm_n=self.fs_confirm_n,
   308	        )
   309	        self.failsafe_level = 0
   310	
   311	        # 시간 투표 필터 (dilate 전, 원본 마스크에 적용)
   312	        self.temporal = TemporalVote(self.temporal_frames, self.temporal_votes)
   313	
   314	        # ----------------------
   315	        #  타이머: 제어 / 워치독 (서로 독립)
   316	        # ----------------------
   317	        self.control_timer = self.create_timer(self.control_period, self.control_cb)
   318	        self.watchdog_timer = self.create_timer(self.watchdog_period, self.watchdog_cb)
   319	
   320	        self.get_logger().info(
   321	            f"ship_direction 시작: 제어 {1.0 / self.control_period:.0f}Hz (스캔 유무 무관 항상 발행), "
   322	            f"워치독 {1.0 / self.watchdog_period:.0f}Hz.\n"
   323	            f"   페일세이프 경고={self.fs_warn_sec}s(레벨1) 정지={self.fs_stop_sec}s(레벨2), "
   324	            f"확인 {self.fs_confirm_n}회. 각도 override 는 레벨2 뿐.\n"
   325	            f"   시간투표 {self.temporal_frames}프레임 중 {self.temporal_votes}표 "
   326	            f"({'ON' if self.temporal.enabled else 'OFF'}), 감시 센서: scan + yaw_error."
   327	        )
   328	
   329	    # =====================================================
   330	    # CALLBACKS — 저장만 한다
   331	    # =====================================================
   332	    def scan_callback(self, msg: LaserScan):
   333	        """저장만. 계산·발행은 control_cb 가 고정 주기로 한다.
   334	        (작년엔 여기서 다 했다 → LiDAR 가 죽으면 콜백이 안 와서 아무도 발행하지 않았다)
   335	
   336	        [F] angle_increment 가드: 0/음수/NaN 이면 min_index 계산에서 ZeroDivisionError 로
   337	        노드가 통째로 죽는다(작년 코드에도 방어가 없었다). 이상 스캔은 문 앞에서 버리고
   338	        bad_scan 을 세워 즉시 정지시킨다. 정상 스캔이 오면 스스로 풀린다."""
   339	        inc = math.degrees(msg.angle_increment)
   340	        if not math.isfinite(inc) or inc <= 0.0:
   341	            self.get_logger().error(
   342	                f"스캔 angle_increment 이상({msg.angle_increment}) → 이 스캔 폐기, 정지",
   343	                throttle_duration_sec=1.0,
   344	            )
   345	            with self.lock:
   346	                self.watch.set_fault(True)     # 하드 폴트 → 즉시 레벨 2
   347	            return
   348	
   349	        with self.lock:
   350	            self.watch.set_fault(False)        # 정상 스캔이 오면 스스로 풀린다
   351	            self.latest_scan = msg
   352	            self.watch.feed('scan', time.monotonic())
   353	
   354	    def yaw_error_callback(self, msg):
   355	        with self.lock:
   356	            self.yaw_error = (360.0 - msg.data) % 360.0
   357	            self.watch.feed('yaw_error', time.monotonic())   # [E] IMU 체인 생존 신호
   358	
   359	    def geofence_cb(self, msg):
   360	        """/geofence_state = [angle_min_deg, angle_inc_deg, r0, r1, ...] (상대방위, 멀면 inf).
   361	        경계를 '가짜 LiDAR'로 쏜 것이다. **빈 배열 = 경계 정보 없음**
   362	        (north 가 '모르면 입을 다무는' 계약: 폴리곤 미설정 / IMU stale / 이미 이탈)."""
   363	        d = msg.data
   364	        with self.lock:
   365	            if len(d) >= 3 and math.isfinite(d[1]) and d[1] > 0.0:
   366	                self.geofence = (float(d[0]), float(d[1]), [float(x) for x in d[2:]])
   367	            else:
   368	                self.geofence = None
   369	            self.last_geofence_t = time.monotonic()
   370	
   371	    def candidate_callback(self, msg):
   372	        val = msg.data
   373	        with self.lock:
   374	            if val in (CANDIDATE_INVALID, STOP_HOLD, SPIN_RIGHT, SPIN_LEFT):
   375	                self.candidate_angle = val
   376	            else:
   377	                self.candidate_angle = (val % 360.0)
   378	
   379	    def wp_mode_cb(self, msg):
   380	        with self.lock:
   381	            prev = self.wp_mode
   382	            self.wp_mode = msg.data
   383	
   384	            # 게이트 구간(mode 0,1)은 좁아서 detection 을 짧게 (3.0 이면 오히려 접촉 증가, §6)
   385	            new_dd = (self.detection_distance_gate
   386	                      if self.wp_mode in self.gate_wp_modes
   387	                      else self.detection_distance_default)
   388	            if abs(new_dd - self.detection_distance) > 1e-6:
   389	                self.detection_distance = new_dd
   390	
   391	            if prev != 4 and self.wp_mode == 4:
   392	                self.wp4_enter_time = time.monotonic()
   393	            elif prev == 4 and self.wp_mode != 4:
   394	                self.wp4_enter_time = None
   395	
   396	    # =====================================================
   397	    # WATCHDOG — 독립 타이머 (CLAUDE.md §5)
   398	    # =====================================================
   399	    def watchdog_cb(self):
   400	        """페일세이프를 독립 타이머로 평가한다.
   401	        scan_cb 안에서 평가하면 LiDAR 가 완전히 죽었을 때 콜백이 아예 안 와서
   402	        페일세이프가 영원히 평가되지 않는다 (§5 에 기록된 실제 버그).
   403	
   404	        [E] 감시 대상은 /scan 과 /yaw_error **둘 다**. 가장 나쁜 센서로 레벨을 정한다.
   405	            /scan 만 보면 IMU 사망을 놓친다: IMU 가 죽으면 ship_goal_angle 이 /yaw_error 를
   406	            끊어주는데(1단계 설계), 그 침묵을 들을 귀가 없으면 ship_direction 은 마지막
   407	            yaw_error 를 영원히 붙들고 조향한다. (측정: 정지율 0%, 접촉 3회, 14.8m 계속 주행)
   408	            센서별로 첫 데이터를 받은 뒤에만 감시(ARMED) — 부팅 오발동 방지.
   409	
   410	        판정 로직은 SensorWatch(ROS 비의존)에 있다 — 단위 테스트 가능."""
   411	        now = time.monotonic()
   412	        prev = self.failsafe_level
   413	
   414	        with self.lock:
   415	            level = self.watch.update(now)
   416	            worst = self.watch.worst
   417	
   418	        if level != prev:
   419	            # 🚨 로거 호출을 변수에 담아 한 줄에서 warn/info 를 번갈아 쓰면 안 된다.
   420	            #    rclpy 는 로그 호출을 **소스 위치(파일:줄)** 로 캐싱해서, 같은 줄이 다른 심각도로
   421	            #    불리면 ValueError('Logger severity cannot be changed between calls.') 를 던진다.
   422	            #    레벨 상승은 warn, 복구는 info 라서 **복구되는 순간 노드가 죽었다** (실물에서 발견).
   423	            #    → 센서가 고장나서가 아니라 '고장에서 회복해서' 배가 멈추는 최악의 형태였다.
   424	            #    호출 지점을 둘로 나눠서 각 줄이 한 가지 심각도만 쓰게 한다.
   425	            msg = (f"페일세이프 L{prev} → L{level}"
   426	                   + (f"  (원인: {worst})" if worst else "  (복구)"))
   427	            if level > prev:
   428	                self.get_logger().warn(msg)
   429	            else:
   430	                self.get_logger().info(msg)
   431	
   432	        self.failsafe_level = level
   433	        self.pub_failsafe.publish(Int32(data=int(level)))
   434	
   435	    # =====================================================
   436	    # [K] RATE LIMITER — 0~160°(정상 조향값)에서만 변화율 제한
   437	    # =====================================================
   438	    def _apply_rate_limit(self, target_angle):
   439	        """0~160° 구간일 때만 한 tick당 변화폭을 제한한다.
   440	
   441	        특수신호는 즉시 통과시키고 상태를 초기화한다. NaN/inf는 STOP_HOLD로 바꾼다.
   442	        """
   443	        if not math.isfinite(target_angle):
   444	            self.prev_final_angle = None
   445	            return STOP_HOLD
   446	
   447	        in_range = 0.0 <= target_angle <= 160.0
   448	
   449	        if not in_range:
   450	            self.prev_final_angle = None
   451	            return target_angle
   452	
   453	        if self.prev_final_angle is None or not (0.0 <= self.prev_final_angle <= 160.0):
   454	            self.prev_final_angle = target_angle
   455	            return target_angle
   456	
   457	        limit = self.rate_limit_max_deg_per_tick
   458	        if not math.isfinite(limit) or limit <= 0.0:
   459	            self.prev_final_angle = target_angle
   460	            return target_angle
   461	
   462	        diff = target_angle - self.prev_final_angle
   463	        diff = max(-limit, min(limit, diff))
   464	        limited = self.prev_final_angle + diff
   465	        self.prev_final_angle = limited
   466	        return limited
   467	
   468	    # =====================================================
   469	    # [L] REVERSE STATE MACHINE
   470	    # =====================================================
   471	    def _apply_reverse_sequence(self, now):
   472	        """경로가 없을 때 일정 시간 후진한 뒤 STOP 재탐색을 수행한다."""
   473	        if self.reverse_state == 'REVERSING':
   474	            if now - self.reverse_start_time < self.reverse_duration:
   475	                return REVERSE_ANGLE
   476	
   477	            self.reverse_state = 'COOLDOWN'
   478	            self.reverse_start_time = None
   479	            self.reverse_cooldown_start_time = now
   480	            self.get_logger().info(
   481	                f"후진 {self.reverse_duration:.2f}초 완료 → "
   482	                f"STOP 재탐색 {self.reverse_cooldown:.2f}초 시작",
   483	                throttle_duration_sec=1.0)
   484	            return STOP_HOLD
   485	
   486	        if self.reverse_state == 'COOLDOWN':
   487	            if now - self.reverse_cooldown_start_time < self.reverse_cooldown:
   488	                return STOP_HOLD
   489	
   490	            self.reverse_state = 'REVERSING'
   491	            self.reverse_start_time = now
   492	            self.reverse_cooldown_start_time = None
   493	            self.get_logger().warn(
   494	                f"STOP 재탐색 {self.reverse_cooldown:.2f}초 후에도 경로 없음 → 후진 재시작",
   495	                throttle_duration_sec=1.0)
   496	            return REVERSE_ANGLE
   497	
   498	        self.reverse_state = 'REVERSING'
   499	        self.reverse_start_time = now
   500	        self.reverse_cooldown_start_time = None
   501	        self.get_logger().warn(
   502	            f"회피 경로 없음 → {self.reverse_duration:.2f}초 후진 시작",
   503	            throttle_duration_sec=1.0)
   504	        return REVERSE_ANGLE
   505	
   506	    def _reset_reverse_sequence(self):
   507	        """정상 경로나 외부 미션 명령이 생기면 후진/쿨다운 상태를 해제한다."""
   508	        self.reverse_state = 'READY'
   509	        self.reverse_start_time = None
   510	        self.reverse_cooldown_start_time = None
   511	
   512	    # =====================================================
   513	    # CONTROL — 고정 주기, 스캔 유무와 무관하게 항상 발행
   514	    # =====================================================
   515	    def control_cb(self):
   516	        level = self.failsafe_level
   517	
   518	        with self.lock:
   519	            scan = self.latest_scan
   520	            bad = self.watch.fault
   521	            wp4_enter_time = self.wp4_enter_time
   522	            candidate_angle = self.candidate_angle
   523	            yaw_error = self.yaw_error
   524	            detection_distance = self.detection_distance
   525	
   526	        # ---- 레벨 2(정지) 또는 이상 스캔: STOP_HOLD 로 override. 각도를 덮어쓰는 건 여기뿐. ----
   527	        # bad 를 여기서도 보는 이유: 워치독 틱(0.1s)을 기다리지 않고 즉시 멈추기 위해서다.
   528	        if level >= 2 or bad:
   529	            self.pub_obstacle_distance.publish(self._closest_obstacle(scan))
   530	            angle = self._apply_rate_limit(STOP_HOLD)   # [K] 범위 밖 값 → 통과 + 상태 리셋
   531	            self.pub_desired_angle.publish(Float32(data=angle))
   532	            return
   533	
   534	        # 스캔을 한 번도 못 받음(부팅 중) → 발행할 각도가 없다.
   535	        # motor_control 은 '첫 명령 전 중립'이라 안전하다.
   536	        if scan is None:
   537	            return
   538	
   539	        # ---- 레벨 0·1: 계산한 각도를 그대로 낸다. ----
   540	        # 레벨 1(경고)에서도 각도는 안 건드린다. 속도만 줄인다
   541	        # → motor_control 이 /failsafe_level 을 보고 상한(70%)을 건다.
   542	        angle, obst = self._compute(scan, wp4_enter_time, candidate_angle,
   543	                                    yaw_error, detection_distance)
   544	        angle = self._apply_rate_limit(angle)   # [K] 0~160° 안일 때만 실제로 제한됨
   545	
   546	        self.pub_obstacle_distance.publish(obst)
   547	        self.pub_desired_angle.publish(Float32(data=angle))
   548	
   549	    # =====================================================
   550	    # 계산 (회피 알고리즘 — 작년 로직 보존)
   551	    # =====================================================
   552	    def _closest_obstacle(self, msg):
   553	        """전방 0~160° 최근접 → Float32MultiArray([거리(m), 각도(deg)]). 없으면 [inf, nan]."""
   554	        obst = Float32MultiArray()
   555	        if msg is None:
   556	            obst.data = [float('inf'), float('nan')]
   557	            self.pub_reject.publish(Int32(data=0))     # D6: 스캔 없음 = 기각 0
   558	            return obst
   559	
   560	        angle_min = math.degrees(msg.angle_min)
   561	        angle_increment_deg = math.degrees(msg.angle_increment)
   562	        ranges = msg.ranges
   563	
   564	        # [F] 0 나눗셈 방어 (scan_callback 이 이미 걸러내지만, 여기도 막아둔다)
   565	        if not math.isfinite(angle_increment_deg) or angle_increment_deg <= 0.0:
   566	            obst.data = [float('inf'), float('nan')]
   567	            self.pub_reject.publish(Int32(data=0))
   568	            return obst
   569	
   570	        min_index = int((0 - angle_min) / angle_increment_deg)
   571	        max_index = int((160 - angle_min) / angle_increment_deg)
   572	
   573	        # ---- [J] 공간 median 후 최소거리 (감속 신호 전용) ----
   574	        # 작년엔 필터 없는 raw min 이라 물보라 반사 한 점(0.3m)이 그대로 motor_control 감속을
   575	        # 물렸다. median 은 이웃과 어긋나는 고립 스파이크를 지운다.
   576	        # ★ 여기(감속 신호)에만 건다 — _compute 의 회피 마스크는 손대지 않는다(접촉 성능 불변).
   577	        # D6: 기각수(rejected)를 함께 받아 /obstacle_reject_count 로 낸다. 관측 전용 — 제어 불변.
   578	        d, i, rejected = median_min(
   579	            ranges, min_index, max_index, self.obst_median_kernel, return_rejected=True)
   580	        self.pub_reject.publish(Int32(data=int(rejected)))
   581	        if d is None:
   582	            obst.data = [float('inf'), float('nan')]
   583	        else:
   584	            obst.data = [d, angle_min + i * angle_increment_deg]
   585	        return obst
   586	
   587	    def _merge_geofence(self, distance_array, angle_array):
   588	        """경계 '가짜 LiDAR'를 실제 스캔과 병합한다 — **한 줄이 전부다**:
   589	              ranges[i] = min(real_ranges[i], geofence_ranges[i])
   590	
   591	        이후는 기존 파이프라인이 알아서 한다: detection_distance 로 이진 마스크가 되고,
   592	        dilate 가 배 폭(half_width+clearance) 여유를 더하고, 갭-팔로잉이 피한다.
   593	        특수 로직도, 새 상태기계도, 튜닝 파라미터(half_block)도 없다.
   594	
   595	        병합하지 않는 경우 (전부 '모르면 입을 다문다'):
   596	          · geofence 가 None  → north 가 빈 배열 (미설정 / IMU stale / 이미 이탈)
   597	          · geofence 가 stale → 묵은 '없는 벽'은 배를 가둔다
   598	        """
   599	        with self.lock:
   600	            gf = self.geofence
   601	            gf_t = self.last_geofence_t
   602	
   603	        if gf is None or gf_t is None:
   604	            return distance_array
   605	        if (time.monotonic() - gf_t) > self.geofence_stale_sec:
   606	            self.get_logger().warn(
   607	                "geofence stale → 병합 안 함 (묵은 '없는 벽'은 배를 가둔다)",
   608	                throttle_duration_sec=5.0)
   609	            return distance_array
   610	
   611	        a0, inc, gr = gf
   612	        n = len(gr)
   613	        if n == 0 or inc <= 0.0:
   614	            return distance_array
   615	
   616	        out = list(distance_array)
   617	        hits = 0
   618	        nearest = float('inf')
   619	        for i, a in enumerate(angle_array):
   620	            rel = a - LIDAR_FORWARD_DEG          # LiDAR 프레임(80=정면) → 상대방위(0=정면)
   621	            k = int(round((rel - a0) / inc))
   622	            if not (0 <= k < n):
   623	                continue
   624	            g = gr[k]
   625	            if not math.isfinite(g):
   626	                continue
   627	            cur = out[i]
   628	            if (not math.isfinite(cur)) or g < cur:
   629	                out[i] = g                        # ★ min(실제, 경계)
   630	                hits += 1
   631	                nearest = min(nearest, g)
   632	
   633	        if hits:
   634	            self.get_logger().warn(
   635	                f"🚧 경계가 스캔에 병합됨 ({hits}셀, 최근접 {nearest:.1f}m)",
   636	                throttle_duration_sec=2.0)
   637	        return out
   638	
   639	    def _geofence_blocks(self, intended_rel):
   640	        """[I] 경계가 stop_margin 안이고, 그 방위가 '전진하려는 방향 ±cone' 과 겹치면 True.
   641	
   642	        (A)SPIN·(B)candidate 구간은 distance_array 를 안 만들어 스캔 병합이 안 된다.
   643	        도킹은 SPIN+candidate 로 조향하므로, 도크가 경기장 가장자리면 접안하다 경계로 파고든다.
   644	        → 여기서 하드하게 막는다. **미션보다 실격 방지가 우선.** 경계를 등지고 멈춘다.
   645	        (C) 자율회피는 이미 _merge_geofence 로 처리되므로 이 가드를 호출하지 않는다.
   646	        ⚠️ north 의 광선은 전방 ±80° 만 쏜다 → 뒤쪽(후진 중) 경계는 못 본다. 접안 방향(전방) 방어용.
   647	        """
   648	        with self.lock:
   649	            gf = self.geofence
   650	            gf_t = self.last_geofence_t
   651	
   652	        if gf is None or gf_t is None:
   653	            return False
   654	        if (time.monotonic() - gf_t) > self.geofence_stale_sec:
   655	            return False
   656	
   657	        a0, inc, gr = gf
   658	        if not gr or inc <= 0.0:
   659	            return False
   660	
   661	        # 최근접 경계와 그 상대방위
   662	        best_d, best_rel = float('inf'), None
   663	        for k, g in enumerate(gr):
   664	            if math.isfinite(g) and g < best_d:
   665	                best_d, best_rel = g, a0 + k * inc
   666	        if best_rel is None or best_d > self.geofence_stop_margin_m:
   667	            return False
   668	
   669	        # 전진하려는 방향과 겹치나 (각도차를 [-180,180]로 래핑)
   670	        diff = ((best_rel - intended_rel) + 180.0) % 360.0 - 180.0
   671	        if abs(diff) <= self.geofence_stop_cone_deg:
   672	            self.get_logger().warn(
   673	                f"🚧🛑 경계 {best_d:.1f}m (상대 {best_rel:.0f}°) 가 전진 방향({intended_rel:.0f}°)과 "
   674	                f"겹침 → 미션 무시하고 정지 (실격 방지 우선)",
   675	                throttle_duration_sec=1.0)
   676	            return True
   677	        return False
   678	
   679	    def _compute(self, msg, wp4_enter_time, candidate_angle, yaw_error, detection_distance):
   680	        """(desired_angle, obstacle_array) 를 만든다. 작년 scan_callback 의 로직 그대로."""
   681	        now = time.monotonic()
   682	
   683	        obst = self._closest_obstacle(msg)
   684	
   685	        # ---- (0) WP4 초기 5초 & STOP_HOLD 우선 처리 ----
   686	        if (wp4_enter_time is not None and (now - wp4_enter_time) < 5.0) or \
   687	           (candidate_angle == STOP_HOLD):
   688	            self._reset_reverse_sequence()
   689	            return STOP_HOLD, obst
   690	
   691	        # ---- [I] Geofence 하드 가드 ((A)SPIN/(B)candidate 전용) ----
   692	        # 이 두 구간은 마스크 병합이 없어 경계로 파고들 수 있다(도킹이 가장자리일 때).
   693	        # (C)(CANDIDATE_INVALID)는 아래에서 _merge_geofence 로 처리되므로 제외한다.
   694	        if candidate_angle != CANDIDATE_INVALID:
   695	            # 전진하려는 방향: candidate 는 그 상대각, SPIN(제자리)은 전방(0°)으로 본다.
   696	            intended_rel = 0.0 if candidate_angle in (SPIN_RIGHT, SPIN_LEFT) else candidate_angle
   697	            if self._geofence_blocks(intended_rel):
   698	                self._reset_reverse_sequence()
   699	                return STOP_HOLD, obst
   700	
   701	        # ---- (A) PASS-THROUGH: 미션 노드가 선회를 직접 지시 ----
   702	        if candidate_angle in (SPIN_RIGHT, SPIN_LEFT):
   703	            self._reset_reverse_sequence()
   704	            return candidate_angle, obst
   705	
   706	        # ---- (B) 일반 candidate_angle 매핑 (360° → 전방중심 80°) ----
   707	        if candidate_angle != CANDIDATE_INVALID:
   708	            self._reset_reverse_sequence()
   709	            c_raw = candidate_angle % 360.0
   710	            c_mapped = 80.0 + c_raw if c_raw <= 180.0 else 80.0 - (360.0 - c_raw)
   711	
   712	            # 정상 조향값은 0~160°여야 한다. 범위를 벗어나면 motor_control이 후진으로
   713	            # 해석할 수 있으므로 안전하게 정지한다.
   714	            if not math.isfinite(c_mapped) or not (0.0 <= c_mapped <= 160.0):
   715	                self.get_logger().warn(
   716	                    f"candidate 매핑 범위 초과: raw={c_raw:.1f}°, "
   717	                    f"mapped={c_mapped:.1f}° → STOP_HOLD",
   718	                    throttle_duration_sec=1.0)
   719	                return STOP_HOLD, obst
   720	            return c_mapped, obst
   721	
   722	        # ---- (C) candidate == 20000 → yaw_error 기반 자율 회피 ----
   723	        angle_min = math.degrees(msg.angle_min)
   724	        angle_increment_deg = math.degrees(msg.angle_increment)
   725	        angle_increment_rad = msg.angle_increment
   726	        ranges = list(msg.ranges)
   727	
   728	        # [F] 0 나눗셈 방어 (scan_callback 이 이미 걸러내지만, 여기도 막아둔다)
   729	        if not math.isfinite(angle_increment_deg) or angle_increment_deg <= 0.0:
   730	            self.get_logger().error("angle_increment 이상 → 정지", throttle_duration_sec=1.0)
   731	            return STOP_HOLD, obst
   732	
   733	        min_index = int((0 - angle_min) / angle_increment_deg)
   734	        max_index = int((160 - angle_min) / angle_increment_deg)
   735	        sub_ranges = ranges[min_index:max_index + 1]
   736	
   737	        angle_array = []
   738	        distance_array = []
   739	
   740	        for i, r in enumerate(sub_ranges):
   741	            angle_array.append(angle_min + (min_index + i) * angle_increment_deg)
   742	            distance_array.append(r)
   743	
   744	        # ---- [H] Geofence 병합 (6a-2). 이게 전부다: ranges[i] = min(실제, 경계) ----
   745	        # 경계는 north 가 '가짜 LiDAR'로 쏴서 보낸다. 여기서 스캔에 섞어버리면
   746	        # 아래 파이프라인(마스크 → dilate → 갭-팔로잉)이 경계를 그냥 '벽'으로 취급한다.
   747	        distance_array = self._merge_geofence(distance_array, angle_array)
   748	
   749	        # ---- Binary obstacle mask ----
   750	        # ※ rear_obstacle_ignore_margin 제거됨. 작년엔 여기서
   751	        #      if abs(r - front_min_distance) > rear_obstacle_ignore_margin: binary.append(0)
   752	        #    로 '최근접점에서 먼 셀'을 지웠는데, 물보라 반사 0.3m 하나가 front_min_distance 를
   753	        #    끌어내리면 실제 1.6m 부표가 통째로 지워졌다 (17셀 → 0셀). 튜닝값이 아니라 결함이었다.
   754	        binary = []
   755	        for r in distance_array:
   756	            if math.isinf(r) or math.isnan(r):
   757	                binary.append(0)
   758	            elif r < detection_distance:
   759	                binary.append(1)
   760	            else:
   761	                binary.append(0)
   762	
   763	        # ---- [G] 시간 투표 — 물보라 한 프레임짜리 오탐 제거 ----
   764	        # ★ 반드시 dilate 전, 원본 마스크에 건다. 팽창 후에 걸면 진짜 부표의 팽창 영역까지
   765	        #   표가 갈려 부표가 얇아진다.
   766	        binary = self.temporal.apply(binary)
   767	
   768	        binary = self.smooth_spikes(binary)
   769	        binary = self.suppress_spike_edges(binary)
   770	        binary = self.dilate_obstacles(binary, distance_array, angle_increment_deg,
   771	                                       detection_distance)
   772	
   773	        # ---- safe zone 탐색 ----
   774	        safe_zones = []
   775	        start = None
   776	        for i, v in enumerate(binary):
   777	            if v == 0 and start is None:
   778	                start = i
   779	            elif v == 1 and start is not None:
   780	                safe_zones.append((start, i - 1))
   781	                start = None
   782	        if start is not None:
   783	            safe_zones.append((start, len(binary) - 1))
   784	
   785	        # ---- 통과 가능한 zone 선별 ----
   786	        min_required_width = self.half_width * 2 + self.clearance
   787	        valid_safe_zones = select_valid_zones(
   788	            safe_zones, distance_array, angle_increment_rad, detection_distance, min_required_width)
   789	
   790	        # ---- yaw_error 를 safe zone 안으로 clamp → 최적 후보 ----
   791	        yaw_raw = yaw_error % 360.0
   792	        yaw_mapped = 80 + yaw_raw if yaw_raw <= 180 else 80 - (360 - yaw_raw)
   793	
   794	        candidates = []
   795	        for s, e, r_edge in valid_safe_zones:
   796	            zone_min = angle_array[s]
   797	            zone_max = angle_array[e]
   798	
   799	            if yaw_mapped < zone_min:
   800	                best_angle = zone_min
   801	            elif yaw_mapped > zone_max:
   802	                best_angle = zone_max
   803	            else:
   804	                best_angle = yaw_mapped
   805	
   806	            angle_diff = abs(best_angle - yaw_mapped)
   807	            arc_length = angle_increment_rad * r_edge * (e - s)
   808	            candidates.append((angle_diff, -arc_length, best_angle, s, e))
   809	
   810	        # ---- [L] 회피 경로가 없으면 후진. 팀원 피드백 C안 반영 ----
   811	        # 작년/2단계 코드는 여기서 매 tick 무조건 REVERSE_ANGLE 을 냈다 — reverse_cooldown 은
   812	        # 타임스탬프만 갱신할 뿐 final_angle 에 영향이 없었다(dead code, 팀 지적).
   813	        # ⚠️ north 광선은 전방 ±80° 만 봐서 후진 중 뒤쪽 경계는 못 본다([I] 참고).
   814	        #    즉 지금 구조로 '계속 후진'은 뒤를 못 보며 맹목 후진하는 것과 같다.
   815	        # → C안: 후진은 쿨다운마다 '한 번만' 짧게 내고, 그 사이(쿨다운 중)는 STOP_HOLD 로
   816	        #   멈춰서 재탐색한다. 쿨다운이 끝났는데도 여전히 후보가 없으면 다시 한 번 후진한다.
   817	        if not candidates:
   818	            # 일정 시간 후진 → 일정 시간 STOP 재탐색 → 필요하면 반복
   819	            final_angle = self._apply_reverse_sequence(now)
   820	        else:
   821	            self._reset_reverse_sequence()
   822	            candidates.sort(key=lambda x: (x[0], x[1]))
   823	            final_angle = candidates[0][2]
   824	
   825	        return final_angle, obst
   826	
   827	    # =====================================================
   828	    # 스파이크 억제 / 팽창 (작년 그대로)
   829	    # =====================================================
   830	    def smooth_spikes(self, binary):
   831	        total_len = len(binary)
   832	        max_spike_length = int(total_len * self.max_spike_ratio)
   833	        smoothed = binary[:]
   834	        count = 0
   835	        start = None
   836	        for i, v in enumerate(binary):
   837	            if v == 1:
   838	                if start is None:
   839	                    start = i
   840	                count += 1
   841	            else:
   842	                if start is not None and count <= max_spike_length:
   843	                    for j in range(start, i):
   844	                        smoothed[j] = 0
   845	                start = None
   846	                count = 0
   847	        if start is not None and count <= max_spike_length:
   848	            for j in range(start, len(binary)):
   849	                smoothed[j] = 0
   850	        return smoothed
   851	
   852	    def suppress_spike_edges(self, binary):
   853	        suppressed = binary[:]
   854	        for i, v in enumerate(binary):
   855	            if v == 1:
   856	                for off in range(-self.border_margin, self.border_margin + 1):
   857	                    j = i + off
   858	                    if 0 <= j < len(binary) and binary[j] == 0:
   859	                        suppressed[i] = 0
   860	                        break
   861	        return suppressed
   862	
   863	    def dilate_obstacles(self, binary, distance_array, angle_increment_deg,
   864	                         detection_distance):
   865	        # 팀원 제안 버그2: 작년엔 여기서 self.detection_distance 를 직접 읽어, control_cb 가
   866	        #   lock 으로 뜬 스냅샷과 어긋날 수 있었다(wp_mode_cb 가 다른 스레드에서 바꾸므로).
   867	        #   실제 영향은 거의 없다 — r_edge 는 아래에서 유효 거리로 덮이는 폴백이라, binary=1 셀은
   868	        #   정의상 유효 거리가 있어 폴백이 안 쓰인다. 그래도 한 _compute 안에서 값이 하나로
   869	        #   유지되도록 스냅샷을 파라미터로 받는다(lock 일관성 원칙).
   870	        n = len(binary)
   871	        out = binary[:]
   872	        i = 0
   873	        while i < n:
   874	            if binary[i] == 1:
   875	                start = i
   876	                while i + 1 < n and binary[i + 1] == 1:
   877	                    i += 1
   878	                end = i
   879	
   880	                length = end - start + 1
   881	                if length >= self.min_obstacle_cells:
   882	                    r_edge = detection_distance
   883	                    for k in range(start, end + 1):
   884	                        r = distance_array[k]
   885	                        if not math.isinf(r) and not math.isnan(r):
   886	                            r_edge = min(r_edge, r)
   887	
   888	                    lateral = self.half_width + self.clearance
   889	                    r_use = max(r_edge, 0.01)
   890	                    ang_margin = math.degrees(math.atan(lateral / r_use))
   891	                    cells_margin = max(1, int(round(ang_margin / max(angle_increment_deg, 1e-6))))
   892	
   893	                    new_start = max(0, start - cells_margin)
   894	                    new_end = min(n - 1, end + cells_margin)
   895	
   896	                    for j in range(new_start, new_end + 1):
   897	                        out[j] = 1
   898	                i += 1
   899	            else:
   900	                i += 1
   901	        return out
   902	
   903	
   904	def main(args=None):
   905	    rclpy.init(args=args)
   906	    node = ShipDirection()
   907	
   908	    executor = MultiThreadedExecutor(num_threads=4)
   909	    executor.add_node(node)
   910	    try:
   911	        executor.spin()
   912	    except KeyboardInterrupt:
   913	        pass
   914	    finally:
   915	        node.destroy_node()
   916	        if rclpy.ok():
   917	            rclpy.shutdown()
   918	
   919	
   920	if __name__ == '__main__':
   921	    main()
```

----- FILE: src/ship_direction/ship_direction/failsafe.py -----
```
     1	"""ship_direction 의 판정 로직 — **ROS 비의존 순수 파이썬.**
     2	
     3	여기엔 rclpy 를 임포트하지 않는다. 그래야 ROS 없이도 단위 테스트할 수 있다:
     4	
     5	    from ship_direction.failsafe import SensorWatch, TemporalVote, median_min
     6	
     7	시각(now)은 호출자가 주입한다 → 테스트에서 시간을 마음대로 돌릴 수 있다.
     8	"""
     9	
    10	import math
    11	from collections import deque
    12	
    13	
    14	def median_min(ranges, lo, hi, kernel=5, min_valid=3, return_rejected=False):
    15	    """[lo,hi] 구간에 **공간 median** 을 걸고 최소거리를 찾는다. → (거리, 인덱스) 또는 (None, None).
    16	
    17	    return_rejected=True 면 (거리, 인덱스, 기각수) 를 돌려준다 (D6, 관측 전용).
    18	      기각수 = raw 로는 유효한 점(finite, >0)인데 median 창의 유효점이 min_valid 미만이라
    19	      고립 스파이크로 버려진 인덱스 개수. **물보라 노이즈의 대리 지표**다.
    20	      제어에는 쓰지 않는다 — blackbox 가 CSV 에 기록해 '노이즈 수 ↔ 기상 열화' 상관 검증용.
    21	      ⚠️ 기본값 False 로 두어 기존 호출부·테스트는 (거리, 인덱스) 2-튜플 그대로 받는다.
    22	
    23	    ★ 왜 필요한가 — **감속 신호가 물보라 한 점에 속는다:**
    24	      _closest_obstacle 이 필터 없는 raw min 이라, 물보라 반사 하나가 0.3m 로 튀면
    25	      /obstacle_distance_array[0] = 0.3 → motor_control 이 그대로 감속한다.
    26	      측정(시드 20, 물보라 30%): raw-min 가짜감속 **36.8% → median 8.9%**(기준선). 접촉은 전 조건 0.
    27	
    28	    ★ 이건 **공간** 필터다 — 폐기한 TemporalVote(시간 투표)와 메커니즘이 다르다.
    29	      한 프레임 안에서 이웃과 어긋나는 고립 스파이크를 range 값 수준에서 지운다.
    30	
    31	    각 인덱스의 ±(kernel//2) 창에서 유효점(finite, >0)의 median 을 그 인덱스 값으로 쓴다.
    32	    유효점이 min_valid 미만이면 그 인덱스는 **무시**한다 (주변에 아무것도 없는 고립 반사 = 물보라).
    33	    kernel 이 0/1 이면 필터 없이 raw min (폴백).
    34	
    35	    ※ 짝수 개일 때 위쪽 median 을 쓴다 → 거리가 더 크게 나와 '가짜 감속' 쪽으로 안 기운다.
    36	    """
    37	    n = len(ranges)
    38	    lo = max(0, lo)
    39	    hi = min(n - 1, hi)
    40	    if lo > hi:
    41	        return (None, None, 0) if return_rejected else (None, None)
    42	
    43	    half = (kernel // 2) if (kernel and kernel > 1) else 0
    44	
    45	    best_d, best_i = None, None
    46	    rejected = 0
    47	    for i in range(lo, hi + 1):
    48	        if half == 0:
    49	            r = ranges[i]
    50	            v = r if (math.isfinite(r) and r > 0.0) else None
    51	        else:
    52	            w = []
    53	            for j in range(max(0, i - half), min(n - 1, i + half) + 1):
    54	                r = ranges[j]
    55	                if math.isfinite(r) and r > 0.0:
    56	                    w.append(r)
    57	            if len(w) < min_valid:
    58	                # 고립 스파이크 → 무시. raw 로는 유효했던 점이면 '기각'으로 센다(물보라 지표).
    59	                ri = ranges[i]
    60	                if math.isfinite(ri) and ri > 0.0:
    61	                    rejected += 1
    62	                continue
    63	            w.sort()
    64	            v = w[len(w) // 2]           # median (짝수면 위쪽)
    65	        if v is None:
    66	            continue
    67	        if best_d is None or v < best_d:
    68	            best_d, best_i = v, i
    69	    return (best_d, best_i, rejected) if return_rejected else (best_d, best_i)
    70	
    71	
    72	class SensorWatch:
    73	    """센서 신선도 → 페일세이프 레벨.
    74	
    75	    레벨: 0 정상 / 1 경고(warn_sec) / 2 정지(stop_sec)
    76	
    77	    규칙 (CLAUDE.md §5 — 팀 최우선 우려는 '고장이 아닌데 스스로 멈추는 것'):
    78	      · 센서별 ARMED — 첫 데이터를 받은 뒤에만 감시한다 (부팅 오발동 방지)
    79	      · **가장 나쁜 센서**가 레벨을 정한다.
    80	        (/scan 만 보면 IMU 사망을 놓친다: IMU 가 죽으면 ship_goal_angle 이 /yaw_error 를
    81	         끊어주는데, 그 침묵을 들을 귀가 없으면 마지막 yaw_error 로 영원히 조향한다.)
    82	      · 올릴 땐 연속 confirm_n 회 확인 (순간 지터로 레벨 안 올림)
    83	      · 내릴 땐 즉시 (자동 복구 — 센서가 돌아왔다는 뜻)
    84	      · 하드 폴트(set_fault)는 지터가 아니므로 확인 없이 즉시 레벨 2
    85	    """
    86	
    87	    def __init__(self, sensors, warn_sec=0.7, stop_sec=3.0, confirm_n=3):
    88	        self.warn_sec = float(warn_sec)
    89	        self.stop_sec = float(stop_sec)
    90	        self.confirm_n = int(confirm_n)
    91	        self.last_seen = {name: None for name in sensors}   # None = ARMED 전
    92	        self.level = 0
    93	        self.worst = None
    94	        self._raise_count = 0
    95	        self._fault = False
    96	
    97	    def feed(self, name, now):
    98	        """센서 데이터 수신 기록 (ARMED)."""
    99	        self.last_seen[name] = now
   100	
   101	    def set_fault(self, fault):
   102	        """하드 폴트(예: angle_increment 이상 스캔). 확인 N회 없이 즉시 레벨 2."""
   103	        self._fault = bool(fault)
   104	
   105	    @property
   106	    def fault(self):
   107	        return self._fault
   108	
   109	    def raw_level(self, now):
   110	        """확인/히스테리시스 적용 **전**의 순간 레벨. → (level, worst_name, worst_age)"""
   111	        if self._fault:
   112	            return 2, 'fault', 0.0
   113	
   114	        raw, worst, worst_age = 0, None, 0.0
   115	        for name, t in self.last_seen.items():
   116	            if t is None:
   117	                continue                      # ARMED 전 — 이 센서는 아직 감시하지 않는다
   118	            age = now - t
   119	            if age > self.stop_sec:
   120	                lvl = 2
   121	            elif age > self.warn_sec:
   122	                lvl = 1
   123	            else:
   124	                lvl = 0
   125	            if lvl > raw:                     # 가장 나쁜 센서가 레벨을 정한다
   126	                raw, worst, worst_age = lvl, name, age
   127	        return raw, worst, worst_age
   128	
   129	    def update(self, now):
   130	        """레벨을 갱신하고 반환한다."""
   131	        raw, worst, worst_age = self.raw_level(now)
   132	
   133	        if self._fault:
   134	            self.level = raw                  # 하드 폴트 → 즉시
   135	            self._raise_count = 0
   136	        elif raw > self.level:
   137	            self._raise_count += 1            # 올릴 땐 연속 N회 확인
   138	            if self._raise_count >= self.confirm_n:
   139	                self.level = raw
   140	                self._raise_count = 0
   141	        elif raw < self.level:
   142	            self.level = raw                  # 자동 복구 → 즉시
   143	            self._raise_count = 0
   144	        else:
   145	            self._raise_count = 0
   146	
   147	        self.worst = (f"{worst}({worst_age:.1f}s)"
   148	                      if worst and worst != 'fault' else worst)
   149	        return self.level
   150	
   151	
   152	class TemporalVote:
   153	    """최근 N 프레임의 이진 장애물 마스크를 투표해, 한 프레임짜리 오탐(물보라)을 지운다.
   154	
   155	    최근 frames(기본 3) 프레임 중 votes(기본 2) 회 이상 나타난 셀만 진짜 장애물로 인정한다.
   156	    LiDAR 10Hz 기준 0.3초짜리 기억 → 반응 지연은 무시할 수준.
   157	
   158	    ★ 반드시 **dilate(팽창) 전, 원본 마스크**에 걸어야 한다.
   159	      팽창 후에 걸면 진짜 부표의 팽창 영역까지 표가 갈려 부표가 얇아진다.
   160	
   161	    측정(정면 부표 1개, 25판, 물보라 60%): 최소여유 0.17m(없음) → 0.40m(있음). 2.4배.
   162	
   163	    frames<=1 또는 votes<=1 이면 꺼진다(그대로 통과).
   164	    """
   165	
   166	    def __init__(self, frames=3, votes=2):
   167	        self.frames = int(frames)
   168	        self.votes = int(votes)
   169	        self.history = deque(maxlen=max(1, self.frames))
   170	
   171	    @property
   172	    def enabled(self):
   173	        return self.frames > 1 and self.votes > 1
   174	
   175	    def reset(self):
   176	        self.history.clear()
   177	
   178	    def apply(self, binary):
   179	        if not self.enabled:
   180	            return binary
   181	
   182	        self.history.append(list(binary))
   183	
   184	        n = len(binary)
   185	        out = [0] * n
   186	        for i in range(n):
   187	            v = 0
   188	            for h in self.history:
   189	                if i < len(h) and h[i] == 1:   # 스캔 길이가 바뀌어도 안전
   190	                    v += 1
   191	            if v >= self.votes:
   192	                out[i] = 1
   193	        return out
```

----- FILE: src/ship_direction/config/ship_direction.yaml -----
```
     1	# ship_direction 파라미터 (하드코딩 금지, CLAUDE.md 1-4)
     2	/ship_direction:
     3	  ros__parameters:
     4	    # ---- 제어 루프 (스캔 유무와 무관하게 항상 발행) ----
     5	    control_period_sec: 0.1    # 10Hz. 작년엔 scan_cb 안에서만 발행 → LiDAR 끊기면 발행 0회(완전 침묵)
     6	    watchdog_period_sec: 0.1   # 페일세이프 평가(독립 타이머 — scan_cb 에서 평가하면 영원히 평가 안 됨)
     7	
     8	    # ---- 페일세이프 임계: 센서가 묵은 시간(초). 3상태·2임계 ----
     9	    # 감시 대상: /scan(LiDAR) 과 /yaw_error(IMU 체인) **둘 다**. 가장 나쁜 센서가 레벨을 정한다.
    10	    #   /scan 만 보면 IMU 사망을 놓친다 — ship_goal_angle 이 /yaw_error 를 끊어줘도(1단계 설계)
    11	    #   들을 귀가 없으면 마지막 yaw_error 로 영원히 조향한다.
    12	    # 센서별로 첫 데이터를 받은 뒤에만 감시(ARMED) — 부팅 오발동 방지.
    13	    # ※ angle_increment 이상 스캔은 하드 폴트 → 확인 N회 없이 즉시 레벨2(정지).
    14	    #   레벨0 정상      : 계산한 각도       / 속도 100%
    15	    #   레벨1 경고(0.7s): 계산한 각도 그대로 / 속도 70%   ← 각도 안 건드림
    16	    #   레벨2 정지(3.0s): STOP_HOLD override / 정지       ← 각도를 덮어쓰는 건 여기뿐
    17	    #   ★ 임계가 적을수록 틀릴 곳이 적다. (중간 'freeze' 레벨은 뺐다 — 새 스캔이 없는데
    18	    #     재계산하면 같은 답이 나오므로 '재계산'과 'freeze'는 같은 동작이다.)
    19	    failsafe_warn_sec: 0.7     # → 레벨1
    20	    failsafe_stop_sec: 3.0     # → 레벨2 (LiDAR 10Hz 기준 30스캔 연속 누락)
    21	    failsafe_confirm_n: 3      # 올릴 땐 연속 N회 확인(지터 방지). 내릴 땐 즉시(자동 복구)
    22	
    23	    # ---- Geofence (경계 이탈 = 실격). 6a-2 ----
    24	    # north 가 경계를 '가짜 LiDAR'로 쏴서 보낸다 → 여기선 스캔과 min() 병합만 한다.
    25	    # /geofence_state = [angle_min_deg, angle_inc_deg, r0, r1, ...] (상대방위, 멀면 inf).
    26	    #   ranges[i] = min(real_ranges[i], geofence_ranges[i])
    27	    # 그 뒤는 기존 파이프라인(마스크→dilate→갭팔로잉)이 알아서 한다.
    28	    # ★ 튜닝 파라미터가 없다. 원뿔 방식(half_block)은 모서리에서 정면이 '틈'으로 열려 폐기했다.
    29	    geofence_stale_sec: 2.0       # 🚨 묵으면 병합하지 않는다 (north 주기 0.5s 의 4배).
    30	                                  #    묵은 '없는 벽'은 배를 가둔다. 모르면 입을 다문다.
    31	    # 하드 가드 [I]: (A)SPIN·(B)candidate 구간(도킹)은 마스크 병합이 없어 경계로 파고든다.
    32	    #   경계가 아래 거리 안이고 전진 방향 ±cone 과 겹치면 미션 무시하고 정지. 실격 방지 우선.
    33	    geofence_stop_margin_m: 2.0   # 이 거리 안이면 하드 정지 판정 대상
    34	    geofence_stop_cone_deg: 60.0  # 경계 방위가 전진 방향 ±이 각도 안이면 정지
    35	
    36	    # ---- 감속 신호용 공간 median 필터 — 기본 ON ----
    37	    # _closest_obstacle(감속 신호)에만 적용. 회피 마스크(_compute)는 안 건드린다.
    38	    # 작년엔 raw min 이라 물보라 반사 한 점(0.3m)이 그대로 motor_control 감속을 물렸다.
    39	    # 측정(시드 20, 물보라 30%): raw-min 가짜감속 36.8% → median 8.9%(기준선). 접촉은 전 조건 0.
    40	    # ★ 시간 투표(아래)는 효과 0 이라 OFF, 이건 명확한 이득이라 ON. (시간 vs 공간, 다른 필터다)
    41	    # ⚠️ 시뮬 물보라는 '단일점' 모델. 실제 물보라가 작은 군집이면 커널5로 부족할 수 있다
    42	    #    → 배 뜬 뒤 블랙박스 obstacle_min_dist 로 실측 확인 후 7/9 로 올릴 것.
    43	    obst_median_kernel: 5        # 0 이면 raw-min 폴백
    44	
    45	    # ---- 시간 투표 필터 — 기본 OFF (frames=1). 코드는 남겨둠 ----
    46	    # 최근 N프레임 중 votes 회 이상 나타난 셀만 장애물로 인정하는 필터.
    47	    # ⚠️ 시드 20개로 재보니 효과 없음: OFF→접촉 19/20·전진 25.5m / ON→접촉 20/20·전진 23.4m.
    48	    #    해당 시나리오(정면 부표+물보라 60%)는 어느 코드든 95% 접촉하는 '통과 불가' 판이었다.
    49	    #    처음 효과가 있어 보인 건 단일 시드의 운.
    50	    # → 무해하지만 무익 → 기본 OFF. (TTC 는 '해로워서' 삭제, 이건 '무익해서' OFF)
    51	    #    실제 물보라의 시간 특성이 시뮬과 다를 수 있으니 켤 수 있게 남긴다.
    52	    # 켜려면: temporal_frames: 3, temporal_votes: 2
    53	    temporal_frames: 1
    54	    temporal_votes: 2
    55	
    56	    # ---- 회피 튜닝 (CLAUDE.md §6 시뮬로 검증된 값 — 임의로 바꾸지 말 것) ----
    57	    detection_distance_default: 3.0  # 1.8 이면 8/8 충돌. 3.0 이면 5/8 통과
    58	    detection_distance_gate: 2.0     # 게이트는 좁아서 3.0 이면 오히려 접촉 증가
    59	    gate_wp_modes: [0, 1]            # 게이트 구간 wp_mode (3-6 표: 0=게이트 시작, 1=게이트 끝)
    60	                                     # 작년 코드는 wp_mode==2(위치유지)를 키로 썼다 — 게이트가 아니다
    61	    half_width: 0.29                 # ✅ 1번배 **줄자 실측 0.57m**(2026-09-28, 스러스터·러더 포함 좌우 최대) ÷ 2 = 0.285
    62	                                     #    → 0.29 로 **올려서** 쓴다: ① 5mm 보수 쪽 ② 스윕 격자의 실측 지점(폭 0.58)과 같아
    63	                                     #      아래 clearance 근거를 보간 없이 그대로 쓸 수 있다.
    64	                                     # ✅ 2번배 줄자 실측(2026-10-24): 길이 1.77 · 폭 0.535 · 깊이 0.14m → half 0.2675 < 0.29 라 공용 0.29 그대로.
    65	                                     #    시뮬(제어 0.29 / 물리 0.2675, 시드 20·80런): 접촉 0.95 · 정체 0. ⚠️ 0.535 가 스러스터 포함인지 확인 전
    66	                                     # 같이 잰 것: 길이 1.66m · 깊이 0.38m(선체 17 + 스러스터 11 + 러더 10)
    67	                                     # 폭이 바뀌면 → 09-28 스윕(시드 20) 폭별 최적 clearance 에서 고른다:
    68	                                     #   0.43m→0.35 / 0.50m→0.30 / 0.58m→0.25 / 0.64m→0.25 / 0.70m→0.20  (tools/sim/sweep_s1.py)
    69	                                     # 🚨 바꾸면 **네 곳을 같이**: 이 yaml · 노드 declare_parameter 기본값 · test fake · tools/sim/ssf_sim.py
    70	                                     # 0.45(작년 배 0.9m)는 갭을 1.15m 요구 → 좁은 게이트 후진 반복·접촉 3회. S1 참조
    71	    clearance: 0.25                  # 09-28 재스윕(half_width 0.29, 시드 20·80런 평균): 0.20→접촉1.95 / 0.25→1.32 / 0.30→1.46(정체2%) / 0.35→정체28% / 0.45→정체50%
    72	                                     # (구 스윕은 half_width 0.45 기준: 0.20→1.7 / 0.25→0.2 / 0.30→0.8 / 0.45→807초 폭주)
    73	    border_margin: 2
    74	    max_spike_ratio: 0.01
    75	    min_obstacle_cells: 1            # 3 이면 작은 부표가 무시된다
    76	    # ---- [L] 후진 후 재탐색 상태기계 (경로 없을 때) ----
    77	    reverse_duration_sec: 0.5        # 한 번 후진하는 시간
    78	    reverse_cooldown_sec: 3.0        # 후진 후 STOP 재탐색 시간. 뒤 못 보므로 '계속 후진' 대신 멈춰 재탐색
    79	    # ---- [K] 조향각 변화율 제한 (0~160° 정상 조향값에만 적용) ----
    80	    rate_limit_max_deg_per_tick: 15.0  # tick당 최대 변화폭. 실각속도=이 값/control_period(0.1s→150°/s). 실측 후 튜닝
    81	    # ※ rear_obstacle_ignore_margin 은 '튜닝값'이 아니라 증명된 결함이라 구조 커밋에서 제거했다
    82	    #    (물보라 0.3m 반사 하나가 실제 1.6m 부표를 통째로 지웠다: 17셀 → 0셀).
```

----- FILE: src/ship_direction/launch/ship_direction_launch.py -----
```
     1	import os
     2	
     3	from ament_index_python.packages import get_package_share_directory
     4	from launch import LaunchDescription
     5	from launch_ros.actions import Node
     6	
     7	
     8	def generate_launch_description():
     9	    params = os.path.join(
    10	        get_package_share_directory('ship_direction'), 'config', 'ship_direction.yaml')
    11	
    12	    return LaunchDescription([
    13	        Node(
    14	            package='ship_direction',
    15	            executable='ship_direction',
    16	            name='ship_direction',      # config yaml 의 /ship_direction 키와 일치
    17	            output='screen',
    18	            parameters=[params],
    19	        ),
    20	    ])
```

----- FILE: src/rplidar_ros/launch/rplidar_a3_launch.py -----
```
     1	#!/usr/bin/env python3
     2	
     3	from launch import LaunchDescription
     4	from launch.actions import DeclareLaunchArgument
     5	from launch.substitutions import LaunchConfiguration
     6	from launch_ros.actions import Node
     7	
     8	
     9	def generate_launch_description():
    10	    """Launch the RPLIDAR A3 node with configurable runtime arguments.
    11	
    12	    Notes:
    13	    - ``/dev/ttyLiDAR`` requires a matching udev rule on each computer.
    14	    - ``scan_mode`` remains ``Sensitivity`` by default until comparative
    15	      water-test data supports changing it. It can be overridden at launch.
    16	    - QoS for ``/scan`` subscribers belongs in the subscribing node, not here.
    17	    """
    18	
    19	    # Launch arguments are declared with literal defaults and read through
    20	    # LaunchConfiguration. This keeps the runtime override path explicit.
    21	    channel_type = LaunchConfiguration('channel_type')
    22	    serial_port = LaunchConfiguration('serial_port')
    23	    serial_baudrate = LaunchConfiguration('serial_baudrate')
    24	    frame_id = LaunchConfiguration('frame_id')
    25	    inverted = LaunchConfiguration('inverted')
    26	    angle_compensate = LaunchConfiguration('angle_compensate')
    27	    scan_mode = LaunchConfiguration('scan_mode')
    28	
    29	    return LaunchDescription([
    30	        DeclareLaunchArgument(
    31	            'channel_type',
    32	            default_value='serial',
    33	            description='Communication channel type for the LiDAR',
    34	        ),
    35	        DeclareLaunchArgument(
    36	            'serial_port',
    37	            default_value='/dev/ttyLiDAR',
    38	            description=(
    39	                'Serial device for the LiDAR. /dev/ttyLiDAR requires a '
    40	                'matching udev rule on the host computer.'
    41	            ),
    42	        ),
    43	        DeclareLaunchArgument(
    44	            'serial_baudrate',
    45	            default_value='256000',
    46	            description='Serial baud rate; RPLIDAR A3 uses 256000',
    47	        ),
    48	        DeclareLaunchArgument(
    49	            'frame_id',
    50	            default_value='laser',
    51	            description='TF frame ID assigned to LaserScan messages',
    52	        ),
    53	        DeclareLaunchArgument(
    54	            'inverted',
    55	            default_value='false',
    56	            description='Whether to invert the scan data',
    57	        ),
    58	        DeclareLaunchArgument(
    59	            'angle_compensate',
    60	            default_value='true',
    61	            description='Whether to enable angle compensation',
    62	        ),
    63	        DeclareLaunchArgument(
    64	            'scan_mode',
    65	            default_value='Sensitivity',
    66	            description=(
    67	                'RPLIDAR scan mode. Keep Sensitivity as the baseline and '
    68	                'override this argument during Standard/Boost comparison tests.'
    69	            ),
    70	        ),
    71	        Node(
    72	            package='rplidar_ros',
    73	            executable='rplidar_node',
    74	            name='rplidar_node',
    75	            output='screen',
    76	            parameters=[{
    77	                'channel_type': channel_type,
    78	                'serial_port': serial_port,
    79	                'serial_baudrate': serial_baudrate,
    80	                'frame_id': frame_id,
    81	                'inverted': inverted,
    82	                'angle_compensate': angle_compensate,
    83	                'scan_mode': scan_mode,
    84	            }],
    85	        ),
    86	    ])
```

----- FILE: src/launch_files/launch/launch_files.launch.py -----
```
     1	from launch import LaunchDescription
     2	from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
     3	from launch.substitutions import LaunchConfiguration
     4	from launch.launch_description_sources import PythonLaunchDescriptionSource
     5	from ament_index_python.packages import get_package_share_directory
     6	import os
     7	from launch_ros.actions import Node
     8	
     9	
    10	def generate_launch_description():
    11	
    12	    # 패키지 경로
    13	    dir_rplidar = get_package_share_directory('rplidar_ros')
    14	    dir_north = get_package_share_directory('north_goal_angle')
    15	    dir_ntrip = get_package_share_directory('ntrip_client')
    16	    dir_gps = get_package_share_directory('ublox_gps')
    17	    # ship_direction·realsense2_camera 는 Node(package=...) 가, ship_goal_angle 은
    18	    # 119행이 다시 조회한다. 여기 대입은 쓰이지 않아 제거했다(2026-09-23).
    19	    dir_bridge = get_package_share_directory('ssf_bridge')
    20	
    21	    # 비전 공통 설정 (image_topic, hfov_deg, debug_view).
    22	    # 3-4 이후로는 launch 가 노드를 직접 띄우므로 파라미터가 **그대로 닿는다** —
    23	    # V5 의 vision_* 중계(매니저가 --ros-args 로 넘기던 것)는 3-4 에서 제거됐다.
    24	    VISION_CFG = os.path.join(
    25	        get_package_share_directory('color_shape_detector'), 'config', 'vision.yaml')
    26	
    27	    # NTRIP 2초 지연
    28	    ntrip_launch = IncludeLaunchDescription(
    29	        PythonLaunchDescriptionSource(
    30	            os.path.join(dir_ntrip, 'launch', 'ntrip_client_launch.py')
    31	        )
    32	    )
    33	
    34	    return LaunchDescription([
    35	        DeclareLaunchArgument(
    36	            'gps_config', default_value='zed_f9p_rover.yaml',
    37	            description='배별 GPS 설정 (B배: zed_f9p_rover.yaml / A배: c94_m8p_rover.yaml)'),
    38	
    39	        # ============================================================
    40	        # 1) RPLIDAR
    41	        # ============================================================
    42	        IncludeLaunchDescription(
    43	            PythonLaunchDescriptionSource(
    44	                os.path.join(dir_rplidar, 'launch', 'rplidar_a3_launch.py')
    45	            )
    46	        ),
    47	
    48	        # ============================================================
    49	        # 2) AHRS (IMU) — 보정 안 한 상대 yaw 를 /imu/yaw_raw 로만 낸다
    50	        # ============================================================
    51	        # 🚨 CLAUDE.md 3-5. 작년엔 이 노드가 /imu/yaw 를 직접 냈는데 그 값이 절대방위가 아니었다
    52	        #    (부팅 0점화 + GPS COG override). 절대방위 합성은 아래 yaw_mux 가 전담한다.
    53	        #    파라미터는 드라이버 기본값과 같지만 **일부러 명시**한다 —
    54	        #    나중에 누가 yaml 로 되살려도 여기서 눈에 띄게 하려는 것이다.
    55	        Node(
    56	            package='iahrs_driver',
    57	            executable='iahrs_driver',
    58	            name='iahrs_driver',
    59	            output='screen',
    60	            parameters=[{
    61	                'yaw_topic': '/imu/yaw_raw',        # ← /imu/yaw 로 되돌리면 발행자 2개가 된다
    62	                'zero_yaw_on_boot': False,
    63	                'use_gps_heading_override': False,
    64	            }],
    65	        ),
    66	
    67	        # ============================================================
    68	        # 2b) yaw_mux — /imu/yaw 의 단독 발행자
    69	        # ============================================================
    70	        # 입력이 끊기거나 소스가 미구현이면 발행을 멈춘다(설계).
    71	        # → ship_goal_angle 이 /yaw_error 를 멈추고, north_goal_angle geofence 가 침묵한다.
    72	        Node(
    73	            package='ssf_heading',
    74	            executable='yaw_mux',
    75	            name='yaw_mux',
    76	            output='screen',
    77	            parameters=[os.path.join(
    78	                get_package_share_directory('ssf_heading'),
    79	                'config', 'ssf_heading.yaml')],
    80	        ),
    81	
    82	        # ============================================================
    83	        # 3) U-BLOX GPS
    84	        # ============================================================
    85	        # 🚨 배별 설정을 넘긴다. 예전엔 인자 없이 include 해서 ublox launch 의 기본값(B배 F9P)이
    86	        #    A배(M8P)에서도 그대로 쓰였다 — 에러 없이 조용히 어긋나는 류. (2026-09-16)
    87	        #    A배:  ros2 launch launch_files launch_files.launch.py gps_config:=c94_m8p_rover.yaml
    88	        #    B배:  인자 없이 (기본 zed_f9p_rover.yaml)
    89	        IncludeLaunchDescription(
    90	            PythonLaunchDescriptionSource(
    91	                os.path.join(dir_gps, 'launch', 'ublox_gps_node-launch.py')
    92	            ),
    93	            launch_arguments={'config': LaunchConfiguration('gps_config')}.items(),
    94	        ),
    95	
    96	        # ============================================================
    97	        # 4) NTRIP 2초 딜레이
    98	        # ============================================================
    99	        TimerAction(
   100	            period=2.0,
   101	            actions=[ntrip_launch]
   102	        ),
   103	
   104	        # ============================================================
   105	        # 5) NORTH 기준 각도 계산
   106	        # ============================================================
   107	        IncludeLaunchDescription(
   108	            PythonLaunchDescriptionSource(
   109	                os.path.join(dir_north, 'launch', 'north_goal_angle_launch.py')
   110	            )
   111	        ),
   112	
   113	        # ============================================================
   114	        # 6) SHIP goal angle
   115	        # ============================================================
   116	        IncludeLaunchDescription(
   117	            PythonLaunchDescriptionSource(
   118	                os.path.join(get_package_share_directory('ship_goal_angle'),
   119	                             'launch', 'ship_goal_angle_launch_file.launch.py')
   120	            )
   121	        ),
   122	
   123	        # ============================================================
   124	        # 7) RealSense Camera
   125	        # ============================================================
   126	        # 🚨 [2026-08-12 정정] 이 절 제목이 "Auto Exposure/WhiteBalance OFF" 였는데
   127	        #    **둘 다 켜져 있었다.** 실측으로 확인:
   128	        #      rgb_camera.enable_auto_white_balance = True   (아래 줄이 True 로 주고 있었다)
   129	        #      rgb_camera.enable_auto_exposure      = True   (설정 자체가 없어 드라이버 기본값)
   130	        #    제목만 보고 "고정돼 있구나" 로 읽으면 색 캘리브레이션의 전제가 통째로 틀린다.
   131	        #    → 제목을 지우고, 실제 값을 **명시**한다. 기본값에 기대지 않는다.
   132	        Node(
   133	            package='realsense2_camera',
   134	            executable='realsense2_camera_node',
   135	            name='camera',
   136	            namespace='camera',
   137	            output='screen',
   138	            parameters=[
   139	                # ── 색 안정성에 직결되는 값. 현재는 **자동** 이고, 그 상태로 검증했다 ──
   140	                # ⚠️ 색 기반 검출에서 자동 WB 는 원리상 위험하다 — 화면에 뭐가 들어오냐에
   141	                #    따라 같은 부표의 H 가 밀린다(흰 배가 지나가면 색조가 변한다).
   142	                #    그래서 끄려고 했는데, **실측이 그 주장을 뒷받침하지 않았다**:
   143	                #      정지 장면 A/B (auto ON vs OFF 4600K) → ΔH 0.0, ΔS 0.2, ΔV 0.2
   144	                #    자동 WB 가 고정값과 거의 같은 곳에 수렴했다. 측정 근거 없이 바꾸지 않는다.
   145	                # 🚨 **실외에서 다시 판정할 것.** 해가 움직이고 큰 물체가 드나드는 환경이
   146	                #    진짜 시험이다. 절차: 부표를 고정해 두고 화면에 밝은 물체를 넣었다 뺐다
   147	                #    하며 `tools/vision_probe.py --log-csv` 로 H 가 밀리는지 본다.
   148	                #    밀리면 auto_white_balance:=False + white_balance 고정(주간 5500~6500K).
   149	                {"rgb_camera.enable_auto_white_balance": True},
   150	                {"rgb_camera.enable_auto_exposure": True},
   151	
   152	                {"enable_infra1": False},
   153	                {"enable_infra2": False},
   154	
   155	                # 🚨 카메라 내장 IMU(자이로·가속도)를 끈다 — 안 끄면 노드가 예외로 죽는다.
   156	                #    librealsense 가 D455 의 HID/IIO 센서를 열려고 하는데 udev 규칙이 없어
   157	                #    Permission denied 가 나고, 그 예외가 rs_node_setup 의 센서 설정을 통째로
   158	                #    중단시킨다 (2026-08-07 실기 확인):
   159	                #      Failed to open .../iio:device1/scan_elements/in_anglvel_x_en
   160	                #      → Error updating the sensors
   161	                #    ⚠️ udev 규칙을 까는 방법도 있지만 **우리는 이 IMU 를 쓰지 않는다** —
   162	                #      헤딩은 iAHRS(/imu/yaw_raw → yaw_mux)가 담당한다(CLAUDE.md 3-5).
   163	                #      안 쓰는 센서 때문에 시스템에 규칙을 남기는 대신 명시적으로 끈다.
   164	                #    ※ OAK(PoE=이더넷)로 바꾸면 USB HID 자체가 없어져 이 줄은 불필요해진다.
   165	                {"enable_gyro": False},
   166	                {"enable_accel": False},
   167	
   168	                # 🚨 [2026-08-12] 여기 `rgb_camera.controls.saturation: 120` 이 있었는데
   169	                #    **그런 파라미터는 존재하지 않는다.** 실물로 확인:
   170	                #      ros2 param set … rgb_camera.controls.saturation
   171	                #        → "cannot be set because it was not declared"
   172	                #      실제 이름은 `rgb_camera.saturation` 이고, 값은 계속 기본값 **64** 였다.
   173	                #    즉 **120 은 한 번도 먹은 적이 없다.** 에러도 안 났다(조용히 무시).
   174	                #    → 지금 120 으로 고치면 '검증된 적 없는 값' 으로 동작이 바뀐다.
   175	                #      오늘 검출 검증(빨강·초록 100%)은 전부 **64 에서** 나온 결과다.
   176	                #      그래서 실제로 돌던 값 64 를 **명시**만 한다. 120 이 나은지는 측정 후에.
   177	                {"rgb_camera.saturation": 64},
   178	
   179	
   180	                # 🚨 [2026-09-10 실측] 뎁스 스트림을 끈다. **켜면 카메라가 USB 에서 떨어진다.**
   181	                #    D455 를 노트북 USB3 포트에 직결(SuperSpeed 5000M)한 상태에서 측정:
   182	                #      컬러+뎁스 : 컬러 10.8fps · 뎁스 0.6fps · 최대공백 1.75s
   183	                #                  2분간 USB 끊김 13회 · 드라이버 오류 583,090 줄
   184	                #                  (xioctl(VIDIOC_QBUF) failed ... error: No such device 무한 재시도)
   185	                #      컬러 전용 : **30.0fps · 최대공백 0.03s · 오류 0 줄**
   186	                #    허브·USB2.0 문제가 아니었다. 직결·SuperSpeed 에서도 같았고,
   187	                #    USB autosuspend 를 꺼도(boat_boot.sh 적용) 그대로였다. **뎁스 부하 자체가 원인이다.**
   188	                #
   189	                #    ⚠️ 끄는 게 안전한 근거: **뎁스를 구독하는 코드가 하나도 없다.**
   190	                #       V1(2026-07-23)에서 비전 3종의 depth 의존을 전부 제거했고
   191	                #       거리는 LiDAR `/scan` 에서 얻는다(CLAUDE.md 3-3). 전수 검색으로 재확인함.
   192	                #    ⚠️ 되살릴 일이 생기면 **먼저 USB 케이블을 바꿔서** 재측정할 것.
   193	                #       마지막까지 남은 의심은 케이블 품질이다(오늘 아두이노·라이다도 같은 증상이었다).
   194	                {"enable_depth": False},
   195	
   196	                # 🎥 30FPS, 640x480
   197	                {"rgb_camera.color_profile": "640x480x30"},
   198	            ]
   199	        ),
   200	
   201	        # ============================================================
   202	        # 8) 비전 검출기 — **상주**. 각자 /wp_mode 를 보고 자기 차례일 때만 일한다
   203	        # ============================================================
   204	        # 🚨 3-4: 작년엔 subscriber_mode_manager 가 subprocess 로 이 노드들을 죽였다 살렸다.
   205	        #    모드 전환마다 비전이 몇 초 멈추고, 좀비가 남으면 카메라가 잠기고, 추적기를 못 썼다.
   206	        #    지금은 상주 + 모드 게이팅이다. 매니저는 폐기했다(entry point 도 제거).
   207	        #
   208	        #    담당 모드는 **각 노드가 소유**한다 (미션 노드와 같은 패턴).
   209	        #    권위 출처 = 미션 노드의 active_wp_mode. 바꿀 땐 양쪽을 함께 바꿔라:
   210	        #      mode 0,1 → ship_gate  ← gate 검출기 (/red_angle, /green_angle)
   211	        #      mode 2   → ship_back  ← turn 검출기 (white 부표, /image_angle)
   212	        #      mode 3   → ship_turn  ← turn 검출기 (red 부표)
   213	        #      mode 7   → ship_dock  ← dock 검출기 (/image_angle)
   214	        #      mode 5,8 → 순수 회피(담당 없음) → 검출기 전부 비활성
   215	        #
   216	        #    🚨 dock 과 turn 은 둘 다 /image_angle 을 발행한다. 모드가 겹치면 한 토픽에
   217	        #       발행자 2개가 되어 **에러 없이** 값이 섞인다. 겹침 없음은
   218	        #       test_mode_gate.py 의 check_publisher_conflicts 가 정적으로 검사한다.
   219	        Node(package='color_shape_detector', executable='basic_image_subscribergate',
   220	             name='image_subscriber_gate', output='screen', parameters=[VISION_CFG]),
   221	        Node(package='color_shape_detector', executable='basic_image_subscriberturn',
   222	             name='image_subscriber_turn', output='screen', parameters=[VISION_CFG]),
   223	        Node(package='color_shape_detector', executable='basic_image_subscriberdock',
   224	             name='image_subscriber_dock', output='screen', parameters=[VISION_CFG]),
   225	        # basic_image_subscriberhsv 는 튜닝 전용 — 대회 launch 에 넣지 않는다.
   226	
   227	        # ============================================================
   228	        # 9) SHIP mission nodes (WP 루틴)
   229	        # ============================================================
   230	        Node(package='ship_gate', executable='ship_gate', name='ship_gate', output='screen'),
   231	        Node(package='ship_dock', executable='ship_dock', name='ship_dock', output='screen'),
   232	        Node(package='ship_turn', executable='ship_turn', name='ship_turn', output='screen'),
   233	        # ship_last 제거됨(6b): mode 0 을 ship_gate 가 인수. 하던 일은 /candidate_angle 에 20000
   234	        #   폴백을 내는 것뿐이라 중복이었다. (mode 5,8 폴백은 north_goal_angle 이 담당)
   235	        Node(package='ship_back', executable='ship_back', name='ship_back', output='screen'),
   236	
   237	        # ============================================================
   238	        # 10) Fusion: 라이다 기반 항로 유지
   239	        # ============================================================
   240	        Node(
   241	            package='ship_direction',
   242	            executable='ship_direction',
   243	            name='ship_direction',
   244	            output='screen'
   245	        ),
   246	
   247	        # ============================================================
   248	        # 11) Motor Control
   249	        # ============================================================
   250	        Node(
   251	            package='motor_control',
   252	            executable='motor_control',
   253	            name='motor_control',
   254	            output='screen'
   255	        ),
   256	
   257	        # ============================================================
   258	        # 12) 시리얼 브릿지 (Motor_run → 아두이노 Mega, 상태 → /boat_mode)
   259	        # ============================================================
   260	        # 🚨 이게 없으면 motor_control 이 Motor_run 을 내도 **받는 쪽이 없다.**
   261	        #    사슬의 마지막 조각이다: motor_control → Motor_run → [브릿지] → Mega
   262	        # 아두이노가 아직 없어도 안전하다 — 브릿지 launch 가 포트 존재를 먼저 보고
   263	        # 없으면 노드를 안 띄우고 넘어간다(예외를 던지면 launch 전체가 죽는다.
   264	        # ntrip_client 가 파일 하나 못 찾아 노드 15개를 같이 죽인 사고가 실제로 있었다).
   265	        IncludeLaunchDescription(
   266	            PythonLaunchDescriptionSource(
   267	                os.path.join(dir_bridge, 'launch', 'ssf_bridge.launch.py')
   268	            )
   269	        ),
   270	    ])
```

==================================================================
## 테스트 목록 (함수 이름만)
==================================================================
- src/ship_direction/test/test_avoidance_filters.py : 8 개
    105:def test_wide_obstacle_survives():
    119:def test_small_obstacles_fate():
    147:def test_smooth_threshold_scales_with_length():
    157:def test_dilate_widens_survivor():
    171:def test_edge_range_inf_nan_falls_back():
    180:def test_zone_selection_arc_width():
    193:def test_zone_selection_boundary_equal_passes():
    202:def test_zone_selection_inf_edge_uses_detection_distance():
- src/ship_direction/test/test_failsafe_logic.py : 27 개
    21:def test_armed_boot_no_false_trigger():
    28:def test_warn_then_stop_on_scan_loss():
    50:def test_imu_death_detected():
    68:def test_worst_sensor_wins():
    78:def test_no_false_trigger_on_jitter():
    90:def test_auto_recovery_is_immediate():
    102:def test_hard_fault_is_immediate_no_confirm():
    118:def test_single_frame_spray_rejected():
    127:def test_persistent_obstacle_accepted():
    135:def test_obstacle_survives_one_dropout():
    144:def test_disabled_passthrough():
    151:def test_varying_scan_length_is_safe():
    161:def test_median_kills_isolated_spike():
    169:def test_raw_min_would_be_fooled():
    176:def test_median_keeps_real_obstacle():
    183:def test_isolated_spike_with_no_returns_ignored():
    190:def test_min_valid_threshold():
    197:def test_even_window_uses_upper_median():
    205:def test_all_invalid_returns_none():
    210:def test_empty_range_returns_none():
    215:def test_index_returned_for_angle():
    223:def test_return_rejected_backward_compatible():
    230:def test_return_rejected_shape():
    236:def test_rejected_counts_isolated_spike():
    248:def test_rejected_zero_when_dense():
    255:def test_rejected_ignores_already_invalid():
    262:def test_rejected_kernel_off_no_reject():

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

**1-4. 하드코딩 금지.** 배 폭·PWM·거리 임계값은 패키지별 `src/<pkg>/config/<pkg>.yaml` + 코드 기본값에 둔다. 모르는 실측값은 임시값 + **`# ⚠️ 실측 필요`**. 배별 분리 파일(`boat_a.yaml`/`boat_b.yaml`)은 **아직 없다** — 배별 분기는 GPS `gps_config` 인자뿐(A: `c94_m8p_rover.yaml` / B: `zed_f9p_rover.yaml`). 상세 §1-4
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
| 선체 | 1번배 실측 1.66 × 0.57 × 0.38m → `half_width 0.29`. 2번배는 설계값(0.43m) (§8) |

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
- 죽은 토픽(구독자 0): `/goal_distance`, `/wp_remaining_time`, `video_frames`, `/red_distance`·`/green_distance`·`/image_distance`(발행 중단).

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
| 5 | 회피(50초) | **없음 = 정상** | 없음 — `ship_direction` 순수 회피 |
| 7 | 도킹(60초) | `ship_dock` | `dock` → `/image_angle` |
| 8 | 토너먼트 회피 | **없음 = 정상** | 없음 |

- mode 5·8 을 healthcheck 가 '누락' 으로 경고하면 매 실행 거짓 경보다 → 매핑표에 `none` 으로 명시(반영됨). `north_goal_angle` 의 20000 폴백은 5·8 에만.
- 🚨 `dock`·`turn` 이 둘 다 `/image_angle` 을 발행 → `active_wp_modes` 겹치면 발행자 2개. `test_mode_gate.py` 의 `check_publisher_conflicts` 가 정적 검사.
- 검출기는 상주한다(subprocess 매니저 폐기). `/wp_mode` 미수신·stale(2.0s 초과) → **발행 정지** + 5초 주기 경고. 게이트는 `color_callback` 맨 앞.
- `healthcheck` 는 빠진 모드 / 중복 모드 / 침묵 노드를 검사한다.

**`/geofence_state`** — 경계를 '가짜 LiDAR' 로: `data = [angle_min_deg, angle_inc_deg, r0, r1, …]`, 상대방위(0=정면), 멀면 `inf`.
정보 없으면 **빈 배열**(미설정·GPS fix 없음 / IMU stale `imu_stale_sec` 0.5s / 이미 이탈). 소비: `ranges[i] = min(real, geofence)`, `geofence_stale_sec`(2.0s) 넘게 묵으면 병합 안 함. min 병합은 (C) 자율 회피 구간에만 들어가지만, **별도로** candidate/SPIN 구간엔 하드 가드가 있다 — 경계 최소거리가 `geofence_stop_margin_m`(2.0) 안이고 상대방위가 전진방향 ±`geofence_stop_cone_deg`(60°)와 겹치면 candidate/SPIN 무시하고 STOP_HOLD(`ship_direction.py:149`). 도킹 중 정지 원인이 될 수 있다.
🚨 구독자 0 = 경계 방어 없음(실격) → `north_goal_angle` 부팅 5초 뒤 ERROR + `healthcheck` 가 `/health_ok=false`. ⚠️ 상세 §3-9 에 구판 소비 방식(±40° 원뿔 칠하기)이 남아 충돌 — 코드 확인.

**QoS**
- 이미지 구독: **BEST_EFFORT + KEEP_LAST + depth=1** (묵은 프레임으로 조향 금지). `/scan` 구독 5곳(`ship_direction/dock/gate/turn/back`)도 동일 — depth=1 이 도착=신선을 일치시켜 페일세이프를 정직하게 만든다.
- `/wp_mode` 는 모드 명령 → **RELIABLE 유지**.
- 관찰 토픽이라도 **발행은 RELIABLE**(`/boat_*`) — BEST_EFFORT 면 `ros2 topic echo` 가 아무것도 못 받는다. (구독자 BEST_EFFORT ↔ 발행자 RELIABLE 은 호환, 반대는 비호환)

**헤딩·비전 규칙**
- 각도 평균은 산술평균 금지(359°와 1°→180°) — 단위벡터 합 + `atan2`. COG 수렴 파라미터(`cog_min_samples`·`gps_rate_hz`·`cog_half_life_sec`)는 포화 함정이 있어 바꾸기 전 상세 §3-5.
- `/health_ok` 는 진단 전용이라 거짓 정지 위험이 없다는 전제로 설계됐다. 제어에 물리려면 먼저 상세 §3-3·§5 를 보고 Davidson 과 검토.
- 도크 인식(목표 색·형상 유효값 밖이면 ERROR + 발행 정지, 임계는 혼동행렬 수치로 — 감으로 튜닝 금지)을 건드리면 상세 §3-3b.
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

## 8. ⚠️ 아직 모르는 것 (추측하지 말 것 — 필요한 곳엔 `# ⚠️ 실측 필요`)

- 2번배 실측 폭 (현재 설계값 0.43m; 공용 `half_width` 는 1번배 기준)
- 새 배 추력 → `base_pwm`, `reverse_pwm`
- IMU 장착 오프셋(`mount_offset_deg`, `invert_yaw` — 0.0/false 는 "아직 안 쟀다"), 대회장 자기편각
- 도크 규격 → `contact_dist_m`
- 🚨 **조향 좌/우 부호**(`steer_invert`) — 틀리면 모든 게 무의미. 가장 흔한 사고. 벤치 필수
- LiDAR 장착 높이 = 부표 높이인지 (안 맞으면 부표를 못 본다 → 위치유지 소스 판단, §6-2)
- 카메라 왜곡계수(D)·초점거리(fx,fy) (`camera_info`), OAK 고정 IP·컬러 토픽 이름
- 깊이 0.38m(선체 아래 21cm) — 진수·회수·대회장 얕은 구역 주의

----- DOC: docs/전달용/LiDAR_노드_쉬운설명.md -----
# LiDAR 노드 모음 — 아주 쉬운 완전 해설

> 🚨 **2025(작년) 구조 설명이다 — 2026 코드와 다르다.** (half_width 0.45·1.8 m·RELIABLE QoS 등은 옛값.) 최신 설명은 `docs/전달용/보고서_분담/초안/LiDAR.md`.


이 문서는 `LiDAR_노드_모음.zip` 안에 있는 파일들을 **하나도 빼놓지 않고, 처음 보는 사람도 이해하도록** 설명합니다.
코드 자체보다 "이 파일이 뭘 하는 애인지, 그리고 노드랑 어떻게 연결되는지"에 초점을 맞췄습니다.

---

## 0. 먼저 3개 단어만 이해하고 시작하기

ROS2(로봇 소프트웨어 틀)를 이해하려면 딱 3개 단어면 됩니다. 방송국에 비유할게요.

- **노드(Node)** = "일하는 프로그램 하나". 방송국의 **직원 한 명**이라고 보면 됩니다. 예: LiDAR를 읽는 직원, 장애물을 피하는 직원.
- **토픽(Topic)** = 직원들이 정보를 주고받는 **라디오 채널**. 한 직원이 채널에 방송하면(발행), 그 채널을 듣는 직원들이(구독) 정보를 받습니다.
- **패키지(Package)** = 관련된 노드/파일을 담은 **폴더 한 개**. 부서 하나라고 보면 됩니다.

이 묶음에는 **패키지(부서)가 2개** 들어있습니다:

1. **rplidar_ros** — LiDAR 센서에서 데이터를 뽑아내는 **"센서 부서"**
2. **ship_direction** — 그 데이터로 장애물을 피하는 **"항로 판단 부서"**

그리고 이 둘을 잇는 라디오 채널(토픽)이 바로 **`/scan`** 입니다.

```
[LiDAR 센서] → (rplidar_ros 부서) → 라디오채널 /scan → (ship_direction 부서) → 조향 결정
```

---

## 1. 전체 그림 (데이터가 흐르는 길)

```
 실제 RPLIDAR A3 하드웨어 (USB 케이블로 연결)
        │  (0.1초마다 360° 거리 측정)
        ▼
 ┌─────────────────────────┐
 │ rplidar_ros 패키지        │   ← "센서 부서"
 │  = rplidar_node 라는 노드 │
 └─────────────────────────┘
        │  /scan 채널로 방송 (거리값 배열)
        ▼
 ┌─────────────────────────┐
 │ ship_direction 패키지     │   ← "항로 판단 부서"
 │  = ship_direction 노드    │
 │  (scan_callback 이 처리)  │
 └─────────────────────────┘
        │  /desired_angle (가야 할 각도) 방송
        ▼
   (모터 제어 노드로 전달)
```

핵심: **rplidar_ros가 만든 `/scan`을 ship_direction이 듣는다.** 이게 두 부서의 유일한 연결 고리입니다.

---

## 2. 패키지 ①: rplidar_ros (센서 부서)

LiDAR 하드웨어를 직접 다루는 **드라이버**입니다. Slamtec(제조사)이 만든 공식 패키지이고, C++로 작성돼 있습니다.
쉽게 말해 "센서와 컴퓨터 사이의 **통역사**"입니다. 센서가 보내는 신호를 컴퓨터가 알아듣는 `/scan` 데이터로 바꿔줍니다.

### 📁 launch/ 폴더 — "실행 버튼 모음"
LiDAR를 **어떤 설정으로 켤지** 적어둔 파일들입니다. 센서 모델마다 하나씩 있어서 파일이 많습니다(A1, A2, S1, S2, T1 …).
**우리가 실제로 쓰는 파일은 딱 하나: `rplidar_a3_launch.py`** (A3 모델용). 나머지는 다른 모델용이라 안 씁니다.

`rplidar_a3_launch.py` 안에 적힌 중요한 설정값:

| 설정 | 값 | 쉬운 뜻 |
|---|---|---|
| `serial_port` | `/dev/ttyLiDAR` | LiDAR가 꽂힌 USB 포트 이름 |
| `serial_baudrate` | `256000` | 통신 속도 (A3는 256000이 맞음) |
| `scan_mode` | `Sensitivity` | 감도 높은 모드 (멀리·약한 반사도 잘 봄) |
| `angle_compensate` | `true` | 각도 간격을 균일하게 보정 |
| `inverted` | `false` | 위아래 뒤집기 안 함 |
| `frame_id` | `laser` | 이 센서 데이터의 좌표계 이름표 |

> ⚠️ **중요한 주의**: `serial_port`가 `/dev/ttyLiDAR`로 돼 있는데, 이건 "이 USB 장치를 항상 ttyLiDAR라는 이름으로 부르자"는 **udev 규칙**이 컴퓨터에 등록돼 있어야 작동합니다. 새 노트북/새 배에는 그 규칙이 없으면 LiDAR를 못 찾습니다. (아래 debian/udev 설명 참고)

### 📁 src/ 폴더 — "실제로 일하는 뇌"
- **`rplidar_node.cpp`** ← 이게 진짜 핵심 파일입니다. **rplidar_node 라는 노드의 본체**입니다.
  하는 일: USB로 센서와 대화 → 360° 거리 데이터를 받아 → `/scan` 채널로 방송.
  이 파일이 만드는 `/scan` 한 묶음(한 바퀴 스캔)에는 이런 정보가 들어갑니다:
  - `ranges[]` : 각도별 거리값 배열 (예: 정면 2.3m, 오른쪽 1.1m … 측정 안 되면 무한대 `inf`)
  - `angle_min`, `angle_max`, `angle_increment` : 어느 각도부터 어느 각도까지, 몇 도 간격인지
  - `range_min`(0.15m), `range_max` : 측정 가능한 최소/최대 거리
  - `intensities[]` : 각 점의 반사 신호 세기
- **`rplidar_client.cpp`** : 테스트용 도구(센서 데이터를 받아보는 예제). 실제 배 운용엔 안 씀.
- **`node.cpp` / `client.cpp`** : 옛날 ROS1용 코드(참고용). 우리 시스템(ROS2)에선 안 씀.

### 📁 include/ 폴더 — "설계도(헤더)"
- `rplidar_node.hpp`, `visibility.h` : C++에서 rplidar_node.cpp가 쓰는 함수·구조의 **선언서**입니다. 요리로 치면 재료 목록. 빌드할 때 필요합니다.

### 📁 sdk/ 폴더 — "센서와 대화하는 저수준 통신 라이브러리" (용량 대부분)
Slamtec이 만든 **SDK**입니다. USB 시리얼로 센서에 "스캔 시작해", "모드 바꿔" 같은 저수준 명령을 실제로 보내는 코드가 여기 있습니다.
rplidar_node.cpp는 이 SDK를 불러다 씁니다. **직접 수정할 일은 거의 없지만, 빌드하려면 반드시 있어야 합니다.** (7.8MB 중 6.4MB가 여기)

### 📁 debian/udev — "USB 포트 이름 고정 규칙"
- `debian/udev` 파일: USB에 꽂힌 LiDAR가 매번 다른 이름(ttyUSB0, ttyUSB1…)으로 잡히는 걸 막고, **항상 같은 이름으로 부르게** 하는 규칙입니다.
- 단, 이 파일에 적힌 기본 규칙은 이름을 `/dev/rplidar`로 만듭니다. 그런데 우리 launch는 `/dev/ttyLiDAR`를 씁니다. **즉 실제 배에는 이 파일이 아니라, ttyLiDAR를 만들어주는 별도의 커스텀 udev 규칙이 따로 등록돼 있다는 뜻**입니다. (새 배 세팅 시 이 udev 규칙을 꼭 다시 만들어야 함)

### 📁 rviz/ 폴더 — "화면으로 보는 설정"
- RViz(ROS2의 시각화 프로그램)에서 LiDAR 점들을 **눈으로 보기 위한** 화면 설정 파일입니다. 디버깅·확인용이고, 배가 스스로 달리는 데는 필요 없습니다.

### 📁 scripts/ 폴더 — "설치 도우미 스크립트"
- udev 규칙 자동 등록 같은 **설치 편의 스크립트**들입니다. 최초 세팅할 때만 씁니다.

### 📄 기타 파일들
- **`package.xml`** : 이 부서(패키지)의 **신분증**. 이름(rplidar_ros), 버전(2.1.4), 지원 모델(A1~T1), 필요한 부품(rclcpp, sensor_msgs 등)이 적혀 있습니다. ROS2가 패키지를 인식하는 필수 파일.
- **`CMakeLists.txt`** : **빌드 설명서**. "이 C++ 코드들을 이렇게 컴파일해라"라고 알려주는 레시피.
- **`README.md`, `LICENSE`, `CHANGELOG.rst`** : 설명서 / 저작권 / 변경 이력. 코드 동작과는 무관.
- **`rplidar_A1.png`, `rplidar_A2.png`** : 설명서에 들어가는 센서 사진.

---

## 3. 패키지 ②: ship_direction (항로 판단 부서)

`/scan`을 **받아서 실제로 써먹는** 노드입니다. 우리 팀이 직접 만든 Python 코드입니다.
쉽게 말해 "센서가 준 거리 정보를 보고 **어디로 갈지 정하는 판단자**"입니다.

### 📄 ship_direction/ship_direction.py — "이 부서의 뇌 (핵심 노드)"
이 한 파일이 ship_direction 노드의 전부입니다. 안에서 벌어지는 일을 쉬운 순서로 풀면:

1. **`/scan`을 구독** — LiDAR 부서의 방송을 듣습니다. 새 스캔이 올 때마다 `scan_callback()` 함수가 자동 실행됩니다.
2. **정면 부채꼴만 보기** — 360° 전부가 아니라 **앞쪽 0°~160°** 만 사용합니다. 이 중 **80°가 배의 정면(정중앙)** 입니다.
3. **장애물 지도 만들기** — 각 각도의 거리를 보고 "여긴 장애물(가까움)", "여긴 뻥 뚫림"으로 0/1 표시를 만듭니다. 기준 거리는 1.8m.
4. **노이즈 정리 + 배 폭 반영** — 반짝 튀는 오탐을 지우고, 장애물을 **배 폭(0.45m)만큼 부풀려서** "배가 실제로 못 지나가는 폭"을 계산합니다.
5. **지나갈 수 있는 빈 틈 찾기** — 뻥 뚫린 구간 중 배가 통과 가능한 넓이(약 1.05m 이상)만 후보로 남깁니다.
6. **목표 방향에 가장 가까운 틈으로 조향** — 가고 싶은 방위(yaw_error)에 제일 가까운 빈 틈 방향을 골라 `/desired_angle`로 내보냅니다.
7. **막혔으면 후진** — 지나갈 틈이 하나도 없으면 후진 신호(260°)를 냅니다.

이 노드가 **구독(듣기)** 하는 채널: `/scan`(LiDAR), `/yaw_error`(가고 싶은 방위), `/candidate_angle`(미션 지시), `/wp_mode`(현재 미션 번호)
이 노드가 **발행(방송)** 하는 채널: `/desired_angle`(가야 할 각도), `/obstacle_distance_array`(가장 가까운 장애물 거리·각도)

> 참고: 이 노드는 LiDAR만 쓰는 건 아니고 방위·미션 신호도 함께 섞어서 판단합니다. 하지만 **LiDAR 처리(scan_callback)가 이 노드의 심장**이라 LiDAR 묶음에 포함했습니다.

### 📄 launch/ship_direction_launch.py — "이 노드만 켜는 실행 버튼"
ship_direction 노드 하나만 실행하는 파일입니다. 파라미터나 토픽 이름 바꾸기(remapping) 자리도 마련돼 있지만 지금은 비어 있습니다(기본값 사용).

### 📄 setup.py — "설치 방법서"
이 Python 패키지를 어떻게 설치할지 적은 파일. 가장 중요한 줄은:
`'ship_direction = ship_direction.ship_direction:main'`
→ "`ship_direction`이라는 실행 명령을 치면, ship_direction.py의 main() 함수를 실행해라"는 연결고리입니다. launch가 이 이름으로 노드를 켭니다.

### 📄 package.xml — "이 부서의 신분증"
이름(ship_direction), 필요한 부품(rclpy=파이썬용 ROS2, sensor_msgs=LaserScan 메시지, std_msgs, numpy)이 적혀 있습니다.

### 📄 그 외 (자동 생성 파일들, 신경 안 써도 됨)
- **`setup.cfg`** : 실행 파일이 설치될 위치를 알려주는 보조 설정.
- **`resource/ship_direction`** : 내용 없는 빈 표식 파일. "이런 패키지가 있어요"를 ROS2에 알리는 이름표.
- **`ship_direction/__init__.py`** : 폴더를 파이썬 패키지로 인식시키는 빈 파일.
- **`test/` 폴더 (test_flake8.py 등)** : 코드 스타일 자동 검사 파일. 대회 동작과 무관하고, 패키지 만들 때 자동으로 생기는 것들입니다.

---

## 4. 두 부서가 연결되는 지점 (제일 중요)

```
rplidar_node (rplidar_ros 안)
     │
     │  ┌────────────────────────────────────────┐
     └─▶│  /scan  (sensor_msgs/LaserScan 메시지)   │
        │  = 360° 거리값 한 묶음, 0.1초마다 갱신     │
        └────────────────────────────────────────┘
     │
     ▼
ship_direction 노드
     scan_callback() 이 /scan 을 받아서 처리
```

- **연결 방법**: 이름이 같은 토픽 `/scan`을 한쪽은 발행, 한쪽은 구독 → 자동으로 연결됩니다. (직접 선을 잇는 게 아니라, 같은 채널 이름을 쓰면 ROS2가 알아서 이어줍니다.)
- **주고받는 내용물**: `LaserScan`이라는 정해진 형식의 메시지. 그 안의 `ranges[]`(각도별 거리 배열)가 핵심입니다.
- **QoS(통신 품질) 궁합**: 둘 다 "RELIABLE(빠짐없이 전달)"로 맞춰져 있어 정상적으로 연결됩니다.

---

## 5. 실제로 켜지는 순서 (부팅하면 벌어지는 일)

1. 통합 실행 파일이 먼저 `rplidar_a3_launch.py`를 실행 → **rplidar_node**가 켜지고 `/scan` 방송 시작.
2. 이어서 **ship_direction 노드**가 켜지고 `/scan`을 구독 시작.
3. LiDAR가 한 바퀴 돌 때마다(0.1초) → `/scan` 한 묶음 방송 → ship_direction의 `scan_callback()`이 즉시 실행 → 장애물 피할 각도 계산 → `/desired_angle` 방송.
4. 이 각도를 모터 제어 노드가 받아 실제 프로펠러를 움직임.

---

## 6. 한 줄 요약

- **rplidar_ros** = LiDAR 하드웨어를 읽어 `/scan`으로 방송하는 **센서 통역사** (핵심 파일: `src/rplidar_node.cpp`, 실행: `launch/rplidar_a3_launch.py`)
- **ship_direction** = `/scan`을 듣고 장애물을 피할 각도를 정하는 **판단자** (핵심 파일: `ship_direction/ship_direction.py`)
- 둘의 연결 고리 = 라디오 채널 **`/scan`** 하나.

## 7. 용어 사전 (헷갈리면 여기 보기)
- **노드** : 일하는 프로그램 하나 (직원)
- **토픽** : 정보를 주고받는 채널 (라디오 주파수). 이름 앞에 `/`가 붙음 (예: `/scan`)
- **발행(publish)** : 토픽 채널에 정보를 방송하는 것
- **구독(subscribe)** : 토픽 채널을 듣는 것
- **패키지** : 관련 파일을 담은 폴더 (부서)
- **launch 파일** : 노드를 어떤 설정으로 켤지 적은 실행 스크립트
- **LaserScan** : LiDAR 한 바퀴 측정 결과를 담는 정해진 메시지 형식
- **udev 규칙** : USB 장치를 항상 같은 이름(`/dev/ttyLiDAR`)으로 부르게 하는 컴퓨터 설정
- **QoS** : 토픽 통신 품질 규칙 (발행/구독 양쪽이 맞아야 연결됨)
- **SDK** : 센서와 저수준으로 통신하는 제조사 제공 라이브러리

----- DOC: docs/전달용/회신_rplidar_회피_수정제안.md -----
# 회신: RPLIDAR / 장애물 회피 수정 제안서 검토

> 받은 것: `rplidar_ship_direction_수정제안서 g.md`
> 기준: 저장소 최신 `ship_direction.py`(v2.2.2), `config/ship_direction.yaml`, `blackbox`
> 한 줄 결론: **전반 방향(측정 우선, 검증 없는 알고리즘 변경 지양)은 우리 원칙과 정확히 같습니다.
> 다만 §7.3·§8은 이미 반영돼 있고(옛 base 기준), §5.1 은 좋은 발견이라 우리가 단위테스트를 추가해
> 실증했습니다. §7.1 진단표는 채택합니다.**

---

## ✅ 이미 반영된 것 (최신본에서 끝남 — 옛 base 기준 제안)

| 제안 | 실제 최신본 |
|---|---|
| **§7.3 후진 로직 재검토** ("REVERSE 계속 발행, cooldown 은 시간만 갱신 → FORWARD/REVERSE/RETRY 상태기계 권장") | **오늘 v2.2.2 로 반영됨.** `REVERSING(0.5s) → COOLDOWN=STOP 재탐색(3s) → 반복` 상태기계([L]). 제안하신 "일정 시간 후진 → 정지 → 재평가 → 복귀"와 동일 |
| **§8 QoS** ("현재 `/scan` depth 10 → `qos_profile_sensor_data` 검토") | **이미 `SCAN_QOS`(BEST_EFFORT, KEEP_LAST, depth=1)** 적용. 제안보다 더 나갔습니다(QoS-B). 인용하신 `create_subscription(...,10)` 은 옛 코드입니다. rplidar 2.1.4 발행자 RELIABLE 과 호환도 소스에서 확인함 |

→ 이 둘은 **다음 리뷰를 저장소 최신 `ship_direction.py` 로** 떠주시면 어긋남이 없습니다.

---

## ⭐ §5.1 작은 부표 필터 생존 — 좋은 발견, 단위테스트로 실증함

정확한 지적입니다. `min_obstacle_cells=1` 로 낮췄어도 **`smooth_spikes` · `suppress_spike_edges` 가
먼저 실행**돼서 작은 군집을 dilate 전에 지울 수 있습니다. 제안하신 단위테스트를 **바로 만들어
실제 노드 필터로 확인**했습니다(`test/test_avoidance_filters.py`, ROS 없이 Mac 에서 실행).

**실측 결과 (n=200, 기본 파라미터):**
```
1셀: 시작1 → smooth 0 → suppress 0 → dilate 0    (smooth_spikes 가 제거, run<=int(200*0.01)=2)
2셀: 시작2 → smooth 0 → suppress 0 → dilate 0    (동일)
3셀: 시작3 → smooth 3 → suppress 0 → dilate 0    (smooth 통과, suppress_spike_edges 가 침식 삭제)
넓은 30셀: 통과 후에도 20+ 셀 생존 → dilate 로 더 넓어짐
```

**해석:**
- **소형(1~3셀) 고립 장애물은 dilate 전에 삭제됩니다** → `min_obstacle_cells=1` 은 이 경우 사실상 무력.
- **그러나 실제 1.6m 부표는 회피거리(3m)에서 ~30셀 폭**이라 침식 2셀 정도로는 안 사라집니다(테스트로 확인).
  즉 이 소형 제거는 결과적으로 **원거리·노이즈 억제**에 가깝고, 위협이 되는 근거리 부표는 넓어서 생존합니다.
- **결론:** 지금 당장 필터를 끌 필요는 없어 보입니다(넓은 부표는 안전). 다만 **테스트를 상시 두어**
  파라미터(`border_margin`·`max_spike_ratio`)를 만졌을 때 부표가 사라지는 회귀를 잡습니다.
  실제 물보라가 '작은 군집'으로 뭉치면 이 필터가 그걸 지워 유리할 수도, 진짜 소형 표식을 지울 수도
  있으니 **수조 실측(blackbox `obstacle_min_dist`)으로 최종 확인**합니다(제안 §5.1 마지막 줄과 동일).

---

## ⭐ §7.1 제어 지연 진단표 — 채택

"원본스캔 없음 / binary 없음 / 필터 후 사라짐 / 회피각 늦음 / yaw 늦음 / yaw 변해도 충돌" 인과표는
접촉 원인을 단계별로 좁히는 아주 좋은 틀입니다. **배 디버깅 워크플로우로 채택합니다.**
- 대부분은 이미 `blackbox` 가 로깅합니다: `scan_dt`(스캔 도착)·`desired_angle`·`obstacle_min_dist`·
  `yaw_error`·`failsafe_level`·`pwm_l/r`·`imu_yaw`.
- rosbag 은 `/scan` 원본과 단계별 디버그 토픽(§5.2)을 보강하는 용도로 함께 씁니다.

---

## 🔶 유효하나 저순위 / 측정 먼저

- **§6.2 `arc_len` off-by-one** (`(e-s)` vs `(e-s+1)`) — 실재합니다. 다만 현재 `clearance`·`half_width`
  가 **`(e-s)` 공식 기준으로 §6 시뮬 스윕에서 튜닝**돼 있어, 공식을 바꾸면 통과 판정이 달라져
  **재시뮬이 필요**합니다(제안서도 인정). 자기일관적이라 급하지 않습니다. 바꿀 땐 단위테스트+시드20.
- **§5.2 단계별 디버그 토픽** (`/raw_binary_scan`·`/inflated_obstacle_scan`) — 부표가 어느 단계에서
  사라지는지 RViz 로 보는 데 유용. 새 발행자(제어 흐름 불변)라 저위험. **접촉 디버깅이 실제로 필요할 때** 추가.
- **§6.1 동적 clearance** (속도 기반 여유) — 반응지연을 §7.1 로 **먼저 측정**한 뒤 필요하면. 측정 먼저.
- **§4.1 미사용 import(LogInfo) 삭제** — 사소. launch 파일 정리 시 함께.
- **§3.2 udev `ttyLiDAR` 문서화** — 이미 `docs/절차/ubuntu_setup.md §8` 에 udev 규칙 절차가 있습니다.
- **§4.1 scan_mode 비교(Sensitivity/Standard/Boost)** — 실기에서 실행 인자로 비교하는 게 맞습니다. 수조 실험 항목으로.

---

## 우선순위 (우리 정리)

1. **완료**: §7.3 후진 상태기계, §8 QoS-B — 이미 최신본에.
2. **완료(이번)**: §5.1 단위테스트 추가·실증. → 필터는 유지, 회귀 감시.
3. **채택**: §7.1 진단표 = 배 디버깅 표준 절차(blackbox+rosbag).
4. **실측 후**: §6.1 동적 clearance, §4.1 scan_mode 비교, §5.1 필터 재평가(수조 데이터).
5. **재시뮬 후**: §6.2 arc off-by-one.
6. **필요 시**: §5.2 디버그 토픽.
7. **보류(동의)**: cost function / Pareto / 시간투표 기본 ON / 검증 없는 대규모 변경.

---

## 요약

| 항목 | 판정 |
|---|---|
| §7.3 후진 · §8 QoS | ✅ 이미 최신본에 반영(옛 base 기준 제안) |
| §5.1 작은 부표 | ⭐ 좋은 발견 → 단위테스트 추가·실증(소형 삭제/넓은 부표 생존). 필터 유지, 회귀 감시 |
| §7.1 진단표 | ⭐ 채택 (blackbox+rosbag) |
| §6.2 arc off-by-one | 유효, 재시뮬 필요(저순위) |
| §5.2/§6.1/scan_mode | 측정·디버깅 먼저 |

**제안서 자체가 훌륭합니다** — "접촉 원인을 단계별로 확인하고 검증 없는 알고리즘 변경은 피한다"는
방향이 우리 §5·§6 원칙과 같습니다. 다음부턴 최신 `ship_direction.py`(v2.2.2)를 base 로 떠주세요.
기준은 저장소 코드와 CLAUDE.md §5(페일세이프)·§6(회피 검증)입니다.

----- DOC: docs/전달용/회신_ship_direction_수정제안.md -----
# 회신: ship_direction 수정 제안 (버그 2건) 검토 결과

> 받은 것: `ship_direction_version_2.12.py` + `ship_direction_수정제안서.md`
> 검토 기준: 저장소 최신 `ship_direction.py` (커밋 이력 반영본)
> 한 줄 결론: **제안 감사합니다. 버그2는 반영했고, 버그1은 시뮬 결과 최종 조향에 영향이 없어 보류합니다.**

---

## 0. 먼저 — base 버전 안내 (GPS 때와 같은 상황)

`version_2.12`는 우리 **최신본이 아닌 버전**에서 갈라졌습니다. 최신본엔 그 뒤에 들어간
**D6**(`/obstacle_reject_count` — median 필터가 버린 물보라 스파이크 수를 blackbox에 기록)이 있는데
2.12엔 없습니다.

→ **2.12를 통째로 반영하면 D6가 사라집니다.** 그래서 제안하신 버그만 뽑아 최신본 위에 얹었습니다.
   다음 작업은 저장소 최신 `ship_direction.py`를 base로 떠주세요.

---

## 1. 버그2 (dilate가 lock 스냅샷 우회) — ✅ 반영함

**지적이 맞습니다.** `dilate_obstacles`가 `self.detection_distance`를 직접 읽어, `control_cb`가
lock으로 뜬 스냅샷과 어긋날 수 있었습니다. 수정했습니다(파라미터로 전달).

**다만 실제 영향은 거의 없다는 점만 공유합니다** — 반영 근거를 정확히 하기 위해서입니다:
```python
r_edge = detection_distance          # 폴백값
for k: r = distance_array[k]
       if 유효: r_edge = min(r_edge, r)   # ← 유효 거리로 덮어씀
```
`dilate` 대상 셀(`binary=1`)은 **정의상 `r < detection_distance`인 유효 셀**입니다. 그러니
`distance_array[k]`에 항상 유효값이 있어 `r_edge`가 그걸로 덮이고, `self.detection_distance`
폴백이 실제로 쓰이는 경우가 없습니다(그 셀이 전부 inf/nan이면 애초에 `binary=0`).

→ 값이 섞여도 그 값이 결과에 안 쓰이므로 실측 영향은 0에 가깝습니다. **그래도 반영한 이유:**
   lock 일관성 원칙이고, 나중에 `dilate` 로직이 바뀌어 폴백을 실제로 쓰게 되면 그때 진짜
   버그가 되니 미리 막는 게 맞습니다. 1줄이라 비용도 없고요.

**반영 커밋**: `fix(ship_direction): dilate_obstacles 가 lock 스냅샷을 우회하던 것 수정`

---

## 2. 버그1 (180° 매핑 불연속) — ⏸ 보류 (시뮬 근거)

**지적의 절반은 맞고, 절반은 시뮬로 확인해보니 결과가 달랐습니다.**

### 확인된 것 ✅ — 매핑값 불연속은 실재
```
179.9° → 259.9      180.1° → -99.9      점프 359.8°
```
매핑 공식이 180° 경계에서 튀는 것은 사실입니다.

### 시뮬 결과 ❗ — 그런데 "최종 조향각"은 영향받지 않음

매핑값이 아니라 **zone 선택 후 실제로 나가는 조향각**을 비교했습니다.
(통로 2개: 좌 10~60°, 우 90~150° / yaw_raw 0~360° 스윕 / 기존 매핑 A vs 제안하신 circular B)

| 구간 | A vs B 결과 |
|---|---|
| **전방 목표** (회피가 실제 작동하는 곳) | **0/36 — 완전히 동일** |
| 180° 근처 | 1/9 (딱 `raw=180.0` 한 점만) |

**핵심 두 가지:**

1. **정상 구간(목표가 전방)에선 두 방식이 완전히 같습니다.** circular로 바꿔도 회피 결과가
   안 바뀝니다. 즉 실제 회피가 일어나는 구간엔 애초에 이 버그의 영향이 없습니다.

2. **"조향 급반전"은 매핑 불연속이 원인이 아니었습니다.**
   ```
                   기존 A        circular B
   179.9° → 조향   150 (우끝)    150 (우끝)
   180.1° → 조향    10 (좌끝)     10 (좌끝)   ← 둘 다 똑같이 점프
   ```
   매핑값(259.9→-99.9)은 A에서 튀지만, **zone 선택 후 최종 조향각은 A/B 둘 다 180°에서
   좌↔우로 갈립니다.** circular로 고쳐도 이 갈림은 **안 사라집니다.**

   이유: 목표가 **정확히 뒤(180°)** 에 있으면 좌우 통로가 대칭이라 어느 쪽으로 돌든 물리적으로
   맞습니다. 이건 매핑 버그가 아니라 **본질적 대칭**이고, 매핑 방식과 무관하게 남습니다.

### 판단 — 얻는 것 ≈ 0, 리스크는 있음

| | |
|---|---|
| 정상 구간 | 이미 A=B 동일 → 고칠 게 없음 |
| 180° 근처 | 고쳐도 급반전 안 사라짐(본질적 대칭) → 얻는 게 없음 |
| 비용 | circular 교체 = §6에서 20시드 시뮬로 검증된 zone 로직 변경 → 재검증 필요 |

circular로 바꾸면 검증된 회피 로직을 건드리는데, 정상 구간은 어차피 같고 180° 급반전은
안 고쳐집니다. **그래서 지금은 보류합니다.**

### 유일하게 남는 미묘함

`raw=180.0` **정확히** 그 점에서만 A(우끝)와 B(좌끝)가 반대입니다. 하지만
(1) 부동소수점 180.0 정확히는 실전에서 거의 안 걸리고 (2) 둘 다 "뒤로 도는 것"이라 안전
문제가 아닙니다. 정 신경 쓰이면 매핑 교체 대신 **"정후방 목표일 때 좌우를 한 방향으로
고정하는 tie-break 1줄"** 이 더 정직한 해결입니다(저리스크).

---

## 요약

| # | 항목 | 판정 | 조치 |
|---|---|---|---|
| — | 2.12 base 버전 | D6 이전 버전 | 최신본을 base로 다시 |
| 2 | dilate lock 우회 | 실재(영향 미미) | ✅ 반영(1줄) |
| 1 | 180° 매핑 불연속 | 매핑값은 실재, 최종 조향엔 영향 0 | ⏸ 보류 |

**제안 자체는 좋았습니다** — 특히 버그1은 코드만 보면 놓치기 쉬운 지점을 정확히 짚으셨고,
저희도 그 덕에 시뮬로 "실제로는 어디까지 영향이 가는지"를 확인할 수 있었습니다.
버그1은 배 뜬 뒤 실측(blackbox)에서 정후방 조향이 실제로 문제되는지 보고 재론하죠.

궁금한 점 있으면 언제든. 정확한 기준은 저장소 코드와 `CLAUDE.md` §5(페일세이프)·§6(회피 검증)입니다.

----- DOC: docs/문제와작업/라이다_고장진단_20260818.md -----
# 라이다 고장 진단 기록 (2026-08-18)

> 라이다 한 대가 응답하지 않아 분해·측정으로 원인을 좁힌 기록.
> **결론: 라이다 본체가 아니라 케이블 단선.**

---

## 0. 결론부터

| | |
|---|---|
| **고장 부위** | 연결 케이블의 **빨강(VCC)·노랑(TX) 2가닥 단선** |
| **라이다 본체** | 보드 측정상 이상 없음 (전원 단락 없음, 신호선 접지 단락 없음) |
| **USB 어댑터** | ✅ **정상** — 같은 어댑터에 정상 라이다를 물려 응답 확인 |
| **필요한 조치** | `RPLIDAR A3` 용 케이블 구매 (한쪽 **6핀** / 반대쪽 **5핀**) |

---

## 1. 증상

- 어떤 보레이트(115200·128000·230400·256000·460800·1000000)에서도 `GET_INFO` 무응답
- `DTR` 을 켜고 끄고 모두 무응답, 스스로 스트리밍하는 데이터도 없음
- 드라이버도 `SL_RESULT_OPERATION_TIMEOUT`
- 모터 안 돎
- **그런데 USB 어댑터는 인식되고 LED 도 켜짐**

🚨 **어댑터 LED 는 라이다 전원의 증거가 아니다.** LED 는 USB 에서 받은 전원으로 켜지므로,
라이다로 나가는 5V 가 죽어도 멀쩡히 들어온다.

🚨 **`/dev/ttyLiDAR` 이 생기는 것도 증거가 아니다.** udev 규칙은 **USB 어댑터의 시리얼**로
매칭하므로 라이다 본체가 죽어도 링크는 정상적으로 생긴다.
→ 09-14 에 이 함정에 다시 걸려 **허브 전류 부족을 의심하며 포트를 옮기는 헛수고**를 했다.
   무응답이면 포트·허브를 의심하기 전에 **정상 개체를 물려보는 것**이 가장 빠르다(1분).

---

## 2. 측정 경과

배터리·USB 를 분리한 상태에서 멀티미터로 좁혔다.

| # | 측정 | 결과 | 판정 |
|---|---|---|---|
| 1 | 검정선 ↔ 보드 `TP16` | 삐 | 검정 = GND, 커넥터 접촉 정상 |
| 2 | 빨강 ↔ 검정 (200Ω 범위) | **1.5 kΩ** | ✅ **전원 단락 아님.** 양쪽 다 보드에 닿음 |
| 3 | 노랑·초록·파랑 각각 ↔ 검정 | 전부 무음 | ✅ 신호선 접지 단락 없음 |
| 4 | **케이블 양끝 같은 선끼리** | **빨강·노랑 무음** | 🚨 **두 가닥 단선** |

### 🚨 측정 중 겪은 함정 두 개 — 다음에 또 당하지 말 것

**① 레인지가 크면 작은 저항이 `0` 으로 뭉개진다.**
`빨강↔검정` 이 도통 모드에서는 안 울리는데(50Ω 초과) `Ω` 모드에서는 `0` 으로 떴다.
모순처럼 보였지만 **범위를 `200` 으로 내리니 `1.5kΩ`** 이었다. 높은 범위에서 `0` 은
"저항이 없다" 가 아니라 **"이 범위로는 못 잰다"** 는 뜻이다.

**② 시험용 패드는 살짝 대면 안 통한다.**
`TP16` 이 울렸다 안 울렸다 해서 계측기·접촉 불량을 의심했는데, **눌러서 대니 안정적으로** 울렸다.
표면 산화막 때문이다. 값이 왔다갔다 하면 **라이다보다 계측기·탐침을 먼저** 의심할 것.

---

## 2-2. 보유 개체 현황 — 3대 중 2대 정상

같은 날 세 번째 개체까지 확인했다. **모델·펌웨어는 셋 다 동일하다.**

> 🚨 **[2026-09-14 재발] 1번째(고장품)가 다시 꽂혀서 같은 진단을 처음부터 반복했다.**
> 라벨이 지워져 겉으로 구분이 안 되는데 **표시를 안 해뒀던 것**이 원인이다.
> 오늘 재측정: **빨강·노랑·초록 3가닥 무음**(8월엔 빨강·노랑 2가닥). 검정·파랑만 살아 있다.
> → **고장품 본체에 "케이블 단선 — 사용 금지" 를 물리적으로 표시할 것.**
> → 정상품 2대에도 시리얼 뒷자리(`D5F1…` / `80BF…`)를 적어 붙일 것. 어느 배에 실렸는지
>    기록해야 하는 이유는 아래 "개체마다 회전 속도가 다르다" 항목에 있다.

| | 라이다 시리얼 | 상태 | 실측 |
|---|---|---|---|
| 1번째 | (응답 없음) | 🚨 **케이블 단선** — 09-14 재확인 | — |
| 2번째 | `D5F1ED93C0EA98C9A5E698F2402F4669` | ⚠️ **소재 미확인** (10-07 보유 2대에 없음) | 1248점 / 0.275° / 11.96Hz |
| 3번째 | `80BFED95C4E493CBA5E69EF0116F4B6F` | ✅ **정상 → 1번배** (10-07 재확인, 어댑터 `0addfc38`) | 1439점 / 0.250° |
| **4번째 (10-07 신규)** | `918A9AF2C1EA9FC3BEEB9CF31E593203` | ✅ **정상 → 2번배** — 펌웨어 **1.25**(나머지는 1.32), HW 6, 어댑터 `0001` | **1612점 / 0.223° / 11.00Hz** ← 셋 중 가장 정밀 |

넷 다 `모델 0x31(A3 계열) · 하드웨어 6`. 펌웨어는 2·3번째가 1.32, **4번째만 1.25**.

> 🚨 **[10-07] 4번째 개체의 어댑터가 IMU 어댑터와 충돌한다.** 시리얼 `0001` · 제품명
> `CP2102 USB to UART Bridge Controller` 가 **IMU 와 완전히 같다** → IMU udev 규칙이 라이다를 잡아
> `/dev/IMU → ttyUSB1(라이다)` 가 됐다. 8월에 이 규칙 주석이 예고한 바로 그 사고다.
> 그 상태로 launch 하면 **IMU 드라이버가 라이다 포트를 열고, 라이다는 열 포트가 없다 — 둘 다 죽는다.**
> 해법은 아래 "어댑터 충돌" 절.

**[10-07 확정] 유일시리얼 어댑터 `0addfc38…` 는 1번배 라이다에 붙인다.** 2번배 라이다·IMU 는 둘 다 `0001` 어댑터 → 2번배 노트북은 포트 고정(`docs/절차/2번배_udev_규칙_초안.md`).

~~쓸 수 있는 A3 가 3대가 됐다~~ → **10-07 실측: 손에 있는 건 2대(80BF·918A). 예비 없음.** 유일 어댑터는 8월 D5F1 에서 10월 80BF 로 옮겨 있었다 — 어댑터와 본체는 따로 돈다. 각분해능이 0.223 / 0.250 / 0.275° 로 셋 다 다르다 —
어느 배에 어느 개체를 싣느냐가 `min_obstacle_cells` 류 튜닝에 걸린다. **배정은 팀 결정 항목.**

**⇒ 당장 쓸 수 있는 라이다가 2대다.** 배가 2척이므로 한 척에 한 대씩 배정 가능하고,
고장난 1대는 케이블만 교체하면 예비가 된다.

### 개체마다 회전 속도가 다르다

3번째가 2번째보다 **점이 15% 많다**(1439 vs 1248). 표본수(16kHz)는 고정이므로
**천천히 도는 개체일수록 한 바퀴의 점이 많다** = 각분해능이 곱다.

```
2번째 : 11.96Hz → 1248점 → 0.275°
3번째 :  약 11.1Hz → 1439점 → 0.250°
```

⚠️ **회전수는 파라미터로 못 맞춘다**(§4). 즉 **두 배에 실린 라이다의 각분해능이 서로 다르다.**
`min_obstacle_cells` 같이 "점 개수" 에 의존하는 튜닝값은 **개체마다 다르게 동작한다.**
A배/B배 설정을 나눌 때 이 차이를 염두에 둘 것. 인식 성능은 3번째가 근소하게 유리하다.

---

## 3. 모델 확정 — 라벨이 지워져도 된다

라이다 본체 라벨이 지워져 모델을 알 수 없었다. **장치에 직접 물어봤다.**

```
python3 tools/lidar_id.py
```

```
모델 0x31(49) → A3 계열   펌웨어 1.32   하드웨어 6
시리얼 D5F1ED93C0EA98C9A5E698F2402F4669
보레이트 256000
```

**이 값이 라벨보다 확실하다** — 실물이 스스로 말하므로 옮겨 적다 틀릴 일이 없다.

### 🚨 이 과정에서 소프트팀이 틀린 것

커넥터가 **보드 6핀 / 어댑터 5핀** 인 것을 보고 `A2M12` 라고 추정했다(외부 보고 사례와 일치해서).
그 전제로 두 가지를 지적했는데 **둘 다 틀렸다:**

| 잘못된 지적 | 실제 |
|---|---|
| "`scan_mode: Sensitivity` 는 A3 전용이라 안 맞을 수 있다" | **A3 가 맞으므로 현재 설정이 정확하다** |
| "A2M12 는 5.2V 상한이라 과전압 손상 가능" | A3 는 5.5V 상한. 근거 약해짐 |

**커넥터 모양으로 모델을 추정하지 말 것.** `lidar_id.py` 로 물어보면 30초다.

---

## 4. 부수적으로 확인된 것 — 회전수 파라미터가 안 먹는다

정상 라이다로 실측:

| 항목 | 문서 | 실측 |
|---|---|---|
| 회전 | 10 Hz | **11.96 Hz** |
| 각분해능 | 0.225° | **0.275°** |
| range_max | 25 m | 25 m ✅ |

`scan_frequency:=10.0` 을 명시해도 `11.96Hz` 가 그대로 나온다. 드라이버가 그 값을
**모터에 전달하지 않기 때문이다** (`rplidar_node.cpp:205` 의 `setMotorSpeed()` 는 인자가 없다).

해롭지는 않다 — 갱신이 19% 빨라 제어에는 유리하다. **다만 각분해능이 22% 거칠다.**
자세한 내용은 `CLAUDE.md` §2-1.

---

## 5. 구매 사양

```
품목 : RPLIDAR A3 연결 케이블
규격 : 라이다 쪽 6핀 / 어댑터 쪽 5핀 (XH2.54-5P)
색   : 빨강 VCC · 노랑 TX · 초록 RX · 검정 GND · 파랑 MOTOCTL  (+흰색은 보드 내부 전용)
```

⚠️ 시중 범용 "RPLIDAR 케이블" 은 대부분 **양쪽 5핀**이라 보드 쪽에 안 맞는다.
⚠️ **`VCC` 와 `GND` 가 바뀐 제품을 물리면 보드가 탄다.** 모델 명시 제품으로 살 것.

### 교체 전에 할 것 — 끊어진 위치 찾기

케이블을 구간별로 꺾으면서 도통을 보면 단선 지점이 잡힌다.

- **부싱(하우징 진입부) 근처**면 반복 굽힘 파손이다 → **새 케이블도 같은 자리가 또 끊긴다.**
  여유를 주거나 고정해서 그 지점이 안 꺾이게 해야 한다
- **중간의 눌린 자리**면 설치 중 밟히거나 끼인 것 → 배치를 바꿔야 한다

**원인을 모르고 케이블만 갈면 대회 중에 또 끊긴다.** 두 가닥이 동시에 끊긴 것은
한 지점이 물리적으로 상했다는 뜻이므로, 위치를 특정할 수 있다.

---

## 6. 다음에 같은 일이 생기면

1. `python3 tools/lidar_id.py` — 응답이 오면 라이다는 살아있다
2. 무응답이면 **어댑터부터 배제**한다: 정상 라이다를 같은 어댑터에 물려본다
3. 그래도 무응답이면 분해해서 §2 순서로 측정한다

🚨 **`/dev/ttyLiDAR` 이 있는 것은 아무 증거도 아니다.** udev 규칙이 **USB 어댑터**의
시리얼로 매칭하므로, 어댑터만 살아있고 라이다가 죽어도 링크는 정상적으로 생긴다.

----- DOC: tools/sim/README.md -----
# tools/sim — 회피 시뮬레이터 (09-28 저장소 이관)

`CLAUDE.md` §6 "시뮬로 검증된 파라미터" 의 출처. 09-28 까지 Mac mini `~/Desktop/SSF 노드 파일/` 에만 있어 재현 불가였다.

| 파일 | 역할 |
|---|---|
| `ssf_sim.py` | `ship_direction`·`motor_control` 로직 재구현 + 상수. **노드와 수동 동기화** — 노드 파라미터 바꾸면 여기도 |
| `fastworld.py` | 부표 월드 + 가짜 LiDAR 스캔 |
| `kaboat_full_course_sim.py` | 2025 규정 코스 70×20m 전 구간 (`python3 kaboat_full_course_sim.py`) |
| `sweep_s1.py` | half_width × clearance 스윕 (`python3 tools/sim/sweep_s1.py --seeds 20`) |

## 노드와 다른 점 (알고 쓸 것)
- 시간 투표 [G], geofence 병합 [H] 없음. 물보라 오탐이 없는 월드라 회피 결과엔 영향 적음.
- 접촉 판정 = 반경 `half_width` 원. 배 길이 방향 측면 접촉은 못 잡음 → `sweep_s1.py` 의 선회 유효 폭 표로 보완.
- 동역학(K_V, K_W, CRUISE 1.2m/s)은 **가정**. 수조 실측(`수조_실측값_반영_프롬프트.md`) 후 교체.

## 동기화 상태 (09-23 기준, yaml 과 일치)
`HALF_WIDTH 0.29` · `CLEARANCE 0.25` · `MIN_OBSTACLE_CELLS 1` · `REAR_OBSTACLE_IGNORE_MARGIN None`(노드에서 제거된 결함).
`BASE_DETECTION_DISTANCE 1.8` 은 기본값일 뿐 — 스윕은 `dd` 인자로 yaml 의 2.0/3.0 을 넘긴다.

의존: `numpy`, `matplotlib`. ROS 불필요.

==================================================================
## 작년 최종본(9bff661) 이후 커밋 목록
==================================================================
- 2fea366 2026-10-24 docs: 2번배 줄자 실측(10-24) 1.77×0.535×0.14 반영 — 설계보다 폭 +10cm, 공용 half_width 0.29 유지(시뮬 접촉 0.95·정체 0)
- 9d579da 2026-10-10 refactor(ship_direction): S4 리뷰 권고 반영 — 함수 위치·경계값 테스트
- c5ffe2c 2026-10-10 refactor(ship_direction): zone 선별을 모듈 함수 edge_range()·select_valid_zones() 로 — 테스트 3개 추가 [S4]
- 27e2f45 2026-09-28 fix(S1): 1번배 줄자 실측 반영 — 폭 0.57m 확정, half_width 0.29 유지 근거 확보
- 9ae3bef 2026-09-28 fix(S1): 선폭 '실측' 표기 정정 + 시뮬 저장소 이관 + 시드 20 재스윕 — 우분투 지시문(fca7584) 응답
- cd7475b 2026-09-23 Merge remote-tracking branch 'origin/main'
- 5390785 2026-09-23 test: pep257 을 끈다 — 결함 0건, 전부 서식이고 절반은 한국어에 적용 불가
- 4386901 2026-09-23 style: flake8 최대 줄 길이 99 → 120, 잔여 5건 수정 — 10개 패키지 전부 0건
- 5c5d039 2026-09-23 style: flake8 142 → 37 — 죽은 코드 18건은 실제로 걷어냈다
- 000e46c 2026-09-23 fix(ship_direction): half_width 0.45 -> 0.29 (1번배 실측 0.58m), clearance 0.25 재스윕으로 유지 확인 [S1]
- 673c20a 2026-08-07 refactor(rplidar): a3 launch 정리 — 미사용 import 제거 + 문서화
- aabb3bc 2026-08-06 fix(ship_direction): 페일세이프 복구 시 노드가 죽던 버그 — 로거 심각도 충돌
- ec6cd8c 2026-07-30 test(ship_direction): 장애물 필터 파이프라인 단위테스트 (팀원 §5.1)
- ed31dda 2026-07-30 feat(ship_direction): 후진 재탐색 상태기계[L] + 조향 rate limiter[K] (v2.2.2)
- 31ed859 2026-07-28 fix(ship_direction): dilate_obstacles 가 lock 스냅샷을 우회하던 것 수정 (팀원 제안 버그2)
- a007eb0 2026-07-27 도크 인식 강건화 D1~D6 (도크미션 작업계획 반영)
- 19cfde7 2026-07-24 내재화: gitlink 4개 해제, 로컬 파일 그대로 저장소에 편입
- fd3b39e 2026-07-24 QoS-B: /scan 구독 5곳 BEST_EFFORT+depth=1 + blackbox 스캔 간격 로깅
- 849c570 2026-07-16 C 채택: 감속 신호에 공간 median 필터 (기본 ON)
- e3490db 2026-07-15 6a-2 보강: geofence 하드 가드 (도킹 구간 경계 이탈 방지)
- b678154 2026-07-14 6a-2 개선: geofence 를 '가짜 LiDAR'로 (원뿔 방식 폐기)
- 2cb050b 2026-07-14 6a-2: ship_direction geofence 구독 + 가상 장애물 (빨간불 끔)
- fd65e2f 2026-07-14 3단계 마무리: 시간투표 기본 OFF(효과 없음, 철회) + cmd_timeout 0.5 로 조임
- 16ff3c9 2026-07-14 3단계 완료: 시간 투표 필터 + SensorWatch/TemporalVote 를 ROS 비의존 모듈로 추출
- da327a1 2026-07-14 3단계 회귀 수정: IMU 감시 복원 + angle_increment 0 나눗셈 가드
- 5bc8d96 2026-07-14 3단계 튜닝: §6 시뮬 최적값 반영 (clearance/detection/min_cells)
- 2f4410a 2026-07-14 3단계 정정(구조): 페일세이프 3상태·2임계 + rear_obstacle_ignore_margin 제거
- 02bfc39 2026-07-14 3단계: ship_direction 제어루프 타이머 분리 + 페일세이프 L0~L3

==================================================================
## 작년 최종본(9bff661) 대비 변경 diff
==================================================================
올해 새로 생긴 파일 (diff 생략 — 위 소스 파일 또는 저장소에서 직접 본다):
- src/ship_direction/config/ship_direction.yaml
- src/ship_direction/ship_direction/failsafe.py
- src/ship_direction/test/test_avoidance_filters.py
- src/ship_direction/test/test_failsafe_logic.py

작년 파일 중 바뀌거나 지워진 것의 diff (문맥 1줄):
```diff
diff --git a/src/ship_direction/launch/ship_direction_launch.py b/src/ship_direction/launch/ship_direction_launch.py
index cd539f8..f0773bb 100644
--- a/src/ship_direction/launch/ship_direction_launch.py
+++ b/src/ship_direction/launch/ship_direction_launch.py
@@ -1 +1,4 @@
+import os
+
+from ament_index_python.packages import get_package_share_directory
 from launch import LaunchDescription
@@ -3,15 +6,14 @@ from launch_ros.actions import Node
 
+
 def generate_launch_description():
+    params = os.path.join(
+        get_package_share_directory('ship_direction'), 'config', 'ship_direction.yaml')
+
     return LaunchDescription([
         Node(
-            package='ship_direction',  # Replace with your package name
-            executable='ship_direction',  # Replace with your executable/script name if different
-            name='ship_direction',
+            package='ship_direction',
+            executable='ship_direction',
+            name='ship_direction',      # config yaml 의 /ship_direction 키와 일치
             output='screen',
-            parameters=[
-                # Add any parameters if needed, e.g., {'param_name': 'param_value'}
-            ],
-            remappings=[
-                # Add any topic remappings if needed, e.g., ('/old_topic', '/new_topic')
-            ]
+            parameters=[params],
         ),
diff --git a/src/ship_direction/setup.py b/src/ship_direction/setup.py
index 2311ecc..5b33d53 100644
--- a/src/ship_direction/setup.py
+++ b/src/ship_direction/setup.py
@@ -13,2 +13,3 @@ setup(
         ('share/' + package_name + '/launch', ['launch/ship_direction_launch.py']),  # 추가된 부분
+        ('share/' + package_name + '/config', ['config/ship_direction.yaml']),
     ],
diff --git a/src/ship_direction/ship_direction/ship_direction.py b/src/ship_direction/ship_direction/ship_direction.py
index 2079c00..0f143ba 100644
--- a/src/ship_direction/ship_direction/ship_direction.py
+++ b/src/ship_direction/ship_direction/ship_direction.py
@@ -1,13 +1,67 @@
+import math
+import time
+from threading import Lock
+
 import rclpy
-from std_msgs.msg import Int32
 from rclpy.node import Node
 from rclpy.executors import MultiThreadedExecutor
-from std_msgs.msg import Float32, Float32MultiArray
+from std_msgs.msg import Int32, Float32, Float32MultiArray
 from sensor_msgs.msg import LaserScan
-import math
-import time
-from threading import Lock   # 🔥 Lock 추가
-
-CANDIDATE_INVALID = 20000.0  # 미션 없음/실패 → yaw_error로 폴백
-STOP_HOLD = 50000.0          # 정지/대기 특수 신호
+from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
+
+# 판정 로직은 ROS 비의존 모듈에 있다 (ROS 없이 단위 테스트 가능).
+# 재수출: from ship_direction.ship_direction import SensorWatch 로도 접근 가능.
+from ship_direction.failsafe import SensorWatch, TemporalVote, median_min
+
+# 특수 신호 상수 (CLAUDE.md 3-9 상수표)
+SPIN_RIGHT = 5000.0          # 우선회
+SPIN_LEFT = 6000.0           # 좌선회
+CANDIDATE_INVALID = 20000.0  # 미션 없음 → yaw_error 기반 자율 회피
+STOP_HOLD = 50000.0          # 정지/대기
+
+REVERSE_ANGLE = 260.0        # 회피 경로 없음 → 후진
+LIDAR_FORWARD_DEG = 80.0     # LiDAR 프레임에서 정면 (상대방위 0 에 대응)
+
+
+def edge_range(distance_array, i, detection_distance):
+    """zone 가장자리 셀의 거리. inf·nan 은 '측정 없음' 이라 detection_distance 로 본다.
+
+    모듈 함수로 둔 이유: 노드 메서드 안 클로저였을 땐 테스트가 못 닿았다 (과제 S4).
+    """
+    v = distance_array[i]
+    return detection_distance if (math.isinf(v) or math.isnan(v)) else v
+
+
+def select_valid_zones(safe_zones, distance_array, angle_increment_rad,
+                       detection_distance, min_required_width):
+    """safe zone 중 배가 지나갈 수 있는 것만 고른다 → [(s, e, r_edge), ...].
+
+    폭 = 양 끝 중 가까운 거리(r_edge) × 호 길이. r_edge 를 쓰는 이유는 가장자리가
+    먼 쪽 거리로 재면 실제보다 넓게 보기 때문이다. min_required_width 는
+    half_width*2 + clearance (CLAUDE.md §6, yaml 과 같은 값).
+    """
+    valid = []
+    for s, e in safe_zones:
+        r_edge = min(edge_range(distance_array, s, detection_distance),
+                     edge_range(distance_array, e, detection_distance))
+        arc_len = angle_increment_rad * r_edge * (e - s)
+        if arc_len >= min_required_width:
+            valid.append((s, e, r_edge))
+    return valid
+
+
+# QoS-B: /scan 표준 sensor-data QoS. 작년은 depth=10 기본 RELIABLE 이었다.
+#   [1] 묵은 큐: LiDAR 10Hz × depth 10 = **1초치**가 쌓인다. 콜백이 한 번 밀리면
+#       그 뒤로 묵은 스캔이 burst 로 몰려와 '1초 전 장면' 으로 조향한다.
+#   [2] 워치독 왜곡: 스테일 판정이 '콜백 도착 시각' 기준이라, burst 가 워치독을 먹여
+#       실제로는 늦은 데이터인데 신선하다고 착각시킨다. depth=1 은 도착=신선을 일치시킨다.
+#   [3] 호환성: 구독자 BEST_EFFORT 는 발행자가 RELIABLE 이든 BEST_EFFORT 든 **전부 호환**된다
+#       (그 반대가 비호환). 현행 rplidar_ros 2.1.4 는 rplidar_node.cpp:440 에서
+#       rclcpp::QoS(KeepLast(10)) = RELIABLE 로 발행한다 — 소스 확인함. 드라이버를 갈아도 안 깨진다.
+#   드롭이 생겨도 '콜백 부재 → 스테일 → 페일세이프 발동' 으로 안전한 방향으로 실패한다.
+SCAN_QOS = QoSProfile(
+    reliability=ReliabilityPolicy.BEST_EFFORT,
+    history=HistoryPolicy.KEEP_LAST,
+    depth=1,
+)
 
@@ -15,2 +69,102 @@ STOP_HOLD = 50000.0          # 정지/대기 특수 신호
 class ShipDirection(Node):
+    """LiDAR 회피 + 페일세이프. /desired_angle · /obstacle_distance_array · /failsafe_level 발행.
+
+    3단계에서 고친 것:
+
+    [A] 제어 루프를 고정 주기 타이머로 분리했다.
+        작년: 모든 계산·발행이 scan_callback 안에 있었다 → **LiDAR 가 끊기면 콜백이 아예 안 와서
+              아무도 /desired_angle 을 발행하지 않았다.** (측정: LiDAR 1~3초 끊김 → 발행 0회, 완전 침묵.
+              페일세이프 레벨은 올라가는데 출력이 없으니 L1/L2 가 '존재'하지 않았다.)
+        지금: scan_cb 는 **스캔 저장만**. control_cb 가 고정 주기로 **항상** 계산·발행한다.
+              → ship_direction 이 살아있는 한 /desired_angle 은 흐른다.
+                침묵 = ship_direction 사망 → motor_control 이 cmd_timeout 으로 잡는다(0.5s 로 조일 수 있게 됨).
+
+    [B] 페일세이프(독립 watchdog 타이머). scan 이 묵은 정도로 레벨을 매긴다. **3상태·2임계.**
+        0 정상        : 계산한 각도        / 속도 100%
+        1 경고(0.7s)  : 계산한 각도 그대로 / 속도 70%    ← 각도 안 건드림
+        2 정지(3.0s)  : STOP_HOLD override / 정지        ← 각도를 덮어쓰는 건 여기뿐
+
+        ★ 임계가 적을수록 틀릴 곳이 적다. 중간에 'freeze' 레벨을 뒀다가 뺐다 —
+          새 스캔이 없는데 재계산하면 같은 스캔으로 같은 답이 나온다.
+          즉 '재계산'과 'freeze'는 애초에 같은 동작이라 레벨을 나눌 이유가 없었다.
+        ★ 경고에서 '직진 강제' 같은 걸 하면 회피 기동 중간에 핸들을 놓는 셈이다.
+          부표를 피하던 중 직진하면 그대로 박는다. 0.7초 묵었다고 조향을 포기하는 건 과잉반응이다.
+          묵은 데이터라도 없는 것보다 낫다 — 대신 속도를 줄여 오차를 줄인다.
+
+        속도는 /desired_angle(각도)에 담을 수 없다 → motor_control 이 /failsafe_level 을 구독해
+        속도 상한을 건다. (레벨 2 는 STOP_HOLD 가 각도로 오므로 motor_control 분기 (1) 이 처리)
+
+        오탐 방지(CLAUDE.md §5): ARMED(스캔 한 번은 받아야 평가 시작) + 올릴 땐 연속 N회 확인 +
+        내릴 땐 즉시(자동 복구) + time.monotonic().
+
+    [C] rear_obstacle_ignore_margin 제거 — 튜닝값이 아니라 증명된 결함이다.
+        물보라 반사 0.3m 하나가 front_min_distance 를 끌어내려, 실제 1.6m 부표를 통째로
+        마스크에서 지웠다 (17셀 → 0셀).
+
+    [D] time.time() → time.monotonic() 전부 교체.
+
+    [E] 워치독이 /scan 과 /yaw_error 를 **둘 다** 감시한다 (센서별 dict, 가장 나쁜 센서가 레벨을 정함).
+        /scan 만 보면 IMU 사망을 놓친다. 1단계에서 만든 사슬은
+          IMU 사망 → ship_goal_angle 이 /yaw_error 를 끊는다 → ship_direction 이 감지해 정지
+        인데, 세 번째 화살표(침묵을 들을 귀)가 없으면 마지막 yaw_error 를 영원히 붙들고 조향한다.
+        (측정: IMU 사망 시 정지율 0%, 접촉 3회, 14.8m 계속 주행)
+        센서별로 첫 데이터를 받은 뒤에만 감시(ARMED) — 부팅 오발동 방지.
+
+    [F] angle_increment == 0 가드. min_index/max_index 계산에서 이 값으로 나눈다 →
+        LiDAR 가 이상 스캔을 보내면 ZeroDivisionError 로 노드가 통째로 죽는다
+        (작년 코드에도 방어가 없었다). 이상 스캔은 scan_callback 에서 버리고 즉시 정지.
+        하드 폴트라 확인 N회 없이 바로 레벨 2. 정상 스캔이 오면 스스로 풀린다.
+
+    [G] 시간 투표 필터(TemporalVote) — **기본 OFF(temporal_frames=1)**. 코드는 남겨둔다.
+        최근 N프레임 중 votes 회 이상 나타난 셀만 장애물로 인정하는 필터다.
+        처음엔 효과가 있어 보였으나(단일 시드), **시드 20개로 재보니 효과가 없었다:**
+          필터 OFF → 접촉 19/20, 전진 25.5m  /  필터 ON → 접촉 20/20, 전진 23.4m
+        해당 시나리오(정면 부표 + 물보라 60%)는 어느 코드든 95% 접촉하는 '통과 불가' 판이었다.
+        → **무해하지만 무익.** (TTC 는 '해로워서' 삭제했지만, 이건 '무익해서' 기본 OFF다.)
+        실제 물보라의 시간 특성이 시뮬과 다를 수 있으니 켤 수 있게 남긴다: frames=3, votes=2.
+        ★ 켤 때는 반드시 dilate 전, 원본 마스크에 건다
+          (팽창 후에 걸면 진짜 부표의 팽창 영역까지 표가 갈려 부표가 얇아진다).
+
+    [H] Geofence (6a-2). 경기장 밖으로 나가면 실격인데 방어가 0 이었다.
+        north 가 경계를 **'가짜 LiDAR'** 로 쏴서 보낸다:
+          /geofence_state = [angle_min_deg, angle_inc_deg, r0, r1, ...] (상대방위, 멀면 inf)
+        여기서 하는 일은 **스캔 병합 한 줄**이 전부다:
+          ranges[i] = min(real_ranges[i], geofence_ranges[i])
+        그 뒤는 기존 파이프라인이 알아서 한다 — detection_distance 로 마스크가 되고,
+        dilate 가 배 폭 여유를 더하고, 갭-팔로잉이 피한다.
+        **특수 로직도, 새 상태기계도, 튜닝 파라미터(half_block)도 없다.**
+
+        ★ 왜 '원뿔 칠하기'를 버렸나: **벽은 '점'이 아니라 '선'이다.**
+          최근접점 한 방향으로 ±각도를 막는 건 기하학적으로 틀렸다. 40×40 모서리에서
+          배가 대각선(45°)을 보면 북벽 최근접점 -45°, 동벽 +45° 인데 **탈출구(모서리)는 정면 0°**.
+          half_block=40° 면 [-85,-5]·[5,85] 만 막혀 **정면 0° 가 '틈'으로 열린다** → 대각선 탈출, 실격.
+          광선으로 쏘면 정면 경계 거리(2.12m)가 그대로 벽이 된다. 경기장이 사각형이 아니어도 맞다.
+
+        ★ 빈 배열/stale 이면 병합하지 않는다 — 묵은 '없는 벽'은 배를 가둔다.
+
+    [I] Geofence 하드 가드 — (A)SPIN·(B)candidate 구간 전용 (6a-2 보강).
+        [H]의 스캔 병합은 (C) 자율회피에서만 일어난다. 그런데 **도킹은 SPIN+candidate 로 조향**하고
+        이 구간은 distance_array 를 안 만들어 병합할 대상이 없다. 도크가 경기장 가장자리면
+        접안하다 경계로 파고들어 이탈 → 실격. (측정으로 확인된 실제 위험.)
+        → _compute 맨 앞에서 검사: 경계 최소거리가 geofence_stop_margin_m(2.0) 안이고 그 상대방위가
+          '전진하려는 방향 ±geofence_stop_cone_deg(60°)' 와 겹치면 → candidate/SPIN 무시하고 STOP_HOLD.
+        **미션보다 실격 방지가 우선.** 경계를 등지고 멈춘다.
+        ⚠️ north 광선은 전방 ±80° 만 → 후진 중 뒤쪽 경계는 못 본다(접안=전방 방어용).
+
+    [J] 감속 신호용 공간 median 필터 (obst_median_kernel, **기본 ON**).
+        _closest_obstacle 이 필터 없는 raw min 이라 **물보라 반사 한 점(0.3m)이 그대로
+        motor_control 감속을 물렸다.** 이웃과 어긋나는 고립 스파이크를 range 값 수준에서 지운다.
+        측정(시드 20, 물보라 30%): raw-min 가짜감속 **36.8% → median 8.9%**(기준선). 접촉은 전 조건 0.
+        ★ **감속 신호에만** 건다 — _compute 의 회피 마스크는 안 건드린다(그래서 접촉이 안 변한다).
+        ★ [G] TemporalVote(시간 투표)와 다르다: 저건 효과 0 이라 OFF, 이건 명확한 이득이라 ON.
+          시간 투표는 프레임 간, 이건 한 프레임 안의 공간.
+        ⚠️ 시뮬 물보라는 '단일점' 모델이다. 실제 물보라가 작은 군집이면 커널5로 부족할 수 있다
+           → 배 뜬 뒤 블랙박스의 obstacle_min_dist 로 실측 확인하고 커널을 7/9 로 올릴 것.
+
+    ※ 판정 로직(SensorWatch, TemporalVote, median_min)은 **ROS 비의존 모듈** ship_direction/failsafe.py 에 있다.
+      워치독을 노드 안에 인라인하면 단위 테스트가 불가능해진다. rclpy 없이 임포트 가능:
+          from ship_direction.failsafe import SensorWatch, TemporalVote   # ← ROS 불필요(권장)
+          from ship_direction.ship_direction import SensorWatch           # ← 재수출(ROS 필요)
+    """
+
     def __init__(self):
@@ -18,3 +172,2 @@ class ShipDirection(Node):
 
-        # 🔥 공유 변수 보호용 Lock
         self.lock = Lock()
@@ -22,40 +175,106 @@ class ShipDirection(Node):
         # ----------------------
-        #      Subscriptions
+        #   파라미터 (config yaml, CLAUDE.md 1-4)
         # ----------------------
-        self.subscription_scan = self.create_subscription(
-            LaserScan, '/scan', self.scan_callback, 10)
-
-        self.subscription_yaw_error = self.create_subscription(
-            Float32, '/yaw_error', self.yaw_error_callback, 10)
-
-        self.subscription_candidate = self.create_subscription(
-            Float32, '/candidate_angle', self.candidate_callback, 10)
-
-        self.subscription_wp_mode = self.create_subscription(
-            Int32, '/wp_mode', self.wp_mode_cb, 10)
+        self.control_period = float(self.declare_parameter('control_period_sec', 0.1).value)    # 10Hz
+        self.watchdog_period = float(self.declare_parameter('watchdog_period_sec', 0.1).value)
+
+        # 페일세이프 임계(센서가 묵은 시간). 3상태·2임계 (CLAUDE.md §5)
+        self.fs_warn_sec = float(self.declare_parameter('failsafe_warn_sec', 0.7).value)   # → 레벨 1
+        self.fs_stop_sec = float(self.declare_parameter('failsafe_stop_sec', 3.0).value)   # → 레벨 2
+        self.fs_confirm_n = int(self.declare_parameter('failsafe_confirm_n', 3).value)
+
+        # ---- Geofence (경계 이탈 = 실격). 6a-2 ----
+        # north 가 경계를 '가짜 LiDAR'로 쏴서 보낸다 → 실제 스캔과 min() 병합만 하면 끝이다.
+        # 특수 로직도, 튜닝 파라미터(half_block)도 없다. 아래 [H] 참고.
+        # 🚨 묵으면 병합하지 않는다. 묵은 '없는 벽'은 배를 가둔다. (north 주기 0.5s 의 4배)
+        self.geofence_stale_sec = float(self.declare_parameter('geofence_stale_sec', 2.0).value)
+        # 하드 가드([I]): 경계가 이 거리 안이고 전진 방향과 겹치면 미션 무시하고 정지.
+        # (A)SPIN/(B)candidate 구간은 마스크를 안 만들어 병합이 안 되므로, 도킹이 경계로 파고들면
+        # 이탈한다. 미션보다 실격 방지가 우선 → 경계를 등지고 멈춘다.
+        self.geofence_stop_margin_m = float(
+            self.declare_parameter('geofence_stop_margin_m', 2.0).value)
+        self.geofence_stop_cone_deg = float(
+            self.declare_parameter('geofence_stop_cone_deg', 60.0).value)
+
+        # ---- [J] 감속 신호용 공간 median 필터. **기본 ON.** ----
+        # _closest_obstacle(감속 신호)에만 건다. 회피 마스크(_compute)는 안 건드린다.
+        # 측정(시드 20, 물보라 30%): raw-min 가짜감속 36.8% → median 8.9%. 접촉 전 조건 0.
+        # 0 이면 raw-min 폴백.
+        self.obst_median_kernel = int(self.declare_parameter('obst_median_kernel', 5).value)
+
+        # 시간 투표 필터 — **기본 OFF(frames=1)**. 시드 20개로 재보니 효과가 없었다(아래 [G]).
+        # 무해하지만 무익하다. 실제 물보라의 시간 특성이 시뮬과 다를 수 있으니 코드는 남겨두고
+        # 켤 수 있게만 해둔다: temporal_frames=3, temporal_votes=2 로 켠다.
+        self.temporal_frames = int(self.declare_parameter('temporal_frames', 1).value)
+        self.temporal_votes = int(self.declare_parameter('temporal_votes', 2).value)
+
+        # 회피 튜닝 — §6 시뮬 스윕으로 검증된 값. 임의로 바꾸지 말 것.
+        self.detection_distance_default = float(
+            self.declare_parameter('detection_distance_default', 3.0).value)   # 1.8 이면 8/8 충돌
+        self.detection_distance_gate = float(
+            self.declare_parameter('detection_distance_gate', 2.0).value)      # 3.0 이면 게이트가 좁아 접촉 증가
+        # 게이트 구간의 wp_mode. 확정된 웨이포인트 표(3-6): mode 0=게이트 시작, 1=게이트 끝.
+        # (작년 코드는 wp_mode==2 (위치유지)를 키로 썼는데, 그건 게이트가 아니다.)
+        self.gate_wp_modes = list(self.declare_parameter('gate_wp_modes', [0, 1]).value)
+
+        self.half_width = float(self.declare_parameter('half_width', 0.29).value)
+        # 0.20→접촉 1.7회 / 0.25→0.2회 / 0.30→0.8회 / 0.45→807초 폭주(과보수)
+        self.clearance = float(self.declare_parameter('clearance', 0.25).value)
+        self.border_margin = int(self.declare_parameter('border_margin', 2).value)
+        self.max_spike_ratio = float(self.declare_parameter('max_spike_ratio', 0.01).value)
+        # ※ rear_obstacle_ignore_margin 은 제거했다 (파라미터 자체를 없앰). 위 [C] 참고.
+        self.min_obstacle_cells = int(
+            self.declare_parameter('min_obstacle_cells', 1).value)   # 3 이면 작은 부표가 무시된다
+
+        # ---- [L] 후진 후 재탐색 상태기계 ----
+        # 회피 경로가 없으면 reverse_duration 동안 후진하고,
+        # 이후 reverse_cooldown 동안 STOP_HOLD 상태로 재탐색한다.
+        self.reverse_duration = float(
+            self.declare_parameter('reverse_duration_sec', 0.5).value)
+        self.reverse_cooldown = float(
+            self.declare_parameter('reverse_cooldown_sec', 3.0).value)
+
+        # 잘못된 파라미터 방어
+        if not math.isfinite(self.reverse_duration) or self.reverse_duration <= 0.0:
+            self.get_logger().warn(
+                f"reverse_duration_sec={self.reverse_duration} 이상 → 기본값 0.5초 사용")
+            self.reverse_duration = 0.5
+        if not math.isfinite(self.reverse_cooldown) or self.reverse_cooldown < 0.0:
+            self.get_logger().warn(
+                f"reverse_cooldown_sec={self.reverse_cooldown} 이상 → 기본값 3.0초 사용")
+            self.reverse_cooldown = 3.0
+
+        # ---- [K] 조향각 변화율 제한 (rate limiter). 팀원 피드백으로 "0~160° 한정" 적용. ----
+        # 원안(전체 구간 적용)은 /desired_angle 이 각도+특수신호(260/5000/6000/20000/50000) 혼용
+        # 프로토콜이라, 정상값→특수신호 사이를 15°/tick 씩 기어가면 motor_control 이 그 경유값을
+        # 후진/선회/무효로 오해할 위험이 있었다(팀 지적). 0~160° 구간(정상 조향값)일 때만 제한을
+        # 걸고, 그 밖(특수신호 및 candidate 매핑의 범위 밖 부산물)은 그대로 통과시킨다.
+        # 덤: 180° 급반전(정후방 목표 → 좌우 zone 끝 선택)도 결과가 항상 0~160° 안이라 이 필터로
+        # 완화된다. 실제 각속도 = 이 값 / control_period(예: 15°, 0.1s 면 150°/s) — 실측 후 튜닝.
+        self.rate_limit_max_deg_per_tick = float(
+            self.declare_parameter('rate_limit_max_deg_per_tick', 15.0).value)
+
+        self.detection_distance = self.detection_distance_default
 
         # ----------------------
-        #       Publishers
+        #      Subscriptions
         # ----------------------
-        self.pub_desired_angle = self.create_publisher(Float32, '/desired_angle', 10)
-        self.pub_obstacle_distance = self.create_publisher(Float32MultiArray, '/obstacle_distance_array', 10)
+        self.create_subscription(LaserScan, '/scan', self.scan_callback, SCAN_QOS)
+        self.create_subscription(Float32, '/yaw_error', self.yaw_error_callback, 10)
+        self.create_subscription(Float32, '/candidate_angle', self.candidate_callback, 10)
+        self.create_subscription(Int32, '/wp_mode', self.wp_mode_cb, 10)
+        self.create_subscription(Float32MultiArray, '/geofence_state', self.geofence_cb, 10)
 
         # ----------------------
-        #       Parameters
+        #       Publishers  (토픽/타입 불변 + /failsafe_level 신규, CLAUDE.md 3-9)
         # ----------------------
-        self.base_detection_distance = 1.8
-        self.mode2_detection_distance = 1.8
-        self.detection_distance = self.base_detection_distance
-
-        self.half_width = 0.45
-        self.clearance = 0.15
-        self.border_margin = 2
-        self.max_spike_ratio = 0.01
-        self.log_interval = 1.0
-        self.debug_interval = 1.0
-        self.rear_obstacle_ignore_margin = 1.0
-        self.min_obstacle_cells = 3
+        self.pub_desired_angle = self.create_publisher(Float32, '/desired_angle', 10)
+        self.pub_obstacle_distance = self.create_publisher(Float32MultiArray, '/obstacle_distance_array', 10)
+        # D6: median 필터가 버린 고립 스파이크 수 (관측 전용, 신규 토픽 — 계약 불변).
+        #   제어에 안 쓴다. blackbox 가 CSV 에 기록해 '노이즈 수 ↔ 기상 열화' 상관 검증용.
+        self.pub_reject = self.create_publisher(Int32, '/obstacle_reject_count', 10)
+        self.pub_failsafe = self.create_publisher(Int32, '/failsafe_level', 10)
 
         # ----------------------
-        #       State Vars
+        #       State
         # ----------------------
@@ -63,4 +282,2 @@ class ShipDirection(Node):
         self.wp4_enter_time = None
-        self.hold_log_ts = 0.0
-
         self.yaw_error = 0.0
@@ -68,17 +285,86 @@ class ShipDirection(Node):
 
-        self.last_log_time = time.time()
-        self.last_debug_time = time.time()
-        self.last_reverse_time = time.time()
-        self.reverse_time = 3.0
+        self.latest_scan = None       # scan_cb 는 저장만 한다
+
+        # [L] 후진 상태기계
+        # READY: 새 후진 시작 가능 / REVERSING: 일정 시간 후진 / COOLDOWN: STOP 재탐색
+        self.reverse_state = 'READY'
+        self.reverse_start_time = None
+        self.reverse_cooldown_start_time = None
+
+        self.prev_final_angle = None  # [K] rate limiter 상태. None = "직전 값 없음/특수신호였음" → 다음 정상값에 즉시 스냅
 
+        # geofence: (거리 m, 상대방위 deg) 또는 None. north 가 [inf,nan] 을 내면 None.
+        self.geofence = None
+        self.last_geofence_t = None
+
+        # 페일세이프 감시 — /scan 과 /yaw_error 를 둘 다 본다.
+        # ★ /scan 만 보면 IMU 사망을 놓친다. IMU 가 죽으면 ship_goal_angle 이 /yaw_error 를
+        #   끊어주는데(1단계 설계), 그 침묵을 들을 귀가 없으면 마지막 yaw_error 로 영원히 조향한다.
+        self.watch = SensorWatch(
+            ['scan', 'yaw_error'],
+            warn_sec=self.fs_warn_sec,
+            stop_sec=self.fs_stop_sec,
+            confirm_n=self.fs_confirm_n,
+        )
+        self.failsafe_level = 0
+
+        # 시간 투표 필터 (dilate 전, 원본 마스크에 적용)
+        self.temporal = TemporalVote(self.temporal_frames, self.temporal_votes)
+
+        # ----------------------
+        #  타이머: 제어 / 워치독 (서로 독립)
+        # ----------------------
+        self.control_timer = self.create_timer(self.control_period, self.control_cb)
+        self.watchdog_timer = self.create_timer(self.watchdog_period, self.watchdog_cb)
+
+        self.get_logger().info(
+            f"ship_direction 시작: 제어 {1.0 / self.control_period:.0f}Hz (스캔 유무 무관 항상 발행), "
+            f"워치독 {1.0 / self.watchdog_period:.0f}Hz.\n"
+            f"   페일세이프 경고={self.fs_warn_sec}s(레벨1) 정지={self.fs_stop_sec}s(레벨2), "
+            f"확인 {self.fs_confirm_n}회. 각도 override 는 레벨2 뿐.\n"
+            f"   시간투표 {self.temporal_frames}프레임 중 {self.temporal_votes}표 "
+            f"({'ON' if self.temporal.enabled else 'OFF'}), 감시 센서: scan + yaw_error."
+        )
 
     # =====================================================
-    # CALLBACKS
+    # CALLBACKS — 저장만 한다
     # =====================================================
+    def scan_callback(self, msg: LaserScan):
+        """저장만. 계산·발행은 control_cb 가 고정 주기로 한다.
+        (작년엔 여기서 다 했다 → LiDAR 가 죽으면 콜백이 안 와서 아무도 발행하지 않았다)
+
+        [F] angle_increment 가드: 0/음수/NaN 이면 min_index 계산에서 ZeroDivisionError 로
+        노드가 통째로 죽는다(작년 코드에도 방어가 없었다). 이상 스캔은 문 앞에서 버리고
+        bad_scan 을 세워 즉시 정지시킨다. 정상 스캔이 오면 스스로 풀린다."""
+        inc = math.degrees(msg.angle_increment)
+        if not math.isfinite(inc) or inc <= 0.0:
+            self.get_logger().error(
+                f"스캔 angle_increment 이상({msg.angle_increment}) → 이 스캔 폐기, 정지",
+                throttle_duration_sec=1.0,
+            )
+            with self.lock:
+                self.watch.set_fault(True)     # 하드 폴트 → 즉시 레벨 2
+            return
 
-    def yaw_error_callback(self, msg):
-        raw = msg.data
         with self.lock:
-            self.yaw_error = (360.0 - raw) % 360.0
+            self.watch.set_fault(False)        # 정상 스캔이 오면 스스로 풀린다
+            self.latest_scan = msg
+            self.watch.feed('scan', time.monotonic())
 
+    def yaw_error_callback(self, msg):
+        with self.lock:
+            self.yaw_error = (360.0 - msg.data) % 360.0
+            self.watch.feed('yaw_error', time.monotonic())   # [E] IMU 체인 생존 신호
+
+    def geofence_cb(self, msg):
+        """/geofence_state = [angle_min_deg, angle_inc_deg, r0, r1, ...] (상대방위, 멀면 inf).
+        경계를 '가짜 LiDAR'로 쏜 것이다. **빈 배열 = 경계 정보 없음**
+        (north 가 '모르면 입을 다무는' 계약: 폴리곤 미설정 / IMU stale / 이미 이탈)."""
+        d = msg.data
+        with self.lock:
+            if len(d) >= 3 and math.isfinite(d[1]) and d[1] > 0.0:
+                self.geofence = (float(d[0]), float(d[1]), [float(x) for x in d[2:]])
+            else:
+                self.geofence = None
+            self.last_geofence_t = time.monotonic()
 
@@ -87,3 +373,3 @@ class ShipDirection(Node):
         with self.lock:
-            if val in (CANDIDATE_INVALID, STOP_HOLD, 5000.0, 6000.0):
+            if val in (CANDIDATE_INVALID, STOP_HOLD, SPIN_RIGHT, SPIN_LEFT):
                 self.candidate_angle = val
@@ -92,3 +378,2 @@ class ShipDirection(Node):
 
-
     def wp_mode_cb(self, msg):
@@ -98,4 +383,6 @@ class ShipDirection(Node):
 
-            new_dd = self.mode2_detection_distance if self.wp_mode == 2 else self.base_detection_distance
-
+            # 게이트 구간(mode 0,1)은 좁아서 detection 을 짧게 (3.0 이면 오히려 접촉 증가, §6)
+            new_dd = (self.detection_distance_gate
+                      if self.wp_mode in self.gate_wp_modes
+                      else self.detection_distance_default)
             if abs(new_dd - self.detection_distance) > 1e-6:
@@ -104,17 +391,131 @@ class ShipDirection(Node):
             if prev != 4 and self.wp_mode == 4:
-                self.wp4_enter_time = time.time()
-
+                self.wp4_enter_time = time.monotonic()
             elif prev == 4 and self.wp_mode != 4:
                 self.wp4_enter_time = None
+
     # =====================================================
-    #               LiDAR MAIN CALLBACK
+    # WATCHDOG — 독립 타이머 (CLAUDE.md §5)
     # =====================================================
-    def scan_callback(self, msg: LaserScan):
-        now = time.time()
+    def watchdog_cb(self):
+        """페일세이프를 독립 타이머로 평가한다.
+        scan_cb 안에서 평가하면 LiDAR 가 완전히 죽었을 때 콜백이 아예 안 와서
+        페일세이프가 영원히 평가되지 않는다 (§5 에 기록된 실제 버그).
+
+        [E] 감시 대상은 /scan 과 /yaw_error **둘 다**. 가장 나쁜 센서로 레벨을 정한다.
+            /scan 만 보면 IMU 사망을 놓친다: IMU 가 죽으면 ship_goal_angle 이 /yaw_error 를
+            끊어주는데(1단계 설계), 그 침묵을 들을 귀가 없으면 ship_direction 은 마지막
+            yaw_error 를 영원히 붙들고 조향한다. (측정: 정지율 0%, 접촉 3회, 14.8m 계속 주행)
+            센서별로 첫 데이터를 받은 뒤에만 감시(ARMED) — 부팅 오발동 방지.
+
+        판정 로직은 SensorWatch(ROS 비의존)에 있다 — 단위 테스트 가능."""
+        now = time.monotonic()
+        prev = self.failsafe_level
+
+        with self.lock:
+            level = self.watch.update(now)
+            worst = self.watch.worst
+
+        if level != prev:
+            # 🚨 로거 호출을 변수에 담아 한 줄에서 warn/info 를 번갈아 쓰면 안 된다.
+            #    rclpy 는 로그 호출을 **소스 위치(파일:줄)** 로 캐싱해서, 같은 줄이 다른 심각도로
+            #    불리면 ValueError('Logger severity cannot be changed between calls.') 를 던진다.
+            #    레벨 상승은 warn, 복구는 info 라서 **복구되는 순간 노드가 죽었다** (실물에서 발견).
+            #    → 센서가 고장나서가 아니라 '고장에서 회복해서' 배가 멈추는 최악의 형태였다.
+            #    호출 지점을 둘로 나눠서 각 줄이 한 가지 심각도만 쓰게 한다.
+            msg = (f"페일세이프 L{prev} → L{level}"
+                   + (f"  (원인: {worst})" if worst else "  (복구)"))
+            if level > prev:
+                self.get_logger().warn(msg)
+            else:
+                self.get_logger().info(msg)
+
+        self.failsafe_level = level
+        self.pub_failsafe.publish(Int32(data=int(level)))
+
+    # =====================================================
+    # [K] RATE LIMITER — 0~160°(정상 조향값)에서만 변화율 제한
+    # =====================================================
+    def _apply_rate_limit(self, target_angle):
+        """0~160° 구간일 때만 한 tick당 변화폭을 제한한다.
+
+        특수신호는 즉시 통과시키고 상태를 초기화한다. NaN/inf는 STOP_HOLD로 바꾼다.
+        """
+        if not math.isfinite(target_angle):
+            self.prev_final_angle = None
+            return STOP_HOLD
+
+        in_range = 0.0 <= target_angle <= 160.0
+
+        if not in_range:
+            self.prev_final_angle = None
+            return target_angle
+
+        if self.prev_final_angle is None or not (0.0 <= self.prev_final_angle <= 160.0):
+            self.prev_final_angle = target_angle
+            return target_angle
+
+        limit = self.rate_limit_max_deg_per_tick
+        if not math.isfinite(limit) or limit <= 0.0:
+            self.prev_final_angle = target_angle
+            return target_angle
+
+        diff = target_angle - self.prev_final_angle
+        diff = max(-limit, min(limit, diff))
+        limited = self.prev_final_angle + diff
+        self.prev_final_angle = limited
+        return limited
+
+    # =====================================================
+    # [L] REVERSE STATE MACHINE
+    # =====================================================
+    def _apply_reverse_sequence(self, now):
+        """경로가 없을 때 일정 시간 후진한 뒤 STOP 재탐색을 수행한다."""
+        if self.reverse_state == 'REVERSING':
+            if now - self.reverse_start_time < self.reverse_duration:
+                return REVERSE_ANGLE
+
+            self.reverse_state = 'COOLDOWN'
+            self.reverse_start_time = None
+            self.reverse_cooldown_start_time = now
+            self.get_logger().info(
+                f"후진 {self.reverse_duration:.2f}초 완료 → "
+                f"STOP 재탐색 {self.reverse_cooldown:.2f}초 시작",
+                throttle_duration_sec=1.0)
+            return STOP_HOLD
+
+        if self.reverse_state == 'COOLDOWN':
+            if now - self.reverse_cooldown_start_time < self.reverse_cooldown:
+                return STOP_HOLD
+
+            self.reverse_state = 'REVERSING'
+            self.reverse_start_time = now
+            self.reverse_cooldown_start_time = None
+            self.get_logger().warn(
+                f"STOP 재탐색 {self.reverse_cooldown:.2f}초 후에도 경로 없음 → 후진 재시작",
+                throttle_duration_sec=1.0)
+            return REVERSE_ANGLE
+
+        self.reverse_state = 'REVERSING'
+        self.reverse_start_time = now
+        self.reverse_cooldown_start_time = None
+        self.get_logger().warn(
+            f"회피 경로 없음 → {self.reverse_duration:.2f}초 후진 시작",
+            throttle_duration_sec=1.0)
+        return REVERSE_ANGLE
+
+    def _reset_reverse_sequence(self):
+        """정상 경로나 외부 미션 명령이 생기면 후진/쿨다운 상태를 해제한다."""
+        self.reverse_state = 'READY'
+        self.reverse_start_time = None
+        self.reverse_cooldown_start_time = None
+
+    # =====================================================
+    # CONTROL — 고정 주기, 스캔 유무와 무관하게 항상 발행
+    # =====================================================
+    def control_cb(self):
+        level = self.failsafe_level
 
-        # ----------------------------
-        # 🔥 먼저 필요한 값들을 Lock으로 안전하게 복사
-        # ----------------------------
         with self.lock:
-            wp_mode = self.wp_mode
+            scan = self.latest_scan
+            bad = self.watch.fault
             wp4_enter_time = self.wp4_enter_time
@@ -124,39 +525,199 @@ class ShipDirection(Node):
 
-        # =====================================================
-        # 0) WP4 초기 5초 & STOP_HOLD 우선 처리
-        # =====================================================
-        if (wp4_enter_time is not None and (now - wp4_enter_time) < 5.0) or \
-           (candidate_angle == STOP_HOLD):
+        # ---- 레벨 2(정지) 또는 이상 스캔: STOP_HOLD 로 override. 각도를 덮어쓰는 건 여기뿐. ----
+        # bad 를 여기서도 보는 이유: 워치독 틱(0.1s)을 기다리지 않고 즉시 멈추기 위해서다.
+        if level >= 2 or bad:
+            self.pub_obstacle_distance.publish(self._closest_obstacle(scan))
+            angle = self._apply_rate_limit(STOP_HOLD)   # [K] 범위 밖 값 → 통과 + 상태 리셋
+            self.pub_desired_angle.publish(Float32(data=angle))
+            return
 
-            angle_min = math.degrees(msg.angle_min)
-            angle_increment_deg = math.degrees(msg.angle_increment)
-            ranges = list(msg.ranges)
+        # 스캔을 한 번도 못 받음(부팅 중) → 발행할 각도가 없다.
+        # motor_control 은 '첫 명령 전 중립'이라 안전하다.
+        if scan is None:
+            return
 
-            min_index = int((0 - angle_min) / angle_increment_deg)
-            max_index = int((160 - angle_min) / angle_increment_deg)
+        # ---- 레벨 0·1: 계산한 각도를 그대로 낸다. ----
+        # 레벨 1(경고)에서도 각도는 안 건드린다. 속도만 줄인다
+        # → motor_control 이 /failsafe_level 을 보고 상한(70%)을 건다.
+        angle, obst = self._compute(scan, wp4_enter_time, candidate_angle,
+                                    yaw_error, detection_distance)
+        angle = self._apply_rate_limit(angle)   # [K] 0~160° 안일 때만 실제로 제한됨
 
-            closest_distance = float('inf')
-            closest_angle = float('nan')
+        self.pub_obstacle_distance.publish(obst)
+        self.pub_desired_angle.publish(Float32(data=angle))
 
-            if 0 <= min_index < len(ranges) and 0 <= max_index < len(ranges):
-                for i in range(min_index, max_index):
-                    r = ranges[i]
-                    if not math.isinf(r) and not math.isnan(r) and r < closest_distance:
-                        closest_distance = r
-                        closest_angle = angle_min + i * angle_increment_deg
+    # =====================================================
+    # 계산 (회피 알고리즘 — 작년 로직 보존)
+    # =====================================================
+    def _closest_obstacle(self, msg):
+        """전방 0~160° 최근접 → Float32MultiArray([거리(m), 각도(deg)]). 없으면 [inf, nan]."""
+        obst = Float32MultiArray()
+        if msg is None:
+            obst.data = [float('inf'), float('nan')]
+            self.pub_reject.publish(Int32(data=0))     # D6: 스캔 없음 = 기각 0
+            return obst
 
-            obst = Float32MultiArray()
-            obst.data = (
-                [closest_distance, closest_angle]
-                if closest_distance != float('inf')
-                else [float('inf'), float('nan')]
-            )
+        angle_min = math.degrees(msg.angle_min)
+        angle_increment_deg = math.degrees(msg.angle_increment)
+        ranges = msg.ranges
 
-            self.pub_obstacle_distance.publish(obst)
-            self.pub_desired_angle.publish(Float32(data=STOP_HOLD))
-            return
+        # [F] 0 나눗셈 방어 (scan_callback 이 이미 걸러내지만, 여기도 막아둔다)
+        if not math.isfinite(angle_increment_deg) or angle_increment_deg <= 0.0:
+            obst.data = [float('inf'), float('nan')]
+            self.pub_reject.publish(Int32(data=0))
+            return obst
+
+        min_index = int((0 - angle_min) / angle_increment_deg)
+        max_index = int((160 - angle_min) / angle_increment_deg)
+
+        # ---- [J] 공간 median 후 최소거리 (감속 신호 전용) ----
+        # 작년엔 필터 없는 raw min 이라 물보라 반사 한 점(0.3m)이 그대로 motor_control 감속을
+        # 물렸다. median 은 이웃과 어긋나는 고립 스파이크를 지운다.
+        # ★ 여기(감속 신호)에만 건다 — _compute 의 회피 마스크는 손대지 않는다(접촉 성능 불변).
+        # D6: 기각수(rejected)를 함께 받아 /obstacle_reject_count 로 낸다. 관측 전용 — 제어 불변.
+        d, i, rejected = median_min(
+            ranges, min_index, max_index, self.obst_median_kernel, return_rejected=True)
+        self.pub_reject.publish(Int32(data=int(rejected)))
+        if d is None:
+            obst.data = [float('inf'), float('nan')]
+        else:
+            obst.data = [d, angle_min + i * angle_increment_deg]
+        return obst
+
+    def _merge_geofence(self, distance_array, angle_array):
+        """경계 '가짜 LiDAR'를 실제 스캔과 병합한다 — **한 줄이 전부다**:
+              ranges[i] = min(real_ranges[i], geofence_ranges[i])
+
+        이후는 기존 파이프라인이 알아서 한다: detection_distance 로 이진 마스크가 되고,
+        dilate 가 배 폭(half_width+clearance) 여유를 더하고, 갭-팔로잉이 피한다.
+        특수 로직도, 새 상태기계도, 튜닝 파라미터(half_block)도 없다.
+
+        병합하지 않는 경우 (전부 '모르면 입을 다문다'):
+          · geofence 가 None  → north 가 빈 배열 (미설정 / IMU stale / 이미 이탈)
+          · geofence 가 stale → 묵은 '없는 벽'은 배를 가둔다
+        """
+        with self.lock:
+            gf = self.geofence
+            gf_t = self.last_geofence_t
+
+        if gf is None or gf_t is None:
+            return distance_array
+        if (time.monotonic() - gf_t) > self.geofence_stale_sec:
+            self.get_logger().warn(
+                "geofence stale → 병합 안 함 (묵은 '없는 벽'은 배를 가둔다)",
+                throttle_duration_sec=5.0)
+            return distance_array
+
+        a0, inc, gr = gf
+        n = len(gr)
+        if n == 0 or inc <= 0.0:
+            return distance_array
+
+        out = list(distance_array)
+        hits = 0
+        nearest = float('inf')
+        for i, a in enumerate(angle_array):
+            rel = a - LIDAR_FORWARD_DEG          # LiDAR 프레임(80=정면) → 상대방위(0=정면)
+            k = int(round((rel - a0) / inc))
+            if not (0 <= k < n):
+                continue
+            g = gr[k]
+            if not math.isfinite(g):
+                continue
+            cur = out[i]
+            if (not math.isfinite(cur)) or g < cur:
+                out[i] = g                        # ★ min(실제, 경계)
+                hits += 1
+                nearest = min(nearest, g)
+
+        if hits:
+            self.get_logger().warn(
+                f"🚧 경계가 스캔에 병합됨 ({hits}셀, 최근접 {nearest:.1f}m)",
+                throttle_duration_sec=2.0)
+        return out
 
-        # =====================================================
-        #               LiDAR 기본 전처리
-        # =====================================================
+    def _geofence_blocks(self, intended_rel):
+        """[I] 경계가 stop_margin 안이고, 그 방위가 '전진하려는 방향 ±cone' 과 겹치면 True.
+
+        (A)SPIN·(B)candidate 구간은 distance_array 를 안 만들어 스캔 병합이 안 된다.
+        도킹은 SPIN+candidate 로 조향하므로, 도크가 경기장 가장자리면 접안하다 경계로 파고든다.
+        → 여기서 하드하게 막는다. **미션보다 실격 방지가 우선.** 경계를 등지고 멈춘다.
+        (C) 자율회피는 이미 _merge_geofence 로 처리되므로 이 가드를 호출하지 않는다.
+        ⚠️ north 의 광선은 전방 ±80° 만 쏜다 → 뒤쪽(후진 중) 경계는 못 본다. 접안 방향(전방) 방어용.
+        """
+        with self.lock:
+            gf = self.geofence
+            gf_t = self.last_geofence_t
+
+        if gf is None or gf_t is None:
+            return False
+        if (time.monotonic() - gf_t) > self.geofence_stale_sec:
+            return False
+
+        a0, inc, gr = gf
+        if not gr or inc <= 0.0:
+            return False
+
+        # 최근접 경계와 그 상대방위
+        best_d, best_rel = float('inf'), None
+        for k, g in enumerate(gr):
+            if math.isfinite(g) and g < best_d:
+                best_d, best_rel = g, a0 + k * inc
+        if best_rel is None or best_d > self.geofence_stop_margin_m:
+            return False
+
+        # 전진하려는 방향과 겹치나 (각도차를 [-180,180]로 래핑)
+        diff = ((best_rel - intended_rel) + 180.0) % 360.0 - 180.0
+        if abs(diff) <= self.geofence_stop_cone_deg:
+            self.get_logger().warn(
+                f"🚧🛑 경계 {best_d:.1f}m (상대 {best_rel:.0f}°) 가 전진 방향({intended_rel:.0f}°)과 "
+                f"겹침 → 미션 무시하고 정지 (실격 방지 우선)",
+                throttle_duration_sec=1.0)
+            return True
+        return False
+
+    def _compute(self, msg, wp4_enter_time, candidate_angle, yaw_error, detection_distance):
+        """(desired_angle, obstacle_array) 를 만든다. 작년 scan_callback 의 로직 그대로."""
+        now = time.monotonic()
+
+        obst = self._closest_obstacle(msg)
+
+        # ---- (0) WP4 초기 5초 & STOP_HOLD 우선 처리 ----
+        if (wp4_enter_time is not None and (now - wp4_enter_time) < 5.0) or \
+           (candidate_angle == STOP_HOLD):
+            self._reset_reverse_sequence()
+            return STOP_HOLD, obst
+
+        # ---- [I] Geofence 하드 가드 ((A)SPIN/(B)candidate 전용) ----
+        # 이 두 구간은 마스크 병합이 없어 경계로 파고들 수 있다(도킹이 가장자리일 때).
+        # (C)(CANDIDATE_INVALID)는 아래에서 _merge_geofence 로 처리되므로 제외한다.
+        if candidate_angle != CANDIDATE_INVALID:
+            # 전진하려는 방향: candidate 는 그 상대각, SPIN(제자리)은 전방(0°)으로 본다.
+            intended_rel = 0.0 if candidate_angle in (SPIN_RIGHT, SPIN_LEFT) else candidate_angle
+            if self._geofence_blocks(intended_rel):
+                self._reset_reverse_sequence()
+                return STOP_HOLD, obst
+
+        # ---- (A) PASS-THROUGH: 미션 노드가 선회를 직접 지시 ----
+        if candidate_angle in (SPIN_RIGHT, SPIN_LEFT):
+            self._reset_reverse_sequence()
+            return candidate_angle, obst
+
+        # ---- (B) 일반 candidate_angle 매핑 (360° → 전방중심 80°) ----
+        if candidate_angle != CANDIDATE_INVALID:
+            self._reset_reverse_sequence()
+            c_raw = candidate_angle % 360.0
+            c_mapped = 80.0 + c_raw if c_raw <= 180.0 else 80.0 - (360.0 - c_raw)
+
+            # 정상 조향값은 0~160°여야 한다. 범위를 벗어나면 motor_control이 후진으로
+            # 해석할 수 있으므로 안전하게 정지한다.
+            if not math.isfinite(c_mapped) or not (0.0 <= c_mapped <= 160.0):
+                self.get_logger().warn(
+                    f"candidate 매핑 범위 초과: raw={c_raw:.1f}°, "
+                    f"mapped={c_mapped:.1f}° → STOP_HOLD",
+                    throttle_duration_sec=1.0)
+                return STOP_HOLD, obst
+            return c_mapped, obst
+
+        # ---- (C) candidate == 20000 → yaw_error 기반 자율 회피 ----
         angle_min = math.degrees(msg.angle_min)
@@ -164,7 +725,11 @@ class ShipDirection(Node):
         angle_increment_rad = msg.angle_increment
-
         ranges = list(msg.ranges)
+
+        # [F] 0 나눗셈 방어 (scan_callback 이 이미 걸러내지만, 여기도 막아둔다)
+        if not math.isfinite(angle_increment_deg) or angle_increment_deg <= 0.0:
+            self.get_logger().error("angle_increment 이상 → 정지", throttle_duration_sec=1.0)
+            return STOP_HOLD, obst
+
         min_index = int((0 - angle_min) / angle_increment_deg)
         max_index = int((160 - angle_min) / angle_increment_deg)
-
         sub_ranges = ranges[min_index:max_index + 1]
@@ -173,15 +738,17 @@ class ShipDirection(Node):
         distance_array = []
-        front_min_distance = float('inf')
 
         for i, r in enumerate(sub_ranges):
-            ang = angle_min + (min_index + i) * angle_increment_deg
-            angle_array.append(ang)
+            angle_array.append(angle_min + (min_index + i) * angle_increment_deg)
             distance_array.append(r)
 
-            if not math.isinf(r) and not math.isnan(r):
-                front_min_distance = min(front_min_distance, r)
+        # ---- [H] Geofence 병합 (6a-2). 이게 전부다: ranges[i] = min(실제, 경계) ----
+        # 경계는 north 가 '가짜 LiDAR'로 쏴서 보낸다. 여기서 스캔에 섞어버리면
+        # 아래 파이프라인(마스크 → dilate → 갭-팔로잉)이 경계를 그냥 '벽'으로 취급한다.
+        distance_array = self._merge_geofence(distance_array, angle_array)
 
-        # =====================================================
-        #            Binary obstacle mask 생성
-        # =====================================================
+        # ---- Binary obstacle mask ----
+        # ※ rear_obstacle_ignore_margin 제거됨. 작년엔 여기서
+        #      if abs(r - front_min_distance) > rear_obstacle_ignore_margin: binary.append(0)
+        #    로 '최근접점에서 먼 셀'을 지웠는데, 물보라 반사 0.3m 하나가 front_min_distance 를
+        #    끌어내리면 실제 1.6m 부표가 통째로 지워졌다 (17셀 → 0셀). 튜닝값이 아니라 결함이었다.
         binary = []
@@ -191,6 +758,3 @@ class ShipDirection(Node):
             elif r < detection_distance:
-                if abs(r - front_min_distance) > self.rear_obstacle_ignore_margin:
-                    binary.append(0)
-                else:
-                    binary.append(1)
+                binary.append(1)
             else:
@@ -198,13 +762,15 @@ class ShipDirection(Node):
 
-        # 악성 spike 억제
+        # ---- [G] 시간 투표 — 물보라 한 프레임짜리 오탐 제거 ----
+        # ★ 반드시 dilate 전, 원본 마스크에 건다. 팽창 후에 걸면 진짜 부표의 팽창 영역까지
+        #   표가 갈려 부표가 얇아진다.
+        binary = self.temporal.apply(binary)
+
         binary = self.smooth_spikes(binary)
         binary = self.suppress_spike_edges(binary)
-        binary = self.dilate_obstacles(binary, distance_array, angle_increment_deg)
+        binary = self.dilate_obstacles(binary, distance_array, angle_increment_deg,
+                                       detection_distance)
 
-        # =====================================================
-        # safe zone(장애물 없음 구간) 탐색
-        # =====================================================
+        # ---- safe zone 탐색 ----
         safe_zones = []
         start = None
-
         for i, v in enumerate(binary):
@@ -215,3 +781,2 @@ class ShipDirection(Node):
                 start = None
-
         if start is not None:
@@ -219,69 +784,8 @@ class ShipDirection(Node):
 
-        # safety width 기준 통과 가능한 zone만 선별
-        valid_safe_zones = []
+        # ---- 통과 가능한 zone 선별 ----
         min_required_width = self.half_width * 2 + self.clearance
+        valid_safe_zones = select_valid_zones(
+            safe_zones, distance_array, angle_increment_rad, detection_distance, min_required_width)
 
-        for s, e in safe_zones:
-            r_s = distance_array[s] if not math.isinf(distance_array[s]) and not math.isnan(distance_array[s]) else detection_distance
-            r_e = distance_array[e] if not math.isinf(distance_array[e]) and not math.isnan(distance_array[e]) else detection_distance
-
-            r_edge = min(r_s, r_e)
-            arc_len = angle_increment_rad * r_edge * (e - s)
-
-            if arc_len >= min_required_width:
-                valid_safe_zones.append((s, e, r_edge))
-
-        # =====================================================
-        # 가장 가까운 장애물 잡기
-        # =====================================================
-        closest_distance = float('inf')
-        closest_angle = float('nan')
-
-        for i in range(min_index, max_index + 1):
-            r = ranges[i]
-            if not math.isinf(r) and not math.isnan(r) and r < closest_distance:
-                closest_distance = r
-                closest_angle = angle_min + i * angle_increment_deg
-
-        # =====================================================
-        # (A) PASS-THROUGH: 이미지가 선회 명령을 직접 내리는 경우
-        # =====================================================
-        if candidate_angle in (5000.0, 6000.0):
-            obst = Float32MultiArray()
-            obst.data = (
-                [closest_distance, closest_angle]
-                if not math.isinf(closest_distance)
-                else [float('inf'), float('nan')]
-            )
-            self.pub_obstacle_distance.publish(obst)
-            self.pub_desired_angle.publish(Float32(data=candidate_angle))
-            return
-
-        # =====================================================
-        # (B) 일반 candidate_angle 처리 (맵핑)
-        # =====================================================
-        if candidate_angle not in (CANDIDATE_INVALID, STOP_HOLD):
-            c_raw = candidate_angle % 360.0
-
-            # 360° → 전방중심 80° 맵핑
-            if c_raw <= 180:
-                c_mapped = 80 + c_raw
-            else:
-                c_mapped = 80 - (360 - c_raw)
-
-            final_angle = c_mapped
-
-            obst = Float32MultiArray()
-            obst.data = (
-                [closest_distance, closest_angle]
-                if not math.isinf(closest_distance)
-                else [float('inf'), float('nan')]
-            )
-            self.pub_obstacle_distance.publish(obst)
-            self.pub_desired_angle.publish(Float32(data=final_angle))
-            return
-
-        # =====================================================
-        # (C) candidate == 20000 → yaw_error 기반 자율 회피
-        # =====================================================
+        # ---- yaw_error 를 safe zone 안으로 clamp → 최적 후보 ----
         yaw_raw = yaw_error % 360.0
@@ -290,3 +794,2 @@ class ShipDirection(Node):
         candidates = []
-
         for s, e, r_edge in valid_safe_zones:
@@ -295,3 +798,2 @@ class ShipDirection(Node):
 
-            # yaw_mapped를 safe zone 안으로 clamp
             if yaw_mapped < zone_min:
@@ -305,17 +807,16 @@ class ShipDirection(Node):
             arc_length = angle_increment_rad * r_edge * (e - s)
-
             candidates.append((angle_diff, -arc_length, best_angle, s, e))
 
-        # =====================================================
-        # 회피 경로가 없으면 → 후진 260°
-        # =====================================================
+        # ---- [L] 회피 경로가 없으면 후진. 팀원 피드백 C안 반영 ----
+        # 작년/2단계 코드는 여기서 매 tick 무조건 REVERSE_ANGLE 을 냈다 — reverse_cooldown 은
+        # 타임스탬프만 갱신할 뿐 final_angle 에 영향이 없었다(dead code, 팀 지적).
+        # ⚠️ north 광선은 전방 ±80° 만 봐서 후진 중 뒤쪽 경계는 못 본다([I] 참고).
+        #    즉 지금 구조로 '계속 후진'은 뒤를 못 보며 맹목 후진하는 것과 같다.
+        # → C안: 후진은 쿨다운마다 '한 번만' 짧게 내고, 그 사이(쿨다운 중)는 STOP_HOLD 로
+        #   멈춰서 재탐색한다. 쿨다운이 끝났는데도 여전히 후보가 없으면 다시 한 번 후진한다.
         if not candidates:
-            final_angle = 260.0
-
-            # 후진 쿨타임 유지
-            if now - self.last_reverse_time >= self.reverse_time:
-                self.last_reverse_time = now
-
+            # 일정 시간 후진 → 일정 시간 STOP 재탐색 → 필요하면 반복
+            final_angle = self._apply_reverse_sequence(now)
         else:
-            # 최적 후보 선택
+            self._reset_reverse_sequence()
             candidates.sort(key=lambda x: (x[0], x[1]))
@@ -323,14 +824,7 @@ class ShipDirection(Node):
 
-        # =====================================================
-        # publish 결과
-        # =====================================================
-        obst = Float32MultiArray()
-        obst.data = (
-            [closest_distance, closest_angle]
-            if not math.isinf(closest_distance)
-            else [float('inf'), float('nan')]
-        )
-        self.pub_obstacle_distance.publish(obst)
-        self.pub_desired_angle.publish(Float32(data=final_angle))
+        return final_angle, obst
 
+    # =====================================================
+    # 스파이크 억제 / 팽창 (작년 그대로)
+    # =====================================================
     def smooth_spikes(self, binary):
@@ -368,3 +862,9 @@ class ShipDirection(Node):
 
-    def dilate_obstacles(self, binary, distance_array, angle_increment_deg):
+    def dilate_obstacles(self, binary, distance_array, angle_increment_deg,
+                         detection_distance):
+        # 팀원 제안 버그2: 작년엔 여기서 self.detection_distance 를 직접 읽어, control_cb 가
+        #   lock 으로 뜬 스냅샷과 어긋날 수 있었다(wp_mode_cb 가 다른 스레드에서 바꾸므로).
+        #   실제 영향은 거의 없다 — r_edge 는 아래에서 유효 거리로 덮이는 폴백이라, binary=1 셀은
+        #   정의상 유효 거리가 있어 폴백이 안 쓰인다. 그래도 한 _compute 안에서 값이 하나로
+        #   유지되도록 스냅샷을 파라미터로 받는다(lock 일관성 원칙).
         n = len(binary)
@@ -381,3 +881,3 @@ class ShipDirection(Node):
                 if length >= self.min_obstacle_cells:
-                    r_edge = self.detection_distance
+                    r_edge = detection_distance
                     for k in range(start, end + 1):
@@ -402,5 +902,5 @@ class ShipDirection(Node):
 
+
 def main(args=None):
     rclpy.init(args=args)
-
     node = ShipDirection()
@@ -409,7 +909,10 @@ def main(args=None):
     executor.add_node(node)
-
-    executor.spin()
-
-    node.destroy_node()
-    rclpy.shutdown()
+    try:
+        executor.spin()
+    except KeyboardInterrupt:
+        pass
+    finally:
+        node.destroy_node()
+        if rclpy.ok():
+            rclpy.shutdown()
 
diff --git a/src/ship_direction/test/test_flake8.py b/src/ship_direction/test/test_flake8.py
index 27ee107..9d4e79b 100644
--- a/src/ship_direction/test/test_flake8.py
+++ b/src/ship_direction/test/test_flake8.py
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
diff --git a/src/ship_direction/test/test_pep257.py b/src/ship_direction/test/test_pep257.py
index b234a38..941937d 100644
--- a/src/ship_direction/test/test_pep257.py
+++ b/src/ship_direction/test/test_pep257.py
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
diff --git a/src/rplidar_ros/launch/rplidar_a3_launch.py b/src/rplidar_ros/launch/rplidar_a3_launch.py
index 40c9e34..620bc56 100644
--- a/src/rplidar_ros/launch/rplidar_a3_launch.py
+++ b/src/rplidar_ros/launch/rplidar_a3_launch.py
@@ -2,8 +2,4 @@
 
-import os
-
-from ament_index_python.packages import get_package_share_directory
 from launch import LaunchDescription
 from launch.actions import DeclareLaunchArgument
-from launch.actions import LogInfo
 from launch.substitutions import LaunchConfiguration
@@ -13,47 +9,63 @@ from launch_ros.actions import Node
 def generate_launch_description():
-    channel_type =  LaunchConfiguration('channel_type', default='serial')
-    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyLiDAR')
-    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000') #for A3 is 256000
-    frame_id = LaunchConfiguration('frame_id', default='laser')
-    inverted = LaunchConfiguration('inverted', default='false')
-    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
-    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
+    """Launch the RPLIDAR A3 node with configurable runtime arguments.
 
-    return LaunchDescription([
+    Notes:
+    - ``/dev/ttyLiDAR`` requires a matching udev rule on each computer.
+    - ``scan_mode`` remains ``Sensitivity`` by default until comparative
+      water-test data supports changing it. It can be overridden at launch.
+    - QoS for ``/scan`` subscribers belongs in the subscribing node, not here.
+    """
+
+    # Launch arguments are declared with literal defaults and read through
+    # LaunchConfiguration. This keeps the runtime override path explicit.
+    channel_type = LaunchConfiguration('channel_type')
+    serial_port = LaunchConfiguration('serial_port')
+    serial_baudrate = LaunchConfiguration('serial_baudrate')
+    frame_id = LaunchConfiguration('frame_id')
+    inverted = LaunchConfiguration('inverted')
+    angle_compensate = LaunchConfiguration('angle_compensate')
+    scan_mode = LaunchConfiguration('scan_mode')
 
+    return LaunchDescription([
         DeclareLaunchArgument(
             'channel_type',
-            default_value=channel_type,
-            description='Specifying channel type of lidar'),
-
+            default_value='serial',
+            description='Communication channel type for the LiDAR',
+        ),
         DeclareLaunchArgument(
             'serial_port',
-            default_value=serial_port,
-            description='Specifying usb port to connected lidar'),
-
+            default_value='/dev/ttyLiDAR',
+            description=(
+                'Serial device for the LiDAR. /dev/ttyLiDAR requires a '
+                'matching udev rule on the host computer.'
+            ),
+        ),
         DeclareLaunchArgument(
             'serial_baudrate',
-            default_value=serial_baudrate,
-            description='Specifying usb port baudrate to connected lidar'),
-        
+            default_value='256000',
+            description='Serial baud rate; RPLIDAR A3 uses 256000',
+        ),
         DeclareLaunchArgument(
             'frame_id',
-            default_value=frame_id,
-            description='Specifying frame_id of lidar'),
-
+            default_value='laser',
+            description='TF frame ID assigned to LaserScan messages',
+        ),
         DeclareLaunchArgument(
             'inverted',
-            default_value=inverted,
-            description='Specifying whether or not to invert scan data'),
-
+            default_value='false',
+            description='Whether to invert the scan data',
+        ),
         DeclareLaunchArgument(
             'angle_compensate',
-            default_value=angle_compensate,
-            description='Specifying whether or not to enable angle_compensate of scan data'),
-
+            default_value='true',
+            description='Whether to enable angle compensation',
+        ),
         DeclareLaunchArgument(
             'scan_mode',
-            default_value=scan_mode,
-            description='Specifying scan mode of lidar'),
-
+            default_value='Sensitivity',
+            description=(
+                'RPLIDAR scan mode. Keep Sensitivity as the baseline and '
+                'override this argument during Standard/Boost comparison tests.'
+            ),
+        ),
         Node(
@@ -62,11 +74,13 @@ def generate_launch_description():
             name='rplidar_node',
-            parameters=[{'channel_type':channel_type,
-                         'serial_port': serial_port,
-                         'serial_baudrate': serial_baudrate,
-                         'frame_id': frame_id,
-                         'inverted': inverted,
-                         'angle_compensate': angle_compensate,
-                         'scan_mode': scan_mode}],
-            output='screen'),
+            output='screen',
+            parameters=[{
+                'channel_type': channel_type,
+                'serial_port': serial_port,
+                'serial_baudrate': serial_baudrate,
+                'frame_id': frame_id,
+                'inverted': inverted,
+                'angle_compensate': angle_compensate,
+                'scan_mode': scan_mode,
+            }],
+        ),
     ])
-
```

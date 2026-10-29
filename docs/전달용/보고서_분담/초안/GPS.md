# GPS 초안
## 1.3) 센서 — 담당 줄
| 장비명 | 용도 |
|---|---|
| u-blox C94-M8P (A배) | NTRIP RTK 위치(`/ublox_gps_node/fix`, 1 Hz)로 다음 웨이포인트 방위·임무 번호(`/wp_mode`)·지오펜스 광선을 만든다. 속도(`/ublox_gps_node/fix_velocity`)는 헤딩 보정에 쓴다(Ⅲ-2.1) [근거: src/ublox/ublox_gps/config/c94_m8p_rover.yaml:17] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:179-187] [근거: src/ssf_heading/config/ssf_heading.yaml:84] |
| u-blox ZED-F9P + ANN-MB (B배) | A배와 같다. 노드·파라미터는 같고 수신기 설정 파일만 다르다 [근거: src/launch_files/launch/launch_files.launch.py:35-37] [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:30] |

## 2.2) GPS RTK
GPS 처리는 세 패키지로 이뤄진다. 수신기 드라이버 `ublox_gps`, RTK 보정을 받아 오는 `ntrip_client`, 위치를 항법 정보로 바꾸는 `north_goal_angle` 이다 [근거: src/launch_files/launch/launch_files.launch.py:84-111]. `north_goal_angle` 은 RTK 위치로 다음 웨이포인트로 가는 방위 `/north_goal_angle_tp`, 현재 임무 번호 `/wp_mode`, 경기장 경계를 가짜 LiDAR 광선으로 바꾼 `/geofence_state` 를 0.5 s 주기로 낸다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:180-184] [근거: src/north_goal_angle/config/north_goal_angle.yaml:4].

설계 원칙은 "모르면 입을 다문다"이다. fix 가 유효하지 않으면 방위를 내지 않고, IMU 헤딩이 묵었거나 이미 경계 밖이면 지오펜스는 빈 배열을 낸다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:447-488] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:357-360]. 올해는 fix 없이 항법하던 결함을 막는 검증, 이상치 필터, 경로선 추종(LOS·ILOS), 광선 지오펜스, 파일 기반 웨이포인트를 새로 넣었다 [근거: 커밋 02944d8] [근거: 커밋 9221750] [근거: 커밋 b678154] [근거: 커밋 2758653].

### 2.2.1) 장비 사양과 선정 이유
| 항목 | A배 | B배 | 출처 |
|---|---|---|---|
| 수신기 | u-blox C94-M8P | u-blox ZED-F9P | [근거: CLAUDE.md:60] |
| 안테나 | 키트 동봉 L1 패치 + 접지판 ≥10 cm (지시서·원고는 ANN-MB — 부록 B) | ANN-MB, 접지판 Ø100~150 mm | [근거: docs/전달용/하드웨어_배치_요구사항.md:84-85] [근거: docs/전달용/하드웨어_배치_요구사항.md:65] |
| 확인된 펌웨어 | ____ | FWVER HPG 1.13 / PROTVER 27.12 | [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:12] |
| 사용 위성계(설정) | GPS·GLONASS·QZSS (BeiDou 끔) | GPS·GLONASS·BeiDou·QZSS | [근거: src/ublox/ublox_gps/config/c94_m8p_rover.yaml:40-43] [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:50-53] |
| 측위 발행 주기(설정) | 1 Hz | 1 Hz | [근거: src/ublox/ublox_gps/config/c94_m8p_rover.yaml:17] [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:30] |
| 동역학 모델 | `sea` | `sea` | [근거: src/ublox/ublox_gps/config/c94_m8p_rover.yaml:21] [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:35] |
| 연결 | USB `/dev/ttyGPS` (udev PID 01a8) | USB `/dev/ttyGPS` (udev PID 01a9) | [근거: src/ublox/ublox_gps/config/c94_m8p_rover.yaml:6] [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:21-22] |
| 보정 | NTRIP (UHF 기지국 무선 안 씀) | NTRIP | [근거: docs/전달용/하드웨어_배치_요구사항.md:85] |
| 채널 수 · RTK 정밀도 · 최대 갱신율 (데이터시트) | ____ | ____ | 저장소에 데이터시트 없음 — 부록 B |

두 수신기는 이미 보유한 장비이며 올해 GPS 관련 구매는 없다 [근거: docs/전달용/구매_목록.md:212] [근거: docs/전달용/구매_목록.md:283]. RTK 를 쓰는 이유는 웨이포인트 간격 때문이다. 단독 측위 오차(±3~5 m)가 부표 간격보다 커서 단독 GPS 로 찍은 웨이포인트는 쓸 수 없다 [근거: docs/절차/웨이포인트_녹화절차.md:31]. 보정은 별도 기지국 무선 대신 인터넷 NTRIP 으로 받아 선체에 UHF 안테나를 달지 않는다 [근거: docs/전달용/하드웨어_배치_요구사항.md:85].

### 2.2.2) 코드 출처
| 패키지 | 원래 출처 | 작년부터 | 올해 바꾼 것 | 근거 커밋 |
|---|---|---|---|---|
| `ublox_gps` | **외부.** u-blox ROS 드라이버 2.3.0 (package.xml: author Johannes Meyer, maintainer Gareth Cross 외, BSD) [근거: src/ublox/ublox_gps/package.xml:4-12] | 있음 | B배용 `zed_f9p_rover.yaml` 신설, launch 에 배별 `config` 인자 / F9P `sea` 모델 / A배 yaml 의 EEPROM `save` 제거·`sea`·`debug 0`, 메인 launch 의 `gps_config` 전달 | c1b0d81 · ed2a144 · c7c0c59 |
| `ntrip_client` | **외부.** LORD-MicroStrain `ntrip_client` 1.3.0 (Parker Hannifin, MIT) [근거: src/ntrip_client/package.xml:4-10] | 있음 | launch 설치 경로 수정 / 계정을 저장소 밖 `~/.ssf/` 로 이전 | 89818a5 · f98a8f9 |
| `north_goal_angle` | 작년 우리 코드 (웨이포인트 FSM) | 있음 | fix 검증 `gps_guard`, LOS·ILOS `los_logic`·필터 `gps_filter`(팀원 제출 코드에서 기능만 추출해 순수 모듈로 재구현), 광선 지오펜스, `waypoints.yaml`·`waypoint_loader`·`waypoint_recorder`, 자율 시작 게이트 | 02944d8 · 9221750 · 2cb050b · b678154 · 2758653 · 7a2e803 · df16eee |

LOS·필터는 팀원이 제출한 GPS 코드에서 LOS, 동적 전방주시, 공분산·점프 필터, 위치 스무딩만 골라 이식했고, IMU 가속도 적분(DR)·쿼터니언 처리는 우리 드라이버가 `/imu/data` 를 내지 않아 넣지 않았다 [근거: docs/전달용/회신_GPS코드_기능추출_수정제안.md:26-37] [근거: docs/전달용/회신_GPS코드_기능추출_수정제안.md:41-50].

### 2.2.3) 데이터 흐름
| 토픽 | 입력·출력 | 메시지 타입 | 주기 | QoS | 상대 노드 |
|---|---|---|---|---|---|
| `/rtcm` | `ntrip_client` → `ublox_gps_node` | `rtcm_msgs` 패키지 메시지 | ____ (휴대 시험 5.4 Hz, 2.2.8 참고) | 드라이버 기본 | 수신기에 USB 로 주입 [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:56-58] [근거: src/ntrip_client/launch/ntrip_client_launch.py:125] |
| `/ublox_gps_node/fix` | 입력 | `sensor_msgs/NavSatFix` | 1 Hz (설정) | RELIABLE, depth 2 (구독) | `ublox_gps_node` [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:107] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:187] |
| `/imu/yaw` | 입력 | `std_msgs/Float64` | IMU 측 | RELIABLE, depth 2 | `yaw_mux` — 지오펜스 광선 방향 계산용 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:188] |
| `/boat_mode` | 입력 | `std_msgs/Int32` | 브릿지 측 | RELIABLE, depth 2 | `ssf_bridge` — 자율 시작 게이트 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:190] |
| `/north_goal_angle_tp` | 출력 | `std_msgs/Float32` (0=북, 시계방향 °) | 2 Hz, fix 유효 시만 | RELIABLE, depth 2 | `ship_goal_angle` [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:482] [근거: src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py:46] |
| `/wp_mode` | 출력 | `std_msgs/Int32` | 2 Hz, fix 무관 | RELIABLE, depth 2 | 임무 노드·검출기·`ship_direction` [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:490] [근거: src/ship_direction/ship_direction/ship_direction.py:265] |
| `/candidate_angle` | 출력 | `std_msgs/Float32` | 2 Hz, mode 5·8 에서만 `20000.0` | RELIABLE, depth 2 | `ship_direction` [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:495-496] [근거: src/ship_direction/ship_direction/ship_direction.py:264] |
| `/geofence_state` | 출력 | `std_msgs/Float32MultiArray` | 2 Hz | RELIABLE, depth 2 | `ship_direction` [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:411] [근거: src/ship_direction/ship_direction/ship_direction.py:266] |
| `/goal_distance`, `/wp_remaining_time` | 출력 | `std_msgs/Float32` | 2 Hz | RELIABLE, depth 2 | `mission_monitor` (관측 전용) [근거: src/ssf_tools/ssf_tools/mission_monitor.py:102-106] |

그림: Ⅲ-2.2-1 GPS 처리 흐름도 — `ntrip_client` → `/rtcm` → 수신기(`ublox_gps_node`) → `/ublox_gps_node/fix` → `gps_guard`(fix 검증) → `gps_filter`(공분산·점프·스무딩) → `los_logic`(LOS·ILOS) → `/north_goal_angle_tp` → `ship_goal_angle`. 같은 노드에서 웨이포인트 FSM → `/wp_mode`·`/candidate_angle`, 지오펜스 → `/geofence_state` → `ship_direction` 으로 갈라지는 가지를 함께 그린다. `/imu/yaw`·`/boat_mode` 입력 화살표 포함.

### 2.2.4) 처리 알고리즘
#### ① 수신기와 RTK 보정
`ntrip_client` 는 계정 파일을 `credentials_file:=` 인자, 환경변수 `SSF_NTRIP_CREDENTIALS`, 기본 경로 `~/.ssf/ntrip_credentials.yaml` 순으로 찾는다 [근거: src/ntrip_client/launch/ntrip_client_launch.py:34-42]. 저장소에는 값이 빈 양식 `ntrip_credentials.example.yaml` 만 둔다 [근거: src/ntrip_client/config/ntrip_credentials.example.yaml:1-2]. 파일이 없거나 host·mountpoint 가 비면 예외를 던지지 않고 노드만 띄우지 않는다. 예외가 나면 메인 launch 전체가 내려가기 때문이다 [근거: src/ntrip_client/launch/ntrip_client_launch.py:13-16] [근거: src/ntrip_client/launch/ntrip_client_launch.py:63-76]. 메인 launch 는 수신기 드라이버를 띄운 뒤 2.0 s 지연 후 `ntrip_client` 를 띄운다 [근거: src/launch_files/launch/launch_files.launch.py:99-101]. 받은 RTCM 은 `/rtcm` 토픽으로 드라이버에 넘어가 수신기와 연결된 USB 포트로 주입된다 [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:56-58]. 세션이 끊기면 5 s 간격으로 최대 10회 재연결한다 [근거: src/ntrip_client/launch/ntrip_client_launch.py:101-105].

드라이버 설정에서 우리가 바꾼 것은 네 가지다. (1) 두 배 모두 `dynamic_model` 을 해상 모델 `sea` 로 했다 [근거: src/ublox/ublox_gps/config/c94_m8p_rover.yaml:21] [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:35]. (2) EEPROM 영구 저장(`save` 블록)을 빼고 RAM 에만 적용한다. 잘못된 설정이 수신기에 남으면 되돌리기 어렵기 때문이다 [근거: src/ublox/ublox_gps/config/c94_m8p_rover.yaml:13] [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:25-27]. (3) B배 F9P 용 이동국 설정을 새로 만들었다. 기존 `zed_f9p.yaml` 은 `tmode3: 1`(기지국)이라 배에 쓸 수 없어 `tmode3: 0`, BeiDou 켬, 부팅 때마다 NACK 을 내던 UBX-AID 발행 끔으로 정했다 [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:5-6] [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:45-53] [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:84-92]. (4) 설정 파일을 배별 인자로 고른다. 원래 launch 는 `c94_m8p_rover.yaml` 고정이라 F9P 를 꽂아도 에러 없이 M8P 설정으로 돌았다. 지금은 메인 launch 의 `gps_config` 가 드라이버 launch 의 `config` 로 전달되고, 기본값은 B배 `zed_f9p_rover.yaml` 이다 [근거: src/ublox/ublox_gps/launch/ublox_gps_node-launch.py:38-41] [근거: src/launch_files/launch/launch_files.launch.py:35-37] [근거: src/launch_files/launch/launch_files.launch.py:93].

#### ② fix 검증과 필터
작년 코드는 `/ublox_gps_node/fix` 의 위경도를 상태 확인 없이 그대로 썼다 [근거: 커밋 9bff661 src/north_goal_angle/north_goal_angle/north_goal_angle.py:54-55]. 드라이버는 fix 가 없어도 status=-1, 위경도 0.0 인 메시지를 1 Hz 로 계속 내므로, 노드가 위도 0°·경도 0°(Null Island)에서 대회장까지의 방위를 발행했다. 2026-08-06 실기에서 fix 없이 `/north_goal_angle_tp` = 47.96° 가 관측됐고, 이는 `calc_angle(0, 0, WP0)` 와 일치했다 [근거: src/north_goal_angle/north_goal_angle/gps_guard.py:3-13]. 값이 그럴듯해 눈으로는 잡기 어렵다 [근거: src/north_goal_angle/north_goal_angle/gps_guard.py:15].

`gps_guard.fix_is_usable` 은 status < 0(NO_FIX), 위경도가 NaN·inf, |위도|·|경도| ≤ 1e-7 인 (0, 0), 위도 ±90°·경도 ±180° 범위 밖을 모두 거부한다 [근거: src/north_goal_angle/north_goal_angle/gps_guard.py:36] [근거: src/north_goal_angle/north_goal_angle/gps_guard.py:55-62]. 거부되면 `have_fix` 를 내리고 방위·거리 발행을 멈춘다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:282-286].

검증을 통과한 fix 는 `GPSFilter` 가 세 단계로 처리한다 [근거: src/north_goal_angle/north_goal_angle/gps_filter.py:3-8].
- 공분산 필터: `position_covariance[0]` 이 `gps_cov_threshold`(2.0)를 넘는 프레임을 버린다. 0 이하는 정보 없음으로 보고 통과시킨다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:290] [근거: src/north_goal_angle/north_goal_angle/gps_filter.py:43-45].
- 점프 필터: 직전 원시 fix 와의 haversine 거리를 시간차로 나눈 속도가 `gps_max_speed_mps`(15.0 m/s)를 넘으면 순간이동(멀티패스)으로 보고 버린다. 통과한 속도는 `estimated_speed_mps` 로 남겨 전방주시 거리와 도착 반경에 쓴다 [근거: src/north_goal_angle/north_goal_angle/gps_filter.py:47-55].
- 적응형 스무딩: 필터 위치와 새 fix 의 거리 e 에 따라 e < 0.5 m 이면 α = 0.15, e < 1.5 m 이면 0.30, 그 이상이면 0.80 으로 놓고 p ← α·p_raw + (1 − α)·p 로 갱신한다. 작은 흔들림은 강하게 누르고 큰 이동은 빨리 따라간다 [근거: src/north_goal_angle/north_goal_angle/gps_filter.py:59-72].

필터가 버리는 것은 한 프레임뿐이며 fix 상실로 보지 않는다. `have_fix` 와 직전 위치는 그대로 둔다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:288-297]. NO_FIX 는 "품질이 나쁜 fix"가 아니라 "fix 가 아닌 것"이라 가드와 필터를 별도 층으로 둔다 [근거: src/north_goal_angle/north_goal_angle/gps_guard.py:20-23].

#### ③ 웨이포인트와 LOS·ILOS 유도
웨이포인트는 코드가 아니라 `config/waypoints.yaml` 에 `{구역, lat, lon, mode, dwell}` 항목의 목록으로 둔다 [근거: src/north_goal_angle/config/waypoints.yaml:9] [근거: src/north_goal_angle/config/waypoints.yaml:19-30]. `waypoint_loader.parse_waypoints` 는 `waypoints` 키·빈 목록·누락 필드, 위도 33~39°·경도 124~132° 범위(빈칸 0.0, 위경도 뒤바뀜 탐지), mode ∈ {0, 1, 2, 3, 5, 7, 8}, dwell ≥ 0 을 검사한다 [근거: src/north_goal_angle/north_goal_angle/waypoint_loader.py:17-21] [근거: src/north_goal_angle/north_goal_angle/waypoint_loader.py:42-91]. 하나라도 틀리면 몇 번째 항목이 왜 틀렸는지 ERROR 로 남기고 예외로 노드를 세운다. 틀린 좌표로 출발하느니 출발 전에 멈춘다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:224-248]. 파일 경로는 launch 인자 `waypoints_file:=` 로 바꿀 수 있다 [근거: src/north_goal_angle/launch/north_goal_angle_launch.py:36-39].

대회장 좌표는 `waypoint_recorder` 로 녹화한다. RC 로 배를 지점에 대고 Enter 를 누르면 최근 2.0 s 안의 status ≥ 0 fix 를 평균해 한 점으로 잡고, 표본 중 최대 공분산이 2.0 을 넘으면 경고한다 [근거: src/north_goal_angle/north_goal_angle/waypoint_recorder.py:86-112] [근거: src/north_goal_angle/north_goal_angle/waypoint_recorder.py:250]. 저장 전에 같은 `parse_waypoints` 로 검증하고, 실전 파일이 아닌 `~/ssf_logs/waypoints_recorded.yaml` 에 새로 쓴다 [근거: src/north_goal_angle/north_goal_angle/waypoint_recorder.py:114-122] [근거: src/north_goal_angle/north_goal_angle/waypoint_recorder.py:246-249]. 녹화 전 RTK FIXED 확인과 측정 장소 기준 마운트포인트 선택(30 km 이내)이 절차서에 있다 [근거: docs/절차/웨이포인트_녹화절차.md:29-49].

방위는 LOS 로 낸다. 작년에는 현 위치에서 목표점으로 가는 직접 방위만 냈다 [근거: 커밋 9bff661 src/north_goal_angle/north_goal_angle/north_goal_angle.py:67-68]. 그러면 조류나 초기 오차로 밀렸을 때 경로로 돌아오지 못하고 비스듬히 접근한다 [근거: src/north_goal_angle/north_goal_angle/los_logic.py:3-6]. LOS 는 이전 웨이포인트 P 에서 목표 W 로 가는 경로선을 기준으로 한다.
- 경로선 이탈 거리: XTE = d(P, 배) · sin(ψ(P→배) − ψ(P→W)). 양수는 진행 방향 오른쪽으로 밀린 것이다 [근거: src/north_goal_angle/north_goal_angle/los_logic.py:45-56].
- 전방주시 거리: L = clamp(v · `los_speed_gain`, `los_min_lookahead_m`, `los_max_lookahead_m`) = clamp(1.2 v, 2.0, 6.0) m. 빠를수록 멀리 봐 완만하게 복귀한다 [근거: src/north_goal_angle/north_goal_angle/los_logic.py:38-42] [근거: src/north_goal_angle/config/north_goal_angle.yaml:7-9].
- 목표 방위: ψ_d = ψ(P→W) − atan2(XTE + k_i·I, max(0.1, L)) [근거: src/north_goal_angle/north_goal_angle/los_logic.py:67-71].
- ILOS 적분: 매 주기 I ← I + XTE·Δt (Δt = 0.5 s), |I| ≤ 50 으로 자른다(anti-windup). k_i = 0.02 이므로 적분항은 최대 1.0 m 의 이탈만큼 방위를 더 꺾어 조류·바람처럼 계속 미는 외란을 상쇄한다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:479-481] [근거: src/north_goal_angle/config/north_goal_angle.yaml:10-11].

첫 구간에는 이전 웨이포인트가 없으므로 fix 를 처음 얻은 위치(또는 AUTO 전환 시점 위치)를 P 로 쓴다. 웨이포인트를 넘길 때마다 P 를 방금 끝낸 웨이포인트로 바꾸고 I 를 0 으로 되돌린다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:465-467] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:548-549]. 도착 반경은 도킹(mode 7)이면 2.0 m, 그 외에는 속도 2 m/s 미만 2.5 m, 5 m/s 미만 3.5 m, 그 이상 4.5 m 이다 [근거: src/north_goal_angle/north_goal_angle/los_logic.py:75-83]. 반경 안에 dwell 초 머무르면 다음 웨이포인트로 넘어간다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:538-554].

`/wp_mode` 를 바꾸는 것은 이 노드 하나다. 현재 웨이포인트의 mode 를 매 주기 발행하고, 위 도착 판정이나 시간 초과로 다음 웨이포인트로 넘어갈 때 값이 바뀐다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:453] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:490]. 시간 초과는 웨이포인트의 mode 로 정한다. 게이트·위치유지·회피(mode 0·1·2·5)는 WP 하나당 120 s, 부표(3)는 90 s, 도킹(7)은 150 s, 토너먼트(8)는 마지막 WP 라 없다 [근거: src/north_goal_angle/config/north_goal_angle.yaml:35-36] [근거: src/north_goal_angle/north_goal_angle/wp_timeout.py:30-31]. 값이 비었거나 0 이하·NaN·길이 불일치면 출발 전에 노드를 세운다 [근거: src/north_goal_angle/north_goal_angle/wp_timeout.py:43-54]. 2026-10-29 이전에는 목록 순번(3~5 → 90 s, ≤6 → 120 s, 7 이후 없음)이라 회피 두 번째 WP 와 도킹에 시간 초과가 없었다 [근거: 커밋 b337ad8]. `/wp_mode` 는 fix 가 없어도 계속 낸다. 멈추면 비전 검출기 3종이 모두 꺼지기 때문이다. 대신 fix 없이는 도착 판정을 하지 않는다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:458-460]. `require_boat_mode_auto` 가 true 이면 RC 모드가 AUTO(`/boat_mode` = 2)일 때만 웨이포인트가 전진한다. 수동으로 시작점까지 몰고 가는 동안 도착·시간 초과로 웨이포인트가 넘어가는 것을 막고, 그동안 타이머·dwell·적분·경로 시작점을 계속 초기화해 AUTO 전환 순간 WP0 부터 시작한다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:498-511] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:87-96].

`/candidate_angle` 에는 담당 임무 노드가 없는 mode 5·8 에서만 `20000.0`(미션 없음 → 전진 폴백)을 낸다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:72] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:77]. 작년에는 mode 7 에도 냈는데, 이는 도킹 노드가 mode 9 로 잘못 잡혀 침묵하던 것을 덮은 흔적이었다. 도킹 노드를 7 로 고치면 같은 토픽에 발행자가 둘이 되어 접안 중 조향이 튀므로 두 수정을 한 커밋으로 묶었다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:4-10] [근거: 커밋 0b854c1]. 전체 임무 상태기계는 Ⅲ-3 에서 다룬다.

#### ④ 지오펜스 광선 생성
경계 방어는 원뿔이 아니라 광선 방식이다. 경계 최근접점 방향으로 ±각도 원뿔을 막으면, 40 × 40 경기장 모서리에서 대각선(45°)을 향한 배는 북벽 최근접점이 상대 −45°, 동벽이 +45° 로 잡혀 정면 0°(모서리 탈출구)가 두 원뿔 사이 틈으로 열린다. 광선으로 쏘면 정면 방향 경계 거리(2.12 m)가 그대로 벽이 되고 튜닝 파라미터도 없다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:18-25] [근거: 커밋 b678154].

계산 순서는 다음과 같다. 경계 폴리곤 `geofence_polygon`(위경도 평탄 배열, 최소 3점)의 꼭짓점을 배 기준 로컬 미터 (동, 북)로 바꾼다. 위도 1° = 110,540 m, 경도 1° = 111,320·cos(위도) m 근사를 쓴다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:100-101] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:345-348]. 배가 폴리곤 안인지 레이 캐스팅으로 확인한다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:587-599]. 상대방위 −80°~+80° 를 1° 간격(161개)으로, 각 광선의 절대 방위 θ = (`/imu/yaw` + 상대방위) mod 360 방향 단위벡터 (sin θ, cos θ) 를 만들고 모든 변과의 교점 중 가장 가까운 거리를 구한다. 30 m 보다 멀거나 교점이 없으면 inf 로 둔다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:391-401] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:558-584]. 각도 격자는 스캔과 같은 상대방위(0 = 정면)다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:139]. 발행 형식은 `[angle_min_deg, angle_inc_deg, r0, r1, …]` = `[-80, 1, r0 … r160]` 이다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:14-16] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:409-410].

다음 경우에는 빈 배열을 낸다. 폴리곤이 3점 미만이거나 fix 가 없을 때, `/imu/yaw` 를 받은 적이 없거나 `imu_stale_sec`(0.5 s)보다 묵었을 때, 배가 이미 경계 밖일 때다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:350-401]. 광선 방향이 헤딩에 통째로 의존하므로 헤딩이 얼면 엉뚱한 곳에 "없는 벽"을 세워 배를 가둘 수 있고, 이미 밖이면 경계 벽이 복귀로를 막기 때문이다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:152-156] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:382]. 지오펜스는 웨이포인트가 모두 끝난 뒤에도 매 주기 낸다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:411]. 부팅 5.0 s 뒤부터 `/geofence_state` 구독자가 0 이면 ERROR 를 낸다. 경계 방어가 조용히 빠지는 것을 막기 위해서다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:413-420]. 이 광선은 `ship_direction` 이 실제 스캔과 합친다(Ⅲ-2.3). 헤딩 오프셋·COG 는 IMU 담당 절(Ⅲ-2.1)을 참고한다. 현재 `geofence_polygon` 은 비어 있어 지오펜스는 꺼진 상태다 [근거: src/north_goal_angle/config/north_goal_angle.yaml:39-41].

그림: Ⅲ-2.2-2 지오펜스 원뿔 방식과 광선 방식 비교 — 40 × 40 m 경기장 모서리 근처에서 대각선(45°)을 향한 배. 왼쪽은 북벽·동벽 최근접점 방향(−45°, +45°)의 ±40° 원뿔 두 개와 그 사이로 열린 정면 0° 틈, 오른쪽은 −80°~+80° 광선이 두 벽에 닿고 정면 광선 길이 2.12 m 가 벽으로 잡히는 모습.

### 2.2.5) 주요 파라미터
| 이름 | 값 | 위치(파일:행) | 정한 근거 |
|---|---|---|---|
| `period_sec` | 0.5 s | src/north_goal_angle/config/north_goal_angle.yaml:4 | 작년 타이머 주기 유지 [근거: 커밋 9bff661 src/north_goal_angle/north_goal_angle/north_goal_angle.py:51] |
| `los_min_lookahead_m` / `los_max_lookahead_m` | 2.0 / 6.0 m | src/north_goal_angle/config/north_goal_angle.yaml:7-8 | 초기값. 느릴 때 민첩, 빠를 때 오버슈트 방지. 실측 튜닝 전 [근거: src/north_goal_angle/config/north_goal_angle.yaml:7-9] |
| `los_speed_gain` | 1.2 | src/north_goal_angle/config/north_goal_angle.yaml:9 | 초기값, "실측 후 튜닝" [근거: src/north_goal_angle/config/north_goal_angle.yaml:9] |
| `ilos_ki` / `ilos_integral_max` | 0.02 / 50.0 | src/north_goal_angle/config/north_goal_angle.yaml:10-11 | 초기값. 크면 진동, 작으면 상쇄 느림. 클램프 ±50 은 팀원 제출본 값 [근거: docs/전달용/회신_LOS강화_IMU추가_검토.md:14] |
| `gps_cov_threshold` | 2.0 | src/north_goal_angle/config/north_goal_angle.yaml:13 | 공분산 큰 fix(RTK 품질 저하) 거부. 실측 검증 전 |
| `gps_max_speed_mps` | 15.0 m/s | src/north_goal_angle/config/north_goal_angle.yaml:14 | 두 fix 간 물리적 속도 상한, 넘으면 멀티패스로 거부 |
| `require_boat_mode_auto` | true | src/north_goal_angle/config/north_goal_angle.yaml:23 | 수동 이동 중 웨이포인트 소모 방지. 벤치(아두이노 없음)만 false |
| `geofence_polygon` | `[]` (꺼짐) | src/north_goal_angle/config/north_goal_angle.yaml:41 | 대회장 경계 좌표 미측정 |
| `geofence_ray_min_deg` / `max` / `inc` | −80 / 80 / 1.0° | src/north_goal_angle/config/north_goal_angle.yaml:46-48 | 스캔과 같은 상대방위 격자, 161방향 × 변 수 @ 2 Hz |
| `geofence_max_range_m` | 30.0 m | src/north_goal_angle/config/north_goal_angle.yaml:49 | 이보다 먼 경계는 inf |
| `imu_stale_sec` | 0.5 s | src/north_goal_angle/config/north_goal_angle.yaml:51 | 헤딩이 묵으면 광선 방향 전체가 틀어짐 |
| 도착 반경 (코드 상수) | 2.0 / 2.5 / 3.5 / 4.5 m | src/north_goal_angle/north_goal_angle/los_logic.py:75-83 | 도킹은 좁게, 빠르면 넓게(오버슈트 방지) |
| 스무딩 α (코드 상수) | 0.15 / 0.30 / 0.80 (경계 0.5 / 1.5 m) | src/north_goal_angle/north_goal_angle/gps_filter.py:65-70 | 오차가 작으면 강하게, 크면 따라감 |
| `wp_timeout_modes` / `wp_timeout_sec` | [0,1,2,3,5,7] / [120,120,120,90,120,150] s | src/north_goal_angle/config/north_goal_angle.yaml:35-36 | 0~5 는 작년 순번 값과 같게(회귀 테스트로 대조), 도킹 150 s 는 ship_dock 최악 ≈129 s·dwell 60 s 보다 크게(Davidson 10-29) [근거: src/north_goal_angle/test/test_wp_timeout.py:1] [근거: 커밋 b337ad8] |
| 수신기 `rate` | 1.0 Hz (A·B) | src/ublox/ublox_gps/config/zed_f9p_rover.yaml:30 | 1.00 Hz 안정 발행 확인. 올릴지는 물 위에서 제어 주기와 함께 판단 [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:30-33] |

### 2.2.6) 이상 상황 대응
| 상황 | 감지 방법 | 코드의 동작 | 근거 |
|---|---|---|---|
| fix 없음 (냉시작·실내·안테나) | status < 0, (0, 0), NaN, 범위 밖 | `have_fix` 해제, 방위·거리 미발행. `/wp_mode` 는 계속, 도착 판정 정지, 시간 초과 전진만 유지. `ship_goal_angle` 은 목표각 2.0 s 무수신 시 `/yaw_error` 중단 | src/north_goal_angle/north_goal_angle/north_goal_angle.py:282-286, 484-488, 540-541 · src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py:104-106 |
| fix 를 얻었다가 잃음 | 위와 같음 | WARN 1회 후 발행 중단, 다시 얻으면 INFO 후 재개 | src/north_goal_angle/north_goal_angle/north_goal_angle.py:283-284, 298-299 |
| RTK 품질 저하 프레임 | `position_covariance[0]` > 2.0 | 그 프레임만 버림. 직전 위치·`have_fix` 유지 | src/north_goal_angle/north_goal_angle/gps_filter.py:44-45 · src/north_goal_angle/north_goal_angle/north_goal_angle.py:288-297 |
| 순간이동(멀티패스) | 직전 fix 대비 속도 > 15 m/s | 그 프레임 버림 | src/north_goal_angle/north_goal_angle/gps_filter.py:48-54 |
| `/ublox_gps_node/fix` 무수신(USB 분리 등) 또는 장시간 연속 기각 | 마지막으로 **받아들인** fix 후 `gps_stale_sec` 2.5 s 초과(1 Hz 기준 두 프레임 연속 손실) | ERROR(진입 1회 + 5 s 주기), 방위·거리 정지, 지오펜스 빈 배열, 도착 판정·ILOS 적분 정지. `/wp_mode`·20000·시간 초과 진행은 유지. 받아들인 프레임이 오면 INFO 후 재개 (2026-10-29 b288014) | src/north_goal_angle/north_goal_angle/north_goal_angle.py:423-441, 447, 464 · src/north_goal_angle/north_goal_angle/gps_guard.py:77, 98 · src/north_goal_angle/config/north_goal_angle.yaml:15 |
| NTRIP 계정 파일 없음 | 파일·host·mountpoint 확인 | `ntrip_client` 만 안 띄우고 나머지는 정상 기동 (RTK 보정 없음) | src/ntrip_client/launch/ntrip_client_launch.py:63-76 |
| NTRIP 세션 끊김 | 드라이버 (`rtcm_timeout_seconds` 4) | 5 s 간격 10회 재연결. 마운트포인트 오류(404)는 재시도 없이 종료 | src/ntrip_client/launch/ntrip_client_launch.py:101-105 |
| `/imu/yaw` 없음·0.5 s 초과 | 마지막 수신 시각 | 지오펜스 빈 배열 | src/north_goal_angle/north_goal_angle/north_goal_angle.py:367-377 |
| 이미 경계 밖 | 레이 캐스팅 | 지오펜스 빈 배열 + ERROR (웨이포인트로 복귀) | src/north_goal_angle/north_goal_angle/north_goal_angle.py:381-386 |
| 지오펜스 구독자 0 | `get_subscription_count()` | 부팅 5 s 후 ERROR 반복 | src/north_goal_angle/north_goal_angle/north_goal_angle.py:415-420 |
| 웨이포인트 파일 오류 | `parse_waypoints` | 위치·이유 ERROR 후 노드 기동 거부 | src/north_goal_angle/north_goal_angle/north_goal_angle.py:231-248 |
| RC 가 AUTO 아님 | `/boat_mode` ≠ 2 | 웨이포인트 전진 정지, 타이머·적분 초기화 | src/north_goal_angle/north_goal_angle/north_goal_angle.py:502-511 |

진단용으로 `healthcheck` 는 fix 메시지가 안 오는 DEAD 와 와도 쓸 수 없는 NOFIX 를 구분해 표시한다. 제어에는 물리지 않는다 [근거: src/ssf_tools/ssf_tools/healthcheck.py:189-194].

### 2.2.7) 2025 대비 개선점
| 작년 | 문제 | 올해 | 근거 커밋 |
|---|---|---|---|
| fix 상태를 보지 않고 위경도 대입 [근거: 커밋 9bff661 src/north_goal_angle/north_goal_angle/north_goal_angle.py:54-55] | fix 없을 때 (0, 0) 기준 방위 47.96° 발행 | `gps_guard.fix_is_usable` 로 거부, 방위 발행 중단 | 02944d8 |
| 직접 방위 + 고정 도착 반경 3.0 m [근거: 커밋 9bff661 src/north_goal_angle/north_goal_angle/north_goal_angle.py:26] | 외란에 밀리면 경로 복귀 못 함, 이상치·지터 무방비 | LOS·ILOS, 공분산·점프·스무딩 필터, 속도별 도착 반경 | 9221750 |
| 경계 방어 없음 | 경기장 이탈은 실격 | `/geofence_state` 신설 후 원뿔 → 광선 방식 | 2cb050b · b678154 |
| mode 7 에서 `20000` 발행 [근거: 커밋 9bff661 src/north_goal_angle/north_goal_angle/north_goal_angle.py:72-73] | 도킹 노드와 발행자 충돌 | 폴백은 mode 5·8 만 | 0b854c1 |
| `time.time()` [근거: 커밋 9bff661 src/north_goal_angle/north_goal_angle/north_goal_angle.py:75] | 벽시계 기준 시간 초과 | `time.monotonic()` | 0b854c1 |
| 웨이포인트 코드 하드코딩 [근거: 커밋 9bff661 src/north_goal_angle/north_goal_angle/north_goal_angle.py:11-23] | 좌표 수정에 코드 편집, 검증 없음 | `waypoints.yaml` + 로더 검증, 녹화 도구, `waypoints_file:=` 인자 | 2758653 · 7a2e803 · 8fda096 · ac486b1 |
| RC 모드와 무관하게 전진 | 수동 이동 중 웨이포인트 소모 | `require_boat_mode_auto` | df16eee |
| 드라이버 launch 가 M8P 설정 고정 | F9P 가 M8P 설정으로 동작, NACK | F9P 이동국 설정 신설, `config` 인자 | c1b0d81 |
| F9P·M8P 모두 `portable` 모델·M8P EEPROM 저장 | 수상 선박에 맞지 않음, 설정이 수신기에 남음 | 두 배 `sea`, `save` 제거, 메인 launch `gps_config` 전달 | ed2a144 · c7c0c59 |
| NTRIP 계정이 launch 에 평문 | 공개 저장소 노출 | 계정 파일을 `~/.ssf/` 로, 없으면 노드만 생략 | f98a8f9 · 89818a5 |

### 2.2.8) 검증과 시험 결과
| 테스트 파일 | 개수 | 검증 내용 |
|---|---|---|
| src/north_goal_angle/test/test_gps_guard.py | 18 (스크립트 check) | NO_FIX·Null Island·NaN·inf·범위 밖 거부, SBAS·GBAS(RTK) fix·남반구·경계값·적도 근처 수용, 2026-08-06 관측 사례 회귀 |
| src/north_goal_angle/test/test_gps_filter.py | 7 | 첫 fix 통과, 공분산 거부·0 은 정보 없음, 점프 거부, 정상 속도 추정, 스무딩 강약 |
| src/north_goal_angle/test/test_los_logic.py | 12 | 거리·방위, 전방주시 클램프, XTE 부호, 좌우 이탈 시 반대로 보정, 경로 위면 경로 방위, 전방주시 클수록 완만, ILOS 보정, 도착 반경 |
| src/north_goal_angle/test/test_mission_gate.py | 5 | AUTO 에서만 전진, MANUAL·WAIT·미수신 대기, 벤치 모드 항상 전진 |
| src/north_goal_angle/test/test_waypoint_loader.py | 15 | dict·list 형식, 순서 보존, 키·필드 누락, 범위 밖 좌표, 허용 외 mode, 정수 아님, 음수 dwell, 숫자 아님, 오류 메시지에 순번 |
| src/north_goal_angle/test/test_waypoint_recorder.py | 8 | 평균(빈 목록·단일·지터 감소), yaml 출력 형식·자동 라벨, 출력이 로더로 다시 읽힘 |

`tools/run_tests.sh north_goal_angle` 실행 결과 47개 통과이고, `test_gps_guard.py` 는 pytest 함수가 아닌 스크립트 방식이라 따로 실행해 18/18 통과했다(2026-10-28, Mac) [근거: src/north_goal_angle/test/test_gps_guard.py:76-81].

| 실측 항목 | A배 | B배 | 측정일 |
|---|---|---|---|
| 냉시작 후 첫 fix 까지 시간 | ____ | ____ | ____ |
| 위성 수 / pDOP | ____ | ____ | ____ |
| RTK FLOAT → FIXED 수렴 시간 | ____ | ____ | ____ |
| RTK FLOAT 위치 산포 (정지 5분) | ____ m | ____ m | ____ |
| RTK FIXED 위치 산포 (정지 5분) | ____ m | ____ m | ____ |
| `/rtcm` 수신 주기 | ____ Hz | ____ Hz | ____ |
| 공분산 필터 기각률 (FLOAT / FIXED) | ____ | ____ | ____ |
| 웨이포인트 도착 오차 (물 위) | ____ m | ____ m | ____ |
| 경로선 이탈 XTE (물 위, 직선 구간) | ____ m | ____ m | ____ |
| 대회장 웨이포인트·경계 좌표 녹화 | ____ | ____ | ____ |

참고로 선체 장착 전 수신기만 들고 나간 2026-08-07 야외 시험 기록(ZED-F9P)이 있다. fix status 0, carrSoln 2(FIXED), 수평 정확도 ±0.015 m, 위성 18개, pDOP 1.63, `/rtcm` 5.4 Hz 였고 처음 60 s 는 FLOAT(±0.10 m)였다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:453-466] [근거: src/ublox/ublox_gps/config/zed_f9p_rover.yaml:70-74]. 같은 날 `waypoint_recorder` 로 4점을 녹화했고 1.93 m 간격 두 점이 구분됐다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:514-521]. 배에 장착한 상태가 아니고 A배 수신기는 시험하지 않았으므로 위 표에는 넣지 않았다(부록 B).

## 부록 A. A배·B배 차이
| 항목 | A배 (1번배) | B배 (2번배) | 근거 |
|---|---|---|---|
| 수신기 | C94-M8P | ZED-F9P | CLAUDE.md:60 |
| 안테나 | 키트 동봉 L1 패치 (확인 필요) | ANN-MB | docs/전달용/하드웨어_배치_요구사항.md:84-85 |
| 설정 파일 | `c94_m8p_rover.yaml` | `zed_f9p_rover.yaml` | src/launch_files/launch/launch_files.launch.py:35-37 |
| 메인 launch 인자 | `gps_config:=c94_m8p_rover.yaml` **필수** | 인자 없음(기본값) | src/launch_files/launch/launch_files.launch.py:87-88 |
| udev idProduct | 01a8 | 01a9 | src/ublox/ublox_gps/config/c94_m8p_rover.yaml:6 · src/ublox/ublox_gps/config/zed_f9p_rover.yaml:21 |
| BeiDou | 끔 | 켬 | src/ublox/ublox_gps/config/c94_m8p_rover.yaml:42 · src/ublox/ublox_gps/config/zed_f9p_rover.yaml:52 |
| `uart1` 블록 | 19200, RTCM3 입력 유지 | 없음(USB 주입만) | src/ublox/ublox_gps/config/c94_m8p_rover.yaml:35-38 · src/ublox/ublox_gps/config/zed_f9p_rover.yaml:55-60 |
| `dgnss_mode` | 3 (적용 여부 미확인) | 없음 — 이 펌웨어에선 무시됨, 그래도 FIXED 도달 | src/ublox/ublox_gps/config/c94_m8p_rover.yaml:45 · src/ublox/ublox_gps/config/zed_f9p_rover.yaml:63-73 |
| UBX-AID 발행 | `aid.hui` 만 끔 | `aid.all: false` | src/ublox/ublox_gps/config/c94_m8p_rover.yaml:53-54 · src/ublox/ublox_gps/config/zed_f9p_rover.yaml:91-92 |
| `sea` 모델 실기 확인 | 미확인 | 휴대 시험만 | src/ublox/ublox_gps/config/c94_m8p_rover.yaml:7 |
| `north_goal_angle` 코드·파라미터 | 같음 | 같음 | src/north_goal_angle/config/north_goal_angle.yaml:1-3 |

## 부록 B. 확인 필요 목록
- **과제 F1 과 기록 불일치.** 과제 목록은 "측위 자체를 아직 한 번도 못 봤다"인데 [근거: docs/문제와작업/과제_목록_전체.md:93], 2026-08-07 휴대 시험에서 ZED-F9P RTK FIXED 기록이 있다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:451-463]. F1 이 "선체 장착 후 / A배 / 현 노트북" 기준인지 팀장 확인 후 2.2.8 참고 문단을 살릴지 정한다.
- **A배 안테나.** 지시서·현재 원고는 "C94-M8P + ANN-MB", 배치 문서는 "키트 동봉 L1 패치 안테나" [근거: docs/전달용/하드웨어_배치_요구사항.md:85]. 실물 확인.
- 두 수신기 데이터시트 값(채널 수, RTK 수평 정밀도, 최대 갱신율, 냉시작 TTFF) — 저장소에 없다. 문서명과 함께 2.2.1 표 ____ 채움.
- A배 펌웨어 버전 ____, `sea` 모델 실기 확인 [근거: src/ublox/ublox_gps/config/c94_m8p_rover.yaml:7], C94-M8P 의 RTK FIXED 도달 여부와 `dgnss_mode: 3` 적용 여부.
- 2.2.8 실측 표 전부: 첫 fix 시간·위성 수, FLOAT/FIXED 산포(정지 5분), 수렴 시간, `/rtcm` 주기 — 첫 야외 측위(F1). 도착 오차·XTE — 물 위 시험. 대회장 좌표 — 대회 당일 녹화(D7).
- RTK 끊김 빈도(blackbox) — LOS 보강·DR 필요 여부 판단용으로 측정하기로 했다 [근거: docs/전달용/회신_LOS강화_IMU추가_검토.md:29] [근거: docs/전달용/회신_LOS강화_IMU추가_검토.md:66].
- `gps_cov_threshold` 2.0 의 물리적 의미(`position_covariance[0]` 단위)와 RTK FLOAT 프레임이 실제로 통과하는지.
- LOS 파라미터(`los_speed_gain`, lookahead 2~6 m, `ilos_ki`)는 초기값이다 [근거: src/north_goal_angle/config/north_goal_angle.yaml:9].
- (해결 2026-10-29 b288014) fix 신선도 감시 — 받아들인 fix 가 `gps_stale_sec`(2.5 s) 넘게 없으면 위치 파생값만 멈춘다 [근거: src/north_goal_angle/north_goal_angle/gps_guard.py:77]. 남은 것: Ubuntu GPS USB 분리 실물 시험(다시 꽂았을 때 `/fix` 복구 여부), 그 뒤 ublox 재시작(C2).
- 드라이버 launch 의 `OnProcessExit → Shutdown` [근거: src/ublox/ublox_gps/launch/ublox_gps_node-launch.py:73-78] 이 메인 launch 에 include 된 상태에서 드라이버 종료 시 전체 launch 를 내리는지 Ubuntu 에서 확인.
- (해결 2026-10-29 b337ad8) 시간 초과가 웨이포인트 순번에 묶여 있던 문제는 mode 기준 파라미터로 바뀌었다 [근거: src/north_goal_angle/config/north_goal_angle.yaml:35-36]. 남은 것: Ubuntu 에서 파라미터 타입·잘못된 yaml 거부·짧은 타임아웃 벤치 확인.
- 마지막 웨이포인트를 끝내면 `/wp_mode` 발행도 멈춘다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:450-451]. 의도인지 확인.
- 현 `waypoints.yaml` 좌표는 작년 하드코딩 좌표와 같다 [근거: src/north_goal_angle/config/waypoints.yaml:20-30] [근거: 커밋 9bff661 src/north_goal_angle/north_goal_angle/north_goal_angle.py:12-22]. 대회장 좌표·경계 폴리곤은 미측정, 마운트포인트도 대회장 기준으로 다시 고른다 [근거: docs/절차/웨이포인트_녹화절차.md:42-49].
- 녹화 절차서의 "도착 반경 3m" [근거: docs/절차/웨이포인트_녹화절차.md:105] 는 현 코드(2.0~4.5 m 가변)와 다르다.

## 부록 C. 다른 담당과 겹치는 내용
- 지오펜스 `min` 병합·정지 판정(`geofence_stop_margin_m`, ±60°)은 LiDAR 담당(Ⅲ-2.3). 이 절은 광선 생성만 쓴다. 광선 상대방위는 `θ = yaw + rel`(시계방향 양수)로 만들어지므로 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:395], `ship_direction` 의 LiDAR 각도 격자(정면 80) 변환 방향과 부호가 맞는지 LiDAR 담당과 함께 확인한다 [근거: src/ship_direction/ship_direction/ship_direction.py:620-621].
- `/ublox_gps_node/fix_velocity` 로 COG 를 구해 IMU 헤딩 오프셋을 잡는 것은 IMU 담당(Ⅲ-2.1). 지오펜스 방향 정확도는 `mount_offset_deg` 실측에 달려 있다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:35-40].
- 전체 임무 상태기계(Ⅲ-3)와 위치유지 `ship_back` 의 RTK 폴백은 팀장 담당. 이 절은 `/wp_mode` 를 누가 어떻게 바꾸는지만 썼다.
- `/north_goal_angle_tp` → `ship_goal_angle` → `/yaw_error` 체인과 2.0 s stale 처리는 하류 노드 쪽 설명과 겹친다 [근거: src/ship_goal_angle/config/ship_goal_angle.yaml:8].
- 현재 원고 Ⅲ-1.2 표의 GPS 토픽 `/fix` 는 실제 이름 `/ublox_gps_node/fix` 로 고쳐야 한다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:187]. 현재 원고 Ⅲ-2.2 의 "FIXED 시 ±1.5 cm 급"은 F9P 휴대 시험 1회 값이다(2.2.8 참고).

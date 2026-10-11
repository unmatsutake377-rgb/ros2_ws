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
- 특수 신호 상수 공유 모듈화 — 패키지 8개 `package.xml`/`setup.py` 를 건드려 빌드 전체가 위험. 상수표(§3 특수 신호) + `review_node.py`(⚠️ 미확인: 저장소에 없음 — 위치 확인 필요) 검사로 충분. 대회 후 정리.
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
| 카메라 | **RealSense D455 현역** / OAK-1 W PoE **미구매**(RGB 전용·뎁스 없음·광각 120~150° DFOV·PoE=이더넷). 코드는 **중립화**(상세 §3-3) |
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

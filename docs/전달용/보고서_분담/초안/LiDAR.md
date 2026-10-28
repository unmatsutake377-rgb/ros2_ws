# LiDAR 초안
## 1.3) 센서 — 담당 줄
| 장비명 | 용도 |
|---|---|
| RPLIDAR A3 — A배 개체 80BF(약 11.1 Hz / 0.250°), B배 개체 918A(11.00 Hz / 0.223°) [근거: docs/문제와작업/라이다_고장진단_20260818.md:76-77] [근거: docs/문제와작업/라이다_고장진단_20260818.md:103] | 장애물·부표 거리, 갭 탐색, 감속 신호, 지오펜스 병합, 센서 감시. Sensitivity 모드, 25 m [근거: CLAUDE.md:58]. 개체는 `tools/lidar_id.py` 로 시리얼을 물어 구분하고 udev 로 `/dev/ttyLiDAR` 를 만든다(A배 어댑터 시리얼, B배 물리 포트) [근거: docs/기준/CLAUDE_상세.md:136-139] |

## 2.3) LiDAR 처리
`rplidar_node` 가 내는 `/scan` 을 `ship_direction` 이 받아 ① 갭 선택 회피(Ⅲ-4.4), ② 감속 신호 `/obstacle_distance_array`, ③ 지오펜스 병합의 바탕 스캔, ④ 센서 감시(Ⅲ-5)에 쓴다 [근거: src/ship_direction/ship_direction/ship_direction.py:262-276]. 이 절은 스캔 수신, 정면 정렬, `inf`·`nan` 처리, 감속 신호 필터, 지오펜스 병합까지의 신호 처리를 다룬다.

임무 노드(`ship_gate`·`ship_back`·`ship_turn`·`ship_dock`)도 같은 QoS 로 `/scan` 을 구독한다 [근거: 커밋 fd3b39e]. 카메라는 방위각만 내고 거리는 LiDAR 전방 섹터 최소거리로 얻는다 [근거: CLAUDE.md:152].

### 2.3.1) 장비 사양과 선정 이유
| 항목 | A배 | B배 | 출처 |
|---|---|---|---|
| 모델 / 펌웨어 | RPLIDAR A3 (0x31) / 1.32 | RPLIDAR A3 (0x31) / 1.25 | [근거: docs/문제와작업/라이다_고장진단_20260818.md:79] [근거: docs/기준/CLAUDE_상세.md:136-138] |
| 개체 시리얼 앞 4자리 | 80BF | 918A | [근거: docs/문제와작업/라이다_고장진단_20260818.md:76-77] |
| 회전 주파수 (실측) | 약 11.1 Hz | 11.00 Hz | [근거: docs/문제와작업/라이다_고장진단_20260818.md:103] [근거: docs/문제와작업/라이다_고장진단_20260818.md:77] |
| 한 바퀴 점 수 / 각분해능 (실측) | 1439점 / 0.250° | 1612점 / 0.223° | [근거: docs/문제와작업/라이다_고장진단_20260818.md:76-77] |
| 제조사 문서값 | 10 Hz / 0.225° | 10 Hz / 0.225° | [근거: docs/기준/CLAUDE_상세.md:111-114] |
| 최대 거리 · 스캔 모드 · 통신 | 25 m · Sensitivity · 256000 bps | 같음 | [근거: CLAUDE.md:58] [근거: src/rplidar_ros/launch/rplidar_a3_launch.py:43-47] [근거: src/rplidar_ros/launch/rplidar_a3_launch.py:63-65] |
| 포트 고정 (udev) | USB 어댑터 고유 시리얼 | 어댑터 시리얼이 IMU 와 같아 `KERNELS` 물리 포트 | [근거: docs/절차/2번배_udev_규칙_초안.md:3-4] [근거: docs/절차/2번배_udev_규칙_초안.md:19-27] |

회전수는 드라이버 파라미터로 바꿀 수 없어 개체마다 실측값이 다르고 [근거: docs/기준/CLAUDE_상세.md:117-125], 그 결과 점 개수에 의존하는 값이 개체별로 다르게 동작한다 [근거: docs/문제와작업/라이다_고장진단_20260818.md:105-107]. 센서와 드라이버는 작년 것을 그대로 쓰며 [근거: docs/전달용/보고서_분담/03_LiDAR.md:56], 거리 정보를 LiDAR 하나가 맡는 구조라 360° 거리 스캔이 필요하다 [근거: CLAUDE.md:152]. 보유한 2대를 두 배에 한 대씩 배정했다 [근거: docs/문제와작업/라이다_고장진단_20260818.md:89].

### 2.3.2) 코드 출처
| 패키지 | 원래 출처 | 작년부터 | 올해 바꾼 것 | 근거 커밋 |
|---|---|---|---|---|
| `rplidar_ros` | 외부. Slamtec 공식 RPLIDAR ROS2 드라이버 2.1.4 [근거: src/rplidar_ros/package.xml:4] | 있음. 외부 저장소 링크를 풀어 편입 [근거: 커밋 19cfde7] | `rplidar_a3_launch.py` 정리만(미사용 import 제거, 인자 기본값 선언 방식, udev·scan_mode·QoS 주석). 기본값과 드라이버 코드는 그대로 [근거: src/rplidar_ros/launch/rplidar_a3_launch.py:10-16] | 673c20a |
| `ship_direction` | 작년 우리 코드 | 있음 [근거: 커밋 9bff661] | 제어·워치독 타이머 분리, 페일세이프 3상태, `/scan` QoS, 감속 신호 median, 시간 투표 OFF, 지오펜스 병합·하드 가드, 후진 상태기계·변화율 제한, `half_width` 0.45→0.29, zone 선별 함수화 | 02bfc39, 2f4410a, da327a1, 16ff3c9, fd65e2f, 5bc8d96, fd3b39e, 849c570, b678154, e3490db, ed31dda, aabb3bc, 000e46c, 27e2f45, c5ffe2c |

### 2.3.3) 데이터 흐름
| 토픽 | 입력·출력 | 메시지 타입 | 주기 | QoS | 상대 노드 |
|---|---|---|---|---|---|
| `/scan` | 입력 | `LaserScan` | 개체 회전수 (약 11 Hz) | BEST_EFFORT · KEEP_LAST · depth 1 [근거: src/ship_direction/ship_direction/ship_direction.py:62-66] | `rplidar_node` |
| `/geofence_state` | 입력 | `Float32MultiArray` | 0.5 s [근거: src/ship_direction/config/ship_direction.yaml:29] | depth 10 [근거: src/ship_direction/ship_direction/ship_direction.py:266] | `north_goal_angle` |
| `/yaw_error` · `/candidate_angle` · `/wp_mode` | 입력 | `Float32` · `Float32` · `Int32` | — | depth 10 [근거: src/ship_direction/ship_direction/ship_direction.py:263-265] | `ship_goal_angle` · 임무 노드·`north_goal_angle` |
| `/desired_angle` | 출력 | `Float32` | 10 Hz [근거: src/ship_direction/config/ship_direction.yaml:5] | depth 10 | `motor_control` |
| `/obstacle_distance_array` | 출력 | `Float32MultiArray` [거리 m, 각도 °] | 10 Hz | depth 10 | `motor_control`, `blackbox` |
| `/failsafe_level` | 출력 | `Int32` (0/1/2) | 10 Hz [근거: src/ship_direction/config/ship_direction.yaml:6] | depth 10 | `motor_control`, `blackbox` |
| `/obstacle_reject_count` | 출력 | `Int32` | 10 Hz | depth 10 | `blackbox` (관측 전용) [근거: src/ship_direction/ship_direction/ship_direction.py:273-275] |

출력은 모두 `control_cb`·`watchdog_cb` 두 고정주기 타이머에서 나간다 [근거: src/ship_direction/ship_direction/ship_direction.py:317-318]. 구독자 BEST_EFFORT 는 RELIABLE 로 내는 현행 드라이버와 호환된다 [근거: src/ship_direction/ship_direction/ship_direction.py:58-60].

그림: LiDAR 데이터 흐름도 — `rplidar_node` →`/scan`→ `ship_direction`[scan_callback(저장) → control_cb 10 Hz(정면 0~160° 추출 → 지오펜스 병합 → 회피 / median 감속 신호), watchdog_cb 10 Hz(SensorWatch)] → `/desired_angle`·`/obstacle_distance_array`·`/failsafe_level` → `motor_control`, `/obstacle_reject_count` → `blackbox`. `north_goal_angle` →`/geofence_state`→ 병합 블록 화살표를 함께 그린다.

### 2.3.4) 처리 알고리즘
**① 수신과 계산의 분리.** `scan_callback` 은 스캔을 저장만 하고 계산·발행은 `control_cb` 가 10 Hz 로 한다 [근거: src/ship_direction/ship_direction/ship_direction.py:332-352] [근거: src/ship_direction/ship_direction/ship_direction.py:515-547]. 작년에는 계산이 스캔 콜백 안에 있어 LiDAR 가 끊기면 `/desired_angle` 이 한 번도 나가지 않았다 [근거: src/ship_direction/ship_direction/ship_direction.py:74-79]. 콜백과 타이머가 여러 스레드에서 돌므로 공유 상태는 `Lock` 안에서 스냅샷을 떠서 쓴다 [근거: src/ship_direction/ship_direction/ship_direction.py:908] [근거: src/ship_direction/ship_direction/ship_direction.py:518-524].

**② 정면 정렬.** LiDAR 프레임의 80° 를 선수 정면으로 둔다(`LIDAR_FORWARD_DEG = 80.0`) [근거: src/ship_direction/ship_direction/ship_direction.py:23]. 0~160°, 즉 정면 ±80° 만 쓰고, 인덱스는 `min_index = int((0 − angle_min)/Δθ)`, `max_index = int((160 − angle_min)/Δθ)` 로 잡는다 [근거: src/ship_direction/ship_direction/ship_direction.py:733-735]. Δθ 를 메시지에서 매번 읽으므로 각분해능이 다른 두 개체에 같은 코드가 돈다 [근거: src/ship_direction/ship_direction/ship_direction.py:723-725]. 이 80° 가 실제 선수와 맞게 장착해야 한다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:549-551].

**③ `inf`·`nan` 처리.** 측정 없음은 쓰임새별로 다르게 다룬다. 회피 마스크에서는 빈 셀(0) [근거: src/ship_direction/ship_direction/ship_direction.py:755-761], 빈 구간 가장자리 거리로는 `detection_distance` [근거: src/ship_direction/ship_direction/ship_direction.py:26-32], 감속 신호에서는 유효점에서 제외(유한하고 0 초과만 유효)다 [근거: src/ship_direction/ship_direction/failsafe.py:53-56]. `detection_distance` 는 기본 3.0 m, 게이트 구간(`wp_mode` 0·1) 2.0 m 로 `/wp_mode` 를 받을 때 바꾼다 [근거: src/ship_direction/ship_direction/ship_direction.py:385-389].

**④ 감속 신호의 공간 median.** 각 인덱스 i 의 창 [i−2, i+2](커널 5)에서 유효점이 3개 이상이면 그 중앙값(짝수면 위쪽)을 i 의 거리로 쓰고, 3개 미만이면 고립 반사로 버린다. 그 최소값과 각도를 `/obstacle_distance_array` 로 낸다 [근거: src/ship_direction/ship_direction/failsafe.py:31-35] [근거: src/ship_direction/ship_direction/ship_direction.py:578-584]. 위쪽 중앙값은 거리를 크게 잡아 가짜 감속 쪽으로 기울지 않게 한다 [근거: src/ship_direction/ship_direction/failsafe.py:35]. 이 필터는 감속 신호에만 걸고 회피 마스크에는 걸지 않아 회피 접촉 성능이 바뀌지 않는다 [근거: src/ship_direction/ship_direction/ship_direction.py:154-160]. 시뮬(시드 20, 물보라 30%)에서 가짜 감속이 36.8% 에서 8.9% 로 줄었고 접촉은 전 조건 0 이었다 [근거: src/ship_direction/config/ship_direction.yaml:39]. 버린 점 수는 `/obstacle_reject_count` 로 내 물보라 대리 지표로 기록한다 [근거: src/ship_direction/ship_direction/failsafe.py:17-20]. 감속비 적용은 `motor_control` 이 이 토픽을 받아 한다(Ⅲ-4.6).

**⑤ 지오펜스 병합과 경계 정지.** 경계 광선은 `north_goal_angle` 이 만든다(GPS 담당 절 참고). `/geofence_state = [angle_min_deg, angle_inc_deg, r0, r1, …]`(상대방위, 멀면 `inf`)를 받아 [근거: src/ship_direction/ship_direction/ship_direction.py:359-369], 자율 회피 구간에서 셀마다 `rel = a − 80`, `k = round((rel − a0)/inc)` 로 광선을 찾고 `ranges[i] = min(real, geofence)` 로 병합한다 [근거: src/ship_direction/ship_direction/ship_direction.py:619-631]. 병합이 마스크 생성 전이라 경계도 실제 장애물과 같은 팽창·갭 선택을 거친다 [근거: src/ship_direction/ship_direction/ship_direction.py:747-771]. 빈 배열이나 2.0 s 넘게 묵은 경계는 병합하지 않는다. 묵은 "없는 벽" 이 배를 가두기 때문이다 [근거: src/ship_direction/ship_direction/ship_direction.py:603-609]. 임무 노드가 조향하는 SPIN·candidate 구간은 병합할 배열이 없어 하드 가드를 따로 둔다. 최근접 경계가 2.0 m 안이고 그 방위가 전진하려는 방향(candidate 각, SPIN 은 0°) ±60° 안이면 미션을 무시하고 STOP_HOLD 를 낸다 [근거: src/ship_direction/ship_direction/ship_direction.py:661-676] [근거: src/ship_direction/ship_direction/ship_direction.py:694-699].

### 2.3.5) 주요 파라미터
| 이름 | 값 | 위치(파일:행) | 정한 근거 |
|---|---|---|---|
| `detection_distance_default` | 3.0 m | src/ship_direction/config/ship_direction.yaml:57 | 시뮬: 1.8 이면 8/8 충돌, 3.0 이면 5/8 통과 |
| `detection_distance_gate` / `gate_wp_modes` | 2.0 m / [0, 1] | src/ship_direction/config/ship_direction.yaml:58-59 | 게이트가 좁아 3.0 이면 접촉 증가. 작년 키 `wp_mode`==2 는 게이트가 아니어서 정정 [근거: 커밋 5bc8d96] |
| `obst_median_kernel` | 5 | src/ship_direction/config/ship_direction.yaml:43 | 가짜 감속 36.8%→8.9%. 군집 물보라면 7/9 검토 [근거: src/ship_direction/config/ship_direction.yaml:41-42] |
| `max_spike_ratio` / `border_margin` | 0.01 / 2셀 | src/ship_direction/config/ship_direction.yaml:73-74 | 작년 값 유지 [근거: 커밋 9bff661 src/ship_direction/ship_direction/ship_direction.py:52-53] |
| `geofence_stale_sec` | 2.0 s | src/ship_direction/config/ship_direction.yaml:29 | north 주기 0.5 s 의 4배 |
| `geofence_stop_margin_m` / `geofence_stop_cone_deg` | 2.0 m / 60° | src/ship_direction/config/ship_direction.yaml:33-34 | 도킹 구간 경계 이탈 방지 [근거: 커밋 e3490db] |

yaml 과 노드 `declare_parameter` 기본값은 모두 같다 [근거: src/ship_direction/ship_direction/ship_direction.py:178-255]. 통합 launch 는 yaml 을 넘기지 않아 운용값은 노드 기본값이다 [근거: src/launch_files/launch/launch_files.launch.py:240-245].

### 2.3.6) 이상 상황 대응
| 상황 | 감지 방법 | 코드의 동작 | 근거 |
|---|---|---|---|
| 스캔 0.7 s / 3.0 s 넘게 끊김 | `SensorWatch` 경과 시간 | 레벨 1(각도 유지·감속) / 레벨 2(STOP_HOLD). Ⅲ-5 | [근거: src/ship_direction/ship_direction/failsafe.py:118-124] |
| 부팅 중 스캔 없음 | 첫 수신 전(ARMED 전) | 감시 안 함, `/desired_angle` 미발행 | [근거: src/ship_direction/ship_direction/ship_direction.py:534-537] |
| `angle_increment` 0·음수·NaN | `scan_callback` 입구 검사 | 스캔 폐기, 즉시 레벨 2, 다음 정상 스캔에서 해제 | [근거: src/ship_direction/ship_direction/ship_direction.py:339-352] [근거: src/ship_direction/ship_direction/ship_direction.py:526-532] |
| 물보라 고립 반사 | median 창 유효점 3개 미만 | 감속 신호에서 제외, 기각 수 발행 | [근거: src/ship_direction/ship_direction/failsafe.py:57-62] |
| 지오펜스 빈 배열·2 s 초과 | 수신값·수신 시각 | 병합·하드 가드 안 함 | [근거: src/ship_direction/ship_direction/ship_direction.py:603-609] [근거: src/ship_direction/ship_direction/ship_direction.py:652-655] |
| 조향값 NaN·inf | 변화율 제한 입구 | STOP_HOLD | [근거: src/ship_direction/ship_direction/ship_direction.py:443-445] |

### 2.3.7) 2025 대비 개선점
| 작년 | 문제 | 올해 | 근거 커밋 |
|---|---|---|---|
| 계산·발행이 `scan_callback` 안 | LiDAR 끊기면 `/desired_angle` 0회 | 저장만, 10 Hz 타이머가 항상 발행 | 02bfc39 |
| `/scan` depth 10 RELIABLE [근거: 커밋 9bff661 src/ship_direction/ship_direction/ship_direction.py:25-26] | 묵은 스캔이 몰려 1초 전 장면으로 조향, 워치독 왜곡 | BEST_EFFORT · depth 1 | fd3b39e |
| `rear_obstacle_ignore_margin` 1.0 [근거: 커밋 9bff661 src/ship_direction/ship_direction/ship_direction.py:56] | 물보라 0.3 m 하나가 1.6 m 부표를 마스크에서 지움(17→0셀) | 제거 | 2f4410a |
| 감속 신호가 필터 없는 최소값 | 물보라 한 점에 감속 | 공간 median 커널 5 | 849c570 |
| `angle_increment` 무방어 | 0 나눗셈으로 노드 사망 | 하드 폴트로 즉시 정지 | da327a1 |
| 탐지 거리 1.8 m, 게이트 키 `wp_mode`==2 [근거: 커밋 9bff661 src/ship_direction/ship_direction/ship_direction.py:46-47] | 시뮬 8/8 충돌, 게이트 아닌 모드 | 3.0 / 2.0 m, `gate_wp_modes` [0, 1] | 5bc8d96 |
| 경계 처리 없음 | 경기장 이탈 = 실격 | 가짜 LiDAR 병합 + 하드 가드 | b678154, e3490db |
| `time.time()` | 벽시계 점프 | `time.monotonic()` | 02bfc39 |

### 2.3.8) 검증과 시험 결과
`failsafe.py` 는 rclpy 를 임포트하지 않고 시각을 인자로 받아 ROS 없이 시험한다 [근거: src/ship_direction/ship_direction/failsafe.py:1-8]. `test_avoidance_filters.py` 는 ROS 모듈을 스텁으로 넣고 실제 노드 메서드를 부른다 [근거: src/ship_direction/test/test_avoidance_filters.py:7-9]. 2026-10-28 `tools/run_tests.sh ship_direction` 에서 35개 모두 통과했다 [근거: tools/run_tests.sh:44-48].

| 테스트 파일 | 개수 | 검증 내용 |
|---|---|---|
| test_failsafe_logic.py (SensorWatch) | 7 | 부팅 오발동 없음, 0.7 s 경고→3.0 s 정지, `/yaw_error` 만 끊김 감지, 가장 나쁜 센서, 지터 무시, 즉시 복구, 하드 폴트 [근거: src/ship_direction/test/test_failsafe_logic.py:21-114] |
| test_failsafe_logic.py (TemporalVote) | 5 | 1프레임 물보라 거부, 지속 장애물 인정, 1프레임 누락 생존, OFF 통과, 스캔 길이 변화 [근거: src/ship_direction/test/test_failsafe_logic.py:116-157] |
| test_failsafe_logic.py (median_min) | 9 | 고립 스파이크 제거, raw 최소값 대조군, 진짜 장애물 유지, 유효점 임계, 위쪽 중앙값, 빈·무효 구간 [근거: src/ship_direction/test/test_failsafe_logic.py:159-220] |
| test_failsafe_logic.py (기각 수) | 6 | 반환 형식 호환, 기각 계수, 원래 무효점 제외, 커널 OFF [근거: src/ship_direction/test/test_failsafe_logic.py:222-267] |
| test_avoidance_filters.py | 8 | 넓은 부표 생존, 1~3셀 소형 소거, 스파이크 임계 길이 비례, 팽창, `inf`·`nan` 가장자리, 호 길이 폭 판정·경계값 [근거: src/ship_direction/test/test_avoidance_filters.py:105-210] |
| test_flake8 · copyright · pep257 | 3 | lint, `colcon test` 에서만. pep257 은 skip [근거: src/ship_direction/test/test_pep257.py:34-37] |

| 실측 항목 | A배 | B배 | 측정일 |
|---|---|---|---|
| 회전 주파수 / 각분해능 | 약 11.1 Hz / 0.250° | 11.00 Hz / 0.223° | 2026-08-18 / 2026-10-07 [근거: docs/문제와작업/라이다_고장진단_20260818.md:76-77] |
| 정면(80°)–선수 정렬 확인 | ____ | ____ | ____ |
| 부표 탐지 최대거리 (`obstacle_min_dist`, 과제 D4) | ____ | ____ | ____ |
| 물보라 기각 수 (회/분, 야외 rosbag) | ____ | ____ | ____ |
| 1 m 안팎 게이트 통과 성공률 (수조) | ____ | ____ | ____ |

배 장착 전 벤치(2026-08-06)에서 손을 대고 떼는 동안 거리와 방위가 함께 움직여, median 이 실제 물체를 통과시킴을 확인했다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:162-170].

## 4.4) 장애물 회피 — ship_direction
`/candidate_angle` 이 20000(미션 없음)이면 LiDAR 갭 선택으로 조향각을 정한다 [근거: src/ship_direction/ship_direction/ship_direction.py:722]. 그 밖의 명령은 먼저 처리한다. 50000 이거나 `wp_mode` 4 진입 후 5 s 이내면 STOP_HOLD [근거: src/ship_direction/ship_direction/ship_direction.py:686-689], 5000·6000 은 그대로 통과 [근거: src/ship_direction/ship_direction/ship_direction.py:702-704], 일반 각 c 는 `80 + c`(c ≤ 180) 또는 `80 − (360 − c)` 로 옮기고 0~160° 밖이면 STOP_HOLD 다 [근거: src/ship_direction/ship_direction/ship_direction.py:707-720].

- 이진 마스크: 병합 거리 r 이 `detection_distance` 미만이면 1(장애물) [근거: src/ship_direction/ship_direction/ship_direction.py:754-761].
- 잡음 정리: 길이 `int(n × max_spike_ratio)` 셀 이하 장애물 구간을 지우고, 양옆 `border_margin` 셀 안에 빈 셀이 있는 장애물 셀을 깎는다 [근거: src/ship_direction/ship_direction/ship_direction.py:830-861]. 고립 1~3셀은 여기서 사라지고 회피 거리의 실제 부표(약 30셀)는 남는다 [근거: src/ship_direction/test/test_avoidance_filters.py:105-143].
- 팽창: 장애물 구간의 최근접 거리 r_min 으로 `m = max(1, round(atan((half_width + clearance)/r_min) / Δθ))` 셀씩 양옆을 넓힌다 [근거: src/ship_direction/ship_direction/ship_direction.py:881-897].
- 폭 판정: 빈 구간 (s, e) 마다 `r_edge = min(양 끝 거리)`, `arc = Δθ·r_edge·(e − s)` 가 `min_required_width = half_width×2 + clearance` 이상인 구간만 남긴다 [근거: src/ship_direction/ship_direction/ship_direction.py:43-50] [근거: src/ship_direction/ship_direction/ship_direction.py:786-788]. 먼 쪽 거리로 재면 실제보다 넓게 보이므로 가까운 쪽을 쓴다 [근거: src/ship_direction/ship_direction/ship_direction.py:39-40]. `(e − s)` 식은 시뮬 튜닝 기준이라 유지한다 [근거: docs/전달용/회신_rplidar_회피_수정제안.md:59-61].
- 후보 선택: 목표 방위 `yaw_mapped` 를 각 구간 [zone_min, zone_max] 로 clamp 한 각을 후보로 삼아, 목표와 차이가 가장 작고, 같으면 호 길이가 긴 후보를 낸다 [근거: src/ship_direction/ship_direction/ship_direction.py:791-808] [근거: src/ship_direction/ship_direction/ship_direction.py:822-823]. 구간 중심이 아니라 구간 안에서 목표에 가장 가까운 각이다. 배 폭 여유는 팽창이 이미 뺐다.
- 경로 없음: 260°(후진)를 0.5 s 내고 3.0 s STOP_HOLD 로 재탐색한 뒤, 그래도 없으면 반복한다 [근거: src/ship_direction/ship_direction/ship_direction.py:471-504]. 뒤쪽은 보지 못하므로 계속 후진하지 않는다 [근거: src/ship_direction/ship_direction/ship_direction.py:813-816].
- 변화율 제한: 0~160° 정상 조향값끼리는 한 틱에 15° 까지만 바꾸고 특수 신호는 즉시 통과시킨다 [근거: src/ship_direction/ship_direction/ship_direction.py:438-466].

| 파라미터 | 값 | 근거 |
|---|---|---|
| `half_width` | 0.29 m | A배 줄자 실측 폭 0.57 m ÷ 2 = 0.285 를 올림(스윕 격자 폭 0.58 과 일치). B배 0.535 m → 0.2675 라 공용 [근거: src/ship_direction/config/ship_direction.yaml:61-64] |
| `clearance` | 0.25 m | half_width 0.29 기준 시드 20 스윕 최적 (Ⅲ-6.1) [근거: src/ship_direction/config/ship_direction.yaml:71] |
| `min_required_width` | 0.83 m | 0.29×2 + 0.25 (계산) [근거: src/ship_direction/ship_direction/ship_direction.py:786] |
| `detection_distance` 기본 / 게이트 | 3.0 / 2.0 m | 시뮬: 1.8 이면 8/8 충돌, 게이트는 3.0 이면 접촉 증가 [근거: src/ship_direction/config/ship_direction.yaml:57-58] |
| `min_obstacle_cells` | 1 | 3 이면 작은 부표 무시. 두 배 공용 [근거: src/ship_direction/config/ship_direction.yaml:75] |
| temporal 투표 | OFF (1 / 2) | 시드 20: OFF 접촉 19/20, ON 20/20. 무해하지만 무익 [근거: src/ship_direction/config/ship_direction.yaml:47-53] |
| `rear_obstacle_ignore_margin` | 제거 | 부표 통째 삭제 결함 [근거: src/ship_direction/config/ship_direction.yaml:81-82] |
| 후진 / 재탐색 | 0.5 / 3.0 s | [근거: src/ship_direction/config/ship_direction.yaml:77-78] |
| `rate_limit_max_deg_per_tick` | 15° (10 Hz 에서 150°/s) | 실측 후 튜닝 [근거: src/ship_direction/config/ship_direction.yaml:80] |

시간 투표(`TemporalVote`)는 최근 N 프레임 중 다수에 나타난 셀만 장애물로 인정한다. 단일 시드에서는 효과가 있어 보였으나 시드 20 에서 차이가 없어 기본 OFF 로 두고 코드만 남겼다. 켤 때는 팽창 전 원본 마스크에 건다 [근거: src/ship_direction/ship_direction/ship_direction.py:118-126]. 벤치에서 경로 없음 상태로 후진 0.50 s, 쿨다운 3.10 s 후 재후진을 확인했다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:108-114].

그림: 갭 선택 개념도 — (a) 정면 0~160° 스캔 점, (b) `detection_distance` 이진 마스크, (c) 스파이크 정리 후 `atan((half_width+clearance)/r)` 만큼 팽창, (d) 빈 구간의 `r_edge`·호 길이 폭 판정(통과/탈락), (e) 목표 방위를 구간 안으로 clamp 해 고른 조향각. 다섯 단을 가로로 그린다.

## 5) 페일세이프 — 센서 감시
`SensorWatch` 가 `/scan` 과 `/yaw_error` 의 신선도로 레벨을 정하고, 가장 나쁜 센서가 레벨을 정한다 [근거: src/ship_direction/ship_direction/ship_direction.py:303-308] [근거: src/ship_direction/ship_direction/failsafe.py:114-127]. `/scan` 만 보면 IMU 사망 시 마지막 `/yaw_error` 로 계속 조향한다(시뮬: 정지율 0%, 접촉 3회, 14.8 m 주행) [근거: src/ship_direction/ship_direction/ship_direction.py:106-110].

| 레벨 | 진입 조건 | `/desired_angle` | 속도 |
|---|---|---|---|
| 0 정상 | — | 계산한 각도 | 100% |
| 1 경고 | 경과 > 0.7 s | 계산한 각도 그대로 | `motor_control` 상한 (Ⅲ-4.6) |
| 2 정지 | 경과 > 3.0 s 또는 이상 스캔 | STOP_HOLD 로 덮어씀 | 정지 |

[근거: src/ship_direction/config/ship_direction.yaml:14-21] [근거: src/ship_direction/ship_direction/ship_direction.py:526-547]

- 독립 타이머: 판정은 10 Hz `watchdog_cb` 에서 한다. 센서가 죽으면 콜백이 불리지 않아 콜백 안 판정은 영원히 실행되지 않는다 [근거: src/ship_direction/ship_direction/ship_direction.py:399-402].
- 오탐 방지: `time.monotonic()` [근거: src/ship_direction/ship_direction/ship_direction.py:352], 센서별 ARMED(첫 수신 전 감시 안 함) [근거: src/ship_direction/ship_direction/failsafe.py:116-117], 올릴 때 연속 3회 확인 [근거: src/ship_direction/ship_direction/failsafe.py:136-140], 내릴 때 즉시 자동 복구 [근거: src/ship_direction/ship_direction/failsafe.py:141-143]. 이상 스캔(하드 폴트)만 확인 없이 즉시 레벨 2 다 [근거: src/ship_direction/ship_direction/failsafe.py:133-135].
- 경고에서 각도를 두는 이유: 회피 중 직진을 강제하면 피하던 부표에 부딪힌다. 묵은 데이터라도 쓰고 속도를 줄인다 [근거: src/ship_direction/ship_direction/ship_direction.py:90-92]. 레벨을 셋으로 줄인 것은 새 스캔 없는 재계산과 마지막 값 고정이 같은 동작이기 때문이다 [근거: 커밋 2f4410a].
- 복구 시 노드 사망 수정: 한 줄에서 warn/info 를 번갈아 부르면 rclpy 가 `ValueError` 를 던져 레벨이 내려가는 순간 노드가 죽었다. 심각도별로 호출 지점을 나눴다 [근거: src/ship_direction/ship_direction/ship_direction.py:419-430] [근거: 커밋 aabb3bc].
- 정지 임계 3.0 s 는 10 Hz 기준 30스캔 연속 누락이다 [근거: src/ship_direction/config/ship_direction.yaml:20]. 실제 회전수(약 11 Hz)로는 약 33스캔이다(계산).
- 벤치(2026-08-06, 배 장착 전): LiDAR 드라이버 종료 후 0.70 s 에 레벨 1, 2.99 s 에 레벨 2, 재기동 즉시 레벨 0 이었고 그동안 `/desired_angle` 은 20 s 에 200회 계속 나갔다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:117-125]. IMU 를 끄면 `/scan` 이 정상이어도 레벨 1·2 로 올랐다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:127-135]. 14.3분 기록에서 의도적으로 끊은 구간 외 오탐은 0건이었다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:194-198]. 물 위 확인은 ____ (부록 B).

명령 워치독·펌웨어 워치독·비상정지는 Ⅲ-5 팀장 담당 부분에 쓴다. TTC 비상제동은 `motor_control` 단계에서 폐기했다(Ⅲ-4.6) [근거: docs/기준/CLAUDE_상세.md:193-196].

그림: 페일세이프 상태도 — 레벨 0(정상) →[경과 > 0.7 s, 3회 연속]→ 레벨 1(각도 유지·감속) →[경과 > 3.0 s, 3회 연속]→ 레벨 2(STOP_HOLD). 이상 스캔은 어느 상태에서든 레벨 2 로 바로 가는 화살표, 새 데이터 수신 시 레벨 0 으로 즉시 돌아오는 복구 화살표. 옆에 감시 대상 `/scan`·`/yaw_error` 와 "가장 나쁜 센서" 를 적는다.

## 6.1) 시뮬레이션 스윕
`tools/sim` 은 `ship_direction`·`motor_control` 로직 재구현, 가짜 LiDAR 부표 월드, 2025 규정 70×20 m 전 코스로 이뤄진다 [근거: tools/sim/README.md:7-9]. 스윕은 폭 5 × clearance 6(0.15~0.45) × 탐지 거리 2(2.0/3.0 m) × 게이트 폭 2(2/3 m) × 시드 20 = 2,400런이다 [근거: tools/sim/sweep_s1.py:63-71] [근거: docs/문제와작업/S1_half_width_스윕/README.md:62]. 폭마다 정체(구간 미도달) 런 비율이 가장 낮고 그다음 접촉이 적은 clearance 를 골랐다 [근거: tools/sim/sweep_s1.py:92-94].

| 선폭 [m] | half_width | 최적 clearance | 접촉 평균 | 정체 |
|---|---|---|---|---|
| 0.43 | 0.215 | 0.35 | 0.36 | 0% |
| 0.50 | 0.25 | 0.30 | 0.85 | 0% |
| 0.58 | 0.29 | 0.25 | 1.32 | 0% |
| 0.64 | 0.32 | 0.25 | 1.44 | 0% |
| 0.70 | 0.35 | 0.20 | 2.23 | 0% |

[근거: docs/문제와작업/S1_half_width_스윕/README.md:64-70] (`tools/sim/out/sweep_s1.csv` 2,400행 재집계로 일치 확인)

- half_width 0.29 에서 clearance 별 접촉은 0.20→1.95, 0.25→1.32, 0.30→1.46(정체 2%), 0.35→1.71(정체 28%), 0.45→0.45(정체 50%)다. 0.45 는 접촉이 가장 적지만 절반이 정체라 고르지 않았다 [근거: src/ship_direction/config/ship_direction.yaml:71] [근거: docs/기준/CLAUDE_상세.md:853].
- 작년 half_width 0.45 는 clearance 0.25 와 함께 1.15 m 갭을 요구해, 시드 1개 시험에서 접촉 3.0회·정체 1.5회였다 [근거: docs/문제와작업/S1_half_width_스윕/README.md:5] [근거: docs/문제와작업/S1_half_width_스윕/README.md:25]. 최적점의 갭 요구는 0.78~0.90 m 에 모인다 [근거: docs/문제와작업/S1_half_width_스윕/README.md:72].
- B배(제어 0.29, 접촉 반경 0.2675, 시드 20·80런)는 접촉 0.95회, 정체 0 이라 공용값을 쓴다 [근거: src/ship_direction/config/ship_direction.yaml:64-65].
- 한계: 접촉 판정이 반경 half_width 원이라 측면 접촉을 못 잡고, 동역학은 가정값이며, 시간 투표·지오펜스 병합은 시뮬에 없다 [근거: tools/sim/README.md:13-15]. 선회 중 유효 폭 W·cosθ + L·sinθ 는 A배(W 0.58, L 1.66)에서 10° 일 때 0.86 m, 15° 일 때 0.99 m 다 [근거: docs/문제와작업/S1_half_width_스윕/README.md:76-78]. 갭 요구 0.83 m 는 약 9° 이내 정렬일 때만 맞고, 1 m 안팎 틈에 비스듬히 들어가면 측면 접촉이 날 수 있어 수조 게이트 시험으로 확인한다 [근거: docs/문제와작업/S1_half_width_스윕/README.md:81].

## 부록 A. A배·B배 차이
| 항목 | A배 (1번배) | B배 (2번배) | 근거 |
|---|---|---|---|
| LiDAR 개체 | 80BF, FW 1.32, 약 11.1 Hz / 0.250° | 918A, FW 1.25, 11.00 Hz / 0.223° | [근거: docs/기준/CLAUDE_상세.md:136-138] [근거: docs/문제와작업/라이다_고장진단_20260818.md:103] |
| udev | 유일 시리얼 어댑터 → 시리얼 규칙 | 라이다·IMU 어댑터 시리얼이 둘 다 `0001` → `KERNELS` 포트 고정(꽂는 구멍이 곧 규칙) | [근거: docs/절차/2번배_udev_규칙_초안.md:3-5] [근거: docs/절차/2번배_udev_규칙_초안.md:24-27] |
| 선폭 실측 → half | 0.57 m → 0.285 | 0.535 m → 0.2675 | [근거: src/ship_direction/config/ship_direction.yaml:61] [근거: src/ship_direction/config/ship_direction.yaml:64] |
| `half_width` / `clearance` | 0.29 / 0.25 (공용) | 0.29 / 0.25 (공용) | [근거: src/ship_direction/config/ship_direction.yaml:61-71] |
| 셀 수 기반 값 | 0.250° 셀 | 0.223° 셀 — 같은 셀 수가 더 좁은 각 | [근거: CLAUDE.md:68] |

코드와 yaml 은 두 배가 같고 배 ID 분기는 없다.

## 부록 B. 확인 필요 목록
- LiDAR 정면(80°)–선수 정렬, 배별 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:549-551].
- LiDAR 장착 높이가 부표 높이와 맞는지 [근거: CLAUDE.md:224].
- 부표 탐지 최대거리(`obstacle_min_dist`, 과제 D4) — 물 위, A배 ____ / B배 ____.
- 물보라 기각 수 `/obstacle_reject_count` ____ 회/분 — 야외 rosbag. 군집이면 커널 7/9 검토.
- 1 m 안팎 게이트 통과 성공률 ____ — 수조.
- 물 위 페일세이프 전이·오탐 ____ — 지금은 벤치 결과만.
- B배 선폭 0.535 m 가 스러스터 포함인지 [근거: src/ship_direction/config/ship_direction.yaml:65].
- 스캔 모드 Sensitivity/Standard/Boost 수조 비교 ____ [근거: src/rplidar_ros/launch/rplidar_a3_launch.py:14-15].
- Ⅲ-1.3 용 장비 사진 배별 1장.

## 부록 C. 다른 담당과 겹치는 내용
- 지오펜스 광선 생성(`north_goal_angle`) → GPS. 이 절은 병합·하드 가드만.
- 감속비·페일세이프 속도 상한(`motor_control`), TTC 폐기, 명령 워치독·비상정지 → 팀장(Ⅲ-4.6, Ⅲ-5).
- `/yaw_error` 생성과 IMU stale 전파 → IMU·팀장. 이 절은 그 침묵을 감시하는 쪽만.
- 카메라 방위 + LiDAR 거리 조합 → 카메라. 거리는 LiDAR 전방 섹터 최소거리.
- 임무 노드의 `/scan` 사용(같은 `SCAN_QOS`) → 팀장.

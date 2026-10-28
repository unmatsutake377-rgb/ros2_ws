# IMU 초안
## 1.3) 센서 — 담당 줄
| 장비명 | 용도 |
|---|---|
| iAHRS RB-SDA-v1 9축 AHRS (A·B배 공통, `/dev/IMU`, 115200 bps) [근거: CLAUDE.md:59] [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:29-30] | 선수 방위(yaw). `iahrs_driver` 가 보정하지 않은 `/imu/yaw_raw` 를 100 Hz 로 내고, `yaw_mux` 가 장착 오프셋을 보정해 `/imu/yaw` 를 단독 발행한다. 목표 방위 오차(`/yaw_error`)와 지오펜스 방향의 기준값이다 [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:213] [근거: src/launch_files/launch/launch_files.launch.py:55-80] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:35] |

그림: (Ⅲ-1.3 장비 사진) iAHRS RB-SDA-v1 을 마스트 베이스 포켓에 고정한 모습. 포켓 바닥의 뱃머리 방향 화살표와 보드의 X축 화살표가 함께 보이게 찍는다 [근거: docs/전달용/하드웨어_배치_요구사항.md:53]. 사진은 ____ (조립 완료 후 촬영).

## 2.1) 좌표계·헤딩
헤딩은 웨이포인트 목표 방위와 빼서 조향 오차를 만드는 기준값이고, 지오펜스 경계를 가짜 LiDAR 광선으로 바꿀 때 광선 방향도 이 값으로 정해진다 [근거: src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py:112] [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:35]. 올해는 IMU 드라이버가 보정 없는 원시 yaw 를 `/imu/yaw_raw` 로만 내고, 새로 만든 `ssf_heading` 패키지의 `yaw_mux` 노드가 이를 보정해 `/imu/yaw` 를 단독으로 발행하도록 구조를 바꿨다 [근거: src/launch_files/launch/launch_files.launch.py:49-80] [근거: src/ssf_heading/ssf_heading/yaw_mux.py:1-14].

각도 규약은 전 구간 같다. 단위는 도(°), 범위 0~360, 0° = 진북, 시계방향 증가(나침반 규약)다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:20-21]. 센서 원본 yaw 는 반시계 증가이므로 드라이버가 시계방향으로 뒤집어 내고 [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:231-232], `invert_yaw` 는 그 부호가 틀렸을 때만 쓰는 예비 스위치다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:22-23]. 헤딩을 믿을 수 없으면 0 같은 대체값을 내지 않고 발행을 멈추며, 소비 노드가 두절을 감지해 안전 동작으로 넘어간다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:55-58].

### 2.1.1) 장비 사양과 선정 이유
| 항목 | A배 | B배 | 출처 |
|---|---|---|---|
| 모델 | iAHRS RB-SDA-v1 | iAHRS RB-SDA-v1 | CLAUDE.md:59, docs/전달용/보고서_분담/01_IMU.md:76 |
| 센서 구성 | 9축 AHRS | 9축 AHRS | CLAUDE.md:59 |
| 연결 | USB 시리얼, udev 고정 이름 `/dev/IMU` | 동일 | src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:29 |
| 통신 속도 | 115200 bps | 115200 bps | src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:30 |
| 사용 명령·출력 | ASCII `e` → roll, pitch, yaw [deg] | 동일 | src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:222, docs/문제와작업/N4_mag_heading_B_준비.md:66 |
| 드라이버 질의 주기 | 100 Hz | 100 Hz | src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:213 |
| 방위 정확도 (정적/동적) | ____ | ____ | 매뉴얼 p3 사양 (저장소에 수치 없음, docs/전달용/케이블_출구_맵.md:32) |

선정 이유. 9축이라 지자기를 포함해 센서 자체로 방위를 낸다 [근거: CLAUDE.md:59] [근거: docs/문제와작업/최종보고_피드백답변_시뮬결과.md:165]. 작년 최종본에 이미 이 모델용 드라이버가 있어 프로토콜을 새로 만들 필요가 없었다 [근거: 커밋 9bff661, src/iahrs_driver_ros2-main] [근거: docs/문제와작업/최종_문제추적표_전체정리.md:118]. 한 글자 ASCII 명령으로 Euler 각을 바로 받으므로 드라이버 수정 범위가 출력 토픽 쪽으로 한정된다 [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:222].

### 2.1.2) 코드 출처
| 패키지 | 원래 출처 | 작년부터 | 올해 바꾼 것 | 근거 커밋 |
|---|---|---|---|---|
| `iahrs_driver` | 외부 iAHRS 드라이버. `package.xml` 관리자 `wbjin`, README 설명자료 `blog.naver.com/zzang0736` (원출처 ____) [근거: src/iahrs_driver_ros2-main/iahrs_driver/package.xml:7] [근거: src/iahrs_driver_ros2-main/README.md:48] | 있음 | 발행 토픽을 파라미터 `yaw_topic`(기본 `/imu/yaw_raw`)로 변경, 부팅 0점화(`zero_yaw_on_boot`)·GPS 방위 덮어쓰기(`use_gps_heading_override`)를 파라미터로 빼고 기본 false [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:75-78]. `/imu/mag_heading` 추가 [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:85]. NAV-PVT 구독 토픽명을 `/ublox_gps_node/navpvt` 로 수정 [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:103-104] | `ace1e33`, `b433662`, `ed2a144`, `74b7d4b`(CMake `std_msgs` 의존 추가) |
| `ship_goal_angle` | 작년 우리 코드 | 있음 | 발행 주기 파라미터화(0.5 s → 0.05 s), 시작값 0.0 → None, IMU·목표각 stale 시 발행 중단 [근거: src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py:18-27] | `026482a`, `100104e`, `2a65fda` |
| `ssf_heading` | 올해 새로 만듦 (`heading_logic.py`, `yaw_mux.py`, 테스트) | 없음 | 전부 신규. `/imu/yaw` 단독 발행, 소스 선택, COG 오프셋 추정, 후진 게이트 | `ace1e33`, `5a69c0a`, `0b00bd8`, `fb42b43` |

### 2.1.3) 데이터 흐름
| 토픽 | 입력·출력 | 메시지 타입 | 주기 | QoS | 상대 노드 |
|---|---|---|---|---|---|
| `/imu/yaw_raw` | `iahrs_driver` 출력 → `yaw_mux` 입력 | `std_msgs/Float64` | 100 Hz [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:213] | 발행 depth 10, 구독 RELIABLE depth 2 [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:82] [근거: src/ssf_heading/ssf_heading/yaw_mux.py:120-121] | `iahrs_driver` → `yaw_mux`, `healthcheck`, `blackbox` |
| `/imu/mag_heading` | `iahrs_driver` 출력 (관찰용) | `std_msgs/Float64` | 100 Hz (같은 루프) [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:237-239] | depth 10 [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:85] | `blackbox` 만 구독 [근거: src/ssf_tools/ssf_tools/blackbox.py:192] |
| `/ublox_gps_node/fix_velocity` | `yaw_mux` 입력 (COG 계산) | `geometry_msgs/TwistWithCovarianceStamped` | `gps_rate_hz` 5.0 가정 [근거: src/ssf_heading/config/ssf_heading.yaml:78] | RELIABLE depth 2 [근거: src/ssf_heading/ssf_heading/yaw_mux.py:124-125] | `ublox_gps_node` (GPS 담당) |
| `/motor_reverse` | `yaw_mux` 입력 (후진 게이트) | `std_msgs/Bool` | 0.05 s 타이머 [근거: src/motor_control/motor_control/motor_control.py:90] | RELIABLE depth 2 [근거: src/ssf_heading/ssf_heading/yaw_mux.py:126-128] | `motor_control` |
| `/imu/yaw` | `yaw_mux` 출력 | `std_msgs/Float64` | 50 Hz (`period_sec` 0.02) [근거: src/ssf_heading/config/ssf_heading.yaml:16] | RELIABLE depth 2 [근거: src/ssf_heading/ssf_heading/yaw_mux.py:120,129] | `ship_goal_angle`, `north_goal_angle`, `ship_turn` [근거: CLAUDE.md:87] |
| `/heading_status` | `yaw_mux` 출력 (진단) | `std_msgs/String` | 상태 변화 시 또는 1 s 마다 [근거: src/ssf_heading/ssf_heading/yaw_mux.py:185-190] | RELIABLE depth 2 [근거: src/ssf_heading/ssf_heading/yaw_mux.py:130] | `healthcheck` [근거: src/ssf_tools/ssf_tools/healthcheck.py:105] |
| `/yaw_error` | `ship_goal_angle` 출력 | `std_msgs/Float32` | 20 Hz (`period_sec` 0.05) [근거: src/ship_goal_angle/config/ship_goal_angle.yaml:4] | RELIABLE depth 2 [근거: src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py:42,47] | `ship_direction` [근거: src/ship_direction/ship_direction/ship_direction.py:263] |

`/imu/yaw` 발행자는 `yaw_mux` 하나여야 한다 [근거: CLAUDE.md:87]. 드라이버가 `/imu/yaw` 를 계속 내는 채로 `yaw_mux` 를 띄우면 한 토픽에 발행자가 둘이 되어 보정 전·후 값이 번갈아 나오고, ROS 는 이를 오류로 알리지 않는다 [근거: src/ssf_heading/ssf_heading/yaw_mux.py:10-14]. 이를 막으려고 드라이버 `yaw_topic` 기본값을 `/imu/yaw_raw` 로 두고 [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:78], 대회 launch 에도 같은 값을 명시했으며 [근거: src/launch_files/launch/launch_files.launch.py:61-63], 드라이버가 `/imu/yaw` 로 설정되면 기동 시 경고를 낸다 [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:92-96].

그림: (Ⅲ-2.1 헤딩 경로 비교) 왼쪽 2025: `iahrs_driver`(부팅 0점화 + NAV-PVT 방위 덮어쓰기) → `/imu/yaw` → `ship_goal_angle`(0.5 s) → `/yaw_error`. 오른쪽 2026: `iahrs_driver` → `/imu/yaw_raw` → `yaw_mux`(← `/ublox_gps_node/fix_velocity`, `/motor_reverse`) → `/imu/yaw` → `ship_goal_angle`(20 Hz) → `/yaw_error` → `ship_direction`. `yaw_mux` 에서 `/heading_status` → `healthcheck`, `/imu/mag_heading` → `blackbox` 를 점선(관찰용)으로 표시한다.

### 2.1.4) 처리 알고리즘
**보정식.** `yaw_mux` 는 선택한 소스의 원시 방위 ψ_raw 에 다음을 적용한다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:92-93].

ψ = wrap360( s·ψ_raw + mount_offset_deg + δ ), s = −1 (invert_yaw) 또는 +1

적용 순서는 부호 반전 → 장착 오프셋 → 자기편각이다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:87-90]. 편각 δ 는 자기북 기준 소스에만 넣고 나머지는 0 으로 둔다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:397-404]. 듀얼 GPS(relPosNED)와 COG 는 이미 진북 기준이라 편각을 더하면 한국 기준 약 8° 이중 보정이 된다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:39-46].

**소스 선택(`heading_source`).** 네 가지가 있고 기본값은 `imu_relative` 다 [근거: src/ssf_heading/config/ssf_heading.yaml:14].
- `imu_relative`: `/imu/yaw_raw` 에 고정 `mount_offset_deg` 를 더한다. 기준축이 임의라 편각은 쓰지 않고 오프셋이 전부 흡수한다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:32,42].
- `cog_offset`: IMU 를 본체로 쓰고 GPS COG 로 추정한 오프셋만 더한다. 정지 중에도 IMU 가 헤딩을 유지한다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:373-384].
- `dual_gps`, `imu_absolute`: 로직에 입력 칸은 있으나 `yaw_mux` 가 해당 토픽을 구독하지 않아 현재는 항상 `NOT_IMPLEMENTED` 로 발행을 멈춘다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:361-371] [근거: src/ssf_heading/ssf_heading/yaw_mux.py:121-128].

**COG 오프셋 추정(`cog_offset`).** GPS 속도벡터에서 COG = atan2(v_E, v_N), 속력 = √(v_E²+v_N²) 를 구한다(`gps_vel_frame` enu 기준) [근거: src/ssf_heading/ssf_heading/heading_logic.py:107-118]. 조건을 통과한 표본마다 e_i = COG − ψ_imu 를 단위벡터 (cos e_i, sin e_i) 로 누적하고, 기존 합은 w = 0.5^(Δt / `cog_half_life_sec`) 로 감쇠시켜 IMU 드리프트를 따라간다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:221-238]. 신뢰도 R = ‖Σ‖ / n (0~1), 오프셋 = atan2(Σsin, Σcos) 이고, n ≥ `cog_min_samples` 이며 R ≥ `cog_min_resultant` 일 때만 수렴으로 인정한다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:246-265]. 산술평균을 쓰지 않는 이유는 359° 와 1° 의 산술평균이 정반대인 180° 가 되기 때문이다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:157-158]. 표본 배제 조건은 다음과 같다.
- `cog_min_speed_mps` 미만: 저속 COG 는 잡음이다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:212-215].
- 선회율이 `cog_max_turn_rate_dps` 초과: 선회 중 COG 가 뱃머리를 못 따라온다. 선회율은 0/360 경계를 넘을 때도 최단 부호차로 미분한다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:216-219,311-313].
- IMU 가 `stale_sec` 넘게 묵음: 오래된 yaw 와 지금 COG 를 짝지으면 그 차이가 오차가 된다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:323-328].
- 후진 중, 또는 `cog_require_reverse_gate` 가 true 인데 `/motor_reverse` 가 없거나 묵음 [근거: src/ssf_heading/ssf_heading/heading_logic.py:203-211]. 후진만 계속되면 표본이 서로 일치한 채 180° 틀려 R = 1.0 으로 수렴하므로 R 로는 걸러지지 않는다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:164-170].

감쇠가 있으면 표본 수에 상한 n_max = 1 / (1 − 0.5^(dt/T½)) 이 생겨, `cog_min_samples` 를 그보다 크게 잡으면 오류 없이 영원히 수렴하지 않는다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:125-130]. 현재값(5 Hz, 60 s)의 n_max 는 약 433 으로 30 보다 충분히 크고 [근거: src/ssf_heading/config/ssf_heading.yaml:79], `yaw_mux` 가 기동 시 이 조합을 검사한다 [근거: src/ssf_heading/ssf_heading/yaw_mux.py:102-108]. 이 추정기는 원리상 장착 오프셋과 그날의 평균 게걸음각을 함께 잰다. 조류·바람으로 게걸음하면 그 각도가 오프셋에 흡수된다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:150-155].

**방위 오차(`ship_goal_angle`).** `/north_goal_angle_tp` 목표 방위에서 `/imu/yaw` 를 빼 0~360 으로 접은 값을 `/yaw_error` 로 낸다 [근거: src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py:77,112-113]. 두 입력이 모두 한 번 이상 들어오고 각자 신선할 때만 발행한다 [근거: src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py:84-110].

### 2.1.5) 주요 파라미터
| 이름 | 값 | 위치(파일:행) | 정한 근거 |
|---|---|---|---|
| `heading_source` | `"imu_relative"` | src/ssf_heading/config/ssf_heading.yaml:14 | 입력이 배선되고 수렴 조건이 없는 소스. 잘못된 이름은 기동 시 예외 [근거: src/ssf_heading/ssf_heading/yaw_mux.py:44-48] |
| `period_sec` | 0.02 (50 Hz) | src/ssf_heading/config/ssf_heading.yaml:16 | 드라이버 100 Hz 와 소비자 20 Hz 사이 [근거: src/ssf_heading/config/ssf_heading.yaml:16] |
| `stale_sec` | 0.5 s | src/ssf_heading/config/ssf_heading.yaml:17 | 소비자 `ship_goal_angle`·`north_goal_angle` 의 IMU stale 한계와 같은 값 [근거: src/ship_goal_angle/config/ship_goal_angle.yaml:6] [근거: src/north_goal_angle/config/north_goal_angle.yaml:33] |
| `invert_yaw` | false | src/ssf_heading/config/ssf_heading.yaml:26 | 장착 전 손 시험에서 정북→정동 회전 시 값 증가 확인. 장착 후 재확인 필요 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:485-492] |
| `mount_offset_deg` | 0.0 | src/ssf_heading/config/ssf_heading.yaml:27 | 미측정 표시. 조립 완료 후 정북 정렬로 정한다 [근거: src/ssf_heading/config/ssf_heading.yaml:20-25] |
| `magnetic_declination_deg` | 0.0 | src/ssf_heading/config/ssf_heading.yaml:33 | `imu_absolute` 에만 적용, 다른 소스에선 무시 [근거: src/ssf_heading/config/ssf_heading.yaml:30-32] |
| `cog_min_speed_mps` | 0.8 m/s | src/ssf_heading/config/ssf_heading.yaml:47 | 저속 COG 배제. 보행 0.70 m/s 시험에서 `rej=slow` 로 걸러짐 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:498] |
| `cog_max_turn_rate_dps` | 8.0 °/s | src/ssf_heading/config/ssf_heading.yaml:49 | 선회 중 표본 배제 [근거: src/ssf_heading/config/ssf_heading.yaml:49] |
| `cog_min_samples` / `cog_min_resultant` | 30 / 0.9 | src/ssf_heading/config/ssf_heading.yaml:50-51 | 수렴 판정. n_max ≈ 433 대비 여유 [근거: src/ssf_heading/config/ssf_heading.yaml:79] |
| `cog_half_life_sec` | 60.0 s | src/ssf_heading/config/ssf_heading.yaml:52 | IMU 드리프트 추종 [근거: src/ssf_heading/config/ssf_heading.yaml:52] |
| `cog_require_reverse_gate` | true | src/ssf_heading/config/ssf_heading.yaml:72 | 대회값 고정 안전 스위치 [근거: CLAUDE.md:210] |
| `gps_vel_frame` | `"enu"` | src/ssf_heading/config/ssf_heading.yaml:99 | 손 시험 북·동 두 방향 offset 차 10.7° 로 프레임 오류(약 90°/180°) 아님 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:505-508] |
| 드라이버 `yaw_topic` / `zero_yaw_on_boot` / `use_gps_heading_override` | `/imu/yaw_raw` / false / false | src/launch_files/launch/launch_files.launch.py:61-63 | 단독 발행자 유지, 작년 ②③ 동작 끔 [근거: src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:75-78] |
| `ship_goal_angle` `period_sec` / `stale_sec` / `goal_stale_sec` | 0.05 / 0.5 / 2.0 s | src/ship_goal_angle/config/ship_goal_angle.yaml:4,6,8 | 목표각은 0.5 s 주기 발행이라 4배 여유로 거짓 정지 방지 [근거: src/ship_goal_angle/config/ship_goal_angle.yaml:8-10] |

### 2.1.6) 이상 상황 대응
| 상황 | 감지 방법 | 코드의 동작 | 근거 |
|---|---|---|---|
| 원시 yaw 두절 (드라이버 종료, USB 분리) | 마지막 수신 후 `stale_sec` 0.5 s 초과 | `STALE`, `/imu/yaw` 발행 중단, 2 s 간격 경고 로그 | src/ssf_heading/ssf_heading/heading_logic.py:389-395, src/ssf_heading/ssf_heading/yaw_mux.py:164-171 |
| 기동 후 아직 미수신 | 수신 시각 없음 | `NO_DATA`, 발행 안 함 | src/ssf_heading/ssf_heading/heading_logic.py:391-392 |
| NaN·inf 입력 | `is_finite` 검사 | 해당 값 무시 | src/ssf_heading/ssf_heading/heading_logic.py:77-79,306-308 |
| 미배선 소스 선택 (`dual_gps`, `imu_absolute`) | 입력 칸 비어 있음 | `NOT_IMPLEMENTED`, 0 대신 발행 정지 | src/ssf_heading/ssf_heading/heading_logic.py:361-371 |
| `heading_source` 오타 | 허용 목록 대조 | 기동 시 `RuntimeError` (기본값으로 떨어지지 않음) | src/ssf_heading/ssf_heading/yaw_mux.py:44-48 |
| `cog_offset` 수렴 전 | 오프셋 None | `NOT_CONVERGED`, 발행 안 함 | src/ssf_heading/ssf_heading/heading_logic.py:380-383 |
| 후진 상태 모름 (`motor_control` 미기동) | `/motor_reverse` 없음 또는 stale | 표본 수집 정지, `/heading_status` 에 `rej=no_reverse_gate`, 10 s 간격 경고 | src/ssf_heading/ssf_heading/heading_logic.py:207-211, src/ssf_heading/ssf_heading/yaw_mux.py:175-181 |
| 후진 게이트를 끈 채 출발 | 설정값 | 기동 시 ERROR 로그, 상태 문자열 `gate=off`, `cog_offset` 일 때 `healthcheck` 가 `/health_ok=false` | src/ssf_heading/ssf_heading/yaw_mux.py:114-118, src/ssf_tools/ssf_tools/healthcheck.py:275-279 |
| 드라이버가 `/imu/yaw` 로 설정됨 | 기동 시 토픽명 비교 | WARN 로그 | src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:92-96 |

`/imu/yaw` 가 끊긴 뒤의 반응은 다음과 같다. `ship_goal_angle` 은 IMU 입력이 0.5 s 넘게 묵으면 `/yaw_error` 발행을 멈춘다 [근거: src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py:95-101]. `ship_direction` 은 `/yaw_error` 수신을 IMU 체인 생존 신호로 보고 두절 시간에 따라 감속·정지한다 [근거: src/ship_direction/ship_direction/ship_direction.py:182-183,357]. 지오펜스는 `/imu/yaw` 가 0.5 s 넘게 묵으면 빈 배열을 낸다 [근거: src/north_goal_angle/north_goal_angle/north_goal_angle.py:294-305]. `healthcheck` 는 `/imu/yaw_raw` 를 함께 보아 원인이 IMU 인지 `yaw_mux` 인지를 로그로 구분하고, 이 판정은 `/health_ok` 에 넣지 않는다 [근거: src/ssf_tools/ssf_tools/healthcheck.py:207-222].

### 2.1.7) 2025 대비 개선점
| 작년 | 문제 | 올해 | 근거 커밋 |
|---|---|---|---|
| 드라이버가 부팅 시점 방위를 0 으로 빼서 `/imu/yaw` 직접 발행 [근거: 커밋 9bff661, src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:59,190-196] | 상대각이라 절대방위인 목표 방위와 빼면 의미가 없음 [근거: src/ssf_heading/ssf_heading/heading_logic.py:6,9-10] | 드라이버는 `/imu/yaw_raw` 만, `zero_yaw_on_boot` 기본 false. `/imu/yaw` 는 `yaw_mux` 단독 | `ace1e33` |
| NAV-PVT 방위가 유효하면 IMU 값을 통째로 덮어씀, `head_acc` 30° 만 검사 [근거: 커밋 9bff661, src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:44,70-71,202-204] | NAV-PVT 방위는 COG 라 뱃머리가 아니고, 속도 0 에서도 통과해 정지 시 잡음 [근거: src/ssf_heading/ssf_heading/heading_logic.py:7,11-15] | `use_gps_heading_override` 기본 false. COG 는 속도·선회율·후진 조건을 통과할 때만 오프셋 추정에 사용 | `ace1e33`, `5a69c0a`, `0b00bd8`, `fb42b43` |
| NAV-PVT 구독 토픽 `/ublox/navpvt` [근거: 커밋 9bff661, src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp:63] | ublox 발행명과 달라 GPS 방위가 들어오지 않음 | `/ublox_gps_node/navpvt` 로 수정 | `ed2a144` |
| 소스 선택·편각 처리 없음 | 장착 오프셋·편각을 넣을 곳이 없음 | `heading_source` 4종, 소스별 편각 적용표 `SOURCE_IS_MAGNETIC`, `/heading_status` 진단 | `ace1e33` |
| `ship_goal_angle` 0.5 s 타이머, 시작값 0.0, stale 검사 없음 [근거: 커밋 9bff661, src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py:27-28,63] | 부팅 직후 `yaw_error=0.0` 을 내 정렬 완료로 오인, 입력이 죽어도 마지막 값을 계속 발행 [근거: src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py:23-26] | 시작값 None, IMU 0.5 s·목표각 2.0 s stale 시 발행 중단, 주기 0.05 s 파라미터화(주기 자체는 성능 개선 아님) [근거: src/ship_goal_angle/ship_goal_angle/ship_goal_angle.py:19-22] | `026482a`, `100104e`, `2a65fda` |
| 없음 | 지자기 방위를 따로 볼 수 없음 | `/imu/mag_heading` 관찰용 발행 (제어 미사용) | `b433662` |

### 2.1.8) 검증과 시험 결과
`tools/run_tests.sh ssf_heading` 으로 로직 테스트 53개가 모두 통과했다(2026-10-28 실행). ROS 없이 시간을 주입해 돌린다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:2].

| 테스트 파일 | 개수 | 검증 내용 |
|---|---|---|
| src/ssf_heading/test/test_heading_logic.py (42~97행) | 8 | wrap360·wrap180·최단 부호차, NaN 차단, 보정 순서(반전 → 오프셋 → 편각) |
| 같은 파일 (98~122행) | 2 | 편각은 자기북 소스에만 적용, 모든 소스가 편각 표에 있음 |
| 같은 파일 (123~226행) | 11 | 미지 소스 예외, NO_DATA·STALE 경계와 복구, NaN 무시, 미배선 소스는 0 이 아닌 침묵, COG 가 `imu_relative` 출력에 섞이지 않음 |
| 같은 파일 (227~256행) | 4 | enu·ned COG 변환, 속도 0, 미지 프레임 예외 |
| 같은 파일 (257~362행) | 10 | 오프셋 복원, 0/360 경계, 잡음 허용, 저속·선회·NaN 배제, 수렴 전 None, 전후진 혼합 시 R 붕괴, 게걸음각 흡수, 감쇠 추종 |
| 같은 파일 (363~490행) | 11 | 게이트 없을 때 후진만으로 180° 오수렴 재현, 게이트 차단·모름 차단·해제·재개, 포화 상한 공식과 도달 불가 검출 |
| 같은 파일 (491행~) | 7 | 수렴 후 발행, 수렴 전 침묵, 정지 중 헤딩 유지, IMU 두절 시 STALE, IMU stale 표본 폐기, 선회율 0/360 경계, 다른 소스로 도는 중 추정 계속 |
| src/ship_goal_angle/test/ | 3 | lint 만(copyright·pep257 은 skip). 로직 단위 테스트 없음 [근거: src/ship_goal_angle/test/test_copyright.py:20] [근거: src/ship_goal_angle/test/test_pep257.py:34] |

배 장착 전 벤치·손 시험 결과(배 구분 없음)는 다음과 같다. 2026-08-06 실기 체인에서 `/imu/yaw_raw` 100.1 Hz, `/imu/yaw` 50.0 Hz, `/yaw_error` 20.0 Hz 로 설계 주기와 일치했다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:100-102]. 드라이버를 3.0 s 에 종료하자 `/imu/yaw` 가 3.3 s, `/yaw_error` 가 3.8 s 에 멈췄고 `ship_direction` 이 4.78 s 에 레벨 1, 7.08 s 에 레벨 2 로 올라갔다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:129-133]. `/imu/mag_heading` 은 `/imu/yaw_raw` 와 400 표본에서 평균 차 0.0015°, 최대 0.012° 로 같았다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:173-174]. 아래 표는 배 조립 후에만 의미가 있다. 가조립 상태 값은 버린다 [근거: docs/문제와작업/과제_목록_전체.md:58].

| 실측 항목 | A배 | B배 | 측정일 |
|---|---|---|---|
| `mount_offset_deg` 확정값 (°) | ____ | ____ | ____ |
| `invert_yaw` 확정값 (정북 → 시계 90°) | ____ | ____ | ____ |
| 재부팅 전후 `/imu/yaw_raw` 일치 여부 (정북 고정) | ____ | ____ | ____ |
| 대회장 자기편각 (°) | ____ | ____ | ____ |
| `gps_vel_frame` 판별: 북·동 직진 offset 차 (°) | ____ | ____ | ____ |
| 정지 중 헤딩 흔들림 (°) | ____ | ____ | ____ |
| 직진 중 헤딩 흔들림 (°) | ____ | ____ | ____ |
| 선회 중 헤딩 흔들림 (°) | ____ | ____ | ____ |

## 부록 A. A배·B배 차이
- IMU 모델과 코드는 같다 [근거: docs/전달용/보고서_분담/01_IMU.md:76]. 장착 오프셋과 주변 자기 환경은 배마다 달라 실측 표를 배별로 둔다 [근거: docs/전달용/보고서_분담/01_IMU.md:76].
- 대회 launch 의 배별 인자는 `gps_config` 하나뿐이고, `yaw_mux` 는 두 배 모두 같은 `ssf_heading.yaml` 을 읽는다 [근거: src/launch_files/launch/launch_files.launch.py:35-37,77-79]. 배별 `mount_offset_deg`·`invert_yaw` 를 저장소에서 나눠 둘 경로는 현재 없다 → 부록 B.

## 부록 B. 확인 필요 목록
- 방위 정확도(정적/동적) A·B: 매뉴얼 p3 사양 수치가 저장소에 없음 [근거: docs/전달용/케이블_출구_맵.md:32].
- `iahrs_driver` 정확한 원출처 (현재 근거는 `package.xml` 관리자와 README 링크뿐).
- Ⅲ-1.3 장비 사진 (조립 완료 후 촬영).
- 2.1.8 실측 표 8항목 × A·B·측정일 전부 (과제 C5 정북 정렬, D9 대회장 편각, 물 위 흔들림) [근거: docs/문제와작업/과제_목록_전체.md:58,75].
- 재부팅 전후 `/imu/yaw_raw` 가 다르면 `imu_relative` 전제가 깨지므로 `cog_offset` 전환을 검토한다 [근거: docs/문제와작업/과제_목록_전체.md:58].
- 정북 정렬 기준: 절차는 나침반으로 정북에 맞추라고 하는데 [근거: src/ssf_heading/config/ssf_heading.yaml:21], 자기 나침반은 자북을 가리키고 `imu_relative` 에서는 편각을 무시한다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:42]. 진북 기준으로 맞추려면 나침반 값에 편각을 반영해 `mount_offset_deg` 를 정해야 하는지 확인 필요(차이는 한국 기준 약 8° [근거: src/ssf_heading/ssf_heading/heading_logic.py:40]).
- 같은 이유로 과제 D9 의 `magnetic_declination_deg` 입력은 기본 소스 `imu_relative` 에서 효과가 없다 [근거: src/ssf_heading/config/ssf_heading.yaml:30-33]. 편각을 어디에 반영할지 정해야 한다.
- 배별 `mount_offset_deg` 저장 방식 (부록 A).
- 2026-08-07 손 시험의 정북 → 정동 변화량이 +137.77° 였다(기대 +90°). 문서는 손 회전 부정확으로 설명한다 [근거: docs/결과분석/실기통합시험_LiDAR_IMU_20260806.md:490,493]. 장착 후 90° 회전 시험으로 재확인한다.
- 지자기 벡터 크기 약 80 µT (지구 자기장 약 50 µT) 측정 이력이 있어 장착 후 지자기 보정(`cm`) 필요 여부 확인 [근거: docs/문제와작업/N4_mag_heading_B_준비.md:97-98].
- `ssf_heading.yaml` 주석이 `cog_offset` 을 "미구현", `imu_relative` 를 "유일하게 구현된 것" 으로 적고 있으나 [근거: src/ssf_heading/config/ssf_heading.yaml:9-10], `cog_offset` 은 `5a69c0a` 에서 구현됐다 [근거: src/ssf_heading/ssf_heading/heading_logic.py:373-384]. 주석 정정 필요(본문은 코드 기준으로 씀).
- `ship_goal_angle` 은 로직 단위 테스트가 없다.

## 부록 C. 다른 담당과 겹치는 내용
- GPS: COG 는 GPS 속도 토픽 `/ublox_gps_node/fix_velocity` 에서 받는다 [근거: src/ssf_heading/config/ssf_heading.yaml:84]. NAV-PVT 토픽명 수정(`ed2a144`)은 GPS 커밋에 함께 들어 있다. 지오펜스가 IMU stale 시 빈 배열을 내는 것은 반응만 적었다(2.1.6).
- LiDAR: `ship_direction` 은 `/yaw_error` 두절 시 감속·정지한다는 한 줄만 적었다(2.1.6). 페일세이프 레벨 판정은 LiDAR 절.
- 제어(`motor_control`): `/motor_reverse` 는 실제 출력 PWM 좌우 평균 > 1500 으로 판정한다 [근거: src/motor_control/motor_control/motor_control.py:162-164]. 판정 방식 설명은 제어 절.
- 하드웨어: IMU 장착 위치(마스트 베이스 포켓, 포켓 바닥 화살표 = 뱃머리)는 언급만 [근거: docs/전달용/하드웨어_배치_요구사항.md:53].
- 진단(`healthcheck`·`blackbox`): `/heading_status`, `/imu/yaw_raw`, `/imu/mag_heading` 을 관찰한다. 제어에는 쓰지 않는다 [근거: src/ssf_tools/ssf_tools/healthcheck.py:207] [근거: src/ssf_tools/ssf_tools/blackbox.py:131-132].

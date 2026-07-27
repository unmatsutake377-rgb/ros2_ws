# 자율운항선박 ROS2 워크스페이스 인수인계 컨텍스트 (SSF 2024)

> 이 문서는 다른 AI 채팅에 그대로 붙여넣어 맥락을 이어받게 하기 위한 것이다.
> 아래 지시문부터 읽고, 이후 사실 정보를 참조하라.

## AI에게 주는 지시
너는 대학 자율운항선박 팀의 작년(2024) 대회용 ROS2 코드베이스를 인수인계받는다.
이 코드는 **재사용 대상**이며, 팀은 올해 **새 선체(A배/B배 2척, 크기 상이)** 를 제작 중이다.
목표는 (1) 코드 전체 구조 이해, (2) 버그·위험요소 파악, (3) 새 하드웨어에 맞춘 이식/수정.
아래 "알려진 버그"는 이미 확인된 것이니 재발견하지 말고, 수정안·검증 위주로 진행하라.
확정 못 한 항목은 "미해결 질문"에 있으니 사용자에게 하드웨어 사실을 물어 확정하라.

---

## 1. 시스템 개요
- 플랫폼: ROS2 Humble, Ubuntu 22.04, Python 3.10.
- 워크스페이스: `ros2_ws/` (colcon, `src/`에 소스). 저장소명 `SSF2024SoftWare`.
- 데이터 흐름(항법 파이프라인):
  `GPS + 비전(도형각) → 목표방위(north_goal_angle) → IMU와 오차(ship_goal_angle) → LiDAR 장애물회피 융합(ship_direction) → 목표각(desired_angle) → 모터PWM(motor_control) → micro-ROS → Arduino Due → ESC 3개`
- 제어 컨벤션: **전방 = 80°** 기준. 각도 0~360, LiDAR 전방 섹터 0~160° 사용(중앙 80°). 모터는 좌/우 차동추진.

## 2. 패키지 인벤토리

### 팀 자체 작성 (핵심)
- **north_goal_angle** — GPS 웨이포인트 FSM. 목표 방위/거리 산출.
- **ship_goal_angle** — IMU yaw와 목표방위의 오차(`/yaw_error`) 계산.
- **ship_direction** — LiDAR 갭-팔로잉 장애물 회피 + 목표각 융합. (파일명 `ship_direction.py`)
- **motor_control** — 목표각 → 좌/우 ESC PWM 인코딩. (`motor_control copy.py`는 구버전 백업, 미사용)
- **color_shape_detector** — 카메라 도형/색 인식 → 도형 상대각(`/image_angle`).
- **launch_files** — 최상위 통합 런치.
- **ssf_interfaces** — 커스텀 srv `NorthGoalAngleSv`(int32 request/response). **현재 노드에서 미사용(orphan).**
- **wall_direction** — `src` 소스가 삭제되고 `build/`, `install/`에만 잔존. 벽면-추종 변형으로 추정되나 **소스 없음/미확인.**

### 서드파티 드라이버
- **rplidar_ros** — RPLIDAR A3 드라이버. `/scan` (LaserScan) 발행.
- **razor-imu-ros2** — SparkFun Razor IMU. **C++ 노드(`razor_imu_ros2_exe`)** 가 실사용본이며 `imu`(Imu)와 **`imu/yaw`(Float64, 0~360 정규화)** 를 발행. 같은 패키지의 `launch/imu_publisher.py`(Python)는 `imu`만 내는 **미사용 대체본**.
- **ublox** — u-blox GPS(`/ublox_gps_node/fix`, NavSatFix). **현재 launch에서 주석 처리됨(비활성).**
- **ntrip_client** — RTK 보정(RTCM). **launch에서 주석 처리됨(비활성).**
- **nmea_msgs, rtcm_msgs** — 메시지 정의 의존성.
- **micro_ros_setup, uros/micro-ROS-Agent** — Arduino Due 펌웨어와 통신하는 micro-ROS 에이전트.

## 3. 노드별 상세 (토픽/역할)

### north_goal_angle (`north_goal_angle.py`)
- 구독: `/ublox_gps_node/fix`(NavSatFix), `/image_angle`(Float64)
- 발행: `/north_goal_angle_tp`(Float32, 목표 방위각), `/goal_distance`(Float32, m)
- 로직: `waypoints` 리스트(위도,경도,flag,dwell)를 순회. 도착판정 `dist<1.5m` + `dwell`초 유지 → 다음 WP.
- **핵심 설정값**: `force_north_goal_angle=True`, `force_north_goal_angle_value=80.0` → **GPS 방위 계산을 무시하고 목표각을 80°로 강제**. `wp_idx=1`에서 시작(0 아님). wp_idx==1일 때만 `image_angle`로 `goal=80+image_angle` 적용.
- 주의: `waypoints` 좌표가 실측(36.6xx,127.2xx)과 플레이스홀더(35.0xx,128.5xx)가 섞여 있음.

### ship_goal_angle (`ship_goal_angle.py`)
- 구독: `/imu/yaw`(Float64), `/north_goal_angle_tp`(Float32)
- 발행: `/yaw_error`(Float64) = `(goal_yaw - current_yaw) mod 360`, 0.5초 타이머.
- setup.py에 `ship_direction_node = ship_goal_angle.ship_direction:main` 엔트리가 있으나 해당 모듈 부재 가능(잔재).

### ship_direction (`ship_direction.py`)
- 구독: `/scan`(LaserScan), `/yaw_error`(Float64)
- 발행: `/desired_angle`(Float32), `/obstacle_distance_array`(Float32MultiArray=[최근접거리,최근접각]), `/rotate_command`(Int32) — **구독자 없음(dead topic)**
- 로직: 0~160° 구간 이진화 → 안전구역 탐색 → 배 폭(half_width=0.08) 고려한 통과 가능 구역 중 목표방위(yaw_mapped)에 가장 가까운 구역 중심으로 조향.
- 특수 출력: 정면(70~90°) 0.3m 미만 → `final_angle=260.0`(후진), `reverse_time=3.0`초 쿨다운. 후보 없음 → `final_angle=5000/6000` + `/rotate_command=5000/6000`(제자리 회전 의도).
- 락 없음(MultiThreadedExecutor인데 공유상태 보호 없음).

### motor_control (`motor_control.py`)
- 구독: `/desired_angle`(Float32), `/obstacle_distance_array`(Float32MultiArray), `/goal_distance`(Float32)
- 발행: `Motor_run`(Int32) = `pwm_r*10000 + pwm_l`
- 로직: `desired_angle==280`이면 강제후진(R,L=-30). 아니면 80° 기준 offset*0.5를 turn(±20 제한)으로, base_speed=20에 차동 적용, smooth_flag=0.3 저역통과. `remap_pwm`: 정지출력=1490.
- **PWM 파라미터(declare_parameter, 배별 튜닝 대상)**: R_F_min1510/R_F_limit1600/R_B_min1430/R_B_limit1350, L_F_min1510/L_F_limit1700/L_B_min1430/L_B_limit1300. 현재 launch에서 파라미터 미전달 → 기본값 사용.

### color_shape_detector
- `basic_image_publisher.py`: `cv2.VideoCapture(6)`(웹캠 인덱스 6 하드코딩) → `video_frames`(Image) 0.1초.
- `basic_image_subscriber.py`: `video_frames` 구독(컬러) + **자체적으로 RealSense depth 파이프라인(pyrealsense2) 직접 오픈** → 도형 인식 → `/image_angle`(Float32). 목표 = **green Triangle**. 실패 시 `10000.0` 발행.
- 각도식(매직넘버): `real_x=(rel_x/80.0)*0.09*(distance/0.5)`, `angle=-deg(atan2(real_x,distance))`.

## 4. 하드웨어/펌웨어 인터페이스 (Arduino Due + micro-ROS)
- 별도 아두이노 스케치(워크스페이스 밖)가 `/Motor_run`(Int32) 구독.
- 디코딩: `AutoRin = data/10000`, `AutoLin = data - AutoRin*10000`. → Python의 `pwm_r*10000+pwm_l`와 정합(정상).
- 게이팅: `if(data > 10000000)` 일 때만 자동 반영. pwm_r가 1350~1600이라 항상 통과.
- ESC 매핑: `autoToPower_R`: 입력[1250,1750]→출력[2000,1000](**R 반전**, 차동추진이라 정상). `autoToPower_L`: [1250,1750]→[1000,2000](정방향). **입력 1500 → ESC 1500 = 중립.** (Python 정지값 1490은 정확한 중립 아님: R≈1520,L≈1480)
- RC: Auto 스위치 PWM `<1500`이면 오토파일럿, 아니면 수동 RC 조종. **RC 신호 없으면(부팅 시 등) 기본이 오토파일럿.**
- ESC 3개(R/L/M). **미들(M) ESC는 오토파일럿 모드에서 제어 안 됨**(수동 분기에서만 갱신).
- **`It_is_Aship` 정의 버그**: 주석은 `A배=0, B배=2`인데 코드는 `if(It_is_Aship==1)` 검사 → 값 2를 넣어도 대체 범위 분기가 안 걸림(=B배가 A배 범위 사용). `==2`로 고치거나 정의 통일 필요.
- **워치독 없음**: `/Motor_run` 수신이 끊겨도 마지막 값 유지(정지 안 함).

## 5. 토픽 연결 맵
```
/ublox_gps_node/fix (NavSatFix) ─▶ north_goal_angle        [현재 GPS launch 주석처리]
/image_angle (Float32/64) ───────▶ north_goal_angle
north_goal_angle ─▶ /north_goal_angle_tp (Float32) ─▶ ship_goal_angle
north_goal_angle ─▶ /goal_distance (Float32) ──────▶ motor_control
razor_imu_ros2_exe ─▶ /imu/yaw (Float64) ──────────▶ ship_goal_angle
ship_goal_angle ─▶ /yaw_error (Float64) ───────────▶ ship_direction
rplidar ─▶ /scan (LaserScan) ──────────────────────▶ ship_direction
ship_direction ─▶ /desired_angle (Float32) ────────▶ motor_control
ship_direction ─▶ /obstacle_distance_array (F32MA) ▶ motor_control
ship_direction ─▶ /rotate_command (Int32) ─────────▶ (구독자 없음, DEAD)
motor_control ─▶ Motor_run (Int32) ─▶ micro-ROS agent ─▶ Arduino Due ─▶ ESC×3
video_frames (Image): basic_image_publisher ─▶ basic_image_subscriber
```

## 6. 웨이포인트/FSM 요약
- 미션 진행은 `north_goal_angle`의 `wp_idx`로만 관리(단일 FSM). 각 WP 도착(1.5m 이내) + dwell초 대기 → 인덱스 증가.
- `wp_idx=1`부터 시작. wp1에서 비전 도형각 사용, 그 외엔 강제각 80°.
- (참고: 다른 폴더 버전에는 wp_mode별 미션노드 ship_gate/dock/turn/last/back가 별도 존재. 이 zip 버전에는 없음 — 더 단순한/이전 버전임.)

## 7. 알려진 버그·위험요소 (확인됨)
1. **후진 신호 불일치(치명적)**: 발행측은 `260`(ship_direction 후진), 수신측 motor_control은 `==280`만 후진 처리 → **후진이 절대 발동 안 함.** 260/280 중 하나로 통일 필요.
2. **정지 수단 부재**: 이 버전 motor_control엔 명시적 정지 분기가 없음(후진만). `/rotate_command`(5000/6000)는 구독자 없어 제자리 회전 미구현. final_angle 5000/6000은 desired_angle로 가서 `%360`(→320/240°)로 오해석됨.
3. **소프트웨어 페일세이프 없음**: 비전 유실만 degrade(10000→강제각). GPS/IMU/LiDAR/노드/통신 유실 시 전부 "마지막 명령 유지"로 계속 주행. 유일 안전장치는 사람 RC 수동전환. + 펌웨어 워치독 없음 + 무RC 시 기본 오토파일럿 → 폭주 위험.
4. **펌웨어 `It_is_Aship==1` 오타**: A/B 두 배 구분 분기가 죽어 있음(4절 참조).
5. **비전 노드 이슈**: (a) 컬러는 webcam 토픽, depth는 자체 RealSense → **정렬 안 됨**; (b) `VideoCapture(6)` 인덱스 하드코딩; (c) `cv2.imshow/waitKey` → GUI 필요(헤드리스 불가); (d) 각도식 매직넘버(카메라 미보정).
6. **launch에서 GPS/NTRIP 주석처리**: `/ublox_gps_node/fix` 미발행 → north_goal_angle의 lat/lon=0 유지 → 거리 비정상(거대값)·방위 무효. 단 force_north_goal_angle=True가 방위를 가림. `goal_distance`가 거대해 감속 로직 무력.
7. **PWM 중립 불일치**: Python 정지=1490인데 실제 ESC 중립은 입력 1500. 미세 크리핑 가능.
8. **ship_direction 스레드 안전성**: 락 없이 공유상태 접근.
9. **orphan 자원**: `ssf_interfaces`(srv 미사용), `wall_direction`(소스 없음), ship_goal_angle setup.py의 부재 모듈 엔트리, `motor_control copy.py` 백업.

## 8. 미해결 질문 (하드웨어/의도 확정 필요)
- **A배/B배 각각의 ESC PWM 범위**(전진/후진 min·limit)와 실제 중립값. 새 선체라 재캘리브레이션 필수.
- **미들(M) 스러스터**: 실제 장착 여부와 자율주행 시 역할(추력/미사용?).
- **페일세이프 의도**: RC 유실→오토, ROS 끊김→마지막값 유지가 의도인지, 중립정지로 바꿀지.
- **후진 규약**: 260이 정답인지 280이 정답인지.
- **GPS/NTRIP**: 원래 활성이어야 하는지(현재 주석). RTK 사용 여부.
- **wall_direction**: 무슨 미션용이었는지(소스 복구 필요).
- **대회 규정/코스**: 웨이포인트 순서·미션 정의 판단에 룰북 필요.

## 9. 권장 다음 작업 (우선순위)
1. 후진값 통일(260↔280) + 명시적 정지 경로 추가(입력 1500/1500 발행).
2. 펌웨어 `It_is_Aship` 분기 수정 + 배별 PWM 파라미터 파일(A/B) 분리.
3. 페일세이프: 펌웨어 워치독(수신 N ms 없으면 중립), ship_direction 센서 신선도 체크, candidate/desired 신선도 타임아웃.
4. 비전: depth-color 정렬(align) 또는 단일 RealSense 스트림으로 통합, 카메라 인덱스/해상도 파라미터화, imshow 디버그 플래그화.
5. GPS/NTRIP 재활성 여부 결정 및 무 GPS 시 안전 동작 정의.

# IMU 노드 모음 — 아주 쉬운 완전 해설

이 문서는 `IMU_노드_모음.zip` 안에 있는 파일들을 **하나도 빼놓지 않고, 처음 보는 사람도 이해하도록** 설명합니다.
코드 자체보다 "이 파일이 뭘 하는 애인지, 그리고 노드랑 어떻게 연결되는지"에 초점을 맞췄습니다.
(LiDAR 쉬운 설명 문서와 같은 방식입니다.)

---

## 0. 먼저 IMU가 뭔지 + 3개 단어

**IMU** = 배가 지금 **어느 방향을 보고 있는지(방위, yaw)** 를 알려주는 센서입니다. 나침반+자이로라고 보면 됩니다.
배가 "북쪽을 0°로 봤을 때 지금 몇 도를 향하는지"를 숫자로 알려줍니다.

ROS2 기본 3단어(방송국 비유):
- **노드(Node)** = 일하는 프로그램 하나 (직원 한 명)
- **토픽(Topic)** = 정보를 주고받는 라디오 채널. 발행(방송)하면 구독(듣기)하는 직원이 받음
- **패키지(Package)** = 관련 파일을 담은 폴더 (부서)

이 묶음에는 **패키지(부서)가 크게 2개**:
1. **iahrs_driver_ros2-main** — IMU 센서에서 방위를 뽑아내는 **"센서 부서"**
2. **ship_goal_angle** — 그 방위로 "목표까지 얼마나 틀어졌는지"를 계산하는 **"방위 오차 계산 부서"**

둘을 잇는 라디오 채널은 **`/imu/yaw`** 입니다.

```
[IMU 센서] → (iahrs_driver 부서) → /imu/yaw 채널 → (ship_goal_angle 부서) → /yaw_error
```

---

## 1. 전체 그림 (데이터가 흐르는 길)

```
 실제 iAHRS 센서 (시리얼 케이블 /dev/IMU)
        │  (100Hz로 방위 측정)
        ▼
 ┌──────────────────────────┐        GPS(/ublox/navpvt) ─┐
 │ iahrs_driver 노드          │◀───────────────────────────┘  (GPS 방위도 참고)
 │ = IMU 방위를 계산/발행      │
 └──────────────────────────┘
        │  /imu/yaw (현재 방위, 0~360°)
        ▼
 ┌──────────────────────────┐    north_goal_angle ─▶ /north_goal_angle_tp (목표 방위)
 │ ship_goal_angle 노드       │◀──────────────────────────────────────────┘
 │ = 목표방위 − 현재방위 계산  │
 └──────────────────────────┘
        │  /yaw_error (얼마나 틀어졌나)
        ▼
   ship_direction (조향 결정에 사용)
```

핵심 한 줄: **iahrs_driver가 만든 `/imu/yaw`(지금 방위)를, ship_goal_angle이 목표 방위와 비교해 `/yaw_error`(틀어진 양)를 만든다.**

---

## 2. 패키지 ①: iahrs_driver_ros2-main (센서 부서)

IMU 하드웨어를 직접 다루는 **드라이버**입니다. C++로 작성돼 있고, 센서와 컴퓨터 사이의 **통역사**입니다.
이 폴더 안에는 다시 **작은 패키지 2개**가 들어 있습니다: `iahrs_driver`(본체)와 `interfaces`(부품).

### 📁 iahrs_driver/ — 진짜 드라이버 부서

#### 📄 src/iahrs_driver.cpp — "이 부서의 뇌 (핵심 파일)"
IMU 노드의 본체입니다. 하는 일을 쉬운 순서로 풀면:
1. **시리얼 포트 `/dev/IMU`를 열어** 센서와 연결 (속도 115200).
2. 100Hz로 센서에게 "방위 값 줘(`e\n`)"라고 물어보고 roll/pitch/**yaw**를 받음.
3. **방향 뒤집기**: 센서 원래 yaw는 반시계(CCW)로 증가 → 나침반처럼 **시계방향(CW) 증가**로 바꿈.
4. **부팅 0점화**: 켜질 때 처음 읽은 방위를 "0도"로 삼음 → 즉 "켤 때 바라보던 방향"이 기준 0°가 됨.
5. **GPS 방위 덮어쓰기(중요)**: GPS(`/ublox/navpvt`)가 주는 진행방향이 충분히 정확하면, IMU yaw 대신 **GPS 방위를 그대로 사용**함.
6. 최종 방위를 **`/imu/yaw`(Float64, 0~360°)** 로 방송. 자세 전체는 `imu/data`(Imu)로도 방송.

> ⚠️ **주의 1 — GPS 덮어쓰기의 함정**: GPS 방위가 한 번 유효해지면, 이후 GPS가 끊겨도 **마지막 GPS 값에 고정된 채 IMU를 계속 무시**합니다(리셋이 없음). 방위가 옛날 값에 멈춰버릴 수 있어 손봐야 할 1순위입니다.
> ⚠️ **주의 2 — 포트 `/dev/IMU`**: 이 이름은 udev 규칙이 있어야 잡힙니다. 새 노트북/배엔 규칙을 다시 등록해야 합니다.

#### 📁 launch/ — "실행 버튼"
- **`iahrs_driver.py`** : 실제로 IMU 노드를 켜는 파일. (파이썬 버전)
- **`iahrs_driver_launch.xml`** : 같은 역할의 XML 버전(대체용). 보통 하나만 씀.

#### 📄 package.xml — "부서 신분증"
이름(iahrs_driver)과 필요한 부품들(rclcpp, sensor_msgs, tf2, geometry_msgs, **ublox_msgs**=GPS 메시지, **interfaces**=아래 리셋 서비스)이 적혀 있습니다.

#### 📄 CMakeLists.txt — "빌드 설명서"
이 C++ 코드를 어떻게 컴파일할지 알려주는 레시피.

### 📁 interfaces/ — "IMU 리셋 명령 부품 부서"

#### 📄 srv/ImuReset.srv — "IMU 방위 초기화 명령서 양식"
IMU에게 "지금 방위를 0으로 리셋해"라고 보내는 **명령(서비스)의 형식**을 정의한 파일입니다.
내용은 아주 단순: 요청은 비어있고(그냥 "리셋해"), 응답으로 `bool result`(성공/실패)를 돌려줍니다.
- `package.xml`, `CMakeLists.txt` : 이 부품 부서의 신분증과 빌드 설명서.

### 📄 README.md
드라이버 사용법 설명서. 코드 동작과는 무관.

---

## 3. 패키지 ②: ship_goal_angle (방위 오차 계산 부서)

`/imu/yaw`(지금 방위)를 받아서, "목표 방향까지 얼마나 더 틀어야 하는지"를 계산하는 노드입니다. 우리 팀이 만든 Python 코드입니다.

### 📄 ship_goal_angle/ship_goal_angle.py — "이 부서의 뇌"
하는 일:
1. **`/imu/yaw` 구독** — 지금 배가 향한 방위를 받음 (current_yaw).
2. **`/north_goal_angle_tp` 구독** — 가야 할 목표 방위를 받음 (goal_yaw). 이 목표 방위는 north_goal_angle 노드가 GPS 웨이포인트로 계산해 줍니다.
3. **오차 계산**: `yaw_error = (목표방위 − 현재방위)` 를 0~360° 범위로 정리.
4. **`/yaw_error`(Float32) 방송** — 0.5초마다.

쉽게 말해 "지금 이만큼 틀어져 있으니 이만큼 돌아야 해"라는 숫자를 만드는 노드입니다. 이 값을 ship_direction이 받아 실제 조향에 씁니다.

이 노드가 **구독** 하는 채널: `/imu/yaw`, `/north_goal_angle_tp`
이 노드가 **발행** 하는 채널: `/yaw_error`

### 📄 launch/ship_goal_angle_launch_file.launch.py — "이 노드만 켜는 실행 버튼"

### 📄 setup.py — "설치 방법서"
가장 중요한 줄: `'ship_goal_angle_node = ship_goal_angle.ship_goal_angle:main'`
→ "이 이름으로 실행하면 ship_goal_angle.py의 main()을 켜라"는 연결고리.

### 📄 그 외 (자동 생성, 신경 안 써도 됨)
- **`package.xml`** : 신분증(필요 부품: rclpy, std_msgs 등).
- **`setup.cfg`** : 실행 파일 설치 위치 보조 설정.
- **`resource/ship_goal_angle`** : "이 패키지 있어요" 표식용 빈 파일.
- **`ship_goal_angle/__init__.py`** : 폴더를 파이썬 패키지로 인식시키는 빈 파일.
- **`test/` (test_flake8.py 등)** : 코드 스타일 자동 검사. 대회 동작과 무관.

---

## 4. 두 부서가 연결되는 지점 (제일 중요)

```
iahrs_driver 노드
     │
     │  ┌──────────────────────────────────────┐
     └─▶│  /imu/yaw  (Float64, 0~360°)          │  ← 지금 배가 향한 방위
        └──────────────────────────────────────┘
     │
     ▼
ship_goal_angle 노드  ──(+ /north_goal_angle_tp 목표방위)──▶  /yaw_error 계산
```

- **연결 방법**: 같은 토픽 이름 `/imu/yaw`를 한쪽은 발행, 한쪽은 구독 → ROS2가 자동으로 이어줍니다.
- **주고받는 내용물**: `Float64` 숫자 하나(0~360° 방위). LiDAR의 `LaserScan`처럼 복잡한 배열이 아니라 **간단한 각도 숫자**입니다.
- ship_goal_angle이 결과를 낼 때 목표 방위(`/north_goal_angle_tp`)가 하나 더 필요한데, 이건 GPS 담당(north_goal_angle)에서 옵니다.

---

## 5. 실제로 켜지는 순서 (부팅하면 벌어지는 일)

1. 통합 실행 파일이 **iahrs_driver 노드**를 켬 → `/dev/IMU` 열고 `/imu/yaw` 방송 시작.
2. **ship_goal_angle 노드**가 켜져서 `/imu/yaw`와 `/north_goal_angle_tp`를 구독.
3. 0.5초마다 (목표방위 − 현재방위)를 계산해 `/yaw_error` 방송.
4. **ship_direction**이 이 `/yaw_error`와 LiDAR `/scan`을 함께 보고 최종 조향각(`/desired_angle`)을 정함.

---

## 6. 한 줄 요약

- **iahrs_driver** = IMU 하드웨어를 읽어 `/imu/yaw`(현재 방위)로 방송하는 **센서 통역사** (핵심 파일: `iahrs_driver/src/iahrs_driver.cpp`, 실행: `launch/iahrs_driver.py`)
- **ship_goal_angle** = 목표 방위와 비교해 `/yaw_error`(틀어진 양)를 내는 **오차 계산자** (핵심 파일: `ship_goal_angle/ship_goal_angle.py`)
- 둘의 연결 고리 = 라디오 채널 **`/imu/yaw`** 하나. (거기에 목표 방위 `/north_goal_angle_tp`가 더해짐)

## 7. LiDAR와의 차이 (한눈에)
| | LiDAR (ship_direction) | IMU (ship_goal_angle) |
|---|---|---|
| 센서가 주는 것 | 사방 거리값 배열(LaserScan) | 방위 각도 하나(Float64) |
| 드라이버 | rplidar_ros | iahrs_driver |
| 소비 노드 | ship_direction | ship_goal_angle |
| 연결 토픽 | /scan | /imu/yaw |
| 결과 | /desired_angle (피할 각도) | /yaw_error (틀어진 방위) |
| 특이점 | 수면 반사·노이즈 | GPS 방위로 덮어쓰기 |

## 8. 용어 사전
- **IMU** : 배의 방위(향한 방향)를 알려주는 센서
- **yaw** : 좌우 회전 방위 각도 (0~360°)
- **노드 / 토픽 / 패키지** : 직원 / 라디오채널 / 부서
- **발행 / 구독** : 방송하기 / 듣기
- **서비스(srv)** : 토픽과 달리 "요청 → 응답"으로 한 번 주고받는 명령 (예: IMU 리셋)
- **udev 규칙** : USB 장치를 항상 같은 이름(`/dev/IMU`)으로 부르게 하는 설정
- **COG(GPS heading)** : GPS가 재는 "진행 방향". 배가 움직일 때만 정확

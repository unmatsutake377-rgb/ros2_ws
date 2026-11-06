# sim/vrx — VRX(Gazebo Garden) 위에서 SSF 스택 돌리기

**상태(11-06): 파일은 다 있고 Mac 에서 생성기·변환 함수 테스트(21개)만 통과. Gazebo 에서 한 번도 안 돌렸다.**
Ubuntu 노트북에서 아래 순서대로 올리고, §5 점검표의 기대값이 나오는지 확인해야 "된다"고 말할 수 있다.
대회 코드(`src/`)는 한 줄도 안 바꿨다. 이 폴더는 **별도 colcon 워크스페이스**다.

## 1. 무엇이 들어 있나

| 경로 | 역할 |
|---|---|
| `src/ssf_vrx/models/ssf_boat_a`, `ssf_boat_b` | 배 모델 SDF (생성 파일, `make_boat.py`). 선체 상자 + VRX 부력(Surface ×2) + 수력(SimpleHydrodynamics) + gz Thruster(A 2기 / B 4기 V자) + 센서 4종(LiDAR·카메라·IMU·GPS) + 지상진실 odometry |
| `src/ssf_vrx/models/kaboat_buoy_*`, `kaboat_obst_*` | KABOAT 부표(높이 1.0 폭 0.5 m)·장애물(0.6/0.4 m) 7색, **static** |
| `src/ssf_vrx/worlds/kaboat_a.sdf`, `kaboat_b.sdf` (+`.json`) | 코스 월드 (생성 파일, `make_world.py`). 원점 = `waypoints.yaml` WP0, 게이트 4·위치유지·탐색 3·장애물·도크 3칸 — 배치 규칙은 `tools/sim/full_course_sim.py` 와 같다(같은 시드 = 같은 코스) |
| `src/ssf_vrx/ssf_vrx/sim_bridge.py` | ssf_bridge + 아두이노 + IMU 드라이버 대역: `Motor_run` → 추진기 N, `/boat_mode` 등 상태, gz IMU → `/imu/yaw_raw` |
| `src/ssf_vrx/ssf_vrx/convert.py` | 순수 변환(PWM→N, ENU→나침반 헤딩, Motor_run 해석, 워치독) — 테스트 대상 |
| `src/ssf_vrx/launch/ssf_vrx.launch.py` | gz sim(VRX) + ros_gz_bridge + sim_bridge (+ 스택) |
| `src/ssf_vrx/launch/ssf_stack_sim.launch.py` | 드라이버·ssf_bridge 없는 대회 스택. 대회 launch 파일의 함수·yaml 을 **import 해서** 쓴다(복사본 아님) |
| `src/ssf_vrx/config/ssf_vrx.yaml` | sim_bridge 파라미터(추력 상한, 데드밴드, 워치독 0.5 s, boat_mode 2) |
| `src/ssf_vrx/test/` | ROS 없이 도는 테스트 21개 |

### 실물 ↔ 시뮬 대응

| 실물 | 시뮬 | 우리 스택이 보는 토픽 |
|---|---|---|
| RPLIDAR A3 (rplidar_ros) | gpu_lidar 1440샘플 360° 11 Hz 25 m, **센서를 -80° 돌려 달아 스캔 80° = 뱃머리**(ship_direction `LIDAR_FORWARD_DEG`) | `/scan` |
| OAK-1 PoE (depthai) | camera 640×360, hfov 63.9°(vision.yaml 과 같음), 30 fps | `/oak/rgb/image_raw` |
| iAHRS → `/imu/yaw_raw` | gz imu(ENU 쿼터니언) → sim_bridge 가 (90 − yaw) mod 360 | `/imu/yaw_raw` → yaw_mux → `/imu/yaw` |
| ZED-F9P (ublox) | navsat 1 Hz, 월드 원점 위경도 기준 | `/ublox_gps_node/fix` (`fix_velocity` 는 없음 — cog_offset 미사용이라 영향 없음) |
| Arduino + ESC + T200 | gz Thruster(N) ← sim_bridge PWM→N 1차 근사(1100 → +47 N, 1900 → −35 N, 1475~1525 정지) | `Motor_run` 그대로 |
| ssf_bridge 상태줄 | sim_bridge 10 Hz, `/boat_mode` = 2(AUTO) 고정 | `/boat_mode`, `/boat_estop`(false), `/boat_id`, `/boat_cmd_watchdog` |
| — | gz OdometryPublisher (지상진실) | `/vrx/ground_truth` (nav_msgs/Odometry) |

**가정값(실측 아님, 수조 결과로 바꿀 것):** 수력 계수(xU 10, xUU 25, nR 6 …), 부력 원기둥(반지름 0.15/0.14 m ×2), PWM→추력 선형, 후진 35 N.
현재 계수로 두 기 1360(각 14.4 N)이면 약 1.0 m/s, 전출력이면 약 1.75 m/s 로 설계했다(계산값, 미검증).

## 2. 전제

- Ubuntu 22.04 + ROS 2 Humble, `~/ros2_ws` 가 빌드돼 있을 것(이 패키지가 `launch_files`·`north_goal_angle` 등의 share 를 읽는다).
- **그래픽:** 배 노트북은 내장 GPU 뿐이다. 되긴 하지만 느릴 수 있다 → §5 의 RTF 확인이 첫 관문. 외장 GPU PC 가 있으면 그쪽에 올리고 노트북은 그대로 둔다.
- 인터넷: 첫 실행 때 VRX 가 Fuel 에서 모델을 받지 않도록 우리 월드는 Fuel 모델을 안 쓴다(`coast_waves`·`symbol_triangle` 은 vrx_gz 안에 있음). Gazebo 설치 때만 필요.

## 3. 설치 (한 번)

```bash
# 1) Gazebo Garden (VRX humble 브랜치가 요구). Garden 은 지원 종료됐지만 패키지는 내려받아진다.
sudo apt-get update && sudo apt-get install -y lsb-release gnupg curl
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update && sudo apt-get install -y gz-garden
```

```bash
# 2) ROS 2 Humble ↔ Garden 브릿지 (패키지 이름이 'gzgarden' 인 변종. 'ros-humble-ros-gz' 는 Fortress용이라 쓰지 않는다)
sudo apt-get install -y ros-humble-ros-gzgarden
```

```bash
# 3) VRX humble 브랜치 — 별도 워크스페이스
mkdir -p ~/vrx_ws/src && cd ~/vrx_ws/src && git clone -b humble https://github.com/osrf/vrx.git
cd ~/vrx_ws && source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y
cd ~/vrx_ws && colcon build --merge-install
```

```bash
# 4) 이 패키지 — ros2_ws 와 vrx_ws 를 깔고(underlay) 그 위에 빌드
cd ~/ros2_ws/sim/vrx && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && source ~/vrx_ws/install/setup.bash && colcon build --symlink-install
```

이후 터미널마다 **이 순서로** source 한다: `/opt/ros/humble` → `~/ros2_ws/install` → `~/vrx_ws/install` → `~/ros2_ws/sim/vrx/install`.

확인: `ros2 pkg prefix vrx_gz`, `ros2 pkg prefix ros_gz_bridge`, `ros2 pkg prefix ssf_vrx` 가 모두 경로를 찍어야 한다.

## 4. 실행

```bash
ros2 launch ssf_vrx ssf_vrx.launch.py boat:=A
```

- Gazebo 창이 뜨고 배(흰 상자)·부표·도크가 보인다. 스택이 같이 떠서 `/boat_mode` 2 라 **바로 자율 주행을 시작**한다(WP0 부터).
- 화면 없이: `headless:=true`. 멈춘 채 시작: `paused:=true`. 시뮬만: `with_stack:=false`.
- B 배: `boat:=B` (월드 `kaboat_b`, 4기 모델).
- 다른 코스(시드)·목표 칸·파도·바람: 소스 폴더에서 월드를 다시 만들고 빌드한다.

```bash
cd ~/ros2_ws/sim/vrx/src/ssf_vrx && python3 -m ssf_vrx.make_world --boat A --seed 7 --bay 1 --wave-gain 0.3 --wind 3.0 && cd ~/ros2_ws/sim/vrx && colcon build --symlink-install
```

블랙박스 CSV 는 실물과 같이 `~/ssf_logs/blackbox_*.csv`, 그림은 `python3 tools/blackbox_plots.py`. 지상진실은 `ros2 bag record /vrx/ground_truth`.

## 5. 처음 올릴 때 점검표 (순서대로, 기대값)

1. **RTF**: `gz stats` 의 real time factor. **0.5 미만이면 멈추고** 노트북 대신 GPU PC 로 가거나 `headless:=true` 로 다시 잰다. 노드가 벽시계로 돌아서 RTF 가 낮으면 센서 간격이 벌어져 페일세이프(0.7/3.0 s)·워치독(0.5 s)이 실제보다 쉽게 걸린다.
2. **토픽이 다 나오나** (`with_stack:=false` 로 먼저):
   `ros2 topic hz /scan` ≈ 11, `/oak/rgb/image_raw` ≈ 30, `/ublox_gps_node/fix` ≈ 1, `/imu/yaw_raw` ≈ 100, `/boat_mode` 10 (값 2), `/vrx/ground_truth` 10.
3. **헤딩 부호**: `ros2 topic echo /imu/yaw_raw` 가 `worlds/kaboat_a.json` 의 `psi0`(나침반, 0=북 시계)과 ±2° 안. 다르면 `convert.heading_from_quat` 또는 모델 초기 yaw 를 본다.
4. **추진 부호·좌우**: 스택 없이 `ros2 topic pub -r 20 Motor_run std_msgs/msg/Int32 "{data: 13601360}"` → 배가 **앞으로**(지상진실 x·y 가 헤딩 방향으로 증가). `"{data: 14001320}"`(오른쪽 1400 약함, 왼쪽 1320 강함) → **오른쪽으로 선회**. 반대면 sim_bridge 의 좌우 토픽이 바뀐 것.
5. **LiDAR 정면**: 부표 하나가 바로 앞에 오게 둔 뒤 `ros2 topic echo /obstacle_distance_array` 의 두 번째 값(각도)이 ≈ 80. 왼쪽 부표는 > 80. 아니면 `make_boat.py` 의 센서 yaw 부호.
6. **카메라 색**: 게이트 앞에서 `ros2 topic echo /red_angle /green_angle` 이 유효값(10000 아님). 안 잡히면 `basic_image_subscriberhsv` 로 HSV 를 보고 `vision.yaml` 이 아니라 **부표 재질 색(`make_world.COLORS`)** 쪽을 맞춘다(실물 색 범위를 시뮬 때문에 바꾸지 않는다). 파랑 표식은 실물 범위(흐린 하늘색)와 안 맞을 수 있다 — 기본 목표는 빨강 사각이라 영향 없음.
7. **속도**: 1360/1360 으로 10 s 뒤 지상진실 속도. 1.0 m/s 근처가 아니면 `make_boat.py` `hydro`(xU·xUU)를 바꾼다. 수조 실측이 나오면 그 값으로.
8. **전체 코스**: 스택 포함으로 띄워 `ros2 topic echo /wp_mode` 가 0→1→2→3→5→7→8 로 넘어가는지, blackbox CSV 로 궤적. `tools/sim/full_course_sim.py` 결과(완주·접촉·시간)와 비교한다.

## 6. 알려진 한계 · 다음 과제

- 부력·수력은 **가정값**. 선체 STL(보고서 선도 파일)로 PolyhedraBuoyancyDrag 메시 부력을 쓰면 더 맞는다(다음 단계).
- 부표가 static 이라 부딪혀도 안 밀린다. 떠다니는 부표는 VRX `PolyhedraBuoyancyDrag` 로 바꿀 수 있다.
- 조류 없음(파도·바람만). RTK 오차·멀티패스·물보라는 모델 안 함 — 센서 noise 만.
- `fix_velocity` 없음, `/imu/data` 없음(블랙박스 열이 비어 보임).
- 노드는 벽시계. `use_sim_time` 을 켜도 노드 안 `time.monotonic()` 로직은 안 바뀐다 → RTF 를 1 근처로 유지하는 것이 유일한 해법.
- Humble 조합(Garden + ros-gzgarden) 은 VRX 2.4.1(2024-01) 기준. 패키지가 안 잡히면 VRX wiki `tutorials-SystemSetupInstall` 의 humble 절을 본다.

## 7. 테스트 (Mac/Ubuntu, ROS 불필요)

```bash
cd ~/ros2_ws/sim/vrx/src/ssf_vrx && python3 -m pytest test -q
```
생성 파일을 바꿨으면 `python3 -m ssf_vrx.make_boat && python3 -m ssf_vrx.make_world --boat A && python3 -m ssf_vrx.make_world --boat B` 로 다시 만들고 함께 커밋한다.

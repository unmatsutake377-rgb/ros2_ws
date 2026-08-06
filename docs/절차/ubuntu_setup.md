# Ubuntu 노트북 셋업 — 초기화 후 실기 제어용 재구축

> **대상 기기:** Lenovo IdeaPad Slim 3 15IAH8 (Intel i5-12450H, **내장 그래픽만** / 외장 GPU 없음)
> **목표:** 공장초기화한 노트북에 ROS2 + 우리 저장소를 세워 실기 제어까지 돌리기.
> **원칙:** 이 문서는 순서표다. 위에서 아래로 따라간다. ⚠️ 는 "실물·실측이 있어야 확정" 표시.
>
> 코드·드라이버는 전부 우리 GitHub 저장소에 있다(작년 잔해 없는 최신본 + 내재화 드라이버).
> 여기서 새로 설치하는 건 **OS·ROS2·시스템 의존성**뿐이다.

---

## 0. 이 기종 특이사항 (먼저 알아둘 것)

| 항목 | 내용 |
|---|---|
| CPU | i5-12450H (12세대, 8코어) — Ubuntu 22.04 기본 커널(5.15+)로 정상 동작 |
| GPU | **Intel 내장(iGPU)뿐.** NVIDIA 없음 → GPU 드라이버 설치 **불필요** |
| RAM | ⚠️ 모델별 8/16GB. `free -h`로 확인. **8GB면 colcon build 병렬 제한 필요**(아래 6단계) |
| 부팅 | 설치 시 BIOS(F2)에서 **Secure Boot 끄기** 권장(드라이버·커널 모듈 충돌 방지) |

---

## 1. Ubuntu 22.04 LTS 설치

- **22.04 를 쓴다** (ROS2 **Humble** 이 22.04 전용). 24.04 아님.
- USB 부팅 설치. 설치 중 "Install third-party software" 체크(Wi-Fi·미디어 코덱).
- 설치 후:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl wget nano build-essential
```

---

## 2. ROS2 Humble 설치

```bash
# locale (UTF-8)
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ROS2 apt 소스 추가
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
# 데스크톱 전체 (rviz 등 포함 — 실기에서 시각화 쓰면 편함)
sudo apt install -y ros-humble-desktop
# 개발 도구
sudo apt install -y ros-dev-tools
```

`~/.bashrc` 에 소싱 추가 (매 터미널 자동):
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. 빌드 도구 + rosdep

```bash
sudo apt install -y python3-colcon-common-extensions python3-vcstool python3-rosdep
sudo rosdep init      # 이미 돼 있으면 에러 나도 무시
rosdep update
```

---

## 4. 저장소 clone

```bash
cd ~
git clone https://github.com/unmatsutake377-rgb/ros2_ws.git
cd ~/ros2_ws
```
- clone하면 **작년 잔해 없이** 최신본이 온다. rplidar_ros·micro_ros 4개 드라이버도
  내재화돼 있어 **빈 폴더로 오지 않는다**(작년 사고 원인 — 우리가 고침).
- ⚠️ 저장소 URL이 바뀌었으면 실제 주소로. `git remote -v`로 확인.

---

## 5. 시스템 의존성 (우리 패키지가 요구하는 것)

### 5-1. RealSense SDK (카메라 — 현역 D455)
```bash
sudo apt install -y ros-humble-librealsense2* ros-humble-realsense2-camera
```
⚠️ apt 버전이 카메라 펌웨어와 안 맞으면 인식이 안 될 수 있다. 그때는 Intel 공식
`librealsense` 저장소로 설치(문서: github.com/IntelRealSense/librealsense).
카메라 교체(OAK) 시엔 `docs/oak_arrival_runbook.md` 참고.

### 5-2. 나머지 ROS 의존성 (rosdep 자동)
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```
이게 `cv_bridge`, `image_transport`, `tf2`, `diagnostic_updater`, `nmea_msgs` 등
package.xml 에 선언된 걸 알아서 깐다.

### 5-3. Python 의존성 (rosdep 이 못 잡는 것)
```bash
pip3 install geopy
sudo apt install -y python3-numpy python3-opencv python3-serial
```
- `geopy` : north_goal_angle 의 거리·방위 계산 (`calc_dist`, `calc_angle`)
- `numpy`·`opencv` : 비전 노드
- ⚠️ blackbox 의 system-ID 분석(`fit_dynamics`)에 `scipy`·`pandas` 가 필요하면 그때 추가.

### 5-4. 시리얼·USB 유틸
```bash
sudo apt install -y usbutils setserial
sudo usermod -aG dialout $USER    # 시리얼 포트 접근 권한 (로그아웃 후 적용)
```

---

## 6. 빌드 (colcon)

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

⚠️ **RAM 8GB면 빌드 중 멈추거나 OOM(메모리 부족)** 가능. `free -h`로 확인 후 8GB면:
```bash
colcon build --symlink-install --parallel-workers 2   # 동시 빌드 수 제한
```
- i5-12450H는 8코어라 기본이 8병렬 → RAM을 크게 먹는다. `--parallel-workers 2`로 낮추면
  느리지만 안전.
- 스왑을 늘려두면 더 안전:
```bash
sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

빌드 후 매 터미널 소싱:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 7. micro-ROS Agent (아두이노 통신)

micro-ROS는 일반 colcon 빌드와 절차가 다르다(별도 워크스페이스 빌드).
우리 저장소에 `micro_ros_setup`·`micro-ROS-Agent`가 내재화돼 있다.
```bash
cd ~/ros2_ws
# agent 빌드 (이미 src 에 있으므로 colcon build 에 포함됨)
# 실행:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```
⚠️ 아두이노 포트(`/dev/ttyACM0`)는 실제 연결 후 확인. 펌웨어는 회로팀 담당
(`arduino/ssf_boat/`). Motor_run 계약은 CLAUDE.md 3-8.

---

## 8. udev rules (장비 포트 고정) — ⚠️ 장비 있어야 함

USB 시리얼 장치는 꽂는 순서에 따라 `/dev/ttyUSB0/1/2`가 바뀐다. 고정명으로 박아야
launch가 매번 같은 포트를 찾는다.
```bash
# 각 장비의 벤더/시리얼 ID 확인 (장비 하나씩 꽂고)
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial" | head
```
확인한 값으로 `/etc/udev/rules.d/99-ssf.rules` 작성:
```
# 예시 — 실제 idVendor/idProduct/serial 로 채운다
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{serial}=="YYYY", SYMLINK+="ttyLiDAR"
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{serial}=="ZZZZ", SYMLINK+="IMU"
```
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```
- 목표 심볼릭명: `/dev/ttyLiDAR`(RPLIDAR A3), `/dev/IMU`(iAHRS). GPS는 ublox launch 설정 확인.
- ⚠️ 이건 **오픈소스가 아니라 이 장비 고유값**이라 GitHub에 없다. 장비 오면 새로 만든다.

---

## 9. NTRIP (RTK GPS 보정) — 새 계정

- NTRIP 계정(아이디/비번/캐스터 주소)을 **새로 발급**받는다(작년 것 폐기).
- 설정 위치: `src/ntrip_client/` 의 launch/config. 발급받은 값으로 채운다.
- ⚠️ 계정 정보도 GitHub에 없다(보안). 발급 후 여기 노트북 로컬에만.

---

## 10. 경기 모드 스크립트 (우리가 만든 것)

launch 전에 매번:
```bash
sudo ~/ros2_ws/tools/boat_boot.sh          # CPU performance, USB autosuspend off, 절전 off
~/ros2_ws/tools/boat_boot.sh --check       # 상태만 확인
```
왜: USB autosuspend가 LiDAR/IMU/GPS 시리얼을 끊는 실재 이슈 차단. CLAUDE.md 7-3.

---

## 11. 검증 (세운 뒤 확인)

```bash
# 1) 빌드 성공했나
cd ~/ros2_ws && colcon build --symlink-install && echo "BUILD OK"

# 2) 노드가 뜨나 (장비 연결 후)
ros2 launch launch_files launch_files.launch.py

# 3) 핵심 토픽이 나오나 (다른 터미널)
ros2 topic hz /scan          # LiDAR
ros2 topic echo /imu/yaw     # 헤딩 (yaw_mux 단독 발행)
ros2 topic echo /health_ok   # 출발 가능 여부

# 4) 순수 로직 테스트 (장비 없이도)
python3 src/ship_direction/test/test_failsafe_logic.py
python3 src/ssf_heading/test/test_heading_logic.py
# ... (docs/ 의 테스트 목록 참고)
```

**출발 전 체크리스트는 CLAUDE.md 7-2** (안전 스위치·벤치 실측 항목).

> 📘 **세운 뒤 '매번 돌리는 방법' 은 `docs/절차/ros2_운용법.md` 에 있다.**
> (소싱·launch 3터미널 구성·장비 없이 시험하는 법·자주 걸리는 것)

---

## 12. 자주 걸리는 것 (트러블슈팅)

| 증상 | 원인 / 해결 |
|---|---|
| `colcon build` 가 멈춤/죽음 | RAM 부족 → `--parallel-workers 2` + 스왑(6단계) |
| RealSense 인식 안 됨 | apt librealsense 버전 불일치 → Intel 공식 저장소 설치 |
| `/dev/ttyUSB*` 가 매번 바뀜 | udev rules 미설정 (8단계) |
| 시리얼 `Permission denied` | `dialout` 그룹 (5-4) + **로그아웃/재로그인** |
| `geopy` ImportError | `pip3 install geopy` (5-3) |
| 노드 clone 후 빈 폴더 | (더는 안 생김 — 내재화 완료. 생기면 저장소 문제) |
| 12세대 CPU 발열/스로틀 | `tools/boat_boot.sh` 로 거버너 performance (10단계) |

---

## 13. 한 장 체크리스트 (순서)

- [ ] Ubuntu 22.04 설치 (Secure Boot off)
- [ ] `apt update && upgrade`
- [ ] ROS2 Humble (`ros-humble-desktop`) + `.bashrc` 소싱
- [ ] colcon / vcstool / rosdep
- [ ] `git clone` 저장소
- [ ] RealSense SDK (`librealsense2`)
- [ ] `rosdep install --from-paths src`
- [ ] `pip3 install geopy` + numpy/opencv/serial
- [ ] `dialout` 그룹 + 재로그인
- [ ] `colcon build` (RAM 8GB면 `--parallel-workers 2`)
- [ ] install setup.bash 소싱
- [ ] (장비) udev rules
- [ ] (장비) NTRIP 새 계정
- [ ] `tools/boat_boot.sh` 동작 확인
- [ ] launch → `/scan`·`/imu/yaw`·`/health_ok` 확인

> ⚠️ 명령·버전은 작성 시점 기준. ROS2 apt 키/URL은 공식 문서(docs.ros.org/en/humble)에서
> 최신 확인 후 진행. 장비 관련(8·9)은 실물이 와야 완성된다.

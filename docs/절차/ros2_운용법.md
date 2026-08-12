# ROS2 운용법 — 매번 돌릴 때 보는 문서

> **`ubuntu_setup.md` 와 역할이 다르다.** 그건 "노트북을 **한 번** 세우는 순서표"고,
> 이건 "세운 뒤 **매번** 돌리는 방법"이다. 셋업이 끝난 상태를 전제로 한다.
>
> 검증 환경: Ubuntu 22.04.5 / ROS2 Humble / `~/ros2_ws` / 28패키지 빌드 통과(2026-08-05).
> ⚠️ 는 "장비·실측이 있어야 확정" 표시.

---

## 0. 소싱 — 안 되면 90%는 이것 때문이다

ROS2 는 터미널마다 **환경 소싱**이 필요하다. `ubuntu_setup.md` 대로 했으면 `~/.bashrc` 에 들어 있어서
**새 터미널을 열면 자동**이다.

```bash
ros2 pkg executables ship_direction     # ship_direction ship_direction 나오면 정상
```

안 나오면 그 터미널만 수동으로:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

🚨 **빌드한 뒤에는 터미널을 새로 열거나 위를 다시 실행해야 새 실행파일이 잡힌다.**
이걸 안 해서 "고쳤는데 그대로다" 를 겪는 게 ROS2 에서 가장 흔한 헛수고다.

---

## 1. 작동 방식 (3줄)

**노드**(프로그램) 여러 개가 각자 돌면서 **토픽**(이름 붙은 메시지 통로)으로 대화한다.
중앙 서버가 없어서 **아무 순서로나 띄워도 되고**, 한 노드가 죽어도 나머지는 계속 돈다.

🚨 그래서 이 프로젝트가 반복해 당한 함정이 생긴다 —
**토픽 이름이 한 글자만 어긋나도 에러가 안 나고 그냥 조용히 아무 값도 안 온다.**
CLAUDE.md 의 "침묵 실패"(도킹 mode 9 vs 7, `/gate_pass_count` vs `/gates_passed`)가 전부 이 유형이다.
→ 그래서 `healthcheck` 와 `blackbox`(0단계)가 존재한다. **눈 없이 돌리지 말 것.**

---

## 2. 명령 한 장 요약

| 하고 싶은 것 | 명령 |
|---|---|
| 노드 하나 띄우기 | `ros2 run <패키지> <실행파일>` |
| 여러 노드 한 번에 | `ros2 launch <패키지> <launch파일>` |
| 어떤 토픽이 있나 | `ros2 topic list` |
| 토픽 값 실시간으로 | `ros2 topic echo <토픽>` |
| 주기(Hz) 재기 | `ros2 topic hz <토픽>` |
| 가짜 값 쏘기(장비 대신) | `ros2 topic pub -r 10 <토픽> <타입> "{data: 값}"` |
| 이 노드가 뭘 주고받나 | `ros2 node info /<노드>` |
| 파라미터 보기 | `ros2 param get /<노드> <이름>` |
| 파라미터 목록 | `ros2 param list /<노드>` |
| 연결 관계 그림으로 | `rqt_graph` |
| 실행파일 목록 | `ros2 pkg executables <패키지>` |

---

## 3. 장비 없이 (지금 — 배·센서 오기 전)

**노드를 하나씩 띄워서 로직을 익히는 게 맞다.** 메인 launch 는 아직 의미가 없다(§5 참고).

```bash
ros2 run ship_direction ship_direction
```
LiDAR 가 없으니 페일세이프가 도는 게 **정상**이다. `Ctrl+C` 로 끈다.

**다른 터미널에서** 관찰:
```bash
ros2 topic echo /desired_angle
```

**가짜 데이터로 시험** — 장비 없이 노드를 흔들어 보는 방법이다:
```bash
ros2 topic pub -r 10 /candidate_angle std_msgs/msg/Float32 "{data: 90.0}"
```

**블랙박스로 기록** (`~/ssf_logs/*.csv` 에 40컬럼으로 쌓인다):
```bash
ros2 launch ssf_tools ssf_tools.launch.py
```
CSV 에 값이 들어오는지로 **배선이 맞는지**를 눈으로 확인할 수 있다.
(검증 예: `/imu/mag_heading` 를 쏘면 `imu_mag_heading` 컬럼이 찬다. 안 차면 이름이 어긋난 것이다.)

**🎨 HSV 색 튜닝** (2026-08-12 도구 재작성):
```bash
ros2 run color_shape_detector basic_image_subscriberhsv --ros-args --params-file $(ros2 pkg prefix color_shape_detector)/share/color_shape_detector/config/vision.yaml
```
마스크를 **보면서** 잡는다 — 흰 부분이 검출되는 영역이다.

| 조작 | 하는 일 |
|---|---|
| **트랙바** | H/S/V 하한·상한 조정 — **조작은 이것 하나뿐이다** |
| `c` / `n` | 다음 색 / 같은 색의 다음 범위(빨강은 2개) |
| **`p`** | **vision.yaml 에 붙여넣을 줄을 출력** |
| `r` / `q` | 원래값 복귀 / 종료(종료 시 전체 값을 찍어준다) |

🚨 **값은 `config/vision.yaml` 의 `hsv.*` 한 곳에만 있다.** 파이썬 코드에 없다.
   (예전엔 검출기 3개 파일에 따로, 게다가 **서로 다른 값**으로 박혀 있었다)

⚠️ **실내에서 잡은 값은 실외에서 안 맞는다.** 햇빛에서 S·V 가 통째로 달라진다.
   실내 튜닝은 '파이프라인이 도는지' 확인용이고, 최종값은 **실제 부표·실제 조명**에서 잡는다.

**📈 연습기간 색 실측 — 나갈 때마다 켜두면 된다** (2026-08-12 신설):
```bash
python3 tools/vision_probe.py --sec 600 --colors red,green --log-csv ~/ssf_logs/hsv_practice.csv
```
CSV 에 시간·부표 HSV(원형통계)·마스크 비율·화면 밝기가 쌓인다. **이어붙기라 매번 켜기만** 하면 된다.

🚨 **왜 한 번으로 안 되나** — 실내에서 잡은 값은 실외에서 안 맞고, 오전에 잡은 값은
오후 역광에서 안 맞는다. 수면 반사(윤슬)는 태양 고도에 따라 완전히 달라진다.
경기는 5회 도전이고 회차마다 시각이 다르다 → **시간대별로 모아야 한다.**

🚨 **부표만 보지 말 것.** 정말 중요한 건 부표의 HSV 가 아니라 **부표와 배경이 갈리는가** 다.
`mask_pct`(화면에서 그 색이 차지하는 비율)가 같이 기록되는 이유다 — 이게 커지면
물·하늘·방파제가 같이 잡히기 시작했다는 뜻이다.
(실내 검증에서 **창밖 나무가 초록으로 안 잡힌 것**이 초록 범위가 좋다는 근거였다)

🚨 **모은 값을 합집합으로 넓히지 말 것.** 모든 시각을 다 포함하도록 범위를 늘리면
그 넓어진 범위가 다른 것도 잡는다. 시간대별 프리셋(`hsv_morning.yaml` 등)으로 나누고
경기 시각에 맞는 걸 고르는 쪽이 낫다.

⚠️ **카메라 자동보정이 아직 '자동' 이다** (`enable_auto_white_balance`·`enable_auto_exposure`).
정지 장면 A/B 에서는 차이가 없었지만(ΔH 0.0), 실외에서 큰 물체가 드나들 때는 H 가 밀릴 수 있다.
연습 때 **밝은 물체를 화면에 넣었다 뺐다** 하며 `h_med` 가 움직이는지 볼 것.
밀리면 WB 고정(주간 5500~6500K)으로 바꾼다.

🚨 검출기 시험은 **대회 launch 말고 카메라+검출기만** 띄운다.
   전체 launch 를 켠 채 `/wp_mode` 를 손으로 쏘면 `north_goal_angle` 과 **발행자가 2개**가 되어
   값이 섞인다.

**순수 로직 테스트는 ROS 없이도 돈다** — 소싱조차 필요 없다:
```bash
python3 src/ship_direction/test/test_failsafe_logic.py
```

---

## 4. 장비 오는 날 — 실제 운용 순서 ⚠️

### ① 경기 모드 고정 (launch 전 **매번**)
```bash
sudo ~/ros2_ws/tools/boat_boot.sh
```
USB autosuspend 로 LiDAR/IMU/GPS 시리얼이 끊기는 **실재 이슈**를 차단한다. CLAUDE.md 7-3.
상태만 보려면 `--check` (sudo 불필요).

### ② 메인 launch — 터미널 1
```bash
ros2 launch launch_files launch_files.launch.py
```
이 하나로 다음이 전부 뜬다:

| # | 뜨는 것 |
|---|---|
| 1 | RPLIDAR A3 |
| 2 | `iahrs_driver` (→ `/imu/yaw_raw` 만. 보정 안 함 — CLAUDE.md 3-5) |
| 2b | `yaw_mux` (**`/imu/yaw` 의 단독 발행자**) |
| 3 | u-blox GPS |
| 4 | NTRIP (2초 지연) |
| 5·6 | `north_goal_angle`, `ship_goal_angle` |
| 7 | RealSense D455 (640x480x30) |
| 8 | 비전 검출기 3종 (gate·turn·dock, **상주 + 모드 게이팅**) |
| 9 | 미션 노드 4종 (`ship_gate`·`ship_dock`·`ship_turn`·`ship_back`) |
| 10·11 | `ship_direction`, `motor_control` |

### ③ 🚨 메인 launch 에 **안 들어 있는** 것 — 터미널 2·3

빠뜨리기 쉽다. 둘 다 따로 띄워야 한다.

블랙박스 + 헬스체크 (**빼먹으면 왜 실패했는지 알 방법이 없다**):
```bash
ros2 launch ssf_tools ssf_tools.launch.py
```

미션 모니터 — 지금 무슨 미션이고 배가 무슨 모드인가 (2026-08-10 신설):
```bash
ros2 run ssf_tools mission_monitor
```
**구독 전용이라 두 대에서 동시에 띄워도 안전하다.**
🚨 `healthcheck` 는 `/health_ok` 를 **발행**하므로 두 대에서 띄우면 안 된다(발행자 2개).

~~아두이노 통신(micro-ROS)~~ → **이제 필요 없다.**
**[2026-08-10 정정]** 여기에 `micro_ros_agent` 를 띄우라고 적혀 있었는데,
**micro-ROS 는 2026-07-25 에 폐기됐다**(→ 시리얼 브릿지 방식, `arduino/README.md`).
그대로 따라 하면 없는 패키지를 찾다가 시간만 버린다.
지금은 **`ssf_bridge` 가 메인 launch 에 들어 있어 자동으로 뜬다.** 따로 띄울 필요 없다.
따로 띄우고 싶을 때만:
```bash
ros2 launch ssf_bridge ssf_bridge.launch.py
```
⚠️ 포트는 udev 로 `/dev/ttyMEGA` 에 고정할 것. 없으면 **노드를 안 띄우고 건너뛴다**(launch 는 안 죽는다).

※ `basic_image_subscriberhsv` 는 HSV 튜닝 전용이라 일부러 launch 에서 뺐다. 튜닝할 때만 따로 띄운다.

### 🔍 파라미터가 **실제로 닿았는지** 검사 (2026-08-12 신설)

```bash
python3 tools/check_params.py
```
전체 launch 를 띄운 상태에서 돌린다. config yaml 의 값과 **노드의 실제 값**을 대조한다.

🚨 **왜 필요한가** — ROS2 는 선언 안 된 파라미터를 **조용히 무시한다.** 경고도 안 낸다.
실제로 `rgb_camera.controls.saturation: 120` 이 **존재하지 않는 이름**이라 넉 달간
아무 효과 없이 방치돼 있었다(08-12 발견). 같은 유형을 하루에 3건 잡았다.

| 표시 | 뜻 |
|---|---|
| 🚨 미선언 | 노드가 선언한 적 없는 이름 = **오타이거나 죽은 설정** |
| 🚨 불일치 | 선언은 됐는데 값이 다르다 = **yaml 이 안 실렸다**(launch 의 `parameters=` 확인) |
| ℹ️ 빈 배열 | yaml 의 `[]` 는 ROS2 가 타입을 못 정해 'not set' — 코드가 `or []` 로 받으면 무해 |

⚠️ **사각지대**: yaml 이 안 실렸는데 그 값이 **코드 기본값과 같으면 못 잡는다.**
"불일치 0" 은 "yaml 이 다 실렸다" 가 아니라 "실린 것과 다른 값이 없다" 는 뜻이다.

⚠️ **파라미터를 추가하거나 이름을 바꾼 뒤에는 반드시 한 번 돌릴 것.**

---

### ④ 출발 전 확인 — 터미널 4
```bash
ros2 topic hz /scan
```
```bash
ros2 topic echo /imu/yaw
```
```bash
ros2 topic echo /health_ok
```

### 🕹 "미션이 안 넘어가요" — 고장이 아니다 (2026-08-12 추가)

`wp_mode` 가 **0 에서 안 움직이고** 이 로그가 계속 뜨면 정상이다:

```
🕹 AUTO 대기 — RC 모드를 AUTO 로 넘기면 WP0 부터 자율 시작
```

**자율 시작 게이트**가 잡고 있는 것이다. RC 로 시작점까지 몰고 가는 동안 GPS 도착·타임아웃으로
웨이포인트가 새는 걸 막는다. **조종기를 AUTO 로 넘기는 순간 WP0 부터 시작**한다.

⚠️ 방위·거리·`/wp_mode` 는 **계속 발행된다**(비전 검출기가 죽지 않게). 전진만 막힌다.

**아두이노 없이 벤치에서 시험할 때**는 `/boat_mode` 가 아무도 안 내므로 영원히 대기한다.
둘 중 하나로 푼다:
```bash
ros2 topic pub -r 10 /boat_mode std_msgs/Int32 "{data: 2}"
```
또는 `north_goal_angle.yaml` 의 `require_boat_mode_auto: false`.
🚨 **후자를 대회 설정에 남기지 말 것** — 안전장치가 꺼진 채 나간다.

---

🚨 **`/health_ok` 가 `true` 인지 반드시 보고 출발한다.**
빨간불을 켜둔 채 출발하는 습관이 최악이다 — CLAUDE.md 7-2.
벤치 편의 스위치(`cog_require_reverse_gate` 등)가 대회 설정에 남아 있으면 여기서 `false` 로 잡힌다.

---

## 5. 지금 안 되는 게 **정상**인 것 (고장으로 오해하지 말 것)

배·센서가 오기 전에는 아래가 실패한다. 이걸 디버깅하면 시간만 버린다.

| 증상 | 이유 |
|---|---|
| 메인 launch 가 에러를 쏟음 | LiDAR/IMU/GPS/카메라가 없다. 지금은 `ros2 run` 으로 하나씩 띄우는 게 맞다 |
| `/dev/ttyLiDAR`, `/dev/IMU` 없음 | udev 규칙 미작성 (`ubuntu_setup.md` 8단계 — **장비 있어야 만든다**) |
| NTRIP 연결 실패 | 계정 미발급 (`ubuntu_setup.md` 9단계) |
| 시리얼 `Permission denied` | `dialout` 그룹 적용 전 — **로그아웃/재로그인** 필요 |
| `north_goal_angle` 단독 실행 시 `/geofence_state 구독자 0` ERROR | **설계된 경고다.** `ship_direction` 을 같이 띄우면 꺼진다 (CLAUDE.md 3-9 이중 방어) |

---

## 6. 빌드 — 매번 전체를 기다리지 말 것

전체 clean 빌드는 **약 5분 30초**(28패키지, i5-12450H 기준)다. 한 노드만 고쳤으면:

```bash
colcon build --symlink-install --packages-select ship_direction
```

**`--symlink-install` 덕분에 Python 노드는 소스만 고치면 재빌드 없이 반영된다.**
C++(`iahrs_driver`, `rplidar_ros`, `realsense2_camera`, `ublox`, `micro_ros_agent`)은 **반드시 재빌드**해야 한다.

빌드가 꼬였을 때(원인 불명의 실패):
```bash
rm -rf build install log && colcon build --symlink-install
```
⚠️ RAM 8GB 머신이면 `--parallel-workers 2` 를 붙인다. **이 노트북은 15GB 라 불필요**하다.

---

## 7. 자주 걸리는 것

| 증상 | 원인 / 해결 |
|---|---|
| 고쳤는데 반영이 안 됨 | **소싱을 다시 안 했다** (§0). 터미널 새로 열기 |
| `Package not found` | 소싱 안 됐거나 그 패키지 빌드가 실패했다. `ros2 pkg executables <pkg>` 로 확인 |
| 토픽이 조용함 (에러도 없음) | **이름 불일치를 의심한다.** `ros2 topic list` 로 실제 이름 확인 → CLAUDE.md 3-9 상수표 대조 |
| 한 토픽에 값이 번갈아 튐 | **발행자가 2개다.** `ros2 topic info <토픽>` 의 Publisher count 확인 |
| `colcon build` 가 `__pycache__` 로 실패 | `setup.py` 의 glob 이 `launch/*` 로 되어 있는 패키지. `launch/*.py` 로 고친다 (74b7d4b 에서 ntrip_client 수정됨) |
| 노드가 안 죽음 | 해당 터미널을 닫는다. `ros2 node list` 로 유령 확인 |

---

## 8. 익히는 순서 (권장)

장비 없이 로직을 익히는 가장 빠른 길:

1. `ros2 run ship_direction ship_direction` 띄운다
2. 다른 터미널에서 `ros2 topic echo /desired_angle` 로 출력을 본다
3. 또 다른 터미널에서 `ros2 topic pub` 으로 `/candidate_angle` 이나 `/scan` 을 **가짜로 쏜다**
4. 출력이 어떻게 바뀌는지 관찰한다
5. `ros2 node info /ship_direction` 으로 "무엇을 받아 무엇을 내는지" 표를 확인한다

토픽 계약의 정답은 **CLAUDE.md 3-9** 다. 이름·특수 신호 상수(`5000`/`6000`/`20000`/`50000`)를 여기서 확인한다.

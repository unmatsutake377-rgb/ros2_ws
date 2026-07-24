# SSF 자율운항선박 — 작년 코드 재활용 프로젝트

> **이 파일을 `~/ros2_ws/CLAUDE.md` 에 두면 Claude Code 가 매 세션 자동으로 읽습니다.**
> 지금까지 별도 분석에서 확정된 사실만 담았습니다. 추측은 "⚠️ 미확인" 으로 표시했습니다.

---

## 0. 프로젝트 한 줄 요약

작년(2025) KABOAT 대회에 실제로 나갔던 ROS2 코드를, **올해 새로 만드는 배 2척(A배/B배)** 에 재활용한다.
배는 **아직 안 만들어졌다.** 아두이노도 아직 없다. 그래서 **하드웨어 없이 검증 가능한 것부터** 고친다.

- ROS2 Humble / Ubuntu 22.04 / Python 3.10
- 빌드: `colcon build --symlink-install`
- 워크스페이스: `~/ros2_ws`

---

## 1. 🚨 작업 규칙 — 반드시 지킬 것

### 1-1. 한 번에 **노드 하나만** 고친다

통째로 갈아엎지 않는다. 이유:
- 문제가 생겼을 때 **어느 노드 때문인지 즉시 안다**
- 팀이 각 변경을 이해할 수 있다
- 롤백이 쉽다

**예외:** 비전 4종(`marker_detector` + `marker_selector` + `tracker` + `ship_gate`)은 **원자적**이다. 쪼개면 중간 상태가 깨진다.

### 1-2. 시작 전에 git

```bash
cd ~/ros2_ws
git init
git add -A
git commit -m "작년 대회 최종본 (수정 전 기준선)"
```

**노드 하나 고칠 때마다 커밋한다.** 커밋 메시지에 무엇을 왜 바꿨는지 적는다.

### 1-3. 토픽 이름을 바꾸지 않는다

기존 토픽 계약은 **그대로 유지**한다. 새 기능이 필요하면 **토픽을 추가**한다 (기존 것 개명 금지).
그래야 "옛 노드 + 새 노드" 가 섞인 중간 상태에서도 시스템이 돈다. (실제로 5/5 조합 정상 동작 검증됨)

### 1-4. 하드코딩 금지 — 파라미터로

배가 2척(A/B, 크기 다름)이다. 배 폭·PWM·거리 임계값은 **전부 `config/boat_a.yaml` / `boat_b.yaml`** 로 뺀다.
아직 모르는 실측값은 **`# ⚠️ 실측 필요` 주석**을 달고 임시값을 넣는다.

### 1-5. `time.time()` 쓰지 말 것

작년 코드는 전부 `time.time()`(벽시계)을 쓴다. NTP 보정이나 시간 점프에 취약하다.
**`time.monotonic()`** 을 쓴다.

---

## 2. 하드웨어 (확정)

| 항목 | 모델 | 비고 |
|---|---|---|
| LiDAR | **RPLIDAR A3** | 25m, **10Hz**, 0.225°, `Sensitivity` 모드, `/dev/ttyLiDAR` |
| IMU | **RB-SDA-v1** (IntelliThings iAHRS) | 9축 AHRS, ASCII `"e\n"` 프로토콜, `/dev/IMU`. **WT901C 아님** |
| GPS (A배) | u-blox **C94-M8P** | NTRIP RTK |
| GPS (B배) | u-blox **ZED-F9P** | NTRIP RTK |
| MCU | **Arduino Due** + micro-ROS | `Motor_run = pwm_r*10000 + pwm_l`, **1500 = 중립** |
| 카메라 | **RealSense D455 (현역)** / OAK-1 W POE (미구매) | 현재 D455 사용. OAK 는 RGB 전용(**뎁스 없음**)·광각 120~150° DFOV·PoE=이더넷. **코드는 중립화** 🚨 3-3 |

**⚠️ 실측 필요 (배 완성 후):** 선폭, 순항 PWM, 후진 PWM, IMU 장착 오프셋, 대회장 자기편각,
카메라 **왜곡계수(D)·초점거리(fx,fy)** — `camera_info` 에서, **고정 IP**

---

## 3. 검증된 버그 — 전부 실제 코드에서 확인함

### 3-1. 🚨 도킹 미션이 통째로 죽어 있었다

```python
# ship_dock.py
self.declare_parameter("active_wp_mode", 9)   # ← 웨이포인트의 도킹 모드는 7
```

**1년 내내 도킹 노드가 침묵했다.** 배는 멀쩡히 돌아다녔고, 그냥 도킹만 안 했다.
**침묵하는 노드는 에러를 내지 않는다.** 이게 이 구조의 근본 위험이다.

**고칠 것:** `active_wp_mode` 를 **7** 로. 그리고 아래 3-6 의 매핑표 검사를 넣을 것.
⚠️ **단, 이 수정은 단독으로 하면 안 된다.** `north_goal_angle` 의 mode-7 폴백과 **원자적 한 쌍**이다 (4장 **6a** 참고). 같은 커밋에서 함께 고칠 것.

### 3-2. 🚨 감속이 연결되어 있지 않았다

`ship_direction` 이 `/obstacle_distance_array` 를 발행하지만,
**`motor_control` 은 `/desired_angle` 하나만 구독한다.**

```python
# motor_control.py — 이게 전부다
self.sub_angle = self.create_subscription(Float32, "/desired_angle", self.angle_callback, 2)
```

**장애물이 코앞이어도 감속하지 않았다.** (`/obstacle_distance_array` 는 `ship_turn`·`ship_back` 만 받음)

**고친 것 (2단계 완료):** `motor_control` 이 `/obstacle_distance_array`(=`[거리(m), 각도(deg)]`, 없으면 `[inf,nan]`)를
구독하고 근접 시 감속. `slow_start_dist=1.2`, `min_speed_ratio=0.7`. 전진 성분(pwm<1500)만 중립 쪽으로 당김
(후진은 안 건드림). + 명령 워치독: `/desired_angle` 이 `cmd_timeout_sec` 넘게 끊기면 중립. 부팅 중립.

**✅ 명령 워치독은 0.5s 다 (3단계에서 3.5 → 0.5 로 조임).**
2단계 땐 3.5s 였다. `ship_direction` 이 `scan_cb` 에서만 `/desired_angle` 을 발행해서, LiDAR 가 잠깐만 끊겨도
`/desired_angle` 이 멈췄기 때문이다 — 짧게 잡으면 `ship_direction` 자체 페일세이프(0.7s 감속/3.0s 정지)를
덮어써 배를 죽였다(시뮬: LiDAR 1s 끊김→0.6s 정지).
**3단계에서 제어루프를 고정주기 타이머로 분리해 전제가 성립했다:** `ship_direction` 이 살아있는 한 항상 발행한다
(LiDAR 4초 끊겨도 STOP_HOLD 를 40회 계속 발행 — 검증됨). → **`/desired_angle` 침묵 = ship_direction 사망**
(LiDAR 끊김이 아니라). 그래서 0.5s 로 조였다.

**🚫 TTC 비상제동은 폐기했다.** `/obstacle_distance_array` 최소거리는 시간필터 없는 생값이라, 미분(접근속도)이
노이즈를 증폭한다. 물보라 반사 하나가 0.4m 로 튀면 접근속도 26m/s → TTC 0.015s → 급정지.
시뮬(물보라 10%, 6회): TTC OFF=접촉0·34.4m·급정지0 / TTC ON=접촉0·8.5m·급정지6.5·35초중 28초 정지. **이득 0, 위험 막대.**
감속(점진적)은 노이즈에 강해 유지, TTC(이진 급정지)만 버린다.

### 3-3. 🚨 카메라 **중립화** — RealSense 현역 유지 + OAK 준비 완료

> **[2026-07-23 정정]** 이전 판은 "카메라 = OAK-1 W POE **확정**", "`basic_image_*.py` **전부 삭제**",
> "`realsense2_camera` → `depthai-ros` **교체**" 라고 적혀 있었다. **셋 다 틀렸다.**
> **OAK-1 W PoE 는 아직 미구매(펀딩 대기)이고, RealSense 가 현역이다.**
> 그 상태에서 "전부 삭제" 를 실행하면 **지금 동작하는 유일한 비전 경로가 죽는다.**
> 목표를 **교체**가 아니라 **중립화**로 바꾼다 — 카메라 도착일에 남는 작업이
> **"yaml 수정 + HSV 재캘리브레이션" 뿐**이 되도록 지금 코드를 손본다.

**현 상태 (사실):**
| 항목 | 상태 |
|---|---|
| RealSense D455 | **현역.** `src/realsense-ros-ros2-master/` 유지 — COLCON_IGNORE 넣지 마라, 삭제도 금지 |
| OAK-1 W PoE | **미구매.** 도착 시점 미정 (RGB 전용·뎁스 없음, 광각 120~150° DFOV, PoE=이더넷) |
| `basic_image_*.py` | **삭제 금지. 현역 코드다.** 중립화 대상 |

**뎁스 문제는 그대로 유효하다 (그래서 지금 끊는다).**
작년 비전 노드 3종이 `/camera/camera/depth/image_rect_raw` 를 구독하고,
`color_callback` 맨 앞에 `if self.latest_depth is None: return` 가드가 있었다.
→ 뎁스 없는 카메라를 물리면 **에러 한 줄 없이 각도 토픽이 영원히 침묵**한다(침묵 사망).
RealSense 를 쓰는 **지금** 끊어야, 지금부터 쌓는 물 위 튜닝이 카메라 교체 후에도 유효하다.

**해법 (채택, 변경 없음):** **카메라는 방위각만, 거리는 LiDAR `/scan` 에서.**
- 부표: 카메라 방위 → 그 방위의 LiDAR 거리
- 도크: **매칭하지 말 것.** 카메라로 "어느 도크인가 + 어느 방향인가"만, 접안 거리는 **LiDAR 전방 섹터 최소거리**.
  (도크는 넓은 구조물이라 표식 방위와 LiDAR 최근접점 방위가 최대 15° 어긋난다 — 계산으로 확인됨)

**[V1 완료 2026-07-23] depth 의존 제거 + 거리 토픽 발행 중단.**
`basic_image_subscriber{gate,dock,turn}.py` 에서 제거한 것:
depth 구독 / `depth_callback` / `latest_depth` 가드 / 거리 계산 / 유효거리 필터(`1.0~6.0m`) /
`/red_distance`·`/green_distance`·`/image_distance` **발행**.
→ 거리 토픽 **소비자는 0개다.** 6단계에서 `ship_gate/dock/turn/back` 이 전부 `/scan` 으로 전환했다(주석만 잔존).
→ 각도 토픽(`/red_angle`, `/green_angle`, `/image_angle`)의 **이름·타입은 불변.**
→ 후보 선택 기준은 `distance` 최소 → **`area` 최대**(면적=거리의 대용).
→ 각도식은 **값이 완전히 동일**하다: 기존 `atan2((rel_x/80)*0.09*(d/0.5), d)` 는 `d` 가 약분되어
   `atan(rel_x * 0.00225)` 와 같다. 여러 거리(1/3/6m)·여러 픽셀에서 소수점까지 일치 확인.

**[V4 완료 2026-07-23] `debug_view` 게이트 + sensor QoS.**
- `cv2.imshow`/`waitKey`/`namedWindow` 를 전부 `debug_view` 파라미터로 감쌌다. **기본 false**
  (`subscriberhsv` 만 true — HSV 를 마우스로 읽는 게 존재 이유라 창이 없으면 할 일이 없다).
  배는 SSH 로 띄운다 = 헤드리스다. `imshow` 는 거기서 **예외를 던져 노드를 죽인다.**
  `try/except` 로 덮지 않고 파라미터로 원천 차단했다. `hsv` 는 `namedWindow` 가 `__init__` 에 있어
  **콜백 전에, 노드 생성 시점에** 터진다(원인 파악이 더 어렵다) — 그래서 거기도 게이트했다.
- false 면 `frame.copy()`(매 프레임 전체 memcpy)와 모든 그리기 연산을 건너뛴다. `view_frame=None`.
- 이미지 구독 QoS: `depth=10 기본 RELIABLE` → **BEST_EFFORT + KEEP_LAST + depth=1**.
  콜백이 밀리면 묵은 프레임 10 장이 큐에 쌓여 **몇 백 ms 전 장면으로 조향**한다. 늦은 프레임은 버린다.
  ※ 구독자 BEST_EFFORT 는 발행자가 RELIABLE 이어도 **호환**된다(그 반대가 비호환) → RealSense/OAK 둘 다 안전.
  ※ `/wp_mode` 는 센서가 아니라 **모드 명령**이라 한 장도 놓치면 안 된다 — RELIABLE 유지.
- `subscribermode` 에 `vision_debug_view` 추가 → 자식에게 `--ros-args -p debug_view:=` 로 전달.
  자식이 `ros2 run` 으로 뜨므로 launch 파라미터가 안 닿는다. 안 넘기면 **대회 실행 경로에서
  debug_view 를 켤 방법이 아예 없다.** (이 subprocess 구조 자체는 3-4 의 제거 대상 — 그때 같이 사라진다.)

**⏸ `/scan` 구독 QoS 는 아직 안 바꿨다 (판단 필요).**
`ship_direction`, `ship_dock`, `ship_gate`, `ship_turn`, `ship_back` **5곳 전부 `depth=10` 기본값**이다.
이미지와 똑같은 묵은-큐 문제가 있다(LiDAR 10Hz → depth 10 = **1초치**). 다만 `ship_direction` 은
페일세이프 경로라 QoS 변경이 스테일 판정에 어떻게 얽히는지 **측정 없이 바꾸면 안 된다.**

**[확정] 🚨 화각이 매직넘버에 박혀 있다 — 카메라 교체일의 시한폭탄.**
`k = (1/80)*0.09/0.5 = 0.00225` → **등가 `fx ≈ 444.4px`** → 640px 기준 **HFOV ≈ 71.5°**.
즉 **RealSense 화각이 하드코딩**돼 있다. OAK-1 W(광각)로 바꾸면 **같은 픽셀이 다른 각도**가 되어
모든 각도 출력과 상위 튜닝(`align_tol_deg`, `pair_min/max_sep_deg` 등)이 **통째로 무효**가 된다.
→ **V3 에서 명시형으로 재작성**: `fx = (msg.width/2) / tan(radians(hfov_deg)/2)`, `angle = -degrees(atan((vX-cx)/fx))`.
   `image_width` 는 파라미터가 아니라 **매 프레임 `msg.width`** 에서 읽는다(해상도 변경 자동 흡수).
   `hfov_deg` 기본값 = **71.5**(현 RealSense 역산 실측치). 환산은 순수 함수로 분리 + 테스트.
→ 광각 도입 시 추가로: **rectified 토픽 구독** 또는 `camera_info` 의 왜곡계수 `D` 를 `cv2.undistortPoints` 로 적용,
   `min_area_px` 재튜닝(각도 해상도가 낮아 먼 표식이 더 작게 보인다).

**[미확정] PoE = 이더넷 → 드라이버 병행.**
`depthai-ros`(`depthai_ros_driver`)는 **교체가 아니라 추가**다. RealSense 드라이버는 남긴다.
**고정 IP 등 네트워크 설정** 필요. 카메라 도착 전까지의 절차는 `docs/oak_arrival_runbook.md`(V5) 에 적는다.
연결 테스트는 **실물이 와야** 가능하다 — 이전 판의 "지금 노트북에 물려 테스트 가능" 은 미구매 상태에서 불가.

### 3-4. 🚨 `image_subscriber_mode` 는 subprocess 로 노드를 죽였다 살린다

```python
self._child = subprocess.Popen(["ros2","run",pkg,exe], preexec_fn=os.setsid)
os.killpg(os.getpgid(self._child.pid), signal.SIGINT)
```

- 모드 전환마다 **비전이 몇 초간 완전히 멈춘다**
- subprocess 가 좀비로 남으면 **카메라가 잠긴다**
- **추적기(tracker)를 쓸 수 없다** — 노드가 죽으면 트랙이 전부 날아간다

**고칠 것:** 노드는 **항상 살아서** 보이는 표식을 전부 발행하고, 목표 선택은 **파라미터**로 한다.
이건 타협 불가다. **문제는 `basic_image_*.py` 가 아니라 `image_subscriber_mode` 의 subprocess 방식**이다.

> **[2026-07-23 정정]** 이전 판은 여기서도 "`basic_image_*.py` 전부 삭제" 라고 했다. **폐기한다.**
> 그 파일들은 **RealSense 현역 경로**다(3-3 참고). 삭제 대상은 **`subscribermode` 의 subprocess 로직**이고,
> 검출 노드들은 **상시 상주 + 파라미터 목표선택**으로 **개조**한다. 삭제가 아니다.

### 3-5. 🚨 IMU 부팅 0점화 ↔ 절대방위 불일치

`iahrs_driver` 가 부팅 시 yaw 를 0 으로 만든다(`yaw_initial_offset`).
그러면 yaw 가 **상대각**이 된다. 그런데 `north_goal_angle` 은 **절대 방위**(정북 기준)를 계산한다.
**둘을 빼면 의미가 없다.**

**[2026-07-23 추가 발견] ③ GPS override 가 ② 보다 더 나빴다.**
```cpp
if (!std::isnan(gps_heading)) yaw_corrected = gps_heading;   // IMU 를 통째로 버림
```
- `NavPVT.heading` 은 msg 정의상 **"Heading of motion 2-D"** = **COG(대지침로)**. 뱃머리가 아니다.
- **정지·저속에서 COG 는 노이즈다.** 접안·정지유지(`ship_back`) 구간에서 yaw 가 튄다.
- **조류·바람에 게걸음하면 COG 와 뱃머리가 벌어진다.** 그 차이를 *추정*하는 게 옵션 B 인데,
  작년 코드는 차이를 **0 이라고 가정**해버렸다.
- `g_speed` 를 안 봤다 → **속도 0 에서도 통과.** `head_acc < 30°` 임계도 너무 헐렁하다.
→ 즉 `/imu/yaw` 는 대부분의 시간 **GPS COG 였다.** ② 의 상대각 문제는 그 뒤에 가려져 있었다.

**[N1 완료 2026-07-23] 드라이버에서 합성을 걷어내고 `yaw_mux` 로 옮겼다.**

| | 작년 | 지금 |
|---|---|---|
| `/imu/yaw` 발행자 | `iahrs_driver` | **`ssf_heading/yaw_mux` 하나뿐** |
| 드라이버 출력 | `/imu/yaw` (0점화+COG override) | **`/imu/yaw_raw`** (보정 없는 상대 yaw) |
| 부팅 0점화 | 항상 ON | `zero_yaw_on_boot` **기본 false** |
| GPS override | 항상 ON | `use_gps_heading_override` **기본 false** |

- 🚨 **드라이버 `yaw_topic` 을 `/imu/yaw` 로 되돌리면 한 토픽에 발행자 2개**가 된다. 에러는 안 난다.
  기본값을 `/imu/yaw_raw` 로 잡아 상류 launch(`iahrs_driver.py`, `iahrs_driver_launch.xml`)가
  파라미터를 안 넘겨도 안전하게 했다. `launch_files` 에는 일부러 명시해 뒀다.
- `heading_source` 로 A/B/C 를 갈아끼운다: `imu_relative`(현재 유일 구현) / `cog_offset`(B, N2) /
  `dual_gps`(A, N3) / `imu_absolute`(C, N4). **미구현 소스는 조용히 0 을 내지 않고 발행을 멈춘다.**
- 헤딩이 없으면 **발행하지 않는다.** 소비자가 이미 침묵을 처리한다 —
  `ship_goal_angle` 은 `/yaw_error` 를 멈추고, `north_goal_angle` 은 geofence 를 침묵시킨다.
- ⚠️ **자기편각은 자기북 기준 소스에만 적용된다.** 듀얼 GPS(relPosNED)와 COG 는 이미 진북 기준이라
  편각을 더하면 **이중 보정**(한국 기준 약 8° 오차)이다. `SOURCE_IS_MAGNETIC` 표로 강제하고 테스트로 못박았다.
- 진단: `/heading_status`(String) 발행. `healthcheck` 가 `/imu/yaw_raw` 를 함께 감시해
  **"IMU 죽음" 과 "yaw_mux 죽음" 을 구분**한다(로그만 — `/health_ok` 판정엔 안 넣는다, 거짓 정지 금지).
  `blackbox` 에 `imu_yaw_raw` 컬럼 추가 → N2 를 구현 **전에** CSV 로 타당성 검증 가능
  (COG 는 `gps_vel_x/y` 로 `atan2`).

**[N2 완료 2026-07-23] 옵션 B — COG 오프셋 추정 (`heading_source: cog_offset`).**
```
offset  = COG − imu_yaw_raw      (직진 중일 때만 표본)
heading = imu_yaw_raw + offset   (정지 중에도 IMU 가 유지 — 작년 override 가 튀던 그 지점)
```
- **각도 평균은 산술로 하면 안 된다.** 359°와 1°의 산술평균은 180°(정반대)다.
  단위벡터로 더한 뒤 `atan2` (원형통계). 신뢰도 R = |벡터합|/표본수 (0~1).
- 표본 배제 조건: `speed < 0.8m/s`(느리면 COG 는 노이즈 — 작년이 `g_speed` 를 안 봐서 당한 지점),
  `|선회율| > 8°/s`(선회 중엔 COG 가 뱃머리를 못 따라온다), IMU stale(짝을 못 지음).
- **후진하면 COG 가 180° 뒤집힌다.** 전·후진이 섞이면 R 이 무너져 `NOT_CONVERGED` 로 떨어진다.
  조용히 중간값(90° 틀림)을 내지 않는다 — 테스트로 박아뒀다.
- 수렴 전엔 **0.0 을 내지 않는다.** 0 은 '보정 없음' 처럼 보인다. `None` + `NOT_CONVERGED`(침묵).
- 🚨 **원리적 한계 — 숨기지 말 것.** COG 는 '간 방향' 이지 '뱃머리' 가 아니다.
  조류·바람에 게걸음하면 그 각도가 offset 에 **통째로 흡수**된다.
  즉 이 추정치 = **장착 오프셋 + 그날의 평균 게걸음각.** 물살이 일정하면 실용상 문제없다.
  절대 헤딩이 정말 필요하면 **옵션 A(듀얼 F9P)가 정답**이다. 테스트가 이 흡수를 명시적으로 검증한다.
- 🚨 **포화 함정 (구현 중 발견).** 감쇠가 있으면 표본수에 상한이 있다:
  `n_max = 1/(1 − 0.5^(dt/half_life))`. `cog_min_samples` 를 그 위로 잡으면
  **영원히 수렴하지 않는다 — 에러도 없이.** 예: GPS 10Hz + half_life 1s → n_max ≈ 14.9.
  `yaw_mux` 가 부팅 때 이 조합을 검사해 ERROR 로그를 낸다.
  현재값(5Hz, half_life 60s) → n_max ≈ 433 ≫ 30. 여유 충분. 수렴까지 직진 약 6초.
- 소스가 `cog_offset` 이 **아니어도 추정은 돌아간다.** `/heading_status` 에 `n / R / off / rej` 를
  실어 "지금 전환하면 쓸 만한가" 를 물 위에서 미리 볼 수 있다.
- ⚠️ `gps_vel_frame`(enu/ned) 을 틀리면 헤딩이 90° 돌거나 좌우가 뒤집힌다. **벤치 확인 항목.**

**⚠️ 아직 안 끝났다 — 벤치 확인이 필수다. 시뮬로 못 잡는다.**
`mount_offset_deg=0.0`, `invert_yaw=false` 는 **추측한 값이 아니라 "아직 안 쟀다"** 는 뜻이다.
절차 (`config/ssf_heading.yaml` 에도 적어둠):
1. 배를 나침반으로 **정북**에 맞춘다
2. `ros2 topic echo /imu/yaw` 가 **0** 을 읽을 때까지 `mount_offset_deg` 조정
3. 뱃머리를 **시계방향(동)으로 90°** → 값이 **90 으로 증가**해야 한다. 감소하면 `invert_yaw: true` 후 2 부터 재시작

**부호가 틀리면 배가 정반대로 간다.**

### 3-6. 🚨 `/candidate_angle` 에 6개 노드가 발행 — 구조적 결함

`north_goal_angle`, `ship_gate`, `ship_dock`, `ship_turn`, `ship_back`, `ship_last`

각 노드가 스스로 `/wp_mode` 를 보고 "내 차례면 발행" 하는 **분산 합의**다.
그런데 `active_wp_mode` 가 **6개 파일에 흩어져** 있어서 아무도 전체 표를 못 본다. → 3-1 사고의 원인.

**고칠 것 (구조는 그대로, 검사만 추가):**
- `wp_mode → 담당 노드` 매핑표를 **yaml 한 곳**에 모은다 (`ssf_tools/config/ssf_tools.yaml`)
- `healthcheck` 가 검사: **빠진 모드 / 중복 모드 / 침묵하는 노드**

#### 실제 웨이포인트 표 (`north_goal_angle.py` 에서 직접 발췌 — 이게 정답)

> ⚠️ **이전 판의 `mode 0,1 → ship_gate` 는 오류였다.** 코드를 읽어보니 `mode 0 → ship_last`, `mode 1 → ship_gate` 다. 아래가 실제다.

| WP | mode | 내용 | 담당 노드 (작년 선언값) | 상태 |
|---|---|---|---|---|
| 0 | **0** | 게이트 시작 | `ship_last` (0) | ⚠️ 게이트인데 비전을 안 쓴다 (GPS 로만 접근) |
| 1 | **1** | 게이트 끝 | `ship_gate` (1) | ✅ |
| 2 | **2** | 위치유지 | `ship_back` (2) | ✅ |
| 3,4,5 | **3** | 초록·빨강·하양 부표 | `ship_turn` (3) | ✅ |
| 6,7 | **5** | 회피 구간 (50초) | **없음 — 정상** | ✅ `ship_direction` 순수 회피 |
| 8,9 | **7** | 도킹 (60초) | `ship_dock` 은 **9** 로 선언 | 🚨 **침묵** (3-1) |
| 10 | **8** | 토너먼트 회피 | **없음 — 정상** | ✅ |

실제 `/wp_mode` 로 나오는 값은 **`{0,1,2,3,5,7,8}`** 뿐이다.

#### ⚠️ `mode 5`, `mode 8` 은 담당 노드가 없는 것이 **정상**이다

순수 장애물 회피 구간이라 `ship_direction` 이 GPS 방위로만 간다.
**healthcheck 가 이걸 '누락' 으로 경고하면 매 실행마다 늑대소년(거짓 경보)이 된다.** — 팀이 가장 두려워하는 것.
매핑표에 **`none`** 으로 명시하고 예외 처리한다. (`ssf_tools.yaml` 에 반영됨)

#### 미션 노드 교체 시 함께 고칠 것 (5·6단계)

- **`ship_dock` 의 `active_wp_mode` 를 9 → 7** (도킹 부활, 3-1)
- **`mode 0` 을 `ship_gate` 가 맡게** 한다 — 게이트 접근 구간부터 비전을 쓰면 정렬 시간을 번다.
  작년엔 `ship_last` 가 mode 0 을 잡고 GPS 폴백만 냈다. → **`ship_last` 제거**, 새 `ship_gate` 는 **`active_wp_modes: [0, 1]`**
  (이때 `ssf_tools.yaml` 의 매핑표도 `0 → ship_gate` 로 함께 갱신한다.)

### 3-7. 🚨 `ship_back` 이 자기를 `ship_turn` 이라고 등록한다 (이름 충돌 지뢰)

```python
# ship_back.py
super().__init__('ship_turn')   # ← 복붙 실수. 파일은 ship_back 인데 이름은 ship_turn
```

**노드 두 개가 같은 이름으로 뜬다.**

작년엔 launch 가 `name='ship_back'` 으로 **덮어써서 가려져 있었다.**
하지만 `ros2 run ship_back ship_back` 으로 **단독 실행하면 진짜 `ship_turn` 과 충돌한다.**

**올해는 더 위험하다.** 작년엔 **yaml 파라미터 파일이 아예 없었다**(전부 하드코딩).
올해는 `boat_a.yaml` / `boat_b.yaml` 을 **노드 이름으로 찾아 적용**한다.
이름이 틀리면 **파라미터가 엉뚱한 노드로 간다.** (`ship_back` 이 `ship_turn` 의 `active_wp_mode: 3` 을 받아버림)

**✅ 6d 에서 고침:** `ship_back` 의 `super().__init__('ship_turn')` → `'ship_back'`, 클래스명 `ShipTurn` → `ShipBack`.
**전 노드 전수 감사 완료** (super().__init__ 이름 vs 패키지명):
`ship_gate·ship_dock·ship_turn·north_goal_angle·ship_direction·ship_goal_angle·motor_control` 는 모두 일치했고,
**`ship_back` 만 어긋나 있었다** → 수정 완료.

### 3-8. 기타 확인된 것들

| 항목 | 문제 | 조치 |
|---|---|---|
| `ship_gate` | 규정은 **빨강-초록**인데 코드에 yellow 폴백 | ✅ 6b: yellow 제거 + 쌍 제약 + `/gates_passed` 카운트 + LiDAR 거리 |
| `ship_turn` | 부표를 **비껴 지나감** (규정은 **선회**) | ✅ 6c: orbit 기동으로 재작성 |
| `ship_turn` | 흰색 부표 인식 없음 | ✅ 6c: /buoy_color 로 빨강·초록=시계 / 흰색=반시계 |
| `ship_back` | **그냥 5초간 PWM 중립** | ✅ 6d: LiDAR 거리로 능동 위치유지(bang-bang+데드밴드) + 이름충돌 수정 |
| `ship_last` | `/candidate_angle` 에 20000 폴백만 발행 | ✅ 6b: **제거됨.** mode 0 은 ship_gate 가 인수 (발행자 둘 충돌 없이 하나→하나) |
| 전체 | **경계 이탈 방지(geofence) 없음** | 경기장 밖으로 나가면 실격. `north_goal_angle` 에 추가 |
| 죽은 토픽 | `/goal_distance`, `/wp_remaining_time`, `video_frames` | 아무도 안 받음 |
| 펌웨어 | `It_is_Aship` 오타 → **B배 분기가 죽어 있음** | ✅ 회로팀: `BoatId{A,B,FAULT}` ID핀 판별 + FAULT 처리 (`arduino/ssf_boat/ssf_boat.ino`) |
| 펌웨어 | 통신 끊겨도 **모터가 계속 돈다** | ✅ 회로팀: `ROS_TIMEOUT_MS=500` 워치독 → 중립. RC도 500ms |

**🔌 펌웨어(회로팀)는 `arduino/` 에 있다** (`ssf_boat.ino` + `COLCON_IGNORE`). colcon 은 안 건드린다(다른 툴체인).
**`Motor_run` 계약 검증 완료** — 펌웨어 디코딩(`r=data/10000, l=data%10000`, 1500=중립, **패스스루/리매핑 없음**)이
`motor_control.py` 인코딩과 일치. 토픽 `/Motor_run` 일치. 전/후진 방향은 펌웨어가 안 정하고 물리 배선이 정한다
→ **`steer_invert`(2단계) + 벤치 확인**의 대상. 펌웨어는 dumb passthrough 라 여기에 숨은 반전이 없다(좋다).
**4단계 상태:** 펌웨어(Arduino)측은 회로팀이 처리. **iahrs_driver 측(부팅 0점화 제거·절대방위)은 아직**(3-5) — 배·벤치 필요.

### 3-9. 🔒 신설 토픽 이름 확정 — 어긋나면 에러 없이 조용히 빈 값

신설 토픽은 **여러 단계에 걸쳐** 만들어진다. 발행자와 구독자가 **다른 단계**에서 태어나므로,
이름이 한 글자라도 어긋나면 **ROS2 는 에러를 내지 않고 그냥 아무것도 안 준다.** 지금 못 박는다.

| 토픽 | 타입 | 발행자 | 신설 단계 | 구독자 |
|---|---|---|---|---|
| `/health_ok` | `Bool` | `healthcheck` | **0단계** | (사람이 봄) |
| `/failsafe_level` | `Int32` | `ship_direction` | **3단계** | `blackbox`, **`motor_control`**(속도 상한) |
| `/gates_passed` | `Int32` | `ship_gate` | **5단계** | `blackbox` |
| `/geofence_state` | `Float32MultiArray` | `north_goal_angle` (**6a**) | **6a 발행 / 6a-2 구독** | `ship_direction` (**6a-2**) |
| `/buoy_color` | `String` | **비전(5단계)** | 6c 구독 | `ship_turn` (**6c**) |

**🚨 `/buoy_color`** (String: `"red"`/`"green"`/`"white"`): `ship_turn`(6c)이 회전 방향을 정하는 데 쓴다
(빨강·초록=시계, 흰색=반시계). **아직 발행자가 없다** — 5단계 비전이 발행해야 한다. 없으면 ship_turn 은
회전 방향을 몰라 SEARCH 에 머문다(틀린 방향으로 돌지 않는다). 작년엔 `/image_color` 였다(개명).

#### 🔒 `/geofence_state` 계약 — 경계를 **'가짜 LiDAR'** 로 낸다

`data = [angle_min_deg, angle_inc_deg, r0, r1, r2, ...]` — **상대방위**(0=정면) 격자. 멀면 그 방향은 `inf`.
정보 없으면 **빈 배열**(미설정 / IMU stale / 이미 이탈).

**소비 (`ship_direction`)는 스캔 병합 한 줄이 전부다:**
```
ranges[i] = min(real_ranges[i], geofence_ranges[i])
```
그 뒤는 기존 파이프라인이 알아서 한다 — `detection_distance` 로 마스크가 되고, `dilate` 가 배 폭 여유를 더하고,
갭-팔로잉이 피한다. **특수 로직도, 새 상태기계도, 튜닝 파라미터도 없다.**

**🚨 왜 '원뿔 칠하기'(half_block)를 버렸나 — 벽은 '점'이 아니라 '선'이다.**
최근접점 **한 방향**으로 ±각도 원뿔을 막는 건 기하학적으로 틀렸다. 40×40 경기장 `(40,40)` 모서리에서
배가 대각선(45°)을 향하면:

| | 상대방위 |
|---|---|
| 북벽 최근접점 | **−45°** |
| 동벽 최근접점 | **+45°** |
| **모서리(탈출구)** | **0° — 배 정면** |

`half_block=40°` 면 차단 구역이 `[−85,−5]` 와 `[5,85]` → **정면 0° 가 정확히 '틈'으로 열린다.**
배가 대각선으로 탈출한다. **실격.** (경기장이 사각형이 아니면 또 뚫린다.)
광선으로 쏘면 정면 경계 거리(**2.12m**, 코드로 실행 검증)가 그대로 벽이 된다.

**빈 배열을 내는 3가지 ('모르면 입을 다문다'):**
- **IMU stale**(`imu_stale_sec` 0.5s): 광선 **방향**이 yaw 에 통째로 의존한다. yaw 가 얼면 벽 전체가 엉뚱한 곳에 서고 배가 **'없는 벽'** 에 갇힌다.
- **이미 이탈:** 밖인데 경계를 벽으로 세우면 **돌아갈 길을 막는다.** ERROR 를 찍고 GPS 웨이포인트가 끌어당기게 둔다.
- **폴리곤 미설정 / GPS fix 없음.**
- (소비자 쪽) `ship_direction` 은 `geofence_stale_sec`(2.0s) 넘게 묵으면 병합하지 않는다.

⚠️ 이 병합은 **(C) 자율 회피 구간에서만** 일어난다. 미션 노드가 각도를 직접 지시하는 구간에서는 마스크를 만들지 않으므로 geofence 가 걸리지 않는다.

**🚨 발행자만 있고 구독자가 0 인 상태 = 경계 방어가 통째로 없는 것.** 도킹이 1년간 침묵한 것과 같은 사고다.
→ **이중 방어:** ① `north_goal_angle` 이 부팅 5초 뒤 구독자 0 이면 ERROR 를 계속 찍는다.
② `healthcheck` 가 출발 전에 검사해 `/health_ok = false` 로 만든다.
**✅ 6a-2 에서 `ship_direction` 이 구독을 붙여 빨간불을 껐다.** (빨간불은 하루도 켜두지 않는다 —
켜둔 채로 며칠 지나면 팀이 "저건 원래 빨간 거야"를 배운다. 출발 전 진단을 무시하는 습관이 최악이다.)

**소비 방식 (6a-2, `ship_direction`):** 경계가 `geofence_margin_m`(2.0) 안이면 그 방위 **±`geofence_half_block_deg`(40°)**
를 이진 마스크에 **1(장애물)로 칠한다** → 기존 갭-팔로잉이 알아서 피한다.
**별도 상태기계도 새 제어기도 만들지 않는다. 경계선을 그냥 '벽'으로 취급하는 것이다.**
`dilate` **전**에 칠해 팽창이 배 폭만큼 여유를 더한다.
⚠️ **코너 함정:** 모서리에서 두 경계가 동시에 잡히면 80°+80° 가 막혀 갈 곳이 없어질 수 있다 → 미션 시뮬 확인 필요.

**🚨 '모르면 입을 다문다' 가 3중으로 걸려 있다** (틀린 벽은 배를 가둔다):
- `north_goal_angle`: `/imu/yaw` 가 `imu_stale_sec`(0.5s) 넘게 묵으면 → `[inf, nan]`.
  (경계 상대방위는 yaw 로 계산한다. yaw 가 얼면 방위가 엉뚱해져 **'없는 벽'** 을 칠하게 된다.)
- `north_goal_angle`: 이미 경기장 **밖**이면 → `[inf, nan]` (칠하면 돌아갈 길을 막는다).
- `ship_direction`: `/geofence_state` 가 `geofence_stale_sec`(2.0s) 넘게 묵으면 → 칠하지 않는다.

**🚨 게이트 통과 수는 `/gates_passed` 다 (`/gate_pass_count` 아님).**
0단계 blackbox 가 처음엔 `/gate_pass_count` 로 구독했는데, 5단계 `ship_gate` 는 `/gates_passed` 로 발행할 예정이라
**그대로 뒀으면 게이트 통과 수가 영원히 빈칸이 됐을 것이다.** → 0단계에서 `/gates_passed` 로 정정 완료.

#### 🔒 `/candidate_angle`·`/desired_angle` 특수 신호 상수표 (값이 곧 계약)

이 값들은 각도가 아니라 **명령 코드**다. `north_goal_angle`·`ship_dock`·`ship_turn`·`ship_back`·`ship_direction`·`motor_control`
**6개 파일에 흩어져 각자 정의**돼 있어서 어긋났다(아래 SPIN 버그의 원인). 값을 여기 못 박는다.

| 값 | 의미 | 비고 |
|---|---|---|
| `5000.0` | **우선회 (SPIN_RIGHT)** | 작년 `ship_dock` 의 `RIGHT_SPIN=5000` 계승 |
| `6000.0` | **좌선회 (SPIN_LEFT)** | 현재 발행하는 노드 없음(예약) |
| `20000.0` | **미션 없음 (CANDIDATE_INVALID)** | → fallback 전진 |
| `50000.0` | **정지 (STOP_HOLD / STOP_VALUE)** | `ship_dock`·`ship_turn`·`ship_back`·`ship_direction` 에 각자 적혀 있음 |

**🚨 작년 SPIN 버그 2개 (2단계에서 고침):**
1. **방향 반대.** `ship_dock` 은 `RIGHT_SPIN=5000` "오른쪽 회전"으로 보내는데 `motor_control` 분기 (3) 은 **"왼쪽 선회"로 처리**했다(이름과 동작 반대).
2. **제자리 선회가 아니라 순항 속도로 원을 그렸다.** 측정: 전진 **1.20m/s**, 선회 **32.6°/s**, **반경 2.1m** → 360° 훑는 데 11초·**13m 이동**.
   도킹 탐색은 도크 코앞에서 하는데 `detection_distance_dock=0.8m` 라 `ship_direction` 이 도크를 장애물로 안 본다 → **도크에 부딪힌다.**

(도킹이 mode 9 로 죽어 있어 1년간 아무도 몰랐다.)

**고친 방식:** `5000→우선회 / 6000→좌선회` 분리 + SPIN 중심을 **`spin_forward_pwm`(기본 1500)** 으로 뺌
(1500 → 1400/1600 = 순 추력 0 = **진짜 제자리 선회** / 1360 → 작년 원 방식, 되돌릴 수 있는 노브. 물 위서 제자리 선회 안 되면 낮춘다).
물리 배선 반전은 **`steer_invert`(기본 false)** — 한 노브가 조향(4)·SPIN(3)을 함께 뒤집는다(배선 반대면 원인 하나). **물 위 첫 시험서 확정 — §8.**

**⚠️ 상수 공유 모듈화는 대회 전엔 하지 말 것.** 패키지 8개의 `package.xml`/`setup.py` 를 전부 건드려야 한다 —
얻는 건 깔끔함, 잃을 수 있는 건 **빌드 전체**다. **이 상수표 + `review_node.py` 검사로 충분하다.** (대회 후 정리)

---

## 4. 교체 우선순위 — 이 순서대로

> **원칙: 하류(下流)부터, 위험 낮은 것부터.**
> `motor_control` 은 체인의 끝이다. 여기를 먼저 고치면 상류에서 뭐가 오든 안전하게 받아낸다.

**⏱ 일정 (대회까지 4개월+, 배는 2주 내 완성 예정):**
- **3단계(시뮬로 검증 가능한 마지막)를 배 완성 전에 끝낸다.** — 지금이 시뮬 창구.
- **배가 뜨면** 실측·캘리브레이션·**조향 부호 확인**(§8, `steer_invert`)이 **최우선**.
- **5단계 비전**은 실물 카메라(OAK-1 W)·조명이 있어야 제대로 된다 → 배 이후.
- **비전 중립화(3-3)는 배 없이 지금 노트북에서** 가능 → 병렬로 진행. (OAK 실물 연결 테스트는 구매 후.)

| 단계 | 대상 | 왜 이 순서인가 |
|---|---|---|
| **0** | `ssf_tools` (blackbox + healthcheck) **신규** | **구독만 하고 발행 안 함 → 아무것도 못 깨뜨림.** 그런데 이후 모든 단계의 **검증 도구**가 된다. 먼저 눈을 확보한다. |
| **1** | `ship_goal_angle` | 변경이 가장 작다 (0.5초 → 0.05초 주기). 토픽 계약 불변. |
| **2** | `motor_control` | 체인의 **끝단**. 감속 복구 + 명령 타임아웃 워치독. (TTC 비상제동은 폐기 — 3-2 참고) |
| **3** | `ship_direction` | 회피·페일세이프. **가장 큰 이득** (접촉 3.0 → 0.2회). |
| **4** | `iahrs_driver` + Arduino 펌웨어 | **가장 위험.** 시뮬로 못 잡는다. **벤치 확인 필수.** |
| **5** | 비전 묶음 ⚛ (`marker_detector`+`marker_selector`+`tracker`+`ship_gate`) | **원자적.** 쪼개면 깨진다. `basic_image_*.py` 전부 삭제. |
| **6a** | ⚛ **`ship_dock` + `north_goal_angle`** (원자적 한 쌍) | **반드시 같은 커밋.** 아래 🚨 참고. 비전(5단계) 이후에 의미가 생긴다. |
| **6b** | `ship_turn`, `ship_back` (각각 독립) | 서로·6a 와 독립. 비전(5단계) 이후에 의미가 생긴다. |

### 🚨 6a 는 원자적이다 — `ship_dock` 과 `north_goal_angle` 을 쪼개면 도킹이 튄다

`north_goal_angle.py` 는 `timer_cb` 에서 **0.5초(2Hz)마다** 다음을 발행한다:

```python
self.create_timer(0.5, self.timer_cb)
...
if wp_mode == 7:
    self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))   # 20000
```

지금은 `ship_dock` 이 **9** 로 선언돼 mode 7 에 침묵하므로(3-1), `/candidate_angle` 발행자가 north_goal 하나뿐이라 충돌이 없다.
**`ship_dock` 만 9→7 로 고치면** 도킹 중 같은 `/candidate_angle` 에 **발행자가 둘**이 된다:
- `ship_dock` : 진짜 도킹 조향각
- `north_goal_angle` : `20000`(INVALID) 폴백 — 0.5초마다

→ **도킹 중 조향이 GPS 방위로 튄다** (제어 주기의 약 2%, 20초에 약 7회 전환). 접안 직전에 이러면 실패한다.

**반드시 같은 커밋에서 함께 고칠 것:**
- `ship_dock` : `active_wp_mode` **9 → 7**
- `north_goal_angle` : **mode-7 폴백 제거**. 폴백은 **담당 노드가 없는 `mode 5`, `8` 에만** 남긴다.
  (`if wp_mode == 7:` → `if wp_mode in (5, 8):`)

**0단계를 절대 건너뛰지 말 것.** 블랙박스가 없으면 이후 단계의 개선을 **측정할 수 없다.**
(규정상 종합임무 **5회 도전 가능, 최고점 채택** — 왜 실패했는지 알아야 다음 회차에서 고친다.)

---

## 5. 페일세이프 — 가장 조심할 것

**팀의 최우선 우려: "고장이 아닌데 스스로 고장이라 판단해서 배가 멈추는 것".**
경기 중 멈추면 그대로 끝이다. 그래서 **오탐지 방지가 최우선**이다.

4중 안전장치:
1. **`time.monotonic()`** — 벽시계 점프에 안 속는다
2. **ARMED 플래그** — 센서를 **한 번이라도 받은 뒤**에만 감시 시작 (부팅 중 오발동 방지)
3. **연속 N회 확인** (`failsafe_confirm_n: 3`) — 순간 지터로 레벨 안 올림
4. **히스테리시스 + 자동 복구** — 센서 돌아오면 스스로 풀림

임계값: `failsafe_warn_sec: 0.7` (감속), `failsafe_stop_sec: 3.0` (정지)
→ LiDAR 10Hz 기준 **30스캔 연속 누락**이어야 정지. 오탐지 거의 불가능.

**⚠️ 실제 버그 사례:** 페일세이프를 `scan_cb` 안에서만 평가하면, LiDAR 가 **완전히 죽었을 때 `scan_cb` 가 아예 안 불려서 페일세이프가 영원히 평가되지 않는다.** → **독립 타이머(`watchdog_cb`)** 로 평가해야 한다. (시뮬에서 실제로 발견)

**RC 두절 처리는 건드리지 않는다** (팀 결정). 자율/RC 구분이 최우선이고, RC 끊겨도 자율로 전환하거나 정지시킬 필요 없다.

---

## 6. 시뮬로 검증된 파라미터 — 임의로 바꾸지 말 것

| 파라미터 | 값 | 근거 |
|---|---|---|
| `clearance` | **0.25** | 스윕 결과: 0.20→접촉1.7회 / **0.25→0.2회** / 0.30→0.8회 / 0.45→**807초 폭주**(과보수) |
| `detection_distance_default` | **3.0** | 1.8 이면 8/8 충돌. 3.0 이면 5/8 통과 |
| `detection_distance_gate` | **2.0** | 3.0 이면 오히려 접촉 증가 (게이트가 좁아서) |
| `min_obstacle_cells` | **1** | 3 이면 작은 부표가 무시됨 |
| `temporal_frames/votes` | **1 / 2 = OFF** | 🚫 **효과 없음(철회).** 아래 참고 |
| `track_gate_deg` | **12.0** | 오탐 35% 환경에서 속는 비율 36% → 0.3% |

**`rear_obstacle_ignore_margin` 은 제거했다.** 물보라 0.3m 반사 하나가 실제 1.6m 부표를 **통째로 사라지게** 만들었다 (17셀 → 0셀).

### 🚫 효과 없어서 기본 OFF: 시간 투표 필터 (`temporal_frames`)

처음엔 효과가 있어 보였으나 **단일 시드로 판단한 착시**였다. 시드 20개로 재보니:

| | 접촉 | 전진 |
|---|---|---|
| 필터 OFF | 19/20 | 25.5m |
| 필터 ON | **20/20** | 23.4m |

해당 시나리오(정면 부표 + 물보라 60%)는 **어느 코드든 95% 접촉하는 '통과 불가' 판**이었다.
→ **무해하지만 무익.** 기본 `temporal_frames: 1`(OFF). **코드는 남겨둔다** — 실제 물보라의 시간 특성이
시뮬과 다를 수 있으니 켤 수 있게(`frames:3, votes:2`). 켤 때는 반드시 **dilate 전, 원본 마스크**에 건다.

> **`TTC` 는 '해로워서' 삭제했고(3-2), 이건 '무익해서' 기본 OFF다.** 둘을 구분할 것.

### 📏 측정으로 효과가 확인된 것 (유지)

| 항목 | 효과 |
|---|---|
| `failsafe_l1_speed: 0.7` (경고 시 감속) | 50판에서 **접촉 24 → 14회** |
| 제어 루프 타이머 분리 | LiDAR 1~3초 끊김 시 `/desired_angle` 발행 **0회 → 10~30회** |
| `rear_obstacle_ignore_margin` 제거 | 부표가 통째로 사라지던 결함 해소 |
| **`obst_median_kernel: 5`** (감속 신호 공간 median) | 물보라 30%에서 **가짜감속 36.8% → 8.9%**(기준선). 접촉은 전 조건 0 |

> **`obst_median_kernel` 은 감속 신호(`_closest_obstacle`)에만 건다.** 회피 마스크(`_compute`)는 안 건드린다 —
> 그래서 접촉 성능이 안 변한다(측정과 일치). ⚠️ 시뮬 물보라는 '단일점' 모델이라, 실제 물보라가 작은 군집이면
> 커널5로 부족할 수 있다 → **배 뜬 뒤 블랙박스 `obstacle_min_dist` 로 실측 확인 후 7/9 로 올릴 것.**

---

## 6-2. 🏁 경쟁팀(KABOAT 2025 상위권) 분석 판단 — 재론 금지

> 8개 팀 기술보고서 대조에서 나온 이식 후보 A~H. **결론: 6개는 이미 있거나 하지 말 것.**
> 상위권의 공통 교훈은 **"정교함 < 실전 검증"** — 우리가 TTC·시간투표를 측정으로 버린 것과 같은 원칙이다.
> (딥러닝 함정: YOLO+3D LiDAR 팀은 도킹 0점, 담백한 OpenCV 팀이 만점. 도킹 만점 팀은 **우리와 하드웨어가 판박이**이고
>  **우리 6a 설계와 같은 방법**(정면접근 FSM + 카메라 방위 + LiDAR 거리)으로 성공했다.)

| # | 아이디어 | 판단 | 근거 |
|---|---|---|---|
| **A** | 갭 선택에 목표방위 가중 | ✅ **이미 있음** | `sort(key=(angle_diff, -arc_length))` — 목표오차 최소 갭 우선, 동점 시 넓은 갭. 해미르와 동일. **최대갭 방식이 아니다** |
| **B** | 최소 갭폭 하드 제약 | ✅ **이미 있고 더 엄격** | `min_required_width = half_width*2 + clearance` = **1.15m** (해미르 0.6m). 할 일은 `half_width` 실측뿐 |
| **C** | 미디언필터 | ✅ **채택**(위 표) | DBSCAN 은 **안 함** — 갭팔로잉은 클러스터 정체성이 불필요 |
| **D** | 위치유지를 RTK 좌표홀드+PD 로 | ⏸ **배 뜬 뒤 판단** | 아래 참고 |
| **E** | 선회 중심 bbox∩LiDAR 매칭 | ✅ **이미 있음** | `ship_turn._lidar_dist_at()` = 카메라 방위 → LiDAR ±3° 최소거리. 클러스터 중심 정밀화는 배 뜬 뒤 |
| **F** | 도킹 표식 확인-N회 | ⏸ **5단계 후** | `marker_ok` 는 신선도만 검사(확인-N 없음). 비전 없이는 측정 불가. `SensorWatch.confirm_n` 패턴 재사용 |
| **G** | 게이트 복소 유체 속도장 | 🚫 **안 함 (과설계)** | 아래 참고 |
| **H** | `obstacle_detector` 패키지 도입 | 🚫 **안 함** | 외부 패키지 = 빌드 위험. 우리 검출은 시뮬 검증됨(접촉 24→14) |

### [D] 위치유지 — LiDAR 유지. RTK 는 폴백으로만. PD 는 미채택.

**규정은 "부표 5m 이내"** — 기준이 **부표**지 GPS 좌표가 아니다. LiDAR 는 그 규정 기준을 **직접** 잰다.
**부표는 계류줄로 묶여 바람·조류로 몇 m 움직인다** → RTK 좌표홀드는 "틀린 점을 정확히" 잡을 수 있다(구조적 한계).
실패 모드가 다르다: RTK 는 fix 저하 시 수 m 오차 / LiDAR 는 **부표가 안 보이면** 실패.

**🚨 PD 는 채택하지 않는다 — 아키텍처 제약:** 서울대 PD 는 **추력**을 낸다. 우리 체인은 `/candidate_angle` = **각도**다.
`motor_control` 은 전진/후진/정지뿐이라 추력 크기를 각도로 표현할 수 없다. PD 를 넣으려면 속도 토픽 신설 +
`motor_control` 인터페이스 변경 = **2·3단계에서 측정 완료된 체인을 대회 전에 뜯는 것**. §1 원칙 위반.

**⚠️ 답을 가르는 사실이 미지:** **LiDAR 가 부표를 보는가?**(§8 장착 높이). 배 뜨면 블랙박스 `obstacle_min_dist` 로 즉시 판정.
→ 못 보면 **bang-bang 은 그대로 두고 거리 소스만 GPS-홀드점 거리로** 교체(작은 변경, 인터페이스 불변).

### [G] 게이트 유체장 — 우리 쌍제약이 푸는 문제를 유체장은 못 푼다

실패 모드는 "**다음 게이트 빨강 + 이번 게이트 초록**이 함께 보임". 빨강 +Γ / 초록 −Γ 를 놓으면
**잘못 짝지어진 그 둘도 그대로 소용돌이를 만든다** — 쌍 판별을 안 하니까. 유체장은 "쌍이 맞다"는 전제 위에서
부드러운 유도를 줄 뿐이고, 부드러움은 이미 갭팔로잉이 담당한다. **검증된 로직을 새 수식으로 교체 = 순수 위험.**

---

## 7. 작업이 끝나면

각 단계를 마치면 **다음을 챙겨서 검토에 넘긴다:**

```bash
git diff HEAD~1 --stat          # 무엇이 바뀌었나
git diff HEAD~1                 # 어떻게 바뀌었나
colcon build --symlink-install  # 빌드는 되나
```

넘길 것: **변경된 노드 파일 + `git diff` + 바꾼 이유**
→ 별도 검토 환경에서 **토픽 계약 대조 + 혼합 호환성 시뮬 + 로직 테스트**를 돌린다.

---

## 8. ⚠️ 아직 모르는 것 (추측하지 말 것)

- 새 배의 **실제 선폭** → `half_width` 미정
- 새 배의 **추력** → `pwm_cruise`, `pwm_reverse` 미정
- **IMU 장착 오프셋**, **대회장 자기편각**
- **도크 규격** (공지 후 `contact_dist_m` 확정)
- **조향 좌/우 부호** — 배선이 반대면 **모든 게 무의미하다. 가장 흔한 사고.** 벤치에서 반드시 확인
- LiDAR **장착 높이 = 부표 높이** 인지 (안 맞으면 부표를 못 본다)

이 값들이 필요한 곳에는 **`# ⚠️ 실측 필요`** 주석을 남기고 임시값을 쓴다.

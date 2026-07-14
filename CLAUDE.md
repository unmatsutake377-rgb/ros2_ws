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
| 카메라 | **OAK-1 W POE 로 교체 예정** | 🚨 아래 3-3 참고 |

**⚠️ 실측 필요 (배 완성 후):** 선폭, 순항 PWM, 후진 PWM, IMU 장착 오프셋, 대회장 자기편각

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

**⚠️ 명령 워치독은 3.5s 다 (0.5s 아님).** 지금 `ship_direction` 은 `scan_cb` 에서만 `/desired_angle` 을 발행해서,
LiDAR 가 잠깐만 끊겨도 `/desired_angle` 이 멈춘다. 0.5s 로 잡으면 `ship_direction` 자체 페일세이프(0.7s 감속/3.0s 정지)를
덮어써 배를 죽인다(시뮬: LiDAR 1s 끊김→0.6s 정지). 3.5>3.0 이라 안 건드림.
**TODO(3단계):** `ship_direction` 제어루프를 `scan_cb` → **고정주기 타이머**로 분리 → 그러면 `ship_direction` 이
살아있는 한 항상 발행하므로 `/desired_angle` 침묵 = ship_direction 사망. 그때 `cmd_timeout_sec` 을 **0.5** 로 조인다.

**🚫 TTC 비상제동은 폐기했다.** `/obstacle_distance_array` 최소거리는 시간필터 없는 생값이라, 미분(접근속도)이
노이즈를 증폭한다. 물보라 반사 하나가 0.4m 로 튀면 접근속도 26m/s → TTC 0.015s → 급정지.
시뮬(물보라 10%, 6회): TTC OFF=접촉0·34.4m·급정지0 / TTC ON=접촉0·8.5m·급정지6.5·35초중 28초 정지. **이득 0, 위험 막대.**
감속(점진적)은 노이즈에 강해 유지, TTC(이진 급정지)만 버린다.

### 3-3. 🚨 카메라를 OAK-1 으로 바꾸면 미션 노드 4개가 죽는다

비전 노드 3종이 **전부 뎁스를 구독**한다:
```
/camera/camera/depth/image_rect_raw
```
**모든 거리값**(`/image_distance`, `/red_distance`, `/green_distance`)이 **RealSense 뎁스에서 나온다.**

**OAK-1 은 단안(mono) 이라 뎁스가 없다.** (뎁스는 OAK-**D** 계열)
→ `ship_gate`, `ship_dock`, `ship_turn`, `ship_back` 이 **전부 거리를 못 받아 죽는다.**

**해법 (채택):** **카메라는 방위각만, 거리는 LiDAR `/scan` 에서.**
- 부표: 카메라 방위 → 그 방위의 LiDAR 거리
- 도크: **매칭하지 말 것.** 카메라로 "어느 도크인가 + 어느 방향인가"만, 접안 거리는 **LiDAR 전방 섹터 최소거리**.
  (도크는 넓은 구조물이라 표식 방위와 LiDAR 최근접점 방위가 최대 15° 어긋난다 — 계산으로 확인됨)

### 3-4. 🚨 `image_subscriber_mode` 는 subprocess 로 노드를 죽였다 살린다

```python
self._child = subprocess.Popen(["ros2","run",pkg,exe], preexec_fn=os.setsid)
os.killpg(os.getpgid(self._child.pid), signal.SIGINT)
```

- 모드 전환마다 **비전이 몇 초간 완전히 멈춘다**
- subprocess 가 좀비로 남으면 **카메라가 잠긴다**
- **추적기(tracker)를 쓸 수 없다** — 노드가 죽으면 트랙이 전부 날아간다

**고칠 것:** `basic_image_*.py` 전부 삭제. 노드는 **항상 살아서** 보이는 표식을 전부 발행하고,
목표 선택은 **파라미터**로 한다. 이건 타협 불가다.

### 3-5. 🚨 IMU 부팅 0점화 ↔ 절대방위 불일치

`iahrs_driver` 가 부팅 시 yaw 를 0 으로 만든다(`yaw_initial_offset`).
그러면 yaw 가 **상대각**이 된다. 그런데 `north_goal_angle` 은 **절대 방위**(정북 기준)를 계산한다.
**둘을 빼면 의미가 없다.** (작년에 GPS heading 으로 yaw 를 덮어쓴 코드가 있는데, 이건 증상을 가린 것으로 보인다)

**고칠 것:**
- 부팅 0점화 제거
- GPS heading override 제거 (**뱃머리 방향은 IMU 가 맡는다**)
- 대신 파라미터 추가: `magnetic_declination_deg`(한국 약 -8°), `mount_offset_deg`, `invert_yaw`

**⚠️ 이건 시뮬로 못 잡는다.** 배를 정북으로 놓고 IMU 출력을 눈으로 읽는 **벤치 확인이 필수**다.
부호가 틀리면 **배가 정반대로 간다.**

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

**고칠 것:** `super().__init__()` 의 이름이 **파일명·패키지명과 일치**하는지 전 노드 확인.

### 3-8. 기타 확인된 것들

| 항목 | 문제 | 조치 |
|---|---|---|
| `ship_gate` | 규정은 **빨강-초록**인데 코드에 yellow 폴백 | 초록으로 |
| `ship_turn` | 부표를 **비껴 지나감** (규정은 **선회**) | orbit 기동으로 재작성 |
| `ship_turn` | 흰색 부표 인식 없음 | 빨강·초록=시계 / 흰색=반시계 |
| `ship_back` | **그냥 5초간 PWM 중립** | 조류 0.5m/s → 5초에 2.5m 밀림 → 실패. 위치 피드백 필요 |
| `ship_last` | `/candidate_angle` 에 20000 폴백만 발행 | 제거 (north_goal_angle 이 담당) |
| 전체 | **경계 이탈 방지(geofence) 없음** | 경기장 밖으로 나가면 실격. `north_goal_angle` 에 추가 |
| 죽은 토픽 | `/goal_distance`, `/wp_remaining_time`, `video_frames` | 아무도 안 받음 |
| 펌웨어 | `It_is_Aship` 오타 → **B배 분기가 죽어 있음** | BOAT_A=0 / BOAT_B=1 로 정리 |
| 펌웨어 | 통신 끊겨도 **모터가 계속 돈다** | 워치독 500ms → 중립(1500/1500) |

### 3-9. 🔒 신설 토픽 이름 확정 — 어긋나면 에러 없이 조용히 빈 값

신설 토픽은 **여러 단계에 걸쳐** 만들어진다. 발행자와 구독자가 **다른 단계**에서 태어나므로,
이름이 한 글자라도 어긋나면 **ROS2 는 에러를 내지 않고 그냥 아무것도 안 준다.** 지금 못 박는다.

| 토픽 | 타입 | 발행자 | 신설 단계 | 구독자 |
|---|---|---|---|---|
| `/health_ok` | `Bool` | `healthcheck` | **0단계** | (사람이 봄) |
| `/failsafe_level` | `Int32` | `ship_direction` | **3단계** | `blackbox` |
| `/gates_passed` | `Int32` | `ship_gate` | **5단계** | `blackbox` |
| `/geofence_state` | `Float32MultiArray` | `north_goal_angle` | **6단계** | `ship_direction` |

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
| `temporal_frames/votes` | **3 / 2** | 물보라 오탐 제거 (10Hz 기준 0.3초) |
| `track_gate_deg` | **12.0** | 오탐 35% 환경에서 속는 비율 36% → 0.3% |

**`rear_obstacle_ignore_margin` 은 제거했다.** 물보라 0.3m 반사 하나가 실제 1.6m 부표를 **통째로 사라지게** 만들었다 (17셀 → 0셀).

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

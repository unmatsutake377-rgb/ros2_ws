# 팀원 전달: `north_goal_angle 개정5` 검토 결과 + 병합 안내

> 받은 파일: `#north_goal_angle 개정5 기존 코드 업그레이드.ver.py`
> 검토 기준: 저장소 최신 `src/north_goal_angle/north_goal_angle/north_goal_angle.py` (커밋 이력 반영본)
> 결론 한 줄: **방향은 좋다. 다만 이 파일은 우리가 최근 이 노드에 넣은 안전 수정 이전 버전에서 갈라졌다.
> 그대로 합치면 그 수정들이 되돌아간다. 최신본을 base로 다시 떠서 새 기능만 얹어 달라.**

---

## 0. 왜 이 안내가 필요한가

개정5는 GPS·IMU·heading·Dead Reckoning을 크게 손봤습니다 — 아이디어 자체는 좋습니다.
그런데 **base로 삼은 `north_goal_angle`이 옛 버전**입니다. 우리가 그 사이에 **같은 파일**을
여러 번 고쳤는데(주로 안전 관련), 개정5에는 그 수정들이 없습니다.

그래서 개정5를 그대로 반영하면 **우리 수정이 사라지는(회귀)** 문제가 생깁니다.
이 문서는 "우리가 그 사이에 이 노드에 **무엇을 왜 고쳤는지**"를 먼저 설명하고, 그 위에서
개정5의 새 기능을 **어떻게 얹으면 되는지**를 정리한 것입니다.

---

## 1. 우리가 `north_goal_angle`에 넣은 수정 (개정5에는 없는 것)

개정5가 되돌리게 되는 항목들입니다. 하나씩, **왜** 넣었는지까지.

### 1-1. 🚨 mode 7 폴백 제거 — 가장 중요

**개정5 (L255-256, 그대로 살아있음):**
```python
if wp_mode == 7:
    self.pub_candidate.publish(Float32(data=CANDIDATE_INVALID))
```

**우리 최신본:** 이 코드를 **뺐습니다.** 폴백은 `mode 5, 8`(순수 회피 구간)에만 냅니다.

**왜:**
- 작년 `ship_dock`(도킹 노드)이 "내 담당은 `wp_mode == 9`"로 잘못 설정돼 **1년간 침묵**했습니다.
  실제 도킹 모드는 **7**입니다. 도킹이 안 되니, 배가 멈추지 않게 `north_goal_angle`이
  mode 7에서 대신 `CANDIDATE_INVALID(20000)`를 흘려보내는 **임시 땜빵**이 이 코드였습니다.
- 우리가 `ship_dock`을 **9 → 7**로 고쳤습니다(정상화). 그러면 mode 7일 때
  `ship_dock`과 `north_goal_angle`이 **둘 다 `/candidate_angle`에 발행** → **발행자 2개 충돌**.
  접안 직전 조향이 GPS 방위로 튑니다(시뮬에서 60초 중 35° 스파이크 20번).
- 그래서 `ship_dock` 수정과 이 폴백 제거는 **반드시 한 쌍**으로 가야 합니다.

> **개정5를 쓰면**: `ship_dock`(7)과 충돌이 되살아납니다. 이 줄은 반드시 빼 주세요.

### 1-2. Geofence(경계 이탈 방지) — 통째로 신규

**개정5:** 없음.
**우리 최신본:** `/geofence_state` 토픽을 새로 발행합니다. 경기장 경계를 **"가짜 LiDAR"** 로 만들어
`ship_direction`이 벽처럼 피하게 합니다.

**왜:**
- 경기장 밖으로 나가면 실격인데, 작년엔 경계 방어가 **0**이었습니다.
- 처음엔 "최근접 벽 방향으로 ±각도 원뿔을 막는" 방식을 생각했는데, **모서리에서 뚫립니다.**
  40×40 경기장 모서리에서 배가 대각선(45°)을 보면 두 벽의 최근접점이 ±45°인데
  **탈출구(모서리)는 정면 0°** 라 원뿔 사이 틈으로 그대로 빠져나갑니다.
- 그래서 경계선을 **각 방향으로 광선(ray)을 쏴서** "이 방향으로 몇 m 가면 경계 밖인가"를
  구하는 방식으로 바꿨습니다. `ship_direction`은 이걸 실제 LiDAR와 `min()`으로 합치기만 하면 됩니다.

> **개정5를 쓰면**: 경계 방어가 사라집니다. 이 기능은 유지돼야 합니다.

### 1-3. heading을 `north_goal_angle`에서 만들지 않음 — 구조가 바뀜

이게 개정5와 **가장 크게 충돌**하는 지점입니다. 자세히 설명합니다.

**개정5의 구조:**
- `/imu/data`(Imu, orientation quaternion)를 직접 구독해서 자체적으로 `imu_yaw`를 뽑고,
- `NavPVT.heading`으로 오프셋을 median 보정해서 `filtered_heading`을 만들고,
- 이걸 **Dead Reckoning(위치 추정)에 내부적으로** 씁니다.

**우리 구조 (N1에서 바꿈):**
- `north_goal_angle`은 heading을 **안 만듭니다.** `/imu/yaw`(보정된 절대방위)를 **구독만** 합니다.
- heading을 만드는 건 **`yaw_mux`라는 새 노드 하나뿐**입니다.

**왜 heading을 한 곳(`yaw_mux`)에 몰았나:**
- 작년엔 IMU 드라이버가 `/imu/yaw`를 직접 냈는데, 그게 **진짜 방위가 아니었습니다.**
  - 부팅 순간을 0으로 잡아 **상대각**이 됐고,
  - GPS가 유효하면 IMU를 버리고 **GPS 이동방향(COG)** 으로 통째로 덮어썼습니다.
    COG는 **배가 실제 간 방향**이지 **뱃머리 방향**이 아닙니다. 정지 중엔 노이즈로 튑니다.
- 그래서 IMU 드라이버는 **날것**(`/imu/yaw_raw`)만 내게 하고, 보정은 `yaw_mux`가 전담,
  `/imu/yaw`의 **발행자는 `yaw_mux` 하나**가 되도록 정리했습니다.
- **여러 노드가 각자 heading을 계산하면**, 노드마다 방위가 미묘하게 달라져 나중에
  "왜 이 노드는 이쪽, 저 노드는 저쪽으로 판단하지?"를 못 잡습니다.

> **개정5를 쓰면**: `north_goal_angle`이 다시 자체 heading을 만들게 되어, heading 소스가 둘로 갈립니다.
> heading 계산 로직은 `yaw_mux`로 옮겨 주세요(3장 참고).

### 1-4. 시계: `time.time()` → `time.monotonic()`

**개정5:** `time.time()`(벽시계) 사용.
**우리:** `time.monotonic()`.

**왜:** 벽시계는 NTP 시간 보정이나 시스템 시간 점프에 영향을 받습니다.
타임아웃·dwell 계산이 시간 점프 한 번에 엉킵니다. 경과시간 측정엔 항상 monotonic을 씁니다(프로젝트 규칙).

### 1-5. 하드코딩 → 파라미터

**개정5:** 임계값·타이머 주기·waypoints가 전부 코드에 박혀 있음.
**우리:** `config/north_goal_angle.yaml`로 뺌.

**왜:** 배가 **두 척(A·B, 크기 다름)** 입니다. 거리·주기 같은 값을 코드에 박으면 배마다 코드를
따로 관리해야 합니다. 설정 파일로 빼면 코드는 하나, 값만 배별로 다르게 둘 수 있습니다.

---

## 2. 개정5의 새 기능 평가 (이건 좋습니다)

되돌리는 것과 별개로, 개정5가 **새로 넣은 것**은 가치가 있습니다. 솔직하게 평가합니다.

| 기능 | 개정5 위치 | 평가 | 결론 |
|---|---|---|---|
| **Covariance 게이트** | `gps_cb` L131-134 | ✅ 싸고 안전. RTK가 풀려 오차가 커지면 그 값을 버림 | **이식 추천** (임계값은 실측으로) |
| **NavPVT heading offset** | `navpvt_cb` L109-122 | ⚠️ 우리 N2와 **개념이 같음** (아래 2-1) | **N2와 통합 검토** |
| **Dead Reckoning** | `timer_cb` L204-234 | ⚠️ 강력하지만 위험. 가속도 적분은 드리프트가 발산 | **실측 후 결정** |
| **Adaptive LPF / 동적 도착반경** | L168-179, L273-278 | ⚠️ 튜닝값. 근거 있는 시도지만 계수가 추측 | **실측 후 결정** |
| **ZUPT / 중력보상** | L207-218 | ✅ DR을 진지하게 만든 부분. DR을 쓴다면 함께 | DR과 세트 |

### 2-1. NavPVT heading — 우리 N2(옵션 B)와 겹칩니다

둘 다 **"GPS가 준 이동방향으로 IMU 방향을 보정"** 하는 **같은 아이디어**입니다.

- **개정5**: `NavPVT.heading`(Ublox 자체 계산) 사용, median(최근 10개), 속도 0.5m/s 게이트
- **우리 N2**: `fix_velocity`에서 COG 유도, 원형통계(단위벡터 평균), **후진 게이트**, 수렴 판정

**핵심**: `NavPVT.heading`은 우리가 N1에서 확인한 그 **COG(heading of motion)** 입니다.
소스가 사실상 같습니다. 다만 두 가지가 다릅니다:

1. **후진 함정**: 배가 후진만 계속하면 COG가 뱃머리와 **정확히 180° 뒤집힙니다.**
   개정5의 median은 이걸 못 막아서 방향을 정반대로 배울 수 있습니다.
   우리 N2는 `motor_control`의 `/motor_reverse`를 받아 후진 중엔 학습을 멈춥니다.
2. **위치**: 개정5는 `north_goal_angle` 안에서 DR용으로만 씀. 우리 N2는 `yaw_mux`에 있어
   `/imu/yaw`를 쓰는 **모든 노드**가 혜택을 받음.

> **제안**: NavPVT를 쓰고 싶으면 `yaw_mux`의 `heading_source`에 `navpvt` 옵션을 추가하는 방향으로.
> 후진 게이트는 우리 것을 유지. `fix_velocity` COG vs `NavPVT.heading` 중 실측으로 나은 쪽 채택.

---

## 3. 그래서 어떻게 합치면 되나 (실행 안내)

### 하지 말 것
- ❌ 개정5로 파일을 **통째 교체**. (geofence·mode7·monotonic·파라미터가 전부 되돌아감)

### 할 것
1. **base를 최신본으로 다시 뜬다.** 저장소의 현재 `north_goal_angle.py`를 받아서 시작하세요.
   (geofence + mode7 폴백 제거 + `/imu/yaw` 구독 + monotonic + 파라미터화가 이미 들어 있음)
2. **heading 계산 로직은 옮긴다.** 개정5의 `imu_cb` yaw 계산 + `navpvt_cb` offset 보정은
   `north_goal_angle`이 아니라 **`yaw_mux`(`src/ssf_heading/`)** 쪽 일입니다.
   NavPVT를 소스로 쓰고 싶으면 거기 `heading_source` 옵션으로 추가하고 **후진 게이트를 유지**하세요.
3. **새 기능만 최신본 위에 얹는다:**
   - **Covariance 게이트**: `gps_cb`에 이식. 임계 `2.0`은 F9P 실측으로 확정.
   - **DR / Adaptive LPF / 동적 반경**: 실측 데이터(blackbox CSV)로 필요성·계수 확인 후.
4. **규칙 두 개만 맞춘다:**
   - `time.time()` → `time.monotonic()` (경과시간 계산 전부)
   - 하드코딩 상수 → 파라미터 (`declare_parameter` + yaml)

### 우선순위 (실기 시작 후)
1. Covariance 게이트 (제일 싸고 안전)
2. NavPVT를 N2 소스 옵션으로 (후진 게이트 유지)
3. DR (GPS 공백이 실제 문제인지 CSV로 확인 후)

---

## 4. 왜 "지금 당장" 안 넣나 (참고)

우리 원칙이 **"측정으로 결정한다"** 입니다. 개정5의 새 기능 상당수는 **실기 검증 전**입니다:

- DR의 가속도 적분은 **실기 IMU 데이터 없이는 드리프트 특성을 모릅니다.** 짧은 GPS 공백엔
  유용하지만, 길어지면 배가 **엉뚱한 위치를 진짜라고 믿습니다.** 이건 조용히 위험합니다.
- Covariance 임계 `2.0`, LPF 계수, 동적 반경 범위는 F9P 실측 없이는 추측값입니다.

검증된 코드(우리 최신본)에 미검증 큰 변경을 덮는 건 위험합니다. 그래서 **아이디어는 채택하되,
실기가 오면 하나씩 측정하며 얹는** 방향을 제안합니다.

---

## 5. 요약 표

| 항목 | 개정5 | 우리 최신본 | 병합 방침 |
|---|---|---|---|
| mode 7 폴백 | 있음(버그) | 제거 | **최신본 유지** (빼기) |
| geofence | 없음 | 있음 | **최신본 유지** |
| heading 계산 | node 자체 | `yaw_mux`가 전담 | **`yaw_mux`로 이동** |
| 시계 | `time.time()` | `monotonic` | **monotonic** |
| 파라미터화 | 하드코딩 | yaml | **yaml** |
| Covariance 게이트 | 있음 ✅ | 없음 | **이식**(임계 실측) |
| NavPVT heading | 있음 | N2(fix_velocity) | **N2 소스 옵션으로 통합** |
| Dead Reckoning | 있음 | 없음 | **실측 후 결정** |
| Adaptive LPF/동적반경 | 있음 | 없음 | **실측 후 결정** |

궁금한 점 있으면 언제든. 정확한 기준은 저장소 `CLAUDE.md`(특히 3-1 도킹, 3-5 heading, 6a geofence, N1/N2)입니다.

---

## 부록. 코드 위치 지도 (실제 파일·줄번호)

> 위에서 설명한 우리 수정이 저장소 **어디에** 있는지. 줄번호는 작성 시점 기준이라 몇 줄 밀릴 수
> 있으니, 옆의 **검색어**로 찾는 게 확실하다. 전부 최신 `main` 브랜치 기준.

### 1) `src/north_goal_angle/north_goal_angle/north_goal_angle.py`
개정5가 base 로 삼은 바로 그 파일의 **최신본**. 여기가 핵심이다.

| 우리 수정 | 줄(약) | 검색어 |
|---|---|---|
| mode 7 폴백 제거 (5,8 에만) | L74, L250 | `FALLBACK_MODES` |
| ↳ 왜 뺐나 (주석) | L247–249 | `발행자가 둘이 되어` |
| geofence 발행 | L118 | `pub_geofence` |
| geofence 계산(가짜 LiDAR) | L158 | `def _geofence_ranges` |
| ↳ 광선-폴리곤 교차(순수) | L288 | `def _ray_polygon_dist` |
| ↳ 점-폴리곤 내부판정(순수) | L317 | `def _point_in_polygon` |
| geofence 파라미터 | L90–104 | `geofence_ray_min_deg` |
| `/imu/yaw` **구독만**(자체계산 안 함) | L122 | `self.create_subscription(Float64, '/imu/yaw'` |
| IMU stale 처리(모르면 침묵) | L110, L179–185 | `imu_stale_sec` |
| monotonic 시계 | L132, L150, L179, L223 | `time.monotonic` |
| 파라미터화(declare_parameter) | L87–110 | `declare_parameter` |

### 2) `src/ssf_heading/ssf_heading/yaw_mux.py` — heading 은 여기서 만든다 (N1)
개정5 가 `north_goal_angle` 안에서 하던 heading 계산(`imu_cb`+`navpvt_cb`)의 **정착지**.

| 항목 | 줄(약) | 검색어 |
|---|---|---|
| `/imu/yaw` **단독 발행** | L129 | `self.yaw_pub` |
| 드라이버 날것 `/imu/yaw_raw` 구독 | L61, L104 | `raw_yaw_topic` |
| heading 소스 선택(A/B/C) | L44 | `heading_source` |
| GPS 속도 구독(N2) | L65, L125 | `gps_vel_topic` |
| 후진 게이트 구독 | L150 | `update_reverse` |

### 3) `src/ssf_heading/ssf_heading/heading_logic.py` — 순수 로직(N2)
개정5 의 NavPVT offset 보정과 **같은 아이디어**가 여기 있다. 통합 시 여기 손댄다.

| 항목 | 줄(약) | 검색어 |
|---|---|---|
| COG 유도(속도벡터→방위) | L97 | `def cog_from_velocity` |
| COG 오프셋 추정기 | L143 | `class COGOffsetEstimator` |
| ↳ 후진 게이트 인자 | L192 | `reverse=False` |
| 소스 상수 | L33 | `SRC_COG_OFFSET` |
| 헤딩 믹서 | (검색) | `class HeadingMux` |

### 4) `src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp` — 드라이버 변경
개정5 가 `/imu/data` orientation 을 직접 쓰는 것과 관련. 우리는 드라이버가 **날것만** 내게 했다.

| 항목 | 줄(약) | 검색어 |
|---|---|---|
| 부팅 0점화 파라미터(기본 off) | L75 | `zero_yaw_on_boot` |
| GPS override 파라미터(기본 off) | L76 | `use_gps_heading_override` |
| 발행 토픽 = `/imu/yaw_raw` | L78, L82 | `yaw_topic` |

### 5) 설정 파일
| 파일 | 무엇 |
|---|---|
| `src/ssf_heading/config/ssf_heading.yaml` | heading 소스·COG·후진게이트·`gps_vel_frame` 파라미터 |
| `src/north_goal_angle/config/` | (있으면) waypoint·geofence·타임아웃 파라미터 |

### 6) 우리가 이 노드들에 왜 이렇게 했는지 — 근거 문서
| 문서 | 내용 |
|---|---|
| `CLAUDE.md` 3-1 | 도킹 mode 9→7 (mode 7 폴백 제거와 한 쌍) |
| `CLAUDE.md` 3-5 | IMU heading 문제(부팅 0점화 + GPS COG override) → yaw_mux |
| `CLAUDE.md` 6a | geofence 레이캐스트 설계(원뿔이 모서리에서 뚫린 경위) |
| `CLAUDE.md` N1/N2 | (3-5 안) heading 단독발행 + COG 오프셋 옵션 B |

> **찾는 순서 추천**: 먼저 `north_goal_angle.py`(1번)를 열어 개정5 와 나란히 놓고 비교 →
> heading 계산을 `yaw_mux.py`(2번)로 옮길 위치 확인 → `CLAUDE.md`(6번)로 "왜"를 확인.

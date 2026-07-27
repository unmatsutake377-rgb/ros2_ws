# 「AI_작업지시_수정사항.md」 코드 대조 검증 결과 (회신)

> **검증 기준:** `unmatsutake377-rgb/ros2_ws` 실제 코드 (커밋 `b91c45a` 시점)
> **범위:** 델타 문서(T1/T2/T8)만 검토. 원본 「AI_작업지시_프롬프트.md」는 미열람.
> **총평:** 대체로 정확하고 채택할 만함. 특히 **T2-3은 실제 치명 버그를 정확히 지목**했다.
> 아래는 ① 코드로 확인된 것 ② 보정이 필요한 것 ③ 문서가 남긴 열린 질문에 대한 답 ④ 기존 문서와의 충돌.

---

## 1. ✅ 코드로 확인됨 (근거 포함)

### T2-3 depth 가드 — **실재. 치명적. 최우선.**

`src/color_shape_detector/color_shape_detector/basic_image_subscribergate.py`

```python
# L25-35 : 구독 (color + depth, 둘 다 QoS depth=10)
self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
self.depth_sub = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)

# L71-74 : color 콜백 '맨 앞'
def color_callback(self, msg):
    if self.latest_depth is None:
        return          # ← OAK 엔 depth 가 없다 → 영원히 여기서 리턴
```

**영향:** OAK 연결 시 `/red_angle`·`/green_angle` 이 **영원히 침묵**한다. 게이트 미션 사망.
**에러 로그는 안 남는다** — 이 프로젝트가 반복해 당한 '침묵 실패' 유형(도킹 `active_wp_mode` 9 vs 7,
`/gate_pass_count` vs `/gates_passed`)과 동일. 지적이 정확하고 우선순위 1번이 맞다.

추가로 같은 파일 L192-196 의 depth 유효거리 게이트도 함께 걸린다:
```python
distance = depth_img[vY, vX] * 0.001
if not (1.0 <= distance <= 6.0): continue     # depth 없으면 전부 탈락
```

### T2-4 화각 하드코딩 — **실재. 수식으로 역산 확인.**

같은 파일 L198-201:
```python
rel_x  = vX - cx
real_x = (rel_x / 80.0) * 0.09 * (distance / 0.5)
angle_deg = -math.degrees(math.atan2(real_x, distance))
```

**`distance` 가 분자·분모에서 약분된다:**
```
real_x = rel_x × (0.09/80) × (distance/0.5) = rel_x × 0.00225 × distance
angle  = atan2(rel_x × 0.00225 × distance, distance) = atan(rel_x × 0.00225)
```
→ **유효 상수 k = 0.00225 → 등가 fx = 1/k ≈ 444.4 px → 640px 기준 HFOV ≈ 71.5°**
→ **RealSense D455 color(~69–71°) 값이 매직넘버에 박혀 있다.** OAK-1 W(120~150° DFOV)로 바꾸면
   **모든 각도 출력과 상위 튜닝(align_tol_deg, pair_min/max_sep_deg 등)이 무효**가 된다.
**지적의 실질은 정확하다.** (단, 표현은 §2-① 참고)

### T2-5 `cv2.imshow` — **실재. "4개 비전 노드" 정확.**

`imshow`/`waitKey` 출현: `subscribergate` 2, `subscriberhsv` 2, `subscriberdock` 2, `subscriberturn` 2, `subscribermode` **0**.
→ 게이트 대상 4개는 dock/gate/turn/hsv 가 맞다. 헤드리스에서 예외로 노드가 죽는 위험도 실재.

### T2-1 / T2-6 — 확인

- `src/realsense-ros-ros2-master/` 에 **COLCON_IGNORE 없음** → 현역 유지 상태. "넣지 마라" 는 현 상태와 일치.
- 이미지 구독 QoS 는 현재 **depth=10 (기본 RELIABLE)**. → BEST_EFFORT+depth=1(sensor-data QoS) 전환 타당.
  (참고: 우리 `ssf_tools/blackbox` 는 이미 BEST_EFFORT 관찰자 QoS 를 쓴다 — 같은 근거.)

---

## 2. ⚠️ 보정이 필요한 것 (3건)

### ① T2-4 표현 — "추출"이 아니라 "**재작성**"이다

문서 표현: *"픽셀 오프셋→각도 환산에 쓰이는 수평 화각을 `hfov_deg` 파라미터로 **추출**"*

**추출할 `hfov` 변수가 코드에 없다.** 매직넘버 3개(`80.0`, `0.09`, `0.5`)에 흩어져 있고,
게다가 `atan2(real_x, distance)` 형태라 **코드만 읽으면 "depth 기반 각도"처럼 보인다**(실제론 약분됨).

→ 구현자가 "hfov 상수를 찾아 파라미터로 빼면 되겠네" 로 접근하면 헤맨다. 지시를 이렇게 바꿔라:

```
픽셀→각도 환산을 명시형으로 재작성한다:
    fx = (image_width/2) / tan(radians(hfov_deg)/2)
    angle_deg = -degrees(atan((vX - cx) / fx))
기존 (rel_x/80.0)*0.09*(distance/0.5) + atan2 조합은 제거한다.
(그 조합은 distance 가 약분되어 실질 fx≈444px / HFOV≈71.5° 를 하드코딩한 것과 같다.)
hfov_deg 기본값은 현 RealSense 실측치(≈71.5)로 두고, 환산은 순수 함수로 분리 + 테스트.
```

### ② T1 지자기 로깅 — **드라이버가 발행하지 않는다. 선행작업 필요.**

`src/iahrs_driver_ros2-main/iahrs_driver/src/iahrs_driver.cpp` L58-59 기준, 발행 토픽은 **둘뿐**:
- `imu/data` (`sensor_msgs/Imu`) — **지자기 필드 없음**
- `/imu/yaw` (`std_msgs/Float64`)

**`sensor_msgs/MagneticField` 토픽이 없다.** 따라서 *"blackbox 로깅 항목에 raw magnetometer heading 추가"* 는
**현 상태로 불가능**하다. 문서가 "(가능 시)" 로 헤지했지만, 실무적으로는 순서가 이렇다:

```
(선행) iahrs 드라이버가 지자기/절대heading 을 발행하도록 수정  →  (그다음) blackbox 로깅 추가
```
이 선행 조건을 지시서에 명시할 것. 아니면 T1-C 는 "드라이버 조사" 단계에서 멈춘다.

### ③ T8 인과 단정 — **미검증 가설이다**

문서 표현: *"연산 부하는 문제가 아니다. **제어 지연을 만드는 것은 전원 관리다**"*

**우리는 제어 지연을 측정한 적이 없다.** 원인 단정의 근거가 없다.

다만 **조치 자체는 찬성**이다: USB autosuspend 가 시리얼 장치(LiDAR/IMU/GPS)를 끊는 건 **실재하는 리눅스 이슈**이고,
거버너/절전 고정은 **싸고 무해한 보험**이다. 표현만 이렇게 바꿔라:

```
"제어 지연의 원인이다" → "제어 지연을 유발할 수 있는 알려진 요인이라 미리 차단한다(원인 규명은 별도 측정)"
```

또한 T8-2 가 *"경고 로그만, 제어 개입은 하지 않는다"* 라고 못박은 것은 **우리 '거짓 정지 금지' 원칙과 일치한다 — 좋다.**
(healthcheck 가 온도로 `/health_ok=false` 를 만들면 출발 전 진단이 늑대소년이 된다.)

---

## 3. 📌 문서가 남긴 열린 질문 — **답변 가능**

> T2-3: *"거리 토픽 발행부의 처리 방침(센티널 유지 vs 발행 중단)은 **기존 소비자 코드를 확인해 결정**하고 근거를 주석으로 남긴다"*

**확인 결과: 소비자가 0개다.**

6단계에서 미션 노드를 전부 LiDAR 거리로 전환하며 카메라 거리 구독을 제거했다:

| 노드 | 이전 | 현재 |
|---|---|---|
| `ship_gate` | `/red_distance`, `/green_distance` 구독 | ❌ 제거 → `/scan` (부표=점물체, 방위 매칭) |
| `ship_dock` | `/image_distance` 구독 | ❌ 제거 → `/scan` 전방섹터 최소거리 |
| `ship_turn` | `/image_distance` 구독 | ❌ 제거 → `/scan` |
| `ship_back` | `/image_distance` 구독 | ❌ 제거 → `/scan` |

(현재 소스에 남은 문자열은 **주석뿐**이다.)

→ **결론: 거리 토픽은 그냥 발행을 중단해도 아무것도 안 깨진다.** 센티널 유지 고민 불필요.
   각도 토픽(`/red_angle`, `/green_angle`, `/image_angle`)의 이름·타입 불변 요구는 그대로 유효하다.

---

## 4. 🚨 기존 문서(CLAUDE.md)와의 충돌 — 우리 쪽을 고쳐야 함

T2 의 전제(**OAK-1 W PoE 미구매 / RealSense 현역**)가 사실이면, 우리 `CLAUDE.md` 3-3 이 틀렸다:

| CLAUDE.md 3-3 현재 서술 | T2 전제 기준 판정 |
|---|---|
| "카메라 = **OAK-1 W POE (확정)**" | ❌ 미구매 → "확정"이 아님 |
| "**[확정1] `basic_image_*.py` 는 전부 삭제한다**" | 🚨 **지금 실행하면 현역 장비가 죽는다** |
| "`realsense2_camera` → `depthai-ros` 로 **교체**" | ⏸ 교체가 아니라 **중립화 후 병행** |

→ CLAUDE.md 3-3 을 "**카메라 중립화**(RealSense 현역 유지 + OAK 준비 완료)" 로 정정 예정.
**단, 3-4(`image_subscriber_mode` 의 subprocess 로 노드를 죽였다 살리는 문제)는 카메라와 무관하게 여전히 유효**하므로 유지한다.

---

## 5. 제안 실행 순서

| 순위 | 항목 | 근거 |
|---|---|---|
| **1** | T2-3 depth 구독·가드 제거 (거리 소비자 0개이므로 안전) | 침묵 사망 차단. RealSense 현역 상태에서도 무해 |
| **2** | CLAUDE.md 3-3 정정 (확정 → 중립화) | 틀린 전제로 "전부 삭제" 실행 시 사고 |
| **3** | T2-4 각도 환산 **재작성** + `hfov_deg` 파라미터 + 순수함수 테스트 | 카메라 교체일에 물 위 튜닝이 통째로 무효화되는 것 방지 |
| **4** | T2-5 `debug_view` 게이트, T2-6 QoS depth=1 | 비용 낮고 효과 확실 |
| **5** | T2-2 `image_topic` 파라미터화, T2-8 OAK 도착 런북 | 도착일 작업량을 yaml+캘리브로 축소 |
| **6** | T8 `boat_boot.sh` (원인 단정은 보류, 보험으로 채택) | 싸고 무해. USB autosuspend 는 실재 이슈 |
| **7** | T1 지자기 | **드라이버 선행작업 필요**(§2-②) |

---

## 부록. 검증에 사용한 근거

- `basic_image_subscribergate.py` L25-35(구독/QoS), L71-74(depth 가드), L192-196(거리 게이트), L198-201(각도)
- `imshow`/`waitKey` 출현 수: gate 2 / hsv 2 / dock 2 / turn 2 / mode 0
- `iahrs_driver.cpp` L58-59 (발행 토픽 2개, MagneticField 없음)
- `src/realsense-ros-ros2-master/COLCON_IGNORE` 부재
- 거리 토픽 소비자 검색: `ship_*`, `north_goal_angle`, `motor_control`, `ssf_tools` 전수 → 실코드 0건(주석만)
- 각도 상수 역산: `k=(1/80)*0.09*(1/0.5)=0.00225`, `fx=1/k≈444.4px`, `HFOV=2·atan(320k)≈71.5°`

# 회신: 상위 3팀 분석 (GPS·IMU 분리 / 진동억제) 검토 결과

> 받은 것: `상위 3팀 분석 후 우리팀에 접목할 점.docx`
> 검토 기준: 저장소 최신 `iahrs_driver.cpp`, `motor_control.py`, `ssf_heading/heading_logic.py`
> 한 줄 결론: **지적1은 이미 고쳐진(그리고 더 나아간) 상태이고, 지적2는 유효합니다 — 다만 "있다/얼마나"는 우리가 blackbox로 실측한 뒤 대응합니다.**

조사 자체가 좋았습니다. 특히 세 상위팀에서 **같은 원칙 두 가지**를 뽑아낸 건 정확한 접근입니다.

---

## 0. 먼저 — base 버전 안내 (GPS 개정5·ship_direction 2.12 때와 같은 상황)

지적1에 인용된 `iahrs_driver.cpp` 코드는 **우리 최신본이 아닙니다.** 인용하신 건

```cpp
if (!std::isnan(gps_heading)) { yaw_corrected = gps_heading; }   // 무조건 덮어씀
```

이지만, 저장소 최신본은 이미 이 문제를 파라미터로 빼고 **기본 OFF** 해둔 상태입니다(아래 §1.2).
→ 다음 조사는 저장소 최신 코드를 base로 떠주시면 이런 어긋남을 막을 수 있습니다.

---

## 1. 지적1 — GPS·IMU 역할 분리 → ✅ 이미 반영 (게다가 더 나아감)

### 1.1 지적은 원칙적으로 100% 맞습니다

상위 3팀 원칙 = **위치는 GPS, 방향은 IMU. GPS heading이 IMU를 대체하면 안 된다.**
근거(COG는 뱃머리가 아님 / 정지·저속 노이즈 / 게걸음 시 벌어짐)도 정확합니다.

### 1.2 그런데 우리 최신 코드는 이미 그 구조입니다

`iahrs_driver.cpp` 현재 상태:

```cpp
// 기본 OFF — 파라미터를 켜야만 override 발생
use_gps_heading_override = declare_parameter<bool>("use_gps_heading_override", false);
...
if (node->use_gps_heading_override && !std::isnan(gps_heading))   // 기본값에선 진입 안 함
    yaw_corrected = gps_heading;
```

- 드라이버는 **보정 안 한 상대 yaw만** 발행합니다.
- 절대방위 합성은 `ssf_heading/yaw_mux`가 전담합니다 = **역할 분리(상위팀 구조) 그대로.**
- 코드 주석에 상위3팀과 **같은 근거**가 이미 적혀 있습니다 (CLAUDE.md 3-5):
  "NavPVT.heading은 COG(대지침로)지 뱃머리가 아니다. 정지/저속에서 노이즈고, 조류·바람에
   게걸음하면 뱃머리와 벌어진다."

### 1.3 우리는 제안하신 "방안 A"보다 한 발 더 갔습니다

| 단계 | 방향 처리 |
|---|---|
| 옛날 버그 | heading = COG 라고 **가정**하고 IMU를 통째로 덮어씀 |
| 제안하신 방안 A | GPS 방향을 **완전 배제** (IMU 드리프트 감수) |
| **우리 현재** | 기본 배제(=방안 A) **+ yaw_mux(N1) + COG 오프셋 추정(N2)** |

우리 방식은 게걸음 각(뱃머리 ↔ 진행방향 차이)을 **가정하지도 배제하지도 않고 추정**합니다.
즉 방안 A의 안전함은 그대로 확보(override 기본 OFF)하면서, 그 단점(IMU 드리프트)을 GPS로
보정할 여지를 남겨둔 구조입니다.

### 1.4 조치

**코드 변경 없음.** `use_gps_heading_override` 파라미터는 실험용으로만 남겨둔 것이고
기본값이 이미 여러분이 권한 "완전 배제" 동작과 같습니다.

---

## 2. 지적2 — 진동억제(slew/미분항) 부재 → ⚠️ 유효 (실측 후 대응)

### 2.1 지적이 맞습니다 — 조향/PWM 계층에 진동억제 없음

`motor_control.py` 확인 결과 그대로입니다:

```python
def linear_diff(self, offset):        # 순수 비례(P)
    off = max(-self.max_angle, min(self.max_angle, offset))
    diff = (abs(off) / self.max_angle) * self.max_diff
    return int(diff)
# timer_callback 이 매 프레임 계산값을 그대로 publish. prev_pwm·slew·D항 전부 없음.
```

상류도 이걸 덮지 못합니다(우리가 확인함):

- ship_direction의 median·lock·confirm = **장애물·페일세이프 노이즈**용이지 조향출력 평활화 아님
- heading_logic의 `max_turn_rate_dps` = **COG 오프셋 추정 시 표본 배제**용이지 조향 rate 제한 아님
- 명령 워치독(0.5s) = 신호 끊김 → 중립. 진동억제와 무관.

→ **조향/PWM 계층엔 진동억제가 실제로 없습니다.** 지적2는 옛날 코드 문제가 아니라 현재 gap입니다.

### 2.2 다만 — "있다/얼마나"는 우리가 blackbox로 실측합니다 (측정으로 결정, 추측 금지)

방금 blackbox에 진동 판정에 필요한 열이 다 들어갔습니다. 배 뜨면 이 3개로 확정합니다:

| 열 | 진동의 증거 |
|---|---|
| `desired_angle` | 프레임 간 점프 크기 (입력이 튀나) |
| `pwm_l` / `pwm_r` | 좌↔우 **부호 반전 반복** = 조향 왕복 = 진동 |
| `imu_yaw` | 목표 근처 좌우 오버슈트 왕복 |

**진동 감지는 우리 몫입니다.** 여러분 조사는 "이런 gap이 구조적으로 있다"까지 정확히 짚었고,
"실제로 우리 배에서 얼마나 일어나나 / slew 한계값을 얼마로 잡나"는 데이터가 있어야 정해집니다.

### 2.3 실측에서 진동이 확인되면 — 방안 A 채택. 단, 팀원 코드 그대로는 버그

제안하신 **방안 A(프레임 간 PWM 변화량 제한)** 가 방향은 맞습니다. 순수 로직이라 배 없이
Mac에서 테스트도 됩니다(우리 다른 모듈들과 같은 방식). **한 가지만 고쳐야 합니다:**

> 제안하신 `rate_limit`을 publish 직전에 **무조건** 걸면 **모드 경계에서 깨집니다.**
> 전진 → SPIN 전환 시 중간 PWM이 끼어 "반쯤 선회"하게 됩니다.

감속 로직이 이미 `slow_ok` 플래그로 SPIN·STOP·후진을 제외하는 것과 **똑같은 carve-out**이
필요합니다. slew는 **정상 조향(구간4) 안에서만** 적용해야 합니다.

```python
# 팀원 방안 A (모드 무시 — SPIN 전환 시 깨짐):
pwm_r = self.rate_limit(pwm_r, self.prev_pwm_r)   # ← 무조건

# 고친 형태 (감속의 slow_ok 와 같은 carve-out):
if steer_mode:      # 구간4(정상 조향)일 때만. SPIN/STOP/후진 제외.
    pwm_r = self.rate_limit(pwm_r, self.prev_pwm_r)
    pwm_l = self.rate_limit(pwm_l, self.prev_pwm_l)
```

그리고 `max_pwm_step` 값은 **blackbox로 측정한 실제 진동 크기에 맞춰** 튜닝합니다(임의값 X).

### 2.4 방안 B(PD)는 비추천

여러분도 단점에 적으셨듯 "미분항은 노이즈에 민감해 흔들림을 증폭"합니다. 우리 배는
물보라 노이즈가 심해(ship_direction의 median 필터가 그래서 존재) D항이 위험합니다.
방안 A로 충분히 잡히면 B는 안 갑니다.

---

## 요약

| # | 지적 | 판정 | 조치 |
|---|---|---|---|
| — | iahrs_driver base | 최신본 아님 | 다음엔 저장소 최신 base |
| 1 | GPS·IMU 분리 | 옛날 코드 기반. 이미 고침(+더 나아감) | 코드 변경 없음 |
| 2 | 진동억제 부재 | **유효(현재 gap)** | 우리가 blackbox 실측 → 진동 확인되면 모드-aware slew(방안 A) |

**두 지적 다 좋았습니다.** 지적1은 우리 설계가 상위팀과 같은 방향임을 재확인해줬고,
지적2는 실제 gap을 정확히 짚었습니다. 지적2는 배 뜬 뒤 실측(blackbox)에서 진동을 확인하고,
확인되면 모드 경계를 살린 방안 A로 대응하겠습니다.

궁금한 점 있으면 언제든. 정확한 기준은 저장소 코드와 CLAUDE.md 3-5(GPS override)·페일세이프 절입니다.

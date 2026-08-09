# N4 mag_heading B (독립 생-자기계 방위) — 준비 문서

> 상태: 준비(알고리즘 검증 완료) · 착수 조건: iAHRS raw mag 시리얼 명령 확보
> 관련: `회신_N4_mag_heading_검토.md`, `iahrs_driver.cpp`(현재 A안 발행 중)

---

## 0. A안 vs B안 다시

- **A안 (현재 발행 중, `b433662`)**: `imu_yaw_cw` = 센서 융합 yaw 를 `/imu/mag_heading` 로 냄.
  `zero_yaw_on_boot` OFF 면 `/imu/yaw_raw` 와 값이 사실상 같아 **중복**이다. N4 목적(지자기가
  GPS 와 얼마나 맞나 독립 검증)엔 한계가 있다.
- **B안 (이 문서)**: iAHRS 의 **생 자기계 벡터(mag x/y/z)** 를 읽어 roll/pitch 로 tilt 보상하고
  편각을 더해 **독립 지자기 방위**를 계산한다. 융합 yaw·GPS 와 **독립된 세 번째 신호**라 대조가
  의미 있다.

→ B 가 준비되면 `/imu/mag_heading` 을 B 값으로 바꾸거나(권장, A 는 어차피 yaw_raw 중복),
  `/imu/mag_heading_raw` 로 따로 낸다. blackbox 는 이미 `imu_mag_heading` 컬럼이 있어 그대로 로깅된다.

---

## 1. ✅ 알고리즘 — 검증 완료 (착수 시 이걸 C++로 옮기면 됨)

tilt 보상 지자기 방위(표준 NED, x 전방·y 우·z 하). **Mac 에서 검증함:**

```python
def tilt_compensated_heading(mx, my, mz, roll_deg, pitch_deg, declination_deg=0.0):
    phi   = radians(roll_deg)    # roll (x축 회전)
    theta = radians(pitch_deg)   # pitch (y축 회전)
    Xh = mx*cos(theta) + my*sin(phi)*sin(theta) + mz*cos(phi)*sin(theta)  # 수평 북성분
    Yh = my*cos(phi) - mz*sin(phi)                                        # 수평 동성분
    hdg = degrees(atan2(-Yh, Xh))
    return (hdg + declination_deg + 360.0) % 360.0
```

**검증 결과 (레벨 + 기울임):**
| 입력 | 결과 | 기대 |
|---|---|---|
| 자북=전방(mx+) | 0.0° | 0 |
| 동(my−) | 90.0° | 90 |
| 서(my+) | 270.0° | 270 |
| 남(mx−) | 180.0° | 180 |
| 북 + 편각 8° | 8.0° | 8 |
| **pitch 30° 기울임** | 0.0° (레벨과 동일) | tilt 보상 확인 |

**⚠️ 축·부호 규약은 iAHRS 데이터시트로 확정해야 한다.** 위는 표준 규약이고, iAHRS 의 mag x/y/z
방향·부호가 다르면 `mx/my/mz` 매핑과 `atan2` 부호를 그 규약에 맞춘다. **자북을 실제로 향하게 두고
0 이 나오는지 벤치로 최종 확인**한다(A안 발행값과 대조).

---

## 2. 📎 착수 전 확보할 것 (이게 있어야 B 시작)

1. **iAHRS raw mag 읽는 시리얼 명령 + 응답 형식** — 현재 드라이버는 `"e"`(Euler roll/pitch/yaw)만
   읽는다(`SendRecv("e\n", data, 10)`). raw mag 를 주는 명령(예: 별도 명령이나 조합 벡터)과
   응답 필드 순서를 데이터시트/제조사에서 받는다. → `SendRecv` 로 그 명령을 추가한다.
2. **mag 축 규약** — x/y/z 가 각각 어느 방향(전/후/좌/우/상/하)이고 부호가 어떤지.
3. **자기 편각(declination)** — 시험/대회장 위치의 WMM 값. 한국은 대략 **−8~−9°**(2026, 자북이
   진북보다 서쪽). **측정 장소마다 다르니 파라미터로.** 현재 시험장(세종) 기준값을 넣고, 대회장
   확정 시 갱신. (NTRIP 마운트포인트처럼 '장소 의존'이다.)

---

## 3. 통합 계획 (드라이버 수정 — 착수 시)

우리 `iahrs_driver.cpp` **최신본 위에**, 기존 경로 안 건드리고:

1. 파라미터: `mag_declination_deg`(기본 −8.5), `mag_heading_source`("fused"|"raw", 기본 raw).
2. 메인 루프: `SendRecv("<mag명령>", magdata, N)` 로 mx/my/mz 읽기(roll/pitch 는 이미 `"e"` 로 읽음).
3. `tilt_compensated_heading(...)` C++ 함수(위 검증식 그대로) 호출.
4. 결과를 `/imu/mag_heading` 로 발행(현재 A 자리 교체) — 발행자·타이밍·다른 토픽 불변.
5. 벤치: 자북 향하게 두고 0 확인 → A값·GPS COG 와 blackbox 로 대조해 "쓸만한지" 실측.

**절대 규칙:** `/imu/yaw`·`/imu/yaw_raw` 경로는 안 건드린다(발행자 2개 = 침묵실패). CMakeLists 도
`std_msgs`(이미 추가됨)면 충분 — mag 는 계산이라 새 의존성 없음.

---

## 4. 요약

| 항목 | 상태 |
|---|---|
| tilt 보상 방위 알고리즘 | ✅ 검증 완료(이 문서 §1) — C++ 이식만 남음 |
| iAHRS raw mag 명령·규약 | ❌ 데이터시트 필요(§2) — **착수 전제조건** |
| 편각(장소별) | ⚠️ 세종 기준값 넣고 대회장 확정 시 갱신 |
| 드라이버 통합 | 계획 확정(§3), 명령 확보되면 바로 |

**한 줄:** 계산은 준비·검증 끝. **iAHRS raw mag 시리얼 명령 하나만 확보되면** C++ 이식 + 편각 넣고
`/imu/mag_heading` 을 독립 신호로 바꾸면 된다.

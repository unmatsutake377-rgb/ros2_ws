# 회신: LOS 강화 north_goal / IMU 데이터 추가 ship_goal — 검토 + 역할 맞추기

> 받은 것: `23 github los강화.ver.py`(north_goal_angle), `IMU 추가 데이터.ver.py`(ship_goal_angle)
> 기준: 저장소 최신 `north_goal_angle.py`, `ship_goal_angle.py`, `iahrs_driver.cpp`, `blackbox.py`
> 한 줄 결론: **방향 잘 잡았습니다 — 특히 geofence를 되살린 것과 yaw를 안 덮어쓴 것. 다만 각 파일에
> 남은 3가지·2가지가 있어, "누가 뭘 맡을지"까지 아래에 정리했습니다. 병렬로 진행하시죠.**

---

## 파일 1 — LOS 강화 `north_goal_angle`

### ✅ 잘 반영됨

- **LOS + ILOS** — `calc_advanced_los_angle`에 cross-track 오차 + **적분항(조류·바람 보상)** + anti-windup(±50 클램프). 요청한 LOS보다 한 발 더 나갔고, 방향 좋습니다.
- **동적 lookahead** / **GPS 공분산 필터** / **위치 스무딩(Adaptive LFP)** — 모두 반영.
- **geofence 復活** ✅✅ — 이전엔 선언만 하고 죽어 있던 것을 `_geofence_ranges`·`_publish_geofence`·`_point_in_polygon`·`_ray_polygon_dist`로 완전히 살렸습니다. 우리가 강조한 "geofence 보존"이 제대로 들어갔습니다.
- **조향클램프·쿼터니언·navpvt오프셋·imu_cb 제거** — 빼달라 한 것들 정리됨.

### ❌ 아직 남은 것

1. **waypoints 하드코딩 (13~25줄)** — 우리 #1 지적. `config/waypoints.yaml` + `waypoint_loader`(좌표 검증·오타 시 fail-loud)로 전환해야 합니다. 지금은 오타 좌표로도 그냥 출발합니다.
2. **GPS 점프 필터 누락** — 이전 버전의 "속도상한 초과 fix 거부"가 빠졌습니다. `estimated_speed_mps`(128줄)가 원시 연속 fix로만 계산돼, GPS가 한 번 튀면 속도가 뻥튀기됩니다.

### ⚠️ DR — 판단 필요 (실행은 되지만 위험 요소 있음)

- 이 DR은 이전의 accel 이중적분판과 **다릅니다**: `/imu/yaw`(발행됨) + GPS 유도 속도로 등속 추측 → **우리 시스템에서 실행은 됩니다.**
- 다만 **시간 상한이 없습니다**(이전엔 15s cap). GPS가 오래 끊기면 무한정 드리프트합니다.
- `gps_timeout_sec=0.3s`가 너무 공격적입니다. RTK 5~10Hz면 2~3프레임만 놓쳐도 DR이 발동합니다. 점프 필터도 없어 뻥튀기된 속도로 DR할 수 있습니다.
- → **유지하려면**: 시간 상한 추가 + 점프 필터 복원 + 트리거 완화. 그리고 **RTK가 실제로 얼마나 끊기는지 blackbox로 실측한 뒤** 필요 여부를 정하는 게 맞습니다(측정으로 결정).

### 권장 구조

`calc_advanced_los_angle`가 자유 함수라 테스트 가능한 건 좋습니다. GPS 필터·LOS를 순수 모듈(`gps_filter`/`los_logic`)로 빼고 단위 테스트를 붙이면 배 없이 검증됩니다(우리 `waypoint_loader`·`heading_logic`과 같은 방식).

---

## 파일 2 — IMU 데이터 추가 `ship_goal_angle`

### ✅ 잘한 것

- **우리 최신 `ship_goal_angle` 위에 얹었습니다** — yaw_error 로직·watchdog·`% 360` 규약 그대로 유지.
- **yaw를 안 덮어씁니다**(78줄 주석대로 `/imu/yaw` 우선) — 이중 yaw 소스를 안 만든 것, 정확합니다.
- `euler_from_quaternion` 자체 구현(tf 의존 없음) — 합리적.

### ❌ 핵심 — 지금은 inert (작동 안 함)

- `/imu/data`를 구독하는데, **우리 `iahrs_driver`는 `/imu/data`를 발행하지 않습니다**(시리얼 `"e"`로 roll/pitch/yaw만 받아 `/imu/yaw_raw`만 냄). → `imu_data_callback`이 **영원히 안 불려** roll/pitch/quaternion/accel이 전부 `None`.
- 즉 **소비자만 있고 생산자(드라이버)가 없습니다** — GPS DR 때와 같은 구조입니다. 이 코드는 **IMU 담당이 진행 중인 드라이버 확장(`/imu/data` 또는 N4의 `/imu/mag_heading` 발행)이 선행돼야** 비로소 살아납니다.

### ⚠️ YAGNI

- 받은 데이터를 저장만 하고 아직 아무 데도 쓰지 않습니다(143줄). 안 쓰는 데이터를 구독·저장하는 건 이릅니다. **실제 쓸 곳(예: roll/pitch로 기울기 보상)이 정해질 때 넣는 게** 깔끔합니다. (무해하지만 dead plumbing)

---

## 🤝 방법 맞추기 — 역할 분담 (병렬 진행)

**진행 방식 — "LOS 안정 후 한 번에 이식" (안 A 확정):**
지금 바로 합치지 않습니다. 팀원이 LOS/필터/DR을 계속 다듬는 중이라, 지금 이식하면 곧 재작업입니다.
**LOS가 안정되면(점프필터·DR 상한까지 정리되면), 우리가 그 기능들을 우리 yaml 기반 최신본에 한 번에 이식**합니다. 충돌 1회로 끝냅니다.

| 항목 | 담당 | 내용 |
|---|---|---|
| north_goal: **waypoints.yaml 전환** | **우리** | 우리 최신본은 이미 `waypoints.yaml`+`waypoint_loader` 사용 중. 팀원은 하드코딩 그대로 둬도 됨 — 이식 시 우리가 yaml 기반으로 바꿈 |
| north_goal: **점프 필터 복원 + DR 상한/트리거** | 팀원 | LOS 작업자가 이어서. 순수 모듈 + 테스트 권장. **이게 끝나면 "안정" 신호 → 우리가 이식** |
| north_goal: **RTK 끊김 실측** | 우리 | blackbox로 DR 필요 여부 판단 근거 제공 |
| **드라이버 `/imu/data`(또는 mag_heading) 발행 확장** | IMU 담당 | 이게 선행돼야 파일2가 작동. **브랜치로 올려주면 우리가 diff 리뷰** |
| ship_goal: IMU 데이터 **활용처 확정 후 결합** | 협의 | 드라이버 확장 완료 후, 쓸 곳 정해서 |

**전달 방식 합의:** 코드는 **main 직접 push 금지**, **브랜치 push** 또는 `git diff` 텍스트로 주세요. 그럼 우리가 diff 보고 기능만 최신본에 반영합니다(env·미완 부분 걸러냄).

---

## 요약

| 파일 | 판정 | 남은 것 |
|---|---|---|
| LOS north_goal | ✅ 방향 좋음(LOS+ILOS·geofence 復活) | waypoints.yaml · 점프필터 · DR 상한/트리거 |
| IMU ship_goal | ✅ base 위 잘 얹음·yaw 안 덮음 | `/imu/data` 미발행이라 inert(드라이버 선행) · 저장만(YAGNI) |

**둘 다 좋은 진전입니다.** geofence 복원과 yaw 비덮어쓰기는 우리 원칙을 정확히 지킨 부분입니다.
남은 건 위 표대로 나눠서 병렬로 가시죠. 기준은 저장소 코드와 CLAUDE.md(waypoint_loader·geofence·IMU 토픽 규약)입니다.

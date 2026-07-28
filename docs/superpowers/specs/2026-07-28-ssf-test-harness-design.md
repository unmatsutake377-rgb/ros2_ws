# ssf_test — 테스트배 브링업 & 모터 검증 하네스 설계

- 날짜: 2026-07-28
- 상태: 설계 확정 (구현 계획 대기)
- 관련: `motor_control`, `blackbox`, `healthcheck`, `color_shape_detector`, `ship_direction`

---

## 1. 목적

곧 완성되는 테스트배(길이 1m)로 다음을 검증한다.

1. **장비 확인** — 카메라·라이다·IMU·GPS가 살아서 토픽을 내보내는가
2. **색 인식** — 카메라가 빨강/초록 부표 각도를 내는가
3. **물체 인식** — 라이다가 장애물 거리를 내는가
4. **자동 모터 제어** — 우리 소프트웨어 체인(`/desired_angle → motor_control → Motor_run → 브리지 → 스러스터`)이
   실제로 배를 움직이는가

테스트배는 대회배(1.6m ×2)와 구조가 전혀 다르고, 수조라는 좁은 공간 제약이 있어 대회처럼 미션 전체를
돌리는 것은 불가능하다. 따라서 미션(FSM·GPS 웨이포인트)을 벗겨낸 **브링업 하네스**만 만든다.

## 2. 핵심 원칙 (비협상)

1. **검출·제어 로직은 재구현하지 않는다.** 실제 노드(`color_shape_detector`, `ship_direction`,
   `motor_control`)를 그대로 돌리고 감싸기만 한다. 테스트의 가치는 "진짜 대회 코드가 도는가"를 보는 것이다.
   포크하면 버릴 코드를 검증하게 된다.
2. **별도 패키지 `ssf_test` 로 격리한다.** 대회 노드를 절대 건드리지 않는다. (반복돼 온 base drift 차단)
3. **배치(스러스터 기하)는 브리지 아래에만 존재한다.** "앞뒤 2개씩"·"일렬 4개" 같은 단어가
   `motor_control` 위층으로 새어나오면 안 된다.

## 3. 계층 계약 (배 무관 / 배별 분리)

```
/desired_angle   =  "전진 + 회전" 의도            ← 배 무관, 안 바뀜
motor_control    =  의도 → 좌/우 추력 (Motor_run)  ← 좌우 차동이면 공용, 배별 숫자는 프로파일 yaml
브리지/펌웨어      =  좌/우 → 물리 스러스터 N개 매핑    ← ★ 배별로 다른 유일한 곳 (회로팀)
```

- `Motor_run = pwm_r * 10000 + pwm_l`, 1500 중립, <1500 전진, >1500 후진 — **기존 계약 유지.**
- 테스트배(2앞2뒤)와 대회배(일렬4)의 차이는 **브리지 매핑 테이블 + 프로파일 숫자**에만 갇힌다.
  `motor_control` 코드는 두 배 공용.

## 4. 구성 요소

### 4.1 신규 패키지 `ssf_test`

| 산출물 | 종류 | 내용 |
|---|---|---|
| `test_bringup.launch.py` | launch | 센서 + 검출노드 + `blackbox` + `healthcheck` 만. GPS 웨이포인트·`north_goal_angle`·모드 FSM·`motor_control` **제외**. 목적 1·2·3 검증. |
| `test_motor.launch.py` | launch | `motor_control`(boat_test 프로파일) + `desired_angle_injector` + `blackbox`. 목적 4 검증. |
| `desired_angle_injector.py` | 노드 | 미션 없이 `/desired_angle` 를 발행해 자동 모터 체인만 구동. 아래 4.2. |
| `injector_logic.py` | 순수 로직 | 시퀀스/명령 → `/desired_angle` 값 매핑. ROS 비의존, Mac 테스트 가능. |
| `config/boat_test.yaml` | 프로파일 | 테스트배용 `motor_control` 파라미터. |
| `config/boat_a.yaml`, `config/boat_b.yaml` | 프로파일 | 대회배 스텁(실측 후 채움). |

### 4.2 `desired_angle_injector` 동작

목적: **미션·GPS 없이** `motor_control` 에 조향 명령을 넣어 "우리 소프트가 스러스터를 움직이는가"만 검증.

- **명령 모드**(파라미터 `cmd`): 사람이 읽는 명령을 정확한 `/desired_angle` 값으로 변환해 반복 발행.
  - `FORWARD` → 80.0 (정면), `LEFT` → 80+N, `RIGHT` → 80−N, `SPIN_LEFT` → 6000, `SPIN_RIGHT` → 5000,
    `REVERSE` → 후진 구간값, `STOP` → 50000(STOP_HOLD)
  - `ros2 topic pub` 로 원시 숫자를 쏘는 것보다 안전·명확(비전공자가 수조에서 쓰기 좋음).
- **시퀀스 모드**(파라미터 `sequence`): `[(cmd, 지속초), ...]` 를 순서대로 실행 후 STOP.
  예: `[(FORWARD,2),(LEFT,2),(STOP,1)]`. 반복·정량 기동 비교와 blackbox 로깅에 쓴다.
- 발행 주기 ≥ 5Hz 로 `motor_control` 명령 워치독(0.5s)을 만족. 시퀀스 끝/STOP 시 발행을 멈추면
  워치독이 자동 중립(1500/1500) → 별도 정지 로직 불필요(페일세이프 재사용).

### 4.3 안전 (수조 = 좁고 벽 있음)

- 첫 기동 확인은 **RC(하드웨어 조종기)** 로 한다 — 스러스터 회전·방향·방수·선체. 소프트웨어 무관.
- 소프트 모터 검증(`test_motor`)은 **낮은 추력**부터. `boat_test.yaml` 의 `base_pwm`/`max_diff` 를
  보수적으로 시작해 blackbox 로 반응 보고 올린다.
- STOP·워치독으로 언제든 중립. 시퀀스는 짧게(수초).

## 5. 검증에 쓰는 blackbox (이미 있음)

`blackbox` 가 이미 로깅한다: `desired_angle`, `pwm_r`, `pwm_l`, `imu_yaw`, `red_angle`, `green_angle`,
`image_angle`, `obstacle_min_dist`, 도착지연(`cam_dt_max`/`imu_dt_max` 등). 따라서:

- 목적 2·3: `red_angle`/`image_angle`/`obstacle_min_dist` 열로 검출 확인.
- 목적 4: 주입한 `desired_angle` vs 실제 `imu_yaw` 변화로 "명령→기동" 대응 확인(정량).

**새 로깅 코드 불필요.**

## 6. 범위 밖 (YAGNI)

- 미션 FSM·GPS 웨이포인트 주행 — 수조에서 불가, 대회배에서만.
- 조향 진동억제(slew/PD) — 별도 사안(상위3팀 회신 참조). 실측 후 결정.
- 우천모드 통합 — 별도 사안(우천모드 회신 참조).
- RC↔자동 전환 로직 — 하드웨어(회로/기구팀).

## 7. 외부 의존 (우리 몫 아님, 착수 전 확인)

1. **테스트배 브리지 노드**(Motor_run → 시리얼 → 스러스터) — 회로팀. `Motor_run`(L/R) 계약을 받는지 확인.
2. **RC ↔ 자동 전환** — 자동 모드에서만 컴퓨터가 `Motor_run` 을 먹는지(하드 mux/arming). 회로/기구팀.
3. **"일렬 4개"가 좌우로 벌어지는지** — 좌우 분리가 없으면 추력차로 회전 불가(하드웨어 문제, 코드로 못 살림).
   벌어지면 우리 계약 그대로.

## 8. 테스트 전략

- `injector_logic`(명령/시퀀스 → 값 매핑)은 **순수 함수** → Mac에서 단위 테스트.
- launch·노드 실행·실제 기동은 Ubuntu `colcon build` + 테스트배 현장에서만.

## 9. 산출물 요약

신규 `ssf_test` 패키지: launch 2개 + 노드 1개 + 순수로직 1개 + 프로파일 yaml 3개 + 단위 테스트.
기존 노드 변경 없음. 재사용 뼈대(`motor_control` + 프로파일)는 이미 존재하며, 대회배 이식은 프로파일
교체 + 브리지 교체로 끝난다.

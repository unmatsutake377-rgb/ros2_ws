# 수조 테스트 방법 설계 — 새 코드 없이 대회 코드 직접 검증

- 날짜: 2026-07-28 (최초) / 개정: 절차 기반으로 전환
- 상태: 확정
- 관련: `motor_control`, `blackbox`, `healthcheck`, `color_shape_detector`, `ship_direction`, 기존 launch
- 산출물: **테스트 절차서**(시스템 설명서 HTML `🧪 수조 테스트` 섹션) + 테스트배 파라미터. **새 코드 0.**

---

## 1. 목적

곧 완성되는 테스트배(1m)와 수조로 대회 전 검증한다. 수조는 좁아 대회 코스 전체를 못 만든다.
따라서 각 부분을 개별 검증한다:

1. 장비 작동 (카메라·라이다·IMU·GPS가 토픽을 내나)
2. 스러스터 작동
3. **노드 간 연결성·호환성, 토픽을 올바르게 주고받나**
4. 색·거리 측정 정확도
5. **미션 알고리즘(판단) 개별 검증**

## 2. 핵심 원칙 (사용자 확정)

**테스트용 코드를 새로 짜지 않는다.** 새로 짜면 대회에서 쓸 진짜 코드가 아니라 테스트용 코드를
검증하는 꼴이 된다. 대신:

- **진짜 대회 노드를 그대로 실행**하고, **ROS2 내장 점검 도구**로 들여다본다.
- 기존 노드를 하나도 수정하지 않는다 → 대회 코드·속도에 영향 0.
- 테스트는 *프로그램*이 아니라 *절차(문서)* 다.

이 원칙이 이전 초안(별도 `ssf_test` 패키지 + `scenario_player`/`injector` 신규 노드)을 폐기한 이유다.
새 노드·launch는 전부 "새 코드"이므로 원칙에 반한다.

## 3. 방법 — 항목별 (전부 기존 도구, 새 코드 0)

| 검증 항목 | 도구/명령 (기존) |
|---|---|
| 장비 살아있나 | `ros2 topic hz /scan`, `/camera/camera/color/image_raw`, `/imu/data`, `/ublox_gps_node/fix` |
| 노드 연결·토픽 올바름 | `ros2 node list`, `ros2 node info <노드>`, `ros2 topic info -v /red_angle`, `rqt_graph` |
| 색·거리 정확도 | 부표 실측 위치 vs `ros2 topic echo /red_angle`·`/obstacle_distance_array` |
| 스러스터 | RC(하드) 먼저 → `ros2 topic pub -r 10 /desired_angle std_msgs/msg/Float32 "{data: 80.0}"` |
| 미션 개별 | `ros2 topic pub /wp_mode std_msgs/msg/Int32 "{data: 0}"` → 진짜 미션 노드 실행 → `ros2 topic echo /desired_angle` |
| 기록 | `blackbox` 가 이미 desired_angle·pwm·imu_yaw·red_angle·거리·도착지연 로깅 |

### 침묵 실패 검출 (우리 숙적)

`ros2 topic info -v /토픽` 의 **Publisher/Subscription count 0** 이 토픽 이름 오타·미실행·이중 발행자·
QoS 불일치를 코드 없이 드러낸다. `rqt_graph` 로 전체 배선을 시각 확인.

### 미션 개별 검증 원리

`/wp_mode` 를 수동 발행하면 기존 mode manager가 **진짜 미션 노드**를 그대로 띄운다. 실센서(수조의
부표) → 미션 노드 판단 → `/desired_angle` 로 조향 결정 확인. 전부 대회 코드. `/wp_mode` 값만 바꿔
turn·dock 각각 검증.

## 4. 유일한 새 산출물 (알고리즘 아님, 대회 영향 0)

1. **테스트배 파라미터** — `motor_control` 은 이미 파라미터화됨(`base_pwm`·`max_diff`·`reverse_pwm`·
   `spin_forward_pwm`·`spin_diff`·`steer_invert`). 테스트배는 값만 다르다:
   `ros2 run motor_control motor_control --ros-args -p base_pwm:=1400 -p max_diff:=60`.
   설정값이지 코드가 아니며, 대회 땐 대회배 값을 쓴다 → 무게·영향 0. (원하면 `boat_test.yaml` 로 저장.)
2. **테스트 절차서** — 시스템 설명서 HTML `🧪 수조 테스트` 섹션. 위 명령을 순서대로, 비전공자도
   따라 하게. 코드 아님.

## 5. 수조의 물리적 한계 (정직히 명시)

- 실제 GPS **주행**, 게이트 **통과 거리**, 다미터 항법 = 공간이 없어 **대회장에서만**. 수조는 "판단"까지.
- 진동억제(slew/PD) = 미구현. "테스트" = 기동시켜 blackbox 로 진동 **측정**(구현은 실측 후).
- 실내조명 ≠ 햇빛 → 색 임계값(HSV)은 대회장 재조정. 수조에선 파이프라인 동작까지.

## 6. 외부 의존 (우리 몫 아님, 착수 전 확인)

1. 테스트배 브리지(Motor_run → 시리얼 → 스러스터) — 회로팀. `Motor_run`(L/R) 계약을 받는지.
2. RC ↔ 자동 전환 (하드 mux/arming) — 회로/기구팀.
3. "일렬 4개"가 좌우로 벌어지는지 — 좌우 분리 없으면 추력차 회전 불가(하드웨어 문제).

## 7. 산출물 요약

- 시스템 설명서 HTML 에 `🧪 수조 테스트` 섹션 추가 (완료).
- 테스트배 파라미터는 현장에서 보수적으로 시작해 blackbox 보며 튜닝(문서에 명령 포함).
- **신규 노드·launch·패키지 없음. 기존 노드 변경 없음.**

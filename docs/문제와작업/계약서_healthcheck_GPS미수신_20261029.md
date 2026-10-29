# 변경 계약서: healthcheck — 부팅 뒤 마감 안에 `/ublox_gps_node/fix` 를 한 번도 못 받으면 GPS 실패 (GPS C2 와 같은 시점)

> ssf-contract 작성 2026-10-29. 상태: **⏹ 하지 않음 — Davidson 10-29 "굳이 할 필요 없다"**. 기록으로 남김. C2(ublox respawn) 뒤에는 장치 없이 부팅해도 `/health_ok=true` 가 될 수 있다는 점(H2)·라이다·IMU·카메라 '영원히 대기'(H5)는 운용 점검(출발 전 `ros2 topic hz`)으로 확인한다.

## 0. 문제 실재 확인

| # | 문제 | 지금 | 근거 |
|---|---|---|---|
| H1 | 한 번도 못 받은 센서는 WAIT, WAIT 는 실패 아님 | **있다** | `healthcheck.py:174-180`(`t is None` → WAIT), :282, :292 `sensors_ok = all(st not in ("DEAD","NOFIX") …)` |
| H2 | 장치 없이 부팅해도 `/health_ok=true` | **C2 전엔 없음, C2 뒤 생김** | 지금은 ublox OnProcessExit → Shutdown(`ublox_gps_node-launch.py:74-77`, include `launch_files.launch.py:125-130`)이 대회 launch 전체를 내림. healthcheck 는 별도 터미널(`ros2_운용법.md:176`, `ssf_tools.launch.py`)이라 살아서 미션 노드 부재(`map_ok` :226-232)·geofence 발행자 0(:260-262)으로 이미 false. C2 뒤엔 launch 가 살아 map·geofence 정상, GPS WAIT → true |
| H3 | ublox 는 살아 있는데 `/fix` 0 (configureUblox 실패, 원 계약 P3b) | **지금도 있음** — 이번 변경이 함께 잡음 | node.cpp:808-811, :910 |
| H4 | 냉시작 NO_FIX | 이미 잡힘 | `_cb_gps` :159-166 `_gps_fix_ok` → :193-194 NOFIX → :292 |
| H5 | 같은 "영원히 WAIT" 가 라이다·IMU·카메라에도 | **있다, 지금도** | 같은 `_sensor_state`·:292. rplidar·iahrs_driver·realsense2_camera 에 종료 핸들러·respawn 없음 → 부팅 때 죽어도 launch 유지, health true. IMU 는 :217-220 로그만. **ship_direction 페일세이프도 ARMED(첫 수신 뒤 감시, ship_direction.py:111, :408)라 처음부터 없는 라이다는 페일세이프로도 안 잡힘 — healthcheck 가 사실상 유일한 신호** |
| Q5 | north 필터 계속 거부(/fix 유효인데 north stale·have_fix False) | healthcheck 는 모름 | north 는 로그로만(:321-332, :487) |

`/health_ok=false` 가 바꾸는 제어: **없음**. 구독자는 `mission_monitor.py:112`(표시, `mission_view.py:131-132`) 하나, mission_monitor 는 발행 없음. healthcheck 로그 info → warn(:328-331)뿐. 진단 전용 전제 유지(§3, §5).

## 계약
- **변경 단위**: 노드 하나 `ssf_tools/healthcheck`. 원자 묶음 무관, 매핑표 불변. 파일: `healthcheck.py`, 신규 순수 모듈 `health_logic.py`(ROS import 없음), `ssf_tools.yaml` `/healthcheck` 절, 신규 테스트 1개.
- **C2 와의 순서: healthcheck 커밋 → C2 커밋**(패키지 달라 나눔). 먼저 넣은 중간 상태는 안전(C2 전엔 장치 없으면 이미 false, 결과가 달라지는 건 "다 정상인데 /fix 메시지만 15 s 없음" 하나 — false 가 정답, H3 도 지금부터 드러남). 반대 순서(C2 먼저)는 "GPS 없이 출발" 이 안 보이는 창을 염. "같은 시점"인 이유는 안전이 아니라 **검증 기회**(C2 벤치 "장치 없이 부팅" 이 이 변경의 실물 검증).
- **토픽**: 이름·타입·QoS 불변, 새 토픽 없음. `/ublox_gps_node/fix`(구독 BE d10 :30-35, :106) 판정만 — 마감 지나도록 미수신이면 새 상태(예 `MISSING`). `/scan`·`/imu/yaw`·`/imu/yaw_raw`·`/heading_status`·카메라·`/wp_mode`(Q5(a) 면 수신 시각)·`/candidate_angle`·`/geofence_state`(그래프 조회) 불변. `/health_ok`(발행 RELIABLE d10 :115 → mission_monitor 표시) 의미만 넓어짐. Q5(a) 면 `/north_goal_angle_tp` 관찰 구독 추가(새 토픽 아님).
- **특수 상수**: 해당 없음, 매핑표 none(5·8) 처리 :227-228·:243-245 불변.
- **파라미터**: `gps_first_msg_deadline_sec`(이름 제안) **15.0**(코드 = yaml, 소수), `ssf_tools.yaml` `/healthcheck:` — `ssf_tools.launch.py` 가 넘김(대회 launch 엔 healthcheck 없음), `declare_parameter` 는 `create_timer`(:121)보다 먼저. `_boot_t = time.monotonic()`. `sensor_timeout_sec` 2.0·`health_period_sec` 2.0 불변. **`require_geofence_subscriber` true(§7-2) 건드리지 말 것.** `tools/check_params.py:58` 이 ssf_tools.yaml 대조 → yaml 에만 넣고 미선언이면 잡힘.
- **관련 조항**: §1-1, §1-2, §1-4, §1-5, §1-6, §1-7, §1-9, §3, §4, §5, §7, §7-2, GPS묶음 위험 2·결정 2, Ubuntu 의견 §2.
- **위험**: ① 새 상태가 `icon` dict(:203)에 없으면 `icon[st]`(:205) KeyError → 타이머 예외 → healthcheck 사망(진단 침묵 최악) — 아이콘·`sensors_ok` 실패 집합을 순수 함수 하나에서 정의. ② 15 s 근거 미실측 — M8P(firmware8)에서 GNSS 설정이 yaml 과 다르면 `configGnss(…, 15 s)` + cold reset 1 s 로 첫 메시지 **16 s 이상**(`ublox_firmware8.cpp:217`, `gps.cpp:343`, `gps.cpp:311`) → 일시적 거짓 실패, 첫 수신 뒤 자동 해제. F9P(firmware9) 경로엔 대기 안 보임(⚠️ 실측). ③ 기준 시각이 healthcheck 시작 — ssf_tools 를 대회 launch 보다 15 s 넘게 먼저 띄우면 즉시 GPS 실패 표시(그때는 미션 노드 부재로 이미 false, 로그 한 줄 증가). ④ "유효 fix" 기준이면 냉시작 TTFF 30 s~수 분(`gps_guard.py:14`)·실내가 새 실패로 잡히지만 이미 NOFIX 로 false — 결과는 같고 원인 라벨만 섞임 → **메시지 수신 기준 + 기존 NOFIX 권장**. ⑤ §1-7: 마감 넘는 순간 ERROR 는 별도 함수·호출 지점, 기존 :328-331 info/warn 분기와 섞지 말 것. 1 Hz GPS 와 sensor_timeout 2.0 경계 깜빡임은 기존 문제·범위 밖.
- **Q5 검토**: 전용 토픽 없음, 기존 토픽 추론 가능 — north `/wp_mode` 는 GPS 무관 매 틱(:490), `/north_goal_angle_tp` 는 gps_ok 일 때만(:482), 미션 끝나면 둘 다 멈춤(:450-451) → "/fix 유효 + /wp_mode 신선 + tp 침묵 N 초" = north 가 유효 fix 거부 중(cov·jump, `gps_filter.py:44`, :54). tp 구독·/wp_mode 수신 시각 추가 필요. 한계: healthcheck 인라인 fix 판정(:162-166)에 위경도 범위 검사 없음(차이 미미). 전용 상태 토픽은 새 토픽 → CLAUDE.md §3 + north 별도 변경 단위(healthcheck 구독 먼저). 거부는 RTK 이전 공분산처럼 정상 운용 중에도 생길 수 있어 /health_ok 반영 여부는 §5 판단.
- **검증**: 기존 ssf_tools 테스트는 test_thermal·test_mission_view 뿐, **healthcheck 로직 테스트 0개**, lint 파일도 없음. 추가 `src/ssf_tools/test/test_health_logic.py`(test_thermal 패턴): 마감 전 미수신 WAIT, 경계(==, +ε) 실패, 마감 뒤 첫 수신 OK, 수신 뒤 timeout DEAD·fix 없음 NOFIX(기존), sensors_ok 가 새 상태를 실패로, 모든 상태가 icon 표에, deadline ≤0·NaN·inf → 결정대로 ERROR + 15.0, yaml `/healthcheck` `*_sec` 전부 float, 선언이 create_timer 보다 앞(정적), Q5(a) 면 추론 함수 테스트(미션 종료·/wp_mode stale 이면 경보 없음). 실행 `tools/run_tests.sh ssf_tools`, Ubuntu colcon. 실물: C2 벤치 장치 없이 부팅 → 15 s 뒤 GPS 실패·/health_ok false → USB 꽂으면 첫 메시지 뒤 NOFIX/OK, **ublox 기동→첫 /fix 시간 실측(F9P·M8P)으로 15 s 확정**. 시뮬 불필요. 문서: tank_test_plan.
- **사람이 정해야 할 것**
  1. 기준: **메시지 수신**(권장) / 유효 fix?
  2. 마감 15 s 유지 / 여유(예 30 s) / C2 벤치 실측 뒤 확정?
  3. 기준 시각: healthcheck 시작 / 첫 `/wp_mode` 수신(= 대회 launch 기동)?
  4. 잘못된 마감값(≤0·NaN·inf): ERROR + 기본값 / inf = 검사 끔 허용?
  5. **H5: 라이다·IMU·카메라에도 같은 마감 규칙(같은 노드·같은 커밋 가능, 센서별 마감값)?**
  6. Q5: (a) 기존 토픽 추론 이번에 / (b) north 상태 토픽 신설(§3 + 별도 단위) / (c) 로그로 충분? 넣으면 /health_ok 반영 / 로그만?
  7. 커밋 순서: C2 **직전** 따로(권장) / C2 벤치 날 함께?

# 변경 계약서: 대회 launch 파라미터 출처를 yaml 로 + blackbox 기록 + 낡은 주석·로그 문구 정리 (값·동작 불변) — 11-05

상태: **승인 (11-05 팀장)** — ① 6개 노드 모두 ② 테스트는 키·타입만 강제(값은 yaml 이 이김; T4 는 타입만) ③ blackbox 실행 인자 `record`, 기본 true ④ healthcheck 넣지 않음(mission_monitor 도 안 넣음) / 메인 세션 기본값: ⑤ ssf_tools.launch.py 유지 + 문서에 '대회 launch 와 같이 띄우지 말 것' ⑥ 커밋은 launch yaml(6노드·테스트·해당 주석) / blackbox / 주석 정리 / 문서로 나눔 ⑦ CLAUDE.md:28 은 팀장이 직접 ⑧ 시뮬 재실행 생략 ⑨ 주석 정리 1커밋

## 핵심 요약
1. **yaml 과 코드 기본값은 6개 노드 모두 이름·값·타입까지 같다.** ship_direction 24키, motor_control 10, ship_gate 12, ship_dock 21, ship_back 17, ship_turn 20 전수 대조: 미선언 키 0, 값 차이 0, 타입 차이 0. yaml 절 이름 = launch `name=` = `super().__init__` 인자. setup.py data_files 가 각 `config/*.yaml` 설치. → yaml 을 넘겨도 동작 동일.
2. **위험의 성격이 바뀐다.** 지금은 yaml 을 잘못 고쳐도 무해. 바꾼 뒤엔 (a) 타입 오타(`3.0`↔`3`, `1360`↔`1360.0`)가 그 노드를 부팅 때 죽임(rclpy InvalidParameterTypeException) (b) 노트북에 커밋 안 된 yaml 수정이 조용히 대회값이 됨. 미선언 키는 예외 없이 조용히 버려짐(`test_search_behavior.py:18-19`, `tools/check_params.py:19`; 노드들은 allow_undeclared/automatically_declare 미사용).
3. **ship_back·ship_turn 은 순서가 중요.** yaml 먼저, search_behavior dict 맨 뒤. 거꾸로면 orbit 일 때 ship_back 이 yaml [2,3] 으로 뜨고 ship_turn 도 켜져 mode 3 `/candidate_angle` 발행자 2개(에러 없음).
4. **blackbox:** 발행 없는 별도 프로세스, 죽어도 다른 노드 영향 없음. `~/ssf_logs` 없으면 생성(blackbox.py:171). launch 마다 파일 1개, 회전 없음. 약 14 MB/h 추정(실측 아님, CPU 미측정). 기존 절차(`ros2_운용법.md:176`, `tank_test_plan.html:130`)대로 ssf_tools.launch.py 를 또 띄우면 /blackbox 2개 → 같은 초 파일명 충돌로 CSV 덮어씀.
5. **주석 정리는 펌웨어 재업로드 불필요**(주석만, `__LINE__`·assert 미사용 → 바이너리 동일). 주석·로그 문구를 읽는 테스트 없음.

## 변경 단위 (권장 순서, 각 커밋은 값 동일)
0. 테스트만 추가(`src/launch_files/test/test_launch_params.py`, T1~T7) — 현재 코드로도 통과.
1. motor_control (사슬 끝; 부팅 실패 → 펌웨어 워치독 중립)
2. ship_direction (실패 → motor_control cmd_timeout 중립)
3. ship_dock, 4. ship_gate (실패해도 정지 안 함 → ship_direction 1.0 s 뒤 20000)
5. ship_back, 6. ship_turn (결정 1이 6개일 때; yaml 앞, dict 뒤)
- 각 커밋에 그 패키지의 "yaml 안 넘긴다" 주석 정정 포함(위험 5 목록).
7. blackbox 추가(실행 인자 조건, `ssf_tools.yaml`, try/except 로 패키지 없으면 건너뜀 — `_oak_camera` 방식). healthcheck 는 결정 4.
8. 주석 정리: 펌웨어 / vision.yaml / north_goal_angle.
9. 문서 갱신.

## 토픽
- 1번 노드들: 토픽 변경 없음(토픽 이름 파라미터 없음).
- blackbox: 구독만 추가(BEST_EFFORT/VOLATILE/KEEP_LAST 10, RELIABLE 발행자와 호환). /wp_mode, GPS fix·velocity, /scan, /imu/yaw·yaw_raw·mag_heading, /imu/data(미발행일 수 있음 → 빈 칸), /yaw_error, /candidate_angle, /desired_angle, Motor_run, /obstacle_distance_array, /failsafe_level, /obstacle_reject_count, /gates_passed, 검출기 각도·색.
- healthcheck 를 넣으면 /health_ok 발행자는 반드시 1개(맥북 `ros2 run ssf_tools healthcheck` 와 중복 주의).

## 파라미터 (값 불변, 출처만)
| 노드 | 키 | 현재 launch → 변경 |
|---|---|---|
| ship_direction | 24 | 없음 → `[ship_direction.yaml]` |
| motor_control | 10 | 없음 → `[motor_control.yaml]` |
| ship_gate | 12 | 없음 → `[ship_gate.yaml]` |
| ship_dock | 21 | 없음 → `[ship_dock.yaml]` |
| ship_back | 17 | dict → `[yaml, dict]` |
| ship_turn | 20 | dict → `[yaml, dict]` |
| blackbox | 15 | 없음(대회 launch 밖) → `[ssf_tools.yaml]` + 조건 |
- 경로 `get_package_share_directory('<pkg>')/config/<pkg>.yaml`. `--symlink-install` 에서 yaml 수정이 재빌드 없이 반영되는지는 미확인.
- ⚠️ §6 시뮬 검증 파라미터(clearance 0.25, detection_distance 3.0/2.0, min_obstacle_cells 1, temporal 1/2, obst_median_kernel 5)가 이제 yaml 에서 실제 적용. 이후 yaml 만 고치면 대회 동작이 바뀌고 `tools/sim/ssf_sim.py:33-38` 사본과 어긋날 수 있음.
- §7-2 안전 스위치 직접 영향 없음.

## 위험
1. ship_back·ship_turn parameters 순서 역전 → orbit 에서 mode 3 발행자 2개(조용히). 지금 테스트는 `parameters is not None` 만 봄(test_search_behavior.py:205) → T5 로 고정.
2. yaml 타입 오타 → 해당 노드 부팅 즉사.
3. 커밋 안 된 yaml 수정이 대회값이 됨(`tank_test_plan.html:160` 이 motor_control.yaml 현장 수정 안내).
4. blackbox 중복 실행 → CSV 덮어씀(초 단위 stamp, "w").
5. "yaml 안 넘긴다" 주석·문서가 틀려짐: ship_direction.yaml:88,96; ship_direction.py:283; ship_gate.yaml:19,23; ship_gate.py:92; gate_logic.py:89; ship_dock.yaml:20,39; ship_dock.py:160,195-196; ship_turn.yaml:32; ship_turn.py:119; 테스트 docstring test_motor_l1_reverse.py:109, test_dock_steer_clamp.py:57, test_dock_transit.py:15, test_gate_logic.py:154, test_gps_fresh.py:118; CLAUDE.md:28(사람이 고침).

## 검증 계획
- 기존: test_search_behavior(①~⑦), test_oak_camera, test_dock_transit(:461, pyyaml — Mac 수집 실패 가능), test_gate_logic, ssf_tools test_config_consistency.
- 신규 test_launch_params.py(pyyaml 없이 정규식·AST): T1 노드별 Node 1개 + yaml 경로 / T2 yaml 절 = name = 노드 이름 / T3 yaml 키 ⊆ declare 키 / T4 값·타입 일치(결정 2) / T5 ship_back·ship_turn parameters 마지막이 dict / T6 setup.py 가 yaml 설치 / T7 blackbox Node 1개·조건·ssf_tools.yaml·인자 기본값.
- 명령: `tools/run_tests.sh launch_files ship_direction motor_control ship_gate ship_dock ship_back ship_turn ssf_tools north_goal_angle color_shape_detector`. Ubuntu `colcon build --symlink-install` 뒤 `colcon test`.
- Ubuntu 부팅 스모크: ① `git status --short src/*/config` 깨끗 ② launch 출력 `InvalidParameter`·`process has died` 0 ③ `ros2 node list | sort | uniq -d` 비어 있음, /blackbox 1개 ④ `for n in ship_direction motor_control ship_gate ship_dock ship_back ship_turn blackbox; do pgrep -af "lib/[a-z_]*/$n( |$)"; done` 각 줄에 `--params-file …/share/<pkg>/config/<pkg>.yaml`(ship_back·turn 은 뒤에 dict 파일 하나 더) ⑤ `ros2 param get` 값 확인(ship_direction clearance 0.25 등, motor_control base_pwm 1360 등, ship_gate red_on_port True, ship_dock active_wp_mode 7, ship_back [2,3], ship_turn enabled False, blackbox rate_hz 10.0) ⑥ `search_behavior:=orbit` → ship_back [2], ship_turn True ⑦ `python3 tools/check_params.py --boat A` 미선언 0·불일치 0 ⑧ blackbox 크기 60 s 간격 두 번(MB/h 실측), CPU %, `record:=false` 면 /blackbox 없음, blackbox kill 뒤에도 /desired_angle 계속.
- 시뮬 재실행 불필요(로직 불변, 값 전수 일치).

## 3번 주석·문구 대상
- `arduino/ssf_boat/ssf_boat.ino` :275 "< 1500 = AUTO"(실제 > 1500, :789), :35 "핀 18"(실제 19, :127), :16-18·:677 "브릿지는 상태줄 안 읽음"(실제 bridge.py:219 읽음). :100 은 08-17 이력이라 유지.
- `src/color_shape_detector/config/vision.yaml` :113 "mode 2 → ship_back([2])"(기본 hold [2,3], orbit [2]), :146 "3=red(ship_turn 선회)"(기본 hold 는 ship_back 위치유지). 값 [2,3] 유지.
- `src/north_goal_angle/north_goal_angle/waypoint_recorder.py:7` 저장 경로 → `~/ssf_logs/waypoints_recorded.yaml`.
- `src/north_goal_angle/north_goal_angle/north_goal_angle.py` :440·:605·:608 주석, :444·:615 INFO 문구 "WP0 부터" → "처음 AUTO 면 WP0, 재진입은 현재 WP 부터"(동작 불변, 심각도·호출 지점 그대로).

## 문서 반영 후보
- 소프트웨어_동작설명서(.md/.html): :117(진단 도구, 참조 9장→5.13), 3.2 노드 표에 blackbox, 5.13 제목, "yaml 없이 켜진다" 주의점, 펌웨어 주석 문장.
- tank_test_plan.html :130·:160, ros2_운용법.md :82·:170-176, boat_launch_checklist.md:129, 맥북_원격조작_설명서.md:168(healthcheck 넣을 때).
- 보고서 Ⅲ-2.3 "대회 launch 는 yaml 을 넘기지 않아 운용값은 노드 기본값이다" → 수정안_대기 항목.
- CLAUDE.md:28 은 사람이 고침.

## 사람이 정할 것
1. 대상 4개 vs 6개(ship_back·ship_turn 포함)
2. 테스트로 "yaml 값 == 코드 기본값" 강제 vs 키·타입만 강제
3. blackbox 실행 인자 이름·기본값(제안 `record`, 기본 true)
4. healthcheck 대회 launch 포함 여부(mission_monitor 는 넣지 않음 권장)
5. ssf_tools.launch.py 유지(벤치 단독용 명시) + 대회 launch 와 같이 쓰지 말라는 경고
6. 커밋 노드별 vs 한 커밋
7. CLAUDE.md:28 (사람이 직접)
8. 시뮬 재실행 생략
9. 주석 정리 1커밋 vs 3커밋

# tools/sim — 회피 시뮬레이터 (09-28 저장소 이관)

`CLAUDE.md` §6 "시뮬로 검증된 파라미터" 의 출처. 09-28 까지 Mac mini `~/Desktop/SSF 노드 파일/` 에만 있어 재현 불가였다.

| 파일 | 역할 |
|---|---|
| `ssf_sim.py` | `ship_direction`·`motor_control` 로직 재구현 + 상수. **노드와 수동 동기화** — 노드 파라미터 바꾸면 여기도 |
| `fastworld.py` | 부표 월드 + 가짜 LiDAR 스캔 |
| `kaboat_full_course_sim.py` | 2025 규정 코스 70×20m 전 구간 (`python3 kaboat_full_course_sim.py`) |
| `sweep_s1.py` | half_width × clearance 스윕 (`python3 tools/sim/sweep_s1.py --seeds 20`) |
| `tournament_sim.py` | 토너먼트 1:1 — 실제 노드 4개(north·ship_goal·ship_direction·motor_control) + 대본 상대 배(none·side_by_side·slow_ahead·fast_behind·cut_in). `--param`, `--fence-inset`(지오펜스 좌우 안쪽, 이탈은 실제 경기장 기준). 결과: 계약서_토너먼트분리_20261109 §11 |
| `full_course_sim.py --world worlds/arena_2025.yaml` | 규정 그림(2025-11) 배치 세계 — 게이트 5쌍·장애물 격자·위치유지·탐색·도크 12 m(칸 2 m·칸막이 1.5 m·깊이 3 m), 2026 OT 순서 13점. 가정은 yaml 머리말. 위경도 사본 `worlds/arena_2025_wp.yaml` 은 `--write-world-wp` 로 생성 |
| `dock_geometry.py` | 도크 선분 생성 순수 함수(`--dock-bay-w/--dock-gap/--dock-depth`, 기본 = 옛 1 m 칸 형상과 선분까지 같음). 테스트 `python3 -m pytest tools/sim/test_dock_geometry.py -q` (LiDAR 전방 섹터 거리 vs 닫힌 식, 옆 비킴 거짓 CONTACT 경계) |

## 노드와 다른 점 (알고 쓸 것)
- 시간 투표 [G], geofence 병합 [H] 없음. 물보라 오탐이 없는 월드라 회피 결과엔 영향 적음.
- 접촉 판정 = 반경 `half_width` 원. 배 길이 방향 측면 접촉은 못 잡음 → `sweep_s1.py` 의 선회 유효 폭 표로 보완.
- 동역학(K_V, K_W, CRUISE 1.2m/s)은 **가정**. 수조 실측(`수조_실측값_반영_프롬프트.md`) 후 교체.

## 동기화 상태 (09-23 기준, yaml 과 일치)
`HALF_WIDTH 0.29` · `CLEARANCE 0.25` · `MIN_OBSTACLE_CELLS 1` · `REAR_OBSTACLE_IGNORE_MARGIN None`(노드에서 제거된 결함).
`BASE_DETECTION_DISTANCE 1.8` 은 기본값일 뿐 — 스윕은 `dd` 인자로 yaml 의 2.0/3.0 을 넘긴다.
`candidate_stale_sec`(11-04, candidate 신선도)는 동기화 대상 아님 — `ssf_sim.py` 는 (C) 경로 사본.
`side_hysteresis_deg`·`cross_penalty_deg`(11-06, (C) 편 유지·가로지름 벌점, 발견 8)는 `ssf_sim.py`·`tools/gap_selection_figure.py` 의 (C) 사본에 **미반영**(옛 규칙 그대로, 계약서_회피편선택_20261106 결정 4).
`side_escape_deg`(11-06, (C) 후보 0 개 + 옆 장애물 → 후진 대신 비킴, 발견 9)도 `ssf_sim.py` 사본에
**미반영**(옛 규칙 = 무조건 후진, 계약서_옆비킴_현장조정_20261106 결정 ⑥).
`side_lock_*`(11-09, 토너먼트 옆 잠금, 기본 끔)는 `ssf_sim.py` 사본에 **갭 경로만** 반영(DWA·탈출 경로 없음). 실노드 시뮬(`tournament_sim.py`·`full_course_sim.py --param ship_direction.side_lock_enabled=true`)은 전부 반영.
회피 성능 기준은 `full_course_sim.py`(실제 노드 코드 `SD.ShipDirection()` 사용 — `avoid_sim`·`gate_sim`·`l1_dropout_sim`·`mission_figs` 도 실노드라 자동 반영). `sweep_s1`·`kaboat`·`avoid_figure` 결과는 옛 규칙 수치다.
`--lidar-noise`(full_course_sim·tournament_sim, 11-09): LiDAR 잡음 σ 3 cm + 결측 5 % + 정면 ±30° 고립 물보라 1 %(합성, `full_course_sim.add_lidar_noise`). 시드 고정이라 재현됨. 도킹 물보라 근거(계약서_도킹물보라필터_20261109).
`full_course_sim.py --param 노드.키=값`(반복 가능)으로 노드 파라미터를 덮어써 끈 기능을 켜 볼 수 있다. 예: `--param ship_direction.side_escape_deg=60.0`, `--param ship_direction.dwa_enabled=true`. 결과 요약에 '직사각형 선체 접촉(선미 포함)' 줄이 함께 나온다(기존 접촉 수치는 중심 원 기준 그대로).

의존: `numpy`, `matplotlib`. ROS 불필요.

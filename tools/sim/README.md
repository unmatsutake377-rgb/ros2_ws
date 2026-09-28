# tools/sim — 회피 시뮬레이터 (09-28 저장소 이관)

`CLAUDE.md` §6 "시뮬로 검증된 파라미터" 의 출처. 09-28 까지 Mac mini `~/Desktop/SSF 노드 파일/` 에만 있어 재현 불가였다.

| 파일 | 역할 |
|---|---|
| `ssf_sim.py` | `ship_direction`·`motor_control` 로직 재구현 + 상수. **노드와 수동 동기화** — 노드 파라미터 바꾸면 여기도 |
| `fastworld.py` | 부표 월드 + 가짜 LiDAR 스캔 |
| `kaboat_full_course_sim.py` | 2025 규정 코스 70×20m 전 구간 (`python3 kaboat_full_course_sim.py`) |
| `sweep_s1.py` | half_width × clearance 스윕 (`python3 tools/sim/sweep_s1.py --seeds 20`) |

## 노드와 다른 점 (알고 쓸 것)
- 시간 투표 [G], geofence 병합 [H] 없음. 물보라 오탐이 없는 월드라 회피 결과엔 영향 적음.
- 접촉 판정 = 반경 `half_width` 원. 배 길이 방향 측면 접촉은 못 잡음 → `sweep_s1.py` 의 선회 유효 폭 표로 보완.
- 동역학(K_V, K_W, CRUISE 1.2m/s)은 **가정**. 수조 실측(`수조_실측값_반영_프롬프트.md`) 후 교체.

## 동기화 상태 (09-23 기준, yaml 과 일치)
`HALF_WIDTH 0.29` · `CLEARANCE 0.25` · `MIN_OBSTACLE_CELLS 1` · `REAR_OBSTACLE_IGNORE_MARGIN None`(노드에서 제거된 결함).
`BASE_DETECTION_DISTANCE 1.8` 은 기본값일 뿐 — 스윕은 `dd` 인자로 yaml 의 2.0/3.0 을 넘긴다.

의존: `numpy`, `matplotlib`. ROS 불필요.

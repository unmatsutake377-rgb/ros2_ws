# 변경 계약서: ship_turn 배별 yaml(ship_turn_A/B.yaml) + 대회 launch boat 층

> 상태: **승인(아래 §승인)** (팀 모드 ①, ssf-contract 작성). 2026-11-15. 브랜치 feat/ship-turn-scan(a366c9d 위).
> 요청: "a.b 번배 런처에 넣어서 턴노드 변경해줘" → 사용자 선택: 배별 yaml 추가(두 배 선회율이 달라 scan_*·선회각을 배별로).

## 변경 단위
- `ship_turn` 의 **파라미터 출처만** 바뀐다(로직·토픽·active_wp_mode 그대로). 원자 묶음 아님.
- 패키지: `ship_turn`(config 2개) + `launch_files`(launch + 테스트). 문서·도구: CLAUDE.md, `tools/check_params.py`, (선택) 카탈로그.
- 순서: ① A/B yaml 추가(launch 가 안 읽어 무영향, `setup.py:16` glob 설치) → ② launch + 테스트. 한 커밋 가능.
- 파일 확인 위치: 순수 함수 `ship_turn_param_files(share_dir, boat)` — `_course_check`(첫 OpaqueFunction) 에서 확인·raise, `_search_nodes` 에서 사용(`course_param_files` 선례). `_search_nodes` 에서 raise 하면 일부 노드만 뜬 채 죽는다.
- parameters 순서: `[공통, 배별, search_behavior dict]` — dict 맨 뒤 규칙 유지(`launch_files.launch.py:174-179`).

## 토픽·특수 상수
- 변경 없음. 5000/6000/20000/50000·SCAN 의미 그대로, 값만 배별.
- 🚨 배별 파일에 `enabled`·`active_wp_mode` 금지 — 배마다 mode 3 담당이 갈릴 수 있고 test_search_behavior·test_mode_gate 는 공통 yaml 만 본다 → 키 교집합 = ∅ 테스트.

## 파라미터 (배별 후보, 지금 공통값)
| 이름 | 공통값 | 비고 |
|---|---|---|
| scan_timeout_sec / scan_angle_deg / scan_tol_deg / scan_lead_sec / scan_stop_sec / scan_hold_sec / scan_clear_m | 25.0 / 90.0 / 5.0 / 0.3 / 1.5 / 1.2 / 1.0 | 선회율·관성 의존 |
| orbit_target_deg | 420.0 | 🚨 orbit_max_sec 60.0 과 묶임 |
| orbit_radius_m | 3.0 | 🚨 lidar_track_max_m 5.0(=r+2)·approach_dist_m 4.0 과 묶임 |
- 절 이름 `/ship_turn:`, 값은 소수점(정수면 부팅 사망). §6·§7-2 무관.
- 벤치 `src/ship_turn/launch/ship_turn.launch.py:10` 는 공통만 — 안 고치면 벤치·대회 값 갈림.

## 관련 조항
§1-1, §1-4(문구 "boat 가 고르는 건 IMU 두 값뿐"·"미션 노드 parameters= 없이" 둘 다 이미 틀림 → 갱신, `CLAUDE_상세.md:57,560` 도), §1-6, §1-9, §3, §7.

## 위험
1. `test_launch_params.py:389-398` T5 가 ship_turn parameters 2개 단정 → 3개로 수정.
2. `tools/check_params.py` 가 파일별 대조 → 배별이 공통 키를 덮으면 공통 대조에서 거짓 '불일치', 배별 파일은 대조 안 됨(BOAT_FILE = ssf_heading 하나).
3. 파라미터 앱 `edit_text` 는 있는 줄만 고침 → 배별 파일에 키가 없으면 그 배만 못 바꾸고 공통을 고쳐 두 배가 같이 바뀜(경고 없음). `scan_files` 는 git ls-files 기준.
4. `test_boat_config.py:152-167` 이 launch 모듈 `files =` 대입 1번 검사 → 새 코드에서 `files =` 쓰지 말 것.
5. 묶인 값이 층별로 갈림(orbit_radius_m ↔ lidar_track_max_m, orbit_target_deg ↔ orbit_max_sec) — 에러 없이 실패 판정.
- 빈 배별 yaml(`ros__parameters: {}`) rcl 수용 ⚠️ 미확인, T3 는 키 0개면 실패.

## 검증 계획
- 기존: `test_launch_params.py`(T1~T8), `test_search_behavior.py`, `test_boat_config.py`, `tools/test_ssf_param.py`, `src/ship_turn/test/*`.
- 추가: (a) NODE_CASES 에 ship_turn_A/B (b) T5 3항목 (c) `ship_turn_param_files` 경로·없으면 RuntimeError·모르는 boat ValueError (d) `_course_check` 호출(T8) (e) 배별 키 ∩ {enabled, active_wp_mode} = ∅ (+화이트리스트?) (f) A·B 키 집합 같음 (g) `_search_nodes` boat perform (h) test_ssf_param: A배에서 ship_turn_B 숨김.
- 실행: `tools/run_tests.sh launch_files ship_turn ssf_heading`, `python3 -m pytest tools/test_ssf_param.py -q`, Ubuntu colcon build/test, 부팅 스모크 boat:=A/B `ros2 param get /ship_turn scan_timeout_sec` + launch LogInfo('ship_turn boat=…, 파일=…').
- 시뮬: ssf_sim 동기화 불필요. full_course_sim 은 ship_turn yaml 을 안 읽음 → 배별 값은 `--param ship_turn.키=값` 으로. B 값이 공통과 달라지면 재시뮬.

## 사람이 정해야 할 것
1. 초기 내용: 빈 덮어쓰기 vs 값 복사.
2. 배별 키 범위 + 화이트리스트 테스트 고정 여부.
3. 공통 파일에서 배별 키를 뺄지(ssf_heading 선례) vs 남기고 check_params 층 병합 수정.
4. 배별 파일 없을 때 RuntimeError 로 launch 정지(선례) vs 공통값으로 계속.
5. 벤치 `ship_turn.launch.py` 에도 boat 층(필수 인자).
6. 파라미터 앱 카탈로그에 scan_timeout_sec·orbit_target_deg 등 추가·limits.

## ✅ 승인 (Davidson, 2026-11-15)
1. **값 복사 + 공통에서 빼기** — 배별 키를 A·B 파일에 지금 공통값 그대로 넣고 공통 `ship_turn.yaml` 에서는 뺀다(ssf_heading 선례). 코드 기본값은 그대로(=지금 값). 공통 파일엔 "이 키들은 ship_turn_A/B.yaml" 주석.
2. **배별 키 = scan_* 7개 + orbit_target_deg + orbit_max_sec** (9개). 화이트리스트를 테스트로 고정. orbit_radius_m·lidar_track_max_m·approach_dist_m 은 공통.
3. (1 에 포함) check_params 는 공통에서 빠지므로 층 병합 수정 불필요 — 배별 파일 대조(BOAT_FILE)에 ship_turn 을 넣을지 구현자가 확인해 필요하면 포함.
4. 배별 파일 없으면 **RuntimeError 로 launch 정지** — yaw_mux·course 선례대로(별도 질문 없이 선례 채택).
5. **벤치 `ship_turn.launch.py` 에도 boat:=A|B 필수** — 대회·벤치 같은 순수 함수(ssf_heading `test_two_launch_helpers_identical` 선례).
6. **파라미터 앱 카탈로그에 추가** — 9개 키(또는 현장에서 만질 핵심: scan_timeout_sec·scan_angle_deg·scan_clear_m·orbit_target_deg·orbit_max_sec 등), hint·advice·limits. when/check/revert 는 현장 조정표에 없으므로 '기준 없음' 규칙을 따른다.
+ CLAUDE.md §1-4 문구 갱신(boat 가 고르는 것: IMU 두 값 + dwa_hull + ship_turn 9개, 미션 노드 yaml 전달 사실), `CLAUDE_상세.md:57,560` 같은 문구.

## 구현·검토·검증 결과 (2026-11-15)
- 구현(ssf-implementer): ship_turn_A/B.yaml(9개 키, 지금 값 복사), 공통 ship_turn.yaml 에서 9개 제거, 대회·벤치 launch `ship_turn_param_files`(모르는 boat ValueError, 파일 없음 RuntimeError — `_course_check` 에서 노드보다 먼저), parameters [공통, 배별, dict], LogInfo, check_params `BOAT_FILES`, 앱 카탈로그 9개, CLAUDE.md §1-4·CLAUDE_상세.md, 테스트 (a)~(h).
- 리뷰(ssf-reviewer): 🔴 차단 0, 🟡 권고 7. 메인 세션 반영: 카탈로그 `orbit_max_sec` 하한 20 → **45**(420° 약 43 s), CLAUDE.md "scan_* 7개(scan_enabled·scan_stale_sec 는 공통)".
  남은 권고: scan_timeout+orbit_max 조합이 mode 3 WP 90 s 를 넘을 수 있음(앱이 조합 검사 안 함, advice 로만), test_ssf_param fixture 가 untracked 도 복사, mission_figs.py 가 공통 yaml 만 읽음, `.claude/agents/ssf-reviewer.md:21`·`ssf-verifier.md:20` 전제가 옛것(사용자 결정).
- 검증(ssf-verifier + 메인 재실행): `tools/run_tests.sh launch_files ship_turn ssf_heading` **449 통과 / 0 실패**. `tools/test_ssf_param.py` 91 통과 / 1 실패(`test_server_launch_routes` — 이 Mac 에 tmux 없음, HEAD a366c9d 에서도 같은 실패 확인 = 기존·환경).
  **build·colcon test·부팅 스모크 미실행(ROS 없음)**.
- Ubuntu 부팅 스모크(리뷰 권고 6): boat:=A/B 각각 로그 `ship_turn boat=…, 파일=… → …ship_turn_<boat>.yaml` · `ros2 param get /ship_turn scan_timeout_sec` 25.0 · `orbit_target_deg` 420.0 · `enabled` True · 배별 파일 치우면 RuntimeError 로 아무것도 안 뜸 · 벤치 boat 없이/소문자 거부 · `check_params.py --boat A|B` 미선언 0·불일치 0.

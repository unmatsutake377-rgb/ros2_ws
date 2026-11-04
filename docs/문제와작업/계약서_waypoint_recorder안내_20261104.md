# 변경 계약서: `waypoint_recorder` 대화식 입력 안내(미션 번호표 + 템플릿 기본값 + 잘못된 번호 재질문 + 순서 비교 경고)

> ssf-contract 작성 2026-11-04(에이전트 본문, 메인 세션이 저장). 상태: ⏸ Davidson 승인 대기. 녹화 도구 하나의 **터미널 안내 문구·입력 흐름**만 바꾼다. 미션 동작(north_goal_angle 본체·로더 검증·출력 yaml 형식·토픽)은 안 바뀐다.

## 0. 코드로 확인한 사실

| # | 항목 | 결과 | 근거 |
|---|---|---|---|
| ① | 지금 프롬프트는 `mode(미션번호)? ` `dwell(머무는 초)? ` `구역 이름(엔터=자동)? ` 세 줄뿐 | 사실 | `waypoint_recorder.py:207-209` |
| ② | 틀린 정수(4·6·9)는 `_ask_int` 통과 → `is_valid` 로더 거부 → `continue` = **캡처 취소**(재질문 아님) | 사실 | `:128-137, :210-211, :232-237` |
| ③ | 허용 mode 권위 = `waypoint_loader.VALID_MODES = {0,1,2,3,5,7,8}` | 사실 | `waypoint_loader.py:21` |
| ④ | `parse_waypoints` 는 라벨을 버리고 `[lat, lon, mode, dwell]` 만 반환 → **반환 형태 변경 금지**(`north_goal_angle.py:455` 4개 언패킹, 테스트). 라벨은 원본 dict `구역` 에서 따로 | 사실 | `waypoint_loader.py:96` |
| ⑤ | 노드 waypoints 경로 규약 = share `get_package_share_directory('north_goal_angle')/config/waypoints.yaml` | 사실 | `north_goal_angle.py:118-121`, launch `:40,44`, `setup.py:17` |
| ⑥ | recorder 는 ROS 파라미터 0개. `avg_window_sec=2.0`·`cov_warn=2.0` 은 `main()` 하드코딩, `out_path` 는 `sys.argv[1]` | 사실 | `:249-250` |
| ⑦ | `sys.argv[1]` 을 그대로 씀(`remove_ros_args` 없음) → `--ros-args` 가 출력 경로가 됨 | 사실 | `:249` |
| ⑧ | pyyaml 일부러 안 씀(직접 포맷). 템플릿 읽기엔 `yaml.safe_load` 지연 import, ImportError = 템플릿 없음 | 사실 | `:16-21, :56-61` |
| ⑨ | dwell 기본표(0·1·2·3→3.0, 5→50, 7→60, 8→60) = 현재 `waypoints.yaml` 값(작년 dwell, 규정 아님) | 사실 | `waypoints.yaml:19-29` |
| ⑩ | mode 3 = launch 기본 hold 에선 ship_back(부표 앞 위치유지), orbit 이면 ship_turn 선회 → 번호표 문구가 행동을 단정하면 안 됨 | 사실 | `launch_files.launch.py:63` |
| ⑪ | `d N` 삭제 후 인덱스가 당겨짐 → 기본값은 매번 `len(recorded)` 기준, 비교는 `q` 때 최종 목록으로 | 사실 | `:183-186` |
| ⑫ | Ctrl-C/EOF 는 어디서든 기록분 저장 → 새 프롬프트도 `input()` 기반 유지 | 사실 | `:216-217` |

## 1. 변경 단위
- 노드 하나 `waypoint_recorder`(패키지 north_goal_angle). 원자 묶음 아님. 커밋 1개.
- 포함: `waypoint_recorder.py` + 새 순수 모듈 `north_goal_angle/recorder_prompts.py` + 테스트(`test_waypoint_recorder.py` 확장 또는 `test_recorder_prompts.py`) + `docs/절차/웨이포인트_녹화절차.md` §0·§3(메인 세션).
- 불변: `north_goal_angle.py`, `waypoint_loader.py`(반환 형태·VALID_MODES), launch, `config/*.yaml`, `format_waypoints_yaml` 출력 형식, `average_fixes`.

## 2. 토픽 계약 (변경 없음)
| 토픽 | 타입 | QoS | 역할 | 반대편 | 변경 |
|---|---|---|---|---|---|
| `/ublox_gps_node/fix` | NavSatFix | 구독 RELIABLE d10 | 구독 | ublox_gps_node | 없음 |
발행 없음. 출력 파일 형식 `{구역, lat, lon, mode, dwell}` 그대로 → 로더 경로 불변.

## 3. 특수 상수·모드
- 번호표는 `VALID_MODES` 를 import 해 생성, 설명 dict 키 집합 == VALID_MODES(테스트 고정). 4·6 "없는 번호" 는 계산으로.
- 5000/6000/20000/50000·추력 방향 비접촉. dwell 기본표는 WP 타임아웃보다 작음.

## 4. 파라미터
| 이름 | 현재 | 변경 |
|---|---|---|
| avg_window_sec / cov_warn / out_path | 하드코딩·argv | 유지 |
| `template_file`(신설) | 기본 = share `config/waypoints.yaml` | 전달 방식 결정 2 |
| `DEFAULT_DWELL_BY_MODE`(상수) | `{0:3,1:3,2:3,3:3,5:50,7:60,8:60}` | 템플릿 못 읽을 때 폴백, 소스 yaml 과 일치 테스트 |

## 5. 설계(구현자용)
1. `mode_menu(valid_modes)`: 설명 dict 키 == valid_modes 아니면 AssertionError. 없는 번호는 `sorted(set(range(max+1)) - valid)`.
2. `load_template(path)` → (parsed, labels) 또는 None(파일 없음·yaml 실패·로더 실패 → 사유 한 줄, 기본값 없이 진행). 시작 화면에 "템플릿: <절대경로> (N개)".
3. `template_defaults(template, i)`: i < len → (label, mode, dwell), 아니면 None("템플릿 밖"). i = 현재 len(recorded).
4. `dwell_default(mode, template_dwell)`: 템플릿 > 표 > None.
5. `ask_mode(input_fn, valid_modes, default)`, `ask_dwell(input_fn, default)`: 엔터+기본 → 채택, 엔터+기본 없음 → 재질문(크래시 금지), 허용 밖 정수 → 허용표+번호표 재표시 후 **재질문**(캡처 취소 → 재질문). `is_valid` 최종 검증 유지.
6. `r N` 은 기존대로. 7. `compare_with_template(recorded, template)` → 경고 목록(개수·i 번째 mode·미기록), **저장은 항상**. 8. 캡처 뒤 "다음에 찍을 곳: WP{i+1} …".

## 6. 관련 조항
§1-1, §1-3, §1-4(dwell 표), §1-6(템플릿 못 읽으면 드러내기), §1-9(팀 모드), §3 wp_mode 표, §7.

## 7. 위험
1. `--ros-args` 가 출력 경로가 됨(ROS 파라미터로 받으려면 argv 정리 선행). 2. install 의 옛 템플릿(재빌드 안 함) → 시작 화면 경로·개수로 드러냄. 3. 로더 반환 형태 변경 유혹. 4. 엔터 의미(기본 없을 때 재질문, 0 채택 금지). 5. 삭제 뒤 인덱스. 6. dwell 폴백표 드리프트(테스트로 묶음).

## 8. 검증
- 기존 테스트 8건 유지, `test_waypoint_loader.py`·`test_wp_timeout.py` 불변.
- 추가(순수, `input_fn` 주입): mode_menu 키/없는 번호, load_template 정상·없음·검증실패·라벨없음, template_defaults 안쪽/짧음/None, dwell_default 우선순위·표==소스 yaml, ask_mode `["4","6","abc","","7"]`→7·재질문, 기본 7+`[""]`→7, 기본 None+`["","3"]`→3, ask_dwell 엔터=기본·음수 재질문, compare_with_template 5케이스.
- `tools/run_tests.sh north_goal_angle`; colcon(flake8·pep257) Ubuntu; 수동: 배 노트북에서 가짜 fix 1 Hz 로 번호표·엔터·4 입력·q 경고 확인. 시뮬 불필요.

## 9. 사람이 정할 것
1. 템플릿 순서 강제 vs 경고만(추천 경고만) 2. template_file 전달: (a) 둘째 위치 인자 / (b) `--template=` 플래그(추천) / (c) ROS 파라미터 3. mode 3 문구 비단정 4. dwell 표 코드 상수+테스트(추천) vs yaml 절 5. dwell≥타임아웃 경고(추천 안 함) 6. 캡처 취소→재질문 전환 동의

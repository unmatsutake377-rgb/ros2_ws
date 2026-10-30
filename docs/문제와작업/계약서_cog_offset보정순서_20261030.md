# 변경 계약서 — cog_offset: invert 를 추정 앞으로, mount 이중 적용 제거 (F11 후속, 후보 B)

> ssf-contract 작성 2026-10-30. `계약서_F11_cog_offset이중적용_20261029.md` 를 이어받는다(그 문서의 해법 (a) = 이 문서의 B).
> 상태: **승인 10-30(C2 순서 해제) → 구현 94ed89f·3638fd5 → Ubuntu 검증 완료 10-30**(build·colcon test 76/0·전체 405/0·스모크 2종, `우분투_인계_cog_offset_20261030.md` §4). 남은 것: 문서 갱신(`헤딩_문서정정_모음_20261030.md`), 실기 북·동 2구간 → CLAUDE.md:152 해제.

## 0. 결함 확인과 A·B 판정
- 결함 실재. 추정기 입력 `heading_logic.py:329` 는 원시 r(`update_imu` :309 에서 wrap360 만), 표본 :222 `COG − r`. 출력 :384 `_correct(wrap360(r+off))` → :397-404 → :92-93 이 invert·mount 를 추정 오프셋 위에 또 적용(편각은 :43 False 라 0).
- 물리 모델: 진짜 뱃머리 H = s·r + m (s=±1 invert, m mount), 게걸음각 c, COG = H + c.
- **A("invert 못 다룬다") 의심은 맞고, 실패 모양은 더 나쁘다.** invert=T 이면 표본 a = 2H + c − m.
  - 한 방향 직진: a 상수 → **R≈1 수렴**. A 출력 r + off = 2H₀ − H + c 는 수렴 방향 H₀ 에서만 맞는다.
  - 선회 Δ: 출력이 **거울상으로 2Δ 틀린다.** 선회 중 표본은 `abs(turn_rate) > 8`(:216)로 버려져 R 이 안 떨어짐 → 틀린 값이 선회 내내 OK. 1 Hz 기준 선회 뒤 약 1~2 표본 들어와야 R < 0.9 로 침묵.
  - 즉 "R 이 안 올라간다"는 방향이 섞인 구간에서만 참. 한 방향 직진은 R 이 오르고 그 뒤 선회 응답 부호가 뒤집힌다(조향 양성 되먹임).
  - invert=F 에서는 A·B 출력 동일(r + off, off = m + c).
- **B 권장.** 근거:
  1. invert=T/F 모두 출력 = H + c.
  2. off 의미 "mount + 게걸음"이 imu_relative 용 `mount_offset_deg` 권장값과 같아진다 → `ssf_heading.yaml` gps_vel_frame 절차·`CLAUDE_상세.md:492-493` "off 절대값이 mount 예상치와 맞는지" 가 invert=T 에서도 참(지금은 거짓).
  3. invert=F 경로 추정기 입력 불변 → 기존 테스트 off 값 그대로.
  4. mount 미측정·무의미할 때(`계약서_배별IMU설정_20261029.md:18`, 재부팅마다 원시 정북 바뀜)도 cog_offset 무영향.
- F11-(b)(추정 입력 s·r + m, 출력 s·r + m + off)도 수학적으로 맞지만 off≈c 가 되어 2번 문구가 틀려짐 → 비권장.

## 계약
- **변경 단위**: 노드 하나 `ssf_heading`(yaw_mux), 커밋 하나. 원자 묶음 아님.
  - `heading_logic.py` `update_cog` 추정기 입력(:325-331) → `wrap360(s·r)`. `SRC_COG_OFFSET` 출력(:384) → `wrap360(wrap360(s·r) + off)` (mount·편각 미적용, `_correct` 안 거침).
  - 같은 커밋: `test_heading_logic.py`, docstring(:143-148, :87-91).
  - 🚨 **`update_imu` 의 `_imu` 슬롯은 원시 r 유지.** invert 를 :309 에 넣으면 imu_relative 가 `_from_slot` → `_correct` 에서 **invert 이중** 적용. 테스트 :519·:587 도 `mux._imu[0]` 원시 가정.
  - `_correct`/`apply_corrections` 손대지 않음(imu_relative 비트 불변).
  - 현재 두 배 모두 imu_relative·invert=F·mount=0.0 → **현재 설정 실효 출력 불변.**
- **토픽 계약**: 이름·타입·QoS 전부 불변, 새 토픽 없음 → §3 토픽 표 갱신 불필요.

| 토픽 | 타입 | QoS | 역할 | 반대편 | 변경 |
|---|---|---|---|---|---|
| `/imu/yaw_raw` | Float64 | RELIABLE d2 (yaw_mux.py:120-121) | 구독 | iahrs_driver (launch :110), blackbox.py:190 | 불변 |
| `/ublox_gps_node/fix_velocity` | TwistWithCovarianceStamped | RELIABLE d2 (:124-125) | 구독 | ublox_gps_node, blackbox.py:143 | 불변. C2 때 노드 이름 바뀌면 n=0 침묵 |
| `/motor_reverse` | Bool | RELIABLE d2 (:128) / 발행 d2 (motor_control.py:82) | 구독 | motor_control | 불변 |
| `/imu/yaw` | Float64 | RELIABLE d2 (:129) | 단독 발행 | north_goal_angle.py:188, ship_goal_angle.py:45, ship_turn.py:102, blackbox.py:186, healthcheck | 형식 불변. 값은 cog_offset + (mount≠0 또는 invert=T) 일 때만 바뀜 |
| `/heading_status` | String | RELIABLE d2 (:130) | 발행 | healthcheck.py:105 (BEST_EFFORT) | 형식 불변. `off=` 기준이 invert=T 일 때만 원시 → invert 후 |

- **특수 상수·모드**: 해당 없음. `SOURCE_IS_MAGNETIC[cog_offset]=False` 유지. 선회율(:313)은 원시 r 부호차, 추정기는 `abs()`(:216)만 → invert 위치 무관. 테스트 :568-573 부호 +2 고정 유지.
- **파라미터**: 이름·기본값·yaml·launch 전부 불변. `mount_offset_deg` 는 cog_offset 에서 더 이상 안 쓰임(다른 소스 그대로). `invert_yaw` 는 cog_offset 에서 적용 위치만(추정 앞). `cog_require_reverse_gate`(§7-2) **손대지 말 것**. 배별 파일 키 두 개 제약(`test_boat_config.py:28,52`) 유지.
- **관련 조항**: §1-1, §1-2, §1-3, §1-4, §1-6, §1-9, §3 헤딩 규칙(CLAUDE.md:148, :151-153), §7, §7-2, 상세 §3-5(:433-500).

## 위험
1. invert 를 `update_imu`(:309)에 넣으면 imu_relative 이중 반전 → 거울상 방위, 소비자 3곳.
2. `_run_mux`(test :492-509)는 COG 를 원시 영역 `imu + true_off` 로 만들어 버그를 못 본다 → 새 테스트는 진짜 H 에서 r = s·(H − m), COG = H + c 로 생성.
3. 한 방향 직진은 invert 오설정도 통과 → 실기·테스트 모두 북→동 2구간 이상 + 선회 중 출력 확인. invert 벤치 3단계(배별 yaml 머리말) 여전히 필수.
4. cog_offset 에서 mount 무시하는데 부팅 로그(:137-143)는 `mount=…°` 를 찍음 → 오해 소지(정할 것 4).
5. 계약 밖: `healthcheck.py:221` `endswith(":OK")` 는 yaw_mux 문자열(:196-200, 뒤에 ` n=… rej=…`) 때문에 항상 거짓 → 매번 `⚠️ heading_status=…` 진단 줄. `/health_ok` 판정에는 무영향. ssf_tools 별도 계약.

## 검증 계획
- 기존 그대로 통과: imu_relative :138, :208, :576-587 / 편각 :99-118 / cog 통합 :512-565 / 선회율 :568 / 후진 게이트 :364-450 / `test_boat_config.py` 전체.
- 추가 (H 에서 생성하는 새 헬퍼):
  1. mount=37, invert=F, H=90 → 90±0.5 (지금 127), off≈37
  2. invert=T, mount=0, 동쪽 단일 구간 → 90±0.5 (지금 270)
  3. invert=T, mount=37, 북→동 각 ≥40 표본, 각 구간 끝 OK·출력 H±0.5, off≈37
  4. invert{F,T} × mount{0, 37} × c{0, 5} 격자 → 출력 = H + c
  5. invert=T, 0° 수렴 뒤 >8°/s 로 90° 선회 중 출력이 H 추종 (A/B 가르는 테스트)
  6. declination −8 이 cog_offset 출력에 안 들어감
  7. imu_relative 불변: invert{F,T} × mount{0, 37, 359.9, −12.5} × r{0, 0.1, 89.9, 180, 359.999, −10} 가 `apply_corrections` 와 `==`, COG 입력 중에도
  8. invert=T 에서 `last_reject=="turning"` 이 abs 로 동작
- 실행: `tools/run_tests.sh ssf_heading` → Ubuntu `colcon build --symlink-install && colcon test --packages-select ssf_heading`.
- 실기: source=cog_offset, gate=true, 북·동 직진 → `/imu/yaw − COG` ≈ 게걸음, 두 구간 off 동일, off ≈ 벤치 mount.
- **시뮬 불필요**(`tools/sim/ssf_sim.py` 에 heading_logic 사본 없음).

## 문서 갱신 목록 (코드 커밋 뒤)
- `ssf_heading.yaml` :28-30, :34-44, gps_vel_frame 절차 주석
- `CLAUDE_상세.md` :445-447 (식 `COG − s·r`, `s·r + off`), :492-493
- `CLAUDE.md:152` 금지 문구 — ⚠️ CLAUDE.md 는 Ubuntu 측 원본(Mac 세션 수정 금지). Ubuntu 측에 요청
- `tank_test_plan.html:103`, `과제_목록_전체.md:95`(F11)
- 보고서: `보고서_분담/통합_Ⅲ소프트웨어.md:52`, `00_현재원고_Ⅲ소프트웨어.md:66`, `초안/IMU.md:50-61`, `묶음/보고서묶음_*.md:207`(생성물 여부 ⚠️ 미확인) → docx 재생성
- 상태 표기: `계약서_F11_…md`, `계약서_배별IMU설정_20261029.md:46,57,67`, `새세션_인계_프롬프트_20261029.md:81`, `우분투_인계_계약3건_20261029.md:102`

## 배별 IMU 계약과의 관계
충돌 없음. 그 계약은 이 버그를 별도로 넘겼고(:46, :67), 배별 키 두 개·float 강제 그대로. A/B 모두 invert=false, mount=0.0(미측정).

## 사람이 정해야 할 것
1. 이 문서가 F11 계약서를 대체하나? "GPS C2 다음" 순서 유지 vs 지금 구현? (파일 안 겹침 — C2 는 launch_files 만)
2. B 확정?
3. CLAUDE.md:152 "cog_offset 사용 금지" 해제 시점: 구현 커밋 때 vs 실기 북·동 2구간 확인 뒤?
4. cog_offset 일 때 "mount 미적용" 부팅 INFO 한 줄 추가? (토픽·파라미터 불변)
5. dual_gps 에 IMU invert·mount 가 적용되는 잠복 문제(:361-365 → `_correct`) 이번에 막나, N3 로 미루나?
6. healthcheck.py:221 `endswith(":OK")` 항상 거짓 — 별도 계약으로?

## Davidson 답 (2026-10-30) — 승인
1. **이 문서가 F11 계약서를 대체. 지금 B 구현**(C2 순서 해제 — 파일 안 겹침).
2. B 확정.
3. CLAUDE.md:152 금지 해제는 **실기 북·동 2구간 확인 뒤**(구현 커밋 때 해제 안 함). CLAUDE.md 는 Ubuntu 측 원본.
4. cog_offset 일 때 "mount_offset_deg 미적용" **부팅 INFO 한 줄 추가**(yaw_mux.py, 토픽·파라미터 불변).
5. dual_gps 잠복 문제는 **N3 로 미룸**.
6. healthcheck `endswith(":OK")` 는 별도 계약(이번 범위 밖).

## 진행 (2026-10-30)
- 구현 94ed89f (팀장 지시로 리뷰 결과 전에 Ubuntu 검증용 커밋). run_tests 76 passed.
- 리뷰: 🔴 차단 0. 🟡 INFO 문구 "invert_yaw 는 추정 앞에서만 적용" 이 "출력엔 invert 안 걸림" 으로 읽힐 수 있음 → 후속 커밋에서 문구만 정정. 🟡 yaw_mux 래퍼 변경 → Ubuntu 부팅 스모크(인계 문서).
- 리뷰 검증: 수정 전 코드에서 새 테스트 1~6 실패, 후보 A 로 바꾸면 3·4·5 실패(선회 중 거울상), invert 를 update_imu 에 넣는 변이는 7 이 잡음.
- 🟢 정정: §0 의 "거울상 2Δ" 는 **후보 A** 의 실패 모양이다. 수정 전 코드의 실제 실패는 상수 오차 `m − 2c − 2H₀`(선회는 같은 부호로 추종). 문서 갱신 때 반영.
- 남은 것: Ubuntu build·colcon test·스모크 → 문서 갱신 목록 → 실기 북·동 2구간 → CLAUDE.md:152 해제(Ubuntu 측).

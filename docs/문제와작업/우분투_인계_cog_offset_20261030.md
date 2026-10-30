# 우분투 세션 인계 — cog_offset 보정 순서 수정(F11) 빌드·lint·부팅 확인 (2026-10-30, Mac 세션 작성)

> 이 파일을 Ubuntu 세션에 그대로 붙여 넣으면 된다. 코드는 origin/main 에 있다.
> Mac 에서 한 것: `tools/run_tests.sh ssf_heading` **76 passed / 0 failed**, py_compile 3파일 통과.
> Mac 에서 **못 한 것**(ROS 없음): `colcon build`, `colcon test`(lint), 부팅 스모크. 이 인계의 핵심이다.

## 0. 무엇이 바뀌었나
- 계약서: `docs/문제와작업/계약서_cog_offset보정순서_20261030.md` (F11 계약서 대체, 후보 B, 팀장 승인 10-30)
- `src/ssf_heading/ssf_heading/heading_logic.py`
  - `heading_source: cog_offset` 일 때 invert 를 추정기 **앞**에서만 적용(`_cog_frame_yaw`), 출력은 `wrap360(s·r + off)` — `_correct` 안 거침 → mount·편각 미적용.
  - `_imu` 슬롯(원시값)·`update_imu`·`_correct`·`apply_corrections` 불변 → **imu_relative 출력 비트 불변**.
- `yaw_mux.py`: source=cog_offset 일 때 부팅 INFO 한 줄("cog_offset: mount_offset_deg 미적용 …"). 토픽·QoS·파라미터 불변.
- 테스트 8개 추가(계약서 검증 계획 1~8). 수정 전 코드에선 1~6 이 실패(127°, 270° 등)하는 것을 확인함.
- 현재 두 배 설정(imu_relative, invert=false, mount=0.0)에서는 **실효 출력 변화 0**.
- 🚨 `CLAUDE.md:152` "cog_offset 사용 금지"는 **그대로 둔다** — 실기 북·동 2구간 확인 뒤 해제(팀장 결정).

## 1. 해야 할 검증
```bash
cd ~/ros2_ws && git pull
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ssf_heading
tools/run_tests.sh ssf_heading                     # 기대: 76 passed
source install/setup.bash
colcon test --packages-select ssf_heading; colcon test-result --verbose --test-result-base build/ssf_heading
#   🚨 lint(flake8·pep257·copyright)는 Mac 에서 전혀 안 돌았다. 여기서 처음 본다.
python3 tools/check_params.py                      # 미선언 0 · 불일치 0
```
부팅 스모크 — 기본(imu_relative) 과 cog_offset 둘 다:
```bash
P=src/ssf_heading/config
ROS_DOMAIN_ID=77 timeout 5 ros2 run ssf_heading yaw_mux --ros-args \
  --params-file $P/ssf_heading.yaml --params-file $P/ssf_heading_A.yaml; echo exit=$?
#   기대: Traceback 없음, "cog_offset: mount_offset_deg 미적용" 줄 **없음**, exit 124
ROS_DOMAIN_ID=77 timeout 5 ros2 run ssf_heading yaw_mux --ros-args \
  --params-file $P/ssf_heading.yaml --params-file $P/ssf_heading_A.yaml -p heading_source:=cog_offset; echo exit=$?
#   기대: Traceback 없음, "cog_offset: mount_offset_deg 미적용" INFO 한 줄 **있음**, exit 124
```
전체 launch 도 한 번(센서 없어도 노드는 떠야 함):
```bash
ROS_DOMAIN_ID=77 timeout 10 ros2 launch launch_files launch_files.launch.py 2>&1 | grep -iE "yaw_mux|traceback|error" | head -20
```

## 2. 결과가 틀리면
- flake8 줄 길이·pep257: 주석·독스트링·테스트 범위만 고쳐도 된다. 로직 손대지 말 것.
- 부팅 에러: `yaw_mux.py` 의 `SRC_COG_OFFSET` import 확인.
- 76 이 아니면 실패 테스트 이름과 출력을 그대로 Mac 세션에 붙여 줄 것.

## 3. 다 통과하면
- 이 파일 맨 위에 `> ✅ Ubuntu 검증 완료 2026-10-30: build · colcon test N/0 · 스모크 2종 · launch` 한 줄.
- `docs/문제와작업/과제_목록_전체.md` F11 줄에 "코드 수정 ✅ (커밋 해시, 날짜), 실기 북·동 2구간 남음".
- 실기 절차(물 위): source=cog_offset, `cog_require_reverse_gate: true`, 북·동 직진 각 40 표본 이상 → 두 구간 `off` 동일, `/imu/yaw − COG` ≈ 게걸음각, off ≈ 벤치 mount. 통과 후에만 CLAUDE.md:152 해제.

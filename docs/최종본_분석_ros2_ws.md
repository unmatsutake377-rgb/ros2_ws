# SSF 자율운항선박 ROS2 최종본 분석 (ros2_ws.zip)

> 이 문서는 사용자가 "진짜 대회 최종본"으로 확인한 `ros2_ws.zip`(.git 포함, iahrs IMU, 미션노드 전체)을 분석한 것이다.
> 이전에 본 `ros22_ws.zip`(razor IMU, 단순 버전)과 `SSF 노드 파일` 폴더(짜깁기)는 참고용 변형이며, 이 문서가 기준이다.
> **중요:** 이 최종본은 이전 버전들에서 지적됐던 정지·후진·회전 버그가 이미 고쳐져 있다.

---

## 1. 어떻게 최종본으로 판단했나
- IMU가 **iAHRS**로, 사용자의 실제 하드웨어와 일치.
- 미션노드(gate/dock/turn/back/last)와 wp_mode FSM이 전부 존재하는 가장 완성된 구성.
- `.git` 이력 포함(대회 시점 커밋 존재), motor_control이 재작성된 최신 형태.
- 사용자가 직접 최종본으로 확인함.

## 2. 시스템 구조 한눈에

```
GPS(/ublox/navpvt, /fix) ┐
                         ├─▶ iahrs_driver ─▶ /imu/yaw (Float64, GPS heading override 포함)
IMU(iAHRS, 시리얼) ───────┘                       │
                                                  ▼
north_goal_angle ─▶ /wp_mode, /goal_distance, /north_goal_angle_tp, /candidate_angle
   │                                              │
   │                            ship_goal_angle: /imu/yaw + /north_goal_angle_tp ─▶ /yaw_error
   │                                              │
미션노드(gate/dock/turn/back/last) ─▶ /candidate_angle
   │                                              │
비전(basic_image_subscribermode → gate/dock/turn) ─▶ /image_angle,/image_distance,/image_color,/red_angle...
                                                  ▼
LiDAR(/scan) ─▶ ship_direction : candidate + yaw_error + LiDAR 융합 ─▶ /desired_angle, /obstacle_distance_array
                                                  ▼
                             motor_control : /desired_angle ─▶ Motor_run(Int32=pwm_r*10000+pwm_l)
                                                  ▼
                        micro-ROS agent ─▶ Arduino Due ─▶ ESC ×3 (R/L/M)
```
- 제어 컨벤션: **전방 = 80°**. LiDAR 전방 0~160° 사용. 미션은 `wp_mode` 하나로 조율.

## 3. wp_mode ↔ 미션노드 매핑

| wp_mode | 담당 노드 | 비전 노드 | 동작 |
|---|---|---|---|
| 0 | ship_gate | basic_image_subscribergate | 게이트(red/yellow 부표) 중앙 통과 |
| 1 | ship_back | (없음) | 진입 후 5초 정지 → 폴백(yaw_error 주행) |
| 2 | ship_dock | basic_image_subscriberdock | 좌우 탐색 회전 → 도형 도킹 → 정지 → 후진 |
| 3 | ship_turn | basic_image_subscriberturn | 부표 색으로 좌/우 판단, LiDAR로 선회 |
| 4 | ship_last | (없음) | candidate=20000 지속(yaw_error 주행) |
| 5~ | north_goal_angle | (없음) | 미션 종료/폴백 |

`basic_image_subscribermode`가 wp_mode에 따라 해당 비전 노드를 서브프로세스로 켰다 껐다 함.

## 4. 노드별 요약

**iahrs_driver (C++)** — 시리얼 IMU에서 yaw를 읽어 CCW→CW 변환, 부팅 시 첫 값을 0점(offset)으로. `/ublox/navpvt`의 GPS heading이 유효(head_acc<30°)하면 **IMU yaw를 GPS heading으로 대체**. `/imu/yaw`(Float64,0~360)와 `imu/data`(Imu) 발행. 100Hz.

**north_goal_angle** — GPS 웨이포인트 FSM. `wp_idx=0`부터 시작(정상). 도착(3.0m 이내)+dwell초 → 다음 WP. `/wp_mode`, `/goal_distance`, `/north_goal_angle_tp`(목표방위), `/candidate_angle`(wp_mode 7에서 20000) 발행. 웨이포인트 좌표는 실측(35.18xx,128.56xx, 부산권).

**ship_goal_angle** — `/imu/yaw`와 `/north_goal_angle_tp`로 `yaw_error=(goal-yaw)%360` 계산해 `/yaw_error`(Float32) 발행. 0.5초 타이머.

**ship_direction** — `/scan`+`/yaw_error`+`/candidate_angle`+`/wp_mode` 융합. LiDAR 갭-팔로잉으로 통과 가능한 안전구역 중 목표방위에 가까운 곳으로 조향. 특수값 처리: candidate 20000(폴백)/50000(정지)/5000·6000(회전). 회피 경로 없으면 후진 260°. Lock으로 스레드 보호(개선됨). `/desired_angle`, `/obstacle_distance_array` 발행.

**motor_control (재작성판)** — `/desired_angle`만 구독. 각도 구간별 PWM 테이블:
- `≥50000` → **정지** 1500/1500
- `20000~50000` → 폴백 전진 base_pwm
- `5000~20000` → 느린 좌선회
- `-1~161` → 80° 기준 정상 조향
- `161~5000`(260 후진 포함) → **후진** 1590/1590
- 그 외 → 직진
`base_pwm=1360` 순항. 1500=중립. **모든 PWM이 하드코딩(ROS 파라미터 아님).**

**color_shape_detector** — `basic_image_publisher`(카메라)+`basic_image_subscribermode`(wp_mode별 비전노드 매니저)+gate/dock/turn/hsv 인식 노드들. RealSense depth로 부표/도형 거리·각도 산출.

## 5. 이전 버전 대비 이미 해결된 것 (재지적 금지)
- **정지(50000) 실제 작동** → 1500/1500 중립. (예전엔 미처리였음)
- **후진 작동** → 260이 후진 구간(161~5000)에 들어가 1590. (예전 260 vs 280 불일치 해소, motor_control 재작성)
- **회전(5000/6000) 처리** → 좌선회로 반영.
- **/imu/yaw 정상 발행** → iahrs가 하드웨어와 정합.
- **yaw_error 타입 일치**(Float32), ship_direction에 Lock 추가.

## 6. 남아 있는 문제·불안 요소 (이 최종본 기준)

### A. IMU/GPS heading (iahrs_driver) — 가장 주의
1. **GPS heading이 NaN으로 리셋되지 않음.** `gps_heading`은 유효한 NavPVT가 한 번 들어오면 계속 non-NaN이고, 이후 매 사이클 IMU yaw를 GPS heading으로 덮어씀. **GPS가 끊겨도 마지막 GPS heading(stale)에 그대로 고정**되어 IMU가 영구 무시됨. 타임아웃 리셋이 필요.
2. **NavPVT heading은 진행방향(COG)** 이라 배가 움직일 때만 의미 있음. 정지/저속에선 노이즈이고, 조류에 밀리면 선수방위와 다름. 이를 선수 heading으로 쓰는 건 위험. 신뢰 임계값 30°도 느슨함.
3. **기준 프레임 불일치.** IMU 경로는 부팅 상대(0점화), GPS 경로는 절대값 → GPS override가 붙었다 떨어질 때 yaw가 불연속으로 점프.
4. **시리얼 포트 고정(#define)** — 새 노트북/배에선 포트가 달라져 못 열 수 있음.

### B. 방위 제어 루프 (ship_goal_angle)
5. **0.5초 주기로 느림.** IMU는 100Hz인데 yaw_error가 2Hz로만 갱신 → 방위 반응 굼뜸. 배가 커지면 사행/오버슈트 위험. 주기 단축 권장.
6. **yaw 신선도 체크 없음.** `/imu/yaw`가 멈춰도 마지막 yaw로 계속 계산.
7. **부호 검증 필요.** ship_direction이 `yaw_error=(360-raw)%360`으로 방향을 뒤집음. IMU 증가방향·LiDAR 각도방향·조향방향이 실제로 일치하는지 벤치에서 확인 필요("반대로 도는" 버그의 단골 지점).

### C. 모터/추진 (motor_control + 펌웨어)
8. **PWM 전부 하드코딩.** base_pwm 1360, 후진 1590, 정지 1500이 코드 상수. **A배/B배 크기가 다른데 배별 튜닝을 코드 수정으로만 가능**(예전 파라미터판에서 후퇴). ROS 파라미터화 필요.
9. **감속 로직 제거됨.** 이 재작성판은 장애물/목표거리 기반 감속을 안 함(순항 고정). 장애물 근처에서도 속도 유지 → 충돌 여유 감소.
10. **펌웨어 `It_is_Aship==1` 오타** — 주석 A=0/B=2인데 코드가 ==1 검사 → B배 분기가 죽어 A배 범위를 씀. 두 배 사용 계획이라 필수 수정.
11. **소프트웨어 페일세이프 없음** — motor_control은 마지막 desired_angle 유지, 펌웨어는 워치독 없어 마지막 명령 유지. 신호 끊김 시 정지하지 않음. 무 RC 시 기본 오토파일럿. 유일 안전장치는 사람 RC 수동전환.
12. **미들(M) 스러스터** 오토파일럿 모드에서 미제어.

### D. 비전
13. gate/dock/turn 노드 `main()`의 `node.pipeline.stop()` — 존재하지 않는 속성(종료 시 에러).
14. `cv2.imshow`/`waitKey` — 헤드리스 환경 불가/성능 저하.
15. depth-color 정렬·HSV 튜닝값 카메라 의존.

### E. 저장소 위생
16. `src/` 안에 잡파일 `GGGGGGGGGGGGGG.zip`, `sgs.zip` 존재(삭제 권장).
17. `build/`, `install/`, `log/`, 벤더링된 `realsense-ros`가 저장소에 포함돼 용량 큼(260MB). `.gitignore` 정리 권장.

## 7. 확인이 필요한 것 (하드웨어/의도)
- **A배/B배 각각의 ESC PWM 범위와 실제 중립값** (새 선체 재캘리브레이션).
- **GPS heading override가 의도된 주 방식인지**, 신뢰 임계값(30°)과 정지 시 처리.
- **미들 스러스터** 장착/역할.
- **페일세이프 의도** (RC 끊김→오토, ROS 끊김→마지막값 유지를 유지할지 중립정지로 바꿀지).

## 8. 권장 우선순위
1. iahrs: GPS heading 타임아웃 리셋(끊기면 NaN 복귀) + 저속 시 IMU 우선 로직.
2. 페일세이프: 펌웨어 워치독(수신 N ms 없으면 중립) + ship_goal_angle/ship_direction 신선도 체크.
3. motor_control PWM 파라미터화 + 펌웨어 It_is_Aship 수정(A/B 분리).
4. yaw_error 부호·프레임 벤치 검증(전방 80° 정렬).
5. ship_goal_angle 주기 단축(0.5→0.05~0.1s).
6. 비전 정리(pipeline.stop 제거, imshow 플래그화), 저장소 잡파일 정리.

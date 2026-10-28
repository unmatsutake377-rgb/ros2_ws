# 지시서 03 — LiDAR (장애물 회피 · 페일세이프)

## 담당 목차와 분량

**양식 파일: `양식_LiDAR.docx`** — 아래 절의 번호·제목·빈 표가 본 보고서 모양으로 들어 있다. 여기에 채워서 낸다.

| 목차 | 지금 원고 | 할 일 |
|---|---|---|
| Ⅲ-1.3 센서 표의 LiDAR 줄 | 한 줄 | 장비 사진 1장, 개체 구분 설명 |
| **Ⅲ-2.3 LiDAR 처리** | 글머리표 4개 | **주 담당.** 공통 양식 8항목으로 확장 |
| **Ⅲ-4.4 장애물 회피 — ship_direction** | 문단 + 파라미터 표 | 알고리즘 상세. 2.3 과 겹치지 않게 2.3 은 "신호 처리", 4.4 는 "경로 선택"으로 나눈다 |
| Ⅲ-5 페일세이프 중 센서 감시 부분 | 글머리표 | `SensorWatch` 부분 보강 |
| Ⅲ-6.1 시뮬레이션 스윕 | 표 + 문단 | 숫자 대조, 설명 보강 (값의 근거가 이 시뮬이다) |

분량: **A4 2~2.5쪽** (표 3~4개, 그림 2장).

## 핵심 한 줄

`ship_direction` 이 LiDAR 스캔에서 배가 지나갈 수 있는 빈 구간을 찾아, 목표 방위에 가장 가까운 구간으로 조향각을 낸다. 올해는 배 폭을 실측해 통과 기준을 다시 잡았고(0.45 → 0.29 m), 물보라에 속지 않도록 감속 신호에만 median 필터를 걸었고, 센서가 죽으면 감속·정지하는 페일세이프를 독립 타이머로 분리했다.

## 반드시 답할 질문

1. RPLIDAR A3 개체는 배마다 무엇이고, 회전 주기와 각분해능이 어떻게 다른가? 그 차이가 셀 단위 파라미터에 왜 중요한가?
2. 드라이버는 그대로 쓰나? 우리가 바꾼 것은 무엇인가? 두 개체를 노트북이 어떻게 구분하나(udev)?
3. `/scan` 의 정면은 몇 도인가(`LIDAR_FORWARD_DEG`)? 구독 QoS 는?
4. `inf`·`nan` 셀은 어떻게 처리하나? 탐지 거리 기본값과 게이트 모드 값은?
5. 안전 구간을 어떻게 찾고, 통과 가능 판정 `min_required_width = half_width×2 + clearance` 와 호 길이 계산은 어떻게 하나? 가장자리 거리로 `r_edge` 를 쓰는 이유는?
6. 후보 구간 중 하나를 고르는 기준은? 특수 명령(5000, 20000, 50000)을 받으면?
7. median 필터는 어디에만 걸고 어디엔 안 거나? 이유와 시뮬 수치(가짜 감속 36.8% → 8.9%)는?
8. 갈 곳이 없으면 후진하나? 후진 시간·쿨다운·조향 변화율 제한 값은?
9. 지오펜스 광선을 실제 스캔과 어떻게 합치나? 경계에 가까우면 정지하는 조건은?
10. 페일세이프: 경고 0.7 s, 정지 3.0 s, 연속 N 회, 복구 조건. 왜 센서 콜백이 아니라 독립 타이머에서 판정하나?
11. 시간 투표(`TemporalVote`)는 왜 꺼 두었고, `rear_obstacle_ignore` 는 왜 없앴나? (TTC 비상제동은 이 노드가 아니라 `motor_control` 단계에서 기각됐다 — 팀장 절)
12. 시뮬 스윕(Ⅲ-6.1)은 어떤 조건을 몇 번 돌렸고, 그 결과로 어떤 값을 골랐나?
13. 단위 테스트는 몇 개이고 무엇을 검증하나?
14. 실측이 없는 것은? 부표 탐지 최대거리(과제 D4), 물보라 기각 수.

## 읽을 파일

| 파일 | 역할 | 우선 |
|---|---|---|
| `src/ship_direction/ship_direction/ship_direction.py` | 회피 노드 전체. 921줄 — 함수 단위로 읽게 할 것 | ★★★ |
| `src/ship_direction/ship_direction/failsafe.py` | `SensorWatch`, `TemporalVote`, `median_min` 순수 로직 | ★★★ |
| `src/ship_direction/config/ship_direction.yaml` | 파라미터 값과 주석 근거 | ★★★ |
| `tools/sim/README.md` | 시뮬 스윕 조건과 결과 | ★★ |
| `src/rplidar_ros/launch/rplidar_a3_launch.py` | 외부 드라이버 실행 설정. 우리가 정리한 부분만 | ★ |
| `src/ship_direction/launch/ship_direction_launch.py`, `src/launch_files/launch/launch_files.launch.py` | 실행 설정 | ★ |
| `src/ship_direction/test/` | 단위 테스트 | ★ |

관련 문서: `CLAUDE.md` 5장(페일세이프)·6장(시뮬 파라미터), `docs/전달용/LiDAR_노드_쉬운설명.md`, `docs/전달용/회신_rplidar_회피_수정제안.md`, `docs/전달용/회신_ship_direction_수정제안.md`, `docs/문제와작업/라이다_고장진단_20260818.md`, `docs/절차/2번배_udev_규칙_초안.md`

## 코드 출처

| 패키지 | 출처 | 작년부터 | 올해 바꾼 것 (근거 커밋) |
|---|---|---|---|
| `rplidar_ros` | **외부.** Slamtec 공식 RPLIDAR ROS 드라이버 | 있음 | A3 실행 파일 정리만(`673c20a`). 드라이버 코드는 그대로 |
| `ship_direction` | 작년 우리 코드 | 있음 | 페일세이프 분리(`02bfc39`, `2f4410a`), 시간 투표 OFF(`fd65e2f`), median(`849c570`), 지오펜스 병합(`b678154`), 후진·rate limit(`ed31dda`), half_width 0.45→0.29(`000e46c`) 등 |

## 작년 대비 변경 보는 명령

```bash
cd ~/ros2_ws
git log --format='%h %ad %s' --date=short 9bff661..HEAD -- src/ship_direction src/rplidar_ros
git diff 9bff661 HEAD -- src/ship_direction
git diff 19cfde7 HEAD -- src/rplidar_ros
```

🚨 `rplidar_ros` 는 `9bff661` 이 아니라 **`19cfde7` 기준**으로 본다. 그 전에는 외부 저장소 링크 형태여서 `9bff661` 과 비교하면 드라이버 전체 9만 줄이 "추가"로 나온다. 우리가 쓴 코드가 아니다.

## 경계

| 내용 | 누가 | 이 절에서는 |
|---|---|---|
| 지오펜스 **병합**과 경계 정지 | **LiDAR** | 전부 씀 |
| 지오펜스 광선 **생성** | GPS | "GPS 담당 절 참고" 한 줄 |
| 카메라 방위 + LiDAR 거리 조합 | 카메라 | "거리는 LiDAR 전방 섹터 최소거리" 한 줄 |
| 임무 노드(`ship_gate`·`ship_back`·`ship_turn`·`ship_dock`)의 `/scan` 사용 | 팀장 | 언급만 |
| 감속비 적용(`motor_control`) | 팀장 (Ⅲ-4.6) | "`/obstacle_distance_array` 를 받아 감속한다" 한 줄 |

## A배·B배

| | A배 (1번배) | B배 (2번배) |
|---|---|---|
| 개체 | 80BF (약 11.1 Hz / 0.250°) | 918A (11.0 Hz / 0.223°) |
| 배 폭 실측 | 0.57 m | 0.535 m |
| `half_width` | 공용 0.29 | 공용 0.29 (실측 폭 0.535 → 0.2675. 0.29 로 시뮬 접촉 0.95·정체 0 확인. 0.215 는 옛 설계폭 0.43 기준 값이라 쓰지 않는다) |

## 빈칸으로 둘 실측

| 항목 | 언제 |
|---|---|
| 부표 탐지 최대거리 (블랙박스 `obstacle_min_dist`) | 물 위 (과제 D4) |
| 물보라 기각 수 `/obstacle_reject_count` (회/분) | 야외 rosbag |
| 1 m 안팎 게이트 통과 성공률 | 수조 게이트 시험 |

## 그림 제안

1. 갭 선택 개념도: 스캔 → 장애물 팽창 → 안전 구간 → 폭 판정 → 목표에 가까운 구간 선택
2. 페일세이프 상태도: 정상 → 경고(감속) → 정지 → 복구

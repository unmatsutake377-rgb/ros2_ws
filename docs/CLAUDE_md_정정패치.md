# CLAUDE.md 정정 패치 — 그대로 Claude Code 에 붙여넣으세요

> **내가 CLAUDE.md 에 사실 오류를 하나 넣었습니다.** Claude Code 가 코드를 직접 읽고 바로잡았습니다.
> 아래 내용으로 `CLAUDE.md` 를 갱신해야 다음 단계들이 틀린 전제 위에 쌓이지 않습니다.

---

## Claude Code 에 이렇게 시키세요

```
CLAUDE.md 를 아래 내용으로 갱신해줘.

1) 3-6절의 wp_mode 매핑표가 틀렸다. 네가 코드에서 읽은 게 맞다.
   아래 '실제 웨이포인트 표' 로 교체할 것.

2) 3-9절(토픽 이름 확정)을 새로 추가할 것.

3) healthcheck 를 고칠 것: mode 5 와 8 은 담당 노드가 없는 것이 정상이다.
   이걸 '누락' 으로 경고하면 매 실행마다 거짓 경보가 뜬다. 예외 처리할 것.

4) blackbox 의 게이트 통과 토픽 이름을 /gate_pass_count → /gates_passed 로 바꿀 것.

끝나면 커밋하고 diff 를 보여줘.
```

---

## 3-6 교체본 — 실제 웨이포인트 표

`north_goal_angle.py` 에서 그대로 발췌한 것:

| WP | mode | 내용 | 담당 노드 (작년 선언값) | 상태 |
|---|---|---|---|---|
| 0 | **0** | 게이트 시작 | `ship_last` (0) | ⚠️ 게이트인데 비전을 안 쓴다 (GPS 로만 접근) |
| 1 | **1** | 게이트 끝 | `ship_gate` (1) | ✅ |
| 2 | **2** | 위치유지 | `ship_back` (2) | ✅ |
| 3,4,5 | **3** | 초록·빨강·하양 부표 | `ship_turn` (3) | ✅ |
| 6,7 | **5** | 회피 구간 | **없음 — 정상** | ✅ `ship_direction` 순수 회피 |
| 8,9 | **7** | 도킹 | `ship_dock` 은 **9** 로 선언 | 🚨 **침묵** |
| 10 | **8** | 토너먼트 회피 | **없음 — 정상** | ✅ |

### ⚠️ `mode 5`, `mode 8` 은 담당 노드가 없는 것이 정상이다

순수 장애물 회피 구간이라 `ship_direction` 이 GPS 방위로만 간다.
**healthcheck 가 이걸 '누락' 으로 경고하면 매 실행마다 늑대소년이 된다.**
매핑표에 `owner: none` 으로 **명시적으로** 적고 예외 처리할 것.

### 고칠 것

- **`ship_dock` 의 `active_wp_mode` 를 9 → 7** (도킹 부활)
- **`mode 0` 을 `ship_gate` 가 맡게** 한다 — 게이트 접근 구간부터 비전을 쓰는 편이 정렬 시간을 번다
  (작년엔 `ship_last` 가 mode 0 을 잡고 GPS 폴백만 냈다. `ship_last` 는 제거)
- 즉 새 `ship_gate` 는 **`active_wp_modes: [0, 1]`**

---

## 3-9 신설 — 🔒 토픽 이름 확정

신설 토픽은 **여러 단계에 걸쳐** 만들어진다.
이름이 어긋나면 **에러 없이 조용히 빈 값**이 된다. 지금 못 박는다.

| 토픽 | 타입 | 발행자 | 신설 단계 | 구독자 |
|---|---|---|---|---|
| `/health_ok` | `Bool` | `healthcheck` | **0단계** | (사람이 봄) |
| `/failsafe_level` | `Int32` | `ship_direction` | **3단계** | `blackbox` |
| `/gates_passed` | `Int32` | `ship_gate` | **5단계** | `blackbox` |
| `/geofence_state` | `Float32MultiArray` | `north_goal_angle` | **6단계** | `ship_direction` |

**🚨 `/gate_pass_count` 아님. `/gates_passed` 다.**
(0단계 blackbox 가 `/gate_pass_count` 로 구독했다. 5단계 `ship_gate` 는 `/gates_passed` 로 발행할 예정이라
**그대로 두면 게이트 통과 수가 영원히 빈칸이 된다.** 지금 고칠 것.)

---

# 📋 0단계 검토 결과 (참고)

## 잘한 것

- **토픽 이름·타입을 먼저 조사했다.** ROS2 에서 토픽 이름이 틀리면 **에러 없이 조용히** 아무것도 안 들어온다. 이걸 먼저 확인한 건 정확히 옳다.
- **`wp_mode` 매핑표를 코드에서 직접 읽어 내 오류를 바로잡았다.** (CLAUDE.md 는 `ship_gate: 0,1` 이라 했지만 실제는 `ship_gate: 1`, `ship_last: 0`)
- **독립 타이머로 평가** — CLAUDE.md 5장의 "LiDAR 가 죽으면 콜백이 안 와서 페일세이프가 영원히 평가되지 않는다" 를 제대로 이행.
- **ARMED 플래그** — 부팅 중 오탐 방지.
- **`time.monotonic()`** 사용.
- **`colcon build` 를 못 돌렸다고 정직하게 밝혔다.** 숨기지 않은 것이 가장 중요하다.

## 관찰자 QoS 에 대한 정확한 평가

`BEST_EFFORT` / `VOLATILE` 로 구독한 건 **방어적으로 옳지만, 이번 경우엔 필수는 아니었다.**

- ROS2 QoS 규칙: 발행자 `BEST_EFFORT` + 구독자 `RELIABLE` → **조용히 연결 안 됨** (진짜 함정)
- 반대로 발행자 `RELIABLE` + 구독자 `BEST_EFFORT` → 정상 연결
- **그런데 이 `rplidar_node.cpp` 는 `rclcpp::QoS(KeepLast(10))` = RELIABLE 로 발행한다.**
  → 어느 쪽으로 구독해도 받았을 것이다.

**결론: 무해하고 방어적인 선택. LiDAR 드라이버를 바꾸거나 카메라 토픽을 추가로 물릴 때 실제로 도움이 된다. 그대로 두면 된다.**

## 확인이 필요한 것

1. **`/gate_pass_count` → `/gates_passed`** (위 3-9절)
2. **`mode 5`, `8` 을 healthcheck 가 '누락' 으로 경고하지 않는지** — 경고하면 거짓 경보
3. **`colcon build` 는 반드시 우분투에서** — Mac 에서 문법만 통과한 상태다

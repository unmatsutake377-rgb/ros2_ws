# 리뷰: `motor_run_bridge_simple.py` (2025 발굴 코드) — 회로 담당 AI 전달용

> 대상: `reference/2025_bridge/motor_run_bridge_simple.py`
> 출처: **작년(2025) 자료에서 발굴.** 새로 받은 파일 아님.
> 검토 기준: 저장소 최신 `src/motor_control/motor_control/motor_control.py`, `arduino/ssf_boat/ssf_boat.ino`
> 한 줄 결론: **작년 코드지만 현재 Mega 펌웨어와 계약이 그대로 맞는다. "빠져있던 시리얼 브리지"를
> 새로 짤 필요 없이 이걸 되살리는 선택지가 생겼다 — 단 포트·패키징·견고성 3가지만 손보면 된다.**

이 폴더는 `COLCON_IGNORE` 라 빌드되지 않는다(참고 보관용). 되살릴 경우 별도 패키지로 복사해 수정한다.

---

## 0. 이게 왜 중요한가

우리 Motor_run 사슬에서 **비어 있던 조각**이 이거다:

```
motor_control.py  →발행→  Motor_run(토픽)  →[여기 브리지]→  시리얼 → Mega(ssf_boat.ino)
   (저장소 있음)                              (그동안 없었음)         (저장소 있음)
```

micro-ROS 를 폐기하고 시리얼 브리지로 바꾸면서 이 노드가 필요했는데, 저장소엔 없었다.
그런데 **작년 자료에 이미 있었다.** 그리고 아래처럼 **현재 펌웨어와 계약이 일치한다.**

---

## 1. 현재 펌웨어와의 계약 호환성 — ✅ 전부 일치

| 항목 | 브리지(작년) | 현재 `ssf_boat.ino` | 일치 |
|---|---|---|---|
| 시리얼 형식 | `f"L{pwmL},R{pwmR}\n"` | ino:139 `"L<좌µs>,R<우µs>\n"`, ino:146 `if (s[0]!='L') return` | ✅ |
| 보드레이트 | 기본 `115200` | ino:62 `SERIAL_BAUD=115200` "브릿지와 합의된 속도(브릿지 기본값)" | ✅ |
| Motor_run 디코드 | `pwm_r=data//10000`, `pwm_l=data-pwm_r*10000` | `motor_control` 인코드 `pwm_r*10000+pwm_l` 과 역연산 일치 (blackbox 도 동일) | ✅ |
| 구독 QoS | 기본(RELIABLE, depth 10) | `motor_control` 발행 `create_publisher(Int32,"Motor_run",2)` = 기본 RELIABLE → 호환 | ✅ |
| L/R 매핑 | `send_pwms(pwm_l, pwm_r)` → L=좌, R=우 | ino 가 L/R 을 스러스터 4개로 팬아웃 | ✅ (물리 좌우는 벤치 확인) |

**즉 로직·계약은 손댈 게 없다.** 되살리면 바로 통신된다.

---

## 2. 손봐야 할 것 (되살릴 경우)

### 2-1. ⚠️ 포트 기본값 `/dev/ttyACM1`
Mega 하나만 꽂으면 보통 `/dev/ttyACM0`. 실제 포트를 `ls /dev/ttyACM*` 로 확인하고,
**udev 심링크(예 `/dev/ttyMEGA`)로 고정** 권장(재부팅마다 번호가 바뀌는 문제 차단).

### 2-2. 패키징 (지금은 실행 불가)
- 어느 패키지에도 안 들어가 있다 → `ros2 run` 도 안 된다.
- 되살리려면: 별도 패키지(예 `ssf_bridge`)에 넣고 `setup.py` 엔트리포인트 + `package.xml` 에
  `python3-serial`(pyserial) 의존 추가 + launch 배선.

### 2-3. 견고성 (안전엔 무방 — 펌웨어 워치독이 커버)
- **종료 시 중립 미발신**: Ctrl+C 시 마지막 PWM 유지한 채 시리얼이 닫힌다. 다만 **펌웨어 워치독
  (ino:53 `CMD_TIMEOUT_MS=500`)이 500ms 무수신 시 중립**으로 잡으므로 폭주는 없다.
  그래도 `finally` 에 `send_pwms(1500,1500)` 한 줄 넣으면 더 확실하다.
- **아두이노 상태 피드백 무시**: 펌웨어가 같은 시리얼로 `S,모드,워치독,배ID,비상정지,FL,FR,RL,RR`
  를 10Hz 로 되쏜다(ino:223). 브리지가 이걸 안 읽어 입력 버퍼가 쌓인다. 최소 주기적
  `reset_input_buffer()`, 이상적으론 **읽어서 로깅**(수조 테스트 때 `watchdogActive`·`estop`·
  `boatId`·모터별 출력 확인에 매우 유용).
- **`ser.write` write_timeout 없음**: 아두이노가 멈추면 executor 가 블록될 수 있다. 우선순위 낮음.

---

## 3. 회로 AI 판단용 요약

| 항목 | 상태 |
|---|---|
| 계약(형식·baud·디코드·QoS·L/R) | ✅ 현재 펌웨어와 일치 — 새로 짤 필요 없음 |
| 포트 기본값 | ⚠️ ACM1 → 실측 + udev 고정 |
| 패키징 | ❌ setup.py/pyserial/launch 필요 |
| 안전(폭주) | ✅ 펌웨어 워치독이 차단 |
| 권장 개선 | 종료 중립 + 상태 피드백 읽기 |

**결정 사항(회로 AI):** 이 2025 브리지를 되살릴지(권장 — 계약이 이미 맞음), 새로 짤지.
되살리면 위 2-1~2-3 만 반영하면 된다. 어느 쪽이든 **Motor_run(Int32, `pwm_r*10000+pwm_l`) 계약과
`L<µs>,R<µs>\n` @115200 형식은 유지**해야 `motor_control` · 펌웨어와 맞는다.

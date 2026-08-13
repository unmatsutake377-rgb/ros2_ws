// =====================================================================
// SSF 자율운항선박 펌웨어 — Arduino Mega 2560 R3, 시리얼 브릿지 방식
// 설계 근거: docs/전달용/펌웨어_설계문서.md (단일 출처, 여기 주석은 요약)
//
// [2026-07-28] 보드 확정: Mega 2560 R3 (Due에서 이식)
//   노트북 motor_control ──(ROS Motor_run)──▶ 브릿지 노드 ──USB 시리얼──▶ 이 펌웨어
//   브릿지가 보내는 명령: "L1500,R1500\n" (µs 단위, 1500=중립, <1500=전진)
//   이 펌웨어의 상태 보고: 같은 USB로 "S,모드,워치독,배ID,비상정지,FL,FR,RL,RR\n" 10Hz
//   (Mega는 USB 1개 — 명령 수신과 상태/디버그 송신이 같은 포트.
//    브릿지는 수신을 안 하므로 상태 줄이 흘러가도 무해. 벤치에선 IDE 시리얼
//    모니터로 그대로 보임. 추후 상태를 ROS로 올리려면 브릿지에 읽기 추가 필요.)
//
// Mega 이식 포인트 (Due 대비):
//   · 5V 보드 → RC 수신기 신호 직결 가능 (레벨시프트 불필요 — 회로 단순해짐)
//   · RC 입력은 반드시 인터럽트 가능 핀에: 2, 3, 18, 19, 20, 21 중에서만
//   · 8비트 보드 → 4바이트 변수(unsigned long)를 ISR과 나눠 쓸 때
//     읽는 순간 인터럽트를 잠깐 꺼서 스냅샷 (Due는 불필요했던 보호)
//
// 대회 3모드 → 펌웨어 2상태:
//   AUTO   = 브릿지 명령 (자율 본경기·자율 토너먼트)
//   MANUAL = RC 조종 (수동 토너먼트 — 노트북 없이 단독 가동)
// =====================================================================

#include <Servo.h>

// ── RC 배선 진단 (기본 꺼짐) ─────────────────────────────────────────────
// 🚨 "모드가 대기(0)에서 안 바뀐다" 를 디버깅할 때만 켠다.
//    모드는 핀 18 만 보므로, 평소엔 **어느 핀까지 배선이 됐는지 알 수가 없다.**
//    켜면 1초마다 세 채널의 펄스 폭을 찍어 배선을 눈으로 확인할 수 있다.
//    켜는 법(코드 수정 없이):
//      arduino-cli compile --fqbn arduino:avr:mega \
//        --build-property compiler.cpp.extra_flags=-DRC_DEBUG=1 arduino/ssf_boat
//    ⚠️ 대회 펌웨어에는 **끈 채로** 올릴 것 — 브릿지가 이 줄을 못 알아본다(경고만 나지만 시끄럽다).
#ifndef RC_DEBUG
#define RC_DEBUG 0
#endif

// ── RC 핀 전수 스캔 (기본 꺼짐) ─────────────────────────────────────────
// 🚨 "선이 어느 핀에 연결됐는지 모르겠다" 를 풀기 위한 **진단 전용** 빌드다.
//    만능기판에 납땜된 배선은 눈으로 추적이 안 된다(08-13 사진으로 확인 실패).
//    → 인터럽트 가능 핀 **6개 전부**에 인터럽트를 걸고 어디에 펄스가 들어오는지 찍는다.
//    켜는 법:
//      arduino-cli compile --fqbn arduino:avr:mega \
//        --build-property compiler.cpp.extra_flags=-DRC_SCAN=1 arduino/ssf_boat
//    ⚠️ 스캔 중에는 **정상 RC 경로가 동작하지 않는다**(같은 핀에 인터럽트를 덮어쓴다).
//       모드는 대기(0)에 머문다 — 고장이 아니다. 확인이 끝나면 반드시 끄고 재업로드할 것.
#ifndef RC_SCAN
#define RC_SCAN 0
#endif

// ── 전 디지털 핀 사냥 (기본 꺼짐) ───────────────────────────────────────
// 🚨 RC_SCAN 은 **인터럽트 가능 핀 6개만** 본다. 거기 다 없으면 선이 다른 핀에 있다는 뜻이라
//    더 넓게 훑어야 한다. 이건 `pulseIn` 폴링으로 **모든 디지털 핀**에서 RC 펄스를 찾는다.
//    켜는 법: --build-property compiler.cpp.extra_flags=-DPIN_HUNT=1
//    ⚠️ 스윕 한 번에 최대 1.5초 블로킹이라 **제어 루프가 멈춘다.** 순수 진단 전용.
//    ⚠️ ESC 출력 핀(5~8)과 스트립 핀은 건드리지 않는다 — 출력 핀을 INPUT 으로 바꾸면
//       ESC 신호가 끊긴다.
#ifndef PIN_HUNT
#define PIN_HUNT 0
#endif

// 룰 표시등 방식 선택 (2026-07-29 회로팀 그림 기준: WS2812 주소지정형 스트립)
//   1 = WS2812 스트립 (현재 계획. IDE 라이브러리 매니저에서 "Adafruit NeoPixel" 설치 필요)
//   0 = 단색 LED 3개 (예비 — 스트립 아닌 걸로 판명되면 이걸로 복귀)
#define LED_USE_WS2812 1
#if LED_USE_WS2812
#include <Adafruit_NeoPixel.h>
#endif

// =====================[ 1. 핀 배치 ]===================================
// ⚠️ 임시 배정 — 회로도 확정 후 이 표만 고치면 됨
// ⚠️ RC 3개는 인터럽트 핀(2,3,18,19,20,21)만 가능. 회로팀에 전달됨
#define PIN_RC_THROTTLE  2    // RC 수신기 전후진 (Mega 5V — 직결 OK)
// 🚨 [2026-08-12] 조향·모드 핀을 **맞바꿨다** (3↔18). 실기 배선에 코드를 맞춘 것이다.
//    실측으로 확인한 상태: CH5(SWA) 가 핀 3 에, 조향(CH2)이 핀 11 에 꽂혀 있었다.
//    · 핀 3 은 인터럽트 핀이라 **모드를 여기로 옮기면 배선 그대로 동작**한다
//    · 🚨 핀 11 은 외부 인터럽트 핀이 아니다(2,3,18,19,20,21 만 가능) →
//      `attachInterrupt` 가 안 붙는다. **어떤 설정으로도 못 읽는다** → 그 선은 옮겨야 한다.
//      (PCINT 로는 가능하나 RC 캡처는 안전 경로라 검증 안 된 두 번째 체계를 붙이지 않는다)
//    ⇒ 결론: 코드 2줄 + 조향선 한 가닥(11→18) = 최소 작업.
#define PIN_RC_STEER    18    // RC 수신기 좌우   (구 3)
#define PIN_RC_MODE      3    // RC 수신기 모드 스위치 (구 18)

// 🚨 [2026-08-13 임시] 핀 5 를 놓는다 — **출력끼리 충돌**하고 있었다.
//    기판 실측: 핀 5 에 **RC 수신기 출력**이 물려 있는데, 펌웨어는 같은 핀을
//    ESC 출력으로 몰고 있었다(`esc[0].attach(5)` → 부팅부터 1500us 송출).
//    두 출력이 맞부딪히면 아두이노 핀이나 수신기 출력단이 상한다.
//    ⚠️ **이건 임시 회피다.** 기판의 ESC 는 핀 10(우)·13(좌) 에 있고 6·7·8 엔 아무것도 없다.
//       즉 지금 펌웨어의 ESC 출력은 **어디에도 연결돼 있지 않다**(모터가 안 도는 게 정상).
//       핀표 확정은 `docs/전달용/회로팀_최종본_회신.md` §8 의 답이 나온 뒤에 한 번에 한다.
//       그때 ESC 는 2채널(좌/우)로 줄고, 핀도 기판에 맞춘다.
#define PIN_ESC_FL       9    // (구 5) 선수 좌 ESC 신호 — 임시로 미사용 핀
#define PIN_ESC_FR       6    // 선수 우 ESC 신호
#define PIN_ESC_RL       7    // 선미 좌 ESC 신호
#define PIN_ESC_RR       8    // 선미 우 ESC 신호

#define PIN_LED_STRIP   40    // WS2812 스트립 Din (LED_USE_WS2812=1일 때 룰 표시등)
#define PIN_LED_GREEN   22    // (예비) 단색 룰 표시등: 수동
#define PIN_LED_YELLOW  24    // (예비) 단색 룰 표시등: 자율
#define PIN_LED_RED     26    // (예비) 단색 룰 표시등: 비상정지
#define PIN_LED_DEBUG   28    // 점검 LED (배ID 깜빡임/워치독 표시 — 스트립과 무관하게 항상 사용)

#define PIN_ID_A        34    // 배 ID: A배면 이 핀을 GND에 (DIP 스위치)
#define PIN_ID_B        36    // 배 ID: B배면 이 핀을 GND에
#define PIN_ESTOP_SENSE 38    // 비상정지 릴레이 상태 감지 (LOW=차단됨 가정, 벤치 확인)

// 방향지시등: 팀 결정으로 미채택 (2026-07-28). 부활 시 설계문서 §2-4 참고

// =====================[ 2. 노브 (튜닝 값) ]============================
// "⚠️벤치" = 벤치에서 확정 / "⚠️물" = 물 위에서 튜닝
const unsigned long ARM_HOLD_MS      = 2000;  // ESC arm 유지 시간 ⚠️벤치(비프음)
const unsigned long RC_TIMEOUT_MS    = 500;   // 수동: RC 무갱신→중립
const unsigned long CMD_TIMEOUT_MS   = 500;   // 자율: 브릿지 명령 무수신→중립 (워치독)
const int RC_PULSE_MIN   = 900;               // 노이즈/유효성 수용 하한 (µs)
const int RC_PULSE_MAX   = 2100;              // 상한
const int ESC_OUT_MIN    = 1100;              // T200 출구 클램프
const int ESC_OUT_MAX    = 1900;
const int RC_DEADBAND_US = 20;                // 스틱 중립 데드밴드 ⚠️벤치
const int STEER_SCALE_N  = 10;                // 조향 감도 = N/10 (10=100%) ⚠️물
const bool STEER_INVERT_RC = false;           // RC 조향 좌우 반전 ⚠️벤치
const int MODE_AUTO_THRESHOLD = 1500;         // 모드 펄스 < 1500 = AUTO (작년 관례 유지)
const long SERIAL_BAUD = 115200;              // 브릿지와 합의된 속도 (브릿지 기본값)
const int STRIP_NUM_PIXELS = 8;               // WS2812 픽셀 수 ⚠️실물 확정
const int STRIP_BRIGHTNESS = 255;             // 0~255. 주광 시인성 위해 최대로 시작

// =====================[ 3. 모터 4개 설정표 ]===========================
// 게인은 정수 연산: 출력편차 = 명령편차 × gain_num / 10
// invert: 배선/프로펠러 방향 따라 벤치에서 확정
struct MotorCfg {
  uint8_t pin;
  bool    invert;   // ⚠️벤치
  int     gain_num; // 10=100%. 선수 7=70% ⚠️물
};
MotorCfg motors[4] = {
  { PIN_ESC_FL, false, 7  },  // [0] 선수 좌
  { PIN_ESC_FR, false, 7  },  // [1] 선수 우
  { PIN_ESC_RL, false, 10 },  // [2] 선미 좌
  { PIN_ESC_RR, false, 10 },  // [3] 선미 우
};
Servo esc[4];

// =====================[ 4. RC 캡처 (인터럽트) ]========================
// 상승엣지=시작시각 기록, 하강엣지=HIGH 폭 계산 → LOW 구간 쓰레기 원천 차단
volatile unsigned long rcRise[3]  = {0, 0, 0};   // [0]스로틀 [1]조향 [2]모드
volatile unsigned long rcPulse[3] = {0, 0, 0};   // 마지막 유효 HIGH 폭 (µs)
volatile unsigned long rcStamp[3] = {0, 0, 0};   // 마지막 유효 수신 시각 (millis)

void rcIsr(int idx, int pin) {
  if (digitalRead(pin) == HIGH) {
    rcRise[idx] = micros();                      // 신호 시작
  } else {
    unsigned long w = micros() - rcRise[idx];    // HIGH 폭
    if (w >= (unsigned long)RC_PULSE_MIN && w <= (unsigned long)RC_PULSE_MAX) {
      rcPulse[idx] = w;                          // 유효한 것만 채택
      rcStamp[idx] = millis();
    }
  }
}
void isrThrottle() { rcIsr(0, PIN_RC_THROTTLE); }
void isrSteer()    { rcIsr(1, PIN_RC_STEER); }
void isrMode()     { rcIsr(2, PIN_RC_MODE); }

#if RC_SCAN
// 인터럽트 가능 핀 전부. 이 순서가 아래 출력 순서다.
const uint8_t SCAN_PINS[6] = {2, 3, 18, 19, 20, 21};
volatile unsigned long scRise[6] = {0}, scPulse[6] = {0}, scStamp[6] = {0};
void scIsr(uint8_t i) {
  uint8_t pin = SCAN_PINS[i];
  if (digitalRead(pin) == HIGH) {
    scRise[i] = micros();
  } else {
    unsigned long w = micros() - scRise[i];
    if (w >= (unsigned long)RC_PULSE_MIN && w <= (unsigned long)RC_PULSE_MAX) {
      scPulse[i] = w; scStamp[i] = millis();
    }
  }
}
void sc0(){scIsr(0);} void sc1(){scIsr(1);} void sc2(){scIsr(2);}
void sc3(){scIsr(3);} void sc4(){scIsr(4);} void sc5(){scIsr(5);}
#endif

#if PIN_HUNT
// 모든 디지털 핀을 훑어 RC 펄스(900~2100us)가 살아있는 핀을 찾는다.
void pinHuntSweep() {
  Serial.print(F("HUNT "));
  bool found = false;
  for (uint8_t p = 2; p <= 53; p++) {
    if (p >= 5 && p <= 8) continue;            // ESC 출력 — 건드리면 신호가 끊긴다
    if (p == PIN_LED_STRIP) continue;          // 스트립 Din
    pinMode(p, INPUT);
    unsigned long w = pulseIn(p, HIGH, 30000UL);   // RC 프레임 20ms → 30ms 면 한 발은 잡힌다
    if (w >= (unsigned long)RC_PULSE_MIN && w <= (unsigned long)RC_PULSE_MAX) {
      Serial.print('p'); Serial.print(p); Serial.print('=');
      Serial.print(w); Serial.print(F("us  "));
      found = true;
    }
  }
  if (!found) Serial.print(F("(어느 핀에도 유효 펄스 없음)"));
  Serial.println();
}
#endif

// Mega(8비트)는 4바이트 변수 읽기가 여러 명령으로 쪼개짐 —
// 읽는 도중 ISR이 값을 바꾸면 반쪽짜리 값이 됨. 잠깐 인터럽트 끄고 스냅샷.
unsigned long snapPulse(int idx) {
  noInterrupts(); unsigned long v = rcPulse[idx]; interrupts(); return v;
}
unsigned long snapStamp(int idx) {
  noInterrupts(); unsigned long v = rcStamp[idx]; interrupts(); return v;
}

bool rcFresh(int idx) {                          // 최근 RC_TIMEOUT_MS 내 갱신됐나
  unsigned long st = snapStamp(idx);
  return st != 0 && (millis() - st) < RC_TIMEOUT_MS;
}

// =====================[ 5. 배 ID (상보 2핀) ]==========================
// A배: PIN_ID_A만 GND / B배: PIN_ID_B만 GND
enum BoatId { BOAT_A = 0, BOAT_B = 1, BOAT_FAULT = 2 };
BoatId boatId = BOAT_FAULT;
bool boatIdDefaulted = false;   // 스위치가 없어 기본값을 쓴 것인가

// 🚨 [2026-08-12] ID 스위치 미배선 시의 기본값.
//    이전엔 '둘 다 open' 도 FAULT 로 봐서 **모터가 영구 중립**이었다.
//    지금 배에는 ID 스위치를 달 수 없다(팀 결정) → 스위치가 없으면 이 값으로 돈다.
//    ⚠️ 두 척을 동시에 운용하게 되면 **스위치를 달거나 B배는 이 줄을 BOAT_B 로** 바꿔야 한다.
//       안 그러면 두 배가 같은 설정으로 돈다 — 작년 `It_is_Aship` 오타로 B배 분기가
//       죽어 있던 것과 **결과가 같은** 사고다.
#define DEFAULT_BOAT_ID  BOAT_A

// 반환형 int: Arduino IDE가 함수 목록을 enum 정의보다 앞에 자동 생성하는 함정 회피
int readBoatId() {
  bool aLow = (digitalRead(PIN_ID_A) == LOW);   // INPUT_PULLUP → GND 에 묶인 쪽만 LOW
  bool bLow = (digitalRead(PIN_ID_B) == LOW);

  // 🚨 '둘 다 GND' 는 여전히 FAULT 다 — 이건 **배선 실수**이지 미배선이 아니다.
  //    미배선(둘 다 open)과 구분해서 남긴다. 실수를 조용히 넘기면 안 된다.
  if (aLow && bLow) return BOAT_FAULT;

  if (aLow) return BOAT_A;
  if (bLow) return BOAT_B;

  // 둘 다 open = 스위치를 안 달았다 → 기본값
  boatIdDefaulted = true;
  return DEFAULT_BOAT_ID;
}

// =====================[ 6. 상태 변수 ]=================================
enum Mode { MODE_WAIT = 0, MODE_MANUAL = 1, MODE_AUTO = 2 };
Mode mode = MODE_WAIT;              // 유효 모드 신호 받기 전 = 대기(중립)

int  autoCmdL = 1500, autoCmdR = 1500;   // 브릿지에서 온 마지막 유효 명령 (µs)
unsigned long autoCmdStamp = 0;          // 그 수신 시각 (millis). 0 = 아직 없음
bool watchdogActive = false;
bool rcLostAlert    = false;   // 수동 모드인데 RC 가 끊겨 중립인 상태 (점검 LED 경보용)
int  finalOut[4] = {1500, 1500, 1500, 1500};  // 최종 ESC 출력 (상태보고용)

// =====================[ 7. 브릿지 명령 수신 (시리얼 파서) ]=============
// 브릿지 계약: 한 줄 = "L<좌µs>,R<우µs>\n"  (예: "L1400,R1600")
// 형식이 조금이라도 어긋나거나 범위 밖이면 그 줄 통째로 폐기 —
// 깨진 명령은 워치독을 먹이지 못한다 (통신 이상 = 침묵 취급)
char rxBuf[24];
uint8_t rxLen = 0;

void parseCommandLine(const char *s) {
  if (s[0] != 'L') return;
  const char *comma = strchr(s, ',');
  if (comma == NULL || comma[1] != 'R') return;
  long l = atol(s + 1);
  long r = atol(comma + 2);
  if (l < RC_PULSE_MIN || l > RC_PULSE_MAX) return;
  if (r < RC_PULSE_MIN || r > RC_PULSE_MAX) return;
  autoCmdL = (int)l;
  autoCmdR = (int)r;
  autoCmdStamp = millis();
}

void pollCommandSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) { rxBuf[rxLen] = '\0'; parseCommandLine(rxBuf); rxLen = 0; }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;   // 버퍼 초과 = 쓰레기 줄 → 폐기하고 다음 줄부터
    }
  }
}

// =====================[ 8. 믹싱 / 분배 / 클램프 ]======================
int applyDeadband(int dev) {
  if (dev > -RC_DEADBAND_US && dev < RC_DEADBAND_US) return 0;
  return dev;
}

// RC 스틱 2개 → 좌/우 명령 (아케이드 믹싱)
void mixManual(int &cmdL, int &cmdR) {
  int thr = applyDeadband((int)snapPulse(0) - 1500);
  int str = applyDeadband((int)snapPulse(1) - 1500) * STEER_SCALE_N / 10;
  if (STEER_INVERT_RC) str = -str;
  cmdL = 1500 + thr + str;
  cmdR = 1500 + thr - str;   // 포화는 출구 클램프가 처리 (직진 최고속 보존)
}

// 좌/우 명령 → ESC 4개. 모든 출력이 반드시 이 관문 하나를 지남
void driveMotors(int cmdL, int cmdR) {
  int cmd[4] = { cmdL, cmdR, cmdL, cmdR };   // [선수L, 선수R, 선미L, 선미R]
  for (int i = 0; i < 4; i++) {
    int dev = (cmd[i] - 1500) * motors[i].gain_num / 10;  // 편차에 게인 (정수 연산)
    if (motors[i].invert) dev = -dev;
    int out = constrain(1500 + dev, ESC_OUT_MIN, ESC_OUT_MAX);  // 유일한 출구 클램프
    esc[i].writeMicroseconds(out);
    finalOut[i] = out;                       // 상태보고용 기록
  }
}

void driveNeutral() { driveMotors(1500, 1500); }

// =====================[ 9. LED ]=======================================
#if LED_USE_WS2812
Adafruit_NeoPixel strip(STRIP_NUM_PIXELS, PIN_LED_STRIP, NEO_GRB + NEO_KHZ800);
uint32_t lastStripColor = 0xFFFFFFFF;   // 마지막으로 보낸 색 (불필요한 show() 방지)

// 스트립 전체를 한 색으로. 색이 바뀔 때만 show() 호출 —
// show()는 전송 중 인터럽트를 잠깐 꺼서 RC 펄스 측정을 흔들 수 있음.
// 상태 색은 어쩌다 한 번 바뀌므로 실질 간섭 없음 (매 주기 호출 금지의 이유)
void setRuleColor(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t c = strip.Color(r, g, b);
  if (c == lastStripColor) return;
  lastStripColor = c;
  strip.fill(c);
  strip.show();
}
#endif

// 부팅 시 배 ID 표시 (점검 LED)
//   긴 점등 1초 → 짧게 N회 = ID 핀이 아니라 **기본값**으로 정해진 것 (미배선)
//   짧게 1회 = A(핀 확정) / 짧게 2회 = B(핀 확정) / 빠른 8회 = 고장(배선 실수)
// ★ 물 위에선 시리얼 로그를 못 본다. 두 척을 돌릴 때 "왜 둘 다 A 지"를
//   눈으로 구분할 수 있어야 해서 기본값 여부를 앞머리 긴 점등으로 표시한다.
void blinkBoatId() {
  if (boatIdDefaulted) {                                   // 기본값 경고 머리표
    digitalWrite(PIN_LED_DEBUG, HIGH); delay(1000);
    digitalWrite(PIN_LED_DEBUG, LOW);  delay(300);
  }
  int n = (boatId == BOAT_A) ? 1 : (boatId == BOAT_B) ? 2 : 8;
  int ms = (boatId == BOAT_FAULT) ? 120 : 400;
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_LED_DEBUG, HIGH); delay(ms);
    digitalWrite(PIN_LED_DEBUG, LOW);  delay(ms);
  }
}

void updateLeds(bool estopActive) {
  // 룰 표시등 — 규정 의미 전용 (초록=수동, 노랑=자율, 빨강=비상정지)
  // 빨강(비상정지)이 모드 색보다 우선 — 규정 의미상 최상위 상태
#if LED_USE_WS2812
  if (estopActive)                 setRuleColor(255, 0, 0);     // 빨강
  else if (mode == MODE_MANUAL)    setRuleColor(0, 255, 0);     // 초록
  else if (mode == MODE_AUTO)      setRuleColor(255, 170, 0);   // 노랑
  else                             setRuleColor(0, 0, 0);       // 대기 = 소등
#else
  digitalWrite(PIN_LED_GREEN,  mode == MODE_MANUAL ? HIGH : LOW);
  digitalWrite(PIN_LED_YELLOW, mode == MODE_AUTO   ? HIGH : LOW);
  digitalWrite(PIN_LED_RED,    estopActive         ? HIGH : LOW);
#endif

  // 점검 LED — 아래 중 하나면 점멸, 정상이면 소등
  //   · 자율 워치독 발동 (노트북/브릿지 침묵)
  //   · 수동인데 RC 끊김 (조종기 꺼짐·전파 두절) ← 물가에서 "왜 안 움직이지"의 답
  //   · 배 ID 고장 (배선 실수)
  bool alert = watchdogActive || rcLostAlert || (boatId == BOAT_FAULT);
  digitalWrite(PIN_LED_DEBUG, (alert && (millis() / 200) % 2) ? HIGH : LOW);
}

// =====================[ 10. 상태 보고 (10Hz, 같은 USB) ]===============
// 한 줄 = "S,모드,워치독,배ID,비상정지,FL,FR,RL,RR"
// 벤치: IDE 시리얼 모니터에서 그대로 보임. 브릿지 가동 중엔 무시됨(브릿지는 안 읽음).
void publishStatus(bool estopActive) {
  Serial.print(F("S,"));
  Serial.print((int)mode);              Serial.print(',');
  Serial.print(watchdogActive ? 1 : 0); Serial.print(',');
  Serial.print((int)boatId);            Serial.print(',');
  Serial.print(estopActive ? 1 : 0);
  for (int i = 0; i < 4; i++) { Serial.print(','); Serial.print(finalOut[i]); }
  Serial.println();
}

// =====================[ 11. setup / loop ]=============================
void setup() {
  Serial.begin(SERIAL_BAUD);     // USB 1개: 브릿지 명령 수신 + 상태/디버그 송신

  // 핀 모드
  pinMode(PIN_RC_THROTTLE, INPUT);
  pinMode(PIN_RC_STEER,    INPUT);
  pinMode(PIN_RC_MODE,     INPUT);
  pinMode(PIN_ID_A, INPUT_PULLUP);      // GND에 묶인 쪽만 LOW로 읽힘
  pinMode(PIN_ID_B, INPUT_PULLUP);
  pinMode(PIN_ESTOP_SENSE, INPUT_PULLUP);
  pinMode(PIN_LED_DEBUG,  OUTPUT);
#if LED_USE_WS2812
  strip.begin();
  strip.setBrightness(STRIP_BRIGHTNESS);
  strip.show();                      // 전체 소등으로 시작
#else
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED,    OUTPUT);
#endif

  // 배 ID 판독 + 표시 (고장이면 이후 루프에서 영구 중립)
  boatId = (BoatId)readBoatId();
  blinkBoatId();
  Serial.print(F("Boat ID: "));
  Serial.print(boatId == BOAT_A ? F("A") : boatId == BOAT_B ? F("B") : F("FAULT"));
  // 스위치로 정해진 값인지, 미배선이라 기본값을 쓴 것인지 구분해서 찍는다.
  // 나중에 두 척을 돌릴 때 "왜 둘 다 A 지" 를 여기서 바로 알 수 있어야 한다.
  Serial.println(boatIdDefaulted ? F("  (ID pins open -> default)") : F("  (from ID pins)"));

  // ESC arm: 1500 출력 유지, 이 동안 모든 명령 무시 (설계 §2-5)
  for (int i = 0; i < 4; i++) { esc[i].attach(motors[i].pin); esc[i].writeMicroseconds(1500); }
  delay(ARM_HOLD_MS);
  Serial.println(F("ESC armed."));

  // RC 인터럽트 연결 (arm 후 — arm 중 명령 무시를 구조로 보장)
  attachInterrupt(digitalPinToInterrupt(PIN_RC_THROTTLE), isrThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_STEER),    isrSteer,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_MODE),     isrMode,     CHANGE);

#if RC_SCAN
  // 🚨 위 3개를 **덮어쓴다** — 스캔 빌드에서는 정상 RC 경로가 죽는다(의도된 것).
  {
    void (*fns[6])() = {sc0, sc1, sc2, sc3, sc4, sc5};
    for (uint8_t i = 0; i < 6; i++) {
      pinMode(SCAN_PINS[i], INPUT);
      attachInterrupt(digitalPinToInterrupt(SCAN_PINS[i]), fns[i], CHANGE);
    }
    Serial.println(F("RC SCAN mode: pins 2,3,18,19,20,21 (normal RC disabled)"));
  }
#endif

  // arm 대기 중 시리얼 버퍼에 쌓인 명령은 전부 버림 (부팅 직후 과거 명령 실행 방지)
  while (Serial.available() > 0) Serial.read();

  Serial.println(F("Setup done. Control loop start."));
}

unsigned long lastControlMs = 0;
unsigned long lastStatusMs  = 0;

void loop() {
  unsigned long now = millis();
  pollCommandSerial();   // 브릿지 명령 수신 (비블로킹 — 브릿지/노트북 없어도 그냥 지나감)

  // ---- 제어루프: 50ms(20Hz) ----
  if (now - lastControlMs >= 50) {
    lastControlMs = now;

    bool estopActive = (digitalRead(PIN_ESTOP_SENSE) == LOW);

    // 배 ID 고장 = 무조건 중립 (설계 §2-2: 시끄럽게 멈춘다)
    if (boatId == BOAT_FAULT) {
      driveNeutral();
      rcLostAlert = false;
    } else {
      // 모드 판정: 유효한 모드 펄스가 온 적 있어야 대기 해제
      if (snapStamp(2) != 0) {
        mode = (snapPulse(2) < (unsigned long)MODE_AUTO_THRESHOLD) ? MODE_AUTO : MODE_MANUAL;
      }
      // (모드 채널이 이후 끊겨도 마지막 모드 유지 — 팀 결정: RC 두절로 모드 전환 안 함)

      int cmdL = 1500, cmdR = 1500;
      watchdogActive = false;
      rcLostAlert    = false;

      if (mode == MODE_MANUAL) {
        // 수동: 스로틀·조향 둘 다 신선해야 구동. 아니면 중립 (RC failsafe)
        if (rcFresh(0) && rcFresh(1)) mixManual(cmdL, cmdR);
        else                          rcLostAlert = true;   // 점검 LED 로 알린다
      } else if (mode == MODE_AUTO) {
        // 자율: 워치독 — 유효 명령이 500ms 내에 있어야 구동
        if (autoCmdStamp != 0 && (now - autoCmdStamp) < CMD_TIMEOUT_MS) {
          cmdL = autoCmdL;
          cmdR = autoCmdR;   // 패스스루 — 리매핑 없음
        } else {
          watchdogActive = true;   // 브릿지/노트북 침묵 → 중립 + LED 경보
        }
      }
      // MODE_WAIT면 그대로 중립

      driveMotors(cmdL, cmdR);
    }

    // 🚨 표시등은 분기 밖에서 항상 갱신한다.
    //    예전엔 else 안에만 있어서 **배 ID 고장이면 룰 표시등이 영영 꺼진 채**였다
    //    (비상정지를 눌러도 빨간등이 안 켜짐 = 규정 위반 + 조용한 고장).
    updateLeds(estopActive);
  }

  // ---- 상태 보고: 100ms(10Hz) ----
  if (now - lastStatusMs >= 100) {
    lastStatusMs = now;
    publishStatus(digitalRead(PIN_ESTOP_SENSE) == LOW);
  }

#if PIN_HUNT
  // ---- 전 디지털 핀 사냥: 3초마다 한 바퀴 ----
  static unsigned long lastHunt = 0;
  if (now - lastHunt >= 3000) { lastHunt = now; pinHuntSweep(); }
#endif

#if RC_SCAN
  // ---- 핀 전수 스캔: 1초마다 6개 핀 상태 ----
  static unsigned long lastScan = 0;
  if (now - lastScan >= 1000) {
    lastScan = now;
    Serial.print(F("SCAN "));
    for (uint8_t i = 0; i < 6; i++) {
      noInterrupts();
      unsigned long pw = scPulse[i], stp = scStamp[i];
      interrupts();
      Serial.print(F("p")); Serial.print(SCAN_PINS[i]); Serial.print('=');
      if (stp == 0) Serial.print(F("-"));
      else { Serial.print(pw); Serial.print(F("us")); }
      Serial.print(i < 5 ? F("  ") : F("\n"));
    }
  }
#endif

#if RC_DEBUG
  // ---- RC 배선 진단: 1초마다 세 채널의 펄스 폭 ----
  static unsigned long lastRcDbg = 0;
  if (now - lastRcDbg >= 1000) {
    lastRcDbg = now;
    Serial.print(F("RC "));
    const char *nm[3] = {"thr(p2)", "str(p18)", "mode(p3)"};
    for (int i = 0; i < 3; i++) {
      Serial.print(nm[i]); Serial.print('=');
      unsigned long st = snapStamp(i);
      if (st == 0) {
        Serial.print(F("없음"));            // 펄스를 한 번도 못 받음 = 배선/전원/바인딩
      } else {
        Serial.print(snapPulse(i));
        Serial.print(F("us(")); Serial.print(now - st); Serial.print(F("ms전)"));
      }
      Serial.print(i < 2 ? F("  ") : F("\n"));
    }
  }
#endif
}

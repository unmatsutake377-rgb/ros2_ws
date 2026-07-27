// =====================================================================
// SSF 자율운항선박 펌웨어 — Arduino Due, 시리얼 브릿지 방식
// 설계 근거: docs/전달용/펌웨어_설계문서.md (단일 출처, 여기 주석은 요약)
//
// [2026-07-25 아키텍처 변경] micro-ROS 제거 → 회로팀 브릿지 노드 수용.
//   노트북 motor_control ──(ROS Motor_run)──▶ 브릿지 노드 ──USB 시리얼──▶ 이 펌웨어
//   브릿지가 보내는 명령: "L1500,R1500\n" (µs 단위, 1500=중립, <1500=전진)
//   이 펌웨어의 상태 보고: Native 포트로 "S,모드,워치독,배ID,비상정지,FL,FR,RL,RR\n" 10Hz
//
// 포트 역할 (Due):
//   Programming 포트(Serial)   = 업로드 + 브릿지 명령 수신
//   Native 포트(SerialUSB)     = 상태 보고 + 디버그 출력
//
// 대회 3모드 → 펌웨어 2상태:
//   AUTO   = 브릿지 명령 (자율 본경기·자율 토너먼트)
//   MANUAL = RC 조종 (수동 토너먼트 — 노트북 없이 단독 가동)
//
// (Mega로 갈 경우 이식 포인트: SerialUSB→예비 UART+USB어댑터, 핀 재배정,
//  RC 레벨시프트 불필요(5V 보드). 로직은 전부 그대로.)
// =====================================================================

#include <Servo.h>

// =====================[ 1. 핀 배치 ]===================================
// ⚠️ 전부 임시 배정 — 회로도 확정 후 이 표만 고치면 됨
#define PIN_RC_THROTTLE  2    // RC 수신기 전후진 (⚠️Due 사용 시 5V→3.3V 분배 필수)
#define PIN_RC_STEER     3    // RC 수신기 좌우   (⚠️분배 필수)
#define PIN_RC_MODE      4    // RC 수신기 모드 스위치 (⚠️분배 필수)

#define PIN_ESC_FL       5    // 선수 좌 ESC 신호
#define PIN_ESC_FR       6    // 선수 우 ESC 신호
#define PIN_ESC_RL       7    // 선미 좌 ESC 신호
#define PIN_ESC_RR       8    // 선미 우 ESC 신호

#define PIN_LED_GREEN   22    // 룰 표시등: 수동
#define PIN_LED_YELLOW  24    // 룰 표시등: 자율
#define PIN_LED_RED     26    // 룰 표시등: 비상정지
#define PIN_LED_DEBUG   28    // 점검 LED (배ID 깜빡임/워치독 표시)
#define PIN_LED_TURN_L  30    // 방향지시등 좌 (철회 가능 — 아래 스위치로 끔)
#define PIN_LED_TURN_R  32    // 방향지시등 우

#define PIN_ID_A        34    // 배 ID: A배면 이 핀을 GND에 (DIP 스위치)
#define PIN_ID_B        36    // 배 ID: B배면 이 핀을 GND에
#define PIN_ESTOP_SENSE 38    // 비상정지 릴레이 상태 감지 (LOW=차단됨 가정, 벤치 확인)

#define ENABLE_TURN_SIGNALS 1 // 방향지시등: 회로판 협의로 취소되면 0으로

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
const int TURN_SIGNAL_DIFF_US = 60;           // 좌우 차이 이만큼 크면 방향지시등
const long CMD_BAUD    = 115200;              // 브릿지와 합의된 속도 (브릿지 기본값)
const long STATUS_BAUD = 115200;              // 상태/디버그 포트

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
// Due는 32비트 원자 읽기라 메인 루프에서 그냥 읽어도 안전
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

bool rcFresh(int idx) {                          // 최근 RC_TIMEOUT_MS 내 갱신됐나
  return rcStamp[idx] != 0 && (millis() - rcStamp[idx]) < RC_TIMEOUT_MS;
}

// =====================[ 5. 배 ID (상보 2핀) ]==========================
// A배: PIN_ID_A만 GND / B배: PIN_ID_B만 GND / 그 외 = 고장 → 전 모터 중립 고정
enum BoatId { BOAT_A = 0, BOAT_B = 1, BOAT_FAULT = 2 };
BoatId boatId = BOAT_FAULT;

// 반환형 int: Arduino IDE가 함수 목록을 enum 정의보다 앞에 자동 생성하는 함정 회피
int readBoatId() {
  bool aLow = (digitalRead(PIN_ID_A) == LOW);
  bool bLow = (digitalRead(PIN_ID_B) == LOW);
  if (aLow && !bLow) return BOAT_A;
  if (!aLow && bLow) return BOAT_B;
  return BOAT_FAULT;   // 둘 다 open(선 빠짐) 또는 둘 다 GND(배선 실수)
}

// =====================[ 6. 상태 변수 ]=================================
enum Mode { MODE_WAIT = 0, MODE_MANUAL = 1, MODE_AUTO = 2 };
Mode mode = MODE_WAIT;              // 유효 모드 신호 받기 전 = 대기(중립)

int  autoCmdL = 1500, autoCmdR = 1500;   // 브릿지에서 온 마지막 유효 명령 (µs)
unsigned long autoCmdStamp = 0;          // 그 수신 시각 (millis). 0 = 아직 없음
bool watchdogActive = false;
int  finalOut[4] = {1500, 1500, 1500, 1500};  // 최종 ESC 출력 (상태보고용)

// =====================[ 7. 브릿지 명령 수신 (시리얼 파서) ]=============
// 브릿지 계약: 한 줄 = "L<좌µs>,R<우µs>\n"  (예: "L1400,R1600")
// 형식이 조금이라도 어긋나거나 범위 밖이면 그 줄 통째로 폐기 —
// 깨진 명령은 워치독을 먹이지 못한다 (통신 이상 = 침묵 취급, micro-ROS 때와 같은 원칙)
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
  int thr = applyDeadband((int)rcPulse[0] - 1500);
  int str = applyDeadband((int)rcPulse[1] - 1500) * STEER_SCALE_N / 10;
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
// 부팅 시 배 ID 표시: 1회=A, 2회=B, 빠른 연속 점멸=고장 (점검 LED)
void blinkBoatId() {
  int n = (boatId == BOAT_A) ? 1 : (boatId == BOAT_B) ? 2 : 8;
  int ms = (boatId == BOAT_FAULT) ? 120 : 400;
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_LED_DEBUG, HIGH); delay(ms);
    digitalWrite(PIN_LED_DEBUG, LOW);  delay(ms);
  }
}

void updateLeds(bool estopActive, int cmdL, int cmdR) {
  // 룰 표시등 — 규정 의미 전용 (초록=수동, 노랑=자율, 빨강=비상정지)
  digitalWrite(PIN_LED_GREEN,  mode == MODE_MANUAL ? HIGH : LOW);
  digitalWrite(PIN_LED_YELLOW, mode == MODE_AUTO   ? HIGH : LOW);
  digitalWrite(PIN_LED_RED,    estopActive         ? HIGH : LOW);

  // 점검 LED — 워치독 발동/배ID 고장이면 점멸, 정상이면 소등
  bool alert = watchdogActive || (boatId == BOAT_FAULT);
  digitalWrite(PIN_LED_DEBUG, (alert && (millis() / 200) % 2) ? HIGH : LOW);

#if ENABLE_TURN_SIGNALS
  // 방향지시등 — 좌우 명령 차이가 크면 도는 중 (0.4초 주기 점멸)
  bool blink = (millis() / 400) % 2;
  int diff = cmdL - cmdR;
  digitalWrite(PIN_LED_TURN_L, (diff < -TURN_SIGNAL_DIFF_US && blink) ? HIGH : LOW);
  digitalWrite(PIN_LED_TURN_R, (diff >  TURN_SIGNAL_DIFF_US && blink) ? HIGH : LOW);
#endif
}

// =====================[ 10. 상태 보고 (10Hz, Native 포트) ]============
// 한 줄 = "S,모드,워치독,배ID,비상정지,FL,FR,RL,RR"
// 나중에 노트북 쪽 수신 노드(신규 작성 예정)가 이 줄을 읽어 ROS 토픽으로 발행.
// 디버그 문장도 같은 포트로 나감 — 수신 노드는 "S," 로 시작하는 줄만 취하면 됨.
void publishStatus(bool estopActive) {
  SerialUSB.print("S,");
  SerialUSB.print((int)mode);          SerialUSB.print(',');
  SerialUSB.print(watchdogActive ? 1 : 0); SerialUSB.print(',');
  SerialUSB.print((int)boatId);        SerialUSB.print(',');
  SerialUSB.print(estopActive ? 1 : 0);
  for (int i = 0; i < 4; i++) { SerialUSB.print(','); SerialUSB.print(finalOut[i]); }
  SerialUSB.println();
}

// =====================[ 11. setup / loop ]=============================
void setup() {
  Serial.begin(CMD_BAUD);        // Programming 포트 = 브릿지 명령 수신
  SerialUSB.begin(STATUS_BAUD);  // Native 포트 = 상태 보고 + 디버그

  // 핀 모드
  pinMode(PIN_RC_THROTTLE, INPUT);
  pinMode(PIN_RC_STEER,    INPUT);
  pinMode(PIN_RC_MODE,     INPUT);
  pinMode(PIN_ID_A, INPUT_PULLUP);      // GND에 묶인 쪽만 LOW로 읽힘
  pinMode(PIN_ID_B, INPUT_PULLUP);
  pinMode(PIN_ESTOP_SENSE, INPUT_PULLUP);
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED,    OUTPUT);
  pinMode(PIN_LED_DEBUG,  OUTPUT);
#if ENABLE_TURN_SIGNALS
  pinMode(PIN_LED_TURN_L, OUTPUT);
  pinMode(PIN_LED_TURN_R, OUTPUT);
#endif

  // 배 ID 판독 + 표시 (고장이면 이후 루프에서 영구 중립)
  boatId = (BoatId)readBoatId();
  blinkBoatId();
  SerialUSB.print("Boat ID: ");
  SerialUSB.println(boatId == BOAT_A ? "A" : boatId == BOAT_B ? "B" : "FAULT");

  // ESC arm: 1500 출력 유지, 이 동안 모든 명령 무시 (설계 §2-5)
  for (int i = 0; i < 4; i++) { esc[i].attach(motors[i].pin); esc[i].writeMicroseconds(1500); }
  delay(ARM_HOLD_MS);
  SerialUSB.println("ESC armed.");

  // RC 인터럽트 연결 (arm 후 — arm 중 명령 무시를 구조로 보장)
  attachInterrupt(digitalPinToInterrupt(PIN_RC_THROTTLE), isrThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_STEER),    isrSteer,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_MODE),     isrMode,     CHANGE);

  // arm 대기 중 시리얼 버퍼에 쌓인 명령은 전부 버림 (부팅 직후 과거 명령 실행 방지)
  while (Serial.available() > 0) Serial.read();

  SerialUSB.println("Setup done. Control loop start.");
}

unsigned long lastControlMs = 0;
unsigned long lastStatusMs  = 0;

void loop() {
  unsigned long now = millis();
  pollCommandSerial();   // 브릿지 명령 수신 (비블로킹 — 브릿지/노트북 없어도 그냥 지나감)

  // ---- 제어루프: 50ms(20Hz) ----
  if (now - lastControlMs >= 50) {
    lastControlMs = now;

    // 배 ID 고장 = 무조건 중립 (설계 §2-2: 시끄럽게 멈춘다)
    if (boatId == BOAT_FAULT) {
      driveNeutral();
    } else {
      // 모드 판정: 유효한 모드 펄스가 온 적 있어야 대기 해제
      if (rcStamp[2] != 0) {
        mode = (rcPulse[2] < (unsigned long)MODE_AUTO_THRESHOLD) ? MODE_AUTO : MODE_MANUAL;
      }
      // (모드 채널이 이후 끊겨도 마지막 모드 유지 — 팀 결정: RC 두절로 모드 전환 안 함)

      int cmdL = 1500, cmdR = 1500;
      watchdogActive = false;

      if (mode == MODE_MANUAL) {
        // 수동: 스로틀·조향 둘 다 신선해야 구동. 아니면 중립 (RC failsafe)
        if (rcFresh(0) && rcFresh(1)) mixManual(cmdL, cmdR);
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
      updateLeds(digitalRead(PIN_ESTOP_SENSE) == LOW, cmdL, cmdR);
    }
  }

  // ---- 상태 보고: 100ms(10Hz) ----
  if (now - lastStatusMs >= 100) {
    lastStatusMs = now;
    publishStatus(digitalRead(PIN_ESTOP_SENSE) == LOW);
  }
}

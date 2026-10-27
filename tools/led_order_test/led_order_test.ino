// WS2812 색 순서(GRB / RGB) 확인 스케치 — 핀 16 스트립에 빨강·초록·파랑을 3초씩 번갈아 보낸다.
//
// 왜 필요한가 (2026-10-27):
//   ssf_boat.ino 는 NEO_GRB 로 보낸다. 09-03 에 "파랑이 나왔으니 색 순서 일치" 로 확인했지만
//   파랑은 GRB·RGB 둘 다 세 번째 바이트라 **이걸로는 구별이 안 된다.** 빨강·초록이 뒤바뀌어 있으면
//   수동 모드(초록)가 빨강으로 보이고, 비상정지(빨강)가 초록으로 보인다.
//
// 판정: 시리얼 모니터(115200)에 "지금 빨강" 이 찍힐 때 스트립이
//   빨강 → NEO_GRB 가 맞다. 펌웨어 그대로.
//   초록 → 스트립이 RGB 순서다. ssf_boat.ino 611행 NEO_GRB → NEO_RGB 로 바꿔야 한다.
//
// 🚨 ESC·RC 핀은 건드리지 않는다. USB 만 꽂아도 된다(배터리 불필요).
// 🚨 시험이 끝나면 반드시 ssf_boat.ino 를 다시 올릴 것.

#include <Adafruit_NeoPixel.h>

#define PIN_LED_STRIP 16       // ssf_boat.ino 와 같은 핀 (09-03 실물 확정)
const int NUM_PIXELS = 34;     // 실물 34개
const int BRIGHTNESS = 40;     // ssf_boat.ino 와 같은 밝기

Adafruit_NeoPixel strip(NUM_PIXELS, PIN_LED_STRIP, NEO_GRB + NEO_KHZ800);   // 펌웨어와 같은 설정으로 보낸다

struct Step { const char *name; uint8_t r, g, b; };
const Step STEPS[] = {
  { "빨강 (255,0,0)  — 수동이 아니라 비상정지 색", 255, 0, 0 },
  { "초록 (0,255,0)  — 수동 모드 색",             0, 255, 0 },
  { "파랑 (0,0,255)  — 대조군",                   0, 0, 255 },
};

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();
  Serial.println(F("=== WS2812 색 순서 확인 — 3초마다 바뀜 ==="));
  Serial.println(F("'지금 빨강' 일 때 스트립이 빨강이면 GRB 정상, 초록이면 RGB(펌웨어 수정 필요)"));
}

void loop() {
  for (const Step &s : STEPS) {
    for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(s.r, s.g, s.b));
    strip.show();
    Serial.print(F("지금 ")); Serial.println(s.name);
    delay(3000);
  }
}

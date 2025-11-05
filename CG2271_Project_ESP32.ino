// ESP32-S2-MINI-1U + KY-018 (LDR) + SYNC to MCXC444
// Board: ESP32S2 Dev Module (Arduino core for ESP32)

#include <Arduino.h>

constexpr int PIN_LDR  = 1;  // ADC1 on ESP32-S2 (GPIO1)
constexpr int PIN_SYNC = 5;  // Digital output to MCXC444 PTC3

// Simple averaging to reduce noise
int readADC() {
  long acc = 0;
  const int N = 8;
  for (int i = 0; i < N; ++i) {
    acc += analogRead(PIN_LDR);
    delayMicroseconds(200);
  }
  return acc / N; // 0..4095 at 12-bit
}

// TUNE THIS if needed (see serial output)
int RAW_THRESHOLD = 300;  // lower reading = darker (typical KY-018 divider)

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_SYNC, OUTPUT);
  digitalWrite(PIN_SYNC, LOW); // start low

  // ESP32-S2 ADC config
  analogReadResolution(12);                    // 0..4095
  analogSetPinAttenuation(PIN_LDR, ADC_11db);  // ~0..3.3V range

  Serial.println("\n[ESP32-S2 + KY-018] Starting...");
  Serial.println("Expect changing ADC values when you cover/uncover the LDR.");
  Serial.println("Columns: raw, sync(0/1)");
}

void loop() {
  int raw = readADC();

  // Darker -> smaller raw; keep original logic shape (minimal change)
  bool syncHigh = (raw > RAW_THRESHOLD);
  digitalWrite(PIN_SYNC, syncHigh ? HIGH : LOW);

  Serial.print(raw);
  Serial.print(", ");
  Serial.println(syncHigh ? 1 : 0);

  delay(200);
}

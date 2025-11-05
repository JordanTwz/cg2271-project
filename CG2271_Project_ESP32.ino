// ESP32-S2-MINI-1U + KY-018 (LDR) + SYNC to MCXC444
// Board: ESP32S2 Dev Module (Arduino core for ESP32)

#include <Arduino.h>

constexpr int PIN_LDR  = 1;  // ADC1 on ESP32-S2 (GPIO1)
constexpr int PIN_TX   = 5;  // UART TX to MCXC444 (connect to PTD2/RX)

int readADC() {
  long acc = 0;
  const int N = 8;
  for (int i = 0; i < N; ++i) {
    acc += analogRead(PIN_LDR);
    delayMicroseconds(200);
  }
  return acc / N;
}

int RAW_THRESHOLD = 300;

void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_LDR, ADC_11db);

  Serial1.begin(9600, SERIAL_8N1, -1, PIN_TX);

  Serial.println("\n[ESP32-S2 + KY-018] Starting...");
  Serial.println("Columns: raw, msg");
}

void loop() {
  int raw = readADC();
  bool dark = (raw < RAW_THRESHOLD);

  if (dark) {
    Serial1.write("1\n");
  } else {
    Serial1.write("0\n");
  }

  Serial.print(raw);
  Serial.print(", ");
  Serial.println(dark ? 1 : 0);

  delay(200);
}

// ESP32-S2-MINI-1U + KY-018 (LDR) + SYNC to MCXC444
// Board: ESP32S2 Dev Module (Arduino core for ESP32)

#include <Arduino.h>

constexpr int PIN_LDR  = 1;
constexpr int UART_TX  = 3;   // TX to MCXC444 RX (PTD2)
constexpr int UART_RX  = 18;  // RX from MCXC444 TX (PTD3)

constexpr int DARK_ON   = 1000;

int readADC() {
  long acc = 0;
  const int N = 8;
  for (int i = 0; i < N; ++i) {
    acc += analogRead(PIN_LDR);
    delayMicroseconds(200);
  }
  return acc / 8;
}

bool ledOn = false;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
}

void loop() {
  int v = readADC();
  Serial.printf("ADC=%d\n", v);

  if (v > DARK_ON) {
    Serial1.println("1");
    Serial.println("TX: 1");
    ledOn = true;
  } else {
    Serial1.println("0");
    Serial.println("TX: 0");
    ledOn = false;
  }

  delay(100);
}

// ESP32-S2-MINI-1U — KY-018 LDR -> MCXC444 via UART
// Sends "1" if ADC > THRESHOLD, else "0"

#include <Arduino.h>

constexpr int PIN_LDR = 1;      // ADC1_CH0 on ESP32-S2
constexpr int UART_TX = 3;      // to MCXC444 RX (PTD2)
constexpr int UART_RX = 18;     // from MCXC444 TX (PTD3)

constexpr int THRESHOLD = 500; // <-- TUNE THIS (0..4095 with 12-bit ADC)

static int ema = 0;             // simple smoothing

int readADC() {
  return analogRead(PIN_LDR);   // single read (fast)
}

void setup() {
  Serial.begin(115200);               // USB debug
  delay(200);

  analogReadResolution(12);           // 0..4095
  analogSetPinAttenuation(PIN_LDR, ADC_11db); // ~0–3.3V range

  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX); // UART to MCXC444

  // Seed EMA with first reading
  ema = analogRead(PIN_LDR);
}

void loop() {
  // Mirror anything from MCXC444 to USB (optional)
  while (Serial1.available()) Serial.write(Serial1.read());

  // Read & smooth (EMA alpha = 1/8)
  int v = readADC();
  ema += (v - ema) / 8;

  // Compare to threshold and transmit
  char bit = (ema < THRESHOLD) ? '1' : '0';
  Serial1.write(bit);
  Serial1.write('\n');

  // Debug to USB
  Serial.printf("ADC=%d  EMA=%d  THRESHOLD=%d  TX:%c\n", v, ema, THRESHOLD, bit);

  delay(100); // 10 Hz updates
}

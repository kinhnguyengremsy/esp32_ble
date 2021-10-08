#include "esp32_ble.h"
#include "esp32_IO.h"

#define LED_DEV_BOARD   34

void setup() {
  // put your setup code here, to run once:
  esp32_IO_init(LED_DEV_BOARD);
  pinMode(34, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("ESP32 BLE TEST !!!");

  digitalWrite(LED_DEV_BOARD, HIGH);
  delay(500);

  digitalWrite(LED_DEV_BOARD, LOW);
  delay(500);
}

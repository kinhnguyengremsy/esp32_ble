#include <Arduino.h>
#include "PEGremsy_BLE.h"

PEGremsy_BLE ble; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  ble.initialize();

}

void loop() {
  // put your main code here, to run repeatedly:
  ble.process();
}
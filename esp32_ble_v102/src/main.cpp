#include <Arduino.h>
#include "taskManagement.h"

taskManagement_t management;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  management.initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
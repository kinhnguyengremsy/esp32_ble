#include <Arduino.h>

#include "PEGremsy_BLE.h"
#include "PEGremsy_FireBase.h"
#include "mavlinkHandle.h"

PEGremsy_BLE_t ble;
PEGremsy_FireBase_t fireBase;
mavlinkHandle_t mavlink;

char BLE_characteristisJigBuffer[2];
String email = "kinh.nguyen@gremsy.com";

void setup()
{

    Serial.begin(115200);

    mavlink.initialize();

    pinMode(22, OUTPUT);

    fireBase.initialize();

    ble.initialize();
}

void loop()
{   
      ble.process();

      if(email.equals(BLE_characteristisJigBuffer) == true)
      {
        memset(BLE_characteristisJigBuffer, 0, 50);
        fireBase.process();
      }  
      else if(BLE_characteristisJigBuffer[0] == 0x01)
      {
        memset(BLE_characteristisJigBuffer, 0, 50);
        fireBase.process();
      } 
      else
      {
        // mavlink.process(NULL);
      }      
}
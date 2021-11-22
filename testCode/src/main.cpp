#include "Arduino.h"

#include "taskManagement.h"
/*
 * This program is a simple binary write/read benchmark.
 */
#include <SPI.h>
#include <UsbFat.h>
#include <masstorage.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

taskManagement_t management;

// Size of read/write.
const size_t BUF_SIZE = 1000;

// File size in MB where MB = 1,000,000 bytes.
const uint32_t FILE_SIZE_MB = 1;

// Write pass count.
const uint8_t WRITE_COUNT = 1;

// Read pass count.
const uint8_t READ_COUNT = 1;
//==============================================================================
// End of configuration constants.
//------------------------------------------------------------------------------
// File size in bytes.
const uint32_t FILE_SIZE = 1000000UL*FILE_SIZE_MB;

uint8_t buf[BUF_SIZE];

// USB host objects.
USB usb;
BulkOnly bulk(&usb);

// File system.
UsbFat key(&bulk);

// Test file.
FatFile file;

// Serial output stream
ArduinoOutStream cout(Serial);
//------------------------------------------------------------------------------
// store error strings in flash to save RAM
#define error(s) Serial.println(s);
//------------------------------------------------------------------------------


/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
    
  If you want to know what pin the on-board LED is connected to on your ESP32 model, check
  the Technical Specs of your board.
*/

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(2, OUTPUT);

  Serial.println("[TaskBlink] init done");

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(2, HIGH);
    delay(1000);
    digitalWrite(2, LOW);
    delay(1000);

    vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskAnalogReadA3(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  static bool isBenchmart = false;
  float s;
  uint32_t t;
  uint32_t maxLatency;
  uint32_t minLatency;
  uint32_t totalLatency;
  uint32_t countReadWrite = 0;
  
  // while (!Serial) {} // wait for Leonardo
  delay(1000);
  cout << F("\nUse a freshly formatted volume for best performance.\n");

  // use uppercase in hex and use 0X base prefix
  cout << uppercase << showbase << endl;

  Serial.println("[TaskAnalogReadA3] init done");

  for (;;)
  {
    if(isBenchmart == false)
    {
      isBenchmart = true;

      cout << F("get free Heap : ") << ESP.getFreeHeap() << endl;

      if (!initUSB(&usb)) {
        cout << F("initUSB failed") << endl;
      }
      // initialize the USB key or hard drive.
      if (!key.begin()) {
        cout << F("key.begin failed") << endl;
        while(1);
      }

      cout << F("Type is FAT") << int(key.vol()->fatType()) << endl;
      cout << F("Volume size: ") << key.volumeBlockCount()*512E-9;
      cout << F(" GB (GB = 1E9 bytes)") << endl;

      // open or create file - truncate existing file.
      if (!file.open("bench.dat", O_CREAT | O_TRUNC | O_RDWR)) {
        error("open failed");
      }

      // fill buf with known data
      for (uint16_t i = 0; i < (BUF_SIZE-2); i++) {
        buf[i] = 'A' + (i % 26);
      }
      buf[BUF_SIZE-2] = '\r';
      buf[BUF_SIZE-1] = '\n';

      cout << F("File size ") << FILE_SIZE_MB << F(" MB\n");
      cout << F("Buffer size ") << BUF_SIZE << F(" bytes\n");
      cout << F("Starting write test, please wait.") << endl << endl;

      // do write test
      uint32_t n = FILE_SIZE/sizeof(buf);
      cout <<F("write speed and latency") << endl;
      cout << F("speed,max,min,avg") << endl;
      cout << F("KB/Sec,usec,usec,usec") << endl;
      for (uint8_t nTest = 0; nTest < WRITE_COUNT; nTest++) {
        file.truncate(0);
        maxLatency = 0;
        minLatency = 9999999;
        totalLatency = 0;
        t = millis();
        for (uint32_t i = 0; i < n; i++) {
          uint32_t m = micros();
          uint16_t writeSize = file.write(buf, sizeof(buf));
          if (writeSize != sizeof(buf)) {
            error("write failed");
          }
          else
          {
            if(++countReadWrite > 100)
            {
              countReadWrite = 0;
              Serial.printf("write %5d byte\n", writeSize);
            }       
          }
          m = micros() - m;
          if (maxLatency < m) {
            maxLatency = m;
          }
          if (minLatency > m) {
            minLatency = m;
          }
          totalLatency += m; 
        }
        file.sync();
        t = millis() - t;
        s = file.fileSize();
        cout << s/t <<',' << maxLatency << ',' << minLatency;
        cout << ',' << totalLatency/n << endl;
      }

      cout << endl << F("Starting read test, please wait.") << endl;
      cout << endl <<F("read speed and latency") << endl;
      cout << F("speed,max,min,avg") << endl;
      cout << F("KB/Sec,usec,usec,usec") << endl;
      // do read test
      for (uint8_t nTest = 0; nTest < READ_COUNT; nTest++) {
        file.rewind();
        maxLatency = 0;
        minLatency = 9999999;
        totalLatency = 0;
        t = millis();
        for (uint32_t i = 0; i < n; i++) {
          buf[BUF_SIZE-1] = 0;
          uint32_t m = micros();
          uint16_t readSize = file.read(buf, sizeof(buf));
          if (readSize != sizeof(buf)) {
            error("read failed");
          }
          else
          {
            if(++countReadWrite > 100)
            {
              countReadWrite = 0;
              Serial.printf("read %5d byte\n", readSize);
            } 
          }
          m = micros() - m;
          if (maxLatency < m) {
            maxLatency = m;
          }
          if (minLatency > m) {
            minLatency = m;
          }
          totalLatency += m;
          if (buf[BUF_SIZE-1] != '\n') {
            error("data check");
          }  
        }
        t = millis() - t;
        cout << s/t <<',' << maxLatency << ',' << minLatency;
        cout << ',' << totalLatency/n << endl;
      }
      cout << endl << F("Done") << endl;
      file.close();
    }
    vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
  }
}

void setup() 
{
  Serial.begin(115200);

  // Now set up two tasks to run independently.
  // xTaskCreatePinnedToCore(
  //   TaskBlink
  //   ,  "TaskBlink"   // A name just for humans
  //   ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
  //   ,  NULL
  //   ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  //   ,  NULL 
  //   ,  ARDUINO_RUNNING_CORE);

  // xTaskCreatePinnedToCore(
  //   TaskAnalogReadA3
  //   ,  "AnalogReadA3"
  //   ,  5 * 1024  // Stack size
  //   ,  NULL
  //   ,  2  // Priority
  //   ,  NULL 
  //   ,  ARDUINO_RUNNING_CORE);

  management.initialize();
}
//------------------------------------------------------------------------------
void loop() {

}
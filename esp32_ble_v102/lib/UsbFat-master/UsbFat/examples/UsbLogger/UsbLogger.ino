/*
 * Simple data logger.
 */
#include <SPI.h>
#include <UsbFat.h>
#include <masstorage.h>

// Interval between data records in milliseconds.
// The interval must be greater than the maximum write latency plus the
// time to acquire and write data to avoid overrun errors.
// Run the UsbBench example to check the quality of your drive.
const uint32_t SAMPLE_INTERVAL_MS = 200;

// Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "Data"
//------------------------------------------------------------------------------
// USB host objects.
USB usb;
BulkOnly bulk(&usb);

// File system.
UsbFat key(&bulk);

// Log file.
File file;

// Time in micros for next data record.  Limits time to about one hour.
uint32_t logTime;
//==============================================================================
// User functions.  Edit writeHeader() and logData() for your requirements.

const uint8_t ANALOG_COUNT = 4;
//------------------------------------------------------------------------------
// Write data header.
void writeHeader() {
  file.print(F("micros"));
  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    file.print(F(",adc"));
    file.print(i, DEC);
  }
  file.println();
}
//------------------------------------------------------------------------------
// Log a data record.
void logData() {
  uint16_t data[ANALOG_COUNT];

  // Read all channels to avoid write latency between readings.
  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    data[i] = analogRead(i);
  }
  // Write data to file.  Start with log time in micros.
  file.print(logTime);

  // Write ADC data to CSV record.
  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    file.write(',');
    file.print(data[i]);
  }
  file.println();
}
//==============================================================================
// Error messages stored in flash.
#define error(msg) {Serial.print(F("Error: "));Serial.println(F(msg));while(1);}
//------------------------------------------------------------------------------
void setup() {
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";

  Serial.begin(9600);
  while (!Serial) {} // wait for Leonardo
  delay(1000);

  Serial.println(F("Type any character to start"));
  while (!Serial.available()) {}
  
  if (!initUSB(&usb)) {
    error("initUSB");
  }
  // Initialize the drive.
  if (!key.begin()) {
    error("key.begin");
  }

  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (key.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }
  if (!file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
    error("file.open");
  }
  do {
    delay(10);
  } while (Serial.read() >= 0);

  Serial.print(F("Logging to: "));
  Serial.println(fileName);
  Serial.println(F("Type any character to stop"));

  // Write data header.
  writeHeader();

  // Start on a multiple of the sample interval.
  logTime = micros()/(1000UL*SAMPLE_INTERVAL_MS) + 1;
  logTime *= 1000UL*SAMPLE_INTERVAL_MS;
}
//------------------------------------------------------------------------------
void loop() {
  // Time for next record.
  logTime += 1000UL*SAMPLE_INTERVAL_MS;

  // Wait for log time.
  int32_t diff;
  do {
    diff = micros() - logTime;
  } while (diff < 0);

  // Check for data rate too high.
  if (diff > 10) {
    error("Missed data record");
  }

  logData();

  // Force data to drive and update the directory entry to avoid data loss.
  if (!file.sync() || file.getWriteError()) {
    error("write error");
  }

  if (Serial.available()) {
    // Close file and stop.
    file.close();
    Serial.println(F("Done"));
    while(1) {}
  }
}
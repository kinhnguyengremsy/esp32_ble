/**
  ******************************************************************************
  * @file    taskManagement.c
  * @author  Gremsy Team
  * @version v100
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ************************************************************
  ******************
  * @par
  * COPYRIGHT NOTICE: (c) 2011 Gremsy.
  * All rights reserved.Firmware coding style V1.0.beta
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/
/* Includes------------------------------------------------------------------------------*/
#include "Arduino.h"
#include "taskManagement.h"
#include "mavlinkHandle.h"
#include "PEGremsy_BLE.h"

#include <SPI.h>
#include <UsbFat.h>
#include <masstorage.h>
/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

// store error strings in flash to save RAM
#define error(s) Serial.println(s);
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
extern taskManagement_t management;
PEGremsy_BLE ble; 
mavlinkHandle_t mavlink;

// Size of read/write.
const size_t BUF_SIZE = 100;

// File size in MB where MB = 1,000,000 bytes.
const uint32_t FILE_SIZE_MB = 5;

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
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/

/** @group TASK_MANAGEMENT_BLE_FUNCTION
    @{
*/#ifndef TASK_MANAGEMENT_BLE_FUNCTION
#define TASK_MANAGEMENT_BLE_FUNCTION

/** @brief  getJigStatus
    @return JigTestStatus_t
*/
JigTestStatus_t taskManagement_t::getJigStatus(void)
{
    JigTestStatus_t status;

    // management.jigReady = mavlink.mavlinkSerial1.heartBeat.flag_heartbeat;

    if(management.jigReady == true)
    {   
        /// kiem tra jig state test
        if(mavlink.state == CONTROL_JIG_STATE_IDLE)
        {
            status = JIG_STATUS_STANBY;
        }
        else
        {
            status = JIG_STATUS_RUNNING;
        }
    }
    else
    {
        status = JIG_STATUS_ERROR;
    }

    return status;
}

/** @brief  getProductStatus
    @return ProductStatus_t
*/
ProductStatus_t taskManagement_t::getProductStatus(void)
{
    ProductStatus_t status;

    if(productReady == true)
    {
        status = JIG_STATUS_A_PRODUCT_ATTACHED;
    }
    else
    {
        status = JIG_STATUS_NO_PRODUCT_ATTACHED;
    }

    return status;
}

/** @brief  getProductOnJigTestStatus
    @return ProductOnJigTestStatus_t
*/
ProductOnJigTestStatus_t taskManagement_t::getProductOnJigTestStatus(void)
{
    ProductOnJigTestStatus_t status;

    if(mavlink.state == CONTROL_JIG_STATE_IDLE)
    {
        status = PRODUCT_WAIT_FOR_QC;
    }
    else if(mavlink.state == CONTROL_JIG_STATE_DONE)
    {
        status = PRODUCT_COMPLETE;
    }
    else if(mavlink.state == CONTROL_JIG_STATE_ERROR)
    {
        status = PRODUCT_FAIL;
    }
    else 
    {
        status = PRODUCT_RUNNING;
    }

    return status;
}

/** @brief  getQcMode
    @return JigTestQcMode_t
*/
JigTestQcMode_t taskManagement_t::getQcMode(void)
{
    JigTestQcMode_t qcMode;

    qcMode = (JigTestQcMode_t)mavlink.mode;

    return qcMode;
}

/** @brief  getQcModeStatus
    @return JigTestQcModeStatus_t
*/
JigTestQcModeStatus_t taskManagement_t::getQcModeStatus(void)
{
    JigTestQcModeStatus_t status;

    status = qcModeStatus;

    return status;
}

/** @brief  getJigControl
    @return jigControl_t
*/
jigControl_t taskManagement_t::getJigControl(void)
{
    jigControl_t controlStatus = JIG_CONTROL_STOP;
    uint8_t control = JigControlBuffer[0];

    if(control == 0)
    {
        controlStatus = JIG_CONTROL_STOP;
    }
    else if(control == 1)
    {
        controlStatus = JIG_CONTROL_START;
    }
    else if(control == 2)
    {
        controlStatus = JIG_CONTROL_PAUSE;
    }
    else if(control == 3)
    {
        controlStatus = JIG_CONTROL_RESUME;
    }

    return controlStatus;
}
#endif
/**
    @}
*/

/** @group TASK_MANAGEMENT_INIT
    @{
*/#ifndef TASK_MANAGEMENT_INIT
#define TASK_MANAGEMENT_INIT

/** @brief  mavlinkTask
    @return none
*/
void mavlinkTask( void *pvParameters )
{
    /// task init

  cout << F("\nUse a freshly formatted volume for best performance.\n");

  // use uppercase in hex and use 0X base prefix
  cout << uppercase << showbase << endl;

    for(;;)
    {
        float s;
        uint32_t t;
        uint32_t maxLatency;
        uint32_t minLatency;
        uint32_t totalLatency;

        // discard any input
        // while (Serial.read() >= 0) {}

        // F( stores strings in flash to save RAM
        cout << F("Type any character to start\n");
        while (Serial.read() <= 0) {}
        delay(400);  // catch Due reset problem

        //  cout << F("Free RAM: ") << FreeRam() << endl;

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
            if (file.write(buf, sizeof(buf)) != sizeof(buf)) {
                error("write failed");
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
            if (file.read(buf, sizeof(buf)) != sizeof(buf)) {
                error("read failed");
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

        vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
    }
}

/** @brief  BLETask
    @return none
*/
void BLETask( void *pvParameters )
{
    /// task init
    ble.initialize();

    mavlink.initialize();

    Serial.println("start mavlink");

    for(;;)
    {
        // mavlink.process(NULL);
        ble.process();
    }
}

/** @brief initialize
    @return none
*/
void taskManagement_t::initialize(void)
{
// Now set up two tasks to run independently.

    xTaskCreatePinnedToCore(
    BLETask
    ,  "BLETask"   // A name just for humans
    ,  5 * 1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE); 

    
    xTaskCreatePinnedToCore(
    mavlinkTask
    ,  "mavlinkTask"   // A name just for humans
    ,  5 * 1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE); 

    xTaskCreate(&BLETask, "BLETask", 10 * 1024, NULL, 2, NULL);
    xTaskCreate(&mavlinkTask, "mavlinkTask", 10 * 1024, NULL, 1, NULL);
}

taskManagement_t::taskManagement_t(/* args */)
{
}

taskManagement_t::~taskManagement_t()
{
}

#endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/



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
#include "taskManagement.h"
#include "mavlinkHandle.h"
#include "PEGremsy_BLE.h"
/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
PEGremsy_BLE ble; 
mavlinkHandle_t mavlink;
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/

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

    mavlink.initialize();

    Serial.println("start mavlink");

    for(;;)
    {
        mavlink.process(NULL);
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
        ble.process();

        mavlink.process(NULL);
    }
}

/** @brief initialize
    @return initialize
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

    
    // xTaskCreatePinnedToCore(
    // mavlinkTask
    // ,  "mavlinkTask"   // A name just for humans
    // ,  10 * 1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    // ,  NULL
    // ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    // ,  NULL 
    // ,  ARDUINO_RUNNING_CORE); 

    // xTaskCreate(&BLETask, "BLETask", 10 * 1024, NULL, 2, NULL);
    // xTaskCreate(&mavlinkTask, "mavlinkTask", 10 * 1024, NULL, 1, NULL);
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



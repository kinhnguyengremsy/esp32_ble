/**
  ******************************************************************************
  * @file    .c
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
#include "esp32_IO.h"

/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/

/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/

// /** @group ESP32_IO_INIT
//     @{
// */#ifndef ESP32_IO_INIT
// #define ESP32_IO_INIT

/** @brief  esp32_IO_init
    @return none
*/
void esp32_IO_init(uint32_t pin)
{
    pinMode(pin, OUTPUT);

    Serial.begin(115200);
}

/** @brief  esp32_IO_set
    @return none
*/
void esp32_IO_set(uint32_t pin)
{
    digitalWrite(pin, HIGH);
}

/** @brief  esp32_IO_reset
    @return none
*/
void esp32_IO_reset(uint32_t pin)
{
    digitalWrite(pin, LOW);
}

/** @brief  esp32_IO_toggle
    @return none
*/
void esp32_IO_toggle(uint32_t pin)
{
    static bool state = false;
    static bool oldState = false;

    if(state == true) state = false;
    else state = true;

    if(oldState = true)
    digitalWrite(pin, HIGH);
    else
    digitalWrite(pin, LOW);

    oldState = ~state;
}

// #endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

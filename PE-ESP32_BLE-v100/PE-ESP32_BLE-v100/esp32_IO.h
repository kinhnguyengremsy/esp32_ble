/** 
  ******************************************************************************
  * @file    esp32_IO.h
  * @author  Gremsy Team
  * @version v1.0.0
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ******************************************************************************
  * @Copyright
  * COPYRIGHT NOTICE: (c) 2011 Gremsy. All rights reserved.
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or 
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __ESP32_IO_H
#define __ESP32_IO_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "Arduino.h"
/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @brief  esp32_IO_init
    @return none
*/
void esp32_IO_init(uint32_t pin);

/** @brief  esp32_IO_set
    @return none
*/
void esp32_IO_set(uint32_t pin);

/** @brief  esp32_IO_reset
    @return none
*/
void esp32_IO_reset(uint32_t pin);

/** @brief  esp32_IO_toggle
    @return none
*/
void esp32_IO_toggle(uint32_t pin);

#endif /* __ESP32_IO_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


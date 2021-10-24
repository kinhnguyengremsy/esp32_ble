/** 
  ******************************************************************************
  * @file    taskManagement.h
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

#ifndef __TASKMANAGEMENT_H
#define __TASKMANAGEMENT_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/*
  BLE Jig characristic reciever
*/
typedef enum
{
  BLE_CONTROL_JIG_STATUS_IDLE,
  BLE_CONTROL_JIG_STATUS_START,
  BLE_CONTROL_JIG_STATUS_STOP,

}BLE_controlJigStatus_t;

class taskManagement_t
{
  private:
    /* data */
  public:

    uint8_t BLE_characteristisJigBuffer[2];

    BLE_controlJigStatus_t getJigStatus(void);

    taskManagement_t(/* args */);
    ~taskManagement_t();

    void initialize(void);
    void BLETask( void *pvParameters );
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/


#endif /*  */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


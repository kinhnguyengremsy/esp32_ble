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

typedef enum
{
	JIG_STATUS_STANBY = 0x00,
	JIG_STATUS_RUNNING,
	JIG_STATUS_ERROR,

}JigTestStatus_t;

typedef enum
{
	JIG_STATUS_NO_PRODUCT_ATTACHED = 0x00,
	JIG_STATUS_A_PRODUCT_ATTACHED,

}ProductStatus_t;

typedef enum
{
	PRODUCT_WAIT_FOR_QC = 0x00,
	PRODUCT_RUNNING,
	PRODUCT_COMPLETE,
	PRODUCT_FAIL,

}ProductOnJigTestStatus_t;

typedef enum
{
	JIG_QC_MODE_IDLE = 0x00,
	JIG_QC_MODE_SBUS,
	JIG_QC_MODE_PPM,
	JIG_QC_MODE_CAN,
	JIG_QC_MODE_COM2,
	JIG_QC_MODE_COM4,
	JIG_QC_MODE_AUX,
	JIG_QC_MODE_VIRATE,

}JigTestQcMode_t;

typedef enum
{
	JIG_QC_MODE_STATUS_IDLE = 0x00,
	JIG_QC_MODE_STATUS_RUNNING,
	JIG_QC_MODE_STATUS_PASSED,
	JIG_QC_MODE_STATUS_FAILED,

}JigTestQcModeStatus_t;

typedef enum
{
  JIG_CONTROL_STOP = 0x00,
  JIG_CONTROL_START,
  JIG_CONTROL_PAUSE,
  JIG_CONTROL_RESUME,

}jigControl_t;

class taskManagement_t
{
  private:
    /* data */
  public:

    uint8_t JigControlBuffer[1];
    uint8_t productProfileBuffer[1];
    uint8_t countDeviceConnected;

    bool jigReady;
    bool productReady;
    bool deviceDisconnected;
    uint8_t oldDeviceConnected;
    bool appOnRead;

    ProductOnJigTestStatus_t  productOnState;
    JigTestQcModeStatus_t     qcModeStatus;

    JigTestStatus_t getJigStatus(void);
    ProductStatus_t getProductStatus(void);
    ProductOnJigTestStatus_t getProductOnJigTestStatus(void);
    JigTestQcMode_t getQcMode(void);
    JigTestQcModeStatus_t getQcModeStatus(void);
    jigControl_t getJigControl(void);

    taskManagement_t(/* args */);
    ~taskManagement_t();

    void initialize(void);
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/


#endif /*  */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


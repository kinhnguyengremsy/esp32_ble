/** 
  ******************************************************************************
  * @file    PEGremsy_BLE.h
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

#ifndef __PEGREMSY_BLE_H
#define __PEGREMSY_BLE_H

/* Includes ------------------------------------------------------------------*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
class PEGremsy_BLE_t
{
  private:
    /* data */
  public:
    void initialize(void);
    void process(void);

		/*
		 * Characteristics for Information Service
		 */
		// BLECharacteristic *pCharacterNameManufacture;
		// BLECharacteristic *pCharacterModelNumber;
		// BLECharacteristic *pCharacterSerialNumber;
		// BLECharacteristic *pCharacterFirmwareVerison;
		// BLECharacteristic *pCharacterHardwareVerison;
		// BLECharacteristic *pCharacterPnPId;

		/*
		 * Characteristics for JIGTEST
		 * */
		BLECharacteristic 	*pCharacteristicsJIGTEST;
		// BLEDescriptor 		*pJigTestDescriptor;

    PEGremsy_BLE_t(/* args */);
    ~PEGremsy_BLE_t();
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/


#endif /*  */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


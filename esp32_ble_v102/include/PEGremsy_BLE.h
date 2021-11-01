/** 
  ******************************************************************************
  * @file    PEGremsy_BLE.h
  * @author  Gremsy Team
  * @version v100
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ******************************************************************************
  * @Copyright
  * COPYRIGHT NOTICE: (c) 2021 Gremsy. All rights reserved.
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

//#ifdef __cplusplus
//extern "C" {
//#endif

/* Includes ------------------------------------------------------------------*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "iostream"
#include <string.h>
#include <stdio.h>
#include "driver/gpio.h"
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include "myMath.h"
#include "mavlinkHandle.h"
#include "taskManagement.h"
/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class PEGremsy_BLE
{
	public:

		mavlink_msg_heartbeat_t 	heartBeat;
		mavlink_msg_param_value_t 	paramValue;

		/*
		 * Characteristics for Battery Service
		 */
		BLECharacteristic *pCharacterBatteryLevel;
		BLEDescriptor 		*pDescriptorBatteryLevel;

		/*
		 * Characteristics for Information Service
		 */
		BLECharacteristic *pCharacterNameManufacture;
		BLECharacteristic *pCharacterModelNumber;
		BLECharacteristic *pCharacterSerialNumber;
		BLECharacteristic *pCharacterFirmwareVerison;
		BLECharacteristic *pCharacterHardwareVerison;
		BLECharacteristic *pCharacterPnPId;

		/*
		 * Characteristics for JIGTEST
		 * */
		BLECharacteristic 	*pCharacteristicsJigStatus;
		BLECharacteristic 	*pCharacteristicsProductOnJigStatus;
		BLECharacteristic 	*pCharacteristicsJigQcMode;
		BLECharacteristic 	*pCharacteristicsJigControl;

		/*
		 * Characteristics for mavlink
		 * */
		BLECharacteristic 	*pCharacteristicsHeartbeat;
		BLEDescriptor 		*pDescriptorHeartbeat;

		BLECharacteristic 	*pCharacteristicsParamValue;

		BLECharacteristic 	*pCharacteristicsRawImu;

		void initialize(void);

		void deinit(void);

		bool heartBeatHandle(void);

		void process(void);

		PEGremsy_BLE();
		virtual ~PEGremsy_BLE();

	private:

		bool flagSendJigStatus;
		bool flagSendProductOnJigStatus;
		bool flagSendJigQcMode;
		bool flagSendJigControl;

		void CharacteristicsJigStatus_Initialize(BLEServer* pServer);
		void CharacteristicsMavlink_Initialize(BLEServer* pServer);
		void CharacteristicsDeviceInfo_Initialize(BLEServer* pServer);
		void send_heartBeatService(mavlink_msg_heartbeat_t *heartbeat, bool Notify)
		{

			uint8_t temp[4];
			temp[0] = heartbeat->custom_mode;
			temp[1] = heartbeat->custom_mode >> 8;
			temp[2] = heartbeat->custom_mode >> 16;
			temp[3] = heartbeat->custom_mode >> 24;

			uint8_t value[10] = {
					temp[0], temp[1], temp[2], temp[3]
					, heartbeat->type
					, heartbeat->autopilot
					, heartbeat->base_mode
					, heartbeat->system_status
					, heartbeat->mavlink_version
					, heartbeat->flag_heartbeat
			};

			pCharacteristicsHeartbeat->setValue((uint8_t *)value, 10);
			pCharacteristicsHeartbeat->notify(Notify);

		}

		void send_paramValueCharastics(mavlink_msg_param_value_t *param_value, bool Notify)
		{
			uint8_t temp[4];
			ConvertFloat2Hex(param_value->param_value, temp);

			uint8_t temp1[2];
			temp1[0] = param_value->param_count;
			temp1[1] = param_value->param_count >> 8;

			uint8_t temp2[2];
			temp2[0] = param_value->param_index;
			temp2[1] = param_value->param_index >> 8;

			uint8_t value[25] = {
					temp[0], temp[1], temp[2], temp[3] /// param_value
					, temp1[0], temp1[1] /// param count
					, temp2[0], temp2[1] /// param_index
			};

			memcpy(value + 6, param_value->param_id, 16); /// param id
			value[24] = param_value->param_type;

			pCharacteristicsParamValue->setValue((uint8_t*)value, 26);
			pCharacteristicsParamValue->notify(Notify);
		}

		void send_rawImuCharastics(mavlink_msg_raw_imu_t *raw_imu, bool Notify)
		{
			int8_t temp[2];

			temp[0] = raw_imu->xacc;
			temp[1] = raw_imu->xacc >> 8;

			int8_t temp1[2];

			temp1[0] = raw_imu->yacc;
			temp1[1] = raw_imu->yacc >> 8;

			int8_t temp2[2];

			temp2[0] = raw_imu->zacc;
			temp2[1] = raw_imu->zacc >> 8;

			int8_t temp3[2];

			temp3[0] = raw_imu->xgyro;
			temp3[1] = raw_imu->xgyro >> 8;

			int8_t temp4[2];

			temp4[0] = raw_imu->ygyro;
			temp4[1] = raw_imu->ygyro >> 8;

			int8_t temp5[2];

			temp5[0] = raw_imu->zgyro;
			temp5[1] = raw_imu->zgyro >> 8;

			int8_t temp6[2];

			temp6[0] = raw_imu->xmag;
			temp6[1] = raw_imu->xmag >> 8;

			int8_t temp7[2];

			temp7[0] = raw_imu->ymag;
			temp7[1] = raw_imu->ymag >> 8;

			int8_t temp8[2];

			temp8[0] = raw_imu->zmag;
			temp8[1] = raw_imu->zmag >> 8;

			uint8_t value[19] = {
				temp[0], temp[1] // xacc
				,temp1[0], temp1[1] // yacc
				,temp2[0], temp2[1] // zacc
				,temp3[0], temp3[1] // xgyro
				,temp4[0], temp4[1] // ygyro
				,temp5[0], temp5[1] // zgyro
				,temp6[0], temp6[1] // xmag
				,temp7[0], temp7[1] // ymag
				,temp8[0], temp8[1] // zmag
				,raw_imu->id
			};

			pCharacteristicsRawImu->setValue((uint8_t *)value, 19);
			pCharacteristicsRawImu->notify(Notify);
		}

		void send_jigStatus(bool Notify);
		void send_ProductOnJigStatus(bool Notify);
		void send_jigQcMode(bool Notify);
		void send_jigControl(void);
};
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

#endif /* __PEGREMSY_BLE_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
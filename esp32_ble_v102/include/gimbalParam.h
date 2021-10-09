/** 
  ******************************************************************************
  * @file    gimbalParam.h
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

#ifndef __GIMBALPARAM_H
#define __GIMBALPARAM_H

//#ifdef __cplusplus
//extern "C" {
//#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported define ------------------------------------------------------------*/
#define JIG_TEST_NUMBER_OF_PARAM_SETTING          36

#define GIMBAL_PARAM_TEST_SBUS_MODE_CHAN          6
#define GIMBAL_PARAM_TEST_SBUS_PITCH_CHAN         1
#define GIMBAL_PARAM_TEST_SBUS_ROLL_CHAN          7
#define GIMBAL_PARAM_TEST_SBUS_YAW_CHAN           0
#define GIMBAL_PARAM_TEST_YAW_D                   2
#define GIMBAL_PARAM_TEST_PITCH_D                 2
#define GIMBAL_PARAM_TEST_NPOLES_YAW              40
#define GIMBAL_PARAM_TEST_NPOLES_ROLL             40
#define GIMBAL_PARAM_TEST_NPOLES_PITCH            40

#define GIMBAL_PARAM_TEST_YAW_P_PIXY              60
#define GIMBAL_PARAM_TEST_ROLL_P_PIXY             50
#define GIMBAL_PARAM_TEST_PITCH_P_PIXY            50

#define GIMBAL_PARAM_TEST_YAW_P_T3V3              90
#define GIMBAL_PARAM_TEST_ROLL_P_T3V3             80
#define GIMBAL_PARAM_TEST_PITCH_P_T3V3            70

#define GIMBAL_PARAM_TEST_YAW_P_S1V3              60
#define GIMBAL_PARAM_TEST_ROLL_P_S1V3             50
#define GIMBAL_PARAM_TEST_PITCH_P_S1V3            45
#define GIMBAL_PARAM_TEST_PITCH_POWER_S1V3        30
#define GIMBAL_PARAM_TEST_ROLL_POWER_S1V3         40
#define GIMBAL_PARAM_TEST_YAW_POWER_S1V3          40

#define GIMBAL_PARAM_TEST_YAW_P_T7                70
#define GIMBAL_PARAM_TEST_ROLL_P_T7               60
#define GIMBAL_PARAM_TEST_PITCH_P_T7              50

#define GIMBAL_PARAM_TEST_PITCH_POWER             40
#define GIMBAL_PARAM_TEST_ROLL_POWER              40
#define GIMBAL_PARAM_TEST_YAW_POWER               40
#define GIMBAL_PARAM_TEST_JOY_AXIS                16
#define GIMBAL_PARAM_TEST_RC_PITCH_SPEED          60
#define GIMBAL_PARAM_TEST_RC_ROLL_SPEED           60
#define GIMBAL_PARAM_TEST_RC_YAW_SPEED            60

#define GIMBAL_PARAM_TEST_GYRO_LPF_T3V3_BMI       2
#define GIMBAL_PARAM_TEST_GYRO_LPF_T3V3_ICM       4
#define GIMBAL_PARAM_TEST_GYRO_LPF_S1V3_BMI       2
#define GIMBAL_PARAM_TEST_GYRO_LPF_S1V3_ICM       2
#define GIMBAL_PARAM_TEST_GYRO_LPF_T7_BMI         2
#define GIMBAL_PARAM_TEST_GYRO_LPF_T7_ICM         8

#define GIMBAL_PARAM_TEST_YAW_I                   4 /// Output filtter
#define GIMBAL_PARAM_TEST_YAW_I_S1V3              4
#define GIMBAL_PARAM_TEST_YAW_I_T7                8

#define GIMBAL_PARAM_TEST_PITCH_I                 120
#define GIMBAL_PARAM_TEST_GYRO_TRUST              210
#define GIMBAL_PARAM_TEST_IMU_RATE                15
#define GIMBAL_PARAM_TEST_HEARTBEAT_EMIT          1
#define GIMBAL_PARAM_TEST_STATUS_RATE             10
#define GIMBAL_PARAM_TEST_ENC_CNT_RATE            10
#define GIMBAL_PARAM_TEST_ORIEN_RATE              10

#define GIMBAL_PARAM_TEST_ACSL_ROTATION_TRAVEL_MIN_PIT      -90/// Rotation TILT Up
#define GIMBAL_PARAM_TEST_ACSL_ROTATION_TRAVEL_MAX_PIT      30/// Rotation TILT Down

#define GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MIN_PIT           -90/// Rotation TILT Up
#define GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MAX_PIT           90/// Rotation TILT Down
#define GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MIN_ROLL          -80/// Rotation ROLL Up
#define GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MAX_ROLL          80/// Rotation ROLL Down
#define GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MIN_PAN           -180/// Rotation PAN Up
#define GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MAX_PAN           180/// Rotation PAN Down

#define GIMBAL_PARAM_TEST_AUTOPOWER_DIR_MOTOR_ROLL          0 /// Auto Power ON
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  GIMBAL_DEVICE_PIXY,
  GIMBAL_DEVICE_T3V3,
  GIMBAL_DEVICE_S1V3,
  GIMBAL_DEVICE_T7,
  GIMBAL_DEVICE_AC30000,
}gimbalDevice_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Private variables----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
float gimbalParam_getValueTest(gimbalDevice_t device, uint8_t pos);
char *gimbalParam_getIdTest(gimbalDevice_t device, uint8_t pos);
//#ifdef __cplusplus
//}
//#endif

#endif /*  */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

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
#define NUMBER_OF_GIMBAL_PARAM_TEST               36

#define GIMBAL_PARAM_TEST_SBUS_MODE_CHAN          6
#define GIMBAL_PARAM_TEST_SBUS_PITCH_CHAN         1
#define GIMBAL_PARAM_TEST_SBUS_ROLL_CHAN          7
#define GIMBAL_PARAM_TEST_SBUS_YAW_CHAN           0
#define GIMBAL_PARAM_TEST_YAW_D                   2
#define GIMBAL_PARAM_TEST_PITCH_D                 2
#define GIMBAL_PARAM_TEST_NPOLES_YAW              40
#define GIMBAL_PARAM_TEST_NPOLES_ROLL             40
#define GIMBAL_PARAM_TEST_NPOLES_PITCH            40

#define GIMBAL_PARAM_TEST_YAW_P_PIXY              25
#define GIMBAL_PARAM_TEST_ROLL_P_PIXY             15
#define GIMBAL_PARAM_TEST_PITCH_P_PIXY            15

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

#define GIMBAL_PARAM_TEST_GYRO_LPF_PIXY_BMI       2
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

#define GIMBAL_PARAM_TEST_PIXY_ROTATION_TRAVEL_MIN_PIT           -90/// Rotation TILT Up
#define GIMBAL_PARAM_TEST_PIXY_ROTATION_TRAVEL_MAX_PIT           90/// Rotation TILT Down

#define GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MIN_PIT           -90/// Rotation TILT Up
#define GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MAX_PIT           90/// Rotation TILT Down
#define GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MIN_ROLL          -80/// Rotation ROLL Up
#define GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MAX_ROLL          80/// Rotation ROLL Down
#define GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MIN_PAN           -180/// Rotation PAN Up
#define GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MAX_PAN           180/// Rotation PAN Down

#define GIMBAL_PARAM_TEST_DIR_MOTOR_ROLL            0 /// Auto Power ON
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  GIMBAL_DEVICE_PIXY,
  GIMBAL_DEVICE_T3V3,
  GIMBAL_DEVICE_S1V3,
  GIMBAL_DEVICE_T7,
  GIMBAL_DEVICE_AC30000,
}gimbalDevice_t;

struct 
{
    float value;
    float value_param_get;
    uint16_t index;
    const char* param_id;
}gimbal_param_setting[NUMBER_OF_GIMBAL_PARAM_TEST] = 
{
    {0, 0, 0, "VERSION_X"},
    {0, 0, 41, "SBUS_YAW_CHAN"},
    {0, 0, 40, "SBUS_ROLL_CHAN"},
    {0, 0, 39, "SBUS_PITCH_CHAN"},
    {0, 0, 42, "SBUS_MODE_CHAN"},
    {0, 0, 10, "YAW_D"},
    {0, 0, 4, "PITCH_D"},
    {0, 0, 23, "NPOLES_YAW"},
    {0, 0, 22, "NPOLES_ROLL"},
    {0, 0, 21, "NPOLES_PITCH"},
    {0, 0, 8, "YAW_P"},
    {0, 0, 5, "ROLL_P"},
    {0, 0, 2, "PITCH_P"},//12
    {0, 0, 11, "PITCH_POWER"},
    {0, 0, 12, "ROLL_POWER"},
    {0, 0, 13, "YAW_POWER"},//15
    {0, 0, 63, "JOY_AXIS"}, /// ???
    {0, 0, 60, "RC_PITCH_SPEED"},
    {0, 0, 61, "RC_ROLL_SPEED"},
    {0, 0, 62, "RC_YAW_SPEED"},
    {0, 0, 29, "GYRO_LPF"},//20
    {0, 0, 9, "YAW_I"},
    {0, 0, 3, "PITCH_I"},
    {0, 0, 20, "GYRO_TRUST"},
    {0, 0, 77, "IMU_RATE"},
    {0, 0, 72, "HEARTBEAT_EMIT"},
    {0, 0, 73, "STATUS_RATE"},
    {0, 0, 74, "ENC_CNT_RATE"},
    {0, 0, 76, "ORIEN_RATE"}, // 28
    {0, 0, 30, "TRAVEL_MIN_PIT"}, 
    {0, 0, 31, "TRAVEL_MAX_PIT"}, 
    {0, 0, 32, "TRAVEL_MIN_ROLL"}, 
    {0, 0, 33, "TRAVEL_MAX_ROLL"}, 
    {0, 0, 69, "TRAVEL_MIN_PAN"}, 
    {0, 0, 70, "TRAVEL_MAX_PAN"}, // 34
    {0, 0, 25, "DIR_MOTOR_ROLL"}, // 35
};

struct 
{
  /* data */
  float value;
}gimbalParam_valueTestPixy[NUMBER_OF_GIMBAL_PARAM_TEST] =
{
  {0},
  {GIMBAL_PARAM_TEST_SBUS_YAW_CHAN},
  {GIMBAL_PARAM_TEST_SBUS_ROLL_CHAN},
  {GIMBAL_PARAM_TEST_SBUS_PITCH_CHAN},
  {GIMBAL_PARAM_TEST_SBUS_MODE_CHAN},
  {GIMBAL_PARAM_TEST_YAW_D},
  {GIMBAL_PARAM_TEST_PITCH_D},
  {GIMBAL_PARAM_TEST_NPOLES_YAW},
  {GIMBAL_PARAM_TEST_NPOLES_ROLL},
  {GIMBAL_PARAM_TEST_NPOLES_PITCH},
  {GIMBAL_PARAM_TEST_YAW_P_PIXY},
  {GIMBAL_PARAM_TEST_ROLL_P_PIXY},
  {GIMBAL_PARAM_TEST_PITCH_P_PIXY},
  {GIMBAL_PARAM_TEST_PITCH_POWER},
  {GIMBAL_PARAM_TEST_ROLL_POWER},
  {GIMBAL_PARAM_TEST_YAW_POWER},
  {GIMBAL_PARAM_TEST_JOY_AXIS},
  {GIMBAL_PARAM_TEST_RC_PITCH_SPEED},
  {GIMBAL_PARAM_TEST_RC_ROLL_SPEED},
  {GIMBAL_PARAM_TEST_RC_YAW_SPEED},
  {GIMBAL_PARAM_TEST_GYRO_LPF_PIXY_BMI},
  {GIMBAL_PARAM_TEST_YAW_I},
  {GIMBAL_PARAM_TEST_PITCH_I},
  {GIMBAL_PARAM_TEST_GYRO_TRUST},
  {GIMBAL_PARAM_TEST_IMU_RATE},
  {GIMBAL_PARAM_TEST_HEARTBEAT_EMIT},
  {GIMBAL_PARAM_TEST_STATUS_RATE},
  {GIMBAL_PARAM_TEST_ENC_CNT_RATE},
  {GIMBAL_PARAM_TEST_ORIEN_RATE},
  {GIMBAL_PARAM_TEST_PIXY_ROTATION_TRAVEL_MIN_PIT},
  {GIMBAL_PARAM_TEST_PIXY_ROTATION_TRAVEL_MAX_PIT},
  {GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MIN_ROLL},
  {GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MAX_ROLL},
  {GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MIN_PAN},
  {GIMBAL_PARAM_TEST_ROTATION_TRAVEL_MAX_PAN},
  {GIMBAL_PARAM_TEST_DIR_MOTOR_ROLL},
};
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Private variables----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

//#ifdef __cplusplus
//}
//#endif

#endif /*  */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

/**
  ******************************************************************************
  * @file .c
  * @author  Gremsy Team
  * @version v100
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  * https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=eziya76&logNo=220988141146
  ************************************************************
  ******************
  * @par
  * COPYRIGHT NOTICE: (c) 2016 Gremsy.
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
#include "gimbalParam.h"

/* Private define------------------------------------------------------------------------------*/
#define JIG_TEST_SETTING_PARAM_TEST_SBUS_MODE_CHAN          6
#define JIG_TEST_SETTING_PARAM_TEST_SBUS_PITCH_CHAN         1
#define JIG_TEST_SETTING_PARAM_TEST_SBUS_ROLL_CHAN          7
#define JIG_TEST_SETTING_PARAM_TEST_SBUS_YAW_CHAN           0
#define JIG_TEST_SETTING_PARAM_TEST_YAW_D                   2
#define JIG_TEST_SETTING_PARAM_TEST_PITCH_D                 2
#define JIG_TEST_SETTING_PARAM_TEST_NPOLES_YAW              40
#define JIG_TEST_SETTING_PARAM_TEST_NPOLES_ROLL             40
#define JIG_TEST_SETTING_PARAM_TEST_NPOLES_PITCH            40

#define JIG_TEST_SETTING_PARAM_TEST_YAW_P_PIXY              60
#define JIG_TEST_SETTING_PARAM_TEST_ROLL_P_PIXY             50
#define JIG_TEST_SETTING_PARAM_TEST_PITCH_P_PIXY            50

#define JIG_TEST_SETTING_PARAM_TEST_YAW_P_T3V3              90
#define JIG_TEST_SETTING_PARAM_TEST_ROLL_P_T3V3             80
#define JIG_TEST_SETTING_PARAM_TEST_PITCH_P_T3V3            70

#define JIG_TEST_SETTING_PARAM_TEST_YAW_P_S1V3              60
#define JIG_TEST_SETTING_PARAM_TEST_ROLL_P_S1V3             50
#define JIG_TEST_SETTING_PARAM_TEST_PITCH_P_S1V3            45
#define JIG_TEST_SETTING_PARAM_TEST_PITCH_POWER_S1V3        30
#define JIG_TEST_SETTING_PARAM_TEST_ROLL_POWER_S1V3         40
#define JIG_TEST_SETTING_PARAM_TEST_YAW_POWER_S1V3          40

#define JIG_TEST_SETTING_PARAM_TEST_YAW_P_T7                70
#define JIG_TEST_SETTING_PARAM_TEST_ROLL_P_T7               60
#define JIG_TEST_SETTING_PARAM_TEST_PITCH_P_T7              50

#define JIG_TEST_SETTING_PARAM_TEST_PITCH_POWER             40
#define JIG_TEST_SETTING_PARAM_TEST_ROLL_POWER              40
#define JIG_TEST_SETTING_PARAM_TEST_YAW_POWER               40
#define JIG_TEST_SETTING_PARAM_TEST_JOY_AXIS                16
#define JIG_TEST_SETTING_PARAM_TEST_RC_PITCH_SPEED          60
#define JIG_TEST_SETTING_PARAM_TEST_RC_ROLL_SPEED           60
#define JIG_TEST_SETTING_PARAM_TEST_RC_YAW_SPEED            60

#define JIG_TEST_SETTING_PARAM_TEST_GYRO_LPF_PIXY_BMI       2
#define JIG_TEST_SETTING_PARAM_TEST_GYRO_LPF_T3V3_BMI       2
#define JIG_TEST_SETTING_PARAM_TEST_GYRO_LPF_T3V3_ICM       4
#define JIG_TEST_SETTING_PARAM_TEST_GYRO_LPF_S1V3_BMI       2
#define JIG_TEST_SETTING_PARAM_TEST_GYRO_LPF_S1V3_ICM       2
#define JIG_TEST_SETTING_PARAM_TEST_GYRO_LPF_T7_BMI         2
#define JIG_TEST_SETTING_PARAM_TEST_GYRO_LPF_T7_ICM         8

#define JIG_TEST_SETTING_PARAM_TEST_YAW_I                   4 /// Output filtter
#define JIG_TEST_SETTING_PARAM_TEST_YAW_I_S1V3              4
#define JIG_TEST_SETTING_PARAM_TEST_YAW_I_T7                8

#define JIG_TEST_SETTING_PARAM_TEST_PITCH_I                 120
#define JIG_TEST_SETTING_PARAM_TEST_GYRO_TRUST              210
#define JIG_TEST_SETTING_PARAM_TEST_IMU_RATE                15
#define JIG_TEST_SETTING_PARAM_TEST_HEARTBEAT_EMIT          1
#define JIG_TEST_SETTING_PARAM_TEST_STATUS_RATE             10
#define JIG_TEST_SETTING_PARAM_TEST_ENC_CNT_RATE            10
#define JIG_TEST_SETTING_PARAM_TEST_ORIEN_RATE              10

#define JIG_TEST_SETTING_PARAM_TEST_PIXY_ROTATION_TRAVEL_MIN_PIT           -90/// Rotation TILT Up
#define JIG_TEST_SETTING_PARAM_TEST_PIXY_ROTATION_TRAVEL_MAX_PIT           90/// Rotation TILT Down

#define JIG_TEST_SETTING_PARAM_TEST_ROTATION_TRAVEL_MIN_PIT           -90/// Rotation TILT Up
#define JIG_TEST_SETTING_PARAM_TEST_ROTATION_TRAVEL_MAX_PIT           90/// Rotation TILT Down
#define JIG_TEST_SETTING_PARAM_TEST_ROTATION_TRAVEL_MIN_ROLL          -80/// Rotation ROLL Up
#define JIG_TEST_SETTING_PARAM_TEST_ROTATION_TRAVEL_MAX_ROLL          80/// Rotation ROLL Down
#define JIG_TEST_SETTING_PARAM_TEST_ROTATION_TRAVEL_MIN_PAN           -180/// Rotation PAN Up
#define JIG_TEST_SETTING_PARAM_TEST_ROTATION_TRAVEL_MAX_PAN           180/// Rotation PAN Down

#define IG_TEST_SETTING_PARAM_TEST_DIR_MOTOR_ROLL            0 /// Auto Power ON
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
/* Private typedef------------------------------------------------------------------------------*/
struct _gimbal_param_setting
{
    float value;
    float value_param_get;
    int16_t index;
    char* param_id;
}gimbal_param_setting[JIG_TEST_NUMBER_OF_PARAM_SETTING] = 
{
    {.value = 0, .index = 0, .param_id = "VERSION_X"},
    {.value = 0, .index = 41, .param_id = "SBUS_YAW_CHAN"},
    {.value = 0, .index = 40, .param_id = "SBUS_ROLL_CHAN"},
    {.value = 0, .index = 39, .param_id = "SBUS_PITCH_CHAN"},
    {.value = 0, .index = 42, .param_id = "SBUS_MODE_CHAN"},
    {.value = 0, .index = 10, .param_id = "YAW_D"},
    {.value = 0, .index = 4, .param_id = "PITCH_D"},
    {.value = 0, .index = 23, .param_id = "NPOLES_YAW"},
    {.value = 0, .index = 22, .param_id = "NPOLES_ROLL"},
    {.value = 0, .index = 21, .param_id = "NPOLES_PITCH"},
    {.value = 0, .index = 8, .param_id = "YAW_P"},
    {.value = 0, .index = 5, .param_id = "ROLL_P"},
    {.value = 0, .index = 2, .param_id = "PITCH_P"},//12
    {.value = 0, .index = 11, .param_id = "PITCH_POWER"},
    {.value = 0, .index = 12, .param_id = "ROLL_POWER"},
    {.value = 0, .index = 13, .param_id = "YAW_POWER"},//15
    {.value = 0, .index = 63, .param_id = "JOY_AXIS"}, /// ???
    {.value = 0, .index = 60, .param_id = "RC_PITCH_SPEED"},
    {.value = 0, .index = 61, .param_id = "RC_ROLL_SPEED"},
    {.value = 0, .index = 62, .param_id = "RC_YAW_SPEED"},
    {.value = 0, .index = 29, .param_id = "GYRO_LPF"},//20
    {.value = 0, .index = 9, .param_id = "YAW_I"},
    {.value = 0, .index = 3, .param_id = "PITCH_I"},
    {.value = 0, .index = 20, .param_id = "GYRO_TRUST"},
    {.value = 0, .index = 77, .param_id = "IMU_RATE"},
    {.value = 0, .index = 72, .param_id = "HEARTBEAT_EMIT"},
    {.value = 0, .index = 73, .param_id = "STATUS_RATE"},
    {.value = 0, .index = 74, .param_id = "ENC_CNT_RATE"},
    {.value = 0, .index = 76, .param_id = "ORIEN_RATE"}, // 28
    {.value = 0, .index = 30, .param_id = "TRAVEL_MIN_PIT"}, 
    {.value = 0, .index = 31, .param_id = "TRAVEL_MAX_PIT"}, 
    {.value = 0, .index = 32, .param_id = "TRAVEL_MIN_ROLL"}, 
    {.value = 0, .index = 33, .param_id = "TRAVEL_MAX_ROLL"}, 
    {.value = 0, .index = 69, .param_id = "TRAVEL_MIN_PAN"}, 
    {.value = 0, .index = 70, .param_id = "TRAVEL_MAX_PAN"}, // 34
    {.value = 0, .index = 25, .param_id = "DIR_MOTOR_ROLL"}, // 35
};

struct _gimbalParam_valueTestPixy
{
  /* data */
  float value;
}gimbalParam_valueTestPixy[JIG_TEST_NUMBER_OF_PARAM_SETTING] =
{
  {.value = 0},
  {.value = JIG_TEST_SETTING_PARAM_TEST_SBUS_YAW_CHAN},
  {.value = JIG_TEST_SETTING_PARAM_TEST_SBUS_ROLL_CHAN},
  {.value = JIG_TEST_SETTING_PARAM_TEST_SBUS_PITCH_CHAN},
  {.value = JIG_TEST_SETTING_PARAM_TEST_SBUS_MODE_CHAN},
  {.value = JIG_TEST_SETTING_PARAM_TEST_YAW_D},
  {.value = JIG_TEST_SETTING_PARAM_TEST_PITCH_D},
  {.value = JIG_TEST_SETTING_PARAM_TEST_NPOLES_YAW},
  {.value = JIG_TEST_SETTING_PARAM_TEST_NPOLES_ROLL},
  {.value = JIG_TEST_SETTING_PARAM_TEST_NPOLES_PITCH},
  {.value = JIG_TEST_SETTING_PARAM_TEST_YAW_P_PIXY},
  {.value = JIG_TEST_SETTING_PARAM_TEST_ROLL_P_PIXY},
  {.value = JIG_TEST_SETTING_PARAM_TEST_PITCH_P_PIXY},
  {.value = JIG_TEST_SETTING_PARAM_TEST_PITCH_POWER},
  {.value = JIG_TEST_SETTING_PARAM_TEST_ROLL_POWER},
  {.value = JIG_TEST_SETTING_PARAM_TEST_YAW_POWER},
  {.value = JIG_TEST_SETTING_PARAM_TEST_JOY_AXIS},
  {.value = JIG_TEST_SETTING_PARAM_TEST_RC_PITCH_SPEED},
  {.value = JIG_TEST_SETTING_PARAM_TEST_RC_ROLL_SPEED},
  {.value = JIG_TEST_SETTING_PARAM_TEST_RC_YAW_SPEED},
  {.value = JIG_TEST_SETTING_PARAM_TEST_GYRO_LPF_PIXY_BMI},
  {.value = JIG_TEST_SETTING_PARAM_TEST_YAW_I},
  {.value = JIG_TEST_SETTING_PARAM_TEST_PITCH_I},
  {.value = JIG_TEST_SETTING_PARAM_TEST_GYRO_TRUST},
  {.value = JIG_TEST_SETTING_PARAM_TEST_IMU_RATE},
  {.value = JIG_TEST_SETTING_PARAM_TEST_HEARTBEAT_EMIT},
  {.value = JIG_TEST_SETTING_PARAM_TEST_STATUS_RATE},
  {.value = JIG_TEST_SETTING_PARAM_TEST_ENC_CNT_RATE},
  {.value = JIG_TEST_SETTING_PARAM_TEST_ORIEN_RATE},
  {.value = JIG_TEST_SETTING_PARAM_TEST_PIXY_ROTATION_TRAVEL_MIN_PIT},
  {.value = JIG_TEST_SETTING_PARAM_TEST_PIXY_ROTATION_TRAVEL_MAX_PIT},
  {.value = JIG_TEST_SETTING_PARAM_TEST_ROTATION_TRAVEL_MIN_ROLL},
  {.value = JIG_TEST_SETTING_PARAM_TEST_ROTATION_TRAVEL_MAX_ROLL},
  {.value = JIG_TEST_SETTING_PARAM_TEST_ROTATION_TRAVEL_MIN_PAN},
  {.value = JIG_TEST_SETTING_PARAM_TEST_ROTATION_TRAVEL_MAX_PAN},
  {.value = IG_TEST_SETTING_PARAM_TEST_DIR_MOTOR_ROLL},
};
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/
float gimbalParam_getValueTest(gimbalDevice_t device, uint8_t pos)
{

  if(device == GIMBAL_DEVICE_PIXY)
  {
    gimbal_param_setting[pos].value = gimbalParam_valueTestPixy[pos].value;
  }
  else if(device == GIMBAL_DEVICE_T3V3)
  {
    
  }
  else if(device == GIMBAL_DEVICE_S1V3)
  {
    
  }
  else if(device == GIMBAL_DEVICE_T7)
  {
    
  }
  else if(device == GIMBAL_DEVICE_AC30000)
  {
    
  }

  return gimbal_param_setting[pos].value;
}

char *gimbalParam_getIdTest(gimbalDevice_t device, uint8_t pos)
{
  return gimbal_param_setting[pos].param_id;
}
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

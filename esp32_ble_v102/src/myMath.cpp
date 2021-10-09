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
#include "Arduino.h"
#include "myMath.h"
/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/

/** @group MY_MATH_FUNCTION
    @{
*/#ifndef MY_MATH_FUNCTION
#define MY_MATH_FUNCTION

/** @brief float to hex
    @return
*/
void ConvertFloat2Hex(float value, uint8_t *hex_array)
{
//	Serial.println("[ConvertFloat2Hex] float_value :" + String(value));

	byte* f_byte = reinterpret_cast<byte*>(&value);
	memcpy(hex_array, f_byte, 4);

//	Serial.print("[ConvertFloat2Hex] hex_array : 0x");
//	Serial.print(hex_array[0], HEX);
//	Serial.print(hex_array[1], HEX);
//	Serial.print(hex_array[2], HEX);
//	Serial.println(hex_array[3], HEX);

}

/** @brief hex to float
    @return
*/
float ConvertHex2ToFloat(uint8_t *hex_array)
{
    union {
        char c[4];
        float f;
    } u;

    memcpy(u.c, hex_array, 4);

//	printf("%.2f\n", u.f);

	return u.f;
}

/** @brief char to int
    @return
*/
uint8_t ConvertChar2Int(char c)
{
	return (uint8_t)(c - '0');
}

/** @brief int to char
    @return
*/
char ConvertInt2Char(char c)
{
	return (char)(c + '0');
}

#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

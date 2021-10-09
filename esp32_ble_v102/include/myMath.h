/** 
  ******************************************************************************
  * @file    myMath.h
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

#ifndef __MYMATH_H
#define __MYMATH_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @brief float to hex
    @return
*/
void ConvertFloat2Hex(float value, uint8_t *hex_array);

/** @brief hex to float
    @return
*/
float ConvertHex2ToFloat(uint8_t *hex_array);

/** @brief char to int
    @return
*/
uint8_t ConvertChar2Int(char c);

#endif /* __MYMATH_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

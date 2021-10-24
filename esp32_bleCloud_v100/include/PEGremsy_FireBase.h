/** 
  ******************************************************************************
  * @file    PEGremsy_FireBase.h
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

#ifndef __PEGREMSY_FIREBASE_H
#define __PEGREMSY_FIREBASE_H

/* Includes ------------------------------------------------------------------*/

/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
class PEGremsy_FireBase_t
{
    private:
      /* data */

    public:
      bool fireBaseReady(void);
      void init(void);
      void process(void);

      PEGremsy_FireBase_t(/* args */);
      ~PEGremsy_FireBase_t();
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/


#endif /*  */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


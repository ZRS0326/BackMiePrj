/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdadc.h
  * @brief   This file contains all the function prototypes for
  *          the sdadc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDADC_H__
#define __SDADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SDADC_HandleTypeDef hsdadc1;

extern SDADC_HandleTypeDef hsdadc3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SDADC1_Init(void);
void MX_SDADC3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SDADC_H__ */


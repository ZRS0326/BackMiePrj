/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define BUFFERSIZE 200           					//可以接收的最大字符个数   
#define FRAMESIZE BUFFERSIZE/4           	//可以接收的最大字符个数   
extern uint8_t ReceiveBuff1[BUFFERSIZE]; 						//接收缓冲区
extern uint8_t base_addr1;													//基地址1
extern uint8_t recv_frame1[FRAMESIZE];						//串口帧
extern uint8_t recv_frame2[FRAMESIZE];						//串口帧

extern uint32_t SDADCBUFF1[4][5];
extern uint32_t SDADCBUFF2[4][3];
extern uint16_t data_frame[8];
extern uint16_t adj_frame[4];
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Uart_Dataframe(UART_HandleTypeDef *huart, uint8_t target,uint8_t size);	//处理串口接收数据帧入口
void get_sdadc_dataframe(void);		//获取一帧sdadc的数据 name: data_frame
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define S1_Pin GPIO_PIN_1
#define S1_GPIO_Port GPIOC
#define S2_Pin GPIO_PIN_2
#define S2_GPIO_Port GPIOC
#define S3_Pin GPIO_PIN_3
#define S3_GPIO_Port GPIOC
#define E1_Pin GPIO_PIN_0
#define E1_GPIO_Port GPIOA
#define E2_Pin GPIO_PIN_1
#define E2_GPIO_Port GPIOA
#define E3_Pin GPIO_PIN_2
#define E3_GPIO_Port GPIOA
#define SADJ_Pin GPIO_PIN_4
#define SADJ_GPIO_Port GPIOA
#define EADJ_Pin GPIO_PIN_5
#define EADJ_GPIO_Port GPIOA
#define WADJ_Pin GPIO_PIN_6
#define WADJ_GPIO_Port GPIOA
#define NADJ_Pin GPIO_PIN_7
#define NADJ_GPIO_Port GPIOA
#define SADA_Pin GPIO_PIN_0
#define SADA_GPIO_Port GPIOB
#define SADB_Pin GPIO_PIN_1
#define SADB_GPIO_Port GPIOB
#define EADA_Pin GPIO_PIN_2
#define EADA_GPIO_Port GPIOB
#define EADB_Pin GPIO_PIN_8
#define EADB_GPIO_Port GPIOE
#define NADA_Pin GPIO_PIN_9
#define NADA_GPIO_Port GPIOE
#define NADB_Pin GPIO_PIN_14
#define NADB_GPIO_Port GPIOB
#define WADB_Pin GPIO_PIN_15
#define WADB_GPIO_Port GPIOB
#define WADA_Pin GPIO_PIN_8
#define WADA_GPIO_Port GPIOD
#define N1_Pin GPIO_PIN_7
#define N1_GPIO_Port GPIOC
#define N2_Pin GPIO_PIN_8
#define N2_GPIO_Port GPIOC
#define N3_Pin GPIO_PIN_9
#define N3_GPIO_Port GPIOC
#define W1_Pin GPIO_PIN_9
#define W1_GPIO_Port GPIOA
#define W2_Pin GPIO_PIN_10
#define W2_GPIO_Port GPIOA
#define W3_Pin GPIO_PIN_11
#define W3_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

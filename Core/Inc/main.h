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

// 参数，上位机控制指令 debug/连续模式/离散模式
typedef struct {
	uint16_t flagMask;					//模式控制掩码、激光器状态 [0x00 0x[lidar][workmask] ]
	int16_t posLow;					//---/测量位置下限/测量位置下限(角度100.0->1000)
	int16_t posHigh;					//---/测量位置上限/测量位置上限(角度100.0->1000)
	int16_t posDiv;					//---/测量分辨率/---(角度100.0->1000)
	int16_t posSet;					//测量点/---/---(角度100.0->1000)
	uint16_t adjTime;						//ADC的增益控制周期
	uint16_t uartUploadTime;		//串口数据上传周期(采样率)
	uint16_t fashionTime;				//舵机单角度运行周期(a)
	uint16_t lidarTime;					//激光器启动延时(b)
}ControlParams;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define BUFFERSIZE 200           					//可以接收的最大字符个数   
#define FRAMESIZE 50           	//可以接收的最大字符个数   
#define DebugMode 0x0001					//Debug模式
#define CMode 0x0002					//连续模式
#define DMode 0x0004					//离散模式
#define Lidar1 0x10 				//激光器1
#define Lidar2 0x20					//激光器2
#define Lidar3 0x40					//激光器3
#define Release 0x00					//锁释放/完成标志
#define Lock 0x01					//锁执行/非完成标志
extern uint8_t ReceiveBuff1[BUFFERSIZE]; 						//接收缓冲区
extern uint8_t base_addr1;													//基地址1
extern uint8_t recv_frame1[FRAMESIZE];						//UART1串口帧
extern uint8_t recv_frame2[FRAMESIZE];						//UART2串口帧
extern uint16_t angle_read;                     // 舵机角度读取缓存

//extern uint32_t SDADCBUFF1[4][5];   // SDADC1 采集的数据DMA缓冲区
//extern uint32_t SDADCBUFF2[4][3];   // SDADC3 采集的数据DMA缓冲区
extern int16_t sdadc_frame[8];      // SDADC 一帧数据
extern uint16_t adc_frame[4];       // ADC 一帧数据

extern uint16_t data_arr;     //1c/s，设置串口上传频率
extern uint16_t adj_arr;       //10c/s，设置自动增益调节频率

extern uint8_t autoadj[8];						//自动增益挡位
extern uint8_t adjaddr[4];   //自动增益芯片地址
extern uint8_t readadj;

extern ControlParams uartCtrl;
extern uint8_t mutex_autoadj;	//自动增益调节过程中的锁
extern uint8_t flag_fashion;		//舵机执行指令成功
extern uint8_t data_frame_upload[40];
extern uint8_t mask_lidar[4];	//00 01 10 11 ....111 000当前只有两个激光器
extern uint8_t index_lidar;			//激光器开启状态（掩码索引）
extern uint16_t data_frame_master;//主帧序号
extern uint16_t data_frame_pos;//子帧序号/位置
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void uartDataFrame(UART_HandleTypeDef *huart, uint8_t target,uint8_t size);	//处理串口接收数据帧入口
void setCtrlParams(void);
void debugModeSet(void);
void cModeSet(void);
void dModeSet(void);
void dataUpload(void);
void modeInit(void);
void fashion_process_response(uint8_t length);
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

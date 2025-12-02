/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "sdadc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fashion_driver.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ControlParams uartCtrl = {0};
uint8_t mutex_autoadj = Release;	//自动增益调节过程中的锁
uint8_t flag_fashion = Release;		//舵机运行完成
uint8_t data_frame_upload[40] = {0};
uint8_t mask_lidar[4] = {0x00,0x01,0x02,0x03};	//00 01 10 11 ....111 000当前只有两个激光器
uint8_t index_lidar = 0;			//激光器开启状态
uint16_t data_frame_master = 0;//主帧序号
uint16_t data_frame_pos = 0;//子帧序号/位置

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	data_frame_upload[0] = 0xA9;
	data_frame_upload[1] = 0xB5;
	data_frame_upload[39] = 0x33;
	uartCtrl.flagMask = 0 | SPMode;					//模式控制掩码
	uartCtrl.posLow=0;
	uartCtrl.posHigh=1000;
	uartCtrl.posDiv=10;
	uartCtrl.posSet=500;
	uartCtrl.adjTime=10;
	uartCtrl.uartUploadTime=1;
	uartCtrl.fashionTime=1000;
	uartCtrl.lidarTime=10;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_SDADC1_Init();
  MX_SDADC3_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,ReceiveBuff1,BUFFERSIZE);
  HAL_UARTEx_ReceiveToIdle_IT(&huart2,recv_frame2,FRAMESIZE);
	
	HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_1);
	HAL_SDADC_PollForCalibEvent(&hsdadc1, HAL_MAX_DELAY);
	HAL_SDADC_CalibrationStart(&hsdadc3, SDADC_CALIBRATION_SEQ_1);
	HAL_SDADC_PollForCalibEvent(&hsdadc3, HAL_MAX_DELAY);
	
	HAL_SDADC_InjectedStart_DMA(&hsdadc1,(uint32_t*)sdadc_frame, 5);
	HAL_SDADC_InjectedStart_DMA(&hsdadc3,(uint32_t*)&sdadc_frame[5], 3);	
	
	//HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_frame,4);
	
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4); 	//发送串口数据

	for(int i=0;i<4;++i){
		uint8_t iicd[2];
		iicd[0] = 0x80;
		iicd[1] = autoadj[i+4];
		HAL_I2C_Master_Transmit_DMA(&hi2c1, adjaddr[i], iicd, 2);
		HAL_Delay(20);
	}	
	for(int i=0;i<4;++i){
		uint8_t iicd[2];
		iicd[0] = 0x00;
		iicd[1] = autoadj[i];
		HAL_I2C_Master_Transmit_DMA(&hi2c1, adjaddr[i], iicd, 2);
		HAL_Delay(20);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(500);
		debugModeSet();
		cModeSet();
		///dModeSet();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC1
                              |RCC_PERIPHCLK_SDADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.SdadcClockSelection = RCC_SDADCSYSCLK_DIV12;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PCLK2_DIV8;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1);
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG3);
}

/* USER CODE BEGIN 4 */
void setCtrlParams(void){
	if(recv_frame2[0]==0xA0&&recv_frame2[1]==0xB3){
		static uint8_t iicdata[2];
		static uint8_t cmd_id;
		static int16_t ang;
		static uint16_t tim;
		switch(recv_frame2[2]){
			case 0x01:	//读取指令
				HAL_UART_Transmit_IT(&huart2,(uint8_t *)&uartCtrl,sizeof(uartCtrl));
				break;
			case 0x02:	//批量写入指令
				memcpy(&uartCtrl,recv_frame2+3,sizeof(uartCtrl));
				break;
			case 0x03:  //设置串口发送频率
				uartCtrl.uartUploadTime = (recv_frame2[3]<<8)+recv_frame2[4];
				data_arr = 10000 / uartCtrl.uartUploadTime - 1;
				if(HAL_TIM_Base_GetState(&htim4)==HAL_TIM_STATE_BUSY){
					HAL_TIM_Base_Stop_IT(&htim4); 	//关闭自动上传
				}
				MX_TIM4_Init();
				break;
			case 0x04:  //设置自动增益频率
				uartCtrl.adjTime = (recv_frame2[3]<<8)+recv_frame2[4];
				adj_arr = 10000 / uartCtrl.adjTime - 1;
				HAL_TIM_Base_Stop_IT(&htim3);
				MX_TIM3_Init();
				HAL_TIM_Base_Start_IT(&htim3);
				break;
			case 0x05:	//设置舵机单运转时间
				uartCtrl.fashionTime = (recv_frame2[3]<<8)+recv_frame2[4];
				break;
			case 0x06:	//设置舵机运转位置参数
				uartCtrl.posLow = (recv_frame2[3]<<8)+recv_frame2[4];
				uartCtrl.posHigh = (recv_frame2[5]<<8)+recv_frame2[6];
				uartCtrl.posDiv = (recv_frame2[7]<<8)+recv_frame2[8];
				uartCtrl.posSet = (recv_frame2[9]<<8)+recv_frame2[10];
				break;
			case 0x07:	//设置工作模式
				uartCtrl.flagMask = (recv_frame2[3]<<8)+recv_frame2[4];
				modeInit();
				break;			
			case 0x08:	//设置激光器开启延时
				uartCtrl.lidarTime = (recv_frame2[3]<<8)+recv_frame2[4];
				break;
			case 0x11:	//调试IIC读命令
				HAL_I2C_Master_Receive_DMA(&hi2c1,adjaddr[recv_frame2[3]],&readadj,1);
				break;			
			case 0x12:	//调试IIC写命令
				iicdata[0] = recv_frame2[4];
				iicdata[1] = recv_frame2[5];
				if(recv_frame2[4]==0x00){
					autoadj[recv_frame2[3]] = recv_frame2[5];
				}else{
					autoadj[recv_frame2[3]+4] = recv_frame2[5];
				}
				HAL_I2C_Master_Transmit_DMA(&hi2c1,adjaddr[recv_frame2[3]],iicdata,2);
				break;
			case 0x13:	//调试舵机是否在线
				fashion_send_ping(recv_frame2[3]);
				break;
			case 0x14:	//调试设置舵机角度
				ang = (recv_frame2[4]<<8)+recv_frame2[5];
				tim = (recv_frame2[6]<<8)+recv_frame2[7];
				fashion_send_single_angle(recv_frame2[3],ang,tim);
				break;
			case 0x15:	//调试读取舵机角度
				fashion_read_servo_angle(recv_frame2[3]);
				break;
			case 0x16:	//调试读取数据
				cmd_id = recv_frame2[4];
				fashion_read_data(recv_frame2[3],cmd_id);
				break;
			case 0x17:	//调试监控数据
				cmd_id = recv_frame2[4];
				fashion_monitor_data(recv_frame2[3],cmd_id);
				break;			
			case 0x18:	//同步四个舵机到指定角度
				ang = (recv_frame2[3]<<8)+recv_frame2[4];
				tim = (recv_frame2[5]<<8)+recv_frame2[6];
				fashion_sync_set_angle(ang,tim);
				break;
			case 0x19:
				fashion_sync_data_monitor();
				break;
			case 0x1A:
				HAL_GPIO_WritePin(GPIOA,E1_Pin | W1_Pin,1);
				//HAL_GPIO_WritePin(GPIOC,S1_Pin | N1_Pin,uartCtrl.flagMask & Lidar1);			
				//HAL_GPIO_WritePin(GPIOA,E2_Pin | W2_Pin,uartCtrl.flagMask & Lidar2);
				HAL_GPIO_WritePin(GPIOC,S2_Pin | N2_Pin,1);			
				//HAL_GPIO_WritePin(GPIOA,E3_Pin | W3_Pin,uartCtrl.flagMask & Lidar3);
				//HAL_GPIO_WritePin(GPIOC,S3_Pin | N3_Pin,uartCtrl.flagMask & Lidar3);
				break;
			case 0x21:	//带参数启动debug
				//需要参数posSet、fashiontime、
				uartCtrl.flagMask = (recv_frame2[3]<<8)+recv_frame2[4];
				uartCtrl.posSet = (recv_frame2[5]<<8)+recv_frame2[6];
				uartCtrl.fashionTime = (recv_frame2[7]<<8)+recv_frame2[8];
				modeInit();
				break;			
			case 0x22:	//带参数启动cMode
				//需要参数posLow、posHigh、fashiontime、lidartime
				uartCtrl.flagMask = (recv_frame2[3]<<8)+recv_frame2[4];
				uartCtrl.posLow = (recv_frame2[5]<<8)+recv_frame2[6];
				uartCtrl.posHigh = (recv_frame2[7]<<8)+recv_frame2[8];
				uartCtrl.fashionTime = (recv_frame2[9]<<8)+recv_frame2[10];
				uartCtrl.lidarTime = (recv_frame2[11]<<8)+recv_frame2[12];
				modeInit();
				break;			
			case 0x23:	//带参数启动dMode
				
				break;
      //后续添加其他指令
			default:
				break;
		}
	}
	memset(recv_frame2,0,FRAMESIZE);
}

void dataUpload(void){
		if(uartCtrl.flagMask&SPMode){
			char buffer[50];
			int V0 = (sdadc_frame[0] + 32767) * 3300.0 / 65535.0;
			int V1 = (sdadc_frame[4] + 32767) * 3300.0 / 65535.0;
			float A_origin = 1000 * V0 / ((256 - autoadj[0]) * 3.92); //nA
			sprintf(buffer,"%.4f,%d,%d,%d,%d,%d\r\n", A_origin,V0,V1,sdadc_frame[0], sdadc_frame[4], autoadj[0]);
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)buffer, strlen(buffer));
			return;
		}
		//ch1[1,0,4]
		//ch4[]
			// 数据帧逻辑
		memset(&data_frame_upload[2],0,37); //清空数据位
		memcpy(&data_frame_upload[2],sdadc_frame,sizeof(sdadc_frame));//sdadc 8*2 = 16Bytes
		memcpy(&data_frame_upload[18],autoadj,sizeof(autoadj));//autoadj 8*1 = 8Bytes
		HAL_UART_Transmit_IT(&huart2,data_frame_upload,26);
		//0-1 		帧头0xA9 0xB5						1*2
		//2-9 		4ch adc 								4*2
		//10-25 	8ch sdadc 							8*2
		//26-33		8ch autoadj 						8*1
		//34-37		2ch frame(master/slave)	2*2
		//38			1ch lidar state					1*1
		//39			帧尾0x33								1*1
//		memcpy(&data_frame_upload[2],adc_frame,sizeof(adc_frame));	//adc 4*2 = 8Bytes
//		memcpy(&data_frame_upload[10],sdadc_frame,sizeof(sdadc_frame));//sdadc 8*2 = 16Bytes
//		memcpy(&data_frame_upload[26],autoadj,sizeof(autoadj));//autoadj 8*1 = 8Bytes
//		memcpy(&data_frame_upload[34],&data_frame_master,sizeof(data_frame_master));//2Bytes
//		memcpy(&data_frame_upload[36],&data_frame_pos,sizeof(data_frame_pos));//2Bytes
//		data_frame_upload[38]=index_lidar;//1Bytes
//		// 增益、帧序号
//		HAL_UART_Transmit_IT(&huart2,data_frame_upload,sizeof(data_frame_upload));
}

void debugModeSet(){
	if(uartCtrl.flagMask & DebugMode){
		fashion_sync_set_angle(uartCtrl.posSet, uartCtrl.fashionTime);
		HAL_Delay(uartCtrl.fashionTime + 20);
		while(uartCtrl.flagMask & DebugMode) {
			// 仅调整舵机位置，和激光器工作状态，工作时序通过定时器完成
			if(uartCtrl.flagMask & CHE){
				HAL_GPIO_WritePin(GPIOA,E1_Pin,uartCtrl.flagMask & Lidar1);
				HAL_GPIO_WritePin(GPIOA,E2_Pin,uartCtrl.flagMask & Lidar2);
				HAL_GPIO_WritePin(GPIOA,E3_Pin,uartCtrl.flagMask & Lidar3);
			}else{
				HAL_GPIO_WritePin(GPIOA,E1_Pin | E2_Pin | E3_Pin,0);
			}			
			if(uartCtrl.flagMask & CHW){
				HAL_GPIO_WritePin(GPIOA,W1_Pin,uartCtrl.flagMask & Lidar1);
				HAL_GPIO_WritePin(GPIOA,W2_Pin,uartCtrl.flagMask & Lidar2);
				HAL_GPIO_WritePin(GPIOA,W3_Pin,uartCtrl.flagMask & Lidar3);
			}else{
				HAL_GPIO_WritePin(GPIOA,W1_Pin | W2_Pin | W3_Pin,0);
			}			
			if(uartCtrl.flagMask & CHS){
				HAL_GPIO_WritePin(GPIOC,S1_Pin,uartCtrl.flagMask & Lidar1);
				HAL_GPIO_WritePin(GPIOC,S2_Pin,uartCtrl.flagMask & Lidar2);
				HAL_GPIO_WritePin(GPIOC,S3_Pin,uartCtrl.flagMask & Lidar3);
			}else{
				HAL_GPIO_WritePin(GPIOC,S1_Pin | S2_Pin | S3_Pin,0);
			}			
			if(uartCtrl.flagMask & CHN){
				HAL_GPIO_WritePin(GPIOC,N1_Pin,uartCtrl.flagMask & Lidar1);
				HAL_GPIO_WritePin(GPIOC,N2_Pin,uartCtrl.flagMask & Lidar2);
				HAL_GPIO_WritePin(GPIOC,N3_Pin,uartCtrl.flagMask & Lidar3);
			}else{
				HAL_GPIO_WritePin(GPIOC,N1_Pin | N2_Pin | N3_Pin,0);
			}
//			HAL_GPIO_WritePin(GPIOA,E1_Pin | W1_Pin,uartCtrl.flagMask & Lidar1);
//			HAL_GPIO_WritePin(GPIOC,S1_Pin | N1_Pin,uartCtrl.flagMask & Lidar1);			
//			HAL_GPIO_WritePin(GPIOA,E2_Pin | W2_Pin,uartCtrl.flagMask & Lidar2);
//			HAL_GPIO_WritePin(GPIOC,S2_Pin | N2_Pin,uartCtrl.flagMask & Lidar2);			
//			HAL_GPIO_WritePin(GPIOA,E3_Pin | W3_Pin,uartCtrl.flagMask & Lidar3);
//			HAL_GPIO_WritePin(GPIOC,S3_Pin | N3_Pin,uartCtrl.flagMask & Lidar3);
			if(flag_fashion == Lock){
				fashion_sync_set_angle(uartCtrl.posSet, uartCtrl.fashionTime);
			}
			if(HAL_TIM_Base_GetState(&htim4)==HAL_TIM_STATE_READY){
				HAL_TIM_Base_Start_IT(&htim4); 	//发送串口数据
			}
			HAL_Delay(uartCtrl.fashionTime);
		}
	}
};
void cModeSet(){
	if(uartCtrl.flagMask & CMode){
		// 初始化
		fashion_sync_set_angle(uartCtrl.posLow, uartCtrl.fashionTime);//启动舵机到初始位置posLow
		HAL_Delay(uartCtrl.fashionTime + uartCtrl.lidarTime);
		int16_t target;
		
    while(uartCtrl.flagMask & CMode) {
			// 设置激光器状态
			HAL_GPIO_WritePin(GPIOA,E1_Pin | W1_Pin,mask_lidar[index_lidar] & 0x01);
			HAL_GPIO_WritePin(GPIOC,S1_Pin | N1_Pin,mask_lidar[index_lidar] & 0x01);
			HAL_GPIO_WritePin(GPIOA,E2_Pin | W2_Pin,mask_lidar[index_lidar] & 0x02);
			HAL_GPIO_WritePin(GPIOC,S2_Pin | N2_Pin,mask_lidar[index_lidar] & 0x02);
			fashion_read_servo_angle(0);
			HAL_Delay(uartCtrl.lidarTime);	//等待激光器启动

			// 切换舵机目标位置
			target = (abs(angle_read-uartCtrl.posLow)<=10) ? uartCtrl.posHigh : uartCtrl.posLow;
			
			
			// 启动舵机并开始发送数据
			fashion_sync_set_angle(target, uartCtrl.fashionTime);
			HAL_Delay(5);
			data_frame_pos = 0;		//子帧计数清零
			HAL_TIM_Base_Start_IT(&htim4); // 启动定时器发送数据
			
			
			// 等待舵机完成
			HAL_Delay(uartCtrl.fashionTime);
			
			// 关闭定时器
			HAL_TIM_Base_Stop_IT(&htim4);
			
			
			// 更新激光器索引
			++index_lidar;
			if(index_lidar >= 4) { // 完成一轮
				index_lidar = 0;
				++data_frame_master;
			}
    }
	}
};

void dModeSet(){
		if(uartCtrl.flagMask & DMode){
		data_frame_pos = uartCtrl.posLow;
		index_lidar = 0;
		uint8_t direction = 1; // 1表示正向扫描，0表示反向扫描
		uint16_t angle_read[1] = {0}; // 用于存储读取的舵机角度
		uint8_t position_reached = 0;
		uint8_t retry_count = 0;
		const uint8_t MAX_RETRY = 3; // 最大重试次数
    
		while(uartCtrl.flagMask & DMode) {
			// 初始化位置标志和重试计数
			position_reached = 0;
			retry_count = 0;
			
			// 检查并修正舵机位置范围
			if(data_frame_pos < uartCtrl.posLow) {
				data_frame_pos = uartCtrl.posLow;
				direction = 1;
			} else if(data_frame_pos > uartCtrl.posHigh) {
				data_frame_pos = uartCtrl.posHigh;
				direction = 0;
			}
			
			// 发送舵机到目标位置
			fashion_send_single_angle(0, data_frame_pos, uartCtrl.fashionTime);
			
			// 等待舵机到达目标位置（带重试机制）
			while(!position_reached && retry_count < MAX_RETRY) {
				HAL_Delay(uartCtrl.fashionTime); // 等待舵机动作
				HAL_Delay(uartCtrl.lidarTime); // 等待激光器启动时间
				
				// 读取实际舵机角度
				fashion_read_servo_angle(0);
				
				// 检查舵机是否到达目标位置
				if(angle_read[0] == data_frame_pos) {
					position_reached = 1;
				} else {
					retry_count++;
					// 重新发送舵机到目标位置
					fashion_send_single_angle(0, data_frame_pos, uartCtrl.fashionTime);
				}
			}
			
			// 只有当舵机到达目标位置时才继续
			if(position_reached) {
				// 依次开启激光器
				HAL_GPIO_WritePin(GPIOA, E1_Pin | W1_Pin, mask_lidar[index_lidar] & 0x01);
				HAL_GPIO_WritePin(GPIOC, S1_Pin | N1_Pin, mask_lidar[index_lidar] & 0x01);
				HAL_GPIO_WritePin(GPIOA, E2_Pin | W2_Pin, mask_lidar[index_lidar] & 0x02);
				HAL_GPIO_WritePin(GPIOC, S2_Pin | N2_Pin, mask_lidar[index_lidar] & 0x02);
				
				// 等待激光器稳定
				HAL_Delay(uartCtrl.lidarTime);
				
				// 等待自动增益调节完成
				while(mutex_autoadj != 0) {
					HAL_Delay(1);
				}
				
				// 上传当前数据
				dataUpload();
				
				// 更新激光器索引
				index_lidar++;
				if(index_lidar > 3) {
					index_lidar = 0;
					
					// 实现来回扫描逻辑
					if(direction) {
						// 正向扫描
						data_frame_pos += uartCtrl.posDiv;
						// 如果到达上边界，改变方向
						if(data_frame_pos > uartCtrl.posHigh) {
							direction = 0;
							data_frame_pos = uartCtrl.posHigh - uartCtrl.posDiv;
							// 确保不小于最小值
							if(data_frame_pos < uartCtrl.posLow) {
								data_frame_pos = uartCtrl.posLow;
							}
						}
					} else {
						// 反向扫描
						data_frame_pos -= uartCtrl.posDiv;
						// 如果到达下边界，改变方向
						if(data_frame_pos < uartCtrl.posLow) {
							direction = 1;
							data_frame_pos = uartCtrl.posLow + uartCtrl.posDiv;
							// 确保不大于最大值
							if(data_frame_pos > uartCtrl.posHigh) {
								data_frame_pos = uartCtrl.posHigh;
							}
						}
					}
					
					// 主帧序号自增
					data_frame_master++;
				}
			}
			
			// 关闭所有激光器
			HAL_GPIO_WritePin(GPIOA, E1_Pin | W1_Pin | E2_Pin | W2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, S1_Pin | N1_Pin | S2_Pin | N2_Pin, GPIO_PIN_RESET);
    }
	}
};

void modeInit(){
	if(HAL_TIM_Base_GetState(&htim4)==HAL_TIM_STATE_BUSY){
		HAL_TIM_Base_Stop_IT(&htim4); 	//关闭自动上传
	}
	if(uartCtrl.flagMask==0 || uartCtrl.flagMask==0x0008){
		HAL_TIM_Base_Start_IT(&htim4);
	}
	flag_fashion = Lock;
	index_lidar = 0;
	data_frame_master = 0;
	data_frame_pos = 0;
}
float V_ad_1[4] = {0};
float A_origin[4] = {0};
float A_thresh = 10;
float V_ref = 3300;
float V_max = 200;
float V_min = 50;
float V_target = 100;
float R_target = 0;
uint8_t autoadj_target = 0;
uint8_t Adj_flag = 0;
uint16_t Arr_label[4] = {0, 1, 2 ,5};
void autoGainAdj(void){
//	uint8_t i = 0;
//	for (i=0;i<4;i++){
//		// 1. 由SDADC的值转换为第一级放大电压V1
//		V_ad_1[i] = (sdadc_frame[Arr_label[i]] + 32767) * V_ref / 65535; //i, Arr_label[i]
//		A_origin[i] = 1000 * V_ad_1[i] / ((256 - autoadj[i]) * 3.92); //i,i,2i+1, 电流单位是nA

//		// 2. 判断是否需要调节
//		if (V_ad_1[i] > V_max) {
//		    Adj_flag = 1;      // 过压：减小增益
//		}
//		else if (V_ad_1[i] <= V_min && A_origin[i] >= A_thresh) {
//		    Adj_flag = 1;      // 欠压且电流仍大：增大增益
//		}

//		// 3. 若需调节则计算新阻值和目标档位
//		if (Adj_flag) {
//		    R_target = 1e6f * V_target / A_origin[i];       // Ω
//		    autoadj_target = (uint8_t)(256 - (R_target / 3920.0f));

//		    // 限幅处理
//		    if (autoadj_target > 255) {
//		    	autoadj_target = 255;
//		    }
//		    if (autoadj_target < 1) {
//		    	autoadj_target = 1;   // 防止0档过小
//		    }

//		    // 更新增益
//		    autoadj[i] = autoadj_target;
//				uint8_t iicdata[2] = {0};
//				iicdata[1] = autoadj_target;
//				HAL_I2C_Master_Transmit_DMA(&hi2c1, adjaddr[i], iicdata, 2);
//		}

//		Adj_flag = 0;
//	}
	uint8_t i = 0;
		// 1. 由SDADC的值转换为第一级放大电压V1
		V_ad_1[i] = (sdadc_frame[Arr_label[i]] + 32767) * V_ref / 65535; //i, Arr_label[i]
		A_origin[i] = 1000 * V_ad_1[i] / ((256 - autoadj[i]) * 3.92); //i,i,2i+1, 电流单位是nA

		// 2. 判断是否需要调节
		if (V_ad_1[i] > V_max) {
		    Adj_flag = 1;      // 过压：减小增益
		}
		else if (V_ad_1[i] <= V_min && A_origin[i] >= A_thresh) {
		    Adj_flag = 1;      // 欠压且电流仍大：增大增益
		}

		// 3. 若需调节则计算新阻值和目标档位
		if (Adj_flag) {
		    R_target = 1e6f * V_target / A_origin[i];       // Ω
		    autoadj_target = (uint8_t)(256 - (R_target / 3920.0f));

		    // 限幅处理
		    if (autoadj_target > 255) {
		    	autoadj_target = 255;
		    }
		    if (autoadj_target < 1) {
		    	autoadj_target = 1;   // 防止0档过小
		    }

		    // 更新增益
		    autoadj[i] = autoadj_target;
				uint8_t iicdata[2] = {0};
				iicdata[1] = autoadj_target;
				HAL_I2C_Master_Transmit_DMA(&hi2c1, adjaddr[i], iicdata, 2);
		}

		Adj_flag = 0;

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim3) {
		autoGainAdj();
	}
	else if(htim == &htim4){
		++data_frame_pos;
		dataUpload();
	}
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
	--mutex_autoadj;	//调节完成后释放锁
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
	HAL_UART_Transmit_IT(&huart2,&readadj,sizeof(readadj));
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

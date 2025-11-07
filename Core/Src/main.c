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
uint8_t mutex_autoadj = 1;	//自动增益调节过程中的锁
uint8_t flag_fashion = 0;		//舵机运行完成
uint8_t data_frame_upload[40] = {0};
uint8_t mask_lidar[4] = {0x00,0x01,0x02,0x03};	//00 01 10 11 ....111 000当前只有两个激光器
uint8_t flag_lidar = 0;			//激光器开启状态
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
	HAL_SDADC_InjectedStart_DMA(&hsdadc3,(uint32_t*)sdadc_frame, 3);	
	
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_frame,4);
	
	HAL_TIM_Base_Start_IT(&htim3);
	//HAL_TIM_Base_Start_IT(&htim4); 	//发送串口数据
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(10);
		debugModefun();
		cModefun();
		dModefun();
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
void set_ctrl_params(void){
	if(recv_frame2[0]==0xA0&&recv_frame2[1]==0xB3){
		switch(recv_frame2[2]){
			case 0x01:	//读取指令
				HAL_UART_Transmit_IT(&huart2,(uint8_t *)&uartCtrl,sizeof(uartCtrl));
				break;
			case 0x02:	//批量写入指令
				memcpy(&uartCtrl,recv_frame2+3,sizeof(uartCtrl));
				break;
			case 0x03:  //设置串口发送频率
				data_arr = 10000 / ((recv_frame2[9]<<8)+recv_frame2[10]) - 1;
				HAL_TIM_Base_Stop_IT(&htim4);
				MX_TIM4_Init();
				break;
			case 0x04:  //设置自动增益频率
				adj_arr = 10000 / ((recv_frame2[11]<<8)+recv_frame2[12]) - 1;
				HAL_TIM_Base_Stop_IT(&htim3);
				MX_TIM3_Init();
				HAL_TIM_Base_Start_IT(&htim3);
				break;
      //后续添加其他指令
			default:
				break;
		}
	}
	memset(recv_frame2,0,FRAMESIZE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim3) {
			// 自动增益调节逻辑
	}
	else if(htim == &htim4){
		data_upload();
	}
}


void data_upload(void){
			// 数据帧逻辑
		memset(&data_frame_upload[2],0,37); //清空数据位
		//0-1 		帧头0xA9 0xB5						1*2
		//2-9 		4ch adc 								4*2
		//10-25 	8ch sdadc 							8*2
		//26-33		8ch autoadj 						8*1
		//34-37		2ch frame(master/slave)	2*2
		//38			1ch lidar state					1*1
		//39			帧尾0x33								1*1
		memcpy(&data_frame_upload[2],adc_frame,sizeof(adc_frame));	//adc 4*2 = 8Bytes
		memcpy(&data_frame_upload[10],sdadc_frame,sizeof(sdadc_frame));//sdadc 8*2 = 16Bytes
		memcpy(&data_frame_upload[26],autoadj,sizeof(autoadj));//autoadj 8*1 = 8Bytes
		memcpy(&data_frame_upload[34],&data_frame_master,sizeof(data_frame_master));//2Bytes
		memcpy(&data_frame_upload[36],&data_frame_pos,sizeof(data_frame_pos));//2Bytes
		data_frame_upload[38]=flag_lidar;//1Bytes
		// 增益、帧序号
		HAL_UART_Transmit_IT(&huart2,data_frame_upload,sizeof(data_frame_upload));
}

void debugModefun(){
	HAL_TIM_Base_Start_IT(&htim4); 	//发送串口数据
	while(uartCtrl.flagMask & DebugMode) {
		// 仅调整舵机位置，和激光器工作状态，工作时序通过定时器完成
		HAL_GPIO_WritePin(GPIOA,E1_Pin | W1_Pin,uartCtrl.flagMask & Lidar1);
		HAL_GPIO_WritePin(GPIOC,S1_Pin | N1_Pin,uartCtrl.flagMask & Lidar1);			
		HAL_GPIO_WritePin(GPIOA,E2_Pin | W2_Pin,uartCtrl.flagMask & Lidar2);
		HAL_GPIO_WritePin(GPIOC,S2_Pin | N2_Pin,uartCtrl.flagMask & Lidar2);			
		HAL_GPIO_WritePin(GPIOA,E3_Pin | W3_Pin,uartCtrl.flagMask & Lidar3);
		HAL_GPIO_WritePin(GPIOC,S3_Pin | N3_Pin,uartCtrl.flagMask & Lidar3);	
		if(!flag_fashion){
			fashion_send_single_angle(1, uartCtrl.posDiv, uartCtrl.fashionTime);
		}
		HAL_Delay(uartCtrl.lidarTime);
	}
};
void cModefun(){
    while(uartCtrl.flagMask & CMode) {
			HAL_GPIO_WritePin(GPIOA,E1_Pin | W1_Pin,mask_lidar[flag_lidar] & 0x01);
			HAL_GPIO_WritePin(GPIOC,S1_Pin | N1_Pin,mask_lidar[flag_lidar] & 0x01);			
			HAL_GPIO_WritePin(GPIOA,E2_Pin | W2_Pin,mask_lidar[flag_lidar] & 0x02);
			HAL_GPIO_WritePin(GPIOC,S2_Pin | N2_Pin,mask_lidar[flag_lidar] & 0x02);	
			HAL_Delay(uartCtrl.lidarTime);	//启动激光器后等待激光器启动
			fashion_send_single_angle(1, uartCtrl.posDiv, uartCtrl.fashionTime);//启动舵机
			HAL_TIM_Base_Start_IT(&htim4); 	//发送串口数据
			HAL_Delay(uartCtrl.fashionTime);	//等待舵机完成//串口中回传信号时处理帧序号和flag_lidar，开启下一子帧测量
    }
};
void dModefun(){
		data_frame_pos = uartCtrl.posLow;
		flag_lidar = 0;
    while(uartCtrl.flagMask & DMode) {
			fashion_send_single_angle(1, data_frame_pos, uartCtrl.fashionTime);//启动舵机
			HAL_GPIO_WritePin(GPIOA,E1_Pin | W1_Pin,mask_lidar[flag_lidar] & 0x01);
			HAL_GPIO_WritePin(GPIOC,S1_Pin | N1_Pin,mask_lidar[flag_lidar] & 0x01);			
			HAL_GPIO_WritePin(GPIOA,E2_Pin | W2_Pin,mask_lidar[flag_lidar] & 0x02);
			HAL_GPIO_WritePin(GPIOC,S2_Pin | N2_Pin,mask_lidar[flag_lidar] & 0x02);	
			HAL_Delay(uartCtrl.lidarTime);	//启动激光器后等待激光器启动
			while(mutex_autoadj==0){HAL_Delay(1);}
			// 发送当前帧数据
			data_upload();
			++flag_lidar;
			if(flag_lidar>3) {
				flag_lidar = 0;
				data_frame_pos+=uartCtrl.posDiv;
				if(data_frame_pos>uartCtrl.posHigh) data_frame_pos = uartCtrl.posLow;
			}
    }
};
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

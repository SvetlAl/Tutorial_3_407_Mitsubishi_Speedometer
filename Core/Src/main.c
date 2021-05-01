/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	  CAN_RxHeaderTypeDef msgHeaderRX;
  uint32_t msgIdRX = 0;
  uint8_t msgDataRX[8];
  

	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX) == HAL_OK){

					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
		}
}

*/



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

void Init_IWDG(uint16_t tw){
// ??? IWDG_PR=7 Tmin=6,4?? RLR=T??*40/256
	IWDG->KR=0x5555; // ???? ??? ??????? ? ???????
	IWDG->PR=7; // ?????????? IWDG_PR
	IWDG->RLR=tw*40/256; // ????????? ??????? ????????????
	IWDG->KR=0xAAAA; // ????????????
	IWDG->KR=0xCCCC; // ???? ???????
	}

	
// ??????? ???????????? ??????????? ??????? IWDG
void IWDG_reset(void){
	IWDG->KR=0xAAAA; // ????????????
	}

	
	
	CAN_TxHeaderTypeDef TxHeaderINCAN1;
	CAN_RxHeaderTypeDef RxHeaderINCAN1;
	uint8_t TxDataINCAN1[8] = {0,};
	uint8_t RxDataINCAN1[8] = {0,};
	uint32_t TxMailboxINCAN1 = 0;
	
	CAN_TxHeaderTypeDef TxHeaderOUTCAN2;
	CAN_RxHeaderTypeDef RxHeaderOUTCAN2;
	uint8_t TxDataOUTCAN2[8] = {0,};
	uint8_t RxDataOUTCAN2[8] = {0,};
	uint32_t TxMailboxOUTCAN2 = 0;
	uint32_t fr = 0x214;
	uint32_t fv = 0x215;
	
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN PV */
	// From Dashboard to CAN
		void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){


					  HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeaderOUTCAN2, RxDataOUTCAN2);
					if ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0))
  {
	//	if((RxHeaderOUTCAN2.StdId!=fr) && (RxHeaderOUTCAN2.StdId!=fv)){
    CAN_TxHeaderTypeDef msgHeader;
    uint8_t msgData[8];
    msgHeader.StdId = RxHeaderOUTCAN2.StdId;
    msgHeader.DLC = 8;
    msgHeader.TransmitGlobalTime = DISABLE;
    msgHeader.RTR = CAN_RTR_DATA;
    msgHeader.IDE = CAN_ID_STD;
    
//    uint32_t mailBoxNum = 0;
    
    for (uint8_t i = 0; i < 8; i++)
    {
      msgData[i] = RxDataOUTCAN2[i];
    }
    
    HAL_CAN_AddTxMessage(&hcan1, &msgHeader, msgData, &TxMailboxINCAN1);
		IWDG_reset();
//	}
	  }
	}
	
	
	
	
	
	typedef struct speed{	
	uint8_t data[2];			// Data 0 - first 1- second
} speed; //creating new type
	
	
	
	uint32_t convertSpeedToUint(speed sp){
		uint32_t convertedSpeed;
		uint32_t A = (uint32_t)sp.data[1]*100/128;
		uint32_t B = (uint32_t)sp.data[0]*200;
		convertedSpeed = (B+A);
		return convertedSpeed;
	}
	
	
	
	speed convertUint32toSpeed(uint32_t sp){
		uint32_t counter=0;
		while(sp>200){
			sp=sp-200;
			counter++;
		}
		speed result;
		result.data[0]=(uint8_t)counter;
		result.data[1]=(uint8_t)(sp*128/100);
		return result;
	}
		
	
	speed calculate_NewSpeed(speed value, uint32_t mod)
	{
		uint32_t sp = convertSpeedToUint(value); 
		sp=sp*mod/100;

		speed result = convertUint32toSpeed(sp);
		return result;
	}
		
	

	// from CAN to dashboard
	void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
	{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		


					  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &RxHeaderINCAN1, RxDataINCAN1);
					if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 0)
  {
		if((RxHeaderINCAN1.StdId!=fr) && (RxHeaderINCAN1.StdId!=fv)){
    CAN_TxHeaderTypeDef msgHeader;
    uint8_t msgData[8];
    msgHeader.StdId = RxHeaderINCAN1.StdId;
    msgHeader.DLC = 8;
    msgHeader.TransmitGlobalTime = DISABLE;
    msgHeader.RTR = CAN_RTR_DATA;
    msgHeader.IDE = CAN_ID_STD;
    
//    uint32_t mailBoxNum = 0;
    
    for (uint8_t i = 0; i < 8; i++)
    {
      msgData[i] = RxDataINCAN1[i];
    }
    HAL_CAN_AddTxMessage(&hcan2, &msgHeader, msgData, &TxMailboxOUTCAN2);
	
	  }
		else if((RxHeaderINCAN1.StdId==fr) || (RxHeaderINCAN1.StdId==fv)){
    CAN_TxHeaderTypeDef msgHeader;
    uint8_t msgData[8];
    msgHeader.StdId = RxHeaderINCAN1.StdId;
    msgHeader.DLC = 8;
    msgHeader.TransmitGlobalTime = DISABLE;
    msgHeader.RTR = CAN_RTR_DATA;
    msgHeader.IDE = CAN_ID_STD;
    
//    uint32_t mailBoxNum = 0;
    
    for (uint8_t i = 2; i < 8; i++)
    {
      msgData[i] = RxDataINCAN1[i];
    }
		
		speed curS;
		curS.data[0]=RxDataINCAN1[0];
		curS.data[1]=RxDataINCAN1[1];

		
		speed newS;
		newS = calculate_NewSpeed(curS, 110);
		
		
		msgData[0]= newS.data[0];
		msgData[1]= newS.data[1];

		
    HAL_CAN_AddTxMessage(&hcan2, &msgHeader, msgData, &TxMailboxOUTCAN2);
				IWDG_reset();
	  }
	}
	}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
	
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
	  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
		
		
		
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
				Init_IWDG(1000);
  while (1)
  {

		
	/*	HAL_Delay(333);
	
	  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 0)
  {
    CAN_TxHeaderTypeDef msgHeaderTX;
    uint8_t msgDataTX[8];
    msgHeaderTX.StdId = 0x333;
    msgHeaderTX.DLC = 8;
    msgHeaderTX.TransmitGlobalTime = DISABLE;
    msgHeaderTX.RTR = CAN_RTR_DATA;
    msgHeaderTX.IDE = CAN_ID_STD;
    
    uint32_t mailBoxNum = 0;
    
    for (uint8_t i = 0; i < 8; i++)
    {
      msgDataTX[i] = i;
    }
    
    HAL_CAN_AddTxMessage(&hcan2, &msgHeaderTX, msgDataTX, &mailBoxNum);
  }
	*/

//  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &msgHeaderRX, msgDataRX);
	

		
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
	
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */



  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; 
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 14 ;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; 
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
 // __disable_irq();
//  while (1)
//  {
//  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

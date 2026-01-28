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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "OLED_IIC_Config.h"
#include "OLED_Function.h"
#include "OLED_Front.h"
#include "string.h"
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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//char buff[] = "USB CDC TESTING Hello World!\r\n";

//CANÈ«ï¿½Ö±ï¿½ï¿½ï¿½
CAN_TxHeaderTypeDef TxHeaderCAN;
CAN_RxHeaderTypeDef RxHeaderCAN;
uint8_t RxDataCAN[8];
uint8_t TxDataCAN[8];

uint8_t CAN_Rx_Flag=0;//CANï¿½ï¿½ï¿½Õ±ï¿½Ö¾
float Set_Angle_And_Omega[2] = {0.0f,20.0f};// DMÉÏÎ»»úÉèÖÃµÄ¶àÈ¦½Ç¶È·¶Î§Îª-13rad - 13rad
float Control_Angle = 0.0f; //¿ØÖÆ½Ç¶È 0 - 90¶È 
//DMÊ¹ÄÜ°ü
uint8_t DM_Motor_CAN_Message_Enter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
/**
 * @brief DMµç»ú»Ø´«µÄÊý¾Ý°ü
 *
 */
typedef struct 
{
  float Now_Angle;//¶àÈ¦½Ç¶È
}Struct_DM_Motor_Rx_Data;
Struct_DM_Motor_Rx_Data DM_Motor_Rx_Data;
//adcÍ¨µÀ0µÄ²ÉÑùÖµ
uint32_t ADC_Values;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void filter_init(void);
void CAN_Test(void);
void CAN1_Send_Test(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF); //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê½ï¿½ï¿½Ó¡,ï¿½ï¿½ï¿½ï¿½1
  return ch;
}

/*
void USB_Disconnected(void) {
    __HAL_RCC_USB_FORCE_RESET();
    HAL_Delay(200);
    __HAL_RCC_USB_RELEASE_RESET();

    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_Initure.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLDOWN;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(300);
}
*/
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	filter_init();
  HAL_CAN_Start(&hcan);
  /*¿ªÖÐ¶Ï*/
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);       //can ½ÓÊÕfifo 0²»Îª¿ÕÖÐ¶Ï
  //¿ªÆôADC1,Ê¹ÄÜÖÐ¶Ï
  HAL_ADC_Start_IT(&hadc1);
  //printf("LAN YUAN TECH\r\n");
  //HAL_UART_Transmit(&huart1,"LAN YUAN TECH\r\n",strlen("LAN YUAN TECH\r\n"),0xFFFF);
  //HAL_Delay(50);

  //printf("LAN YUAN TECH\r\n");
  //printf("SYSTEM START\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint8_t alive_cnt = 0;
    alive_cnt ++;
    if(alive_cnt >= 10)
    {
      Control_DM_Motor_Enable();//20hz ·¢³öÊ¹ÄÜÖ¡ºó²Å¿ÉÒÔ¿ØÖÆ
      alive_cnt = 0;
    }

    Control_DM_POSITION_OMEGA_Process_PeriodElapsedCallback();//200hz ²ÉÓÃÎ»ÖÃËÙ¶È»·¿ØÖÆ

		HAL_Delay(5);
    
		 //CDC_Transmit_FS(buff,sizeof(buff)); //USB ï¿½ï¿½ï¿½â´®ï¿½Ú·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	  //HAL_Delay(500);

	 // printf("LAN YUAN TECH\r\n");

	  //HAL_UART_Transmit(&huart1,"LAN YUAN TECH\r\n",strlen("LAN YUAN TECH\r\n"),0xFFFF);
    #ifdef OLD
	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  HAL_Delay(200);
	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  HAL_Delay(200);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_Delay(200);
	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  HAL_Delay(200);
	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  HAL_Delay(200);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_Delay(200);


	  //printf("LAN YUAN TECH\r\n"); //USART1ï¿½ï¿½ï¿½ï¿½-USB
	  HAL_UART_Transmit(&huart1,"LAN YUAN TECH\r\n",strlen("LAN YUAN TECH\r\n"),0xFFFF);

	  HAL_GPIO_WritePin(TNOW_GPIO_Port, TNOW_Pin, SET);								//USART2-TNOW ï¿½ßµï¿½Æ½ ï¿½ï¿½ï¿½ï¿½Ê¹ï¿½ï¿½
	  HAL_UART_Transmit(&huart2,"LAN YUAN TECH",strlen("LAN YUAN TECH"),0xFFFF);	//USART2ï¿½ï¿½ï¿½ï¿½-RS485
	  HAL_GPIO_WritePin(TNOW_GPIO_Port, TNOW_Pin, RESET);							//USART2-TNOW ï¿½Íµï¿½Æ½ ï¿½ï¿½ï¿½ï¿½Ê¹ï¿½ï¿½

	  HAL_UART_Transmit(&huart3,"LAN YUAN TECH",strlen("LAN YUAN TECH"),0xFFFF);	//USART3ï¿½ï¿½ï¿½ï¿½-RS232

	  //CAN_Test();
	 	 //ï¿½ï¿½ï¿½ï¿½CANï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	 	if(CAN_Rx_Flag)
	 	{
	 	 CAN_Rx_Flag = 0;//ï¿½ï¿½ï¿½CANï¿½ï¿½ï¿½Õ±ï¿½Ö¾
	 	 printf("CANï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý£ï¿½\r\n");
	 	 for(int i = 0;i<8;i++) printf(" 0x%02x",RxDataCAN[i]);
	 	 printf("\r\n");
	 	}
    #endif

 
	
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TNOW_GPIO_Port, TNOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TNOW_Pin */
  GPIO_InitStruct.Pin = TNOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(TNOW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : K1_Pin K2_Pin */
  GPIO_InitStruct.Pin = K1_Pin|K2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void filter_init(void)
{
    HAL_StatusTypeDef HAL_Status;
    CAN_FilterTypeDef Filter0;
    Filter0.FilterBank = 1;//ï¿½Ë²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
    Filter0.FilterMode = CAN_FILTERMODE_IDMASK;
    Filter0.FilterScale = CAN_FILTERSCALE_32BIT;
    Filter0.FilterIdHigh = 0x00;
    Filter0.FilterIdLow = 0x00;
    Filter0.FilterMaskIdHigh = 0x00;
    Filter0.FilterMaskIdLow = 0x00;
    Filter0.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    Filter0.FilterActivation = CAN_FILTER_ENABLE;

    HAL_Status = HAL_CAN_ConfigFilter(&hcan,&Filter0);
    if(HAL_Status != HAL_OK)
    {
       	printf("CAN Filter set Fail!code:%drn",HAL_Status);
        Error_Handler();
    }
}

//canÖÐ¶Ï»Øµ÷º¯Êý-´¦Àí´ïÃîµÄ»Ø´«Êý¾Ý
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_StatusTypeDef HAL_Status;
    if(hcan->Instance == CAN1)
    {
        HAL_Status = HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeaderCAN,RxDataCAN);
        if(HAL_Status == HAL_OK)
        {
            //½â°ü
            CAN_Rx_Flag = 1;
            int16_t tmp_position = (RxDataCAN[1] << 8) | (RxDataCAN[2]);
            //½«±àÂëÆ÷¿Ì¶ÈÖµÓ³Éäµ½¶È
            DM_Motor_Rx_Data.Now_Angle = (float)tmp_position / 65536.0f * 26.0f / PI * 180.0f;
        }
    }
}

void CAN_Test(void)
{
	 //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½CAN
	      TxHeaderCAN.ExtId = 0x1800F001;
	      TxHeaderCAN.DLC = 8;
	      TxHeaderCAN.IDE = CAN_ID_STD;
	      TxHeaderCAN.RTR = CAN_RTR_DATA;
	      TxHeaderCAN.StdId = 0x01;
	      TxHeaderCAN.TransmitGlobalTime = ENABLE;

	      uint32_t TxMailBox;
	      HAL_StatusTypeDef HAL_Status;

    printf("\r\n------------------CANÍ¨ï¿½Å²ï¿½ï¿½ï¿½------------------\r\n");

    for (int i = 0; i < 8; ++i) TxDataCAN[i] = i;
    printf("CANï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý£ï¿½\r\n");
    for (int i = 0; i < 8; ++i) printf(" 0x%02x",TxDataCAN[i]);
    printf("\r\n");
    
    HAL_CAN_AddTxMessage(&hcan,&TxHeaderCAN,TxDataCAN,&TxMailBox);
}

/*
void CAN1_Send_Test(void)
{
	 //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½CAN
	    TxHeaderCAN.ExtId = 0x1800F001;
	    TxHeaderCAN.DLC = 8;
	    TxHeaderCAN.IDE = CAN_ID_STD;
	    TxHeaderCAN.RTR = CAN_RTR_DATA;
	    TxHeaderCAN.StdId = 0x01;
	    TxHeaderCAN.TransmitGlobalTime = ENABLE;

	    uint32_t TxMailBox;
	    HAL_StatusTypeDef HAL_Status;

	    HAL_CAN_AddTxMessage(&hcan,&TxHeaderCAN,TxDataCAN,&TxMailBox);


}
*/
// ï¿½ï¿½ main.c ï¿½ï¿½ï¿½ï¿½Ð´ï¿½Øµï¿½ï¿½ï¿½ï¿½ï¿½
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == K1_Pin)
    {  // KEY_PIN ï¿½ï¿½ï¿½æ»»Îªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Å£ï¿½ï¿½ï¿? GPIO_PIN_0ï¿½ï¿½
        // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½Æ½ï¿½Ç·ï¿½ï¿½È¶ï¿½
        //HAL_Delay(10);  // ×¢ï¿½â£ºï¿½Ð¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½Êµï¿½ï¿½ï¿½ï¿½Ä¿ï¿½ï¿½ï¿½ï¿½ï¿½Ã±ï¿½ï¿½Î?+ï¿½ï¿½Ñ­ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
        if (HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin) == GPIO_PIN_RESET)
        {
            // Ö´ï¿½Ð°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ç·­×ª LED
        	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
            //printf("KEY1 trigger \r\n");
        	HAL_UART_Transmit(&huart1,"KEY1 trigger \r\n",strlen("KEY1 trigger \r\n"),0xFFFF);
        }
    }

    if (GPIO_Pin == K2_Pin) {  // KEY_PIN ï¿½ï¿½ï¿½æ»»Îªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Å£ï¿½ï¿½ï¿? GPIO_PIN_0ï¿½ï¿½
            // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½Æ½ï¿½Ç·ï¿½ï¿½È¶ï¿½
          //  HAL_Delay(10);  // ×¢ï¿½â£ºï¿½Ð¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½Êµï¿½ï¿½ï¿½ï¿½Ä¿ï¿½ï¿½ï¿½ï¿½ï¿½Ã±ï¿½ï¿½Î?+ï¿½ï¿½Ñ­ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
            if (HAL_GPIO_ReadPin(K2_GPIO_Port, K2_Pin) == GPIO_PIN_RESET)
            {
                // Ö´ï¿½Ð°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ç·­×ª LED
            	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
                //printf("KEY2 trigger\r\n");
                HAL_UART_Transmit(&huart1,"KEY2 trigger \r\n",strlen("KEY2 trigger \r\n"),0xFFFF);
            }
        }


}
/**
 * @brief ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¡
 *
 * @param hcan CANï¿½ï¿½ï¿?
 * @param ID ID
 * @param Data ï¿½ï¿½ï¿½ï¿½ï¿½Íµï¿½ï¿½ï¿½ï¿½ï¿½Ö¸ï¿½ï¿½
 * @param Length ï¿½ï¿½ï¿½ï¿½
 * @return uint8_t Ö´ï¿½ï¿½×´Ì¬
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    // ï¿½ï¿½â´?ï¿½ï¿½ï¿½Ç·ï¿½ï¿½ï¿½È·
    assert_param(hcan != NULL);

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = Length;

    if (HAL_ERROR == HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox))
        return HAL_ERROR;
    else
        return HAL_OK;
}
void Control_DM_Motor_Enable()
{
  CAN_Send_Data(&hcan,0x1f1, DM_Motor_CAN_Message_Enter, 8);
}
void Math_Constrain(float *value, float min, float max)
{
  if(*value < min)
  {
    *value = min;
  }
  else if(*value > max)
  {
    *value = max;
  }
}
void Control_DM_POSITION_OMEGA_Process_PeriodElapsedCallback()
{
  float set_angle = Control_Angle;

  Math_Constrain(&set_angle,0.0f,90.0f);

  Set_Angle_And_Omega[0] = (set_angle - 0.0f) / 90.0f * 4.0f * PI;

  memcpy(&TxDataCAN[0], &Set_Angle_And_Omega[0], sizeof(float));

  memcpy(&TxDataCAN[4], &Set_Angle_And_Omega[1], sizeof(float));

  CAN_Send_Data(&hcan,0x1f1, TxDataCAN, 8);
}
//ADC×ª»»Íê³É×Ô¶¯µ÷ÓÃº¯Êý
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    
     //»ñÈ¡Öµ²¢´æ´¢
   ADC_Values = HAL_ADC_GetValue(hadc);
     

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

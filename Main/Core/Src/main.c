/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_MAX_SPEED 999
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#define ARR_CNT 5
#define CMD_SIZE 50
#define DS18B20_PIN_GPIO_Port GPIOC
#define DS18B20_PIN GPIO_PIN_2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
volatile uint32_t adc_value;
volatile uint8_t percentage;
volatile uint32_t height;
volatile int tim3Flag1Sec;
volatile unsigned int tim3Sec;
uint8_t rx2char;
volatile unsigned char rx2Flag = 0;
volatile char rx2Data[50];
volatile unsigned char btFlag = 0;
uint8_t btchar;
char btData[50];
__IO uint16_t ADCxConvertValue[3] = {0};
__IO int adcFlag = 0;
uint16_t adc1;
float vSense;
volatile uint8_t rx_data;
volatile float Temperature;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void Send_Data_To_SQL(const char *name, const char *date, const char *time,
                      uint16_t turbidity, uint16_t temperature, uint16_t level1, uint16_t level2);
void Get_Current_Time(char *date, char *time);
void Motor_SetSpeed(uint16_t speed);
void Motor_SetDirection(uint8_t direction);
void Motor_Start(void);
void Motor_Stop(void);
void Motor2_SetSpeed(uint16_t speed);
void Motor2_SetDirection(uint8_t direction);
void Motor2_Start(void);
void Motor2_Stop(void);
uint8_t Convert_ADC_To_Percentage(uint16_t adc_value);
void bluetooth_Event();

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
	uint8_t Presence=0;
	uint8_t Temp_byte1,Temp_byte2;
	uint16_t TEMP;
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_RTC_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  char buff[30];
  DS18B20_init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_UART_Receive_IT(&huart2, &rx2char,1);
  HAL_UART_Receive_IT(&huart6, &btchar,1);
  if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
 	 Error_Handler();
  }
  if(HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADCxConvertValue, 3) != HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Presence = DS18B20_Start ();
	  if(Presence==1)
	  {
		  DS18B20_Write (0xCC);  // skip ROM
		  DS18B20_Write (0x44);  // convert t

		  Presence = DS18B20_Start();
		  if(Presence==1)
		  {
			  DS18B20_Write (0xCC);  // skip ROM
			  DS18B20_Write (0xBE);  // Read Scratch-pad
			  Temp_byte1 = DS18B20_Read();
			  Temp_byte2 = DS18B20_Read();
			  TEMP = ((Temp_byte2<<8))|Temp_byte1;
			  Temperature = (float)TEMP/16.0;  // resolution is 0.0625
		  }
	  }
	  int temperature = (int)Temperature;
      int Turbidity = ADCxConvertValue[0];
      int Level1 = ADCxConvertValue[1];
      int Level2 = ADCxConvertValue[2];
	  if(rx2Flag)
	  {
		  printf("recv2 : %s\r\n",rx2Data);
		  rx2Flag =0;
	  }
	  if(btFlag)
	  {
		  btFlag =0;
		  bluetooth_Event();
	  }
	  if(tim3Flag1Sec)
	  {
		  tim3Flag1Sec = 0;
	      char date[11];
	      char time[9];
		  char sendsBuf[CMD_SIZE];

	      Get_Current_Time(date, time);
		  printf("tim3Sec : %d\r\n",tim3Sec);
	      if(tim3Sec % 5 == 0)
	      {
			  Send_Data_To_SQL("KSW_SQL", date, time, Turbidity, temperature, Level1, Level2);
	      }
	      else
	      {
			  snprintf(sendsBuf, CMD_SIZE, "[%s]%s@%u@%u@%u@%u\n","KSW_ARD", "DATA", Turbidity, temperature, Level1, Level2);
			  HAL_UART_Transmit(&huart6, (uint8_t *)sendsBuf, strlen(sendsBuf), HAL_MAX_DELAY);
			  printf("Sending Arduino: %s\r\n", sendsBuf);
	      }
		  if(adcFlag)
		  {
			  if(HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADCxConvertValue, 3) != HAL_OK)
			  {
				  Error_Handler();
			  }
		  }
		  if(Turbidity < 2000 && Level1 > 1800  && Level2 < 3900)
		  {
			  Motor_SetDirection(0); //공급 모터
			  Motor_SetSpeed(800);
			  Motor2_SetDirection(0); //배출 모터
			  Motor2_SetSpeed(800);
			  printf("Cleaning Water\r\n");
		  }
		  else if(Level2 > 3900)
		  {
			  Motor_SetDirection(1); //배출 모터
			  Motor_SetSpeed(800);
			  printf("Discharge\r\n");
		  }
		  else if(Level2 < 2500)
		  {
			  Motor2_SetDirection(1); //공급 모터
			  Motor2_SetSpeed(800);
			  printf("supply\r\n");
		  }
		  else if(Level1 < 1500)
		  {
			  printf("Motor Stop\r\n");
			  printf("Supply water_Shortage\r\n");
			  Motor_Stop();
			  Motor2_Stop();
		  }
		  else
		  {
			  printf("Motor Stop\r\n");
			  Motor_Stop();
			  Motor2_Stop();
		  }
	  }
	  //Motor_SetDirection(1); //배출 모터
	  //Motor_SetSpeed(999);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x23;
  sTime.Minutes = 0x10;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = 0x03;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 0x27;
  sDate.Year = 0x24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84 -1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 84-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR1_Pin|DIR2_Pin|DS18B20_Pin|DIR3_Pin
                          |DIR4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR1_Pin DIR2_Pin DS18B20_Pin DIR3_Pin
                           DIR4_Pin */
  GPIO_InitStruct.Pin = DIR1_Pin|DIR2_Pin|DS18B20_Pin|DIR3_Pin
                          |DIR4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Get_Current_Time(char *date, char *time)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    snprintf(date, 11, "%04d.%02d.%02d", 2000 + sDate.Year, sDate.Month, sDate.Date);
    snprintf(time, 9, "%02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
}

void Send_Data_To_SQL(const char *name, const char *date, const char *time, uint16_t turbidity, uint16_t temperature, uint16_t level1, uint16_t level2)
{
    char sendBuf[CMD_SIZE];
    snprintf(sendBuf, CMD_SIZE, "[%s]%s@%s@%u@%u@%u@%u\n",
             name, date, time, turbidity, temperature, level1, level2);
    printf("Sending Data: %s\r\n", sendBuf);
    HAL_UART_Transmit(&huart6, (uint8_t *)sendBuf, strlen(sendBuf), HAL_MAX_DELAY);
}

uint8_t Convert_ADC_To_Percentage(uint16_t adc_value)
{
    return (uint8_t)((adc_value * 100) / 4095);
}
void Motor_SetSpeed(uint16_t speed)
{
  if (speed > MOTOR_MAX_SPEED)
    speed = MOTOR_MAX_SPEED;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}
void Motor_SetDirection(uint8_t direction)
{
  if (direction == 1)
  {
    HAL_GPIO_WritePin(GPIOA, DIR1_Pin, GPIO_PIN_SET);  // DIR1 = HIGH
    HAL_GPIO_WritePin(GPIOA, DIR2_Pin, GPIO_PIN_RESET); // DIR2 = LOW
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, DIR1_Pin, GPIO_PIN_RESET); // DIR1 = LOW
    HAL_GPIO_WritePin(GPIOA, DIR2_Pin, GPIO_PIN_SET);  // DIR2 = HIGH
  }
}
void Motor_Start(void)
{
  Motor_SetSpeed(999);
}
void Motor_Stop(void)
{
  Motor_SetSpeed(0);
}
void Motor2_SetSpeed(uint16_t speed)
{
  if (speed > MOTOR_MAX_SPEED)
    speed = MOTOR_MAX_SPEED;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
}
void Motor2_SetDirection(uint8_t direction)
{
  if (direction == 1)
  {
    HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_SET);  // DIR1 = HIGH
    HAL_GPIO_WritePin(GPIOA, DIR4_Pin, GPIO_PIN_RESET); // DIR2 = LOW
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_RESET); // DIR1 = LOW
    HAL_GPIO_WritePin(GPIOA, DIR4_Pin, GPIO_PIN_SET);  // DIR2 = HIGH
  }
}
void Motor2_Start(void)
{
  Motor2_SetSpeed(999);
}
void Motor2_Stop(void)
{
  Motor2_SetSpeed(0);
}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART6 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int tim3Cnt = 0;
	tim3Cnt++;
	if(tim3Cnt == 1000) //1ms * 1000
	{
		tim3Flag1Sec = 1;
		tim3Sec++;
		tim3Cnt = 0;
	}
}
void bluetooth_Event()
{

  int i=0;
  char * pToken;
  char * pArray[ARR_CNT]={0};
  char recvBuf[CMD_SIZE]={0};
  char sendBuf[CMD_SIZE]={0};
  strcpy(recvBuf,btData);

  printf("btData : %s\r\n",btData);

  pToken = strtok(recvBuf,"[@]");
  while(pToken != NULL)
  {
    pArray[i] =  pToken;
    if(++i >= ARR_CNT)
      break;
    pToken = strtok(NULL,"[@]");
  }
//  printf("pArray[0] : %s\r\n",pArray[0]);
//  printf("pArray[1] : %s\r\n",pArray[1]);
//  printf("pArray[2] : %s\r\n",pArray[2]);
  if(!strncmp(pArray[1]," New conn",sizeof(" New conn")))
  {
      return;
  }
  else if(!strncmp(pArray[1]," Already log",sizeof(" Already log")))
  {
      return;
  }
  else
      return;

  sprintf(sendBuf,"[%s]%s@%s\n",pArray[0],pArray[1],pArray[2]);
  HAL_UART_Transmit(&huart6, (uint8_t *)sendBuf, strlen(sendBuf), 0xFFFF);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
    	static int i=0;
    	rx2Data[i] = rx2char;
    	if((rx2Data[i] == '\r')||(btData[i] == '\n'))
    	{
    		rx2Data[i] = '\0';
    		rx2Flag = 1;
    		i = 0;
    	}
    	else
    	{
    		i++;
    	}
    	HAL_UART_Receive_IT(&huart2, &rx2char,1);
    }
    if(huart->Instance == USART6)
    {
    	static int i=0;
    	btData[i] = btchar;
    	if((btData[i] == '\n') || btData[i] == '\r')
    	{
    		btData[i] = '\0';
    		btFlag = 1;
    		i = 0;
    	}
    	else
    	{
    		i++;
    	}
    	HAL_UART_Receive_IT(&huart6, &btchar,1);
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
	adcFlag = 1;
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

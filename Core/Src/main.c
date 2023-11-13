/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <math.h>
#include <malloc.h>

#include "ntshell.h"
#include "usrcmd.h"

#include "wave_capture.h"

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

ntshell_t nts;

WaveCapture_t wavecap;


float wave_data[4] = {0};
int32_t wave_data_i[4] = {0};

int test_counter = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void __io_putchar(uint8_t ch)
{
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}


int wave_tx_func(const uint8_t *buf, int length)
{
	HAL_StatusTypeDef status;
	status = HAL_UART_Transmit(&huart2, buf, (uint16_t)length, 1000);
	if(status != HAL_OK)
	{
		return 0;
	}
	return length;
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void get_adc(uint16_t* ad_arr)
{

	while(HAL_GPIO_ReadPin(nEOLC_GPIO_Port, nEOLC_Pin) == GPIO_PIN_SET){}

	HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_RESET);


	for(int ch = 0; ch < 4; ch++)
	{
	  HAL_GPIO_WritePin(nRD_GPIO_Port, nRD_Pin, GPIO_PIN_RESET);


	  uint32_t GB;
	  uint16_t D, DH, DL;
	  GB = GPIOB->IDR;
	  DL = (GB >> 2) & 0x000001FF;
	  DH = (GB >> 12) & 0x00000007;
	  D = (DH << 9) | DL;
	  ad_arr[ch] = D;

	  HAL_GPIO_WritePin(nRD_GPIO_Port, nRD_Pin, GPIO_PIN_SET);

	}

	HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_RESET);

}




float omega = 377;
float phase = 0.0f;
float amp_u = 0;
float amp_v = 0;
float amp_w = 0;

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{

	if(htim->Instance == TIM8 && __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8))
	{
		uint16_t ad_arr[4];

		get_adc(ad_arr);



		phase += omega * 100E-6;
		if(phase > M_PI)
		{
			phase -= 2*M_PI;
		}

		amp_u = 0.9 * cosf(phase);
		amp_v = 0.9 * cosf(phase - M_PI*2/3.0f);
		amp_w = 0.9 * cosf(phase + M_PI*2/3.0f);

		amp_u = 0.95;
		amp_v = 0.5;
		amp_w = 0;

		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, htim8.Init.Period / 2 * (1 + amp_u));
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, htim8.Init.Period / 2 * (1 + amp_v));
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, htim8.Init.Period / 2 * (1 + amp_w));

		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);


		wave_data[0] = ad_arr[0]/4096.0f * 5.0f;
		wave_data[1] = ad_arr[1]/4096.0f * 5.0f;
		wave_data[2] = ad_arr[2]/4096.0f * 5.0f;
		wave_data[3] = ad_arr[3]/4096.0f * 5.0f;

		WaveCapture_Sampling(&wavecap);


	}

}


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
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  	printf("Hello World\n");

  	WaveCapture_Type_e wavecap_type[] = {
			WAVECAPTURE_TYPE_FLOAT,
			WAVECAPTURE_TYPE_FLOAT,
			WAVECAPTURE_TYPE_FLOAT,
			WAVECAPTURE_TYPE_FLOAT,
  	};
  	void* wavecap_var_ptr_array[] = {
  			&(wave_data[0]),
			&(wave_data[1]),
			&(wave_data[2]),
			&(wave_data[3]),
  	};
  	WaveCapture_Init_t wave_init;
  	wave_init.channel_num = 4;
  	wave_init.func_write = wave_tx_func;
  	wave_init.sampling_length = 1024;
  	wave_init.type_array = wavecap_type;
  	wave_init.var_ptr_array = wavecap_var_ptr_array;
  	int rtn = WaveCapture_Init(&wavecap, &wave_init);
  	printf("return = %d\n", rtn);


  	printf("type[0] = %d\n", wavecap.init.type_array[0]);
  	printf("type[1] = %d\n", wavecap.init.type_array[1]);
  	printf("type[2] = %d\n", wavecap.init.type_array[2]);
  	printf("type[3] = %d\n", wavecap.init.type_array[3]);
  	printf("sizeof(WaveCapture_Type_e) = %d\n", sizeof(WaveCapture_Type_e));
  	printf("sizeof(void*) = %d\n", sizeof(void*));
  	printf("type_array size = %d\n", malloc_usable_size(wavecap.init.type_array));
  	printf("var_ptr_array size = %d\n", malloc_usable_size(wavecap.init.var_ptr_array));
  	printf("wavedata size = %d\n", malloc_usable_size(wavecap.wavedata));
  	printf("wavedata[0] size = %d\n", malloc_usable_size(wavecap.wavedata[0]));
  	printf("wavedata_length = %d\n", wavecap.init.sampling_length);





//	HAL_Delay(1000);
//
//	HAL_GPIO_WritePin(MC_GPIO_Port, MC_Pin, GPIO_PIN_SET);
//
//	HAL_Delay(1000);
//
//	HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);



	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);


	__HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

	HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE);

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, htim8.Init.Period / 2 * (1 + 0));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, htim8.Init.Period / 2 * (1 + 0));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, htim8.Init.Period / 2 * (1 - 0));
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, htim8.Init.Period / 2 * (1 - 0.9));

	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_4);

	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_3);



	// ADC Setting

	// Initialize
  	HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(nRD_GPIO_Port, nRD_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(nWR_GPIO_Port, nWR_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(CONVST_GPIO_Port, CONVST_Pin, GPIO_PIN_SET);

  	// Write mode
  	HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(nWR_GPIO_Port, nWR_Pin, GPIO_PIN_RESET);

  	// Set GPIO as output port
    GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|D3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// channel activation
  	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);

  	// Write
  	HAL_GPIO_WritePin(nWR_GPIO_Port, nWR_Pin, GPIO_PIN_SET);
  	HAL_Delay(1);
  	HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_SET);

  	// Set GPIO as input port
	GPIO_InitStruct.Pin = D0_Pin|D8_Pin|D9_Pin|D10_Pin
						  |D11_Pin|nEOC_Pin|D1_Pin|D2_Pin
						  |D3_Pin|D4_Pin|D5_Pin|D6_Pin
						  |D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);




  	ntshell_usr_init(&nts);

	ntshell_execute(&nts);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay(100);


//	  printf("temperature flag = %d\n", HAL_GPIO_ReadPin(TH_GPIO_Port, TH_Pin));


//	  HAL_GPIO_WritePin(CONVST_GPIO_Port, CONVST_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(1);
//	  HAL_GPIO_WritePin(CONVST_GPIO_Port, CONVST_Pin, GPIO_PIN_SET);
//
//	  while(HAL_GPIO_ReadPin(nEOLC_GPIO_Port, nEOLC_Pin) == GPIO_PIN_SET){}
//
//	  HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_RESET);
//
//	  uint16_t ad_arr[4];
//
//	  for(int ch = 0; ch < 4; ch++)
//	  {
//		  HAL_GPIO_WritePin(nRD_GPIO_Port, nRD_Pin, GPIO_PIN_RESET);
//		  HAL_Delay(1);
//
//		  uint32_t GB;
//		  uint16_t D, DH, DL;
//		  GB = GPIOB->IDR;
//		  DL = (GB >> 2) & 0x000001FF;
//		  DH = (GB >> 12) & 0x00000007;
//		  D = (DH << 9) | DL;
//		  ad_arr[ch] = D;
//
//		  HAL_GPIO_WritePin(nRD_GPIO_Port, nRD_Pin, GPIO_PIN_SET);
//
//		  HAL_Delay(1);
//	  }
//
//	  HAL_GPIO_WritePin(nCS_GPIO_Port, nCS_Pin, GPIO_PIN_RESET);


//	  for(int ch = 0; ch < 4; ch++)
//	  {
//		  printf("%5f  ", ad_arr[ch] / 4096.0f);
//	  }

//	  for(int i = 0; i < 12; i++)
//	  {
//		  int b = (ad_arr[0] >> (12-1-i)) & 0x1;
//		  printf("%d", b);
//		  if(((i+1) & 0x3) == 0)
//		  {
//			  printf(" ");
//		  }
//	  }

	  printf("\n");


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 87;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim8.Init.Period = 9000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 205;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|nWR_Pin|nCS_Pin|nRD_Pin
                          |CONVST_Pin|SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MC_Pin|SD_Pin|CLK_Pin|nCHSHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FLT_Pin TH_Pin */
  GPIO_InitStruct.Pin = FLT_Pin|TH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin nWR_Pin nCS_Pin nRD_Pin
                           CONVST_Pin SHDN_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|nWR_Pin|nCS_Pin|nRD_Pin
                          |CONVST_Pin|SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MC_Pin SD_Pin CLK_Pin nCHSHDN_Pin */
  GPIO_InitStruct.Pin = MC_Pin|SD_Pin|CLK_Pin|nCHSHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin D8_Pin D9_Pin D10_Pin
                           D11_Pin nEOC_Pin D1_Pin D2_Pin
                           D3_Pin D4_Pin D5_Pin D6_Pin
                           D7_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D8_Pin|D9_Pin|D10_Pin
                          |D11_Pin|nEOC_Pin|D1_Pin|D2_Pin
                          |D3_Pin|D4_Pin|D5_Pin|D6_Pin
                          |D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : nEOLC_Pin */
  GPIO_InitStruct.Pin = nEOLC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(nEOLC_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

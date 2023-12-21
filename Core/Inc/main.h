/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define FLT_Pin GPIO_PIN_2
#define FLT_GPIO_Port GPIOC
#define TH_Pin GPIO_PIN_3
#define TH_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define nWR_Pin GPIO_PIN_4
#define nWR_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define nCS_Pin GPIO_PIN_6
#define nCS_GPIO_Port GPIOA
#define MC_Pin GPIO_PIN_4
#define MC_GPIO_Port GPIOC
#define SD_Pin GPIO_PIN_5
#define SD_GPIO_Port GPIOC
#define D0_Pin GPIO_PIN_2
#define D0_GPIO_Port GPIOB
#define D8_Pin GPIO_PIN_10
#define D8_GPIO_Port GPIOB
#define D9_Pin GPIO_PIN_12
#define D9_GPIO_Port GPIOB
#define D10_Pin GPIO_PIN_13
#define D10_GPIO_Port GPIOB
#define D11_Pin GPIO_PIN_14
#define D11_GPIO_Port GPIOB
#define nEOC_Pin GPIO_PIN_15
#define nEOC_GPIO_Port GPIOB
#define nRD_Pin GPIO_PIN_10
#define nRD_GPIO_Port GPIOA
#define CONVST_Pin GPIO_PIN_11
#define CONVST_GPIO_Port GPIOA
#define SHDN_Pin GPIO_PIN_12
#define SHDN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define nEOLC_Pin GPIO_PIN_15
#define nEOLC_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_11
#define CLK_GPIO_Port GPIOC
#define nCHSHDN_Pin GPIO_PIN_12
#define nCHSHDN_GPIO_Port GPIOC
#define D1_Pin GPIO_PIN_3
#define D1_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_4
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_5
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_6
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_7
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_8
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_9
#define D7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

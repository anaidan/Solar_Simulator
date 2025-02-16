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
#include "stm32h5xx_hal.h"

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
#define ADC_LOAD_CURR_Pin GPIO_PIN_0
#define ADC_LOAD_CURR_GPIO_Port GPIOA
#define ADC_LOAD_VOL_Pin GPIO_PIN_1
#define ADC_LOAD_VOL_GPIO_Port GPIOA
#define ADC_POT_Pin GPIO_PIN_2
#define ADC_POT_GPIO_Port GPIOA
#define FAN_PWM_Pin GPIO_PIN_3
#define FAN_PWM_GPIO_Port GPIOA
#define ADC_VIN_Pin GPIO_PIN_5
#define ADC_VIN_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_6
#define EN_GPIO_Port GPIOA
#define _5V_DIV_Pin GPIO_PIN_7
#define _5V_DIV_GPIO_Port GPIOA
#define ADC_TEMP_Pin GPIO_PIN_0
#define ADC_TEMP_GPIO_Port GPIOB
#define DISP_CS_Pin GPIO_PIN_12
#define DISP_CS_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_14
#define LED_BLUE_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_15
#define LED_YELLOW_GPIO_Port GPIOB
#define OUTPUT_EN_SW_Pin GPIO_PIN_8
#define OUTPUT_EN_SW_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

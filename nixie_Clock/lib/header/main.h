/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define MICRO_LED_Pin GPIO_PIN_13
#define MICRO_LED_GPIO_Port GPIOC
#define SWITCH_Pin GPIO_PIN_15
#define SWITCH_GPIO_Port GPIOC
#define LED_HD_Pin GPIO_PIN_0
#define LED_HD_GPIO_Port GPIOA
#define LED_HU_Pin GPIO_PIN_1
#define LED_HU_GPIO_Port GPIOA
#define LED_MD_Pin GPIO_PIN_2
#define LED_MD_GPIO_Port GPIOA
#define LED_MU_Pin GPIO_PIN_3
#define LED_MU_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOA
#define BUTTON_ALARM_Pin GPIO_PIN_5
#define BUTTON_ALARM_GPIO_Port GPIOA
#define HOUR_POT_Pin GPIO_PIN_6
#define HOUR_POT_GPIO_Port GPIOA
#define DOT1_Pin GPIO_PIN_7
#define DOT1_GPIO_Port GPIOA
#define DOT2_Pin GPIO_PIN_0
#define DOT2_GPIO_Port GPIOB
#define MIN_POT_Pin GPIO_PIN_1
#define MIN_POT_GPIO_Port GPIOB
#define BUTTON_MODE_Pin GPIO_PIN_10
#define BUTTON_MODE_GPIO_Port GPIOB
#define CLK_50HZ_Pin GPIO_PIN_11
#define CLK_50HZ_GPIO_Port GPIOB
#define CLK_50HZ_EXTI_IRQn EXTI15_10_IRQn
#define MU0_Pin GPIO_PIN_12
#define MU0_GPIO_Port GPIOB
#define MU1_Pin GPIO_PIN_13
#define MU1_GPIO_Port GPIOB
#define MU_EN_Pin GPIO_PIN_14
#define MU_EN_GPIO_Port GPIOB
#define MU_A2_Pin GPIO_PIN_15
#define MU_A2_GPIO_Port GPIOB
#define MU_A1_Pin GPIO_PIN_8
#define MU_A1_GPIO_Port GPIOA
#define MU_A0_Pin GPIO_PIN_9
#define MU_A0_GPIO_Port GPIOA
#define MD_EN_Pin GPIO_PIN_10
#define MD_EN_GPIO_Port GPIOA
#define MD_A2_Pin GPIO_PIN_11
#define MD_A2_GPIO_Port GPIOA
#define MD_A1_Pin GPIO_PIN_12
#define MD_A1_GPIO_Port GPIOA
#define HU_A1_Pin GPIO_PIN_5
#define HU_A1_GPIO_Port GPIOB
#define HU_A2_Pin GPIO_PIN_6
#define HU_A2_GPIO_Port GPIOB
#define HU_8_Pin GPIO_PIN_7
#define HU_8_GPIO_Port GPIOB
#define HU_9_Pin GPIO_PIN_8
#define HU_9_GPIO_Port GPIOB
#define HD_1_Pin GPIO_PIN_9
#define HD_1_GPIO_Port GPIOB

struct Time {
   uint8_t  hour;
   uint8_t  minutes;
   uint8_t alarm;
} time; 

struct Time g_time;
struct Time alarm;


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

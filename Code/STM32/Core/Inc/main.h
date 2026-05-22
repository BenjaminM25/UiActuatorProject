/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWMAL_Pin GPIO_PIN_13
#define PWMAL_GPIO_Port GPIOC
#define PWMAH_Pin GPIO_PIN_0
#define PWMAH_GPIO_Port GPIOC
#define PWMBH_Pin GPIO_PIN_1
#define PWMBH_GPIO_Port GPIOC
#define PWMCH_Pin GPIO_PIN_2
#define PWMCH_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOA
#define HALL_U_Pin GPIO_PIN_5
#define HALL_U_GPIO_Port GPIOA
#define HALL_U_EXTI_IRQn EXTI9_5_IRQn
#define HALL_V_Pin GPIO_PIN_6
#define HALL_V_GPIO_Port GPIOA
#define HALL_V_EXTI_IRQn EXTI9_5_IRQn
#define HALL_W_Pin GPIO_PIN_7
#define HALL_W_GPIO_Port GPIOA
#define HALL_W_EXTI_IRQn EXTI9_5_IRQn
#define PWMBL_Pin GPIO_PIN_0
#define PWMBL_GPIO_Port GPIOB
#define PWMCL_Pin GPIO_PIN_1
#define PWMCL_GPIO_Port GPIOB
#define LIMIT_SWITCH_Pin GPIO_PIN_6
#define LIMIT_SWITCH_GPIO_Port GPIOC
#define VBUS_Pin GPIO_PIN_9
#define VBUS_GPIO_Port GPIOA
#define DRV8316nSCS_Pin GPIO_PIN_2
#define DRV8316nSCS_GPIO_Port GPIOD
#define nFAULT_Pin GPIO_PIN_3
#define nFAULT_GPIO_Port GPIOB
#define nSLEEP_Pin GPIO_PIN_4
#define nSLEEP_GPIO_Port GPIOB
#define DRVOFF_Pin GPIO_PIN_5
#define DRVOFF_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

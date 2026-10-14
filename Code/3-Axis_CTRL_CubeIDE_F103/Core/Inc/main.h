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
#define X_HOME_Pin GPIO_PIN_0
#define X_HOME_GPIO_Port GPIOA
#define Y_HOME_Pin GPIO_PIN_1
#define Y_HOME_GPIO_Port GPIOA
#define X__MANUAL_Pin GPIO_PIN_2
#define X__MANUAL_GPIO_Port GPIOA
#define X__MANUALA3_Pin GPIO_PIN_3
#define X__MANUALA3_GPIO_Port GPIOA
#define Z_HOME_Pin GPIO_PIN_4
#define Z_HOME_GPIO_Port GPIOA
#define Z_DIR_Pin GPIO_PIN_0
#define Z_DIR_GPIO_Port GPIOB
#define X_PUL_Pin GPIO_PIN_12
#define X_PUL_GPIO_Port GPIOB
#define X_DIR_Pin GPIO_PIN_13
#define X_DIR_GPIO_Port GPIOB
#define Y_PUL_Pin GPIO_PIN_14
#define Y_PUL_GPIO_Port GPIOB
#define Y_DIR_Pin GPIO_PIN_15
#define Y_DIR_GPIO_Port GPIOB
#define Y__MANUAL_Pin GPIO_PIN_8
#define Y__MANUAL_GPIO_Port GPIOA
#define BTN_HOMING_Pin GPIO_PIN_11
#define BTN_HOMING_GPIO_Port GPIOA
#define BTN_RMLOCK_Pin GPIO_PIN_12
#define BTN_RMLOCK_GPIO_Port GPIOA
#define BTN_AUTO15_Pin GPIO_PIN_13
#define BTN_AUTO15_GPIO_Port GPIOA
#define BTN_AUTO30_Pin GPIO_PIN_14
#define BTN_AUTO30_GPIO_Port GPIOA
#define CONT_STOP_Pin GPIO_PIN_3
#define CONT_STOP_GPIO_Port GPIOB
#define Y__MANUALB5_Pin GPIO_PIN_5
#define Y__MANUALB5_GPIO_Port GPIOB
#define Z__MANUAL_Pin GPIO_PIN_6
#define Z__MANUAL_GPIO_Port GPIOB
#define Z__MANUALB7_Pin GPIO_PIN_7
#define Z__MANUALB7_GPIO_Port GPIOB
#define X_LIMIT_Pin GPIO_PIN_8
#define X_LIMIT_GPIO_Port GPIOB
#define Y_LIMIT_Pin GPIO_PIN_9
#define Y_LIMIT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

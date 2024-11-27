/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTORA_IN1_Pin GPIO_PIN_0
#define MOTORA_IN1_GPIO_Port GPIOB
#define MOTORA_IN2_Pin GPIO_PIN_1
#define MOTORA_IN2_GPIO_Port GPIOB
#define MOTORB_IN1_Pin GPIO_PIN_10
#define MOTORB_IN1_GPIO_Port GPIOB
#define MOTORB_IN2_Pin GPIO_PIN_11
#define MOTORB_IN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define DEBUG 1

// #define WHEEL1_ID 0
// #define WHEEL2_ID 1
#define WHEEL1_ID 2
#define WHEEL2_ID 3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
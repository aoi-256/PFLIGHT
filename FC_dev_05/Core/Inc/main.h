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
#include "stm32f7xx_hal.h"

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
#define BUZZER_Pin GPIO_PIN_2
#define BUZZER_GPIO_Port GPIOC
#define SPI1_CS1_Pin GPIO_PIN_4
#define SPI1_CS1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_0
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_1
#define LED4_GPIO_Port GPIOB
#define Servo_01_Pin GPIO_PIN_14
#define Servo_01_GPIO_Port GPIOB
#define Servo_02_Pin GPIO_PIN_15
#define Servo_02_GPIO_Port GPIOB
#define Motor_05_Pin GPIO_PIN_6
#define Motor_05_GPIO_Port GPIOC
#define Motor_06_Pin GPIO_PIN_7
#define Motor_06_GPIO_Port GPIOC
#define Motor_07_Pin GPIO_PIN_8
#define Motor_07_GPIO_Port GPIOC
#define Motor_08_Pin GPIO_PIN_9
#define Motor_08_GPIO_Port GPIOC
#define Motor_01_Pin GPIO_PIN_8
#define Motor_01_GPIO_Port GPIOA
#define Motor_02_Pin GPIO_PIN_9
#define Motor_02_GPIO_Port GPIOA
#define Motor_03_Pin GPIO_PIN_10
#define Motor_03_GPIO_Port GPIOA
#define Motor_04_Pin GPIO_PIN_11
#define Motor_04_GPIO_Port GPIOA
#define SPI1_CS2_Pin GPIO_PIN_12
#define SPI1_CS2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

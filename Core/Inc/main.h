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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC__Pin GPIO_PIN_1
#define ADC__GPIO_Port GPIOA
#define ADC_A2_Pin GPIO_PIN_2
#define ADC_A2_GPIO_Port GPIOA
#define DAC_OUT_Pin GPIO_PIN_5
#define DAC_OUT_GPIO_Port GPIOA
#define AR1_Pin GPIO_PIN_0
#define AR1_GPIO_Port GPIOB
#define GAIN1_Pin GPIO_PIN_1
#define GAIN1_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_6
#define BTN1_GPIO_Port GPIOC
#define GAIN2_Pin GPIO_PIN_7
#define GAIN2_GPIO_Port GPIOC
#define AR2_Pin GPIO_PIN_8
#define AR2_GPIO_Port GPIOC
#define MODE4_Pin GPIO_PIN_9
#define MODE4_GPIO_Port GPIOC
#define MODE3_Pin GPIO_PIN_8
#define MODE3_GPIO_Port GPIOA
#define MODE2_Pin GPIO_PIN_9
#define MODE2_GPIO_Port GPIOA
#define MODE1_Pin GPIO_PIN_10
#define MODE1_GPIO_Port GPIOA
#define DAC_EN_Pin GPIO_PIN_10
#define DAC_EN_GPIO_Port GPIOC
#define BTN2_Pin GPIO_PIN_2
#define BTN2_GPIO_Port GPIOD
#define BTN3_Pin GPIO_PIN_6
#define BTN3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/******************************************************************************
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "logging.h"
#include "config.h"
#include "neo_m8_gps.h"
#include "lsm6dso.h"
#include "lps22hb.h"
#include "radio.h"
#include "tusb_config.h"
#include "usb_descriptors.h"
#include "tusb.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
     extern SPI_HandleTypeDef hspi2;
     extern SPI_HandleTypeDef hspi3;

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
#define MEM_CS_Pin GPIO_PIN_13
#define MEM_CS_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_14
#define SD_CS_GPIO_Port GPIOC
#define ACCEL1_CS_Pin GPIO_PIN_15
#define ACCEL1_CS_GPIO_Port GPIOC
#define ACCEL2_CS_Pin GPIO_PIN_2
#define ACCEL2_CS_GPIO_Port GPIOB
#define BARO_CS_Pin GPIO_PIN_9
#define BARO_CS_GPIO_Port GPIOA
#define LORA_CS_Pin GPIO_PIN_6
#define LORA_CS_GPIO_Port GPIOB
#define ALARM_PWM_Pin GPIO_PIN_7
#define ALARM_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

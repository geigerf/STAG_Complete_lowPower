/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f7xx_lp_modes.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim6;
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
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOJ
#define Dmux_sel1_Pin GPIO_PIN_8
#define Dmux_sel1_GPIO_Port GPIOC
#define Mux_sel0_Pin GPIO_PIN_7
#define Mux_sel0_GPIO_Port GPIOC
#define Mux_sel1_Pin GPIO_PIN_6
#define Mux_sel1_GPIO_Port GPIOC
#define Dmux_sel2_Pin GPIO_PIN_7
#define Dmux_sel2_GPIO_Port GPIOF
#define Mux_sel3_Pin GPIO_PIN_6
#define Mux_sel3_GPIO_Port GPIOF
#define Dmux_EN_Pin GPIO_PIN_4
#define Dmux_EN_GPIO_Port GPIOJ
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOJ
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOA
#define Dmux_sel3_Pin GPIO_PIN_3
#define Dmux_sel3_GPIO_Port GPIOJ
#define P_EN_Pin GPIO_PIN_6
#define P_EN_GPIO_Port GPIOH
#define Dmux_sel0_Pin GPIO_PIN_0
#define Dmux_sel0_GPIO_Port GPIOJ
#define Mux_sel2_Pin GPIO_PIN_1
#define Mux_sel2_GPIO_Port GPIOJ
/* USER CODE BEGIN Private defines */
#define DATA_SIZE 1024
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

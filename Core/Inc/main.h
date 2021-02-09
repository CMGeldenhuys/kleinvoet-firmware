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
#define OSC_LSE_IN_Pin GPIO_PIN_14
#define OSC_LSE_IN_GPIO_Port GPIOC
#define OSC_LSE_OUT_Pin GPIO_PIN_15
#define OSC_LSE_OUT_GPIO_Port GPIOC
#define OSC_HSE_IN_Pin GPIO_PIN_0
#define OSC_HSE_IN_GPIO_Port GPIOH
#define OSC_HSE_OUT_Pin GPIO_PIN_1
#define OSC_HSE_OUT_GPIO_Port GPIOH
#define SDIO_CD_Pin GPIO_PIN_0
#define SDIO_CD_GPIO_Port GPIOC
#define ADC_SAI_SD_A_Pin GPIO_PIN_1
#define ADC_SAI_SD_A_GPIO_Port GPIOC
#define GPS_TX_Pin GPIO_PIN_0
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_1
#define GPS_RX_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define LED_STATUS_Pin GPIO_PIN_5
#define LED_STATUS_GPIO_Port GPIOA
#define LED_STATUS_1_Pin GPIO_PIN_6
#define LED_STATUS_1_GPIO_Port GPIOA
#define USR_BTN_Pin GPIO_PIN_7
#define USR_BTN_GPIO_Port GPIOA
#define USR_BTN_EXTI_IRQn EXTI9_5_IRQn
#define GPS_nSAFEBOOT_Pin GPIO_PIN_0
#define GPS_nSAFEBOOT_GPIO_Port GPIOB
#define V_MONITOR_Pin GPIO_PIN_1
#define V_MONITOR_GPIO_Port GPIOB
#define GPS_WAKE_Pin GPIO_PIN_10
#define GPS_WAKE_GPIO_Port GPIOB
#define ADC_SAI_BCLK_Pin GPIO_PIN_12
#define ADC_SAI_BCLK_GPIO_Port GPIOB
#define ADC_nRST_Pin GPIO_PIN_7
#define ADC_nRST_GPIO_Port GPIOC
#define ADC_SAI_SD_B_Pin GPIO_PIN_9
#define ADC_SAI_SD_B_GPIO_Port GPIOA
#define ADC_CTRL_SCL_Pin GPIO_PIN_6
#define ADC_CTRL_SCL_GPIO_Port GPIOB
#define ADC_CTRL_SDA_Pin GPIO_PIN_7
#define ADC_CTRL_SDA_GPIO_Port GPIOB
#define ADC_SAI_FS_Pin GPIO_PIN_9
#define ADC_SAI_FS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

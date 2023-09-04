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
extern uint32_t KLEINVOET_UUID;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPS_SYNC_Pin GPIO_PIN_13
#define GPS_SYNC_GPIO_Port GPIOC
#define GPS_SYNC_EXTI_IRQn EXTI15_10_IRQn
#define OSC_LSE_IN_Pin GPIO_PIN_14
#define OSC_LSE_IN_GPIO_Port GPIOC
#define OSC_LSE_OUT_Pin GPIO_PIN_15
#define OSC_LSE_OUT_GPIO_Port GPIOC
#define OSC_HSE_IN_Pin GPIO_PIN_0
#define OSC_HSE_IN_GPIO_Port GPIOH
#define OSC_HSE_OUT_Pin GPIO_PIN_1
#define OSC_HSE_OUT_GPIO_Port GPIOH
#define EPD_NCMD_Pin GPIO_PIN_0
#define EPD_NCMD_GPIO_Port GPIOC
#define EPD_NRST_Pin GPIO_PIN_1
#define EPD_NRST_GPIO_Port GPIOC
#define EPD_BUSY_Pin GPIO_PIN_2
#define EPD_BUSY_GPIO_Port GPIOC
#define LORA_INT_Pin GPIO_PIN_3
#define LORA_INT_GPIO_Port GPIOC
#define GPS_TX_Pin GPIO_PIN_0
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_1
#define GPS_RX_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define BATT_NCHG_Pin GPIO_PIN_4
#define BATT_NCHG_GPIO_Port GPIOA
#define SRAM_SCK_Pin GPIO_PIN_5
#define SRAM_SCK_GPIO_Port GPIOA
#define SRAM_MISO_Pin GPIO_PIN_6
#define SRAM_MISO_GPIO_Port GPIOA
#define SRAM_MOSI_Pin GPIO_PIN_7
#define SRAM_MOSI_GPIO_Port GPIOA
#define CHARGE_EN2_Pin GPIO_PIN_4
#define CHARGE_EN2_GPIO_Port GPIOC
#define CHARGE_EN1_Pin GPIO_PIN_5
#define CHARGE_EN1_GPIO_Port GPIOC
#define I_CHARGE_Pin GPIO_PIN_0
#define I_CHARGE_GPIO_Port GPIOB
#define V_MONITOR_Pin GPIO_PIN_1
#define V_MONITOR_GPIO_Port GPIOB
#define LED_ORANGE_Pin GPIO_PIN_2
#define LED_ORANGE_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_10
#define LED_BLUE_GPIO_Port GPIOB
#define ADC_SAI_BCLK_Pin GPIO_PIN_12
#define ADC_SAI_BCLK_GPIO_Port GPIOB
#define CONN_SCK_Pin GPIO_PIN_13
#define CONN_SCK_GPIO_Port GPIOB
#define CONN_MISO_Pin GPIO_PIN_14
#define CONN_MISO_GPIO_Port GPIOB
#define CONN_MOSI_Pin GPIO_PIN_15
#define CONN_MOSI_GPIO_Port GPIOB
#define LORA_NSS_Pin GPIO_PIN_6
#define LORA_NSS_GPIO_Port GPIOC
#define EPD_NSS_Pin GPIO_PIN_7
#define EPD_NSS_GPIO_Port GPIOC
#define AVCC_EN_Pin GPIO_PIN_8
#define AVCC_EN_GPIO_Port GPIOA
#define ADC_SAI_SD_B_Pin GPIO_PIN_9
#define ADC_SAI_SD_B_GPIO_Port GPIOA
#define LORA_NRST_Pin GPIO_PIN_10
#define LORA_NRST_GPIO_Port GPIOA
#define USB_DN_Pin GPIO_PIN_11
#define USB_DN_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SDIO_CD_Pin GPIO_PIN_15
#define SDIO_CD_GPIO_Port GPIOA
#define SDIO_CLK_Pin GPIO_PIN_12
#define SDIO_CLK_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MAG_INT_Pin GPIO_PIN_4
#define MAG_INT_GPIO_Port GPIOB
#define USR_BTN_Pin GPIO_PIN_5
#define USR_BTN_GPIO_Port GPIOB
#define ADC_CTRL_SCL_Pin GPIO_PIN_6
#define ADC_CTRL_SCL_GPIO_Port GPIOB
#define ADC_CTRL_SDA_Pin GPIO_PIN_7
#define ADC_CTRL_SDA_GPIO_Port GPIOB
#define ADC_FS_Pin GPIO_PIN_8
#define ADC_FS_GPIO_Port GPIOB
#define ADC_SAI_FS_Pin GPIO_PIN_9
#define ADC_SAI_FS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

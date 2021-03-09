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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define COM_TX_Pin GPIO_PIN_2
#define COM_TX_GPIO_Port GPIOA
#define COM_RX_Pin GPIO_PIN_3
#define COM_RX_GPIO_Port GPIOA
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define COM_SYNC_Pin GPIO_PIN_4
#define COM_SYNC_GPIO_Port GPIOC
#define ODOM_SYNC1_Pin GPIO_PIN_5
#define ODOM_SYNC1_GPIO_Port GPIOC
#define ODOM_SYNC2_Pin GPIO_PIN_0
#define ODOM_SYNC2_GPIO_Port GPIOB
#define RC_CHAN_1_Pin GPIO_PIN_8
#define RC_CHAN_1_GPIO_Port GPIOE
#define RC_CHAN_2_Pin GPIO_PIN_9
#define RC_CHAN_2_GPIO_Port GPIOE
#define RC_CHAN_3_Pin GPIO_PIN_10
#define RC_CHAN_3_GPIO_Port GPIOE
#define RC_CHAN_4_Pin GPIO_PIN_11
#define RC_CHAN_4_GPIO_Port GPIOE
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define WENC_TACHO_RR_Pin GPIO_PIN_12
#define WENC_TACHO_RR_GPIO_Port GPIOB
#define WENC_TACHO_RR_EXTI_IRQn EXTI15_10_IRQn
#define WENC_TACHO_RL_Pin GPIO_PIN_13
#define WENC_TACHO_RL_GPIO_Port GPIOB
#define WENC_TACHO_RL_EXTI_IRQn EXTI15_10_IRQn
#define WENC_TACHO_FR_Pin GPIO_PIN_14
#define WENC_TACHO_FR_GPIO_Port GPIOB
#define WENC_TACHO_FR_EXTI_IRQn EXTI15_10_IRQn
#define WENC_TACHO_FL_Pin GPIO_PIN_15
#define WENC_TACHO_FL_GPIO_Port GPIOB
#define WENC_TACHO_FL_EXTI_IRQn EXTI15_10_IRQn
#define TERM_TX_Pin GPIO_PIN_8
#define TERM_TX_GPIO_Port GPIOD
#define TERM_RX_Pin GPIO_PIN_9
#define TERM_RX_GPIO_Port GPIOD
#define LED_GN_Pin GPIO_PIN_12
#define LED_GN_GPIO_Port GPIOD
#define LED_OR_Pin GPIO_PIN_13
#define LED_OR_GPIO_Port GPIOD
#define LED_RD_Pin GPIO_PIN_14
#define LED_RD_GPIO_Port GPIOD
#define LED_BL_Pin GPIO_PIN_15
#define LED_BL_GPIO_Port GPIOD
#define SERVO_PWM_Pin GPIO_PIN_6
#define SERVO_PWM_GPIO_Port GPIOC
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define I2S3_SCK_Pin GPIO_PIN_10
#define I2S3_SCK_GPIO_Port GPIOC
#define I2S3_SD_Pin GPIO_PIN_12
#define I2S3_SD_GPIO_Port GPIOC
#define WENC_DIR_RR_Pin GPIO_PIN_0
#define WENC_DIR_RR_GPIO_Port GPIOD
#define WENC_DIR_RL_Pin GPIO_PIN_1
#define WENC_DIR_RL_GPIO_Port GPIOD
#define WENC_DIR_FR_Pin GPIO_PIN_2
#define WENC_DIR_FR_GPIO_Port GPIOD
#define WENC_DIR_FL_Pin GPIO_PIN_3
#define WENC_DIR_FL_GPIO_Port GPIOD
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ESC_PWM_Pin GPIO_PIN_5
#define ESC_PWM_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_9
#define Audio_SDA_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

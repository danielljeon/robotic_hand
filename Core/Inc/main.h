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
#define EXTI4_INT_BNO085_Pin GPIO_PIN_4
#define EXTI4_INT_BNO085_GPIO_Port GPIOA
#define EXTI4_INT_BNO085_EXTI_IRQn EXTI4_IRQn
#define SPI1_SCK_BNO085_Pin GPIO_PIN_5
#define SPI1_SCK_BNO085_GPIO_Port GPIOA
#define SPI1_MISO_BNO085_Pin GPIO_PIN_6
#define SPI1_MISO_BNO085_GPIO_Port GPIOA
#define SPI1_MOSI_BNO085_Pin GPIO_PIN_7
#define SPI1_MOSI_BNO085_GPIO_Port GPIOA
#define GPIO_RST_BNO085_Pin GPIO_PIN_4
#define GPIO_RST_BNO085_GPIO_Port GPIOC
#define GPIO_P0_BNO085_Pin GPIO_PIN_5
#define GPIO_P0_BNO085_GPIO_Port GPIOC
#define TIM1_CH1_WS2812B_Pin GPIO_PIN_8
#define TIM1_CH1_WS2812B_GPIO_Port GPIOA
#define GPIO_SYNC_ADC_Pin GPIO_PIN_11
#define GPIO_SYNC_ADC_GPIO_Port GPIOA
#define GPIO_RSTN_ADC_Pin GPIO_PIN_12
#define GPIO_RSTN_ADC_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SPI3_CS_ADC_Pin GPIO_PIN_15
#define SPI3_CS_ADC_GPIO_Port GPIOA
#define SPI3_SCK_ADC_Pin GPIO_PIN_10
#define SPI3_SCK_ADC_GPIO_Port GPIOC
#define SPI3_MISO_ADC_Pin GPIO_PIN_11
#define SPI3_MISO_ADC_GPIO_Port GPIOC
#define SPI3_MOSI_ADC_Pin GPIO_PIN_12
#define SPI3_MOSI_ADC_GPIO_Port GPIOC
#define EXTI2_DRDY_ADC_Pin GPIO_PIN_2
#define EXTI2_DRDY_ADC_GPIO_Port GPIOD
#define EXTI2_DRDY_ADC_EXTI_IRQn EXTI2_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define GPIO_XSHUT_VL53L4CD_Pin GPIO_PIN_4
#define GPIO_XSHUT_VL53L4CD_GPIO_Port GPIOB
#define GPIO_Input_GPIO_VL53L4CD_Pin GPIO_PIN_5
#define GPIO_Input_GPIO_VL53L4CD_GPIO_Port GPIOB
#define I2C1_SCL_VL53L4CD_Pin GPIO_PIN_6
#define I2C1_SCL_VL53L4CD_GPIO_Port GPIOB
#define I2C1_SDA_VL53L4CD_Pin GPIO_PIN_7
#define I2C1_SDA_VL53L4CD_GPIO_Port GPIOB
#define SPI1_CS_BNO085_Pin GPIO_PIN_9
#define SPI1_CS_BNO085_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

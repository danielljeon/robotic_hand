/**
 *******************************************************************************
 * @file:  ads114s08_hal_spi.h
 * @brief: ADS114S08 functions: abstracting STM32 HAL: SPI.
 *******************************************************************************
 */

#ifndef ADS114S08_HAL_SPI_H
#define ADS114S08_HAL_SPI_H

/** Includes. *****************************************************************/

#include "stm32f4xx_hal.h"

/** STM32 Port and Pin Configurations. ****************************************/

extern SPI_HandleTypeDef hspi3;
#define ADS114S08_HSPI hspi3
#define ADS114S08_CS_PORT GPIOA
#define ADS114S08_CS_PIN GPIO_PIN_4

#define ADS114S08_NRESET_PORT GPIOA
#define ADS114S08_NRESET_PIN GPIO_PIN_12

#define ADS114S08_START_SYNC_PORT GPIOA
#define ADS114S08_START_SYNC_PIN GPIO_PIN_11

#define ADS114S08_DRDY_EXTI_IRQn EXTI2_IRQn
#define ADS114S08_DRDY_PORT GPIOD
#define ADS114S08_DRDY_PIN GPIO_PIN_2

/** Public variables. *********************************************************/

extern uint16_t channel_data[12];          // Final readings per channel.
extern uint32_t full_adcs_updated_counter; // Number of times all channels have
                                           // been updated.

/** User implementations of STM32 NVIC HAL (overwriting HAL). *****************/

void HAL_GPIO_EXTI_Callback_ads114s08(uint16_t GPIO_Pin);

/** Public functions. *********************************************************/

/**
 * @brief Initializes the ADS114S08 ADC.
 */
void ads114s08_init(void);

#endif

/**
 *******************************************************************************
 * @file:  ads114s0x_hal_spi.h
 * @brief: ADS114S0x functions: abstracting STM32 HAL: SPI.
 *******************************************************************************
 */

#ifndef ADS114S0X_HAL_SPI_H
#define ADS114S0X_HAL_SPI_H

/** Includes. *****************************************************************/

#include "stm32f4xx_hal.h"

/** STM32 Port and Pin Configurations. ****************************************/

extern SPI_HandleTypeDef hspi3;
#define ADS114S0X_HSPI hspi3
#define ADS114S0X_CS_PORT GPIOA
#define ADS114S0X_CS_PIN GPIO_PIN_4

#define ADS114S0X_NRESET_PORT GPIOA
#define ADS114S0X_NRESET_PIN GPIO_PIN_12

#define ADS114S0X_START_SYNC_PORT GPIOA
#define ADS114S0X_START_SYNC_PIN GPIO_PIN_11

#define ADS114S0X_DRDY_EXTI_IRQn EXTI2_IRQn
#define ADS114S0X_DRDY_PORT GPIOD
#define ADS114S0X_DRDY_PIN GPIO_PIN_2

// Channels to monitor without including reference pin.
#define NUM_CHANNELS 12 // 12 AIN channels total.

/** Public variables. *********************************************************/

extern uint16_t ads114s0x_data[NUM_CHANNELS]; // Final readings per channel.
extern uint32_t ads114s0x_update_counter; // Number of times all channels have
                                          // been updated.

/** User implementations of STM32 NVIC HAL (overwriting HAL). *****************/

/**
 * @brief Callback function called when a DMA SPI transmit completes.
 *
 * @param hspi_ptr Pointer to the SPI handle.
 */
void HAL_SPI_TxCpltCallback_ads114s0x(SPI_HandleTypeDef *hspi_ptr);

/**
 * @brief Callback function called when a DMA SPI receive completes.
 * @param hspi_ptr Pointer to the SPI handle.
 */
void HAL_SPI_RxCpltCallback_ads114s0x(SPI_HandleTypeDef *hspi_ptr);

/**
 * @brief Callback function called when a full-duplex DMA SPI completes.
 *
 * @param hspi_ptr Pointer to the SPI handle.
 */
void HAL_SPI_TxRxCpltCallback_ads114s0x(SPI_HandleTypeDef *hspi_ptr);

/**
 * @brief Callback function called when a GPIO EXTI is triggered.
 *
 * @param GPIO_Pin GPIO pin triggered.
 */
void HAL_GPIO_EXTI_Callback_ads114s0x(uint16_t GPIO_Pin);

/** Public functions. *********************************************************/

/**
 * @brief Initializes the ADS114S0x ADC.
 */
void ads114s0x_init(void);

#endif

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
#include <stdint.h>

/** STM32 Port and Pin Configurations. ****************************************/

extern SPI_HandleTypeDef hspi3;
#define ADS114S08_HSPI hspi3
#define ADS114S08_CS_PORT GPIOA
#define ADS114S08_CS_PIN GPIO_PIN_4

#define ADS114S08_NRESET_PORT GPIOA
#define ADS114S08_NRESET_PIN GPIO_PIN_12

#define ADS114S08_START_SYNC_PORT GPIOA
#define ADS114S08_START_SYNC_PIN GPIO_PIN_11

#define ADS114S08_DRDY_PORT GPIOD
#define ADS114S08_DRDY_PIN GPIO_PIN_2

/** Public functions. *********************************************************/

/**
 * @brief Reads a single register from the ADS114S08 ADC.
 *
 * @param address The register address to be read.
 * @param rx_buffer The RX data byte buffer.
 * @param rx_length The number of bytes to be read.
 *
 * This function constructs the read register command by combining the
 * ADS114S08_CMD_RREG command with the register address and then retrieves
 * the value from the ADC.
 */
void ads114s08_read_register(uint8_t address, uint8_t *rx_buffer,
                             uint8_t rx_length);

/**
 * @brief Writes a command to the ADS114S08.
 *
 * @param cmd The command byte to send.
 */
void ads114s08_write_command(uint8_t cmd);

/**
 * @brief Writes a value to a register in the ADS114S08.
 *
 * @param address The register address to write to.
 * @param value The value to be written.
 *
 * This function constructs the write register command by combining the
 * ADS114S08_CMD_WREG command with the register address and then sends the
 * value to be written.
 */
void ads114s08_write_register(uint8_t address, uint8_t value);

/**
 * @brief Initializes the ADS114S08 ADC.
 */
void ads114s08_init(void);

/**
 * @brief Reads all single ended ADC data from the ADS114S08.
 *
 * @param neg_ain The negative reference AIN to compare against.
 * @param data 12 element 16-bit array to store ADC values.
 */
void ads114s08_all_read_data(uint8_t neg_ain, uint16_t *data);

#endif

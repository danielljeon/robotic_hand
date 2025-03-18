/**
 *******************************************************************************
 * @file:  ads114s08_hal_spi.h
 * @brief: ADS114S08 functions: abstracting STM32 HAL: SPI.
 *******************************************************************************
 */

#ifndef ADS114S08_HAL_SPI_H
#define ADS114S08_HAL_SPI_H

/** Includes. *****************************************************************/

#include <stdint.h>

/** Public functions. *********************************************************/

/**
 * @brief Reads a single register from the ADS114S08 ADC.
 *
 * @param address The register address to be read.
 *
 * @retval The 8-bit value retrieved from the specified register.
 *
 * This function constructs the read register command by combining the
 * ADS114S08_CMD_RREG command with the register address and then retrieves
 * the value from the ADC.
 */
uint8_t ads114s08_read_register(uint8_t address);

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
 * @brief Reads ADC data from the ADS114S08.
 *
 * @retval 16-bit ADC conversion result.
 *
 * The response is assumed to include one status byte followed by two data bytes
 * which are combined into a 16-bit ADC result.
 */
uint32_t ads114s08_all_read_data(void);

#endif

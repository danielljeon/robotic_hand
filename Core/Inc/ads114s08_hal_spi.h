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

// Initialize the ADS114S08
void ads114s08_init(void);

// Send a command to the ADS114S08
void ads114s08_write_command(uint8_t cmd);

// Read conversion data (assumed 3 bytes; adjust if necessary)
uint32_t ads114s08_read_data(void);

#endif

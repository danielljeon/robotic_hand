/*******************************************************************************
 * @file vl53l4cd_hal_i2c.h
 * @brief VL53L4CD functions: abstracting STM32 HAL: I2C.
 *******************************************************************************
 * @note:
 * Based on STMicroelectronics's original example code found here:
 * https://www.st.com/en/embedded-software/stsw-img049.html. The original
 * license header is found below.
 *******************************************************************************
 */

/*******************************************************************************
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software component is licensed by ST under SLA0103 open source license.
 * You may obtain a copy of the License at:
 * https://www.st.com/content/ccc/resource/legal/legal_agreement/
 * license_agreement/group0/6f/17/42/59/94/31/49/6f/DM01026308/files/
 * DM01026308.pdf/jcr:content/translations/en.DM01026308.pdf
 *
 *
 *******************************************************************************
 */

#ifndef VL53L4CD_HAL_I2C_H
#define VL53L4CD_HAL_I2C_H

/** Includes. *****************************************************************/

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>

/** VL53L4CD constants. *******************************************************/

// Max I2C buffer size.
#define VL53L4CD_MAX_I2C_XFER_SIZE 8

/** STM32 port and pin configs. ***********************************************/

extern I2C_HandleTypeDef hi2c1;

// I2C.
#define VL53L4CD_HI2C hi2c1

/**
 * @brief If the macro below is defined, the device will be programmed to run
 * with I2C Fast Mode Plus (up to 1MHz). Otherwise, default max value is 400kHz.
 */
// #define VL53L4CD_I2C_FAST_MODE_PLUS

/** Public structs. ***********************************************************/

/**
 * @brief Device instance.
 */
typedef uint16_t Dev_t;

/**
 * @brief Error instance.
 */
typedef uint8_t VL53L4CD_Error;

/** Public functions. *********************************************************/

/**
 * @brief Read 32 bits through I2C.
 */
uint8_t VL53L4CD_RdDWord(uint16_t dev, uint16_t registerAddr, uint32_t *value);

/**
 * @brief Read 16 bits through I2C.
 */
uint8_t VL53L4CD_RdWord(uint16_t dev, uint16_t registerAddr, uint16_t *value);

/**
 * @brief Read 8 bits through I2C.
 */
uint8_t VL53L4CD_RdByte(uint16_t dev, uint16_t registerAddr, uint8_t *value);

/**
 * @brief Write 8 bits through I2C.
 */
uint8_t VL53L4CD_WrByte(uint16_t dev, uint16_t registerAddr, uint8_t value);

/**
 * @brief Write 16 bits through I2C.
 */
uint8_t VL53L4CD_WrWord(uint16_t dev, uint16_t RegisterAdress, uint16_t value);

/**
 * @brief Write 32 bits through I2C.
 */
uint8_t VL53L4CD_WrDWord(uint16_t dev, uint16_t RegisterAdress, uint32_t value);

/**
 * @brief Wait during N milliseconds.
 */
uint8_t WaitMs(Dev_t dev, uint32_t time_ms);

#endif

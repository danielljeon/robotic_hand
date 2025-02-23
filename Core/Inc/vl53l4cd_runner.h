/*******************************************************************************
 * @file vl53l4cd_runner.h
 * @brief VL53L4CD runner: init and data reading.
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

#ifndef VL53L4CD_HI2C_H
#define VL53L4CD_HI2C_H

/** Includes. *****************************************************************/

#include "VL53L4CD_api.h"
#include "VL53L4CD_calibration.h"
#include "stm32f4xx_hal.h"
#include "vl53l4cd_hal_i2c.h"

/** Private variables. ********************************************************/

Dev_t dev;

/** Public variables. *********************************************************/

extern uint16_t vl53l4cd_distance_mm;

/** Public functions. *********************************************************/

/**
 * @brief Initialize VL53L4CD with HAL abstraction driver.
 *
 * @return Result of BMP3 API execution status.
 * @retval == 0 -> Success.
 * @retval > 0  -> Error.
 */
VL53L4CD_Error vl53l4cd_init(void);

/**
 * @brief Update range data.
 *
 * @return Result of VL53L4CD API execution status.
 * @retval == 0 -> Success.
 * @retval > 0  -> Error.
 */
VL53L4CD_Error vl53l4cd_get_data(void);

#endif

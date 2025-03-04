/*******************************************************************************
 * @file vl53l4cd_runner.c
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

/** Includes. *****************************************************************/

#include "vl53l4cd_runner.h"

/** Public variables. *********************************************************/

Dev_t dev;
uint16_t vl53l4cd_distance_mm;

/** Public functions. *********************************************************/

VL53L4CD_Error vl53l4cd_init(void) {
  VL53L4CD_Error status = 0;
  dev = VL53L4CD_DEFAULT_I2C_ADDRESS;

  // Toggle XSHUT for reset.
  HAL_GPIO_WritePin(VL53L4CD_XSHUT_PORT, VL53L4CD_XSHUT_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(VL53L4CD_XSHUT_PORT, VL53L4CD_XSHUT_PIN, GPIO_PIN_SET);
  HAL_Delay(5);

  // Verify sensor ID.
  uint16_t sensor_id;
  status = VL53L4CD_GetSensorId(dev, &sensor_id);
  if (status || (sensor_id != 0xEBAA)) { // Sensor ID response failed.
    return status;
  }

  // Initialize sensor.
  status = VL53L4CD_SensorInit(dev);
  if (status) { // Initialization failed.
    return status;
  }

  // Configuration.

  // 50ms integration time with 100ms period.
  status = VL53L4CD_SetRangeTiming(dev, 50, 100);
  if (status) { // Configuration failed.
    return status;
  }

  // Start.
  status = VL53L4CD_StartRanging(dev);
  if (status) { // TOF ranging run failed.
    return status;
  }

  return status;
}

VL53L4CD_Error vl53l4cd_get_data(void) {
  VL53L4CD_Error status = 0;
  VL53L4CD_ResultsData_t results;
  uint8_t is_new_data_ready;

  // Poll data ready state.
  status = VL53L4CD_CheckForDataReady(dev, &is_new_data_ready);
  if (is_new_data_ready) { // New data is ready.

    // Read results.
    status = VL53L4CD_GetResult(dev, &results);
    if (results.range_status == 0) { // Valid results.
      vl53l4cd_distance_mm = results.distance_mm;
    } else {
      return status;
    }

    // Clear interrupt on the sensor.
    status = VL53L4CD_ClearInterrupt(dev);

  } else {
    HAL_Delay(5);
  }

  return status;
}

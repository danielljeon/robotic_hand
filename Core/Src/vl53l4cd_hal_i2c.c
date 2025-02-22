/*******************************************************************************
 * @file vl53l4cd_hal_i2c.c
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

/** Includes. *****************************************************************/

#include "vl53l4cd_hal_i2c.h"

/** Public functions. *********************************************************/

uint8_t VL53L4CD_RdDWord(uint16_t dev, uint16_t RegisterAdress,
                         uint32_t *value) {
  uint8_t status = 0;
  uint8_t data_write[2];
  uint8_t data_read[4];

  data_write[0] = RegisterAdress >> 8 & 0xFF;
  data_write[1] = RegisterAdress & 0xFF;
  HAL_I2C_Master_Transmit(&VL53L4CD_HI2C, dev, data_write, 2, 100);
  status = HAL_I2C_Master_Receive(&VL53L4CD_HI2C, dev, data_read, 4, 100);
  *value = data_read[0] << 24 | data_read[1] << 16 | data_read[2] << 8 |
           data_read[3];
  return status;
}

uint8_t VL53L4CD_RdWord(uint16_t dev, uint16_t RegisterAdress,
                        uint16_t *value) {
  uint8_t status = 0;
  uint8_t data_write[2];
  uint8_t data_read[2];

  data_write[0] = RegisterAdress >> 8 & 0xFF;
  data_write[1] = RegisterAdress & 0xFF;
  HAL_I2C_Master_Transmit(&VL53L4CD_HI2C, dev, data_write, 2, 100);
  status = HAL_I2C_Master_Receive(&VL53L4CD_HI2C, dev, data_read, 2, 100);
  *value = data_read[0] << 8 | data_read[1];
  return status;
}

uint8_t VL53L4CD_RdByte(uint16_t dev, uint16_t RegisterAdress, uint8_t *value) {
  uint8_t status = 0;
  uint8_t data_write[2];
  uint8_t data_read[1];

  data_write[0] = RegisterAdress >> 8 & 0xFF;
  data_write[1] = RegisterAdress & 0xFF;
  HAL_I2C_Master_Transmit(&VL53L4CD_HI2C, dev, data_write, 2, 100);
  status = HAL_I2C_Master_Receive(&VL53L4CD_HI2C, dev, data_read, 1, 100);
  *value = data_read[0];
  return status;
}

uint8_t VL53L4CD_WrByte(uint16_t dev, uint16_t RegisterAdress, uint8_t value) {
  uint8_t data_write[3];
  uint8_t status = 0;

  data_write[0] = RegisterAdress >> 8 & 0xFF;
  data_write[1] = RegisterAdress & 0xFF;
  data_write[2] = value & 0xFF;
  status = HAL_I2C_Master_Transmit(&VL53L4CD_HI2C, dev, data_write, 3, 100);
  return status;
}

uint8_t VL53L4CD_WrWord(uint16_t dev, uint16_t RegisterAdress, uint16_t value) {
  uint8_t data_write[4];
  uint8_t status = 0;
  data_write[0] = RegisterAdress >> 8 & 0xFF;
  data_write[1] = RegisterAdress & 0xFF;
  data_write[2] = value >> 8 & 0xFF;
  data_write[3] = value & 0xFF;
  status = HAL_I2C_Master_Transmit(&VL53L4CD_HI2C, dev, data_write, 4, 100);
  return status;
}

uint8_t VL53L4CD_WrDWord(uint16_t dev, uint16_t RegisterAdress,
                         uint32_t value) {
  uint8_t data_write[6];
  uint8_t status = 0;

  data_write[0] = RegisterAdress >> 8 & 0xFF;
  data_write[1] = RegisterAdress & 0xFF;
  data_write[2] = value >> 24 & 0xFF;
  data_write[3] = value >> 16 & 0xFF;
  data_write[4] = value >> 8 & 0xFF;
  data_write[5] = value & 0xFF;
  status = HAL_I2C_Master_Transmit(&VL53L4CD_HI2C, dev, data_write, 6, 100);
  return status;
}

uint8_t WaitMs(Dev_t dev, uint32_t time_ms) {
  HAL_Delay(time_ms);
  return 0;
}

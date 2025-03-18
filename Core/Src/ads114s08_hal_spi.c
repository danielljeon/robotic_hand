/**
 *******************************************************************************
 * @file:  ads114s08_hal_spi.c
 * @brief: ADS114S08 functions: abstracting STM32 HAL: SPI.
 *******************************************************************************
 */

/** Includes ******************************************************************/

#include "ads114s08_hal_spi.h"
#include "stm32f4xx_hal.h"

/** ADS114S08 Command Definitions *********************************************/

#define ADS114S08_CMD_RESET 0x06
#define ADS114S08_CMD_START 0x08
#define ADS114S08_CMD_STOP 0x0A
#define ADS114S08_CMD_RDATA 0x12
#define ADS114S08_CMD_RREG 0x20
#define ADS114S08_CMD_WREG 0x40
#define ADS114S08_CMD_STATUS 0x01

/** STM32 Port and Pin Configurations *****************************************/

extern SPI_HandleTypeDef hspi1;

#define ADS114S08_CS_PORT GPIOA
#define ADS114S08_CS_PIN GPIO_PIN_4

/** Private Functions *********************************************************/

/**
 * @brief Select the ADS114S08 chip (active low).
 */
static void ads114s08_select(void) {
  HAL_GPIO_WritePin(ADS114S08_CS_PORT, ADS114S08_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Deselect the ADS114S08 chip.
 */
static void ads114s08_deselect(void) {
  HAL_GPIO_WritePin(ADS114S08_CS_PORT, ADS114S08_CS_PIN, GPIO_PIN_SET);
}

/** Public Functions **********************************************************/

uint8_t ads114s08_read_register(const uint8_t address) {
  uint8_t cmd[2] = {0};
  uint8_t value = 0;

  /*
   * Construct the read register command:
   * First byte: 0x20 | (address & 0x1F).
   * Second byte: number of registers to read minus one (0 for one register).
   */
  cmd[0] = ADS114S08_CMD_RREG | (address & 0x1F);
  cmd[1] = 0x00;

  ads114s08_select();
  HAL_SPI_Transmit(&hspi1, cmd, 2, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &value, 1, HAL_MAX_DELAY);
  ads114s08_deselect();

  return value;
}

void ads114s08_write_command(uint8_t cmd) {
  ads114s08_select();
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  ads114s08_deselect();
}

void ads114s08_write_register(const uint8_t address, const uint8_t value) {
  uint8_t cmdBuffer[3] = {0};

  /*
   * Construct the write register command:
   * First byte: 0x40 | (address & 0x1F).
   * Second byte: number of registers to write minus one (0 for one register).
   * Third byte: the value to write.
   */
  cmdBuffer[0] = ADS114S08_CMD_WREG | (address & 0x1F);
  cmdBuffer[1] = 0x00;
  cmdBuffer[2] = value;

  ads114s08_select();
  HAL_SPI_Transmit(&hspi1, cmdBuffer, 3, HAL_MAX_DELAY);
  ads114s08_deselect();
}

void ads114s08_init(void) {
  HAL_Delay(3); // Wait at least 2.2 ms for power stabilization.

  // Reset the device.
  ads114s08_write_command(ADS114S08_CMD_RESET);
  HAL_Delay(1); // Wait for reset to complete.

  // Read the status register to check the RDY bit.
  uint8_t nrdy_bit = ads114s08_read_register(ADS114S08_CMD_STATUS) & 0x40;

  if (nrdy_bit == 0) {
    // Clear the FL_POR flag by writing to the status register if needed.
    ads114s08_write_command(ADS114S08_CMD_STATUS);

    // TODO: Configure additional registers via ads114s08_write_register()
    //       and verify settings as needed.

    // Start ADC conversions.
    ads114s08_write_command(ADS114S08_CMD_START);
  } else {
    // Handle error: device not ready.
  }
}

uint32_t ads114s08_all_read_data(void) {
  uint8_t rxBuffer[3] = {0};
  uint32_t data = 0;
  uint8_t txBuffer = ADS114S08_CMD_RDATA;

  ads114s08_select();
  // Use blocking mode to transmit the RDATA command and receive response.
  HAL_SPI_TransmitReceive(&hspi1, &txBuffer, rxBuffer, 3, HAL_MAX_DELAY);
  ads114s08_deselect();

  // Combine the two ADC data bytes into a 16-bit value.
  data = ((uint32_t)rxBuffer[1] << 8) | rxBuffer[2];

  return data;
}

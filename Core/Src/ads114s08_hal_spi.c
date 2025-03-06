/**
 *******************************************************************************
 * @file:  ads114s08_hal_spi.h
 * @brief: ADS114S08 functions: abstracting STM32 HAL: SPI.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "ads114s08_hal_spi.h"
#include "stm32f4xx_hal.h"

/** ADS114S08 constants. ******************************************************/

#define ADS114S08_CMD_RESET 0x06
#define ADS114S08_CMD_START 0x08
#define ADS114S08_CMD_STOP 0x0A
#define ADS114S08_CMD_RDATA 0x12
#define ADS114S08_CMD_RREG 0x20
#define ADS114S08_CMD_WREG 0x40

/** STM32 port and pin configs. ***********************************************/

// External SPI handle (ensure it’s properly instantiated in your project)
extern SPI_HandleTypeDef hspi1;

// SPI.
#define ADS114S08_HI2C hspi1

#define ADS114S08_CS_PORT GPIOA
#define ADS114S08_CS_PIN GPIO_PIN_4

/** Private functions. ********************************************************/

static void ads114s08_select(void) {
  HAL_GPIO_WritePin(ADS114S08_CS_PORT, ADS114S08_CS_PIN, GPIO_PIN_RESET);
}

static void ads114s08_deselect(void) {
  HAL_GPIO_WritePin(ADS114S08_CS_PORT, ADS114S08_CS_PIN, GPIO_PIN_SET);
}

uint8_t ads114s08_read_register(const uint8_t address) {
  uint8_t cmd[2]; // Read register is 2 byte command.
  uint8_t value = 0;

  // Format the read register command.
  // The command is typically: 001r rrrr -> 0x20 | (regAddress & 0x1F). The
  // second byte indicates the number of registers to read minus 1 (0 for one
  // register).
  cmd[0] = ADS114S08_CMD_RREG | (address & 0x1F);
  cmd[1] = 0x00; // Read one register.

  ads114s08_select();

  // Transmit the read register command.
  HAL_SPI_Transmit(&hspi1, cmd, 2, HAL_MAX_DELAY);

  // Receive the register value.
  HAL_SPI_Receive(&hspi1, &value, 1, HAL_MAX_DELAY);

  ads114s08_deselect();

  return value;
}

/** Public functions. *********************************************************/

void ads114s08_write_command(uint8_t cmd) {
  ads114s08_select();

  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);

  ads114s08_deselect();
}

uint32_t ads114s08_read_data(void) {
  uint8_t rxBuffer[3] = {0};
  uint32_t data = 0;

  ads114s08_select();
  // Send the RDATA command to trigger a read
  uint8_t rdataCmd = ADS114S08_CMD_RDATA;
  HAL_SPI_Transmit(&hspi1, &rdataCmd, 1, HAL_MAX_DELAY);

  // Read 3 bytes of data (the ADC outputs a 24-bit word)
  HAL_SPI_Receive(&hspi1, rxBuffer, 3, HAL_MAX_DELAY);
  ads114s08_deselect();

  // Combine the three bytes into a 24-bit value.
  // Adjust shifting if your data format is different.
  data = ((uint32_t)rxBuffer[0] << 16) | ((uint32_t)rxBuffer[1] << 8) |
         rxBuffer[2];

  return data;
}

void ads114s08_init(void) {
  ads114s08_write_command(ADS114S08_CMD_RESET); // Reset.

  HAL_Delay(1); // Delay time = 4096 * tCLK, tCLK = 1 / fCLK.

  // Start conversions
  ads114s08_write_command(ADS114S08_CMD_START); //
}

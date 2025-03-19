/**
 *******************************************************************************
 * @file:  ads114s08_hal_spi.c
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
#define ADS114S08_CMD_SYS 0x09
#define ADS114S08_CMD_RREG 0x20
#define ADS114S08_CMD_WREG 0x40
#define ADS114S08_CMD_ID 0x00
#define ADS114S08_CMD_STATUS 0x01

#define NUM_CHANNELS 12 // 12 AIN channels.

// Analog input channel definitions.
#define AIN0 0x00   // 0000: AIN0 (default).
#define AIN1 0x01   // 0001: AIN1.
#define AIN2 0x02   // 0010: AIN2.
#define AIN3 0x03   // 0011: AIN3.
#define AIN4 0x04   // 0100: AIN4.
#define AIN5 0x05   // 0101: AIN5.
#define AIN6 0x06   // 0110: AIN6 (ADS114S08 only).
#define AIN7 0x07   // 0111: AIN7 (ADS114S08 only).
#define AIN8 0x08   // 1000: AIN8 (ADS114S08 only).
#define AIN9 0x09   // 1001: AIN9 (ADS114S08 only).
#define AIN10 0x0A  // 1010: AIN10 (ADS114S08 only).
#define AIN11 0x0B  // 1011: AIN11 (ADS114S08 only).
#define AINCOM 0x0C // 1100: AINCOM.

/** STM32 Port and Pin Configurations. ****************************************/

extern SPI_HandleTypeDef hspi1;

#define ADS114S08_CS_PORT GPIOA
#define ADS114S08_CS_PIN GPIO_PIN_4

#define ADS114S08_NRESET_PORT GPIOA
#define ADS114S08_NRESET_PIN GPIO_PIN_12

/** Private Variables. ********************************************************/

const uint8_t channels[NUM_CHANNELS] = {AIN0, AIN1, AIN2, AIN3, AIN4,  AIN5,
                                        AIN6, AIN7, AIN8, AIN9, AIN10, AIN11};

/** Private Functions. ********************************************************/

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

/** Public Functions. *********************************************************/

void ads114s08_read_register(const uint8_t address, uint8_t *rx_buffer,
                             const uint8_t rx_length) {
  uint8_t cmd[2] = {0};

  /*
   * Construct the read register command:
   * First byte: 0x20 | (address & 0x1F).
   * Second byte: (number of registers to read - 1) & 0x1F.
   *    0 = Read 1 register.
   */
  cmd[0] = ADS114S08_CMD_RREG | (address & 0x1F);
  cmd[1] = (rx_length - 1) & 0x1F;

  ads114s08_select();
  HAL_SPI_Transmit(&hspi1, cmd, 2, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, rx_buffer, rx_length, HAL_MAX_DELAY);
  ads114s08_deselect();

  // Toggle ADC NRESET pin to break command transmission lock.
  HAL_GPIO_WritePin(ADS114S08_NRESET_PORT, ADS114S08_NRESET_PIN,
                    GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(ADS114S08_NRESET_PORT, ADS114S08_NRESET_PIN, GPIO_PIN_SET);
}

void ads114s08_write_command(uint8_t cmd) {
  ads114s08_select();
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  ads114s08_deselect();

  // Toggle ADC NRESET pin to break command transmission lock.
  HAL_GPIO_WritePin(ADS114S08_NRESET_PORT, ADS114S08_NRESET_PIN,
                    GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(ADS114S08_NRESET_PORT, ADS114S08_NRESET_PIN, GPIO_PIN_SET);
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
  HAL_GPIO_WritePin(ADS114S08_NRESET_PORT, ADS114S08_NRESET_PIN,
                    GPIO_PIN_RESET);
  HAL_Delay(3); // Wait at least 2.2 ms for power stabilization.
  HAL_GPIO_WritePin(ADS114S08_NRESET_PORT, ADS114S08_NRESET_PIN, GPIO_PIN_SET);

  uint8_t rx_buffer[1];

  HAL_Delay(3); // Wait at least 2.2 ms for power stabilization.

  // Read device ID.
  ads114s08_read_register(ADS114S08_CMD_ID, rx_buffer, 1);
  HAL_Delay(1); // Wait for reset to complete.

  // Reset the device.
  ads114s08_write_command(ADS114S08_CMD_RESET);
  HAL_Delay(1); // Wait for reset to complete.

  // Read the status register to check the RDY bit.
  uint8_t nrdy[1] = {0};
  ads114s08_read_register(ADS114S08_CMD_STATUS, nrdy, 1);

  if ((nrdy[0] & 0x40) == 0) {
    // Clear the FL_POR flag by writing to the status register if needed.
    ads114s08_write_register(ADS114S08_CMD_SYS, 0x0);

    // Load default config and enable SENDSTAT status byte prepending.
    ads114s08_write_register(ADS114S08_CMD_SYS, 0x11);

    // Read back configurations.
    ads114s08_read_register(ADS114S08_CMD_SYS, rx_buffer, 1);

    if (rx_buffer[0] & 0x01) { // Configuration success.
      // Start ADC conversions.
      ads114s08_write_command(ADS114S08_CMD_START);

    } else { // Configuration failure.
      // Handle error: configuration failed.
    }

  } else {
    // Handle error: device not ready.
  }
}

void ads114s08_all_read_data(const uint8_t neg_ain, uint16_t *data) {
  uint8_t rx_buffer[3] = {0};
  uint8_t tx_buffer = 0;

  for (uint8_t i = 0; i < 12; i++) {
    if (neg_ain != channels[i]) { // Not using itself as AIN reference.
      tx_buffer = (neg_ain << 4) | channels[i];
      ads114s08_write_register(ADS114S08_CMD_RDATA, tx_buffer);
      ads114s08_read_register(ADS114S08_CMD_RDATA, rx_buffer, 3);
      data[i] = ((uint32_t)rx_buffer[1] << 8) | rx_buffer[2];

    } else { // Using itself as AIN reference, ADC assumed zero.
      data[i] = 0;
    }
  }
}

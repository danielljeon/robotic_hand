/**
 *******************************************************************************
 * @file:  ads114s08_hal_spi.c
 * @brief: ADS114S08 functions: abstracting STM32 HAL: SPI.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "ads114s08_hal_spi.h"
#include <stdbool.h>

/** ADS114S08 constants. ******************************************************/

#define ADS114S08_DEVICE_ID 0x4

#define ADS114S08_CMD_RESET 0x06
#define ADS114S08_CMD_START 0x08
#define ADS114S08_CMD_STOP 0x0A
#define ADS114S08_CMD_RDATA 0x12
#define ADS114S08_CMD_RREG 0x20
#define ADS114S08_CMD_WREG 0x40

#define ADS114S08_ID_REGISTER 0x00
#define ADS114S08_STATUS_REGISTER 0x01
#define ADS114S08_INPMUX_REGISTER 0x02
#define ADS114S08_REF_REGISTER 0x05
#define ADS114S08_SYS_REGISTER 0x09

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

// Configured negative (GND) reference channel.
#define NEG_AIN AIN6

// Channels to monitor without including reference pin.
#define NUM_CHANNELS 9 // 12 AIN channels total.

// Channels to monitor array.
const uint8_t channels[NUM_CHANNELS] = {AIN0, AIN1, AIN2,  AIN3, AIN4,
                                        AIN8, AIN9, AIN10, AIN11};

/** Private Variables. ********************************************************/

volatile bool ads114s08_is_init = false;    // Flag tracks if init is complete.
volatile uint8_t current_channel_index = 0; // Tracks current channel.
volatile bool mux_settling = false;

/** Public Variables. *********************************************************/

uint16_t channel_data[NUM_CHANNELS];

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
  HAL_SPI_Transmit(&ADS114S08_HSPI, cmd, 2, HAL_MAX_DELAY);
  HAL_SPI_Receive(&ADS114S08_HSPI, rx_buffer, rx_length, HAL_MAX_DELAY);
  ads114s08_deselect();
}

void ads114s08_write_command(const uint8_t cmd) {
  ads114s08_select();
  HAL_SPI_Transmit(&ADS114S08_HSPI, &cmd, 1, HAL_MAX_DELAY);
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
  HAL_SPI_Transmit(&ADS114S08_HSPI, cmdBuffer, 3, HAL_MAX_DELAY);
  ads114s08_deselect();
}

void ads114s08_init(void) {
  uint8_t rx_buffer[1] = {0};

  HAL_Delay(3); // Wait at least 2.2 ms for power stabilization.

  // Tie the START/SYNC pin to DGND to control conversions by commands.
  HAL_GPIO_WritePin(ADS114S08_START_SYNC_PORT, ADS114S08_START_SYNC_PIN,
                    GPIO_PIN_RESET);

  // Tie the CS pin to DGND.
  HAL_GPIO_WritePin(ADS114S08_CS_PORT, ADS114S08_CS_PIN, GPIO_PIN_RESET);

  // Tie the RESET pin to IOVDD if the RESET pin is not used.
  HAL_GPIO_WritePin(ADS114S08_NRESET_PORT, ADS114S08_NRESET_PIN, GPIO_PIN_SET);

  // Initialize SPI CS.
  ads114s08_deselect();

  HAL_Delay(3); // Wait at least 2.2 ms for power stabilization.

  // Reset the device.
  ads114s08_write_command(ADS114S08_CMD_RESET);
  HAL_Delay(5); // Wait for reset to complete.

  // Read device ID.
  ads114s08_read_register(ADS114S08_ID_REGISTER, rx_buffer, 1);
  HAL_Delay(1);

  // Check expected device ID.
  if ((rx_buffer[0] & 0x7) == ADS114S08_DEVICE_ID) {

    // Read the status register to check the RDY bit.
    uint8_t nrdy[1] = {0};
    ads114s08_read_register(ADS114S08_STATUS_REGISTER, nrdy, 1);

    if ((nrdy[0] & 0x40) == 0) { // Device is ready.
      // Clear the FL_POR flag by writing to the status register if needed.
      ads114s08_write_register(ADS114S08_SYS_REGISTER, 0x0);

      // Project specific configuration.
      // Configure reference input selection as REFP1, REFN1.
      ads114s08_write_register(ADS114S08_REF_REGISTER, 0x04);

      // Start ADC conversions.
      ads114s08_write_command(ADS114S08_CMD_START);

    } else { // Device not ready.
      // Handle error: device not ready.
    }

  } else { // Device ID does not match expected.
    // Handle error: device ID does not match.
  }

  ads114s08_is_init = true; // Flag initialization as complete.
}

/** Override Functions. *******************************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ADS114S08_DRDY_PIN && ads114s08_is_init) {

    if (mux_settling) {
      // First DRDY after mux change is settling, skip reading.
      mux_settling = false;
      return;
    }

    // Transmit read data command with unified SPI CS.
    uint8_t rx_buffer[3];
    uint8_t command = ADS114S08_CMD_RDATA;

    ads114s08_select();
    HAL_SPI_Transmit(&ADS114S08_HSPI, &command, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&ADS114S08_HSPI, rx_buffer, 3, HAL_MAX_DELAY);
    ads114s08_deselect();

    // Bit merge result.
    int32_t raw = ((int32_t)rx_buffer[0] << 16) | ((int32_t)rx_buffer[1] << 8) |
                  (int32_t)rx_buffer[2];

    // Sign-extend 24-bit value.
    if (raw & 0x800000) {
      raw |= 0xFF000000;
    }

    channel_data[current_channel_index] = (uint16_t)((raw >> 8) & 0xFFFF);

    // Advance to next channel.
    current_channel_index++;
    if (current_channel_index >= NUM_CHANNELS)
      current_channel_index = 0;

    // Configure MUX.
    const uint8_t mux =
        ((channels[current_channel_index] << 4) & 0xF0) | (NEG_AIN & 0x0F);
    ads114s08_write_register(ADS114S08_INPMUX_REGISTER, mux);

    // Restart conversion to ensure DRDY toggles.
    ads114s08_write_command(ADS114S08_CMD_START);
    mux_settling = true;
  }
}

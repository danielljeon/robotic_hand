/**
 *******************************************************************************
 * @file:  ads114s0x_hal_spi.c
 * @brief: ADS114S0x functions: abstracting STM32 HAL: SPI.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "ads114s0x_hal_spi.h"
#include "filter.h"

/** ADS114S0x constants. ******************************************************/

// Datasheet defined commands.
#define ADS114S0X_CMD_RESET 0x06
#define ADS114S0X_CMD_START 0x08
#define ADS114S0X_CMD_STOP 0x0A
#define ADS114S0X_CMD_RDATA 0x12
#define ADS114S0X_CMD_RREG 0x20
#define ADS114S0X_CMD_WREG 0x40

// Datasheet defined register addresses.
#define ADS114S0X_ID_REGISTER 0x00
#define ADS114S0X_STATUS_REGISTER 0x01
#define ADS114S0X_INPMUX_REGISTER 0x02
#define ADS114S0X_DATARATE_REGISTER 0x04
#define ADS114S0X_REF_REGISTER 0x05
#define ADS114S0X_SYS_REGISTER 0x09

// Datasheet defined analog input channel definitions.
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

// Datasheet defined samples per second (SPS) definitions.
#define SPS_2_5 0x00      // 0000 : 2.5 SPS.
#define SPS_5 0x01        // 0001 : 5 SPS.
#define SPS_10 0x02       // 0010 : 10 SPS.
#define SPS_16_6 0x03     // 0011 : 16.6 SPS.
#define SPS_20 0x04       // 0100 : 20 SPS (default).
#define SPS_50 0x05       // 0101 : 50 SPS.
#define SPS_60 0x06       // 0110 : 60 SPS.
#define SPS_100 0x07      // 0111 : 100 SPS.
#define SPS_200 0x08      // 1000 : 200 SPS.
#define SPS_400 0x09      // 1001 : 400 SPS.
#define SPS_800 0x0A      // 1010 : 800 SPS.
#define SPS_1000 0x0B     // 1011 : 1000 SPS.
#define SPS_2000 0x0C     // 1100 : 2000 SPS.
#define SPS_4000 0x0D     // 1101 : 4000 SPS.
#define SPS_4000_ALT 0x0E // 1110 : 4000 SPS (alternate code).

/** Definitions. **************************************************************/

// SPI DMA based buffer size.
#define SPI_DMA_BUFFER_SIZE 2

// User configured negative (GND) reference channel.
#define NEG_AIN AINCOM

// User configured sample rate.
#define SAMPLE_RATE SPS_4000

/** Private types. ************************************************************/

/**
 * @brief State machine to manage ADC (multi)channel reading.
 *
 * States are required to manage the firmware between GPIO EXTI, DMA interrupts,
 * etc.
 */
typedef enum {
  ADS114S0X_STARTUP = 0,
  ADS114S0X_INIT = 1,
  ADS114S0X_AWAIT_DATA_READY = 2,
  ADS114S0X_AWAIT_DATA_RX = 3,
  ADS114S0X_DATA_RECEIVED = 4,
  ADS114S0X_ERROR = 5,
} ads114s0x_state_t;

typedef enum {
  SETTLING = 0,
  SETTLED = 1,
} ads114s0x_mux_state_t;

/** Private Variables. ********************************************************/

// Channels to monitor array.
const static uint8_t channels[NUM_CHANNELS] = {
    AIN0, AIN1, AIN2, AIN3, AIN4, AIN5, AIN6, AIN7, AIN8, AIN9, AIN10, AIN11};

// Respective channel moving average windows and combined array.
static moving_average_filter_t ain0_filter = {0};
static moving_average_filter_t ain1_filter = {0};
static moving_average_filter_t ain2_filter = {0};
static moving_average_filter_t ain3_filter = {0};
static moving_average_filter_t ain4_filter = {0};
static moving_average_filter_t ain5_filter = {0};
static moving_average_filter_t ain6_filter = {0};
static moving_average_filter_t ain7_filter = {0};
static moving_average_filter_t ain8_filter = {0};
static moving_average_filter_t ain9_filter = {0};
static moving_average_filter_t ain10_filter = {0};
static moving_average_filter_t ain11_filter = {0};
static moving_average_filter_t *filter_array[NUM_CHANNELS] = {
    &ain0_filter, &ain1_filter, &ain2_filter,  &ain3_filter,
    &ain4_filter, &ain5_filter, &ain6_filter,  &ain7_filter,
    &ain8_filter, &ain9_filter, &ain10_filter, &ain11_filter,
};

static uint8_t spi_rx_dma_buffer[SPI_DMA_BUFFER_SIZE];

// SPI bus access state machine.
static volatile ads114s0x_state_t state = ADS114S0X_STARTUP;
static volatile ads114s0x_mux_state_t mux_state = SETTLING;

// Store current channel index for input channel mux management.
static volatile uint8_t current_channel_index = 0; // Tracks current channel.

/** Public Variables. *********************************************************/

uint16_t ads114s0x_data[NUM_CHANNELS];
uint32_t ads114s0x_update_counter = 0;

/** Private Functions. ********************************************************/

/**
 * @brief Select the ADS114S0x chip (active low).
 */
static void ads114s0x_select(void) {
  HAL_GPIO_WritePin(ADS114S0X_CS_PORT, ADS114S0X_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Deselect the ADS114S0x chip.
 */
static void ads114s0x_deselect(void) {
  HAL_GPIO_WritePin(ADS114S0X_CS_PORT, ADS114S0X_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief Reads a single register from the ADS114S0x ADC.
 *
 * @param address The register address to be read.
 * @param rx_buffer The RX data byte buffer.
 * @param rx_length The number of bytes to be read.
 *
 * This function constructs the read register command by combining the
 * ADS114S0X_CMD_RREG command with the register address and then retrieves
 * the value from the ADC.
 *
 * @note SPI CS pin must be controlled externally.
 */
void ads114s0x_read_register(const uint8_t address, uint8_t *rx_buffer,
                             const uint8_t rx_length) {
  uint8_t cmd[2] = {0};

  /*
   * Construct the read register command:
   * First byte: 0x20 | (address & 0x1F).
   * Second byte: (number of registers to read - 1) & 0x1F.
   *    0 = Read 1 register.
   */
  cmd[0] = ADS114S0X_CMD_RREG | (address & 0x1F);
  cmd[1] = (rx_length - 1) & 0x1F;

  HAL_SPI_Transmit(&ADS114S0X_HSPI, cmd, 2, HAL_MAX_DELAY);
  HAL_SPI_Receive(&ADS114S0X_HSPI, rx_buffer, rx_length, HAL_MAX_DELAY);
}

/**
 * @brief Writes a command to the ADS114S0x.
 *
 * @param cmd The command byte to send.
 *
 * @note SPI CS pin must be controlled externally.
 */
void ads114s0x_write_command(const uint8_t cmd) {
  HAL_SPI_Transmit(&ADS114S0X_HSPI, &cmd, 1, HAL_MAX_DELAY);
}

/**
 * @brief Writes a value to a register in the ADS114S0x.
 *
 * @param address The register address to write to.
 * @param value The value to be written.
 *
 * This function constructs the write register command by combining the
 * ADS114S0X_CMD_WREG command with the register address and then sends the
 * value to be written.
 *
 * @note SPI CS pin must be controlled externally.
 */
void ads114s0x_write_register(const uint8_t address, const uint8_t value) {
  uint8_t cmdBuffer[3] = {0};

  /*
   * Construct the write register command:
   * First byte: 0x40 | (address & 0x1F).
   * Second byte: number of registers to write minus one (0 for one register).
   * Third byte: the value to write.
   */
  cmdBuffer[0] = ADS114S0X_CMD_WREG | (address & 0x1F);
  cmdBuffer[1] = 0x00;
  cmdBuffer[2] = value;

  HAL_SPI_Transmit(&ADS114S0X_HSPI, cmdBuffer, 3, HAL_MAX_DELAY);
}

/**
 * @brief Reset and send configuration commands.
 *
 * Runs all steps to reset, configure and restart the ADS114S0x.
 */
void ads114s0x_configure(void) {
  ads114s0x_select(); // Bring CS pin low.

  HAL_SPI_Transmit(&ADS114S0X_HSPI, (const uint8_t *)ADS114S0X_CMD_RESET, 1,
                   HAL_MAX_DELAY); // Reset the device.

  HAL_Delay(5); // Wait 4096 * tCLK.

  uint8_t nrdy[1] = {0}; // Read the status register to check the RDY bit.

  // Read status register to ensure device is ready.
  ads114s0x_read_register(ADS114S0X_STATUS_REGISTER, nrdy, 1);

  if ((nrdy[0] & 0x40) == 0) { // Device is ready.
    // Clear the FL_POR flag by writing to the status register if needed.
    ads114s0x_write_register(ADS114S0X_SYS_REGISTER, 0x0);

    // Configure data rate for default config with modified samples per second.
    ads114s0x_write_register(ADS114S0X_DATARATE_REGISTER, 0x10 | SAMPLE_RATE);

    // Configure reference input selection as REFP0, REFN0.
    ads114s0x_write_register(ADS114S0X_REF_REGISTER, 0x00);

    ads114s0x_write_command(ADS114S0X_CMD_START); // Start ADC conversions.
  }

  ads114s0x_deselect(); // Bring CS pin high.
}

/**
 * @brief Configure mux for given channel.
 *
 * @param ain_channel Channel to configure input channel mux.
 */
void ads114s0x_mux(uint8_t ain_channel) {
  ads114s0x_select(); // Bring CS pin low.

  // Configure mux.
  const uint8_t mux = ((ain_channel << 4) & 0xF0) | (NEG_AIN & 0x0F);
  ads114s0x_write_register(ADS114S0X_INPMUX_REGISTER, mux);

  ads114s0x_write_command(ADS114S0X_CMD_START); // Restart conversion.

  ads114s0x_deselect(); // Bring CS pin high.

  mux_state = SETTLING; // Update mux state to settling.
}

/** Public Functions. *********************************************************/

void ads114s0x_init(void) {
  state = ADS114S0X_INIT; // Update state for initializing.

  // Initialize channel filters.
  init_filter(&ain8_filter);
  init_filter(&ain9_filter);

  // Hardware reset on startup.
  HAL_GPIO_WritePin(ADS114S0X_NRESET_PORT, ADS114S0X_NRESET_PIN,
                    GPIO_PIN_RESET);

  HAL_Delay(3); // Wait at least 2.2 ms for power stabilization.

  // Tie the START/SYNC pin to DGND to control conversions by commands.
  HAL_GPIO_WritePin(ADS114S0X_START_SYNC_PORT, ADS114S0X_START_SYNC_PIN,
                    GPIO_PIN_RESET);

  // Tie the RESET pin to IOVDD if the RESET pin is not used (going further).
  HAL_GPIO_WritePin(ADS114S0X_NRESET_PORT, ADS114S0X_NRESET_PIN, GPIO_PIN_SET);

  HAL_Delay(3); // Wait at least 2.2 ms for power stabilization.

  ads114s0x_configure(); // Reset, configure and restart.

  HAL_Delay(1); // Wait td(CSSC) = 20 ns.

  state = ADS114S0X_AWAIT_DATA_READY; // Wait for data ready interrupt.
}

/** Override Functions. *******************************************************/

void HAL_SPI_RxCpltCallback_ads114s0x(SPI_HandleTypeDef *hspi_ptr) {
  if (hspi_ptr == &ADS114S0X_HSPI && state == ADS114S0X_AWAIT_DATA_RX &&
      mux_state == SETTLED) {
    ads114s0x_deselect(); // Bring CS pin high.

    // Store result using moving average filter.
    ads114s0x_data[current_channel_index] =
        update_filter(filter_array[current_channel_index],
                      (float)((uint16_t)spi_rx_dma_buffer[0] << 8 |
                              (uint16_t)spi_rx_dma_buffer[1]));

    // Advance to next channel.
    current_channel_index++;
    if (current_channel_index >= NUM_CHANNELS) {
      current_channel_index = 0;
      ads114s0x_update_counter++; // Increment when all channels are updated.
    }

    ads114s0x_mux(channels[current_channel_index]); // Configure mux.

    state = ADS114S0X_AWAIT_DATA_READY; // Wait for next data ready.
  }
}

void HAL_GPIO_EXTI_Callback_ads114s0x(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ADS114S0X_DRDY_PIN && state == ADS114S0X_AWAIT_DATA_READY) {

    // First DRDY after mux change is assumed to be settling, skip reading.
    if (mux_state == SETTLING) {
      mux_state = SETTLED; // Update state to mark mux has settled.
      return;
    }

    ads114s0x_select(); // Bring CS pin low.

    // Trigger start reading via SPI DMA receive.
    ads114s0x_write_command(ADS114S0X_CMD_RDATA);
    HAL_SPI_Receive_DMA(&ADS114S0X_HSPI, spi_rx_dma_buffer,
                        SPI_DMA_BUFFER_SIZE);
    state = ADS114S0X_AWAIT_DATA_RX; // Mark data as ready.
  }
}

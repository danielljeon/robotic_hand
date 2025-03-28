/*******************************************************************************
 * @file callbacks.c
 * @brief STM32 HAL callback implementations overriding weak declarations.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "callbacks.h"
#include "ads114s0x_hal_spi.h"
#include "sh2_hal_spi.h"

/** Public variables. *********************************************************/

bool read_imu_flag = false;

/** Private variables. ********************************************************/

volatile uint32_t prev_adcs_updated_counter = 0;

/** Collection of user implementations of STM32 NVIC HAL (overwriting HAL). ***/

void HAL_GPIO_EXTI_Callback(uint16_t n) {
  HAL_GPIO_EXTI_Callback_ads114s0x(n);
  if (read_imu_flag) {
    HAL_GPIO_EXTI_Callback_bno085(n);
  }
}

/** Collection of user implementations of STM32 SPI HAL (overwriting HAL). ****/

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi_ptr) {
  HAL_SPI_RxCpltCallback_ads114s0x(hspi_ptr);
}

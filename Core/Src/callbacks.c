/*******************************************************************************
 * @file callbacks.c
 * @brief STM32 HAL callback implementations overriding weak declarations.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "callbacks.h"
#include "ads114s08_hal_spi.h"
#include "sh2_hal_spi.h"

/** Public variables. *********************************************************/

bool read_imu_flag = false;

/** Private variables. ********************************************************/

volatile uint32_t prev_adcs_updated_counter = 0;

/** Collection of user implementations of STM32 NVIC HAL (overwriting HAL). ***/

void HAL_GPIO_EXTI_Callback(uint16_t n) {
  HAL_GPIO_EXTI_Callback_ads114s08(n);
  if (read_imu_flag) {
    HAL_GPIO_EXTI_Callback_bno085(n);
  }
}

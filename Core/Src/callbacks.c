/*******************************************************************************
 * @file callbacks.c
 * @brief STM32 HAL callback implementations overriding weak declarations.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "ads114s08_hal_spi.h"
#include "sh2_hal_spi.h"
#include "stm32f4xx_hal.h"

/** Collection of user implementations of STM32 NVIC HAL (overwriting HAL). ***/
void HAL_GPIO_EXTI_Callback(uint16_t n) {
  HAL_GPIO_EXTI_Callback_ads114s08(n);
  HAL_GPIO_EXTI_Callback_bno085(n);
}

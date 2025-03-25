/*******************************************************************************
 * @file sh2_hal_spi.h
 * @brief BNO085 SH2 functions: abstracting STM32 HAL: SPI.
 *******************************************************************************
 * @note
 * Developed using https://github.com/ceva-dsp/sh2-demo-nucleo as reference.
 *******************************************************************************
 */

#ifndef SH2_HAL_SPI_H
#define SH2_HAL_SPI_H

/** Includes. *****************************************************************/

#include "sh2_hal.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_tim.h"

/** STM32 port and pin configs. ***********************************************/

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim5;

// SPI.
#define SH2_HSPI hspi1
#define SH2_HSPI_IRQ SPI1_IRQn // Global SPI interrupt.
#define SH2_CSN_PORT GPIOB
#define SH2_CSN_PIN GPIO_PIN_9

// Timer for signals.
#define SH2_HTIM htim5

// GPIO_EXTI for INTN.
#define SH2_INTN_EXTI_IRQ EXTI4_IRQn
#define SH2_INTN_PORT GPIOA
#define SH2_INTN_PIN GPIO_PIN_4

// GPIO output for wake/1 of 2 communication peripheral selection pins.
#define SH2_PS0_WAKEN_PORT GPIOC
#define SH2_PS0_WAKEN_PIN GPIO_PIN_5

// GPIO output for reset.
#define SH2_RSTN_PORT GPIOC
#define SH2_RSTN_PIN GPIO_PIN_4

/** Definitions. **************************************************************/

// Keep reset asserted this long.
// (Some targets have a long RC decay on reset.)
#define RESET_DELAY_US (10000)

// Timeout time to see first interrupt from SH.
#define START_DELAY_US (2000000)

// How many bytes to read when reading the length field.
#define READ_LEN (4)

// Macros.
#define ARRAY_LEN(a) ((sizeof(a)) / (sizeof(a[0])))

/** User implementations of STM32 NVIC HAL (overwriting HAL). *****************/

void HAL_GPIO_EXTI_Callback_bno085(uint16_t n);

/** Public functions. *********************************************************/

/**
 * @brief SH2 SPI HAL open method.
 *
 * @param self SH2 HAL instance pointer.
 */
static int sh2_spi_hal_open(sh2_Hal_t *self);

/**
 * @brief SH2 SPI HAL close method.
 *
 * @param self SH2 HAL instance pointer.
 */
static void sh2_spi_hal_close(sh2_Hal_t *self);

/**
 * @brief SH2 SPI HAL read function.
 *
 * @param self SH2 HAL instance pointer.
 * @param pBuffer Pointer to the data buffer to store the read data.
 * @param len Number of bytes to read.
 * @param t Timestamp to the time the SH2 interrupt was detected.
 *
 * @return Status of execution.
 * @retval == 1 -> Success.
 * @retval <= 0 -> Failure/incomplete SHTP transfer.
 */
static int sh2_spi_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                            uint32_t *t);

/**
 * @brief SH2 SPI HAL write function.
 *
 * @param self SH2 HAL instance pointer.
 * @param pBuffer Pointer to the data buffer whose value is to be written.
 * @param len Number of bytes to write.
 *
 * @return Status of execution.
 * @retval == 1 -> Success.
 * @retval <= 0 -> Failure/incomplete SHTP transfer.
 */
static int sh2_spi_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);

/**
 * @brief SH2 implementation for the current time in us.
 *
 * @param self SH2 HAL instance pointer.
 *
 * @return Status of execution.
 */
static uint32_t sh2_spi_hal_get_time_us(sh2_Hal_t *self);

/**
 * @brief STM32 HAL abstraction initialization.
 *
 * @return Status of execution.
 */
sh2_Hal_t *sh2_hal_init(void);

#endif

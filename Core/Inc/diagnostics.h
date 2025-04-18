/*******************************************************************************
 * @file diagnostics.h
 * @brief Centralized high level diagnostics module.
 *******************************************************************************
 */

#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

/** Includes. *****************************************************************/

#include "stm32f4xx_hal.h"

/** Public variables. *********************************************************/

extern uint8_t bno085_fault_count;

/** Public functions. *********************************************************/

void bno085_fault(void);

#endif

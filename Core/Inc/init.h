/*******************************************************************************
 * @file init.h
 * @brief Centralized init logic running in main.c.
 *******************************************************************************
 */

#ifndef NERVE__INIT_H
#define NERVE__INIT_H

/** Includes. *****************************************************************/

#include "ads114s08_hal_spi.h"
#include "bno085_runner.h"
#include "scheduler.h"
#include "stepper.h"
#include "vl53l4cd_runner.h"
#include "ws2812b_hal_pwm.h"
#include "xbee_api_hal_uart.h"
#include <stdio.h>
#include <string.h>

/** Public functions. *********************************************************/

void init(void);

#endif

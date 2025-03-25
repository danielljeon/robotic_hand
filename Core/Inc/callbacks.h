/*******************************************************************************
 * @file callbacks.h
 * @brief STM32 HAL callback implementations overriding weak declarations.
 *******************************************************************************
 */

#ifndef CALLBACKS_H
#define CALLBACKS_H

/** Includes. *****************************************************************/

#include "stdbool.h"

/** Public variables. *********************************************************/

extern bool read_analog_flag; // Mark when analog reading ISR is enabled.
extern bool read_imu_flag;    // Mark when IMU reading ISR is enabled.

#endif

/**
 *******************************************************************************
 * @file:  stepper.h
 * @brief: 28BYJ-48 stepper motor functions.
 *******************************************************************************
 */

#ifndef STEPPER_H
#define STEPPER_H

/** Includes. *****************************************************************/

#include "stm32f4xx_hal.h"

/** Definitions. **************************************************************/

// Define the number of steps in our half-step sequence.
#define STEPPER_SEQUENCE_SIZE 8

/** Public structs. ***********************************************************/

/**
 * @brief Motor structure to hold a stepper motor's coil configuration.
 */
typedef struct {
  GPIO_TypeDef *coil_ports[4]; // GPIO ports for each coil.
  uint16_t coil_pins[4];       // GPIO pins for each coil.
  uint8_t current_step;        // Current step index (0..7 for half-step).
} stepper_motor_t;

/** Public functions. *********************************************************/

/**
 * @brief Initialize the stepper motor (resets current_step to 0).
 *
 * @param motor Pointer to a stepper_motor_t instance.
 */
void stepper_init(stepper_motor_t *motor);

/**
 * @brief Move the motor using half-step mode (8-step sequence).
 *
 * @param motor Pointer to the motor instance.
 * @param steps Number of steps to move (positive forward, negative backward).
 * @param delay_ms Delay (ms) between each step.
 */
void stepper_half_step(stepper_motor_t *motor, int steps, uint32_t delay_ms);

/**
 * @brief Move the motor using full-step mode (4-step sequence).
 *
 * @param motor Pointer to the motor instance.
 * @param steps Number of steps to move (positive forward, negative backward).
 * @param delay_ms Delay (ms) between each step.
 */
void stepper_full_step(stepper_motor_t *motor, int steps, uint32_t delay_ms);

#endif

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

/** Public stepper definitions. ***********************************************/

#define STEPPER_1_1_PORT GPIOC
#define STEPPER_1_1_PIN GPIO_PIN_0
#define STEPPER_1_2_PORT GPIOC
#define STEPPER_1_2_PIN GPIO_PIN_3
#define STEPPER_1_3_PORT GPIOC
#define STEPPER_1_3_PIN GPIO_PIN_2
#define STEPPER_1_4_PORT GPIOC
#define STEPPER_1_4_PIN GPIO_PIN_1

#define STEPPER_2_1_PORT GPIOA
#define STEPPER_2_1_PIN GPIO_PIN_0
#define STEPPER_2_2_PORT GPIOA
#define STEPPER_2_2_PIN GPIO_PIN_3
#define STEPPER_2_3_PORT GPIOA
#define STEPPER_2_3_PIN GPIO_PIN_2
#define STEPPER_2_4_PORT GPIOA
#define STEPPER_2_4_PIN GPIO_PIN_1

#define STEPPER_3_1_PORT GPIOB
#define STEPPER_3_1_PIN GPIO_PIN_0
#define STEPPER_3_2_PORT GPIOB
#define STEPPER_3_2_PIN GPIO_PIN_10
#define STEPPER_3_3_PORT GPIOB
#define STEPPER_3_3_PIN GPIO_PIN_2
#define STEPPER_3_4_PORT GPIOB
#define STEPPER_3_4_PIN GPIO_PIN_1

#define STEPPER_4_1_PORT GPIOB
#define STEPPER_4_1_PIN GPIO_PIN_15
#define STEPPER_4_2_PORT GPIOB
#define STEPPER_4_2_PIN GPIO_PIN_12
#define STEPPER_4_3_PORT GPIOB
#define STEPPER_4_3_PIN GPIO_PIN_13
#define STEPPER_4_4_PORT GPIOB
#define STEPPER_4_4_PIN GPIO_PIN_14

#define STEPPER_5_1_PORT GPIOC
#define STEPPER_5_1_PIN GPIO_PIN_6
#define STEPPER_5_2_PORT GPIOC
#define STEPPER_5_2_PIN GPIO_PIN_9
#define STEPPER_5_3_PORT GPIOC
#define STEPPER_5_3_PIN GPIO_PIN_8
#define STEPPER_5_4_PORT GPIOC
#define STEPPER_5_4_PIN GPIO_PIN_7

extern stepper_motor_t stepper1;
extern stepper_motor_t stepper2;
extern stepper_motor_t stepper3;
extern stepper_motor_t stepper4;
extern stepper_motor_t stepper5;

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
 * @param steps Number of steps to move (positive forward, negative reverse).
 * @param delay_ms Delay (ms) between each step.
 */
void stepper_half_step(stepper_motor_t *motor, int steps, uint32_t delay_ms);

/**
 * @brief Move the motor using full-step mode (4-step sequence).
 *
 * @param motor Pointer to the motor instance.
 * @param steps Number of steps to move (positive forward, negative reverse).
 * @param delay_ms Delay (ms) between each step.
 */
void stepper_full_step(stepper_motor_t *motor, int steps, uint32_t delay_ms);

/**
 * @brief Steps multiple stepper motors concurrently using full-step sequence.
 *
 * This function allows multiple stepper motors to be stepped concurrently with
 * the same delay between each step. Each motor can have a different number of
 * steps (positive for forward, negative for reverse). The function continues
 * stepping until all motor step counts reach zero.
 *
 * @param motors Array of pointers to stepper_motor_t instances.
 * @param steps  Array of steps per motor (positive forward, negative reverse).
 * @param motor_count Number of motors in the motors array.
 * @param delay_ms Delay (in milliseconds) between steps (common to all motors).
 */
void stepper_multi_full_step(stepper_motor_t **motors, int *steps,
                             size_t motor_count, uint32_t delay_ms);

#endif

/**
 *******************************************************************************
 * @file:  stepper.c
 * @brief: 28BYJ-48 stepper motor functions.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "stepper.h"
#include <stdbool.h>

/** Private variables. ********************************************************/

/* -- Full-step (4 steps) sequence (double-coil). ------------------------------
 * Two coils are energized at once:
 *   0x03 -> coils 1+2.
 *   0x06 -> coils 2+3.
 *   0x0C -> coils 3+4.
 *   0x09 -> coils 4+1.
 */
static const uint8_t full_step_sequence[4] = {
    0x03, // 0011.
    0x06, // 0110.
    0x0C, // 1100.
    0x09  // 1001.
};

/* -- Half-step (8 steps) sequence. --------------------------------------------
 * Each element is a bit mask for the 4 coils:
 * bit0=coil1, bit1=coil2, bit2=coil3, bit3=coil4.
 */
static const uint8_t half_step_sequence[8] = {
    0x01, // 0001.
    0x03, // 0011.
    0x02, // 0010.
    0x06, // 0110.
    0x04, // 0100.
    0x0C, // 1100.
    0x08, // 1000.
    0x09  // 1001.
};

/** Private functions. ********************************************************/

/**
 * @brief Set the motor coils based on a given step index and sequence pattern.
 *
 * @param motor Pointer to the motor instance.
 * @param sequence Pointer to an array of coil bitmasks.
 * @param seq_size Number of steps in the sequence (4 for full, 8 for half).
 * @param step_idx Current step index (0..seq_size-1).
 */
static void stepper_set_coils(stepper_motor_t *motor, const uint8_t *sequence,
                              uint8_t seq_size, uint8_t step_idx) {
  uint8_t pattern = sequence[step_idx % seq_size];
  for (int i = 0; i < 4; i++) {
    if (pattern & (1 << i)) {
      HAL_GPIO_WritePin(motor->coil_ports[i], motor->coil_pins[i],
                        GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(motor->coil_ports[i], motor->coil_pins[i],
                        GPIO_PIN_RESET);
    }
  }
}

/**
 * @brief Generic stepping function that increments/decrements the motor step.
 *
 * @param motor Pointer to the motor instance.
 * @param steps Number of steps to move (positive forward, negative backward).
 * @param delay_ms Delay (ms) between steps.
 * @param sequence Pointer to the coil sequence array (full_step or half_step).
 * @param seq_size Size of the sequence array (4 or 8).
 */
static void stepper_do_steps(stepper_motor_t *motor, int steps,
                             uint32_t delay_ms, const uint8_t *sequence,
                             uint8_t seq_size) {
  if (steps == 0) {
    return; // No movement requested.
  }

  // Forward steps.
  if (steps > 0) {
    for (int i = 0; i < steps; i++) {
      motor->current_step = (motor->current_step + 1) % seq_size;
      stepper_set_coils(motor, sequence, seq_size, motor->current_step);
      HAL_Delay(delay_ms);
    }
  }
  // Reverse steps.
  else { // steps < 0.
    int abs_steps = -steps;
    for (int i = 0; i < abs_steps; i++) {
      // Add (seq_size - 1) modulo seq_size to go backwards.
      motor->current_step = (motor->current_step + seq_size - 1) % seq_size;
      stepper_set_coils(motor, sequence, seq_size, motor->current_step);
      HAL_Delay(delay_ms);
    }
  }
}

/** Public functions. *********************************************************/

void stepper_init(stepper_motor_t *motor) {
  // Initialize step index and set coils to known starting state.
  motor->current_step = 0;
  // Defaults to half step sequence index zero.
  stepper_set_coils(motor, half_step_sequence, STEPPER_SEQUENCE_SIZE, 0);
}

void stepper_deinit(stepper_motor_t *motor) {
  for (int i = 0; i < 4; i++) {
    HAL_GPIO_WritePin(motor->coil_ports[i], motor->coil_pins[i],
                      GPIO_PIN_RESET);
  }
  // Optionally reset the current step index.
  motor->current_step = 0;
}

void stepper_half_step(stepper_motor_t *motor, int steps, uint32_t delay_ms) {
  stepper_do_steps(motor, steps, delay_ms, half_step_sequence,
                   STEPPER_SEQUENCE_SIZE);
}

void stepper_full_step(stepper_motor_t *motor, int steps, uint32_t delay_ms) {
  stepper_do_steps(motor, steps, delay_ms, full_step_sequence, 4);
}

void stepper_multi_full_step(stepper_motor_t **motors, int *steps,
                             const size_t motor_count,
                             const uint32_t delay_ms) {
  bool is_still_stepping = true;

  while (is_still_stepping) {
    // Assume all motors are done unless one still needs to step.
    is_still_stepping = false;

    for (size_t i = 0; i < motor_count; i++) {
      if (steps[i] != 0) {
        is_still_stepping = true;
        if (steps[i] > 0) {
          // Step forward: increment step index modulo 4 (full-step sequence).
          motors[i]->current_step = (motors[i]->current_step + 1) % 4;
          stepper_set_coils(motors[i], full_step_sequence, 4,
                            motors[i]->current_step);
          steps[i]--; // One forward step completed.
        } else {      // steps[i] < 0.
          // Step backward: subtract one step using modulo arithmetic.
          motors[i]->current_step = (motors[i]->current_step + 3) % 4;
          stepper_set_coils(motors[i], full_step_sequence, 4,
                            motors[i]->current_step);
          steps[i]++; // One reverse step completed (moving toward zero).
        }
      }
    }
    HAL_Delay(delay_ms);
  }
}

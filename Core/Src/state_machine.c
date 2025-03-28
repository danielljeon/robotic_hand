/*******************************************************************************
 * @file state_machine.c
 * @brief State machine: State machine organizing high level functionality.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "state_machine.h"
#include "ads114s0x_hal_spi.h"
#include "bno085_runner.h"
#include "pid.h"
#include "stepper.h"
#include "ws2812b_hal_pwm.h"
#include "xbee_api_hal_uart.h"
#include <stdio.h>
#include <string.h>

/** Definitions. **************************************************************/

#define XBEE_DESTINATION_64 0x0013A200425E94A0
#define XBEE_DESTINATION_16 0xFFFE

#define THUMB_ADC1_CHANNEL_INDEX 8
#define THUMB_ADC2_CHANNEL_INDEX 9

/** Stepper motor pinout. *****************************************************/

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

/** Private variables. ********************************************************/

uint8_t xbee_sensor_data_transmit_index = 0;

// Stepper motor declarations.
stepper_motor_t thumb_stepper = {
    .coil_ports = {STEPPER_1_1_PORT, STEPPER_1_1_PORT, STEPPER_1_1_PORT,
                   STEPPER_1_1_PORT},
    .coil_pins = {STEPPER_1_1_PIN, STEPPER_1_2_PIN, STEPPER_1_3_PIN,
                  STEPPER_1_4_PIN},
    .current_step = 0};
stepper_motor_t index_stepper = {
    .coil_ports = {STEPPER_2_1_PORT, STEPPER_2_1_PORT, STEPPER_2_1_PORT,
                   STEPPER_2_1_PORT},
    .coil_pins = {STEPPER_2_1_PIN, STEPPER_2_2_PIN, STEPPER_2_3_PIN,
                  STEPPER_2_4_PIN},
    .current_step = 0};
stepper_motor_t middle_stepper = {
    .coil_ports = {STEPPER_3_1_PORT, STEPPER_3_2_PORT, STEPPER_3_3_PORT,
                   STEPPER_3_4_PORT},
    .coil_pins = {STEPPER_3_1_PIN, STEPPER_3_2_PIN, STEPPER_3_3_PIN,
                  STEPPER_3_4_PIN},
    .current_step = 0};
stepper_motor_t ring_stepper = {
    .coil_ports = {STEPPER_4_1_PORT, STEPPER_4_1_PORT, STEPPER_4_1_PORT,
                   STEPPER_4_1_PORT},
    .coil_pins = {STEPPER_4_1_PIN, STEPPER_4_2_PIN, STEPPER_4_3_PIN,
                  STEPPER_4_4_PIN},
    .current_step = 0};
stepper_motor_t pinky_stepper = {
    .coil_ports = {STEPPER_5_1_PORT, STEPPER_5_1_PORT, STEPPER_5_1_PORT,
                   STEPPER_5_1_PORT},
    .coil_pins = {STEPPER_5_1_PIN, STEPPER_5_2_PIN, STEPPER_5_3_PIN,
                  STEPPER_5_4_PIN},
    .current_step = 0};

// PID set points.
float thumb_setpoint = 27000;

// PID controller declarations.
pid_controller_t thumb_pid = {.k_p = 0.3,
                              .k_i = 0,
                              .k_d = 0,
                              .tau = 0,
                              .output_min = -100,
                              .output_max = 100,
                              .integral_min = 0,
                              .integral_max = 0,
                              .T = 0.01,
                              .integrator = 0,
                              .prev_error = 0,
                              .differentiator = 0,
                              .prev_measurement = 0,
                              .out = 0};

// PID output commands.
float thumb_command = 0;

/** Public variables. *********************************************************/

robotic_hand_state_t system_state = STATE_INIT;
bool system_running = true;

/** Private functions. ********************************************************/

/**
 * @brief Sequential sensor data transmission manager.
 */
void transmit_sensor_data(char *data) {
  send(XBEE_DESTINATION_64, XBEE_DESTINATION_16, (const uint8_t *)data,
       strlen(data), 0);
}

/**
 * @brief Sequential sensor data transmission manager.
 */
void sequential_transmit_sensor_data(void) {
  char data[256];

  sprintf(data, "%f,%f,%d,%d", thumb_setpoint, thumb_command, ads114s0x_data[0],
          ads114s0x_data[1]);

  // Transmit the data after forming the string.
  transmit_sensor_data(data);
}

/** Private state handler functions. ******************************************/

robotic_hand_state_t handle_state_init(void) {
  // On-board miscellaneous components.
  ws2812b_init();
  for (uint8_t led_i = 0; led_i < LED_COUNT; led_i++) {
    ws2812b_set_colour(led_i, 4, 1, 1); // Very dim purple.
  }
  ws2812b_update();

  // Initialize sensors.
  ads114s0x_init();
  // bno085_reset(); // TODO: WIP Implementation.
  // bno085_init(); // TODO: WIP Implementation.
  // vl53l4cd_init(); // TODO: WIP Implementation.

  // Initialize PID controllers per finger.
  pid_init(&thumb_pid);

  // Initialize the stepper motors.
  stepper_init(&thumb_stepper);
  stepper_init(&index_stepper);
  stepper_init(&middle_stepper);
  stepper_init(&ring_stepper);
  stepper_init(&pinky_stepper);

  // State exit actions.
  return STATE_PID;
}

robotic_hand_state_t handle_state_pid(void) {
  // Lowest measurement.
  uint16_t measurement = 0;
  if (ads114s0x_data[THUMB_ADC1_CHANNEL_INDEX] <
      ads114s0x_data[THUMB_ADC2_CHANNEL_INDEX]) {
    measurement = ads114s0x_data[THUMB_ADC2_CHANNEL_INDEX];
  } else {
    measurement = ads114s0x_data[THUMB_ADC1_CHANNEL_INDEX];
  }

  // Update PID controllers per finger.
  thumb_command = pid_update(&thumb_pid, thumb_setpoint, measurement);

  // State exit actions.
  return STATE_UPDATE_MOTORS;
}

robotic_hand_state_t handle_state_update_motors(void) {
  // Set LED colours based on reaching target and direction.
  for (uint8_t led_i = 0; led_i < LED_COUNT; led_i++) {
    if (-0.05 < thumb_command && thumb_command < 0.05) {
      ws2812b_set_colour(led_i, 0, 20, 0); // Green LED.
    } else if (0 < thumb_command) {        // Forward.
      ws2812b_set_colour(led_i, 20, 0, 0); // Red LED.
    } else if (thumb_command < 0) {        // Backward.
      ws2812b_set_colour(led_i, 0, 0, 20); // Blue LED.
    }
  }
  ws2812b_update();

  // Stepper motor array.
  stepper_motor_t *steppers[5] = {&thumb_stepper, &index_stepper,
                                  &middle_stepper, &ring_stepper,
                                  &pinky_stepper};

  // Move steppers according to realized step.
  if (thumb_command < 0) {
    int steps[5] = {-2, -2, -2, -2, -2};
    stepper_multi_full_step(steppers, steps, 5, 1);
  } else if (thumb_command > 0) {
    int steps[5] = {2, 2, 2, 2, 2};
    stepper_multi_full_step(steppers, steps, 5, 1);
  }

  // State exit actions.
  return STATE_POST_PROCESSING;
}

robotic_hand_state_t handle_state_post_processing(void) {
  sequential_transmit_sensor_data(); // Transmit data.

  // State exit actions.
  return STATE_IDLE;
}

robotic_hand_state_t handle_state_error(void) { return STATE_IDLE; }

robotic_hand_state_t handle_state_idle(void) {
  // State exit actions.
  return STATE_PID;
}

/** Public functions. *********************************************************/

void run_state_machine(void) {
  while (system_running) {
    switch (system_state) {
    case STATE_INIT:
      system_state = handle_state_init();
      break;
    case STATE_PID:
      system_state = handle_state_pid();
      break;
    case STATE_UPDATE_MOTORS:
      system_state = handle_state_update_motors();
      break;
    case STATE_POST_PROCESSING:
      system_state = handle_state_post_processing();
      break;
    case STATE_ERROR:
      system_state = handle_state_error();
      break;
    case STATE_IDLE:
      system_state = handle_state_idle();
      break;
    default:
      system_running = false;
      break;
    }
  }
}

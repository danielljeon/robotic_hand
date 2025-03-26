/*******************************************************************************
 * @file state_machine.c
 * @brief State machine: State machine organizing high level functionality.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "state_machine.h"
#include "ads114s08_hal_spi.h"
#include "bno085_runner.h"
#include "callbacks.h"
#include "pid.h"
#include "stepper.h"
#include "ws2812b_hal_pwm.h"
#include "xbee_api_hal_uart.h"
#include <stdio.h>
#include <string.h>

/** Definitions. **************************************************************/

#define XBEE_DESTINATION_64 0x0013A200425E94A0
#define XBEE_DESTINATION_16 0xFFFE

#define FIXED_PID_TARGETS 30000

#define THUMB_ADC_CHANNEL_INDEX 0
#define INDEX_ADC_CHANNEL_INDEX 1
#define MIDDLE_ADC_CHANNEL_INDEX 2
#define RING_ADC_CHANNEL_INDEX 3
#define PINKY_ADC_CHANNEL_INDEX 4

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
float thumb_setpoint = 12000;
float index_setpoint = 12000;
float middle_setpoint = 12000;
float ring_setpoint = 12000;
float pinky_setpoint = 12000;

// PID controller declarations.
pid_controller_t thumb_pid_controller = {.k_p = 0.01,
                                         .k_i = 0,
                                         .k_d = 0.005,
                                         .tau = 0,
                                         .output_min = -2,
                                         .output_max = 2,
                                         .integral_min = 0,
                                         .integral_max = 10,
                                         .T = 0.01,
                                         .integrator = 0,
                                         .prev_error = 0,
                                         .differentiator = 0,
                                         .prev_measurement = 0,
                                         .out = 0};
pid_controller_t index_pid_controller = {.k_p = 0.01,
                                         .k_i = 0,
                                         .k_d = 0.005,
                                         .tau = 0,
                                         .output_min = -2,
                                         .output_max = 2,
                                         .integral_min = 0,
                                         .integral_max = 10,
                                         .T = 0.01,
                                         .integrator = 0,
                                         .prev_error = 0,
                                         .differentiator = 0,
                                         .prev_measurement = 0,
                                         .out = 0};
pid_controller_t middle_pid_controller = {.k_p = 0.01,
                                          .k_i = 0,
                                          .k_d = 0.005,
                                          .tau = 0,
                                          .output_min = -2,
                                          .output_max = 2,
                                          .integral_min = 0,
                                          .integral_max = 10,
                                          .T = 0.01,
                                          .integrator = 0,
                                          .prev_error = 0,
                                          .differentiator = 0,
                                          .prev_measurement = 0,
                                          .out = 0};
pid_controller_t ring_pid_controller = {.k_p = 0.01,
                                        .k_i = 0,
                                        .k_d = 0.005,
                                        .tau = 0,
                                        .output_min = -2,
                                        .output_max = 2,
                                        .integral_min = 0,
                                        .integral_max = 10,
                                        .T = 0.01,
                                        .integrator = 0,
                                        .prev_error = 0,
                                        .differentiator = 0,
                                        .prev_measurement = 0,
                                        .out = 0};
pid_controller_t pinky_pid_controller = {.k_p = 0.01,
                                         .k_i = 0,
                                         .k_d = 0.005,
                                         .tau = 0,
                                         .output_min = -2,
                                         .output_max = 2,
                                         .integral_min = 0,
                                         .integral_max = 10,
                                         .T = 0.01,
                                         .integrator = 0,
                                         .prev_error = 0,
                                         .differentiator = 0,
                                         .prev_measurement = 0,
                                         .out = 0};

// PID output commands.
float thumb_command = 0;
float index_command = 0;
float middle_command = 0;
float ring_command = 0;
float pinky_command = 0;

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

  // Reset index if out of bounds.
  if (xbee_sensor_data_transmit_index < 0 ||
      xbee_sensor_data_transmit_index > 8) {
    xbee_sensor_data_transmit_index = 0;
  }

  switch (xbee_sensor_data_transmit_index) {
  case 0:
    sprintf(data, "adc->J9=%d,J10=%d,J11=%d,J12=%d,J13=%d", channel_data[0],
            channel_data[1], channel_data[2], channel_data[3], channel_data[4]);
    break;
  case 1:
    sprintf(data, "command->t=%f,i=%f,m=%f,r=%f,p=%f", thumb_command,
            index_command, middle_command, ring_command, pinky_command);
    break;
  case 2:
    sprintf(data, "setpoint->t=%f,i=%f,m=%f,r=%f,p=%f", thumb_setpoint,
            index_setpoint, middle_setpoint, ring_setpoint, pinky_setpoint);
    break;
  case 3:
    sprintf(data, "w=%f,i=%f,j=%f,k=%f,f=%u", bno085_quaternion_real,
            bno085_quaternion_i, bno085_quaternion_j, bno085_quaternion_k,
            bno085_fault_count);
    break;
  case 4:
    sprintf(data, "accuracy_rad=%f,accuracy_deg=%f",
            bno085_quaternion_accuracy_rad, bno085_quaternion_accuracy_deg);
    break;
  case 5:
    sprintf(data, "gyro_x=%f,gyro_y=%f,gyro_z=%f", bno085_gyro_x, bno085_gyro_y,
            bno085_gyro_z);
    break;
  case 6:
    sprintf(data, "accel_x=%f,accel_y=%f,accel_z=%f", bno085_accel_x,
            bno085_accel_y, bno085_accel_z);
    break;
  case 7:
    sprintf(data, "lin_accel_x=%f,lin_accel_y=%f,lin_accel_z=%f",
            bno085_lin_accel_x, bno085_lin_accel_y, bno085_lin_accel_z);
    break;
  case 8:
    sprintf(data, "gravity_x=%f,gravity_y=%f,gravity_z=%f", bno085_gravity_x,
            bno085_gravity_y, bno085_gravity_z);
    break;
  default:
    xbee_sensor_data_transmit_index = 0;
    break; // Unknown index.
  }

  // Transmit the data after forming the string.
  transmit_sensor_data(data);

  // Increment the index and wrap around.
  xbee_sensor_data_transmit_index = (xbee_sensor_data_transmit_index + 1) % 9;
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
  ads114s08_init();
  // bno085_reset(); // TODO: WIP Implementation.
  // bno085_init(); // TODO: WIP Implementation.
  // vl53l4cd_init(); // TODO: WIP Implementation.

  // Initialize PID controllers per finger.
  pid_init(&thumb_pid_controller);
  pid_init(&index_pid_controller);
  pid_init(&middle_pid_controller);
  pid_init(&ring_pid_controller);
  pid_init(&pinky_pid_controller);

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
  // Update PID controllers per finger.
  thumb_command = pid_update(&thumb_pid_controller, thumb_setpoint,
                             channel_data[THUMB_ADC_CHANNEL_INDEX]);
  index_command = pid_update(&index_pid_controller, index_setpoint,
                             channel_data[INDEX_ADC_CHANNEL_INDEX]);
  middle_command = pid_update(&middle_pid_controller, middle_setpoint,
                              channel_data[MIDDLE_ADC_CHANNEL_INDEX]);
  ring_command = pid_update(&ring_pid_controller, ring_setpoint,
                            channel_data[RING_ADC_CHANNEL_INDEX]);
  pinky_command = pid_update(&pinky_pid_controller, pinky_setpoint,
                             channel_data[PINKY_ADC_CHANNEL_INDEX]);

  // State exit actions.
  return STATE_UPDATE_MOTORS;
}

robotic_hand_state_t handle_state_update_motors(void) {
  // Initialize arrays to help simplify loop logic.
  stepper_motor_t steppers[5] = {thumb_stepper, index_stepper, middle_stepper,
                                 ring_stepper, pinky_stepper};
  const float commands[5] = {thumb_command, index_command, middle_command,
                             ring_command, pinky_command};

  // Convert commands into command steps.
  int realized_steps[5] = {thumb_command, index_command, middle_command,
                           ring_command,
                           pinky_command}; // 2 = 1 full step, 1 = half step.
  // TODO: WIP Implementation.

  for (size_t i = 0; i < 5; i++) {
    // Set LED colours based on reaching target and direction.
    if (-0.05 < commands[i] && commands[i] < 0.05) {
      ws2812b_set_colour(i + 1, 0, 20, 0); // Green LED.
    } else if (0 < commands[i]) {          // Forward.
      ws2812b_set_colour(i + 1, 20, 0, 0); // Red LED.
    } else {                               // Backward.
      ws2812b_set_colour(i + 1, 0, 0, 20); // Blue LED.
    }
    ws2812b_update(); // Update new set colours.

    // Move stepper according to realized step (full or half step).
    if (realized_steps[i] == -2) {
      stepper_full_step(&steppers[i], -4, 1);
    } else if (realized_steps[i] == 2) {
      stepper_full_step(&steppers[i], 4, 1);
    } else if (realized_steps[i] == -1) {
      stepper_half_step(&steppers[i], -1, 1);
    } else if (realized_steps[i] == 1) {
      stepper_half_step(&steppers[i], 1, 1);
    }
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

/*******************************************************************************
 * @file init.c
 * @brief Centralized init logic running in main.c.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "init.h"

/** Definitions. **************************************************************/

#define XBEE_DESTINATION_64 0x0013A200425E94A0
#define XBEE_DESTINATION_16 0xFFFE

/** Private variables. ********************************************************/

uint8_t xbee_sensor_data_transmit_index = 0;

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
      xbee_sensor_data_transmit_index > 7) {
    xbee_sensor_data_transmit_index = 0;
  }

  // TODO: Implement centralized time metric/system and diagnostics.
  switch (xbee_sensor_data_transmit_index) {
  case 0:
    sprintf(data, "w=%f,i=%f,j=%f,k=%f,f=%u", bno085_quaternion_real,
            bno085_quaternion_i, bno085_quaternion_j, bno085_quaternion_k,
            bno085_fault_count);
    break;
  case 1:
    sprintf(data, "accuracy_rad=%f,accuracy_deg=%f",
            bno085_quaternion_accuracy_rad, bno085_quaternion_accuracy_deg);
    break;
  case 2:
    sprintf(data, "gyro_x=%f,gyro_y=%f,gyro_z=%f", bno085_gyro_x, bno085_gyro_y,
            bno085_gyro_z);
    break;
  case 3:
    sprintf(data, "accel_x=%f,accel_y=%f,accel_z=%f", bno085_accel_x,
            bno085_accel_y, bno085_accel_z);
    break;
  case 4:
    sprintf(data, "lin_accel_x=%f,lin_accel_y=%f,lin_accel_z=%f",
            bno085_lin_accel_x, bno085_lin_accel_y, bno085_lin_accel_z);
    break;
  case 5:
    sprintf(data, "gravity_x=%f,gravity_y=%f,gravity_z=%f", bno085_gravity_x,
            bno085_gravity_y, bno085_gravity_z);
    break;
  case 6:
    sprintf(data, "1=%d,2=%d,3=%d,4=%d,J8=%d,J12=%d,J13=%d,J14=%d,9=%d\n",
            channel_data[0], channel_data[1], channel_data[2], channel_data[3],
            channel_data[4], channel_data[5], channel_data[6], channel_data[7],
            channel_data[8]);
    break;
  default:
    xbee_sensor_data_transmit_index = 0;
    break; // Unknown index.
  }

  // Transmit the data after forming the string.
  transmit_sensor_data(data);

  // Increment the index and wrap around.
  xbee_sensor_data_transmit_index = (xbee_sensor_data_transmit_index + 1) % 8;
}

/** Public functions. *********************************************************/

void init(void) {
  // On-board miscellaneous components.
  ws2812b_init();
  for (uint8_t led_i = 0; led_i < LED_COUNT; led_i++) {
    ws2812b_set_colour(led_i, 4, 1, 1); // Very dim purple.
  }
  ws2812b_update();

  // Sensors.
  ads114s08_init();
  // bno085_reset(); // TODO: WIP Implementation.
  // bno085_init(); // TODO: WIP Implementation.
  // vl53l4cd_init(); // TODO: WIP Implementation.

  //  // Steppers. // TODO: WIP Implementation.
  //  // Initialize the stepper motors.
  //  stepper_init(&stepper1);
  //  stepper_init(&stepper2);
  //  stepper_init(&stepper3);
  //  stepper_init(&stepper4);
  //  stepper_init(&stepper5);
  //  // Create stepper motor array.
  //  stepper_motor_t *steppers[5] = {&stepper1, &stepper2, &stepper3,
  //  &stepper4,
  //                                  &stepper5};
  //  // Red LED.
  //  ws2812b_set_colour(0, 6, 0, 0);
  //  ws2812b_update();
  //  // Step forward.
  //  int steps_1[5] = {2048, 2048, 2048, 2048, 2048};
  //  stepper_multi_full_step(steppers, steps_1, 5, 1);
  //  // Blue LED.
  //  ws2812b_set_colour(0, 0, 0, 6);
  //  ws2812b_update();
  //  // Step reverse.
  //  int steps_2[5] = {-2048, -2048, -2048, -2048, -2048};
  //  stepper_multi_full_step(steppers, steps_2, 5, 1);
  //  // Deinitialize the stepper motors.
  //  stepper_deinit(&stepper1);
  //  stepper_deinit(&stepper2);
  //  stepper_deinit(&stepper3);
  //  stepper_deinit(&stepper4);
  //  stepper_deinit(&stepper5);

  // Scheduler.
  scheduler_init(); // Initialize scheduler.
  scheduler_add_task(sequential_transmit_sensor_data, 50);
  //  scheduler_add_task(vl53l4cd_get_data, 50); // TODO: WIP Implementation.
}

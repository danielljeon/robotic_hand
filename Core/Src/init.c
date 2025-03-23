/*******************************************************************************
 * @file init.c
 * @brief Centralized init logic running in main.c.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "init.h"

/** Definitions. **************************************************************/

#define XBEE_DESTINATION_64 0x0123456789ABCDEF
#define XBEE_DESTINATION_16 0xFFFE

/** Private variables. ********************************************************/

/** Private functions. ********************************************************/

/**
 * @brief Sequential sensor data transmission manager.
 */
void sequential_transmit_sensor_data(void) {
  char xbee_buffer[100];
  sprintf(xbee_buffer, "1=%d,2=%d,3=%d,4=%d,J8=%d,J12=%d,J13=%d,J14=%d,9=%d\n",
          channel_data[0], channel_data[1], channel_data[2], channel_data[3],
          channel_data[4], channel_data[5], channel_data[6], channel_data[7],
          channel_data[8]);
  send(0x0013A200425E94A0, 0xFFFE, (const uint8_t *)xbee_buffer,
       strlen(xbee_buffer), 0);
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
  scheduler_add_task(sequential_transmit_sensor_data, 100);
  //  scheduler_add_task(vl53l4cd_get_data, 50); // TODO: WIP Implementation.
}

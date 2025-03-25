/*******************************************************************************
 * @file state_machine.h
 * @brief State machine: State machine organizing high level functionality.
 *******************************************************************************
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

/** Includes. *****************************************************************/

#include <stdbool.h>

/** Public structs. ***********************************************************/

// Defines all states of the state machine.
typedef enum {
  STATE_INIT = 0,              // Initialize the system.
  STATE_READ_ANALOG = 1,       // Read data from analog force sensors.
  STATE_READ_IMU = 2,          // Read data from IMU.
  STATE_READ_MISC_SENSORS = 3, // Read data from additional sensors.
  STATE_PID = 4,               // Perform one cycle of PID calculations.
  STATE_UPDATE_MOTORS = 5,     // Update the motor step rates based on PID.
  STATE_POST_PROCESSING = 6,   // Handle post-PID actions such as telemetry.
  STATE_ERROR = 7,             // Handle any errors that occur during operation.
  STATE_IDLE = 8               // Keep the system idle.
} robotic_hand_state_t;

/** Public variables. *********************************************************/

extern robotic_hand_state_t system_state;
extern bool systemRunning;

/** Public functions. *********************************************************/

void run_state_machine(void);

#endif

/*******************************************************************************
 * @file filter.h
 * @brief Filter: Manages simple data filtering.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "filter.h"

/** Public functions. *********************************************************/

void init_filter(moving_average_filter_t *filter) {
  for (int i = 0; i < WINDOW_SIZE; i++) {
    filter->buffer[i] = 0.0f;
  }
  filter->index = 0;
  filter->count = 0;
  filter->sum = 0.0f;
}

float update_filter(moving_average_filter_t *filter, const float new_value) {
  // Subtract the oldest value from the sum.
  filter->sum -= filter->buffer[filter->index];

  // Replace the oldest value with the new one.
  filter->buffer[filter->index] = new_value;

  // Add the new value to the sum.
  filter->sum += new_value;

  // Move to the next index in the circular buffer.
  filter->index = (filter->index + 1) % WINDOW_SIZE;

  // Update count (max WINDOW_SIZE).
  if (filter->count < WINDOW_SIZE)
    filter->count++;

  // Return the average.
  return filter->sum / (float)filter->count;
}

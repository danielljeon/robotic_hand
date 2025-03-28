/*******************************************************************************
 * @file filter.h
 * @brief Filter: Manages simple data filtering.
 *******************************************************************************
 */

#ifndef FILTER_H
#define FILTER_H

/** Definitions. **************************************************************/

#define WINDOW_SIZE 5

/** Public types. *************************************************************/

typedef struct {
  float buffer[WINDOW_SIZE];
  int index;
  int count;
  float sum;
} moving_average_filter_t;

/** Public functions. *********************************************************/

void init_filter(moving_average_filter_t *filter);

float update_filter(moving_average_filter_t *filter, float new_value);

#endif

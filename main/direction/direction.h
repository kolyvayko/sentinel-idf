// SPDX-License-Identifier: MIT
#ifndef DIRECTION_H
#define DIRECTION_H

#include <stdbool.h>
#include <stdint.h>

#define DIRECTION_WINDOW_SIZE 5

typedef enum {
  DIRECTION_UNKNOWN = 0,
  DIRECTION_INCREASING = 1,
  DIRECTION_DECREASING = -1,
} direction_t;

typedef struct {
  int last;
  int max;
  bool active;
  direction_t direction;
  int window[DIRECTION_WINDOW_SIZE];
  int32_t window_sum;
  uint8_t window_count;
  uint8_t window_index;
} direction_state_t;

void direction_init(direction_state_t *state);

bool direction_update(direction_state_t *state, int value, int threshold,
                      int hysteresis, int *out_peak,
                      direction_t *out_direction);

#endif // DIRECTION_H

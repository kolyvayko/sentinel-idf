// SPDX-License-Identifier: MIT
#include "direction.h"
#include <stddef.h>

void direction_init(direction_state_t *state) {
  if (state == NULL) {
    return;
  }
  state->last = 0;
  state->max = 0;
  state->active = false;
  state->direction = DIRECTION_UNKNOWN;
  state->window_sum = 0;
  state->window_count = 0;
  state->window_index = 0;
  for (int i = 0; i < DIRECTION_WINDOW_SIZE; i++) {
    state->window[i] = 0;
  }
}

static int direction_smooth_update(direction_state_t *state, int value) {
  if (state->window_count < DIRECTION_WINDOW_SIZE) {
    state->window_sum += value;
    state->window[state->window_index++] = value;
    state->window_count++;
    if (state->window_index >= DIRECTION_WINDOW_SIZE) {
      state->window_index = 0;
    }
  } else {
    state->window_sum -= state->window[state->window_index];
    state->window_sum += value;
    state->window[state->window_index++] = value;
    if (state->window_index >= DIRECTION_WINDOW_SIZE) {
      state->window_index = 0;
    }
  }

  if (state->window_count == 0) {
    return value;
  }

  return (int)(state->window_sum / (int32_t)state->window_count);
}

bool direction_update(direction_state_t *state, int value, int threshold,
                      int hysteresis, int *out_peak,
                      direction_t *out_direction) {
  if (state == NULL) {
    return false;
  }

  bool event = false;
  int smoothed = direction_smooth_update(state, value);

  if (smoothed > threshold) {
    if (!state->active) {
      state->active = true;
      state->max = smoothed;
      state->direction = DIRECTION_UNKNOWN;
    }

    if (smoothed > state->max) {
      state->max = smoothed;
      state->direction = (smoothed >= state->last) ? DIRECTION_INCREASING
                                                   : DIRECTION_DECREASING;
    }

    if ((state->max - smoothed) >= hysteresis &&
        state->direction != DIRECTION_UNKNOWN) {
      event = true;
      if (out_peak != NULL) {
        *out_peak = state->max;
      }
      if (out_direction != NULL) {
        *out_direction = state->direction;
      }
      state->active = false;
    }
  } else {
    state->active = false;
  }

  state->last = smoothed;
  return event;
}

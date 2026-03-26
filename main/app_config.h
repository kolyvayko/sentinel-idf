// SPDX-License-Identifier: MIT
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define DISPLAY_UPDATE_INTERVAL 50
#define ADC_QUEUE_LENGTH 1
#define ADC_TASK_STACK_SIZE 4096
#define ADC_TASK_PRIORITY 5
#define DISPLAY_TASK_STACK_SIZE 4096
#define DISPLAY_TASK_PRIORITY 4

// --- Sentinel: Bearing ---
#define SENTINEL_FREQ_HZ                900e6f
#define SENTINEL_ANTENNA_SPACING_M      0.165f   // λ/2 @ 900 MHz
#define SENTINEL_VPHS_ZERO_MV           1790     // AD8302 VPHS at 0° phase diff (max output ≈1.79V = 1790mV)
#define SENTINEL_SIGNAL_THRESHOLD       100      // ADC counts, minimum valid signal
#define SENTINEL_ELEVATION_ENABLED      0        // Phase 1: horizontal only

// --- Sentinel: Calibration button ---
#define SENTINEL_CAL_BTN_GPIO           GPIO_NUM_9
#define SENTINEL_CAL_BTN_HOLD_MS        3000     // hold duration to trigger calibration

// --- Bearing task ---
#define BEARING_TASK_STACK_SIZE         4096
#define BEARING_TASK_PRIORITY           5

#endif // APP_CONFIG_H

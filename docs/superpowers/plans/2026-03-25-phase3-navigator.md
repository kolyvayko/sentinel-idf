# Phase 3 — Navigator + Autonomous Horizontal Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the navigator state machine (IDLE→SEARCHING→TRACKING) with P-controller that translates bearing angles into attitude commands, enabling the drone to autonomously track an RF source horizontally.

**Architecture:** New `navigator/` module owns the state machine and P-controller. It reads `bearing_t` from `g_bearing_nav_queue`, computes `nav_cmd_t`, and pushes to `g_nav_cmd_queue` (consumed by existing `mavlink_task`). The navigator also gates MAVLink commands: only sends attitude targets when state ≥ TRACKING. The display is updated to show current nav state.

**Tech Stack:** ESP-IDF ≥5.3.0, FreeRTOS, Unity tests.

**Prerequisite:** Phase 2 complete — `mavlink_task` running, FC responding in GUIDED_NOGPS.

---

## File Map

| Action | Path | Purpose |
|--------|------|---------|
| Create | `main/navigator/navigator.h` | `nav_state_t` enum, `nav_cmd_t` (move from mavlink_task.h), public API |
| Create | `main/navigator/navigator.c` | state machine + P-controller |
| Modify | `main/mavlink/mavlink_task.h` | remove `nav_cmd_t` (now in navigator.h) |
| Modify | `main/display/display.c` | show nav state string on SSD1306 |
| Modify | `main/main.c` | create navigator_task |
| Modify | `main/CMakeLists.txt` | register navigator/ |
| Create | `test/test_navigator.c` | Unity tests for P-controller output |

---

## Task 1: Create navigator/ module

**Files:**
- Create: `main/navigator/navigator.h`
- Create: `main/navigator/navigator.c`

- [ ] **Step 1: Write navigator.h**

```c
// main/navigator/navigator.h
#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "bearing/bearing.h"
#include <stdint.h>

typedef enum {
    NAV_STATE_IDLE = 0,
    NAV_STATE_SEARCHING,
    NAV_STATE_TRACKING,
    NAV_STATE_INTERCEPT,   // terminal — never exits
} nav_state_t;

typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float thrust;
} nav_cmd_t;

// Returns human-readable state name (for display)
const char *nav_state_str(nav_state_t state);

nav_state_t navigator_get_state(void);

// Pure function: compute nav_cmd from bearing and current state.
// Used both in navigator_task and in unit tests.
nav_cmd_t navigator_compute(const bearing_t *b, nav_state_t state,
                             float kp, float cruise_pitch_deg,
                             float cruise_thrust,
                             float intercept_pitch_deg, float intercept_thrust);

void navigator_task(void *param);

#endif
```

- [ ] **Step 2: Write failing tests before implementing**

```c
// test/test_navigator.c
#include "unity.h"
#include "navigator/navigator.h"
#include <math.h>

static bearing_t make_bearing(float az, float el) {
    return (bearing_t){ .azimuth_deg=az, .elevation_deg=el, .adc1_raw=500, .adc2_raw=500 };
}

// Zero bearing → zero roll, cruise pitch, cruise thrust
TEST_CASE("navigator_compute: zero bearing → cruise straight", "[navigator]") {
    bearing_t b = make_bearing(0.0f, 0.0f);
    nav_cmd_t cmd = navigator_compute(&b, NAV_STATE_TRACKING,
                                       0.02f, 5.0f, 0.55f, 20.0f, 0.85f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, cmd.roll_rad);
    // pitch = cruise_pitch_deg in radians = 5 * pi/180
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f * M_PI / 180.0f, cmd.pitch_rad);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.55f, cmd.thrust);
}

// Positive azimuth → positive roll (bank right)
TEST_CASE("navigator_compute: positive azimuth → positive roll", "[navigator]") {
    bearing_t b = make_bearing(30.0f, 0.0f);
    nav_cmd_t cmd = navigator_compute(&b, NAV_STATE_TRACKING,
                                       0.02f, 5.0f, 0.55f, 20.0f, 0.85f);
    // roll = Kp * 30° in radians
    float expected_roll = 0.02f * 30.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, expected_roll, cmd.roll_rad);
}

// INTERCEPT state → fixed max pitch + max thrust, zero roll
TEST_CASE("navigator_compute: intercept state → full thrust forward", "[navigator]") {
    bearing_t b = make_bearing(10.0f, 5.0f);
    nav_cmd_t cmd = navigator_compute(&b, NAV_STATE_INTERCEPT,
                                       0.02f, 5.0f, 0.55f, 20.0f, 0.85f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, cmd.roll_rad);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f * M_PI / 180.0f, cmd.pitch_rad);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.85f, cmd.thrust);
}

// IDLE/SEARCHING → zero attitude command
TEST_CASE("navigator_compute: idle → zero commands", "[navigator]") {
    bearing_t b = make_bearing(45.0f, 10.0f);
    nav_cmd_t cmd = navigator_compute(&b, NAV_STATE_IDLE,
                                       0.02f, 5.0f, 0.55f, 20.0f, 0.85f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, cmd.roll_rad);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, cmd.pitch_rad);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, cmd.thrust);
}

TEST_CASE("nav_state_str: returns non-null string for all states", "[navigator]") {
    TEST_ASSERT_NOT_NULL(nav_state_str(NAV_STATE_IDLE));
    TEST_ASSERT_NOT_NULL(nav_state_str(NAV_STATE_SEARCHING));
    TEST_ASSERT_NOT_NULL(nav_state_str(NAV_STATE_TRACKING));
    TEST_ASSERT_NOT_NULL(nav_state_str(NAV_STATE_INTERCEPT));
}
```

- [ ] **Step 3: Build to verify tests compile but fail (symbol undefined)**

```bash
idf.py build
```
Expected: linker error.

- [ ] **Step 4: Implement navigator.c**

```c
// main/navigator/navigator.c
#include "navigator.h"
#include "config/config.h"
#include "app_config.h"
#include "mavlink/mavlink_task.h"
#include <math.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

static const char *TAG = "NAVIGATOR";
static nav_state_t s_state = NAV_STATE_IDLE;
static uint32_t s_last_signal_ms = 0;

extern QueueHandle_t g_bearing_nav_queue;
extern QueueHandle_t g_nav_cmd_queue;

const char *nav_state_str(nav_state_t state) {
    switch (state) {
        case NAV_STATE_IDLE:      return "IDLE";
        case NAV_STATE_SEARCHING: return "SEARCHING";
        case NAV_STATE_TRACKING:  return "TRACKING";
        case NAV_STATE_INTERCEPT: return "INTERCEPT";
        default:                  return "UNKNOWN";
    }
}

nav_state_t navigator_get_state(void) { return s_state; }

nav_cmd_t navigator_compute(const bearing_t *b, nav_state_t state,
                              float kp, float cruise_pitch_deg,
                              float cruise_thrust,
                              float intercept_pitch_deg, float intercept_thrust) {
    nav_cmd_t cmd = {0};

    if (state == NAV_STATE_INTERCEPT) {
        cmd.roll_rad  = 0.0f;
        cmd.pitch_rad = intercept_pitch_deg * (M_PI / 180.0f);
        cmd.thrust    = intercept_thrust;
        return cmd;
    }

    if (state == NAV_STATE_TRACKING) {
        cmd.roll_rad  = kp * b->azimuth_deg;
#if SENTINEL_ELEVATION_ENABLED
        cmd.pitch_rad = cruise_pitch_deg * (M_PI/180.0f)
                        - kp * b->elevation_deg;
#else
        cmd.pitch_rad = cruise_pitch_deg * (M_PI / 180.0f);
#endif
        cmd.thrust    = cruise_thrust;
    }
    // IDLE, SEARCHING: return zero cmd (attitude loop in mavlink_task does nothing)
    return cmd;
}

static nav_state_t next_state(nav_state_t current, const bearing_t *b,
                               float vmag_threshold, uint32_t now_ms) {
    if (current == NAV_STATE_INTERCEPT) return NAV_STATE_INTERCEPT; // terminal

    bool has_signal = (b->adc1_raw >= SENTINEL_SIGNAL_THRESHOLD);

    switch (current) {
        case NAV_STATE_IDLE:
            // mavlink_get_state() is defined in mavlink_task.h (Phase 2).
            // MAV_CONN_GUIDED_NOGPS is the renamed enum value from mavlink_task.h
            // (renamed from MAV_STATE_GUIDED_NOGPS to avoid collision with MAVLink's MAV_STATE_*).
            if (mavlink_get_state() == MAV_CONN_GUIDED_NOGPS)
                return NAV_STATE_SEARCHING;
            break;

        case NAV_STATE_SEARCHING:
            if (has_signal) {
                s_last_signal_ms = now_ms;
                return NAV_STATE_TRACKING;
            }
            break;

        case NAV_STATE_TRACKING:
            if (!has_signal) {
                if ((now_ms - s_last_signal_ms) > MAVLINK_WATCHDOG_MS)
                    return NAV_STATE_SEARCHING;
            } else {
                s_last_signal_ms = now_ms;
                // Check intercept condition
                bool angle_close = (fabsf(b->azimuth_deg) < 5.0f);
#if SENTINEL_VMAG_ENABLED
                float vmag_norm = b->adc1_raw / 4095.0f;
                if (vmag_norm > vmag_threshold && angle_close)
                    return NAV_STATE_INTERCEPT;
#else
                // Without VMAG: use only angle threshold as proximity indicator
                // (will be refined in Phase 4)
                (void)vmag_threshold;
#endif
            }
            break;

        default: break;
    }
    return current;
}

void navigator_task(void *param) {
    bearing_t b = {0};

    for (;;) {
        if (xQueueReceive(g_bearing_nav_queue, &b, pdMS_TO_TICKS(100)) == pdPASS) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

            nav_state_t new_state = next_state(s_state, &b,
                config_get_float(CFG_VMAG_THRESHOLD), now);
            if (new_state != s_state) {
                ESP_LOGI(TAG, "state: %s → %s",
                    nav_state_str(s_state), nav_state_str(new_state));
                s_state = new_state;
            }

            nav_cmd_t cmd = navigator_compute(&b, s_state,
                config_get_float(CFG_KP),
                config_get_float(CFG_CRUISE_PITCH),
                config_get_float(CFG_CRUISE_THRUST),
                config_get_float(CFG_INTERCEPT_PITCH),
                config_get_float(CFG_INTERCEPT_THRUST));

            if (xQueueSend(g_nav_cmd_queue, &cmd, 0) != pdPASS) {
                nav_cmd_t tmp;
                xQueueReceive(g_nav_cmd_queue, &tmp, 0);
                xQueueSend(g_nav_cmd_queue, &cmd, 0);
            }
        }
    }
}
```

- [ ] **Step 5: Build and run tests**

```bash
idf.py build && idf.py flash monitor
```
Expected: `5 Tests 0 Failures 0 Ignored`

- [ ] **Step 6: Commit**

```bash
git add main/navigator/navigator.h main/navigator/navigator.c test/test_navigator.c
git commit -m "feat(navigator): state machine + P-controller with Unity tests"
```

---

## Task 2: Move nav_cmd_t, wire navigator into main

**Files:**
- Modify: `main/mavlink/mavlink_task.h` (remove nav_cmd_t, import from navigator.h)
- Modify: `main/CMakeLists.txt`
- Modify: `main/main.c`

- [ ] **Step 1: Update mavlink_task.h to use nav_cmd_t from navigator.h**

Remove `nav_cmd_t` struct from `main/mavlink/mavlink_task.h`. Add:
```c
#include "navigator/navigator.h"
```

- [ ] **Step 2: Update CMakeLists.txt**

Add `"navigator/navigator.c"` to SRCS and `"navigator"` to INCLUDE_DIRS.

- [ ] **Step 3: Verify `g_bearing_nav_queue` and `g_nav_cmd_queue` are defined in main.c**

These queues were created in Phase 1 (`g_bearing_nav_queue`) and Phase 2 (`g_nav_cmd_queue`). Confirm both `QueueHandle_t` declarations exist in `main.c` with `extern` declarations in their respective modules. No change needed if Phase 1 and 2 are complete — this is a verification step only.

- [ ] **Step 4: Add navigator_task to main.c**

In `app_main()`:
```c
#include "navigator/navigator.h"
// ...
xTaskCreate(navigator_task, "nav_task", NAVIGATOR_TASK_STACK_SIZE,
            NULL, NAVIGATOR_TASK_PRIORITY, NULL);
```

Add to app_config.h if not present:
```c
#define NAVIGATOR_TASK_STACK_SIZE  4096
#define NAVIGATOR_TASK_PRIORITY    4
```

- [ ] **Step 5: Update display_task in main.c to show nav state**

Phase 1 hardcoded `"SEARCHING"` as the state string. Replace it with the live nav state:

```c
// In display_task loop in main.c — replace hardcoded string:
display_show_bearing(disp, &b, nav_state_str(navigator_get_state()));
```

Add `#include "navigator/navigator.h"` to `main.c` if not already present.

- [ ] **Step 5: Build**

```bash
idf.py build
```
Expected: `Build successful`

- [ ] **Step 6: Flash and verify state transitions**

```bash
idf.py flash monitor
```

With FC connected in GUIDED_NOGPS:
1. No RF signal → serial shows `IDLE → SEARCHING`
2. Turn on transmitter → `SEARCHING → TRACKING`
3. Turn off transmitter, wait 3s → `TRACKING → SEARCHING`

SSD1306 shows current state on second line.

- [ ] **Step 7: Commit**

```bash
git add main/mavlink/mavlink_task.h main/CMakeLists.txt main/main.c main/app_config.h
git commit -m "feat(main): wire navigator_task, state-driven display"
```

---

## Task 3: Field test — tethered flight

No code changes. Manual test.

- [ ] **Step 1: Arm drone and verify GUIDED_NOGPS**

Arm drone. Verify Mission Planner shows mode = GUIDED_NOGPS. Serial: `GUIDED_NOGPS confirmed`.

- [ ] **Step 2: Tethered hover test (drone on a rope)**

Keep drone tethered. Turn on target transmitter at ~5m directly in front.
Expected: drone banks toward transmitter, SSD1306 shows `TRACKING`.

- [ ] **Step 3: Tune Kp via MAVLink**

In Mission Planner, set `SNT_KP = 0.01` (less aggressive). Observe reduced bank angle.
Try `SNT_KP = 0.03`. Find value where drone tracks without oscillating.

- [ ] **Step 4: Commit tuning result**

```bash
git commit --allow-empty -m "test(phase3): tethered tracking verified, Kp=0.02 stable"
```

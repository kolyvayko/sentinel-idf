# Phase 2 — MAVLink Connection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** ESP32-C6 connects to ArduPilot FC via UART, sends heartbeat, switches FC to GUIDED_NOGPS mode, and implements MAVLink PARAM protocol for all calibration parameters.

**FC connection is optional.** If the FC is not connected (no heartbeat received), `mavlink_task` waits silently — no attitude commands are sent. Bearing acquisition and OLED display run regardless of FC connection state. When FC connects, heartbeat exchange begins, mode is set to GUIDED_NOGPS, and attitude commands start flowing. If the FC heartbeat is lost, commands stop until reconnection.

**Architecture:** New `mavlink/` module owns the UART connection, heartbeat loop, mode switching, watchdog, and PARAM protocol. `config/` module gains `config_to_mavlink()` and `config_from_mavlink()` for PARAM_SET/PARAM_VALUE serialisation. `mavlink_task` runs independently, receives `nav_cmd_t` from a queue (placeholder — full navigator in Phase 3), and sends heartbeat + handles PARAM requests. `display_task` reads `mavlink_get_state()` to show FC connection status on the OLED.

**Tech Stack:** ESP-IDF ≥5.3.0, mavlink/c_library_v2 (ArduPilotMega dialect), FreeRTOS UART driver, Unity tests.

**Prerequisite:** Phase 1 complete and verified.

---

## File Map

| Action | Path | Purpose |
|--------|------|---------|
| Create | `components/mavlink/` | mavlink/c_library_v2 header-only library |
| Create | `components/mavlink/CMakeLists.txt` | register as IDF component |
| Create | `main/mavlink/mavlink_task.h` | public API: init, queue handle |
| Create | `main/mavlink/mavlink_task.c` | heartbeat, mode cmd, watchdog, PARAM protocol |
| Create | `main/mavlink/euler_to_quat.h` | pure function: euler → quaternion |
| Create | `main/mavlink/euler_to_quat.c` | implementation |
| Modify | `main/config/config.h` | add float param support + MAVLink serialisation |
| Modify | `main/config/config.c` | implement float storage, config_to/from_mavlink |
| Modify | `main/app_config.h` | add MAVLINK_* and navigator constants |
| Modify | `main/CMakeLists.txt` | register mavlink/ sources |
| Modify | `main/main.c` | create mavlink_task |
| Create | `test/test_euler_quat.c` | Unity tests for euler_to_quat |

---

## Task 1: Install mavlink/c_library_v2

**Files:**
- Create: `components/mavlink/CMakeLists.txt`

- [ ] **Step 1: Clone mavlink library**

```bash
git submodule add https://github.com/mavlink/c_library_v2.git components/mavlink/c_library_v2
```
Expected: `components/mavlink/c_library_v2/` directory with `ardupilotmega/` dialect.

- [ ] **Step 2: Write CMakeLists.txt for component**

```cmake
# components/mavlink/CMakeLists.txt
idf_component_register(
    INCLUDE_DIRS "c_library_v2"
)
```

- [ ] **Step 3: Build to verify includes work**

Create a temporary test include in `main/main.c` at the top:
```c
#include <ardupilotmega/mavlink.h>
```
Run:
```bash
idf.py build
```
Expected: `Build successful`. Remove temporary include.

- [ ] **Step 4: Commit**

```bash
git add components/mavlink/ .gitmodules
git commit -m "chore: add mavlink/c_library_v2 as submodule component"
```

---

## Task 2: Add float params + MAVLink serialisation to config/

**Files:**
- Modify: `main/config/config.h`
- Modify: `main/config/config.c`

- [ ] **Step 1: Extend cfg_param_id_t and add float API**

Replace `main/config/config.h` with:

```c
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// Forward-declare MAVLink types to avoid pulling in the full mavlink headers
// from config.h. Concrete types only used in config.c and config_mavlink helpers.
typedef struct mavlink_param_value_t mavlink_param_value_t;
typedef struct mavlink_param_set_t   mavlink_param_set_t;

// Integer params
typedef enum {
    CFG_VPHS_ZERO_MV = 0,
    CFG_VPHS_ZERO2_MV,
    CFG_SIGNAL_THRESHOLD,
    CFG_FREQ_MHZ,
    CFG_U16_COUNT
} cfg_u16_id_t;

// Float params
typedef enum {
    CFG_KP = 0,
    CFG_CRUISE_PITCH,
    CFG_CRUISE_THRUST,
    CFG_INTERCEPT_PITCH,
    CFG_INTERCEPT_THRUST,
    CFG_VMAG_THRESHOLD,
    CFG_ANT_SPACING,
    CFG_FLOAT_COUNT
} cfg_float_id_t;

void     config_init(void);
uint16_t config_get_u16(cfg_u16_id_t id);
void     config_set_u16(cfg_u16_id_t id, uint16_t value);
float    config_get_float(cfg_float_id_t id);
void     config_set_float(cfg_float_id_t id, float value);

// MAVLink PARAM protocol support
// Returns number of params serialised into out_msg (call repeatedly with index 0..N-1)
int config_total_params(void);
// Serialise param at index to PARAM_VALUE message. Returns false if index out of range.
bool config_param_value(int index, mavlink_param_value_t *out);
// Apply a PARAM_SET message. Returns true if param was found and set.
bool config_param_set(const mavlink_param_set_t *in);

void config_auto_zero(int adc1_raw, int adc2_raw);
void config_btn_task(void *param);
#endif
```

- [ ] **Step 2: Implement float storage and MAVLink serialisation in config.c**

Extend `main/config/config.c` — add float tables and implement `config_get/set_float`, `config_param_value`, `config_param_set`. Key logic:

```c
// Float param names for MAVLink (16 chars max)
static const char *s_float_mav_ids[CFG_FLOAT_COUNT] = {
    [CFG_KP]               = "SNT_KP",
    [CFG_CRUISE_PITCH]     = "SNT_CR_PITCH",
    [CFG_CRUISE_THRUST]    = "SNT_CR_THRUST",
    [CFG_INTERCEPT_PITCH]  = "SNT_INT_PITCH",
    [CFG_INTERCEPT_THRUST] = "SNT_INT_THRUST",
    [CFG_VMAG_THRESHOLD]   = "SNT_VMAG_THR",
    [CFG_ANT_SPACING]      = "SNT_ANT_SPACE",
};
static const char *s_float_nvs_keys[CFG_FLOAT_COUNT] = {
    "kp", "cr_pitch", "cr_thrust", "int_pitch", "int_thrust", "vmag_thr", "ant_space"
};
static const float s_float_defaults[CFG_FLOAT_COUNT] = {
    [CFG_KP]               = SENTINEL_KP,
    [CFG_CRUISE_PITCH]     = SENTINEL_CRUISE_PITCH_DEG,
    [CFG_CRUISE_THRUST]    = SENTINEL_CRUISE_THRUST,
    [CFG_INTERCEPT_PITCH]  = SENTINEL_INTERCEPT_PITCH_DEG,
    [CFG_INTERCEPT_THRUST] = SENTINEL_INTERCEPT_THRUST,
    [CFG_VMAG_THRESHOLD]   = SENTINEL_VMAG_INTERCEPT_THRESHOLD,
    [CFG_ANT_SPACING]      = SENTINEL_ANTENNA_SPACING_M,
};
static float s_float_values[CFG_FLOAT_COUNT];
```

`config_param_value(index, out)`: iterate u16 params (indices 0..CFG_U16_COUNT-1) then float params. Fill `out->param_id`, `out->param_value` (cast u16 → float for integer params), `out->param_type`, `out->param_count`, `out->param_index`.

`config_param_set(in)`: search by `param_id` string, parse value, call `config_set_u16` or `config_set_float`.

- [ ] **Step 3: Build**

```bash
idf.py build
```
Expected: `Build successful`

- [ ] **Step 4: Commit**

```bash
git add main/config/config.h main/config/config.c
git commit -m "feat(config): float params + MAVLink PARAM serialisation"
```

---

## Task 3: euler_to_quat pure function

**Files:**
- Create: `main/mavlink/euler_to_quat.h`
- Create: `main/mavlink/euler_to_quat.c`
- Create: `test/test_euler_quat.c`

- [ ] **Step 1: Write failing tests**

```c
// test/test_euler_quat.c
#include "unity.h"
#include "mavlink/euler_to_quat.h"
#include <math.h>

TEST_CASE("euler_to_quat: zero angles → identity quaternion", "[mavlink]") {
    float q[4];
    euler_to_quat(0.0f, 0.0f, 0.0f, q);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, q[0]); // w
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, q[1]); // x
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, q[2]); // y
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, q[3]); // z
}

TEST_CASE("euler_to_quat: quaternion is unit length", "[mavlink]") {
    float q[4];
    euler_to_quat(0.3f, -0.2f, 0.1f, q);
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, norm);
}

TEST_CASE("euler_to_quat: 90 deg roll", "[mavlink]") {
    float q[4];
    euler_to_quat(M_PI/2, 0.0f, 0.0f, q);
    // Expected: w=cos(45°)=0.7071, x=sin(45°)=0.7071, y=0, z=0
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.7071f, q[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.7071f, q[1]);
}
```

- [ ] **Step 2: Build to confirm test fails (symbol undefined)**

```bash
idf.py build
```
Expected: linker error.

- [ ] **Step 3: Write header and implementation**

```c
// main/mavlink/euler_to_quat.h
#ifndef EULER_TO_QUAT_H
#define EULER_TO_QUAT_H

// Convert roll/pitch/yaw (radians) to unit quaternion [w, x, y, z].
// Output array q must have 4 elements.
void euler_to_quat(float roll, float pitch, float yaw, float q[4]);

#endif
```

```c
// main/mavlink/euler_to_quat.c
#include "euler_to_quat.h"
#include <math.h>

void euler_to_quat(float roll, float pitch, float yaw, float q[4]) {
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy; // w
    q[1] = sr * cp * cy - cr * sp * sy; // x
    q[2] = cr * sp * cy + sr * cp * sy; // y
    q[3] = cr * cp * sy - sr * sp * cy; // z
}
```

- [ ] **Step 4: Build and run tests on target**

```bash
idf.py build && idf.py flash monitor
```
Expected: `3 Tests 0 Failures 0 Ignored`

- [ ] **Step 5: Commit**

```bash
git add main/mavlink/euler_to_quat.h main/mavlink/euler_to_quat.c test/test_euler_quat.c
git commit -m "feat(mavlink): euler_to_quat pure function with Unity tests"
```

---

## Task 4: MAVLink task

**Files:**
- Create: `main/mavlink/mavlink_task.h`
- Create: `main/mavlink/mavlink_task.c`

- [ ] **Step 1: Write mavlink_task.h**

```c
// main/mavlink/mavlink_task.h
#ifndef MAVLINK_TASK_H
#define MAVLINK_TASK_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdbool.h>

// Prefixed MAV_CONN_ to avoid collision with MAVLink's own MAV_STATE_* enum values.
typedef enum {
    MAV_CONN_DISCONNECTED = 0,
    MAV_CONN_CONNECTED,
    MAV_CONN_GUIDED_NOGPS,
} mav_conn_state_t;

// nav_cmd_t: attitude command sent to mavlink_task from navigator (Phase 3).
// Phase 2: navigator not yet present — mavlink_task sends zero attitude to keep FC alive.
typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float thrust;     // 0.0 – 1.0
} nav_cmd_t;

// g_nav_cmd_queue is read by mavlink_task. Created in main.c.
extern QueueHandle_t g_nav_cmd_queue;

mav_conn_state_t mavlink_get_state(void);
void mavlink_task(void *param);

#endif
```

- [ ] **Step 2: Write mavlink_task.c**

```c
// main/mavlink/mavlink_task.c
#include "mavlink_task.h"
#include "euler_to_quat.h"
#include "config/config.h"
#include "app_config.h"
#include <ardupilotmega/mavlink.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

static const char *TAG = "MAVLINK";

// Internal state
static mav_conn_state_t s_state = MAV_CONN_DISCONNECTED;
static uint8_t  s_target_sys  = 1;
static uint8_t  s_target_comp = 1;
static uint32_t s_last_hb_rx_ms = 0;

QueueHandle_t g_nav_cmd_queue = NULL;

mav_conn_state_t mavlink_get_state(void) { return s_state; }

// --- UART helpers ---
static void uart_init(void) {
    uart_config_t cfg = {
        .baud_rate  = MAVLINK_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(MAVLINK_UART_PORT, &cfg));
    // Set TX/RX pins for UART1 on ESP32-C6 (GPIO16=TX, GPIO17=RX — adjust for your board).
    // Without this call UART1 uses default pins which may not be exposed on your hardware.
    ESP_ERROR_CHECK(uart_set_pin(MAVLINK_UART_PORT,
        MAVLINK_TX_GPIO, MAVLINK_RX_GPIO,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(MAVLINK_UART_PORT, 1024, 0, 0, NULL, 0));
}

static void send_msg(mavlink_message_t *msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_write_bytes(MAVLINK_UART_PORT, (char*)buf, len);
}

static void send_heartbeat(void) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(255, 190, &msg,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE);
    send_msg(&msg);
}

static void send_set_mode_guided_nogps(void) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(255, 190, &msg,
        s_target_sys, s_target_comp,
        MAV_CMD_DO_SET_MODE, 0,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        20, // COPTER_MODE_GUIDED_NOGPS
        0, 0, 0, 0, 0);
    send_msg(&msg);
    ESP_LOGI(TAG, "sent SET_MODE GUIDED_NOGPS");
}

static void send_attitude_target(const nav_cmd_t *cmd) {
    float q[4];
    euler_to_quat(cmd->roll_rad, cmd->pitch_rad, cmd->yaw_rad, q);
    mavlink_message_t msg;
    mavlink_msg_set_attitude_target_pack(255, 190, &msg,
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
        s_target_sys, s_target_comp,
        0b00000111,  // ignore body rates
        q, 0, 0, 0, cmd->thrust);
    send_msg(&msg);
}

static void handle_param_request_list(void) {
    int count = config_total_params();
    for (int i = 0; i < count; i++) {
        mavlink_param_value_t pv;
        if (config_param_value(i, &pv)) {
            mavlink_message_t msg;
            mavlink_msg_param_value_pack(255, 190, &msg,
                pv.param_id, pv.param_value, pv.param_type,
                pv.param_count, pv.param_index);
            send_msg(&msg);
            vTaskDelay(pdMS_TO_TICKS(10)); // avoid flooding FC
        }
    }
}

static void handle_param_set(mavlink_param_set_t *ps) {
    if (config_param_set(ps)) {
        // Ack with PARAM_VALUE
        int count = config_total_params();
        for (int i = 0; i < count; i++) {
            mavlink_param_value_t pv;
            if (config_param_value(i, &pv) &&
                strncmp(pv.param_id, ps->param_id, 16) == 0) {
                mavlink_message_t msg;
                mavlink_msg_param_value_pack(255, 190, &msg,
                    pv.param_id, pv.param_value, pv.param_type,
                    pv.param_count, pv.param_index);
                send_msg(&msg);
                break;
            }
        }
    }
}

static void process_rx(void) {
    uint8_t byte;
    mavlink_message_t msg;
    mavlink_status_t status;

    while (uart_read_bytes(MAVLINK_UART_PORT, &byte, 1, 0) == 1) {
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    s_target_sys = msg.sysid;
                    s_last_hb_rx_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (s_state == MAV_CONN_DISCONNECTED) {
                        s_state = MAV_CONN_CONNECTED;
                        ESP_LOGI(TAG, "FC connected sysid=%d", s_target_sys);
                        send_set_mode_guided_nogps();
                    }
                    break;
                case MAVLINK_MSG_ID_COMMAND_ACK: {
                    mavlink_command_ack_t ack;
                    mavlink_msg_command_ack_decode(&msg, &ack);
                    if (ack.command == MAV_CMD_DO_SET_MODE &&
                        ack.result == MAV_RESULT_ACCEPTED) {
                        s_state = MAV_CONN_GUIDED_NOGPS;
                        ESP_LOGI(TAG, "GUIDED_NOGPS confirmed");
                    }
                    break;
                }
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                    handle_param_request_list();
                    break;
                case MAVLINK_MSG_ID_PARAM_SET: {
                    mavlink_param_set_t ps;
                    mavlink_msg_param_set_decode(&msg, &ps);
                    handle_param_set(&ps);
                    break;
                }
            }
        }
    }
}

void mavlink_task(void *param) {
    uart_init();
    ESP_LOGI(TAG, "started, waiting for FC heartbeat");

    uint32_t last_hb_tx = 0;
    uint32_t last_att_tx = 0;

    for (;;) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        process_rx();

        // Watchdog: FC heartbeat timeout
        if (s_state != MAV_CONN_DISCONNECTED &&
            (now - s_last_hb_rx_ms) > MAVLINK_WATCHDOG_MS) {
            ESP_LOGW(TAG, "FC heartbeat lost — disconnected");
            s_state = MAV_CONN_DISCONNECTED;
        }

        // Send our heartbeat every 1s
        if ((now - last_hb_tx) >= MAVLINK_HEARTBEAT_INTERVAL_MS) {
            send_heartbeat();
            last_hb_tx = now;
        }

        // Send attitude target @ 20Hz when in GUIDED_NOGPS
        if (s_state == MAV_CONN_GUIDED_NOGPS &&
            (now - last_att_tx) >= MAVLINK_ATTITUDE_RATE_MS) {
            nav_cmd_t cmd = {0};
            xQueuePeek(g_nav_cmd_queue, &cmd, 0); // non-blocking; zeros if navigator not running
            send_attitude_target(&cmd);
            last_att_tx = now;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
```

- [ ] **Step 3: Build**

```bash
idf.py build
```
Expected: `Build successful`

- [ ] **Step 4: Commit**

```bash
git add main/mavlink/mavlink_task.h main/mavlink/mavlink_task.c
git commit -m "feat(mavlink): heartbeat, GUIDED_NOGPS mode, watchdog, PARAM protocol"
```

---

## Task 5: Wire mavlink_task in main.c + CMakeLists

**Files:**
- Modify: `main/CMakeLists.txt`
- Modify: `main/main.c`
- Modify: `main/app_config.h`

- [ ] **Step 1: Add MAVLINK + navigator constants to app_config.h**

```c
// --- Bearing (carry over from Phase 1 if not already present) ---
#define SENTINEL_ANTENNA_SPACING_M         0.165f   // λ/2 @ 900 MHz

// --- Navigator (defaults — runtime via MAVLink PARAM) ---
#define SENTINEL_KP                        0.02f
#define SENTINEL_CRUISE_PITCH_DEG          5.0f
#define SENTINEL_CRUISE_THRUST             0.55f
#define SENTINEL_INTERCEPT_PITCH_DEG       20.0f
#define SENTINEL_INTERCEPT_THRUST          0.85f
#define SENTINEL_VMAG_INTERCEPT_THRESHOLD  0.85f
#define SENTINEL_VMAG_ENABLED              0

// --- MAVLink ---
#define MAVLINK_UART_PORT               UART_NUM_1
#define MAVLINK_TX_GPIO                 GPIO_NUM_16  // adjust for your board wiring
#define MAVLINK_RX_GPIO                 GPIO_NUM_17  // adjust for your board wiring
#define MAVLINK_BAUD_RATE               57600
#define MAVLINK_HEARTBEAT_INTERVAL_MS   1000
#define MAVLINK_ATTITUDE_RATE_MS        50
#define MAVLINK_WATCHDOG_MS             3000

// --- MAVLink task ---
#define MAVLINK_TASK_STACK_SIZE         8192
#define MAVLINK_TASK_PRIORITY           4
```

- [ ] **Step 2: Update CMakeLists.txt**

```cmake
idf_component_register(
    SRCS
        "main.c"
        "adc/adc.c"
        "display/display.c"
        "bearing/bearing.c"
        "config/config.c"
        "mavlink/mavlink_task.c"
        "mavlink/euler_to_quat.c"
    INCLUDE_DIRS "." "adc" "display" "bearing" "config" "mavlink"
    REQUIRES mavlink
)
```

- [ ] **Step 3: Add mavlink_task creation to main.c**

Add `#include "mavlink/mavlink_task.h"` at top of `main.c`.

In `app_main()`, after existing task creation:

```c
g_nav_cmd_queue = xQueueCreate(2, sizeof(nav_cmd_t));
xTaskCreate(mavlink_task, "mavlink_task", MAVLINK_TASK_STACK_SIZE,
            NULL, MAVLINK_TASK_PRIORITY, NULL);
```

Also update `display_task` to show FC connection state instead of hardcoded `"SEARCHING"`:

```c
static void display_task(void *param) {
    ssd1306_handle_t disp = init_display();
    if (!disp) { ESP_LOGE(TAG, "Display init failed"); vTaskDelete(NULL); return; }

    bearing_t b = {0};
    for (;;) {
        if (xQueueReceive(g_bearing_disp_queue, &b, portMAX_DELAY) == pdPASS) {
            const char *state_str;
            switch (mavlink_get_state()) {
                case MAV_CONN_DISCONNECTED:  state_str = "NO FC";   break;
                case MAV_CONN_CONNECTED:     state_str = "FC OK";   break;
                case MAV_CONN_GUIDED_NOGPS:  state_str = "GUIDED";  break;
                default:                     state_str = "?";       break;
            }
            display_show_bearing(disp, &b, state_str);
        }
    }
}
```

> Note: display always updates regardless of FC state — OLED shows bearing + connection status at all times.

- [ ] **Step 4: Build**

```bash
idf.py build
```
Expected: `Build successful`

- [ ] **Step 5: Flash and verify**

```bash
idf.py flash monitor
```

Connect ESP32 UART1 TX/RX to ArduPilot TELEM port. Expected serial output:
```
I (xxx) MAVLINK: started, waiting for FC heartbeat
I (xxx) MAVLINK: FC connected sysid=1
I (xxx) MAVLINK: sent SET_MODE GUIDED_NOGPS
I (xxx) MAVLINK: GUIDED_NOGPS confirmed
```
In Mission Planner: ESP32 should appear as GCS. Full Parameter List should show `SNT_KP`, `SNT_CR_PITCH`, etc.

- [ ] **Step 6: Verify PARAM_SET round-trip**

In Mission Planner Full Parameter List, change `SNT_KP` to `0.05`. Verify in ESP32 serial:
```
I (xxx) CONFIG: saved kp = 0.050000
```
Power-cycle and verify `kp = 0.05` in boot log.

- [ ] **Step 7: Commit**

```bash
git add main/app_config.h main/CMakeLists.txt main/main.c
git commit -m "feat(main): wire mavlink_task, nav_cmd_queue for Phase 2"
```

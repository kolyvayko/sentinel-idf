# Phase 1 — Bearing PoC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Read VPHS from one AD8302 (azimuth only), compute bearing angle, display on SSD1306, and calibrate via NVS + auto-zero button.

**Architecture:** New `config/` module owns all runtime parameters (NVS-backed, fallback to `app_config.h` defaults). New `bearing/` module converts raw ADC → angle. A new `bearing_task` reads from `adc_task` queue and pushes to two output queues (navigator, display). Elevation code is compiled but gated by `SENTINEL_ELEVATION_ENABLED=0`.

**Tech Stack:** ESP-IDF ≥5.3.0, FreeRTOS, ESP NVS, ESP ADC continuous, Unity (for math unit tests), SSD1306 component.

---

## File Map

| Action | Path | Purpose |
|--------|------|---------|
| Modify | `main/app_config.h` | Add all Phase 1 `SENTINEL_*` constants |
| Create | `main/config/config.h` | Param ID enum + `config_get/set` API |
| Create | `main/config/config.c` | NVS load/save, auto-zero button task |
| Create | `main/bearing/bearing.h` | `bearing_t` struct + `vphs_to_angle` signature |
| Create | `main/bearing/bearing.c` | `vphs_to_angle()` + `bearing_task()` |
| Modify | `main/display/display.h` | Add `display_show_bearing()` |
| Modify | `main/display/display.c` | Implement `display_show_bearing()` |
| Modify | `main/main.c` | Create queues + tasks for bearing and display |
| Modify | `main/CMakeLists.txt` | Register `config/`, `bearing/` sources |
| Create | `test/test_bearing.c` | Unity tests for `vphs_to_angle()` |
| Create | `test/CMakeLists.txt` | Test component registration |

---

## Task 1: Extend app_config.h

**Files:**
- Modify: `main/app_config.h`

- [ ] **Step 1: Add Phase 1 constants**

Open `main/app_config.h` and append after existing constants:

```c
// --- Sentinel: Bearing ---
#define SENTINEL_FREQ_HZ                900e6f
#define SENTINEL_ANTENNA_SPACING_M      0.165f   // λ/2 @ 900 MHz
#define SENTINEL_VPHS_ZERO_MV           900      // default 0° calibration (mV)
#define SENTINEL_SIGNAL_THRESHOLD       100      // ADC counts, minimum valid signal
#define SENTINEL_ELEVATION_ENABLED      0        // Phase 1: horizontal only

// --- Sentinel: Calibration button ---
#define SENTINEL_CAL_BTN_GPIO           GPIO_NUM_9
#define SENTINEL_CAL_BTN_HOLD_MS        3000     // hold duration to trigger calibration

// --- ADC sampling ---
#define DISPLAY_UPDATE_INTERVAL         50       // send to bearing queue every N ADC reads

// --- Bearing task ---
#define BEARING_TASK_STACK_SIZE         4096
#define BEARING_TASK_PRIORITY           5
```

- [ ] **Step 2: Build to verify no compile errors**

```bash
idf.py build
```
Expected: `Build successful`

- [ ] **Step 3: Commit**

```bash
git add main/app_config.h
git commit -m "feat(config): add Phase 1 SENTINEL constants to app_config.h"
```

---

## Task 2: Create config/ module

**Files:**
- Create: `main/config/config.h`
- Create: `main/config/config.c`

- [ ] **Step 1: Write config.h**

```c
// main/config/config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

typedef enum {
    CFG_VPHS_ZERO_MV = 0,   // uint16, azimuth zero (mV)
    CFG_VPHS_ZERO2_MV,      // uint16, elevation zero (mV) — Phase 5
    CFG_SIGNAL_THRESHOLD,   // uint16, min ADC counts
    CFG_PARAM_COUNT
} cfg_param_id_t;

void    config_init(void);
uint16_t config_get_u16(cfg_param_id_t id);
void    config_set_u16(cfg_param_id_t id, uint16_t value);

// Auto-zero: reads current VPHS ADC values and saves as zero calibration.
// adc1_raw: current raw ADC reading for azimuth channel.
// adc2_raw: current raw ADC reading for elevation channel (ignored if ELEVATION_ENABLED=0).
void config_auto_zero(int adc1_raw, int adc2_raw);

#endif // CONFIG_H
```

- [ ] **Step 2: Write config.c**

```c
// main/config/config.c
#include "config.h"
#include "app_config.h"
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>
#include <string.h>

static const char *TAG = "CONFIG";
static const char *NVS_NS = "sentinel";

static const char *s_keys[CFG_PARAM_COUNT] = {
    [CFG_VPHS_ZERO_MV]    = "vphs_zero",
    [CFG_VPHS_ZERO2_MV]   = "vphs_zero2",
    [CFG_SIGNAL_THRESHOLD] = "sig_thr",
};

static const uint16_t s_defaults[CFG_PARAM_COUNT] = {
    [CFG_VPHS_ZERO_MV]    = SENTINEL_VPHS_ZERO_MV,
    [CFG_VPHS_ZERO2_MV]   = SENTINEL_VPHS_ZERO_MV,
    [CFG_SIGNAL_THRESHOLD] = SENTINEL_SIGNAL_THRESHOLD,
};

static uint16_t s_values[CFG_PARAM_COUNT];

void config_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open(NVS_NS, NVS_READONLY, &h));
    for (int i = 0; i < CFG_PARAM_COUNT; i++) {
        uint16_t v = s_defaults[i];
        nvs_get_u16(h, s_keys[i], &v);   // ignore ESP_ERR_NVS_NOT_FOUND
        s_values[i] = v;
        ESP_LOGI(TAG, "%s = %u", s_keys[i], v);
    }
    nvs_close(h);
}

uint16_t config_get_u16(cfg_param_id_t id) {
    return s_values[id];
}

void config_set_u16(cfg_param_id_t id, uint16_t value) {
    s_values[id] = value;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u16(h, s_keys[id], value);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "saved %s = %u", s_keys[id], value);
}

void config_auto_zero(int adc1_raw, int adc2_raw) {
    // Convert raw ADC to mV: 3300 mV / 4095 counts
    uint16_t mv1 = (uint16_t)((adc1_raw * 3300) / 4095);
    config_set_u16(CFG_VPHS_ZERO_MV, mv1);
    ESP_LOGI(TAG, "auto-zero: AZ=%umV", mv1);

#if SENTINEL_ELEVATION_ENABLED
    uint16_t mv2 = (uint16_t)((adc2_raw * 3300) / 4095);
    config_set_u16(CFG_VPHS_ZERO2_MV, mv2);
    ESP_LOGI(TAG, "auto-zero: EL=%umV", mv2);
#endif
}
```

- [ ] **Step 3: Note — skip build here**

`config/` is not yet registered in `main/CMakeLists.txt` (done in Task 6). These files won't compile until then — building now would give a false pass. Continue to Task 3.

- [ ] **Step 4: Commit**

```bash
git add main/config/config.h main/config/config.c
git commit -m "feat(config): NVS-backed config module with auto-zero"
```

---

## Task 3: Create bearing/ module

**Files:**
- Create: `main/bearing/bearing.h`
- Create: `main/bearing/bearing.c`

- [ ] **Step 1: Write bearing.h**

```c
// main/bearing/bearing.h
#ifndef BEARING_H
#define BEARING_H

typedef struct {
    float azimuth_deg;    // negative = left, positive = right
    float elevation_deg;  // negative = below, positive = above (0 if ELEVATION_ENABLED=0)
    int   adc1_raw;       // raw ADC value azimuth channel (for auto-zero)
    int   adc2_raw;       // raw ADC value elevation channel
} bearing_t;

// Pure function: convert raw ADC reading to angle in degrees.
// adc_raw:          12-bit ADC count (0–4095)
// antenna_spacing_m: distance between antenna pair (metres)
// freq_hz:           target RF frequency (Hz)
// vphs_zero_v:       calibrated midpoint voltage (volts), typically 0.9
float vphs_to_angle(int adc_raw, float antenna_spacing_m,
                    float freq_hz, float vphs_zero_v);

void bearing_task(void *param);

#endif // BEARING_H
```

- [ ] **Step 2: Write the Unity test first (TDD)**

Create `test/test_bearing.c`:

```c
// test/test_bearing.c
#include "unity.h"
#include "bearing.h"
#include <math.h>

// vphs_to_angle at midpoint (zero calibration = actual midpoint) must return 0°
TEST_CASE("vphs_to_angle: midpoint → 0 degrees", "[bearing]") {
    // adc_raw that gives exactly 0.9V: 0.9 / 3.3 * 4095 = 1117
    float angle = vphs_to_angle(1117, 0.165f, 900e6f, 0.9f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, angle);
}

// Full-scale right: VPHS = 1.8V → adc_raw ≈ 2234 → should return positive angle
TEST_CASE("vphs_to_angle: max right → positive angle", "[bearing]") {
    float angle = vphs_to_angle(2234, 0.165f, 900e6f, 0.9f);
    TEST_ASSERT_GREATER_THAN(0.0f, angle);
}

// Full-scale left: VPHS = 0V → adc_raw = 0 → should return negative angle
TEST_CASE("vphs_to_angle: max left → negative angle", "[bearing]") {
    float angle = vphs_to_angle(0, 0.165f, 900e6f, 0.9f);
    TEST_ASSERT_LESS_THAN(0.0f, angle);
}

// Output must be clamped to [-90, +90]
TEST_CASE("vphs_to_angle: clamps to ±90 degrees", "[bearing]") {
    float angle_max = vphs_to_angle(4095, 0.165f, 900e6f, 0.9f);
    float angle_min = vphs_to_angle(0,    0.165f, 900e6f, 0.9f);
    TEST_ASSERT_LESS_OR_EQUAL(90.0f,  angle_max);
    TEST_ASSERT_GREATER_OR_EQUAL(-90.0f, angle_min);
}

// Calibration offset shifts the zero point
TEST_CASE("vphs_to_angle: calibration offset applied", "[bearing]") {
    // raw=1117 → 0.9V; with zero calibrated to 0.95V it should read negative
    float angle = vphs_to_angle(1117, 0.165f, 900e6f, 0.95f);
    TEST_ASSERT_LESS_THAN(0.0f, angle);
}
```

Create `test/CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "test_bearing.c" "../main/bearing/bearing.c"
    INCLUDE_DIRS "../main" "../main/bearing"
    REQUIRES unity esp_adc
)
```

- [ ] **Step 3: Write bearing.c**

```c
// main/bearing/bearing.c
#include "bearing.h"
#include "config/config.h"
#include "app_config.h"
#include <math.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "adc/adc.h"

static const char *TAG = "BEARING";

// Queues populated by bearing_task. Created in main.c before tasks start.
extern QueueHandle_t g_bearing_nav_queue;
extern QueueHandle_t g_bearing_disp_queue;
// Queue from adc_task (defined in main.c)
extern QueueHandle_t g_adc_queue;

float vphs_to_angle(int adc_raw, float antenna_spacing_m,
                    float freq_hz, float vphs_zero_v) {
    float vphs      = adc_raw * 3.3f / 4095.0f;
    float delta_phi = (vphs - vphs_zero_v) / vphs_zero_v * M_PI;
    float lambda    = 3e8f / freq_hz;
    float sin_theta = delta_phi * lambda / (2.0f * M_PI * antenna_spacing_m);
    sin_theta = fmaxf(-1.0f, fminf(1.0f, sin_theta));
    return asinf(sin_theta) * (180.0f / M_PI);
}

void bearing_task(void *param) {
    // adc_sample_t must be defined in main.c or a shared header
    typedef struct { int adc1; int adc2; } adc_sample_t;
    adc_sample_t sample;

    for (;;) {
        if (xQueueReceive(g_adc_queue, &sample, portMAX_DELAY) != pdPASS) continue;

        if (sample.adc1 < SENTINEL_SIGNAL_THRESHOLD) {
            ESP_LOGD(TAG, "signal below threshold, skipping");
            continue;
        }

        float vphs_zero_v = config_get_u16(CFG_VPHS_ZERO_MV) / 1000.0f;

        bearing_t b = {
            .adc1_raw = sample.adc1,
            .adc2_raw = sample.adc2,
            .azimuth_deg = vphs_to_angle(
                sample.adc1,
                SENTINEL_ANTENNA_SPACING_M,
                SENTINEL_FREQ_HZ,
                vphs_zero_v
            ),
#if SENTINEL_ELEVATION_ENABLED
            .elevation_deg = vphs_to_angle(
                sample.adc2,
                SENTINEL_ANTENNA_SPACING_M,
                SENTINEL_FREQ_HZ,
                config_get_u16(CFG_VPHS_ZERO2_MV) / 1000.0f
            ),
#else
            .elevation_deg = 0.0f,
#endif
        };

        ESP_LOGI(TAG, "AZ=%.1f° EL=%.1f°", b.azimuth_deg, b.elevation_deg);

        // Send to display queue (drop oldest if full)
        if (xQueueSend(g_bearing_disp_queue, &b, 0) != pdPASS) {
            bearing_t tmp;
            xQueueReceive(g_bearing_disp_queue, &tmp, 0);
            xQueueSend(g_bearing_disp_queue, &b, 0);
        }
        // Send to navigator queue (drop oldest if full)
        if (xQueueSend(g_bearing_nav_queue, &b, 0) != pdPASS) {
            bearing_t tmp;
            xQueueReceive(g_bearing_nav_queue, &tmp, 0);
            xQueueSend(g_bearing_nav_queue, &b, 0);
        }
    }
}
```

- [ ] **Step 5: Build and run tests**

```bash
idf.py build
```
Expected: `Build successful`

To run Unity tests on target hardware:
```bash
idf.py flash monitor
```
Expected in serial output: `5 Tests 0 Failures 0 Ignored`

- [ ] **Step 6: Commit**

```bash
git add main/bearing/bearing.h main/bearing/bearing.c test/test_bearing.c test/CMakeLists.txt
git commit -m "feat(bearing): vphs_to_angle + bearing_task with Unity tests"
```

---

## Task 4: Update display to show bearing

**Files:**
- Modify: `main/display/display.h`
- Modify: `main/display/display.c`

- [ ] **Step 1: Add `display_show_bearing()` to display.h**

Add after existing declarations in `main/display/display.h`:

```c
#include "bearing/bearing.h"

void display_show_bearing(ssd1306_handle_t d, const bearing_t *b, const char *state_str);
```

- [ ] **Step 2: Implement `display_show_bearing()` in display.c**

Add at the end of `main/display/display.c`:

```c
void display_show_bearing(ssd1306_handle_t d, const bearing_t *b, const char *state_str) {
    if (!d) return;
    ESP_ERROR_CHECK(ssd1306_clear(d));

    char line1[32];
    char line2[32];
    snprintf(line1, sizeof(line1), "AZ:%+.1f EL:%+.1f", b->azimuth_deg, b->elevation_deg);
    snprintf(line2, sizeof(line2), "%s", state_str);

    ESP_ERROR_CHECK(ssd1306_draw_text_scaled(d, 0, 0,  line1, true, 1));
    ESP_ERROR_CHECK(ssd1306_draw_text_scaled(d, 0, 16, line2, true, 1));
    ESP_ERROR_CHECK(ssd1306_display(d));
}
```

- [ ] **Step 3: Build**

```bash
idf.py build
```
Expected: `Build successful`

- [ ] **Step 4: Commit**

```bash
git add main/display/display.h main/display/display.c
git commit -m "feat(display): add display_show_bearing for azimuth/elevation readout"
```

---

## Task 5: Auto-zero button task

**Files:**
- Modify: `main/config/config.c` (add button task function)
- Modify: `main/config/config.h` (add button task declaration)

- [ ] **Step 1: Add `config_btn_task` declaration to config.h**

Add to `main/config/config.h`:

```c
// FreeRTOS task that monitors the calibration button (GPIO SENTINEL_CAL_BTN_GPIO).
// Param: pointer to QueueHandle_t of adc_sample_t (reads latest ADC value for auto-zero).
void config_btn_task(void *param);
```

- [ ] **Step 2: Implement `config_btn_task` in config.c**

Add to `main/config/config.c`:

```c
#include <driver/gpio.h>
#include <freertos/task.h>
#include "adc/adc.h"   // for adc_sample_t

void config_btn_task(void *param) {
    QueueHandle_t adc_q = (QueueHandle_t)param;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << SENTINEL_CAL_BTN_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);

    for (;;) {
        if (gpio_get_level(SENTINEL_CAL_BTN_GPIO) == 0) {
            // Button pressed (active low) — start hold timer
            uint32_t held_ms = 0;
            while (gpio_get_level(SENTINEL_CAL_BTN_GPIO) == 0
                   && held_ms < SENTINEL_CAL_BTN_HOLD_MS) {
                vTaskDelay(pdMS_TO_TICKS(50));
                held_ms += 50;
            }
            if (held_ms >= SENTINEL_CAL_BTN_HOLD_MS) {
                // Average VPHS over 100 ms to get stable zero reading.
                // Spec requires signal to be present during calibration.
                int64_t sum1 = 0, sum2 = 0;
                int count = 0;
                uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;
                adc_sample_t s = {0};
                while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start) < 100) {
                    if (xQueuePeek(adc_q, &s, pdMS_TO_TICKS(10)) == pdPASS) {
                        sum1 += s.adc1;
                        sum2 += s.adc2;
                        count++;
                    }
                }
                if (count > 0) {
                    config_auto_zero((int)(sum1 / count), (int)(sum2 / count));
                    ESP_LOGI("CONFIG_BTN", "auto-zero triggered (%d samples)", count);
                } else {
                    ESP_LOGW("CONFIG_BTN", "auto-zero skipped: no ADC signal");
                }
                // Debounce: wait for release
                while (gpio_get_level(SENTINEL_CAL_BTN_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
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
git add main/config/config.h main/config/config.c
git commit -m "feat(config): auto-zero button task (hold 3s to calibrate VPHS zero)"
```

---

## Task 6: Wire everything in main.c + CMakeLists

**Files:**
- Modify: `main/main.c`
- Modify: `main/CMakeLists.txt`

- [ ] **Step 1: Update CMakeLists.txt**

Replace the contents of `main/CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS
        "main.c"
        "adc/adc.c"
        "display/display.c"
        "bearing/bearing.c"
        "config/config.c"
    INCLUDE_DIRS "." "adc" "display" "bearing" "config"
)
```

- [ ] **Step 2: Add `adc_sample_t` to adc.h**

The existing `main/main.c` has a local `typedef struct { int adc1; int adc2; } adc_sample_t`. Move it to `main/adc/adc.h` so all modules share it. Add to `main/adc/adc.h` alongside existing declarations:

```c
typedef struct {
    int adc1;   // VPHS azimuth  (ADC CH1)
    int adc2;   // VPHS elevation (ADC CH2)
    int adc3;   // VMAG optional  (ADC CH3) — always 0 until Phase 4
} adc_sample_t;
```

Remove any local `typedef` of `adc_sample_t` from `main.c` and `bearing.c`.

- [ ] **Step 3: Rewrite main.c**

> **Note:** This completely replaces the existing `main.c`. The original `adc_task` defined inline in `main.c` is kept as-is but updated to use the shared `adc_sample_t` from `adc.h`. The existing `adc_task` in the original code is a thin wrapper around `adc_read_values()` — this plan retains that pattern.

```c
// main/main.c
#include "adc/adc.h"
#include "bearing/bearing.h"
#include "config/config.h"
#include "display/display.h"
#include "app_config.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

static const char *TAG = "MAIN";

// Shared queues — extern'd by bearing.c and display_task
QueueHandle_t g_adc_queue          = NULL;
QueueHandle_t g_bearing_nav_queue  = NULL;
QueueHandle_t g_bearing_disp_queue = NULL;

static void adc_task(void *param) {
    adc_continuous_handle_t handle = adc_init();
    if (!handle) { ESP_LOGE(TAG, "ADC init failed"); vTaskDelete(NULL); return; }

    adc_sample_t s = {0};
    uint32_t n = 0;
    while (1) {
        adc_read_values(handle, &s.adc1, &s.adc2);
        n++;
        if ((n % DISPLAY_UPDATE_INTERVAL) == 0) {
            if (xQueueSend(g_adc_queue, &s, 0) != pdPASS) {
                adc_sample_t tmp;
                xQueueReceive(g_adc_queue, &tmp, 0);
                xQueueSend(g_adc_queue, &s, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void display_task(void *param) {
    ssd1306_handle_t disp = init_display();
    if (!disp) { ESP_LOGE(TAG, "Display init failed"); vTaskDelete(NULL); return; }

    bearing_t b = {0};
    for (;;) {
        if (xQueueReceive(g_bearing_disp_queue, &b, portMAX_DELAY) == pdPASS) {
            display_show_bearing(disp, &b, "SEARCHING");
        }
    }
}

void app_main(void) {
    config_init();

    g_adc_queue          = xQueueCreate(ADC_QUEUE_LENGTH,  sizeof(adc_sample_t));
    g_bearing_nav_queue  = xQueueCreate(1,                 sizeof(bearing_t));
    g_bearing_disp_queue = xQueueCreate(1,                 sizeof(bearing_t));

    if (!g_adc_queue || !g_bearing_nav_queue || !g_bearing_disp_queue) {
        ESP_LOGE(TAG, "Queue creation failed"); return;
    }

    xTaskCreate(adc_task,        "adc_task",     ADC_TASK_STACK_SIZE,     NULL, ADC_TASK_PRIORITY,     NULL);
    xTaskCreate(bearing_task,    "bearing_task", BEARING_TASK_STACK_SIZE, NULL, BEARING_TASK_PRIORITY, NULL);
    xTaskCreate(display_task,    "disp_task",    DISPLAY_TASK_STACK_SIZE, NULL, DISPLAY_TASK_PRIORITY, NULL);
    xTaskCreate(config_btn_task, "cal_btn_task", 2048,                    (void*)g_adc_queue, 3, NULL);
}
```

- [ ] **Step 3: Build the full project**

```bash
idf.py build
```
Expected: `Build successful` with no warnings about missing symbols.

- [ ] **Step 4: Flash and verify on hardware**

```bash
idf.py flash monitor
```

Expected serial output:
```
I (xxx) CONFIG: vphs_zero = 900
I (xxx) CONFIG: sig_thr = 100
I (xxx) BEARING: AZ=X.X° EL=0.0°
```
SSD1306 should display `AZ:+0.0 EL:+0.0` (or current angle).

- [ ] **Step 5: Commit**

```bash
git add main/main.c main/CMakeLists.txt
git commit -m "feat(main): wire bearing_task, config, display for Phase 1 PoC"
```

---

## Task 7: Field verification

No code changes — manual verification only.

- [ ] **Step 1: Bench test at known angles**

Set up AD8302 #1 with antenna pair. Point antenna at 0°, 30°, 60° from the transmitter.
Observe SSD1306 and serial output. Expected angle within ±5° of actual.

- [ ] **Step 2: Auto-zero calibration**

Point drone directly at transmitter (0° azimuth).
Hold GPIO9 button for 3s.
Serial output should show: `auto-zero triggered`, `saved vphs_zero = NNNmV`.
Verify SSD1306 shows ~0.0° after calibration.

- [ ] **Step 3: Power-cycle persistence**

After auto-zero, power cycle ESP32.
Verify `CONFIG: vphs_zero = NNN` in boot log matches calibrated value (not 900).

- [ ] **Step 4: Commit verification notes**

```bash
git commit --allow-empty -m "test(phase1): bearing PoC field verified — angles within ±5°"
```

# Phase 5 — Elevation + Reliability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enable 3D tracking by activating AD8302 #2 (elevation), add dual-frequency support (900 MHz / 2.4 GHz profile switching), MAVLink STATUSTEXT logging, and stress-test reliability.

**Architecture:** Set `SENTINEL_ELEVATION_ENABLED 1` — all elevation code paths that were compiled but gated now activate. Add `SNT_FREQ_MHZ` param handling: when changed via MAVLink, recompute `SENTINEL_ANTENNA_SPACING_M` at runtime. Add `STATUSTEXT` broadcast for state transitions. Stress-test: signal loss, interference, range changes.

**Tech Stack:** ESP-IDF ≥5.3.0, FreeRTOS.

**Prerequisite:** Phase 4 complete — INTERCEPT state working.

---

## File Map

| Action | Path | Purpose |
|--------|------|---------|
| Modify | `main/app_config.h` | Set `SENTINEL_ELEVATION_ENABLED 1` |
| Modify | `main/config/config.c` | Handle `SNT_FREQ_MHZ` change → recompute antenna spacing |
| Modify | `main/bearing/bearing.c` | Use elevation channel when enabled |
| Modify | `main/mavlink/mavlink_task.c` | Broadcast STATUSTEXT on state transitions |
| Modify | `main/navigator/navigator.c` | Notify mavlink_task of state changes for STATUSTEXT |
| Modify | `test/test_bearing.c` | Add elevation angle tests |

---

## Task 1: Enable elevation channel

**Files:**
- Modify: `main/app_config.h`
- Modify: `test/test_bearing.c`

- [ ] **Step 1: Enable elevation in app_config.h**

```c
#define SENTINEL_ELEVATION_ENABLED  1
```

- [ ] **Step 2: Add elevation angle tests**

Add to `test/test_bearing.c`:

```c
// Elevation channel uses same physics as azimuth
TEST_CASE("vphs_to_angle: elevation at midpoint → 0 degrees", "[bearing]") {
    float angle = vphs_to_angle(1117, 0.165f, 900e6f, 0.9f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, angle);
}

// Elevation and azimuth calculations are symmetric (same formula)
TEST_CASE("vphs_to_angle: elevation positive input → positive angle", "[bearing]") {
    float angle = vphs_to_angle(2000, 0.165f, 900e6f, 0.9f);
    TEST_ASSERT_GREATER_THAN(0.0f, angle);
}
```

- [ ] **Step 3: Build and run tests**

```bash
idf.py build && idf.py flash monitor
```
Expected: all tests pass.

- [ ] **Step 4: Flash and verify SSD1306 shows non-zero elevation**

Point drone up/down relative to transmitter. Verify `EL:` value changes on display.

- [ ] **Step 5: Commit**

```bash
git add main/app_config.h test/test_bearing.c
git commit -m "feat(bearing): enable elevation channel (SENTINEL_ELEVATION_ENABLED=1)"
```

---

## Task 2: Dual-frequency runtime switching

**Files:**
- Modify: `main/config/config.c`
- Modify: `main/bearing/bearing.c`

- [ ] **Step 1: Add runtime frequency → antenna spacing mapping in config.c**

When `SNT_FREQ_MHZ` is set via `config_param_set()`, automatically update `CFG_ANT_SPACING`:

```c
// In config_param_set(), after applying the value:
if (strcmp(in->param_id, "SNT_FREQ_MHZ") == 0) {
    uint16_t freq_mhz = (uint16_t)in->param_value;
    float spacing;
    if (freq_mhz == 2400) {
        spacing = 0.0625f;  // λ/2 @ 2.4 GHz = 12.5cm / 2
    } else {
        spacing = 0.165f;   // λ/2 @ 900 MHz = 33.3cm / 2
    }
    config_set_float(CFG_ANT_SPACING, spacing);
    ESP_LOGI(TAG, "freq=%uMHz → antenna_spacing=%.3fm", freq_mhz, spacing);
}
```

- [ ] **Step 2: Add CFG_FREQ_MHZ to config module first**

> **Note:** This step must come before Step 3 because `bearing.c` will call `config_get_u16(CFG_FREQ_MHZ)` — the enum value must exist before the code compiles.

Add `CFG_FREQ_MHZ` to `cfg_u16_id_t` enum in `config.h` (before `CFG_U16_COUNT`). Add NVS key `"freq_mhz"` and default `900` to the u16 tables in `config.c`. Add MAVLink param ID `"SNT_FREQ_MHZ"`.

- [ ] **Step 3: Use runtime freq in bearing.c**

In `bearing_task()`, replace compile-time constant usage:

```c
// Instead of SENTINEL_FREQ_HZ (compile-time), derive from config:
float freq_hz = config_get_u16(CFG_FREQ_MHZ) * 1e6f;
float spacing = config_get_float(CFG_ANT_SPACING);

b.azimuth_deg   = vphs_to_angle(sample.adc1, spacing, freq_hz, vphs_zero_v);
b.elevation_deg = vphs_to_angle(sample.adc2, spacing, freq_hz,
                                 config_get_u16(CFG_VPHS_ZERO2_MV) / 1000.0f);
```

- [ ] **Step 4: Build**

```bash
idf.py build
```
Expected: `Build successful`

- [ ] **Step 5: Verify frequency switching**

Flash. In Mission Planner, set `SNT_FREQ_MHZ = 2400`.
Serial should show: `freq=2400MHz → antenna_spacing=0.063m`
SSD1306 angles should shift (different λ).

- [ ] **Step 6: Commit**

```bash
git add main/config/config.c main/config/config.h main/bearing/bearing.c
git commit -m "feat(config): runtime dual-frequency switching via SNT_FREQ_MHZ param"
```

---

## Task 3: MAVLink STATUSTEXT logging

**Files:**
- Modify: `main/mavlink/mavlink_task.h`
- Modify: `main/mavlink/mavlink_task.c`
- Modify: `main/navigator/navigator.c`

- [ ] **Step 1: Add statustext API to mavlink_task.h**

```c
// Broadcast a text message to GCS (visible in Mission Planner Messages tab).
// severity: MAV_SEVERITY_INFO, MAV_SEVERITY_WARNING, etc.
void mavlink_send_statustext(uint8_t severity, const char *text);
```

- [ ] **Step 2: Implement in mavlink_task.c**

```c
void mavlink_send_statustext(uint8_t severity, const char *text) {
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(255, 190, &msg, severity, text, 0, 0);
    send_msg(&msg);
}
```

- [ ] **Step 3: Broadcast on state transitions in navigator.c**

> **Note:** `nav_state_str()` is declared in `navigator.h` and implemented in `navigator.c` (Phase 3). No new implementation needed — it's already available.

```c
#include "mavlink/mavlink_task.h"
// In navigator_task(), after state change:
if (new_state != s_state) {
    char buf[50];
    snprintf(buf, sizeof(buf), "Sentinel: %s->%s",
             nav_state_str(s_state), nav_state_str(new_state));
    mavlink_send_statustext(MAV_SEVERITY_INFO, buf);
    s_state = new_state;
}
```

- [ ] **Step 4: Build and verify in Mission Planner**

```bash
idf.py build && idf.py flash monitor
```
In Mission Planner Messages tab: state transitions should appear as:
`Sentinel: SEARCHING->TRACKING`

- [ ] **Step 5: Commit**

```bash
git add main/mavlink/mavlink_task.h main/mavlink/mavlink_task.c main/navigator/navigator.c
git commit -m "feat(mavlink): STATUSTEXT state transition logging to GCS"
```

---

## Task 4: Reliability stress tests

No code changes. Manual field tests.

- [ ] **Step 1: Signal loss test**

Tracker active in TRACKING state. Block RF signal for 3s.
Expected: `TRACKING → SEARCHING`. Restore signal → `SEARCHING → TRACKING`.

- [ ] **Step 2: Interference test**

Activate a second 2.4 GHz transmitter nearby.
Observe if bearing degrades. Tune `SNT_SIG_THR` to ignore weak/noisy signal.

- [ ] **Step 3: Range test**

Move transmitter to 20m, 50m, 100m.
Record minimum `adc1_raw` values at max range. Update `SNT_SIG_THR` accordingly.

- [ ] **Step 4: Power-cycle persistence**

Change several params via Mission Planner. Power-cycle.
Verify all params survive reboot via boot log.

- [ ] **Step 5: Elevation auto-zero calibration**

The `config_btn_task` in `config.c` (Phase 1) already reads `s.adc2` and calls `config_auto_zero(sum1/count, sum2/count)`. The `config_auto_zero()` function saves `CFG_VPHS_ZERO2_MV` only when `SENTINEL_ELEVATION_ENABLED=1` (gated by `#if`). No code change needed — the elevation branch was written in Phase 1 but inactive until now.

With `SENTINEL_ELEVATION_ENABLED=1` flashed, hold cal button 3s.
Verify `CAL OK  AZ:NNNmV EL:NNNmV` on SSD1306.

- [ ] **Step 6: Final commit**

```bash
git commit --allow-empty -m "test(phase5): full 3D tracking + reliability verified in field"
```

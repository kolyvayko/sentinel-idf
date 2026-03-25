# Phase 4 — INTERCEPT + VMAG Proximity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enable the INTERCEPT state — full-throttle forward pitch on proximity detection. Optionally add VMAG (amplitude) as a second proximity trigger via ADC CH4.

**Architecture:** The navigator state machine already has INTERCEPT wired as a terminal state. This phase: (1) validates the INTERCEPT attitude command is correct, (2) optionally enables VMAG ADC channel (GPIO4) in `adc/` module, (3) adds VMAG normalisation to `bearing_t`, (4) enables the VMAG proximity threshold in navigator. All VMAG functionality remains behind `SENTINEL_VMAG_ENABLED` flag.

**Tech Stack:** ESP-IDF ≥5.3.0, FreeRTOS.

**Prerequisite:** Phase 3 complete — `navigator_task` tracking a horizontal target.

---

## File Map

| Action | Path | Purpose |
|--------|------|---------|
| Modify | `main/bearing/bearing.h` | Add `vmag_norm` field to `bearing_t` |
| Modify | `main/adc/adc.h` | Extend `adc_sample_t` to include `adc3` (VMAG) |
| Modify | `main/adc/adc.c` | Add optional CH3 to ADC continuous config |
| Modify | `main/bearing/bearing.c` | Compute `vmag_norm` when `SENTINEL_VMAG_ENABLED` |
| Modify | `main/navigator/navigator.c` | Use `vmag_norm` in TRACKING → INTERCEPT transition |
| Modify | `main/app_config.h` | Add `SENTINEL_VMAG_ENABLED`, `SENTINEL_VMAG_ADC_CH` |
| Modify | `test/test_navigator.c` | Add INTERCEPT transition tests |

---

## Task 1: Extend adc_sample_t with VMAG channel

**Files:**
- Modify: `main/adc/adc.h`
- Modify: `main/adc/adc.c`
- Modify: `main/app_config.h`

- [ ] **Step 1: Add VMAG constants to app_config.h**

```c
// --- VMAG (optional proximity channel) ---
#define SENTINEL_VMAG_ENABLED   0          // set to 1 in Phase 4 field test
#define SENTINEL_VMAG_ADC_CH    ADC_CHANNEL_4  // GPIO4 (JTAG/MTMS — usable after boot)
```

- [ ] **Step 2: Extend adc_sample_t in adc.h**

The `adc_sample_t` struct is currently defined in `main.c`. Move it to `main/adc/adc.h` so all modules share the same definition:

```c
// main/adc/adc.h  (add alongside existing declarations)
typedef struct {
    int adc1;   // VPHS azimuth  (ADC CH1, GPIO1)
    int adc2;   // VPHS elevation (ADC CH2, GPIO2)
    int adc3;   // VMAG optional  (ADC CH4, GPIO4) — 0 if SENTINEL_VMAG_ENABLED=0
} adc_sample_t;
```

Remove the local `typedef struct { int adc1; int adc2; } adc_sample_t;` from `main.c` and `bearing.c`, add `#include "adc/adc.h"` where needed.

- [ ] **Step 3: Add CH3 to ADC continuous config in adc.c**

In `adc_init()`, extend the channel array conditionally:

```c
// main/adc/adc.c — in adc_init()
#if SENTINEL_VMAG_ENABLED
static adc_channel_t channel[3] = {ADC_CHANNEL_1, ADC_CHANNEL_2, SENTINEL_VMAG_ADC_CH};
#else
static adc_channel_t channel[2] = {ADC_CHANNEL_1, ADC_CHANNEL_2};
#endif
```

In `adc_read_values()`, route `ADC_CHANNEL_4` results to the `adc3` field:
```c
} else if (parsed_data[i].channel == SENTINEL_VMAG_ADC_CH) {
    *adc3 = parsed_data[i].raw_data;
}
```

Update `adc_read_values` signature:
```c
void adc_read_values(adc_continuous_handle_t handle, int *adc1, int *adc2, int *adc3);
```

- [ ] **Step 4: Update all call sites of adc_read_values**

In `main/main.c` `adc_task`:
```c
adc_read_values(handle, &s.adc1, &s.adc2, &s.adc3);
```

- [ ] **Step 5: Build**

```bash
idf.py build
```
Expected: `Build successful`

- [ ] **Step 6: Commit**

```bash
git add main/adc/adc.h main/adc/adc.c main/app_config.h main/main.c main/bearing/bearing.c
git commit -m "feat(adc): add optional VMAG channel (CH4/GPIO4) to adc_sample_t"
```

---

## Task 2: Add vmag_norm to bearing_t

**Files:**
- Modify: `main/bearing/bearing.h`
- Modify: `main/bearing/bearing.c`

- [ ] **Step 1: Add vmag_norm to bearing_t**

In `main/bearing/bearing.h`, add field:
```c
typedef struct {
    float azimuth_deg;
    float elevation_deg;
    float vmag_norm;   // [0.0, 1.0], 0 if SENTINEL_VMAG_ENABLED=0
    int   adc1_raw;
    int   adc2_raw;
} bearing_t;
```

- [ ] **Step 2: Compute vmag_norm in bearing_task**

In `bearing_task()`, after computing angles:
```c
#if SENTINEL_VMAG_ENABLED
    b.vmag_norm = fminf(1.0f, sample.adc3 / 4095.0f);
#else
    b.vmag_norm = 0.0f;
#endif
```

- [ ] **Step 3: Build**

```bash
idf.py build
```
Expected: `Build successful`

- [ ] **Step 4: Commit**

```bash
git add main/bearing/bearing.h main/bearing/bearing.c
git commit -m "feat(bearing): add vmag_norm field to bearing_t"
```

---

## Task 3: INTERCEPT transition tests + enable VMAG in navigator

**Files:**
- Modify: `test/test_navigator.c`
- Modify: `main/navigator/navigator.c`

- [ ] **Step 1: Add INTERCEPT transition tests**

Add to `test/test_navigator.c`:

```c
// High VMAG + small angle → should produce INTERCEPT-level thrust in compute()
TEST_CASE("navigator_compute: intercept pitch > cruise pitch", "[navigator]") {
    bearing_t b = make_bearing(2.0f, 0.0f);
    nav_cmd_t tracking = navigator_compute(&b, NAV_STATE_TRACKING,
                                            0.02f, 5.0f, 0.55f, 20.0f, 0.85f);
    nav_cmd_t intercept = navigator_compute(&b, NAV_STATE_INTERCEPT,
                                             0.02f, 5.0f, 0.55f, 20.0f, 0.85f);
    // INTERCEPT pitch must be greater than TRACKING pitch
    TEST_ASSERT_GREATER_THAN(tracking.pitch_rad, intercept.pitch_rad);
    TEST_ASSERT_GREATER_THAN(tracking.thrust,    intercept.thrust);
}
```

- [ ] **Step 2: Run tests**

```bash
idf.py build && idf.py flash monitor
```
Expected: all tests pass.

- [ ] **Step 3: Wire vmag_norm into TRACKING → INTERCEPT in navigator.c**

Update `next_state()` in `main/navigator/navigator.c` for the TRACKING case:

```c
case NAV_STATE_TRACKING:
    if (!has_signal) {
        if ((now_ms - s_last_signal_ms) > MAVLINK_WATCHDOG_MS)
            return NAV_STATE_SEARCHING;
    } else {
        s_last_signal_ms = now_ms;
        bool angle_close = (fabsf(b->azimuth_deg) < 5.0f);
#if SENTINEL_VMAG_ENABLED
        if (b->vmag_norm > vmag_threshold && angle_close)
            return NAV_STATE_INTERCEPT;
#else
        (void)vmag_threshold;
#endif
    }
    break;
```

- [ ] **Step 4: Build**

```bash
idf.py build
```
Expected: `Build successful`

- [ ] **Step 5: Commit**

```bash
git add test/test_navigator.c main/navigator/navigator.c
git commit -m "feat(navigator): VMAG-gated INTERCEPT transition + tests"
```

---

## Task 4: Field test — INTERCEPT with stationary target

No code changes initially. Manual test with `SENTINEL_VMAG_ENABLED=0`.

- [ ] **Step 1: Verify INTERCEPT with angle-only trigger**

Hold transmitter 2m in front of drone. Verify drone transitions to INTERCEPT when azimuth < 5°.
Serial: `TRACKING → INTERCEPT`. Drone should pitch forward and increase throttle.

- [ ] **Step 2: Enable VMAG (optional)**

If desired: set `SENTINEL_VMAG_ENABLED 1` in `app_config.h`, rebuild and flash.
Move transmitter closer. Observe `SNT_VMAG_THR` in Mission Planner.
Tune `SNT_VMAG_THR` via Mission Planner until transition happens at desired distance.

- [ ] **Step 3: Commit field results**

```bash
git commit --allow-empty -m "test(phase4): INTERCEPT verified, VMAG threshold SNT_VMAG_THR=0.75"
```

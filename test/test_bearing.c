// test/test_bearing.c
#include "unity.h"
#include "bearing.h"
#include <math.h>

// AD8302 VPHS: maximum at 0° phase difference, ~0V at ±180°.
// vphs_zero_v = 1.79V = calibrated voltage at 0° (maximum VPHS).
// adc_raw at 1.79V = 1.79/3.3 * 4095 ≈ 2220

// When VPHS equals calibrated max (0° phase diff) → bearing = 0°
TEST_CASE("vphs_to_angle: vphs_zero input -> 0 degrees", "[bearing]") {
    float angle = vphs_to_angle(2220, 0.165f, 900e6f, 1.79f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, angle);
}

// When VPHS is below zero (off-axis) → angle > 0°
TEST_CASE("vphs_to_angle: vphs below zero -> positive angle", "[bearing]") {
    float angle = vphs_to_angle(1117, 0.165f, 900e6f, 1.79f);
    TEST_ASSERT_GREATER_THAN(0.0f, angle);
}

// When VPHS = 0 (maximum phase diff = maximum off-axis) → maximum angle
TEST_CASE("vphs_to_angle: vphs=0 -> max positive angle", "[bearing]") {
    float angle = vphs_to_angle(0, 0.165f, 900e6f, 1.79f);
    TEST_ASSERT_GREATER_THAN(0.0f, angle);
}

// Output always in [0, 90] — magnitude only
TEST_CASE("vphs_to_angle: clamps to [0, 90] degrees", "[bearing]") {
    float angle_max = vphs_to_angle(0,    0.165f, 900e6f, 1.79f);
    float angle_zero = vphs_to_angle(4095, 0.165f, 900e6f, 1.79f); // vphs > vphs_zero -> 0
    TEST_ASSERT_LESS_OR_EQUAL(90.0f, angle_max);
    TEST_ASSERT_GREATER_OR_EQUAL(0.0f, angle_max);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, angle_zero);
}

// Closer to zero-point = smaller angle
TEST_CASE("vphs_to_angle: larger rel deviation -> larger angle", "[bearing]") {
    // raw=2220 (vphs≈vphs_zero) → ~0°; raw=0 (vphs=0) → maximum angle
    float angle_near  = vphs_to_angle(2220, 0.165f, 900e6f, 1.79f);
    float angle_far   = vphs_to_angle(0,    0.165f, 900e6f, 1.79f);
    TEST_ASSERT_GREATER_THAN(angle_near, angle_far);
}

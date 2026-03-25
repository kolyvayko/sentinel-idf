// test/test_bearing.c
#include "unity.h"
#include "bearing.h"
#include <math.h>

// vphs_to_angle at midpoint (zero calibration = actual midpoint) must return 0 degrees
TEST_CASE("vphs_to_angle: midpoint -> 0 degrees", "[bearing]") {
    // adc_raw that gives exactly 0.9V: 0.9 / 3.3 * 4095 = 1117
    float angle = vphs_to_angle(1117, 0.165f, 900e6f, 0.9f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, angle);
}

// Full-scale right: VPHS = 1.8V -> adc_raw ~ 2234 -> positive angle
TEST_CASE("vphs_to_angle: max right -> positive angle", "[bearing]") {
    float angle = vphs_to_angle(2234, 0.165f, 900e6f, 0.9f);
    TEST_ASSERT_GREATER_THAN(0.0f, angle);
}

// Full-scale left: VPHS = 0V -> adc_raw = 0 -> negative angle
TEST_CASE("vphs_to_angle: max left -> negative angle", "[bearing]") {
    float angle = vphs_to_angle(0, 0.165f, 900e6f, 0.9f);
    TEST_ASSERT_LESS_THAN(0.0f, angle);
}

// Output must be clamped to [-90, +90]
TEST_CASE("vphs_to_angle: clamps to +/-90 degrees", "[bearing]") {
    float angle_max = vphs_to_angle(4095, 0.165f, 900e6f, 0.9f);
    float angle_min = vphs_to_angle(0,    0.165f, 900e6f, 0.9f);
    TEST_ASSERT_LESS_OR_EQUAL(90.0f,   angle_max);
    TEST_ASSERT_GREATER_OR_EQUAL(-90.0f, angle_min);
}

// Calibration offset shifts the zero point
TEST_CASE("vphs_to_angle: calibration offset applied", "[bearing]") {
    // raw=1117 -> 0.9V; with zero calibrated to 0.95V it should read negative
    float angle = vphs_to_angle(1117, 0.165f, 900e6f, 0.95f);
    TEST_ASSERT_LESS_THAN(0.0f, angle);
}

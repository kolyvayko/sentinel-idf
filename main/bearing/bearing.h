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
// adc_raw:           12-bit ADC count (0-4095)
// antenna_spacing_m: distance between antenna pair (metres)
// freq_hz:           target RF frequency (Hz)
// vphs_zero_v:       calibrated midpoint voltage (volts), typically 0.9
float vphs_to_angle(int adc_raw, float antenna_spacing_m,
                    float freq_hz, float vphs_zero_v);

void bearing_task(void *param);

#endif // BEARING_H

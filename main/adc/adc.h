#ifndef ADC_H
#define ADC_H

#include <esp_adc/adc_continuous.h>

adc_continuous_handle_t adc_init();
void adc_read_values(adc_continuous_handle_t handle, int *adc1, int *adc2);

typedef struct {
    int adc1;   // VPHS azimuth  (ADC1_CH1 / GPIO1)
    int adc2;   // VPHS elevation (ADC1_CH2 / GPIO2)
    int adc3;   // VMAG optional  (ADC1_CH4 / GPIO4) — always 0 until Phase 4
} adc_sample_t;
#define ADC_SAMPLE_T_DEFINED

#endif
#ifndef ADC_H
#define ADC_H

#include <esp_adc/adc_continuous.h>

adc_continuous_handle_t adc_init();
void adc_read_values(adc_continuous_handle_t handle, int *adc1, int *adc2);

#endif
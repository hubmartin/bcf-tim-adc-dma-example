#ifndef _ADC_H
#define _ADC_H

#include <stdbool.h>
#include <stdint.h>

void mic_init(void);
bool mic_start_measure(void);
bool mic_measure_is_done(void);

//uint8_t _adc_sync_read_8_bit();

#endif // _ADC_H

#ifndef _MIC_H
#define _MIC_H

#include <stdbool.h>
#include <stdint.h>

void mic_init(void);
bool mic_start_measure(void);
bool mic_measure_is_done(void);
float mic_get_rms(void);

#endif // _MIC_H

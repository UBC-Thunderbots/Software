#ifndef ADC_H
#define ADC_H

#include "util/log.h"

void adc_init(void);
void adc_tick(log_record_t *record);
float adc_vdd(void);
unsigned int adc_temperature(void);
float adc_battery(void);
float adc_battery_unfiltered(void);
float adc_capacitor(void);
float adc_breakbeam(void);
float adc_lps(void);

#endif

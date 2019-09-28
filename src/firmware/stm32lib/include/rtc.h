#ifndef STM32LIB_RTC_H
#define STM32LIB_RTC_H

#include <stdint.h>

uint64_t rtc_get(void);
void rtc_set(uint64_t stamp);

#endif

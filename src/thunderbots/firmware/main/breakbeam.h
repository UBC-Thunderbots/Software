#ifndef BREAKBEAM_H
#define BREAKBEAM_H

#include "log.h"
#include <stdbool.h>

float breakbeam_difference(void);
bool breakbeam_interrupted(void);
void breakbeam_tick(log_record_t *log);
void breakbeam_tick_fast(void);

#endif


#ifndef HALL_H
#define HALL_H

#include <stdint.h>

void hall_init(void);
void hall_tick(void);
void hall_lock_wheels(void);
void hall_lock_dribbler(void);
int16_t hall_speed(unsigned int index);

#endif

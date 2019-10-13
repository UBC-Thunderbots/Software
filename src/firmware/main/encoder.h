#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoder_init(void);
void encoder_tick(void);
void encoder_check_commutation_errors(void);
int16_t encoder_speed(unsigned int index);

#endif

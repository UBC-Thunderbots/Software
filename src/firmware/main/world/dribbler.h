#pragma once

#include <stdint.h>

struct Dribbler;
typedef struct Dribbler Dribbler;

Dribbler* Dribbler_create(void (*set_speed)(uint32_t rpm),
                              unsigned int (*get_temperature)(void));
void Dribbler_setSpeed(Dribbler* dribbler, uint32_t rpm);
unsigned int Dribbler_getTemperature(Dribbler* dribbler);
void Dribbler_destroy(Dribbler* dribbler);

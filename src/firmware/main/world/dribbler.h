#pragma once

#include <stdint.h>

struct Dribbler;
typedef struct Dribbler Dribbler;

Dribbler* Dribbler__construct(
    void (*set_speed)(uint32_t rpm),
    unsigned int (*get_temperature)(void)
    );
void Dribbler__setSpeed(Dribbler* this, uint32_t rpm);
unsigned int Dribbler__getTemperature(Dribbler* this);

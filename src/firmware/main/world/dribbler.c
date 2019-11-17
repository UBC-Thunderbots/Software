#include "world/dribbler.h"

#include <stdlib.h>

struct Dribbler {
    void (*set_speed)(uint32_t rpm);
    unsigned int (*get_temperature)(void);
};

Dribbler* Dribbler__construct(
    void (*set_speed)(uint32_t rpm),
    unsigned int (*get_temperature)(void)
){
    Dribbler* new_dribbler = malloc(sizeof(Dribbler));

    new_dribbler->set_speed = set_speed;

    return new_dribbler;
}

void Dribbler__setSpeed(Dribbler* this, uint32_t rpm){
    this->set_speed(rpm);
}

unsigned int Dribbler__getTemperature(Dribbler* this){
    return this->get_temperature();
}

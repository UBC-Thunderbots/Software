#include "world/dribbler.h"

#include <stdlib.h>

struct Dribbler
{
    void (*set_speed)(uint32_t rpm);
    unsigned int (*get_temperature_deg_c)(void);
};

Dribbler* Dribbler_create(void (*set_speed)(uint32_t rpm),
                              unsigned int (*get_temperature_deg_c)(void))
{
    Dribbler* new_dribbler = malloc(sizeof(Dribbler));

    new_dribbler->set_speed = set_speed;
    new_dribbler->get_temperature_deg_c = get_temperature_deg_c;

    return new_dribbler;
}

void Dribbler_destroy(Dribbler* dribbler){
    free(dribbler);
}

void Dribbler_setSpeed(Dribbler* dribbler, uint32_t rpm)
{
    dribbler->set_speed(rpm);
}

unsigned int Dribbler_getTemperature(Dribbler* dribbler)
{
    return dribbler->get_temperature_deg_c();
}

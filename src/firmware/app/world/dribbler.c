#include "firmware/app/world/dribbler.h"

#include <stdlib.h>

struct Dribbler
{
    void (*set_speed)(uint32_t rpm);
    void (*coast)(void);
    unsigned int (*get_temperature_deg_c)(void);
};

Dribbler_t* app_dribbler_create(void (*set_speed)(uint32_t rpm), void (*coast)(void),
                                unsigned int (*get_temperature_deg_c)(void))
{
    Dribbler_t* new_dribbler = (Dribbler_t*)malloc(sizeof(Dribbler_t));

    new_dribbler->set_speed             = set_speed;
    new_dribbler->coast                 = coast;
    new_dribbler->get_temperature_deg_c = get_temperature_deg_c;

    return new_dribbler;
}

void app_dribbler_destroy(Dribbler_t* dribbler)
{
    free(dribbler);
}

void app_dribbler_setSpeed(Dribbler_t* dribbler, uint32_t rpm)
{
    dribbler->set_speed(rpm);
}


void app_dribbler_coast(Dribbler_t* dribbler)
{
    dribbler->coast();
}

unsigned int app_dribbler_getTemperature(Dribbler_t* dribbler)
{
    return dribbler->get_temperature_deg_c();
}

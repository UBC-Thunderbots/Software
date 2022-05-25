#include "firmware/app/world/chicker.h"

#include <stdlib.h>

struct Chicker
{
    void (*kick)(float speed_m_per_s);
    void (*chip)(float distance_m);
    void (*enable_autokick)(float speed_m_per_s);
    void (*enable_autochip)(float distance_m);
    void (*disable_autokick)(void);
    void (*disable_autochip)(void);
};

Chicker_t* app_chicker_create(void (*kick)(float speed_m_per_s),
                              void (*chip)(float distance_m),
                              void (*enable_autokick)(float speed_m_per_s),
                              void (*enable_autochip)(float distance_m),
                              void (*disable_autokick)(void),
                              void (*disable_autochip)(void))
{
    Chicker_t* new_chicker = (Chicker_t*)malloc(sizeof(Chicker_t));

    new_chicker->kick             = kick;
    new_chicker->chip             = chip;
    new_chicker->enable_autochip  = enable_autochip;
    new_chicker->enable_autokick  = enable_autokick;
    new_chicker->disable_autokick = disable_autokick;
    new_chicker->disable_autochip = disable_autochip;

    return new_chicker;
}

void app_chicker_destroy(Chicker_t* chicker)
{
    free(chicker);
}

void app_chicker_kick(Chicker_t* chicker, float speed_m_per_s)
{
    chicker->kick(speed_m_per_s);
}

void app_chicker_chip(Chicker_t* chicker, float distance_m)
{
    chicker->chip(distance_m);
}

void app_chicker_enableAutokick(Chicker_t* chicker, float speed_m_per_s)
{
    chicker->enable_autokick(speed_m_per_s);
}

void app_chicker_enableAutochip(Chicker_t* chicker, float distance_m)
{
    chicker->enable_autochip(distance_m);
}

void app_chicker_disableAutokick(Chicker_t* chicker)
{
    chicker->disable_autokick();
}

void app_chicker_disableAutochip(Chicker_t* chicker)
{
    chicker->disable_autochip();
}

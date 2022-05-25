#include "firmware/app/world/charger.h"

#include <stdlib.h>

struct Charger
{
    void (*charge_capacitor)(void);
    void (*discharge_capacitor)(void);
    void (*float_capacitor)(void);
};

Charger_t* app_charger_create(void (*charge_capacitor)(void),
                              void (*discharge_capacitor)(void),
                              void (*float_capacitor)(void))
{
    Charger_t* new_charger = (Charger_t*)malloc(sizeof(Charger_t));

    new_charger->charge_capacitor    = charge_capacitor;
    new_charger->discharge_capacitor = discharge_capacitor;
    new_charger->float_capacitor     = float_capacitor;

    return new_charger;
}

void app_charger_destroy(Charger_t* charger)
{
    free(charger);
}

void app_charger_charge_capacitor(Charger_t* charger)
{
    charger->charge_capacitor();
}

void app_charger_discharge_capacitor(Charger_t* charger)
{
    charger->discharge_capacitor();
}

void app_charger_float_capacitor(Charger_t* charger)
{
    charger->float_capacitor();
}

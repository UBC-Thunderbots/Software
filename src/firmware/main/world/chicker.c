#include "world/chicker.h"

#include <stdlib.h>

struct Chicker
{
    void (*kick)(float speed_m_per_s);
    void (*chip)(float distance_m);
    void (*enable_autokick)(void);
    void (*enable_autochip)(void);
    void (*disable_autokick)(void);
    void (*disable_autochip)(void);
};

Chicker* Chicker__construct(void (*kick)(float speed_m_per_s),
                            void (*chip)(float distance_m), void (*enable_autokick)(void),
                            void (*enable_autochip)(void), void (*disable_autokick)(void),
                            void (*disable_autochip)(void)){
    Chicker* new_chicker = malloc(sizeof(Chicker));

    new_chicker->kick = kick;
    new_chicker->chip = chip;
    new_chicker->enable_autochip = enable_autochip;
    new_chicker->enable_autokick = enable_autokick;

    return new_chicker;
}

void Chicker__kick(Chicker* this, float speed_m_per_s){
    this->kick(speed_m_per_s);
}

void Chicker__chip(Chicker* this, float distance_m){
    this->chip(distance_m);
}

void Chicker__enableAutokick(Chicker* this){
    this->enable_autokick();
}

void Chicker__enableAutochip(Chicker* this){
    this->enable_autochip();
}

void Chicker__disableAutokick(Chicker* this){
    this->disable_autokick();
}

void Chicker__disableAutochip(Chicker* this){
    this->disable_autochip();
}

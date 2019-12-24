#include "app/world/wheel.h"

#include <stdlib.h>

struct Wheel
{
    void (*apply_wheel_force)(float force_in_newtons);
};

Wheel_t* app_wheel_create(void (*apply_wheel_force)(float force_in_newtons))
{
    Wheel_t* new_wheel = malloc(sizeof(Wheel_t));

    new_wheel->apply_wheel_force = apply_wheel_force;

    return new_wheel;
}

void app_wheel_destroy(Wheel_t* wheel)
{
    free(wheel);
}

void app_wheel_applyForce(Wheel_t* wheel, float force_in_newtons)
{
    wheel->apply_wheel_force(force_in_newtons);
}

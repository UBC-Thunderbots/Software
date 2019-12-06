#include "app/world/wheel.h"

#include <stdlib.h>

struct Wheel
{
    void (*apply_wheel_force)(float force_in_newtons);
};

Wheel* app_wheel_create(void (*apply_wheel_force)(float force_in_newtons))
{
    Wheel* new_wheel = malloc(sizeof(Wheel));

    new_wheel->apply_wheel_force = apply_wheel_force;

    return new_wheel;
}

void app_wheel_destroy(Wheel* wheel)
{
    free(wheel);
}

void app_wheel_applyForce(Wheel* wheel, float force_in_newtons)
{
    wheel->apply_wheel_force(force_in_newtons);
}

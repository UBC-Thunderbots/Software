#include "world/wheel.h"

#include <stdlib.h>

struct Wheel {
    void (*apply_wheel_force)(float force_in_newtons);
};

Wheel* Wheel_create(void (*apply_wheel_force)(float force_in_newtons)){
    Wheel* new_wheel = malloc(sizeof(Wheel));

    new_wheel->apply_wheel_force = apply_wheel_force;

    return new_wheel;
}

void Wheel_destroy(Wheel* wheel){
    free(wheel);
}

void Wheel_applyForce(Wheel* wheel, float force_in_newtons){
    wheel->apply_wheel_force(force_in_newtons);
}

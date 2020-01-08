#include "app/world/wheel.h"

#include <stdlib.h>

struct Wheel
{
    void (*apply_wheel_force)(float force_in_newtons);
    float (*get_wheel_speed_rpm)();
    WheelConstants_t wheel_constants;
};

Wheel_t* app_wheel_create(
    void (*apply_wheel_force)(float force_in_newtons), float (*get_wheel_speed_rpm)(),
    WheelConstants_t wheel_constants)
{
    Wheel_t* new_wheel = malloc(sizeof(Wheel_t));

    new_wheel->apply_wheel_force   = apply_wheel_force;
    new_wheel->get_wheel_speed_rpm = get_wheel_speed_rpm;
    new_wheel->wheel_constants     = wheel_constants;

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

float app_wheel_getWheelSpeedRPM(Wheel_t* wheel)
{
    return wheel->get_wheel_speed_rpm();
}

float app_wheel_getMotorSpeedRPM(Wheel_t* wheel)
{
    float gear_ratio = wheel->wheel_constants.motor_rotations_per_wheel_rotation;
    return gear_ratio * app_wheel_getWheelSpeedRPM(wheel);
}

const WheelConstants_t app_wheel_getWheelConstants(Wheel_t* wheel)
{
    return wheel->wheel_constants;
}

#include "firmware/app/world/force_wheel.h"

#include <stdlib.h>

struct ForceWheel
{
    void (*apply_wheel_force)(float force_in_newtons);
    float (*get_motor_speed_rpm)(void);
    void (*brake)(void);
    void (*coast)(void);
    WheelConstants_t wheel_constants;
};

ForceWheel_t* app_force_wheel_create(void (*apply_wheel_force)(float),
                                     float (*get_motor_speed_rpm)(void),
                                     void (*brake)(void), void (*coast)(void),
                                     WheelConstants_t wheel_constants)
{
    ForceWheel_t* new_wheel = malloc(sizeof(ForceWheel_t));

    new_wheel->apply_wheel_force   = apply_wheel_force;
    new_wheel->get_motor_speed_rpm = get_motor_speed_rpm;
    new_wheel->wheel_constants     = wheel_constants;
    new_wheel->brake               = brake;
    new_wheel->coast               = coast;

    return new_wheel;
}

void app_force_wheel_destroy(ForceWheel_t* wheel)
{
    free(wheel);
}

void app_force_wheel_applyForce(ForceWheel_t* wheel, float force_in_newtons)
{
    wheel->apply_wheel_force(force_in_newtons);
}

float app_force_wheel_getWheelSpeedRPM(ForceWheel_t* wheel)
{
    float gear_ratio = wheel->wheel_constants.wheel_rotations_per_motor_rotation;
    return wheel->get_motor_speed_rpm() * gear_ratio;
}

float app_force_wheel_getMotorSpeedRPM(const ForceWheel_t* wheel)
{
    return wheel->get_motor_speed_rpm();
}

void app_force_wheel_coast(const ForceWheel_t* wheel)
{
    wheel->coast();
}

void app_force_wheel_brake(const ForceWheel_t* wheel)
{
    wheel->brake();
}

WheelConstants_t app_force_wheel_getWheelConstants(const ForceWheel_t* wheel)
{
    return wheel->wheel_constants;
}

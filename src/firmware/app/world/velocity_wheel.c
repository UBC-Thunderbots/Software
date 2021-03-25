#include <stdlib.h>

#include "firmware/app/world/velocity_wheel.h"

struct VelocityWheel
{
    void (*apply_wheel_force)(float force_in_newtons);
    void (*set_target_velocity)(float velocity);
    float (*get_motor_speed_rpm)(void);
    void (*brake)(void);
    void (*coast)(void);
    VelocityWheelConstants_t wheel_constants;
};

VelocityWheel_t* app_velocity_wheel_create(void (*apply_wheel_force)(float),
                                     void (*set_target_velocity)(float),
                                     float (*get_motor_speed_rpm)(void),
                                     void (*brake)(void), void (*coast)(void),
                                     VelocityWheelConstants_t wheel_constants)
{
    VelocityWheel_t* new_wheel = malloc(sizeof(VelocityWheel_t));

    new_wheel->apply_wheel_force   = apply_wheel_force;
    new_wheel->set_target_velocity = set_target_velocity;
    new_wheel->get_motor_speed_rpm = get_motor_speed_rpm;
    new_wheel->wheel_constants     = wheel_constants;
    new_wheel->brake               = brake;
    new_wheel->coast               = coast;

    return new_wheel;
}

void app_velocity_wheel_destroy(VelocityWheel_t* wheel)
{
    free(wheel);
}

void app_velocity_wheel_applyForce(VelocityWheel_t* wheel, float force_in_newtons)
{
    wheel->apply_wheel_force(force_in_newtons);
}

float app_velocity_wheel_getWheelSpeedRPM(VelocityWheel_t* wheel)
{
    float gear_ratio = wheel->wheel_constants.wheel_rotations_per_motor_rotation;
    return wheel->get_motor_speed_rpm() * gear_ratio;
}

float app_velocity_wheel_getMotorSpeedRPM(const VelocityWheel_t* wheel)
{
    return wheel->get_motor_speed_rpm();
}

void app_velocity_wheel_coast(const VelocityWheel_t* wheel)
{
    wheel->coast();
}

void app_velocity_wheel_brake(const VelocityWheel_t* wheel)
{
    wheel->brake();
}

void app_velocity_wheel_setTargetVelocity(VelocityWheel_t* wheel, float velocity)
{
    wheel->set_target_velocity(velocity);
}

VelocityWheelConstants_t app_velocity_wheel_getWheelConstants(const VelocityWheel_t* wheel)
{
    return wheel->wheel_constants;
}

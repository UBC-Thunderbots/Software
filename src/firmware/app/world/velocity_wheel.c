#include "firmware/app/world/velocity_wheel.h"

#include <math.h>
#include <stdlib.h>

struct VelocityWheel
{
    void (*set_target_rpm)(float rpm);
    float (*get_motor_speed_rpm)(void);
    void (*brake)(void);
    void (*coast)(void);
    WheelConstants_t wheel_constants;
};

VelocityWheel_t* app_velocity_wheel_create(void (*set_target_rpm)(float),
                                           float (*get_motor_speed_rpm)(void),
                                           void (*brake)(void), void (*coast)(void),
                                           WheelConstants_t wheel_constants)
{
    VelocityWheel_t* new_wheel = malloc(sizeof(VelocityWheel_t));

    new_wheel->set_target_rpm      = set_target_rpm;
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

void app_velocity_wheel_setTargetVelocity(VelocityWheel_t* wheel, float velocity_m_per_s)
{
    float rpm = velocity_m_per_s * 60 /
                (2 * (float)M_PI * wheel->wheel_constants.wheel_radius_meters);
    wheel->set_target_rpm(rpm);
}

void app_velocity_wheel_setTargetRPM(VelocityWheel_t* wheel, float rpm)
{
    wheel->set_target_rpm(rpm);
}

WheelConstants_t app_velocity_wheel_getWheelConstants(const VelocityWheel_t* wheel)
{
    return wheel->wheel_constants;
}

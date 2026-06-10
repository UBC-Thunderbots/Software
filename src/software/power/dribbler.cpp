#include "dribbler.h"

#include <Arduino.h>

#include <algorithm>

#include "pins.h"

Dribbler::Dribbler()
{
    pinMode(DRIBBLER_PIN, OUTPUT);
}

void Dribbler::setTargetSpeed(uint32_t speed_rpm)
{
    target_speed_rpm_ = std::min(speed_rpm, MAX_SPEED_RPM);
}

void Dribbler::update()
{
    if (target_speed_rpm_ <= current_speed_rpm_)
    {
        current_speed_rpm_ = target_speed_rpm_;
    }
    else
    {
        current_speed_rpm_ = current_speed_rpm_ +
                             (target_speed_rpm_ - current_speed_rpm_) / RAMP_FACTOR + 1;
    }

    const uint32_t pwm_duty_cycle = current_speed_rpm_ * UINT8_MAX / MAX_SPEED_RPM;
    analogWrite(DRIBBLER_PIN, pwm_duty_cycle);
}

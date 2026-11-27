#include "dribbler_controller.h"

#include <algorithm>
#include <limits>

DribblerController::DribblerController(PowerHAL& hal) : hal_(hal) {}

void DribblerController::setTargetSpeed(uint32_t speed_rpm)
{
    target_speed_rpm_ = std::min(speed_rpm, MAX_SPEED_RPM);
}

void DribblerController::update()
{
    if (target_speed_rpm_ <= current_speed_rpm_)
    {
        current_speed_rpm_ = target_speed_rpm_;
    }
    else
    {
        current_speed_rpm_ += (target_speed_rpm_ - current_speed_rpm_) / RAMP_FACTOR + 1;
    }

    const uint8_t duty_cycle = static_cast<uint8_t>(
        current_speed_rpm_ * std::numeric_limits<uint8_t>::max() / MAX_SPEED_RPM);
    hal_.setDribblerPwm(duty_cycle);
}

#pragma once

#include <cstdint>

#include "power_hal.h"

/** Applies the dribbler speed limit and ramps the requested speed into PWM. */
class DribblerController
{
   public:
    explicit DribblerController(PowerHAL& hal);

    void setTargetSpeed(uint32_t speed_rpm);
    void update();

   private:
    PowerHAL& hal_;
    uint32_t target_speed_rpm_  = 0;
    uint32_t current_speed_rpm_ = 0;

    static constexpr uint32_t MAX_SPEED_RPM = 11040;
    static constexpr uint32_t RAMP_FACTOR   = 4;
};

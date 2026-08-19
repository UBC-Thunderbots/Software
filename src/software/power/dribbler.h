#pragma once

#include <cstdint>

/**
 * Represents the dribbler on the power board
 */
class Dribbler
{
   public:
    /**
     * Creates a dribbler, setting up pins.
     */
    Dribbler();

    /**
     * Updates the desired dribbler speed in RPM.
     */
    void setTargetSpeed(uint32_t speed_rpm);

    /**
     * Advances the ramp and applies the resulting PWM output.
     */
    void update();

   private:
    static constexpr uint32_t MAX_SPEED_RPM = 11040;
    static constexpr uint32_t RAMP_FACTOR   = 4;

    uint32_t target_speed_rpm_  = 0;
    uint32_t current_speed_rpm_ = 0;
};

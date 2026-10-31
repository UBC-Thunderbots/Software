#pragma once

#include <cstdint>

#include "power_hal.h"

/** Transitional power logic, intended to be replaced by the Boost.SML state machine. */
class PowerController
{
   public:
    explicit PowerController(PowerHAL& hal);
    ~PowerController();

    // Chicker policy.
    void kick(uint32_t pulse_width_us);
    void chip(uint32_t pulse_width_us);
    void autokick(uint32_t pulse_width_us);
    void autochip(uint32_t pulse_width_us);

    // Charging and dribbler policies.
    void setDribblerTargetSpeed(uint32_t speed_rpm);
    void update();

   private:
    enum class ChargeToggleState : uint8_t
    {
        Idle,
        WaitToDisable,
        WaitToEnable,
    };

    // Chicker and charging sequencing.
    void startChickerPulse(ChickerOutput output, uint32_t pulse_width_us);
    void restartCapacitorCharging(uint32_t now_us);

    PowerHAL& hal_;
    bool pulsing_                          = false;
    bool on_cooldown_                      = false;
    bool charging_                         = false;
    ChargeToggleState charge_toggle_state_ = ChargeToggleState::Idle;
    uint32_t cooldown_started_us_          = 0;
    uint32_t charge_toggle_started_us_     = 0;
    uint32_t charge_started_us_            = 0;
    uint32_t target_dribbler_rpm_          = 0;
    uint32_t current_dribbler_rpm_         = 0;

    static constexpr uint32_t COOLDOWN_US            = 3000000;
    static constexpr uint32_t CHARGE_TOGGLE_DELAY_US = 5000;
    static constexpr uint32_t MAX_CHARGE_DURATION_US = 2000000;
    static constexpr uint32_t MAX_DRIBBLER_SPEED_RPM = 11040;
    static constexpr uint32_t DRIBBLER_RAMP_FACTOR   = 4;
};

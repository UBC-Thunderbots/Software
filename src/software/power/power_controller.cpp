#include "power_controller.h"

#include <algorithm>
#include <limits>

PowerController::PowerController(PowerHAL& hal) : hal_(hal)
{
    hal_.setCapacitorCharging(true);
    charging_          = true;
    charge_started_us_ = hal_.micros();
}

PowerController::~PowerController()
{
    hal_.stopChickerPulse();
}

// Chicker policy.
void PowerController::kick(uint32_t pulse_width_us)
{
    startChickerPulse(ChickerOutput::Kicker, pulse_width_us);
}

void PowerController::chip(uint32_t pulse_width_us)
{
    startChickerPulse(ChickerOutput::Chipper, pulse_width_us);
}

void PowerController::autokick(uint32_t pulse_width_us)
{
    if (hal_.getBreakBeamTripped())
        kick(pulse_width_us);
}

void PowerController::autochip(uint32_t pulse_width_us)
{
    if (hal_.getBreakBeamTripped())
        chip(pulse_width_us);
}

void PowerController::startChickerPulse(ChickerOutput output, uint32_t pulse_width_us)
{
    if (pulsing_ || on_cooldown_ || !hal_.startChickerPulse(output, pulse_width_us))
    {
        return;
    }

    pulsing_             = true;
    on_cooldown_         = true;
    cooldown_started_us_ = hal_.micros();
}

// Charging policy.
void PowerController::restartCapacitorCharging(uint32_t now_us)
{
    charge_toggle_state_      = ChargeToggleState::WaitToDisable;
    charge_toggle_started_us_ = now_us;
}

// Dribbler policy.
void PowerController::setDribblerTargetSpeed(uint32_t speed_rpm)
{
    target_dribbler_rpm_ = std::min(speed_rpm, MAX_DRIBBLER_SPEED_RPM);
}

void PowerController::update()
{
    const uint32_t now_us = hal_.micros();

    if (pulsing_ && !hal_.isChickerPulseActive())
    {
        pulsing_ = false;
        restartCapacitorCharging(now_us);
    }

    if (!pulsing_ && on_cooldown_ &&
        static_cast<uint32_t>(now_us - cooldown_started_us_) >= COOLDOWN_US)
    {
        on_cooldown_ = false;
    }

    if (charge_toggle_state_ == ChargeToggleState::WaitToDisable &&
        static_cast<uint32_t>(now_us - charge_toggle_started_us_) >=
            CHARGE_TOGGLE_DELAY_US)
    {
        hal_.setCapacitorCharging(false);
        charging_                 = false;
        charge_toggle_state_      = ChargeToggleState::WaitToEnable;
        charge_toggle_started_us_ = now_us;
    }
    else if (charge_toggle_state_ == ChargeToggleState::WaitToEnable &&
             static_cast<uint32_t>(now_us - charge_toggle_started_us_) >=
                 CHARGE_TOGGLE_DELAY_US)
    {
        hal_.setCapacitorCharging(true);
        charging_            = true;
        charge_started_us_   = now_us;
        charge_toggle_state_ = ChargeToggleState::Idle;
    }

    if (charging_ && !hal_.isCapacitorChargeDone() &&
        static_cast<uint32_t>(now_us - charge_started_us_) >= MAX_CHARGE_DURATION_US)
    {
        hal_.setCapacitorCharging(false);
        charging_ = false;
    }

    if (target_dribbler_rpm_ <= current_dribbler_rpm_)
    {
        current_dribbler_rpm_ = target_dribbler_rpm_;
    }
    else
    {
        current_dribbler_rpm_ +=
            (target_dribbler_rpm_ - current_dribbler_rpm_) / DRIBBLER_RAMP_FACTOR + 1;
    }

    const uint8_t duty_cycle =
        static_cast<uint8_t>(current_dribbler_rpm_ * std::numeric_limits<uint8_t>::max() /
                             MAX_DRIBBLER_SPEED_RPM);
    hal_.setDribblerPwm(duty_cycle);
}

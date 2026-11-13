#include "power_controller.h"

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
    if (pulse_width_us > MAX_CHICKER_PULSE_WIDTH_US || pulsing_ || on_cooldown_ ||
        !hal_.startChickerPulse(output, pulse_width_us))
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
}

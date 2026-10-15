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
    hal_.cancelTimer(PowerTimer::ChickerCooldown);
    hal_.cancelTimer(PowerTimer::CapacitorChargeToggle);
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
    if (on_cooldown_ || !hal_.startChickerPulse(output, pulse_width_us,
                                                &PowerController::onTimerExpired, this))
    {
        return;
    }

    on_cooldown_ = true;
    // Stay locked out if the cooldown timer could not be armed.
    hal_.startOneShotTimer(PowerTimer::ChickerCooldown, COOLDOWN_US,
                           &PowerController::onTimerExpired, this);
    restartCapacitorCharging();
}

// Charging policy.
void PowerController::restartCapacitorCharging()
{
    charge_toggle_should_enable_ = false;
    hal_.startOneShotTimer(PowerTimer::CapacitorChargeToggle, CHARGE_TOGGLE_DELAY_US,
                           &PowerController::onTimerExpired, this);
}

void PowerController::onTimerExpired(void* context, PowerTimer timer)
{
    static_cast<PowerController*>(context)->handleTimerExpired(timer);
}

void PowerController::handleTimerExpired(PowerTimer timer)
{
    switch (timer)
    {
        case PowerTimer::ChickerPulse:
            break;
        case PowerTimer::ChickerCooldown:
            on_cooldown_ = false;
            break;
        case PowerTimer::CapacitorChargeToggle:
            if (!charge_toggle_should_enable_)
            {
                hal_.setCapacitorCharging(false);
                charging_                    = false;
                charge_toggle_should_enable_ = true;
                hal_.startOneShotTimer(PowerTimer::CapacitorChargeToggle,
                                       CHARGE_TOGGLE_DELAY_US,
                                       &PowerController::onTimerExpired, this);
            }
            else
            {
                hal_.setCapacitorCharging(true);
                charging_                    = true;
                charge_started_us_           = hal_.micros();
                charge_toggle_should_enable_ = false;
            }
            break;
    }
}

// Dribbler policy.
void PowerController::setDribblerTargetSpeed(uint32_t speed_rpm)
{
    target_dribbler_rpm_ = std::min(speed_rpm, MAX_DRIBBLER_SPEED_RPM);
}

void PowerController::update()
{
    if (charging_ && !hal_.isCapacitorChargeDone() &&
        static_cast<uint32_t>(hal_.micros() - charge_started_us_) >=
            MAX_CHARGE_DURATION_US)
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

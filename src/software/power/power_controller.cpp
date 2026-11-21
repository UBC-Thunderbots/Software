#include "power_controller.h"

PowerController::PowerController(PowerHAL& hal)
    : hal_(hal), fsm_(PowerStateMachine{hal}), charge_started_us_(hal.micros())
{
}

PowerController::~PowerController()
{
    hal_.stopChickerPulse();
}

bool PowerController::isValidPulseWidth(uint32_t pulse_width_us) const
{
    return pulse_width_us > 0 && pulse_width_us <= MAX_CHICKER_PULSE_WIDTH_US;
}

void PowerController::requestPulse(ChickerOutput output, uint32_t pulse_width_us)
{
    if (!isValidPulseWidth(pulse_width_us))
    {
        return;
    }

    // Only the chicker region handles KickRequested. The transition succeeds
    // only when the HAL actually starts a pulse (the FSM guard checks this).
    if (fsm_.process_event(PowerStateMachine::KickRequested{output, pulse_width_us}))
    {
        cooldown_started_us_ = hal_.micros();
    }
}

void PowerController::requestArm(ChickerOutput output, uint32_t pulse_width_us)
{
    if (!isValidPulseWidth(pulse_width_us))
    {
        return;
    }

    if (fsm_.process_event(PowerStateMachine::ArmRequested{output, pulse_width_us}))
    {
        arm_refreshed_us_ = hal_.micros();
    }
}

void PowerController::kick(uint32_t pulse_width_us)
{
    requestPulse(ChickerOutput::Kicker, pulse_width_us);
}

void PowerController::chip(uint32_t pulse_width_us)
{
    requestPulse(ChickerOutput::Chipper, pulse_width_us);
}

void PowerController::autokick(uint32_t pulse_width_us)
{
    requestArm(ChickerOutput::Kicker, pulse_width_us);
}

void PowerController::autochip(uint32_t pulse_width_us)
{
    requestArm(ChickerOutput::Chipper, pulse_width_us);
}

void PowerController::update()
{
    const uint32_t now_us = hal_.micros();

    // A short hardware pulse may start and finish between update() calls.
    // Pulsing is our proof that the pulse really did start earlier.
    if (isInState<PowerStateMachine::Pulsing>() && !hal_.isChickerPulseActive())
    {
        fsm_.process_event(PowerStateMachine::PulseFinished{});
        restart_stage_started_us_ = now_us;
    }

    // The cooldown deadline is measured from the start of the pulse.
    if (isInState<PowerStateMachine::Cooldown>() &&
        static_cast<uint32_t>(now_us - cooldown_started_us_) >= COOLDOWN_US)
    {
        fsm_.process_event(PowerStateMachine::CooldownFinished{});
    }

    if (isInState<PowerStateMachine::Armed>())
    {
        if (static_cast<uint32_t>(now_us - arm_refreshed_us_) >= ARM_TIMEOUT_US)
        {
            fsm_.process_event(PowerStateMachine::ArmExpired{});
        }
        else if (hal_.getBreakBeamTripped() &&
                 fsm_.process_event(PowerStateMachine::BeamTripped{}))
        {
            // Record only a successful automatic pulse, not the arm request.
            cooldown_started_us_ = hal_.micros();
        }
    }

    // Preserve the original 5 ms wait, LOW, 5 ms wait, HIGH sequence.
    // Both delays remain provisional pending hardware/team verification.
    if (isInState<PowerStateMachine::Restarting>() &&
        static_cast<uint32_t>(now_us - restart_stage_started_us_) >=
            CHARGE_TOGGLE_DELAY_US)
    {
        fsm_.process_event(PowerStateMachine::RestartDisableElapsed{});
        restart_stage_started_us_ = now_us;
    }
    else if (isInState<PowerStateMachine::RestartLow>() &&
             static_cast<uint32_t>(now_us - restart_stage_started_us_) >=
                 CHARGE_TOGGLE_DELAY_US)
    {
        fsm_.process_event(PowerStateMachine::RestartEnableElapsed{});
        charge_started_us_ = now_us;
    }
    else if (isInState<PowerStateMachine::Charging>())
    {
        if (hal_.isCapacitorChargeDone())
        {
            fsm_.process_event(PowerStateMachine::ChargeCompleted{});
        }
        else if (static_cast<uint32_t>(now_us - charge_started_us_) >=
                 MAX_CHARGE_DURATION_US)
        {
            fsm_.process_event(PowerStateMachine::ChargeTimedOut{});
        }
    }
}

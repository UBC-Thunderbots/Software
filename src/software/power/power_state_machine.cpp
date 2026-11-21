#include "power_state_machine.h"

PowerStateMachine::PowerStateMachine(PowerHAL& hal) : hal_(hal)
{
    // The original PowerController starts charging on construction.
    hal_.setCapacitorCharging(true);
}

bool PowerStateMachine::tryStartPulse(const KickRequested& event)
{
    // Width was validated by PowerController. Hardware may still refuse firing.
    return hal_.startChickerPulse(event.output, event.pulse_width_us);
}

bool PowerStateMachine::tryStartArmedPulse(const BeamTripped&)
{
    // The arm request was validated before we stored it.
    return hal_.startChickerPulse(armed_output_, armed_pulse_width_us_);
}

void PowerStateMachine::rememberArm(const ArmRequested& event)
{
    armed_output_         = event.output;
    armed_pulse_width_us_ = event.pulse_width_us;
}

void PowerStateMachine::setCharging(bool enabled)
{
    hal_.setCapacitorCharging(enabled);
}

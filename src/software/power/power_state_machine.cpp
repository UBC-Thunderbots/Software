#include "power_state_machine.h"

PowerStateMachine::PowerStateMachine(PowerHAL& hal) : hal_(hal)
{
    // The original PowerController starts charging on construction.
    hal_.setCapacitorCharging(true);
}

bool PowerStateMachine::tryStartPulse(const PulseRequested& event)
{
    // Width was validated by PowerController. Hardware may still refuse firing.
    return hal_.startChickerPulse(event.output, event.pulse_width_us);
}

void PowerStateMachine::setCharging(bool enabled)
{
    hal_.setCapacitorCharging(enabled);
}

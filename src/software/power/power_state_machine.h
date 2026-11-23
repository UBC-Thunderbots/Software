#pragma once

#include <cstdint>

#ifdef PLATFORMIO_BUILD
#include <sml_headers.h>
#else
#include <include/boost/sml.hpp>
#endif

#include "power_hal.h"

/** Chicker and capacitor-charger state transitions (two parallel SML regions). */
class PowerStateMachine
{
   public:
    explicit PowerStateMachine(PowerHAL& hal);

    // Events contain validated requests or report observations made by the controller.
    struct PulseRequested
    {
        ChickerOutput output;
        uint32_t pulse_width_us;
        bool require_break_beam;
    };
    struct PulseFinished
    {
    };
    struct CooldownFinished
    {
    };
    struct ChargeCompleted
    {
    };
    struct ChargeTimedOut
    {
    };
    struct RestartDisableElapsed
    {
    };
    struct RestartEnableElapsed
    {
    };

    // Chicker region.
    struct Ready
    {
    };
    struct Pulsing
    {
    };
    struct Cooldown
    {
    };

    // Charger region.
    struct Charging
    {
    };
    struct Charged
    {
    };
    struct Restarting
    {
    };  // Waiting before lowering CHRG.
    struct RestartLow
    {
    };  // CHRG is LOW; waiting before raising it.
    struct TimedOut
    {
    };

    auto operator()()
    {
        using namespace boost::sml;

        // Auto requests only fire if the beam is tripped when the command arrives.
        // The HAL also decides whether the pulse can actually start.
        const auto pulseStarted = [this](const PulseRequested& event)
        {
            return (!event.require_break_beam || hal_.getBreakBeamTripped()) &&
                   tryStartPulse(event);
        };

        const auto lowerChargePin = [this] { setCharging(false); };

        const auto raiseChargePin = [this] { setCharging(true); };

        return make_transition_table(
            // Manual and automatic requests share the same ready-to-pulse transition.
            *state<Ready> + event<PulseRequested>[pulseStarted] = state<Pulsing>,
            state<Pulsing> + event<PulseFinished>               = state<Cooldown>,
            state<Cooldown> + event<CooldownFinished>           = state<Ready>,

            // Charger: reacts independently to the same PulseFinished event.
            *state<Charging> + event<ChargeCompleted>                = state<Charged>,
            state<Charging> + event<ChargeTimedOut> / lowerChargePin = state<TimedOut>,

            state<Charging> + event<PulseFinished> = state<Restarting>,
            state<Charged> + event<PulseFinished>  = state<Restarting>,
            state<TimedOut> + event<PulseFinished> = state<Restarting>,

            // Preserve the two existing 5 ms waits; timing stays in controller.
            state<Restarting> + event<RestartDisableElapsed> / lowerChargePin =
                state<RestartLow>,

            state<RestartLow> + event<RestartEnableElapsed> / raiseChargePin =
                state<Charging>);
    }

    bool tryStartPulse(const PulseRequested& event);
    void setCharging(bool enabled);

   private:
    PowerHAL& hal_;
};

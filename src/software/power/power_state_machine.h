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
    struct KickRequested
    {
        ChickerOutput output;
        uint32_t pulse_width_us;
    };

    struct ArmRequested
    {
        ChickerOutput output;
        uint32_t pulse_width_us;
    };

    struct DisarmRequested {};
    struct BeamTripped {};
    struct ArmExpired {};
    struct PulseFinished {};
    struct CooldownFinished {};
    struct ChargeCompleted {};
    struct ChargeTimedOut {};
    struct RestartDisableElapsed {};
    struct RestartEnableElapsed {};

    // Chicker region.
    struct Ready {};
    struct Armed {};
    struct Pulsing {};
    struct Cooldown {};

    // Charger region.
    struct Charging {};
    struct Charged {};
    struct Restarting {};  // Waiting before lowering CHRG.
    struct RestartLow {};  // CHRG is LOW; waiting before raising it.
    struct TimedOut {};

    auto operator()()
    {
        using namespace boost::sml;

        // These guards deliberately attempt the pulse. The HAL returns false
        // if it cannot fire, so the FSM must not enter Pulsing in that case.
        // Pulse-width validation happens in PowerController.
        const auto pulseStarted = [this](const KickRequested& event) {
            return tryStartPulse(event);
        };

        const auto armedPulseStarted = [this](const BeamTripped& event) {
            return tryStartArmedPulse(event);
        };

        const auto rememberArmRequest = [this](const ArmRequested& event) {
            rememberArm(event);
        };

        const auto lowerChargePin = [this] {
            setCharging(false);
        };

        const auto raiseChargePin = [this] {
            setCharging(true);
        };

        return make_transition_table(
            // Chicker: manual commands can fire from Ready or Armed.
            *state<Ready> + event<KickRequested>
                [pulseStarted] = state<Pulsing>,

            state<Ready> + event<ArmRequested>
                / rememberArmRequest = state<Armed>,

            state<Armed> + event<ArmRequested>
                / rememberArmRequest,

            state<Armed> + event<KickRequested>
                [pulseStarted] = state<Pulsing>,

            state<Armed> + event<BeamTripped>
                [armedPulseStarted] = state<Pulsing>,

            state<Armed> + event<ArmExpired> = state<Ready>,
            state<Armed> + event<DisarmRequested> = state<Ready>,
            state<Pulsing> + event<PulseFinished> = state<Cooldown>,
            state<Cooldown> + event<CooldownFinished> = state<Ready>,

            // Charger: reacts independently to the same PulseFinished event.
            *state<Charging> + event<ChargeCompleted> = state<Charged>,
            state<Charging> + event<ChargeTimedOut>
                / lowerChargePin = state<TimedOut>,

            state<Charging> + event<PulseFinished> = state<Restarting>,
            state<Charged> + event<PulseFinished> = state<Restarting>,
            state<TimedOut> + event<PulseFinished> = state<Restarting>,

            // Preserve the two existing 5 ms waits; timing stays in controller.
            state<Restarting> + event<RestartDisableElapsed>
                / lowerChargePin = state<RestartLow>,

            state<RestartLow> + event<RestartEnableElapsed>
                / raiseChargePin = state<Charging>
        );
    }

    bool tryStartPulse(const KickRequested& event);
    bool tryStartArmedPulse(const BeamTripped& event);
    void rememberArm(const ArmRequested& event);
    void setCharging(bool enabled);

   private:
    PowerHAL& hal_;
    ChickerOutput armed_output_     = ChickerOutput::Kicker;
    uint32_t armed_pulse_width_us_ = 0;
};

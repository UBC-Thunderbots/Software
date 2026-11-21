#pragma once

#include <cstdint>
#include <type_traits>

#include "power_hal.h"
#include "power_state_machine.h"

/** Converts power requests and sensor/timer observations into FSM events. */
class PowerController
{
   public:
    explicit PowerController(PowerHAL& hal);
    ~PowerController();

    void kick(uint32_t pulse_width_us);
    void chip(uint32_t pulse_width_us);
    void autokick(uint32_t pulse_width_us);
    void autochip(uint32_t pulse_width_us);

    void update();

   private:
    // SML's single-argument is(state<X>) only tests the first parallel region
    // in the team's pinned version. Visit both regions instead.
    template <typename State>
    bool isInState() const
    {
        bool active = false;
        fsm_.visit_current_states([&active](auto current) {
            active |= std::is_same<typename decltype(current)::type, State>::value;
        });
        return active;
    }

    bool isValidPulseWidth(uint32_t pulse_width_us) const;
    void requestPulse(ChickerOutput output, uint32_t pulse_width_us);
    void requestArm(ChickerOutput output, uint32_t pulse_width_us);

    PowerHAL& hal_;
    boost::sml::sm<PowerStateMachine> fsm_;

    // Timing only. Operating states belong to the FSM, not to these members.
    uint32_t cooldown_started_us_ = 0;
    uint32_t charge_started_us_   = 0;
    uint32_t restart_stage_started_us_ = 0;
    uint32_t arm_refreshed_us_    = 0;

    uint32_t target_dribbler_rpm_  = 0;
    uint32_t current_dribbler_rpm_ = 0;

    // Values carried over from the original power_controller.h.
    static constexpr uint32_t COOLDOWN_US                = 3000000;
    static constexpr uint32_t CHARGE_TOGGLE_DELAY_US     = 5000;  // TODO: Verify with team.
    static constexpr uint32_t MAX_CHARGE_DURATION_US     = 2000000;
    static constexpr uint32_t MAX_CHICKER_PULSE_WIDTH_US = 20000;
};


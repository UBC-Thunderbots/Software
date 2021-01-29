#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/stop_intent.h"

struct StopFSM
{
    class idle_state;
    class stop_state;

    struct ControlParams
    {
        bool coast;
    };


    UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        const auto idle_s   = state<idle_state>;
        const auto stop_s   = state<stop_state>;
        const auto update_e = event<Update>;

        const auto update_stop = [](auto event) {
            event.common.set_intent(std::make_unique<StopIntent>(
                event.common.robot.id(), event.control_params.coast));
        };

        const auto stop_done = [](auto event) {
            return robotStopped(event.common.robot);
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *idle_s + update_e / update_stop            = stop_s,
            stop_s + update_e[!stop_done] / update_stop = stop_s,
            stop_s + update_e[stop_done] / update_stop  = X,
            X + update_e[!stop_done] / update_stop      = X);
    }
};

#pragma once

#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/ai/passing/pass.h"

struct PasserFSM
{
    struct ControlParams
    {
        // The pass to execute
        std::optional<Pass> pass = std::nullopt;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        const auto dribble_s = state<DribbleFSM>;
        const auto kick_s    = state<KickFSM>;
        const auto update_e  = event<Update>;

        /**
         * Action that updates the DribbleFSM
         *
         * @param event PasserFSM::Update event
         * @param processEvent processes the DribbleFSM::Update
         */
        const auto update_dribble = [](auto event,
                                       back::process<DribbleFSM::Update> processEvent) {
            if (event.control_params.pass)
            {
                DribbleFSM::ControlParams control_params{
                    .dribble_destination = std::make_optional<Point>(
                        event.control_params.pass->passerPoint()),
                    .final_dribble_orientation = std::make_optional<Angle>(
                        event.control_params.pass->passerOrientation()),
                    .allow_excessive_dribbling = false};

                // Update the dribble fsm
                processEvent(DribbleFSM::Update(control_params, event.common));
            }
        };

        /**
         * Action that updates the KickFSM
         *
         * @param event PasserFSM::Update event
         * @param processEvent processes the KickFSM::Update
         */
        const auto update_kick = [](auto event,
                                    back::process<KickFSM::Update> processEvent) {
            if (event.control_params.pass)
            {
                KickFSM::ControlParams control_params{
                    .kick_origin    = event.control_params.pass->passerPoint(),
                    .kick_direction = event.control_params.pass->passerOrientation(),
                    .kick_speed_meters_per_second = event.control_params.pass->speed()};

                // Update the kick fsm
                processEvent(KickFSM::Update(control_params, event.common));
            }
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *dribble_s + update_e / update_dribble, dribble_s = kick_s,
            kick_s + update_e / update_kick, kick_s           = X);
    }
};

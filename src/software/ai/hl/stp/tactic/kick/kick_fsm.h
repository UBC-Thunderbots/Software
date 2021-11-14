#pragma once

#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/kick_intent.h"

struct KickFSM
{
   public:
    class KickState;

    struct ControlParams
    {
        // The location where the kick will be taken from
        Point kick_origin;
        // The direction the Robot will kick in
        Angle kick_direction;
        // How fast the Robot will kick the ball in meters per second
        double kick_speed_meters_per_second;
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        const auto get_behind_ball_s = state<GetBehindBallFSM>;
        const auto kick_s            = state<KickState>;
        const auto update_e          = event<Update>;

        /**
         * Action that updates the KickIntent
         *
         * @param event KickFSM::Update event
         */
        const auto update_kick = [](auto event) {
            event.common.set_intent(std::make_unique<KickIntent>(
                event.common.robot.id(), event.control_params.kick_origin,
                event.control_params.kick_direction,
                event.control_params.kick_speed_meters_per_second,
                event.common.robot.robotConstants()));
        };

        /**
         * Action that updates the GetBehindBallFSM
         *
         * @param event KickFSM::Update event
         * @param processEvent processes the GetBehindBallFSM::Update
         */
        const auto update_get_behind_ball =
            [](auto event, back::process<GetBehindBallFSM::Update> processEvent) {
                GetBehindBallFSM::ControlParams control_params{
                    .ball_location   = event.control_params.kick_origin,
                    .chick_direction = event.control_params.kick_direction};

                // Update the get behind ball fsm
                processEvent(GetBehindBallFSM::Update(control_params, event.common));
            };

        /**
         * Guard that checks if the ball has been chicked
         *
         * @param event KickFSM::Update event
         *
         * @return if the ball has been chicked
         */
        const auto ball_chicked = [](auto event) {
            return event.common.world.ball().hasBallBeenKicked(
                event.control_params.kick_direction);
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *get_behind_ball_s + update_e / update_get_behind_ball,
            get_behind_ball_s                              = kick_s,
            kick_s + update_e[!ball_chicked] / update_kick = kick_s,
            kick_s + update_e[ball_chicked]                = X);
    }
};

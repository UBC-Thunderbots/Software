#pragma once

#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/kick_intent.h"
#include "software/ai/intent/chip_intent.h"

struct OneTouchKickFSM
{
   public:
    class KickState;

    struct ControlParams
    {
        // The location where the kick will be taken from
        Point kick_origin;
        // The direction the Robot will kick in
        Angle kick_direction;
        // How the robot will chip or kick the ball
        AutoChipOrKick auto_chip_or_kick;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        const auto get_behind_ball_s = state<GetBehindBallFSM>;
        const auto kick_s            = state<KickState>;
        const auto update_e          = event<Update>;

        /**
         * Action that updates the KickIntent
         *
         * @param event OneTouchKickFSM::Update event
         */
        const auto update_kick = [](auto event) {
            if(event.control_params.auto_chip_or_kick.auto_chip_kick_mode==AutoChipOrKickMode::AUTOKICK)
            {
            event.common.set_intent(std::make_unique<KickIntent>(
                event.common.robot.id(), event.control_params.kick_origin,
                event.control_params.kick_direction,
                event.control_params.auto_chip_or_kick.autokick_speed_m_per_s,
                event.common.robot.robotConstants()));
            }
            else if(event.control_params.auto_chip_or_kick.auto_chip_kick_mode==AutoChipOrKickMode::AUTOCHIP)
            {
            event.common.set_intent(std::make_unique<ChipIntent>(
                event.common.robot.id(), event.control_params.kick_origin,
                event.control_params.kick_direction,
                event.control_params.auto_chip_or_kick.autochip_distance_m,
                event.common.robot.robotConstants()));
            }
        };

        /**
         * Action that updates the GetBehindBallFSM
         *
         * @param event OneTouchKickFSM::Update event
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
         * @param event OneTouchKickFSM::Update event
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

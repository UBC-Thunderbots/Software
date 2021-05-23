#pragma once

#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/logger/logger.h"

struct PivotKickFSM
{
    class KickState;
    class StartState;

    struct ControlParams
    {
        // The location where the kick will be taken from
        Point kick_origin;
        // The direction the Robot will kick in
        Angle kick_direction;
        // How the robot will chip or kick the ball
        AutoChipOrKick auto_chip_or_kick;
    };

    // this struct defines the only event that the PivotKickFSM responds to
    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        const auto get_possession_and_pivot_s = state<DribbleFSM>;
        const auto start_s                    = state<StartState>;
        const auto kick_ball_s                = state<KickState>;

        // update_e is the _event_ that the PivotKickFSM responds to
        const auto update_e = event<Update>;

        /**
         * Action that updates the DribbleFSM to get possession of the ball and pivot
         *
         * @param event PivotKickFSM::Update event
         * @param processEvent processes the GetBehindBallFSM::Update
         */
        const auto get_possession_and_pivot =
            [this](auto event, back::process<DribbleFSM::Update> processEvent) {
                DribbleFSM::ControlParams control_params{
                    .dribble_destination       = event.control_params.kick_origin,
                    .final_dribble_orientation = event.control_params.kick_direction,
                    .allow_excessive_dribbling = false};

                processEvent(DribbleFSM::Update(control_params, event.common));
            };

        /**
         * Action that kicks the ball
         *
         * @param event PivotKickFSM::Update event
         */
        const auto kick_ball = [this](auto event) {
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), event.control_params.kick_origin,
                event.control_params.kick_direction, 0, DribblerMode::OFF,
                BallCollisionType::ALLOW, event.control_params.auto_chip_or_kick,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
        };

        /**
         * Guard that checks if the ball has been kicked
         *
         * @param event PivotKickFSM::Update event
         *
         * @return if the ball has been kicked
         */
        const auto ball_kicked = [](auto event) {
            return event.common.world.ball().hasBallBeenKicked(
                event.control_params.kick_direction);
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *start_s + update_e / get_possession_and_pivot = get_possession_and_pivot_s,
            get_possession_and_pivot_s + update_e / get_possession_and_pivot,
            get_possession_and_pivot_s = kick_ball_s,
            kick_ball_s + update_e[!ball_kicked] / kick_ball,
            kick_ball_s + update_e[ball_kicked] = X);
    }
};

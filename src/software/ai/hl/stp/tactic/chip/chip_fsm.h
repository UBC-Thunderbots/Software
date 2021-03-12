#pragma once

#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/chip_intent.h"

struct ChipFSM
{
    class ChipState;

    struct ControlParams
    {
        // The location where the chip will be taken from
        Point chip_origin;
        // The direction the Robot will chip in
        Angle chip_direction;
        // The distance the robot will chip to
        double chip_distance_meters;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        const auto get_behind_ball_s = state<GetBehindBallFSM>;
        const auto chip_s            = state<ChipState>;
        const auto update_e          = event<Update>;

        /**
         * Action that updates the ChipIntent
         *
         * @param event ChipFSM::Update event
         */
        const auto update_chip = [](auto event) {
            event.common.set_intent(std::make_unique<ChipIntent>(
                event.common.robot.id(), event.control_params.chip_origin,
                event.control_params.chip_direction,
                event.control_params.chip_distance_meters));
        };

        /**
         * Action that updates the GetBehindBallFSM
         *
         * @param event ChipFSM::Update event
         * @param processEvent processes the GetBehindBallFSM::Update
         */
        const auto update_get_behind_ball =
            [](auto event, back::process<GetBehindBallFSM::Update> processEvent) {
                GetBehindBallFSM::ControlParams control_params{
                    .ball_location   = event.control_params.chip_origin,
                    .chick_direction = event.control_params.chip_direction};

                // Update the get behind ball fsm
                processEvent(GetBehindBallFSM::Update(control_params, event.common));
            };

        /**
         * Guard that checks if the ball has been chicked
         *
         * @param event ChipFSM::Update event
         *
         * @return if the ball has been chicked
         */
        const auto ball_chicked = [](auto event) {
            return event.common.world.ball().hasBallBeenKicked(
                event.control_params.chip_direction);
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *get_behind_ball_s + update_e / update_get_behind_ball,
            get_behind_ball_s                              = chip_s,
            chip_s + update_e[!ball_chicked] / update_chip = chip_s,
            chip_s + update_e[ball_chicked]                = X);
    }
};

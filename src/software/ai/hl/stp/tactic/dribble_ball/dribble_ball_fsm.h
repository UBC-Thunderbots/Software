#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/pass.h"
#include "software/ai/hl/stp/tactic/get_possession/get_possession_fsm.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"

struct DribbleBallFSM
{
    class DribbleBallState;

    struct ControlParams
    {
        // Destination to dribble the ball to
        Point dribble_destination;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        const auto dribble_ball_s   = state<DribbleBallState>;
        const auto get_possession_s = state<GetPossessionFSM>;

        const auto update_e = event<Update>;

        /**
         * Guard that checks if the ball has been have_possession
         *
         * @param event DribbleBallFSM::Update
         *
         * @return if the ball has been have_possession
         */
        const auto have_possession = [](auto event) {
            return event.common.robot.isNearDribbler(
                event.common.world.ball().position());
        };

        /**
         * Guard that checks if the ball is at the dribble_destination
         *
         * @param event DribbleBallFSM::Update
         *
         * @return if the ball is at the dribble_destination
         */
        const auto ball_at_destination = [](auto event) {
            return ((event.common.world.ball().position() -
                     event.control_params.dribble_destination)
                        .length() < 0.05);
        };

        /**
         * Action to update the MoveIntent to hold onto the ball
         *
         * @param event DribbleBallFSM::Update
         */
        const auto hold_ball = [this](auto event) {
            auto ball_position    = event.common.world.ball().position();
            auto face_ball_vector = (ball_position - event.common.robot.position());
            auto point_in_front_of_ball =
                ball_position - face_ball_vector.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                           BALL_MAX_RADIUS_METERS);
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), point_in_front_of_ball,
                face_ball_vector.orientation(), 0, DribblerMode::MAX_FORCE,
                BallCollisionType::ALLOW, AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                MaxAllowedSpeedMode::PHYSICAL_LIMIT));
        };

        /**
         * Action that updates the GetPossessionFSM
         *
         * @param event DribbleBallFSM update event
         * @param processEvent processes the GetPossessionFSM::Update
         */
        const auto update_get_possession =
            [](auto event, back::process<GetPossessionFSM::Update> processEvent) {
                // Update the get possession fsm
                processEvent(GetPossessionFSM::Update({}, event.common));
            };

        /**
         * Action to update to get possession of the ball
         *
         * @param event DribbleBallFSM::Update
         */
        const auto dribble_ball = [this](auto event) {
            auto destination = event.control_params.dribble_destination;
            auto face_ball_orientation =
                (destination - event.common.world.ball().position()).orientation();
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), destination, face_ball_orientation, 0,
                DribblerMode::MAX_FORCE, BallCollisionType::ALLOW,
                AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                MaxAllowedSpeedMode::PHYSICAL_LIMIT));
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *get_possession_s + update_e / update_get_possession,
            get_possession_s = dribble_ball_s,
            dribble_ball_s + update_e[ball_at_destination] / dribble_ball = X,
            dribble_ball_s + update_e[!ball_at_destination] / dribble_ball,
            dribble_ball_s + update_e[have_possession] / dribble_ball,
            dribble_ball_s + update_e[!have_possession] / update_get_possession =
                get_possession_s,
            X + update_e[!ball_at_destination] / update_get_possession = get_possession_s,
            X + update_e[ball_at_destination] / hold_ball);
    }
};

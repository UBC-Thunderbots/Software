#pragma once
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/ray.h"
#include "software/logger/logger.h"

// Which way the crease defender should be aligned
struct MoveGoalieToGoalLineFSM
{
    // this struct defines the unique control parameters that the MoveGoalieToGoalLineFSM
    // requires in its update
    struct ControlParams
    {
    };
    // this struct defines the only event that the MoveGoalieToGoalLineFSM responds to
    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS
    auto operator()()
    {
        using namespace boost::sml;
        const auto move_to_goal_line_s = state<MoveFSM>;
        // update_e is the event that the MoveGoalieToGoalLineFSM responds to
        const auto update_e = event<Update>;
        /**
         * This is an Action that blocks the threat
         *
         * @param event MoveGoalieToGoalLineFSM::Update event
         */
        const auto move_to_goal_line =
            [this](auto event, back::process<MoveFSM::Update> processEvent) {
                Field field       = event.common.world.field();
                Point destination = field.friendlyGoalCenter();
                Angle face_center = Angle::zero();
                MoveFSM::ControlParams control_params{
                    .destination         = destination,
                    .final_orientation   = face_center,
                    .final_speed         = 0.0,
                    .dribbler_mode       = DribblerMode::OFF,
                    .ball_collision_type = BallCollisionType::AVOID,
                    .auto_chip_or_kick   = AutoChipOrKick{AutoChipOrKickMode::OFF, 0.0},
                    .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                    .target_spin_rev_per_s  = 0.0};
                // Update the get behind ball fsm
                processEvent(MoveFSM::Update(control_params, event.common));
            };
        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *move_to_goal_line_s + update_e / move_to_goal_line, move_to_goal_line_s = X,
            X + update_e / move_to_goal_line = move_to_goal_line_s);
    }
};

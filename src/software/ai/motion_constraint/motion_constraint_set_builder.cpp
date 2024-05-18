#include "software/ai/motion_constraint/motion_constraint_set_builder.h"

#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/ai/motion_constraint/motion_constraint_visitor.h"

std::set<TbotsProto::MotionConstraint> buildMotionConstraintSet(
    const GameState &game_state, const Tactic &tactic)
{
    MotionConstraintVisitor motion_constraint_visitor;

    std::set<TbotsProto::MotionConstraint> current_motion_constraints =
        buildMotionConstraintSetFromGameState(game_state);

    // updates current_allowed_constraints
    current_motion_constraints = motion_constraint_visitor.getUpdatedMotionConstraints(
        tactic, current_motion_constraints);

    current_motion_constraints.insert(TbotsProto::MotionConstraint::FRIENDLY_GOAL);

    return current_motion_constraints;
}


std::set<TbotsProto::MotionConstraint> buildMotionConstraintSetFromGameState(
    const GameState &game_state)
{
    std::set<TbotsProto::MotionConstraint> motion_constraints;
    // Robots are allowed to enter the defense area during ball placement
    if (!game_state.isTheirBallPlacement())
    {
        motion_constraints.insert(TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA);
    }

    if (game_state.stayAwayFromBall())
    {
        motion_constraints.insert(TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL);
    }

    if (game_state.isPenalty())
    {
        if (game_state.isOurPenalty())
        {
            motion_constraints.insert(TbotsProto::MotionConstraint::ENEMY_HALF);
        }
        else
        {
            // Is their penalty
            // TODO (#2611): implement penalty kick motion constraint
            motion_constraints.insert(TbotsProto::MotionConstraint::FRIENDLY_HALF);
        }
    }
    else if (game_state.isKickoff())
    {
        motion_constraints.insert({TbotsProto::MotionConstraint::CENTER_CIRCLE,
                                   TbotsProto::MotionConstraint::ENEMY_HALF});
        if (game_state.isSetupState())
        {
            motion_constraints.insert(
                TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL);
        }
    }
    else if (game_state.isTheirBallPlacement())
    {
        motion_constraints.insert(
            TbotsProto::MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE);
    }
    else
    {
        motion_constraints.insert(
            TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA);
    }

    return motion_constraints;
}

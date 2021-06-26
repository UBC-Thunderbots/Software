#include "software/ai/motion_constraint/motion_constraint_set_builder.h"

#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/ai/motion_constraint/motion_constraint_visitor.h"

std::set<MotionConstraint> buildMotionConstraintSet(const GameState &game_state,
                                                    const Tactic &tactic)
{
    std::set<MotionConstraint> current_allowed_constraints;
    MotionConstraintVisitor motion_constraint_visitor;

    std::set<MotionConstraint> current_motion_constraints =
        buildMotionConstraintSetFromGameState(game_state);
    try
    {
        // updates current_allowed_constraints
        current_allowed_constraints =
            motion_constraint_visitor.getCurrentAllowedConstraints(tactic);
    }
    catch (std::invalid_argument &)
    {
        // Tactic didn't implement accept so don't add any more motion constraints
    }

    for (const auto &constraint : current_allowed_constraints)
    {
        current_motion_constraints.erase(constraint);
    }

    return current_motion_constraints;
}


std::set<MotionConstraint> buildMotionConstraintSetFromGameState(
    const GameState &game_state)
{
    std::set<MotionConstraint> motion_constraints;
    motion_constraints.insert(MotionConstraint::FRIENDLY_DEFENSE_AREA);

    if (game_state.stayAwayFromBall())
    {
        motion_constraints.insert(MotionConstraint::HALF_METER_AROUND_BALL);
    }

    if (game_state.isPenalty())
    {
        if (game_state.isOurPenalty())
        {
            motion_constraints.insert(MotionConstraint::ENEMY_HALF);
        }
        else
        {
        }
    }
    else if (game_state.isKickoff() && !game_state.isPlaying())
    {
        motion_constraints.insert({MotionConstraint::HALF_METER_AROUND_BALL,
                                   MotionConstraint::CENTER_CIRCLE,
                                   MotionConstraint::ENEMY_HALF,
                                   MotionConstraint::ENEMY_HALF_EXCEPT_CENTRE_CIRCLE});
    }
    else if (game_state.isOurBallPlacement())
    {
        motion_constraints.erase(MotionConstraint::FRIENDLY_DEFENSE_AREA);
    }
    else if (game_state.isTheirBallPlacement())
    {
        motion_constraints.insert({MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE,
                                   MotionConstraint::HALF_METER_AROUND_BALL,
                                   MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA});
    }
    else
    {
        motion_constraints.insert(MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA);
    }

    return motion_constraints;
}

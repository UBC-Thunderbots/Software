#include "software/ai/motion_constraint/motion_constraint_manager.h"

#include "software/ai/hl/stp/tactic/all_tactics.h"

std::set<MotionConstraint> MotionConstraintManager::getMotionConstraints(
    const GameState &game_state, Tactic &tactic)
{
    current_whitelisted_constraints.clear();
    std::set<MotionConstraint> current_motion_constraints =
        getMotionConstraintsFromGameState(game_state);
    try
    {
        // updates current_whitelisted_constraints
        tactic.accept(*this);
    }
    catch (std::invalid_argument)
    {
        // Tactic didn't implement accept so don't add any more motion constraints
    }

    for (const auto &constraint : current_whitelisted_constraints)
    {
        current_motion_constraints.erase(constraint);
    }

    return current_motion_constraints;
}

// We disable clang-format here because it makes these lists borderline unreadable,
// and certainly way more difficult to edit
// clang-format off
void MotionConstraintManager::visit(CherryPickTactic &tactic) {}

void MotionConstraintManager::visit(ShadowFreekickerTactic &tactic) {}

void MotionConstraintManager::visit(GoalieTactic &tactic)
{
    current_whitelisted_constraints = std::set<MotionConstraint>({
       MotionConstraint::FRIENDLY_DEFENSE_AREA,
       MotionConstraint::FRIENDLY_DEFENSE_AREA,
       MotionConstraint::HALF_METER_AROUND_BALL,
       MotionConstraint::FRIENDLY_HALF
       });
}

void MotionConstraintManager::visit(CreaseDefenderTactic &tactic) {}

void MotionConstraintManager::visit(ShadowEnemyTactic &tactic) {}

void MotionConstraintManager::visit(MoveTactic &tactic) {}

void MotionConstraintManager::visit(ChipTactic &tactic) {}

void MotionConstraintManager::visit(KickoffChipTactic &tactic)
{
    current_whitelisted_constraints = std::set<MotionConstraint>({
       MotionConstraint::CENTER_CIRCLE,
       MotionConstraint::HALF_METER_AROUND_BALL
       });
}

void MotionConstraintManager::visit(StopTactic &tactic) {}

void MotionConstraintManager::visit(PatrolTactic &tactic) {}

void MotionConstraintManager::visit(PenaltyKickTactic &tactic)
{
    current_whitelisted_constraints = std::set<MotionConstraint>({
        MotionConstraint::HALF_METER_AROUND_BALL,
        MotionConstraint::ENEMY_DEFENSE_AREA,
        MotionConstraint::ENEMY_HALF
        });
}

void MotionConstraintManager::visit(PenaltySetupTactic &tactic)
{
    current_whitelisted_constraints = std::set<MotionConstraint>({
        MotionConstraint::ENEMY_HALF,
        MotionConstraint::ENEMY_DEFENSE_AREA,
        MotionConstraint::FRIENDLY_HALF,
        MotionConstraint::HALF_METER_AROUND_BALL
        });
}

void MotionConstraintManager::visit(ReceiverTactic &tactic) {}

void MotionConstraintManager::visit(ShootGoalTactic &tactic)
{
    current_whitelisted_constraints = std::set<MotionConstraint>({
        MotionConstraint::HALF_METER_AROUND_BALL
        });
}

void MotionConstraintManager::visit(PasserTactic &tactic) {}

void MotionConstraintManager::visit(DefenseShadowEnemyTactic &tactic) {}

void MotionConstraintManager::visit(MoveTestTactic &tactic) {}

void MotionConstraintManager::visit(StopTestTactic &tactic) {}

void MotionConstraintManager::visit(GoalieTestTactic &tactic) {}

// clang-format on

std::set<MotionConstraint> MotionConstraintManager::getMotionConstraintsFromGameState(
    const GameState &game_state) const
{
    std::set<MotionConstraint> motion_constraints;
    motion_constraints.insert(MotionConstraint::FRIENDLY_DEFENSE_AREA);
    motion_constraints.insert(MotionConstraint::ENEMY_ROBOTS_COLLISION);

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
            // Is their penalty
            motion_constraints.insert(MotionConstraint::FRIENDLY_HALF);
        }
    }
    else if (game_state.isKickoff())
    {
        motion_constraints.insert({MotionConstraint::HALF_METER_AROUND_BALL,
                                   MotionConstraint::CENTER_CIRCLE,
                                   MotionConstraint::ENEMY_HALF});
    }
    else
    {
        if (game_state.stayAwayFromBall() || game_state.isOurKickoff())
        {
            motion_constraints.insert(MotionConstraint::HALF_METER_AROUND_BALL);
        }

        if (game_state.isOurPenalty())
        {
            motion_constraints.insert(MotionConstraint::ENEMY_DEFENSE_AREA);
        }
        else
        {
            motion_constraints.insert(MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA);
        }
    }

    return motion_constraints;
}

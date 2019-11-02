#include "software/ai/motion_constraint/motion_constraint_manager.h"

#include "software/ai/hl/stp/tactic/all_tactics.h"

std::set<MotionConstraint> MotionConstraintManager::getMotionConstraints(
    const GameState &game_state, const Tactic &tactic)
{
    std::set<MotionConstraint> current_motion_constraints =
        getMotionConstraintsFromGameState(game_state);
    current_whitelisted_constraints.clear();
    tactic.accept(*this);  // updates current_whitelisted_constraints
    for (const auto &constraint : current_whitelisted_constraints)
    {
        current_motion_constraints.erase(constraint);
    }
    return current_motion_constraints;
}

void MotionConstraintManager::visit(const CherryPickTactic &tactic) {}

void MotionConstraintManager::visit(const ShadowFreekickerTactic &tactic) {}

void MotionConstraintManager::visit(const GoalieTactic &tactic)
{
    current_whitelisted_constraints.insert(MotionConstraint::FRIENDLY_DEFENSE_AREA);
}

void MotionConstraintManager::visit(const CreaseDefenderTactic &tactic) {}

void MotionConstraintManager::visit(const ShadowEnemyTactic &tactic) {}

void MotionConstraintManager::visit(const BlockShotPathTactic &tactic) {}

void MotionConstraintManager::visit(const MoveTactic &tactic) {}

void MotionConstraintManager::visit(const ChipTactic &tactic) {}

void MotionConstraintManager::visit(const KickoffChipTactic &tactic) {}

void MotionConstraintManager::visit(const StopTactic &tactic) {}

void MotionConstraintManager::visit(const PenaltyKickTactic &tactic) {}

void MotionConstraintManager::visit(const PenaltySetupTactic &tactic) {}

void MotionConstraintManager::visit(const ReceiverTactic &tactic) {}

void MotionConstraintManager::visit(const PatrolTactic &tactic) {}

void MotionConstraintManager::visit(const ShootGoalTactic &tactic) {}

void MotionConstraintManager::visit(const PasserTactic &tactic) {}

void MotionConstraintManager::visit(const DefenseShadowEnemyTactic &tactic) {}

void MotionConstraintManager::visit(const GrabBallTactic &tactic) {}

void MotionConstraintManager::visit(const MoveTestTactic &tactic) {}

void MotionConstraintManager::visit(const StopTestTactic &tactic) {}

std::set<MotionConstraint> MotionConstraintManager::getMotionConstraintsFromGameState(
    const GameState &game_state)
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
        motion_constraints.insert(MotionConstraint::HALF_METER_AROUND_BALL);
        motion_constraints.insert(MotionConstraint::CENTER_CIRCLE);
        motion_constraints.insert(MotionConstraint::ENEMY_HALF);
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

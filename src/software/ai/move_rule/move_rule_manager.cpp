#include "software/ai/move_rule/move_rule_manager.h"

#include "software/ai/hl/stp/tactic/all_tactics.h"


std::set<MoveRule> MoveRuleManager::getMoveRules(const GameState &game_state, const Tactic &tactic)
{
    setCurrentMoveRulesFromGameState(game_state);
    tactic.accept(*this);
    return current_move_rules;
}

void MoveRuleManager::visit(const CherryPickTactic &tactic)
{
}

void MoveRuleManager::visit(const ShadowFreekickerTactic &tactic)
{
}

void MoveRuleManager::visit(const GoalieTactic &tactic)
{
}

void MoveRuleManager::visit(const CreaseDefenderTactic &tactic)
{
}

void MoveRuleManager::visit(const ShadowEnemyTactic &tactic)
{
}

void MoveRuleManager::visit(const BlockShotPathTactic &tactic)
{
}

void MoveRuleManager::visit(const MoveTactic &tactic)
{
}

void MoveRuleManager::visit(const ChipTactic &tactic)
{
}

void MoveRuleManager::visit(const StopTactic &tactic)
{
}

void MoveRuleManager::visit(const PenaltyKickTactic &tactic)
{
}

void MoveRuleManager::visit(const ReceiverTactic &tactic)
{
}

void MoveRuleManager::visit(const PatrolTactic &tactic)
{
}

void MoveRuleManager::visit(const ShootGoalTactic &tactic)
{
}

void MoveRuleManager::visit(const PasserTactic &tactic)
{
}

void MoveRuleManager::setCurrentMoveRulesFromGameState(const GameState &game_state)
{
    current_move_rules.clear();

    current_move_rules.insert(MoveRule::FRIENDLY_DEFENSE_AREA);
    current_move_rules.insert(MoveRule::ENEMY_ROBOTS_COLLISION);

    if (game_state.stayAwayFromBall())
    {
        current_move_rules.insert(MoveRule::HALF_METER_AROUND_BALL);
    }

    if (game_state.isPenalty())
    {
        if (game_state.isOurPenalty())
        {
            current_move_rules.insert(MoveRule::ENEMY_HALF);
        }
        else
        {
            // Is their penalty
            current_move_rules.insert(MoveRule::FRIENDLY_HALF);
        }
    }
    else if (game_state.isKickoff())
    {
        current_move_rules.insert(MoveRule::HALF_METER_AROUND_BALL);
        current_move_rules.insert(MoveRule::CENTER_CIRCLE);
        current_move_rules.insert(MoveRule::ENEMY_HALF);
    }
    else
    {
        if (game_state.stayAwayFromBall() || game_state.isOurKickoff())
        {
            current_move_rules.insert(MoveRule::HALF_METER_AROUND_BALL);
        }

        if (game_state.isOurPenalty())
        {
            current_move_rules.insert(MoveRule::ENEMY_DEFENSE_AREA);
        }
        else
        {
            current_move_rules.insert(MoveRule::INFLATED_ENEMY_DEFENSE_AREA);
        }
    }
}

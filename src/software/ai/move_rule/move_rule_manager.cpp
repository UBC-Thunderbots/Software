#include "software/ai/move_rule/move_rule_manager.h"

#include "software/ai/hl/stp/tactic/all_tactics.h"

std::set<MoveRule> MoveRuleManager::getMoveRules(const GameState &game_state,
                                                 const Tactic &tactic)
{
    std::set<MoveRule> current_move_rules = getMoveRulesFromGameState(game_state);
    current_whitelisted_rules.clear();
    tactic.accept(*this); // updates current_whitelisted_rules
    for (const auto& rule : current_whitelisted_rules){
        current_move_rules.erase(rule);
    }
    return current_move_rules;
}

void MoveRuleManager::visit(const CherryPickTactic &tactic) {}

void MoveRuleManager::visit(const ShadowFreekickerTactic &tactic) {}

void MoveRuleManager::visit(const GoalieTactic &tactic)
{
    current_whitelisted_rules.insert(MoveRule::FRIENDLY_DEFENSE_AREA);
}

void MoveRuleManager::visit(const CreaseDefenderTactic &tactic) {}

void MoveRuleManager::visit(const ShadowEnemyTactic &tactic) {}

void MoveRuleManager::visit(const BlockShotPathTactic &tactic) {}

void MoveRuleManager::visit(const MoveTactic &tactic) {}

void MoveRuleManager::visit(const ChipTactic &tactic) {}

void MoveRuleManager::visit(const StopTactic &tactic) {}

void MoveRuleManager::visit(const PenaltyKickTactic &tactic) {}

void MoveRuleManager::visit(const ReceiverTactic &tactic) {}

void MoveRuleManager::visit(const PatrolTactic &tactic) {}

void MoveRuleManager::visit(const ShootGoalTactic &tactic) {}

void MoveRuleManager::visit(const PasserTactic &tactic) {}

void MoveRuleManager::visit(const MoveTestTactic &tactic) {}

void MoveRuleManager::visit(const StopTestTactic &tactic) {}

std::set<MoveRule> MoveRuleManager::getMoveRulesFromGameState(const GameState &game_state)
{
    std::set<MoveRule> move_rules;
    move_rules.insert(MoveRule::FRIENDLY_DEFENSE_AREA);
    move_rules.insert(MoveRule::ENEMY_ROBOTS_COLLISION);

    if (game_state.stayAwayFromBall())
    {
        move_rules.insert(MoveRule::HALF_METER_AROUND_BALL);
    }

    if (game_state.isPenalty())
    {
        if (game_state.isOurPenalty())
        {
            move_rules.insert(MoveRule::ENEMY_HALF);
        }
        else
        {
            // Is their penalty
            move_rules.insert(MoveRule::FRIENDLY_HALF);
        }
    }
    else if (game_state.isKickoff())
    {
        move_rules.insert(MoveRule::HALF_METER_AROUND_BALL);
        move_rules.insert(MoveRule::CENTER_CIRCLE);
        move_rules.insert(MoveRule::ENEMY_HALF);
    }
    else
    {
        if (game_state.stayAwayFromBall() || game_state.isOurKickoff())
        {
            move_rules.insert(MoveRule::HALF_METER_AROUND_BALL);
        }

        if (game_state.isOurPenalty())
        {
            move_rules.insert(MoveRule::ENEMY_DEFENSE_AREA);
        }
        else
        {
            move_rules.insert(MoveRule::INFLATED_ENEMY_DEFENSE_AREA);
        }
    }

    return move_rules;
}

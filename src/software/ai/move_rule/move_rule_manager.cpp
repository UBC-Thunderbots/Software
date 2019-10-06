#include "software/ai/move_rule/move_rule_manager.h"

#include "software/ai/hl/stp/tactic/all_tactics.h"

std::set<MoveRule> MoveRuleManager::getMoveRules(const GameState &game_state,
                                                 const Tactic &tactic)
{
    std::set<MoveRule> current_move_rules;

    // function for adding move rules
    std::function<void(MoveRule)> add_move_rule = [&](MoveRule rule) {
        current_move_rules.erase(rule);
    };
    addCurrentMoveRulesFromGameState(game_state, add_move_rule);

    // only expose removing rules to accept
    remove_move_rule = [&](MoveRule rule) { current_move_rules.erase(rule); };

    tactic.accept(*this);
    return current_move_rules;
}

void MoveRuleManager::visit(const CherryPickTactic &tactic) {}

void MoveRuleManager::visit(const ShadowFreekickerTactic &tactic) {}

void MoveRuleManager::visit(const GoalieTactic &tactic)
{
    remove_move_rule(MoveRule::FRIENDLY_DEFENSE_AREA);
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

void MoveRuleManager::addCurrentMoveRulesFromGameState(
    const GameState &game_state, std::function<void(MoveRule)> add_move_rule)
{
    add_move_rule(MoveRule::FRIENDLY_DEFENSE_AREA);
    add_move_rule(MoveRule::ENEMY_ROBOTS_COLLISION);

    if (game_state.stayAwayFromBall())
    {
        add_move_rule(MoveRule::HALF_METER_AROUND_BALL);
    }

    if (game_state.isPenalty())
    {
        if (game_state.isOurPenalty())
        {
            add_move_rule(MoveRule::ENEMY_HALF);
        }
        else
        {
            // Is their penalty
            add_move_rule(MoveRule::FRIENDLY_HALF);
        }
    }
    else if (game_state.isKickoff())
    {
        add_move_rule(MoveRule::HALF_METER_AROUND_BALL);
        add_move_rule(MoveRule::CENTER_CIRCLE);
        add_move_rule(MoveRule::ENEMY_HALF);
    }
    else
    {
        if (game_state.stayAwayFromBall() || game_state.isOurKickoff())
        {
            add_move_rule(MoveRule::HALF_METER_AROUND_BALL);
        }

        if (game_state.isOurPenalty())
        {
            add_move_rule(MoveRule::ENEMY_DEFENSE_AREA);
        }
        else
        {
            add_move_rule(MoveRule::INFLATED_ENEMY_DEFENSE_AREA);
        }
    }
}

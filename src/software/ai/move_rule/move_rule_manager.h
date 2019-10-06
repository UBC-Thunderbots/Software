#pragma once

#include <set>

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/ai/move_rule/move_rule.h"
#include "software/ai/world/game_state.h"

class MoveRuleManager : public TacticVisitor
{
   public:
    explicit MoveRuleManager(){};

    /**
     * Gets Move Rules based on gamestate and tactic
     *
     * @param gamestate Current GameState to process
     * @param tactic Current Tactic to process
     *
     * @return set of MoveRules
     */
    std::set<MoveRule> getMoveRules(const GameState &game_state, const Tactic &tactic);

    /**
     * Visits a CherryPickTactic to perform an operation.
     *
     * @param tactic The CherryPickTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const CherryPickTactic &tactic) override;

    /**
     * Visits a ShadowFreekickerTactic to perform an operation.
     *
     * @param tactic The ShadowFreekickerTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const ShadowFreekickerTactic &tactic) override;

    /**
     * Visits a GoalieTactic to perform an operation.
     *
     * @param tactic The GoalieTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const GoalieTactic &tactic) override;

    /**
     * Visits a CreaseDefenderTactic to perform an operation.
     *
     * @param tactic The CreaseDefenderTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const CreaseDefenderTactic &tactic) override;

    /**
     * Visits a ShadowEnemyTactic to perform an operation.
     *
     * @param tactic The ShadowEnemyTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const ShadowEnemyTactic &tactic) override;

    /**
     * Visits a BlockShotPathTactic to perform an operation.
     *
     * @param tactic The BlockShotPathTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const BlockShotPathTactic &tactic) override;

    /**
     * Visits a MoveTactic to perform an operation.
     *
     * @param tactic The MoveTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const MoveTactic &tactic) override;

    /**
     * Visits a ChipTactic to perform an operation.
     *
     * @param tactic The ChipTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const ChipTactic &tactic) override;

    /**
     * Visits a StopTactic to perform an operation.
     *
     * @param tactic The StopTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const StopTactic &tactic) override;

    /**
     * Visits a PenaltyKickTactic to perform an operation.
     *
     * @param tactic The PenaltyKickTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const PenaltyKickTactic &tactic) override;

    /**
     * Visits a ReceiverTactic to perform an operation.
     *
     * @param tactic The ReceiverTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const ReceiverTactic &tactic) override;

    /**
     * Visits a PatrolTactic to perform an operation.
     *
     * @param tactic The PatrolTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const PatrolTactic &tactic) override;

    /**
     * Visits a ShootGoalTactic to perform an operation.
     *
     * @param tactic The ShootGoalTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const ShootGoalTactic &tactic) override;

    /**
     * Visits a PasserTactic to perform an operation.
     *
     * @param tactic The PasserTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const PasserTactic &tactic) override;

    /**
     * Visits a MoveTestTactic to perform an operation.
     *
     * @param tactic The MoveTestTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const MoveTestTactic &tactic) override;

    /**
     * Visits a StopTestTactic to perform an operation.
     *
     * @param tactic The StopTestTactic to visit
     *
     * @modifies current_whitelisted_rules
     */
    void visit(const StopTestTactic &tactic) override;

   private:
    std::set<MoveRule> current_whitelisted_rules;

    /**
     * Adds move rules determined from gamestate to current_move_rules
     *
     * @param game_state GameState to generate move rules from
     */
    std::set<MoveRule> getMoveRulesFromGameState(const GameState &game_state);
};

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
     */
    void visit(const CherryPickTactic &tactic) override;

    /**
     * Visits a ShadowFreekickerTactic to perform an operation.
     *
     * @param tactic The ShadowFreekickerTactic to visit
     */
    void visit(const ShadowFreekickerTactic &tactic) override;

    /**
     * Visits a GoalieTactic to perform an operation.
     *
     * @param tactic The GoalieTactic to visit
     */
    void visit(const GoalieTactic &tactic) override;

    /**
     * Visits a CreaseDefenderTactic to perform an operation.
     *
     * @param tactic The CreaseDefenderTactic to visit
     */
    void visit(const CreaseDefenderTactic &tactic) override;

    /**
     * Visits a ShadowEnemyTactic to perform an operation.
     *
     * @param tactic The ShadowEnemyTactic to visit
     */
    void visit(const ShadowEnemyTactic &tactic) override;

    /**
     * Visits a BlockShotPathTactic to perform an operation.
     *
     * @param tactic The BlockShotPathTactic to visit
     */
    void visit(const BlockShotPathTactic &tactic) override;

    /**
     * Visits a MoveTactic to perform an operation.
     *
     * @param tactic The MoveTactic to visit
     */
    void visit(const MoveTactic &tactic) override;

    /**
     * Visits a ChipTactic to perform an operation.
     *
     * @param tactic The ChipTactic to visit
     */
    void visit(const ChipTactic &tactic) override;

    /**
     * Visits a StopTactic to perform an operation.
     *
     * @param tactic The StopTactic to visit
     */
    void visit(const StopTactic &tactic) override;

    /**
     * Visits a PenaltyKickTactic to perform an operation.
     *
     * @param tactic The PenaltyKickTactic to visit
     */
    void visit(const PenaltyKickTactic &tactic) override;

    /**
     * Visits a ReceiverTactic to perform an operation.
     *
     * @param tactic The ReceiverTactic to visit
     */
    void visit(const ReceiverTactic &tactic) override;

    /**
     * Visits a PatrolTactic to perform an operation.
     *
     * @param tactic The PatrolTactic to visit
     */
    void visit(const PatrolTactic &tactic) override;

    /**
     * Visits a ShootGoalTactic to perform an operation.
     *
     * @param tactic The ShootGoalTactic to visit
     */
    void visit(const ShootGoalTactic &tactic) override;

    /**
     * Visits a PasserTactic to perform an operation.
     *
     * @param tactic The PasserTactic to visit
     */
    void visit(const PasserTactic &tactic) override;

   private:
    std::set<MoveRule> current_move_rules;

    void setCurrentMoveRulesFromGameState(const GameState &game_state);
};

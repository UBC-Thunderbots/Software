#pragma once

#include <set>

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/world/game_state.h"

class MotionConstraintManager : public TacticVisitor
{
   public:
    explicit MotionConstraintManager() = default;

    /**
     * Gets Motion Constraint based on gamestate and tactic
     *
     * @param gamestate Current GameState to process
     * @param tactic Current Tactic to process
     *
     * @return set of MotionConstraints
     */
    std::set<MotionConstraint> getMotionConstraints(const GameState &game_state,
                                                    Tactic &tactic);

    /**
     * Visits a tactic to register the associated motion constraint
     *
     * @param The tactic to register
     *
     * @modifies current_allowed_constraints
     */
    void visit(const CherryPickTactic &tactic) override;
    void visit(const ShadowFreekickerTactic &tactic) override;
    void visit(const GoalieTactic &tactic) override;
    void visit(const CreaseDefenderTactic &tactic) override;
    void visit(const ShadowEnemyTactic &tactic) override;
    void visit(const MoveTactic &tactic) override;
    void visit(const ChipTactic &tactic) override;
    void visit(const KickoffChipTactic &tactic) override;
    void visit(const StopTactic &tactic) override;
    void visit(const PatrolTactic &tactic) override;
    void visit(const PenaltyKickTactic &tactic) override;
    void visit(const PenaltySetupTactic &tactic) override;
    void visit(const ReceiverTactic &tactic) override;
    void visit(const ShootGoalTactic &tactic) override;
    void visit(const PasserTactic &tactic) override;
    void visit(const DefenseShadowEnemyTactic &tactic) override;
    void visit(const MoveTestTactic &tactic) override;
    void visit(const StopTestTactic &tactic) override;
    void visit(const GoalieTestTactic &tactic) override;

   private:
    std::set<MotionConstraint> current_allowed_constraints;

    /**
     * Adds move constraints determined from gamestate to current_motion_constraints
     *
     * @param game_state GameState to generate move constraints from
     */
    std::set<MotionConstraint> getMotionConstraintsFromGameState(
        const GameState &game_state) const;
};

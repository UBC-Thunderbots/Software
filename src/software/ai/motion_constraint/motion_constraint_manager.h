#pragma once

#include <set>

#include "software/ai/hl/stp/tactic/mutable_tactic_visitor.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/world/game_state.h"

class MotionConstraintManager : public MutableTacticVisitor
{
   public:
    explicit MotionConstraintManager(){};

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
     * @modifies current_whitelisted_constraints
     */
    void visit(CherryPickTactic &tactic) override;
    void visit(ShadowFreekickerTactic &tactic) override;
    void visit(GoalieTactic &tactic) override;
    void visit(CreaseDefenderTactic &tactic) override;
    void visit(ShadowEnemyTactic &tactic) override;
    void visit(MoveTactic &tactic) override;
    void visit(ChipTactic &tactic) override;
    void visit(KickoffChipTactic &tactic) override;
    void visit(StopTactic &tactic) override;
    void visit(PatrolTactic &tactic) override;
    void visit(PenaltyKickTactic &tactic) override;
    void visit(PenaltySetupTactic &tactic) override;
    void visit(ReceiverTactic &tactic) override;
    void visit(ShootGoalTactic &tactic) override;
    void visit(PasserTactic &tactic) override;
    void visit(DefenseShadowEnemyTactic &tactic) override;
    void visit(MoveTestTactic &tactic) override;
    void visit(StopTestTactic &tactic) override;
    void visit(GoalieTestTactic &tactic) override;

   private:
    std::set<MotionConstraint> current_whitelisted_constraints;

    /**
     * Adds move constraints determined from gamestate to current_motion_constraints
     *
     * @param game_state GameState to generate move constraints from
     */
    std::set<MotionConstraint> getMotionConstraintsFromGameState(
        const GameState &game_state) const;
};

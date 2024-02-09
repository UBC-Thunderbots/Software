#pragma once

#include <set>

#include "proto/primitive.pb.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/world/game_state.h"

class MotionConstraintVisitor : public TacticVisitor
{
   public:
    explicit MotionConstraintVisitor() = default;

    /**
     * Visits a tactic to register the associated motion constraint
     *
     * @param The tactic to register
     *
     * @modifies current_motion_constraints
     */
    void visit(const GoalieTactic &tactic) override;
    void visit(const CreaseDefenderTactic &tactic) override;
    void visit(const ShadowEnemyTactic &tactic) override;
    void visit(const MoveTactic &tactic) override;
    void visit(const ChipTactic &tactic) override;
    void visit(const KickTactic &tactic) override;
    void visit(const KickoffChipTactic &tactic) override;
    void visit(const StopTactic &tactic) override;
    void visit(const PenaltyKickTactic &tactic) override;
    void visit(const PenaltySetupTactic &tactic) override;
    void visit(const ReceiverTactic &tactic) override;
    void visit(const AttackerTactic &tactic) override;
    void visit(const DefenseShadowEnemyTactic &tactic) override;
    void visit(const MoveTestTactic &tactic) override;
    void visit(const StopTestTactic &tactic) override;
    void visit(const GoalieTestTactic &tactic) override;
    void visit(const GetBehindBallTactic &tactic) override;
    void visit(const MoveGoalieToGoalLineTactic &tactic) override;
    void visit(const PrepareKickoffMoveTactic &tactic) override;
    void visit(const PassDefenderTactic &tactic) override;
    void visit(const AssignedSkillTactic &tactic) override;

    /**
     * Gets the motion constraints updated with the requirements of the tactics
     *
     * @param The tactic to use to update the motion constraints
     * @param The existing motion constraints from other sources
     *
     * @modifies current_motion_constraints
     * @return set of MotionConstraints
     */
    std::set<TbotsProto::MotionConstraint> getUpdatedMotionConstraints(
        const Tactic &tactic,
        const std::set<TbotsProto::MotionConstraint> &existing_motion_constraints);

   private:
    std::set<TbotsProto::MotionConstraint> current_motion_constraints;
};

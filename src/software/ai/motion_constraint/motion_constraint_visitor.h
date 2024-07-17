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
    void visit(const StopTactic &tactic) override;
    void visit(const PenaltyKickTactic &tactic) override;
    void visit(const PenaltySetupTactic &tactic) override;
    void visit(const ReceiverTactic &tactic) override;
    void visit(const AttackerTactic &tactic) override;
    void visit(const MoveGoalieToGoalLineTactic &tactic) override;
    void visit(const PrepareKickoffMoveTactic &tactic) override;
    void visit(const BallPlacementMoveTactic &tactic) override;
    void visit(const AvoidInterferenceTactic &tactic) override;
    void visit(const PassDefenderTactic &tactic) override;
    void visit(const AssignedSkillTactic<ChipSkill> &tactic) override;
    void visit(const AssignedSkillTactic<DribbleSkill> &tactic) override;
    void visit(const AssignedSkillTactic<GetBehindBallSkill> &tactic) override;
    void visit(const AssignedSkillTactic<KeepAwaySkill> &tactic) override;
    void visit(const AssignedSkillTactic<KickSkill> &tactic) override;
    void visit(const AssignedSkillTactic<KickPassSkill> &tactic) override;
    void visit(const AssignedSkillTactic<ChipPassSkill> &tactic) override;
    void visit(const AssignedSkillTactic<PivotKickSkill> &tactic) override;
    void visit(const AssignedSkillTactic<ShootSkill> &tactic) override;
    void visit(const AssignedSkillTactic<DribbleShootSkill> &tactic) override;
    void visit(const KickoffChipSkillTactic &tactic) override;
    void visit(const BallPlacementDribbleTactic &tactic) override;

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

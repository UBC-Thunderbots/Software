#include "software/ai/motion_constraint/motion_constraint_visitor.h"

#include "software/ai/hl/stp/tactic/all_tactics.h"

void MotionConstraintVisitor::visit(const GoalieTactic &tactic)
{
    current_motion_constraints.erase(TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA);
    current_motion_constraints.erase(
        TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL);
    current_motion_constraints.erase(TbotsProto::MotionConstraint::FRIENDLY_HALF);
}

void MotionConstraintVisitor::visit(const CreaseDefenderTactic &tactic)
{
    current_motion_constraints.erase(
        TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL);
}

void MotionConstraintVisitor::visit(const ShadowEnemyTactic &tactic) {}

void MotionConstraintVisitor::visit(const MoveTactic &tactic) {}

void MotionConstraintVisitor::visit(const PrepareKickoffMoveTactic &tactic)
{
    current_motion_constraints.erase(TbotsProto::MotionConstraint::CENTER_CIRCLE);
    current_motion_constraints.erase(
        TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL);
    current_motion_constraints.erase(TbotsProto::MotionConstraint::ENEMY_HALF);
    current_motion_constraints.insert(
        TbotsProto::MotionConstraint::ENEMY_HALF_WITHOUT_CENTRE_CIRCLE);
}

void MotionConstraintVisitor::visit(const StopTactic &tactic) {}

void MotionConstraintVisitor::visit(const PenaltyKickTactic &tactic)
{
    current_motion_constraints.erase(
        TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL);
    current_motion_constraints.erase(TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA);
    current_motion_constraints.erase(TbotsProto::MotionConstraint::ENEMY_HALF);
}

void MotionConstraintVisitor::visit(const PenaltySetupTactic &tactic)
{
    current_motion_constraints.erase(TbotsProto::MotionConstraint::ENEMY_HALF);
    current_motion_constraints.erase(TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA);
    current_motion_constraints.erase(TbotsProto::MotionConstraint::FRIENDLY_HALF);
    current_motion_constraints.erase(
        TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL);
}

void MotionConstraintVisitor::visit(const ReceiverTactic &tactic) {}

void MotionConstraintVisitor::visit(const AttackerTactic &tactic) {}

void MotionConstraintVisitor::visit(const MoveGoalieToGoalLineTactic &tactic)
{
    current_motion_constraints.erase(TbotsProto::MotionConstraint::FRIENDLY_HALF);
    current_motion_constraints.erase(TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA);
}

void MotionConstraintVisitor::visit(const BallPlacementMoveTactic &tactic)
{
    current_motion_constraints.clear();
}

void MotionConstraintVisitor::visit(const AvoidInterferenceTactic &tactic)
{
    current_motion_constraints.clear();
}

void MotionConstraintVisitor::visit(const PassDefenderTactic &tactic) {}

void MotionConstraintVisitor::visit(const AssignedSkillTactic<ChipSkill> &tactic) {}

void MotionConstraintVisitor::visit(const AssignedSkillTactic<DribbleSkill> &tactic) {}

void MotionConstraintVisitor::visit(const AssignedSkillTactic<GetBehindBallSkill> &tactic)
{
}

void MotionConstraintVisitor::visit(const AssignedSkillTactic<KeepAwaySkill> &tactic) {}

void MotionConstraintVisitor::visit(const AssignedSkillTactic<KickSkill> &tactic) {}

void MotionConstraintVisitor::visit(const AssignedSkillTactic<KickPassSkill> &tactic) {}

void MotionConstraintVisitor::visit(const AssignedSkillTactic<ChipPassSkill> &tactic) {}

void MotionConstraintVisitor::visit(const AssignedSkillTactic<PivotKickSkill> &tactic) {}

void MotionConstraintVisitor::visit(const AssignedSkillTactic<ShootSkill> &tactic) {}

void MotionConstraintVisitor::visit(const AssignedSkillTactic<DribbleShootSkill> &tactic)
{
}

void MotionConstraintVisitor::visit(const KickoffChipSkillTactic &tactic)
{
    current_motion_constraints.erase(TbotsProto::MotionConstraint::CENTER_CIRCLE);
    current_motion_constraints.erase(TbotsProto::MotionConstraint::ENEMY_HALF);
    current_motion_constraints.erase(
        TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL);
    current_motion_constraints.insert(
        TbotsProto::MotionConstraint::ENEMY_HALF_WITHOUT_CENTRE_CIRCLE);
}

void MotionConstraintVisitor::visit(const BallPlacementDribbleTactic &tactic)
{
    current_motion_constraints.clear();
}

std::set<TbotsProto::MotionConstraint>
MotionConstraintVisitor::getUpdatedMotionConstraints(
    const Tactic &tactic,
    const std::set<TbotsProto::MotionConstraint> &existing_motion_constraints)
{
    current_motion_constraints = existing_motion_constraints;
    tactic.accept(*this);
    return current_motion_constraints;
}

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

void MotionConstraintVisitor::visit(const ChipTactic &tactic) {}

void MotionConstraintVisitor::visit(const KickTactic &tactic) {}

void MotionConstraintVisitor::visit(const PivotKickTactic &tactic) {}

void MotionConstraintVisitor::visit(const KickoffChipTactic &tactic)
{
    current_motion_constraints.erase(TbotsProto::MotionConstraint::CENTER_CIRCLE);
    current_motion_constraints.erase(TbotsProto::MotionConstraint::ENEMY_HALF);
    current_motion_constraints.erase(
        TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL);
    current_motion_constraints.insert(
        TbotsProto::MotionConstraint::ENEMY_HALF_WITHOUT_CENTRE_CIRCLE);
}

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

void MotionConstraintVisitor::visit(const DefenseShadowEnemyTactic &tactic) {}

void MotionConstraintVisitor::visit(const MoveTestTactic &tactic) {}

void MotionConstraintVisitor::visit(const StopTestTactic &tactic) {}

void MotionConstraintVisitor::visit(const GoalieTestTactic &tactic) {}

void MotionConstraintVisitor::visit(const DribbleTactic &tactic) {}

void MotionConstraintVisitor::visit(const GetBehindBallTactic &tactic) {}

void MotionConstraintVisitor::visit(const MoveGoalieToGoalLineTactic &tactic)
{
    current_motion_constraints.erase(TbotsProto::MotionConstraint::FRIENDLY_HALF);
    current_motion_constraints.erase(TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA);
}

void MotionConstraintVisitor::visit(const PlaceBallTactic &tactic)
{
    current_motion_constraints.erase(TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA);
    current_motion_constraints.erase(TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA);
    current_motion_constraints.erase(
        TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE);
}

void MotionConstraintVisitor::visit(const PlaceBallMoveTactic &tactic)
{
    current_motion_constraints.erase(TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA);
    current_motion_constraints.erase(TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA);
    current_motion_constraints.erase(
        TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE);
}

void MotionConstraintVisitor::visit(const WallKickoffTactic &tactic)
{
    current_motion_constraints.erase(TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA);
    current_motion_constraints.erase(
        TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE);
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

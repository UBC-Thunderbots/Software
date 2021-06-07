#include "software/ai/motion_constraint/motion_constraint_visitor.h"

#include "software/ai/hl/stp/tactic/all_tactics.h"



// We disable clang-format here because it makes these lists borderline unreadable,
// and certainly way more difficult to edit
// clang-format off
void MotionConstraintVisitor::visit(const ShadowFreekickerTactic &tactic) {}

void MotionConstraintVisitor::visit(const GoalieTactic &tactic)
{
    current_allowed_constraints = std::set<MotionConstraint>({
        MotionConstraint::FRIENDLY_DEFENSE_AREA,
        MotionConstraint::HALF_METER_AROUND_BALL,
        MotionConstraint::FRIENDLY_HALF
    });
}

void MotionConstraintVisitor::visit(const CreaseDefenderTactic &tactic) {
    current_allowed_constraints = std::set<MotionConstraint>({
        MotionConstraint::HALF_METER_AROUND_BALL,
    });
}

void MotionConstraintVisitor::visit(const ShadowEnemyTactic &tactic) {}

void MotionConstraintVisitor::visit(const MoveTactic &tactic) {}

void MotionConstraintVisitor::visit(const ChipTactic &tactic) {}

void MotionConstraintVisitor::visit(const KickTactic &tactic) {}

void MotionConstraintVisitor::visit(const PivotKickTactic &tactic) {}

void MotionConstraintVisitor::visit(const KickoffChipTactic &tactic)
{
    current_allowed_constraints = std::set<MotionConstraint>({
        MotionConstraint::CENTER_CIRCLE,
        MotionConstraint::HALF_METER_AROUND_BALL
    });
}

void MotionConstraintVisitor::visit(const StopTactic &tactic) {}

void MotionConstraintVisitor::visit(const PenaltyKickTactic &tactic)
{
    current_allowed_constraints = std::set<MotionConstraint>({
        MotionConstraint::HALF_METER_AROUND_BALL,
        MotionConstraint::ENEMY_DEFENSE_AREA,
        MotionConstraint::ENEMY_HALF
    });
}

void MotionConstraintVisitor::visit(const PenaltySetupTactic &tactic)
{
    current_allowed_constraints = std::set<MotionConstraint>({
        MotionConstraint::ENEMY_HALF,
        MotionConstraint::ENEMY_DEFENSE_AREA,
        MotionConstraint::FRIENDLY_HALF,
        MotionConstraint::HALF_METER_AROUND_BALL
    });
}

void MotionConstraintVisitor::visit(const ReceiverTactic &tactic) {}

void MotionConstraintVisitor::visit(const AttackerTactic &tactic) { }

void MotionConstraintVisitor::visit(const DefenseShadowEnemyTactic &tactic) {}

void MotionConstraintVisitor::visit(const MoveTestTactic &tactic) {}

void MotionConstraintVisitor::visit(const StopTestTactic &tactic) {}

void MotionConstraintVisitor::visit(const GoalieTestTactic &tactic) {}

void MotionConstraintVisitor::visit(const DribbleTactic &tactic) {}

void MotionConstraintVisitor::visit(const GetBehindBallTactic &tactic) {}

void MotionConstraintVisitor::visit(const MoveGoalieToGoalLineTactic &tactic)
{
    current_allowed_constraints = std::set<MotionConstraint>({
        MotionConstraint::FRIENDLY_HALF,
        MotionConstraint::FRIENDLY_DEFENSE_AREA
    });
}

// clang-format on

std::set<MotionConstraint> MotionConstraintVisitor::getCurrentAllowedConstraints(
    const Tactic &tactic)
{
    tactic.accept(*this);
    return current_allowed_constraints;
}

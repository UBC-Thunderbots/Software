#pragma once

// We forward-declare all the tactics because if we include them we induce a circular
// dependency between the individual library for each tactic and this visitor. This is
// because tactic.h includes tactic_visitor.h, and each individual library includes
// tactic.h. Note: every subclass of this visitor must include all of the classes listed
// below
class ChipTactic;
class CreaseDefenderTactic;
class DefenseShadowEnemyTactic;
class DribbleTactic;
class GetBehindBallTactic;
class GoalieTactic;
class GoalieTestTactic;
class KickTactic;
class KickoffChipTactic;
class MoveTactic;
class MoveTestTactic;
class AttackerTactic;
class PassDefenderTactic;
class PenaltyKickTactic;
class PenaltySetupTactic;
class PivotKickTactic;
class ReceiverTactic;
class ShadowEnemyTactic;
class StopTactic;
class StopTestTactic;
class MoveGoalieToGoalLineTactic;
class PrepareKickoffMoveTactic;

/**
 * Refer to the docs about why we use the Visitor Design Pattern
 */
class TacticVisitor
{
   public:
    virtual ~TacticVisitor() = default;

    // The javadoc comment for all methods here can be read as:
    /**
     * Visits an instance of X to perform an operation
     *
     * @param tactic The tactic to visit
     */

    virtual void visit(const ChipTactic &tactic)                 = 0;
    virtual void visit(const CreaseDefenderTactic &tactic)       = 0;
    virtual void visit(const DefenseShadowEnemyTactic &tactic)   = 0;
    virtual void visit(const DribbleTactic &tactic)              = 0;
    virtual void visit(const GetBehindBallTactic &tactic)        = 0;
    virtual void visit(const GoalieTactic &tactic)               = 0;
    virtual void visit(const GoalieTestTactic &tactic)           = 0;
    virtual void visit(const KickTactic &tactic)                 = 0;
    virtual void visit(const KickoffChipTactic &tactic)          = 0;
    virtual void visit(const MoveTactic &tactic)                 = 0;
    virtual void visit(const MoveTestTactic &tactic)             = 0;
    virtual void visit(const AttackerTactic &tactic)             = 0;
    virtual void visit(const PassDefenderTactic &tactic)         = 0;
    virtual void visit(const PenaltyKickTactic &tactic)          = 0;
    virtual void visit(const PenaltySetupTactic &tactic)         = 0;
    virtual void visit(const PivotKickTactic &tactic)            = 0;
    virtual void visit(const ReceiverTactic &tactic)             = 0;
    virtual void visit(const ShadowEnemyTactic &tactic)          = 0;
    virtual void visit(const StopTactic &tactic)                 = 0;
    virtual void visit(const StopTestTactic &tactic)             = 0;
    virtual void visit(const MoveGoalieToGoalLineTactic &tactic) = 0;
    virtual void visit(const PrepareKickoffMoveTactic &tactic)   = 0;
};

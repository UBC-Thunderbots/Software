#pragma once

// We forward-declare all the tactics because if we include them we induce a circular
// dependency between the individual library for each tactic and this visitor. This is
// because tactic.h includes tactic_visitor.h, and each individual library includes
// tactic.h. Note: every subclass of this visitor must include all of the classes listed
// below
class CherryPickTactic;
class ShadowFreekickerTactic;
class GoalieTactic;
class CreaseDefenderTactic;
class ShadowEnemyTactic;
class MoveTactic;
class ChipTactic;
class KickoffChipTactic;
class StopTactic;
class PatrolTactic;
class PenaltyKickTactic;
class PenaltySetupTactic;
class ReceiverTactic;
class ShootGoalTactic;
class PasserTactic;
class DefenseShadowEnemyTactic;
class MoveTestTactic;
class StopTestTactic;
class GoalieTestTactic;

/**
 * Refer to the docs about why we use the Visitor Design Pattern
 */
class MutableTacticVisitor
{
   public:
    virtual ~MutableTacticVisitor() = default;

    // The javadoc comment for all methods here can be read as:
    /**
     * Visits an instance of X to perform an operation
     *
     * @param tactic The tactic to visit
     */

    virtual void visit(CherryPickTactic &tactic)         = 0;
    virtual void visit(ShadowFreekickerTactic &tactic)   = 0;
    virtual void visit(GoalieTactic &tactic)             = 0;
    virtual void visit(CreaseDefenderTactic &tactic)     = 0;
    virtual void visit(ShadowEnemyTactic &tactic)        = 0;
    virtual void visit(MoveTactic &tactic)               = 0;
    virtual void visit(ChipTactic &tactic)               = 0;
    virtual void visit(KickoffChipTactic &tactic)        = 0;
    virtual void visit(StopTactic &tactic)               = 0;
    virtual void visit(PatrolTactic &tactic)             = 0;
    virtual void visit(PenaltyKickTactic &tactic)        = 0;
    virtual void visit(PenaltySetupTactic &tactic)       = 0;
    virtual void visit(ReceiverTactic &tactic)           = 0;
    virtual void visit(ShootGoalTactic &tactic)          = 0;
    virtual void visit(PasserTactic &tactic)             = 0;
    virtual void visit(DefenseShadowEnemyTactic &tactic) = 0;
    virtual void visit(MoveTestTactic &tactic)           = 0;
    virtual void visit(StopTestTactic &tactic)           = 0;
    virtual void visit(GoalieTestTactic &tactic)         = 0;
};

#pragma once

// We forward-declare all the tactics because if we include them we induce a
// circular dependency between the Individual library for each tactic and this
// visitor. Ex: `CherryPickTactic` includes `TacticVisitor`, but `TacticVisitor`
// also includes `CherryPickTactic`
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


/**
 * This class provides a mutable interface for all Tactic Visitors. The Visitor design
 * pattern allows us to perform operations on Tactic objects without needing to check
 * which concrete type it is with an if/else statement, and we don't need to pollute the
 * Tactic classes with information or functions that are specific to the task we
 * want to perform. The mutable version allows us to update parameters within
 * each Tactic.
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
};
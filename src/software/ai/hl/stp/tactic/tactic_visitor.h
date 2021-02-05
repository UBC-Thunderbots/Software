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
class PenaltyKickTactic;
class PenaltySetupTactic;
class ReceiverTactic;
class ShootGoalTactic;
class PasserTactic;
class DefenseShadowEnemyTactic;
class MoveTestTactic;
class StopTestTactic;
class GoalieTestTactic;
class InterceptBallTactic;

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

    virtual void visit(const CherryPickTactic &tactic)         = 0;
    virtual void visit(const ShadowFreekickerTactic &tactic)   = 0;
    virtual void visit(const GoalieTactic &tactic)             = 0;
    virtual void visit(const CreaseDefenderTactic &tactic)     = 0;
    virtual void visit(const ShadowEnemyTactic &tactic)        = 0;
    virtual void visit(const MoveTactic &tactic)               = 0;
    virtual void visit(const ChipTactic &tactic)               = 0;
    virtual void visit(const KickoffChipTactic &tactic)        = 0;
    virtual void visit(const StopTactic &tactic)               = 0;
    virtual void visit(const PenaltyKickTactic &tactic)        = 0;
    virtual void visit(const PenaltySetupTactic &tactic)       = 0;
    virtual void visit(const ReceiverTactic &tactic)           = 0;
    virtual void visit(const ShootGoalTactic &tactic)          = 0;
    virtual void visit(const PasserTactic &tactic)             = 0;
    virtual void visit(const DefenseShadowEnemyTactic &tactic) = 0;
    virtual void visit(const MoveTestTactic &tactic)           = 0;
    virtual void visit(const StopTestTactic &tactic)           = 0;
    virtual void visit(const GoalieTestTactic &tactic)         = 0;
    virtual void visit(const InterceptBallTactic &tactic)      = 0;
};

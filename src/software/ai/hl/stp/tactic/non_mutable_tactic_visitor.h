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
class MoveTestTactic;
class StopTestTactic;
class DefenseShadowEnemyTactic;

/**
 * Refer to the docs about why we use the Visitor Design Pattern
 */
class NonMutableTacticVisitor
{
   public:
    virtual ~NonMutableTacticVisitor() = default;

    // The javadoc comment for all methods here can be read as:
    /**
     * Visits an instance of X to perform an operation
     *
     * @param tactic The tactic to visit
     */

    virtual void visit(const CherryPickTactic &tactic) const         = 0;
    virtual void visit(const ShadowFreekickerTactic &tactic) const   = 0;
    virtual void visit(const GoalieTactic &tactic) const             = 0;
    virtual void visit(const CreaseDefenderTactic &tactic) const     = 0;
    virtual void visit(const ShadowEnemyTactic &tactic) const        = 0;
    virtual void visit(const MoveTactic &tactic) const               = 0;
    virtual void visit(const ChipTactic &tactic) const               = 0;
    virtual void visit(const KickoffChipTactic &tactic) const        = 0;
    virtual void visit(const StopTactic &tactic) const               = 0;
    virtual void visit(const PatrolTactic &tactic) const             = 0;
    virtual void visit(const PenaltyKickTactic &tactic) const        = 0;
    virtual void visit(const PenaltySetupTactic &tactic) const       = 0;
    virtual void visit(const ReceiverTactic &tactic) const           = 0;
    virtual void visit(const ShootGoalTactic &tactic) const          = 0;
    virtual void visit(const PasserTactic &tactic) const             = 0;
    virtual void visit(const DefenseShadowEnemyTactic &tactic) const = 0;
    virtual void visit(const MoveTestTactic &tactic) const           = 0;
    virtual void visit(const StopTestTactic &tactic) const           = 0;
};

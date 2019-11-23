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
class BlockShotPathTactic;
class MoveTactic;
class ChipTactic;
class KickoffChipTactic;
class StopTactic;
class PenaltyKickTactic;
class PenaltySetupTactic;
class ReceiverTactic;
class PatrolTactic;
class ShootGoalTactic;
class PasserTactic;
class GrabBallTactic;
class MoveTestTactic;
class StopTestTactic;
class DefenseShadowEnemyTactic;

/**
 * This class provides an interface for all Tactic Visitors. The Visitor design pattern
 * allows us to perform operations on Tactic objects without needing to check which
 * concrete type it is with an if/else statement, and we don't need to pollute the
 * Tactic classes with information or functions that are specific to the task we
 * want to perform.
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
    virtual void visit(const ShadowFreekickerTactic &tactic) = 0;
    virtual void visit(const GoalieTactic &tactic) = 0;
    virtual void visit(const CreaseDefenderTactic &tactic) = 0;
    virtual void visit(const ShadowEnemyTactic &tactic) = 0;
    virtual void visit(const BlockShotPathTactic &tactic) = 0;
    virtual void visit(const MoveTactic &tactic) = 0;
    virtual void visit(const ChipTactic &tactic) = 0;
    virtual void visit(const KickoffChipTactic &tactic)        = 0;
    virtual void visit(const StopTactic &tactic) = 0;
    virtual void visit(const PenaltyKickTactic &tactic) = 0;
    virtual void visit(const PenaltySetupTactic &tactic)       = 0;
    virtual void visit(const ReceiverTactic &tactic) = 0;
    virtual void visit(const PatrolTactic &tactic) = 0;
    virtual void visit(const ShootGoalTactic &tactic) = 0;
    virtual void visit(const PasserTactic &tactic) = 0;
    virtual void visit(const GrabBallTactic &tactic) = 0;
    virtual void visit(const MoveTestTactic &tactic) = 0;
    virtual void visit(const StopTestTactic &tactic) = 0;
    virtual void visit(const DefenseShadowEnemyTactic &tactic) = 0;
};

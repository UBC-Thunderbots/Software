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


    /**
     * Visits a CherryPickTactic to perform an operation.
     *
     * @param tactic The CherryPickTactic to visit
     */
    virtual void visit(const CherryPickTactic &tactic) = 0;

    /**
     * Visits a ShadowFreekickerTactic to perform an operation.
     *
     * @param tactic The ShadowFreekickerTactic to visit
     */
    virtual void visit(const ShadowFreekickerTactic &tactic) = 0;

    /**
     * Visits a GoalieTactic to perform an operation.
     *
     * @param tactic The GoalieTactic to visit
     */
    virtual void visit(const GoalieTactic &tactic) = 0;

    /**
     * Visits a CreaseDefenderTactic to perform an operation.
     *
     * @param tactic The CreaseDefenderTactic to visit
     */
    virtual void visit(const CreaseDefenderTactic &tactic) = 0;

    /**
     * Visits a ShadowEnemyTactic to perform an operation.
     *
     * @param tactic The ShadowEnemyTactic to visit
     */
    virtual void visit(const ShadowEnemyTactic &tactic) = 0;

    /**
     * Visits a BlockShotPathTactic to perform an operation.
     *
     * @param tactic The BlockShotPathTactic to visit
     */
    virtual void visit(const BlockShotPathTactic &tactic) = 0;

    /**
     * Visits a MoveTactic to perform an operation.
     *
     * @param tactic The MoveTactic to visit
     */
    virtual void visit(const MoveTactic &tactic) = 0;

    /**
     * Visits a ChipTactic to perform an operation.
     *
     * @param tactic The ChipTactic to visit
     */
    virtual void visit(const ChipTactic &tactic) = 0;

    /**
     * Visits a KickoffChipTactic to perform an operation.
     *
     * @param tactic The KickoffChipTactic to visit
     */
    virtual void visit(const KickoffChipTactic &tactic) = 0;

    /**
     * Visits a StopTactic to perform an operation.
     *
     * @param tactic The StopTactic to visit
     */
    virtual void visit(const StopTactic &tactic) = 0;

    /**
     * Visits a PenaltyKickTactic to perform an operation.
     *
     * @param tactic The PenaltyKickTactic to visit
     */
    virtual void visit(const PenaltyKickTactic &tactic) = 0;

    /**
     * Visits a PenaltySetupTactic to perform an operation.
     *
     * @param tactic The PenaltySetupTactic to visit
     */
    virtual void visit(const PenaltySetupTactic &tactic) = 0;

    /**
     * Visits a ReceiverTactic to perform an operation.
     *
     * @param tactic The ReceiverTactic to visit
     */
    virtual void visit(const ReceiverTactic &tactic) = 0;

    /**
     * Visits a PatrolTactic to perform an operation.
     *
     * @param tactic The PatrolTactic to visit
     */
    virtual void visit(const PatrolTactic &tactic) = 0;

    /**
     * Visits a ShootGoalTactic to perform an operation.
     *
     * @param tactic The ShootGoalTactic to visit
     */
    virtual void visit(const ShootGoalTactic &tactic) = 0;

    /**
     * Visits a PasserTactic to perform an operation.
     *
     * @param tactic The PasserTactic to visit
     */
    virtual void visit(const PasserTactic &tactic) = 0;

    /**
     * Visits a GrabBallTactic to perform an operation.
     *
     * @param tactic The GrabBallTactic to visit
     */
    virtual void visit(const GrabBallTactic &tactic) = 0;

    /**
     * Visits a MoveTestTactic to perform an operation.
     *
     * @param tactic The MoveTestTactic to visit
     */
    virtual void visit(const MoveTestTactic &tactic) = 0;

    /**
     * Visits a StopTestTactic to perform an operation.
     *
     * @param tactic The StopTestTactic to visit
     */
    virtual void visit(const StopTestTactic &tactic) = 0;

    /**
     * Visits a DefenseShadowEnemyTactic to perform an operation.
     *
     * @param tactic The DefenseShadowEnemyTactic to visit
     */
    virtual void visit(const DefenseShadowEnemyTactic &tactic) = 0;
};

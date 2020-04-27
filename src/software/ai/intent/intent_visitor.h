#pragma once

// We forward-declare all the intents because if we include them we induce a
// circular dependency between the Individual library for each intent and this
// visitor. Ex: `CatchIntent` includes `IntentVisitor`, but `IntentVisitor`
// also includes `CatchIntent`
class CatchIntent;
class ChipIntent;
class DirectVelocityIntent;
class DirectWheelsIntent;
class DribbleIntent;
class KickIntent;
class MoveIntent;
class MoveSpinIntent;
class PivotIntent;
class StopIntent;

/**
 * This class provides an interface for all Intent Visitors. The Visitor design pattern
 * allows us to perform operations on Intent objects without needing to check which
 * concrete type it is with an if/else statement, and we don't need to pollute the
 * Intent classes with information or functions that are specific to the task we
 * want to perform.
 */
class IntentVisitor
{
   public:
    virtual ~IntentVisitor() = default;

    /**
     * Visits a CatchIntent to perform an operation.
     *
     * @param catch_intent The CatchIntent to visit
     */
    virtual void visit(const CatchIntent &catch_intent) = 0;

    /**
     * Visits a ChipIntent to perform an operation.
     *
     * @param chip_intent The ChipIntent to visit
     */
    virtual void visit(const ChipIntent &chip_intent) = 0;

    /**
     * Visits a DirectVelocityIntent to perform an operation.
     *
     * @param direct_velocity_intent The DirectVelocityIntent to visit
     */
    virtual void visit(const DirectVelocityIntent &direct_velocity_intent) = 0;

    /**
     * Visits a DirectWheelsIntent to perform an operation.
     *
     * @param direct_wheels_intent The DirectWheelsIntent to visit
     */
    virtual void visit(const DirectWheelsIntent &direct_wheels_intent) = 0;

    /**
     * Visits a DribbleIntent to perform an operation.
     *
     * @param direct_wheels_intent The DribbleIntent to visit
     */
    virtual void visit(const DribbleIntent &direct_wheels_intent) = 0;

    /**
     * Visits a KickIntent to perform an operation.
     *
     * @param kick_intent The KickIntent to visit
     */
    virtual void visit(const KickIntent &kick_intent) = 0;

    /**
     * Visits a MoveIntent to perform an operation.
     *
     * @param move_intent The MoveIntent to visit
     */
    virtual void visit(const MoveIntent &move_intent) = 0;

    /**
     * Visits a MoveSpinIntent to perform an operation.
     *
     * @param move_spin_intent The MoveSpinIntent to visit
     */
    virtual void visit(const MoveSpinIntent &move_spin_intent) = 0;

    /**
     * Visits a PivotIntent to perform an operation.
     *
     * @param pivot_intent The PivotIntent to visit
     */
    virtual void visit(const PivotIntent &pivot_intent) = 0;

    /**
     * Visits a StopIntent to perform an operation.
     *
     * @param stop_intent The StopIntent to visit
     */
    virtual void visit(const StopIntent &stop_intent) = 0;
};

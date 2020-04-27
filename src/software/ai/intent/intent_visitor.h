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
     * Visits an Intent to perform an operation.
     *
     * @param The Intent to visit
     */
    virtual void visit(const CatchIntent &catch_intent)                    = 0;
    virtual void visit(const ChipIntent &chip_intent)                      = 0;
    virtual void visit(const DirectVelocityIntent &direct_velocity_intent) = 0;
    virtual void visit(const DirectWheelsIntent &direct_wheels_intent)     = 0;
    virtual void visit(const DribbleIntent &direct_wheels_intent)          = 0;
    virtual void visit(const KickIntent &kick_intent)                      = 0;
    virtual void visit(const MoveIntent &move_intent)                      = 0;
    virtual void visit(const MoveSpinIntent &move_spin_intent)             = 0;
    virtual void visit(const PivotIntent &pivot_intent)                    = 0;
    virtual void visit(const StopIntent &stop_intent)                      = 0;
};

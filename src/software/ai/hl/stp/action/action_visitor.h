#pragma once

// We forward-declare all the actions because if we include them we induce a
// circular dependency between the individual library for each action and this
// visitor. Ex. `MoveAction` includes `ActionVisitor`, but `ActionVisitor`
// also includes `MoveAction`
class ChipAction;
class DribbleAction;
class InterceptBallAction;
class KickAction;
class MoveAction;
class MoveSpinAction;
class PivotAction;
class StopAction;

/**
 * This class provides an interface for all Action visitors.
 * (ie. the "Visitor" design pattern)
 */
class ActionVisitor
{
   public:
    virtual ~ActionVisitor() = default;

    // The javadoc comment for all methods here can be read as:
    /**
     * Visits an instance of X to perform an operation
     *
     * @param action The action to visit
     */

    virtual void visit(const ChipAction& action)          = 0;
    virtual void visit(const DribbleAction& action)       = 0;
    virtual void visit(const InterceptBallAction& action) = 0;
    virtual void visit(const KickAction& action)          = 0;
    virtual void visit(const MoveAction& action)          = 0;
    virtual void visit(const MoveSpinAction& action)      = 0;
    virtual void visit(const PivotAction& action)         = 0;
    virtual void visit(const StopAction& action)          = 0;
};

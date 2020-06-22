#pragma once

// We forward-declare all the actions because if we include them we induce a circular
// dependency between the individual library for each action and this visitor. This is
// because action.h includes action_visitor.h, and each individual library includes
// action.h. Note: every subclass of this visitor must include all of the classes listed
// below
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
class MutableActionVisitor
{
   public:
    virtual ~MutableActionVisitor() = default;

    // The javadoc comment for all methods here can be read as:
    /**
     * Visits an instance of X to perform an operation
     *
     * @param action The action to visit
     */

    virtual void visit(ChipAction& action)          = 0;
    virtual void visit(DribbleAction& action)       = 0;
    virtual void visit(InterceptBallAction& action) = 0;
    virtual void visit(KickAction& action)          = 0;
    virtual void visit(MoveAction& action)          = 0;
    virtual void visit(MoveSpinAction& action)      = 0;
    virtual void visit(PivotAction& action)         = 0;
    virtual void visit(StopAction& action)          = 0;
};

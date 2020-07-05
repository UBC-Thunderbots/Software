#pragma once

// We forward-declare all the primitives because if we include them we induce a circular
// dependency between the individual library for each primitive and this visitor. This is
// because primitive.h includes primitive_visitor.h, and each individual library includes
// primitive.h. Note: every subclass of this visitor must include all of the classes
// listed below
class CatchPrimitive;
class ChipPrimitive;
class DirectVelocityPrimitive;
class DirectWheelsPrimitive;
class DribblePrimitive;
class KickPrimitive;
class MovePrimitive;
class MoveSpinPrimitive;
class PivotPrimitive;
class StopPrimitive;

/**
 * This class provides an interface for all Primitive Visitors. The Visitor design pattern
 * allows us to perform operations on Primitive objects without needing to check which
 * concrete type it is with an if/else statement, and we don't need to pollute the
 * Primitive classes with information or functions that are specific to the task we
 * want to perform.
 */
class PrimitiveVisitor
{
   public:
    virtual ~PrimitiveVisitor() = default;

    // The javadoc comment for all methods here can be read as:
    /**
     * Visits an instance of X to perform an operation
     *
     * @param primitive The primitive to visit
     */

    virtual void visit(const CatchPrimitive &catch_primitive)                    = 0;
    virtual void visit(const ChipPrimitive &chip_primitive)                      = 0;
    virtual void visit(const DirectVelocityPrimitive &direct_velocity_primitive) = 0;
    virtual void visit(const DirectWheelsPrimitive &direct_wheels_primitive)     = 0;
    virtual void visit(const KickPrimitive &kick_primitive)                      = 0;
    virtual void visit(const MovePrimitive &move_primitive)                      = 0;
    virtual void visit(const MoveSpinPrimitive &movespin_primitive)              = 0;
    virtual void visit(const DribblePrimitive &dribble_primitive)                = 0;
    virtual void visit(const PivotPrimitive &pivot_primitive)                    = 0;
    virtual void visit(const StopPrimitive &stop_primitive)                      = 0;
};

#pragma once

// We forward-declare all the intents because if we include them we induce a circular
// dependency between the individual library for each intent and this visitor. This is
// because intent.h includes intent_visitor.h, and each individual library includes
// intent.h. Note: every subclass of this visitor must include all of the classes listed
// below
class MoveIntent;
class NavigatingIntent;
class DirectPrimitiveIntent;

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
    virtual void visit(const MoveIntent &intent)            = 0;
    virtual void visit(const NavigatingIntent &intent)            = 0;
    virtual void visit(const DirectPrimitiveIntent &intent) = 0;
};

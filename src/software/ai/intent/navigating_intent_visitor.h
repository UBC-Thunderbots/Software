#pragma once

// We forward-declare all the intents because if we include them we induce a circular
// dependency between the individual library for each intent and this visitor. This is
// because intent.h includes intent_visitor.h, and each individual library includes
// intent.h. Note: every subclass of this visitor must include all of the classes listed
// below
class MoveIntent;

/**
 * This class provides an interface for all NavigatingIntent Visitors. The Visitor design
 * pattern allows us to perform operations on NavigatingIntent objects without needing to
 * check which concrete type it is with an if/else statement, and we don't need to pollute
 * the NavigatingIntent classes with information or functions that are specific to the
 * task we want to perform.
 */
class NavigatingIntentVisitor
{
   public:
    virtual ~NavigatingIntentVisitor() = default;

    /**
     * Visits an NavigatingIntent to perform an operation.
     *
     * @param The NavigatingIntent to visit
     */
    virtual void visit(const MoveIntent &intent) = 0;
};

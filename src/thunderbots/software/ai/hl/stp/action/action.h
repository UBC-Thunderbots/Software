#pragma once

#include <boost/bind.hpp>
#include <boost/coroutine2/all.hpp>

#include "ai/intent/intent.h"
#include "ai/world/robot.h"

// We typedef the coroutine return type to make it shorter, more descriptive,
// and easier to work with
typedef boost::coroutines2::coroutine<std::unique_ptr<Intent>> intent_coroutine;

/**
 * The Action class is the lowest level of abstraction in our STP architecture.
 * They abstract just above the Intent layer, and typically performs similar behavior
 * to the Intents but while making sure preconditions are met.
 */
class Action
{
   public:
    /**
     * Creates a new Action for the given robot.
     *
     * @param robot The robot that will be performing this Action
     */
    explicit Action(const Robot &robot);

    /**
     * Returns true if the Action is done and false otherwise
     *
     * @return true if the Action is done and false otherwise
     */
    bool done() const;

    virtual ~Action() = default;

   protected:
    /**
     * Runs the coroutine and get the next Intent to run from the calculateNextIntent
     * function. If the Action is not done, the next Intent is returned. If the Action
     * is done, a null unique_ptr is returned.
     *
     * @return A unique pointer to the next Intent that should be run for the Action.
     * If the Action is done, an empty/null unique pointer is returned.
     */
    std::unique_ptr<Intent> getNextIntent();

    // The coroutine that sequentially returns the Intents the Action wants to run
    intent_coroutine::pull_type intent_sequence;
    // The robot performing this Action
    Robot robot;

   private:
    /**
     * A wrapper function for the calculateNextIntent function.
     *
     * This function exists because when the coroutine (intent_sequence) is first
     * constructed the coroutine is called/entered. This would normally cause the
     * calculateNextIntentWrapper to be run once and potentially return incorrect results
     * due to default constructed values.
     *
     * This wrapper function will yield a null pointer the first time it's called and
     * otherwise use the calculateNextIntent function. This first "null" value will never
     * be seen/used by the rest of the system since this will be during construction,
     * and the coroutine will be called again with valid parameters before any values are
     * returned. This effectively "shields" the logic from any errors caused by default
     * values during construction.
     *
     * @param yield The coroutine push_type for the Action
     *
     * @return A unique pointer to the next Intent that should be run for the Action.
     * If the Action is done, an empty/null unique pointer is returned. The very first
     * time this function is called, a null pointer will be returned (this does not
     * signify the Action is done).
     */
    std::unique_ptr<Intent> calculateNextIntentWrapper(
        intent_coroutine::push_type &yield);

    /**
     * Calculates the next Intent for the Action. If the Action is done
     * (ie. it has achieved its objective and has no more Intents to return),
     * an empty/null unique pointer is returned.
     *
     * @param yield The coroutine push_type for the Action
     *
     * @return A unique pointer to the next Intent that should be run for the Action.
     * If the Action is done, an empty/null unique pointer is returned.
     */
    virtual std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type &yield) = 0;
};

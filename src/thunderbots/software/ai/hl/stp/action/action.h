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
    virtual ~Action() = default;

   private:
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

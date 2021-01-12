#pragma once

#include <boost/bind.hpp>
#include <boost/coroutine2/all.hpp>

#include "software/ai/intent/intent.h"
#include "software/world/robot.h"
#include "software/world/world.h"

// We typedef the coroutine return type to make it shorter, more descriptive,
// and easier to work with
typedef boost::coroutines2::coroutine<std::unique_ptr<Intent>> IntentCoroutine;

/**
 * The Action class is the lowest level of abstraction in our STP architecture.
 * Actions are stateful, and represent simple Actions any robot can perform.
 * Examples of this are
 * - Moving to a position (without colliding with anything)
 * - Shooting / kicking at a target
 * - Intercepting / catching the ball
 *
 * Actions use Intents to implement their behaviour. Actions are different than
 * Intents because Intents are not stateful, and generally do not check if their
 * preconditions are met when they are run. Actions are responsible for making sure
 * any preconditions of an Intent are met before running it, which is where the
 * statefulness comes into play. For example, an Action may need to move a robot
 * into the correct position / alignment before shooting.
 */
class Action
{
   public:
    /**
     * Creates a new Action for the given robot.
     * @param loop_forever: whether action should continuously restart once its done
     */
    explicit Action(bool loop_forever);

    Action() = delete;

    /**
     * Returns true if the Action is done and false otherwise. The Action is considered
     * done when its coroutine is done (the calculateNextIntent() function has no more
     * work to do).
     *
     * @return true if the Action is done and false otherwise
     */
    bool done() const;

    /**
     * Restarts the action logic
     */
    void restart();

    /**
     * Runs the coroutine and get the next Intent to run from the calculateNextIntent
     * function. If the Action is not done, the next Intent is returned. If the Action
     * is done, a null unique_ptr is returned.
     *
     * @return A unique pointer to the next Intent that should be run for the Action.
     * If the Action is done, an empty/null unique pointer is returned.
     */
    std::unique_ptr<Intent> getNextIntent();

    /**
     * Gets the robot assigned to this action, if one is assigned
     *
     * @return the robot assigned to this action, if one is assigned
     */
    std::optional<Robot> getRobot();

    /**
     * Updates the robot
     *
     * @param robot The robot to update with
     */
    void updateRobot(const Robot &robot);

    /**
     * Updates the world parameters for this action
     *
     * @param world The world that will be used to update the action
     */
    virtual void updateWorldParams(const World &world) = 0;

    virtual ~Action() = default;

   protected:
    // The coroutine that sequentially returns the Intents the Action wants to run
    IntentCoroutine::pull_type intent_sequence;
    // The robot performing this Action
    std::optional<Robot> robot;

   private:
    /**
     * A wrapper function for the calculateNextIntent function.
     *
     * This function exists because when the coroutine (intent_sequence) is first
     * constructed the coroutine is called/entered. This would normally cause the
     * calculateNextIntentWrapper to be run once and potentially return incorrect results
     * due to default constructed values. Calling the calculateNextIntent function this
     * early also results in virtual function errors because this early in object
     * construction, the concrete implementation of this Action doesn't exist yet so
     * we would actually be trying to call the virtual function, which doesn't work.
     *
     * This wrapper function will yield a null pointer the first time it's called and
     * otherwise use the calculateNextIntent function. This first "null" value will never
     * be seen/used by the rest of the system since this will be during construction,
     * and the coroutine will be called again with valid parameters before any values are
     * returned. This effectively "shields" the logic from any errors caused by default
     * values during construction.
     *
     * This function yields a unique pointer to the next Intent that should be run for the
     * Action. If the Action is done, an empty/null unique pointer is returned. The very
     * first time this function is called, a null pointer will be returned (this does not
     * signify the Action is done). This yield happens in place of a return.
     *
     * @param yield The coroutine push_type for the Action
     */
    void calculateNextIntentWrapper(IntentCoroutine::push_type &yield);

    /**
     * Calculates the next Intent for the Action. If the Action is done
     * (ie. it has achieved its objective and has no more Intents to return),
     * an empty/null unique pointer is returned.
     *
     * This function yields a unique pointer to the next Intent that should be run for the
     * Action. If the Action is done, an empty/null unique pointer is returned. This yield
     * happens in place of a return
     *
     * @param yield The coroutine push_type for the Action
     */
    virtual void calculateNextIntent(IntentCoroutine::push_type &yield) = 0;

    // Whether or not this action should loop forever by restarting each time it is done
    bool loop_forever;
};

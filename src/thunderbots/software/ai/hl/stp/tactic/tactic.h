#pragma once

#include <boost/coroutine2/all.hpp>
#include <optional>

#include "ai/hl/stp/action/action.h"
#include "ai/intent/intent.h"
#include "ai/world/world.h"

/**
 * In the STP framework, a Tactic represents a role or objective for a single robot. For
 * example, being the goalie, shooting on the opposing goal, moving to receive a pass,
 * or acting as a defender. Tactics are where most of the complicated logic takes place,
 * and they tend to rely a lot on our Evaluation functions. Ultimately, Tactics will
 * return the next Intent that the Robot assigned to this Tactic should run in order
 * to work towards its objective.
 *
 * HOW THIS CLASS IS USED:
 * Plays will construct and return the Tactics they want to be running. Every time a play
 * is run, it will update the parameters of each tactic with the updateParams(...)
 * function (see the concrete implementations of this class for examples). This is done
 * every time in order for the Tactics to have the most up to date information when they
 * calculate the next Intent they want to run (for example if we were following a moving
 * robot, we need to constantly update our destination).
 *
 * The calulateRobotCost() and getNextIntent() functions will be called after the params
 * are updated. Params must be updated first so that these functions can make the correct
 * decisions.
 *
 * See the Play and PlayExecutor classes for more details on how Tactics are used
 */
class Tactic
{
   public:
    /**
     * Creates a new Tactic. The Tactic will initially have no Robot assigned to it.
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes and will never report done
     */
    explicit Tactic(bool loop_forever);

    /**
     * Returns true if the Tactic is done and false otherwise. If the Tactic is supposed
     * to loop forever, this function will always return false.
     *
     * @return true if the Tactic is done and false otherwise
     */
    bool done() const;

    /**
     * Returns the Robot assigned to this Tactic
     *
     * @return an std::optional containing the Robot assigned to this Tactic if one has
     * been assigned, otherwise returns std::nullopt
     */
    std::optional<Robot> getAssignedRobot() const;

    /**
     * Updates the robot assigned to this Tactic
     *
     * @param robot The updated state of the Robot that should be performing
     * this Tactic
     */
    void updateRobot(const Robot &robot);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. The returned cost
     * value must be in the range [0, 1], with smaller values indicating a higher
     * preference for the robot.
     *
     * For example, a tactic that wanted a robot to shoot the ball would return lower
     * costs for robots closer to the ball than for robots far from the ball.
     *
     * @param robot The Robot to calculate the cost for
     * @param world The state of the world used to perform the cost calculation
     *
     * @return A cost value in the range [0, 1] indicating the cost of assigning the given
     * robot to this Tactic. Lower cost values indicate more preferred robots.
     */
    virtual double calculateRobotCost(const Robot &robot, const World &world) = 0;

    /**
     * Runs the coroutine and get the next Intent to run from the calculateNextIntent
     * function. If the Tactic is not done, the next Intent is returned. If the Tactic
     * is done, a nullptr is returned.
     *
     * @return A unique pointer to the next Intent that should be run for the Tactic.
     * If the Tactic is done, a nullptr is returned.
     */
    std::unique_ptr<Intent> getNextIntent();

    /**
     * Returns the name of the Tactic
     *
     * @return the name of the Tactic
     */
    virtual std::string getName() const = 0;

    virtual ~Tactic() = default;

   protected:
    // The robot performing this Tactic
    std::optional<Robot> robot;

   private:
    /**
     * A wrapper function for the calculateNextIntent function.
     *
     * This function exists because when the coroutine (intent_sequence) is first
     * constructed the coroutine is called/entered. This would normally cause the
     * calculateNextIntent to be run once and potentially return incorrect results
     * due to default constructed values.
     *
     * This wrapper function will yield a null pointer the first time it's called and
     * otherwise use the calculateNextIntent function. This first "null" value will never
     * be seen/used by the rest of the system since this will be during construction,
     * and the coroutine will be called again with valid parameters before any values are
     * returned. This effectively "shields" the logic from any errors caused by default
     * values during construction.
     *
     * @param yield The coroutine push_type for the Tactic
     *
     * @return A unique pointer to the next Intent that should be run for the Tactic.
     * If the Tactic is done, an empty/null unique pointer is returned. The very first
     * time this function is called, a null pointer will be returned (this does not
     * signify the Tactic is done).
     */
    std::unique_ptr<Intent> calculateNextIntentWrapper(
        intent_coroutine::push_type &yield);

    /**
     * Calculates the next Intent for the Tactic. If the Tactic is done
     * (ie. it has achieved its objective and has no more Intents to return),
     * a nullptr is returned.
     *
     * @param yield The coroutine push_type for the Tactic
     *
     * @return A unique pointer to the next Intent that should be run for the Tactic.
     * If the Tactic is done, a nullptr is returned.
     */
    virtual std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type &yield) = 0;

    /**
     * A helper function that runs the intent_sequence coroutine and returns the result
     * of the coroutine. The done_ member variable is also updated to reflect whether
     * or not the Tactic is done. If the Tactic is done, a nullptr is returned.
     *
     * @return the next Intent this Tactic wants to run. If the Tactic is done, a nullptr
     * is returned
     */
    std::unique_ptr<Intent> getNextIntentHelper();

    // The coroutine that sequentially returns the Intents the Tactic wants to run
    intent_coroutine::pull_type intent_sequence;
    // Whether or not this Tactic is done
    bool done_;
    // Whether or not this tactic should loop forever by restarting each time it is done
    bool loop_forever;
};

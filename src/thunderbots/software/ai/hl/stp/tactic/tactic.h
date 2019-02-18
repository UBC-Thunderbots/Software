#pragma once

#include <boost/coroutine2/all.hpp>

#include "ai/hl/stp/action/action.h"
#include "ai/intent/intent.h"

/**
 * In the STP framework, a Tactic represents a role or objective for a single robot. For
 * example, being the goalie, shooting on the opposing goal, moving to receive a pass,
 * or acting as a defender. Tactics are where most of the complicated logic takes place,
 * and they tend to rely a lot on our Evaluation functions. Ultimately, Tactics will
 * return the next Intent that the Robot assigned to this Tactic should run in order
 * to work towards its objective.
 */
class Tactic
{
   public:
    /**
     * Creates a new Tactic with the given robot
     *
     * @param robot The robot that should perform this Tactic
     */
    explicit Tactic(const Robot& robot);

    /**
     * Returns true if the Tactic is done and false otherwise
     *
     * @return true if the Tactic is done and false otherwise
     */
    bool done() const;

    /**
     * Evaluates the given Robot and returns a score indicating how optimal it would be
     * for that Robot to perform this Tactic. A lower score indicates a more optimal
     * robot, and returned score values must be >= 0.
     *
     * @param robot The Robot to evaluate for this Tactic
     *
     * @return A score value >= 0 that indicates how optimal it would be for the given
     * robot to perform this Tactic. Lower scores indicate more optimal/preferred robots.
     */
    virtual double evaluateRobot(const Robot& robot) = 0;

    virtual ~Tactic() = default;

   protected:
    /**
     * Runs the coroutine and get the next Intent to run from the calculateNextIntent
     * function. If the Tactic is not done, the next Intent is returned. If the Tactic
     * is done, a null unique_ptr is returned.
     *
     * @return A unique pointer to the next Intent that should be run for the Tactic.
     * If the Tactic is done, an empty/null unique pointer is returned.
     */
    std::unique_ptr<Intent> getNextIntent();

    // The coroutine that sequentially returns the Intents the Tactic wants to run
    intent_coroutine::pull_type intent_sequence;
    // The robot performing this Tactic
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
     * @param yield The coroutine push_type for the Tactic
     *
     * @return A unique pointer to the next Intent that should be run for the Tactic.
     * If the Tactic is done, an empty/null unique pointer is returned. The very first
     * time this function is called, a null pointer will be returned (this does not
     * signify the Tactic is done).
     */
    std::unique_ptr<Intent> calculateNextIntentWrapper(
        intent_coroutine::push_type& yield);

    /**
     * Calculates the next Intent for the Tactic. If the Tactic is done
     * (ie. it has achieved its objective and has no more Intents to return),
     * an empty/null unique pointer is returned.
     *
     * @param yield The coroutine push_type for the Tactic
     *
     * @return A unique pointer to the next Intent that should be run for the Tactic.
     * If the Tactic is done, an empty/null unique pointer is returned.
     */
    virtual std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) = 0;
};

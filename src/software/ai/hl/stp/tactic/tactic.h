#pragma once

#include <boost/coroutine2/all.hpp>
#include <optional>

#include "software/ai/hl/stp/action/action.h"
#include "software/ai/intent/intent.h"
#include "software/world/world.h"

// We forward-declare the TacticVisitor interface (pure virtual class) because we need
// to know about the existence of this class in order to accept visitors with the
// accept() function. We cannot use an #include statement because this creates a cyclic
// dependency
//
// This class can be found in ai/hl/stp/tactic/tactic_visitor.h
class TacticVisitor;

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
 * is run, it will update the parameters of each tactic with the updateWorldParams(...)
 * and the updateControlParams(...) function (see the concrete implementations of this
 * class for examples). The updateWorldParams(...) updates any parameters that are derived
 * from the World independent of play, and the updateControlParams(...) updates all other
 * parameters. This is done every time in order for the Tactics to have the most up to
 * date information when they calculate the next Intent they want to run (for example if
 * we were following a moving robot, we need to constantly update our destination).
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
    explicit Tactic(bool loop_forever,
                    const std::set<RobotCapabilities::Capability> &capability_reqs_ = {});

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
     * robot hardware capability requirements of the tactic.
     */
    const std::set<RobotCapabilities::Capability> &robotCapabilityRequirements() const;

    /**
     * Mutable robot hardware capability requirements of the tactic.
     */
    std::set<RobotCapabilities::Capability> &mutableRobotCapabilityRequirements();


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
     * @param game_state_opt The current game state, defaults to std::nullopt
     *
     * @return A unique pointer to the next Intent that should be run for the Tactic.
     * If the Tactic is done, a nullptr is returned.
     */
    std::unique_ptr<Intent> getNextIntent(
        const std::optional<GameState> &game_state_opt = std::nullopt);

    /**
     * Returns the name of the Tactic
     *
     * @return the name of the Tactic
     */
    virtual std::string getName() const = 0;

    /**
     * Add an area to the list of areas this tactic should always be allowed to move
     *
     * If this area conflicts with the areas the tactic is not allowed to move due
     * to game state, this will take precedence
     */
    void addWhitelistedAvoidArea(AvoidArea area);

    /**
     * Add an extra area that this intents yielded by this tactic should avoid moving into
     *
     * This will override any areas put in the whitelist
     *
     * @param area The area to add the blacklist of areas to avoid
     */
    void addBlacklistedAvoidArea(AvoidArea area);

    /**
     * Remove an extra area that this intents yielded by this tactic should avoid moving
     * into
     *
     * If the area was not previously added, does nothing
     *
     * @param area The area to remove from the blacklist of areas to avoid
     */
    void removeBlacklistedAvoidArea(AvoidArea area);

    /**
     * Accepts a Tactic Visitor and calls the visit function on itself
     *
     * @param visitor A Tactic Visitor
     */
    virtual void accept(TacticVisitor &visitor) const = 0;

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
     * This function yields a unique pointer to the next Intent that should be run for the
     * Tactic. If the Tactic is done, an empty/null unique pointer is returned. The very
     * first time this function is called, a null pointer will be returned (this does not
     * signify the Tactic is done). This yield happens in place of a return.
     *
     * @param yield The coroutine push_type for the Tactic
     */
    void calculateNextIntentWrapper(IntentCoroutine::push_type &yield);

    /**
     * Calculates the next Intent for the Tactic. If the Tactic is done
     * (ie. it has achieved its objective and has no more Intents to return),
     * a nullptr is returned.
     *
     * This function yields a unique pointer to the next Intent that should be run for the
     * Tactic. If the Tactic is done, a nullptr is returned. This yield happens in place
     * of a return
     *
     * @param yield The coroutine push_type for the Tactic
     */
    virtual void calculateNextIntent(IntentCoroutine::push_type &yield) = 0;

    /**
     * Get all the areas that this tactic should not be allowed to move into based on
     * game state and the current whitelist for this tactic
     *
     * @param game_state The current game state
     *
     * @return a vector of areas this tactic should avoid
     */
    std::vector<AvoidArea> getAreasToAvoid(const GameState &game_state);

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
    IntentCoroutine::pull_type intent_sequence;

    // Whether or not this Tactic is done
    bool done_;

    // Whether or not this tactic should loop forever by restarting each time it is done
    bool loop_forever;

    // These are areas that the intents yielded by this tactic should be permitted
    // to move in, even if they would normally be in this game state. These are used
    // when one Tactic, such as GoalieTactic, is permitted in an area of the field
    // that the rest of the robots are not (in the case of the goalie, the friendly
    // defense area)
    std::vector<AvoidArea> whitelisted_avoid_areas;

    // These are areas that will be added to all intents yielded by this function,
    // regardless of whitelisted areas or game state
    std::vector<AvoidArea> blacklisted_avoid_areas;

    // robot capability requirements
    std::set<RobotCapabilities::Capability> capability_reqs;
};

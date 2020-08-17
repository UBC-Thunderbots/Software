#pragma once

#include <boost/coroutine2/all.hpp>
#include <optional>

#include "software/ai/hl/stp/action/action.h"
#include "software/ai/hl/stp/tactic/mutable_tactic_visitor.h"
#include "software/world/world.h"

// We typedef the coroutine return type to make it shorter, more descriptive,
// and easier to work with
typedef boost::coroutines2::coroutine<std::shared_ptr<Action>> ActionCoroutine;

/**
 * In the STP framework, a Tactic represents a role or objective for a single robot.
 * This can be thought of as a "position" on a typical soccer team. Some examples are:
 * - The goalie
 * - A "striker" that tries to get the ball and shoot on the enemy goal
 * - A defender that shadows enemy robots
 * - A passer
 * - A receiver (for a pass)
 *
 * Tactics are stateful, and use Actions to implement their behaviour. They also
 * make heavy use of our Evaluation functions in order to help them make decisions.
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
    explicit Tactic(bool loop_forever, const std::set<RobotCapability> &capability_reqs_);

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
    const std::set<RobotCapability> &robotCapabilityRequirements() const;

    /**
     * Mutable robot hardware capability requirements of the tactic.
     */
    std::set<RobotCapability> &mutableRobotCapabilityRequirements();


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
     * Whether or not this tactic is one used by the goalie
     * @return Whether or not this tactic is one used by the goalie
     */
    virtual bool isGoalieTactic() const;

    /**
     * Runs the coroutine and get the next Action to run from the calculateNextAction
     * function. If the Tactic is not done, the next Action is returned. If the Tactic
     * is done, a nullptr is returned.
     *
     * @return A unique pointer to the next Action that should be run for the Tactic.
     * If the Tactic is done, a nullptr is returned.
     */
    std::shared_ptr<Action> getNextAction(void);

    /**
     * Accepts a Tactic Visitor and calls the visit function on itself
     *
     * @param visitor A Tactic Visitor
     */
    virtual void accept(MutableTacticVisitor &visitor) = 0;

    virtual ~Tactic() = default;

   protected:
    // The robot performing this Tactic
    std::optional<Robot> robot;

   private:
    /**
     * A wrapper function for the calculateNextAction function.
     *
     * This function exists because when the coroutine (action_sequence) is first
     * constructed the coroutine is called/entered. This would normally cause the
     * calculateNextAction to be run once and potentially return incorrect results
     * due to default constructed values.
     *
     * This wrapper function will yield a null pointer the first time it's called and
     * otherwise use the calculateNextAction function. This first "null" value will never
     * be seen/used by the rest of the system since this will be during construction,
     * and the coroutine will be called again with valid parameters before any values are
     * returned. This effectively "shields" the logic from any errors caused by default
     * values during construction.
     *
     * This function yields a unique pointer to the next Action that should be run for the
     * Tactic. If the Tactic is done, an empty/null unique pointer is returned. The very
     * first time this function is called, a null pointer will be returned (this does not
     * signify the Tactic is done). This yield happens in place of a return.
     *
     * @param yield The coroutine push_type for the Tactic
     */
    void calculateNextActionWrapper(ActionCoroutine::push_type &yield);

    /**
     * Calculates the next Action for the Tactic. If the Tactic is done
     * (ie. it has achieved its objective and has no more Actions to return),
     * a nullptr is returned.
     *
     * This function yields a unique pointer to the next Action that should be run for the
     * Tactic. If the Tactic is done, a nullptr is returned. This yield happens in place
     * of a return
     *
     * @param yield The coroutine push_type for the Tactic
     */
    virtual void calculateNextAction(ActionCoroutine::push_type &yield) = 0;

    /**
     * A helper function that runs the action_sequence coroutine and returns the result
     * of the coroutine. The done_ member variable is also updated to reflect whether
     * or not the Tactic is done. If the Tactic is done, a nullptr is returned.
     *
     * @return the next Action this Tactic wants to run. If the Tactic is done, a nullptr
     * is returned
     */
    std::shared_ptr<Action> getNextActionHelper();

    // The coroutine that sequentially returns the Actions the Tactic wants to run
    ActionCoroutine::pull_type action_sequence;

    // Whether or not this Tactic is done
    bool done_;

    // Whether or not this tactic should loop forever by restarting each time it is done
    bool loop_forever;

    // robot capability requirements
    std::set<RobotCapability> capability_reqs;
};

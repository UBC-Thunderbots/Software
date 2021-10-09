#pragma once

#include <boost/coroutine2/all.hpp>  // TODO (#1888): remove this dependency
#include <optional>                  // TODO (#1888): remove this dependency

#include "software/ai/hl/stp/action/action.h"  // TODO (#1888): remove this dependency
#include "software/ai/hl/stp/tactic/tactic_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"
#include "software/ai/intent/intent.h"
#include "software/world/world.h"

#define COPY_CLASS(new_class, parent_class) \
class new_class : public parent_class \
{ \
    using parent_class::parent_class; \
};

// TODO (#1888): remove this typedef
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
 * Tactics are stateful, and use Intents to implement their behaviour. They also
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
     * @param capability_reqs_ The capability requirements for running this tactic
     */
    explicit Tactic(bool loop_forever, const std::set<RobotCapability> &capability_reqs_);

    Tactic() = delete;

    /**
     * Returns true if the Tactic is done and false otherwise. If the Tactic is supposed
     * to loop forever, this function will always return false.
     *
     * Default implementation is when coroutine is done
     *
     * @return true if the Tactic is done and false otherwise
     */
    virtual bool done() const;  // TODO (#1888): make this function pure virtual

    // TODO (#1888): remove this function
    /**
     * Returns the Robot assigned to this Tactic
     *
     * @return an std::optional containing the Robot assigned to this Tactic if one has
     * been assigned, otherwise returns std::nullopt
     */
    std::optional<Robot> getAssignedRobot() const;

    // TODO (#1888): remove this function
    /**
     * Updates the robot assigned to this Tactic
     *
     * @param robot The updated state of the Robot that should be performing
     * this Tactic
     */
    void updateRobot(const Robot &robot);

    // TODO (#1888): remove this function
    /**
     * Updates the world parameters for this tactic
     *
     * @param world The current state of the world
     */
    virtual void updateWorldParams(const World &world) = 0;

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
    virtual double calculateRobotCost(const Robot &robot, const World &world) const = 0;

    /**
     * Whether or not this tactic is one used by the goalie
     * @return Whether or not this tactic is one used by the goalie
     */
    // TODO (#1859): Delete this!
    virtual bool isGoalieTactic() const;

    // TODO (#1888): remove this function
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
     * Updates and returns the next intent from this tactic
     *
     * @param robot The robot this tactic is being assigned
     * @param world The updated world
     *
     * @return the next intent
     */
    std::unique_ptr<Intent> get(const Robot &robot, const World &world);

    /**
     * Accepts a Tactic Visitor and calls the visit function on itself
     *
     * @param visitor A Tactic Visitor
     */
    virtual void accept(TacticVisitor &visitor) const = 0;

    virtual ~Tactic() = default;

   protected:
    // TODO (#1888): remove this field
    // The robot performing this Tactic
    std::optional<Robot> robot_;

   private:
    // TODO (#1888): remove this function
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

    // TODO (#1888): remove this function
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

    // TODO (#1888): remove this function
    /**
     * A helper function that runs the action_sequence coroutine and returns the result
     * of the coroutine. The done_ member variable is also updated to reflect whether
     * or not the Tactic is done. If the Tactic is done, a nullptr is returned.
     *
     * @return the next Action this Tactic wants to run. If the Tactic is done, a nullptr
     * is returned
     */
    std::shared_ptr<Action> getNextActionHelper();

    // TODO (#1888): remove this field
    // The coroutine that sequentially returns the Actions the Tactic wants to run
    ActionCoroutine::pull_type action_sequence;

    // TODO (#1888): remove this field
    // Whether or not this Tactic is done
    bool done_;

    std::unique_ptr<Intent> intent;

    /**
     * Updates the intent ptr with the new intent
     *
     * @param tactic_update The tactic_update struct that contains all the information for
     * updating the intent
     */
    // TODO (#1888): make this function pure virtual
    virtual void updateIntent(const TacticUpdate &tactic_update);

    // Whether or not this tactic should loop forever by restarting each time it is done
    bool loop_forever;

    // robot capability requirements
    std::set<RobotCapability> capability_reqs;
};

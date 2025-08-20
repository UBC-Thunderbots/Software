#pragma once

#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"
#include "software/world/world.h"

/**
 * In the STP framework, a Tactic represents a role or objective for a single robot.
 * This can be thought of as a "position" on a typical soccer team. Some examples are:
 * - The goalie
 * - A "striker" that tries to get the ball and shoot on the enemy goal
 * - A defender that shadows enemy robots
 * - A passer
 * - A receiver (for a pass)
 *
 * Tactics are stateful, and use Primitives to implement their behaviour. They also
 * make heavy use of our Evaluation functions in order to help them make decisions.
 */

class Tactic
{
   public:
    /**
     * Returns true if the Tactic is done and false otherwise.
     *
     * @return true if the Tactic is done and false otherwise
     */
    virtual bool done() const = 0;

    /**
     * Gets the FSM state of the tactic
     *
     * @return the FSM state
     */
    virtual std::string getFSMState() const = 0;

    /**
     * robot hardware capability requirements of the tactic.
     *
     * @return the robot capability requirements
     */
    virtual const std::set<RobotCapability> &robotCapabilityRequirements() const = 0;

    /**
     * Mutable robot hardware capability requirements of the tactic.
     *
     * @return the Mutable robot hardware capability requirements of the tactic
     */
    virtual std::set<RobotCapability> &mutableRobotCapabilityRequirements() = 0;

    /**
     * Updates the last execution robot
     *
     * @param last_execution_robot The robot id of the robot that last executed the
     * primitive for this tactic
     */
    virtual void setLastExecutionRobot(std::optional<RobotId> last_execution_robot) = 0;

    /**
     * Updates and returns a set of primitives for all friendly robots from this tactic
     *
     * @param world The updated world
     *
     * @return the next primitive
     */
    virtual std::map<RobotId, std::shared_ptr<Primitive>> get(
        const WorldPtr &world_ptr) = 0;

    /**
     * Accepts a Tactic Visitor and calls the visit function on itself
     *
     * @param visitor A Tactic Visitor
     */
    virtual void accept(TacticVisitor &visitor) const = 0;

    virtual ~Tactic() = default;
};

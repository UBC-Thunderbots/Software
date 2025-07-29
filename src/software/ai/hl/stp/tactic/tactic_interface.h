#pragma once

/*
 * Interface for Tactics, so that we can make a std::vector<TacticInterface>
 * Otherwise, we would have to use a template which doesn't really work.
 */

class TacticInterface {
public:
    Tactic() = delete;

    /**
     * Returns true if the Tactic is done and false otherwise.
     *
     * @return true if the Tactic is done and false otherwise
     */
    virtual bool done() const =0;

    /**
     * Gets the FSM state of the tactic
     *
     * @return the FSM state
     */
    virtual std::string getFSMState() const =0;

    /**
     * robot hardware capability requirements of the tactic.
     *
     * @return the robot capability requirements
     */
    virtual const std::set<RobotCapability> &robotCapabilityRequirements() const =0;

    /**
     * Mutable robot hardware capability requirements of the tactic.
     *
     * @return the Mutable robot hardware capability requirements of the tactic
     */
    virtual std::set<RobotCapability> &mutableRobotCapabilityRequirements() =0;

    /**
     * Updates the last execution robot
     *
     * @param last_execution_robot The robot id of the robot that last executed the
     * primitive for this tactic
     */
    virtual void setLastExecutionRobot(std::optional<RobotId> last_execution_robot) =0;

    /**
     * Updates and returns a set of primitives for all friendly robots from this tactic
     *
     * @param world The updated world
     *
     * @return the next primitive
     */
     virtual std::map<RobotId, std::shared_ptr<Primitive>> get(const WorldPtr &world_ptr) =0;

    /**
     * Accepts a Tactic Visitor and calls the visit function on itself
     *
     * @param visitor A Tactic Visitor
     */
    virtual void accept(TacticVisitor &visitor) const = 0;

    virtual ~Tactic() = default;
};
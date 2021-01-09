#pragma once

#include <functional>
#include <include/boost/sml.hpp>

#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"
#include "software/ai/intent/intent.h"
#include "software/world/world.h"

struct TacticFSMUpdate
{
    Robot robot;
    World world;
    std::function<void(std::unique_ptr<Intent>)> set_intent;
};

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
     * @return true if the Tactic is done and false otherwise
     */
    virtual bool done() const = 0;

    /**
     * robot hardware capability requirements of the tactic.
     */
    const std::set<RobotCapability> &robotCapabilityRequirements() const;

    /**
     * Mutable robot hardware capability requirements of the tactic.
     */
    // TODO: this should be automatically updated or handled because it's too much to
    // expect of users
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
    virtual double cost(const Robot &robot, const World &world) const = 0;

    /**
     * Whether or not this tactic is one used by the goalie
     * @return Whether or not this tactic is one used by the goalie
     */
    // TODO (#1859): Delete this!
    virtual bool isGoalieTactic() const;

    /**
     * Updates and returns the next intent from this tactic
     *
     * @param robot The robot this tactic is being assigned
     * @param world The updated world
     *
     * @return the next intent
     */
    std::unique_ptr<Intent> next(const Robot &robot, const World &world);

    /**
     * Accepts a Tactic Visitor and calls the visit function on itself
     *
     * @param visitor A Tactic Visitor
     */
    virtual void accept(TacticVisitor &visitor) const = 0;

    virtual ~Tactic() = default;

   protected:
    std::unique_ptr<Intent> intent;

   private:
    virtual void updateFSM(const TacticFSMUpdate &tactic_fsm_update_event) = 0;

    // Whether or not this tactic should loop forever by restarting each time it is done
    bool loop_forever;

    // robot capability requirements
    std::set<RobotCapability> capability_reqs;
};

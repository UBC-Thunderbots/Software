#pragma once

#include "software/ai/hl/stp/tactic/tactic_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"
#include "software/ai/intent/intent.h"
#include "software/world/world.h"

/**
 * Copies a tactic, the new tactic will be the same besides having a different name
 *
 * @param new_class The new class that will be created
 * @param parent_class The class that is being copied
 */
#define COPY_TACTIC(new_class, parent_class)                                             \
    class new_class : public parent_class                                                \
    {                                                                                    \
        using parent_class::parent_class;                                                \
                                                                                         \
        void accept(TacticVisitor &visitor) const                                        \
        {                                                                                \
            visitor.visit(*this);                                                        \
        }                                                                                \
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
     * @param capability_reqs_ The capability requirements for running this tactic
     */
    explicit Tactic(const std::set<RobotCapability>& capability_reqs_);

    Tactic() = delete;

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
    const std::set<RobotCapability> &robotCapabilityRequirements() const;

    /**
     * Mutable robot hardware capability requirements of the tactic.
     *
     * @return the Mutable robot hardware capability requirements of the tactic
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

   private:
    std::unique_ptr<Intent> intent;

    /**
     * Updates the intent ptr with the new intent
     *
     * @param tactic_update The tactic_update struct that contains all the information for
     * updating the intent
     */
    virtual void updateIntent(const TacticUpdate &tactic_update) = 0;

    // robot capability requirements
    std::set<RobotCapability> capability_reqs;
};

#pragma once

#include "software/ai/hl/stp/tactic/primitive.h"
#include "software/ai/hl/stp/tactic/tactic_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"
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
 * Tactics are stateful, and use Primitives to implement their behaviour. They also
 * make heavy use of our Evaluation functions in order to help them make decisions.
 */
template<class TacticFsm, class... SubFsms>
class Tactic
{
   public:
    /**
     * Creates a new Tactic. The Tactic will initially have no Robot assigned to it.
     *
     * @param capability_reqs_ The capability requirements for running this tactic
     */
    explicit Tactic(const std::set<RobotCapability> &capability_reqs_, std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    Tactic() = delete;

    /**
     * Returns true if the Tactic is done and false otherwise.
     *
     * @return true if the Tactic is done and false otherwise
     */
    bool done() const;

    /**
     * Gets the FSM state of the tactic
     *
     * @return the FSM state
     */
    std::string getFSMState() const;

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
     * Updates the last execution robot
     *
     * @param last_execution_robot The robot id of the robot that last executed the
     * primitive for this tactic
     */
    void setLastExecutionRobot(std::optional<RobotId> last_execution_robot);

    /**
     * Updates and returns a set of primitives for all friendly robots from this tactic
     *
     * @param world The updated world
     *
     * @return the next primitive
     */
    std::map<RobotId, std::shared_ptr<Primitive>> get(const WorldPtr &world_ptr);

    /**
     * Accepts a Tactic Visitor and calls the visit function on itself
     *
     * @param visitor A Tactic Visitor
     */
    virtual void accept(TacticVisitor &visitor) const = 0;

    virtual ~Tactic() = default;

   protected:
    std::optional<RobotId> last_execution_robot;

    std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr;

    std::map<RobotId, std::unique_ptr<FSM<TacticFsm>>> fsm_map;

   private:
    /** Function to initialize the FSM. By default initializes the template FSM. Some FSMs may override if they initialize sub-FSMs.
     *
     * @return a pointer to the created FSM.
     */
    virtual std::unique_ptr<FSM<TacticFsm>> fsm_init();

    std::shared_ptr<Primitive> primitive;

    /**
     * Updates the primitive ptr with the new primitive
     * NEED TO REFACTOR CONTROL PARAMS?
     *
     * @param tactic_update The tactic_update struct that contains all the information for
     * updating the primitive
     */
    virtual void updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm);

    // robot capability requirements
    std::set<RobotCapability> capability_reqs;
};

#pragma once

#include <Tracy.hpp>

#include "software/ai/hl/stp/tactic/primitive.h"
#include "software/ai/hl/stp/tactic/tactic.h"
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
 * A template class to build all other Tactics out of
 *
 * @tparam TacticFsm The TacticBase FSM to base this tactic off of (e.g. AttackerTactic
 * needs AttackerFSM)
 * @tparam TacticSubFsms the sub FSMs this tactic uses (e.g. AttackerTactic needs
 * DribbleFSM as a sub FSM)
 */
template <class TacticFsm, class... TacticSubFsms>
class TacticBase : public Tactic
{
   public:
    /**
     * Creates a new TacticBase. The TacticBase will initially have no Robot assigned to
     * it.
     *
     * @param capability_reqs_ The capability requirements for running this tactic
     */
    explicit TacticBase(const std::set<RobotCapability> &capability_reqs_,
                        std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
        : last_execution_robot(std::nullopt),
          ai_config_ptr(ai_config_ptr),
          fsm_map(),
          control_params(),
          capability_reqs(capability_reqs_)
    {
        for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
        {
            fsm_map[id] = fsmInit();
        }
    }

    TacticBase() = delete;

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
    const std::set<RobotCapability> &robotCapabilityRequirements() const
    {
        return capability_reqs;
    }

    /**
     * Mutable robot hardware capability requirements of the tactic.
     *
     * @return the Mutable robot hardware capability requirements of the tactic
     */
    std::set<RobotCapability> &mutableRobotCapabilityRequirements()
    {
        return capability_reqs;
    }

    /**
     * Updates the last execution robot
     *
     * @param last_execution_robot The robot id of the robot that last executed the
     * primitive for this tactic
     */
    void setLastExecutionRobot(std::optional<RobotId> last_execution_robot)
    {
        this->last_execution_robot = last_execution_robot;
        FSMLogger::getInstance().flush_with_robot_id(*last_execution_robot);
    }

    /**
     * Updates and returns a set of primitives for all friendly robots from this tactic
     *
     * @param world The updated world
     *
     * @return the next primitive
     */
    std::map<RobotId, std::shared_ptr<Primitive>> get(const WorldPtr &world_ptr);

    /**
     * Accepts a TacticBase Visitor and calls the visit function on itself
     *
     * @param visitor A TacticBase Visitor
     */
    virtual void accept(TacticVisitor &visitor) const = 0;

    virtual ~TacticBase() = default;

   protected:

    std::optional<RobotId> last_execution_robot;

    // A shared pointer to the ai configuration to configure ai behaviour, shared by all
    // Plays, Tactics, and FSMs
    std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr;

    // The mapping of robots to their respective FSMs.
    std::map<RobotId, std::unique_ptr<FSM<TacticFsm>>> fsm_map;

    // The parameters this tactic uses control its FSMs.
    TacticFsm::ControlParams control_params;

    /** Function to initialize the FSM. By default initializes the template FSM and all
     * subFSMs.
     *
     * @return a pointer to the created FSM.
     */
    virtual std::unique_ptr<FSM<TacticFsm>> fsmInit()
    {
        return std::make_unique<FSM<TacticFsm>>(TacticFsm(ai_config_ptr),
                                                TacticSubFsms(ai_config_ptr)...,
                                                FSMLogger::getInstance());
    }

   private:
    std::shared_ptr<Primitive> primitive;

    /**
     * Updates the primitive ptr with the new primitive
     *
     * @param tactic_update The tactic_update struct that contains all the information for
     * updating the primitive
     */
    virtual void updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm);


    // robot capability requirements
    std::set<RobotCapability> capability_reqs;
};

template <class TacticFsm, class... TacticSubFsms>
bool TacticBase<TacticFsm, TacticSubFsms...>::done() const
{
    bool is_done = false;
    if (last_execution_robot.has_value())
    {
        is_done = fsm_map.at(last_execution_robot.value())->is(boost::sml::X);
    }
    return is_done;
}

template <class TacticFsm, class... TacticSubFsms>
std::string TacticBase<TacticFsm, TacticSubFsms...>::getFSMState() const
{
    std::string state_str = "";
    if (last_execution_robot.has_value())
    {
        state_str = getCurrentFullStateName(*fsm_map.at(last_execution_robot.value()));
    }
    return state_str;
}

template <class TacticFsm, class... TacticSubFsms>
std::map<RobotId, std::shared_ptr<Primitive>>
TacticBase<TacticFsm, TacticSubFsms...>::get(const WorldPtr &world_ptr)
{
    TbotsProto::RobotNavigationObstacleConfig obstacle_config;
    std::map<RobotId, std::shared_ptr<Primitive>> primitives_map;

    {
        ZoneNamedN(_tracy_tactic_set_primitive, "Tactic: Get primitives for each robot",
                   true);

        for (const auto &robot : world_ptr->friendlyTeam().getAllRobots())
        {
            updatePrimitive(TacticUpdate(robot, world_ptr,
                                         [this](std::shared_ptr<Primitive> new_primitive)
                                         { primitive = std::move(new_primitive); }),
                            !last_execution_robot.has_value() ||
                                last_execution_robot.value() != robot.id());

            CHECK(primitive != nullptr)
                << "Primitive for " << objectTypeName(*this) << " in state "
                << getFSMState() << " was not set" << std::endl;
            primitives_map[robot.id()] = std::move(primitive);
        }
    }

    return primitives_map;
}

template <class TacticFsm, class... TacticSubFsms>
void TacticBase<TacticFsm, TacticSubFsms...>::updatePrimitive(
    const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = fsmInit();
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(typename TacticFsm::Update(control_params, tactic_update));
}

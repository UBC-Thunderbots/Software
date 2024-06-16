#pragma once

#include "software/ai/hl/stp/skill/skill.h"

/**
 * Base implementation of a Skill that manages an FSM for each robot requesting
 * primitives from the Skill.
 *
 * @tparam TSkillFSM the FSM for the Skill; BaseSkill will use this FSM to generate
 *         primitives for each robot requesting primitives from this skill
 * @tparam TSkillSubFSMs sub-FSMs of TSkillFSM that need to be instantiated separately
 *         and dependency injected upon creation of TSkillFSM
 */
template <typename TSkillFSM, typename... TSkillSubFSMs>
class BaseSkill : public Skill
{
   public:
    virtual void updatePrimitive(const Robot& robot, const WorldPtr& world_ptr,
                                 const SetPrimitiveCallback& set_primitive) override;

    void reset(const Robot& robot) override;

    std::string getFSMState(RobotId robot_id) const override;

    bool done(const RobotId& robot) const override;

    bool suspended(const RobotId& robot) const override;

    bool tryResumingIfSuspended(const RobotId& robot, const WorldPtr& world_ptr) override;

   protected:
    explicit BaseSkill(std::shared_ptr<Strategy> strategy)
        : Skill(strategy), fsm_map_(), control_params_()
    {
    }

    std::map<RobotId, std::unique_ptr<FSM<TSkillFSM>>> fsm_map_;
    typename TSkillFSM::ControlParams control_params_;
};

template <typename TSkillFSM, typename... TSkillSubFSMs>
void BaseSkill<TSkillFSM, TSkillSubFSMs...>::updatePrimitive(
    const Robot& robot, const WorldPtr& world_ptr,
    const SetPrimitiveCallback& set_primitive)
{
    if (!fsm_map_.contains(robot.id()))
    {
        reset(robot);
    }

    fsm_map_.at(robot.id())
        ->process_event(typename TSkillFSM::Update(
            control_params_, SkillUpdate(robot, world_ptr, strategy_, set_primitive)));
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
void BaseSkill<TSkillFSM, TSkillSubFSMs...>::reset(const Robot& robot)
{
    fsm_map_[robot.id()] =
        std::make_unique<FSM<TSkillFSM>>(TSkillSubFSMs()..., TSkillFSM());
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
bool BaseSkill<TSkillFSM, TSkillSubFSMs...>::done(const RobotId& robot) const
{
    return fsm_map_.contains(robot) && fsm_map_.at(robot)->is(boost::sml::X);
}

template <typename TSkillFSM>
concept HasSuspendedState = requires
{
    typename TSkillFSM::Suspended;
    typename TSkillFSM::SuspendedUpdate;
};

template <typename TSkillFSM, typename... TSkillSubFSMs>
bool BaseSkill<TSkillFSM, TSkillSubFSMs...>::suspended(const RobotId& robot) const
{
    // Check at compile time that type TSkillFSM::Suspended exists
    if constexpr (HasSuspendedState<TSkillFSM>)
    {
        return fsm_map_.contains(robot) &&
               fsm_map_.at(robot)->is(boost::sml::state<typename TSkillFSM::Suspended>);
    }

    // Otherwise, TSkillFSM has no Suspended state and thus can never 
    // suspend execution
    return false;
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
bool BaseSkill<TSkillFSM, TSkillSubFSMs...>::tryResumingIfSuspended(
    const RobotId& robot, const WorldPtr& world_ptr)
{
    // Check at compile time that type TSkillFSM::SuspendedUpdate exists
    if constexpr (HasSuspendedState<TSkillFSM>)
    {
        if (suspended(robot))
        {
            fsm_map_.at(robot)->process_event(
                typename TSkillFSM::SuspendedUpdate(world_ptr, strategy_));
        }
    }

    // The FSM may have left SuspendedState after processing the SuspendedUpdate,
    // so we need to recheck whether the skill is suspended
    return suspended(robot);
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
std::string BaseSkill<TSkillFSM, TSkillSubFSMs...>::getFSMState(RobotId robot_id) const
{
    std::string state_str = TYPENAME(*this);
    if (fsm_map_.contains(robot_id))
    {
        state_str += "." + getCurrentFullStateName(*fsm_map_.at(robot_id));
    }
    return state_str;
}

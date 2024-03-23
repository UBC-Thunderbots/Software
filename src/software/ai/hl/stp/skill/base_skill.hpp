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
    void updatePrimitive(const Robot& robot, const WorldPtr& world_ptr,
                         const SetPrimitiveCallback& set_primitive) override;

    void reset(const Robot& robot) override;

    bool done(const Robot& robot) const override;

    std::string getFSMState(RobotId robot_id) const override;

   protected:
    explicit BaseSkill(std::shared_ptr<Strategy> strategy) : Skill(strategy), fsm_map_()
    {
    }

   private:
    std::map<RobotId, std::unique_ptr<FSM<TSkillFSM>>> fsm_map_;
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

    fsm_map_[robot.id()]->process_event(typename TSkillFSM::Update(
        typename TSkillFSM::ControlParams{},
        SkillUpdate(robot, world_ptr, strategy_, set_primitive)));
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
void BaseSkill<TSkillFSM, TSkillSubFSMs...>::reset(const Robot& robot)
{
    fsm_map_[robot.id()] =
        std::make_unique<FSM<TSkillFSM>>(TSkillSubFSMs()..., TSkillFSM());
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
bool BaseSkill<TSkillFSM, TSkillSubFSMs...>::done(const Robot& robot) const
{
    return fsm_map_.contains(robot.id()) && fsm_map_.at(robot.id())->is(boost::sml::X);
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

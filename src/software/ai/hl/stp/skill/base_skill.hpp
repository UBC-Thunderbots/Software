#pragma once

#include "software/ai/hl/stp/skill/skill.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * Base implementation of a Skill that manages an FSM for each robot requesting
 * primitives from the Skill.
 *
 * This template also implements and exposes a Tactic that will execute this Skill.
 * The Tactic can be used in simulated gameplay tests, field tests, or set plays that
 * require robots to execute a single skill.
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
    explicit BaseSkill(std::shared_ptr<Strategy> strategy);

    virtual void updatePrimitive(const Robot& robot, const WorldPtr& world_ptr,
                                 const SetPrimitiveCallback& set_primitive) override;

    void reset(const Robot& robot) override;

    bool done(const RobotId robot_id) const override;

    bool suspended(const RobotId robot_id) const override;

    bool tryResumingIfSuspended(const RobotId robot_id,
                                const WorldPtr& world_ptr) override;

    std::string getFSMState(const RobotId robot_id) const override;

    SkillState getSkillState(const RobotId robot_id) const override;

    /**
     * Implements a Tactic that executes this Skill.
     *
     * The Tactic can be used in simulated gameplay tests, field tests, or set plays that
     * require robots to execute a single skill.
     */
    class SkillTactic : public Tactic
    {
       public:
        explicit SkillTactic(std::shared_ptr<Strategy> strategy);

        void accept(TacticVisitor& visitor) const override;

        bool done() const override;

        std::string getFSMState() const override;

        /**
         * Updates the control params for the Skill FSM that this tactic is executing
         *
         * @param control_params the new control params to overwrite the
         * current params with
         */
        void updateControlParams(const typename TSkillFSM::ControlParams& control_params);

       private:
        BaseSkill<TSkillFSM, TSkillSubFSMs...> skill_;

        void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;
    };

   protected:
    std::map<RobotId, std::unique_ptr<FSM<TSkillFSM>>> fsm_map_;
    typename TSkillFSM::ControlParams control_params_;
    SkillState skill_state_;
};

/**
 * Copies a BaseSkill::SkillTactic.
 * The new tactic will be the same besides having a different name.
 *
 * @param new_class the new class that will be created
 * @param skill the Skill whose BaseSkill::SkillTactic is being copied
 */
#define COPY_SKILL_TACTIC(new_class, skill)                                              \
    class new_class : public skill::SkillTactic                                          \
    {                                                                                    \
        using skill::SkillTactic::SkillTactic;                                           \
                                                                                         \
        void accept(TacticVisitor& visitor) const                                        \
        {                                                                                \
            visitor.visit(*this);                                                        \
        }                                                                                \
    };

template <typename TSkillFSM, typename... TSkillSubFSMs>
BaseSkill<TSkillFSM, TSkillSubFSMs...>::BaseSkill(std::shared_ptr<Strategy> strategy)
    : Skill(strategy), fsm_map_(), control_params_(), skill_state_()
{
}

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
            control_params_, SkillUpdate(robot, world_ptr, strategy_, set_primitive,
                                         [&](const SkillState& skill_state)
                                         { skill_state_ = skill_state; })));
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
void BaseSkill<TSkillFSM, TSkillSubFSMs...>::reset(const Robot& robot)
{
    fsm_map_[robot.id()] =
        std::make_unique<FSM<TSkillFSM>>(TSkillSubFSMs()..., TSkillFSM());
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
bool BaseSkill<TSkillFSM, TSkillSubFSMs...>::done(const RobotId robot_id) const
{
    return fsm_map_.contains(robot_id) && fsm_map_.at(robot_id)->is(boost::sml::X);
}

template <typename TSkillFSM>
concept HasSuspendedState = requires
{
    typename TSkillFSM::Suspended;
    typename TSkillFSM::SuspendedUpdate;
};

template <typename TSkillFSM, typename... TSkillSubFSMs>
bool BaseSkill<TSkillFSM, TSkillSubFSMs...>::suspended(
    [[maybe_unused]] const RobotId robot_id) const
{
    // Check at compile time that type TSkillFSM::Suspended exists
    if constexpr (HasSuspendedState<TSkillFSM>)
    {
        return fsm_map_.contains(robot_id) &&
               fsm_map_.at(robot_id)->is(
                   boost::sml::state<typename TSkillFSM::Suspended>);
    }

    // Otherwise, TSkillFSM has no Suspended state and thus can never
    // suspend execution
    return false;
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
bool BaseSkill<TSkillFSM, TSkillSubFSMs...>::tryResumingIfSuspended(
    const RobotId robot_id, const WorldPtr& world_ptr)
{
    // Check at compile time that type TSkillFSM::SuspendedUpdate exists
    if constexpr (HasSuspendedState<TSkillFSM>)
    {
        if (suspended(robot_id))
        {
            fsm_map_.at(robot_id)->process_event(typename TSkillFSM::SuspendedUpdate(
                world_ptr, strategy_,
                [&](const SkillState& skill_state) { skill_state_ = skill_state; }));
        }
    }

    // The FSM may have left SuspendedState after processing the SuspendedUpdate,
    // so we need to recheck whether the skill is suspended
    return !suspended(robot_id);
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
std::string BaseSkill<TSkillFSM, TSkillSubFSMs...>::getFSMState(
    const RobotId robot_id) const
{
    std::string state_str = TYPENAME(*this);
    if (fsm_map_.contains(robot_id))
    {
        state_str += "." + getCurrentFullStateName(*fsm_map_.at(robot_id));
    }
    return state_str;
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
SkillState BaseSkill<TSkillFSM, TSkillSubFSMs...>::getSkillState(
    const RobotId robot_id) const
{
    return skill_state_;
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
BaseSkill<TSkillFSM, TSkillSubFSMs...>::SkillTactic::SkillTactic(
    std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      skill_(strategy)
{
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
void BaseSkill<TSkillFSM, TSkillSubFSMs...>::SkillTactic::accept(
    TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
bool BaseSkill<TSkillFSM, TSkillSubFSMs...>::SkillTactic::done() const
{
    return last_execution_robot && skill_.done(*last_execution_robot);
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
std::string BaseSkill<TSkillFSM, TSkillSubFSMs...>::SkillTactic::getFSMState() const
{
    if (last_execution_robot)
    {
        return skill_.getFSMState(*last_execution_robot);
    }
    return std::string();
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
void BaseSkill<TSkillFSM, TSkillSubFSMs...>::SkillTactic::updateControlParams(
    const typename TSkillFSM::ControlParams& control_params)
{
    skill_.control_params_ = control_params;
}

template <typename TSkillFSM, typename... TSkillSubFSMs>
void BaseSkill<TSkillFSM, TSkillSubFSMs...>::SkillTactic::updatePrimitive(
    const TacticUpdate& tactic_update, bool reset_fsm)
{
    skill_.updatePrimitive(tactic_update.robot, tactic_update.world_ptr,
                           tactic_update.set_primitive);
}
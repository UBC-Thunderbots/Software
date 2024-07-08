#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/util/is_template_base_of/is_template_base_of.hpp"

/**
 * All AssignedSkillTactics inherit from SkillTactic so that they
 * share a common non-template base class.
 */
class SkillTactic
{
};

/**
 * Implements a Tactic that executes a Skill.
 *
 * The AssignedSkillTactic can be used in simulated gameplay tests, field tests, or set
 * plays that require robots to execute a single skill.
 *
 * @tparam TSkill the Skill to execute (must inherit BaseSkill)
 */
template <typename TSkill>
class AssignedSkillTactic : public Tactic, public SkillTactic
{
    static_assert(is_template_base_of<BaseSkill, TSkill>::value,
                  "TSkill must derive from an instantiation of the BaseSkill template");

   public:
    /**
     * Create an AssignedSkillTactic that executes the specified Skill.
     *
     * @param strategy the Strategy shared by all of AI
     */
    explicit AssignedSkillTactic(std::shared_ptr<Strategy> strategy);

    void accept(TacticVisitor& visitor) const override;

    bool done() const override;

    std::string getFSMState() const override;

    /**
     * Updates the control params for the Skill that this tactic is executing
     *
     * @param control_params the new control params to overwrite the
     * current params with
     */
    void updateControlParams(const typename TSkill::ControlParams& control_params);

   private:
    TSkill skill_;

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;
};

template <typename TSkill>
AssignedSkillTactic<TSkill>::AssignedSkillTactic(std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Kick, RobotCapability::Chip, RobotCapability::Move}),
      skill_(strategy)
{
}

template <typename TSkill>
void AssignedSkillTactic<TSkill>::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

template <typename TSkill>
bool AssignedSkillTactic<TSkill>::done() const
{
    return last_execution_robot && skill_.done(*last_execution_robot);
}

template <typename TSkill>
std::string AssignedSkillTactic<TSkill>::getFSMState() const
{
    if (last_execution_robot)
    {
        return skill_.getFSMState(*last_execution_robot);
    }
    return std::string();
}

template <typename TSkill>
void AssignedSkillTactic<TSkill>::updateControlParams(
    const typename TSkill::ControlParams& control_params)
{
    skill_.control_params_ = control_params;
}

template <typename TSkill>
void AssignedSkillTactic<TSkill>::updatePrimitive(const TacticUpdate& tactic_update,
                                                  bool reset_fsm)
{
    skill_.updatePrimitive(tactic_update.robot, tactic_update.world_ptr,
                           tactic_update.set_primitive);
}

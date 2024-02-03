#pragma once

#include "software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h"
#include "software/ai/hl/stp/skill/skill.h"

class ShootSkill : public Skill
{
   public:
    explicit ShootSkill(std::shared_ptr<Strategy> strategy) : Skill(strategy){};

    double getViability(const Robot& robot, const World& world) const override;

    void updatePrimitive(const Robot& robot, const World& world,
                         const SetPrimitiveCallback& set_primitive) override;

    void reset(const Robot& robot) override;

    DEFINE_SKILL_DONE_AND_GET_FSM_STATE

   private:
    std::map<RobotId, std::unique_ptr<FSM<ShootSkillFSM>>> fsm_map_;
};

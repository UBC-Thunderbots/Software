#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h"

class ShootSkill : public BaseSkill<ShootSkillFSM, DribbleSkillFSM>
{
   public:
    explicit ShootSkill(std::shared_ptr<Strategy> strategy) : BaseSkill(strategy){};

    double getViability(const Robot& robot, const World& world) const override;
};

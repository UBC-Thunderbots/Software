#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"

class PassSkill : public BaseSkill<PassSkillFSM>
{
   public:
    explicit PassSkill(std::shared_ptr<Strategy> strategy) : BaseSkill(strategy){};

    double getViability(const Robot& robot, const World& world) const override;
};

#pragma once

#include "software/ai/hl/stp/tactic/attacker/skill/skill.h"

class ShootSkill : public Skill
{
   public:
    explicit ShootSkill(std::shared_ptr<Strategy> strategy) : Skill(strategy){};

    double getViability(const Robot& robot, const World& world) const override;
    std::unique_ptr<SkillFSM> getFSM() const override;
};

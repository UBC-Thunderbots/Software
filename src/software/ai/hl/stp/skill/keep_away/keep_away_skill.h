#pragma once

#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/keep_away/keep_away_skill_fsm.h"

class KeepAwaySkill : public BaseSkill<KeepAwaySkillFSM, DribbleSkillFSM>
{
   public:
    explicit KeepAwaySkill(std::shared_ptr<Strategy> strategy) : BaseSkill(strategy){};

    double getViability(const Robot& robot, const World& world) const override;
};

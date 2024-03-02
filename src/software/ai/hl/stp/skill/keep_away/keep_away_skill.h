#pragma once

#include "software/ai/evaluation/scoring/skills/skill_visitor.h"
#include "software/ai/hl/stp/skill/base_skill.hpp"
#include "software/ai/hl/stp/skill/keep_away/keep_away_skill_fsm.h"

class KeepAwaySkill : public BaseSkill<KeepAwaySkillFSM, DribbleSkillFSM>
{
   public:
    explicit KeepAwaySkill(std::shared_ptr<Strategy> strategy) : BaseSkill(strategy){};

    void accept(SkillVisitor& visitor) override;
};

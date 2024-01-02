#pragma once

#include "software/ai/hl/stp/skill/all_skills.h"

class SkillScorer
{
   public:
    virtual void score(const Skill&) = delete;

    virtual void score(const DribbleSkill& skill);
};

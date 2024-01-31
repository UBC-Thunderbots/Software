#pragma once

#include "software/ai/hl/stp/strategy/strategy.h"
#include "software/ai/hl/stp/tactic/attacker/skill/skill_fsm.h"

class Skill
{
   public:
    explicit Skill(std::shared_ptr<Strategy> strategy) : strategy_(strategy) {}

    virtual double getViability(const Robot& robot, const World& world) const = 0;

    virtual std::unique_ptr<SkillFSM> getFSM() const = 0;

   protected:
    std::shared_ptr<Strategy> strategy_;
};

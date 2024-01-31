#pragma once

#include "software/ai/hl/stp/strategy/strategy.h"
#include "software/ai/hl/stp/tactic/tactic_fsm.h"

class SkillFSM
{
   public:
    explicit SkillFSM(std::shared_ptr<Strategy> strategy) : strategy_(strategy) {}

    virtual void updatePrimitive(const TacticUpdate& tactic_update) = 0;

    virtual bool done() const = 0;

   protected:
    std::shared_ptr<Strategy> strategy_;
};
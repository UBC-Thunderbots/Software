#pragma once

#include "software/ai/hl/stp/strategy/strategy.h"
#include "software/ai/hl/stp/tactic/offense_support_tactics/offense_support_type.h"
#include "software/ai/hl/stp/tactic/tactic.h"

class OffenseSupportTactic : public Tactic
{
   public:
    explicit OffenseSupportTactic(const std::set<RobotCapability> &capability_reqs,
                                  std::shared_ptr<Strategy> strategy);

    virtual OffenseSupportType getOffenseSupportType() const = 0;

    virtual void commit() = 0;

   protected:
    std::shared_ptr<Strategy> strategy_;
};

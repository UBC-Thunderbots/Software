#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/util/make_enum/make_enum.h"

MAKE_ENUM(OffenseSupportType, PASS_RECEIVER, SUPPORT);

class OffenseSupportTactic : public Tactic
{
   public:
    explicit OffenseSupportTactic(const std::set<RobotCapability> &capability_reqs, std::shared_ptr<Strategy> strategy);

    void commit();

    virtual OffenseSupportType getOffenseSupportType() const = 0;

    virtual void updateControlParams() = 0;

   private:
    std::shared_ptr<Strategy> strategy_;
};

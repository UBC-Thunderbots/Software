#pragma once

#include "software/ai/hl/stp/skill/skill_fsm.h"
#include "software/ai/hl/stp/strategy/strategy.h"

class Skill
{
   public:
    explicit Skill(std::shared_ptr<Strategy> strategy) : strategy_(strategy) {}

    virtual double getViability(const Robot& robot, const World& world) const = 0;

    virtual void updatePrimitive(const Robot& robot, const World& world,
                                 const SetPrimitiveCallback& set_primitive) = 0;

    virtual void reset(const Robot& robot) = 0;

    virtual bool done(const Robot& robot) const = 0;

    virtual std::string getFSMState(RobotId robot_id) const = 0;

   protected:
    std::shared_ptr<Strategy> strategy_;
};

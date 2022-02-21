#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/navigator/navigator.h"

class TacticStepper
{
   public:
    explicit TacticStepper(std::shared_ptr<Tactic> tactic_to_run,
                           const std::set<MotionConstraint>& motion_constraints,
                           std::shared_ptr<const ThunderbotsConfig> thunderbots_config);

    std::unique_ptr<TbotsProto::PrimitiveSet> getPrimitives(const World& world,
                                                            unsigned robot_id);

   private:
    std::shared_ptr<Tactic> tactic;
    std::set<MotionConstraint> motion_constraints;
    std::shared_ptr<Navigator> navigator;
};

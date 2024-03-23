#pragma once

#include "software/ai/hl/stp/tactic/assigned_skill/assigned_skill_tactic.hpp"
#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"

class PlaceBallTactic : public AssignedSkillTactic<DribbleSkillFSM>
{
   public:
    explicit PlaceBallTactic(std::shared_ptr<Strategy> strategy)
        : AssignedSkillTactic(strategy)
    {
    }
};

class WallKickoffTactic : public AssignedSkillTactic<PivotKickSkillFSM, DribbleSkillFSM>
{
   public:
    explicit WallKickoffTactic(std::shared_ptr<Strategy> strategy)
        : AssignedSkillTactic(strategy)
    {
    }
};

#pragma once

#include "software/ai/hl/stp/tactic/assigned_skill/assigned_skill_tactic.hpp"
#include "software/ai/hl/stp/skill/kick/kick_skill_fsm.h"
#include "software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h"

class DribbleSkillTactic : public AssignedSkillTactic<DribbleSkillFSM>
{
    using AssignedSkillTactic::AssignedSkillTactic;
};

class KickSkillTactic : public AssignedSkillTactic<KickSkillFSM, GetBehindBallSkillFSM>
{
    using AssignedSkillTactic::AssignedSkillTactic;
};

class PivotKickSkillTactic : public AssignedSkillTactic<PivotKickSkillFSM, DribbleSkillFSM>
{
    using AssignedSkillTactic::AssignedSkillTactic;
};

class ShootSkillTactic : public AssignedSkillTactic<ShootSkillFSM, ShootSkillFSM::GetBallControlFSM,
                                                    DribbleSkillFSM, PivotKickSkillFSM>
{
    using AssignedSkillTactic::AssignedSkillTactic;
};

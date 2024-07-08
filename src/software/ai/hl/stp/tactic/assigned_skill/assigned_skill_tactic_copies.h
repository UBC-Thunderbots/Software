#pragma once

#include "software/ai/hl/stp/skill/all_skills.h"
#include "software/ai/hl/stp/tactic/assigned_skill/assigned_skill_tactic.hpp"

// The COPY_TACTIC macro doesn't support class template instantiations
// as the parent class, so we need to create a type alias for each 
// AssignedSkillTactic instantiation.  

using ChipSkillTactic = AssignedSkillTactic<ChipSkill>;
COPY_TACTIC(KickoffChipSkillTactic, ChipSkillTactic);

using DribbleSkillTactic = AssignedSkillTactic<DribbleSkill>;
COPY_TACTIC(PlaceBallSkillTactic, DribbleSkillTactic);

using PivotKickSkillTactic = AssignedSkillTactic<PivotKickSkill>;
COPY_TACTIC(WallKickoffSkillTactic, PivotKickSkillTactic);

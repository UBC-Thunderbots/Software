#pragma once

#include "software/ai/strategy.h"
#include "software/ai/hl/stp/skill/keep_away/keep_away_skill.h"
#include "software/ai/hl/stp/skill/pass/chip_pass_skill.h"
#include "software/ai/hl/stp/skill/pass/kick_pass_skill.h"
#include "software/ai/hl/stp/skill/shoot/shoot_skill.h"
#include "software/util/make_enum/reflective_enum.h"

MAKE_REFLECTIVE_ENUM(AttackerMdpAction, KEEP_AWAY, CHIP_PASS, KICK_PASS, SHOOT)

std::unique_ptr<Skill> createSkillFromAttackerMdpAction(
    AttackerMdpAction::Enum action, std::shared_ptr<Strategy> strategy);

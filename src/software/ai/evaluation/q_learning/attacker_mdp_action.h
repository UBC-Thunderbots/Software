#pragma once

#include "software/ai/hl/stp/skill/keep_away/keep_away_skill.h"
#include "software/ai/hl/stp/skill/pass/chip_pass_skill.h"
#include "software/ai/hl/stp/skill/pass/kick_pass_skill.h"
#include "software/ai/hl/stp/skill/shoot/shoot_skill.h"
#include "software/ai/strategy.h"
#include "software/util/make_enum/reflective_enum.h"

/**
 * Enum of actions that the Attacker agent can take in the
 * Markov decision process (MDP) modelling its gameplay decision making.
 *
 * These actions correspond to Skills that the AttackerTactic can execute.
 */
MAKE_REFLECTIVE_ENUM(AttackerMdpAction, KEEP_AWAY, CHIP_PASS, KICK_PASS, SHOOT)

/**
 * Creates a new Skill corresponding to the given AttackerMdpAction enum value.
 *
 * @param action the AttackerMdpAction enum value of the Skill to create
 * @param strategy the Strategy to pass to the created Skill
 *
 * @return a newly created Skill corresponding to the given action
 */
std::unique_ptr<Skill> createSkillFromAttackerMdpAction(
    AttackerMdpAction::Enum action, std::shared_ptr<Strategy> strategy);

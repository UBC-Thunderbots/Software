#pragma once

#include "software/ai/hl/stp/skill/keep_away/keep_away_skill.h"
#include "software/ai/hl/stp/skill/pass/pass_skill.hpp"
#include "software/ai/hl/stp/skill/shoot/shoot_skill.hpp"
#include "software/ai/strategy.h"
#include "software/util/make_enum/make_enum.hpp"

/**
 * Enum of actions that the Attacker agent can take in the
 * Markov decision process (MDP) modelling its gameplay decision making.
 *
 * These actions correspond to Skills that the AttackerTactic can execute.
 */
// clang-format off
MAKE_ENUM(AttackerMdpAction, 
          KEEP_AWAY, 
          KICK_PASS, 
          SHOOT,
//          DRIBBLE_SHOOT,
)
// clang-format on

/**
 * Creates a new Skill corresponding to the given AttackerMdpAction enum value.
 *
 * @param action the AttackerMdpAction enum value of the Skill to create
 * @param strategy the Strategy to pass to the created Skill
 *
 * @return a newly created Skill corresponding to the given action
 */
std::unique_ptr<Skill> createSkillFromAttackerMdpAction(
    AttackerMdpAction action, std::shared_ptr<Strategy> strategy);

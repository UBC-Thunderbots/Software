#include "software/ai/hl/stp/tactic/attacker/skill/head_skill.h"

HeadSkill::HeadSkill(const TbotsProto::AiConfig& ai_config,
                     std::shared_ptr<Strategy> strategy,
                     std::optional<unsigned> randomization_seed)
    : Skill(ai_config, strategy, 0, randomization_seed)
{
}

void HeadSkill::updatePrimitive(const TacticUpdate& tactic_update)
{
    LOG(FATAL) << "[HeadSkill] Error: This Skill is unable to generate a Primitive";
}

std::string HeadSkill::getCurrentState() const
{
    LOG(FATAL) << "[HeadSkill] This Skill should not be used to generate a Primitive";

    return std::string();
}

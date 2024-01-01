#include "software/ai/hl/stp/skill/skill.h"

Skill::Skill(const TbotsProto::AiConfig& ai_config, double default_score, std::optional<unsigned> seed)
    : ai_config(ai_config),
      score(default_score)
{
    std::srand(std::time(0));
    if (seed)
    {
        std::srand(seed);
    }
}

void Skill::updateScore(double score)
{
    // TODO(#3096): Update this skill's score
}

std::shared_ptr<Skill> getNextSkill(const World& world)
{
    if (!children)
    {
        children = std::make_optional<std::vector<std::shared_ptr<Skill>>>();
        auto all_registered_skill_constructors =
            GenericFactory<std::string, Skill, Play, TbotsProto::AiConfig, double>::getRegisteredConstructors();
        std::transform(all_registered_skill_constructors.begin(), all_registered_skill_constructors.end(),
                children.value().begin(),
                [&ai_config, &DEFAULT_SKILL_SCORE](auto& constructor)
                {
                    return std::move(constructor(ai_config, DEFAULT_SKILL_SCORE)); 
                })
    }

    // TODO(#3096): Instead of randomly selecting the skill, select one based on the children's scores
    return children.value()[std::rand() % children.value().size()]; 
}

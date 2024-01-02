#include "software/ai/hl/stp/skill/skill.h"

Skill::Skill(const TbotsProto::AiConfig& ai_config, double default_score,
             std::optional<unsigned> seed)
    : ai_config(ai_config), score(default_score)
{
    random_generator = std::mt19937(std::random_device()());
    if (seed)
    {
        random_generator = std::mt19937(seed.value());
    }
}

void Skill::updateScore(double score)
{
    // TODO(#3096): Update this skill's score
}

std::shared_ptr<Skill> Skill::getNextSkill(const Robot& robot, const World& world)
{
    if (!children)
    {
        children = std::make_optional<std::vector<std::shared_ptr<Skill>>>();
        auto all_registered_skill_constructors =
            GenericFactory<std::string, Skill, TbotsProto::AiConfig,
                           double>::getRegisteredConstructors();
        CHECK(all_registered_skill_constructors.size() > 0)
            << "No Skills registered in the Skill factory!";
        std::transform(all_registered_skill_constructors.begin(),
                       all_registered_skill_constructors.end(), children.value().begin(),
                       [&](auto& constructor) {
                           return std::move(constructor(ai_config, DEFAULT_SKILL_SCORE));
                       });
    }

    unsigned max_weight = 0;
    std::map<unsigned, std::shared_ptr<Skill>> skill_dist_map;
    std::for_each(
        children.value().begin(), children.value().end(),
        [&skill_dist_map, &max_weight, &robot, &world](std::shared_ptr<Skill> skill) {
            unsigned viability = std::clamp(
                static_cast<unsigned>(std::round(skill->calculateViability(robot, world) *
                                                 MAX_VIABILITY_SCORE)),
                0u, MAX_VIABILITY_SCORE);
            skill_dist_map[viability] = skill;
            max_weight += viability;
        });
    CHECK(max_weight > 0) << "No skill is viable for the next action!";

    std::uniform_int_distribution<> skill_dist(1, max_weight);
    unsigned next_skill_random_weight = skill_dist(random_generator);

    for (const auto& [key, value] : skill_dist_map)
    {
        if (next_skill_random_weight <= key)
        {
            return value;
        }
    }

    LOG(FATAL)
        << "[Skill::calculatNextSkill] Randomization failed to yield a valid Skill!";
    return children.value()[0];
}

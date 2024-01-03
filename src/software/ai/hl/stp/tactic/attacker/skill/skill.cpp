#include "software/ai/hl/stp/tactic/attacker/skill/skill.h"

Skill::Skill(const TbotsProto::AiConfig& ai_config, std::shared_ptr<Strategy> strategy,
             double default_score, std::optional<unsigned> seed)
    : ai_config_(ai_config), strategy_(strategy), score_(default_score)
{
    random_generator = std::mt19937(std::random_device()());
    if (seed)
    {
        random_generator = std::mt19937(seed.value());
    }
}

double Skill::calculateViability(const Robot& robot, const World& world)
{
    return score_;
}

double Skill::getScore()
{
    if (!children)
    {
        return score_;
    }

    // TODO(#3096): This returned score should reflect the maximum score of the children
    return 1.0;
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
                           std::shared_ptr<Strategy>,
                           double>::getRegisteredConstructors();
        CHECK(all_registered_skill_constructors.size() > 0)
            << "No Skills registered in the Skill factory!";
        std::transform(
            all_registered_skill_constructors.begin(),
            all_registered_skill_constructors.end(), children.value().begin(),
            [&](auto& constructor) {
                // TODO(#3097): Make the default values adjustable via dynamic params on a
                // per-skill basis
                return std::move(constructor(ai_config_, strategy_, DEFAULT_SKILL_SCORE));
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

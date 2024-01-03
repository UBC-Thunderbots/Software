#include "software/ai/hl/stp/tactic/attacker/skill/winger_skill.h"

WingerSkill::WingerSkill(const TbotsProto::AiConfig& ai_config,
                         std::shared_ptr<Strategy> strategy, double initial_score)
    : Skill(ai_config, strategy, initial_score)
{
}

double WingerSkill::calculateViability(const Robot& robot, const World& world)
{
    std::vector<OffenseSupportType> committed_offense_support =
        strategy_->getCommittedOffenseSupport();
    auto it =
        std::find(committed_offense_support.begin(), committed_offense_support.end(),
                  OffenseSupportType::PASS_RECEIVER);
    if (it == committed_offense_support.end())
    {
        return 0;
    }

    return score_;
}

bool WingerSkill::done() const
{
    // TODO(#3075): Implement WingerSkill
    return true;
}

void WingerSkill::updatePrimitive(const TacticUpdate& tactic_update)
{
    // TODO(#3075): Implement WingerSkill
}


// TODO(#3075): Register in the GenericFactory after implementing WingerSkill
// static TGenericFactory<std::string, Skill, WingerSkill, TbotsProto::AiConfig,
// std::shared_ptr<Strategy>, double> factory

#include "software/ai/hl/stp/strategy/strategy.h"

#include "software/ai/passing/pass_evaluation.hpp"
#include "software/ai/passing/pass_with_rating.h"

Strategy::Strategy(const TbotsProto::AiConfig& ai_config, const Field& field)
    : strategy_(std::make_shared<StrategyImpl<EighteenZonePitchDivision, EighteenZoneId>>(ai_config, field))
{
}

const TbotsProto::AiConfig& Strategy::getAiConfig() const
{
    return strategy_->getAiConfig();
}

void Strategy::updateAiConfig(const TbotsProto::AiConfig& ai_config)
{
    strategy_->updateAiConfig(ai_config);
}

void Strategy::updateWorld(const WorldPtr& world_ptr)
{
    strategy_->updateWorld(world_ptr);
}

std::shared_ptr<StrategyImpl<EighteenZonePitchDivision, EighteenZoneId>> Strategy::operator->()
{
    CHECK(strategy_->hasWorld())
        << "[StrategyImpl] Cannot generate next Strategy without a World!";
    return strategy_;
}

#include "software/ai/hl/stp/strategy/strategy.h"

#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_evaluation.hpp"
#include "software/ai/passing/pass_with_rating.h"

Strategy::Strategy(const TbotsProto::AiConfig& ai_config, const Field& field)
    : strategy_(std::make_shared<StrategyImpl>(ai_config, field))
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

void Strategy::updateWorld(const World& world)
{
    strategy_->updateWorld(world);
}

std::shared_ptr<StrategyImpl> Strategy::operator->()
{
    CHECK(strategy_->hasWorld())
        << "[StrategyImpl] Cannot generate next Strategy without a World!";
    return strategy_;
}

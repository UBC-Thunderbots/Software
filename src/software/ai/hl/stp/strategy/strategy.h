#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/strategy/strategy_impl.hpp"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/geom/pose.h"
#include "software/world/robot.h"
#include "software/world/robot_state.h"

class Strategy
{
   public:
    Strategy(const TbotsProto::AiConfig& ai_config,
             const Field& field = Field::createSSLDivisionBField());

    const TbotsProto::AiConfig& getAiConfig() const;
    void updateAiConfig(const TbotsProto::AiConfig& ai_config);
    void updateWorld(const WorldPtr& world_ptr);

    std::shared_ptr<StrategyImpl<EighteenZonePitchDivision>> operator->();

   private:
    std::shared_ptr<StrategyImpl<EighteenZonePitchDivision>> strategy_;
};

#pragma once

#include "software/ai/ai.h"

class PlayStepper
{
   public:
    explicit PlayStepper(std::unique_ptr<Play> play_to_run,
                         std::shared_ptr<const AiConfig> ai_config);

    std::unique_ptr<TbotsProto::PrimitiveSet> getPrimitives(const World& world);

   private:
    // The AI being tested and used in simulation
    AI ai;
};

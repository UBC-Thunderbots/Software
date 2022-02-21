#include "software/simulated_tests/play_stepper.h"


PlayStepper::PlayStepper(std::unique_ptr<Play> play_to_run,
                         std::shared_ptr<const AiConfig> ai_config)
    : ai(ai_config)
{
    ai.overridePlay(std::move(play_to_run));
}

std::unique_ptr<TbotsProto::PrimitiveSet> PlayStepper::getPrimitives(const World& world)
{
    return ai.getPrimitives(world);
}

#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * An example Play that moves the robots in a circle around the ball
 */
class ExamplePlay : public Play
{
   public:
    explicit ExamplePlay(std::shared_ptr<const AiConfig> config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
